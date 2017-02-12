#include "Arduino.h"
#include "ais.h"
#include "radio.h"
#include "fifo.h"

//////////////////////////////////////////////////////////////////////////////
// AIS support
//////////////////////////////////////////////////////////////////////////////

enum PH_STATE {
  PH_STATE_OFF = 0,
  PH_STATE_RESET,           // reset/restart packet handler
  PH_STATE_WAIT_FOR_SYNC,   // wait for preamble (010101..) and start flag (0x7e)
  PH_STATE_PREFETCH,        // receive first 8 bits of packet
  PH_STATE_RECEIVE_PACKET   // receive packet payload
};

// packet handler errors
enum PH_ERROR {
  PH_ERROR_NONE = 0,
  PH_ERROR_STUFFBIT,        // invalid stuff-bit
  PH_ERROR_NOEND,           // no end flag after more than 1020 bits, message too long
  PH_ERROR_CRC             // CRC error
};

enum PH_SYNC_STATE {
	PH_SYNC_RESET = 0,			  // reset/restart sync detection
	PH_SYNC_0,					      // last bit was a 0
	PH_SYNC_1,					      // last bit was a 1
	PH_SYNC_FLAG				      // detecting start flag
};
#define PH_PREAMBLE_LENGTH  8   // minimum number of alternating bits we need for a valid preamble
#define PH_SYNC_TIMEOUT 16      // number of bits we wait for a preamble to start before changing channel

volatile uint8_t ph_state = PH_STATE_RESET;
volatile uint8_t ph_last_error = PH_ERROR_NONE;
volatile uint8_t ph_radio_channel = 0;

void ais_interrupt() {
  static uint16_t rx_bitstream;       // shift register with incoming data
  static uint16_t rx_bit_count;       // bit counter for various purposes
  static uint16_t rx_crc;             // word for AIS payload CRC calculation
  static uint8_t rx_one_count;        // counter of 1's to identify stuff bits
  static uint8_t rx_data_byte;        // byte to receive actual package data
  static uint8_t rx_prev_bit_NRZI;    // previous bit for NRZI decoding
  static uint8_t rx_sync_state;       // state of preamble and start flag detection
  static uint8_t rx_sync_count;       // length of valid bits in current sync sequence

  uint8_t rx_this_bit_NRZI;           // current bit for NRZI decoding
  uint8_t rx_bit;                     // current decoded bit


  // read data bit and decode NRZI
  rx_this_bit_NRZI = digitalRead(radio_data) ? 1 : 0;
  rx_bit = !(rx_prev_bit_NRZI ^ rx_this_bit_NRZI); 	// NRZI decoding: change = 0-bit, no change = 1-bit, i.e. 00,11=>1, 01,10=>0, i.e. NOT(A XOR B)
  rx_prev_bit_NRZI = rx_this_bit_NRZI;				// store encoded bit for next round of decoding

  // add decoded bit to bit-stream (receiving LSB first)
  rx_bitstream >>= 1;
  if (rx_bit) rx_bitstream |= 0x8000;

  // packet handler state machine
  switch (ph_state) {

    // STATE: OFF
    case PH_STATE_OFF:                            // state: off, do nothing
      break;

    // STATE: RESET
    case PH_STATE_RESET:                          // state: reset, prepare state machine for next packet
      rx_bitstream &= 0x8000;                     // reset bit-stream (but don't throw away incoming bit)
      rx_bit_count = 0;                           // reset bit counter
      fifo_new_packet();                          // reset fifo packet
      fifo_write_byte(ph_radio_channel);          // indicate channel for this packet
      ph_state = PH_STATE_WAIT_FOR_SYNC;          // next state: wait for training sequence
      rx_sync_state = PH_SYNC_RESET;
      break;

    // STATE: WAIT FOR PREAMBLE AND START FLAG
    case PH_STATE_WAIT_FOR_SYNC:                  // state: waiting for preamble and start flag
      rx_bit_count++;                             // count processed bits since reset
      // START OF SYNC STATE MACHINE
      switch (rx_sync_state) {
          // SYNC STATE: RESET
          case PH_SYNC_RESET:                     // sub-state: (re)start sync process
              if (rx_bit_count > PH_SYNC_TIMEOUT) {// if we exceeded sync time out
                  ph_state = PH_STATE_RESET;      // reset state machine, will trigger channel hop
              }
              else {                              // else
                  rx_sync_count = 0;              // start new preamble
                  rx_sync_state = rx_bit ? PH_SYNC_1 : PH_SYNC_0;
              }
              break;

          // SYNC STATE: 0-BIT
          case PH_SYNC_0:                        // sub-state: last bit was a 0
              if (rx_bit) {                      // if we get a 1
                  rx_sync_count++;               // valid preamble bit
                  rx_sync_state = PH_SYNC_1;     // next state
              } else {                           // if we get another 0
                  if (rx_sync_count > PH_PREAMBLE_LENGTH)	{	// if we have a sufficient preamble length
                      rx_sync_count = 7;  // treat this as part of start flag, we already have 1 out of 8 bits (0.......)
                      rx_sync_state = PH_SYNC_FLAG;				// next state flag detection
                  }
                  else                    // if not
                    rx_sync_state = PH_SYNC_RESET; // invalid preamble bit, restart preamble detection
              }
              break;

          // SYNC STATE: 1-BIT
          case PH_SYNC_1:                   // sub-state: last bit was a 1
              if (!rx_bit) {                // if we get a 0
                  rx_sync_count++;          // valid preamble bit
                  rx_sync_state = PH_SYNC_0;					// next state
              } else {									// if we get another 1
                  if (rx_sync_count > PH_PREAMBLE_LENGTH)	{	// if we have a sufficient preamble length
                      rx_sync_count = 5;  // treat this as part of start flag, we already have 3 out of 8 bits (011.....)
                      rx_sync_state = PH_SYNC_FLAG;				// next state flag detection
                  }
                  else										// if not
                      rx_sync_state = PH_SYNC_RESET;				// treat this as invalid preamble bit
              }
              break;

          // SYNC STATE: START FLAG
          case PH_SYNC_FLAG:								// sub-state: start flag detection
              rx_sync_count--;							// count down bits
              if (rx_sync_count != 0) {					// if this is not the last bit of start flag
                  if (!rx_bit)								// we expect a 1, 0 is an error
                      rx_sync_state = PH_SYNC_RESET;			// restart preamble detection
              } else {									// if this is the last bit of start flag
                  if (!rx_bit) {								// we expect a 0
                      rx_bit_count = 0;							// reset bit counter
                      ph_state = PH_STATE_PREFETCH;				// next state: start receiving packet
                  } else										// 1 is an error
                      rx_sync_state = PH_SYNC_RESET;				// restart preamble detection
              }
              break;
      }
      // END OF SYNC STATE MACHINE - preamble and start flag detected
      break;

    // STATE: PREFETCH FIRST PACKET BYTE
    case PH_STATE_PREFETCH:								// state: pre-fill receive buffer with 8 bits
        rx_bit_count++;									// increase bit counter
        if (rx_bit_count == 8) {						// after 8 bits arrived
          rx_bit_count = 0;							// reset bit counter
          rx_one_count = 0;							// reset counter for stuff bits
          rx_data_byte = 0;							// reset buffer for data byte
          rx_crc = 0xffff;							// init CRC calculation
          ph_state = PH_STATE_RECEIVE_PACKET;			// next state: receive and process packet
        }
        break;											// do nothing for the first 8 bits to fill buffer

    // STATE: RECEIVE PACKET
    case PH_STATE_RECEIVE_PACKET:						// state: receiving packet data
      rx_bit = rx_bitstream & 0x80;					// extract data bit for processing

      if (rx_one_count == 5) {						// if we expect a stuff-bit..
        if (rx_bit) {								// if stuff bit is not zero the packet is invalid
          ph_last_error = PH_ERROR_STUFFBIT;		// report invalid stuff-bit error
          ph_state = PH_STATE_RESET;				// reset state machine
        } else
          rx_one_count = 0;						// else ignore bit and reset stuff-bit counter
        break;
      }

      rx_data_byte = rx_data_byte >> 1 | rx_bit;		// shift bit into current data byte

      if (rx_bit) {									// if current bit is a 1
        rx_one_count++;								// count 1's to identify stuff bit
        rx_bit = 1;									// avoid shifting for CRC
      } else
        rx_one_count = 0;							// or reset stuff-bit counter

      if (rx_bit ^ (rx_crc & 0x0001))					// CCITT CRC calculation (according to Dr. Dobbs)
        rx_crc = (rx_crc >> 1) ^ 0x8408;
      else
        rx_crc >>= 1;
      if ((rx_bit_count & 0x07)==0x07) {				// every 8th bit.. (counter started at 0)
        fifo_write_byte(rx_data_byte);				// add buffered byte to FIFO
        rx_data_byte = 0;							// reset buffer
      }
      rx_bit_count++;									// count valid, de-stuffed data bits
      if ((rx_bitstream & 0xff00) == 0x7e00) {		// if we found the end flag 0x7e we're done
        //Serial.println(rx_crc,HEX);
        if (rx_crc != 0xf0b8)						// if CRC verification failed
          ph_last_error = PH_ERROR_CRC;			// report CRC error
        else {
          fifo_commit_packet();					// else commit packet in FIFO
        }
        ph_state = PH_STATE_RESET;					// reset state machine
      }
      else if (rx_bit_count > 1020) {						// if packet is too long, it's probably invalid
        ph_last_error = PH_ERROR_NOEND;				// report error
        ph_state = PH_STATE_RESET;					// reset state machine
      }
      break;
  }
  // END OF PACKET HANDLER STATE MACHINE

  if (ph_state == PH_STATE_RESET) {   // if next state is reset
    ph_radio_channel ^= 1;          // toggle radio channel between 0 and 1
    radio_rx(ph_radio_channel);    // initiate channel hop
  }
}

void ais_off() {
  ph_state = PH_STATE_OFF;
}

void ais_on() {
  ph_state = PH_STATE_RESET;
}

void ais_print_state() {
  switch (ph_state){
    case PH_STATE_OFF:
      Serial.println("STATE_OFF");
      break;
    case PH_STATE_RESET:
      Serial.println("STATE_RESET");
      break;
    case PH_STATE_WAIT_FOR_SYNC:
      Serial.println("STATE_WAIT_SYNC");
      break;
  case PH_STATE_PREFETCH:
      Serial.println("STATE_PREFETCH");
      break;
  case PH_STATE_RECEIVE_PACKET:
      Serial.println("STATE_RX_PACKET");
      break;
  }
}

void ais_setup() {
  ph_last_error = PH_ERROR_NONE;
  ph_radio_channel = 0;
  ph_state = PH_STATE_RESET;
  fifo_reset();
}

