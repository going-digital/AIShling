// AIS receiver for Sparkfun Pro Micro + M4463D Si4463 module
// Peter Knight
//
#include "radio.h"
#include "ais.h"
#include "fifo.h"
#include "nmea.h"

////////////////////////////////////////////////////////////////////////////// 
// Setup
//////////////////////////////////////////////////////////////////////////////
void setup() {
  // Give USB terminal time to start up
  RXLED1; // Yellow LED
  delay(10000); // Time to open serial terminal
  RXLED0;
  startup_message();

  // Set up radio
  Serial.print("Setting up radio...");
  radio_setup();
  Serial.println("done");

  // Connect AIS decoder
  TXLED1; // Green
  attachInterrupt(
    digitalPinToInterrupt(radio_clock),
    ais_interrupt,
    RISING
  );
}

void startup_message() {
  // Startup message
  Serial.println("    _|_|    _|_|_|    _|_|_|  _|        _|  _|                      ");
  Serial.println("  _|    _|    _|    _|        _|_|_|    _|      _|_|_|      _|_|_|  ");
  Serial.println("  _|_|_|_|    _|      _|_|    _|    _|  _|  _|  _|    _|  _|    _|  ");
  Serial.println("  _|    _|    _|          _|  _|    _|  _|  _|  _|    _|  _|    _|  ");
  Serial.println("  _|    _|  _|_|_|  _|_|_|    _|    _|  _|  _|  _|    _|    _|_|_|  ");
  Serial.println("                                                                _|  ");
  Serial.println("  _|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|    ");
  Serial.println();
}

////////////////////////////////////////////////////////////////////////////// 
// Loop
//////////////////////////////////////////////////////////////////////////////
void loop() {
  if (fifo_get_packet()) {
    Serial.println("Packet!");
    nmea_process_packet();
    fifo_remove_packet();
  }
  if (Serial.available()) {
    uint8_t c = Serial.read();
    switch (c) {
      case 'h':
        startup_message();
        break;
      case 'e':
        ais_print_state();
        break;
      case 's':
        Serial.println(radio_get_chip_status());
        break;
      default:
        break;
    }
  }
}
