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
  ais_setup();
  
  while (!Serial);
  // Give USB terminal time to start up
  startup_message();
  // Set up radio
  Serial.print("Setting up radio...");
  unsigned long t;
  t = millis();
  radio_setup();
  t = millis() - t;
  Serial.print("done in ");
  Serial.print(t);
  Serial.println("ms");

  radio_test();
  
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
  Serial.println("AIShling: AIS receiver");
  Serial.println("http://github.com/going-digital/AIShling");
  Serial.println();
  Serial.println("h: help");
  Serial.println("e: AIS state");
  Serial.println("f: Radio crystal finetune");
  Serial.println("q: Enable oscillator output");
  Serial.println("w: Disable oscillator output");
}

////////////////////////////////////////////////////////////////////////////// 
// Loop
//////////////////////////////////////////////////////////////////////////////
void loop() {
  if (fifo_get_packet()) {
    nmea_process_packet();
    fifo_remove_packet();
  }
  if (Serial.available()) {
    uint8_t c = Serial.read();
    switch (c) {
      case 'h': // Help message
        startup_message();
        break;
      case 'e':
        ais_print_state();
        break;
      case 'f':
        ais_off();
        radio_finetune();
        ais_on();
        break;
      case 'q':
        radio_test_clock(true);
        Serial.println("30MHz test output on NIRQ enabled");
        break;
      case 'w':
        radio_test_clock(false);
        Serial.println("30MHz test output on NIRQ disabled");
        break;

      default:
        break;
    }
  }
}
