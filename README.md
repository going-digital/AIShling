# AIShling AIS receiver

## Work in progress. Currently nonfunctional.

## Introduction

AIShling is an AIS receiver. It detects ships within several miles, returning
their GPS position and more. AIShling uses cheaply available common parts.

## Output
AIShling outputs standard !AIVDM messages over a USB virtual serial port,
which can be used by OpenCPN, kplex or other tools.

## Operation
The Si4463 radio chip on the M4463D module is a very capable data receiver.
On powerup, it runs a self-calibration cycle, and then reconfigures itself to
receive AIS on the two AIS frequencies - 161.9875MHz and 162.0125MHz. The RX
data bits are clocked into the Pro Micro, which decodes and error checks the
messages. It also runs a USB protocol stack, feeding NMEA encoded message to
the host PC. It operates a channel hopping algorithm to receive on both
channels simultaneously. Alternatively this can be disabled to run a true
dual channel system.

The M4463D module has a poorly documented quirk - it is capable of transmission
and reception, but an antenna switch is connected to the radio GPIO2 and GPIO3
pins. For proper operation, GPIO2 must go high for receive, and GPIO3 must go
high for transmit. The programming of the Si4463 chip makes this happen
automatically within the module.

## Bill of Materials

  * M4463D module
      * Search eBay for "M4463D"
  * Sparkfun Pro Micro compatible board, must be 3.3V 8MHz
      * Search eBay for "Pro Micro 3.3V 8MHz"
  * 220nH inductor, 0402 size
  * 150nH inductor, 0402 size
  * 6.2pF capacitor, 0402 size
  * 12pF capacitor, 0402 size
  * SMA telescopic antenna
      * Search ebay for "SMA telescopic"

The 0402 components can be replaced with 0603 components, but they are a very
tight fit on the pads.

## Construction
*Danger - this bit is tricky:* Replace the components shown on the M4463D
module. This retunes the module to receive at marine band frequencies. These
components are 0402 size (1mm x 0.5mm), so you will need a flux pen, thin
solder, tweezers, solder wick and an 0.5mm or smaller solder tip to complete
this modification. Having extra components or modules on hand would be a good
idea.

TODO: Add images of modification

Wire the modules together as follows

| M4463D | Pro Micro  | Signal      |
|--------|------------|-------------|
| VDD    | VDD        | 3.3V power  |
| GPIO0  | D7         | RX clock    |
| GPIO1  | D16        | RX data     |
| IRQ    | no connect | Not used    |
| SCK    | D14        | SPI clock   |
| MISO   | D15        | SPI data    |
| MOSI   | D6         | SPI data    |
| NSEL   | D5         | SPI select  |
| GPIO2  | no connect | RX enable   |
| GPIO3  | no connect | TX enable   |
| SDN    | D2         | Radio reset |
| GND    | GND        | 0V          |
TODO: Add schematic


TODO: Add DSC support

## Notes
Cheap Sparkfun Pro Micro clones may not be programmed correctly - the ones the
author ordered arrived with the JTAG pins enabled, which means the Analog
pins A0-A3 can *NOT* be used for digital output.
