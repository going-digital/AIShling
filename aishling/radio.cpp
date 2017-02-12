#include "Arduino.h"
#include "radio.h"
#include "SPI.h"
#include <avr/pgmspace.h>

const int si4463_sdn   = 9;  // Shutdown
const int si4463_nsel  = 10;  // SPI
const int si4463_mosi  = 16;  // SPI
const int si4463_gpio0 = radio_clock;  // RX clock
const int si4463_sck   = 15; // SPI
const int si4463_miso  = 14; // SPI
const int si4463_gpio1 = radio_data; // RX data, data guaranteed valid when clock rises.

#define T_POR (6) // ms
#define T_SPI (1) // us Simplification of SPI timing scheme in data sheet table 8

/////////////////////////////////////////////////////////////////////////////
// SI4463 defines
/////////////////////////////////////////////////////////////////////////////

// SI4463 commands (from API documentation)
#define CMD_POWER_UP             0x02
#define CMD_NOP                  0x00
#define CMD_PART_INFO            0x01
#define CMD_FUNC_INFO            0x10
#define CMD_SET_PROPERTY         0x11
#define CMD_GET_PROPERTY         0x12
#define CMD_GPIO_PIN_CFG         0x13
#define CMD_FIFO_INFO            0x15
#define CMD_GET_INT_STATUS       0x20
#define CMD_REQUEST_DEVICE_STATE 0x33
#define CMD_CHANGE_STATE         0x34
#define CMD_READ_CMD_BUFF        0x44
#define CMD_FRR_A_READ           0x50
#define CMD_FRR_B_READ           0x53
#define CMD_FRR_C_READ           0x55
#define CMD_FRR_D_READ           0x57

#define CMD_IRCAL                0x17
#define CMD_IRCAL_MANUAL         0x1A

#define CMD_START_TX             0x31
#define CMD_WRITE_TX_FIFO        0x66

#define CMD_PACKET_INFO          0x16
#define CMD_GET_MODEM_STATUS     0x22
#define CMD_START_RX             0x32
#define CMD_RX_HOP               0x36
#define CMD_READ_RX_FIFO         0x77

#define CMD_GET_ADC_READING      0x14
#define CMD_GET_PH_STATUS        0x21
#define CMD_GET_CHIP_STATUS      0x23

// SI4463 property groups
#define GRP_GLOBAL        0x00
#define GRP_INT_CTL       0x01
#define GRP_FRR_CTL       0x02
#define GRP_PREAMBLE      0x10
#define GRP_SYNC          0x11
#define GRP_PKT           0x12
#define GRP_MODEM         0x20
#define GRP_MODEM_CHFLT   0x21
#define GRP_PA            0x22
#define GRP_SYNTH         0x23
#define GRP_MATCH         0x30
#define GRP_FREQ_CONTROL  0x40
#define GRP_RX_HOP        0x50

/////////////////////////////////////////////////////////////////////////////
// SPI routines
/////////////////////////////////////////////////////////////////////////////

// Start SPI conversation
void si4463_spi_start() {
  digitalWrite(si4463_nsel, LOW);
  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
}

// End SPI conversation / move bus to idle
void si4463_spi_end() {
  digitalWrite(si4463_nsel, HIGH);
  //delayMicroseconds(T_SPI);
}

// 8 bit SPI transaction with input and output
uint8_t si4463_byte(uint8_t out) {
  return SPI.transfer(out);
}


// Radio ready flag
bool si4463_wait_cts() {
  uint8_t result;
  do {
    si4463_spi_start();
    si4463_byte(CMD_READ_CMD_BUFF);
    result = si4463_byte(0);
    si4463_spi_end();
  } while (result != 0xff);
  return true;
}

// Command / response sequence
int si4463_cmd(
  int wr_len, const uint8_t* wr_data,
  int rd_len, uint8_t* rd_data
) {
  if (!si4463_wait_cts()) return 1;
  si4463_spi_start();
  for (;wr_len--;) si4463_byte(*wr_data++);
  si4463_spi_end();
  if (rd_len> 0) {
    if (!si4463_wait_cts()) return 2;
    si4463_spi_start();
    si4463_byte(CMD_READ_CMD_BUFF);
    si4463_byte(0);
    for (;rd_len--;) *rd_data++ = si4463_byte(0);
    si4463_spi_end();
  }
  return 0;
}

void si4463_set_prop(
  uint8_t group,
  uint8_t number,
  uint8_t value
) {
  uint8_t cmd[] = {CMD_SET_PROPERTY, 0, 1, 0, 0};
  cmd[1] = group;
  cmd[3] = number;
  cmd[4] = value;
  si4463_cmd(5, cmd, 0, NULL);
  si4463_wait_cts();
}

/////////////////////////////////////////////////////////////////////////////
// External routines
/////////////////////////////////////////////////////////////////////////////

void radio_finetune() {
  static uint8_t cosc=0x52;
  Serial.print("GLOBAL_XO_TUNE = 0x");
  Serial.println(cosc,HEX);
}

void radio_rx(uint8_t channel)
{
  uint8_t cmd[] = {CMD_START_RX, 0, 0, 0, 0, 0, 0, 0};
  cmd[1] = channel;
  si4463_cmd(8, cmd, 0, NULL);
  si4463_wait_cts();
}

int radio_rssi()
{
  uint8_t result;
  si4463_wait_cts();
  si4463_spi_start();
  si4463_byte(CMD_FRR_A_READ);
  result = si4463_byte(0);
  si4463_spi_end();
  return ((int) result >> 1) - 134;
}

// Based on dAISy
// Also https://github.com/peterantypas/ais_transponder/blob/master/application/src/radio_config_ph_all_channels.h

const uint8_t si4463_setup_data[] PROGMEM = {
  // Power up radio
  0x07, CMD_POWER_UP,
    0x01, //BOOT_OPTIONS - NO_PATCH, PRO mode
    0x00, //XTAL_OPTIONS - XTAL
    0x01, 0xC9, 0xC3, 0x80, // XO_FREQ - 30MHz

  // Configure GPIO to RX_DATA_CLK, RX_DATA, RX_STATE, TX_STATE
  0x08, CMD_GPIO_PIN_CFG,
    0x11, // GPIO0 - 2 low 3 high 11 RX_DATA_CLK
    0x14, // GPIO1 - 14 RX_DATA
    0x61, // GPIO2 - RX_STATE mfr uses 0x61
    0x60, // GPIO3 - TX_STATE mfr uses 0x60
    0x00, // NIRQ - DONOTHING
    0x00, // SDO - 0 DONOTHING 0B SDO
    0x00, // GEN_CONFIG - DRV_STRENGTH max

  // Configure for image rejection calibration
  //
  0x05, CMD_SET_PROPERTY, GRP_GLOBAL, 0x01, 0x00,
    0x52, // GLOBAL_XO_TUNE crystal oscillator fine tune. Verified with TCXO SDR to be accurate for prototype module.
  0x05, CMD_SET_PROPERTY, GRP_GLOBAL, 0x01, 0x03,
    0x60, // GLOBAL_CONFIG RESERVED=1 SEQUENCER_MODE=1
  0x10, CMD_SET_PROPERTY, GRP_MODEM, 0x0C, 0x00,
    0x0B, // MODEM_MOD_TYPE
    0x00, // MODEM_MAP_CONTROL
    0x07, // MODEM_DSM_CTRL
    0x02, 0x71, 0x00, // MODEM_DATA_RATE 0x027100
    0x05, 0xC9, 0xC3, 0x80, // MODEM_TX_NCO_MODE TXOSR=1 NCOMOD=0x1C9C380
    0x00, 0x00, // MODEM_FREQ_DEV
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x0C,
    0xD2, // MODEM_FREQ_DEV
  0x0C, CMD_SET_PROPERTY, GRP_MODEM, 0x08, 0x18,
//  0x0E, CMD_SET_PROPERTY, GRP_MODEM, 0x0A, 0x18, RevC2 only
    0x01, // MODEM_TX_RAMP_DELAY
    0x00, // MODEM_MDM_CTRL
    0x08, // MODEM_IF_CONTROL
    0x02, 0x80, 0x00, // MODEM_IF_FREQ
    0xB0, // MODEM_DECIMATION_CFG1
    0x10, // MODEM_DECIMATION_CFG0
    //0x0c, // MODEM_DECIMATION_CFG2 RevC2 only
    //0xe8, // MODEM_IFPKD_THRESHOLDS RevC2 only
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x22,
    0x00, 0x4E, // MODEM_BCR_OSR
    0x06, 0x8D, 0xB9, // MODEM_BCR_NCO_OFFSET
    0x00, 0x00, // MODEM_BCR_GAIN
    0x02, // MODEM_BCR_GEAR 00 on ais_transponder
    0xC0, // MODEM_MISC_1 D0 on ais_transponder, sets ESC_MIDPT=ESCAPE_PHASE_ERR instead of ESCAPE_1CLK
    // 0x08 MODEM_MISC_0 on ais_transponder
  0x0B, CMD_SET_PROPERTY, GRP_MODEM, 0x07, 0x2C,
    0x00, // MODEM_AFC_GEAR
    0x12, //MODEM_AFC_WAIT
    0x00, 0x69, // MODEM_AFC_GAIN 0x80 69 on ais_transponder
    0x01, 0x5C, // MODEM_AFC_LIMITER
    0xA0, //MODEM_AFC_MISC
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x35,
    0xE2, // MODEM_AGC_CONTROL E0 on ais_transponder
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x38,
    0x11, // MODEM_AGC_WINDOW_SIZE
    0x11, // MODEM_AGC_RFPD_DECAY
    0x11, // MODEM_AGC_IFPD_DECAY
    0x00, // MODEM_FSK4_GAIN1 0x80 on ais_transponder
    0x1A, // MODEM_FSK4_GAIN0
    0x20, 0x00,// MODEM_FSK4_TH
    0x00, // MODEM_FSK4_MAP
    0x28, // MODEM_OOK_PDTC
    // 0x0C on ais_transponder
  0x0C, CMD_SET_PROPERTY, GRP_MODEM, 0x08, 0x42,
    0xA4, // MODEM_OOK_CNT1
    0x03, // MODEM_OOK_MISC 23 on ais_transponder
    0xD6, // MODEM_RAW_SEARCH
    0x03, // MODEM_RAW_CONTROL 02 on ais_transponder
    0x00, 0x7B, // MODEM_RAW_EYE 00A3 on ais_transponder
    0x01, // MODEM_ANT_DIV_MODE 02 on ais_transponder
    0x80, // MODEM_ANT_DIV_CONTROL

 // ais_transponder sets the following:
 // 2048: 06 09 14 40
    
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x4E,
    0x22, // MODEM_RSSI_COMP not used on ais_transponder

// ais_transponder sets the following:
// 2250 00
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x51,
    0x0D, // MODEM_CLKGEN_BAND
// ais_transponder sets the following:
// 2254: 03,07
// 2257: 00
// 225B: 40,04,04,78,20
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x00,
    0x7E, 0x64, 0x1B, 0xBA, 0x58, 0x0B, 0xDD, 0xCE, 0xD6, 0xE6, 0xF6, 0x00,
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x0C,
    0x03, 0x03, 0x15, 0xF0, 0x3F, 0x00, 0x7E, 0x64, 0x1B, 0xBA, 0x58, 0x0B,
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x18,
    0xDD, 0xCE, 0xD6, 0xE6, 0xF6, 0x00, 0x03, 0x03, 0x15, 0xF0, 0x3F, 0x00,
  0x05, CMD_SET_PROPERTY, 0x22, 0x01, 0x02,
    0x3D, //Not used on ais_transponder
// ais_transponder sets the following:
// 2203: 1D
  0x0B, CMD_SET_PROPERTY, GRP_SYNTH, 0x07, 0x00,
    0x2C, // SYNTH_PFDCP_CPFF
    0x0E, // SYNTH_PFDCP_CPINT
    0x0B, // SYNTH_VCO_KV
    0x04, // SYNTH_LPFILT3
    0x0C, // SYNTH_LPFILT2
    0x73, // SYNTH_LPFILT1
    0x03, // SYNTH_LPFILT0
  0x0C, CMD_SET_PROPERTY, GRP_FREQ_CONTROL, 0x08, 0x00,
    0x3B, // FREQ_CONTROL_INTE
    0x0B, 0x00, 0x00, // FREQ_CONTROL_FRAC
    0x28, 0xF6, // FREQ_CONTROL_CHANNEL_STEP_SIZE ais_transponder: 14 7B
    0x20, // FREQ_CONTROL_W_SIZE
    0xFA, // FREQ_CONTROL_VCOCNT_RX_ADJ
  //
  // Perform image rejection calibration
  //
  0x08, CMD_START_RX,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x05, CMD_IRCAL,
    0x56, 0x10, 0xCA, 0xF0,
  0x05, CMD_IRCAL,
    0x13, 0x10, 0xCA, 0xF0,
  //
  // Reconfigure for AIS reception
  //
  0x05, CMD_SET_PROPERTY, GRP_GLOBAL, 0x01, 0x01,
    0x00,// GLOBAL_CLK_CFG clock output disabled, 32kHz clock disabled ais_transponder: 01
  0x05, CMD_SET_PROPERTY, GRP_GLOBAL, 0x01, 0x03,
    0x60,// GLOBAL_CONFIG RESERVED=1 SEQUENCER_MODE=1 (no change) ais_transponder: 20
  0x05, CMD_SET_PROPERTY, GRP_INT_CTL, 0x01, 0x00,
    0x00,// INT_CTL_ENABLE
  // ais_transponder: 07 18 01 08
  0x08, CMD_SET_PROPERTY, GRP_FRR_CTL, 0x04, 0x00,
    0x0A, // FRR_CTL_A_MODE LATCHED_RSSI
    0x09, // FRR_CTL_B_MODE CURRENT_STATE
    0x00, // FRR_CTL_C_MODE DISABLED
    0x00, // FRR_CTL_D_MODE DISABLED
  0x05, CMD_SET_PROPERTY, GRP_PREAMBLE, 0x01, 0x01,
    0x14, // PREAMBLE_TX_LENGTH
    // ais_transponder: 0x08, 0x14, 0x00, 0x0F, 0x31, 0x00, 0x00, 0x00, 0x00
// ais_transponder sets the following:
// 1100: 01 CC CC 00 00 00
// 1200: 00 01 08 ff ff 20 02
// 1208: 00 00 00 30 40 00 30 04 00 00 00 00
// 1214: 00 00 00 00 00 00 00 00 00 00 00 00
// 1220: 00 00 00 00 00 00 00 00 00 00 00 00
// 122C: 00 00 00 00 00 00 00 00 00
// 1236: 00 00 00 00
  0x05, CMD_SET_PROPERTY, GRP_PKT, 0x01, 0x06,
    0x40, // PKT_CONFIG1
  0x10, CMD_SET_PROPERTY, GRP_MODEM, 0x0C, 0x00,
    0x0B, // MODEM_MOD_TYPE ais_transponder sets 03
    0x00, // MODEM_MAP_CONTROL
    0x07, // MODEM_DSM_CTRL
    0x05, 0xDC, 0x00, // MODEM_DATA_RATE 0x05DC00
    0x05, 0xC9, 0xC3, 0x80, // MODEM_TX_NCO_MODE TXOSR=1 NCOMOD=0x1C9C380
    0x00, 0x01, // MODEM_FREQ_DEV 0x1F7
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x0C,
    0xF7, // MODEM_FREQ_DEV
  0x0C, CMD_SET_PROPERTY, GRP_MODEM, 0x08, 0x18,
    0x01, // MODEM_TX_RAMP_DELAY
    0x80,// MODEM_MDM_CTRL PH_SRC_SEL=0
    0x08, // MODEM_IF_CONTROL ZEROIF=NORMAL FIXIF=FIXED
    0x02, 0x80, 0x00, // MODEM_IF_FREQ 0x028000
    0x70, // MODEM_DECIMATION_CFG1 NDEC2=/2 NDEC1=/8 NDEC0=/1
    0x20, // MODEM_DECIMATION_CFG0 CHFLT_LOPW 0 DROOPFLTBYP 0 DWN3BYP 1 DWN2BYP 0 RXGAINX2 0
    // ais_transponder contines 00 E8
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x22,
    0x00, 0x62, // MODEM_BCR_OSR RXOSR=0x62
    0x05, 0x3E, 0x2D, // MODEM_BCR_NCO_OFFSET = 0x53E2D
    0x02, 0x9D, // MODEM_BCR_GAIN= 0x29D
    0x00, // MODEM_BCR_GEAR
    0xC2, // MODEM_MISC_1
    // ais_transponder continues 00
  0x0B, CMD_SET_PROPERTY, GRP_MODEM, 0x07, 0x2C,
    0x54, // MODEM_AFC_GEAR
    0x36, //MODEM_AFC_WAIT ais_transponder: 62
    0x81, 0x01, // MDOEM_AFC_GAIN
    0x02, 0x4E, // MODEM_AFC_LIMITER ais_transponder: 0213
    0x80, //MODEM_AFC_MISC ais_transponder: 80
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x35,
    0xE2, // MODEM_AGC_CONTROL ais_transponder: E0
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x38,
    0x11,// MODEM_AGC_WINDOW_SIZE
    0x15, // MODEM_AGC_RFPD_DECAY
    0x15, // MODEM_AGC_IFPD_DECAY
    0x00,// MODEM_FSK4_GAIN1 ais_transponder: 80
    0x1A, // MODEM_FSK4_GAIN0
    0x20, 0x00,// MODEM_FSK4_TH
    0x00, // MODEM_FSK4_MAP
    0x28, // MODEM_OOK_PDTC
    // ais_transponder continues 0C
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x42,
    0x84, // MODEM_OOK_CNT1
    0x03, // MODEM_OOK_MISC ais_transponder: 23
    0xD6, // MODEM_RAW_SEARCH ais_transponder doesnt change
    0x8F, // MODEM_RAW_CONTROL
    0x00, 0x62, // MODEM_RAW_EYE ais_transponder: 006A
    0x01,// MODEM_ANT_DIV_MODE ais_transponder: 02
    0x80, // MODEM_ANT_DIV_CONTROL ais_transponder: 00
    0x80, // MODEM_RSSI_THRESH ais_transponder: 0x46
    // ais_transponder continues 06 23
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x4C,
    0x03, // ais_transponder: 09 (reset from line above!)
    // ais_transponder continues 1c
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x4E,
    0x40, // MODEM_RSSI_COMP
  // ais_transponder sets the following:
  // 2050: 94
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x51,
    0x0D, // MODEM_CLKGEN_BAND
  // ais_transponder sets the following:
  // 2054: 03 07
  // 2057: 00
  // 205B: 42 04 04 78 20
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x00,
    0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C,
    //  0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x0C,
    0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5,
    //  0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x18,
    0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00,
    //  0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00
  0x05, CMD_SET_PROPERTY, 0x22, 0x01, 0x03,
    0x3D, // ais_transponder 1D
  0x0B, CMD_SET_PROPERTY, GRP_SYNTH, 0x07, 0x00,
    0x2C, // SYNTH_PFDCP_CPFF
    0x0E, // SYNTH_PFDCP_CPINT
    0x0B, // SYNTH_VCO_KV
    0x04, // SYNTH_LPFILT3
    0x0C, // SYNTH_LPFILT2
    0x73, // SYNTH_LPFILT1
    0x03, // SYNTH_LPFILT0
  // ais_transponder sets the following:
  // 3000: 00 00 00 00 00 00 00 00 00 00 00 00
  
  0x0C, CMD_SET_PROPERTY, GRP_FREQ_CONTROL, 0x08, 0x00,
    0x3F,// FREQ_CONTROL_INTE 0x3f
    0x0E, 0x51, 0xEB, // FREQ_CONTROL_FRAC 0xe51eb ais_transponder 0ccccc
    0x28, 0xF6, // FREQ_CONTROL_CHANNEL_STEP_SIZE 0x28f6 ais_transponder 147b
    0x20, // FREQ_CONTROL_W_SIZE
    0xFA,// FREQ_CONTROL_VCOCNT_RX_ADJ
  0x00
};

void radio_test() {
  uint8_t result[32];
  uint8_t command[16];
  int i;
  command[0] = CMD_PART_INFO;
  si4463_cmd(1, command, 8, result);
  Serial.print("PART_INFO: Si");
  Serial.print(result[1] >> 4,HEX);
  Serial.print(result[1] & 15,HEX);
  Serial.print(result[2] >> 4,HEX);
  Serial.print(result[2] & 15,HEX);
  Serial.print(" REV ");
  Serial.print(result[0]);
  Serial.print(" PART BUILD ");
  Serial.print(result[3]);
  Serial.print(" ID ");
  Serial.print(result[4]*256+result[5]);
  Serial.print(" CUSTOMER ");
  Serial.print(result[6]);
  Serial.print(" ROM_ID ");
  Serial.println(result[7]);
  uint32_t n;
  n = ((uint32_t)result[1] << 16) | (result[2]<<8) | result[7];
  switch(n) {
    //http://community.silabs.com/t5/Proprietary/Si446x-PART-INFO-and-FUNC-INFO-API-commands-question/td-p/153149
    case 0x406003: Serial.println("Si4060-B1B"); break;
    case 0x406303: Serial.println("Si4063-B1B"); break;
    case 0x435503: Serial.println("Si4355-B1B"); break;
    case 0x436203: Serial.println("Si4362-B1B"); break;
    case 0x443803: Serial.println("Si4438-B1B"); break;
    case 0x445503: Serial.println("Si4455-B1B"); break;
    case 0x446003: Serial.println("Si4460-B1B"); break;
    case 0x446103: Serial.println("Si4461-B1B"); break;
    case 0x446303: Serial.println("Si4463-B1B"); break;
    case 0x446403: Serial.println("Si4464-B1B"); break;
    case 0x405506: Serial.println("Si4055-C2A"); break;
    case 0x406006: Serial.println("Si4060-C2A"); break;
    case 0x406306: Serial.println("Si4063-C2A"); break;
    case 0x435506: Serial.println("Si4355-C2A"); break;
    case 0x436206: Serial.println("Si4362-C2A"); break;
    case 0x443806: Serial.println("Si4438-C2A"); break;
    case 0x445506: Serial.println("Si4455-C2A"); break;
    case 0x446006: Serial.println("Si4460-C2A"); break;
    case 0x446106: Serial.println("Si4461-C2A"); break;
    case 0x446306: Serial.println("Si4463-C2A"); break;
    case 0x446406: Serial.println("Si4464-C2A"); break;
    case 0x446706: Serial.println("Si4467-A2A"); break;
    case 0x446806: Serial.println("Si4468-A2A"); break;
    default: Serial.println("Device unknown!"); break;
  }
  
  command[0] = CMD_FUNC_INFO;
  command[1] = 0;
  si4463_cmd(1, command, 6, result);
  Serial.print("FUNC_INFO: EXT ");
  Serial.print(result[0]);
  Serial.print(" BRANCH ");
  Serial.print(result[1]);
  Serial.print(" INT ");
  Serial.print(result[2]);
  Serial.print(" PATCH ");
  Serial.print(result[3]*256+result[4]);
  Serial.print(" FUNC ");
  Serial.println(result[5]);
}

void radio_setup() {
  uint8_t si4463_cmd_buffer[16];
  // Upload configuration to radio.
  // This is a 2GMSK demodulator channel hopping between AIS1 and AIS2.
  // Data on GPIO0, Clock on GPIO1.
  pinMode(si4463_sdn, OUTPUT);
  pinMode(si4463_nsel, OUTPUT);
  pinMode(si4463_mosi, OUTPUT);
  pinMode(si4463_miso, INPUT);
  pinMode(si4463_sck, OUTPUT);
  pinMode(si4463_gpio1, INPUT);
  pinMode(si4463_gpio0, INPUT);

  SPI.begin();

  // Reset SI4463
  digitalWrite(si4463_sdn, HIGH);
  delayMicroseconds(10);
  digitalWrite(si4463_sdn, LOW);
  delay(T_POR); // Wait tPOR = 5ms

  // Program SI4463
  int i = 0;
  while (pgm_read_byte_near(si4463_setup_data+i)) {
    int len;
    len = pgm_read_byte_near(si4463_setup_data + i);
    i++;
    memcpy_P(si4463_cmd_buffer, si4463_setup_data + i, len);
    si4463_cmd(len, si4463_cmd_buffer, 0, NULL);
    i += len;
  }
}

void radio_test_clock(bool state) {
  uint8_t cmd[] = {
    CMD_GPIO_PIN_CFG,
    0x11, // GPIO0 - 2 low 3 high 11 RX_DATA_CLK
    0x14, // GPIO1 - 14 RX_DATA
    0x61, // GPIO2 - RX_STATE mfr uses 0x61
    0x60, // GPIO3 - TX_STATE mfr uses 0x60
    0x00, // NIRQ - DONOTHING
    0x0B, // SDO - 0 DONOTHING 0B SDO
    0x00, // GEN_CONFIG - DRV_STRENGTH max
  };
  if (state) {
    // Output system clock
    cmd[5] = 7; // DIV_CLK
    si4463_cmd(8, cmd, 0, NULL);
    si4463_set_prop(0, 1, 0x40); // 40 for 30MHz, 48 for 15MHz, 50 for 10MHz, 58 for 4MHz, 60 for 3MHz, 68 for 2MHz, 70 for 1MHz
  } else {
    // Output system clock
    cmd[5] = 2; // DRIVE_0
    si4463_cmd(8, cmd, 0, NULL);
    si4463_set_prop(0, 1, 0x00); // 40 for 30MHz, 48 for 15MHz, 50 for 10MHz, 58 for 4MHz, 60 for 3MHz, 68 for 2MHz, 70 for 1MHz
  }
}

