#include "Arduino.h"
#include "radio.h"

const int si4463_sdn   = 2;  // Shutdown
const int si4463_nsel  = 5;  // SPI
const int si4463_mosi  = 6;  // SPI
const int si4463_gpio0 = radio_clock;  // RX clock
const int si4463_sck   = 14; // SPI
const int si4463_miso  = 15; // SPI
const int si4463_gpio1 = radio_data; // RX data, data guaranteed valid when clock rises.

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
  digitalWrite(si4463_mosi, LOW);
  digitalWrite(si4463_sck, LOW);
  digitalWrite(si4463_nsel, LOW);
}

// End SPI conversation / move bus to idle
void si4463_spi_end() {
  digitalWrite(si4463_mosi, LOW);
  digitalWrite(si4463_sck, LOW);
  digitalWrite(si4463_nsel, HIGH);
  delayMicroseconds(10);
}

// 8 bit SPI transaction with input and output
uint8_t si4463_byte(uint8_t out) {
  uint8_t res = 0;
  uint8_t i = 8;
  for (;i--;) {
    digitalWrite(si4463_mosi, (out & 0x80) ? HIGH : LOW);
    out <<= 1;
    digitalWrite(si4463_sck, HIGH); // Active edge
    res = (res << 1) | (digitalRead(si4463_miso) ? 1 : 0);
    delayMicroseconds(10);
    digitalWrite(si4463_sck, LOW);
    delayMicroseconds(10);
  }
  return res;
}


// Radio ready flag
bool si4463_wait_cts() {
  int i = 1000;
  uint8_t result;
  for (i=1000; i; i--) {
    si4463_spi_start();
    si4463_byte(CMD_READ_CMD_BUFF);
    result = si4463_byte(0);
    si4463_spi_end();
    if (result == 0xff) break;
    delayMicroseconds(10);
  }
  return (result == 0xff);
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

/////////////////////////////////////////////////////////////////////////////
// External routines
/////////////////////////////////////////////////////////////////////////////

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

uint8_t si4463_setup_data[] = {
  // Power up radio
  0x07, CMD_POWER_UP,
    0x01, //BOOT_OPTIONS - NO_PATCH, PRO mode
    0x00, //XTAL_OPTIONS - XTAL
    0x01, 0xC9, 0xC3, 0x80, // XO_FREQ - 30MHz

  // Configure GPIO to RX_DATA_CLK, RX_DATA, RX_STATE, TX_STATE
  0x08, CMD_GPIO_PIN_CFG,
    0x11, // GPIO0 - RX_DATA_CLK
    0x14, // GPIO1 - RX_DATA
    0x61, // GPIO2 - RX_STATE mfr uses 0x61
    0x60, // GPIO3 - TX_STATE mfr uses 0x60
    0x00, // NIRQ - DONOTHING
    0x00, // SDO - DONOTHING
    0x00, // GEN_CONFIG - DRV_STRENGTH max

  // Configure for image rejection calibration
  //
  0x05, CMD_SET_PROPERTY, GRP_GLOBAL, 0x01, 0x00,
    0x52,// GLOBAL_XO_TUNE crystal oscillator fine tune
  0x05, CMD_SET_PROPERTY, GRP_GLOBAL, 0x01, 0x03,
    0x60,// GLOBAL_CONFIG RESERVED=1 SEQUENCER_MODE=1
  0x10, CMD_SET_PROPERTY, GRP_MODEM, 0x0C, 0x00,
    0x0B, // MODEM_MOD_TYPE
    0x00, // MODEM_MAP_CONTROL
    0x07, // MODEM_DSM_CTRL
    0x01, 0x38, 0x80, // MODEM_DATA_RATE 0x013880
    0x05, 0xC9, 0xC3, 0x80, // MODEM_TX_NCO_MODE TXOSR=1 NCOMOD=0x1C9C380
    0x00, 0x00, // MODEM_FREQ_DEV
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x0C,
    0x69, // MODEM_FREQ_DEV
  0x0B, CMD_SET_PROPERTY, GRP_MODEM, 0x07, 0x19,
    0x00, // MODEM_MDM_CTRL
    0x08, // MODEM_IF_CONTROL
    0x02, 0x80, 0x00, // MODEM_IF_FREQ
    0xF0, // MODEM_DECIMATION_CFG1
    0x10, // MODEM_DECIMATION_CFG0
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x22,
    0x00, 0x4E, // MODEM_BCR_OSR
    0x06, 0x8D, 0xB9, // MODEM_BCR_NCO_OFFSET
    0x00, 0x00, // MODEM_BCR_GAIN
    0x02, // MODEM_BCR_GEAR
    0xC0, // MODEM_MISC_1
  0x0B, CMD_SET_PROPERTY, GRP_MODEM, 0x07, 0x2C,
    0x00, // MODEM_AFC_GEAR
    0x12, //MODEM_AFC_WAIT
    0x00, 0x34, // MDOEM_AFC_GAIN
    0x01, 0x5F, // MODEM_AFC_LIMITER
    0xA0, //MODEM_AFC_MISC
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x35,
    0xE2, // MODEM_AGC_CONTROL
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x38,
    0x11, // MODEM_AGC_WINDOW_SIZE
    0x11, // MODEM_AGC_RFPD_DECAY
    0x11, // MODEM_AGC_IFPD_DECAY
    0x00, // MODEM_FSK4_GAIN1
    0x1A, // MODEM_FSK4_GAIN0
    0x20, 0x00,// MODEM_FSK4_TH
    0x00, // MODEM_FSK4_MAP
    0x28, // MODEM_OOK_PDTC
  0x0C, CMD_SET_PROPERTY, GRP_MODEM, 0x08, 0x42,
    0xA4, // MODEM_OOK_CNT1
    0x03, // MODEM_OOK_MISC
    0xD6, // MODEM_RAW_SEARCH
    0x03, // MODEM_RAW_CONTROL
    0x00, 0x7B, // MODEM_RAW_EYE
    0x01, // MODEM_ANT_DIV_MODE
    0x80, // MODEM_ANT_DIV_CONTROL
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x4E,
    0x22, // MODEM_RSSI_COMP
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x51,
    0x0D, // MODEM_CLKGEN_BAND
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x00,
    0x7E, 0x64, 0x1B, 0xBA, 0x58, 0x0B, 0xDD, 0xCE, 0xD6, 0xE6, 0xF6, 0x00,
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x0C,
    0x03, 0x03, 0x15, 0xF0, 0x3F, 0x00, 0x7E, 0x64, 0x1B, 0xBA, 0x58, 0x0B,
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x18,
    0xDD, 0xCE, 0xD6, 0xE6, 0xF6, 0x00, 0x03, 0x03, 0x15, 0xF0, 0x3F, 0x00,
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
    0x28, 0xF6, // FREQ_CONTROL_CHANNEL_STEP_SIZE
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
  0x05, CMD_SET_PROPERTY, GRP_GLOBAL, 0x01, 0x03,
    0x60,// GLOBAL_CONFIG RESERVED=1 SEQUENCER_MODE=1 (no change)
  0x05, CMD_SET_PROPERTY, GRP_INT_CTL, 0x01, 0x00,
    0x00,// INT_CTL_ENABLE
  0x08, CMD_SET_PROPERTY, GRP_FRR_CTL, 0x04, 0x00,
    0x0A, // FRR_CTL_A_MODE LATCHED_RSSI
    0x09, // FRR_CTL_B_MODE CURRENT_STATE
    0x00, // FRR_CTL_C_MODE DISABLED
    0x00, // FRR_CTL_D_MODE DISABLED
  0x05, CMD_SET_PROPERTY, GRP_PREAMBLE, 0x01, 0x01,
    0x14, // PREMBKLE_TX_LENGTH
  0x05, CMD_SET_PROPERTY, GRP_PKT, 0x01, 0x06,
    0x40, // PKT_CONFIG1
  0x10, CMD_SET_PROPERTY, GRP_MODEM, 0x0C, 0x00,
    0x0B, // MODEM_MOD_TYPE
    0x00, // MODEM_MAP_CONTROL
    0x07, // MODEM_DSM_CTRL
    0x05, 0xDC, 0x00, // MODEM_DATA_RATE 0x05DC00
    0x05, 0xC9, 0xC3, 0x80, // MODEM_TX_NCO_MODE TXOSR=1 NCOMOD=0x1C9C380
    0x00, 0x01, // MODEM_FREQ_DEV 0x1F7
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x0C,
    0xF7, // MODEM_FREQ_DEV
  0x0B, CMD_SET_PROPERTY, GRP_MODEM, 0x07, 0x19,
    0x80,// MODEM_MDM_CTRL PH_SRC_SEL=0
    0x08, // MODEM_IF_CONTROL ZEROIF=NORMAL FIXIF=FIXED
    0x02, 0x80, 0x00, // MODEM_IF_FREQ 0x028000
    0x70, // MODEM_DECIMATION_CFG1 NDEC2=/2 NDEC1=/8 NDEC0=/1
    0x20, // MODEM_DECIMATION_CFG0 CHFLT_LOPW 0 DROOPFLTBYP 0 DWN3BYP 1 DWN2BYP 0 RXGAINX2 0
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x22,
    0x00, 0x62, // MODEM_BCR_OSR RXOSR=0x62
    0x05, 0x3E, 0x2D, // MODEM_BCR_NCO_OFFSET = 0x53E2D
    0x02, 0x9D, // MODEM_BCR_GAIN= 0x29D
    0x00, // MODEM_BCR_GEAR
    0xC2, // MODEM_MISC_1
  0x0B, CMD_SET_PROPERTY, GRP_MODEM, 0x07, 0x2C,
    0x54, // MODEM_AFC_GEAR
    0x36, //MODEM_AFC_WAIT
    0x81, 0x01, // MDOEM_AFC_GAIN
    0x02, 0x4E, // MODEM_AFC_LIMITER
    0x80, //MODEM_AFC_MISC
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x35,
    0xE2, // MODEM_AGC_CONTROL
  0x0D, CMD_SET_PROPERTY, GRP_MODEM, 0x09, 0x38,
    0x11,// MODEM_AGC_WINDOW_SIZE
    0x15, // MODEM_AGC_RFPD_DECAY
    0x15, // MODEM_AGC_IFPD_DECAY
    0x00,// MODEM_FSK4_GAIN1
    0x1A, // MODEM_FSK4_GAIN0
    0x20, 0x00,// MODEM_FSK4_TH
    0x00, // MODEM_FSK4_MAP
    0x28, // MODEM_OOK_PDTC
  0x0F, CMD_SET_PROPERTY, GRP_MODEM, 0x0B, 0x42,
    0x84, // MODEM_OOK_CNT1
    0x03, // MODEM_OOK_MISC
    0xD6, // MODEM_RAW_SEARCH
    0x8F, // MODEM_RAW_CONTROL
    0x00, 0x62, // MODEM_RAW_EYE
    0x01,// MODEM_ANT_DIV_MODE
    0x80, // MODEM_ANT_DIV_CONTROL
    0x80, // MODEM_RSSI_THRESH
    0x0C, // MODEM_RSSI_JUMP_THRESH
    0x03, // MODEM_RSSI_CONTROL
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x4E,
    0x40, // MODEM_RSSI_COMP
  0x05, CMD_SET_PROPERTY, GRP_MODEM, 0x01, 0x51,
    0x0D, // MODEM_CLKGEN_BAND
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x00,
    0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C,
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x0C,
    0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5,
  0x10, CMD_SET_PROPERTY, GRP_MODEM_CHFLT, 0x0C, 0x18,
    0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00,
  0x0B, CMD_SET_PROPERTY, GRP_SYNTH, 0x07, 0x00,
    0x2C, // SYNTH_PFDCP_CPFF
    0x0E, // SYNTH_PFDCP_CPINT
    0x0B, // SYNTH_VCO_KV
    0x04, // SYNTH_LPFILT3
    0x0C, // SYNTH_LPFILT2
    0x73, // SYNTH_LPFILT1
    0x03, // SYNTH_LPFILT0
  0x0C, CMD_SET_PROPERTY, GRP_FREQ_CONTROL, 0x08, 0x00,
    0x3F,// FREQ_CONTROL_INTE 0x3f
    0x0E, 0x51, 0xEB, // FREQ_CONTROL_FRAC 0xe51eb
    0x28, 0xF6, // FREQ_CONTROL_CHANNEL_STEP_SIZE 0x28f6
    0x20, // FREQ_CONTROL_W_SIZE
    0xFA,// FREQ_CONTROL_VCOCNT_RX_ADJ
  0x00
};

void radio_test() {
  // Test SPI bus
}

void radio_setup() {
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

  si4463_spi_end(); // Put SPI pins into idle state

  // Reset SI4463
  digitalWrite(si4463_sdn, HIGH);
  delayMicroseconds(1000);
  digitalWrite(si4463_sdn, LOW);

  // Program SI4463
  uint8_t *data = si4463_setup_data;
  while (*data) {
    int len = *data++;
    si4463_cmd(len, data, 0, NULL);
    data += len;
  }
}
