const int radio_data = 16;
const int radio_clock = 7;

void radio_setup();
int radio_rssi();
void radio_rx(uint8_t channel);

uint8_t radio_get_chip_status();

