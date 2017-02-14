const int radio_data = 2;
const int radio_gpio1 = 3;

void radio_setup();
int radio_rssi();
void radio_rx(uint8_t channel);

uint8_t radio_get_chip_status();
void radio_test();
void radio_finetune();
void radio_test_clock(bool);

