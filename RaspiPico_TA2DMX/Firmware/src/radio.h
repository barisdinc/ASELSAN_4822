#ifndef RADIO_H
#define RADIO_H

#include "config.h"

void radio_init();
void set_pll(uint32_t frequency);
void set_band_pins(uint32_t frequency, int radio_type); // 0: VHF, 1: UHF
void start_tone(float frequency);
void stop_tone();
void check_ptt();
void read_swr();

extern ChannelConfig current_channel;
extern bool is_transmitting;

#endif
