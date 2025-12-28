#ifndef HELPERS_H
#define HELPERS_H

#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

void numberToFrequency(uint32_t Freq, char *rFRQ);
bool Calculate_Frequency(char mFRQ[9], uint32_t *result_freq);
void format_frequency(uint32_t freq, char *buffer);

// CTCSS Tone Listesi (Sadece extern tanımları, verinin kendisi .cpp'de)
extern const float tone_list[];
extern const uint8_t tone_count;
void get_tone_name(uint8_t index, char* buffer);

#endif