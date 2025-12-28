#include "helpers.h"
#include "config.h"

// Standart 38/50 CTCSS Tonları
const float tone_list[] = {
    67.0, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8,
    97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3,
    131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8,
    179.9, 186.2, 192.8, 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3
};
const uint8_t tone_count = sizeof(tone_list) / sizeof(tone_list[0]);

void numberToFrequency(uint32_t Freq, char *rFRQ) {
    sprintf(rFRQ, "%03lu.%03lu ", Freq / 1000, Freq % 1000);
    rFRQ[7] = ' ';
    rFRQ[8] = 0;
}

bool Calculate_Frequency(char mFRQ[9], uint32_t *result_freq) {
    if(strlen(mFRQ) < 7 || mFRQ[3] != '.') return false;
    
    uint32_t part1 = (mFRQ[0] - '0') * 100 + (mFRQ[1] - '0') * 10 + (mFRQ[2] - '0');
    uint32_t part2 = (mFRQ[4] - '0') * 100 + (mFRQ[5] - '0') * 10 + (mFRQ[6] - '0');
    
    *result_freq = (part1 * 1000) + part2;

    if (*result_freq >= 10000 && *result_freq <= 500000) return true;
    return false;
}

void get_tone_name(uint8_t index, char* buffer) {
    if (index >= tone_count) index = 0;
    // float to string (basit)
    int tam = (int)tone_list[index];
    int ondalik = (int)((tone_list[index] - tam) * 10);
    sprintf(buffer, "T %d.%d  ", tam, ondalik);
}