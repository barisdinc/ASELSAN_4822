#ifndef DISPLAY_H
#define DISPLAY_H

#include "pico/stdlib.h"

void display_init();
void display_write_freq(const char* freq_str);
void display_write_text(const char* text);

#endif
