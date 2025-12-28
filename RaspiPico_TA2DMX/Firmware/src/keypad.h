#ifndef KEYPAD_H
#define KEYPAD_H

#include "pico/stdlib.h"

#define PCF8574_KEYB_ADDR 0x20  // TD2 Entegresi
#define PCF8574_LED_ADDR  0x21  // TD3 Entegresi (LED'ler burada)

// TD3 Entegresi (0x21) üzerindeki bağlantılar:
// P0-P3: Tuş Takımı Tarama
// P4: LA1, P5: LA2, P6: LA3, P7: Backlight
#define LED_LA1       1//0x10  // Binary: 0001 0000 (P4)
#define LED_LA2       2//0x20  // Binary: 0010 0000 (P5)
#define LED_LA3       4//0x40  // Binary: 0100 0000 (P6)
#define LED_BACKLIGHT 16//0x80  // Binary: 1000 0000 (P7)

#define LED_GREEN     LED_LA1
#define LED_RED       LED_LA2
#define LED_YELLOW    LED_LA3

void keypad_init();
char keypad_read(); 
void set_led(uint8_t led_mask, bool state); 
void set_backlight(bool state);
void led_set_status(uint8_t status);

extern uint8_t current_led_status;

#endif