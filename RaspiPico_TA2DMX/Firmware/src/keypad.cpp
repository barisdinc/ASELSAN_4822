#include "keypad.h"
#include "config.h"
#include "hardware/i2c.h"
#include <stdio.h>

const char* keymap[4] = {
    "123DSX", "456RB", "789OC", "*0#UM"
};

// Varsayılan: Backlight AÇIK (0), LEDler KAPALI (1) -> 0xE7
uint8_t current_led_status = 0xE7; 

void pcf_write(uint8_t addr, uint8_t data) {
    i2c_write_blocking(I2C_PORT, addr, &data, 1, false);
}

uint8_t pcf_read(uint8_t addr) {
    uint8_t data = 0xFF;
    i2c_read_blocking(I2C_PORT, addr, &data, 1, false);
    return data;
}

void keypad_init() {
    pcf_write(PCF8574_KEYB_ADDR, 0xFF);
    current_led_status = 0xE7;
    pcf_write(PCF8574_LED_ADDR, current_led_status);
}

void led_set_status(uint8_t status) {
    current_led_status = status;
    pcf_write(PCF8574_LED_ADDR, current_led_status);
}

void set_led(uint8_t led_mask, bool state) {
    uint8_t old_status = current_led_status;
    
    if (state) current_led_status &= ~led_mask; // 0 = YANIK
    else current_led_status |= led_mask;        // 1 = SÖNÜK
    
    // Sadece durum değiştiyse I2C'ye yaz (Trafiği azaltır)
    if (current_led_status != old_status) {
        led_set_status(current_led_status);
    }
}

void set_backlight(bool state) {
    set_led(LED_BACKLIGHT, state);
}

int read_row() {
    // LED portu üzerinden sütunları LOW'a çekiyoruz
    // current_led_status'u kullanıyoruz ki yanan LED sönmesin
    // Ancak tarama için alt 4 biti (P0-P3) sıfırlıyoruz.
    uint8_t scan_val = current_led_status & 0xF0; 
    pcf_write(PCF8574_LED_ADDR, scan_val);

    uint8_t r = pcf_read(PCF8574_KEYB_ADDR);
    
    // LED durumunu geri yükle
    pcf_write(PCF8574_LED_ADDR, current_led_status);

    r = 255 - (r | 0b00000011);
    r = r >> 3;

    if (r == 16) return 0;
    if (r ==  8) return 1;
    if (r ==  4) return 2;
    if (r ==  2) return 3;
    if (r ==  1) return 4;
    return -1;
}

int read_col() {
    pcf_write(PCF8574_KEYB_ADDR, 0x00);
    
    // Sütunları okumak için Input(1) yapmalıyız.
    // Ancak Backlight(Bit 4) durumunu korumalıyız yoksa ışık titrer.
    // LED'leri (Bit 0-3) 1 yapıyoruz (Söndür/Input Modu).
    uint8_t input_mask = (current_led_status & 0xF0) | 0x0F;
    pcf_write(PCF8574_LED_ADDR, input_mask); 

    uint8_t c = pcf_read(PCF8574_LED_ADDR);
    
    pcf_write(PCF8574_KEYB_ADDR, 0xFF); 
    pcf_write(PCF8574_LED_ADDR, current_led_status); // Eski duruma dön

    c = 255 - c;

    if (c == 8) return 0;
    if (c == 4) return 1;
    if (c == 2) return 2;
    if (c == 1) return 3;
    return -1;
}

char keypad_read() {
    int satir = read_row();
    if (satir != -1) {
        int sutun = read_col();
        if (sutun != -1 && satir < 6) {
             sleep_ms(20); 
             if (read_row() == satir) return keymap[sutun][satir];
        }
    }
    return 0; 
}