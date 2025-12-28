#include "keypad.h"
#include "config.h"
#include "hardware/i2c.h"
#include <stdio.h>

const char* keymap[4] = {
    "123DSX", "456RB", "789OC", "*0#UM"
};

// Varsayılan Başlangıç:
// 1 = SÖNÜK (OFF), 0 = YANIK (ON)
// LED'ler (P4,P5,P6) Sönük olsun -> 1
// Backlight (P7) Sönük olsun -> 1
// Keypad Pinleri (P0-P3) Input Modunda (High) kalsın -> 1
// Sonuç: 1111 1111 -> 0xFF
uint8_t current_led_status = 0xFF; 

void pcf_write(uint8_t addr, uint8_t data) {
    i2c_write_blocking(I2C_PORT, addr, &data, 1, false);
    printf("W %x\n", addr);
}

uint8_t pcf_read(uint8_t addr) {
    uint8_t data = 0xFF;
    i2c_read_blocking(I2C_PORT, addr, &data, 1, false);
    printf("R %x\n", addr);
    return data;
}

void keypad_init() {
    //Onterrupt pinini ayarla
    gpio_init(KEYPAD_INT_PIN);
    gpio_set_dir(KEYPAD_INT_PIN, GPIO_IN);
    gpio_pull_up(KEYPAD_INT_PIN); 


    // TD2 (Sadece Keypad): Hepsini High yap (Input/Passive)
    pcf_write(PCF8574_KEYB_ADDR, 0xFF);
    
    // TD3 (LED + Keypad): Hepsini High yap (LEDler sönük, Tuşlar Input)
    current_led_status = 0x00;//FF;
    pcf_write(PCF8574_LED_ADDR, current_led_status);

}

void led_set_status(uint8_t status) {
    current_led_status = status;
    // Sadece LED portuna yazıyoruz (0x21)
    pcf_write(PCF8574_LED_ADDR, current_led_status);
}

void set_led(uint8_t led_mask, bool state) {
    // state: true = YAK, false = SÖNDÜR
    // Donanım Active Low (0=Yanar, 1=Söner)
    uint8_t old_status = current_led_status;
    
    if (state) {
        // YAKMAK için biti 0 yap (AND işlemi)
        current_led_status &= ~led_mask; 
    } else {
        // SÖNDÜRMEK için biti 1 yap (OR işlemi)
        current_led_status |= led_mask;        
    }
    
    if (current_led_status != old_status) {
        led_set_status(current_led_status);
        printf("led : %d \n", led_mask);
    }
}

void set_backlight(bool state) {
    set_led(LED_BACKLIGHT, state);
}

int read_row() {
    // TD3 (0x21) üzerinden satır taraması yapacağız.
    // P0-P3 (Tuşlar) LOW'a çekilmeli, ama P4-P7 (LEDler) korunmalı.
    
    // 1. Mevcut LED durumunu al (Örn: 1110 1111 -> Backlight açık)
    uint8_t scan_val = current_led_status;
    
    // 2. Alt 4 biti (P0-P3) sıfırla (LOW yap). 
    // Üst 4 bit (LEDler) olduğu gibi kalır, böylece ışık titreşmez.
    scan_val &= 0xF0; 
    
    pcf_write(PCF8574_LED_ADDR, scan_val); // 0x21'e yaz

    // TD2'den (0x20) okuma yap
    uint8_t r = pcf_read(PCF8574_KEYB_ADDR);
    
    // Tarama bitti, pinleri eski haline (Input/High) getir.
    // LED durumunu (current_led_status) geri yüklüyoruz.
    pcf_write(PCF8574_LED_ADDR, current_led_status);

    // Okunan veriyi işle (Active Low olduğu için tersini al)
    r = 255 - (r | 0b00000011); // Bit maskeleme (Devreye özgü gürültü filtresi olabilir)
    r = r >> 3;

    if (r == 16) return 0;
    if (r ==  8) return 1;
    if (r ==  4) return 2;
    if (r ==  2) return 3;
    if (r ==  1) return 4;
    return -1;
}

int read_col() {
    // Bu sefer TD2 (0x20) hepsini LOW yapıyoruz
    pcf_write(PCF8574_KEYB_ADDR, 0x00);
    
    // TD3'ü (0x21) okumak için alt 4 biti Input (1) yapmalıyız.
    // Yine LED'leri (Üst 4 bit) korumalıyız.
    
    // Mevcut LED durumu | 0x0F (Alt 4 bit 1 olsun ki okuma yapabilelim)
    uint8_t input_mask = (current_led_status & 0xF0) | 0x0F;
    pcf_write(PCF8574_LED_ADDR, input_mask); 

    uint8_t c = pcf_read(PCF8574_LED_ADDR);
    
    // Eski durumlara dön
    pcf_write(PCF8574_KEYB_ADDR, 0xFF); 
    pcf_write(PCF8574_LED_ADDR, current_led_status); 

    // Sadece alt 4 bit (tuşlar) bizi ilgilendiriyor
    c = c & 0x0F; 
    c = 15 - c; // 15 (0x0F) - okunan değer (tersleme)

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
        // Şemaya göre keypad 4x6 değil, farklı bir matris olabilir 
        // ama kodunuzdaki matris mantığını korudum.
        if (sutun != -1 && satir < 6) {
             sleep_ms(20); 
             // Debounce kontrolü (Basit)
             if (read_row() == satir) return keymap[sutun][satir];
        }
    }
    return 0; 
}