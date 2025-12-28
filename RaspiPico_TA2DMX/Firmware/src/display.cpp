#include "display.h"
#include "hardware/i2c.h"
#include "config.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

// --- PCF8576 ve LCD Sabitleri ---
#define PCF8576_ADDR        0x38

#define NEXTCMD             0x80
#define LASTCMD             0x00

// LCD Ayar Byte'ları (Orijinal koddan)
#define MODESET             64
#define MODE_POWERSAVING    16
#define DISPLAY_ENABLED     8
#define BIAS_THIRD          0
#define DRIVE_4             0
#define BLINK               112
#define BLINKING_ALTERNATION 4
#define BLINK_FREQUENCY_OFF 0
#define LOADDATAPOINTER     0
#define BANKSELECT          120
#define DEVICE_SELECT       0x64

// Ayar Kombinasyonları
const uint8_t set_modeset = MODESET | MODE_POWERSAVING | DISPLAY_ENABLED | BIAS_THIRD | DRIVE_4;
const uint8_t set_blink = BLINK | BLINKING_ALTERNATION | BLINK_FREQUENCY_OFF;
const uint8_t set_datapointer = LOADDATAPOINTER | 0;
const uint8_t set_deviceselect = DEVICE_SELECT;

// --- İKON TANIMLAMALARI ---
const uint8_t SPKR[3] = { 0b00000000, 0b00000000, 0b10000000 }; //pos 1 
const uint8_t LOOP[3] = { 0b00000000, 0b00010000, 0b00000000 }; //pos 1
const uint8_t LOCK[3] = { 0b00000000, 0b10000000, 0b00000000 }; //pos 1
const uint8_t ARRW[3] = { 0b00000000, 0b00010000, 0b00000000 }; //pos 2
const uint8_t ASEL[3] = { 0b00000000, 0b10000000, 0b00000000 }; //pos 2
const uint8_t MENU[3] = { 0b00000000, 0b00000000, 0b10000000 }; //pos 7
const uint8_t THUN[3] = { 0b00000000, 0b00000000, 0b00100000 }; //pos 7
const uint8_t NOTE[3] = { 0b00000000, 0b00000000, 0b00010000 }; //pos 7
const uint8_t PLUS[3] = { 0b00100101, 0b01000000, 0b00000000 }; //pos 7
const uint8_t MINS[3] = { 0b00100000, 0b01000000, 0b00000000 }; //pos 7
const uint8_t SPLX[3] = { 0b00000000, 0b00000000, 0b00000000 }; //pos 7

// Karakter İndeksi
const char font_index[] = "_ /-.*!?<>[]ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789%";

// --- EKSİKSİZ FONT DİZİSİ ---
// Orijinal 'main.cpp' içindeki binary format (Bxxxx) C++ standartı olan 0bxxxx formatına çevrildi.
const unsigned char font[] = {
  0b00010000, 0b00000000, 0b00001111, // _ underscore
  0b00000000, 0b00000000, 0b00000000, // space
  0b01000000, 0b00000100, 0b00000000, // / division
  0b00100000, 0b00000100, 0b00000000, // - dash
  0b00000000, 0b00000000, 0b00010000, // . dot
  0b11100101, 0b00000111, 0b00000000, // * asterix
  0b00000101, 0b00000000, 0b00010000, // ! exclamation mark
  0b00000011, 0b00000010, 0b00110000, // ? question mark
  0b00000000, 0b00000011, 0b00000000, // < less than
  0b11000000, 0b00000000, 0b00000000, // > greater than
  0b00010010, 0b00000000, 0b01100000, // [ bracket open
  0b00010010, 0b01100000, 0b00000000, // ] bracket close
  0b00100010, 0b01100100, 0b01100000, //A
  0b00110010, 0b01100100, 0b01100000, //B
  0b00010010, 0b00000000, 0b01100000, //C
  0b00010010, 0b01100000, 0b01100000, //D
  0b00110010, 0b00000100, 0b01100000, //E
  0b00100010, 0b00000000, 0b01100000, //F
  0b00110010, 0b01000100, 0b01100000, //G
  0b00100000, 0b01100100, 0b01100000, //H
  0b00010111, 0b00000000, 0b00000000, //I
  0b01000110, 0b00000000, 0b01000000, //J
  0b00000101, 0b00000011, 0b00000000, //K
  0b00010000, 0b00000000, 0b01100000, //L
  0b10000000, 0b01100010, 0b01100000, //M
  0b10000000, 0b01100001, 0b01100000, //N
  0b00010010, 0b01100000, 0b01100000, //O
  0b00100010, 0b00100100, 0b01100000, //P
  0b00010010, 0b01100001, 0b01100000, //Q
  0b00100010, 0b00100101, 0b01100000, //R
  0b00110010, 0b01000100, 0b00100000, //S
  0b00000111, 0b00000000, 0b00000000, //T
  0b00010000, 0b01100000, 0b01100000, //U
  0b10000000, 0b01100001, 0b00000000, //V
  0b01000000, 0b01100001, 0b01100000, //W
  0b11000000, 0b00000011, 0b00000000, //X
  0b10000001, 0b00000010, 0b00000000, //Y
  0b01010010, 0b00000010, 0b00000000, //Z
  0b00010010, 0b01100000, 0b01100000, //O (Sıfır benzeri)
  0b00000000, 0b01100010, 0b00000000, //1
  0b00110010, 0b00100100, 0b01000000, //2
  0b00110010, 0b01100100, 0b00000000, //3
  0b00100000, 0b01100100, 0b00100000, //4
  0b00110010, 0b01000100, 0b00100000, //5
  0b00110010, 0b01000100, 0b01100000, //6
  0b00000010, 0b01100000, 0b00000000, //7
  0b00110010, 0b01100100, 0b01100000, //8
  0b00110010, 0b01100100, 0b00100000, //9
  0b00000000, 0b00000000, 0b11110000, //% Tüm segmentleri yakmak için özel karakter
};

// Global Değişkenler
unsigned char matrix[24];
unsigned char chr2wr[3];
uint8_t Position_Signs[8][3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };

// I2C Yardımcı Fonksiyon
void i2c_send_data(uint8_t *data, size_t len) {
    i2c_write_blocking(I2C_PORT, PCF8576_ADDR, data, len, false);
}

void sendToLcd(uint8_t *data, uint8_t position) {
    uint8_t buffer[5];
    buffer[0] = NEXTCMD | set_deviceselect;
    buffer[1] = LASTCMD | (position * 5); // Adres hesaplama
    buffer[2] = data[0];
    buffer[3] = data[1];
    buffer[4] = data[2];
    
    i2c_send_data(buffer, 5);
}

void display_init() {
    // I2C Başlatma
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Ekranı Başlatma Komutları
    // 1. Ekranı Temizle (Mode Set)
    uint8_t init_cmd1[24];
    init_cmd1[0] = NEXTCMD | set_modeset;
    init_cmd1[1] = NEXTCMD | set_deviceselect;
    init_cmd1[2] = NEXTCMD | set_blink;
    init_cmd1[3] = LASTCMD | set_datapointer;
    for(int i=0; i<20; i++) init_cmd1[4+i] = 0x00;
    i2c_send_data(init_cmd1, 24);
    sleep_ms(100);

    // 2. Tüm Segmentleri Yak (Test)
    uint8_t init_cmd2[24];
    init_cmd2[0] = NEXTCMD | set_modeset;
    init_cmd2[1] = NEXTCMD | set_deviceselect;
    init_cmd2[2] = NEXTCMD | set_blink;
    init_cmd2[3] = LASTCMD | set_datapointer;
    for(int i=0; i<20; i++) init_cmd2[4+i] = 0xFF;
    i2c_send_data(init_cmd2, 24);
    sleep_ms(500);

    // 3. Normal Çalışma Modu (Temizle)
    i2c_send_data(init_cmd1, 24);
    sleep_ms(100);
}

void display_write_text(const char* text) {
    memset(chr2wr, 0, 3);
    size_t len = strlen(text);

    for (uint8_t idx = 0; idx < len; idx++) {
        if (idx > 7) break; // Ekran 8 karakterlik

        char c_char = (char)toupper((unsigned char)text[idx]);
        const char *ptr = strchr(font_index, c_char);
        int pos = 0;

        if (ptr == NULL) {
            pos = 0; // Bulunamazsa underscore
        } else {
            pos = ptr - font_index;
        }

        // Matris Hesaplama (Orijinal Mantık)
        // İlk BYTE
        matrix[3*idx+0] = font[(pos * 3)+0] | Position_Signs[idx][0]; 
        // İkinci BYTE
        matrix[3*idx+1] = font[(pos * 3)+1] | Position_Signs[idx][1]; 
        
        // Üçüncü BYTE (Önceki karakterle birleşim mantığı)
        if (idx > 0) {
            matrix[3*idx+2] = font[(pos * 3)+2] | (matrix[3*(idx+1)] & 0x0F) | Position_Signs[idx][2];
        } else {
            matrix[3*idx+2] = font[(pos * 3)+2] | Position_Signs[idx][2];
        }

        chr2wr[0] = matrix[3*idx+0];
        chr2wr[1] = matrix[3*idx+1];
        chr2wr[2] = matrix[3*idx+2];
        
        sendToLcd(chr2wr, idx);
    }
}

// Orijinal writeFRQToLcd fonksiyonuna karşılık gelir
void display_write_freq(const char* freq_str) {
    // İkonları sıfırla
    memset(Position_Signs, 0, sizeof(Position_Signs));

    // Not: Normalde burada global radyo durumlarına (SQL, SHIFT vb.) bakılarak
    // Position_Signs dizisi güncellenir.
    // Örnek:
    // Position_Signs[0][0] |= SPKR[0]; 
    
    // Frekansı yaz
    display_write_text(freq_str);
}