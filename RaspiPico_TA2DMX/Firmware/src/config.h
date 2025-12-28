#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"
#include "stdint-gcc.h"

// --- I2C (Ekran ve Keypad) ---
// GÜNCELLENDİ: GP16 (SDA) ve GP17 (SCL) olarak değiştirildi.
// Pico W pinout'una göre GP16/17 "I2C0" donanımına bağlıdır.
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     16 // GP16
#define I2C_SCL_PIN     17 // GP17

// --- GPS Bağlantısı ---
#define GPS_TX_PIN      0  // UART0 TX
#define GPS_RX_PIN      1  // UART0 RX

// --- PLL Kontrol (MC145158) ---
#define PLL_CLK_PIN     9  //2  // GP2
#define PLL_DATA_PIN    10 //3  // GP3
#define PLL_ENA_PIN     5  //6  // GP6
#define PLL_SEC_PIN     7 //7  //---?????? A2 ,, orta alttan 2  // GP7

// --- Genel Kontrol Pinleri ---
#define KEYPAD_INT_PIN  4  //15 // GP15
#define MUTE_PIN        6  //6  //10 // GP10
#define BAND_SEL0_PIN   13 //8  // GP8
#define BAND_SEL1_PIN   8  //9  // GP9

// --- Telsiz Arayüzü ---
#define SQL_ACTIVE_PIN  2  //20 // GP20------------------------------>????????
#define PTT_IN_PIN      14 //21 // GP21
#define PTT_OUT_PIN     12 //11 // GP11

// --- Buzzer (DEĞİŞTİ) ---
// GP16'yı I2C aldığı için, Buzzer'ı eski I2C pini olan GP4'e taşıdık.
#define ALERT_BUZZER    4  // GP4 (Eski SDA pini)

// --- ADC Pinleri (SWR Metre) ---
#define FWD_PWR_PIN     21 //26 // ADC0
#define REF_PWR_PIN     22 //27 // ADC1

// --- Ses Çıkışı ---
#define AUDIO_PWM_PIN   14 

// --- Yapılandırma Struct'ları ---
struct ChannelConfig {
    uint32_t frequency; // Örn: 145000 (kHz)
    int16_t  shift;     // Örn: 600
    int8_t   shift_dir; // 0: Simplex, 1: +, -1: -
    uint8_t  tone_pos;
    bool     tone_enabled;
};

#endif