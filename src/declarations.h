#ifndef DECLARATIONS_H
#define DECLARATIONS_H

#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>
#include <stdio.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "driver/dac.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <vector>
#include <algorithm>

// --- ESP32 Pin Tanımlamaları ---
#define I2C_SDA 21
#define I2C_SCL 22
#define KeypadIntPin 4
#define FWD_POWER_PIN 34
#define REF_POWER_PIN 35
#define PLL_SEC 14
#define BAND_SELECT_1 27
#define BAND_SELECT_0 23
#define PTT_OUTPUT_PIN 5
#define PTT_INPUT_PIN 12
#define RF_POWER_PIN 25
#define ALERT_PIN 13
#define MIC_PIN 26
#define SQL_ACTIVE 2
#define MUTE_PIN_1 18
#define pll_clk_pin 17
#define pll_data_pin 16
#define pll_ena_pin 19
#define GPS_RX_PIN 33
#define GPS_TX_PIN 32

// --- YENİ: DAC ile Sinüs Dalgası Üretimi İçin Tanımlamalar ---
#define SINE_TABLE_SIZE 256
const int tone_amplitude_numerator = 100;

// 8-bit (0-255) çözünürlüğünde önceden hesaplanmış sinüs dalgası tablosu
extern const uint8_t sine_wave[SINE_TABLE_SIZE];

// --- Font ve İkon Bildirimleri ---
// Bu değişkenlerin tanımlamaları (içerikleri) lcd_functions.cpp dosyasındadır.
// 'extern' anahtar kelimesi, diğer dosyalara bu değişkenlerin var olduğunu bildirir.
extern const unsigned char font[];
extern const byte SPKR[3];
extern const byte LOOP[3];
extern const byte LOCK[3];
extern const byte ARRW[3];
extern const byte ASEL[3];
extern const byte MENU[3];
extern const byte THUN[3];
extern const byte NOTE[3];
extern const byte SPLX[3];
extern const byte MINS[3];
extern const byte PLUS[3];


// --- Diğer Tanımlamalar ---
#define Serialprint(format, ...) Serial.printf(format, ##__VA_ARGS__)
#define Streamprint(stream, format, ...) stream.printf(format, ##__VA_ARGS__)
#define SERIALMENU 1
#define NEXTCMD 128
#define LASTCMD 0
#define SW_MAJOR 3
#define SW_MINOR 1
#define MODESET 64
#define MODE_NORMAL 0
#define MODE_POWERSAVING 16
#define DISPLAY_DISABLED 0
#define DISPLAY_ENABLED 8
#define BIAS_THIRD 0
#define BIAS_HALF 4
#define DRIVE_STATIC 1
#define DRIVE_2 2
#define DRIVE_3 3
#define DRIVE_4 0
#define BLINK  112
#define BLINKING_NORMAL 0
#define BLINKING_ALTERNATION 4
#define BLINK_FREQUENCY_OFF 0
#define BLINK_FREQUENCY2 1
#define BLINK_FREQUENCY1 2
#define BLINK_FREQUENCY05 3
#define LOADDATAPOINTER  0
#define BANKSELECT 120
#define BANKSELECT_O1_RAM0 0
#define BANKSELECT_O1_RAM2 2
#define BANKSELECT_O2_RAM0 0
#define BANKSELECT_O2_RAM2 1
#define DEVICE_SELECT B01100100
#define PCF8576_LCD         0x38
#define PCF8574_KEYB        0x20
#define PCF8574_KEYB_LED    0x21
#define green_led  128
#define yellow_led 64
#define red_led    32
#define backlight  16
#define minusSHIFT -1
#define noSHIFT     0
#define  SIMPLEX    0
#define plusSHIFT   1
#define RX 0
#define TX 1
#define HIGH_POWER 0
#define LOW_POWER  1
#define ALERT_OFF 0
#define ALERT_ON  100
#define NO_tone   0
#define OK_tone   1
#define ERR_tone  2
#define SUCC_tone 3
#define CTCSS_OFF 0
#define CTCSS_ON  1
#define TOTAL_TONES 20
#define TONE_CORRECTION 0.7
#define SQL_OFF 0
#define SQL_ON  1
#define TimeoutValue  100;
#define scrNORMAL 0
#define scrMENU   1
#define menuNONE   0
#define menuSQL    1
#define menuTONE   2
#define menuSCAN   3
#define menuRPT    4
#define menuMENU   5
#define DEFAULT_VHF_MINIMUM_FREQ  10720
#define DEFAULT_UHF_MINIMUM_FREQ  32000
#define DEFAULT_VHF_MAXIMUM_FREQ  13920
#define DEFAULT_UHF_MAXIMUM_FREQ  37600
#define DEFAULT_APRS_VHF_FREQ     11584
#define DEFAULT_APRS_UHF_FREQ     34600
#define DEFAULT_ISS_APRS_FREQ     11666
#define DEFAULT_ISS_APRS_UHF_FREQ 35004 
#define DEFAULT_VHF_SCAN_LOWER    11520
#define DEFAULT_UHF_SCAN_LOWER    34400
#define DEFAULT_VHF_SCAN_UPPER    11680
#define DEFAULT_UHF_SCAN_UPPER    35200
#define _1200   1
#define _2400   0
#define _FLAG       0x7e
#define _CTRL_ID    0x03
#define _PID        0xf0
#define _DT_EXP     ','
#define _DT_STATUS  '>'
#define _DT_POS     '!'
#define _FIXPOS         1
#define _STATUS         2
#define _FIXPOS_STATUS  3

// --- Struct Tanımlamaları ---
struct channel_t {
  uint32_t frequency;
  int16_t  shift;
  int8_t   shift_dir    = -1;
  uint8_t  tone_pos     = 0x08;
  uint8_t  tone_enabled = 0;
};

// GÜNCELLENDİ: memorych_t yapısı modernize edildi.
struct memorych_t {
  uint16_t frequency125;
  uint16_t shift25;
  uint8_t  tone_position;
  // --- HATA DÜZELTME: Veri tipi tutarlılığı için 'int8_t' olarak değiştirildi ---
  int8_t   shift_dir;     // 0: Simplex, 1: Artı Shift, 2: Eksi Shift
  uint8_t  tone_enabled;  // 0: Kapalı, 1: Açık
  uint8_t  power;         // YENİ: 0: Yüksek Güç, 1: Düşük Güç
  char     ChannelName[5];
};

struct freqLimits_t {
  uint16_t trx_min_125 = DEFAULT_VHF_MINIMUM_FREQ;
  uint16_t trx_max_125 = DEFAULT_VHF_MAXIMUM_FREQ;
  uint16_t scn_min_125 = DEFAULT_VHF_SCAN_LOWER;
  uint16_t scn_max_125 = DEFAULT_VHF_SCAN_UPPER;
};

// --- Global Değişken Bildirimleri (extern) ---
extern AsyncWebServer server;
extern DynamicJsonDocument wifiConfig;
extern hw_timer_t* dac_timer;
extern volatile int phase_accumulator;
extern HardwareSerial GPS_Serial;
extern TinyGPSPlus gps;
extern byte Led_Status;
extern byte KeyVal;
extern byte old_KeyVal;
extern bool pttToggler;
extern bool web_ptt_active; // YENİ: Web arayüzü PTT durumunu tutan değişken
extern bool web_scan_request; // YENİ: Web'den tarama isteği geldiğini belirten bayrak.
extern bool web_vna_request;  // YENİ: Web'den VNA isteği geldiğini belirten bayrak.
extern byte radio_type;
extern byte TRX_MODE;
extern byte LST_MODE;
extern byte RF_POWER_STATE;
extern byte ALERT_MODE;
extern byte CHANNEL_BUSY;
extern int SQL_MODE;
extern int scrTimer;
extern char pressedKEY;
extern byte scrMODE;
extern byte subMENU;
extern boolean hasASEL;
extern boolean hasLOCK;
extern boolean hasSPKR;
extern boolean hasTHUN;
extern boolean hasARRW;
extern boolean hasMENU;
extern boolean hasLOOP;
extern boolean hasNOTE;
extern byte Position_Signs[8][3];
extern unsigned char matrix[24];
extern unsigned char chr2wr[3];
extern const char* keymap[4];
extern const char numbers[];
extern const char font_index[];
extern byte numChar;
extern char FRQ[9];
extern char FRQ_old[9];
extern boolean validFRQ;
extern char SHIFT_INPUT[6];
extern byte shiftInputPos;
extern float ctcss_tone_list[TOTAL_TONES];
extern byte old_ctcss_tone_pos;
extern int old_frqSHIFT;
extern channel_t current_ch;
extern memorych_t memoryChannels[100];
extern freqLimits_t freqLimits;

// YENİ: APRS Frekansları için global değişkenler
extern uint16_t aprs_freq_125;
extern uint16_t iss_freq_125;

extern float minSWR;
extern long lowestFRQ;
extern long highestFRQ;
extern String commandString;
extern bool commandComplete;
extern String currentIP;
extern String ipToDisplay;
extern String greetingMessage; // DEĞİŞİKLİK: CALLSIGN -> greetingMessage
extern uint16_t tot_timer;     // YENİ: Gönderme Zaman Aşımı (TOT)
extern String mycall;
extern String APRS_Message;
extern String lat;
extern String lon;
extern unsigned int APRS_Timeout;
extern unsigned long APRS_Counter;
extern bool use_gps;
extern bool tot_lockout; // YENİ: TOT kilitleme bayrağı
extern bool nada;
extern const float baud_adj;
extern const float adj_1200;
extern const float adj_2400;
extern unsigned int tc1200;
extern unsigned int tc2400;
extern char myssid;
extern const char *dest;
extern const char *digi;
extern char digissid;
extern const char sym_ovl;
extern const char sym_tab;
extern char bit_stuff;
extern unsigned short crc;

// --- Fonksiyon Prototip Bildirimleri ---

// web_functions.cpp
void startAPMode();
void connectToWiFi();
void setupWebServer();
void handleKeypress(char key); // YENİ: Web'den gelen tuş basımlarını işleyecek fonksiyon

// radio_functions.cpp
void IRAM_ATTR onDacTimer();
void startDacTone(float frequency);
void stopDacTone();
void send_SPIBit(int Counter, byte length);
void send_SPIEnable();
void SetTone(int toneSTATE);
void Alert_Tone(int ToneType);
void SetPLLLock(uint32_t Frequency);
void write_FRQ(uint32_t Frequency);
void SetRFPower();
void readRfPower();

// lcd_functions.cpp
void InitLCD();
void sendToLcd(byte *data, byte position);
void writeToLcd(const char text[]);
void writeFRQToLcd(const char frq[9]);
void scroll(const char *text, int speed);
void displayAndScrollOnce(const char* text);
void Greetings();
void write_SHIFTtoLCD(uint16_t FRQshift);
void write_TONEtoLCD(unsigned long tone_pos);

// helper_functions.cpp
// GÜNCELLENDİ: Hata düzeltmesi için fonksiyon bildirimleri güncellendi.
void createDefaultSettings(bool forceUHF, bool fullReset);
void applyBandSpecificDefaults(bool forceUHF);
void initializeFileSystem();
bool loadConfig();
void saveConfig();
bool loadAprs();
void saveAprs();
bool loadCurrentChannel();
void saveCurrentChannel();
bool loadMemoryChannels();
void saveMemoryChannels();
bool loadWifiConfig();
void saveWifiConfig();
void createDefaultWifiConfig();
void set_nada_1200(void);
void set_nada_2400(void);
void set_nada(bool nada);
void send_char_NRZI(unsigned char in_byte, bool enBitStuff);
void send_string_len(String in_string, int len);
void calc_crc(bool in_bit);
void send_crc(void);
void send_packet(char packet_type, uint32_t frequency);
void send_flag(unsigned char flag_len);
void send_header(void);
void send_payload(char type);
void getGPSData();
boolean Calculate_Frequency(char mFRQ[9]);
void numberToFrequency(uint32_t Freq, char *rFRQ);
void StoreFrequency(char mCHNL[9], char mFRQ[9]);
void GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg);
void GetMemoryChannel(char mFRQ[9]);
void PrintMenu();
void commandYardim(char komut);
void commandCevrim(char komut);
void commandAcilis();
void commandAyarDok();
void commandHafizaDok();
void commandAPRSSure();
void commandAPRSMesaj();
void getFSData();
void commandAPRSmycall();
void commandTogglePTT();
int readColumn();
int readRow();
void StoreSpecialFrequency(char mCHNL[9], char mFRQ[9]);
void startScan();
void serialEvent();

#endif // DECLARATIONS_H

