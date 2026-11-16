#ifndef MAIN_H
#define MAIN_H

#include "Arduino.h"
#include <EEPROM.h>
#include "../lib/fontsandicons.h"
#include <Wire.h>
#include "Config.h"

#define SERIALMENU 1
#define RESETWATCHDOG

#define NEXTCMD 128
#define LASTCMD 0

#define SW_MAJOR 3
#define SW_MINOR 5

#define green_led  128
#define yellow_led 64
#define red_led    32
#define backlight  16

extern uint8_t Led_Status;
extern uint8_t KeypadIntPin;
extern uint8_t KeyVal;
extern uint8_t old_KeyVal;

#define FWD_POWER_PIN 21
#define REF_POWER_PIN 22

extern bool pttToggler;

#define PLL_SEC        18
#define BAND_SELECT_1  13
#define BAND_SELECT_0  8

extern uint8_t radio_type;

#define minusSHIFT -1
#define noSHIFT     0
#define SIMPLEX     0
#define plusSHIFT   1
extern int old_frqSHIFT;

#define PTT_OUTPUT_PIN 5
#define PTT_INPUT_PIN  14

#define RX 0
#define TX 1
extern uint8_t TRX_MODE;
extern uint8_t LST_MODE;

#define RF_POWER_PIN     15
#define AUDIO_RX_ADC_PIN 26
#define TNC_PTT_PIN      12
#define HIGH_POWER       0
#define LOW_POWER        1
extern uint8_t RF_POWER_STATE;
extern uint8_t micTxMode;

#define AUDIO_RING_SIZE  4096
extern volatile uint8_t  audioRxRing[AUDIO_RING_SIZE];
extern volatile uint16_t audioRxHead;
extern volatile uint16_t audioRxTail;
extern volatile uint8_t  audioRxPeak;
extern volatile uint8_t  audioTxRing[AUDIO_RING_SIZE];
extern volatile uint16_t audioTxHead;
extern volatile uint16_t audioTxTail;
extern volatile bool     webAudioRxActive;
extern volatile unsigned long lastAudioRxPollMs;
extern volatile bool     webPttActive;
extern volatile unsigned long lastWebPttMs;
void startWebAudioEngine();
void setWebPttState(bool on);

#define ALERT_PIN  28
#define ALERT_OFF  0
#define ALERT_ON   100
extern uint8_t ALERT_MODE;

#define NO_tone   0
#define OK_tone   1
#define ERR_tone  2
#define SUCC_tone 3

#define MIC_PIN  10

#define CTCSS_OFF    0
#define CTCSS_ON     1
#define TOTAL_TONES  39
#define TONE_CORRECTION 0.7
extern float ctcss_tone_list[TOTAL_TONES];
extern uint8_t old_ctcss_tone_pos;

#define SQL_ACTIVE 2
#define MUTE_PIN_1 6
extern uint8_t CHANNEL_BUSY;

#define pll_clk_pin  9
#define pll_data_pin 11
#define pll_ena_pin  7

#define PLL_MODE_COUNT 5
extern uint8_t  pllMode;
extern uint16_t pllBitDelayUs;
extern bool     pllDoubleWrite;
extern uint16_t pllRefreshMs;
extern uint16_t pllR;
extern int8_t   uhfBandOverride;
void pllApplyMode(uint8_t m);

extern uint8_t pttActiveLevel;
void pttKey(bool on);

#define WIFI_AT_BOOT     1
#define PLL_TX_SETTLE_MS 20
#define PLL_DEBUG

#define SQL_OFF 0
#define SQL_ON  1
extern int SQL_MODE;

extern int scrTimer;
#define TimeoutValue 1000
extern char pressedKEY;

#define scrNORMAL 0
#define scrMENU   1
extern uint8_t scrMODE;

#define MODE_VFO 0
#define MODE_MR  1
extern uint8_t op_mode;
extern uint8_t current_memory_channel;

#define menuNONE 0
#define menuSQL  1
#define menuTONE 2
#define menuSCAN 3
#define menuRPT  4
#define menuMENU 5
extern uint8_t subMENU;

extern uint8_t numChar;
extern char FRQ[9];
extern char FRQ_old[9];
extern boolean validFRQ;

#define Serialprint(format, ...) StreamPrint_progmem(Serial, PSTR(format), ##__VA_ARGS__)

extern uint8_t TOT_TX_TIME[16];
extern uint8_t TOT_TX_LOCK_TIME[16];
extern unsigned long lastPllRefresh;
extern unsigned long prevTotMillis;

struct tot_t {
  uint8_t time_out_time = 60;
  uint8_t lock_time = 5;
};
extern tot_t tot_values;

struct channel_t {
  uint32_t frequency;
  uint8_t  bozuk;
  int16_t  shift;
  int8_t   shift_dir    = -1;
  uint8_t  tone_pos     = 0x08;
  uint8_t  tone_enabled = 0;
};
extern channel_t current_ch;
extern uint16_t freq_step;

struct memorych_t {
  uint16_t frequency25;
  uint8_t  shift25;
  uint8_t  tone_position;
  uint8_t  SSTP;
  char     ChannelName[7];
};

struct freqLimits_t {
  uint16_t trx_min_25 = DEFAULT_VHF_MINIMUM_FREQ;
  uint16_t trx_max_25 = DEFAULT_VHF_MAXIMUM_FREQ;
  uint16_t scn_min_25 = DEFAULT_VHF_SCAN_LOWER;
  uint16_t scn_max_25 = DEFAULT_VHF_SCAN_UPPER;
  uint16_t aprs_25    = DEFAULT_APRS_VHF_FREQ;
  uint16_t iss_25     = DEFAULT_ISS_APRS_FREQ;
  uint16_t vna_min_25 = DEFAULT_VHF_VNA_MINIMUM_FREQ;
  uint16_t vna_max_25 = DEFAULT_VHF_VNA_MAXIMUM_FREQ;
};
extern freqLimits_t freqLimits;

// EEPROM Adres Tanimlari
#define EEPROM_CONFDATA_BLCKSTART   0
#define EEPROM_CHECKIT_ADDR         0
#define EEPROM_VERSMAJ_ADDR         1
#define EEPROM_VERSMIN_ADDR         2
#define EEPROM_CLLSIGN_ADDR         3
#define EEPROM_MESSAGE_ADDR         9
#define EEPROM_RADIOTP_ADDR         17
#define EEPROM_OP_MODE_ADDR         18
#define EEPROM_CURR_MEMCH_ADDR      19
#define EEPROM_APRSMSG_ADDR         20
#define EEPROM_TOT                  50
#define EEPROM_CURRFRQ_ADDR         50
#define EEPROM_CURRSHF_ADDR         52
#define EEPROM_CURRTON_ADDR         54
#define EEPROM_APRSDATA_BLCKSTART   60
#define EEPROM_APRSCLL_ADDR         60
#define EEPROM_APRSTIM_ADDR         66
#define EEPROM_APRSLAT_ADDR         67
#define EEPROM_APRSLON_ADDR         75
#define EEPROM_SPECIALFRQ_BLCKSTART 84
#define EEPROM_CURRCHNL_BLCKSTART   100
#define EEPROM_MEMDATA_BLCKSTART    120
#define EEPROM_CHNNL01_ADDR         120
#define EEPROM_CHNNL_SIZE           12

// WebUI & Cihaz Ek Ayarlari (750..910)
#define EEPROM_SETUP_DONE_ADDR 750
#define SETUP_MAGIC_DONE       0xA5
#define EEPROM_APRS_SSID_ADDR  751
#define EEPROM_RFPOWER_ADDR    752
#define EEPROM_SQLMODE_ADDR    753
#define EEPROM_WIFI_PASS_ADDR  754
#define WIFI_PASS_MAX_LEN      32
#define EEPROM_SQLLEVEL_ADDR   788
#define EEPROM_LAYOUT_VER_ADDR 789
#define EEPROM_LAYOUT_VERSION  0xA6

#define EEPROM_NET_MAGIC_ADDR  790
#define NET_MAGIC_VALID        0xB2
#define EEPROM_WIFI_MODE_ADDR  791
#define EEPROM_STA_SSID_ADDR   792
#define EEPROM_STA_PASS_ADDR   825
#define EEPROM_STA_IP_ADDR     858
#define EEPROM_PEER_IP_ADDR    875

extern String wifiPass;
extern bool isSetupDone;
extern uint8_t sql_level;
extern uint8_t wifiMode;
extern String staSsid;
extern String staPass;
extern String staStaticIp;
extern String peerIp;

extern float minSWR;
extern long lowestFRQ;
extern long highestFRQ;

extern String commandString;
extern bool commandComplete;

// APRS Tanimlari
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

extern bool nada;
extern const float baud_adj;
extern const float adj_1200;
extern const float adj_2400;
extern unsigned int tc1200;
extern unsigned int tc2400;

extern String mycall;
extern String APRS_Message;
extern String lat;
extern String lon;
extern unsigned int APRS_Timeout;
extern unsigned long APRS_Counter;
extern String webLogBuffer;

extern char myssid;
extern const char *dest;
extern const char *digi;
extern char digissid;
extern const char sym_ovl;
extern const char sym_tab;

extern char bit_stuff;
extern unsigned short crc;

// PCF8576 LCD ve PCF8574 Tus Takimi Tanimlari
#define PCF8576_LCD      0x38
#define PCF8574_KEYB     0x20
#define PCF8574_KEYB_LED 0x21

#define MODESET          64
#define MODE_NORMAL      0
#define MODE_POWERSAVING 16
#define DISPLAY_DISABLED 0
#define DISPLAY_ENABLED  8
#define BIAS_THIRD       0
#define BIAS_HALF        4
#define DRIVE_STATIC     1
#define DRIVE_2          2
#define DRIVE_3          3
#define DRIVE_4          0
extern uint8_t set_modeset;

#define BLINK                112
#define BLINKING_NORMAL      0
#define BLINKING_ALTERNATION 4
#define BLINK_FREQUENCY_OFF  0
#define BLINK_FREQUENCY2     1
#define BLINK_FREQUENCY1     2
#define BLINK_FREQUENCY05    3
extern uint8_t set_blink;

#define LOADDATAPOINTER 0
extern uint8_t set_datapointer;

#define BANKSELECT         120
#define BANKSELECT_O1_RAM0 0
#define BANKSELECT_O1_RAM2 2
#define BANKSELECT_O2_RAM0 0
#define BANKSELECT_O2_RAM2 1
extern uint8_t set_bankselect;

#define DEVICE_SELECT B01100100
extern uint8_t set_deviceselect;

extern boolean hasASEL;
extern boolean hasLOCK;
extern boolean hasSPKR;
extern boolean hasTHUN;
extern boolean hasARRW;
extern boolean hasMENU;
extern boolean hasLOOP;
extern boolean hasNOTE;

extern uint8_t Position_Signs[8][3];
extern unsigned char matrix[24];
extern unsigned char chr2wr[3];
extern const char* keymap[4];
extern const char numbers[];
extern const char font_index[];

void InitLCD();
void sendToLcd(uint8_t *data, uint8_t position);
void writeToLcd(const char text[]);
void writeFRQToLcd(const char frq[9]);
void scroll(const char *text, int speed);
void Greetings();
void send_SPIBit(int Counter, uint8_t length);
void send_SPIEnable();
void pllWriteWords(uint32_t f);
void pllWriteNA(int N_Counter, int A_Counter);
int readColumn();
int readRow();
void SetTone(int toneSTATE);
void Alert_Tone(int ToneType);
void SetPLLLock(uint32_t Frequency);
void write_FRQ(uint32_t Frequency);
void write_SHIFTtoLCD(uint16_t FRQshift);
void write_TONEtoLCD(unsigned long tone_pos);
void SetRFPower();
void readRfPower();
void numberToFrequency(uint32_t Freq, char *rFRQ);
void eeread_nbytes(String *dst, uint8_t sz, uint16_t addr);
void eeprom_readAPRS();
void eewrite_nbytes(String dst, uint8_t sz, uint16_t addr);
void eeprom_writeAPRS();
void initialize_eeprom();
void StoreFrequency(char mCHNL[9], char mFRQ[9]);
void GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg);
void GetMemoryChannel(char mFRQ[9]);
void PrintMenu();
void commandRadioType(char komut);
void commandStartupMSG();
void commandDumpConfig();
void commandMemoryDump();
void commandAPRSTimeout();
void commandAPRSMessage();
void commandAPRSmycall();
void commandTogglePTT();
void StoreSpecialFrequency(char mCHNL[9], char mFRQ[9]);
void startScan();
void StreamPrint_progmem(Print &out, PGM_P format, ...);
void serialEvent();
void set_nada_1200(void);
void set_nada_2400(void);
void set_nada(bool nada);
void calc_crc(bool in_bit);
void send_crc(void);
void send_header(void);
void send_payload(char type);
void send_char_NRZI(unsigned char in_byte, bool enBitStuff);
void send_string_len(String in_string, int len);
void send_flag(unsigned char flag_len);
void send_packet(char packet_type, uint32_t frequency);
void softResetDevice();
boolean Calculate_Frequency(char mFRQ[9]);
void changeTotSettings(uint8_t timeout, uint8_t lock);

#endif
