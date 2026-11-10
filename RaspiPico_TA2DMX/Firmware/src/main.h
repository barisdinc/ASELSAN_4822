#ifndef MAIN_H
#define MAIN_H

#include "Arduino.h"
#include <EEPROM.h>
#include "../lib/fontsandicons.h"
#include <Wire.h>
#include "Config.h"

#define SERIALMENU 1
#define RESETWATCHDOG
//8576 LCD Driver settings
#define NEXTCMD 128     // Issue when there will be more commands after this one
#define LASTCMD 0       // Issue when when this is the last command before ending transmission

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

// ADC Pins (SWR) (ADC desteklenmediği için dummy)
#define FWD_POWER_PIN 21 // GP21 (A6)
#define REF_POWER_PIN 22 // GP22 (A7)

extern bool pttToggler;

#define PLL_SEC 18 // GP18 (TAMSAT A2 -> Pico A6)

// Ozel PCB icin gecerli Pin Tanimlamalari (TAMSAT'tan bagimsiz)
#define BAND_SELECT_1  13 // GP13 (D11) = BS1 (Original TAMSAT / Actually working mapping)
#define BAND_SELECT_0  8  // GP8 (D8) = BS0 (Original TAMSAT / Actually working mapping)

extern uint8_t radio_type;

#define minusSHIFT -1
#define noSHIFT     0
#define  SIMPLEX    0
#define plusSHIFT   1
extern int old_frqSHIFT;

#define PTT_OUTPUT_PIN 5  // GP5 (D5) -> PTT_OUT
#define PTT_INPUT_PIN  14 // GP14 (D12) -> PTT_IN

#define RX 0
#define TX 1
extern uint8_t TRX_MODE;
extern uint8_t LST_MODE;

#define RF_POWER_PIN 26 // GP26 (ADC0) -> POWER_CONTROL
#define HIGH_POWER 0
#define LOW_POWER  1
extern uint8_t RF_POWER_STATE;
extern uint8_t micTxMode;

#define ALERT_PIN 28 // Dummy pin for Buzzer (was 4, but that collided with KeypadIntPin)

#define ALERT_OFF 0
#define ALERT_ON  100
extern uint8_t ALERT_MODE;

#define NO_tone   0
#define OK_tone   1
#define ERR_tone  2
#define SUCC_tone 3

#define MIC_PIN  10  // GP10 (PWM0/D3)

#define CTCSS_OFF 0
#define CTCSS_ON  1
#define TOTAL_TONES 39
#define TONE_CORRECTION 0.7 
extern float ctcss_tone_list[TOTAL_TONES];
extern uint8_t old_ctcss_tone_pos;

#define SQL_ACTIVE 2 // GP2 (D2)
#define MUTE_PIN_1 6 // GP6 (D6)
extern uint8_t CHANNEL_BUSY;

#define pll_clk_pin  9  // GP9 (D9) -> SPI_CLK
#define pll_data_pin 11 // GP11 (D10) -> SPI_DATA
#define pll_ena_pin  7  // GP7 (D7) -> PLL_SEC (Enable)

// PLL (MC145158) yazim profili. Calisma aninda 'm' ile degistirilebilir.
// 0: Nano birebir (1ms bit, tek yazim, tazeleme yok)  <- varsayilan
// 1: 100us bit, tek yazim, tazeleme yok
// 2: 10us bit, tek yazim, tazeleme yok
// 3: 10us bit, cift yazim, 200ms tazeleme
// 4: 1ms bit, cift yazim, 200ms tazeleme
#define PLL_MODE_COUNT 5
extern uint8_t  pllMode;
extern uint16_t pllBitDelayUs;
extern bool     pllDoubleWrite;
extern uint16_t pllRefreshMs;
extern uint16_t pllR;
void pllApplyMode(uint8_t m);
void pllNextMode();

// PTT cikis polaritesi. Pico PCB'sinde Q1 (DTC143EKA) tersleyici olarak bagli:
// GP5 HIGH -> Q1 iletimde -> PTT_OUT LOW. TAMSAT kablolamasinda ise PTT_OUT
// dogrudan suruluyordu (HIGH = gonderim). Bu yuzden polarite calisma aninda
// 't' komutu ile denenebilir.
extern uint8_t pttActiveLevel;
void pttKey(bool on);

// WiFi acilista baslatilsin mi?
// Pico 2 W'nin CYW43 cipi acilista ve TX sirasinda 250-350 mA tepe akim ceker.
// Kartta sadece 4 x 100nF dekuplaj var; bulk (elektrolitik/tantal) kondansator yok.
// Telsizin 5V raili bu ani yuku karsilayamiyor, gerilim 5V'tan 1.5V'a dusup
// RP2350 brownout oluyor (USB bagliyken sorun gorunmez, PC portu besler).
//   1 = acilista baslat   |   0 = sadece seri menuden 'W' ile baslat
// Kalici cozum: VCC-GND arasina 470uF + 10uF bulk kondansator.
#define WIFI_AT_BOOT 1

#define PLL_TX_SETTLE_MS    20 // PTT'den once VCO'nun kilitlenmesi icin bekleme

// Seri portta her PLL yaziminda N/A degerlerini doker (ayar dogrulamasi icin)
#define PLL_DEBUG

#define SQL_OFF 0
#define SQL_ON  1
extern int SQL_MODE;

extern int scrTimer;
#define TimeoutValue  1000
extern char pressedKEY;

#define scrNORMAL 0
#define scrMENU   1
extern uint8_t scrMODE;

#define MODE_VFO 0
#define MODE_MR 1
extern uint8_t op_mode;
extern uint8_t current_memory_channel;
#define menuNONE   0
#define menuSQL    1
#define menuTONE   2
#define menuSCAN   3
#define menuRPT    4
#define menuMENU   5
extern uint8_t subMENU;



extern uint8_t numChar;
extern char FRQ[9];
extern char FRQ_old[9];
extern boolean validFRQ; //Is the calculated frequenct valid for our ranges;

//Special Frequency Definition limits are moved to Config.h

#define Serialprint(format, ...) StreamPrint_progmem(Serial,PSTR(format),##__VA_ARGS__)

//defining structures here

extern uint8_t TOT_TX_TIME[16];
extern uint8_t TOT_TX_LOCK_TIME[16];
extern unsigned long lastPllRefresh;

struct tot_t {
  uint8_t  time_out_time = 60; // default 60 (index 4)
  uint8_t  lock_time = 5;      // default 5 (index 1)
};
extern tot_t tot_values;
struct channel_t {	
  uint32_t frequency;	
  uint8_t  bozuk; //this memory location is corrupted in my development environment
  int16_t  shift;	
  int8_t   shift_dir    = -1;	  // Minus Shift
  uint8_t  tone_pos     = 0x08; //88.5 by default
  uint8_t  tone_enabled = 0;	  //Tone Off
};	
extern channel_t current_ch;
extern uint16_t freq_step;

struct memorych_t {
  uint16_t frequency25; //frequency divided by 25
  uint8_t  shift25;      //shift divided bye 25
  uint8_t  tone_position;
  uint8_t  SSTP;         //ShiftShiftTonePower 
  char     ChannelName[7];
};

struct freqLimits_t {
  uint16_t trx_min_25 = DEFAULT_VHF_MINIMUM_FREQ; //Lower limit of working frequency for VHF
  uint16_t trx_max_25 = DEFAULT_VHF_MAXIMUM_FREQ; //Lower limit of working frequency for VHF
  uint16_t scn_min_25 = DEFAULT_VHF_SCAN_LOWER; //Lower scan limit for VHF
  uint16_t scn_max_25 = DEFAULT_VHF_SCAN_UPPER; //Higher scan limit for VHF
  uint16_t aprs_25    = DEFAULT_APRS_VHF_FREQ; //Default APRS Frequency
  uint16_t iss_25     = DEFAULT_ISS_APRS_FREQ; //Default ISS frequency for APRS transmission
  uint16_t vna_min_25 = DEFAULT_VHF_VNA_MINIMUM_FREQ; //Lower limit of VNA frequency for VHF
  uint16_t vna_max_25 = DEFAULT_VHF_VNA_MAXIMUM_FREQ; //Lower limit of VNA frequency for VHF

};
extern freqLimits_t freqLimits;

//EEPROM ADDRESS DEFINITIONS
#define EEPROM_CONFDATA_BLCKSTART 0
#define EEPROM_CHECKIT_ADDR 0
#define EEPROM_VERSMAJ_ADDR 1
#define EEPROM_VERSMIN_ADDR 2
#define EEPROM_CLLSIGN_ADDR 3
#define EEPROM_MESSAGE_ADDR 9
#define EEPROM_RADIOTP_ADDR 17
#define EEPROM_OP_MODE_ADDR 18
#define EEPROM_CURR_MEMCH_ADDR 19

#define EEPROM_APRSMSG_ADDR 20

#define EEPROM_CURRCHNL_BLCKSTART 100
#define EEPROM_CURRFRQ_ADDR 50
#define EEPROM_CURRSHF_ADDR 52
#define EEPROM_CURRTON_ADDR 54

#define EEPROM_APRSDATA_BLCKSTART 60
#define EEPROM_APRSCLL_ADDR 60
#define EEPROM_APRSTIM_ADDR 66
#define EEPROM_APRSLAT_ADDR 67
#define EEPROM_APRSLON_ADDR 75

#define EEPROM_SPECIALFRQ_BLCKSTART 84

#define EEPROM_MEMDATA_BLCKSTART 120
#define EEPROM_CHNNL01_ADDR 120
#define EEPROM_CHNNL_SIZE   12 // Size of memorych_t


#define EEPROM_TOT 50

// WebUI & Cihaz Ek Ayarlari (750..790)
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

extern String wifiPass;
extern bool isSetupDone;
extern uint8_t sql_level;

//VNA variables
extern float minSWR;
extern long lowestFRQ;
extern long highestFRQ;

//Serial port variables
extern String commandString;
extern bool commandComplete;

//APRS Defines
// Defines the Square Wave Output Pin
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

//APRS defaults are moved to Config.h

//Allowed to change
extern String mycall;
extern String APRS_Message;
extern String lat;
extern String lon;
extern unsigned int APRS_Timeout;
extern unsigned long APRS_Counter;
extern String webLogBuffer;

//Not allowed to change
extern char myssid;
extern const char *dest;
extern const char *digi;
extern char digissid;
extern const char sym_ovl;
extern const char sym_tab;

extern char bit_stuff;
extern unsigned short crc;



//8576 LCD Driver settings
#define NEXTCMD 128     // Issue when there will be more commands after this one
#define LASTCMD 0       // Issue when when this is the last command before ending transmission
#define PCF8576_LCD         0x38 //B111000   // This is the address of the PCF on the i2c bus
#define PCF8574_KEYB        0x20 //PCF854 connected to LED and KEYBOARD
#define PCF8574_KEYB_LED    0x21 //PCF8574 connected to KEYBOARD and has interrupt connected to MCU

/* Constants and default settings for the PCF */
// MODE SET
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
extern uint8_t set_modeset;
//BLINK
#define BLINK  112
#define BLINKING_NORMAL 0
#define BLINKING_ALTERNATION 4
#define BLINK_FREQUENCY_OFF 0
#define BLINK_FREQUENCY2 1
#define BLINK_FREQUENCY1 2
#define BLINK_FREQUENCY05 3
extern uint8_t set_blink;
//LOADDATAPOINTER
#define LOADDATAPOINTER  0
extern uint8_t set_datapointer;
//BANK SELECT
#define BANKSELECT 120
#define BANKSELECT_O1_RAM0 0
#define BANKSELECT_O1_RAM2 2
#define BANKSELECT_O2_RAM0 0
#define BANKSELECT_O2_RAM2 1
extern uint8_t set_bankselect;
//#define DEVICE_SELECT 96
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

// Matrix which hold the LCD data (8 segments * 3 bytes per segment)
extern unsigned char matrix[24];
extern unsigned char chr2wr[3];

extern const char* keymap[4];
extern const char numbers[];

/* Text to LCD segment mapping. You can add your own symbols, but make sure the index and font arrays match up */
extern const char font_index[];









void calc_crc(bool in_bit);
void send_flag(unsigned char flag_len);
void send_header(void);
void send_payload(char type);
//void set_io(void);
void getGPSData();
//void StreamPrint_progmem(Print &out,PGM_P format,...);
void InitLCD();
void sendToLcd(uint8_t *data, uint8_t position) ;
void writeToLcd(const char text[]) ;
void writeFRQToLcd(const char frq[9]);
void scroll(const char *text, int speed) ;
void Greetings() ;
void send_SPIBit(int Counter, uint8_t length) ;
void send_SPIEnable() ;
void pllWriteWords(uint32_t f) ;
void pllWriteNA(int N_Counter, int A_Counter) ;
void pllSweepTest() ;
int readColumn();
int readRow();
void SetTone(int toneSTATE) ;
void Alert_Tone(int ToneType);
void SetPLLLock(uint32_t Frequency);
void write_FRQ(uint32_t Frequency) ;
void write_SHIFTtoLCD(uint16_t FRQshift) ;
void write_TONEtoLCD(unsigned long tone_pos) ;
//void SetRFPower(int rfpowerSTATE) ;
void SetRFPower() ;
void setRadioPower() ;
void readRfPower() ;
void numberToFrequency(uint32_t Freq, char *rFRQ) ;
void eeread_nbytes(String *dst, uint8_t sz, uint16_t addr);
void eeprom_readAPRS() ;
void eewrite_nbytes(String dst, uint8_t sz, uint16_t addr) ;
void eeprom_writeAPRS() ;
void initialize_eeprom() ;
void StoreFrequency(char mCHNL[9], char mFRQ[9]) ;
void GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg) ;
void GetMemoryChannel(char mFRQ[9]) ;
void PrintMenu() ;
void commandHelp(char komut) ;
void commandRadioType(char komut) ;
void commandStartupMSG() ;
void commandDumpConfig() ;
void commandMemoryDump() ;
void commandMemoryChannel() ;
void commandAPRSTimeout() ;
void commandAPRSMessage() ;
void commandFrequencyLowerLimit();
void commandFrequencyUpperLimit();
void commandScanLowerLimit();
void commandScanUpperLimit();
void commandAprsFrequency();
void commandISSFrequency();
void getEEPROMData() ;
void commandAPRSmycall() ;
void commandTogglePTT() ;
void StoreSpecialFrequency(char mCHNL[9], char mFRQ[9]) ;
void startScan() ;
void StreamPrint_progmem(Print &out,PGM_P format,...) ;
void serialEvent() ;
void readParam(char *szParam, int iMaxLen) ;
void set_nada_1200(void) ;
void set_nada_2400(void) ;
void set_nada(bool nada) ;
void calc_crc(bool in_bit) ;
void send_crc(void) ;
void send_header(void) ;
void send_payload(char type) ;
void send_char_NRZI(unsigned char in_byte, bool enBitStuff) ;
void send_string_len(String in_string, int len) ;
void send_flag(unsigned char flag_len) ;
void send_packet(char packet_type, uint32_t frequency) ;
void randomize(unsigned int &var, unsigned int low, unsigned int high) ;
void getGPSData() ;
void softResetDevice();
boolean Calculate_Frequency(char mFRQ[9]);
void changeTotSettings(uint8_t timeout, uint8_t lock);






#endif
extern unsigned long prevTotMillis;
