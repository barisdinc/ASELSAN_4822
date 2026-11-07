#include "main.h"

uint8_t Led_Status= 240;
uint8_t KeypadIntPin = 4;
uint8_t KeyVal = 0;
uint8_t old_KeyVal= 0;
bool pttToggler = false;
uint8_t radio_type = 0;
int old_frqSHIFT;
uint8_t TRX_MODE = RX; 
uint8_t LST_MODE = TX; 
uint8_t RF_POWER_STATE = HIGH_POWER; 
uint8_t ALERT_MODE = ALERT_ON;
float ctcss_tone_list[TOTAL_TONES] = {67,69.3,71.9,74.4,77,79.7,82.5,85.4,88.5,91.5,94.8,97.4,100,103.5,107.2,110.9,114.8,118.8,123,127.3};
uint8_t old_ctcss_tone_pos;
uint8_t CHANNEL_BUSY = 1;
int SQL_MODE = SQL_ON;
int scrTimer = 0;  
char pressedKEY = ' ';
uint8_t scrMODE = scrNORMAL;
uint8_t op_mode = 0; // 0=VFO, 1=MR
uint8_t current_memory_channel = 1;
uint8_t subMENU = menuNONE;
uint8_t numChar = 0;
char FRQ[9];
char FRQ_old[9];
boolean validFRQ;
channel_t current_ch;
uint16_t freq_step = 25;
freqLimits_t freqLimits;
float minSWR;
long lowestFRQ;
long highestFRQ;
String commandString = "";         
bool commandComplete = false;  
bool nada = _2400;
const float baud_adj = 0.975;
const float adj_1200 = 1.0 * baud_adj;
const float adj_2400 = 1.0 * baud_adj;
unsigned int tc1200 = (unsigned int)(0.5 * adj_1200 * 1000000.0 / 1200.0);
unsigned int tc2400 = (unsigned int)(0.5 * adj_2400 * 1000000.0 / 2400.0);
String mycall = APRS_DEFAULT_MYCALL;
String APRS_Message = APRS_DEFAULT_MESSAGE;
String lat = APRS_DEFAULT_LAT; 
String lon = APRS_DEFAULT_LON; 
unsigned int APRS_Timeout  = 0;
unsigned long APRS_Counter = 0;
char myssid = 9;
const char *dest = "TA2DMX";
const char *digi = "ARISS";
char digissid = 1;
const char sym_ovl = '>';
const char sym_tab = '/';
char bit_stuff = 0;
unsigned short crc=0xffff;
uint8_t set_modeset = MODESET | MODE_POWERSAVING | DISPLAY_ENABLED | BIAS_THIRD | DRIVE_4;
uint8_t set_blink = BLINK | BLINKING_ALTERNATION | BLINK_FREQUENCY_OFF;
uint8_t set_datapointer = LOADDATAPOINTER | 0;
uint8_t set_bankselect = BANKSELECT | BANKSELECT_O1_RAM0 | BANKSELECT_O2_RAM0; 
uint8_t set_deviceselect = DEVICE_SELECT;      
boolean hasASEL = false;
boolean hasLOCK = false;
boolean hasSPKR = false;
boolean hasTHUN = false;
boolean hasARRW = false;
boolean hasMENU = false;
boolean hasLOOP = false;
boolean hasNOTE = false;
uint8_t Position_Signs[8][3] = { 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0 } ; 
unsigned char matrix[24];
unsigned char chr2wr[3];
const char* keymap[4] = {  "123DSX",  "456RB", "789OC", "*0#UM"  };
const char  numbers[] = "0123456789ABCDEF";
const char font_index[] = "_ /-.*!?<>[]ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789%";
uint8_t TOT_TX_TIME[16] = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225};
uint8_t TOT_TX_LOCK_TIME[16] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 65, 70, 75};
tot_t tot_values;
unsigned long prevTotMillis = 0;
unsigned long lastPllRefresh = 0;

// PLL yazim profili (main.h'de aciklama)
uint8_t  pllMode = 0;
uint16_t pllBitDelayUs = 1000;
bool     pllDoubleWrite = false;
uint16_t pllRefreshMs = 0;
uint8_t  pttActiveLevel = HIGH;   // 't' ile degistirilir
uint16_t pllR = 1024;             // R sayaci: 1024 -> PD 12.5 kHz (TAMSAT_KIT birebir), 512 -> PD 25 kHz
uint8_t  micTxMode = 0;           // 'v' ile degistirilir (0: INPUT, 1: OUTPUT LOW [TAMSAT], 2: OUTPUT HIGH, 3: PWM)

void pttKey(bool on) {
  digitalWrite(PTT_OUTPUT_PIN, on ? pttActiveLevel : (uint8_t)!pttActiveLevel);
}

String webLogBuffer = "";
