#ifndef ASELSAN_4822_H
#define ASELSAN_4822_H

#include "Arduino.h"
#include <EEPROM.h>
#include "../lib/fontsandicons.h"
#include <Wire.h>

#define SERIALMENU 1
#define RESETWATCHDOG
//8576 LCD Driver settings
#define NEXTCMD 128     // Issue when there will be more commands after this one
#define LASTCMD 0       // Issue when when this is the last command before ending transmission

#define SW_MAJOR 3
#define SW_MINOR 0

#define green_led  128
#define yellow_led 64
#define red_led    32
#define backlight  16

uint8_t Led_Status= 240;

uint8_t KeypadIntPin = 4;  //Interrupt Input PIN for MCU, D4 pin (PCINT20)
uint8_t KeyVal = 0;     // variable to store the read value
uint8_t old_KeyVal= 0;

#define FWD_POWER_PIN A6
#define REF_POWER_PIN A7

//Will be used for toogling PTT from serial port
bool pttToggler = false;

//#define POWER_ON_PIN A2
//#define POWER_ON_OFF A3

//#define SYS_OFF 1  //input state low  means radio turned off
//#define SYS_ON  0  //input state high means radio turned on
//int SYS_MODE = SYS_OFF; 

#define PLL_SEC A2
//Band Selection PINS for RX and TX VCOs
//BS0=0, BS1=0   152.2 Mhz - 172.6 Mhz
//BS0=0, BS1=1   147.9 Mhz - 165.5 Mhz
//BS0=1, BS1=0   141.6 Mhz - 172.6 Mhz
//BS0=1, BS1=1   137.2 Mhz - 151.4 Mhz

#define BAND_SELECT_1  11
#define BAND_SELECT_0  8

//RADIO Type
uint8_t radio_type = 0; //0 VHF 1 UHF

//DUPLEX mode Shift Settinngs

#define minusSHIFT -1
#define noSHIFT     0
#define  SIMPLEX    0 //Just incase that we can use this term for noSHIFT
#define plusSHIFT   1
int old_frqSHIFT;       //to store old shift value before entering submenu

//Receive/Transmit and PTT
#define PTT_OUTPUT_PIN 5
#define PTT_INPUT_PIN  12
//Transceiver modes
#define RX 0
#define TX 1
uint8_t TRX_MODE = RX; //default transceiver mode is receiving
uint8_t LST_MODE = TX; //this will hold the last receive transmit state. Start with TX because we want to write to PLL on startup 

//RF power control definitions
#define RF_POWER_PIN A0
#define HIGH_POWER 0
#define LOW_POWER  1
uint8_t RF_POWER_STATE = HIGH_POWER; //Initial Power Level is Hight Power



//Key sounds and alerts
#define ALERT_PIN 13
#define ALERT_OFF 0
#define ALERT_ON  100
uint8_t ALERT_MODE = ALERT_ON;

//Tone Types
#define NO_tone   0
#define OK_tone   1
#define ERR_tone  2
#define SUCC_tone 3



//Tone Control For CTCSS tones
#define MIC_PIN  3  //D3 is our tone generation PIN (PWM)
#define CTCSS_OFF 0
#define CTCSS_ON  1
//uint8_t ctcss_tone_pos = 8;
#define TOTAL_TONES 20
#define TONE_CORRECTION 0.7 //tone shift correction
//Cem ile digital role tetsi yapiyoruz
//float ctcss_tone_list[18] = { 87.70, 87.75,87.80,87.85 ,87.9 ,87.95,88.0,88.05,88.1,88.15,88.2,88.25,88.3,88.35,88.4,88.45,88.5};
float ctcss_tone_list[TOTAL_TONES] = {67,69.3,71.9,74.4,77,79.7,82.5,85.4,88.5,91.5,94.8,97.4,100,103.5,107.2,110.9,114.8,118.8,123,127.3};
//float ctcss_tone_list[50] = {67,69.3,71.9,74.4,77,79.7,82.5,85.4,88.5,91.5,94.8,97.4,100,103.5,107.2,110.9,114.8,118.8,123,127.3,131.8,136.5,141.3,146.3,151.4,156.7,159.8,162.2,165.5,167.9,171.3,173.8,177.3,179.9,183.5,186.2,189.9,192.8,196.6,199.5,203.5,206.5,210.7,218.1,225.7,229.1,233.6,241.8,250.3,254.1};
uint8_t old_ctcss_tone_pos; //to store old tone selection before getting into submenu


#define SQL_ACTIVE 2 //CHANNEL ACTIVE (SQUELCH) PIN
#define MUTE_PIN_1 6 //PIN for Audio Muting
//#define MUTE_PIN_2 5
uint8_t CHANNEL_BUSY = 1;

//MC145158 Programming
#define pll_clk_pin  9
#define pll_data_pin 10
#define pll_ena_pin  7

#define SQL_OFF 0
#define SQL_ON  1
int SQL_MODE = SQL_ON; //initial value for Squelch state

int scrTimer = 0;  //this timer will hold the Press-and-hold timer count
#define TimeoutValue  100; //ticks for timeout time of keypressed
char pressedKEY = ' ';

#define scrNORMAL 0
#define scrMENU   1
uint8_t scrMODE = scrNORMAL;

#define menuNONE   0
#define menuSQL    1
#define menuTONE   2
#define menuSCAN   3
#define menuRPT    4
#define menuMENU   5
uint8_t subMENU = menuNONE;



uint8_t numChar = 0;
char FRQ[9];// = "145.675 ";
char FRQ_old[9];// = FRQ;
boolean validFRQ; //Is the calculated frequenct valid for our ranges

//Special Frequency Definition
#define DEFAULT_VHF_MINIMUM_FREQ  10720 //134000/12.5
#define DEFAULT_UHF_MINIMUM_FREQ  32000 //400000/12.5
#define DEFAULT_VHF_MAXIMUM_FREQ  13920 //174000/12.5
#define DEFAULT_UHF_MAXIMUM_FREQ  37600 //470000/12.5
#define DEFAULT_APRS_VHF_FREQ     11584 //144800/12.5
#define DEFAULT_ISS_APRS_FREQ     11666 //145825/12.5
#define DEFAULT_VHF_SCAN_LOWER    11520 //144000/12.5
#define DEFAULT_UHF_SCAN_LOWER    34400 //430000/12.5
#define DEFAULT_VHF_SCAN_UPPER    11680 //146000/12.5
#define DEFAULT_UHF_SCAN_UPPER    35200 //440000/12.5
#define DEFAULT_VHF_VNA_MINIMUM_FREQ  11200 //140000/125.
#define DEFAULT_VHF_VNA_MAXIMUM_FREQ  12000 //150000/12.5

//defining structures here
struct channel_t {	
  uint32_t frequency;	
  uint8_t  bozuk; //this memory location is corrupted in my development environment
  int16_t  shift;	
  int8_t   shift_dir    = -1;	  // Minus Shift
  uint8_t  tone_pos     = 0x08; //88.5 by default
  uint8_t  tone_enabled = 0;	  //Tone Off
};	
channel_t current_ch;

struct memorych_t {
  uint16_t frequency125; //frequency divided by 12.5
  uint8_t  shift25;      //shift divided bye 25
  uint8_t  tone_position;
  uint8_t  SSTP;         //ShiftShiftTonePower 
  char     ChannelName[4];
  uint8_t  reserved = 0;
};

struct freqLimits_t {
  uint16_t trx_min_125 = DEFAULT_VHF_MINIMUM_FREQ; //Lower limit of working frequency for VHF
  uint16_t trx_max_125 = DEFAULT_VHF_MAXIMUM_FREQ; //Lower limit of working frequency for VHF
  uint16_t scn_min_125 = DEFAULT_VHF_SCAN_LOWER; //Lower scan limit for VHF
  uint16_t scn_max_125 = DEFAULT_VHF_SCAN_UPPER; //Higher scan limit for VHF
  uint16_t aprs_125    = DEFAULT_APRS_VHF_FREQ; //Default APRS Frequency
  uint16_t iss_125     = DEFAULT_ISS_APRS_FREQ; //Default ISS frequency for APRS transmission
  uint16_t vna_min_125 = DEFAULT_VHF_VNA_MINIMUM_FREQ; //Lower limit of VNA frequency for VHF
  uint16_t vna_max_125 = DEFAULT_VHF_VNA_MAXIMUM_FREQ; //Lower limit of VNA frequency for VHF

};
freqLimits_t freqLimits;

//EEPROM ADDRESS DEFINITIONS
#define EEPROM_CONFDATA_BLCKSTART 0
#define EEPROM_CHECKIT_ADDR 0
#define EEPROM_VERSMAJ_ADDR 1
#define EEPROM_VERSMIN_ADDR 2
#define EEPROM_CLLSIGN_ADDR 3
#define EEPROM_MESSAGE_ADDR 9
#define EEPROM_RADIOTP_ADDR 17

#define EEPROM_APRSMSG_ADDR 20

#define EEPROM_CURRCHNL_BLCKSTART 50
#define EEPROM_CURRFRQ_ADDR 50
#define EEPROM_CURRSHF_ADDR 52
#define EEPROM_CURRTON_ADDR 54

#define EEPROM_APRSDATA_BLCKSTART 60
#define EEPROM_APRSCLL_ADDR 60
#define EEPROM_APRSTIM_ADDR 66
#define EEPROM_APRSLAT_ADDR 67
#define EEPROM_APRSLON_ADDR 75

#define EEPROM_SPECIALFRQ_BLCKSTART 84

#define EEPROM_MEMDATA_BLCKSTART 100
#define EEPROM_CHNNL01_ADDR 100
#define EEPROM_CHNNL_SIZE   10   Size of memorych_t


#define EEPROM_TOT 59

//VNA variables
float minSWR;
long lowestFRQ;
long highestFRQ;

//Serial port variables
String commandString = "";         // a String to hold incoming commands
bool commandComplete = false;  // whether the command is complete

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

bool nada = _2400;
const float baud_adj = 0.975;
const float adj_1200 = 1.0 * baud_adj;
const float adj_2400 = 1.0 * baud_adj;
unsigned int tc1200 = (unsigned int)(0.5 * adj_1200 * 1000000.0 / 1200.0);
unsigned int tc2400 = (unsigned int)(0.5 * adj_2400 * 1000000.0 / 2400.0);

//defines
#define APRS_DEFAULT_MESSAGE  "TAMSAT KIT - APRS TEST      "
#define APRS_DEFAULT_MYCALL   "TAMSAT" 
#define APRS_DEFAULT_LAT      "3955.50N" //Latitude  - Anitkabir
#define APRS_DEFAULT_LON      "3250.22E" //longitude - Anitkabir

//Allowed to change
String mycall = APRS_DEFAULT_MYCALL;
String APRS_Message = APRS_DEFAULT_MESSAGE;
String lat = APRS_DEFAULT_LAT; 
String lon = APRS_DEFAULT_LON; 
unsigned int APRS_Timeout  = 0;
unsigned long APRS_Counter = 0;

//Not allowed to change
char myssid = 9;
const char *dest = "TAMSAT";
const char *digi = "ARISS";
char digissid = 1;
const char sym_ovl = '>';
const char sym_tab = '/';

char bit_stuff = 0;
unsigned short crc=0xffff;



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
uint8_t set_modeset = MODESET | MODE_POWERSAVING | DISPLAY_ENABLED | BIAS_THIRD | DRIVE_4; // default init mode
//BLINK
#define BLINK  112
#define BLINKING_NORMAL 0
#define BLINKING_ALTERNATION 4
#define BLINK_FREQUENCY_OFF 0
#define BLINK_FREQUENCY2 1
#define BLINK_FREQUENCY1 2
#define BLINK_FREQUENCY05 3
uint8_t set_blink = BLINK | BLINKING_ALTERNATION | BLINK_FREQUENCY_OFF; // Whatever you do, do not blink. 
//LOADDATAPOINTER
#define LOADDATAPOINTER  0
uint8_t set_datapointer = LOADDATAPOINTER | 0;
//BANK SELECT
#define BANKSELECT 120
#define BANKSELECT_O1_RAM0 0
#define BANKSELECT_O1_RAM2 2
#define BANKSELECT_O2_RAM0 0
#define BANKSELECT_O2_RAM2 1
uint8_t set_bankselect = BANKSELECT | BANKSELECT_O1_RAM0 | BANKSELECT_O2_RAM0; 
//#define DEVICE_SELECT 96
#define DEVICE_SELECT B01100100
uint8_t set_deviceselect = DEVICE_SELECT;      


boolean hasASEL = false; //ASELSAN sign
boolean hasLOCK = false; //KEY LOCK sign
boolean hasSPKR = false; //SPEAKER sign
boolean hasTHUN = false; //TUNHDER sign
boolean hasARRW = false; //ARROW sign
boolean hasMENU = false; //MENU sign
boolean hasLOOP = false; //LOOP sign
boolean hasNOTE = false; //NOTE sign

uint8_t Position_Signs[8][3] = { 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0 } ; //The signs that appear while printing at position 0 to 7 

// Matrix which hold the LCD data (8 segments * 3 bytes per segment)
unsigned char matrix[24];
unsigned char chr2wr[3];

const char* keymap[4] = {  "123DSX",  "456RB", "789OC", "*0#UM"  };
const char  numbers[] = "0123456789ABCDEF";

/* Text to LCD segment mapping. You can add your own symbols, but make sure the index and font arrays match up */
const char index[] = "_ /-.*!?<>[]ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789%";









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






//LCD FUNCTIONS


// Initialize the LCD
void InitLCD() {

  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_modeset); 
  Wire.write(NEXTCMD | set_deviceselect); 
  Wire.write(NEXTCMD | set_blink); 
  Wire.write(LASTCMD | set_datapointer); 
  for (int i=0;i<20;i++)   Wire.write(B00000000);  
  Wire.endTransmission();  
  delay(100);
  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_modeset); 
  Wire.write(NEXTCMD | set_deviceselect); 
  Wire.write(NEXTCMD | set_blink); 
  Wire.write(LASTCMD | set_datapointer); 
  for (int i=0;i<20;i++)   Wire.write(B11111111);  
  Wire.endTransmission();  
  delay(500);
  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_modeset); 
  Wire.write(NEXTCMD | set_deviceselect); 
  Wire.write(NEXTCMD | set_blink); 
  Wire.write(LASTCMD | set_datapointer); 
  for (int i=0;i<20;i++)   Wire.write(B00000000);  
  Wire.endTransmission();  
  delay(100);
}

/* Physically send out the given data */
void sendToLcd(uint8_t *data, uint8_t position) {
  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_deviceselect);     // I think this is needed, as internally the data can overflow and the PCF will automatically select the next device.
  Wire.write(LASTCMD | position * 5 | 0);  // This will always set everything at once, starting from the beginning
  Wire.write(data, 3);                     // Store all 20 bytes
  Wire.endTransmission();    
}

void writeToLcd(const char text[]) {
  memset(chr2wr, 0, 3);
  for (uint8_t idx=0; idx!=strlen(text); idx++) {
    if (idx > 7) break;   
    char *c = strchr(index, (int)toupper(text[idx]));
    uint8_t pos;
    if (c == NULL) { 
      pos = 0;      // Char not found, use underscore space instead
    } else {
      pos = c - index;
    }
    matrix[3*idx+0] = font[(pos * 3)+0] | Position_Signs[idx][0]; //first BYTE
    matrix[3*idx+1] = font[(pos * 3)+1] | Position_Signs[idx][1]; //second BYTE
    if (idx>0) {                         //third BYTE should include previous character if this is not the first
      matrix[3*idx+2] = font[(pos * 3)+2] | (matrix[3*(idx+1)] & B00001111) | Position_Signs[idx][2];  // four bits should be from the existing character
    } else {
      matrix[3*idx+2] = font[(pos * 3)+2] | Position_Signs[idx][2]; 
    }
    
    chr2wr[0] = matrix[3*idx+0];
    chr2wr[1] = matrix[3*idx+1];
    chr2wr[2] = matrix[3*idx+2];
    sendToLcd(chr2wr,idx);    //Send data to LCD for appropriate position 
  }
//  sendToLcd(matrix);
}


void writeFRQToLcd(const char frq[9])
{
  //Prepare the display environment for special signs
  Position_Signs[0][0] = 0;
  Position_Signs[0][1] = 0;
  Position_Signs[0][2] = 0;
  Position_Signs[1][0] = 0;
  Position_Signs[1][1] = 0;
  Position_Signs[7][0] = 0;
  Position_Signs[7][1] = 0;
  Position_Signs[7][2] = 0;

  //First lets check special conditions/states and turn on special characters on the display
  if (SQL_MODE == SQL_OFF) hasSPKR = true; else hasSPKR = false;
  //if (SQL_MODE == SQL_OFF) hasLOOP = true; else hasLOOP = false;
  //if (SQL_MODE == SQL_OFF) hasLOCK = true; else hasLOCK = false;
   
  //if (SQL_MODE == SQL_OFF) hasASEL = true; else hasASEL = false;
  //if (SQL_MODE == SQL_OFF) hasARRW = true; else hasARRW = false;
  if (APRS_Timeout > 0) hasASEL = true; else hasASEL = false;
   
  //if (SQL_MODE == SQL_OFF) hasMENU = true; else hasMENU = false;
  if (RF_POWER_STATE == HIGH_POWER) hasTHUN = true; else hasTHUN = false;
  if (current_ch.tone_enabled == CTCSS_ON) hasNOTE = true; else hasNOTE = false;
  
  if (hasSPKR) {
    Position_Signs[0][0] = Position_Signs[0][0] | SPKR[0];
    Position_Signs[0][1] = Position_Signs[0][1] | SPKR[1];
    Position_Signs[0][2] = Position_Signs[0][2] | SPKR[2];
  }
  if (hasLOOP) {
    Position_Signs[0][0] = Position_Signs[0][0] | LOOP[0];
    Position_Signs[0][1] = Position_Signs[0][1] | LOOP[1];
    Position_Signs[0][2] = Position_Signs[0][2] | LOOP[2];      
  }
  if (hasLOCK) {
    Position_Signs[0][0] = Position_Signs[0][0] | LOCK[0];
    Position_Signs[0][1] = Position_Signs[0][1] | LOCK[1];
    Position_Signs[0][2] = Position_Signs[0][2] | LOCK[2];      
  }
  if (hasARRW) {
    Position_Signs[1][0] = Position_Signs[1][0] | ARRW[0];
    Position_Signs[1][1] = Position_Signs[1][1] | ARRW[1];
    Position_Signs[1][2] = Position_Signs[1][2] | ARRW[2];      
  }    
  if (hasASEL) {
    Position_Signs[1][0] = Position_Signs[1][0] | ASEL[0];
    Position_Signs[1][1] = Position_Signs[1][1] | ASEL[1];
    Position_Signs[1][2] = Position_Signs[1][2] | ASEL[2];      
  }
  if (hasMENU) {
    Position_Signs[7][0] = Position_Signs[7][0] | MENU[0];
    Position_Signs[7][1] = Position_Signs[7][1] | MENU[1];
    Position_Signs[7][2] = Position_Signs[7][2] | MENU[2];      
  }
  if (hasTHUN) {
    Position_Signs[7][0] = Position_Signs[7][0] | THUN[0];
    Position_Signs[7][1] = Position_Signs[7][1] | THUN[1];
    Position_Signs[7][2] = Position_Signs[7][2] | THUN[2];      
  }
  if (hasNOTE) {
    Position_Signs[7][0] = Position_Signs[7][0] | NOTE[0];
    Position_Signs[7][1] = Position_Signs[7][1] | NOTE[1];
    Position_Signs[7][2] = Position_Signs[7][2] | NOTE[2];      
  }
  if (current_ch.shift_dir == noSHIFT) {
    Position_Signs[7][0] = Position_Signs[7][0] | SPLX[0];
    Position_Signs[7][1] = Position_Signs[7][1] | SPLX[1];
    Position_Signs[7][2] = Position_Signs[7][2] | SPLX[2];            
  }
  if (current_ch.shift_dir == minusSHIFT) {
    Position_Signs[7][0] = Position_Signs[7][0] | MINS[0];
    Position_Signs[7][1] = Position_Signs[7][1] | MINS[1];
    Position_Signs[7][2] = Position_Signs[7][2] | MINS[2];            
  }
  if (current_ch.shift_dir == plusSHIFT) {
    Position_Signs[7][0] = Position_Signs[7][0] | PLUS[0];
    Position_Signs[7][1] = Position_Signs[7][1] | PLUS[1];
    Position_Signs[7][2] = Position_Signs[7][2] | PLUS[2];            
  }
 //send the FREQUENCY to display as usual
 writeToLcd(frq); 
}
/* Scrolls a text over the LCD */
void scroll(const char *text, int speed) {
  for (uint8_t i=0; i<=strlen(text)-5; i++) {
     writeToLcd(text+i);
     delay(speed);
  }
}


//Print Greeting messages to LCD
void Greetings() {

 char MSG[8];
 MSG[0] = EEPROM.read(9);   //T
 MSG[1] = EEPROM.read(10);  //A
 MSG[2] = EEPROM.read(11);  //M
 MSG[3] = EEPROM.read(12);  //S
 MSG[4] = EEPROM.read(13);  //A
 MSG[5] = EEPROM.read(14);  //T
 MSG[6] = 48+EEPROM.read(1);  //SW_MAJOR
 MSG[7] = 48+EEPROM.read(2);  //SW_MINOR
 MSG[8] = 0;
 writeToLcd(MSG);
 delay(1000);
 MSG[0] = EEPROM.read(3);
 MSG[1] = EEPROM.read(4);
 MSG[2] = EEPROM.read(5);
 MSG[3] = EEPROM.read(6);
 MSG[4] = EEPROM.read(7);
 MSG[5] = EEPROM.read(8);
 MSG[6] = ' ';
 MSG[7] = ' ';
 MSG[8] = 0;
 writeToLcd(MSG);
 delay(1000);

}



//HELPER FUNCTIONS
//MC145158 programming routines
//R_counter=512
//N_counter=100
//A_counter=0

void send_SPIBit(int Counter, uint8_t length) {
  for (int i=length-1;i>=0;i--) {
    uint8_t data=bitRead(Counter,i);
    if (data==1) {
     digitalWrite(pll_data_pin, HIGH);    // Load 1 on DATA
    } else {
     digitalWrite(pll_data_pin, LOW);    // Load 1 on DATA
    }
    delay(1);
    digitalWrite(pll_clk_pin,HIGH);    // Bring pin CLOCK high
    delay(1);
    digitalWrite(pll_clk_pin,LOW);    // Then back low
    
    /* For backlight Flickering Issue #44 */
    Wire.beginTransmission(0x21);
    Wire.write(224);
    Wire.endTransmission();
  }
}

void send_SPIEnable() {
  digitalWrite(pll_ena_pin, HIGH);   // Bring ENABLE high
  delay(1);
  digitalWrite(pll_ena_pin, LOW);    // Then back low  
}

void SetTone(int toneSTATE) {
  noTone(ALERT_PIN);
  if (toneSTATE == CTCSS_ON) { 
    if (TRX_MODE == TX)  tone(MIC_PIN, ctcss_tone_list[current_ch.tone_pos]);
      else noTone(MIC_PIN);
  }
}







#endif
