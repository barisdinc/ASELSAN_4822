#include <Arduino.h> //For platformio compability
#include <Wire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdio.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
//#if defined(AVR)
//#include <avr/pgmspace.h>
//#else  //defined(AVR)
//#include <pgmspace.h>
//#endif  //defined(AVR)
#include "../lib/fontsandicons.h"
//#include "./libraries/PinChangeInt.h"
//TODO: add PC routines
//TODO: add antenna test step size and upper lower freq limits
//TODO: fix power on/off issues
//TODO: add internal (onboard) eeprom usage
//TODO: fix memory structure for channels
//TODO: add scan function
//TODO: add dual watch
//TODO: try to add squelch level control
//TODO: add end beep
//TODO: Add SWR alarm
//TODO: LCD light control 
#define Serialprint(format, ...) StreamPrint_progmem(Serial,PSTR(format),##__VA_ARGS__)
#define Streamprint(stream,format, ...) StreamPrint_progmem(stream,PSTR(format),##__VA_ARGS__)
#define SERIALMENU 1
//8576 LCD Driver settings
#define NEXTCMD 128     // Issue when there will be more commands after this one
#define LASTCMD 0       // Issue when when this is the last command before ending transmission

#define SW_MAJOR 2
#define SW_MINOR 8

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
byte set_modeset = MODESET | MODE_POWERSAVING | DISPLAY_ENABLED | BIAS_THIRD | DRIVE_4; // default init mode
//BLINK
#define BLINK  112
#define BLINKING_NORMAL 0
#define BLINKING_ALTERNATION 4
#define BLINK_FREQUENCY_OFF 0
#define BLINK_FREQUENCY2 1
#define BLINK_FREQUENCY1 2
#define BLINK_FREQUENCY05 3
byte set_blink = BLINK | BLINKING_ALTERNATION | BLINK_FREQUENCY_OFF; // Whatever you do, do not blink. 
//LOADDATAPOINTER
#define LOADDATAPOINTER  0
byte set_datapointer = LOADDATAPOINTER | 0;
//BANK SELECT
#define BANKSELECT 120
#define BANKSELECT_O1_RAM0 0
#define BANKSELECT_O1_RAM2 2
#define BANKSELECT_O2_RAM0 0
#define BANKSELECT_O2_RAM2 1
byte set_bankselect = BANKSELECT | BANKSELECT_O1_RAM0 | BANKSELECT_O2_RAM0; 
//#define DEVICE_SELECT 96
#define DEVICE_SELECT B01100100
byte set_deviceselect = DEVICE_SELECT;      

#define PCF8576_LCD         0x38 //B111000   // This is the address of the PCF on the i2c bus
#define PCF8574_KEYB        0x20 //PCF854 connected to LED and KEYBOARD
#define PCF8574_KEYB_LED    0x21 //PCF8574 connected to KEYBOARD and has interrupt connected to MCU

#define green_led  128
#define yellow_led 64
#define red_led    32
#define backlight  16

byte Led_Status= 240;

byte KeypadIntPin = 4;  //Interrupt Input PIN for MCU, D4 pin (PCINT20)
byte KeyVal = 0;     // variable to store the read value
byte old_KeyVal= 0;

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
byte radio_type = 0; //0 VHF 1 UHF

//DUPLEX mode Shift Settinngs
int frqSHIFT;

#define minusSHIFT -1
#define noSHIFT     0
#define  SIMPLEX    0 //Just incase that we can use this term for noSHIFT
#define plusSHIFT   1
int shiftMODE = noSHIFT; // we start with noSHIFT (SIMPLEX)
int old_frqSHIFT;       //to store old shift value before entering submenu

//Receive/Transmit and PTT
#define PTT_OUTPUT_PIN 5
#define PTT_INPUT_PIN  12
//Transceiver modes
#define RX 0
#define TX 1
byte TRX_MODE = RX; //default transceiver mode is receiving
byte LST_MODE = TX; //this will hold the last receive transmit state. Start with TX because we want to write to PLL on startup 

//RF power control definitions
#define RF_POWER_PIN A0
#define HIGH_POWER 0
#define LOW_POWER  1
byte RF_POWER_STATE = HIGH_POWER; //Initial Power Level is Hight Power


//Key sounds and alerts
#define ALERT_PIN 13
#define ALERT_OFF 0
#define ALERT_ON  100
byte ALERT_MODE = ALERT_ON;

//Tone Types
#define NO_tone  0
#define OK_tone  1
#define ERR_tone 2



//Tone Control For CTCSS tones
#define MIC_PIN  3  //D3 is our tone generation PIN (PWM)
#define CTCSS_OFF 0
#define CTCSS_ON  1
byte TONE_CTRL = CTCSS_ON; //we start without CTCSS Tone Control
byte ctcss_tone_pos = 8;
#define TOTAL_TONES 20
#define TONE_CORRECTION 0.7 //tone shift correction
//Cem ile digital role tetsi yapiyoruz
//float ctcss_tone_list[18] = { 87.70, 87.75,87.80,87.85 ,87.9 ,87.95,88.0,88.05,88.1,88.15,88.2,88.25,88.3,88.35,88.4,88.45,88.5};
float ctcss_tone_list[TOTAL_TONES] = {67,69.3,71.9,74.4,77,79.7,82.5,85.4,88.5,91.5,94.8,97.4,100,103.5,107.2,110.9,114.8,118.8,123,127.3};
//float ctcss_tone_list[50] = {67,69.3,71.9,74.4,77,79.7,82.5,85.4,88.5,91.5,94.8,97.4,100,103.5,107.2,110.9,114.8,118.8,123,127.3,131.8,136.5,141.3,146.3,151.4,156.7,159.8,162.2,165.5,167.9,171.3,173.8,177.3,179.9,183.5,186.2,189.9,192.8,196.6,199.5,203.5,206.5,210.7,218.1,225.7,229.1,233.6,241.8,250.3,254.1};
byte old_ctcss_tone_pos; //to store old tone selection before getting into submenu


#define SQL_ACTIVE 2 //CHANNEL ACTIVE (SQUELCH) PIN
#define MUTE_PIN_1 6 //PIN for Audio Muting
//#define MUTE_PIN_2 5
byte CHANNEL_BUSY = 1;

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
byte scrMODE = scrNORMAL;

#define menuNONE   0
#define menuSQL    1
#define menuTONE   2
#define menuSCAN   3
#define menuRPT    4
#define menuMENU   5
byte subMENU = menuNONE;

boolean hasASEL = false; //ASELSAN sign
boolean hasLOCK = false; //KEY LOCK sign
boolean hasSPKR = false; //SPEAKER sign
boolean hasTHUN = false; //TUNHDER sign
boolean hasARRW = false; //ARROW sign
boolean hasMENU = false; //MENU sign
boolean hasLOOP = false; //LOOP sign
boolean hasNOTE = false; //NOTE sign

byte Position_Signs[8][3] = { 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0 } ; //The signs that appear while printing at position 0 to 7 

// Matrix which hold the LCD data (8 segments * 3 bytes per segment)
unsigned char matrix[24];
unsigned char chr2wr[3];

const char* keymap[4] = {  "123DSX",  "456RB", "789OC", "*0#UM"  };
const char  numbers[] = "0123456789ABCDEF";

byte numChar = 0;
char FRQ[9];// = "145.675 ";
char FRQ_old[9];// = FRQ;
long calc_frequency;
boolean validFRQ; //Is the calculated frequenct valid for our ranges

/* Text to LCD segment mapping. You can add your own symbols, but make sure the index and font arrays match up */
const char index[] = "_ /-.*!?<>[]ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789%";

//Memory and Channel tyoes
struct channelInfo_t {
  uint16_t FRQ;
  int16_t SHIFT; 
  int8_t TONE; 
  char NAME[4]; 
  uint8_t RESERVED;
};
//channelInfo_t channelInfo;


//VNA variables
float minSWR;
long lowestFRQ;
long highestFRQ;

//Serial port variables
String commandString = "";         // a String to hold incoming commands
bool commandComplete = false;  // whether the command is complete
String CALLSIGN = "TAMSAT";    //INitial greetings callsign

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

//Allowed to change
String mycall = "TA7W";
String APRS_Mesaj = "TAMSAT KIT - APRS TEST";
String lat = "3955.50N"; //Anitkabir Latitude
String lon = "3250.22E"; //Anitkabir longitude
unsigned int APRS_Timeout = 300;
unsigned int APRS_Counter = 0;

//Not allowed to change
char myssid = 9;
const char *dest = "TAMSAT";
const char *digi = "WIDE2";
char digissid = 1;
const char sym_ovl = '>';
const char sym_tab = '/';


char bit_stuff = 0;
unsigned short crc=0xffff;

void set_nada_1200(void);
void set_nada_2400(void);
void set_nada(bool nada);

void send_char_NRZI(unsigned char in_byte, bool enBitStuff);
void send_string_len(String in_string, int len);

void calc_crc(bool in_bit);
void send_crc(void);

void send_packet(char packet_type);
void send_flag(unsigned char flag_len);
void send_header(void);
void send_payload(char type);

//void set_io(void);

void getGPSData();

//Function definitions
//TODO: move these to a header file
void StreamPrint_progmem(Print &out,PGM_P format,...);

//Interrupts       
ISR(PCINT20_vect) // PCINT20 is in PCINT2 vector
{
 KeyVal = digitalRead(KeypadIntPin);
 //digitalWrite(PTT_OUTPUT_PIN, KeyVal); //for testing
}



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

/* Physically send out the given data */
void sendToLcd(byte *data, byte position) {
  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_deviceselect);     // I think this is needed, as internally the data can overflow and the PCF will automatically select the next device.
  Wire.write(LASTCMD | position * 5 | 0);  // This will always set everything at once, starting from the beginning
  Wire.write(data, 3);                     // Store all 20 bytes
  Wire.endTransmission();    
}

void writeToLcd(const char text[]) {
  memset(chr2wr, 0, 3);
  for (int idx=0; idx!=strlen(text); idx++) {
    if (idx > 7) break;   
    char *c = strchr(index, (int)toupper(text[idx]));
    byte pos;
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
   
  //if (SQL_MODE == SQL_OFF) hasMENU = true; else hasMENU = false;
  if (RF_POWER_STATE == HIGH_POWER) hasTHUN = true; else hasTHUN = false;
  if (TONE_CTRL == CTCSS_ON) hasNOTE = true; else hasNOTE = false;
  
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
//  if (TONE_CTRL == CTCSS_ON) {
  if (hasNOTE) {
    Position_Signs[7][0] = Position_Signs[7][0] | NOTE[0];
    Position_Signs[7][1] = Position_Signs[7][1] | NOTE[1];
    Position_Signs[7][2] = Position_Signs[7][2] | NOTE[2];      
  }
  if (shiftMODE == noSHIFT) {
    Position_Signs[7][0] = Position_Signs[7][0] | SPLX[0];
    Position_Signs[7][1] = Position_Signs[7][1] | SPLX[1];
    Position_Signs[7][2] = Position_Signs[7][2] | SPLX[2];            
  }
  if (shiftMODE == minusSHIFT) {
    Position_Signs[7][0] = Position_Signs[7][0] | MINS[0];
    Position_Signs[7][1] = Position_Signs[7][1] | MINS[1];
    Position_Signs[7][2] = Position_Signs[7][2] | MINS[2];            
  }
  if (shiftMODE == plusSHIFT) {
    Position_Signs[7][0] = Position_Signs[7][0] | PLUS[0];
    Position_Signs[7][1] = Position_Signs[7][1] | PLUS[1];
    Position_Signs[7][2] = Position_Signs[7][2] | PLUS[2];            
  }
 //send the FREQUENCY to display as usual
 writeToLcd(frq); 
}
/* Scrolls a text over the LCD */
void scroll(const char *text, int speed) {
  for (int i=0; i<=strlen(text)-5; i++) {
     writeToLcd(text+i);
     delay(speed);
  }
}
//MC145158 programming routines
//R_counter=512
//N_counter=100
//A_counter=0

void send_SPIBit(int Counter, byte length) {
  for (int i=length-1;i>=0;i--) {
    byte data=bitRead(Counter,i);
    if (data==1) {
     digitalWrite(pll_data_pin, HIGH);    // Load 1 on DATA
    } else {
     digitalWrite(pll_data_pin, LOW);    // Load 1 on DATA
    }
    delay(1);
    digitalWrite(pll_clk_pin,HIGH);    // Bring pin CLOCK high
    delay(1);
    digitalWrite(pll_clk_pin,LOW);    // Then back low
  }
}

void send_SPIEnable() {
  digitalWrite(pll_ena_pin, HIGH);   // Bring ENABLE high
  delay(1);
  digitalWrite(pll_ena_pin, LOW);    // Then back low  
}

/*
char numberToArray (int Number) //max 4 digits
{
//TODO: 
//  char result = "         ";
//  result[0] = String(Number / 1000);
//  result[1] = String(Number / 100);
//  result[2] = String(Number / 10);
//  result[3] = String(Number / 1);
  
  
}
*/
boolean Calculate_Frequency (char mFRQ[9]) {

  calc_frequency = ((mFRQ[0]-48) * 100000L) + ((mFRQ[1]-48) * 10000L)  + ((mFRQ[2]-48) * 1000) + ((mFRQ[4]-48) * 100) + ((mFRQ[5]-48) * 10) + (mFRQ[6]-48);
  if (radio_type==0 && (calc_frequency >= 134000L) & (calc_frequency <= 174000L)) return true; //valid frequency for VHF
  else if (radio_type==1 && (calc_frequency >= 400000L) & (calc_frequency < 468400L)) return true; //valid frequency for UHF
  else 
    {
      Alert_Tone(ERR_tone);
      return false; //invalid frequency
    }
}

void write_FRQ(unsigned long Frequency) {
//VHF BAND SELECTION  
//BS0 BS1
// 0  0     152.2   172.6
// 0  1     147.9   165.5
// 1  0     141.6   157.1
// 1  1     137.2   151.4
//+------------+---------------+-----+-----+
//| Radio Type |  Frequency    | BS0 | BS1 |
//+------------+---------------+-----+-----+
//| VHF        | 152.2   172.6 |   0 |   0 |
//|            | 147.9   165.5 |   0 |   1 |
//|            | 141.6   157.1 |   1 |   0 |
//|            | 137.2   151.4 |   1 |   1 |
//+------------+---------------+-----+-----+

//+------------+---------------+-----+-----+
//| Radio Type |  Frequency    | BS0 | BS1 |
//+------------+---------------+-----+-----+
//| UHF        | 406.0   418.0 |   1 |   1 |
//|            | 418.0   430.0 |   0 |   1 |
//|            | 440.0   455.0 |   1 |   0 |
//|            | 455.0   470.0 |   0 |   0 |
//+------------+---------------+-----+-----+

double UpdatedFrq = 0;
//float divider = 0;
  if (validFRQ) {
    if(radio_type==0)
    {
    if ((Frequency < 174000L) & (Frequency >= (164000L)))  { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, LOW);  }
    if ((Frequency < 164000L) & (Frequency >= (154000L)))  { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, HIGH); } 
    if ((Frequency < 154000L) & (Frequency >= (144000L)))  { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, LOW);  } 
    if ((Frequency < 146000L) & (Frequency >= (134000L)))  { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, HIGH); } 
    // Update EEPROM for last used Frequncy
    UpdatedFrq = Frequency - 130000; // Subtrack 130000 to fit the frequency into double size (2 bytes) 

    }
    else if(radio_type==1)
    {
       if ((Frequency < 470000L) & (Frequency >= (452000L)))  
          {
            //Serialprint("UHF 4: [468.400-452.000]"); 
            digitalWrite(BAND_SELECT_0, LOW); 
            digitalWrite(BAND_SELECT_1, LOW); 
          }
       if ((Frequency < 452000L) & (Frequency >= (430000L)))  
          {
            //Serialprint("UHF 3: [430.000-451.99]"); 
            digitalWrite(BAND_SELECT_0, HIGH); 
            digitalWrite(BAND_SELECT_1, HIGH); 
          }
    // Update EEPROM for last used Frequncy
    UpdatedFrq = Frequency - 400000; // Subtrack 130000 to fit the frequency into double size (2 bytes) 
    }
    UpdatedFrq = UpdatedFrq / 12.5;
    byte FRQ_L = UpdatedFrq / 256;
    byte FRQ_H = UpdatedFrq - ( FRQ_L * 256);
    EEPROM.write(50,FRQ_L); 
    EEPROM.write(51,FRQ_H);  
    write_SHIFTtoEE(frqSHIFT);
    write_TONEtoEE(ctcss_tone_pos);
//    EEPROM.write(52,0x00); // SHFT_L
//    EEPROM.write(53,0x00); // SHFT_H
//    EEPROM.write(54,0x00); // TONE
    
    SetPLLLock(Frequency);
    
  
    
  } //validFRQ 
}

void SetPLLLock(unsigned long Frequency)
{
  int R_Counter =0;
  int N_Counter = 0;
  int A_Counter = 0;
  if(radio_type==0)
  {
    if (TRX_MODE == RX) Frequency = Frequency + 45000L;
    if (TRX_MODE == TX) Frequency = Frequency + (shiftMODE * frqSHIFT) ; // Add/remove transmission shift
  
     R_Counter = 12800 / 12.5;  //12.8Mhz reference clock, 25Khz step
     N_Counter = Frequency / 12.5 / 80 ; //prescaler = 80, channel steps 25Khz
     A_Counter = (Frequency / 12.5) - (80 * N_Counter);


    digitalWrite(PLL_SEC, LOW); //SELECT PLL for SPI BUS  
    send_SPIBit(R_Counter,14);
    send_SPIBit(1,1); // Tell PLL that it was the R Counter
    send_SPIEnable();
    send_SPIBit(N_Counter,10);
    send_SPIBit(A_Counter,7);
    send_SPIBit(0,1); // Tell PLL that it was the A and N Counters  
    send_SPIEnable();  
    digitalWrite(PLL_SEC, HIGH); //DE-SELECT PLL for SPI BUS
  }
  else
  {
    if (TRX_MODE == RX) Frequency = Frequency - 45000L;
    if (TRX_MODE == TX) Frequency = Frequency + (shiftMODE * frqSHIFT) ; // Add/remove transmission shift
  
     R_Counter = 12800 / 12.5;  //12.8Mhz reference clock, 25Khz step
     N_Counter = Frequency / 12.5 / 80 ; //prescaler = 80, channel steps 25Khz
     A_Counter =(Frequency / 12.5) - (80 * N_Counter);

  
    digitalWrite(PLL_SEC, LOW);
    send_SPIBit(R_Counter,14);
    send_SPIBit(1,1); 
    send_SPIEnable();
    send_SPIBit(N_Counter,10);
    send_SPIBit(A_Counter,7);
    send_SPIBit(0,1); 
    send_SPIEnable();  
    digitalWrite(PLL_SEC, HIGH); 
  }
}



void write_SHIFTtoEE(unsigned long FRQshift) {
   byte FRQshift_L; 
   byte FRQshift_H;
   if (shiftMODE==noSHIFT)    {FRQshift_L=0; FRQshift_H=0;}
   else
     {
      FRQshift_L=FRQshift / 256; FRQshift_H=FRQshift - (FRQshift_L * 256); //pozitive shift
      if (shiftMODE==minusSHIFT) {FRQshift_L +=128;} //first bit is sign bit (negative shift)
     }
   EEPROM.write(52,FRQshift_L); // SHFT_L
   EEPROM.write(53,FRQshift_H); // SHFT_H
}

void write_SHIFTtoLCD(unsigned long FRQshift) {
 if (FRQshift>=9975) FRQshift=9975; //upper limit check
 if (FRQshift<=0)    FRQshift=0;    //lower limit check
 
 char MSG[8];
 dtostrf(FRQshift, 7, 0, MSG);
 MSG[8] = 0;
 writeToLcd(MSG);
    
}

void write_TONEtoEE(int tone_pos) {
  if (TONE_CTRL==CTCSS_ON) EEPROM.write(54,tone_pos+128); else EEPROM.write(54,tone_pos); // first bit is TONE STATE
}

void write_TONEtoLCD(unsigned long tone_pos) {
 char MSG[8];
 dtostrf(ctcss_tone_list[tone_pos], 7, 1, MSG);
 MSG[8] = 0;
 writeToLcd(MSG);
}


void SetTone(int toneSTATE) {
  noTone(ALERT_PIN);
  if (toneSTATE == CTCSS_ON) { 
    if (TRX_MODE == TX)  tone(MIC_PIN, ctcss_tone_list[ctcss_tone_pos]);
      else noTone(MIC_PIN);
  }
  write_TONEtoEE(ctcss_tone_pos);
}


void Alert_Tone(int ToneType)
{
  if (TRX_MODE == TX)  return; //If we are transmitting, do not play tones, because tone pin might be busy with CTCSS generation
  noTone(MIC_PIN); //First silence the TONE output first
  if (ToneType == OK_tone)  tone(ALERT_PIN,1000,ALERT_MODE);   //short 1Khz is OK  tone
  if (ToneType == ERR_tone) tone(ALERT_PIN,400 ,ALERT_MODE*2); //long 440hz is ERR tone
  delay(ALERT_MODE); //TODO: find a better way to plat two tones simultaneously
  SetTone(TONE_CTRL); //resume Tone Generation 
}

//void SetRFPower(int rfpowerSTATE) {
void SetRFPower() {
    digitalWrite(RF_POWER_PIN, RF_POWER_STATE);
}


void setRadioPower() {
  //Is the radio turned on ?
  //SYS_MODE = digitalRead(POWER_ON_OFF);
  //if ( SYS_MODE == SYS_ON)  digitalWrite(POWER_ON_PIN, HIGH) ;
  //if ( SYS_MODE == SYS_OFF) digitalWrite(POWER_ON_PIN, LOW) ; 

  //if ( SYS_MODE == SYS_ON)  Serial.println("POWER ON")  ; 
  //if ( SYS_MODE == SYS_OFF) Serial.println("POWER OFF") ; 
}


void readRfPower()
{
   int refPower = 0;
   int fwdPower = 0;
   
   fwdPower = analogRead(FWD_POWER_PIN);
   refPower = analogRead(REF_POWER_PIN);
   refPower = refPower * 2;
   int Ptoplam = fwdPower + refPower;
   int Pfark   = fwdPower - refPower;
   float swr = (float)Ptoplam / (float)Pfark;
   Serial.print("\t");
   Serial.print(fwdPower);
   Serial.print("\t");
   Serial.print(refPower);
   Serial.print("\t");
   Serial.print(swr);
   Serial.println("");
   if (swr < minSWR)
   {
      minSWR=swr;
      lowestFRQ=calc_frequency;
      highestFRQ=calc_frequency;
   } else if (swr == minSWR)
   {
      highestFRQ=calc_frequency;
   }

}


void numberToFrequency(long Freq, char *rFRQ) {
  
  word f1,f2,f3,f4,f5,f6; //sgould be byte, but arduino math fails here.. so changed to double

  f1   = Freq / 100000;
  Freq = Freq - ( f1 * 100000);
  f2   = Freq / 10000;
  Freq = Freq - ( f2 * 10000);
  f3   = Freq / 1000;
  Freq = Freq - ( f3 * 1000);
  f4   = Freq / 100;
  Freq = Freq - ( f4 * 100);
  f5   = Freq / 10;
  Freq = Freq - ( f5 * 10);
  f6   = Freq;

  rFRQ[0] = numbers[f1];
  rFRQ[1] = numbers[f2];
  rFRQ[2] = numbers[f3];
  rFRQ[3] = '.';
  rFRQ[4] = numbers[f4];
  rFRQ[5] = numbers[f5];
  rFRQ[6] = numbers[f6];
  rFRQ[7] = ' ';
  rFRQ[8] = 0;
  
  //strcpy(rFRQ,"145.775 ");
}

void initialize_eeprom() {  //Check gthub documents for eeprom structure...
    //Serialprint("initializing EEPROM...");
    EEPROM.write(0, 127); // make eeprom initialized
    EEPROM.write(1, SW_MAJOR);   //SW Version
    EEPROM.write(2, SW_MINOR);   //
    EEPROM.write(3, CALLSIGN[0]); // Callsign
    EEPROM.write(4, CALLSIGN[1]); // Callsign
    EEPROM.write(5, CALLSIGN[2]); // Callsign
    EEPROM.write(6, CALLSIGN[3]); // Callsign
    EEPROM.write(7, CALLSIGN[4]); // Callsign
    EEPROM.write(8, CALLSIGN[5]); // Callsign
    EEPROM.write(9, 'T'); // Message
    EEPROM.write(10,'A'); // Message
    EEPROM.write(11,'M'); // Message
    EEPROM.write(12,'S'); // Message
    EEPROM.write(13,'A'); // Message
    EEPROM.write(14,'T'); // Message
    EEPROM.write(15,' '); // Message
    EEPROM.write(16,' '); // Message
    EEPROM.write(17,radio_type); // Program device as VHF=0 or UHF=1

    //for (int location=18;location < 300;location++) EEPROM.write(location,0); // Zeroise the rest of the memory

    if (radio_type == 0)
    { 
      EEPROM.write(50,0x04); // FRQ_L
      EEPROM.write(51,0xE0); // FRQ_H (Default frequency 145.600)
    }
    else
    {
      EEPROM.write(50,0x0A); // FRQ_L
      EEPROM.write(51,0xE0); // FRQ_H (Default frequency UHF)
    }
    if (radio_type==0)
    {
      EEPROM.write(52,0x02); // SHFT_L
      EEPROM.write(53,0x58); // SHFT_H
    } 
    else
    {
      EEPROM.write(52,0x1D); // SHFT_L
      EEPROM.write(53,0xB0); // SHFT_H    
    }
    EEPROM.write(54,0x08); // TONE 88.5Hz
/*
    //TODO: Move to a common function for PC program integration
    for (int ch=0;ch<100;ch++)
    {
      Serialprint("%d %d\r\n",ch,100+ch*10);
      EEPROM.write(100+ch*10+0,0x04); //FRQ1 //1240 dec = 0x04D8 //VHF 145500, UHF 415500
      EEPROM.write(100+ch*10+1,0xD8); //FRQ2 //1240 dec = 0x04D8 //VHF 145500, UHF 415500
      EEPROM.write(100+ch*10+3,0x00); //SHFT1 0-NoShift
      EEPROM.write(100+ch*10+4,0x00); //SHFT2 0-NoShift
      EEPROM.write(100+ch*10+5,0x00); //TONE1 0-NoTone
      EEPROM.write(100+ch*10+6,0x16); //"K" = 0x16 from Display Array
      EEPROM.write(100+ch*10+7,0x18); //"N" = 0x18 from Display Array
      EEPROM.write(100+ch*10+8,0x41); //"0"-"9" Channel Number from Display Array
      EEPROM.write(100+ch*10+9,0x42); //"0"-"9" Channel Number from Display Array
    }
*/

    //Serialprint("done..");
}

// Stores frequency data to the desired EEPROM location
void StoreFrequency(char mCHNL[9], char mFRQ[9]) {
    byte ChannelNumber = ((mCHNL[0] - 48) * 10) + (mCHNL[1] - 48);
    byte ChannelLocation = 100 + ChannelNumber * 10;
    Calculate_Frequency(mFRQ); 
    double FrqToStore;
    if (radio_type==0)
      {
        FrqToStore = calc_frequency - 130000;
      } 
      else
      {
        FrqToStore = calc_frequency - 400000;
      }
    FrqToStore = FrqToStore / 12.5;
    
    byte FRQ_L = FrqToStore / 256;
    byte FRQ_H = FrqToStore - ( FRQ_L * 256);
    EEPROM.write(ChannelLocation  ,FRQ_L); 
    EEPROM.write(ChannelLocation+1,FRQ_H);  
    //shiftMODE
    byte FRQshift_L; 
    byte FRQshift_H;
    //Serialprint("SHIFT : %d \n\r",shiftMODE);
    if (shiftMODE==noSHIFT)    
      {
        //Serialprint("noSHIFT \n\r");
        FRQshift_L=0; 
        FRQshift_H=0;
        }
    else
      {
        FRQshift_L=frqSHIFT / 256; FRQshift_H=frqSHIFT - (FRQshift_L * 256); //pozitive shift
        if (shiftMODE==minusSHIFT) 
          {
            FRQshift_L +=128;
            //Serialprint("minusSHIFT \n\r");
          } //first bit is sign bit (negative shift)
      }
    EEPROM.write(ChannelLocation+2,FRQshift_L); // SHFT_L
    EEPROM.write(ChannelLocation+3,FRQshift_H); // SHFT_H

    if (TONE_CTRL==CTCSS_ON) EEPROM.write(ChannelLocation+4,ctcss_tone_pos+128); else EEPROM.write(ChannelLocation+4,0); // first bit is TONE STATE
    // Serialprint("STR Channel Parameters CH NO :%d ChannelLOC:%d FRQ_L:%d FRQ_H:%d FRQShift_L:%d FRQShift_H:%d CTCSS:%d \n\r",ChannelNumber,ChannelLocation,FRQ_L,FRQ_H,FRQshift_L,FRQshift_H,ctcss_tone_pos);
}


/*
 * Retrieves the requested Memory Channel Information from EEPROM and returns a channelInfo_t tyep object
 */
channelInfo_t GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg) {
      channelInfo_t l_channelInfo;
      //byte ChannelNumber = ((mFRQ[0] - 48) * 10) + (mFRQ[1] - 48);
      byte ChannelLocation = 100 + channel_number * 10;
      l_channelInfo.FRQ = (EEPROM.read(ChannelLocation) * 256 + EEPROM.read(ChannelLocation+1));// * 12.5;
      //numberToFrequency(freq, FRQ);
      l_channelInfo.SHIFT = EEPROM.read(ChannelLocation+2) * 256 + EEPROM.read(ChannelLocation+3);
      l_channelInfo.TONE  = EEPROM.read(ChannelLocation+4) ; 
      if (dbg) 
      {
        Serialprint("{c:%d,f:%d,s:%d,t:%d}",channel_number,l_channelInfo.FRQ,l_channelInfo.SHIFT,l_channelInfo.TONE); //TODO: add name
      }
}


//Retrieves the requested Memory Channel Information from EEPROM
//TODO: combine this with the previuous function
void GetMemoryChannel(char mFRQ[9]) {
 byte ChannelNumber = ((mFRQ[0] - 48) * 10) + (mFRQ[1] - 48);
 byte ChannelLocation = 100 + ChannelNumber * 10;
 byte byte1,byte2;
 byte1 = EEPROM.read(ChannelLocation);
 byte2 = EEPROM.read(ChannelLocation+1);
long freq;
 freq = (byte1 * 256) + byte2 ;
 freq = freq * 12.5;
 
if (radio_type==0)
 {
  freq = freq + 130000;
 } else 
 {
  freq = freq + 400000; 
 } 
 
 numberToFrequency(freq, FRQ);
 
 
 byte FRQshift_L = EEPROM.read(ChannelLocation+2);
 byte FRQshift_H = EEPROM.read(ChannelLocation+3);

 shiftMODE=noSHIFT;//default value for SHIFTMODE
 if ((FRQshift_L>0) || (FRQshift_H>0)) shiftMODE=plusSHIFT;
 if ( FRQshift_L>127) {FRQshift_L=FRQshift_L - 128; shiftMODE=minusSHIFT;}
 frqSHIFT = FRQshift_L * 256 + FRQshift_H;


 ctcss_tone_pos  = EEPROM.read(ChannelLocation+4) ; // TONE
 TONE_CTRL=CTCSS_OFF;
 if (ctcss_tone_pos>127) {TONE_CTRL=CTCSS_ON;ctcss_tone_pos-=128;} //TONEbit is on.. 
 //TONE_CTRL=CTCSS_ON;
  // Serialprint("RTR Channel Parameters CH NO :%d ChannelLOC:%d %d %d FRQ_L:%d FRQ_H:%d FRQShift_L:%d FRQShift_H:%d CTCSS:%d SHIFT:%d \n\r",ChannelNumber,ChannelLocation,byte1,byte2,FRQshift_L,FRQshift_H,ctcss_tone_pos,shiftMODE);
}

void PrintMenu()
{  
//  print_version();
  Serialprint("ASELSAN 48xx - TAMSAT Kit\n\r");
  Serialprint("-------------------------\n\r");
  //Serialprint("Y-Yardim\n\r");
  Serialprint("C-VHF/UHF Cevrimi Yap\n\r");
  Serialprint("A-Acilis Mesaji Degistir\n\r");
  Serialprint("F-Frekans Sinirlari\n\r");
  Serialprint("V-Analizor Sinirlari\n\r");
  Serialprint("H-Hafiza Islemleri\n\r");
  Serialprint("K-Konfigurasyon Yaz\n\r");
  Serialprint("S-APRS cagri isareti\n\r");
  Serialprint("T-APRS sessizlik suresi\n\r");
  Serialprint("M-APRS Mesaji\n\r");  
  Serialprint("G-GPS Oku\n\r");  
  Serialprint("Seciminiz >");
}

void commandYardim(char komut)
{
  /*
  Serialprint("Yardim Menusu\r\n=====================\r\n");
  if (komut=='\n')
  {
    Serialprint("Yardim almak istediginiz komutu belirtiniz...\r\n");
    Serialprint("Ornek : Y C  veya Y F  veya Y A \r\n");
  } else if (komut == 'C')
  {
    Serialprint("Bu komut cihazinizi UHF ya da VHF olarak programlamanizi saglar. (sadece TAMSAT Kartini)\r\n");
    Serialprint("Kullanimi : \r\n      C V : VHF yap\r\n      C U : UHF yap");  
  } 
  else if (komut == 'A')
  {
    Serialprint("Kullanimi : \r\n      A [MESAJ]   Ornek:  A TA7W  (maksimum 6 karakter)\r\n"); 
  } else if (komut == 'H')
  {
    Serialprint("Kullanimi:   H [Kanal_No #2] [isim #6] [Frekans #6] [Shift #5] [Ton #4] \r\n"); // !!!CHANGED!!!!!
    Serialprint("Ornek:       H 01 ROLE-0 145600 +0600 0885 \r\n");
  } else if (komut == 'T')
  {
    Serialprint("Kullanimi:   T [3 basamakli olarak Sure (saniye) (000-999)] \r\n");
    Serialprint("Ornek:       T 300\r\n");
  } else if (komut == 'M')
  {
    Serialprint("Kullanimi:   M [MESAJ (maksimum 30 karakter) \r\n");
    Serialprint("Ornek:       M Merhaba, Ben BARIS DINC - OH2UDS  \r\n");
  }
  F [Alt-frekans] [ust-frekans]        Frekans araligi 
  V [Alt-frekans] [ust-frekans]        VNA Frekans araligi
  S [0125-5000]                        VNA Step
  H                                    Memory Dump as Json

*/  
}


void commandCevrim(char komut)
{
  //Serialprint("\r\nCevrim Yapiliyor\r\n=====================\r\n");
  if (komut == 'V')
    {
      //Serial.print("TAMSAT Karti VHF olarak ayarlanmistir. 4822 veya 4922 cihaz ile kullanabilirsiniz\r\n");
      Serialprint("VHF OK\r\n");
      radio_type = 0;
      initialize_eeprom();
    }
  if (komut == 'U')
    {
      //Serial.print("TAMSAT Karti UHF olarak ayarlanmistir. 4826 veya 4926 cihaz ile kullanabilirsiniz\r\n");
      Serialprint("UHF OK\r\n");
      radio_type = 1;
      initialize_eeprom();
    }
}


void commandAcilis()
{
  //Serial.print("ACILIS MESAJI = ");
  Serial.print(commandString.substring(2,8));
  //Serial.println(" olarak duzenlendi...");
  CALLSIGN=commandString.substring(2,8);
  initialize_eeprom();
}


/*
 * Print the configurations in form of JSON array to Serial Port
 */
void commandAyarDok()
{
  byte Check = EEPROM.read(0);
  byte SW_major = EEPROM.read(1);
  byte SW_minor = EEPROM.read(2);
  char CallSign[6];
  CallSign[0] = EEPROM.read(3);
  CallSign[1] = EEPROM.read(4);
  CallSign[2] = EEPROM.read(5);
  CallSign[3] = EEPROM.read(6);
  CallSign[4] = EEPROM.read(7);
  CallSign[5] = EEPROM.read(8);
  CallSign[6] = 0;
  char Message[8];
  Message[0] = EEPROM.read(9);
  Message[1] = EEPROM.read(10);
  Message[2] = EEPROM.read(11);
  Message[3] = EEPROM.read(12);
  Message[4] = EEPROM.read(13);
  Message[5] = EEPROM.read(14);
  Message[6] = EEPROM.read(15);
  Message[7] = EEPROM.read(16);
  Message[8] = 0;
  byte RadioType = EEPROM.read(17);
  Serialprint("\r\nCD{");    //CD Configuration Dump
  Serialprint("cs:%d,smj:%d,smn:%d,c:'%s',m:'%s',r:%d",Check,SW_major,SW_minor,CallSign,Message,RadioType);
  Serialprint("}\r\n");
}


/*
 * Print the memory channels in form of JSON array to Serial Port
 */
void commandHafizaDok()
{
  Serialprint("\r\nMD[");    //Memory Dump
  for (int ch=0;ch<100;ch++)
    {
      channelInfo_t chInfo = GetPrintMemoryChannelInfo(ch,true);
      Serialprint(",");
    }
    Serialprint("{}]\r\n");
}

void commandHafizaKoy()
{
  
    //H [Kanal_No #2] [isim #6] [Frekans #6] [Shift #5] [Ton #4]
    //Ornek:       H 01 ROLE-0 145600 +0600 0885

}

void commandAPRSSure()
{
  //Serial.print("APRS bekleme suresi ");
  Serial.print(commandString.substring(2,5));
  Serialprint(" OK\r\n");
  //Serial.println(" olarak duzenlendi");
  APRS_Timeout = commandString.substring(2,3).toInt();
  if (APRS_Timeout <= 60) APRS_Timeout = 60;  
}

void commandAPRSMesaj()
{
  //Serial.print("APRS mesajiniz '");
  Serial.print(commandString.substring(2,30));
  //Serial.println("' olarak duzenlendi");
  APRS_Mesaj = commandString.substring(2,30);
  Serial.print(APRS_Mesaj);
  Serialprint(" OK\r\n");
}

void commandAPRSmycall()
{
  //Serial.print("APRS cagri isaretiniz '");
  Serial.print(commandString.substring(2,8));
  Serialprint(" OK\r\n");
  //Serial.println("' olarak duzenlendi");
  mycall = commandString.substring(2,8);  
}

void commandTogglePTT()
{
  pttToggler = !pttToggler; 
  if (pttToggler) Serialprint("TX\r\n");
  else Serialprint("RX\r\n");
}


int readColumn()
{
    Wire.beginTransmission(B0100000); //Here we're pushing VCC from one of PCF8574 and reading the other
    Wire.write(0); 
    Wire.endTransmission();
    Wire.beginTransmission(B0100001);
    Wire.write(255); 
    Wire.endTransmission();
    
    Wire.requestFrom(0x21,1);
    int c = Wire.read();
    c = 255 - c;
    int sutun = 0 ;
    if (c == 8) sutun = 0;
    if (c == 4) sutun = 1;
    if (c == 2) sutun = 2;
    if (c == 1) sutun = 3;

    return sutun;
}

int readRow()
{
    Wire.requestFrom(0x20,1);
    int r = Wire.read();    // receive a byte as character
    r = 255 - (r | B00000011) ; 
    r = r >> 3;
    int satir = 5;
    if (r == 16) satir = 0;
    if (r ==  8) satir = 1;
    if (r ==  4) satir = 2;
    if (r ==  2) satir = 3;
    if (r ==  1) satir = 4;
    
    return satir;
}

    








void setup() {
  cli(); // Turn Off Interrupts
  //PCICR  |= 0b00000100;  
  PCICR = (1 << PCIE2);  //same as above but different :)
 // PCMSK1 |= 0b00010000; // PCINT20 - Digital 4 Pin
 // PCMSK2 |= 0b00010000; // PCINT20 - Digital 4 Pin
  PCMSK2 |= (1 << PCINT20); //same as above but different :)
  sei();
  delay(100);

  Serial.begin(9600);
  commandString.reserve(200);

  // Check EEPROM for stored values
  byte eeprom_state=0;
 // EEPROM.write(0,127);
  eeprom_state = EEPROM.read(0);//EEPROM Check For Modification Board
  //Serialprint("ACILIS DEGERI %d\r\n",eeprom_state);
  if (eeprom_state != 127) initialize_eeprom();
  // if (eeprom_state != 127) Serialprint("EEPROM Sifirlaniyor \n\r");
  radio_type = EEPROM.read(17);//UHF VHF Seçimi
  //Serialprint("CIHAZ TIPI %d\r\n",radio_type);
  if (EEPROM.read(1) != SW_MAJOR or EEPROM.read(2) != SW_MINOR) initialize_eeprom();
  //Serialprint("MAJOR %d MINOR %d\r\n",EEPROM.read(1),EEPROM.read(2));
  
  //initialize_eeprom();
  //Read Last used frequency
  byte byte1,byte2;
  byte1 = EEPROM.read(50);
  byte2 = EEPROM.read(51);
  long freq;
  freq = (byte1 * 256) + byte2 ;
  freq = freq * 12.5;

   if (radio_type == 0)
     {
       freq = freq + 130000;
       frqSHIFT = 600;
     }
   else
     {
      freq = freq + 400000;  
      frqSHIFT = 7600;
     }

  numberToFrequency(freq, FRQ);
  strcpy(FRQ_old,FRQ);
 
  byte FRQshift_L = EEPROM.read(52);
  byte FRQshift_H = EEPROM.read(53);

  shiftMODE=noSHIFT;//default value for SHIFTMODE
  if ((FRQshift_L>0) && (FRQshift_H>0)) shiftMODE=plusSHIFT;
  if ( FRQshift_L>127) {FRQshift_L-=128; shiftMODE=minusSHIFT;}
  frqSHIFT = FRQshift_L * 256 + FRQshift_H;

  ctcss_tone_pos  = EEPROM.read(54) ; // TONE
  TONE_CTRL=CTCSS_OFF;
  if (ctcss_tone_pos>127) {TONE_CTRL=CTCSS_ON;ctcss_tone_pos-=128;} //TONEbit is on.. 
  //Serialprint("Startup with shift : %d ShiftMode: %d \n\r",frqSHIFT,shiftMODE);

  //setRadioPower();  //Check power switch mode and turn adio on immediately
  //pinMode(POWER_ON_OFF, INPUT);
  //pinMode(POWER_ON_PIN, OUTPUT);

  pinMode(MIC_PIN, OUTPUT);
  SetTone(TONE_CTRL);

  pinMode(RF_POWER_PIN, OUTPUT); //RF power control is output
  digitalWrite(RF_POWER_PIN, RF_POWER_STATE);
  //TODO: store last power state and restore on every boot 
  
  pinMode(MUTE_PIN_1, OUTPUT);
  digitalWrite(MUTE_PIN_1, HIGH); //Mute the Audio output

  pinMode(SQL_ACTIVE, INPUT);

  pinMode(BAND_SELECT_0, OUTPUT);
  pinMode(BAND_SELECT_1, OUTPUT);

  pinMode(PTT_INPUT_PIN,  INPUT_PULLUP);
  pinMode(PTT_OUTPUT_PIN, OUTPUT);
  digitalWrite(PTT_OUTPUT_PIN, LOW); //No PTT at startup

  pinMode(FWD_POWER_PIN, INPUT);
  pinMode(REF_POWER_PIN, INPUT);


  //pinMode(KeypadIntPin,    INPUT);
	pinMode(KeypadIntPin, INPUT_PULLUP);
 // attachPinChangeInterrupt(KeypadIntPin, KeyPadInterrupt, RISING);						   
  pinMode(pll_clk_pin, OUTPUT);
  pinMode(pll_data_pin,OUTPUT);
  pinMode(pll_ena_pin, OUTPUT);
  pinMode(PLL_SEC, OUTPUT);

  digitalWrite(pll_clk_pin, LOW);
  digitalWrite(pll_data_pin,LOW);
  digitalWrite(pll_ena_pin, LOW);
 
  memset(matrix, 0, 24);
  Wire.begin(); 
  delay(100);  // Give some time to boot

  // Init LCD
  InitLCD();
  Greetings();

  writeFRQToLcd(FRQ);
  
  old_KeyVal = 1; //initial keypad read
  validFRQ = Calculate_Frequency(FRQ); // start with default frequency //TODO: Change this to last frequemcy set

  scrTimer = TimeoutValue;  

  if (pgm_read_byte (32700) != 188)
   while (1) 
    Alert_Tone(ERR_tone);

  PrintMenu();
}

void loop() {
  //setRadioPower(); //Check power switch and set radio power mode on or off
  
  if (scrMODE==scrNORMAL) writeFRQToLcd(FRQ); //TODO: We should update the display only on proper display changes.. But this works...

  //Output data to Keyboard... First first bits for keyboard, next bits for backlight and leds... 
  Wire.beginTransmission(PCF8574_KEYB_LED);
  Led_Status = 240;
  if ((CHANNEL_BUSY==0) or (SQL_MODE==SQL_OFF)) Led_Status = Led_Status - green_led; //we are receivig
  if (CHANNEL_BUSY==0) APRS_Counter = 0; //If channel is busy, dont transmit APRS, wait the musy to finish and restart counter
  if (TRX_MODE == TX) Led_Status = Led_Status - red_led;  //We are transmitting
  if (pttToggler) send_packet(_STATUS);

  //Led_Status = Led_Status - yellow_led;
  //Led_Status = Led_Status - red_led;
  if (APRS_Counter < 2000) Led_Status = Led_Status - backlight;
  Wire.write(Led_Status); 
  Wire.endTransmission();

  CHANNEL_BUSY = digitalRead(SQL_ACTIVE);  
  if (CHANNEL_BUSY == 0) digitalWrite(MUTE_PIN_1, LOW);
    else if (SQL_MODE == SQL_ON) digitalWrite(MUTE_PIN_1, HIGH); 
      else digitalWrite(MUTE_PIN_1, LOW);

  TRX_MODE = digitalRead(PTT_INPUT_PIN); //read PTT state
//  if (pttToggler && TRX_MODE == RX) TRX_MODE = TX; //if pttToggler set from serial, then mode is transmission
  if (TRX_MODE != LST_MODE) {
    LST_MODE = TRX_MODE;
    write_FRQ(calc_frequency); //Update frequenct on every state change
  }
  if (TRX_MODE == TX ) digitalWrite(PTT_OUTPUT_PIN,HIGH); // now start transmitting
    else digitalWrite(PTT_OUTPUT_PIN, LOW);

  SetTone(TONE_CTRL); //Change Tone Generation State

  //if (subMENU == menuRPT) writeToLcd(cstr);

 //check APRS timer timeout, and send APRS message if timeout reached
 //TODO: Convert timeout to seconds instead of loop counter
  APRS_Counter += 1;
  if (APRS_Counter >= APRS_Timeout * 20000) {
     send_packet(_FIXPOS_STATUS);
     APRS_Counter = 0;
  }

  //this is our interrupt pin... Move this to a proper interrupt rutine
  KeyVal = digitalRead(KeypadIntPin);


  //Long press on a key detection...
  if ((KeyVal == old_KeyVal) & (KeyVal ==  0) & (scrTimer>0)) scrTimer -= 1; //Keypressed and there are counts to go
    else if (KeyVal==1) {
     if (scrTimer==0) { //key released and timeout occured
//       writeToLcd("SELECT   ");
//       delay(500); //TODO do not use DELAY, change to a timer
       if (pressedKEY=='B') { writeToLcd("TONE     "); subMENU = menuTONE; delay(1000);write_TONEtoLCD(ctcss_tone_pos); old_ctcss_tone_pos = ctcss_tone_pos;}
       if (pressedKEY=='S') { writeToLcd("SQL      "); subMENU = menuSQL;  }
       if (pressedKEY=='O') { writeToLcd("SCAN     "); subMENU = menuSCAN; }
       if (pressedKEY=='R') { writeToLcd("SHIFT    "); subMENU = menuRPT;  delay(1000);write_SHIFTtoLCD(frqSHIFT); old_frqSHIFT=frqSHIFT;}
       if (pressedKEY=='M') { writeToLcd("MENU     "); subMENU = menuMENU; }
       delay(500); //TODO do not use DELAY, change to a timer
       scrTimer = TimeoutValue; //Restart the timer
       numChar = 0; //if we were entering frq from keyboard
       scrMODE = scrMENU;    
     }//scrTimer
    }//Keyval==0

  
  if (KeyVal != old_KeyVal) { 
    if (KeyVal == 0) Alert_Tone(OK_tone);  

    APRS_Counter = 0; //if a key is pressed, restart the APRS timer

    scrTimer = TimeoutValue; //Restart the timer

    int satir = readRow();
    int sutun = readColumn();
    pressedKEY = keymap[sutun][satir]; //LOOKUP for the pressedKEY
    // Serialprint("set keyboard pressedKey:%d \n\r  ", pressedKEY);
    
  /* -----------------------------------------------------
     SCREEN MODE MENU..  
     ----------------------------------------------------- */

    if (scrMODE == scrMENU)  {
      // Serialprint("srcmode src menu \n\r  ");
      if (pressedKEY != 'X') {
        // Serialprint("src menu not X pressedKey:%d \n\r  ", pressedKEY);
        switch (pressedKEY) {
          case 'U':
            if ( subMENU == menuRPT)  { frqSHIFT += 25; write_SHIFTtoLCD(frqSHIFT);}
            if ( subMENU == menuTONE) { ctcss_tone_pos += 1; if (ctcss_tone_pos>=19) ctcss_tone_pos = 19 ; write_TONEtoLCD(ctcss_tone_pos);}
            break;
          case 'D':
            if ( subMENU == menuRPT)  {frqSHIFT -= 25; write_SHIFTtoLCD(frqSHIFT); }
            if ( subMENU == menuTONE) { ctcss_tone_pos -= 1;  if (ctcss_tone_pos<=0) ctcss_tone_pos = 0 ; write_TONEtoLCD(ctcss_tone_pos);}
            
            break;
          case '#': //means CANCEL
            if (subMENU == menuRPT)  frqSHIFT = old_frqSHIFT;
            if (subMENU == menuTONE) ctcss_tone_pos = old_ctcss_tone_pos;
            //testing the lock reported by TA2GY .. memory recall after shift setup locks device
            numChar = 0;
            // Serialprint("set to X pressedKey:%d \n\r  ", pressedKEY);
            pressedKEY = 'X';
            scrMODE = scrNORMAL; //we are in menu or submenus.. return to normal display
            subMENU = menuNONE;
          break; // '#'
          case '*': //means OK
            if ( subMENU == menuRPT)  write_SHIFTtoEE(frqSHIFT);
            if ( subMENU == menuTONE) write_TONEtoEE(ctcss_tone_pos);
            //testing the lock reported by TA2GY .. memory recall after shift setup locks device
            numChar = 0;
            // Serialprint("set to X * pressedKey:%d \n\r  ", pressedKEY);
            pressedKEY = 'X';
            scrMODE = scrNORMAL; //we are in menu or submenus.. return to normal display
            subMENU = menuNONE;
          break; // '*'
          default:
          break; 
        }//switch
      }//pressedKEY!='X'
    }//scrMENU
    
    
    
    
  /* -----------------------------------------------------
   SCREEN MODE NORMAL.. WE READ FREQUENCY AND OTHER KEYS 
   ----------------------------------------------------- */
    if (scrMODE == scrNORMAL) { //mode NORMAL and key released
      // Serialprint("src normal pressedKey:%d \n\r  ", pressedKEY);
      if (pressedKEY != 'X') {
        // Check for the COMMAND keys first
        switch (pressedKEY) {
          case 'R':
            if (shiftMODE == noSHIFT) { 
              shiftMODE = plusSHIFT;
            } else if (shiftMODE == plusSHIFT) {
              shiftMODE = minusSHIFT;
            } else { 
              shiftMODE = noSHIFT;
            }
            // write_SHIFTtoEE(frqSHIFT);
          break; //'R'
          case 'B':
            if (TONE_CTRL == CTCSS_OFF) {
              TONE_CTRL = CTCSS_ON;
            } else {
              TONE_CTRL = CTCSS_OFF;
            }
            SetTone(TONE_CTRL); //Change Tone Generation State
          break; // 'B'
          case 'S':
            if (SQL_MODE == SQL_OFF) {
              SQL_MODE = SQL_ON;
            } else {
              SQL_MODE = SQL_OFF;
            }
          break; //'S'
          case 'O':
             if (RF_POWER_STATE == HIGH_POWER) {
                RF_POWER_STATE = LOW_POWER;
             } else {
                RF_POWER_STATE = HIGH_POWER;
             }
//             SetRFPower(RF_POWER_STATE);           
             SetRFPower();           
          break; //'O'
          case 'U':
            //numberToFrequency(calc_frequency+1000,FRQ);
            numberToFrequency(calc_frequency+25,FRQ);
            //delay(1000);
            Calculate_Frequency(FRQ);  
          //  delay(1000);
            write_FRQ(calc_frequency); 
           // delay(2000);
          break; // 'U' 
          case 'D':
            numberToFrequency(calc_frequency-25,FRQ);
            Calculate_Frequency(FRQ);  
            write_FRQ(calc_frequency);           
          break; // 'D'
          case 'M':
           //Serialprint(pressedKEY);
          break; // 'M'
          case 'C':
            //VNA Vector Network analizor Subroutines
            Serial.print("pressedKEY");
            Serial.println(pressedKEY,DEC);
            minSWR = 9999;
            lowestFRQ=0;
            highestFRQ=0;           
            //TODO: Store old values of variables (FRQ, SHIFT, etc)
            //TODO: Put transmitter on low power before operation
            //TODO: If any key pressed cancel VNA operation
            strcpy(FRQ_old,FRQ); //store old frequency for recall
            shiftMODE = noSHIFT; //get into SIMPLEX mode for caculations
//            SetRFPower(LOW_POWER);
            SetRFPower();
            long min_vna_freq;
            long max_vna_freq;
            int  stp_vna_freq;
            if (radio_type==0)
            {
              min_vna_freq = 14000;
              max_vna_freq = 15000;
              stp_vna_freq = 10;
            }
            else
            {
              min_vna_freq = 43000;
              max_vna_freq = 45000;              
              stp_vna_freq = 10;
            }
            Serialprint("#VNA#\t%l\t%l\r\n",min_vna_freq,max_vna_freq); //START
            for (long vna_freq=min_vna_freq; vna_freq < max_vna_freq; vna_freq += stp_vna_freq)
              {
                TRX_MODE = TX;
                //Serial.print(vna_freq); 
                numberToFrequency(vna_freq*10,FRQ);
                validFRQ = Calculate_Frequency(FRQ);
                //write_FRQ(calc_frequency);
                writeFRQToLcd(FRQ);
                Serialprint(">%d\t",vna_freq);
                //digitalWrite(PTT_OUTPUT_PIN,HIGH);
                delay(50);
                readRfPower(); //TODO: Move under a menu item
                delay(25);
                //digitalWrite(PTT_OUTPUT_PIN,LOW);
                if (readColumn() != 0) break; //a key is pressed
              }
              Serialprint("@VNA@\t%d\t%d\r\n",min_vna_freq,max_vna_freq); //END
              //Restoring OLD values or displaying the best frequency
              //SetRFPower(RF_POWER_STATE);
              SetRFPower();
              TRX_MODE = RX;
              //strcpy(FRQ,FRQ_old);
              numberToFrequency((highestFRQ+lowestFRQ)/2,FRQ);
              validFRQ = Calculate_Frequency(FRQ);
              write_FRQ(calc_frequency);            
              PrintMenu(); //print menu for user selection  
          break; // 'C'
          case '#':
            if (numChar == 2) StoreFrequency(FRQ,FRQ_old); // User is trying to store the actual frequency : FRQ[0..1] ccontains memory channel and FRQ_old contains the frequency to be stored
              strcpy(FRQ,FRQ_old);
              validFRQ = Calculate_Frequency(FRQ);
              numChar = 0;
              write_FRQ(calc_frequency);
          break; // '#'
          case '*':
            if (numChar == 2) GetMemoryChannel(FRQ); // User wanted to retrieve the memory channel from EEPRM
            else strcpy(FRQ,FRQ_old); // Otherwise user wanted to cancel the ongoing operation.. return to previous (old) frequency
            validFRQ = Calculate_Frequency(FRQ);
            numChar = 0;
            write_FRQ(calc_frequency);
          break; // '*'
         
          default:  
            /* ------------------    FRQ INPUT ------------ */
            // Serialprint("Before If FRQ :%s numChar:%d pressedKey:%d \n\r  ",FRQ,numChar,pressedKEY);
            if (numChar <= 6) { //Not a command Key so print it into frequency
              if (numChar == 0) {
              //  strcpy(FRQ,"        "); //just pressed keys, so clear the screen
              FRQ[0]=' ';
              FRQ[1]=' ';
              FRQ[2]=' ';
              FRQ[3]=' ';
              FRQ[4]=' ';
              FRQ[5]=' ';
              FRQ[6]=' ';
              FRQ[7]=' ';
              }
              
              FRQ[numChar] = pressedKEY;
              numChar = numChar + 1;
              if (numChar == 3) { //place a dot for the 4th character
                FRQ[3] = '.';
                numChar = numChar + 1;
              } //numchar==3
              
              if (numChar == 7) { //input finished 
                validFRQ = Calculate_Frequency(FRQ);
                numChar = 0;
                write_FRQ(calc_frequency);
                if (validFRQ) {
                  strcpy(FRQ_old,FRQ);
                } else {
                  strcpy(FRQ,FRQ_old); //"   .   ";
                  numChar = 0;
                  validFRQ = Calculate_Frequency(FRQ);
                  write_FRQ(calc_frequency);
                  //sound audible error here
                }
              } // numChar==7
            }//numchar<6
            
          break; 
        }
      }//pressedKEY != 'X'    
    } //scrMODE=scrNORMAL
    //Put 8574_ into READ mode by setting ports high
    Wire.beginTransmission(0x20);
    Wire.write(255); 
    Wire.endTransmission();
    //Toggle interupt PIN state holder
    old_KeyVal = KeyVal;
   } //KeyVal!=old_keyval


if (commandComplete) {
    if (commandString.charAt(0) == '\n') PrintMenu();
    //if (commandString.charAt(0) == 'Y') commandYardim(commandString.charAt(2));
    if (commandString.charAt(0) == 'C') commandCevrim(commandString.charAt(2));
    if (commandString.charAt(0) == 'A') commandAcilis();
    if (commandString.charAt(0) == 'T') commandAPRSSure();
    if (commandString.charAt(0) == 'M') commandAPRSMesaj();    
    if (commandString.charAt(0) == 'S') commandAPRSmycall();    
    if (commandString.charAt(0) == 'H') commandHafizaDok();
    if (commandString.charAt(0) == 'K') commandAyarDok();
    if (commandString.charAt(0) == 'C') commandHafizaKoy();
    if (commandString.charAt(0) == 'P') commandTogglePTT();
    if (commandString.charAt(0) == 'G') getGPSData();
    
//   Serial.println("Gecersiz bir komut... tekrar deneyiniz...");
    commandString = "";
    commandComplete = false;
    PrintMenu();
    Serialprint("\r\nSeciminiz>");
  }


//  send_packet(_STATUS);  
//  delay(tx_delay);
//  randomize(tx_delay, 10, 5000);
//  randomize(str_len, 10, 420);
   
} //loop


void StreamPrint_progmem(Print &out,PGM_P format,...)
{
  #ifdef SERIALMENU 
  char formatString[128], *ptr;
  strncpy_P( formatString, format, sizeof(formatString) ); // copy in from program mem
  // null terminate - leave last char since we might need it in worst case for result's \0
  formatString[ sizeof(formatString)-2 ]='\0'; 
  ptr=&formatString[ strlen(formatString)+1 ]; // our result buffer...
  va_list args;
  va_start (args,format);
  vsnprintf(ptr, sizeof(formatString)-1-strlen(formatString), formatString, args );
  va_end (args);
  formatString[ sizeof(formatString)-1 ]='\0'; 
  out.print(ptr);
  #endif
}


void serialEvent() {
  //Serialprint(".");
  cli();
  while (Serial.available()) {
    //char inChar = UDR0; //
    char inChar = (char)Serial.read();
    //Serialprint(inChar); //local echo
    commandString += inChar;
    if (inChar == '\n') {
      commandComplete = true;
    }
  }
  sei();
}


/*
 * seri olarak okunan byte lari 0x09 veya 0x04 e kadar olanini geri dondurmek icin
 */
/*
void readParam(char *szParam, int iMaxLen) {
  byte c;
  int iSize;
  unsigned long iMilliTimeout = millis() + 2000; 

  for (iSize=0; iSize<iMaxLen; iSize++) szParam[iSize] = 0x00; 
  iSize = 0;   

  while (millis() < iMilliTimeout) {

    if (Serial.available()) {
      c = Serial.read();

      if (c == 0x09 || c == 0x04) {
        Serialprint("\r\n");
        return;
      }
      if (iSize < iMaxLen) {
        szParam[iSize] = c;
        iSize++;
      }
    }
  }

}

bool getFromSerialport() {
  char szParam[10]; //maximum length of a variable value
  unsigned long iMilliTimeout = millis() + 10000;    
  while (millis() < iMilliTimeout) {
  while (!Serial.available()) { } 
    if (Serial.read() == 0x01) {   //The stream starts with a 0x01
      readParam(szParam, sizeof(szParam));
      if (strcmp(szParam, VERSIYON) != 0) {
        DEBUG_PORT.println(szParam);
          DEBUG_PORT.println(F("E99 Versiyonlar uyumsuz..."));
        return false;
      }
    
      readParam(szParam, sizeof(Ayarlar.APRS_CagriIsareti));    //CagriIsareti
      strcpy(Ayarlar.APRS_CagriIsareti, szParam);
      readParam(szParam, 1);    //CagriIsareti SSID
      Ayarlar.APRS_CagriIsaretiSSID = szParam[0];


      readParam(szParam, sizeof(Ayarlar.APRS_Destination));    //Destination
      strcpy(Ayarlar.APRS_Destination, szParam);
      readParam(szParam, 1);    //SSID
      Ayarlar.APRS_DestinationSSID = szParam[0];

      readParam(szParam, sizeof(Ayarlar.APRS_Path1));    //Path1
      strcpy(Ayarlar.APRS_Path1, szParam);
      readParam(szParam, 1);    //SSID
      Ayarlar.APRS_Path1SSID = szParam[0];

      readParam(szParam, sizeof(Ayarlar.APRS_Path2));    //Path2
      strcpy(Ayarlar.APRS_Path2, szParam);
      readParam(szParam, 1);    //SSID
      Ayarlar.APRS_Path2SSID = szParam[0];

      //Symbol/Tab
      readParam(szParam, 1);
      Ayarlar.APRS_Sembolu = szParam[0];
      readParam(szParam, 1);
      Ayarlar.APRS_SembolTabi = szParam[0];

      //BeaconTipi
      readParam(szParam, sizeof(szParam));
      Ayarlar.APRS_BeaconTipi = atoi(szParam);

      //Beacon Suresi
      readParam(szParam, sizeof(szParam));
      Ayarlar.APRS_BeaconSuresi = atoi(szParam);

      //GPS Seri Hizi
      readParam(szParam, sizeof(szParam));
      Ayarlar.APRS_GPSSeriHizi = atoi(szParam);

      //Status Message
      readParam(szParam, sizeof(szParam));
      strcpy(Ayarlar.APRS_Mesaj, szParam);

      //GPS Var/Yok
      readParam(szParam, sizeof(szParam));
      Ayarlar.gps_varyok = atoi(szParam);    



      unsigned int iCheckSum = 0;
      for (int i=0; i<7; i++) {
        iCheckSum += Ayarlar.APRS_CagriIsareti[i];
      }
      Ayarlar.CheckSum = iCheckSum;
      return true;    //OKuma tamamlandi
    } // read 0x01
  } //millis
  return false;

}
*/





/*
 * 
 */
void set_nada_1200(void)
{
  digitalWrite(MIC_PIN, HIGH);
  delayMicroseconds(tc1200);
  digitalWrite(MIC_PIN, LOW);
  delayMicroseconds(tc1200);
}

void set_nada_2400(void)
{
  digitalWrite(MIC_PIN, HIGH);
  delayMicroseconds(tc2400);
  digitalWrite(MIC_PIN, LOW);
  delayMicroseconds(tc2400);
  
  digitalWrite(MIC_PIN, HIGH);
  delayMicroseconds(tc2400);
  digitalWrite(MIC_PIN, LOW);
  delayMicroseconds(tc2400);
}

void set_nada(bool nada)
{
  if(nada)
    set_nada_1200();
  else
    set_nada_2400();
}

/*
 * This function will calculate CRC-16 CCITT for the FCS (Frame Check Sequence)
 * as required for the HDLC frame validity check.
 * 
 * Using 0x1021 as polynomial generator. The CRC registers are initialized with
 * 0xFFFF
 */
void calc_crc(bool in_bit)
{
  unsigned short xor_in;
  
  xor_in = crc ^ in_bit;
  crc >>= 1;

  if(xor_in & 0x01)
    crc ^= 0x8408;
}

void send_crc(void)
{
  unsigned char crc_lo = crc ^ 0xff;
  unsigned char crc_hi = (crc >> 8) ^ 0xff;

  send_char_NRZI(crc_lo, HIGH);
  send_char_NRZI(crc_hi, HIGH);
}

void send_header(void)
{
  char temp;

  /*
   * APRS AX.25 Header 
   * ........................................................
   * |   DEST   |  SOURCE  |   DIGI   | CTRL FLD |    PID   |
   * --------------------------------------------------------
   * |  7 bytes |  7 bytes |  7 bytes |   0x03   |   0xf0   |
   * --------------------------------------------------------
   * 
   * DEST   : 6 byte "callsign" + 1 byte ssid
   * SOURCE : 6 byte your callsign + 1 byte ssid
   * DIGI   : 6 byte "digi callsign" + 1 byte ssid
   * 
   * ALL DEST, SOURCE, & DIGI are left shifted 1 bit, ASCII format.
   * DIGI ssid is left shifted 1 bit + 1
   * 
   * CTRL FLD is 0x03 and not shifted.
   * PID is 0xf0 and not shifted.
   */

  /********* DEST ***********/
  temp = strlen(dest);
  for(int j=0; j<temp; j++)
    send_char_NRZI(dest[j] << 1, HIGH);
  if(temp < 6)
  {
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, HIGH);
  }
  send_char_NRZI('0' << 1, HIGH);

  
  /********* SOURCE *********/
  temp = mycall.length();
  for(int j=0; j<temp; j++)
    send_char_NRZI(mycall[j] << 1, HIGH);
  if(temp < 6)
  {
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, HIGH);
  }
  send_char_NRZI((myssid + '0') << 1, HIGH);

  
  /********* DIGI ***********/
  temp = strlen(digi);
  for(int j=0; j<temp; j++)
    send_char_NRZI(digi[j] << 1, HIGH);
  if(temp < 6)
  {
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, HIGH);
  }
  send_char_NRZI(((digissid + '0') << 1) + 1, HIGH);

  /***** CTRL FLD & PID *****/
  send_char_NRZI(_CTRL_ID, HIGH);
  send_char_NRZI(_PID, HIGH);
}

void send_payload(char type)
{
  /*
   * APRS AX.25 Payloads
   * 
   * TYPE : POSITION
   * ........................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|
   * --------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |
   * --------------------------------------------------------
   * 
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * 
   * 
   * TYPE : STATUS
   * ..................................
   * |DATA TYPE |    STATUS TEXT      |
   * ----------------------------------
   * |  1 byte  |       N bytes       |
   * ----------------------------------
   * 
   * DATA TYPE  : >
   * STATUS TEXT: Free form text
   * 
   * 
   * TYPE : POSITION & STATUS
   * ..............................................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|    STATUS TEXT      |
   * ------------------------------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |       N bytes       |
   * ------------------------------------------------------------------------------
   * 
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * STATUS TEXT: Free form text
   * 
   * 
   * All of the data are sent in the form of ASCII Text, not shifted.
   * 
   */
  if(type == _FIXPOS)
  {
    send_char_NRZI(_DT_POS, HIGH);
    send_string_len(lat, lat.length());
    send_char_NRZI(sym_ovl, HIGH);
    send_string_len(lon, lon.length());
    send_char_NRZI(sym_tab, HIGH);
  }
  else if(type == _STATUS)
  {
    send_char_NRZI(_DT_STATUS, HIGH);
    send_string_len(APRS_Mesaj, APRS_Mesaj.length());
  }
  else if(type == _FIXPOS_STATUS)
  {
    send_char_NRZI(_DT_POS, HIGH);
    send_string_len(lat, lat.length());
    send_char_NRZI(sym_ovl, HIGH);
    send_string_len(lon, lon.length());
    send_char_NRZI(sym_tab, HIGH);

    send_char_NRZI(' ', HIGH);
    send_string_len(APRS_Mesaj, APRS_Mesaj.length());
  }
}

/*
 * This function will send one byte input and convert it
 * into AFSK signal one bit at a time LSB first.
 * 
 * The encode which used is NRZI (Non Return to Zero, Inverted)
 * bit 1 : transmitted as no change in tone
 * bit 0 : transmitted as change in tone
 */
void send_char_NRZI(unsigned char in_byte, bool enBitStuff)
{
  bool bits;
  
  for(int i = 0; i < 8; i++)
  {
    bits = in_byte & 0x01;

    calc_crc(bits);

    if(bits)
    {
      set_nada(nada);
      bit_stuff++;

      if((enBitStuff) && (bit_stuff == 5))
      {
        nada ^= 1;
        set_nada(nada);
        
        bit_stuff = 0;
      }
    }
    else
    {
      nada ^= 1;
      set_nada(nada);

      bit_stuff = 0;
    }

    in_byte >>= 1;
  }
}

void send_string_len(String in_string, int len)
{
  for(int j=0; j<len; j++)
    send_char_NRZI(in_string[j], HIGH);
}

void send_flag(unsigned char flag_len)
{
  for(int j=0; j<flag_len; j++)
    send_char_NRZI(_FLAG, LOW); 
}

/*
 * In this preliminary test, a packet is consists of FLAG(s) and PAYLOAD(s).
 * Standard APRS FLAG is 0x7e character sent over and over again as a packet
 * delimiter. In this example, 100 flags is used as the preamble and 3 flags as
 * the postamble.
 */
void send_packet(char packet_type)
{
  /*
   * AX25 FRAME
   * 
   * ........................................................
   * |  FLAG(s) |  HEADER  | PAYLOAD  | FCS(CRC) |  FLAG(s) |
   * --------------------------------------------------------
   * |  N bytes | 22 bytes |  N bytes | 2 bytes  |  N bytes |
   * --------------------------------------------------------
   * 
   * FLAG(s)  : 0x7e
   * HEADER   : see header
   * PAYLOAD  : 1 byte data type + N byte info
   * FCS      : 2 bytes calculated from HEADER + PAYLOAD
   */

   strcpy(FRQ_old,FRQ); //store old frequency for recall
   shiftMODE = noSHIFT; //get into SIMPLEX mode for caculations
   TRX_MODE = TX;
   numberToFrequency(144800,FRQ);
   validFRQ = Calculate_Frequency(FRQ);
   write_FRQ(calc_frequency);
   writeFRQToLcd(FRQ);
   digitalWrite(PTT_OUTPUT_PIN,HIGH);
   
  //Serialprint("Sending packet..."); 
  noTone(MIC_PIN);
  delay(300); //TODO: put define for TXDELAY
  
  //prepare and send APRS message
  send_flag(100);
  crc = 0xffff;
  send_header();
  send_payload(packet_type);
  send_crc();
  send_flag(3);

  //APRS message transmission ended
  digitalWrite(PTT_OUTPUT_PIN,LOW); // now stop transmitting
  //Restoring OLD values
  TRX_MODE = RX;
  strcpy(FRQ,FRQ_old);
  //numberToFrequency((highestFRQ+lowestFRQ)/2,FRQ);
  validFRQ = Calculate_Frequency(FRQ);
  write_FRQ(calc_frequency);              
}

/*
 * Function to randomized the value of a variable with defined low and hi limit value.
 * Used to create random AFSK pulse length.
 */
 /*
void randomize(unsigned int &var, unsigned int low, unsigned int high)
{
  var = random(low, high);
}
*/
/*
void print_version(void)
{
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
  Serial.println(" ");
}
*/

void getGPSData()
{
  TinyGPS gps;
  SoftwareSerial ss(A1, A3);
  ss.begin(9600);
  float flat, flon;
  unsigned long age = 0;
  //unsigned long age, date, time, chars = 0;
  //unsigned short sentences = 0, failed = 0;
  
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < 1000); //read within 1 second

  //Serialprint(gps.satellites());
  gps.f_get_position(&flat, &flon, &age);
/*
  Serialprint(flat);
  Serialprint(" ");
  Serialprint(flon);
  Serialprint(" ");
  Serialprint(age);
  Serialprint(" ");
  Serialprint(gps.f_altitude());
  Serialprint(" ");
  Serialprint(gps.f_course());
  Serialprint(" ");
  Serialprint(gps.f_speed_kmph());
  Serialprint("\r\n");
*/
      
}
