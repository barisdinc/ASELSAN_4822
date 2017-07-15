#include <Wire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>


//8576 LCD Driver settings
#define NEXTCMD 128     // Issue when there will be more commands after this one
#define LASTCMD 0       // Issue when when this is the last command before ending transmission
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

int Led_Status= 240;

int KeypadIntPin = 4;  //Interrupt Input PIN for MCU
int KeyVal = 0;     // variable to store the read value
int old_KeyVal= 0;

#define FWD_POWER_PIN A6
#define REF_POWER_PIN A7

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

//DUPLEX mode Shift Settinngs
int frqSHIFT = 600;
#define minusSHIFT -1
#define noSHIFT     0
#define  SIMPLEX    0 //Just incase that we can use this term for noSHIFT
#define plusSHIFT   1
int shiftMODE = minusSHIFT; // we start with noSHIFT (SIMPLEX)

//Receive/Transmit and PTT
#define PTT_OUTPUT_PIN 5
#define PTT_INPUT_PIN  12
//Transceiver modes
#define RX 0
#define TX 1
int TRX_MODE = RX; //default transceiver mode is receiving
int LST_MODE = TX; //this will hold the last receive transmit state. Start with TX because we want to write to PLL on startup 


//RF power control definitions
#define RF_POWER_PIN A0
#define HIGH_POWER 0
#define LOW_POWER  1
int RF_POWER_STATE = HIGH_POWER; //Initial Power Level is Hight Power



//Key sounds and alerts
#define ALERT_PIN 13
#define ALERT_OFF 0
#define ALERT_ON  100
int ALERT_MODE = ALERT_ON;

//Tone Types
#define NO_tone  0
#define OK_tone  1
#define ERR_tone 2



//Tone Control For CTCSS tones
#define TONE_PIN  3  //D3 is our tone generation PIN (PWM)
#define CTCSS_OFF 0
#define CTCSS_ON  1
int TONE_CTRL = CTCSS_ON; //we start without CTCSS Tone Control

#define SQL_ACTIVE 2 //CHANNEL ACTIVE (SQUELCH) PIN
#define MUTE_PIN_1 6 //PIN for Audio Muting
//#define MUTE_PIN_2 5
int CHANNEL_BUSY = 1;

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
int scrMODE = scrNORMAL;

#define menuNONE   0
#define menuSQL    1
#define menuTONE   2
#define menuSCAN   3
#define menuRPT    4
#define menuMENU   5
int subMENU = menuNONE;

boolean hasASEL = false; //ASELSAN sign
boolean hasLOCK = false; //KEY LOCK sign
boolean hasSPKR = false; //SPEAKER sign
boolean hasTHUN = false; //TUNHDER sign
boolean hasARRW = false; //ARROW sign
boolean hasMENU = false; //MENU sign
boolean hasLOOP = false; //LOOP sign
boolean hasNOTE = false; //NOTE sign

byte Position_Signs[8][3] = { 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0 } ; //The signs that appear while printing at position 0 to 7 

byte SPKR[3] = { B00000000, B00000000, B10000000 }; //pos 1 
byte LOOP[3] = { B00000000, B00010000, B00000000 }; //pos 1
byte LOCK[3] = { B00000000, B10000000, B00000000 }; //pos 1
byte ARRW[3] = { B00000000, B00010000, B00000000 }; //pos 2
byte ASEL[3] = { B00000000, B10000000, B00000000 }; //pos 2
byte MENU[3] = { B00000000, B00000000, B10000000 }; //pos 7
byte THUN[3] = { B00000000, B00000000, B00100000 }; //pos 7
byte NOTE[3] = { B00000000, B00000000, B00010000 }; //pos 7
byte PLUS[3] = { B00100101, B01000000, B00000000 }; //pos 7
byte MINS[3] = { B00100000, B01000000, B00000000 }; //pos 7
byte SPLX[3] = { B00000000, B00000000, B00000000 }; //pos 7

// Matrix which hold the LCD data (8 segments * 3 bytes per segment)
unsigned char matrix[24];
unsigned char chr2wr[3];

const char* keymap[4] = {  "123DSX",  "456RB", "789OC", "*0#UM"  };
const char  numbers[] = "0123456789ABCDEF";

int numChar = 0;
char FRQ[9];// = "145.675 ";
char FRQ_old[9];// = FRQ;
long calc_frequency;
boolean validFRQ; //Is the calculated frequenct valid for our ranges


/* Text to LCD segment mapping. You can add your own symbols, but make sure the index and font arrays match up */
const char index[] = "_ /-.*!?<>[]ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789%";
const unsigned char font[] = {     // only first 20 bits belong to this character, rest is 1 for logica & (AND) operation
  B00010000, B00000000, B00001111, // _ underscore
  B00000000, B00000000, B00000000, // space  
  B01000000, B00000010, B00000000, // / division
  B00100000, B00000100, B00000000, // - dash  
  B00000000, B00000000, B00010000, // - dot
  B11100101, B00000111, B00000000, // * asterix
  B00000101, B00000000, B00010000, // ! exclamation mark
  B00000011, B00000010, B00110000, // ? question mark
  B00000000, B00000011, B00000000, // < less than
  B11000000, B00000000, B00000000, // > greater than
  B00010010, B00000000, B01100000, // [ bracket open
  B00010010, B01100000, B00000000, // ] bracket close 
  B00100010, B01100100, B01100000, //A
  B00110010, B01100100, B01100000, //B
  B00010010, B00000000, B01100000, //C
  B00010010, B01100000, B01100000, //D
  B00110010, B00000100, B01100000, //E
  B00100010, B00000000, B01100000, //F
  B00110010, B01000100, B01100000, //G
  B00100000, B01100100, B01100000, //H
  B00010111, B00000000, B00000000, //I
  B01000110, B00000000, B01000000, //J
  B00000101, B00000011, B00000000, //K
  B00010000, B00000000, B01100000, //L
  B10000000, B01100010, B01100000, //M
  B10000000, B01100001, B01100000, //N
  B00010010, B01100000, B01100000, //O
  B00100010, B00100100, B01100000, //P
  B00010010, B01100001, B01100000, //Q
  B00100010, B00100101, B01100000, //R
  B00110010, B01000100, B00100000, //S
  B00000111, B00000000, B00000000, //T
  B00010000, B01100000, B01100000, //U
  B10000000, B01100001, B00000000, //V
  B01000000, B01100001, B01100000, //W
  B11000000, B00000011, B00000000, //X
  B10000001, B00000010, B00000000, //Y
  B01010010, B00000010, B00000000, //Z
  B00010010, B01100000, B01100000, //O
  B00000000, B01100010, B00000000, //1
  B00110010, B00100100, B01000000, //2
  B00110010, B01100100, B00000000, //3
  B00100000, B01100100, B00100000, //4
  B00110010, B01000100, B00100000, //5
  B00110010, B01000100, B01100000, //6
  B00000010, B01100000, B00000000, //7
  B00110010, B01100100, B01100000, //8
  B00110010, B01100100, B00100000, //9
  B00000000, B00000000, B11110000, //% Special character to light all segments
};

//VNA variables
float minSWR;
long lowestFRQ;
long highestFRQ;


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
 MSG[0] = EEPROM.read(9);
 MSG[1] = EEPROM.read(10);
 MSG[2] = EEPROM.read(11);
 MSG[3] = EEPROM.read(12);
 MSG[4] = EEPROM.read(13);
 MSG[5] = EEPROM.read(14);
 MSG[6] = EEPROM.read(15);
 MSG[7] = EEPROM.read(16);
 MSG[8] = 0;
 writeToLcd(MSG);
 delay(500);
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
    //Serial.println(strlen(text),DEC);
    if (idx > 7) break;   
    char *c = strchr(index, (int)toupper(text[idx]));
    int pos;
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
  //Serial.println("");
}

void send_SPIEnable() {
  digitalWrite(pll_ena_pin, HIGH);   // Bring ENABLE high
  delay(1);
  digitalWrite(pll_ena_pin, LOW);    // Then back low  
}


char numberToArray (int Number) //max 4 digits
{
//TODO: 
//  char result = "         ";
//  result[0] = String(Number / 1000);
//  result[1] = String(Number / 100);
//  result[2] = String(Number / 10);
//  result[3] = String(Number / 1);
  
  
}

boolean Calculate_Frequency (char mFRQ[9]) {

  calc_frequency = ((mFRQ[0]-48) * 100000L) + ((mFRQ[1]-48) * 10000L)  + ((mFRQ[2]-48) * 1000) + ((mFRQ[4]-48) * 100) + ((mFRQ[5]-48) * 10) + (mFRQ[6]-48);
  if ((calc_frequency >= 134000L) & (calc_frequency <= 174000L)) return true; //valid frequency
  else 
    {
      Alert_Tone(ERR_tone);
      return false; //invalid frequency
    }
}

void write_FRQ(unsigned long Frequency) {
// 0 0     152.2   172.6
// 0 1     147.9   165.5
// 1 0     141.6   157.1
// 1 1     137.2   151.4
  if (validFRQ) {
    if ((Frequency < 174000L) & Frequency >= (164000L))  { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, LOW);  }
    if ((Frequency < 164000L) & Frequency >= (154000L))  { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, HIGH); } 
    if ((Frequency < 154000L) & Frequency >= (144000L))  { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, LOW);  } 
    if ((Frequency < 146000L) & Frequency >= (134000L))  { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, HIGH); } 

    // Update EEPROM for last used Frequncy
    double UpdatedFrq = Frequency - 130000; // Subtrack 130000 to fit the frequency into double size (2 bytes) 
    byte FRQ_L = UpdatedFrq / 256;
    byte FRQ_H = UpdatedFrq - ( FRQ_L * 256);
    EEPROM.write(50,FRQ_L); 
    EEPROM.write(51,FRQ_H);  
    
    if (TRX_MODE == RX) Frequency = Frequency + 45000L;
    if (TRX_MODE == TX) Frequency = Frequency + (shiftMODE * frqSHIFT) ; // Add/remove transmission shift
  
    int R_Counter = 12800 / 25;  //12.8Mhz reference clock, 25Khz step
    int N_Counter = Frequency / 25 / 80 ; //prescaler = 80, channel steps 25Khz
    int A_Counter = (Frequency / 25) - (80 * N_Counter);
  
    digitalWrite(PLL_SEC, LOW); //SELECT PLL for SPI BUS  
    send_SPIBit(R_Counter,14);
    send_SPIBit(1,1); // Tell PLL that it was the R Counter
    send_SPIEnable();
    send_SPIBit(N_Counter,10);
    send_SPIBit(A_Counter,7);
    send_SPIBit(0,1); // Tell PLL that it was the A and N Counters  
    send_SPIEnable();  
    digitalWrite(PLL_SEC, HIGH); //DE-SELECT PLL for SPI BUS
    
  } //validFRQ 
}


void SetTone(int toneSTATE) {
  //Serial.println(toneSTATE);
  noTone(ALERT_PIN);
  if (toneSTATE == CTCSS_ON) { 
    if (TRX_MODE == TX)  tone(TONE_PIN, 88.5);
      else noTone(TONE_PIN);
  }
}


void Alert_Tone(int ToneType)
{
  if (TRX_MODE == TX)  return; //If we are transmitting, do not play tones, because tone pin might be busy with CTCSS generation
  noTone(TONE_PIN); //First silence the TONE output first
  if (ToneType == OK_tone)  tone(ALERT_PIN,1000,ALERT_MODE);   //short 1Khz is OK  tone
  if (ToneType == ERR_tone) tone(ALERT_PIN,400 ,ALERT_MODE*2); //long 440hz is ERR tone
  delay(ALERT_MODE); //TODO: find a better way to plat two tones simultaneously
  SetTone(TONE_CTRL);
  
  //SetTone(TONE_CTRL); //resume Tone Generation 
}

void SetRFPower(int rfpowerSTATE) {
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
 Serial.print("initializing EEPROM...");
 EEPROM.write(0, 127); // make eeprom initialized
 EEPROM.write(1, 1);   //Version is 1.0
 EEPROM.write(2, 0);   //
 EEPROM.write(3, 'T'); // Callsign
 EEPROM.write(4, 'A'); // Callsign
 EEPROM.write(5, 'M'); // Callsign
 EEPROM.write(6, 'S'); // Callsign
 EEPROM.write(7, 'A'); // Callsign
 EEPROM.write(8, 'T'); // Callsign
 EEPROM.write(9, ' '); // Message
 EEPROM.write(10,' '); // Message
 EEPROM.write(11,'T'); // Message
 EEPROM.write(12,'A'); // Message
 EEPROM.write(13,'M'); // Message
 EEPROM.write(14,'S'); // Message
 EEPROM.write(15,'A'); // Message
 EEPROM.write(16,'T'); // Message

 for (int location=17;location < 300;location++) EEPROM.write(location,0); // Zeroise the rest of the memory

 EEPROM.write(50,0x3C); // FRQ_L
 EEPROM.write(51,0xF0); // FRQ_H (Default frequency 145.600)
 EEPROM.write(52,0x02); // SHFT_L
 EEPROM.write(53,0x58); // SHFT_H
 EEPROM.write(54,0x01); // TONE
 Serial.println("done..");
}

// Stores frequency data to the desired EEPROM location
void StoreFrequency(char mCHNL[9], char mFRQ[9]) {
 byte ChannelNumber = ((mCHNL[0] - 48) * 10) + (mCHNL[1] - 48);
 byte ChannelLocation = 100 + ChannelNumber * 10;
 Calculate_Frequency(mFRQ); 
 double FrqToStore = calc_frequency - 130000;
 byte FRQ_L = FrqToStore / 256;
 byte FRQ_H = FrqToStore - ( FRQ_L * 256);
 EEPROM.write(ChannelLocation  ,FRQ_L); 
 EEPROM.write(ChannelLocation+1,FRQ_H);  
 //TODO: Add shift and tone information for the channel
 //TODO: Store power level into channel
}

//Retrieves the requested Memory Channel Information from EEPROM
void GetMemoryChannel(char mFRQ[9]) {
 byte ChannelNumber = ((mFRQ[0] - 48) * 10) + (mFRQ[1] - 48);
 byte ChannelLocation = 100 + ChannelNumber * 10;
 byte byte1,byte2;
 byte1 = EEPROM.read(ChannelLocation);
 byte2 = EEPROM.read(ChannelLocation+1);
 long freq = 130000 + (byte1 * 256) + byte2 ;
 numberToFrequency(freq, FRQ);
 Serial.println(FRQ);
 //strcpy(FRQ_old,FRQ);
 //TODO: retrieve shift, tone and power elvels as well
}



void setup() {
  
  //
  // Let's prepare the radio
  //
  delay(100); //startup delay

 // Initialize serial port fr debugging
 Serial.begin(57600);
 Serial.println("Init start");

 // Check EEPROM for stored values
 byte eeprom_state=0;
 eeprom_state = EEPROM.read(0); // first address tells us eeprom status, if different then 127, we need to initialize eeprom structure for first use
 if (eeprom_state != 127) initialize_eeprom();
 if (eeprom_state != 127) Serial.println("EPROM sifirlaniyor");

 //Read Last used frequency
 byte byte1,byte2;
 byte1 = EEPROM.read(50);
 byte2 = EEPROM.read(51);
 long freq = 130000 + (byte1 * 256) + byte2 ;
 numberToFrequency(freq, FRQ);
 strcpy(FRQ_old,FRQ);
  
  //setRadioPower();  //Check power switch mode and turn adio on immediately
  //pinMode(POWER_ON_OFF, INPUT);
  //pinMode(POWER_ON_PIN, OUTPUT);

  pinMode(TONE_PIN, OUTPUT);
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
  digitalWrite(PTT_OUTPUT_PIN, HIGH); //No PTT at startup

  pinMode(FWD_POWER_PIN, INPUT);
  pinMode(REF_POWER_PIN, INPUT);


  pinMode(KeypadIntPin,    INPUT);
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



//  Serial.println("Init completed");
}

void loop() {
  //setRadioPower(); //Check power switch and set radio power mode on or off
  
  if (scrMODE==scrNORMAL) writeFRQToLcd(FRQ); //TODO: We should update the display only on proper display changes.. But this works...

  //Output data to Keyboard... First first bits for keyboard, next bits for backlight and leds... 
  Wire.beginTransmission(PCF8574_KEYB_LED);
  Led_Status = 240;
  if ((CHANNEL_BUSY==0) or (SQL_MODE==SQL_OFF)) Led_Status = Led_Status - green_led; //we are receivig
  if (TRX_MODE == TX) Led_Status = Led_Status - red_led;  //We are transmitting

  //Led_Status = Led_Status - yellow_led;
  //Led_Status = Led_Status - red_led;
  Led_Status = Led_Status - backlight;
  Wire.write(Led_Status); 
  Wire.endTransmission();

  CHANNEL_BUSY = digitalRead(SQL_ACTIVE);  
  if (CHANNEL_BUSY == 0) digitalWrite(MUTE_PIN_1, LOW);
    else if (SQL_MODE == SQL_ON) digitalWrite(MUTE_PIN_1, HIGH); 
      else digitalWrite(MUTE_PIN_1, LOW);

  TRX_MODE = digitalRead(PTT_INPUT_PIN); //read PTT state
  if (TRX_MODE != LST_MODE) {
    LST_MODE = TRX_MODE;
    write_FRQ(calc_frequency); //Update frequenct on every state change
  }
  if (TRX_MODE == TX) digitalWrite(PTT_OUTPUT_PIN,LOW); // now start transmitting
    else digitalWrite(PTT_OUTPUT_PIN, HIGH);

  SetTone(TONE_CTRL); //Change Tone Generation State

  //if (subMENU == menuRPT) writeToLcd(cstr);




  //this is our interrupt pin... Move this to a proper interrupt rutine
  KeyVal = digitalRead(KeypadIntPin);

  
  //Long press on a key detection...
  
  if ((KeyVal == old_KeyVal) & (KeyVal ==  0) & (scrTimer>0)) scrTimer -= 1; //Keypressed and there are counts to go
    else if (KeyVal==1) {
     if (scrTimer==0) { //key released and timeout occured
       writeToLcd("SELECT   ");
       delay(500); //TODO do not use DELAY, change to a timer
       if (pressedKEY=='B') { writeToLcd("TONE     "); subMENU = menuTONE; }
       if (pressedKEY=='S') { writeToLcd("SQL      "); subMENU = menuSQL;  }
       if (pressedKEY=='O') { writeToLcd("SCAN     "); subMENU = menuSCAN; }
       if (pressedKEY=='R') { writeToLcd("REPEAT   "); subMENU = menuRPT;  }
       if (pressedKEY=='M') { writeToLcd("MENU     "); subMENU = menuMENU; }
       delay(500); //TODO do not use DELAY, change to a timer
       scrTimer = TimeoutValue; //Restart the timer
       numChar = 0; //if we were entering frq from keyboard
       scrMODE = scrMENU;    
     }//scrTimer
    }//Keyval==0

  
  
  if (KeyVal != old_KeyVal) { 
    if (KeyVal == 0) Alert_Tone(OK_tone);  

    scrTimer = TimeoutValue; //Restart the timer
    int satir,sutun;
    Wire.requestFrom(0x20,1);
    int c = Wire.read();    // receive a byte as character
    c = 255 - (c | B00000011) ; 
    c = c >> 3;
    satir = 5;
    if (c == 16) satir = 0;
    if (c ==  8) satir = 1;
    if (c ==  4) satir = 2;
    if (c ==  2) satir = 3;
    if (c ==  1) satir = 4;
    
    Wire.beginTransmission(B0100000);
    Wire.write(0); 
    Wire.endTransmission();
    Wire.beginTransmission(B0100001);
    Wire.write(255); 
    Wire.endTransmission();
    
    Wire.requestFrom(0x21,1);
    int r = Wire.read();
    r = 255 - r;
    sutun = 0 ;
    if (r == 8) sutun = 0;
    if (r == 4) sutun = 1;
    if (r == 2) sutun = 2;
    if (r == 1) sutun = 3;
    pressedKEY = keymap[sutun][satir]; //LOOKUP for the pressedKEY
    
  /* -----------------------------------------------------
  /* SCREEN MODE MENU..  
  /* ----------------------------------------------------- */

    if (scrMODE == scrMENU)  { 
      if (pressedKEY != 'X') {
        switch (pressedKEY) {
          case 'U':
            Serial.println("UP");
            if ( subMENU == menuRPT) frqSHIFT += 25;
          case 'D':
            if ( subMENU == menuRPT) frqSHIFT -= 25;            
          case '#': //means CANCEL
            scrMODE = scrNORMAL; //we are in menu or submenus.. return to normal display
            subMENU = menuNONE;
          break; // '#'
          case '*': //means OK
            scrMODE = scrNORMAL; //we are in menu or submenus.. return to normal display
            subMENU = menuNONE;
          break; // '*'
          default:
                Serial.println("def");      
          break; 
        }//switch
      }//pressedKEY!='X'
    }//scrMENU
    
    
    
    
  /* -----------------------------------------------------
  /* SCREEN MODE NORMAL.. WE READ FREQUENCY AND OTHER KEYS 
  /* ----------------------------------------------------- */
    if (scrMODE == scrNORMAL) { //mode NORMAL and key released           
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
             SetRFPower(RF_POWER_STATE);           
          break; //'O'
          case 'U':
            //Serial.print("pressedKEY");
            //Serial.println(pressedKEY,DEC);
            numberToFrequency(calc_frequency+25,FRQ);
            Calculate_Frequency(FRQ);            
          break; // 'U' 
          case 'D':
            //Serial.print("pressedKEY");
            //Serial.println(pressedKEY,DEC);
            numberToFrequency(calc_frequency-25,FRQ);
            Calculate_Frequency(FRQ);            
          break; // 'D'
          case 'M':
            Serial.print("pressedKEY");
            Serial.println(pressedKEY,DEC);
          break; // 'M'
          case 'C':
            //VNA Vector Network analizor Subroutines
            //Serial.print("pressedKEY");
            //Serial.println(pressedKEY,DEC);
            minSWR = 9999;
            lowestFRQ=0;
            highestFRQ=0;           
            //TODO: Store old values of variables (FRQ, SHIFT, etc)
            //TODO: Put transmitter on low power before operation
            //TODO: If any key pressed cancel VNA operation
            strcpy(FRQ_old,FRQ); //store old frequency for recall
            int shiftMODE_old;
            shiftMODE_old = shiftMODE; //store shitODE for recall
            shiftMODE = noSHIFT; //get into SIMPLEX mode for caculations
            SetRFPower(LOW_POWER);
            for (long vna_freq=14000; vna_freq < 15000; vna_freq += 10)
              {
                TRX_MODE = TX;
                //Serial.print(vna_freq); 
                numberToFrequency(vna_freq*10,FRQ);
                validFRQ = Calculate_Frequency(FRQ);
                write_FRQ(calc_frequency);
                writeFRQToLcd(FRQ);
                digitalWrite(PTT_OUTPUT_PIN,LOW);
                delay(50);
                readRfPower(); //TODO: Move under a menu item
                delay(25);
                digitalWrite(PTT_OUTPUT_PIN,HIGH);
              }
              //Restoring OLD values or displaying the best frequency
              SetRFPower(RF_POWER_STATE);
              TRX_MODE = RX;
              //shiftMODE=shiftMODE_old;
              //strcpy(FRQ,FRQ_old);
              numberToFrequency((highestFRQ+lowestFRQ)/2,FRQ);
              validFRQ = Calculate_Frequency(FRQ);
              write_FRQ(calc_frequency);              
          break; // 'C'
          case '#':
            if (numChar == 2) StoreFrequency(FRQ,FRQ_old); // User is trying to store the actual frequency : FRQ[0..1] ccontains memory channel and FRQ_old contains the frequency to be stored
               numChar = 0;
              strcpy(FRQ,FRQ_old);
              validFRQ = Calculate_Frequency(FRQ);
              write_FRQ(calc_frequency);
          break; // '#'
          case '*':
            if (numChar == 2) GetMemoryChannel(FRQ); // User wanted to retrieve the memory channel from EEPRM
            else strcpy(FRQ,FRQ_old); // Otherwise user wanted to cancel the ongoing operatin=on.. return to previous (old) frequency
            validFRQ = Calculate_Frequency(FRQ);
            numChar = 0;
            write_FRQ(calc_frequency);
          break; // '*'
          default:
            /* ------------------    FRQ INPUT ------------ */
            if (numChar <= 6) { //Not a command Key so print it into frequency
              if (numChar == 0) {
                strcpy(FRQ,"        "); //just pressed keys, so clear the screen
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
//  scroll("       TA7W.... MERHABA BIR TELSIZIM, SIMDILIK SADECE YAZI YAZIYORUM... AMA YAKINDA HERSEYIM CALISACAK...   ", 200);
} //loop
