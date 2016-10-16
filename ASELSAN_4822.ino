#include <Wire.h>


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

int inPin = 6;  //Interrupt Input PIN for MCU
int val = 0;     // variable to store the read value
int old_val= 0;


// Matrix which hold the LCD data (8 segments * 3 bytes per segment)
unsigned char matrix[24];
unsigned char chr2wr[3];


const char* keymap[4] = {  "123DSX",  "456TB", "789OC", "*0#UM"  };

int numChar = 0;
char* FRQ = "156.500";


//
//MC145158 Programming
//
int pll_clk  = 9;
int pll_data = 8;
int pll_ena  = 7;


//GLOBAL VARIABLES

enum TRX_MODE { RX = 1, TX = 2 };

//enum TRX { TX,RX };

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
  B11111111, B11111111, B11110000, //% Special character to light all segments
};

/* Physically send out the given data */
void sendToLcd(byte *data, byte position) {
  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_deviceselect);     // I think this is needed, as internally the data can overflow and the PCF will automatically select the next device.
  Wire.write(LASTCMD | position * 5 | 0);  // This will always set everything at once, starting from the beginning
  Wire.write(data, 3);                     // Store all 20 bytes
  Wire.endTransmission();    
}


void writeToLcd(const char *text) {
  memset(chr2wr, 0, 3);
  
  for (int idx=0; idx!=strlen(text); idx++) {
      if (idx > 7) break;   
      char *c = strchr(index, (int)toupper(text[idx]));
      int pos;
      if (c == NULL) { 
                        pos = 0;      // Char not found, use underscore space instead
                      } else 
                      {
                        pos = c - index;
                      }
      matrix[3*idx+0] = font[(pos * 3)+0]; 
      matrix[3*idx+1] = font[(pos * 3)+1];
      if (idx>0) {
            matrix[3*idx+2] = font[(pos * 3)+2] | (matrix[3*(idx+1)] & B00001111);  // four bits should be from the existing character
          } else
          {
            matrix[3*idx+2] = font[(pos * 3)+2]; 
          }
      
      chr2wr[0] = matrix[3*idx+0];
      chr2wr[1] = matrix[3*idx+1];
      chr2wr[2] = matrix[3*idx+2];
      //Send data to LCD for appropriate position 
      sendToLcd(chr2wr,idx);
  }
  

//  sendToLcd(matrix);
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

void send_SPIBit(int Counter, byte length)
{
  for (int i=length-1;i>=0;i--)
  {
    byte data=bitRead(Counter,i);
    if (data==1) 
      {
       digitalWrite(pll_data, HIGH);    // Load 1 on DATA
      } else {
       digitalWrite(pll_data, LOW);    // Load 1 on DATA
      }
      delay(1);
      digitalWrite(pll_clk,HIGH);    // Bring pin CLOCK high
      delay(1);
      digitalWrite(pll_clk,LOW);    // Then back low
  }
  Serial.println("");
  
  
}
void   send_SPIEnable()
{
  digitalWrite(pll_ena, HIGH);   // Bring ENABLE high
  delay(1);
  digitalWrite(pll_ena, LOW);    // Then back low  
}


void write_FRQ(unsigned long Frequency)
{
  Frequency = Frequency + 90000;
  int R_Counter = 12800 / 25;  //12.8Mhz reference clock, 25Khz step
  int N_Counter = Frequency / 25 / 80 ; //prescaler = 80, channel steps 25Khz
  int A_Counter = (Frequency / 25) - (80 * N_Counter);

  Serial.print("FREQUENCY :");
  Serial.println(Frequency,DEC);  
  Serial.print("R :");
  Serial.println(R_Counter,DEC);
  Serial.print("N :");
  Serial.println(N_Counter,DEC);
  Serial.print("A :");
  Serial.println(A_Counter,DEC);
  
  send_SPIBit(R_Counter,14);
  send_SPIBit(1,1); // Tell PLL that it was the R Counter
  send_SPIEnable();
  send_SPIBit(N_Counter,10);
  send_SPIBit(A_Counter,7);
  send_SPIBit(0,1); // Tell PLL that it was the A and N Counters  
  send_SPIEnable();  
  
}



void setup(){
  Serial.begin(9600);
  Serial.println("Init start");

 pinMode(inPin,    INPUT);
 pinMode(pll_clk, OUTPUT);
 pinMode(pll_data,OUTPUT);
 pinMode(pll_ena, OUTPUT);
 
 
 digitalWrite(pll_clk, LOW);
 digitalWrite(pll_data,LOW);
 digitalWrite(pll_ena, LOW);
 
 //write_FRQ(200900);
 
  memset(matrix, 0, 24);
  Wire.begin(); 
  delay(100);  // Give some time to boot

  // Init
  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_modeset); 
  Wire.write(NEXTCMD | set_deviceselect); 
  Wire.write(NEXTCMD | set_blink); 
  Wire.write(LASTCMD | set_datapointer); 
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.write(B00000000);  
  Wire.endTransmission();  

  writeToLcd("TA7W");
  delay(1500);
  writeToLcd("BARIS");
  delay(1500);
  writeToLcd(FRQ);
  
//  Serial.println("Init completed");
}

void loop(){
  writeToLcd(FRQ);
  
  //Output data to Keyboard... First first bits for keyboard, next bits for backlight and leds... 
  Wire.beginTransmission(PCF8574_KEYB_LED);
  Wire.write(0); 
  Wire.endTransmission();

  //this is our interrupt pin... Move this to a proper interrupt rutine
  val = digitalRead(inPin);

if (val != old_val) 
{
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
    
//    Serial.print("SUTUN : ");
//    Serial.println(c,DEC);         // print the character
    
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
    
    char BASILAN = keymap[sutun][satir];
    if (BASILAN != 'X')
      {
        Serial.print("BASILAN  : ");
        Serial.println(BASILAN);


        if (BASILAN == '*') 
          {
            Serial.println(FRQ[0]-48,DEC);
            Serial.println(FRQ[1]-48,DEC);
            Serial.println(FRQ[2]-48,DEC);
            Serial.println(FRQ[4]-48,DEC);
            Serial.println(FRQ[5]-48,DEC);
            Serial.println(FRQ[6]-48,DEC);
            unsigned long frekans = ((FRQ[0]-48) * 100000) + ((FRQ[1]-48) * 10000) + ((FRQ[2]-48) * 1000) + ((FRQ[4]-48) * 100) + ((FRQ[5]-48) * 10) + (FRQ[6]-48);
            Serial.println(frekans,DEC);
            numChar = 0;
            write_FRQ(frekans);
          }
        else  
          {
            if (BASILAN == '#') 
              {
                FRQ = "___.___";
                numChar = 0;
              } 
             else
            { 
              if (numChar <= 6) 
                {
                  FRQ[numChar] = BASILAN;
                  numChar = numChar + 1;
                  if (numChar == 3) 
                    {
                      FRQ[3] = '.';
                      numChar = numChar + 1;
                    }
                }
            }
          }
          
      }   
//    Serial.print("SATIR : ");
//    Serial.println(r,DEC);  
    //Put 8574_ into READ mode by setting ports high
    Wire.beginTransmission(0x20);
    Wire.write(255); 
    Wire.endTransmission();
    //Toggle interupt PIN state holder
    old_val = val;
}

//  scroll("       TA7W.... MERHABA BIR TELSIZIM, SIMDILIK SADECE YAZI YAZIYORUM... AMA YAKINDA HERSEYIM CALISACAK...   ", 200);
}

