#include "main.h"

void InitLCD() {
  byte err;
  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_modeset); 
  Wire.write(NEXTCMD | set_deviceselect); 
  Wire.write(NEXTCMD | set_blink); 
  Wire.write(LASTCMD | set_datapointer); 
  for (int i=0;i<20;i++)   Wire.write((uint8_t)0x00);  
  err = Wire.endTransmission();  
  Serial.print("InitLCD Adim 1 Sonuc: "); Serial.println(err);
  delay(100);

  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_modeset); 
  Wire.write(NEXTCMD | set_deviceselect); 
  Wire.write(NEXTCMD | set_blink); 
  Wire.write(LASTCMD | set_datapointer); 
  for (int i=0;i<20;i++)   Wire.write((uint8_t)0xFF);  
  err = Wire.endTransmission();  
  Serial.print("InitLCD Adim 2 Sonuc: "); Serial.println(err);
  delay(500);

  Wire.beginTransmission(PCF8576_LCD);
  Wire.write(NEXTCMD | set_modeset); 
  Wire.write(NEXTCMD | set_deviceselect); 
  Wire.write(NEXTCMD | set_blink); 
  Wire.write(LASTCMD | set_datapointer); 
  for (int i=0;i<20;i++)   Wire.write((uint8_t)0x00);  
  err = Wire.endTransmission();  
  Serial.print("InitLCD Adim 3 Sonuc: "); Serial.println(err);
  delay(100);
}

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
    char *c = strchr(font_index, (int)toupper(text[idx]));
    uint8_t pos;
    if (c == NULL) { 
      pos = 0;      // Char not found, use underscore space instead
    } else {
      pos = c - font_index;
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

void scroll(const char *text, int speed) {
  for (uint8_t i=0; i<=strlen(text)-5; i++) {
     writeToLcd(text+i);
     delay(speed);
  }
}

void Greetings() {

 char MSG[9];
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
 delay(1500);

}

void numberToFrequency(uint32_t Freq, char *rFRQ) {
  
  word f1,f2,f3,f4,f5,f6,f7; 

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
  //Serialprint("%c %c %c %c %c %c \r\n",rFRQ[0],rFRQ[1],rFRQ[2],rFRQ[3],rFRQ[4],rFRQ[5],rFRQ[6]);
  //strcpy(rFRQ,"145.775 ");
}

void write_SHIFTtoLCD(uint16_t FRQshift) {
 if (FRQshift>=9975) FRQshift=9975; //upper limit check
 if (FRQshift<=0)    FRQshift=0;    //lower limit check
 
 char MSG[9];
 dtostrf(FRQshift, 7, 0, MSG);
 MSG[8] = 0;
 writeToLcd(MSG);
    
}

void write_TONEtoLCD(unsigned long tone_pos) {
 char MSG[9];
 dtostrf(ctcss_tone_list[tone_pos], 7, 1, MSG);
 MSG[8] = 0;
 writeToLcd(MSG);
}

