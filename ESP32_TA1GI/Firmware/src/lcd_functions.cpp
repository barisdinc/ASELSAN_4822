#include "declarations.h"
// --- İKON TANIMLAMALARI ---
// `const` anahtar kelimesi eklendi, böylece `declarations.h` içindeki `extern const` bildirimiyle eşleşiyorlar.
const byte SPKR[3] = { B00000000, B00000000, B10000000 }; //pos 1 
const byte LOOP[3] = { B00000000, B00010000, B00000000 }; //pos 1
const byte LOCK[3] = { B00000000, B10000000, B00000000 }; //pos 1
const byte ARRW[3] = { B00000000, B00010000, B00000000 }; //pos 2
const byte ASEL[3] = { B00000000, B10000000, B00000000 }; //pos 2
const byte MENU[3] = { B00000000, B00000000, B10000000 }; //pos 7
const byte THUN[3] = { B00000000, B00000000, B00100000 }; //pos 7
const byte NOTE[3] = { B00000000, B00000000, B00010000 }; //pos 7
const byte PLUS[3] = { B00100101, B01000000, B00000000 }; //pos 7
const byte MINS[3] = { B00100000, B01000000, B00000000 }; //pos 7
const byte SPLX[3] = { B00000000, B00000000, B00000000 }; //pos 7

// Keypad ve LCD karakter setleri
const char* keymap[4] = {  "123DSX",  "456RB", "789OC", "*0#UM"  };
const char  numbers[] = "0123456789ABCDEF";
const char font_index[] = "_ /-.*!?<>[]ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789%";

// LCD Sabitleri
byte set_modeset = MODESET | MODE_POWERSAVING | DISPLAY_ENABLED | BIAS_THIRD | DRIVE_4;
byte set_blink = BLINK | BLINKING_ALTERNATION | BLINK_FREQUENCY_OFF;
byte set_datapointer = LOADDATAPOINTER | 0;
byte set_bankselect = BANKSELECT | BANKSELECT_O1_RAM0 | BANKSELECT_O2_RAM0;
byte set_deviceselect = DEVICE_SELECT;

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
void sendToLcd(byte *data, byte position) {
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
    byte pos;
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
  if (APRS_Timeout > 0) hasASEL = true; else hasASEL = false;
  if (RF_POWER_STATE == HIGH_POWER) hasTHUN = true; else hasTHUN = false;
  if (current_ch.tone_enabled == CTCSS_ON) hasNOTE = true; else hasNOTE = false;

  if (hasSPKR) { Position_Signs[0][0] |= SPKR[0]; Position_Signs[0][1] |= SPKR[1]; Position_Signs[0][2] |= SPKR[2]; }
  if (hasLOOP) { Position_Signs[0][0] |= LOOP[0]; Position_Signs[0][1] |= LOOP[1]; Position_Signs[0][2] |= LOOP[2]; }
  if (hasLOCK) { Position_Signs[0][0] |= LOCK[0]; Position_Signs[0][1] |= LOCK[1]; Position_Signs[0][2] |= LOCK[2]; }
  if (hasARRW) { Position_Signs[1][0] |= ARRW[0]; Position_Signs[1][1] |= ARRW[1]; Position_Signs[1][2] |= ARRW[2]; }
  if (hasASEL) { Position_Signs[1][0] |= ASEL[0]; Position_Signs[1][1] |= ASEL[1]; Position_Signs[1][2] |= ASEL[2]; }
  if (hasMENU) { Position_Signs[7][0] |= MENU[0]; Position_Signs[7][1] |= MENU[1]; Position_Signs[7][2] |= MENU[2]; }
  if (hasTHUN) { Position_Signs[7][0] |= THUN[0]; Position_Signs[7][1] |= THUN[1]; Position_Signs[7][2] |= THUN[2]; }
  if (hasNOTE) { Position_Signs[7][0] |= NOTE[0]; Position_Signs[7][1] |= NOTE[1]; Position_Signs[7][2] |= NOTE[2]; }

  if (current_ch.shift_dir == noSHIFT) { Position_Signs[7][0] |= SPLX[0]; Position_Signs[7][1] |= SPLX[1]; Position_Signs[7][2] |= SPLX[2]; }
  if (current_ch.shift_dir == minusSHIFT) { Position_Signs[7][0] |= MINS[0]; Position_Signs[7][1] |= MINS[1]; Position_Signs[7][2] |= MINS[2]; }
  if (current_ch.shift_dir == plusSHIFT) { Position_Signs[7][0] |= PLUS[0]; Position_Signs[7][1] |= PLUS[1]; Position_Signs[7][2] |= PLUS[2]; }

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

// ISTEK: Verilen metni ekranda bir kez kaydırır ve sonrasında frekans ekranını geri yükler
void displayAndScrollOnce(const char* text) {
  char tempFRQ[9];
  strcpy(tempFRQ, FRQ); // Mevcut ekran metnini (frekansı) sakla
  
  int textLen = strlen(text);
  // Metin 8 karakterden kısaysa, sadece göster ve çık.
  if (textLen <= 8) {
      char buffer[9];
      snprintf(buffer, sizeof(buffer), "%-8s", text); // Sağa boşlukla doldur
      writeToLcd(buffer);
      delay(2000); // 2 saniye göster
  } else {
      // Metin 8 karakterden uzunsa kaydır
      for (int i = 0; i <= textLen - 8; i++) {
         char buffer[9];
         strncpy(buffer, text + i, 8);
         buffer[8] = '\0';
         writeToLcd(buffer);
         delay(300); // Kaydırma hızı
      }
      delay(1000); // Son kısmı biraz bekle
  }

  // Saklanan frekans bilgisini ekrana geri yaz
  writeFRQToLcd(tempFRQ);
}


// DEĞİŞİKLİK: Açılış mesajı fonksiyonu basitleştirildi.
void Greetings() {
    char paddedMessage[9]; // 8 karakter + null terminator
    // Mesajı 8 karaktere sığdır, gerekirse boşlukla doldur.
    snprintf(paddedMessage, sizeof(paddedMessage), "%-8.8s", greetingMessage.c_str());
    writeToLcd(paddedMessage);
    delay(2000); // Mesajı 2 saniye göster
}

void write_SHIFTtoLCD(uint16_t FRQshift) {
 if (FRQshift>=9975) FRQshift=9975; //upper limit check
 char MSG[9];
 sprintf(MSG, "%7d", FRQshift);
 writeToLcd(MSG);
}


void write_TONEtoLCD(unsigned long tone_pos) {
 char MSG[9];
 dtostrf(ctcss_tone_list[tone_pos], 7, 1, MSG);
 writeToLcd(MSG);
}

