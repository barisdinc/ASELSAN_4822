#include "main.h"
#include "WebUI.h"
#include <Arduino.h> //For platformio compability
//#include <avr/wdt.h>
#include <Wire.h>
#include <EEPROM.h>
//#include <avr/pgmspace.h>
//#include <avr/interrupt.h>
//#include <avr/io.h>
#include <math.h>
#include <stdio.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
//#include "ASELSAN_lcd.h"
//#include "../lib/fontsandicons.h"
//#include "./libraries/PinChangeInt.h"

#define Streamprint(stream,format, ...) StreamPrint_progmem(stream,PSTR(format),##__VA_ARGS__)

//Function definitions
//TODO: move these to a header file
void StreamPrint_progmem(Print &out,PGM_P format,...);






//Interrupts      
// PCINT20 is in PCINT2 vector 
//ISR(PCINT20_vect) 
//{
// KeyVal = digitalRead(KeypadIntPin);
// //digitalWrite(PTT_OUTPUT_PIN, KeyVal); //for testing
//}


void Alert_Tone(int ToneType)
{
  if (TRX_MODE == TX)  return; //If we are transmitting, do not play tones, because tone pin might be busy with CTCSS generation
  noTone(MIC_PIN); //First silence the TONE output first
  pinMode(MIC_PIN, OUTPUT); 
  digitalWrite(MIC_PIN, LOW); // Provide solid ground for microphone to prevent RX audio muffling
  if (ToneType == OK_tone)  { tone(ALERT_PIN,1000,ALERT_MODE); Serialprint(" OK\r\n");}   //short 1Khz is OK  tone
  if (ToneType == ERR_tone)  tone(ALERT_PIN,400 ,ALERT_MODE*2); //long 440hz is ERR tone
  if (ToneType == SUCC_tone) {tone(ALERT_PIN,600 ,ALERT_MODE);delay(200);tone(ALERT_PIN,1000 ,ALERT_MODE);} //2 500msec success tones
  delay(ALERT_MODE); //TODO: find a better way to play two tones simultaneously
  SetTone(current_ch.tone_enabled); //resume Tone Generation 
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
  uint32_t calc_freq = 0;
  if (mFRQ[0] >= '0' && mFRQ[0] <= '9') calc_freq += (mFRQ[0]-48) * 100000L;
  if (mFRQ[1] >= '0' && mFRQ[1] <= '9') calc_freq += (mFRQ[1]-48) * 10000L;
  if (mFRQ[2] >= '0' && mFRQ[2] <= '9') calc_freq += (mFRQ[2]-48) * 1000L;
  if (mFRQ[4] >= '0' && mFRQ[4] <= '9') calc_freq += (mFRQ[4]-48) * 100L;
  if (mFRQ[5] >= '0' && mFRQ[5] <= '9') calc_freq += (mFRQ[5]-48) * 10L;
  if (mFRQ[6] >= '0' && mFRQ[6] <= '9') calc_freq += (mFRQ[6]-48) * 1L;

  if (radio_type==0 && (calc_freq >= (freqLimits.trx_min_25*25)) && (calc_freq <= (freqLimits.trx_max_25*25))) {
      current_ch.frequency = calc_freq;
      return true; //valid frequency for VHF
  }
  else if (radio_type==1 && (calc_freq >= 400000L) && (calc_freq < 468400L)) {
      current_ch.frequency = calc_freq;
      return true; //valid frequency for UHF
  }
  else {
      Alert_Tone(ERR_tone);
      return false; //invalid frequency
  }
}


void SetPLLLock(uint32_t Frequency)
{
  uint32_t f = Frequency;

  // RX: LO = f +/- 45MHz.  TX: shift uygulanir.
  if (TRX_MODE == RX) f = (radio_type == 0) ? (f + 45000L) : (f - 45000L);
  else                f = (uint32_t)((int32_t)f + (int32_t)current_ch.shift_dir * (int32_t)current_ch.shift);

  // Tek yol: RX/TX/UHF farki sadece yukaridaki ofsette.
  pllWriteWords(f);

  // Cift yazim: hattaki parazit bir kelimeyi yarim latch ederse duzeltir.
  if (pllDoubleWrite) {
    delay(1);
    pllWriteWords(f);
  }
}


/* PLL kalibrasyon taramasi.
   N=190 sabit, A 36..56 arasi 12.5kHz adimlarla. Her adimda seri porta ve LCD'ye
   o adimin karsilik geldigi RX frekansi yazilir. Kullanici baska bir telsizden
   145.575 gonderirken hangi adimda duydugunu not eder: aradaki fark gercek
   VCO/filtre hatasini verir. */
void pllSweepTest() {
  const int N = 190;
  Serialprint("\r\n[SWEEP] basladi. Durdurmak icin bir tusa basin.\r\n");
  for (int A = 36; A <= 56; A++) {
    pllWriteNA(N, A);
    uint32_t rx = (uint32_t)((N * 80 + A) * 12.5) - 45000UL;
    numberToFrequency(rx, FRQ);
    writeFRQToLcd(FRQ);
    Serialprint("[SWEEP] A=%d  vco=%lu  rx=%lu\r\n", A,
                (unsigned long)((N * 80 + A) * 12.5), (unsigned long)rx);
    for (int i = 0; i < 12; i++) {          // ~1.2 s, tus kontrolu ile
      delay(100);
      if (readColumn() != 0) { A = 999; break; }
    }
  }
  Serialprint("[SWEEP] bitti.\r\n");
  validFRQ = Calculate_Frequency(FRQ);
  write_FRQ(current_ch.frequency);
}

void write_FRQ(uint32_t Frequency) {//VHF BAND SELECTION  
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

//uint32_t UpdatedFrq = 0;
//float divider = 0;
  writeFRQToLcd(FRQ);
  if (validFRQ) {
    if(radio_type==0)
    {
      if ((Frequency < 174000L) & (Frequency >= (164000L)))  { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, LOW);  }
      if ((Frequency < 164000L) & (Frequency >= (156000L)))  { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, HIGH); } 
      if ((Frequency < 156000L) & (Frequency >= (146000L)))  { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, LOW);  } 
      if ((Frequency < 146000L) & (Frequency >= (134000L)))  { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, HIGH); } 
    }
    else if(radio_type==1)
    {
       if ((Frequency < 470000L) & (Frequency >= (452000L))) {digitalWrite(BAND_SELECT_0, LOW); digitalWrite(BAND_SELECT_1, LOW); }
       if ((Frequency < 452000L) & (Frequency >= (430000L))) {digitalWrite(BAND_SELECT_0, HIGH);digitalWrite(BAND_SELECT_1, HIGH);}
    // Update EEPROM for last used Frequncy
    }
//BD1    current_ch.frequency = Frequency; //UpdatedFrq;
//BD1    EEPROM.put(EEPROM_CURRCHNL_BLCKSTART,current_ch); 
    SetPLLLock(Frequency);
  } //validFRQ 
  else {
    Serialprint("[PLL] ATLANDI! validFRQ=0, freq=%lu\r\n", (unsigned long)Frequency);
  }
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
   //refPower = refPower * 2;
   //int Ptoplam = fwdPower + refPower;
   int Pfark   = fwdPower - refPower;
   float swr = Pfark; //gecici (float)Ptoplam / (float)Pfark;
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
      lowestFRQ=current_ch.frequency;
      highestFRQ=current_ch.frequency;
   } else if (swr == minSWR)
   {
      highestFRQ=current_ch.frequency;
   }

}















/*
  TOT Function

  BAKIM ONARIM KITABI - SF. 205 - 6/25
  
  Göndermede kalma süresi isteğe göre 0-225 saniye aralığında 15 saniyelik adımlarla seçilebilir;
  standart programda 1 dakikadır. Zaman sınırlama süresi bitiminde tekrar bas/konuş mandalina basıldığında
  telsizin göndermeye geçmesi için gereken süre de isteğe göre 0 ile 75 saniye aralığında 5 saniyelik
  adımlarla programlanabilir; standart alarak 0 saniyedir.
*/


int TOT_FLAG = 0;
int TOT_WARN_STATE = 1;

void changeTotSettings(uint8_t timeout, uint8_t lock){
  tot_t  tot_values_new;
  tot_values_new.time_out_time = timeout;
  tot_values_new.lock_time = lock;
  EEPROM.put(EEPROM_TOT, tot_values_new);
  tot_values = tot_values_new;
}

uint8_t TXTimeOutTimer(uint8_t PTT_PIN_STATE) {
  unsigned long currentMillis = millis();
  if (PTT_PIN_STATE == HIGH) {
    if (currentMillis - prevTotMillis >= tot_values.time_out_time* 1000) {
      if (currentMillis - prevTotMillis >= tot_values.time_out_time* 1000 + tot_values.lock_time* 1000) {
        prevTotMillis = currentMillis;
      }
      //digitalWrite(POW_INH, HIGH);
      //Serial.println("transmit interrupted by tot...");
      TOT_FLAG = 1;
      return LOW;
    }
    TOT_FLAG = 0;
    TOT_WARN_STATE = 1;
    return HIGH;
  } else {
    TOT_FLAG = 0;
    TOT_WARN_STATE = 1;
    return LOW;
  }
}





// Stores frequency data to the desired EEPROM location



/*
 * Retrieves the requested Memory Channel Information from EEPROM and returns a memorych_t tyep object
 */
//memorych_t GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg) {


/*
 * Retrieves the requested Memory Channel Information from EEPROM
 * TODO: combine this with the previuous function
 */






/*
 * Change device type between VHF/UHF
 */


/*
 * Change startup screen msg
 */


/*
 * Print the configurations in form of JSON array to Serial Port
 */



/*
 * Print the memory channels in form of JSON array to Serial Port
 */






























int readColumn()
{
    Wire.beginTransmission(0x20); //Here we're pushing VCC from one of PCF8574 and reading the other
    Wire.write((uint8_t)0x00); 
    Wire.endTransmission();
    Wire.beginTransmission(0x21);
    Wire.write((uint8_t)((Led_Status & 0xF0) | 0x0F)); 
    Wire.endTransmission();
    
    Wire.requestFrom(0x21,1);
    int c = Wire.read();

    // Once 0x21 alt 4 bit HIGH iken 0x20'ye 0xFF yaz ki PCF8574 referans durumu 0xFF (tus birakildi) olsun
    Wire.beginTransmission(0x20);
    Wire.write((uint8_t)0xFF);
    Wire.endTransmission();
    // Sonra 0x21 sutunlarini tekrar LOW'a (Led_Status) cek
    Wire.beginTransmission(0x21);
    Wire.write(Led_Status);
    Wire.endTransmission();

    c = (~c) & 0x0F;
    int sutun = 0 ;
    if (c == 8) sutun = 0;
    if (c == 4) sutun = 1;
    if (c == 2) sutun = 2;
    if (c == 1) sutun = 3;

    return sutun;
}

int readRow()
{
    Wire.beginTransmission(0x21);
    Wire.write((uint8_t)(Led_Status & 0xF0));
    Wire.endTransmission();
    Wire.beginTransmission(0x20);
    Wire.write((uint8_t)0xFF);
    Wire.endTransmission();

    Wire.requestFrom(0x20,1);
    int r = Wire.read();    // receive a byte as character
    r = 255 - (r | 0x03) ; 
    r = r >> 3;
    int satir = 5;
    if (r == 16) satir = 0;
    if (r ==  8) satir = 1;
    if (r ==  4) satir = 2;
    if (r ==  2) satir = 3;
    if (r ==  1) satir = 4;
    
    return satir;
}




void startScan()
{


  unsigned long scan_frequency = 144000;
  int8_t frq_step = 25;
  while (1)
  {
        scan_frequency += frq_step;
        if (scan_frequency >= (freqLimits.scn_max_25*25)) scan_frequency = (freqLimits.scn_min_25*25);
        else if (scan_frequency <= (freqLimits.scn_min_25*25)) scan_frequency = (freqLimits.scn_max_25*25);

        numberToFrequency(scan_frequency,FRQ);
        validFRQ = Calculate_Frequency(FRQ);
        write_FRQ(current_ch.frequency);
        writeFRQToLcd(FRQ);
        //Serialprint(">%d\t",scan_freq);
        delay(100);
        CHANNEL_BUSY = digitalRead(SQL_ACTIVE);  
        //int sutun = readColumn();
        //int satir = readRow();
        //Serialprint("keypres= %d %d \r\n",sutun,satir);
        if (CHANNEL_BUSY == 0) break;
        if (readColumn() != 0) break;
  }

}


void resetKeypadPCF() {
  Wire.beginTransmission(PCF8574_KEYB_LED);
  Wire.write((uint8_t)((Led_Status & 0xF0) | 0x0F));
  Wire.endTransmission();
  Wire.beginTransmission(PCF8574_KEYB);
  Wire.write((uint8_t)0xFF);
  Wire.endTransmission();
  Wire.requestFrom(PCF8574_KEYB, 1);
  if (Wire.available()) (void)Wire.read();
  Wire.beginTransmission(PCF8574_KEYB_LED);
  Wire.write(Led_Status);
  Wire.endTransmission();
}

void setup() {
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();
  Wire.setTimeout(10); // 10ms I2C timeout - Pico varsayilani 1000ms, PCF8574 yoksa donmayi onler
  EEPROM.begin(1024); // Pico requires EEPROM.begin
  //wdt_enable(WDTO_4S);
//pin modes and initial states
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(SQL_ACTIVE, INPUT); // Use external pull-up from the radio
  pinMode(MIC_PIN, OUTPUT);
  digitalWrite(MIC_PIN, LOW); // Provide solid ground for microphone at startup to prevent RX audio muffling

  pinMode(RF_POWER_PIN, OUTPUT); //RF power control is output
  digitalWrite(RF_POWER_PIN, RF_POWER_STATE);
  //TODO: store last power state and restore on every boot 
  
  pinMode(MUTE_PIN_1, OUTPUT);
  digitalWrite(MUTE_PIN_1, HIGH); //Mute the Audio output

  pinMode(BAND_SELECT_0, OUTPUT);
  pinMode(BAND_SELECT_1, OUTPUT);

  pinMode(PTT_INPUT_PIN,  INPUT_PULLUP);
  pinMode(PTT_OUTPUT_PIN, OUTPUT);
  pttKey(false); //No PTT at startup

  // Commit 82860a95 ile birebir uyumluluk: GP20 (CDT0) HIGH, GP26 OUTPUT
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  pinMode(26, OUTPUT);
  digitalWrite(26, RF_POWER_STATE);

  pinMode(FWD_POWER_PIN, INPUT);
  pinMode(REF_POWER_PIN, INPUT);

  //pinMode(KeypadIntPin,    INPUT);
	pinMode(KeypadIntPin, INPUT_PULLUP);
 // attachPinChangeInterrupt(KeypadIntPin, KeyPadInterrupt, RISING);						   
  pinMode(pll_clk_pin, OUTPUT);
  pinMode(pll_data_pin,OUTPUT);
  pinMode(pll_ena_pin, OUTPUT);
  pinMode(PLL_SEC, OUTPUT);

  // PLL hatlari 5V CMOS girise gidiyor (VIH = 0.7*VDD = 3.5V). Pico 3.3V ile
  // marj sifir. RP2040 varsayilan surme gucu 4mA; 12mA'ya cikarip dahili
  // pull'lari kapatinca hat daha sert surulur ve esik asilir.
  gpio_set_drive_strength(pll_clk_pin,  GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(pll_data_pin, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(pll_ena_pin,  GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(PLL_SEC,      GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_pulls(pll_clk_pin,  false, false);
  gpio_set_pulls(pll_data_pin, false, false);
  gpio_set_pulls(pll_ena_pin,  false, false);
  gpio_set_pulls(PLL_SEC,      false, false);

  digitalWrite(pll_clk_pin, LOW);
  digitalWrite(pll_data_pin,LOW);
  digitalWrite(pll_ena_pin, LOW);
  digitalWrite(PLL_SEC, LOW); // Keep TD9 (74HC32) OR gate inputs at 0V so 3.3V SPI passes cleanly

  // PLL yazim profilini uygula.
  // Mod 1 = 100us bit (~10ms toplam yazim suresi, K4 uzerindeki 1nF filtre kapasitorleri icin güvenli).
  pllApplyMode(1);

  //cli(); // Turn Off Interrupts
  //PCICR = (1 << PCIE2);  
  //PCMSK2 |= (1 << PCINT20); 
  //sei();
  delay(100);

  Serial.begin(115200);
  delay(3000); // Wait for USB Serial connection to be established
  commandString.reserve(200);

  // Check EEPROM for stored values
  uint8_t eeprom_state = EEPROM.read(0); // EEPROM Check For Modification Board
  uint8_t layout_ver = EEPROM.read(EEPROM_LAYOUT_VER_ADDR);
  
  if (eeprom_state != 128 || layout_ver != EEPROM_LAYOUT_VERSION) {
      Serial.println("EEPROM Sifirlaniyor / Onariliyor...");
      radio_type = EEPROM.read(17);
      if (radio_type > 1) radio_type = 0;
      initialize_eeprom();
  } else if (EEPROM.read(1) != SW_MAJOR || EEPROM.read(2) != SW_MINOR) {
      Serial.println("Yazilim Guncellendi, EEPROM versiyon damgasi yenileniyor...");
      EEPROM.write(1, SW_MAJOR);
      EEPROM.write(2, SW_MINOR);
      EEPROM.commit();
  }

  bool needEepromCommit = false;
  radio_type = EEPROM.read(17); // UHF VHF Seçimi
  if (radio_type > 1) {
      radio_type = 0;
      EEPROM.write(17, 0);
      needEepromCommit = true;
  }

  eeprom_readAPRS();

  EEPROM.get(EEPROM_SPECIALFRQ_BLCKSTART, freqLimits);
  if (freqLimits.trx_min_25 < 5000 || freqLimits.trx_min_25 > 6000 ||
      freqLimits.trx_max_25 < 6500 || freqLimits.trx_max_25 > 7500 ||
      freqLimits.trx_min_25 >= freqLimits.trx_max_25 ||
      freqLimits.scn_min_25 < 5000 || freqLimits.scn_max_25 > 7500 ||
      freqLimits.scn_min_25 >= freqLimits.scn_max_25 ||
      freqLimits.aprs_25 < 5000 || freqLimits.aprs_25 > 7500 ||
      freqLimits.iss_25 < 5000 || freqLimits.iss_25 > 7500 ||
      freqLimits.vna_min_25 < 5000 || freqLimits.vna_max_25 > 7500) {
      Serial.println("[EEPROM] Bozuk frekans limitleri tespit edildi, varsayilana donuluyor.");
      freqLimits = freqLimits_t();
      EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART, freqLimits);
      needEepromCommit = true;
  }

  EEPROM.get(EEPROM_TOT, tot_values);
  if (tot_values.time_out_time < 15 || tot_values.time_out_time > 240 || tot_values.lock_time > 75) {
      Serial.println("[EEPROM] Bozuk TOT degerleri tespit edildi, 60sn/5sn olarak onarildi.");
      tot_values.time_out_time = 60;
      tot_values.lock_time = 5;
      EEPROM.put(EEPROM_TOT, tot_values);
      needEepromCommit = true;
  }

  EEPROM.get(EEPROM_CURRCHNL_BLCKSTART, current_ch);
  bool needChSave = false;
  uint32_t minValidF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_min_25 * 25UL) : 400000UL;
  uint32_t maxValidF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_max_25 * 25UL) : 468375UL;
  if (current_ch.frequency < minValidF || current_ch.frequency > maxValidF || (current_ch.frequency % 25 != 0)) {
      current_ch.frequency = (radio_type == 0) ? 145575UL : 433500UL;
      current_ch.shift = (radio_type == 0) ? 600 : 7600;
      current_ch.shift_dir = noSHIFT;
      needChSave = true;
  }
  if (current_ch.shift_dir < -1 || current_ch.shift_dir > 1) {
      current_ch.shift_dir = noSHIFT;
      needChSave = true;
  }
  if (current_ch.shift < 0 || current_ch.shift > 20000) {
      current_ch.shift = (radio_type == 0) ? 600 : 7600;
      needChSave = true;
  }
  if (current_ch.tone_enabled > 1) {
      current_ch.tone_enabled = CTCSS_OFF;
      needChSave = true;
  }
  if (current_ch.tone_pos >= TOTAL_TONES) {
      current_ch.tone_pos = 8; // 88.5 Hz varsayilan
      needChSave = true;
  }
  if (needChSave) {
      EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
      needEepromCommit = true;
  }

  numberToFrequency(current_ch.frequency, FRQ);
  strcpy(FRQ_old, FRQ);

  op_mode = EEPROM.read(EEPROM_OP_MODE_ADDR);
  if (op_mode > 1) {
      op_mode = MODE_VFO;
      EEPROM.write(EEPROM_OP_MODE_ADDR, op_mode);
      needEepromCommit = true;
  }
  
  current_memory_channel = EEPROM.read(EEPROM_CURR_MEMCH_ADDR);
  if (current_memory_channel < 1 || current_memory_channel > 50) current_memory_channel = 1;
  
  if (op_mode == MODE_MR) {
      char chStr[3];
      sprintf(chStr, "%02d", current_memory_channel);
      GetMemoryChannel(chStr); 
      
      // Eger acilista denk gelinen kanal bossa (NULL ise), siradaki dolu kanali bul
      if (strncmp(FRQ, "NULL", 4) == 0) {
          int starting_channel = current_memory_channel;
          do {
              current_memory_channel++;
              if (current_memory_channel > 50) current_memory_channel = 1;
              sprintf(chStr, "%02d", current_memory_channel);
              GetMemoryChannel(chStr);
          } while (strncmp(FRQ, "NULL", 4) == 0 && current_memory_channel != starting_channel);
          
          if (strncmp(FRQ, "NULL", 4) == 0) {
              // Hicbir hafiza kanali dolu degilse VFO moduna geri don!
              op_mode = MODE_VFO;
              EEPROM.write(EEPROM_OP_MODE_ADDR, MODE_VFO);
              EEPROM.get(EEPROM_CURRCHNL_BLCKSTART, current_ch);
              numberToFrequency(current_ch.frequency, FRQ);
              strcpy(FRQ_old, FRQ);
              needEepromCommit = true;
          } else if (current_memory_channel != starting_channel) {
              EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
              needEepromCommit = true;
          }
      }
  }

  if (needEepromCommit) {
      EEPROM.commit();
  }
 
  SetTone(current_ch.tone_enabled);

  //setRadioPower();  //Check power switch mode and turn adio on immediately
  //pinMode(POWER_ON_OFF, INPUT);
  //pinMode(POWER_ON_PIN, OUTPUT);
 
  memset(matrix, 0, 24);
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin(); 
  Wire.setClock(100000); // PCF8576 maximum is 100kHz
  // DIKKAT: Wire.begin() sonrasi pinMode(16/17, INPUT_PULLUP) CAGRISI YAPILMAMALIDIR!
  // RP2040'ta pinMode() pini GPIO_FUNC_I2C'den GPIO_FUNC_SIO'ya cevirerek I2C'yi bozar.
  // Sadece dahili pull-up'lari aktif tutmak icin gpio_pull_up() kullanilir:
  gpio_pull_up(16);
  gpio_pull_up(17);
  
  delay(100);  // Give some time to boot

  Serial.println("I2C Taramasi Basliyor...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C cihazi bulundu: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Bilinmeyen hata 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("Hicbir I2C cihazi bulunamadi!\n");
  else
    Serial.println("I2C Taramasi Tamamlandi.\n");

  // Ekran ve tus takimi arka plan isigini (0x21 Bit 7 = backlight = 128, Aktif-LOW) acilista hemen yak
  Led_Status = 240 - backlight;
  Wire.beginTransmission(PCF8574_KEYB_LED);
  Wire.write(Led_Status);
  Wire.endTransmission();

  // Init LCD
  InitLCD();
  Greetings();

  writeFRQToLcd(FRQ);
  
  // Keypad PCF8574 (0x20 ve 0x21) baslangic durumunu kur ve olasi takili INT'i temizle
  resetKeypadPCF();
  old_KeyVal = 1; //initial keypad read
  if (op_mode == MODE_MR) {
    validFRQ = true;
  } else {
    validFRQ = Calculate_Frequency(FRQ); // start with default frequency
  }
  write_FRQ(current_ch.frequency);

  // ACILIS TESHIS: PLL limitleri ve hedef kelimeler seri porta dokulur
  Serialprint("[BOOT] radio_type=%d validFRQ=%d freq=%lu shift_dir=%d shift=%d\r\n",
              radio_type, validFRQ, (unsigned long)current_ch.frequency,
              current_ch.shift_dir, current_ch.shift);
  Serialprint("[BOOT] limits(25): trx_min=%u trx_max=%u scn_min=%u scn_max=%u\r\n",
              freqLimits.trx_min_25, freqLimits.trx_max_25,
              freqLimits.scn_min_25, freqLimits.scn_max_25);

  scrTimer = TimeoutValue;  

//  if (pgm_read_byte (32700) != 188)
//    while (1) 
//      Alert_Tone(ERR_tone);

  // Start WebUI and WiFi at boot
#if WIFI_AT_BOOT
  initWebUI();
#else
  Serial.println("[WIFI] Acilista baslatilmadi (WIFI_AT_BOOT=0). Baslatmak icin 'W' gonderin.");
#endif

  // WiFi AP acilisi biter bitmez ekran isigini (240 - backlight = 0x70) acik tut ve INT hattini temizle
  Led_Status = 240 - backlight;
  resetKeypadPCF();
  old_KeyVal = 1;
  if (validFRQ) {
    SetPLLLock(current_ch.frequency);
  }

  PrintMenu();
}


void loop() {

  //setRadioPower(); //Check power switch and set radio power mode on or off
  
  //if (scrMODE==scrNORMAL) writeFRQToLcd(FRQ); //TODO: We should update the display only on proper display changes.. But this works...

  if (scrMODE == scrNORMAL && TRX_MODE != TX)
  {
    uint8_t hb = (millis() / 1000) % 2;
    static uint8_t last_hb = 2;
    if (hb != last_hb) {
      last_hb = hb;
      digitalWrite(LED_BUILTIN, hb);
    }
  }

  // Output data to Keyboard LEDs... SADECE Led_Status degistiginde I2C'ye yaz!
  Led_Status = 240;
  if ((CHANNEL_BUSY == 0) || (SQL_MODE == SQL_OFF)) Led_Status = Led_Status - green_led; // we are receiving
  if (CHANNEL_BUSY == 0) APRS_Counter = 0; // If channel is busy, dont transmit APRS
  if (TRX_MODE == TX) Led_Status = Led_Status - red_led;  // We are transmitting
  if (pttToggler) send_packet(_STATUS, (freqLimits.aprs_25 * 25)); // TODO: what if UHF
  if (APRS_Counter < 2000) Led_Status = Led_Status - backlight;

  static uint8_t old_Led_Status = 0xFF;
  if (Led_Status != old_Led_Status) {
    old_Led_Status = Led_Status;
    Wire.beginTransmission(PCF8574_KEYB_LED);
    Wire.write(Led_Status); 
    Wire.endTransmission();
  }

  // 5-Kademeli Dijital Squelch Filtresi ve Histerezis (Pir-pir / anlik parazit onleyici)
  static unsigned long lastSqlSampleMs = 0;
  static int16_t sqlScore = 0;
  static uint8_t filteredBusy = 1; // 1 = Kanal sessiz (Mute), 0 = Sinyal var (RX Acik)
  static const int16_t sqlOpenThresholds[6] = { 0, 0, 10, 25, 45, 80 }; // Kademe 1..5 acilma gecikmesi (ms)
  const int16_t sqlHoldMax = 180; // Sinyal acildiktan sonra 180ms tutma (pir-pir onleyici kapanma histerezisi)

  if (TRX_MODE == TX) {
    sqlScore = 0;
    filteredBusy = 1;
  } else if (SQL_MODE == SQL_OFF) {
    filteredBusy = 0;
  } else {
    unsigned long nowMs = millis();
    if (nowMs != lastSqlSampleMs) {
      uint16_t dt = (uint16_t)(nowMs - lastSqlSampleMs);
      if (dt > 20) dt = 20;
      lastSqlSampleMs = nowMs;

      uint8_t lvl = (sql_level >= 1 && sql_level <= 5) ? sql_level : 3;
      int16_t openThr = sqlOpenThresholds[lvl];

      if (digitalRead(SQL_ACTIVE) == 0) {
        sqlScore += dt;
        if (filteredBusy == 1 && sqlScore >= openThr) {
          filteredBusy = 0; // Squelch ACILDI
          sqlScore = openThr + sqlHoldMax;
        } else if (filteredBusy == 0 && sqlScore > openThr + sqlHoldMax) {
          sqlScore = openThr + sqlHoldMax;
        }
      } else {
        sqlScore -= dt;
        if (sqlScore <= 0) {
          sqlScore = 0;
          filteredBusy = 1; // Squelch KAPANDI
        }
      }
    }
  }

  CHANNEL_BUSY = filteredBusy;
  if (CHANNEL_BUSY == 0) digitalWrite(MUTE_PIN_1, LOW);
  else if (SQL_MODE == SQL_ON) digitalWrite(MUTE_PIN_1, HIGH); 
  else digitalWrite(MUTE_PIN_1, LOW);


  /*
  TOT Function
  Call Method.
  */
  TRX_MODE =  TXTimeOutTimer(digitalRead(PTT_INPUT_PIN)); //read PTT state
  if (TOT_FLAG == 1) {
    writeToLcd("TOTWARN ");
    delay(10);
    if (TOT_WARN_STATE == 1) Alert_Tone(ERR_tone);
    TOT_WARN_STATE = 0;
  }


//TRX_MODE = digitalRead(PTT_INPUT_PIN); //read PTT state
//  if (pttToggler && TRX_MODE == RX) TRX_MODE = TX; //if pttToggler set from serial, then mode is transmission
  if (TRX_MODE != LST_MODE) {
    LST_MODE = TRX_MODE;
    Serialprint("[TRX] %s  f=%lu  FRQ=%s\r\n", TRX_MODE == TX ? "TX" : "RX",
                (unsigned long)current_ch.frequency, FRQ);
    if (TRX_MODE == TX) {
       pttKey(true); // TAMSAT kartinda TX'i fiziksel olarak acmak icin GEREKLI!
       write_FRQ(current_ch.frequency);   // PLL'i TX frekansina programla
       SetTone(current_ch.tone_enabled);
       lastPllRefresh = millis();
    } else {
       pttKey(false); // Once PA'i kapat, SONRA PLL'i cevir
       write_FRQ(current_ch.frequency);
       SetTone(current_ch.tone_enabled);
    }
  }

  // TX sirasinda PLL'i periyodik tazele (yalnizca profil istiyorsa)
  if (TRX_MODE == TX && pllRefreshMs && (millis() - lastPllRefresh) >= pllRefreshMs) {
    lastPllRefresh = millis();
    SetPLLLock(current_ch.frequency);
  }

  //if (subMENU == menuRPT) writeToLcd(cstr);

  // APRS periyodik beacon zamanlayicisi (dakika bazinda, kanal bos ve RX modundayken)
  static unsigned long lastAprsMillis = 0;
  if (APRS_Timeout > 0 && TRX_MODE == RX && CHANNEL_BUSY != 0) {
    unsigned long intervalMs = (unsigned long)APRS_Timeout * 60000UL;
    if (millis() - lastAprsMillis >= intervalMs) {
      lastAprsMillis = millis();
      send_packet(_FIXPOS_STATUS, (uint32_t)freqLimits.aprs_25 * 25UL);
    }
  } else if (CHANNEL_BUSY == 0 || TRX_MODE == TX) {
    // Kanal mesgulken veya konusurken sayaci oteleyerek konusma ustune basmasini onle
    if (APRS_Timeout > 0 && (millis() - lastAprsMillis) >= ((unsigned long)APRS_Timeout * 60000UL)) {
      lastAprsMillis = millis() - ((unsigned long)APRS_Timeout * 60000UL) + 5000UL;
    }
  }

  //this is our interrupt pin... Move this to a proper interrupt rutine
  KeyVal = digitalRead(KeypadIntPin);

  static unsigned long keyPressStartMs = 0;

  //Long press on a key detection (1000 ms basili tutma) + PCF8574 INT takilma onleyici...
  if ((KeyVal == old_KeyVal) && (KeyVal == 0)) {
    if ((scrTimer > 0) && (millis() - keyPressStartMs >= 1000)) {
      scrTimer = 0;
    }
    // Eger INT pini 1200ms'den uzun suredir LOW kaldiysa ama gercekte basili bir satir yoksa, PCF8574'u kurtar
    if (millis() - keyPressStartMs >= 1200) {
      if (readRow() >= 5) {
        resetKeypadPCF();
        old_KeyVal = 1;
        scrTimer = TimeoutValue;
      } else {
        resetKeypadPCF();
        keyPressStartMs = millis();
      }
    }
  } else if (KeyVal == 1) {
    if (scrTimer == 0) { //key released and timeout occured
      if (pressedKEY == 'B') {
        writeToLcd("TONE     ");
        subMENU = menuTONE;
        scrMODE = scrMENU;
        delay(1000);
        write_TONEtoLCD(current_ch.tone_pos);
        old_ctcss_tone_pos = current_ch.tone_pos;
      } else if (pressedKEY == 'O') {
        writeToLcd("SCAN     ");
        delay(1000);
        startScan();
        scrMODE = scrNORMAL;
        subMENU = menuNONE;
      } else if (pressedKEY == 'S') {
        subMENU = menuSQL;
        scrMODE = scrMENU;
        char sqlBuf[10];
        sprintf(sqlBuf, "SQL   %d ", sql_level);
        writeToLcd(sqlBuf);
      }
      scrTimer = TimeoutValue; //Restart the timer
      numChar = 0; //if we were entering frq from keyboard
    }//scrTimer
  }//Keyval==0

  
  if (KeyVal != old_KeyVal) { 
    if (KeyVal == 0) {
        // ONCE tusu oku (Alert_Tone 100ms gecikme yaptigi icin once cagrilirsa kisa basista tus kacirilir!)
        int satir = readRow();
        int sutun = readColumn();
        if (satir >= 0 && satir < 5 && sutun >= 0 && sutun < 4) {
          pressedKEY = keymap[sutun][satir]; //LOOKUP for the pressedKEY
        } else {
          pressedKEY = 'X';
        }
        APRS_Counter = 0; //if a key is pressed, restart the APRS timer
        scrTimer = TimeoutValue; //Restart the timer
        keyPressStartMs = millis();
        if (pressedKEY != 'X') {
          Alert_Tone(OK_tone);
        }
    
    // KEYPAD LOGGING
    Serialprint("[KEYPAD] Satir: %d, Sutun: %d -> Basilan Tus: '%c'\r\n", satir, sutun, pressedKEY);

    if (pressedKEY != 'X') {

    if (scrMODE == scrMENU)  {
      // Serialprint("srcmode src menu \n\r  ");
        // Serialprint("src menu not X pressedKey:%d \n\r  ", pressedKEY);
        switch (pressedKEY) {
          case 'U':
            if ( subMENU == menuRPT)  { current_ch.shift += 25; write_SHIFTtoLCD(current_ch.shift); }  //TODO: check overflows... + or -
            if ( subMENU == menuTONE) { current_ch.tone_pos += 1; if (current_ch.tone_pos >= TOTAL_TONES - 1) current_ch.tone_pos = TOTAL_TONES - 1; write_TONEtoLCD(current_ch.tone_pos); EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);}
            if ( subMENU == menuSQL)  { if (sql_level < 5) sql_level++; SQL_MODE = SQL_ON; EEPROM.write(EEPROM_SQLMODE_ADDR, SQL_MODE); EEPROM.write(EEPROM_SQLLEVEL_ADDR, sql_level); char b[10]; sprintf(b, "SQL   %d ", sql_level); writeToLcd(b); }
            break;
          case 'D':
            if ( subMENU == menuRPT)  {current_ch.shift -= 25; write_SHIFTtoLCD(current_ch.shift); }
            if ( subMENU == menuTONE) { current_ch.tone_pos -= 1;  if (current_ch.tone_pos<=0) current_ch.tone_pos = 0 ; write_TONEtoLCD(current_ch.tone_pos); EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);}
            if ( subMENU == menuSQL)  { if (sql_level > 1) sql_level--; SQL_MODE = SQL_ON; EEPROM.write(EEPROM_SQLMODE_ADDR, SQL_MODE); EEPROM.write(EEPROM_SQLLEVEL_ADDR, sql_level); char b[10]; sprintf(b, "SQL   %d ", sql_level); writeToLcd(b); }
            break;
          case '#': //means CANCEL
            if (subMENU == menuRPT)  current_ch.shift = old_frqSHIFT;
            if (subMENU == menuTONE) current_ch.tone_pos = old_ctcss_tone_pos;
            //testing the lock reported by TA2GY .. memory recall after shift setup locks device
            numChar = 0;
            // Serialprint("set to X pressedKey:%d \n\r  ", pressedKEY);
            pressedKEY = 'X';
            scrMODE = scrNORMAL; //we are in menu or submenus.. return to normal display
            subMENU = menuNONE;
            writeFRQToLcd(FRQ);
          break; // '#'
          case '*': //means OK
            EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
            EEPROM.commit();
            //if ( subMENU == menuRPT)  write_SHIFTtoEE(frqSHIFT);
            //if ( subMENU == menuTONE) write_TONEtoEE(ctcss_tone_pos);
            //testing the lock reported by TA2GY .. memory recall after shift setup locks device
            numChar = 0;
            // Serialprint("set to X * pressedKey:%d \n\r  ", pressedKEY);
            pressedKEY = 'X';
            scrMODE = scrNORMAL; //we are in menu or submenus.. return to normal display
            subMENU = menuNONE;
            writeFRQToLcd(FRQ);
          break; // '*'
          default:
          break; 
        }//switch
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
            if (current_ch.shift_dir == noSHIFT) { 
              current_ch.shift_dir = plusSHIFT;
            } else if (current_ch.shift_dir == plusSHIFT) {
              current_ch.shift_dir = minusSHIFT;
            } else { 
              current_ch.shift_dir = noSHIFT;
            }
            EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
            EEPROM.commit();
            write_FRQ(current_ch.frequency);
          break; //'R'
          case 'B':
            if (current_ch.tone_enabled == CTCSS_OFF) {
              current_ch.tone_enabled = CTCSS_ON;
            } else {
              current_ch.tone_enabled = CTCSS_OFF;
            }
            EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);    
            EEPROM.commit();
            SetTone(current_ch.tone_enabled); //Change Tone Generation State
            write_FRQ(current_ch.frequency);
          break; // 'B'
          case 'S':
            if (SQL_MODE == SQL_OFF) {
              SQL_MODE = SQL_ON;
            } else {
              SQL_MODE = SQL_OFF;
            }
            EEPROM.write(EEPROM_SQLMODE_ADDR, (uint8_t)SQL_MODE);
            EEPROM.commit();
            write_FRQ(current_ch.frequency);
          break; //'S'
          case 'O':
             if (RF_POWER_STATE == HIGH_POWER) {
                RF_POWER_STATE = LOW_POWER;
             } else {
                RF_POWER_STATE = HIGH_POWER;
             }
             EEPROM.write(EEPROM_RFPOWER_ADDR, RF_POWER_STATE);
             EEPROM.commit();
//             SetRFPower(RF_POWER_STATE);           
             SetRFPower();           
             write_FRQ(current_ch.frequency);
          break; //'O'
          case 'U':
            if (op_mode == MODE_VFO) {
                numberToFrequency(current_ch.frequency+freq_step,FRQ);
                validFRQ = Calculate_Frequency(FRQ);  
                write_FRQ(current_ch.frequency); 
                strcpy(FRQ_old, FRQ);
                EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
                EEPROM.commit();
            } else {
                int starting_channel = current_memory_channel;
                char chStr[3];
                do {
                    current_memory_channel++;
                    if (current_memory_channel > 50) current_memory_channel = 1;
                    sprintf(chStr, "%02d", current_memory_channel);
                    GetMemoryChannel(chStr);
                } while (strncmp(FRQ, "NULL", 4) == 0 && current_memory_channel != starting_channel);
                
                EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
                EEPROM.commit();
                write_FRQ(current_ch.frequency); // For now just show freq, later show name
            }
          break; // 'U' 
          case 'D':
            if (op_mode == MODE_VFO) {
                numberToFrequency(current_ch.frequency-freq_step,FRQ);
                validFRQ = Calculate_Frequency(FRQ);  
                write_FRQ(current_ch.frequency);           
                strcpy(FRQ_old, FRQ);
                EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
                EEPROM.commit();
            } else {
                int starting_channel = current_memory_channel;
                char chStr[3];
                do {
                    current_memory_channel--;
                    if (current_memory_channel < 1) current_memory_channel = 50;
                    sprintf(chStr, "%02d", current_memory_channel);
                    GetMemoryChannel(chStr);
                } while (strncmp(FRQ, "NULL", 4) == 0 && current_memory_channel != starting_channel);
                
                EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
                EEPROM.commit();
                write_FRQ(current_ch.frequency);
            }
          break; // 'D'
          case 'M': //Reverse was originally here, but 'R' does it too.
            // Toggle mode VFO/MR
            if (op_mode == MODE_VFO) {
                op_mode = MODE_MR;
                Serialprint("[MODE] Hafiza (MR) Moduna Gecildi\r\n");
                char chStr[3];
                sprintf(chStr, "%02d", current_memory_channel);
                GetMemoryChannel(chStr); 
                
                // Eger gecis yapilan kanal bossa, siradaki dolu kanali bul
                if (strncmp(FRQ, "NULL", 4) == 0) {
                    int starting_channel = current_memory_channel;
                    do {
                        current_memory_channel++;
                        if (current_memory_channel > 50) current_memory_channel = 1;
                        sprintf(chStr, "%02d", current_memory_channel);
                        GetMemoryChannel(chStr);
                    } while (strncmp(FRQ, "NULL", 4) == 0 && current_memory_channel != starting_channel);
                    
                    if (strncmp(FRQ, "NULL", 4) == 0) {
                        // Hicbir hafiza kanali dolu degilse VFO modunda kal
                        Serialprint("[MODE] Dolu hafiza kanali yok, VFO modunda kalindi\r\n");
                        op_mode = MODE_VFO;
                        strcpy(FRQ, FRQ_old);
                        validFRQ = Calculate_Frequency(FRQ);
                    } else if (current_memory_channel != starting_channel) {
                        EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
                    }
                }
            } else {
                op_mode = MODE_VFO;
                Serialprint("[MODE] Serbest (VFO) Moduna Gecildi\r\n");
                strcpy(FRQ,FRQ_old);
                validFRQ = Calculate_Frequency(FRQ);
            }
            numChar = 0;
            EEPROM.write(EEPROM_OP_MODE_ADDR, op_mode);
            EEPROM.commit();
            write_FRQ(current_ch.frequency);
            writeFRQToLcd(FRQ);
          break; // 'M'
          case 'C':
            //VNA Vector Network analizor Subroutines
            TRX_MODE = TX; //needed for PLL locking to TX frequrency
            minSWR = 9999;
            lowestFRQ=0;
            highestFRQ=0;           
            //TODO: Store old values of variables (FRQ, SHIFT, etc)
            //TODO: Put transmitter on low power before operation
            //TODO: If any key pressed cancel VNA operation
            strcpy(FRQ_old,FRQ); //store old frequency for recall
            current_ch.shift_dir = noSHIFT; //get into SIMPLEX mode for caculations
//            SetRFPower(LOW_POWER);
            SetRFPower();
            long min_vna_freq;
            long max_vna_freq;
            int  stp_vna_freq;
            if (radio_type==0)
            {
              min_vna_freq = freqLimits.vna_min_25 * 25;
              max_vna_freq = freqLimits.vna_max_25 * 25;
              stp_vna_freq = 10;
            }
            else //TODO: different defaults for VHF and UHF
            {
              min_vna_freq = 43000;
              max_vna_freq = 45000;              
              stp_vna_freq = 10;
            }
            numberToFrequency(min_vna_freq,FRQ);
            validFRQ = Calculate_Frequency(FRQ);
            write_FRQ(current_ch.frequency);   // Once PLL, sonra PA
            delay(PLL_TX_SETTLE_MS);
            pttKey(true);
            Serialprint("\r\n#VNA#\t%l\t%l\r\n",min_vna_freq,max_vna_freq); //START
            for (long vna_freq=min_vna_freq; vna_freq < max_vna_freq; vna_freq += stp_vna_freq)
              {
                numberToFrequency(vna_freq,FRQ);
                validFRQ = Calculate_Frequency(FRQ);
                write_FRQ(current_ch.frequency);
                writeFRQToLcd(FRQ);
                Serialprint(">%d\t",vna_freq);
                delay(100);
                //pttKey(true);
                //delay(50);
                readRfPower(); //TODO: Move under a menu item
                //delay(25);
                //pttKey(false);

                if (readColumn() != 0) break; //a key is pressed
              }
              pttKey(false);
              Serialprint("@VNA@\t%d\t%d\r\n",min_vna_freq,max_vna_freq); //END
              //SetRFPower(RF_POWER_STATE);
              SetRFPower();
              TRX_MODE = RX; //PLL should lock to RX
              //strcpy(FRQ,FRQ_old);
              numberToFrequency((highestFRQ+lowestFRQ)/2,FRQ);
              validFRQ = Calculate_Frequency(FRQ);
              write_FRQ(current_ch.frequency);            
              writeFRQToLcd(FRQ);
              PrintMenu(); //print menu for user selection  
          break; // 'C'
          case '#':
            if (numChar == 2) {
                StoreFrequency(FRQ,FRQ_old); 
                numChar = 0;
                write_FRQ(current_ch.frequency);
            }
            else if (numChar == 4) {
                StoreSpecialFrequency(FRQ,FRQ_old);
                numChar = 0;
                write_FRQ(current_ch.frequency);
            }
            else {
                strcpy(FRQ,FRQ_old);
                validFRQ = Calculate_Frequency(FRQ);
                numChar = 0;
                write_FRQ(current_ch.frequency);
            }
          break; // '#'
          case '*':
            if (numChar == 2) GetMemoryChannel(FRQ); // User wanted to retrieve the memory channel from EEPRM
            else strcpy(FRQ,FRQ_old); // Otherwise user wanted to cancel the ongoing operation.. return to previous (old) frequency
            validFRQ = Calculate_Frequency(FRQ);
            numChar = 0;
            write_FRQ(current_ch.frequency);
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
              
              writeFRQToLcd(FRQ); // SHOW PARTIAL INPUT ON SCREEN
              
              if (numChar == 7) { //input finished 
                validFRQ = Calculate_Frequency(FRQ);
                numChar = 0;
                write_FRQ(current_ch.frequency);
                if (validFRQ) {
                  current_ch.shift_dir = noSHIFT; // Reset shift to Simplex when a new frequency is entered manually
                  EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch); // Save the new simplex mode
                  EEPROM.commit();
                  strcpy(FRQ_old,FRQ);
                } else {
                  strcpy(FRQ,FRQ_old); //"   .   ";
                  numChar = 0;
                  validFRQ = Calculate_Frequency(FRQ);
                  write_FRQ(current_ch.frequency);
                  //sound audible error here
                }
              } // numChar==7
            }//numchar<6
            
          break; 
        }//switch
      }//pressedKEY != 'X'
    }//scrMODE=scrNORMAL
    } // pressedKEY != 'X' GLOBAL
    
    } // KeyVal == 0
    
    // Put 8574 (0x20) into READ mode with 0xFF reference state (while 0x21 columns are HIGH)
    resetKeypadPCF();
    if (validFRQ) {
      SetPLLLock(current_ch.frequency);
    }
    
    //Toggle interupt PIN state holder
    old_KeyVal = KeyVal;
   } //KeyVal!=old_keyval

if (commandComplete) {
    //TODO: convert to case/switch
    if (commandString.charAt(0) == '\n') PrintMenu();
    //if (commandString.charAt(0) == 'Y') commandYardim(commandString.charAt(2));
    if (commandString.charAt(0) == 'C') commandRadioType(commandString.charAt(2));
    if (commandString.charAt(0) == 'A') commandStartupMSG();
    if (commandString.charAt(0) == 'T') commandAPRSTimeout();
    if (commandString.charAt(0) == 'M') commandAPRSMessage();    
    if (commandString.charAt(0) == 'S') commandAPRSmycall();    
    if (commandString.charAt(0) == 'H') commandMemoryDump();
    if (commandString.charAt(0) == 'K') commandDumpConfig();
    if (commandString.charAt(0) == 'C') commandMemoryChannel();
    if (commandString.charAt(0) == 'P') commandTogglePTT();
    if (commandString.charAt(0) == 'G') getGPSData();
    if (commandString.charAt(0) == 'R') getEEPROMData();
    if (commandString.charAt(0) == 'N') startScan();
    if (commandString.charAt(0) == 'X') softResetDevice();
    if (commandString.charAt(0) == 'f') commandFrequencyLowerLimit();
    if (commandString.charAt(0) == 'F') commandFrequencyUpperLimit();
    if (commandString.charAt(0) == 'q') commandScanLowerLimit();
    if (commandString.charAt(0) == 'Q') commandScanUpperLimit();
    if (commandString.charAt(0) == 'B') commandAprsFrequency();
    if (commandString.charAt(0) == 'I') commandISSFrequency();
    if (commandString.charAt(0) == 'W') { initWebUI(); }
    if (commandString.charAt(0) == 'Y') { pllSweepTest(); }
    if (commandString.charAt(0) == 'm') { pllNextMode(); }
    if (commandString.charAt(0) == 'p') {          // PTT otomatik testi
        Serialprint("\r\n[PTTTEST] basliyor. 145.575'e donuluyor.\r\n");
        current_ch.frequency = (radio_type == 0) ? 145575UL : 433500UL;
        current_ch.shift_dir = noSHIFT;
        numberToFrequency(current_ch.frequency, FRQ);
        validFRQ = Calculate_Frequency(FRQ);
        write_FRQ(current_ch.frequency);
        Serialprint("[PTTTEST] dinleme PLL yazildi. PTT_IN=%d\r\n", digitalRead(PTT_INPUT_PIN));
        delay(500);
        TRX_MODE = TX;
        write_FRQ(current_ch.frequency);
        delay(PLL_TX_SETTLE_MS);
        pttKey(true);
        Serialprint("[PTTTEST] PTT_OUT=HIGH, 3 saniye gonderim\r\n");
        for (int i = 0; i < 6; i++) {
            delay(500);
            Serialprint("[PTTTEST] t=%dms PTT_IN=%d SQL=%d\r\n",
                        (i + 1) * 500, digitalRead(PTT_INPUT_PIN), digitalRead(SQL_ACTIVE));
        }
        pttKey(false);
        TRX_MODE = RX;
        LST_MODE = RX;
        write_FRQ(current_ch.frequency);
        Serialprint("[PTTTEST] bitti, PTT_OUT=LOW\r\n");
    }
    if (commandString.charAt(0) == 't') {          // PTT polaritesi
        pttActiveLevel = (pttActiveLevel == HIGH) ? LOW : HIGH;
        Serialprint("[PTT] aktif seviye = %s (GP%d)\r\n",
                    pttActiveLevel == HIGH ? "HIGH" : "LOW", PTT_OUTPUT_PIN);
    }
    if (commandString.charAt(0) == 'r') {          // R sayaci / PD frekansi
        pllR = (pllR == 1024) ? 512 : 1024;
        Serialprint("[PLL] R=%u -> PD=%.1f kHz. Yeniden programlaniyor...\r\n",
                    pllR, 12800.0 / pllR);
        validFRQ = Calculate_Frequency(FRQ);
        write_FRQ(current_ch.frequency);
    }
    if (commandString.charAt(0) == 'z') {          // A=0 testi: 1 MHz tam katina programla
        current_ch.frequency = (radio_type == 0) ? 145000UL : 433000UL;
        current_ch.shift_dir = noSHIFT;
        numberToFrequency(current_ch.frequency, FRQ);
        validFRQ = Calculate_Frequency(FRQ);
        write_FRQ(current_ch.frequency);
        Serialprint("[TESTA0] %lu kHz -> TX N=%lu A=0, RX N=%lu A=0\r\n",
                    (unsigned long)current_ch.frequency,
                    (unsigned long)(current_ch.frequency / 1000),
                    (unsigned long)((current_ch.frequency + 45000UL) / 1000));
    }
    if (commandString.charAt(0) == 'J') { 
        freq_step = 25; Serialprint("Adim 25 KHz (Donanimsal Sabit)\r\n");
    }
    if (commandString.charAt(0) == 'v') {
        micTxMode = (micTxMode + 1) % 2;
        const char* modeNames[2] = {
            "0: Standart CTCSS PWM (%2 duty, PR1 pot ile uyumlu)",
            "1: DDS Sinus CTCSS PWM (256-ornek tablo)"
        };
        Serialprint("[CTCSS] Ton Uretim Modu = %s\r\n", modeNames[micTxMode]);
        SetTone(current_ch.tone_enabled);
    }
    
//   Serial.println("Gecersiz bir komut... tekrar deneyiniz...");
    commandString = "";
    commandComplete = false;
    PrintMenu();
    //Serialprint("\r\nSeciminiz>");
    //EEPROM.get(EEPROM_CURRCHNL_BLCKSTART, current_ch);
  }

//13:49:03$ fm TA7W-9 to TAMSAT-0 via ARISS-1 UI  PID=F0!3955.50N>3250.22E/ TAMSAT KIT - APRS TEST
//  send_packet(_STATUS);  
//  delay(tx_delay);
//  randomize(tx_delay, 10, 5000);
//  randomize(str_len, 10, 420);
  if (TRX_MODE != TX) {
    handleWebUI();
  }
  serialEvent();   // Pico'da loop'tan cagrilmazsa seri menu hic calismaz

  #ifdef RESETWATCHDOG
	//wdt_reset();
  #endif
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
  
  webLogBuffer += ptr;
  if(webLogBuffer.length() > 2000) {
    webLogBuffer = webLogBuffer.substring(webLogBuffer.length() - 2000);
  }
  #endif
}


void serialEvent() {
  // NOT: AVR'deki cli()/sei() kaldirildi. RP2040'ta I2C/USB servisleri kesmeye
  // bagli; kesmeleri kapatmak tus takimini kilitleyebiliyor. Gerek de yok.
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    commandString += inChar;
    if (inChar == '\n') {
      commandComplete = true;
    }
  }
}

/*
 * seri olarak okunan byte lari 0x09 veya 0x04 e kadar olanini geri dondurmek icin
 */
/*
void readParam(char *szParam, int iMaxLen) {
  uint8_t c;
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
      strcpy(Ayarlar.APRS_Message, szParam);

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






/*
 * This function will calculate CRC-16 CCITT for the FCS (Frame Check Sequence)
 * as required for the HDLC frame validity check.
 * 
 * Using 0x1021 as polynomial generator. The CRC registers are initialized with
 * 0xFFFF
 */








/*
 * This function will send one byte input and convert it
 * into AFSK signal one bit at a time LSB first.
 * 
 * The encode which used is NRZI (Non Return to Zero, Inverted)
 * bit 1 : transmitted as no change in tone
 * bit 0 : transmitted as change in tone
 */






/*
 * In this preliminary test, a packet is consists of FLAG(s) and PAYLOAD(s).
 * Standard APRS FLAG is 0x7e character sent over and over again as a packet
 * delimiter. In this example, 100 flags is used as the preamble and 3 flags as
 * the postamble.
 */


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

void(* resetFunc) (void) = 0;
void softResetDevice()
{
  rp2040.reboot();
}
//LCD FUNCTIONS


// Initialize the LCD


/* Physically send out the given data */






/* Scrolls a text over the LCD */



//Print Greeting messages to LCD




//HELPER FUNCTIONS
//MC145158 programming routines
//R_counter=1024 (12.8MHz / 12.5kHz ref)
//N_counter=190, A_counter=46 (for 145.575 RX)

/* Ham N/A yazimi. Sweep testi de bunu kullanir. */
void pllWriteNA(int N_Counter, int A_Counter) {
  digitalWrite(PLL_SEC, LOW); //SELECT PLL for SPI BUS (keep permanently LOW)
  send_SPIBit(pllR, 14);
  send_SPIBit(1, 1);          // R kaydi
  send_SPIEnable();
  send_SPIBit(N_Counter, 10);
  send_SPIBit(A_Counter, 7);
  send_SPIBit(0, 1);          // A/N kaydi
  send_SPIEnable();
  // NOT: PLL_SEC HIGH yapilmaz! K3 kartindaki TD9 (74HC32, 5V CMOS) OR kapilarina
  // 3.3V HIGH vermek (VIH_min=3.5V) kapiyi esik bolgesinde birakir ve ENABLE sonrasi
  // sahte CLOCK darbesi uretir. TD10 kullanilmadigi icin PLL_SEC surekli LOW kalmalidir.
}

/* PLL kelimelerini yazar. R ve N/A hedef VCO frekansindan.
   PD frekansi = 12800 kHz / pllR.  R=1024 -> 12.5 kHz, R=512 -> 25 kHz.
   Dikkat: N ve A, PD frekansina gore hesaplanir; R degisince ikisi de degisir. */
void pllWriteWords(uint32_t f) {
  uint32_t stepKhz10 = 12800UL * 10UL / pllR;        // 12.5 -> 125, 25 -> 250 (0.1 kHz birimi)
  uint32_t f10 = f * 10UL;                            // hedef VCO, 0.1 kHz birimi
  int N_Counter = (int)(f10 / stepKhz10 / 80);
  int A_Counter = (int)(f10 / stepKhz10) - (80 * N_Counter);
  pllWriteNA(N_Counter, A_Counter);

#if defined(PLL_DEBUG)
  Serialprint("[PLL] vco=%lu R=%u N=%d A=%d ref=%lu\r\n",
              (unsigned long)f, pllR, N_Counter, A_Counter,
              (unsigned long)((N_Counter * 80 + A_Counter) * stepKhz10 / 10));
#endif
}

void send_SPIBit(int Counter, uint8_t length) {
  for (int i=length-1;i>=0;i--) {
    digitalWrite(pll_data_pin, bitRead(Counter,i));
    delayMicroseconds(pllBitDelayUs);
    digitalWrite(pll_clk_pin,HIGH);    // Bring pin CLOCK high
    delayMicroseconds(pllBitDelayUs);
    digitalWrite(pll_clk_pin,LOW);     // Then back low
    delayMicroseconds(pllBitDelayUs);
    // NOT: Arduino TAMSAT'taki Wire.beginTransmission(0x21) blogu KALDIRILDI.
    // Sebep: Pico PCB'de GP17 (I2C SCL) ile GP9 (PLL CLK) arasinda
    // elektromanyetik kuplaj var. I2C saat darbeleri PLL verisini bozuyordu.
    // Arduino'da sorun yok cunku I2C ve SPI farkli portlarda (Port C vs Port B).
  }
  digitalWrite(pll_data_pin, LOW); // hat sonunda veri hattini serbest birak
}

void send_SPIEnable() {
  digitalWrite(pll_ena_pin, HIGH);   // Bring ENABLE high
  delayMicroseconds(pllBitDelayUs);
  digitalWrite(pll_ena_pin, LOW);    // Then back low
  delayMicroseconds(pllBitDelayUs);
}

/* Yazim profilini uygula. Modlar main.h'de listeli. */
void pllApplyMode(uint8_t m) {
  if (m >= PLL_MODE_COUNT) m = 0;
  pllMode = m;
  switch (m) {
    case 0: pllBitDelayUs = 1000; pllDoubleWrite = false; pllRefreshMs = 0;   break;
    case 1: pllBitDelayUs = 100;  pllDoubleWrite = false; pllRefreshMs = 0;   break;
    case 2: pllBitDelayUs = 10;   pllDoubleWrite = false; pllRefreshMs = 0;   break;
    case 3: pllBitDelayUs = 10;   pllDoubleWrite = true;  pllRefreshMs = 200; break;
    case 4: pllBitDelayUs = 1000; pllDoubleWrite = true;  pllRefreshMs = 200; break;
  }
  Serialprint("[PLLMODE] %d: bit=%uus cift=%d tazeleme=%ums\r\n",
              pllMode, pllBitDelayUs, pllDoubleWrite ? 1 : 0, pllRefreshMs);
}

void pllNextMode() {
  pllApplyMode((pllMode + 1) % PLL_MODE_COUNT);
  validFRQ = Calculate_Frequency(FRQ);
  write_FRQ(current_ch.frequency);   // hemen uygula ve dinle
}

#include "hardware/pwm.h"
#include "hardware/timer.h"

// Sinüs Tablosu (8-bit)
#define SINE_TABLE_SIZE 256
const uint8_t sine_wave[SINE_TABLE_SIZE] = {
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173,
  176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 236, 237, 239, 240, 242, 243, 245,
  246, 247, 248, 249, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 255,
  255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 249, 248, 247, 246, 245,
  243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 220, 218, 215,
  213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 188, 185, 182, 179, 176, 173,
  170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131, 128, 125,
  122, 119, 116, 113, 110, 107, 104, 101, 98, 95, 92, 89, 86, 83, 80, 77,
  74, 71, 68, 65, 62, 60, 57, 54, 52, 49, 47, 44, 42, 39, 37, 35,
  32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 13, 11, 10, 8, 7, 5,
  4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 3,
  4, 5, 6, 7, 8, 10, 11, 13, 14, 16, 18, 20, 22, 24, 26, 28,
  30, 32, 35, 37, 39, 42, 44, 47, 49, 52, 54, 57, 60, 62, 65, 68,
  71, 74, 77, 80, 83, 86, 89, 92, 95, 98, 101, 104, 107, 110, 113, 116,
  119, 122, 125, 127, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128
};

// PWM Değişkenleri
uint slice_num;
volatile int phase_acc = 0;
struct repeating_timer tone_timer;
bool timer_active = false;

// Timer Callback (Kesme Fonksiyonu)
bool timer_callback(struct repeating_timer *t) {
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MIC_PIN), sine_wave[phase_acc]);
    phase_acc = (phase_acc + 1) % SINE_TABLE_SIZE;
    return true;
}

void start_tone(float frequency) {
    if (timer_active) return;
    float interval_us = 1000000.0f / (frequency * SINE_TABLE_SIZE);
    
    gpio_set_function(MIC_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(MIC_PIN);
    pwm_set_wrap(slice_num, 255);
    pwm_set_enabled(slice_num, true);
    
    add_repeating_timer_us(-interval_us, timer_callback, NULL, &tone_timer);
    timer_active = true;
}

void stop_tone() {
    if (timer_active) {
        cancel_repeating_timer(&tone_timer);
        timer_active = false;
    }
    // PWM'i pinden ayir ve pini TAMAMEN serbest birak.
    // DIKKAT: gpio_set_dir(IN) tek basina yetmez. Onceki digitalWrite(LOW) latch'i
    // LOW'ta kalir; RP2040'ta input + latch LOW = dahili PULL-DOWN aktif olur ve
    // mikrofon hattini GND'ye ceker -> TX sesi kisilir. Arduino'da pinMode(INPUT)
    // latch'i onemsemedigi icin bu sorun yoktu.
    gpio_set_function(MIC_PIN, GPIO_FUNC_SIO);
    gpio_put(MIC_PIN, 1);                    // latch HIGH: pull-down olusmasin
    gpio_set_dir(MIC_PIN, GPIO_IN);
    gpio_set_pulls(MIC_PIN, false, false);   // ne pull-up ne pull-down: gercek high-Z
}

void SetTone(int toneSTATE) {
  noTone(ALERT_PIN);
  if (timer_active) {
      cancel_repeating_timer(&tone_timer);
      timer_active = false;
  }
  pwm_set_enabled(pwm_gpio_to_slice_num(MIC_PIN), false);

  if (TRX_MODE == RX) {
      gpio_init(MIC_PIN);
      gpio_set_dir(MIC_PIN, GPIO_OUT);
      gpio_put(MIC_PIN, 0); // Mute mic during RX to prevent muffled speaker audio
  } else {
      // TRX_MODE == TX
      if (toneSTATE == CTCSS_ON) {
          float f = (current_ch.tone_pos >= 0 && current_ch.tone_pos < TOTAL_TONES)
                    ? ctcss_tone_list[current_ch.tone_pos] : 88.5f;
          if (micTxMode == 1) {
              // Alternatif: 256-ornek DDS Sinus PWM
              start_tone(f);
          } else {
              // Varsayilan (micTxMode == 0): %2 duty CTCSS PWM (mikrofon sesini bastirmaz, PR1 ile ayarlanir)
              analogWriteFreq(f);
              analogWrite(MIC_PIN, 5);
          }
      } else {
          // CTCSS_OFF: GP10 (MIC_PIN) tamamen serbest / High-Z (GPIO_FUNC_NULL, IE=0, OD=1, pull=0)
          gpio_init(MIC_PIN);
          gpio_put(MIC_PIN, 1);
          gpio_set_dir(MIC_PIN, GPIO_IN);
          gpio_set_pulls(MIC_PIN, false, false);
          gpio_deinit(MIC_PIN);
      }
  }
}







