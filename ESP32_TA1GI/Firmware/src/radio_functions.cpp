#include "declarations.h"

// YENİ: Timer kesmesi her çalıştığında bu fonksiyon çağrılır
// DEĞİŞİKLİK: Tonun genliğini (ses seviyesini) ayarlamak için güncelleme yapıldı.
void IRAM_ATTR onDacTimer() {
  // Orijinal sinüs değerini al (0-255)
  uint8_t original_value = sine_wave[phase_accumulator];
  
  // Değeri, merkez olan 128'e göre -128 ile +127 arasına çek
  int16_t centered_value = original_value - 128;
  
  // Genlik faktörü ile değeri düşür (zayıflat)
  // int16_t attenuated_value = (int16_t)(centered_value * tone_amplitude_factor); // RESET SORUNU GİDERME: Yavaş olan float işlemi kaldırıldı.
  
  // RESET SORUNU GİDERME: Hızlı olan tam sayı (integer) matematiği kullanılıyor.
  int16_t attenuated_value = (centered_value * tone_amplitude_numerator) / 100;

  // Zayıflatılmış değeri tekrar 0-255 aralığına taşı
  uint8_t final_value = attenuated_value + 128;

  // ISR-GÜVENLİ: dacWrite() yerine doğrudan register yazma (GPIO26 = DAC2)
  SET_PERI_REG_BITS(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_DAC, final_value, RTC_IO_PDAC2_DAC_S);
  
  phase_accumulator = (phase_accumulator + 1) % SINE_TABLE_SIZE; // Tablodaki konumu bir artır
}

// YENİ: Belirtilen frekansta sinüs tonu üretimini başlatır
// DEĞİŞİKLİK: PTT kapalıyken DC voltaj oluşmasını önlemek için DAC kanalı etkinleştiriliyor.
void startDacTone(float frequency) {
  // DAC2 CW (Cosine Wave) üretecini kapat — aksi hâlde donanım, yazılımdan gelen değerleri ezer
  CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
  CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);

  // DAC2 (GPIO26) doğrudan register ile aktifleştir
  SET_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_MUX_SEL);
  SET_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC | RTC_IO_PDAC2_DAC_XPD_FORCE);

  uint64_t timer_frequency = (uint64_t)(frequency * SINE_TABLE_SIZE); // Gerekli timer frekansını hesapla
  uint64_t alarm_val = 1000000 / timer_frequency;

  timerWrite(dac_timer, 0); // Sayacı sıfırla
  timerAlarm(dac_timer, alarm_val, true, 0); // Timer'ı ayarla
  timerStart(dac_timer); // KRİTİK: timerStop() sonrası timer durdu, timerAlarm() tek başına yeniden başlatmıyor!
}

// YENİ: Ton üretimini durdurur
// DEĞİŞİKLİK: PTT kapalıyken DC voltaj oluşmasını önlemek için DAC kanalı tamamen kapatılıyor.
void stopDacTone() {
  timerStop(dac_timer); // Timer'ı durdur

  // DC voltajı önlemek için DAC'ı orta noktaya çek (register ile)
  SET_PERI_REG_BITS(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_DAC, 128, RTC_IO_PDAC2_DAC_S);
  // DAC gücünü kapat - pini yüksek empedans durumuna getirir
  CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC | RTC_IO_PDAC2_DAC_XPD_FORCE);
}


//MC145158 programming routines
void send_SPIBit(int Counter, byte length) {
  for (int i=length-1;i>=0;i--) {
    byte data=bitRead(Counter,i);
    digitalWrite(pll_data_pin, data);
    delayMicroseconds(10); // Gecikme eklendi
    digitalWrite(pll_clk_pin,HIGH);
    delayMicroseconds(10); // Gecikme eklendi
    digitalWrite(pll_clk_pin,LOW);
  }
}

void send_SPIEnable() {
  digitalWrite(pll_ena_pin, HIGH);
  delayMicroseconds(10); // Gecikme eklendi
  digitalWrite(pll_ena_pin, LOW);
  delayMicroseconds(10); // Gecikme eklendi
}

// GÜNCELLENDİ: SetTone fonksiyonu artık DAC yöntemini kullanıyor
void SetTone(int toneSTATE) {
  if (toneSTATE == CTCSS_ON && TRX_MODE == TX) {
    startDacTone(ctcss_tone_list[current_ch.tone_pos]);
  } else {
    stopDacTone();
  }
}


void Alert_Tone(int ToneType)
{
  if (TRX_MODE == TX)  return; //If we are transmitting, do not play tones, because tone pin might be busy with CTCSS generation
  
  // Önce DAC tonunu durdur (varsa)
  stopDacTone();

  if (ToneType == OK_tone)   {ledcWriteTone(ALERT_PIN,1000); delay(ALERT_MODE);}
  if (ToneType == ERR_tone)  {ledcWriteTone(ALERT_PIN,400); delay(ALERT_MODE*2);}
  if (ToneType == SUCC_tone) {ledcWriteTone(ALERT_PIN,600);delay(ALERT_MODE+150);ledcWriteTone(ALERT_PIN,1000);delay(ALERT_MODE);}
  
  ledcWriteTone(ALERT_PIN,0);      // Uyarı tonunu kapat
  
  // CTCSS tonunu (gerekiyorsa) yeniden başlat
  SetTone(current_ch.tone_enabled);
}

void SetPLLLock(uint32_t Frequency)
{
  int R_Counter =0;
  int N_Counter = 0;
  int A_Counter = 0;
  uint32_t freqToSet = Frequency;

  if(radio_type==0) //VHF
  {
    if (TRX_MODE == RX) freqToSet += 45000L;
    else freqToSet += (current_ch.shift_dir * current_ch.shift) ; // Add/remove transmission shift
  }
  else //UHF
  {
    if (TRX_MODE == RX) freqToSet -= 45000L;
    else freqToSet += (current_ch.shift_dir * current_ch.shift) ; // Add/remove transmission shift
  }
     R_Counter = 12800 / 12.5;  //12.8Mhz reference clock, 12.5Khz step
     N_Counter = freqToSet / 12.5 / 80 ; //prescaler = 80
     A_Counter = (freqToSet / 12.5) - (80 * N_Counter);


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


void write_FRQ(uint32_t Frequency) {
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

  if (validFRQ) {
    if(radio_type==0) //VHF
    {
      if ((Frequency < 174000L) & (Frequency >= 164000L))  { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, LOW);  }
      else if ((Frequency < 164000L) & (Frequency >= 154000L))  { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, HIGH); }
      else if ((Frequency < 154000L) & (Frequency >= 146000L))  { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, LOW);  }
      else if ((Frequency < 146000L) & (Frequency >= 134000L))  { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, HIGH); }
    }
    else if(radio_type==1) //UHF
    {
       if ((Frequency < 470000L) & (Frequency >= 452000L)) {digitalWrite(BAND_SELECT_0, LOW); digitalWrite(BAND_SELECT_1, LOW); }
       else if ((Frequency < 452000L) & (Frequency >= 430000L)) {digitalWrite(BAND_SELECT_0, HIGH);digitalWrite(BAND_SELECT_1, HIGH);}
    }
    SetPLLLock(Frequency);
  } //validFRQ
}

void SetRFPower() {
    digitalWrite(RF_POWER_PIN, RF_POWER_STATE);
}

void readRfPower()
{
   int refPower = 0;
   int fwdPower = 0;

   fwdPower = analogRead(FWD_POWER_PIN);
   refPower = analogRead(REF_POWER_PIN);
   int Pfark   = fwdPower - refPower;
   float swr = Pfark;
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
