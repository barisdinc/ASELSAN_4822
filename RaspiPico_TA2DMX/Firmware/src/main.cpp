#include "main.h"
#include "WebUI.h"
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

#define Streamprint(stream, format, ...) StreamPrint_progmem(stream, PSTR(format), ##__VA_ARGS__)

void Alert_Tone(int ToneType)
{
  if (TRX_MODE == TX) return;
  noTone(MIC_PIN);
  pinMode(MIC_PIN, OUTPUT);
  digitalWrite(MIC_PIN, LOW);
  if (ToneType == OK_tone)   { tone(ALERT_PIN, 1000, ALERT_MODE); Serialprint(" OK\r\n"); }
  if (ToneType == ERR_tone)  { tone(ALERT_PIN, 400, ALERT_MODE * 2); }
  if (ToneType == SUCC_tone) { tone(ALERT_PIN, 600, ALERT_MODE); delay(200); tone(ALERT_PIN, 1000, ALERT_MODE); }
  delay(ALERT_MODE);
  SetTone(current_ch.tone_enabled);
}

boolean Calculate_Frequency(char mFRQ[9])
{
  uint32_t calc_freq = 0;
  if (mFRQ[0] >= '0' && mFRQ[0] <= '9') calc_freq += (mFRQ[0] - 48) * 100000L;
  if (mFRQ[1] >= '0' && mFRQ[1] <= '9') calc_freq += (mFRQ[1] - 48) * 10000L;
  if (mFRQ[2] >= '0' && mFRQ[2] <= '9') calc_freq += (mFRQ[2] - 48) * 1000L;
  if (mFRQ[4] >= '0' && mFRQ[4] <= '9') calc_freq += (mFRQ[4] - 48) * 100L;
  if (mFRQ[5] >= '0' && mFRQ[5] <= '9') calc_freq += (mFRQ[5] - 48) * 10L;
  if (mFRQ[6] >= '0' && mFRQ[6] <= '9') calc_freq += (mFRQ[6] - 48) * 1L;

  if (radio_type == 0 && (calc_freq >= (freqLimits.trx_min_25 * 25)) && (calc_freq <= (freqLimits.trx_max_25 * 25))) {
    current_ch.frequency = calc_freq;
    return true;
  } else if (radio_type == 1 && (calc_freq >= 428150UL) && (calc_freq <= 450000UL)) {
    current_ch.frequency = calc_freq;
    return true;
  } else {
    Alert_Tone(ERR_tone);
    return false;
  }
}

void SetPLLLock(uint32_t Frequency)
{
  uint32_t f = Frequency;
  if (TRX_MODE == RX) f = (radio_type == 0) ? (f + 45000L) : (f - 45000L);
  else                f = (uint32_t)((int32_t)f + (int32_t)current_ch.shift_dir * (int32_t)current_ch.shift);

  pllWriteWords(f);
  if (pllDoubleWrite) {
    delay(1);
    pllWriteWords(f);
  }
}

void write_FRQ(uint32_t Frequency)
{
  writeFRQToLcd(FRQ);
  if (validFRQ) {
    if (radio_type == 0) {
      if ((Frequency < 174000L) & (Frequency >= 164000L)) { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, LOW);  }
      if ((Frequency < 164000L) & (Frequency >= 156000L)) { digitalWrite(BAND_SELECT_0, LOW);  digitalWrite(BAND_SELECT_1, HIGH); }
      if ((Frequency < 156000L) & (Frequency >= 146000L)) { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, LOW);  }
      if ((Frequency < 146000L) & (Frequency >= 134000L)) { digitalWrite(BAND_SELECT_0, HIGH); digitalWrite(BAND_SELECT_1, HIGH); }
    } else if (radio_type == 1) {
      uint32_t bandFreq = Frequency;
      if (TRX_MODE == TX) {
        bandFreq = (uint32_t)((int32_t)Frequency + (int32_t)current_ch.shift_dir * (int32_t)current_ch.shift);
      }
      if (uhfBandOverride >= 0 && uhfBandOverride <= 3) {
        digitalWrite(BAND_SELECT_0, (uhfBandOverride & 0x02) ? HIGH : LOW);
        digitalWrite(BAND_SELECT_1, (uhfBandOverride & 0x01) ? HIGH : LOW);
      } else {
        digitalWrite(BAND_SELECT_0, HIGH);
        digitalWrite(BAND_SELECT_1, (bandFreq <= 450000L) ? HIGH : LOW);
      }
    }
    SetPLLLock(Frequency);
  } else {
    Serialprint("[PLL] ATLANDI! validFRQ=0, freq=%lu\r\n", (unsigned long)Frequency);
  }
}

void SetRFPower()
{
  digitalWrite(RF_POWER_PIN, RF_POWER_STATE);
}

void readRfPower()
{
  int fwdPower = analogRead(FWD_POWER_PIN);
  int refPower = analogRead(REF_POWER_PIN);
  int Pfark = fwdPower - refPower;
  float swr = Pfark;
  Serial.print("\t");
  Serial.print(fwdPower);
  Serial.print("\t");
  Serial.print(refPower);
  Serial.print("\t");
  Serial.println(swr);
  if (swr < minSWR) {
    minSWR = swr;
    lowestFRQ = current_ch.frequency;
    highestFRQ = current_ch.frequency;
  } else if (swr == minSWR) {
    highestFRQ = current_ch.frequency;
  }
}

int TOT_FLAG = 0;
int TOT_WARN_STATE = 1;

void changeTotSettings(uint8_t timeout, uint8_t lock)
{
  tot_t tot_values_new;
  tot_values_new.time_out_time = timeout;
  tot_values_new.lock_time = lock;
  EEPROM.put(EEPROM_TOT, tot_values_new);
  tot_values = tot_values_new;
}

uint8_t TXTimeOutTimer(uint8_t PTT_PIN_STATE)
{
  unsigned long currentMillis = millis();
  if (PTT_PIN_STATE == HIGH) {
    if (currentMillis - prevTotMillis >= tot_values.time_out_time * 1000) {
      if (currentMillis - prevTotMillis >= tot_values.time_out_time * 1000 + tot_values.lock_time * 1000) {
        prevTotMillis = currentMillis;
      }
      TOT_FLAG = 1;
      return LOW;
    }
    TOT_FLAG = 0;
    TOT_WARN_STATE = 1;
    return HIGH;
  } else {
    prevTotMillis = currentMillis;
    TOT_FLAG = 0;
    TOT_WARN_STATE = 1;
    return LOW;
  }
}

int readColumn()
{
  Wire.beginTransmission(0x20);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x21);
  Wire.write((uint8_t)((Led_Status & 0xF0) | 0x0F));
  Wire.endTransmission();

  Wire.requestFrom(0x21, 1);
  int c = Wire.read();

  Wire.beginTransmission(0x20);
  Wire.write((uint8_t)0xFF);
  Wire.endTransmission();
  Wire.beginTransmission(0x21);
  Wire.write(Led_Status);
  Wire.endTransmission();

  c = (~c) & 0x0F;
  int sutun = 0;
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

  Wire.requestFrom(0x20, 1);
  int r = Wire.read();
  r = 255 - (r | 0x03);
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
  unsigned long scan_min = (radio_type == 0) ? ((unsigned long)freqLimits.scn_min_25 * 25UL) : 428150UL;
  unsigned long scan_max = (radio_type == 0) ? ((unsigned long)freqLimits.scn_max_25 * 25UL) : 450000UL;
  unsigned long scan_frequency = scan_min;
  int8_t frq_step = 25;

  while (1) {
    scan_frequency += frq_step;
    if (scan_frequency > scan_max) scan_frequency = scan_min;
    else if (scan_frequency < scan_min) scan_frequency = scan_max;

    numberToFrequency(scan_frequency, FRQ);
    validFRQ = Calculate_Frequency(FRQ);
    write_FRQ(current_ch.frequency);
    writeFRQToLcd(FRQ);
    delay(100);
    CHANNEL_BUSY = digitalRead(SQL_ACTIVE);
    if (CHANNEL_BUSY == 0) break;
    if (readColumn() != 0) break;
  }
}

void resetKeypadPCF()
{
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

void setup()
{
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();
  Wire.setTimeout(10);
  EEPROM.begin(1024);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(SQL_ACTIVE, INPUT);
  pinMode(MIC_PIN, OUTPUT);
  digitalWrite(MIC_PIN, LOW);

  pinMode(RF_POWER_PIN, OUTPUT);
  digitalWrite(RF_POWER_PIN, RF_POWER_STATE);

  pinMode(MUTE_PIN_1, OUTPUT);
  digitalWrite(MUTE_PIN_1, HIGH);

  pinMode(BAND_SELECT_0, OUTPUT);
  pinMode(BAND_SELECT_1, OUTPUT);
  gpio_set_drive_strength(BAND_SELECT_0, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(BAND_SELECT_1, GPIO_DRIVE_STRENGTH_12MA);

  pinMode(PTT_INPUT_PIN, INPUT_PULLUP);
  pinMode(PTT_OUTPUT_PIN, OUTPUT);
  pinMode(TNC_PTT_PIN, OUTPUT);
  digitalWrite(TNC_PTT_PIN, LOW);
  gpio_set_drive_strength(PTT_OUTPUT_PIN, GPIO_DRIVE_STRENGTH_12MA);
  pttKey(false);

  // K3 TD10 (68HC68P1) CE pini LOW tutularak SPI ve DEV_COMP hatti serbest birakilir
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);

  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);

  // GP26 (ADC0) SP ses girisi olarak hazirlanir (R2/R5 1.65V orta noktasini ezmemek icin pull-up/down kapatilir)
  adc_init();
  adc_gpio_init(AUDIO_RX_ADC_PIN);
  gpio_disable_pulls(AUDIO_RX_ADC_PIN);
  adc_select_input(0);
  hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);

  pinMode(FWD_POWER_PIN, INPUT);
  pinMode(REF_POWER_PIN, INPUT);
  pinMode(KeypadIntPin, INPUT_PULLUP);

  pinMode(pll_clk_pin, OUTPUT);
  pinMode(pll_data_pin, OUTPUT);
  pinMode(pll_ena_pin, OUTPUT);
  pinMode(PLL_SEC, OUTPUT);

  gpio_set_drive_strength(pll_clk_pin,  GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(pll_data_pin, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(pll_ena_pin,  GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(PLL_SEC,      GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_pulls(pll_clk_pin,  false, false);
  gpio_set_pulls(pll_data_pin, false, false);
  gpio_set_pulls(pll_ena_pin,  false, false);
  gpio_set_pulls(PLL_SEC,      false, false);

  digitalWrite(pll_clk_pin, LOW);
  digitalWrite(pll_data_pin, LOW);
  digitalWrite(pll_ena_pin, LOW);
  digitalWrite(PLL_SEC, LOW);

  pllApplyMode(1);
  delay(100);

  Serial.begin(115200);
  delay(3000);
  commandString.reserve(200);

  uint8_t eeprom_state = EEPROM.read(0);
  uint8_t layout_ver = EEPROM.read(EEPROM_LAYOUT_VER_ADDR);

  if (eeprom_state != 128 || layout_ver != EEPROM_LAYOUT_VERSION) {
    Serial.println("EEPROM Sifirlaniyor / Onariliyor...");
    radio_type = EEPROM.read(17);
    if (radio_type > 1) radio_type = 0;
    initialize_eeprom();
  } else if (EEPROM.read(1) != SW_MAJOR || EEPROM.read(2) != SW_MINOR) {
    EEPROM.write(1, SW_MAJOR);
    EEPROM.write(2, SW_MINOR);
    EEPROM.commit();
  }

  bool needEepromCommit = false;
  radio_type = EEPROM.read(17);
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
    freqLimits = freqLimits_t();
    EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART, freqLimits);
    needEepromCommit = true;
  }

  EEPROM.get(EEPROM_TOT, tot_values);
  if (tot_values.time_out_time < 15 || tot_values.time_out_time > 240 || tot_values.lock_time > 75) {
    tot_values.time_out_time = 60;
    tot_values.lock_time = 5;
    EEPROM.put(EEPROM_TOT, tot_values);
    needEepromCommit = true;
  }

  EEPROM.get(EEPROM_CURRCHNL_BLCKSTART, current_ch);
  bool needChSave = false;
  uint32_t minValidF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_min_25 * 25UL) : 428150UL;
  uint32_t maxValidF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_max_25 * 25UL) : 450000UL;
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
    current_ch.tone_pos = 8;
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

    if (strncmp(FRQ, "NULL", 4) == 0) {
      int starting_channel = current_memory_channel;
      do {
        current_memory_channel++;
        if (current_memory_channel > 50) current_memory_channel = 1;
        sprintf(chStr, "%02d", current_memory_channel);
        GetMemoryChannel(chStr);
      } while (strncmp(FRQ, "NULL", 4) == 0 && current_memory_channel != starting_channel);

      if (strncmp(FRQ, "NULL", 4) == 0) {
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

  memset(matrix, 0, 24);
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();
  Wire.setClock(100000);
  gpio_pull_up(16);
  gpio_pull_up(17);
  delay(100);

  Led_Status = 240 - backlight;
  Wire.beginTransmission(PCF8574_KEYB_LED);
  Wire.write(Led_Status);
  Wire.endTransmission();

  InitLCD();
  Greetings();
  writeFRQToLcd(FRQ);

  resetKeypadPCF();
  old_KeyVal = 1;
  validFRQ = (op_mode == MODE_MR) ? true : Calculate_Frequency(FRQ);
  write_FRQ(current_ch.frequency);

  Serialprint("[BOOT] radio_type=%d validFRQ=%d freq=%lu shift_dir=%d shift=%d\r\n",
              radio_type, validFRQ, (unsigned long)current_ch.frequency,
              current_ch.shift_dir, current_ch.shift);

  scrTimer = TimeoutValue;

#if WIFI_AT_BOOT
  initWebUI();
#endif
  startWebAudioEngine();

  Led_Status = 240 - backlight;
  resetKeypadPCF();
  old_KeyVal = 1;
  if (validFRQ) {
    SetPLLLock(current_ch.frequency);
  }

  PrintMenu();
}

void loop()
{
  if (scrMODE == scrNORMAL && TRX_MODE != TX) {
    uint8_t hb = (millis() / 1000) % 2;
    static uint8_t last_hb = 2;
    if (hb != last_hb) {
      last_hb = hb;
      digitalWrite(LED_BUILTIN, hb);
    }
  }

  // PTT girisi 40ms debounce filtresi (ortak sase uzerinden ses/RF sicramalarini engeller)
  static uint8_t debouncedPttInput = HIGH;
  static unsigned long lastPttEdgeMs = 0;
  uint8_t rawPttInput = digitalRead(PTT_INPUT_PIN);
  if (rawPttInput != debouncedPttInput) {
    if (millis() - lastPttEdgeMs >= 40) {
      debouncedPttInput = rawPttInput;
      lastPttEdgeMs = millis();
    }
  } else {
    lastPttEdgeMs = millis();
  }

  if (webPttActive && (millis() - lastWebPttMs > 1800UL)) {
    setWebPttState(false);
  }
  if (webAudioRxActive && (millis() - lastAudioRxPollMs > 3500UL)) {
    webAudioRxActive = false;
  }

  uint8_t effectivePtt = (debouncedPttInput == HIGH || webPttActive) ? HIGH : LOW;
  TRX_MODE = TXTimeOutTimer(effectivePtt);
  if (TOT_FLAG == 1) {
    writeToLcd("TOTWARN ");
    delay(10);
    if (TOT_WARN_STATE == 1) Alert_Tone(ERR_tone);
    TOT_WARN_STATE = 0;
  }

  // 5-Kademeli Dijital Squelch Filtresi ve Histerezis
  static unsigned long lastSqlSampleMs = 0;
  static int16_t sqlScore = 0;
  static uint8_t filteredBusy = 1;
  static const int16_t sqlOpenThresholds[6] = { 0, 0, 10, 25, 45, 80 };
  const int16_t sqlHoldMax = 180;

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
          filteredBusy = 0;
          sqlScore = openThr + sqlHoldMax;
        } else if (filteredBusy == 0 && sqlScore > openThr + sqlHoldMax) {
          sqlScore = openThr + sqlHoldMax;
        }
      } else {
        sqlScore -= dt;
        if (sqlScore <= 0) {
          sqlScore = 0;
          filteredBusy = 1;
        }
      }
    }
  }

  CHANNEL_BUSY = filteredBusy;
  if (CHANNEL_BUSY == 0) digitalWrite(MUTE_PIN_1, LOW);
  else if (SQL_MODE == SQL_ON) digitalWrite(MUTE_PIN_1, HIGH);
  else digitalWrite(MUTE_PIN_1, LOW);

  // Tus takimi LED durumu (TX gecisinde PLL SPI yazimindan ONCE I2C islemi tamamlanir)
  Led_Status = 240;
  if ((CHANNEL_BUSY == 0) || (SQL_MODE == SQL_OFF)) Led_Status -= green_led;
  if (CHANNEL_BUSY == 0) APRS_Counter = 0;
  if (TRX_MODE == TX) Led_Status -= red_led;
  if (pttToggler) send_packet(_STATUS, (freqLimits.aprs_25 * 25));
  if (APRS_Counter < 2000) Led_Status -= backlight;

  static uint8_t old_Led_Status = 0xFF;
  if (Led_Status != old_Led_Status) {
    old_Led_Status = Led_Status;
    Wire.beginTransmission(PCF8574_KEYB_LED);
    Wire.write(Led_Status);
    Wire.endTransmission();
  }

  if (TRX_MODE != LST_MODE) {
    LST_MODE = TRX_MODE;
    Serialprint("[TRX] %s  f=%lu  FRQ=%s\r\n", TRX_MODE == TX ? "TX" : "RX",
                (unsigned long)current_ch.frequency, FRQ);
    if (TRX_MODE == TX) {
      if (radio_type == 1) {
        write_FRQ(current_ch.frequency);
        delay(5);
        pttKey(true);
        SetTone(current_ch.tone_enabled);
      } else {
        pttKey(true);
        write_FRQ(current_ch.frequency);
        SetTone(current_ch.tone_enabled);
      }
      lastPllRefresh = millis();
    } else {
      pttKey(false);
      write_FRQ(current_ch.frequency);
      SetTone(current_ch.tone_enabled);
    }
  }

  if (TRX_MODE == TX && pllRefreshMs && (millis() - lastPllRefresh) >= pllRefreshMs) {
    lastPllRefresh = millis();
    SetPLLLock(current_ch.frequency);
  }

  // APRS periyodik beacon zamanlayicisi
  static unsigned long lastAprsMillis = 0;
  if (APRS_Timeout > 0 && TRX_MODE == RX && CHANNEL_BUSY != 0) {
    unsigned long intervalMs = (unsigned long)APRS_Timeout * 60000UL;
    if (millis() - lastAprsMillis >= intervalMs) {
      lastAprsMillis = millis();
      send_packet(_FIXPOS_STATUS, (uint32_t)freqLimits.aprs_25 * 25UL);
    }
  } else if (CHANNEL_BUSY == 0 || TRX_MODE == TX) {
    if (APRS_Timeout > 0 && (millis() - lastAprsMillis) >= ((unsigned long)APRS_Timeout * 60000UL)) {
      lastAprsMillis = millis() - ((unsigned long)APRS_Timeout * 60000UL) + 5000UL;
    }
  }

  KeyVal = (TRX_MODE == TX) ? 1 : digitalRead(KeypadIntPin);
  static unsigned long keyPressStartMs = 0;

  if ((KeyVal == old_KeyVal) && (KeyVal == 0)) {
    if ((scrTimer > 0) && (millis() - keyPressStartMs >= 1000)) {
      scrTimer = 0;
    }
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
    if (scrTimer == 0) {
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
      scrTimer = TimeoutValue;
      numChar = 0;
    }
  }

  if (KeyVal != old_KeyVal) {
    if (KeyVal == 0) {
      int satir = readRow();
      int sutun = readColumn();
      if (satir >= 0 && satir < 5 && sutun >= 0 && sutun < 4) {
        pressedKEY = keymap[sutun][satir];
      } else {
        pressedKEY = 'X';
      }
      APRS_Counter = 0;
      scrTimer = TimeoutValue;
      keyPressStartMs = millis();
      if (pressedKEY != 'X') {
        Alert_Tone(OK_tone);
      }

      Serialprint("[KEYPAD] Satir: %d, Sutun: %d -> Basilan Tus: '%c'\r\n", satir, sutun, pressedKEY);

      if (pressedKEY != 'X') {
        if (scrMODE == scrMENU) {
          switch (pressedKEY) {
            case 'U':
              if (subMENU == menuRPT)  { current_ch.shift += 25; write_SHIFTtoLCD(current_ch.shift); }
              if (subMENU == menuTONE) { current_ch.tone_pos += 1; if (current_ch.tone_pos >= TOTAL_TONES - 1) current_ch.tone_pos = TOTAL_TONES - 1; write_TONEtoLCD(current_ch.tone_pos); EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch); }
              if (subMENU == menuSQL)  { if (sql_level < 5) sql_level++; SQL_MODE = SQL_ON; EEPROM.write(EEPROM_SQLMODE_ADDR, SQL_MODE); EEPROM.write(EEPROM_SQLLEVEL_ADDR, sql_level); char b[10]; sprintf(b, "SQL   %d ", sql_level); writeToLcd(b); }
              break;
            case 'D':
              if (subMENU == menuRPT)  { current_ch.shift -= 25; write_SHIFTtoLCD(current_ch.shift); }
              if (subMENU == menuTONE) { current_ch.tone_pos -= 1; if (current_ch.tone_pos <= 0) current_ch.tone_pos = 0; write_TONEtoLCD(current_ch.tone_pos); EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch); }
              if (subMENU == menuSQL)  { if (sql_level > 1) sql_level--; SQL_MODE = SQL_ON; EEPROM.write(EEPROM_SQLMODE_ADDR, SQL_MODE); EEPROM.write(EEPROM_SQLLEVEL_ADDR, sql_level); char b[10]; sprintf(b, "SQL   %d ", sql_level); writeToLcd(b); }
              break;
            case '#':
              if (subMENU == menuRPT)  current_ch.shift = old_frqSHIFT;
              if (subMENU == menuTONE) current_ch.tone_pos = old_ctcss_tone_pos;
              numChar = 0;
              pressedKEY = 'X';
              scrMODE = scrNORMAL;
              subMENU = menuNONE;
              writeFRQToLcd(FRQ);
              break;
            case '*':
              EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
              EEPROM.commit();
              numChar = 0;
              pressedKEY = 'X';
              scrMODE = scrNORMAL;
              subMENU = menuNONE;
              writeFRQToLcd(FRQ);
              break;
            default:
              break;
          }
        }

        if (scrMODE == scrNORMAL && pressedKEY != 'X') {
          switch (pressedKEY) {
            case 'R':
              if (current_ch.shift_dir == noSHIFT) current_ch.shift_dir = plusSHIFT;
              else if (current_ch.shift_dir == plusSHIFT) current_ch.shift_dir = minusSHIFT;
              else current_ch.shift_dir = noSHIFT;
              EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
              EEPROM.commit();
              write_FRQ(current_ch.frequency);
              break;
            case 'B':
              current_ch.tone_enabled = (current_ch.tone_enabled == CTCSS_OFF) ? CTCSS_ON : CTCSS_OFF;
              EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
              EEPROM.commit();
              SetTone(current_ch.tone_enabled);
              write_FRQ(current_ch.frequency);
              break;
            case 'S':
              SQL_MODE = (SQL_MODE == SQL_OFF) ? SQL_ON : SQL_OFF;
              EEPROM.write(EEPROM_SQLMODE_ADDR, (uint8_t)SQL_MODE);
              EEPROM.commit();
              write_FRQ(current_ch.frequency);
              break;
            case 'O':
              RF_POWER_STATE = (RF_POWER_STATE == HIGH_POWER) ? LOW_POWER : HIGH_POWER;
              EEPROM.write(EEPROM_RFPOWER_ADDR, RF_POWER_STATE);
              EEPROM.commit();
              SetRFPower();
              write_FRQ(current_ch.frequency);
              break;
            case 'U':
              if (op_mode == MODE_VFO) {
                numberToFrequency(current_ch.frequency + freq_step, FRQ);
                validFRQ = Calculate_Frequency(FRQ);
                if (validFRQ) {
                  write_FRQ(current_ch.frequency);
                  strcpy(FRQ_old, FRQ);
                  EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
                  EEPROM.commit();
                } else {
                  strcpy(FRQ, FRQ_old);
                  validFRQ = true;
                  writeFRQToLcd(FRQ);
                }
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
                write_FRQ(current_ch.frequency);
              }
              break;
            case 'D':
              if (op_mode == MODE_VFO) {
                numberToFrequency(current_ch.frequency - freq_step, FRQ);
                validFRQ = Calculate_Frequency(FRQ);
                if (validFRQ) {
                  write_FRQ(current_ch.frequency);
                  strcpy(FRQ_old, FRQ);
                  EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
                  EEPROM.commit();
                } else {
                  strcpy(FRQ, FRQ_old);
                  validFRQ = true;
                  writeFRQToLcd(FRQ);
                }
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
              break;
            case 'M':
              if (op_mode == MODE_VFO) {
                op_mode = MODE_MR;
                char chStr[3];
                sprintf(chStr, "%02d", current_memory_channel);
                GetMemoryChannel(chStr);
                if (strncmp(FRQ, "NULL", 4) == 0) {
                  int starting_channel = current_memory_channel;
                  do {
                    current_memory_channel++;
                    if (current_memory_channel > 50) current_memory_channel = 1;
                    sprintf(chStr, "%02d", current_memory_channel);
                    GetMemoryChannel(chStr);
                  } while (strncmp(FRQ, "NULL", 4) == 0 && current_memory_channel != starting_channel);

                  if (strncmp(FRQ, "NULL", 4) == 0) {
                    op_mode = MODE_VFO;
                    strcpy(FRQ, FRQ_old);
                    validFRQ = Calculate_Frequency(FRQ);
                  } else if (current_memory_channel != starting_channel) {
                    EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
                  }
                }
              } else {
                op_mode = MODE_VFO;
                strcpy(FRQ, FRQ_old);
                validFRQ = Calculate_Frequency(FRQ);
              }
              numChar = 0;
              EEPROM.write(EEPROM_OP_MODE_ADDR, op_mode);
              EEPROM.commit();
              write_FRQ(current_ch.frequency);
              writeFRQToLcd(FRQ);
              break;
            case 'C':
              TRX_MODE = TX;
              minSWR = 9999;
              lowestFRQ = 0;
              highestFRQ = 0;
              strcpy(FRQ_old, FRQ);
              current_ch.shift_dir = noSHIFT;
              SetRFPower();
              long min_vna_freq, max_vna_freq;
              int stp_vna_freq;
              if (radio_type == 0) {
                min_vna_freq = freqLimits.vna_min_25 * 25;
                max_vna_freq = freqLimits.vna_max_25 * 25;
                stp_vna_freq = 10;
              } else {
                min_vna_freq = 43000;
                max_vna_freq = 45000;
                stp_vna_freq = 10;
              }
              numberToFrequency(min_vna_freq, FRQ);
              validFRQ = Calculate_Frequency(FRQ);
              write_FRQ(current_ch.frequency);
              delay(PLL_TX_SETTLE_MS);
              pttKey(true);
              Serialprint("\r\n#VNA#\t%l\t%l\r\n", min_vna_freq, max_vna_freq);
              for (long vna_freq = min_vna_freq; vna_freq < max_vna_freq; vna_freq += stp_vna_freq) {
                numberToFrequency(vna_freq, FRQ);
                validFRQ = Calculate_Frequency(FRQ);
                write_FRQ(current_ch.frequency);
                writeFRQToLcd(FRQ);
                Serialprint(">%d\t", vna_freq);
                delay(100);
                readRfPower();
                if (readColumn() != 0) break;
              }
              pttKey(false);
              Serialprint("@VNA@\t%d\t%d\r\n", min_vna_freq, max_vna_freq);
              SetRFPower();
              TRX_MODE = RX;
              numberToFrequency((highestFRQ + lowestFRQ) / 2, FRQ);
              validFRQ = Calculate_Frequency(FRQ);
              write_FRQ(current_ch.frequency);
              writeFRQToLcd(FRQ);
              PrintMenu();
              break;
            case '#':
              if (numChar == 2) {
                StoreFrequency(FRQ, FRQ_old);
                numChar = 0;
                write_FRQ(current_ch.frequency);
              } else if (numChar == 4) {
                StoreSpecialFrequency(FRQ, FRQ_old);
                numChar = 0;
                write_FRQ(current_ch.frequency);
              } else {
                strcpy(FRQ, FRQ_old);
                validFRQ = Calculate_Frequency(FRQ);
                numChar = 0;
                write_FRQ(current_ch.frequency);
              }
              break;
            case '*':
              if (numChar == 2) GetMemoryChannel(FRQ);
              else strcpy(FRQ, FRQ_old);
              validFRQ = Calculate_Frequency(FRQ);
              numChar = 0;
              write_FRQ(current_ch.frequency);
              break;
            default:
              if (numChar <= 6) {
                if (numChar == 0) {
                  for (int i = 0; i < 8; i++) FRQ[i] = ' ';
                }
                FRQ[numChar] = pressedKEY;
                numChar++;
                if (numChar == 3) {
                  FRQ[3] = '.';
                  numChar++;
                }
                writeFRQToLcd(FRQ);
                if (numChar == 7) {
                  validFRQ = Calculate_Frequency(FRQ);
                  numChar = 0;
                  write_FRQ(current_ch.frequency);
                  if (validFRQ) {
                    current_ch.shift_dir = noSHIFT;
                    EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
                    EEPROM.commit();
                    strcpy(FRQ_old, FRQ);
                  } else {
                    strcpy(FRQ, FRQ_old);
                    numChar = 0;
                    validFRQ = Calculate_Frequency(FRQ);
                    write_FRQ(current_ch.frequency);
                  }
                }
              }
              break;
          }
        }
      }
    }

    resetKeypadPCF();
    if (validFRQ && TRX_MODE != TX) {
      SetPLLLock(current_ch.frequency);
    }
    old_KeyVal = KeyVal;
  }

  if (commandComplete) {
    char cmd = commandString.charAt(0);
    switch (cmd) {
      case '\n':
      case '\r': break;
      case 'C': commandRadioType(commandString.charAt(2)); break;
      case 'A': commandStartupMSG(); break;
      case 'S': commandAPRSmycall(); break;
      case 'M': commandAPRSMessage(); break;
      case 'T': commandAPRSTimeout(); break;
      case 'H': commandMemoryDump(); break;
      case 'K': commandDumpConfig(); break;
      case 'P': commandTogglePTT(); break;
      case 'N': startScan(); break;
      case 'W': initWebUI(); break;
      case 'X': softResetDevice(); break;
      default: break;
    }
    commandString = "";
    commandComplete = false;
    PrintMenu();
  }

  handleWebUI();
  serialEvent();
}

void StreamPrint_progmem(Print &out, PGM_P format, ...)
{
#ifdef SERIALMENU
  char formatString[128], *ptr;
  strncpy_P(formatString, format, sizeof(formatString));
  formatString[sizeof(formatString) - 2] = '\0';
  ptr = &formatString[strlen(formatString) + 1];
  va_list args;
  va_start(args, format);
  vsnprintf(ptr, sizeof(formatString) - 1 - strlen(formatString), formatString, args);
  va_end(args);
  formatString[sizeof(formatString) - 1] = '\0';
  out.print(ptr);

  webLogBuffer += ptr;
  if (webLogBuffer.length() > 2000) {
    webLogBuffer = webLogBuffer.substring(webLogBuffer.length() - 2000);
  }
#endif
}

void serialEvent()
{
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    commandString += inChar;
    if (inChar == '\n') {
      commandComplete = true;
    }
  }
}

void softResetDevice()
{
  rp2040.reboot();
}

void pllWriteNA(int N_Counter, int A_Counter)
{
  digitalWrite(PLL_SEC, LOW);
  send_SPIBit(pllR, 14);
  send_SPIBit(1, 1);
  send_SPIEnable();
  send_SPIBit(N_Counter, 10);
  send_SPIBit(A_Counter, 7);
  send_SPIBit(0, 1);
  send_SPIEnable();
}

void pllWriteWords(uint32_t f)
{
  uint32_t stepKhz10 = 12800UL * 10UL / pllR;
  uint32_t f10 = f * 10UL;
  int N_Counter = (int)(f10 / stepKhz10 / 80);
  int A_Counter = (int)(f10 / stepKhz10) - (80 * N_Counter);
  pllWriteNA(N_Counter, A_Counter);

#if defined(PLL_DEBUG)
  Serialprint("[PLL] vco=%lu R=%u N=%d A=%d ref=%lu BS0=%d BS1=%d\r\n",
              (unsigned long)f, pllR, N_Counter, A_Counter,
              (unsigned long)((N_Counter * 80 + A_Counter) * stepKhz10 / 10),
              digitalRead(BAND_SELECT_0), digitalRead(BAND_SELECT_1));
#endif
}

void send_SPIBit(int Counter, uint8_t length)
{
  for (int i = length - 1; i >= 0; i--) {
    digitalWrite(pll_data_pin, bitRead(Counter, i));
    delayMicroseconds(pllBitDelayUs);
    digitalWrite(pll_clk_pin, HIGH);
    delayMicroseconds(pllBitDelayUs);
    digitalWrite(pll_clk_pin, LOW);
    delayMicroseconds(pllBitDelayUs);
  }
  digitalWrite(pll_data_pin, LOW);
}

void send_SPIEnable()
{
  digitalWrite(pll_ena_pin, HIGH);
  delayMicroseconds(pllBitDelayUs);
  digitalWrite(pll_ena_pin, LOW);
  delayMicroseconds(pllBitDelayUs);
}

void pllApplyMode(uint8_t m)
{
  if (m >= PLL_MODE_COUNT) m = 0;
  pllMode = m;
  switch (m) {
    case 0: pllBitDelayUs = 1000; pllDoubleWrite = false; pllRefreshMs = 0;   break;
    case 1: pllBitDelayUs = 100;  pllDoubleWrite = false; pllRefreshMs = 0;   break;
    case 2: pllBitDelayUs = 10;   pllDoubleWrite = false; pllRefreshMs = 0;   break;
    case 3: pllBitDelayUs = 10;   pllDoubleWrite = true;  pllRefreshMs = 200; break;
    case 4: pllBitDelayUs = 1000; pllDoubleWrite = true;  pllRefreshMs = 200; break;
  }
}

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

uint slice_num;
volatile int phase_acc = 0;
struct repeating_timer tone_timer;
bool timer_active = false;

bool timer_callback(struct repeating_timer *t)
{
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(MIC_PIN), sine_wave[phase_acc]);
  phase_acc = (phase_acc + 1) % SINE_TABLE_SIZE;
  return true;
}

void start_tone(float frequency)
{
  if (timer_active) return;
  float interval_us = 1000000.0f / (frequency * SINE_TABLE_SIZE);

  gpio_set_function(MIC_PIN, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(MIC_PIN);
  pwm_set_wrap(slice_num, 255);
  pwm_set_enabled(slice_num, true);

  add_repeating_timer_us(-interval_us, timer_callback, NULL, &tone_timer);
  timer_active = true;
}

void stop_tone()
{
  if (timer_active) {
    cancel_repeating_timer(&tone_timer);
    timer_active = false;
  }
  gpio_set_function(MIC_PIN, GPIO_FUNC_SIO);
  gpio_put(MIC_PIN, 1);
  gpio_set_dir(MIC_PIN, GPIO_IN);
  gpio_set_pulls(MIC_PIN, false, false);
}

void SetTone(int toneSTATE)
{
  noTone(ALERT_PIN);
  if (timer_active) {
    cancel_repeating_timer(&tone_timer);
    timer_active = false;
  }
  pwm_set_enabled(pwm_gpio_to_slice_num(MIC_PIN), false);

  if (TRX_MODE == RX) {
    gpio_init(MIC_PIN);
    gpio_set_dir(MIC_PIN, GPIO_OUT);
    gpio_put(MIC_PIN, 0);
  } else {
    if (toneSTATE == CTCSS_ON && !webPttActive) {
      float f = (current_ch.tone_pos >= 0 && current_ch.tone_pos < TOTAL_TONES)
                ? ctcss_tone_list[current_ch.tone_pos] : 88.5f;
      if (micTxMode == 1) {
        start_tone(f);
      } else {
        analogWriteFreq(f);
        analogWrite(MIC_PIN, 5);
      }
    } else if (webPttActive) {
      gpio_set_function(MIC_PIN, GPIO_FUNC_PWM);
      uint s = pwm_gpio_to_slice_num(MIC_PIN);
      pwm_set_wrap(s, 255);
      pwm_set_chan_level(s, pwm_gpio_to_channel(MIC_PIN), 128);
      pwm_set_enabled(s, true);
    } else {
      gpio_init(MIC_PIN);
      gpio_put(MIC_PIN, 1);
      gpio_set_dir(MIC_PIN, GPIO_IN);
      gpio_set_pulls(MIC_PIN, false, false);
      gpio_deinit(MIC_PIN);
    }
  }
}

static struct repeating_timer web_audio_timer;
static bool web_audio_timer_started = false;
static int32_t adc_dc_accum = (2048 << 8);

bool audio_engine_timer_cb(struct repeating_timer *t)
{
  if (webPttActive && TRX_MODE == TX) {
    uint16_t tail = audioTxTail;
    if (tail != audioTxHead) {
      uint8_t sample = audioTxRing[tail];
      audioTxTail = (tail + 1) & (AUDIO_RING_SIZE - 1);
      pwm_set_chan_level(pwm_gpio_to_slice_num(MIC_PIN), pwm_gpio_to_channel(MIC_PIN), sample);
    }
    return true;
  }

  // RX modunda GP26 (ADC0 - SP hatti) uzerinden 8000 Hz ornekleme
  if (TRX_MODE == RX) {
    uint16_t raw = (uint16_t)(adc_hw->result & 0x0FFFU);
    hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);

    adc_dc_accum += ((int32_t)raw - (adc_dc_accum >> 8));
    int32_t centered = (int32_t)raw - (adc_dc_accum >> 8);
    int32_t absVal = (centered < 0) ? -centered : centered;
    uint8_t instPeak = (absVal > 511) ? 100 : (uint8_t)((absVal * 100) >> 9);
    if (instPeak > audioRxPeak) {
      audioRxPeak = instPeak;
    }

    if (webAudioRxActive) {
      int32_t pcm = (centered >> 2) + 128;
      if (pcm < 0) pcm = 0;
      else if (pcm > 255) pcm = 255;

      uint16_t nextHead = (audioRxHead + 1) & (AUDIO_RING_SIZE - 1);
      if (nextHead != audioRxTail) {
        audioRxRing[audioRxHead] = (uint8_t)pcm;
        audioRxHead = nextHead;
      }
    }
  }
  return true;
}

void startWebAudioEngine()
{
  if (web_audio_timer_started) return;
  adc_select_input(0);
  hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
  add_repeating_timer_us(-125, audio_engine_timer_cb, NULL, &web_audio_timer);
  web_audio_timer_started = true;
}

void setWebPttState(bool on)
{
  if (on) {
    lastWebPttMs = millis();
    if (!webPttActive) {
      webPttActive = true;
      gpio_set_function(MIC_PIN, GPIO_FUNC_PWM);
      uint s = pwm_gpio_to_slice_num(MIC_PIN);
      pwm_set_wrap(s, 255);
      pwm_set_chan_level(s, pwm_gpio_to_channel(MIC_PIN), 128);
      pwm_set_enabled(s, true);
    }
  } else {
    if (webPttActive) {
      webPttActive = false;
      audioTxTail = audioTxHead;
    }
  }
}
