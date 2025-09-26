#include "declarations.h"
#include <Update.h> // OTA için eklendi

// --- HATA ÇÖZÜMÜ: Global Değişken Tanımlamaları ---
// "undefined reference" hatasını gidermek için 'font' ve 'sine_wave' dizilerinin
// içerikleri buraya taşındı. 'declarations.h' dosyasındaki 'extern' bildirimleri sayesinde
// diğer .cpp dosyaları bu değişkenlere erişebilir.

const unsigned char font[] = {
  B00010000, B00000000, B00001111, // _ underscore
  B00000000, B00000000, B00000000, // space
  B01000000, B00000100, B00000000, // / division
  B00100000, B00000100, B00000000, // - dash
  B00000000, B00000000, B00010000, // . dot
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
// --- BİTTİ: HATA ÇÖZÜMÜ ---


// --- Global Değişkenlerin Tanımlanması ---
// 'declarations.h' dosyasında 'extern' olarak bildirilen tüm değişkenler burada somut olarak oluşturulur.

// WiFi ve Web Sunucusu
AsyncWebServer server(80);
DynamicJsonDocument wifiConfig(2048);

// DAC Ton Üretimi
hw_timer_t * dac_timer = NULL;
volatile int phase_accumulator = 0;

// GPS
HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

// LCD
byte Led_Status= 240;
byte KeyVal = 0;
byte old_KeyVal= 0;
unsigned char matrix[24];
unsigned char chr2wr[3];
byte Position_Signs[8][3] = { 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0 } ; //The signs that appear while printing at position 0 to 7

// Telsiz Durum Değişkenleri
bool pttToggler = false;
bool web_ptt_active = false; // YENİ: Web arayüzü PTT durumunu tutan değişken
bool web_scan_request = false; // YENİ: Web'den tarama isteği geldiğini belirten bayrak.
bool web_vna_request = false;  // YENİ: Web'den VNA isteği geldiğini belirten bayrak.
byte radio_type = 0; //0 VHF 1 UHF
byte TRX_MODE = RX;
byte LST_MODE = TX;
byte RF_POWER_STATE = HIGH_POWER;
byte ALERT_MODE = ALERT_ON;
byte CHANNEL_BUSY = 1;
int SQL_MODE = SQL_ON;

// Arayüz ve Menü
int scrTimer = 0;
char pressedKEY = ' ';
byte scrMODE = scrNORMAL;
byte subMENU = menuNONE;
boolean hasASEL = false;
boolean hasLOCK = false;
boolean hasSPKR = false;
boolean hasTHUN = false;
boolean hasARRW = false;
boolean hasMENU = false;
boolean hasLOOP = false;
boolean hasNOTE = false;

// Frekans Yönetimi
byte numChar = 0;
char FRQ[9];
char FRQ_old[9];
boolean validFRQ;
char SHIFT_INPUT[6];
byte shiftInputPos = 0;
byte old_ctcss_tone_pos;
int old_frqSHIFT;

// Kanal ve Ayar Yapıları
channel_t current_ch;
memorych_t memoryChannels[100];
freqLimits_t freqLimits;

// YENİ: APRS Frekansları için global değişken tanımlamaları
uint16_t aprs_freq_125;
uint16_t iss_freq_125;

// VNA
float minSWR;
long lowestFRQ;
long highestFRQ;

// Seri Port
String commandString = "";
bool commandComplete = false;
String currentIP = "NO IP";
String ipToDisplay = "";

// LittleFS Ayarları
String ui_theme = "dark"; // YENİ: Web arayüzü temasını tutar ("dark" veya "light")
String greetingMessage = "";
uint16_t tot_timer = 0;
String mycall = "N0CALL";
String APRS_Message = "ASELSAN 48XX/49XX - APRS TEST";
String lat = "4102.74N";
String lon = "02902.06E";
unsigned int APRS_Timeout = 3;
unsigned long APRS_Counter = 0;

// APRS
bool use_gps = true;
bool tot_lockout = false; // YENİ: TOT kilitleme bayrağı
bool nada = _2400;
char myssid = 9;
char bit_stuff = 0;
unsigned short crc=0xffff;


void setup() {
//pin modes and initial states
  pinMode(SQL_ACTIVE, INPUT);
  // MIC_PIN'in başlangıç modu SetTone içinde yönetileceği için buradan kaldırıldı.
  pinMode(RF_POWER_PIN, OUTPUT);
  digitalWrite(RF_POWER_PIN, RF_POWER_STATE);
  pinMode(MUTE_PIN_1, OUTPUT);
  digitalWrite(MUTE_PIN_1, HIGH);
  pinMode(BAND_SELECT_0, OUTPUT);
  pinMode(BAND_SELECT_1, OUTPUT);
  pinMode(PTT_INPUT_PIN,  INPUT_PULLUP);
  pinMode(PTT_OUTPUT_PIN, OUTPUT);
  digitalWrite(PTT_OUTPUT_PIN, LOW);
  pinMode(FWD_POWER_PIN, INPUT);
  pinMode(REF_POWER_PIN, INPUT);
  pinMode(KeypadIntPin,    INPUT_PULLUP);
  pinMode(pll_clk_pin, OUTPUT);
  pinMode(pll_data_pin,OUTPUT);
  pinMode(pll_ena_pin, OUTPUT);
  pinMode(PLL_SEC, OUTPUT);

  digitalWrite(pll_clk_pin, LOW);
  digitalWrite(pll_data_pin,LOW);
  digitalWrite(pll_ena_pin, LOW);

  // DEĞİŞTİ: LEDC yerine DAC timer'ı başlat
  // 0 -> timer #0, 80 -> prescaler (80MHz/80=1MHz), true -> count up
  dac_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(dac_timer, &onDacTimer, true); // Kesme fonksiyonunu bağla

  delay(100);

  Serial.begin(9600);
  commandString.reserve(200);

  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  initializeFileSystem();

  // --- YENİ: WiFi ve Web Sunucusunu Başlatma ---
  // GÜNCELLEME: WiFi sadece ayarlarda aktif ise başlatılır.
  if (wifiConfig["wifi_enabled"].as<bool>()) {
    connectToWiFi(); // WiFi'ye bağlanmayı dene veya AP modunu başlat
    setupWebServer(); // Web sunucusunu kur ve başlat
  } else {
    Serial.println("WiFi kapalı. WiFi özellikleri ve web sunucusu devre dışı.");
    WiFi.mode(WIFI_OFF); // Güç tasarrufu için WiFi modülünü tamamen kapat
  }
  // --- BİTTİ: WiFi ve Web Sunucusunu Başlatma ---

  numberToFrequency(current_ch.frequency, FRQ);
  strcpy(FRQ_old,FRQ);

  SetTone(current_ch.tone_enabled);

  memset(matrix, 0, 24);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  InitLCD();
  Greetings();
  writeFRQToLcd(FRQ);

  old_KeyVal = 1; //initial keypad read
  validFRQ = Calculate_Frequency(FRQ);
  scrTimer = TimeoutValue;

  PrintMenu();
}

void loop() {
  // YENİ: TOT (Gönderme Zaman Aşımı) Mantığı
  static unsigned long pttStartTime = 0; // PTT'ye basılma zamanını saklar

  // ISTEK: WiFi bağlantısından sonra gösterilecek bir IP adresi varsa, şimdi göster.
  // Bu, I2C çakışmalarını önlemek için ana döngüde yapılır.
  if (ipToDisplay.length() > 0) {
    displayAndScrollOnce(ipToDisplay.c_str());
    ipToDisplay = ""; // Flag'i temizle, böylece tekrar gösterilmez.
  }

  // YENİ: Web arayüzünden gelen asenkron istekleri ana döngüde işle
  if (web_scan_request) {
    web_scan_request = false; // Bayrağı hemen sıfırla ki tekrar çalışmasın
    Serial.println("Ana donguden tarama baslatiliyor (web istegi)...");
    startScan();
  }

  if (web_vna_request) {
    web_vna_request = false; // Bayrağı hemen sıfırla
    Serial.println("Ana donguden VNA baslatiliyor (web istegi)...");
    
    // Bu kod bloğu, fiziksel 'C' tuşuna basıldığındaki mantığın birebir kopyasıdır.
    TRX_MODE = TX;
    minSWR = 9999;
    lowestFRQ=0; highestFRQ=0;
    strcpy(FRQ_old,FRQ);
    current_ch.shift_dir = noSHIFT;
    SetRFPower();
    long min_vna_freq;
    long max_vna_freq;
    int  stp_vna_freq;
    if (radio_type==0) {
      min_vna_freq = 14000;
      max_vna_freq = 15000;
      stp_vna_freq = 10;
    } else {
      min_vna_freq = 43000;
      max_vna_freq = 45000;
      stp_vna_freq = 10;
    }
    digitalWrite(PTT_OUTPUT_PIN,HIGH);
    Serialprint("\r\n#VNA#\t%l\t%l\r\n",min_vna_freq,max_vna_freq); //START
    for (long vna_freq=min_vna_freq; vna_freq < max_vna_freq; vna_freq += stp_vna_freq)
      {
        numberToFrequency(vna_freq*10,FRQ);
        validFRQ = Calculate_Frequency(FRQ);
        write_FRQ(current_ch.frequency);
        writeFRQToLcd(FRQ);
        Serialprint(">%d\t",vna_freq);
        delay(100);
        readRfPower();

        if (readColumn() != 0) break; //a key is pressed
      }
      digitalWrite(PTT_OUTPUT_PIN,LOW);
      Serialprint("@VNA@\t%d\t%d\r\n",min_vna_freq,max_vna_freq); //END
      SetRFPower();
      TRX_MODE = RX;
      numberToFrequency((highestFRQ+lowestFRQ)/2,FRQ);
      validFRQ = Calculate_Frequency(FRQ);
      write_FRQ(current_ch.frequency);
      writeFRQToLcd(FRQ);
      PrintMenu();
  }


  if (scrMODE==scrNORMAL)
  {
    if (TRX_MODE == TX)
    {
      numberToFrequency(current_ch.frequency+current_ch.shift_dir*current_ch.shift,FRQ_old);
      writeFRQToLcd(FRQ_old);
    } else
    {
      writeFRQToLcd(FRQ);
    }
  }

  Wire.beginTransmission(PCF8574_KEYB_LED);
  Led_Status = 240;
  if ((CHANNEL_BUSY==0) || (SQL_MODE==SQL_OFF)) Led_Status = Led_Status - green_led;
  if (CHANNEL_BUSY==0) APRS_Counter = 0;
  if (TRX_MODE == TX) Led_Status = Led_Status - red_led;

  if (pttToggler) send_packet(_STATUS,(aprs_freq_125*12.5));

  if (APRS_Counter < 2000) Led_Status = Led_Status - backlight;
  Wire.write(Led_Status);
  Wire.endTransmission();

  CHANNEL_BUSY = digitalRead(SQL_ACTIVE);
  if (CHANNEL_BUSY == 0) digitalWrite(MUTE_PIN_1, LOW);
    else if (SQL_MODE == SQL_ON) digitalWrite(MUTE_PIN_1, HIGH);
      else digitalWrite(MUTE_PIN_1, LOW);

// Orijinal PTT mantığına geri dönüldü ve web PTT eklendi
  TRX_MODE = digitalRead(PTT_INPUT_PIN); // Orijinal, sizin için doğru çalışan satır.
  if (web_ptt_active) {
    // Eğer web'den PTT aktif edildiyse, fiziksel durumu geçersiz kıl ve TX'e geç.
    TRX_MODE = TX;
  }

  // YENİ: TOT Kilitleme ve Sıfırlama Mantığı
  if (TRX_MODE == RX) {
    // Mandal bırakıldıysa, kilidi her zaman kaldır.
    tot_lockout = false;
  } else { // TRX_MODE == TX
    // Mandal basılıysa ve kilit aktifse, göndermeyi engelle.
    if (tot_lockout) {
      TRX_MODE = RX;
    }
  }

  // ESKİ: TOT (Gönderme Zaman Aşımı) Kontrolü
  if (TRX_MODE == TX && LST_MODE == RX) {
    // PTT'ye yeni basıldı, zamanlayıcıyı başlat
    pttStartTime = millis();
  }

  if (TRX_MODE == TX && tot_timer > 0) {
    if (millis() - pttStartTime > (tot_timer * 1000UL)) {
      Serial.println("TOT Aşıldı! Gönderim durduruluyor.");
      TRX_MODE = RX; // Gönderimi zorla durdur
      web_ptt_active = false; // Web PTT'yi de kapat
      tot_lockout = true; // YENİ: Mandal bırakılana kadar kilitle
      digitalWrite(PTT_OUTPUT_PIN, LOW); // PTT'yi bırak
      Alert_Tone(ERR_tone); // Kullanıcıyı uyar
    }
  }

  if (TRX_MODE != LST_MODE) {
    LST_MODE = TRX_MODE;
    write_FRQ(current_ch.frequency);
  }

  if (TRX_MODE == TX ) {
      digitalWrite(PTT_OUTPUT_PIN, HIGH);
  } else {
      digitalWrite(PTT_OUTPUT_PIN, LOW);
  }

  SetTone(current_ch.tone_enabled);

  APRS_Counter += 1;
  if (((APRS_Counter/150) >= (APRS_Timeout * 60)) && (APRS_Timeout > 0)) {
     send_packet(_FIXPOS_STATUS,(aprs_freq_125*12.5));
     if (radio_type == 0) {
        send_packet(_FIXPOS_STATUS,(iss_freq_125*12.5));
     }
     APRS_Counter = 0;
  }

  KeyVal = digitalRead(KeypadIntPin);


  if ((KeyVal == old_KeyVal) & (KeyVal ==  0) & (scrTimer>0)) scrTimer -= 1;
    else if (KeyVal==1) {
     if (scrTimer==0) {
       if (pressedKEY=='B') { writeToLcd("TONE     "); subMENU = menuTONE; delay(1000);write_TONEtoLCD(current_ch.tone_pos); old_ctcss_tone_pos = current_ch.tone_pos;}
       if (pressedKEY=='S') { writeToLcd("SQL      "); subMENU = menuSQL;  }
       if (pressedKEY=='O') { writeToLcd("SCAN     "); delay(1000); startScan(); }
       if (pressedKEY=='R') {
            writeToLcd("SHIFT    ");
            subMENU = menuRPT;
            delay(1000);
            write_SHIFTtoLCD(current_ch.shift);
            old_frqSHIFT=current_ch.shift;
            shiftInputPos = 0;
        }
       if (pressedKEY=='M') { writeToLcd("MENU     "); subMENU = menuMENU; }
       delay(500);
       scrTimer = TimeoutValue;
       numChar = 0;
       scrMODE = scrMENU;
     }
    }


  if (KeyVal != old_KeyVal) {
    if (KeyVal == 0) Alert_Tone(OK_tone);

    APRS_Counter = 0;

    scrTimer = TimeoutValue;

    int satir = readRow();
    int sutun = readColumn();
    pressedKEY = keymap[sutun][satir];

  /* -----------------------------------------------------
     SCREEN MODE MENU..
     ----------------------------------------------------- */

    if (scrMODE == scrMENU)  {
        if ( subMENU == menuRPT) {
            if (pressedKEY >= '0' && pressedKEY <= '9') {
                if (shiftInputPos == 0) {
                    memset(SHIFT_INPUT, ' ', 5);
                    SHIFT_INPUT[5] = 0;
                }
                if (shiftInputPos < 5) {
                    SHIFT_INPUT[shiftInputPos++] = pressedKEY;
                    writeToLcd(SHIFT_INPUT);
                }
            } else {
                switch (pressedKEY) {
                    case 'U':
                        shiftInputPos = 0;
                        current_ch.shift += 25;
                        if(current_ch.shift > 9975) current_ch.shift = 9975;
                        write_SHIFTtoLCD(current_ch.shift);
                        break;
                    case 'D':
                        shiftInputPos = 0;
                        current_ch.shift -= 25;
                        if(current_ch.shift < 0) current_ch.shift = 0;
                        write_SHIFTtoLCD(current_ch.shift);
                        break;
                    case '#': // CANCEL
                        current_ch.shift = old_frqSHIFT;
                        shiftInputPos = 0;
                        numChar = 0;
                        pressedKEY = 'X';
                        scrMODE = scrNORMAL;
                        subMENU = menuNONE;
                        break;
                    case '*': // OK
                        if (shiftInputPos > 0) {
                           current_ch.shift = atoi(SHIFT_INPUT);
                        }
                        shiftInputPos = 0;
                        saveCurrentChannel();
                        numChar = 0;
                        pressedKEY = 'X';
                        scrMODE = scrNORMAL;
                        subMENU = menuNONE;
                        break;
                }
            }
        } else if (subMENU == menuTONE) {
            switch (pressedKEY) {
                case 'U':
                    current_ch.tone_pos += 1; if (current_ch.tone_pos>=TOTAL_TONES-1) current_ch.tone_pos = TOTAL_TONES-1; write_TONEtoLCD(current_ch.tone_pos);
                    break;
                case 'D':
                    current_ch.tone_pos -= 1; if (current_ch.tone_pos<=0) current_ch.tone_pos = 0; write_TONEtoLCD(current_ch.tone_pos);
                    break;
                case '#':
                    current_ch.tone_pos = old_ctcss_tone_pos;
                    numChar = 0;
                    pressedKEY = 'X';
                    scrMODE = scrNORMAL;
                    subMENU = menuNONE;
                    break;
                case '*':
                    saveCurrentChannel();
                    numChar = 0;
                    pressedKEY = 'X';
                    scrMODE = scrNORMAL;
                    subMENU = menuNONE;
                    break;
            }
        } else {
            pressedKEY = 'X';
            scrMODE = scrNORMAL;
            subMENU = menuNONE;
        }
    }


  /* -----------------------------------------------------
   SCREEN MODE NORMAL.. WE READ FREQUENCY AND OTHER KEYS
   ----------------------------------------------------- */
    if (scrMODE == scrNORMAL) { //mode NORMAL and key released
      if (pressedKEY != 'X') {
        // Check for the COMMAND keys first
        switch (pressedKEY) {
          case 'R':
            if (current_ch.shift_dir == noSHIFT) current_ch.shift_dir = plusSHIFT;
            else if (current_ch.shift_dir == plusSHIFT) current_ch.shift_dir = minusSHIFT;
            else current_ch.shift_dir = noSHIFT;
            saveCurrentChannel();
          break; //'R'
          case 'B':
            if (current_ch.tone_enabled == CTCSS_OFF) current_ch.tone_enabled = CTCSS_ON;
            else current_ch.tone_enabled = CTCSS_OFF;
            saveCurrentChannel();
          break; // 'B'
          case 'S':
            if (SQL_MODE == SQL_OFF) SQL_MODE = SQL_ON;
            else SQL_MODE = SQL_OFF;
          break; //'S'
          case 'O':
             if (RF_POWER_STATE == HIGH_POWER) RF_POWER_STATE = LOW_POWER;
             else RF_POWER_STATE = HIGH_POWER;
             SetRFPower();
          break; //'O'
          case 'U':
            current_ch.frequency += 25;
            numberToFrequency(current_ch.frequency,FRQ);
            validFRQ = Calculate_Frequency(FRQ);
            if(validFRQ) write_FRQ(current_ch.frequency);
            else current_ch.frequency -= 25;
            saveCurrentChannel();
          break; // 'U'
          case 'D':
            current_ch.frequency -= 25;
            numberToFrequency(current_ch.frequency,FRQ);
            validFRQ = Calculate_Frequency(FRQ);
            if(validFRQ) write_FRQ(current_ch.frequency);
            else current_ch.frequency += 25;
            saveCurrentChannel();
          break; // 'D'
          case 'M': //Reverse
            numberToFrequency(current_ch.frequency+current_ch.shift_dir*current_ch.shift,FRQ);
            if (current_ch.shift_dir == plusSHIFT) current_ch.shift_dir = minusSHIFT;
            else if (current_ch.shift_dir == minusSHIFT) current_ch.shift_dir = plusSHIFT;
            Calculate_Frequency(FRQ);
            write_FRQ(current_ch.frequency);
          break; // 'M'
          case 'C':
            //VNA Vector Network analizor Subroutines
            TRX_MODE = TX; //needed for PLL locking to TX frequrency
            minSWR = 9999;
            lowestFRQ=0;
            highestFRQ=0;
            strcpy(FRQ_old,FRQ); //store old frequency for recall
            current_ch.shift_dir = noSHIFT; //get into SIMPLEX mode for caculations
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
            digitalWrite(PTT_OUTPUT_PIN,HIGH);
            Serialprint("\r\n#VNA#\t%l\t%l\r\n",min_vna_freq,max_vna_freq); //START
            for (long vna_freq=min_vna_freq; vna_freq < max_vna_freq; vna_freq += stp_vna_freq)
              {
                numberToFrequency(vna_freq*10,FRQ);
                validFRQ = Calculate_Frequency(FRQ);
                write_FRQ(current_ch.frequency);
                writeFRQToLcd(FRQ);
                Serialprint(">%d\t",vna_freq);
                delay(100);
                readRfPower();

                if (readColumn() != 0) break; //a key is pressed
              }
              digitalWrite(PTT_OUTPUT_PIN,LOW);
              Serialprint("@VNA@\t%d\t%d\r\n",min_vna_freq,max_vna_freq); //END
              SetRFPower();
              TRX_MODE = RX; //PLL should lock to RX
              numberToFrequency((highestFRQ+lowestFRQ)/2,FRQ);
              validFRQ = Calculate_Frequency(FRQ);
              write_FRQ(current_ch.frequency);
              writeFRQToLcd(FRQ);
              PrintMenu();
          break; // 'C'

          // ##### DEĞİŞİKLİK BAŞLANGICI: Kanal Kaydetme ve Özel Komut Mantığı #####
          case '#':
              // Eğer bir girişin ortasındaysak (numChar > 0)...
              if (numChar > 0) {
                  // Ve bu giriş * ile başladıysa, bu bir özel komut çalıştırmasıdır.
                  if (FRQ[0] == '*') {
                      char commandNumStr[4];
                      if (numChar > 1 && numChar <= 4) { // * ve en az 1 hane var mı?
                          strncpy(commandNumStr, &FRQ[1], numChar - 1);
                          commandNumStr[numChar - 1] = '\0';
                          StoreSpecialFrequency(commandNumStr, FRQ_old);
                      }
                      // İşlem sonrası her durumda normal moda dönülür.
                      strcpy(FRQ, FRQ_old);
                      numChar = 0;
                  }
                  // Eğer giriş bizim özel bayrağımızla (?) başladıysa, bu bir kaydetme onayıdır.
                  else if (FRQ[0] == '>') {
                      if (numChar > 1) { // ? ve en az 1 hane var mı?
                          char channelNumStr[4];
                          strncpy(channelNumStr, &FRQ[1], numChar - 1);
                          channelNumStr[numChar - 1] = '\0';
                          StoreFrequency(channelNumStr, FRQ_old);
                          Alert_Tone(SUCC_tone);
                      }
                      strcpy(FRQ, FRQ_old);
                      numChar = 0;
                  }
                  // Diğer tüm durumlarda (normal frekans girişi vb.) # tuşu iptal eder.
                  else {
                      strcpy(FRQ, FRQ_old);
                      numChar = 0;
                  }
              }
              // Normal ekrandaysak (numChar == 0), Kanal Kaydetme modunu başlat.
              else {
                  strcpy(FRQ_old, FRQ);
                  memset(FRQ, ' ', 8);
                  FRQ[8] = 0;
                  FRQ[0] = '>'; // Ekranda görünmeyecek özel bayrağımız.
                  numChar = 1;
                  writeToLcd(&FRQ[1]); // Ekrana sadece boşlukları yazdır (ekranı temizle).
              }
              writeFRQToLcd(FRQ);
              break;

          case '*':
              // Eğer bir komut girişi yapılıyorsa...
              if (numChar > 0) {
                  // Ve bu giriş * ile başladıysa, Kanal Çağırma işlemini tamamla.
                  if (FRQ[0] == '*') {
                      if (numChar > 1) { // Araya rakam girildiğinden emin ol
                          char channelNumStr[4];
                          strncpy(channelNumStr, &FRQ[1], numChar - 1);
                          channelNumStr[numChar - 1] = '\0';
                          GetMemoryChannel(channelNumStr);
                          validFRQ = Calculate_Frequency(FRQ);
                          write_FRQ(current_ch.frequency);
                      } else { // Sadece "**" basıldı, iptal et.
                          strcpy(FRQ, FRQ_old);
                      }
                  }
                  // Diğer tüm durumlarda (# ile başlayan veya frekans girişi), iptal et.
                  else {
                      strcpy(FRQ, FRQ_old);
                  }
                  numChar = 0;
              }
              // Normal ekrandaysak, Kanal Çağırma / Özel Komut modunu başlat.
              else {
                  strcpy(FRQ_old, FRQ);
                  memset(FRQ, ' ', 8);
                  FRQ[0] = '*';
                  FRQ[8] = 0;
                  numChar = 1;
              }
              writeFRQToLcd(FRQ);
              break;

          default:
            /* ------------------    FRQ/KOMUT GİRİŞİ ------------ */
            // Eğer bir komut modundaysak (# veya * ile başlamışsa), rakamları işle
            if (FRQ[0] == '>' || FRQ[0] == '*') {
                if (numChar < 4) { // Maksimum 3 hane + başı = 4 karakter
                    FRQ[numChar] = pressedKEY;
                    numChar++;
                    // Özel bayrağı göstermemek için +1'den başlayarak yazdırıyoruz.
                    writeToLcd(&FRQ[1]);
                }
            }
            // Normal frekans girişi
            else {
              if (numChar == 0) {
                // Yeni bir frekans girişi başlıyor, mevcut frekansı yedekle
                strcpy(FRQ_old, FRQ);
                memset(FRQ, ' ', 8);
                FRQ[8] = 0;
              }

              if (numChar <= 6) {
                  FRQ[numChar] = pressedKEY;
                  numChar++;
                  if (numChar == 3) { // 3. haneden sonra noktayı koy
                    FRQ[3] = '.';
                    numChar++;
                  }
              }

              writeFRQToLcd(FRQ); // Girilen karakteri anında ekranda göster

              if (numChar == 7) { // Frekans girişi tamamlandı (örn: 145.600)
                validFRQ = Calculate_Frequency(FRQ);
                numChar = 0;
                if (validFRQ) {
                  write_FRQ(current_ch.frequency);
                  strcpy(FRQ_old,FRQ);
                  saveCurrentChannel();
                } else {
                  strcpy(FRQ,FRQ_old); // Hatalı giriş, eski frekansa dön
                  validFRQ = Calculate_Frequency(FRQ);
                }
              }
            }//numchar<=6
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
    if (commandString.charAt(0) == 'Y') commandYardim(commandString.charAt(2));
    if (commandString.charAt(0) == 'C') commandCevrim(commandString.charAt(2));
    if (commandString.charAt(0) == 'A') commandAcilis();
    if (commandString.charAt(0) == 'T') commandAPRSSure();
    if (commandString.charAt(0) == 'M') commandAPRSMesaj();
    if (commandString.charAt(0) == 'S') commandAPRSmycall();
    if (commandString.charAt(0) == 'H') commandHafizaDok();
    if (commandString.charAt(0) == 'K') commandAyarDok();
    if (commandString.charAt(0) == 'P') commandTogglePTT();
    if (commandString.charAt(0) == 'G') getGPSData();
    if (commandString.charAt(0) == 'R') getFSData();
    if (commandString.charAt(0) == 'N') startScan();

    commandString = "";
    commandComplete = false;
    PrintMenu();
    Serialprint("\r\nSeciminiz>");
  }
}


