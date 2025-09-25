#include "declarations.h"

// CTCSS Ton Listesi
float ctcss_tone_list[TOTAL_TONES] = {67,69.3,71.9,74.4,77,79.7,82.5,85.4,88.5,91.5,94.8,97.4,100,103.5,107.2,110.9,114.8,118.8,123,127.3};

// APRS Sabitleri
const float baud_adj = 0.975;
const float adj_1200 = 1.0 * baud_adj;
const float adj_2400 = 1.0 * baud_adj;
unsigned int tc1200 = (unsigned int)(0.5 * adj_1200 * 1000000.0 / 1200.0);
unsigned int tc2400 = (unsigned int)(0.5 * adj_2400 * 1000000.0 / 2400.0);
const char *dest = "TAMSAT";
const char *digi = "ARISS";
char digissid = 1;
const char sym_ovl = '>';
const char sym_tab = '/';


// YENİ: Sadece banda özel ayarları (limitler, VFO, APRS frekansları)
// kullanıcı verilerine (kanallar, wifi, vb.) dokunmadan güncelleyen fonksiyon.
void applyBandSpecificDefaults(bool forceUHF) {
    Serial.printf("Banda ozel varsayilanlar %s icin uygulaniyor...\n", forceUHF ? "UHF" : "VHF");
    
    radio_type = forceUHF ? 1 : 0;

    // Frekans limitlerini güncelle
    if (radio_type == 0) { // VHF
        freqLimits.trx_min_125 = DEFAULT_VHF_MINIMUM_FREQ;
        freqLimits.trx_max_125 = DEFAULT_VHF_MAXIMUM_FREQ;
        freqLimits.scn_min_125 = DEFAULT_VHF_SCAN_LOWER;
        freqLimits.scn_max_125 = DEFAULT_VHF_SCAN_UPPER;
    } else { // UHF
        freqLimits.trx_min_125 = DEFAULT_UHF_MINIMUM_FREQ;
        freqLimits.trx_max_125 = DEFAULT_UHF_MAXIMUM_FREQ;
        freqLimits.scn_min_125 = DEFAULT_UHF_SCAN_LOWER;
        freqLimits.scn_max_125 = DEFAULT_UHF_SCAN_UPPER;
    }
    // Sadece radio_type ve limitleri içeren config.json'u kaydet
    // Mevcut açılış mesajı ve TOT ayarı bu işlemden etkilenmez.
    saveConfig();

    // VFO (aktif kanal) frekansını güncelle
    if (radio_type == 0) { // VHF
        current_ch.frequency = 145600;
        current_ch.shift = 600;
    } else { // UHF
        current_ch.frequency = 433500;
        current_ch.shift = 7600;
    }
    saveCurrentChannel();

    // Banda özel APRS frekanslarını güncelle
    if (radio_type == 0) { // VHF
        aprs_freq_125 = DEFAULT_APRS_VHF_FREQ;
        iss_freq_125 = DEFAULT_ISS_APRS_FREQ;
    } else { // UHF
        aprs_freq_125 = DEFAULT_APRS_UHF_FREQ;
        iss_freq_125 = DEFAULT_ISS_APRS_UHF_FREQ;
    }
    // APRS ayarlarını kaydet (çağrı işareti, mesaj gibi diğer ayarlar korunur)
    saveAprs();

    // ----- YENİ EKLENEN BÖLÜM BAŞLANGICI -----
    // 'Boş' hafıza kanallarının (adı 'KNL ' olanlar) varsayılanlarını yeni banda göre güncelle
    for (int i = 0; i < 100; i++) {
        if (strcmp(memoryChannels[i].ChannelName, "KNL ") == 0) {
            if (radio_type == 0) { // VHF Varsayılanları
                memoryChannels[i].frequency125 = 11520; // 144.000 MHz
                memoryChannels[i].shift25 = 24;         // 600 kHz
            } else { // UHF Varsayılanları
                memoryChannels[i].frequency125 = 34400; // 430.000 MHz
                memoryChannels[i].shift25 = 304;        // 7600 kHz
            }
        }
    }
    // Yapılan kanal değişikliklerini kalıcı olarak kaydet
    saveMemoryChannels(); 
    // ----- YENİ EKLENEN BÖLÜM SONU -----
}
// --- LittleFS JSON Fonksiyonları ---

void initializeFileSystem() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS başlatılamadı!");
        return;
    }
    
    bool success = loadConfig() && loadAprs() && loadCurrentChannel() && loadMemoryChannels() && loadWifiConfig();

    if (!success) {
        Serial.println("Bir veya daha fazla ayar dosyası okunamadı, varsayılanlar oluşturuluyor...");
        createDefaultSettings(false, true); // İLK KURULUM: Her şeyi (WiFi dahil) tam sıfırla
    }
}

// GÜNCELLENDİ: Bu fonksiyon artık tam bir "fabrika ayarlarına sıfırlama" işlevi görüyor.
// `fullReset` parametresi eklendi. `true` ise kanallar ve WiFi dahil her şey silinir.
void createDefaultSettings(bool forceUHF, bool fullReset) {
    // Sadece tam sıfırlama durumunda genel kullanıcı ayarları silinir.
    if (fullReset) {
        greetingMessage = "ASELSAN 48XX/49XX";
        mycall = "N0CALL";
        APRS_Message = "ASELSAN 48XX/49XX - APRS TEST";
        lat = "4102.74N";
        lon = "02902.06E";
        APRS_Timeout = 3;
        myssid = 9;
        use_gps = true;
    }
    
    radio_type = forceUHF ? 1 : 0;
    tot_timer = 0; // TOT her zaman varsayılana döner

    // --- Aşağıdaki ayarlar hem tam sıfırlamada hem de bant değişiminde güncellenir ---

    if (radio_type == 0) { // VHF
        freqLimits.trx_min_125 = DEFAULT_VHF_MINIMUM_FREQ;
        freqLimits.trx_max_125 = DEFAULT_VHF_MAXIMUM_FREQ;
        freqLimits.scn_min_125 = DEFAULT_VHF_SCAN_LOWER;
        freqLimits.scn_max_125 = DEFAULT_VHF_SCAN_UPPER;
    } else { // UHF
        freqLimits.trx_min_125 = DEFAULT_UHF_MINIMUM_FREQ;
        freqLimits.trx_max_125 = DEFAULT_UHF_MAXIMUM_FREQ;
        freqLimits.scn_min_125 = DEFAULT_UHF_SCAN_LOWER;
        freqLimits.scn_max_125 = DEFAULT_UHF_SCAN_UPPER;
    }
    saveConfig();

    if (radio_type == 0) { // VHF
        aprs_freq_125 = DEFAULT_APRS_VHF_FREQ;
        iss_freq_125 = DEFAULT_ISS_APRS_FREQ;
    } else { // UHF
        aprs_freq_125 = DEFAULT_APRS_UHF_FREQ;
        iss_freq_125 = DEFAULT_ISS_APRS_UHF_FREQ;
    }
    saveAprs();

    if (radio_type == 0) { // VHF
        current_ch.frequency = 145600;
        current_ch.shift = 600;
    } else { // UHF
        current_ch.frequency = 433500;
        current_ch.shift = 7600;
    }
    current_ch.shift_dir = -1;
    current_ch.tone_pos = 8;
    current_ch.tone_enabled = 0;
    saveCurrentChannel();

    // Sadece tam sıfırlama durumunda kanallar ve WiFi sıfırlanır.
    if (fullReset) {
        memorych_t memch;
        memch.frequency125 = (radio_type == 0) ? 11520 : 34400;
        memch.shift25 = (radio_type == 0) ? 24 : 304;
        memch.tone_position = 0;
        memch.shift_dir = 0;
        memch.tone_enabled = 0;
        memch.power = 0;
        strcpy(memch.ChannelName, "KNL ");
        for (int ch = 0; ch < 100; ch++) {
            memoryChannels[ch] = memch;
        }
        saveMemoryChannels();
        createDefaultWifiConfig();
    }
}


// DEĞİŞİKLİK: Fonksiyon tüm genel ayarları ve limitleri okuyacak şekilde genişletildi.
bool loadConfig() {
    File configFile = LittleFS.open("/config.json", "r");
    if (!configFile) { return false; }
    StaticJsonDocument<1024> doc; // Daha fazla ayar için boyut artırıldı
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();
    if (error) { return false; }

    greetingMessage = doc["greeting"].as<String>();
    radio_type = doc["radio_type"];
    tot_timer = doc["tot_timer"];

    // Limitler de artık bu dosyadan okunuyor
    freqLimits.trx_min_125 = doc["trx_min_125"];
    freqLimits.trx_max_125 = doc["trx_max_125"];
    freqLimits.scn_min_125 = doc["scn_min_125"];
    freqLimits.scn_max_125 = doc["scn_max_125"];
    
    return true;
}

// DEĞİŞİKLİK: Fonksiyon tüm genel ayarları ve limitleri kaydedecek şekilde genişletildi.
void saveConfig() {
    StaticJsonDocument<1024> doc; // Daha fazla ayar için boyut artırıldı
    doc["greeting"] = greetingMessage;
    doc["radio_type"] = radio_type;
    doc["tot_timer"] = tot_timer;

    // Limitler de artık bu dosyaya kaydediliyor
    doc["trx_min_125"] = freqLimits.trx_min_125;
    doc["trx_max_125"] = freqLimits.trx_max_125;
    doc["scn_min_125"] = freqLimits.scn_min_125;
    doc["scn_max_125"] = freqLimits.scn_max_125;

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) { return; }
    serializeJson(doc, configFile);
    configFile.close();
}

bool loadAprs() {
    File aprsFile = LittleFS.open("/aprs.json", "r");
    if (!aprsFile) return false;
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, aprsFile);
    aprsFile.close();
    if (error) return false;
    mycall = doc["mycall"].as<String>();
    APRS_Message = doc["message"].as<String>();
    lat = doc["lat"].as<String>();
    lon = doc["lon"].as<String>();
    APRS_Timeout = doc["timeout"];
    myssid = doc["myssid"].as<int>();

    if (doc.containsKey("use_gps")) {
        use_gps = doc["use_gps"];
    } else {
        use_gps = true;
    }

    if (doc.containsKey("aprs_freq")) {
        aprs_freq_125 = doc["aprs_freq"];
    } else {
        aprs_freq_125 = (radio_type == 0) ? DEFAULT_APRS_VHF_FREQ : DEFAULT_APRS_UHF_FREQ;
    }
    
    if (doc.containsKey("iss_freq")) {
        iss_freq_125 = doc["iss_freq"];
    } else {
        iss_freq_125 = (radio_type == 0) ? DEFAULT_ISS_APRS_FREQ : DEFAULT_ISS_APRS_UHF_FREQ;
    }

    return true;
}

void saveAprs() {
    StaticJsonDocument<512> doc;
    doc["mycall"] = mycall;
    doc["message"] = APRS_Message;
    doc["lat"] = lat;
    doc["lon"] = lon;
    doc["timeout"] = APRS_Timeout;
    doc["myssid"] = (int)myssid;
    doc["use_gps"] = use_gps;
    doc["aprs_freq"] = aprs_freq_125;
    doc["iss_freq"] = iss_freq_125;
    File aprsFile = LittleFS.open("/aprs.json", "w");
    serializeJson(doc, aprsFile);
    aprsFile.close();
}

bool loadCurrentChannel() {
    File file = LittleFS.open("/current.json", "r");
    if (!file) return false;
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) return false;
    current_ch.frequency = doc["frequency"];
    current_ch.shift = doc["shift"];
    current_ch.shift_dir = doc["shift_dir"];
    current_ch.tone_pos = doc["tone_pos"];
    current_ch.tone_enabled = doc["tone_enabled"];
    return true;
}

void saveCurrentChannel() {
    StaticJsonDocument<256> doc;
    doc["frequency"] = current_ch.frequency;
    doc["shift"] = current_ch.shift;
    doc["shift_dir"] = current_ch.shift_dir;
    doc["tone_pos"] = current_ch.tone_pos;
    doc["tone_enabled"] = current_ch.tone_enabled;
    File file = LittleFS.open("/current.json", "w");
    serializeJson(doc, file);
    file.close();
}

// KALDIRILDI: loadFreqLimits() fonksiyonu silindi.
// KALDIRILDI: saveFreqLimits() fonksiyonu silindi.

// --- HATA DÜZELTME BAŞLANGICI: Geri Yükleme Fonksiyonu Güncellendi ---
// `loadMemoryChannels` fonksiyonu, artık sadece yeni ve doğru yedekleme formatını
// okuyacak şekilde güncellendi. Eski `sstp` alanı ile ilgili uyumluluk kodu
// tamamen kaldırıldı. Bu, yedek geri yükleme sırasındaki çökme sorununu çözer.
bool loadMemoryChannels() {
    File file = LittleFS.open("/channels.json", "r");
    if (!file) return false;

    // GÜNCELLENDİ: JSON nesne boyutu yeni alanlar için artırıldı (5 -> 7)
    const size_t capacity = JSON_ARRAY_SIZE(100) + 100 * JSON_OBJECT_SIZE(7) + 2048;
    DynamicJsonDocument doc(capacity);

    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) { return false; }

    JsonArray array = doc.as<JsonArray>();
    int i = 0;
    for (JsonObject obj : array) {
        if (i >= 100) break;
        memoryChannels[i].frequency125 = obj["f"];
        memoryChannels[i].shift25 = obj["s"];
        memoryChannels[i].tone_position = obj["t"];
        
        // Sadece yeni format okunuyor.
        memoryChannels[i].shift_dir = obj["sd"];
        memoryChannels[i].tone_enabled = obj["te"];
        memoryChannels[i].power = obj["p"];
        
        strncpy(memoryChannels[i].ChannelName, obj["n"], 4);
        memoryChannels[i].ChannelName[4] = '\0'; // Null-terminate
        i++;
    }
    return true;
}
// --- HATA DÜZELTME SONU ---

void saveMemoryChannels() {
    // GÜNCELLENDİ: JSON nesne boyutu yeni alanlar için artırıldı (5 -> 7)
    const size_t capacity = JSON_ARRAY_SIZE(100) + 100 * JSON_OBJECT_SIZE(7) + 2048;
    DynamicJsonDocument doc(capacity);
    JsonArray array = doc.to<JsonArray>();

    for (int i = 0; i < 100; i++) {
        JsonObject obj = array.createNestedObject();
        obj["f"] = memoryChannels[i].frequency125;
        obj["s"] = memoryChannels[i].shift25;
        obj["t"] = memoryChannels[i].tone_position;
        // GÜNCELLENDİ: SSTP yerine yeni alanlar kaydediliyor.
        obj["sd"] = memoryChannels[i].shift_dir;
        obj["te"] = memoryChannels[i].tone_enabled;
        obj["p"] = memoryChannels[i].power;
        obj["n"] = memoryChannels[i].ChannelName;
    }

    File file = LittleFS.open("/channels.json", "w");
    if (!file) { return; }
    serializeJson(doc, file);
    file.close();
}

bool loadWifiConfig() {
    File file = LittleFS.open("/wifi.json", "r");
    if (!file) {
        Serial.println("/wifi.json bulunamadı.");
        createDefaultWifiConfig(); 
        file = LittleFS.open("/wifi.json", "r");
        if (!file) return false;
    }
    
    DeserializationError error = deserializeJson(wifiConfig, file);
    file.close();
    
    if (error) {
        Serial.print("deserializeJson() /wifi.json okurken başarısız: ");
        Serial.println(error.c_str());
        return false;
    }
    
    return true;
}

void saveWifiConfig() {
    File file = LittleFS.open("/wifi.json", "w");
    if (!file) {
        Serial.println("/wifi.json yazmak için açılamadı.");
        return;
    }
    
    if (serializeJson(wifiConfig, file) == 0) {
        Serial.println("/wifi.json'a yazma başarısız.");
    }
    
    file.close();
}

void createDefaultWifiConfig() {
    Serial.println("Varsayılan /wifi.json oluşturuluyor.");
    wifiConfig["wifi_enabled"] = true;
    
    JsonObject web_auth = wifiConfig.createNestedObject("web_auth");
    web_auth["user"] = "admin";
    web_auth["pass"] = "aselsan";

    JsonObject ap_settings = wifiConfig.createNestedObject("ap_settings");
    ap_settings["ssid"] = "ASELSAN_4822_ESP32";
    ap_settings["pass"] = "12345678";
    ap_settings["ip"] = "192.168.4.1";

    wifiConfig.createNestedArray("saved_networks");

    saveWifiConfig();
}

// --- APRS Fonksiyonları ---

void set_nada_1200(void) {
  stopDacTone();
}

void set_nada_2400(void) {
  stopDacTone();
}

void set_nada(bool nada) {
  // if(nada) set_nada_1200();
  // else set_nada_2400();
}

void calc_crc(bool in_bit) {
  unsigned short xor_in;
  xor_in = crc ^ in_bit;
  crc >>= 1;
  if(xor_in & 0x01) crc ^= 0x8408;
}

void send_crc(void) {
  unsigned char crc_lo = crc ^ 0xff;
  unsigned char crc_hi = (crc >> 8) ^ 0xff;
  send_char_NRZI(crc_lo, HIGH);
  send_char_NRZI(crc_hi, HIGH);
}

void send_header(void) {
  char temp;
  temp = strlen(dest);
  for(int j=0; j<temp; j++) send_char_NRZI(dest[j] << 1, HIGH);
  if(temp < 6) { for(int j=0; j<(6 - temp); j++) send_char_NRZI(' ' << 1, HIGH); }
  send_char_NRZI('0' << 1, HIGH);

  temp = mycall.length();
  for(int j=0; j<temp; j++) send_char_NRZI(mycall[j] << 1, HIGH);
  if(temp < 6) { for(int j=0; j<(6 - temp); j++) send_char_NRZI(' ' << 1, HIGH); }
  send_char_NRZI((myssid + '0') << 1, HIGH);

  temp = strlen(digi);
  for(int j=0; j<temp; j++) send_char_NRZI(digi[j] << 1, HIGH);
  if(temp < 6) { for(int j=0; j<(6 - temp); j++) send_char_NRZI(' ' << 1, HIGH); }
  send_char_NRZI(((digissid + '0') << 1) + 1, HIGH);

  send_char_NRZI(_CTRL_ID, HIGH);
  send_char_NRZI(_PID, HIGH);
}

void send_payload(char type) {
  if(type == _FIXPOS) {
    send_char_NRZI(_DT_POS, HIGH);
    send_string_len(lat, lat.length());
    send_char_NRZI(sym_ovl, HIGH);
    send_string_len(lon, lon.length());
    send_char_NRZI(sym_tab, HIGH);
  }
  else if(type == _STATUS) {
    send_char_NRZI(_DT_STATUS, HIGH);
    send_string_len(APRS_Message, APRS_Message.length());
  }
  else if(type == _FIXPOS_STATUS) {
    send_char_NRZI(_DT_POS, HIGH);
    send_string_len(lat, lat.length());
    send_char_NRZI(sym_ovl, HIGH);
    send_string_len(lon, lon.length());
    send_char_NRZI(sym_tab, HIGH);
    send_char_NRZI(' ', HIGH);
    send_string_len(APRS_Message, APRS_Message.length());
  }
}

void send_char_NRZI(unsigned char in_byte, bool enBitStuff) {
  bool bits;
  for(int i = 0; i < 8; i++) {
    bits = in_byte & 0x01;
    calc_crc(bits);
    if(bits) {
      set_nada(nada);
      bit_stuff++;
      if((enBitStuff) && (bit_stuff == 5)) {
        nada ^= 1;
        set_nada(nada);
        bit_stuff = 0;
      }
    }
    else {
      nada ^= 1;
      set_nada(nada);
      bit_stuff = 0;
    }
    in_byte >>= 1;
  }
}

void send_string_len(String in_string, int len) {
  for(int j=0; j<len; j++) send_char_NRZI(in_string[j], HIGH);
}

void send_flag(unsigned char flag_len) {
  for(int j=0; j<flag_len; j++) send_char_NRZI(_FLAG, LOW);
}

void send_packet(char packet_type, uint32_t frequency) {
   strcpy(FRQ_old,FRQ);
   current_ch.shift_dir = noSHIFT;
   TRX_MODE = TX;
   
   stopDacTone();

   numberToFrequency(frequency,FRQ);
   validFRQ = Calculate_Frequency(FRQ);
   write_FRQ(current_ch.frequency);
   writeFRQToLcd(FRQ);
   digitalWrite(PTT_OUTPUT_PIN,HIGH);
   delay(300);

  send_flag(100);
  crc = 0xffff;
  send_header();
  send_payload(packet_type);
  send_crc();
  send_flag(3);

  digitalWrite(PTT_OUTPUT_PIN,LOW);
  TRX_MODE = RX;
  strcpy(FRQ,FRQ_old);
  validFRQ = Calculate_Frequency(FRQ);
  write_FRQ(current_ch.frequency);

  SetTone(current_ch.tone_enabled);
}

void getGPSData() {
  unsigned long start = millis();
  do {
    while (GPS_Serial.available())
      gps.encode(GPS_Serial.read());
  } while (millis() - start < 1000);

  if (gps.location.isValid()) {
    double lat_decimal = gps.location.lat();
    double lon_decimal = gps.location.lng();
    
    char lat_str[10];
    int lat_deg = int(abs(lat_decimal));
    double lat_min = (abs(lat_decimal) - lat_deg) * 60.0;
    char lat_hem = (lat_decimal >= 0) ? 'N' : 'S';
    sprintf(lat_str, "%02d%05.2f%c", lat_deg, lat_min, lat_hem);
    lat = String(lat_str);

    char lon_str[11];
    int lon_deg = int(abs(lon_decimal));
    double lon_min = (abs(lon_decimal) - lon_deg) * 60.0;
    char lon_hem = (lon_decimal >= 0) ? 'E' : 'W';
    sprintf(lon_str, "%03d%05.2f%c", lon_deg, lon_min, lon_hem);
    lon = String(lon_str);
    
    Serialprint("Lat: %s, Lon: %s\r\n", lat.c_str(), lon.c_str());
    saveAprs();
  } else {
    Serialprint("GPS verisi alinamadi.\r\n");
  }
}

boolean Calculate_Frequency (char mFRQ[9]) {
    if(strlen(mFRQ) < 7 || mFRQ[3] != '.') return false;

    current_ch.frequency = ((mFRQ[0] - 48) * 100000L) + ((mFRQ[1] - 48) * 10000L) + ((mFRQ[2] - 48) * 1000) + ((mFRQ[4] - 48) * 100) + ((mFRQ[5] - 48) * 10) + (mFRQ[6] - 48);

    uint32_t min_freq_khz = freqLimits.trx_min_125 * 12.5;
    uint32_t max_freq_khz = freqLimits.trx_max_125 * 12.5;

    if (current_ch.frequency >= min_freq_khz && current_ch.frequency <= max_freq_khz) {
        return true;
    }

    Alert_Tone(ERR_tone);
    return false;
}

void numberToFrequency(uint32_t Freq, char *rFRQ) {
  sprintf(rFRQ, "%03lu.%03lu ", Freq / 1000, Freq % 1000);
  rFRQ[7] = ' ';
  rFRQ[8] = 0;
}

void StoreFrequency(char mCHNL[9], char mFRQ[9]) {
    // Gelen karakter dizisini tam sayıya çevir. Artık 1, 2 veya 3 haneli olabilir.
    int ChannelNumber_UI = atoi(mCHNL); // Kullanıcının girdiği 1-100 arası numara.

    // Gelen kanal numarasını kontrol et (1-100 aralığında olmalı).
    if (ChannelNumber_UI < 1 || ChannelNumber_UI > 100) {
        Alert_Tone(ERR_tone);
        return;
    }

    // Dizi indeksi için 1 çıkar (0-99 aralığı).
    int ChannelIndex = ChannelNumber_UI - 1;

    // ##### DEĞİŞİKLİK BAŞLANGICI: Otomatik Kanal Adı Atama #####
    // Tuş takımı ile kaydedilen kanala, web arayüzünde görünmesi için
    // kanal numarasını otomatik olarak isim olarak ata.
    snprintf(memoryChannels[ChannelIndex].ChannelName, 5, "%d", ChannelNumber_UI);
    // ##### DEĞİŞİKLİK SONU #####

    Calculate_Frequency(mFRQ);

    memoryChannels[ChannelIndex].frequency125 = (uint16_t)(current_ch.frequency / 12.5);
    memoryChannels[ChannelIndex].shift25 = (uint16_t)(current_ch.shift / 25);
    memoryChannels[ChannelIndex].tone_position = current_ch.tone_pos;
    
    // shift_dir'i -1,0,1'den 2,0,1'e eşle. (2:-, 0:Simplex, 1:+)
    uint8_t dir_to_store = (current_ch.shift_dir == 0) ? 0 : ((current_ch.shift_dir == 1) ? 1 : 2);
    memoryChannels[ChannelIndex].shift_dir = dir_to_store;
    memoryChannels[ChannelIndex].tone_enabled = current_ch.tone_enabled;
    memoryChannels[ChannelIndex].power = RF_POWER_STATE; // O anki global güç ayarını kaydet

    saveMemoryChannels();
}

void GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg) {
      if (dbg)
      {
        Serialprint("{c:%d,f:%d,s:%d,t:%d}",channel_number,memoryChannels[channel_number].frequency125,memoryChannels[channel_number].shift25,memoryChannels[channel_number].tone_position);
      }
}

void GetMemoryChannel(char mCHNL[9]) {
    // Gelen karakter dizisini tam sayıya çevir. Artık 1, 2 veya 3 haneli olabilir.
    int ChannelNumber_UI = atoi(mCHNL); // Kullanıcının girdiği 1-100 arası numara.

    // Gelen kanal numarasını kontrol et (1-100 aralığında olmalı).
    if (ChannelNumber_UI < 1 || ChannelNumber_UI > 100) {
        Alert_Tone(ERR_tone);
        return;
    }
    
    // Dizi indeksi için 1 çıkar (0-99 aralığı).
    int ChannelIndex = ChannelNumber_UI - 1;

    memorych_t l_memorych = memoryChannels[ChannelIndex];
    current_ch.frequency = l_memorych.frequency125 * 12.5;
    current_ch.shift     = l_memorych.shift25 * 25;
    current_ch.tone_pos  = l_memorych.tone_position;
    
    // shift_dir'i 0,1,2'den 0,1,-1'e geri eşle. (0:Simplex, 1:+, 2:-)
    current_ch.shift_dir = (l_memorych.shift_dir == 0) ? 0 : ((l_memorych.shift_dir == 1) ? 1 : -1);
    current_ch.tone_enabled = l_memorych.tone_enabled;
    
    // Kayıtlı güç ayarını yükle ve donanımda anında uygula
    RF_POWER_STATE = l_memorych.power;
    SetRFPower();

    numberToFrequency(current_ch.frequency, FRQ);
}

void PrintMenu()
{
  Serialprint("ASELSAN 48xx -  Kit (ESP32 v%d.%d-%s) \n\r",SW_MAJOR,SW_MINOR, __DATE__);
  Serialprint("-------------------------\n\r");
  Serialprint("Y-Yardim\n\r");
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
  Serialprint("Yardim Menusu\r\n=====================\r\n");
  if (komut=='\n')
  {
    Serialprint("Yardim almak istediginiz komutu belirtiniz...\r\n");
    Serialprint("Ornek : Y C  veya Y F  veya Y A \r\n");
  } else if (komut == 'C')
  {
    Serialprint("Bu komut cihazinizi UHF ya da VHF olarak programlamanizi saglar.\r\n");
    Serialprint("Kullanimi : \r\n      C V : VHF yap\r\n      C U : UHF yap");
  }
  else if (komut == 'A')
  {
    Serialprint("Kullanimi : \r\n      A [MESAJ]   Ornek:  A TA7W  (maksimum 6 karakter)\r\n");
  } else if (komut == 'H')
  {
    Serialprint("Kullanimi:   H [Kanal_No #2] [isim #6] [Frekans #6] [Shift #5] [Ton #4] \r\n");
    Serialprint("Ornek:       H 01 ROLE-0 145600 +0600 0885 \r\n");
  } else if (komut == 'T')
  {
    Serialprint("Kullanimi:   T [2 basamakli olarak Sure (dakika) (00-99)] (00 iptal demektir) \r\n");
    Serialprint("Ornek:       T 300\r\n");
  } else if (komut == 'M')
  {
    Serialprint("Kullanimi:   M [MESAJ (maksimum 30 karakter) \r\n");
    Serialprint("Ornek:       M Merhaba, Ben BARIS DINC - OH2UDS  \r\n");
  }
}

// GÜNCELLENDİ: Hata düzeltmesi. Artık güvenli olan 'applyBandSpecificDefaults' fonksiyonunu çağırıyor.
void commandCevrim(char komut)
{
  if (komut == 'V')
    {
      Serialprint("VHF OK\r\n");
      applyBandSpecificDefaults(false); // VHF için güvenli bant değiştirme
    }
  if (komut == 'U')
    {
      Serialprint("UHF OK\r\n");
      applyBandSpecificDefaults(true); // UHF için güvenli bant değiştirme
    }
}

void commandAcilis()
{
  Serial.print(commandString.substring(2,8));
  greetingMessage=commandString.substring(2,8); // DEĞİŞİKLİK: CALLSIGN -> greetingMessage
  saveConfig();
}

void commandAyarDok()
{
  Serialprint("\r\nCD{");
  Serialprint("cs:\"%s\",smj:%d,smn:%d,r:%d",greetingMessage.c_str(), SW_MAJOR,SW_MINOR,radio_type); // DEĞİŞİKLİK: CALLSIGN -> greetingMessage
  Serialprint("}\r\n");
}

void commandHafizaDok()
{
  Serialprint("\r\nMD[");
  for (int ch=0;ch<100;ch++)
    {
      GetPrintMemoryChannelInfo(ch,true);
      if (ch < 99) Serialprint(",");
    }
    Serialprint("]\r\n");
}

void commandAPRSSure()
{
  Serialprint(" OK\r\n");
  APRS_Timeout = commandString.substring(2,4).toInt();
  saveAprs();
}

void commandAPRSMesaj()
{
  Serial.print(commandString.substring(2,30));
  APRS_Message = commandString.substring(2,30);
  Serial.print(APRS_Message);
  Serialprint(" OK\r\n");
  saveAprs();
}

void getFSData()
{
    Serialprint("currentch %lu %d %d %d %d\r\n", current_ch.frequency, current_ch.shift, current_ch.shift_dir, current_ch.tone_pos, current_ch.tone_enabled);
    File file = LittleFS.open("/config.json", "r");
    if (file) {
        Serial.println("--- config.json ---");
        while(file.available()){
            Serial.write(file.read());
        }
        Serial.println("\n--------------------");
        file.close();
    }
}

void commandAPRSmycall()
{
  Serial.print(commandString.substring(2,8));
  Serialprint(" OK\r\n");
  mycall = commandString.substring(2,8);
  saveAprs();
}

void commandTogglePTT()
{
  pttToggler = !pttToggler;
  if (pttToggler) Serialprint("TX\r\n");
  else Serialprint("RX\r\n");
}

int readColumn()
{
    Wire.beginTransmission(B0100000);
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
    int r = Wire.read();
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

void StoreSpecialFrequency(char mCHNL[9], char mFRQ[9])
{
    uint16_t ChannelNumber = 0;
    if(strlen(mCHNL) >= 1) { // 1 haneli de olabilir
      ChannelNumber = atoi(mCHNL);
    }

    if (Calculate_Frequency(mFRQ))
    {
        bool configChanged = false;
        bool aprsSettingsChanged = false;

        if (ChannelNumber == 101) { freqLimits.trx_min_125 = current_ch.frequency / 12.5; configChanged = true;}
        if (ChannelNumber == 151) { freqLimits.trx_min_125 = DEFAULT_VHF_MINIMUM_FREQ; configChanged = true; }
        if (ChannelNumber == 102) { freqLimits.trx_max_125 = current_ch.frequency / 12.5; configChanged = true; }
        if (ChannelNumber == 152) { freqLimits.trx_max_125 = DEFAULT_VHF_MAXIMUM_FREQ; configChanged = true; }
        if (ChannelNumber == 201) { freqLimits.scn_min_125 = current_ch.frequency / 12.5; configChanged = true; }
        if (ChannelNumber == 202) { freqLimits.scn_max_125 = current_ch.frequency / 12.5; configChanged = true; }
        
        if (ChannelNumber == 301) { aprs_freq_125 = current_ch.frequency / 12.5; aprsSettingsChanged = true; }
        if (ChannelNumber == 302) { iss_freq_125 = current_ch.frequency / 12.5; aprsSettingsChanged = true; }

        if (configChanged) saveConfig();
        if (aprsSettingsChanged) saveAprs();

        if (ChannelNumber == 600) { APRS_Timeout = current_ch.frequency % 1000; saveAprs(); }
    }

    if (ChannelNumber == 996) {
        char ipBuffer[30]; // "IP 192.168.255.255" gibi bir metin için yeterli alan
        snprintf(ipBuffer, sizeof(ipBuffer), "IP %s", currentIP.c_str());
        ipToDisplay = ipBuffer; // Ana döngünün IP'yi ekrana yazdırması için bayrağı ayarla
        Alert_Tone(SUCC_tone);  // Komutun alındığına dair sesli onay
        return; // Fonksiyondan hemen çık, frekansla ilgili işlemleri yapma
    }

    if (ChannelNumber == 997) {
        loadWifiConfig();
        bool wifi_status = wifiConfig["wifi_enabled"];
        wifiConfig["wifi_enabled"] = !wifi_status;
        saveWifiConfig();

        if (!wifi_status) {
            writeToLcd("WIFI ON ");
        } else {
            writeToLcd("WIFI OFF");
        }
        delay(2000);
        ESP.restart();
    }
    // GÜNCELLENDİ: Artık createDefaultSettings yerine sadece banda özel ayarları değiştiren fonksiyon çağrılıyor.
    if (ChannelNumber == 998) { applyBandSpecificDefaults(true); } // UHF
    if (ChannelNumber == 999) { applyBandSpecificDefaults(false);} // VHF

    if (ChannelNumber == 998 || ChannelNumber == 999)
    {
        // applyBandSpecificDefaults içinde zaten ayarlar kaydedilip güncellendiği için
        // burada tekrar yüklemeye gerek yok. Sadece ekranı güncellemek yeterli.
        numberToFrequency(current_ch.frequency, FRQ);
        strcpy(FRQ_old, FRQ);
    }
    Alert_Tone(SUCC_tone);
}


void startScan()
{
  unsigned long scan_frequency = freqLimits.scn_min_125*12.5;
  int8_t frq_step = 25;
  while (1)
  {
        scan_frequency += frq_step;
        if (scan_frequency >= (freqLimits.scn_max_125*12.5)) scan_frequency = (freqLimits.scn_min_125*12.5);
        else if (scan_frequency <= (freqLimits.scn_min_125*12.5)) scan_frequency = (freqLimits.scn_max_125*12.5);

        numberToFrequency(scan_frequency,FRQ);
        validFRQ = Calculate_Frequency(FRQ);
        write_FRQ(current_ch.frequency);
        writeFRQToLcd(FRQ);
        delay(100);
        CHANNEL_BUSY = digitalRead(SQL_ACTIVE);
        if (CHANNEL_BUSY == 0) break;
        if (readColumn() != 0) break;
  }

}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    commandString += inChar;
    if (inChar == '\n') {
      commandComplete = true;
    }
  }
}

