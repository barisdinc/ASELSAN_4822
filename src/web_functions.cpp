#include "declarations.h"
#include "esp_task_wdt.h" // RESET SORUNU İÇİN EKLENDİ
#include <Update.h> // OTA için eklendi

// Cihazı Access Point modunda başlatan fonksiyon
void startAPMode() {
  Serial.println("\nAP modunda başlatılıyor...");
  const char* ssid = wifiConfig["ap_settings"]["ssid"];
  const char* pass = wifiConfig["ap_settings"]["pass"];
  
  WiFi.softAP(ssid, pass);
  delay(100); // AP'nin başlaması için kısa bir bekleme
  Serial.print("AP IP adresi: ");
  Serial.println(WiFi.softAPIP());
  // ISTEK: AP IP adresini al ve gösterilmek üzere işaretle
  currentIP = WiFi.softAPIP().toString();
  char ipBuffer[30]; // "AP 192.168.255.255       " için yeterli alan
  snprintf(ipBuffer, sizeof(ipBuffer), "AP %s       ", currentIP.c_str());
  ipToDisplay = ipBuffer; // ISTEK: Flag'i ayarla, loop() içinde gösterilecek.
}

// WiFi'ye bağlanmayı deneyen ana fonksiyon
// GÜNCELLENDİ: Bağlantıdan önce çevredeki ağlar taranır ve sadece bulunan kayıtlı ağlara bağlanmaya çalışılır.
void connectToWiFi() {
  // GÜNCELLEME: Bu fonksiyon artık sadece WiFi'nin etkin olduğu durumda çağrıldığı için
  // başlangıçtaki `wifi_enabled` kontrolü kaldırılmıştır. Fonksiyonun asıl amacı olan
  // ağa bağlanma veya bağlanamazsa AP moduna geçme mantığı korunmaktadır.
  
  WiFi.mode(WIFI_STA); // Cihazı istasyon moduna al
  WiFi.disconnect(); // Önceki bağlantıları temizle
  delay(100);

  JsonArray savedNetworks = wifiConfig["saved_networks"].as<JsonArray>();
  if (savedNetworks.size() == 0) {
      Serial.println("Kayitli WiFi ağı yok.");
      startAPMode();
      return;
  }

  Serial.println("Kayitli aglari bulmak icin tarama yapiliyor...");
  // DÜZELTME: Gizli ağları da taramaya dahil et (ikinci parametre 'true').
  int n = WiFi.scanNetworks(false, true);
  Serial.printf("%d adet ağ bulundu.\n", n);
  
  // Bulunan ağların SSID'lerini bir listede topla
  std::vector<String> found_ssids;
  for (int i = 0; i < n; i++) {
    found_ssids.push_back(WiFi.SSID(i));
  }
  WiFi.scanDelete(); // Tarama sonuçlarını temizle, hafızayı boşalt

  Serial.println("Kayitli ve aktif olan aglara baglaniliyor...");
  
  // JSON'dan ağları bir vektöre kopyalayıp önceliğe göre sırala
  struct Network { String ssid; String pass; int priority; bool hidden; };
  std::vector<Network> networks;
  for(JsonObject net : savedNetworks) {
      // JSON'da priority olmayabilir, kontrol et
      int priority = net.containsKey("priority") ? net["priority"].as<int>() : 99;
      networks.push_back({net["ssid"], net["pass"], priority, net["hidden"]});
  }
  std::sort(networks.begin(), networks.end(), [](const Network& a, const Network& b) {
      return a.priority < b.priority;
  });

  bool connected = false;
  for (const auto& net : networks) {
    // Kayıtlı ağ, tarama sonucunda bulunan ağlar arasında var mı diye kontrol et
    bool is_available = false;
    for (const auto& found_ssid : found_ssids) {
        if (found_ssid == net.ssid) {
            is_available = true;
            break;
        }
    }
    
    // GÜNCELLENDİ: Eğer ağ gizli olarak kayıtlıysa ve taramada bulunamadıysa bile bağlanmayı dene.
    // Bazen gizli ağlar taramada görünmeyebilir.
    if (net.hidden && !is_available) {
        Serial.printf("'%s' gizli ağı taramada bulunamadı, yine de deneniyor...\n", net.ssid.c_str());
        is_available = true; 
    }

    if (!is_available) {
        Serial.printf("'%s' ağı çevrede bulunamadı, atlanıyor.\n", net.ssid.c_str());
        continue; // Ağ listede yoksa bu ağı atla
    }

    Serial.print(net.ssid);
    Serial.print(" ağına bağlanılıyor...");
    
    if (net.hidden) {
        WiFi.begin(net.ssid.c_str(), net.pass.c_str(), 0, NULL, true);
    } else {
        WiFi.begin(net.ssid.c_str(), net.pass.c_str());
    }

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) { // 10 saniye bekle
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nBağlantı başarılı!");
      Serial.print("IP Adresi: ");
      Serial.println(WiFi.localIP());
      connected = true;
      // ISTEK: STA IP adresini al ve gösterilmek üzere işaretle
      currentIP = WiFi.localIP().toString();
      char ipBuffer[30]; // "IP 192.168.255.255       " için yeterli alan
      snprintf(ipBuffer, sizeof(ipBuffer), "IP %s       ", currentIP.c_str());
      ipToDisplay = ipBuffer; // ISTEK: Flag'i ayarla, loop() içinde gösterilecek.
      break; 
    } else {
      Serial.println("\nBağlantı başarısız.");
      WiFi.disconnect();
    }
  }

  if (!connected) {
    Serial.println("Kayitli ve aktif olan WiFi ağlarina bağlanilamadi.");
    startAPMode();
  }
}

// Web sunucusunu kuran ve başlatan fonksiyon
void setupWebServer() {
    const char* user = wifiConfig["web_auth"]["user"];
    const char* pass = wifiConfig["web_auth"]["pass"];

    // Ana sayfa
    server.on("/", HTTP_GET, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        request->send(LittleFS, "/index.html", "text/html", false);
    });

    // --- YENİ: MERKEZİ AYARLAR API ENDPOINT'İ ---

    // GET /api/settings/config: Tüm genel ayarları ve limitleri gönderir
    server.on("/api/settings/config", HTTP_GET, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();

        DynamicJsonDocument json(1024);
        json["greeting"] = greetingMessage;
        json["radio_type"] = radio_type;
        json["tot_timer"] = tot_timer;
        json["ui_theme"] = ui_theme; // YENİ: Kayıtlı UI temasını ekle

        // Frekansları _125 formatından MHz formatına çevirerek ekle
        char freq_buffer[10];
        dtostrf(((float)freqLimits.trx_min_125 * 12.5) / 1000.0, 7, 3, freq_buffer);
        json["trx_min"] = freq_buffer;
        dtostrf(((float)freqLimits.trx_max_125 * 12.5) / 1000.0, 7, 3, freq_buffer);
        json["trx_max"] = freq_buffer;
        dtostrf(((float)freqLimits.scn_min_125 * 12.5) / 1000.0, 7, 3, freq_buffer);
        json["scn_min"] = freq_buffer;
        dtostrf(((float)freqLimits.scn_max_125 * 12.5) / 1000.0, 7, 3, freq_buffer);
        json["scn_max"] = freq_buffer;
        
        String response;
        serializeJson(json, response);
        request->send(200, "application/json", response);
    });

    // POST /api/settings/config: Yeni genel ayarları kaydeder
    AsyncCallbackJsonWebHandler* settingsConfigHandler = new AsyncCallbackJsonWebHandler("/api/settings/config", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        JsonObject obj = json.as<JsonObject>();
        
        byte old_radio_type = radio_type;
        byte new_radio_type = obj["radio_type"];

        // Ayarları global değişkenlere al
        greetingMessage = obj["greeting"].as<String>();
        radio_type = new_radio_type;
        tot_timer = obj["tot_timer"];
        ui_theme = obj["ui_theme"].as<String>(); // YENİ: Gelen UI temasını al
        freqLimits.trx_min_125 = (uint16_t)((obj["trx_min"].as<float>() * 1000.0) / 12.5);
        freqLimits.trx_max_125 = (uint16_t)((obj["trx_max"].as<float>() * 1000.0) / 12.5);
        freqLimits.scn_min_125 = (uint16_t)((obj["scn_min"].as<float>() * 1000.0) / 12.5);
        freqLimits.scn_max_125 = (uint16_t)((obj["scn_max"].as<float>() * 1000.0) / 12.5);

        // Önce tüm genel ayarları (yeni bant dahil) kaydet
        saveConfig();

        // GÜNCELLENDİ: Bant değiştirildiyse, artık yeniden başlatma yok.
        // Sadece banda özel ayarları (VFO, APRS frekansları vb.) güncelleyen
        // güvenli fonksiyon çağrılıyor. Kanallar, WiFi, TOT gibi ayarlar korunuyor.
        if (old_radio_type != new_radio_type) {
            Serial.println("Bant değiştirildi, veriler korunarak guncelleniyor...");
            applyBandSpecificDefaults(new_radio_type == 1); 

            // Değişikliğin anında telsize yansıması için VFO'yu güncelle
            numberToFrequency(current_ch.frequency, FRQ);
            write_FRQ(current_ch.frequency);
            writeFRQToLcd(FRQ);
        }

        // Başarılı yanıtı gönder (yeniden başlatma yok)
        request->send(200, "application/json", "{\"success\":true}");
    });
    server.addHandler(settingsConfigHandler);


    // --- MEVCUT APRS API ENDPOINT'LERİ ---

    // GET /api/aprs/config: Mevcut APRS ayarlarını gönderir
    server.on("/api/aprs/config", HTTP_GET, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();

        DynamicJsonDocument json(1024);
        json["mycall"] = mycall;
        json["myssid"] = (int)myssid;
        json["timeout"] = APRS_Timeout;
        json["message"] = APRS_Message;
        json["lat"] = lat;
        json["lon"] = lon;
        json["use_gps"] = use_gps;
        
        char freq_buffer[10];
        dtostrf(((float)aprs_freq_125 * 12.5) / 1000.0, 7, 3, freq_buffer);
        json["aprs_freq"] = freq_buffer;

        dtostrf(((float)iss_freq_125 * 12.5) / 1000.0, 7, 3, freq_buffer);
        json["iss_freq"] = freq_buffer;
        
        String response;
        serializeJson(json, response);
        request->send(200, "application/json", response);
    });

    // POST /api/aprs/config: Yeni APRS ayarlarını kaydeder
    AsyncCallbackJsonWebHandler* aprsConfigHandler = new AsyncCallbackJsonWebHandler("/api/aprs/config", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        JsonObject obj = json.as<JsonObject>();

        mycall = obj["mycall"].as<String>();
        myssid = obj["myssid"].as<int>();
        APRS_Timeout = obj["timeout"].as<unsigned int>();
        APRS_Message = obj["message"].as<String>();
        lat = obj["lat"].as<String>();
        lon = obj["lon"].as<String>();
        use_gps = obj["use_gps"].as<bool>();
        
        float aprs_freq_mhz = obj["aprs_freq"].as<float>();
        aprs_freq_125 = (uint16_t)((aprs_freq_mhz * 1000.0) / 12.5);

        float iss_freq_mhz = obj["iss_freq"].as<float>();
        iss_freq_125 = (uint16_t)((iss_freq_mhz * 1000.0) / 12.5);

        saveAprs(); // Ayarları dosyaya kaydet
        
        // Değişikliği anında LCD'ye yansıt (örn. APRS simgesi)
        writeFRQToLcd(FRQ); 

        request->send(200, "application/json", "{\"success\":true}");
    });
    server.addHandler(aprsConfigHandler);

    // GET /api/gps/now: Anlık GPS konumunu alır ve gönderir
    server.on("/api/gps/now", HTTP_GET, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        getGPSData(); // Global lat/lon değişkenlerini günceller

        DynamicJsonDocument json(256);
        json["lat"] = lat;
        json["lon"] = lon;
        
        String response;
        serializeJson(json, response);
        request->send(200, "application/json", response);
    });

    // POST /api/aprs/send_beacon: Manuel olarak beacon gönderir
    server.on("/api/aprs/send_beacon", HTTP_POST, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        if (APRS_Timeout > 0) {
             send_packet(_FIXPOS_STATUS, (aprs_freq_125*12.5));
             request->send(200, "application/json", "{\"success\":true}");
        } else {
             request->send(400, "application/json", "{\"success\":false, \"error\":\"APRS kapalı.\"}");
        }
    });

    // --- KANAL YÖNETİMİ API ENDPOINT'LERİ (GÜNCELLENDİ) ---

    // GET /api/channels/get_all: Sadece dolu kanalları JSON olarak döndürür
    server.on("/api/channels/get_all", HTTP_GET, [user, pass](AsyncWebServerRequest *request) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();

        const size_t capacity = JSON_ARRAY_SIZE(100) + 100 * JSON_OBJECT_SIZE(8);
        DynamicJsonDocument doc(capacity);
        JsonArray channelsArray = doc.to<JsonArray>();

        for (int i = 0; i < 100; i++) {
            // GÜNCELLENDİ: Sadece adı "KNL " olmayan, yani "dolu" olan kanalları listeye ekle.
            if (strcmp(memoryChannels[i].ChannelName, "KNL ") != 0) {
                JsonObject ch = channelsArray.createNestedObject();
                ch["no"] = i + 1;
                ch["name"] = memoryChannels[i].ChannelName;

                char freq_buffer[10];
                dtostrf(((float)memoryChannels[i].frequency125 * 12.5) / 1000.0, 7, 3, freq_buffer);
                ch["freq"] = freq_buffer;
                
                ch["shift"] = (unsigned long)memoryChannels[i].shift25 * 25;
                
                if (memoryChannels[i].tone_enabled) {
                    ch["tone"] = ctcss_tone_list[memoryChannels[i].tone_position];
                } else {
                    ch["tone"] = 0;
                }
                ch["tone_enabled"] = memoryChannels[i].tone_enabled;

                if (memoryChannels[i].shift_dir == 0) ch["shift_dir"] = "S";
                else if (memoryChannels[i].shift_dir == 1) ch["shift_dir"] = "+";
                else ch["shift_dir"] = "-";

                ch["power"] = (memoryChannels[i].power == 0) ? "Yüksek" : "Düşük";
            }
        }

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // POST /api/channels/update: Tek bir kanalı günceller
    AsyncCallbackJsonWebHandler* updateChannelHandler = new AsyncCallbackJsonWebHandler("/api/channels/update", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();

        JsonObject obj = json.as<JsonObject>();
        int ch_no = obj["no"]; // Arayüzden 1-100 aralığında gelir
        if (ch_no < 1 || ch_no > 100) {
            request->send(400, "application/json", "{\"success\":false, \"error\":\"Geçersiz kanal numarası.\"}");
            return;
        }
        int index = ch_no - 1; // Dizi indeksi için 1 çıkar (0-99)

        strncpy(memoryChannels[index].ChannelName, obj["name"], 4);
        memoryChannels[index].ChannelName[4] = '\0';

        float freq_mhz = obj["freq"].as<float>();
        memoryChannels[index].frequency125 = (uint16_t)((freq_mhz * 1000.0) / 12.5);
        
        unsigned long shift_khz = obj["shift"];
        memoryChannels[index].shift25 = (uint16_t)(shift_khz / 25);
        
        memoryChannels[index].tone_enabled = obj["tone_enabled"];
        float tone_hz = obj["tone"].as<float>();
        if (tone_hz > 0) {
            for(int i = 0; i < TOTAL_TONES; i++) {
                if (abs(ctcss_tone_list[i] - tone_hz) < 0.01) {
                    memoryChannels[index].tone_position = i;
                    break;
                }
            }
        } else {
            memoryChannels[index].tone_position = 0;
        }

        String shift_dir_str = obj["shift_dir"];
        if (shift_dir_str == "S") memoryChannels[index].shift_dir = 0;
        else if (shift_dir_str == "+") memoryChannels[index].shift_dir = 1;
        else memoryChannels[index].shift_dir = 2;

        String power_str = obj["power"];
        memoryChannels[index].power = (power_str == "Yüksek") ? 0 : 1;

        saveMemoryChannels();
        request->send(200, "application/json", "{\"success\":true}");
    });
    server.addHandler(updateChannelHandler);

    // POST /api/channels/activate: Bir kanalı telsize yükler
    AsyncCallbackJsonWebHandler* activateChannelHandler = new AsyncCallbackJsonWebHandler("/api/channels/activate", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        JsonObject obj = json.as<JsonObject>();
        int ch_no = obj["no"]; // Arayüzden 1-100 aralığında gelir
        if (ch_no < 1 || ch_no > 100) {
            request->send(400, "application/json", "{\"success\":false, \"error\":\"Geçersiz kanal numarası.\"}");
            return;
        }
        
        char ch_str[4]; // 3 haneli sayı + null terminator için
        sprintf(ch_str, "%d", ch_no); // Kanal numarasını string'e çevir
        GetMemoryChannel(ch_str); // Bu fonksiyon 1-100 arası string'i işler
        
        write_FRQ(current_ch.frequency);
        writeFRQToLcd(FRQ);
        saveCurrentChannel(); // Aktif kanalı kaydet

        request->send(200, "application/json", "{\"success\":true}");
    });
    server.addHandler(activateChannelHandler);

    // POST /api/channels/add - Boş bir kanal bulur ve numarasını döner
    server.on("/api/channels/add", HTTP_POST, [user, pass](AsyncWebServerRequest *request) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();

        for (int i = 0; i < 100; i++) {
            if (strcmp(memoryChannels[i].ChannelName, "KNL ") == 0) {
                // Boş kanal bulundu, ismini geçici olarak değiştirerek listede görünmesini sağla
                strcpy(memoryChannels[i].ChannelName, "YENI");
                saveMemoryChannels();

                char response_json[64];
                sprintf(response_json, "{\"success\":true, \"new_channel_no\": %d}", i + 1);
                request->send(200, "application/json", response_json);
                return;
            }
        }

        // Boş kanal bulunamadı
        request->send(409, "application/json", "{\"success\":false, \"error\":\"Hafıza dolu.\"}");
    });
    
    // POST /api/channels/delete - Belirtilen kanalı sıfırlar
    AsyncCallbackJsonWebHandler* deleteChannelHandler = new AsyncCallbackJsonWebHandler("/api/channels/delete", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        JsonObject obj = json.as<JsonObject>();
        int ch_no = obj["no"]; // Arayüzden 1-100 aralığında gelir
        if (ch_no < 1 || ch_no > 100) {
            request->send(400, "application/json", "{\"success\":false, \"error\":\"Geçersiz kanal numarası.\"}");
            return;
        }
        int index = ch_no - 1; // Dizi indeksi (0-99)

        // GÜNCELLENDİ: Kanalı, helper_functions'taki varsayılanlarla aynı "boş" duruma sıfırla.
        strcpy(memoryChannels[index].ChannelName, "KNL ");
        memoryChannels[index].frequency125 = (radio_type == 0) ? 11520 : 34400;
        memoryChannels[index].shift25 = (radio_type == 0) ? 24 : 304;
        memoryChannels[index].tone_position = 0;
        memoryChannels[index].shift_dir = 0;     // Simplex
        memoryChannels[index].tone_enabled = 0;  // Kapalı
        memoryChannels[index].power = 0;         // Yüksek

        saveMemoryChannels();
        request->send(200, "application/json", "{\"success\":true}");
    });
    server.addHandler(deleteChannelHandler);

    // POST /api/channels/delete_multiple - Birden fazla kanalı sıfırlar
    AsyncCallbackJsonWebHandler* deleteMultipleChannelsHandler = new AsyncCallbackJsonWebHandler("/api/channels/delete_multiple", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();
            
        JsonObject obj = json.as<JsonObject>();
        JsonArray channelsToDelete = obj["channels"].as<JsonArray>();

        for(JsonVariant ch_no_var : channelsToDelete) {
            int ch_no = ch_no_var.as<int>();
            if (ch_no >= 1 && ch_no <= 100) {
                int index = ch_no - 1;
                // GÜNCELLENDİ: Kanalı, helper_functions'taki varsayılanlarla aynı "boş" duruma sıfırla.
                strcpy(memoryChannels[index].ChannelName, "KNL ");
                memoryChannels[index].frequency125 = (radio_type == 0) ? 11520 : 34400;
                memoryChannels[index].shift25 = (radio_type == 0) ? 24 : 304;
                memoryChannels[index].tone_position = 0;
                memoryChannels[index].shift_dir = 0;     // Simplex
                memoryChannels[index].tone_enabled = 0;  // Kapalı
                memoryChannels[index].power = 0;         // Yüksek
            }
        }

        saveMemoryChannels();
        request->send(200, "application/json", "{\"success\":true}");
    });
    server.addHandler(deleteMultipleChannelsHandler);
    
// YENİ: Telsiz sekmesinden mevcut (aktif) kanalı boş bir hafıza kanalına kaydeder
    AsyncCallbackJsonWebHandler* saveCurrentChannelHandler = new AsyncCallbackJsonWebHandler("/api/channels/save_current", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();

        // 1. Boş bir hafıza kanalı bul
        int found_index = -1;
        for (int i = 0; i < 100; i++) {
            if (strcmp(memoryChannels[i].ChannelName, "KNL ") == 0) {
                found_index = i;
                break; // İlk boş kanalı bulduk, döngüden çık
            }
        }

        // Eğer boş kanal bulunamadıysa hata gönder
        if (found_index == -1) {
            request->send(409, "application/json", "{\"success\":false, \"error\":\"Hafıza dolu.\"}");
            return;
        }

        // 2. Gelen JSON'dan kanal adını al
        JsonObject obj = json.as<JsonObject>();
        String channel_name = obj["name"];
        if (channel_name.length() == 0 || channel_name.length() > 4) {
             request->send(400, "application/json", "{\"success\":false, \"error\":\"Geçersiz kanal adı.\"}");
            return;
        }

        // 3. Aktif kanal bilgilerini (current_ch) boş bulunan hafıza kanalına kopyala
        memoryChannels[found_index].frequency125 = (uint16_t)(current_ch.frequency / 12.5);
        memoryChannels[found_index].shift25 = (uint16_t)(current_ch.shift / 25);
        memoryChannels[found_index].tone_position = current_ch.tone_pos;
        memoryChannels[found_index].tone_enabled = current_ch.tone_enabled;
        memoryChannels[found_index].power = RF_POWER_STATE; // O anki global güç ayarını al

        // Shift yönünü doğru formata çevir (-1,0,1 -> 2,0,1)
        uint8_t dir_to_store = 0; // Simplex
        if (current_ch.shift_dir == 1) dir_to_store = 1;      // Plus
        else if (current_ch.shift_dir == -1) dir_to_store = 2; // Minus
        memoryChannels[found_index].shift_dir = dir_to_store;

        // 4. Yeni kanal adını ata
        strncpy(memoryChannels[found_index].ChannelName, channel_name.c_str(), 4);
        memoryChannels[found_index].ChannelName[4] = '\0'; // Güvenlik için sonuna null karakter ekle

        // 5. Değişiklikleri kalıcı olarak kaydet
        saveMemoryChannels();

        // 6. Başarı mesajı gönder
        char response_json[128];
        snprintf(response_json, sizeof(response_json), "{\"success\":true, \"message\":\"Kanal '%s' adıyla %d. sıraya kaydedildi.\"}", channel_name.c_str(), found_index + 1);
        request->send(200, "application/json", response_json);
    });
    server.addHandler(saveCurrentChannelHandler);

    // GET /api/channels/export_csv: Kanalları CSV olarak dışa aktarır
    server.on("/api/channels/export_csv", HTTP_GET, [user, pass](AsyncWebServerRequest *request) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();

        String csv = "KanalNo,Isim,Frekans,Ton,Shift,Yon,Guc\n";
        for (int i = 0; i < 100; i++) {
            // Sadece dolu kanalları dışa aktar
            if (strcmp(memoryChannels[i].ChannelName, "KNL ") != 0) {
                csv += String(i + 1) + ",";
                csv += String(memoryChannels[i].ChannelName) + ",";

                char freq_buffer[10];
                dtostrf(((float)memoryChannels[i].frequency125 * 12.5) / 1000.0, 7, 3, freq_buffer);
                csv += String(freq_buffer) + ",";

                if (memoryChannels[i].tone_enabled) {
                    csv += String(ctcss_tone_list[memoryChannels[i].tone_position], 1) + ",";
                } else {
                    csv += "0.0,";
                }
                
                csv += String((unsigned long)memoryChannels[i].shift25 * 25) + ",";

                if (memoryChannels[i].shift_dir == 0) csv += "S,";
                else if (memoryChannels[i].shift_dir == 1) csv += "+,";
                else csv += "-,";
                
                csv += String((memoryChannels[i].power == 0) ? "Yuksek" : "Dusuk") + "\n";
            }
        }
        
        AsyncWebServerResponse *response = request->beginResponse(200, "text/csv", csv);
        response->addHeader("Content-Disposition", "attachment; filename=kanallar.csv");
        request->send(response);
    });

    // POST /api/channels/import_csv: CSV dosyasını içe aktarır
    server.on("/api/channels/import_csv", HTTP_POST, [user, pass](AsyncWebServerRequest *request) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();
        // Bu endpoint sadece body handler ile çalışır.
        request->send(200, "application/json", "{\"success\":true, \"message\":\"Processing started.\"}");
    }, NULL, [user, pass](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();

        static String csv_buffer;
        if (index == 0) {
            csv_buffer = "";
        }
        csv_buffer += String((char*)data).substring(0, len);

        if (index + len == total) {
            Serial.println("CSV içe aktarma tamamlandı, işleniyor...");
            int line_start = 0;
            int line_end = csv_buffer.indexOf('\n');
            
            // Başlık satırını atla
            line_start = line_end + 1;
            line_end = csv_buffer.indexOf('\n', line_start);

            while(line_end > -1) {
                String line = csv_buffer.substring(line_start, line_end);
                line.trim();
                
                int val_start = 0;
                int val_end = line.indexOf(',');
                int col = 0;
                
                int ch_no = 0;
                
                while (val_end > -1 && col < 7) {
                    String value = line.substring(val_start, val_end);
                    if (col == 0) { // Kanal No
                        ch_no = value.toInt();
                        if (ch_no < 1 || ch_no > 100) break;
                    } else if (ch_no > 0) {
                        int idx = ch_no - 1;
                        if (col == 1) { // İsim
                            strncpy(memoryChannels[idx].ChannelName, value.c_str(), 4);
                            memoryChannels[idx].ChannelName[4] = '\0';
                        } else if (col == 2) { // Frekans
                            memoryChannels[idx].frequency125 = (uint16_t)((value.toFloat() * 1000.0) / 12.5);
                        } else if (col == 3) { // Ton
                            float tone_hz = value.toFloat();
                            if (tone_hz > 0) {
                                memoryChannels[idx].tone_enabled = 1;
                                for(int i=0; i<TOTAL_TONES; i++) {
                                    if(abs(ctcss_tone_list[i] - tone_hz) < 0.01) {
                                        memoryChannels[idx].tone_position = i; break;
                                    }
                                }
                            } else { memoryChannels[idx].tone_enabled = 0; }
                        } else if (col == 4) { // Shift
                            memoryChannels[idx].shift25 = (uint16_t)(value.toInt() / 25);
                        } else if (col == 5) { // Yön
                            if (value == "S") memoryChannels[idx].shift_dir = 0;
                            else if (value == "+") memoryChannels[idx].shift_dir = 1;
                            else memoryChannels[idx].shift_dir = 2;
                        } else if (col == 6) { // Güç
                            value.toLowerCase();
                            if (value == "yuksek" || value == "yüksek") memoryChannels[idx].power = 0;
                            else memoryChannels[idx].power = 1;
                        }
                    }
                    val_start = val_end + 1;
                    val_end = line.indexOf(',', val_start);
                    if (val_end == -1 && col == 5) { // Son sütun
                         value = line.substring(val_start);
                         value.toLowerCase();
                         if (value == "yuksek" || value == "yüksek") memoryChannels[ch_no-1].power = 0;
                         else memoryChannels[ch_no-1].power = 1;
                    }
                    col++;
                }
                
                line_start = line_end + 1;
                line_end = csv_buffer.indexOf('\n', line_start);
            }
            saveMemoryChannels();
            Serial.println("CSV başarıyla işlendi ve kanallar kaydedildi.");
            csv_buffer = "";
        }
    });

    // --- MEVCUT WIFI API ENDPOINT'LERİ ---

    // WiFi ayarlarını gönderen API
    server.on("/api/wifi/config", HTTP_GET, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        String response;
        serializeJson(wifiConfig, response);
        request->send(200, "application/json", response);
    });

    // WiFi durumunu gönderen API
    server.on("/api/wifi/status", HTTP_GET, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        DynamicJsonDocument json(256);
        if (WiFi.getMode() == WIFI_AP) {
            json["mode"] = "AP";
            json["ip"] = WiFi.softAPIP().toString();
            json["ssid"] = WiFi.softAPSSID();
        } else if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
            json["mode"] = "STA";
            json["ip"] = WiFi.localIP().toString();
            json["ssid"] = WiFi.SSID();
        } else {
            json["mode"] = "Disconnected";
        }
        
        String response;
        serializeJson(json, response);
        request->send(200, "application/json", response);
    });
    
    // Ağları tarayan API
    server.on("/api/wifi/scan", HTTP_GET, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
             return request->requestAuthentication();
        
        Serial.println("WiFi tarama isteği alındı...");
        
        esp_task_wdt_delete(NULL);
        int n = WiFi.scanNetworks();
        esp_task_wdt_add(NULL);
        
        Serial.println("WiFi tarama tamamlandı.");
        
        DynamicJsonDocument json(4096);
        JsonArray networks = json.to<JsonArray>();

        if (n > 0) {
            Serial.printf("%d ağ bulundu.\n", n);
            for (int i = 0; i < n; ++i) {
                JsonObject net = networks.createNestedObject();
                net["ssid"] = WiFi.SSID(i);
                net["rssi"] = WiFi.RSSI(i);
                switch (WiFi.encryptionType(i)) {
                    case WIFI_AUTH_OPEN: net["auth"] = "Açık"; break;
                    case WIFI_AUTH_WEP: net["auth"] = "WEP"; break;
                    case WIFI_AUTH_WPA_PSK: net["auth"] = "WPA-PSK"; break;
                    case WIFI_AUTH_WPA2_PSK: net["auth"] = "WPA2-PSK"; break;
                    case WIFI_AUTH_WPA_WPA2_PSK: net["auth"] = "WPA/WPA2"; break;
                    default: net["auth"] = "Bilinmiyor"; break;
                }
            }
        } else {
            Serial.println("Tarama sonucunda ağ bulunamadı veya hata oluştu.");
        }
        
        String output;
        serializeJson(json, output);
        request->send(200, "application/json", output);
        WiFi.scanDelete();
    });


    // WiFi ayarlarını kaydeden API
    AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/api/wifi/config", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        wifiConfig.clear();
        wifiConfig = json.as<JsonObject>();
        saveWifiConfig();
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", "{\"success\":true}");
        response->addHeader("Connection", "close");
        request->send(response);
        delay(1000);
        ESP.restart();
    });
    server.addHandler(handler);

    // OTA Güncelleme API'si
    server.on("/update", HTTP_POST, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        response->addHeader("Connection", "close");
        request->send(response);
        ESP.restart();
    }, [user, pass](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        if (!index) {
            Serial.printf("Update start: %s\n", filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
                Update.printError(Serial);
            }
        }
        if (len) {
            if (Update.write(data, len) != len) {
                Update.printError(Serial);
            }
        }
        if (final) {
            if (!Update.end(true)) {
                Update.printError(Serial);
            } else {
                Serial.println("Update complete");
            }
        }
    });

    // LittleFS OTA Güncelleme API'si
    server.on("/update_fs", HTTP_POST, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        response->addHeader("Connection", "close");
        request->send(response);
        ESP.restart();
    }, [user, pass](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        if (!index) {
            Serial.printf("LittleFS Update Start: %s\n", filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_SPIFFS)) { 
                Update.printError(Serial);
            }
        }
        if (len) {
            if (Update.write(data, len) != len) {
                Update.printError(Serial);
            }
        }
        if (final) {
            if (!Update.end(true)) {
                Update.printError(Serial);
            } else {
                Serial.println("LittleFS Update Complete");
            }
        }
    });

    // Yapılandırma Dışa Aktarma API'si
    server.on("/api/backup/export", HTTP_GET, [user, pass](AsyncWebServerRequest *request) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();

        Serial.println("Yapılandırma yedekleme isteği alındı (Streaming).");

        AsyncResponseStream *response = request->beginResponseStream("application/json");
        response->addHeader("Content-Disposition", "attachment; filename=\"aselsan4822_backup.json\"");
        
        response->print("{\n");

        const char *configFiles[] = {"config", "aprs", "current", "wifi"};
        for (int i = 0; i < sizeof(configFiles) / sizeof(configFiles[0]); i++) {
            String filename = "/" + String(configFiles[i]) + ".json";
            File file = LittleFS.open(filename, "r");
            if (file) {
                response->printf("  \"%s\": ", configFiles[i]);
                DynamicJsonDocument fileDoc(2048);
                deserializeJson(fileDoc, file);
                serializeJsonPretty(fileDoc, *response);
                response->print(",\n");
                file.close();
            }
        }

        // "channels" verisini anlık olarak `memoryChannels` dizisinden oluştur
        response->print("  \"channels\": ");
        const size_t capacity = JSON_ARRAY_SIZE(100) + 100 * JSON_OBJECT_SIZE(7) + 2048;
        DynamicJsonDocument doc(capacity);
        JsonArray array = doc.to<JsonArray>();

        for (int i = 0; i < 100; i++) {
            JsonObject obj = array.createNestedObject();
            obj["f"] = memoryChannels[i].frequency125;
            obj["s"] = memoryChannels[i].shift25;
            obj["t"] = memoryChannels[i].tone_position;
            // Yeni struct formatına uygun alanlar yazılıyor
            obj["sd"] = memoryChannels[i].shift_dir;
            obj["te"] = memoryChannels[i].tone_enabled;
            obj["p"] = memoryChannels[i].power;
            obj["n"] = memoryChannels[i].ChannelName;
        }
        serializeJsonPretty(doc, *response);
        response->print("\n");

        response->print("}\n");
        request->send(response);
    });

    // Yapılandırma İçe Aktarma API'si
    server.on("/api/backup/import", HTTP_POST, [user, pass](AsyncWebServerRequest *request) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();
            
        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", "{\"success\":true}");
        response->addHeader("Connection", "close");
        request->send(response);
        delay(1000);
        ESP.restart();

    }, [user, pass](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        if (!request->authenticate(user, pass))
            return request->requestAuthentication();
        if (!index) {
            Serial.printf("Yedek geri yükleme başladı: %s\n", filename.c_str());
        }

        static std::vector<uint8_t> backupBuffer; 
        if(index == 0) {
          backupBuffer.clear();
        }
        backupBuffer.insert(backupBuffer.end(), data, data + len);

        if (final) {
            Serial.println("Yedek dosyası tamamen yüklendi.");
            backupBuffer.push_back('\0');
            DynamicJsonDocument backupDoc(16384);
            DeserializationError error = deserializeJson(backupDoc, (const char*)backupBuffer.data());
            backupBuffer.clear();

            if (error) {
                Serial.printf("Geri yükleme hatası: JSON ayrıştırılamadı - %s\n", error.c_str());
                return;
            }

            JsonObject root = backupDoc.as<JsonObject>();
            for (JsonPair kv : root) {
                String key = kv.key().c_str();
                String filename = "/" + key + ".json";
                Serial.printf("'%s' dosyası geri yükleniyor...\n", filename.c_str());

                File file = LittleFS.open(filename, "w");
                if (!file) {
                    Serial.printf("Hata: '%s' dosyası yazılamadı.\n", filename.c_str());
                    continue; 
                }
                if (serializeJson(kv.value(), file) == 0) {
                    Serial.printf("Hata: '%s' dosyasına içerik yazılamadı.\n", filename.c_str());
                }
                file.close();
            }
            Serial.println("Tüm dosyalar geri yüklendi. Cihaz yeniden başlatılacak.");
        }
    });

    // ==========================================================================================
    // === GÜNCELLENMİŞ VE YENİ EKLENMİŞ BÖLÜM: TELSİZ KONTROL API ENDPOINT'LERİ ===================
    // ==========================================================================================

    // GET /api/radio/status: Telsizin anlık durumunu JSON olarak gönderir
    server.on("/api/radio/status", HTTP_GET, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();

        DynamicJsonDocument json(1024);

        // Alış Frekansı
        char freq_buffer[10];
        numberToFrequency(current_ch.frequency, freq_buffer);
        String freqStr(freq_buffer);
        freqStr.trim();
        json["frequency"] = freqStr;
        
        // GÜNCELLEME: Gönderme frekansını hesapla ve ekle
        if (TRX_MODE == TX) {
            uint32_t tx_frequency_khz = current_ch.frequency + (current_ch.shift_dir * current_ch.shift);
            numberToFrequency(tx_frequency_khz, freq_buffer);
            String txFreqStr(freq_buffer);
            txFreqStr.trim();
            json["tx_frequency"] = txFreqStr;
        } else {
            json["tx_frequency"] = ""; // RX modunda boş gönder
        }

        json["shift_khz"] = (unsigned long)current_ch.shift;
        
        // Shift yönünü metne çevir
        if (current_ch.shift_dir == -1) json["shift_dir"] = "-";
        else if (current_ch.shift_dir == 1) json["shift_dir"] = "+";
        else json["shift_dir"] = "S";

        // Ton bilgilerini hazırla
        if (current_ch.tone_enabled) {
            json["tone_hz"] = ctcss_tone_list[current_ch.tone_pos];
        } else {
            json["tone_hz"] = 0;
        }

        json["squelch_on"] = (SQL_MODE == SQL_ON);
        json["power_high"] = (RF_POWER_STATE == HIGH_POWER);

        // Gönderme/Alma durumunu ve sinyal seviyesini belirle
        if (TRX_MODE == TX) {
            json["mode"] = "TX";
            json["signal_level"] = analogRead(FWD_POWER_PIN); // Göndermede çıkış gücünü oku
        } else {
            json["mode"] = "RX";
            // Alma modunda, sinyal olup olmadığını SQL_ACTIVE pininden anlıyoruz.
            // Bu, analog bir seviye değil, "var/yok" bilgisidir.
            // CHANNEL_BUSY=0 ise sinyal var demektir.
            json["signal_level"] = (CHANNEL_BUSY == 0) ? 4095 : 0; // Sinyal varsa tam, yoksa sıfır göster
        }
        
        // GÜNCELLEME: Web PTT'nin aktif durumunu da gönder
        json["web_ptt_on"] = web_ptt_active;
        
        String response;
        serializeJson(json, response);
        request->send(200, "application/json", response);
    });

    // POST /api/radio/control: Web arayüzünden gelen temel ayar komutlarını işler
    AsyncCallbackJsonWebHandler* radioControlHandler = new AsyncCallbackJsonWebHandler("/api/radio/control", [user, pass](AsyncWebServerRequest *request, JsonVariant &json) {
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        JsonObject obj = json.as<JsonObject>();
        String command = obj["command"].as<String>();
        bool success = true;
        String message = "OK";

        if (command == "set_frequency") {
            String freq_str = obj["value"];
            char freq_char[9];
            // Gelen string'i, Calculate_Frequency'nin beklediği formata (sağa boşluklu) getir
            snprintf(freq_char, sizeof(freq_char), "%-8s", freq_str.c_str());
            if(Calculate_Frequency(freq_char)) {
                numberToFrequency(current_ch.frequency, FRQ);
                write_FRQ(current_ch.frequency);
                strcpy(FRQ_old, FRQ); // FRQ_old'u da güncelle ki senkron kalsın
                saveCurrentChannel();
            } else {
                success = false;
                message = "Geçersiz frekans.";
            }
        } else if (command == "set_shift_khz") {
            current_ch.shift = obj["value"].as<uint16_t>();
            saveCurrentChannel();
        } else if (command == "set_shift_dir") {
            String dir = obj["value"].as<String>();
            if (dir == "+") current_ch.shift_dir = 1;
            else if (dir == "-") current_ch.shift_dir = -1;
            else current_ch.shift_dir = 0;
            saveCurrentChannel();
        } else if (command == "set_tone") {
            float tone_hz = obj["value"].as<float>();
            if (tone_hz > 0) {
                current_ch.tone_enabled = 1;
                // Gelen Hz değerine en yakın tonu listede bul
                for(int i = 0; i < TOTAL_TONES; i++) {
                    if (abs(ctcss_tone_list[i] - tone_hz) < 0.01) {
                        current_ch.tone_pos = i;
                        break;
                    }
                }
            } else {
                current_ch.tone_enabled = 0;
                current_ch.tone_pos = 0;
            }
            SetTone(current_ch.tone_enabled); // Tonu anında donanıma uygula
            saveCurrentChannel();
        } else if (command == "set_squelch") {
            SQL_MODE = obj["value"].as<bool>() ? SQL_ON : SQL_OFF;
        } else if (command == "set_power") {
            String pwr = obj["value"].as<String>();
            RF_POWER_STATE = (pwr == "high") ? HIGH_POWER : LOW_POWER;
            SetRFPower();
        } 
        // GÜNCELLEME: Hatalı çalışan PTT ve Tarama komutları bu genel handler'dan kaldırıldı.
        // Onların yerine aşağıda yeni ve özel API endpoint'leri oluşturuldu.
        else {
            success = false;
            message = "Bilinmeyen komut.";
        }

        // Başarı veya hata durumunu JSON olarak geri bildir
        char response_json[128];
        snprintf(response_json, sizeof(response_json), "{\"success\":%s, \"message\":\"%s\"}", success ? "true" : "false", message.c_str());
        request->send(200, "application/json", response_json);
    });
    server.addHandler(radioControlHandler);
    
    // YENİ BÖLÜM: Özel Telsiz Aksiyonları için API Endpoint'leri
    
    // POST /api/radio/action/toggle_ptt: Web PTT'sini açar/kapatır
    server.on("/api/radio/action/toggle_ptt", HTTP_POST, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        web_ptt_active = !web_ptt_active; // Global PTT durum değişkenini tersine çevir
        Serial.printf("Web PTT durumu: %s\n", web_ptt_active ? "AKTIF" : "KAPALI");
        request->send(200, "application/json", "{\"success\":true}");
    });

    // POST /api/radio/action/reverse: Frekansları ters çevirir (Fiziksel 'M' tuşu gibi)
    server.on("/api/radio/action/reverse", HTTP_POST, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();

        // main.cpp'deki 'M' tuşuna basıldığında çalışan mantığın aynısı
        numberToFrequency(current_ch.frequency + current_ch.shift_dir * current_ch.shift, FRQ);
        if (current_ch.shift_dir == plusSHIFT) current_ch.shift_dir = minusSHIFT;
        else if (current_ch.shift_dir == minusSHIFT) current_ch.shift_dir = plusSHIFT;
        
        if (Calculate_Frequency(FRQ)) {
            write_FRQ(current_ch.frequency);
            saveCurrentChannel();
        }
        
        request->send(200, "application/json", "{\"success\":true}");
    });

    // POST /api/radio/action/scan: Taramayı başlatır (Fiziksel 'SCAN' tuşu gibi)
    server.on("/api/radio/action/scan", HTTP_POST, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        Serial.println("Web'den tarama istegi alindi, ana donguye bildiriliyor.");
        web_scan_request = true; // Tarama isteği bayrağını ayarla, işlemi loop() yapacak.
        
        request->send(200, "application/json", "{\"success\":true, \"message\":\"Tarama istegi alindi.\"}");
    });

    // POST /api/radio/action/vna: VNA modunu başlatır (Fiziksel 'C' tuşu gibi)
    server.on("/api/radio/action/vna", HTTP_POST, [user, pass](AsyncWebServerRequest *request){
        if(!request->authenticate(user, pass))
            return request->requestAuthentication();
        
        Serial.println("Web'den VNA istegi alindi, ana donguye bildiriliyor.");
        web_vna_request = true; // VNA isteği bayrağını ayarla, işlemi loop() yapacak.
        
        request->send(200, "application/json", "{\"success\":true, \"message\":\"VNA istegi alindi.\"}");
    });

    // ==========================================================================================
    // === YENİ VE GÜNCELLENMİŞ BÖLÜM SONU =======================================================
    // ==========================================================================================

    server.onNotFound([](AsyncWebServerRequest *request){
        request->send(404, "text/plain", "Sayfa bulunamadı");
    });

    server.begin();
    Serial.println("Web sunucusu başlatıldı.");
}

