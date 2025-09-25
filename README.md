# ASELSAN 4822/49XX ESP32 Modernizasyon Kiti

Bu proje, Barış Dinç (OH2UDS/TA7W) tarafından başlatılan ve Aselsan'ın 48XX/49XX serisi klasik amatör telsizlerini modern bir mikrodenetleyici ile yeniden canlandırmayı amaçlayan [orijinal projenin](https://github.com/barisdinc/ASELSAN_4822) üzerine inşa edilmiş, yeni nesil bir versiyonudur.

Bu branch, projeyi çok daha güçlü ve yetenekli bir platform olan **ESP32**'ye taşır. ESP32'nin getirdiği WiFi ve Bluetooth yetenekleri sayesinde telsize, herhangi bir modern web tarayıcısı üzerinden erişilebilen, kapsamlı bir **web arayüzü** eklenmiştir. Artık telsizinizin tüm ayarlarını, kanal hafızasını ve anlık kontrollerini bilgisayarınızdan veya telefonunuzdan kolayca yönetebilirsiniz.

![Web Arayüzü Önizlemesi](https://github.com/barisdinc/ASELSAN_4822/blob/master/docs/web_ui_preview.png?raw=true)
*(Not: Bu görsel, arayüzün genel görünümünü temsil etmesi için eklenmiştir. Gerçek arayüz farklılık gösterebilir.)*

## 🚀 Temel Özellikler

Bu ESP32 versiyonu, orijinal projenin tüm yeteneklerini korurken üzerine devrim niteliğinde yeni özellikler ekler:

### Genel Telsiz Özellikleri
* **VHF/UHF Desteği:** Tek bir yazılımla hem VHF hem de UHF bantlarında çalışabilme.
* **Geniş Frekans Aralığı:** Ayarlanabilir alt ve üst frekans limitleri.
* **CTCSS Ton Desteği:** 20 farklı CTCSS tonu.
* **Kanal Hafızası:** 100 adet programlanabilir hafıza kanalı.
* **APRS Desteği:**
    * Dahili GPS modülü ile veya manuel olarak girilen sabit koordinatlarla konum bildirimi (Beacon).
    * Ayarlanabilir beacon gönderme aralığı.
    * Standart APRS ve ISS APRS frekansları için ayrı hafıza.
* **VNA (Vektör Network Analizör):** Anteninizin SWR değerini belirli bir frekans aralığında analiz etme imkanı.
* **Frekans Tarama:** Belirlenen aralıkta hızlı frekans tarama.
* **TOT (Time-out Timer):** Ayarlanabilir gönderme zaman aşımı süresi.

### 🌐 Web Arayüzü Özellikleri
Telsize, bağlandığı yerel ağ üzerinden veya kendi oluşturduğu Access Point üzerinden erişilebilen modern ve mobil uyumlu web arayüzü:

* **Anlık Telsiz Kontrolü:**
    * Frekans, Shift, Ton, Güç (Yüksek/Düşük) ve Squelch ayarlarını anlık olarak değiştirme.
    * Canlı S-Metre ve durum göstergesi (RX/TX).
    * Web üzerinden PTT mandalı.
    * Tarama ve VNA fonksiyonlarını web arayüzünden başlatma.

* **Gelişmiş Kanal Yönetimi:**
    * Tüm kayıtlı kanalları liste halinde görme, düzenleme ve silme.
    * Web arayüzünden yeni kanal ekleme.
    * Mevcut VFO ayarlarını tek tıkla yeni bir kanala kaydetme.
    * **CSV ile İçe/Dışa Aktarma:** Tüm kanal listenizi `.csv` dosyası olarak yedekleyin veya Chirp gibi programlardan oluşturduğunuz listeleri kolayca telsize yükleyin.

* **APRS Ayarları:**
    * Çağrı işareti, SSID, mesaj, beacon periyodu gibi tüm APRS ayarlarını webden yapılandırma.
    * Anlık GPS konumunu alıp forma doldurma.
    * Tek tuşla manuel beacon gönderme.

* **WiFi Yönetimi:**
    * Çevredeki kablosuz ağları tarama ve bağlanma.
    * Kayıtlı ağların önceliğini değiştirme.
    * Telsizin Access Point (AP) modu ayarlarını (SSID, şifre) değiştirme.
    * Web arayüzü için kullanıcı adı ve şifre belirleme.

* **Sistem ve Bakım (OTA):**
    * **OTA Firmware Güncelleme:** Telsizin ana yazılımını kablosuz olarak, web arayüzünden `.bin` dosyası seçerek güncelleme.
    * **OTA Dosya Sistemi Güncelleme:** Web arayüzünü (HTML/CSS/JS) içeren LittleFS dosya sistemini kablosuz olarak güncelleme.
    * **Yapılandırma Yedekleme/Geri Yükleme:** Kanallar, WiFi, APRS ve genel ayarlar dahil tüm cihaz yapılandırmasını tek bir `.json` dosyası olarak yedekleme ve geri yükleme.

## 🛠️ Kurulum

Bu yazılımı derlemek ve telsizinize yüklemek için [PlatformIO IDE](https://platformio.org/)'nin kullanılması şiddetle tavsiye edilir.

### Gerekli Donanımlar
* Aselsan 4822/4826/4922 serisi bir telsiz. (Sadece arayüzü test edecekseni gerekli değil.)
* ESP32 Geliştirme Kartı (örn: ESP32-DevKitC).

### Yazılım Yükleme Adımları
1.  Projeyi **Visual Studio Code** ile PlatformIO eklentisini kullanarak açın.
2.  PlatformIO, `platformio.ini` dosyasında belirtilen tüm kütüphaneleri (ArduinoJson, TinyGPSPlus, ESPAsyncWebServer vb.) otomatik olarak kuracaktır.
3.  ESP32 kartınızı bilgisayarınıza bağlayın.
4.  PlatformIO arayüzünden **"Build"** (Derle) ve ardından **"Upload"** (Yükle) işlemlerini gerçekleştirin.
5.  **ÇOK ÖNEMLİ:** Web arayüzünün çalışması için dosya sistemini de yüklemeniz gerekmektedir. PlatformIO arayüzünden **"Upload Filesystem Image"** seçeneğini çalıştırın.

## 🚀 İlk Kullanım ve Ayarlar

1.  Yazılım ilk yüklendiğinde, cihaz çevrede kayıtlı bir WiFi ağı bulamazsa **Access Point (AP)** modunda başlayacaktır.
2.  Telefonunuzdan veya bilgisayarınızdan WiFi ağlarını taratın ve aşağıdaki ağa bağlanın:
    * **SSID:** `ASELSAN_4822_ESP32`
    * **Şifre:** `12345678`
3.  Bağlandıktan sonra web tarayıcınızı açın ve `http://192.168.4.1` adresine gidin.
4.  Sizden bir kullanıcı adı ve şifre istenecektir. Varsayılan bilgiler:
    * **Kullanıcı Adı:** `admin`
    * **Şifre:** `aselsan`
5.  Arayüz açıldıktan sonra **"WiFi"** sekmesine gidin. Buradan kendi kablosuz ağınızı taratıp seçerek ve şifresini girerek telsizin internete bağlanmasını sağlayabilirsiniz. Ayarları kaydettikten sonra cihaz yeniden başlayacak ve artık kendi ağınız üzerinden atanan IP adresi ile erişilebilir olacaktır.

## 🙏 Teşekkür

Bu projenin temelini atan, orijinal donanım modifikasyonlarını ve yazılımını geliştiren **Barış Dinç (OH2UDS/TA7W)**'e sonsuz teşekkürler. Bu çalışma, onun başlattığı harika projenin bir devamı ve geliştirmesidir.
