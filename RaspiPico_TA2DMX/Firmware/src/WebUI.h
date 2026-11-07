#ifndef WEBUI_H
#define WEBUI_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Updater.h>
#include <ArduinoJson.h>
#include "main.h"

extern String mycall;
extern String APRS_Message;
extern unsigned int APRS_Timeout;
extern void eewrite_nbytes(String dst, uint8_t sz, uint16_t addr);
extern String webLogBuffer;
extern String commandString;
extern bool commandComplete;

WebServer server(80);
bool wifiEnabled = false;

static String trimStoredString(const String &s) {
    String out = s;
    out.trim();
    return out;
}

static String jsonEscape(const String &s) {
    String out = "";
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c >= 32 && c <= 126) out += c;
    }
    return out;
}

// ============================================================================
// 1) ILK KURULUM EKRANI (DLS_ASELSAN_KIT / deeplabstudio)
// ============================================================================
const char* setupHtmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <title>DLS ASELSAN 4822 - İlk Kurulum</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #0b2545 0%, #134074 100%);
            color: #333;
            margin: 0;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }
        .setup-card {
            background: #ffffff;
            width: 100%;
            max-width: 420px;
            border-radius: 14px;
            padding: 30px 26px;
            box-shadow: 0 12px 32px rgba(0,0,0,0.35);
        }
        .logo-badge {
            background: #0056b3;
            color: #fff;
            display: inline-block;
            padding: 5px 12px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
            letter-spacing: 0.8px;
            margin-bottom: 10px;
        }
        h1 {
            margin: 0 0 8px 0;
            color: #0b2545;
            font-size: 24px;
        }
        p.sub {
            margin: 0 0 22px 0;
            color: #555;
            font-size: 14px;
            line-height: 1.45;
        }
        label {
            display: block;
            font-weight: 600;
            font-size: 14px;
            margin-bottom: 6px;
            color: #222;
        }
        input[type="text"], input[type="password"] {
            width: 100%;
            padding: 11px 12px;
            border: 1.5px solid #ccd6e0;
            border-radius: 8px;
            font-size: 16px;
            margin-bottom: 16px;
            outline: none;
            transition: border-color 0.2s;
        }
        input[type="text"]:focus, input[type="password"]:focus {
            border-color: #0056b3;
        }
        #callsign {
            text-transform: uppercase;
            font-family: monospace;
            font-weight: bold;
            letter-spacing: 1px;
        }
        .pass-wrap {
            position: relative;
        }
        .pass-toggle {
            position: absolute;
            right: 10px;
            top: 9px;
            background: #eef3f8;
            border: none;
            padding: 4px 8px;
            border-radius: 5px;
            font-size: 12px;
            cursor: pointer;
            color: #0056b3;
        }
        .btn-save {
            background: #28a745;
            color: white;
            border: none;
            padding: 13px;
            border-radius: 8px;
            width: 100%;
            font-size: 16px;
            font-weight: 600;
            cursor: pointer;
            transition: background 0.2s;
        }
        .btn-save:hover {
            background: #218838;
        }
        .hint {
            font-size: 12px;
            color: #666;
            margin-top: -12px;
            margin-bottom: 16px;
        }
        .footer {
            text-align: center;
            margin-top: 18px;
            font-size: 12px;
            color: #888;
        }
    </style>
</head>
<body>
    <div class="setup-card">
        <div class="logo-badge">DEEP LAB STUDIO &bull; %VERSION%</div>
        <h1>Telsiz İlk Kurulumu</h1>
        <p class="sub">Hoş geldiniz! Cihazınızı kullanıma açmak için çağrı işaretinizi ve yeni WiFi şifrenizi belirleyin. Kaydettikten sonra telsiz yeniden başlayacak ve <b>ASELSAN_ÇAĞRIİŞARETİ</b> adıyla WiFi açacaktır.</p>
        <form method="POST" action="/initialSetup" onsubmit="return validateSetup()">
            <label for="callsign">Çağrı İşareti (Callsign):</label>
            <input type="text" id="callsign" name="callsign" maxlength="6" placeholder="Örn: TA2DMX" required autocomplete="off">
            <div class="hint">Maksimum 6 karakter. Açılış mesajı ve WiFi adı buna göre ayarlanır.</div>

            <label for="wifipass">Yeni WiFi Şifresi:</label>
            <div class="pass-wrap">
                <input type="password" id="wifipass" name="wifipass" minlength="8" maxlength="32" placeholder="En az 8 karakter" required>
                <button type="button" class="pass-toggle" onclick="togglePass()">Göster</button>
            </div>
            <div class="hint">WPA2 güvenliği için en az 8 karakter olmalıdır.</div>

            <button type="submit" class="btn-save">Kaydet ve Başlat</button>
        </form>
        <div class="footer">DLS_ASELSAN_KIT &bull; 192.168.4.1</div>
    </div>
    <script>
        function togglePass() {
            var p = document.getElementById('wifipass');
            var b = document.querySelector('.pass-toggle');
            if (p.type === 'password') { p.type = 'text'; b.innerText = 'Gizle'; }
            else { p.type = 'password'; b.innerText = 'Göster'; }
        }
        function validateSetup() {
            var cs = document.getElementById('callsign').value.trim().toUpperCase();
            var pw = document.getElementById('wifipass').value;
            if (cs.length < 3 || cs.length > 6) {
                alert('Lütfen geçerli bir çağrı işareti giriniz (3-6 karakter).');
                return false;
            }
            if (pw.length < 8) {
                alert('WiFi şifresi en az 8 karakter olmalıdır!');
                return false;
            }
            document.getElementById('callsign').value = cs;
            return true;
        }
    </script>
</body>
</html>
)rawliteral";

// ============================================================================
// 2) ANA WEB ARAYUZU (5 MENU: Cihaz Ayarlari, APRS, Kanallar, Konsol, Guncelleme)
// ============================================================================
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8"> 
    <title>ASELSAN 4822 Yönetim Paneli</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #f0f3f8; color: #333; margin: 0; padding: 0; }
        .navbar { background: linear-gradient(90deg, #0b2545 0%, #0056b3 100%); color: white; display: flex; justify-content: space-between; padding: 12px 20px; align-items: center; flex-wrap: wrap; gap: 10px; box-shadow: 0 2px 8px rgba(0,0,0,0.15); }
        .navbar h2 { margin: 0; font-size: 20px; display: flex; align-items: center; gap: 8px; }
        .badge-ver { background: rgba(255,255,255,0.2); padding: 3px 8px; border-radius: 12px; font-size: 12px; font-weight: normal; }
        .nav-links { display: flex; gap: 6px; flex-wrap: wrap; }
        .nav-links button { background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.4); color: white; padding: 8px 14px; border-radius: 6px; cursor: pointer; transition: 0.2s; font-size: 14px; font-weight: 500; }
        .nav-links button:hover, .nav-links button.active { background: white; color: #0b2545; border-color: white; font-weight: 600; }
        
        .container { max-width: 960px; margin: 20px auto; background: white; padding: 24px; border-radius: 12px; box-shadow: 0 4px 14px rgba(0,0,0,0.08); }
        h3 { color: #0b2545; margin-top: 0; border-bottom: 2px solid #e9ecef; padding-bottom: 8px; }
        .section-box { background: #f8fafd; border: 1px solid #e2e8f0; border-radius: 8px; padding: 16px; margin-bottom: 20px; }
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
        .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 14px; }
        label { display: block; font-size: 13px; font-weight: 600; color: #444; margin-bottom: 4px; }
        input[type="text"], input[type="number"], input[type="password"], input[type="file"], select {
            width: 100%; padding: 9px 10px; margin-bottom: 10px; border: 1px solid #cbd5e1; border-radius: 6px; font-size: 14px; background: #fff;
        }
        button.btn-primary { background: #0056b3; color: white; border: none; padding: 11px 16px; border-radius: 6px; cursor: pointer; width: 100%; font-size: 15px; font-weight: 600; transition: 0.2s; }
        button.btn-primary:hover { background: #004494; }
        button.btn-success { background: #28a745; color: white; border: none; padding: 11px 16px; border-radius: 6px; cursor: pointer; width: 100%; font-size: 15px; font-weight: 600; transition: 0.2s; }
        button.btn-success:hover { background: #218838; }
        button.btn-warning { background: #f59e0b; color: white; border: none; padding: 10px 14px; border-radius: 6px; cursor: pointer; font-size: 14px; font-weight: 600; }
        button.btn-danger { background: #dc3545; color: white; border: none; padding: 10px 14px; border-radius: 6px; cursor: pointer; font-size: 14px; font-weight: 600; }
        .status-msg { text-align: center; font-weight: 600; margin-top: 10px; min-height: 20px; font-size: 14px; }
        .footer { text-align: center; margin-top: 24px; font-size: 12px; color: #666; }
        .tabcontent { display: none; }
        
        table { width: 100%; border-collapse: collapse; margin-top: 10px; font-size: 13px; }
        th, td { border: 1px solid #dee2e6; padding: 6px; text-align: center; }
        th { background-color: #f1f5f9; color: #1e293b; font-weight: 600; }
        td input, td select { width: 100%; padding: 5px; border: 1px solid #cbd5e1; border-radius: 4px; text-align: center; margin: 0; font-family: monospace; font-size: 13px; }
        @media screen and (max-width: 680px) {
            .navbar { flex-direction: column; text-align: center; }
            .nav-links { justify-content: center; }
            .grid-2, .grid-3 { grid-template-columns: 1fr; gap: 6px; }
            table { font-size: 11px; }
            th, td { padding: 3px; }
        }
    </style>
</head>
<body>
    <div class="navbar">
        <h2>ASELSAN 4822 <span id="navCallsign" class="badge-ver">%CALLSIGN%</span> <span class="badge-ver">%VERSION%</span></h2>
        <div class="nav-links">
            <button class="tablinks active" onclick="openTab(event, 'Ayarlar')">1. Cihaz Ayarları</button>
            <button class="tablinks" onclick="openTab(event, 'APRS')">2. APRS</button>
            <button class="tablinks" onclick="openTab(event, 'Kanallar')">3. Kanallar (CHIRP)</button>
            <button class="tablinks" onclick="openTab(event, 'Konsol')">4. Konsol</button>
            <button class="tablinks" onclick="openTab(event, 'OTA')">5. Güncelleme</button>
        </div>
    </div>

    <div class="container">

        <!-- ================================================================
             1. MENU: CIHAZ AYARLARI
             ================================================================ -->
        <div id="Ayarlar" class="tabcontent" style="display:block;">
            <h3>Telsiz Canlı Kontrol &amp; Çalışma Ayarları</h3>
            <div class="section-box">
                <div class="grid-3">
                    <div>
                        <label>Çıkış Gücü (RF Power):</label>
                        <select id="rf_power">
                            <option value="0">Yüksek Güç (High - 25W)</option>
                            <option value="1">Düşük Güç (Low - 5W)</option>
                        </select>
                    </div>
                    <div>
                        <label>Susturma (Squelch Kademesi):</label>
                        <select id="sql_mode">
                            <option value="0">0 - Kapalı (Monitör / Sürekli Açık)</option>
                            <option value="1">1 - Çok Hassas (Zayıf Sinyaller)</option>
                            <option value="2">2 - Hassas</option>
                            <option value="3">3 - Normal (Varsayılan)</option>
                            <option value="4">4 - Sıkı (Parazit Önleyici)</option>
                            <option value="5">5 - Çok Sıkı (Maksimum Filtre)</option>
                        </select>
                    </div>
                    <div>
                        <label>Çalışma Modu (VFO / Hafıza):</label>
                        <select id="op_mode">
                            <option value="0">VFO (Serbest Frekans)</option>
                            <option value="1">MR (Hafıza Kanalı)</option>
                        </select>
                    </div>
                </div>

                <div class="grid-3">
                    <div>
                        <label>Aktif VFO Frekansı (KHz):</label>
                        <input type="number" id="vfo_freq" step="25" min="134000" max="470000" placeholder="145575">
                    </div>
                    <div>
                        <label>Röle Shift Yönü:</label>
                        <select id="vfo_shift_dir">
                            <option value="0">Simplex (Shift Yok)</option>
                            <option value="-1">- Shift (Eksi)</option>
                            <option value="1">+ Shift (Artı)</option>
                        </select>
                    </div>
                    <div>
                        <label>Röle Shift Miktarı (KHz):</label>
                        <input type="number" id="vfo_shift" step="25" min="0" max="10000" placeholder="600">
                    </div>
                </div>

                <div class="grid-3">
                    <div>
                        <label>CTCSS Röle Tonu Durumu:</label>
                        <select id="vfo_tone_en">
                            <option value="0">Kapalı (Ton Yok)</option>
                            <option value="1">Açık (CTCSS Aktif ♪)</option>
                        </select>
                    </div>
                    <div>
                        <label>CTCSS Ton Frekansı (Hz):</label>
                        <select id="vfo_tone_pos"></select>
                    </div>
                    <div>
                        <label>Gönderme Süre Sınırı (TOT sn):</label>
                        <input type="number" id="tot_time" min="0" max="225" step="15" placeholder="60">
                    </div>
                </div>
            </div>

            <h3>Kimlik &amp; WiFi Hotspot Ayarları</h3>
            <div class="section-box">
                <div class="grid-3">
                    <div>
                        <label>Çağrı İşareti (Callsign):</label>
                        <input type="text" id="callsign" maxlength="6" style="text-transform:uppercase; font-family:monospace; font-weight:bold;">
                    </div>
                    <div>
                        <label>LCD Açılış Mesajı (Maks 6 krk):</label>
                        <input type="text" id="startup" maxlength="6" style="text-transform:uppercase; font-family:monospace;">
                    </div>
                    <div>
                        <label>WiFi Şifresi (En az 8 krk):</label>
                        <input type="text" id="wifipass" minlength="8" maxlength="32">
                    </div>
                </div>
                <div class="grid-2">
                    <div>
                        <label>Cihaz Bandı (TAMSAT Kart Tipi):</label>
                        <select id="radio_type">
                            <option value="0">VHF (134 - 174 MHz - ASELSAN 4822)</option>
                            <option value="1">UHF (400 - 470 MHz - ASELSAN 4826)</option>
                        </select>
                    </div>
                    <div style="display:flex; align-items:flex-end;">
                        <button type="button" class="btn-success" onclick="saveDeviceSettings()">Cihaz Ayarlarını Kaydet ve Uygula</button>
                    </div>
                </div>
                <div id="deviceSaveStatus" class="status-msg"></div>
            </div>

            <h3>Sistem ve Sıfırlama İşlemleri</h3>
            <div class="section-box" style="border-color: #f5c2c7; background: #fff5f5;">
                <p style="margin-top:0; font-size:13px; color:#555;">
                    <b>Yeniden Başlat:</b> Telsizi ayarları silmeden yeniden başlatır (Çağrı işareti veya WiFi şifresi değiştirdiyseniz yeni WiFi adının aktif olması için kullanın).<br>
                    <b>Fabrika Ayarlarına Sıfırla:</b> Tüm ayarları ve kanalları siler, WiFi ağını tekrar <b>DLS_ASELSAN_KIT</b> (şifre: <b>deeplabstudio</b>) ilk kurulum moduna döndürür.
                </p>
                <div class="grid-2">
                    <button type="button" class="btn-warning" onclick="rebootDevice()">Cihazı Yeniden Başlat (Reboot)</button>
                    <button type="button" class="btn-danger" onclick="factoryResetDevice()">Fabrika Ayarlarına Sıfırla (İlk Kurulum Modu)</button>
                </div>
            </div>
        </div>

        <!-- ================================================================
             2. MENU: APRS AYARLARI
             ================================================================ -->
        <div id="APRS" class="tabcontent">
            <h3>APRS Konum, SSID ve Beacon Ayarları</h3>
            <div class="section-box">
                <div class="grid-3">
                    <div>
                        <label>APRS Periyodik Beacon:</label>
                        <select id="aprs_enable">
                            <option value="0">Pasif (Kapalı)</option>
                            <option value="1">Aktif (Otomatik Gönderim)</option>
                        </select>
                    </div>
                    <div>
                        <label>APRS Çağrı İşareti:</label>
                        <input type="text" id="aprs_callsign" maxlength="6" style="text-transform:uppercase; font-family:monospace; font-weight:bold;" readonly>
                    </div>
                    <div>
                        <label>APRS SSID (0 - 15):</label>
                        <select id="aprs_ssid">
                            <option value="0">-0 (Sabit İstasyon)</option>
                            <option value="1">-1 (Digipeater / Ek)</option>
                            <option value="2">-2</option>
                            <option value="3">-3</option>
                            <option value="4">-4</option>
                            <option value="5">-5 (IGate / Mobil)</option>
                            <option value="6">-6 (Uydu / Özel)</option>
                            <option value="7">-7 (El Telsizi)</option>
                            <option value="8">-8 (Tekne / Deniz)</option>
                            <option value="9">-9 (Araç / Mobil Varsayılan)</option>
                            <option value="10">-10 (İnternet)</option>
                            <option value="11">-11 (Balon / Hava)</option>
                            <option value="12">-12</option>
                            <option value="13">-13 (Hava Durumu)</option>
                            <option value="14">-14 (Kamyon)</option>
                            <option value="15">-15</option>
                        </select>
                    </div>
                </div>

                <div class="grid-2">
                    <div>
                        <label>APRS Enlem (Latitude - DDMM.SSN):</label>
                        <input type="text" id="aprs_lat" maxlength="8" placeholder="3955.50N" style="font-family:monospace;">
                    </div>
                    <div>
                        <label>APRS Boylam (Longitude - DDDMM.SSE):</label>
                        <input type="text" id="aprs_lon" maxlength="8" placeholder="3250.22E" style="font-family:monospace;">
                    </div>
                </div>

                <div style="background:#eef4fb; padding:10px 12px; border-radius:6px; margin-bottom:14px; display:flex; flex-wrap:wrap; gap:10px; align-items:center; justify-content:space-between;">
                    <span style="font-size:13px; color:#1e3a8a;"><b>Konum Yardımcısı:</b> Telefon/bilgisayar GPS'inden veya ondalık koordinattan otomatik APRS formatına çevirin:</span>
                    <button type="button" class="btn-primary" style="width:auto; padding:7px 12px; font-size:13px;" onclick="fillGpsFromBrowser()">📍 Cihaz GPS Konumunu Al</button>
                </div>

                <div class="grid-3">
                    <div>
                        <label>Ondalık Enlem (Örn: 39.9250):</label>
                        <input type="number" id="dec_lat" step="0.0001" placeholder="39.9250">
                    </div>
                    <div>
                        <label>Ondalık Boylam (Örn: 32.8369):</label>
                        <input type="number" id="dec_lon" step="0.0001" placeholder="32.8369">
                    </div>
                    <div style="display:flex; align-items:flex-end; padding-bottom:10px;">
                        <button type="button" class="btn-warning" style="width:100%;" onclick="convertDecimalToAprs()">APRS Formatına Çevir</button>
                    </div>
                </div>

                <div class="grid-3">
                    <div style="grid-column: span 2;">
                        <label>APRS Durum Mesajı (Maks 28 karakter):</label>
                        <input type="text" id="aprsmsg" maxlength="28" placeholder="ASELSAN 4822 TAMSAT KIT">
                    </div>
                    <div>
                        <label>Gönderim Aralığı (Dakika):</label>
                        <input type="number" id="aprstime" min="1" max="99" placeholder="10">
                    </div>
                </div>

                <div class="grid-2">
                    <div>
                        <label>APRS Kara Frekansı (KHz):</label>
                        <input type="number" id="aprs_freq" step="25" placeholder="144800">
                    </div>
                    <div>
                        <label>ISS Uydu APRS Frekansı (KHz):</label>
                        <input type="number" id="iss_freq" step="25" placeholder="145825">
                    </div>
                </div>

                <div class="grid-2" style="margin-top:8px;">
                    <button type="button" class="btn-success" onclick="saveAprsSettings()">APRS Ayarlarını Kaydet</button>
                    <button type="button" class="btn-primary" onclick="sendAprsBeaconNow()">📡 Şimdi APRS Test Paketi Gönder</button>
                </div>
                <div id="aprsSaveStatus" class="status-msg"></div>
            </div>
        </div>

        <!-- ================================================================
             3. MENU: KANALLAR (CHIRP)
             ================================================================ -->
        <div id="Kanallar" class="tabcontent">
            <h3>Kanal Listesi (50 Hafıza Kanalı - CHIRP Uyumlu)</h3>
            <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:10px; margin-bottom:10px;">
                <span style="font-size:13px; color:#555;">Frekansları <b>KHz</b> cinsinden giriniz (Örn: <code>145575</code>, Shift: <code>600</code>).</span>
                <div style="display:flex; gap:8px; flex-wrap:wrap;">
                    <button type="button" class="btn-primary" style="width:auto; padding:8px 12px; font-size:13px;" onclick="exportChirpCsv()">📥 CHIRP CSV İndir</button>
                    <label style="margin:0; background:#0b2545; color:white; padding:8px 12px; border-radius:6px; cursor:pointer; font-size:13px;">
                        📤 CHIRP CSV Yükle
                        <input type="file" id="csvFileInput" accept=".csv" style="display:none;" onchange="importChirpCsv(event)">
                    </label>
                    <button type="button" class="btn-success" style="width:auto; padding:8px 16px; font-size:13px;" onclick="saveChannels()" id="btnSave">💾 Telsize Kaydet</button>
                </div>
            </div>
            <div id="channelSaveStatus" class="status-msg"></div>

            <div style="overflow-x:auto;">
                <table>
                    <thead>
                        <tr>
                            <th>CH</th>
                            <th>İsim (7 Krk)</th>
                            <th>Frekans (KHz)</th>
                            <th>Shift Yönü</th>
                            <th>Shift (KHz)</th>
                            <th>CTCSS Durum</th>
                            <th>CTCSS Ton (Hz)</th>
                            <th>İşlem</th>
                        </tr>
                    </thead>
                    <tbody id="channelBody">
                        <tr><td colspan="8">Kanallar yükleniyor...</td></tr>
                    </tbody>
                </table>
            </div>
            <div style="margin-top: 15px;">
                <button class="btn-success" onclick="saveChannels()" id="btnSaveBottom">💾 Tüm Kanalları Telsize Kaydet</button>
            </div>
        </div>

        <!-- ================================================================
             4. MENU: KONSOL
             ================================================================ -->
        <div id="Konsol" class="tabcontent">
            <h3>Seri Port Konsolu ve Canlı Loglar</h3>
            <textarea id="logArea" style="width:100%; height:260px; background:#0d1117; color:#3fb950; font-family:monospace; padding:10px; border-radius:6px; border:1px solid #30363d;" readonly></textarea>
            <form onsubmit="sendCommand(event)" style="display:flex; gap:8px; margin-top:8px;">
                <input type="text" id="cmdInput" placeholder="Seri komut yazın (örn: K, H, m, v) ve Gönder'e basın" autocomplete="off" required style="margin:0;">
                <button type="submit" class="btn-success" style="width:160px;">Gönder</button>
                <button type="button" class="btn-danger" style="width:130px;" onclick="clearLogs()">Temizle</button>
            </form>
        </div>

        <!-- ================================================================
             5. MENU: GUNCELLEME (OTA)
             ================================================================ -->
        <div id="OTA" class="tabcontent">
            <h3>OTA Kablosuz Yazılım Güncelleme</h3>
            <div class="section-box">
                <p style="margin-top:0;">PlatformIO derlemesinden çıkan <b>firmware.bin</b> dosyasını seçerek kablosuz yazılım yükleyebilirsiniz.</p>
                <form method="POST" action="/update" enctype="multipart/form-data">
                    <input type="file" name="update" accept=".bin" required>
                    <button type="submit" class="btn-success">Yazılımı Güncelle (.bin)</button>
                </form>
            </div>
        </div>

        <div class="footer">
            DEEP LAB STUDIO &bull; TAMSAT &amp; TA2DMX &copy; 2026
        </div>
    </div>

    <script>
    const CTCSS_TONES = [
        67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5,
        94.8, 97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3,
        131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2, 165.5, 167.9,
        171.3, 173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 250.3
    ];

    function buildToneOptions(selectedIndex) {
        let html = '';
        for (let i = 0; i < CTCSS_TONES.length; i++) {
            let sel = (i === Number(selectedIndex)) ? ' selected' : '';
            html += `<option value="${i}"${sel}>${CTCSS_TONES[i].toFixed(1)} Hz</option>`;
        }
        return html;
    }

    document.addEventListener("DOMContentLoaded", function() {
        document.getElementById('vfo_tone_pos').innerHTML = buildToneOptions(8);
        loadConfig();
    });

    function loadConfig() {
        fetch('/getConfig')
            .then(res => res.json())
            .then(data => {
                document.getElementById('callsign').value = data.callsign || '';
                document.getElementById('aprs_callsign').value = data.callsign || '';
                document.getElementById('navCallsign').innerText = data.callsign || '';
                document.getElementById('startup').value = data.startup || '';
                document.getElementById('wifipass').value = data.wifipass || '';
                document.getElementById('rf_power').value = data.rf_power;
                document.getElementById('sql_mode').value = data.sql_mode;
                document.getElementById('op_mode').value = data.op_mode;
                document.getElementById('radio_type').value = data.radio_type;
                document.getElementById('vfo_freq').value = data.vfo_freq;
                document.getElementById('vfo_shift_dir').value = data.vfo_shift_dir;
                document.getElementById('vfo_shift').value = data.vfo_shift;
                document.getElementById('vfo_tone_en').value = data.vfo_tone_en;
                document.getElementById('vfo_tone_pos').value = data.vfo_tone_pos;
                document.getElementById('tot_time').value = data.tot_time;

                document.getElementById('aprs_enable').value = data.aprs_enable;
                document.getElementById('aprs_ssid').value = data.aprs_ssid;
                document.getElementById('aprs_lat').value = data.aprs_lat || '';
                document.getElementById('aprs_lon').value = data.aprs_lon || '';
                document.getElementById('aprsmsg').value = data.aprsmsg || '';
                document.getElementById('aprstime').value = data.aprstime;
                document.getElementById('aprs_freq').value = data.aprs_freq;
                document.getElementById('iss_freq').value = data.iss_freq;
            }).catch(e => console.error(e));
    }

    function openTab(evt, tabName) {
        var i, tabcontent, tablinks;
        tabcontent = document.getElementsByClassName("tabcontent");
        for (i = 0; i < tabcontent.length; i++) {
            tabcontent[i].style.display = "none";
        }
        tablinks = document.getElementsByClassName("tablinks");
        for (i = 0; i < tablinks.length; i++) {
            tablinks[i].className = tablinks[i].className.replace(" active", "");
        }
        document.getElementById(tabName).style.display = "block";
        evt.currentTarget.className += " active";

        if (tabName === 'Kanallar') {
            loadChannels();
        }
    }

    function showStatus(id, msg, isErr) {
        let el = document.getElementById(id);
        el.style.color = isErr ? '#dc3545' : '#28a745';
        el.innerText = msg;
        setTimeout(() => { el.innerText = ''; }, 4500);
    }

    function saveDeviceSettings() {
        let cs = document.getElementById('callsign').value.trim().toUpperCase();
        let pw = document.getElementById('wifipass').value;
        if (cs.length < 3) { showStatus('deviceSaveStatus', 'Çağrı işareti en az 3 karakter olmalıdır!', true); return; }
        if (pw.length < 8) { showStatus('deviceSaveStatus', 'WiFi şifresi en az 8 karakter olmalıdır!', true); return; }

        let params = new URLSearchParams();
        params.append('callsign', cs);
        params.append('startup', document.getElementById('startup').value.toUpperCase());
        params.append('wifipass', pw);
        params.append('rf_power', document.getElementById('rf_power').value);
        params.append('sql_mode', document.getElementById('sql_mode').value);
        params.append('op_mode', document.getElementById('op_mode').value);
        params.append('radio_type', document.getElementById('radio_type').value);
        params.append('vfo_freq', document.getElementById('vfo_freq').value);
        params.append('vfo_shift_dir', document.getElementById('vfo_shift_dir').value);
        params.append('vfo_shift', document.getElementById('vfo_shift').value);
        params.append('vfo_tone_en', document.getElementById('vfo_tone_en').value);
        params.append('vfo_tone_pos', document.getElementById('vfo_tone_pos').value);
        params.append('tot_time', document.getElementById('tot_time').value);

        fetch('/saveDevice', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: params.toString()
        }).then(res => {
            if (res.ok) {
                showStatus('deviceSaveStatus', 'Cihaz ayarları kaydedildi ve telsize anında uygulandı!', false);
                loadConfig();
            } else {
                showStatus('deviceSaveStatus', 'Ayarlar kaydedilirken hata oluştu!', true);
            }
        });
    }

    function saveAprsSettings() {
        let params = new URLSearchParams();
        params.append('aprs_enable', document.getElementById('aprs_enable').value);
        params.append('aprs_ssid', document.getElementById('aprs_ssid').value);
        params.append('aprs_lat', document.getElementById('aprs_lat').value.trim().toUpperCase());
        params.append('aprs_lon', document.getElementById('aprs_lon').value.trim().toUpperCase());
        params.append('aprsmsg', document.getElementById('aprsmsg').value);
        params.append('aprstime', document.getElementById('aprstime').value);
        params.append('aprs_freq', document.getElementById('aprs_freq').value);
        params.append('iss_freq', document.getElementById('iss_freq').value);

        fetch('/saveAprs', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: params.toString()
        }).then(res => {
            if (res.ok) {
                showStatus('aprsSaveStatus', 'APRS ayarları başarıyla kaydedildi!', false);
                loadConfig();
            } else {
                showStatus('aprsSaveStatus', 'APRS ayarları kaydedilemedi!', true);
            }
        });
    }

    function sendAprsBeaconNow() {
        showStatus('aprsSaveStatus', 'APRS paketi gönderiliyor (TX)...', false);
        fetch('/sendAprsNow', { method: 'POST' })
            .then(res => {
                if (res.ok) showStatus('aprsSaveStatus', 'APRS paketi başarıyla havaya gönderildi!', false);
                else showStatus('aprsSaveStatus', 'Gönderim başarısız!', true);
            });
    }

    function decimalToAprsLat(lat) {
        let hemi = lat >= 0 ? 'N' : 'S';
        lat = Math.abs(lat);
        let deg = Math.floor(lat);
        let min = (lat - deg) * 60.0;
        let degStr = String(deg).padStart(2, '0');
        let minStr = min.toFixed(2).padStart(5, '0');
        return degStr + minStr + hemi; // 8 chars: 3955.50N
    }

    function decimalToAprsLon(lon) {
        let hemi = lon >= 0 ? 'E' : 'W';
        lon = Math.abs(lon);
        let deg = Math.floor(lon);
        let min = (lon - deg) * 60.0;
        let degStr = String(deg).padStart(2, '0');
        let minStr = min.toFixed(2).padStart(5, '0');
        return degStr + minStr + hemi; // 8 chars: 3250.22E
    }

    function convertDecimalToAprs() {
        let dlat = parseFloat(document.getElementById('dec_lat').value);
        let dlon = parseFloat(document.getElementById('dec_lon').value);
        if (isNaN(dlat) || isNaN(dlon)) {
            showStatus('aprsSaveStatus', 'Lütfen geçerli ondalık enlem ve boylam girin!', true);
            return;
        }
        document.getElementById('aprs_lat').value = decimalToAprsLat(dlat);
        document.getElementById('aprs_lon').value = decimalToAprsLon(dlon);
        showStatus('aprsSaveStatus', 'Koordinatlar APRS formatına çevrildi. Kaydet butonuna basmayı unutmayın.', false);
    }

    function fillGpsFromBrowser() {
        if (!navigator.geolocation) {
            showStatus('aprsSaveStatus', 'Tarayıcınız konum özelliğini desteklemiyor.', true);
            return;
        }
        navigator.geolocation.getCurrentPosition(function(pos) {
            document.getElementById('dec_lat').value = pos.coords.latitude.toFixed(5);
            document.getElementById('dec_lon').value = pos.coords.longitude.toFixed(5);
            convertDecimalToAprs();
        }, function(err) {
            showStatus('aprsSaveStatus', 'Konum alınamadı (HTTP bağlantıda tarayıcı izin vermeyebilir, ondalık kutuları kullanabilirsiniz).', true);
        });
    }

    function rebootDevice() {
        if (!confirm('Telsizi yeniden başlatmak istediğinize emin misiniz?')) return;
        fetch('/reboot', { method: 'POST' }).then(() => {
            alert('Cihaz yeniden başlatılıyor. Birkaç saniye sonra sayfayı yenileyin.');
        });
    }

    function factoryResetDevice() {
        if (!confirm('DİKKAT! Tüm kanallar, çağrı işareti ve WiFi şifresi silinecek ve cihaz DLS_ASELSAN_KIT ilk kurulum moduna dönecek. Onaylıyor musunuz?')) return;
        fetch('/factoryReset', { method: 'POST' }).then(() => {
            document.body.innerHTML = "<div style='text-align:center; padding:50px; font-family:sans-serif;'><h2>Fabrika Ayarlarına Dönüldü!</h2><p>Lütfen WiFi ayarlarınızdan <b>DLS_ASELSAN_KIT</b> ağına (şifre: <b>deeplabstudio</b>) bağlanıp 192.168.4.1 adresine giriniz.</p></div>";
        });
    }

    function updateLogs() {
        if (document.getElementById('Konsol').style.display === 'block') {
            fetch('/getLogs').then(response => response.text()).then(data => {
                var logArea = document.getElementById('logArea');
                if (logArea.value !== data) {
                    logArea.value = data;
                    logArea.scrollTop = logArea.scrollHeight;
                }
            }).catch(err => {});
        }
    }
    setInterval(updateLogs, 1000);

    function sendCommand(e) {
        e.preventDefault();
        var cmd = document.getElementById('cmdInput').value;
        fetch('/sendCommand', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: 'cmd=' + encodeURIComponent(cmd)
        }).then(() => {
            document.getElementById('cmdInput').value = '';
            setTimeout(updateLogs, 400);
        });
    }

    function clearLogs() {
        fetch('/clearLogs', { method: 'POST' }).then(() => {
            document.getElementById('logArea').value = '';
        });
    }

    let channelsData = [];
    function loadChannels() {
        fetch('/getChannels')
            .then(res => res.json())
            .then(data => {
                channelsData = data;
                renderChannels();
            })
            .catch(err => console.error("Kanallar alinamadi: ", err));
    }

    function createRow(i, ch) {
        let f = (ch && ch.f) ? ch.f : "";
        let sd = (ch && ch.sd !== undefined) ? ch.sd : 0;
        let s = (ch && ch.s !== undefined && ch.f) ? ch.s : "";
        let te = (ch && ch.te !== undefined) ? ch.te : 0;
        let t = (ch && ch.t !== undefined && ch.t < CTCSS_TONES.length) ? ch.t : 8;
        let n = (ch && ch.n) ? ch.n : "";
        return `<tr id="row_${i}">
            <td><b>${i+1}</b></td>
            <td><input type="text" id="n_${i}" maxlength="7" value="${n}" placeholder="BOŞ" style="text-transform:uppercase;"></td>
            <td><input type="number" id="f_${i}" step="25" value="${f}" placeholder="145500"></td>
            <td>
                <select id="sd_${i}">
                    <option value="0" ${sd == 0 ? 'selected' : ''}>Simplex</option>
                    <option value="-1" ${sd == -1 ? 'selected' : ''}>- Shift</option>
                    <option value="1" ${sd == 1 ? 'selected' : ''}>+ Shift</option>
                </select>
            </td>
            <td><input type="number" id="s_${i}" step="25" value="${s}" placeholder="600"></td>
            <td>
                <select id="te_${i}">
                    <option value="0" ${te == 0 ? 'selected' : ''}>Kapalı</option>
                    <option value="1" ${te == 1 ? 'selected' : ''}>Açık ♪</option>
                </select>
            </td>
            <td>
                <select id="t_${i}">${buildToneOptions(t)}</select>
            </td>
            <td><button type="button" onclick="clearRow(${i})" style="padding: 5px 8px; background: #dc3545; color: white; border: none; border-radius: 4px; cursor: pointer;">Sil</button></td>
        </tr>`;
    }

    function renderChannels() {
        let html = '';
        for (let i = 0; i < 50; i++) {
            let ch = channelsData[i];
            if (!ch || !ch.f || ch.f === 0 || ch.f >= 1638375) {
                ch = { f: "", s: "", sd: 0, te: 0, t: 8, n: "" };
            }
            html += createRow(i, ch);
        }
        document.getElementById('channelBody').innerHTML = html;
    }

    function clearRow(i) {
        document.getElementById(`f_${i}`).value = "";
        document.getElementById(`s_${i}`).value = "";
        document.getElementById(`sd_${i}`).value = "0";
        document.getElementById(`te_${i}`).value = "0";
        document.getElementById(`t_${i}`).value = "8";
        document.getElementById(`n_${i}`).value = "";
    }

    function saveChannels() {
        let btn1 = document.getElementById('btnSave');
        let btn2 = document.getElementById('btnSaveBottom');
        btn1.disabled = true; btn2.disabled = true;

        let channelsToSave = [];
        for (let i = 0; i < 50; i++) {
            let fVal = parseInt(document.getElementById(`f_${i}`).value);
            let sVal = parseInt(document.getElementById(`s_${i}`).value);
            let sdVal = parseInt(document.getElementById(`sd_${i}`).value);
            let teVal = parseInt(document.getElementById(`te_${i}`).value);
            let tVal = parseInt(document.getElementById(`t_${i}`).value);
            let nVal = document.getElementById(`n_${i}`).value.trim().toUpperCase();

            if (isNaN(fVal) || fVal <= 0) {
                channelsToSave.push({ f: 1638375, s: 0, sd: 0, te: 0, t: 8, n: "" });
            } else {
                if (nVal.length === 0) nVal = "CH-" + String(i + 1).padStart(2, '0');
                channelsToSave.push({
                    f: fVal,
                    s: isNaN(sVal) ? 0 : sVal,
                    sd: isNaN(sdVal) ? 0 : sdVal,
                    te: isNaN(teVal) ? 0 : teVal,
                    t: isNaN(tVal) ? 8 : tVal,
                    n: nVal
                });
            }
        }

        fetch('/saveChannels', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(channelsToSave)
        }).then(res => {
            btn1.disabled = false; btn2.disabled = false;
            if (res.ok) {
                showStatus('channelSaveStatus', 'Tüm kanallar telsize başarıyla kaydedildi!', false);
                loadChannels();
            } else {
                showStatus('channelSaveStatus', 'Kanal kaydetme sırasında hata oluştu!', true);
            }
        });
    }

    function exportChirpCsv() {
        let csv = "Location,Name,Frequency,Duplex,Offset,Tone,rToneFreq,cToneFreq,DtcsCode,DtcsPolarity,Mode,TStep,Skip,Comment,URCALL,RPT1CALL,RPT2CALL\n";
        for (let i = 0; i < 50; i++) {
            let fVal = parseInt(document.getElementById(`f_${i}`).value);
            if (isNaN(fVal) || fVal <= 0 || fVal >= 1638375) continue;
            let nVal = document.getElementById(`n_${i}`).value.trim().toUpperCase() || ("CH-" + (i + 1));
            let sdVal = parseInt(document.getElementById(`sd_${i}`).value);
            let sVal = parseInt(document.getElementById(`s_${i}`).value) || 0;
            let teVal = parseInt(document.getElementById(`te_${i}`).value);
            let tIdx = parseInt(document.getElementById(`t_${i}`).value) || 8;
            let toneHz = (CTCSS_TONES[tIdx] || 88.5).toFixed(1);
            let freqMhz = (fVal / 1000.0).toFixed(6);
            let offsetMhz = (sVal / 1000.0).toFixed(6);
            let duplex = sdVal === -1 ? "-" : (sdVal === 1 ? "+" : "");
            let toneMode = teVal === 1 ? "Tone" : "";
            csv += `${i+1},${nVal},${freqMhz},${duplex},${offsetMhz},${toneMode},${toneHz},${toneHz},023,NN,FM,25.00,,,,,\n`;
        }
        let blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
        let url = URL.createObjectURL(blob);
        let a = document.createElement('a');
        a.href = url;
        a.download = 'aselsan_4822_chirp_channels.csv';
        a.click();
    }

    function importChirpCsv(evt) {
        let file = evt.target.files[0];
        if (!file) return;
        let reader = new FileReader();
        reader.onload = function(e) {
            let lines = e.target.result.split(/\r?\n/);
            for (let i = 0; i < 50; i++) clearRow(i);
            let imported = 0;
            for (let l = 1; l < lines.length; l++) {
                let cols = lines[l].split(',');
                if (cols.length < 7) continue;
                let loc = parseInt(cols[0]) - 1;
                if (isNaN(loc) || loc < 0 || loc >= 50) continue;
                let name = (cols[1] || "").trim().toUpperCase().substring(0, 7);
                let freqKhz = Math.round(parseFloat(cols[2]) * 1000);
                if (isNaN(freqKhz) || freqKhz <= 0) continue;
                let duplex = (cols[3] || "").trim();
                let sd = duplex === "-" ? -1 : (duplex === "+" ? 1 : 0);
                let offsetKhz = Math.round(parseFloat(cols[4] || "0") * 1000);
                let toneMode = (cols[5] || "").trim();
                let te = (toneMode === "Tone" || toneMode === "TSQL") ? 1 : 0;
                let toneHz = parseFloat(cols[6] || "88.5");
                let bestIdx = 8;
                let minDiff = 999;
                for (let k = 0; k < CTCSS_TONES.length; k++) {
                    let d = Math.abs(CTCSS_TONES[k] - toneHz);
                    if (d < minDiff) { minDiff = d; bestIdx = k; }
                }
                document.getElementById(`n_${loc}`).value = name;
                document.getElementById(`f_${loc}`).value = freqKhz;
                document.getElementById(`sd_${loc}`).value = String(sd);
                document.getElementById(`s_${loc}`).value = offsetKhz;
                document.getElementById(`te_${loc}`).value = String(te);
                document.getElementById(`t_${loc}`).value = String(bestIdx);
                imported++;
            }
            showStatus('channelSaveStatus', imported + ' kanal CSV dosyasından tabloya yüklendi. "Telsize Kaydet" butonuna basınız.', false);
            evt.target.value = '';
        };
        reader.readAsText(file);
    }
    </script>
</body>
</html>
)rawliteral";

void handleRoot() {
    String versionString = "v" + String(SW_MAJOR) + "." + String(SW_MINOR);
    if (!isSetupDone) {
        String page = String(setupHtmlPage);
        page.replace("%VERSION%", versionString);
        server.send(200, "text/html", page);
    } else {
        String page = String(htmlPage);
        page.replace("%VERSION%", versionString);
        page.replace("%CALLSIGN%", trimStoredString(mycall));
        server.send(200, "text/html", page);
    }
}

void initWebUI() {
    if (wifiEnabled) return;

    Serial.println("Starting WiFi AP...");
    String ssid;
    String pass;

    if (!isSetupDone) {
        ssid = "DLS_ASELSAN_KIT";
        pass = "deeplabstudio";
        Serial.println("[WIFI] Ilk Kurulum Modu: SSID=DLS_ASELSAN_KIT");
    } else {
        String cleanCall = trimStoredString(mycall);
        if (cleanCall.length() == 0) cleanCall = "TA2DMX";
        ssid = "ASELSAN_" + cleanCall;
        pass = (wifiPass.length() >= 8) ? wifiPass : String("deeplabstudio");
        Serial.printf("[WIFI] Normal Mod: SSID=%s\r\n", ssid.c_str());
    }

    WiFi.defaultLowPowerMode();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str(), pass.c_str());

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP adresi: ");
    Serial.println(IP);

    server.on("/", HTTP_GET, handleRoot);

    // ------------------------------------------------------------------------
    // ILK KURULUM KAYIT ENDPOINT'I (/initialSetup)
    // ------------------------------------------------------------------------
    server.on("/initialSetup", HTTP_POST, []() {
        String cs = server.arg("callsign");
        cs.trim();
        cs.toUpperCase();
        if (cs.length() > 6) cs = cs.substring(0, 6);

        String pw = server.arg("wifipass");
        pw.trim();

        if (cs.length() < 3 || pw.length() < 8) {
            server.send(400, "text/html", "<html><head><meta charset='UTF-8'></head><body><h3>Hata: Çağrı işareti (3-6 karakter) ve WiFi şifresi (en az 8 karakter) geçerli olmalıdır!</h3><a href='/'>Geri Dön</a></body></html>");
            return;
        }

        mycall = cs;
        eewrite_nbytes(mycall, 6, EEPROM_CLLSIGN_ADDR);
        eewrite_nbytes(mycall, 6, EEPROM_APRSCLL_ADDR);
        eewrite_nbytes(mycall, 6, EEPROM_MESSAGE_ADDR); // Acilis mesajini da cagri isareti yap

        wifiPass = pw;
        for (uint8_t i = 0; i <= WIFI_PASS_MAX_LEN; i++) {
            uint8_t c = (i < wifiPass.length()) ? (uint8_t)wifiPass[i] : 0;
            EEPROM.write(EEPROM_WIFI_PASS_ADDR + i, c);
        }

        isSetupDone = true;
        EEPROM.write(EEPROM_SETUP_DONE_ADDR, SETUP_MAGIC_DONE);
        EEPROM.commit();

        String newSsid = "ASELSAN_" + cs;
        String resp = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
                      "<style>body{font-family:sans-serif;background:#0b2545;color:#fff;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0;padding:20px;}"
                      ".box{background:#fff;color:#222;padding:30px;border-radius:12px;max-width:420px;text-align:center;box-shadow:0 8px 24px rgba(0,0,0,0.3);}</style></head>"
                      "<body><div class='box'><h2 style='color:#28a745;margin-top:0;'>Kurulum Tamamlandı!</h2>"
                      "<p>Telsiziniz yeniden başlatılıyor...</p>"
                      "<p>Lütfen birkaç saniye sonra WiFi ayarlarınızdan:<br><b style='font-size:18px;color:#0056b3;'>" + newSsid + "</b><br>ağına belirlediğiniz yeni şifre ile bağlanıp <b>http://192.168.4.1</b> sayfasını yenileyin.</p>"
                      "</div></body></html>";
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", resp);
        delay(1200);
        rp2040.reboot();
    });

    // ------------------------------------------------------------------------
    // TUM AYARLARI OKUMA (/getConfig)
    // ------------------------------------------------------------------------
    server.on("/getConfig", HTTP_GET, []() {
        String startupMsg = "";
        eeread_nbytes(&startupMsg, 6, EEPROM_MESSAGE_ADDR);
        startupMsg = trimStoredString(startupMsg);

        String cleanCall = trimStoredString(mycall);
        String cleanAprsMsg = trimStoredString(APRS_Message);
        String cleanLat = trimStoredString(lat);
        String cleanLon = trimStoredString(lon);

        String json = "{";
        json += "\"callsign\":\"" + jsonEscape(cleanCall) + "\",";
        json += "\"startup\":\"" + jsonEscape(startupMsg) + "\",";
        json += "\"wifipass\":\"" + jsonEscape(wifiPass) + "\",";
        json += "\"rf_power\":" + String(RF_POWER_STATE) + ",";
        json += "\"sql_mode\":" + String(SQL_MODE == SQL_OFF ? 0 : sql_level) + ",";
        json += "\"op_mode\":" + String(op_mode) + ",";
        json += "\"radio_type\":" + String(radio_type) + ",";
        json += "\"vfo_freq\":" + String(current_ch.frequency) + ",";
        json += "\"vfo_shift_dir\":" + String(current_ch.shift_dir) + ",";
        json += "\"vfo_shift\":" + String(current_ch.shift) + ",";
        json += "\"vfo_tone_en\":" + String(current_ch.tone_enabled) + ",";
        json += "\"vfo_tone_pos\":" + String(current_ch.tone_pos) + ",";
        json += "\"tot_time\":" + String(tot_values.time_out_time) + ",";
        json += "\"aprs_enable\":" + String(APRS_Timeout > 0 ? 1 : 0) + ",";
        json += "\"aprs_ssid\":" + String((int)myssid) + ",";
        json += "\"aprs_lat\":\"" + jsonEscape(cleanLat) + "\",";
        json += "\"aprs_lon\":\"" + jsonEscape(cleanLon) + "\",";
        json += "\"aprsmsg\":\"" + jsonEscape(cleanAprsMsg) + "\",";
        json += "\"aprstime\":" + String(APRS_Timeout > 0 ? APRS_Timeout : 10) + ",";
        json += "\"aprs_freq\":" + String((uint32_t)freqLimits.aprs_25 * 25UL) + ",";
        json += "\"iss_freq\":" + String((uint32_t)freqLimits.iss_25 * 25UL);
        json += "}";
        server.send(200, "application/json", json);
    });

    // ------------------------------------------------------------------------
    // 1. MENU: CIHAZ AYARLARINI KAYDET VE CANLI UYGULA (/saveDevice)
    // ------------------------------------------------------------------------
    server.on("/saveDevice", HTTP_POST, []() {
        if (server.hasArg("callsign")) {
            String cs = server.arg("callsign");
            cs.trim();
            cs.toUpperCase();
            if (cs.length() >= 3) {
                mycall = cs.substring(0, 6);
                eewrite_nbytes(mycall, 6, EEPROM_CLLSIGN_ADDR);
                eewrite_nbytes(mycall, 6, EEPROM_APRSCLL_ADDR);
            }
        }
        if (server.hasArg("startup")) {
            String st = server.arg("startup");
            st.trim();
            st.toUpperCase();
            eewrite_nbytes(st, 6, EEPROM_MESSAGE_ADDR);
        }
        if (server.hasArg("wifipass")) {
            String pw = server.arg("wifipass");
            pw.trim();
            if (pw.length() >= 8 && pw.length() <= WIFI_PASS_MAX_LEN) {
                wifiPass = pw;
                for (uint8_t i = 0; i <= WIFI_PASS_MAX_LEN; i++) {
                    uint8_t c = (i < wifiPass.length()) ? (uint8_t)wifiPass[i] : 0;
                    EEPROM.write(EEPROM_WIFI_PASS_ADDR + i, c);
                }
            }
        }
        if (server.hasArg("rf_power")) {
            RF_POWER_STATE = (server.arg("rf_power").toInt() == 1) ? LOW_POWER : HIGH_POWER;
            EEPROM.write(EEPROM_RFPOWER_ADDR, RF_POWER_STATE);
            SetRFPower();
        }
        if (server.hasArg("sql_mode")) {
            int sm = server.arg("sql_mode").toInt();
            if (sm <= 0) {
                SQL_MODE = SQL_OFF;
            } else {
                SQL_MODE = SQL_ON;
                if (sm > 5) sm = 5;
                sql_level = (uint8_t)sm;
                EEPROM.write(EEPROM_SQLLEVEL_ADDR, sql_level);
            }
            EEPROM.write(EEPROM_SQLMODE_ADDR, (uint8_t)SQL_MODE);
        }
        if (server.hasArg("radio_type")) {
            uint8_t rt = (server.arg("radio_type").toInt() == 1) ? 1 : 0;
            if (rt != radio_type) {
                radio_type = rt;
                EEPROM.write(EEPROM_RADIOTP_ADDR, radio_type);
            }
        }
        if (server.hasArg("tot_time")) {
            int tt = server.arg("tot_time").toInt();
            if (tt >= 0 && tt <= 240) {
                tot_values.time_out_time = (uint8_t)tt;
                EEPROM.put(EEPROM_TOT, tot_values);
            }
        }
        if (server.hasArg("vfo_freq")) {
            uint32_t vf = (uint32_t)server.arg("vfo_freq").toInt();
            if (vf >= 134000UL && vf <= 470000UL) {
                current_ch.frequency = (vf / 25UL) * 25UL;
            }
        }
        if (server.hasArg("vfo_shift_dir")) {
            int sd = server.arg("vfo_shift_dir").toInt();
            if (sd == -1 || sd == 0 || sd == 1) current_ch.shift_dir = (int8_t)sd;
        }
        if (server.hasArg("vfo_shift")) {
            int sh = server.arg("vfo_shift").toInt();
            if (sh >= 0 && sh <= 20000) current_ch.shift = (int16_t)((sh / 25) * 25);
        }
        if (server.hasArg("vfo_tone_en")) {
            current_ch.tone_enabled = (server.arg("vfo_tone_en").toInt() == 1) ? CTCSS_ON : CTCSS_OFF;
        }
        if (server.hasArg("vfo_tone_pos")) {
            int tp = server.arg("vfo_tone_pos").toInt();
            if (tp >= 0 && tp < TOTAL_TONES) current_ch.tone_pos = (uint8_t)tp;
        }
        if (server.hasArg("op_mode")) {
            uint8_t newMode = (server.arg("op_mode").toInt() == 1) ? MODE_MR : MODE_VFO;
            op_mode = newMode;
            EEPROM.write(EEPROM_OP_MODE_ADDR, op_mode);
        }

        EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
        EEPROM.commit();

        // Canli olarak telsize ve LCD'ye uygula
        if (op_mode == MODE_MR) {
            char chStr[3];
            sprintf(chStr, "%02d", current_memory_channel);
            GetMemoryChannel(chStr);
        } else {
            numberToFrequency(current_ch.frequency, FRQ);
            strcpy(FRQ_old, FRQ);
            validFRQ = Calculate_Frequency(FRQ);
        }
        SetTone(current_ch.tone_enabled);
        write_FRQ(current_ch.frequency);
        writeFRQToLcd(FRQ);
        Alert_Tone(OK_tone);

        server.send(200, "text/plain", "OK");
    });

    // ------------------------------------------------------------------------
    // 2. MENU: APRS AYARLARINI KAYDET (/saveAprs) VE TEST PAKETI (/sendAprsNow)
    // ------------------------------------------------------------------------
    server.on("/saveAprs", HTTP_POST, []() {
        if (server.hasArg("aprs_ssid")) {
            int ssid = server.arg("aprs_ssid").toInt();
            if (ssid >= 0 && ssid <= 15) myssid = (char)ssid;
        }
        if (server.hasArg("aprs_lat")) {
            String l = server.arg("aprs_lat");
            l.trim();
            l.toUpperCase();
            if (l.length() > 0) lat = l;
        }
        if (server.hasArg("aprs_lon")) {
            String ln = server.arg("aprs_lon");
            ln.trim();
            ln.toUpperCase();
            if (ln.length() > 0) lon = ln;
        }
        if (server.hasArg("aprsmsg")) {
            APRS_Message = server.arg("aprsmsg");
        }
        if (server.hasArg("aprs_enable")) {
            if (server.arg("aprs_enable") == "0") {
                APRS_Timeout = 0;
            } else {
                int t = server.arg("aprstime").toInt();
                APRS_Timeout = (t > 0 && t <= 99) ? (unsigned int)t : 10;
            }
        }
        if (server.hasArg("aprs_freq")) {
            uint32_t af = (uint32_t)server.arg("aprs_freq").toInt();
            if (af >= 134000UL && af <= 470000UL) freqLimits.aprs_25 = (uint16_t)(af / 25UL);
        }
        if (server.hasArg("iss_freq")) {
            uint32_t isf = (uint32_t)server.arg("iss_freq").toInt();
            if (isf >= 134000UL && isf <= 470000UL) freqLimits.iss_25 = (uint16_t)(isf / 25UL);
        }

        EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART, freqLimits);
        eeprom_writeAPRS();
        EEPROM.commit();
        Alert_Tone(OK_tone);

        server.send(200, "text/plain", "OK");
    });

    server.on("/sendAprsNow", HTTP_POST, []() {
        server.send(200, "text/plain", "OK");
        send_packet(_FIXPOS_STATUS, (uint32_t)freqLimits.aprs_25 * 25UL);
    });

    // ------------------------------------------------------------------------
    // REBOOT VE FABRIKA AYARLARINA SIFIRLAMA (/reboot, /factoryReset)
    // ------------------------------------------------------------------------
    server.on("/reboot", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", "Rebooting...");
        delay(800);
        rp2040.reboot();
    });

    server.on("/factoryReset", HTTP_POST, []() {
        EEPROM.write(EEPROM_SETUP_DONE_ADDR, 0x00);
        EEPROM.write(0, 129); // Force initialize_eeprom() on next boot
        EEPROM.commit();
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", "<html><head><meta charset='UTF-8'></head><body><h2>Fabrika Ayarlarına Dönüldü!</h2></body></html>");
        delay(1000);
        rp2040.reboot();
    });

    // ------------------------------------------------------------------------
    // 3. MENU: KANALLAR (/getChannels, /saveChannels)
    // ------------------------------------------------------------------------
    server.on("/getChannels", HTTP_GET, []() {
        String json = "[";
        for (int ch = 0; ch < 50; ch++) {
            memorych_t m;
            uint16_t loc = EEPROM_MEMDATA_BLCKSTART + (ch + 1) * EEPROM_CHNNL_SIZE;
            EEPROM.get(loc, m);
            int8_t sd = ((m.SSTP) & 0x03) - 1;
            uint8_t te = ((m.SSTP) & 0x04) >> 2;
            if (m.frequency25 == 0 || m.frequency25 >= 65535) {
                sd = 0;
                te = 0;
            }
            json += "{\"f\":";
            json += String((uint32_t)m.frequency25 * 25UL);
            json += ",\"s\":";
            json += String((uint16_t)m.shift25 * 25U);
            json += ",\"sd\":";
            json += String((int)sd);
            json += ",\"te\":";
            json += String((int)te);
            json += ",\"t\":";
            json += String(m.tone_position < TOTAL_TONES ? m.tone_position : 8);
            json += ",\"n\":\"";
            String chName = "";
            for (int i = 0; i < 7; i++) {
                char c = m.ChannelName[i];
                if (c == 0 || (uint8_t)c == 0xFF) break;
                if (c >= 32 && c <= 126 && c != '"' && c != '\\') chName += c;
            }
            chName.trim();
            json += chName + "\"}";
            if (ch < 49) json += ",";
        }
        json += "]";
        server.send(200, "application/json", json);
    });

    server.on("/saveChannels", HTTP_POST, []() {
        if (!server.hasArg("plain")) {
            server.send(400, "text/plain", "Body not found");
            return;
        }

        String json = server.arg("plain");
        DynamicJsonDocument doc(12288);
        DeserializationError error = deserializeJson(doc, json);

        if (error) {
            server.send(400, "text/plain", "Invalid JSON");
            return;
        }

        JsonArray arr = doc.as<JsonArray>();
        int count = 0;
        for (JsonObject repo : arr) {
            if (count >= 50) break;

            uint32_t freq = repo["f"].as<uint32_t>();
            if (freq == 0) freq = 1638375;
            uint16_t shift = repo["s"].as<uint16_t>();
            int8_t sd = repo["sd"] | 0;
            uint8_t te = repo["te"] | 0;
            uint8_t tone = repo["t"] | 8;
            if (tone >= TOTAL_TONES) tone = 8;
            String name = repo["n"].as<String>();
            name.trim();
            name.toUpperCase();

            memorych_t m;
            uint16_t loc = EEPROM_MEMDATA_BLCKSTART + (count + 1) * EEPROM_CHNNL_SIZE;

            if (freq >= 1638375) {
                m.frequency25 = 65535;
                m.shift25 = 0;
                m.tone_position = 0;
                m.SSTP = 1; // noSHIFT (0 + 1), tone off
                memset(m.ChannelName, 0x00, 7);
            } else {
                m.frequency25 = (uint16_t)(freq / 25UL);
                m.shift25 = (uint8_t)(shift / 25U);
                m.tone_position = tone;
                m.SSTP = (uint8_t)((sd + 1) & 0x03) | ((te & 0x01) << 2);
                for (int j = 0; j < 7; j++) {
                    if (j < (int)name.length()) m.ChannelName[j] = name[j];
                    else m.ChannelName[j] = ' ';
                }
            }

            EEPROM.put(loc, m);
            count++;
        }
        EEPROM.commit();

        if (op_mode == MODE_MR) {
            char chStr[3];
            sprintf(chStr, "%02d", current_memory_channel);
            GetMemoryChannel(chStr);
            SetTone(current_ch.tone_enabled);
            write_FRQ(current_ch.frequency);
            writeFRQToLcd(FRQ);
        }
        Alert_Tone(OK_tone);
        server.send(200, "text/plain", "OK");
    });

    // ------------------------------------------------------------------------
    // 4. MENU: KONSOL LOGLARI VE KOMUT (/getLogs, /clearLogs, /sendCommand)
    // ------------------------------------------------------------------------
    server.on("/getLogs", HTTP_GET, []() {
        server.send(200, "text/plain", webLogBuffer);
    });

    server.on("/clearLogs", HTTP_POST, []() {
        webLogBuffer = "";
        server.send(200, "text/plain", "OK");
    });

    server.on("/sendCommand", HTTP_POST, []() {
        if (server.hasArg("cmd")) {
            commandString = server.arg("cmd") + "\n";
            commandComplete = true;
        }
        server.send(200, "text/plain", "OK");
    });

    // ------------------------------------------------------------------------
    // 5. MENU: OTA GUNCELLEME (/update)
    // ------------------------------------------------------------------------
    server.on("/update", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "Guncelleme BASARISIZ!" : "Guncelleme BASARILI! Cihaz yeniden baslatiliyor...");
        delay(1000);
        rp2040.reboot();
    }, []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin(upload.totalSize > 0 ? upload.totalSize : 2000000)) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) {
                Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
                Update.printError(Serial);
            }
        }
    });

    server.begin();
    Serial.println("HTTP sunucusu basladi.");
    wifiEnabled = true;
}

void handleWebUI() {
    if (wifiEnabled) {
        server.handleClient();
    }
}

#endif
