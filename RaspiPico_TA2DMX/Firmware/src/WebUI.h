#ifndef WEBUI_H
#define WEBUI_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Updater.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "main.h"

extern uint8_t _FS_start;
extern uint8_t _FS_end;

extern String mycall;
extern String APRS_Message;
extern unsigned int APRS_Timeout;
extern void eewrite_nbytes(String dst, uint8_t sz, uint16_t addr);
extern String webLogBuffer;
extern String commandString;
extern bool commandComplete;

WebServer server(80);
bool wifiEnabled = false;
static String otaErrorMsg = "";

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

static void sendCorsHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static String getCurrentDeviceIp() {
    if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
        return WiFi.localIP().toString();
    }
    return WiFi.softAPIP().toString();
}

// ============================================================================
// 1) ILK KURULUM EKRANI (DLS_ASELSAN_KIT / deeplabstudio)
// ============================================================================
const char* setupHtmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <title>DLS ASELSAN 4822/4826 - İlk Kurulum</title>
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
            max-width: 460px;
            border-radius: 14px;
            padding: 28px 24px;
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
            margin: 0 0 20px 0;
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
        .band-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-bottom: 18px;
        }
        .band-option {
            border: 2px solid #cbd5e1;
            border-radius: 10px;
            padding: 12px 10px;
            text-align: center;
            cursor: pointer;
            transition: all 0.2s;
            background: #f8fafc;
        }
        .band-option input {
            display: none;
        }
        .band-option .b-title {
            font-weight: 800;
            font-size: 16px;
            color: #0b2545;
            display: block;
        }
        .band-option .b-sub {
            font-size: 12px;
            color: #64748b;
            margin-top: 3px;
            display: block;
        }
        .band-option.selected {
            border-color: #0056b3;
            background: #eff6ff;
            box-shadow: 0 0 0 3px rgba(0,86,179,0.15);
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
        <p class="sub">Hoş geldiniz! Telsizinizin modelini (VHF/UHF), çağrı işaretinizi ve WiFi şifrenizi belirleyin. Kaydettikten sonra cihaz <b>ASELSAN_ÇAĞRIİŞARETİ</b> adıyla WiFi açacaktır.</p>
        <form method="POST" action="/initialSetup" onsubmit="return validateSetup()">
            <label>Telsiz Modeli / Bandı Seçiniz:</label>
            <div class="band-grid">
                <label class="band-option %SEL_VHF%" id="opt_vhf" onclick="selectBand(0)">
                    <input type="radio" name="radio_type" id="rt_0" value="0" %CHK_VHF%>
                    <span class="b-title">VHF &bull; MV 4822</span>
                    <span class="b-sub">134.000 - 174.000 MHz</span>
                </label>
                <label class="band-option %SEL_UHF%" id="opt_uhf" onclick="selectBand(1)">
                    <input type="radio" name="radio_type" id="rt_1" value="1" %CHK_UHF%>
                    <span class="b-title">UHF &bull; MV 4826</span>
                    <span class="b-sub">428.150 - 450.000 MHz</span>
                </label>
            </div>

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
        <div class="footer">TAMSAT &amp; <a href="https://deeplabstudio.com/" target="_blank" rel="noopener noreferrer" style="color:#0056b3; text-decoration:none; font-weight:600;">Deeplab Studio</a> (TA2DMX) &copy; 2026</div>
    </div>
    <script>
        function selectBand(b) {
            document.getElementById('rt_0').checked = (b === 0);
            document.getElementById('rt_1').checked = (b === 1);
            document.getElementById('opt_vhf').classList.toggle('selected', b === 0);
            document.getElementById('opt_uhf').classList.toggle('selected', b === 1);
        }
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
// 2) ANA WEB ARAYUZU (CANLI ON PANEL + 5 MENU)
// ============================================================================
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8"> 
    <title>ASELSAN 4822/4826 Yönetim Paneli</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #e8eef5; color: #333; margin: 0; padding: 0; }
        .navbar { background: linear-gradient(90deg, #0b2545 0%, #0056b3 100%); color: white; display: flex; justify-content: space-between; padding: 12px 20px; align-items: center; flex-wrap: wrap; gap: 10px; box-shadow: 0 2px 8px rgba(0,0,0,0.18); }
        .navbar h2 { margin: 0; font-size: 19px; display: flex; align-items: center; gap: 8px; }
        .badge-ver { background: rgba(255,255,255,0.2); padding: 3px 8px; border-radius: 12px; font-size: 12px; font-weight: normal; }
        .nav-links { display: flex; gap: 6px; flex-wrap: wrap; }
        .nav-links button { background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.4); color: white; padding: 8px 14px; border-radius: 6px; cursor: pointer; transition: 0.2s; font-size: 13px; font-weight: 500; }
        .nav-links button:hover, .nav-links button.active { background: white; color: #0b2545; border-color: white; font-weight: 600; }
        
        .container { max-width: 980px; margin: 18px auto; background: white; padding: 20px; border-radius: 12px; box-shadow: 0 4px 14px rgba(0,0,0,0.08); transition: max-width 0.25s; }
        .container.dual-expanded { max-width: 1560px; }
        .dual-radio-bar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            flex-wrap: wrap;
            gap: 10px;
            background: #eef4fb;
            border: 1px solid #cbd5e1;
            border-radius: 8px;
            padding: 8px 14px;
            margin-bottom: 14px;
            font-size: 13px;
        }
        .dual-radio-bar input[type="text"] {
            width: 135px;
            margin: 0;
            padding: 5px 8px;
            font-family: monospace;
            font-weight: bold;
            font-size: 13px;
        }
        .dual-rack {
            display: grid;
            grid-template-columns: 1fr;
            gap: 16px;
        }
        .dual-rack.active-dual {
            grid-template-columns: 1fr 1fr;
        }
        @media screen and (max-width: 1180px) {
            .dual-rack.active-dual { grid-template-columns: 1fr; }
        }
        h3 { color: #0b2545; margin-top: 0; border-bottom: 2px solid #e9ecef; padding-bottom: 8px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px; }
        .section-box { background: #f8fafd; border: 1px solid #e2e8f0; border-radius: 8px; padding: 16px; margin-bottom: 20px; }
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
        .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 14px; }
        .grid-4 { display: grid; grid-template-columns: 1fr 1fr 1fr 1fr; gap: 12px; }
        label { display: block; font-size: 12px; font-weight: 600; color: #444; margin-bottom: 4px; }
        input[type="text"], input[type="number"], input[type="password"], input[type="file"], select {
            width: 100%; padding: 8px 10px; margin-bottom: 8px; border: 1px solid #cbd5e1; border-radius: 6px; font-size: 14px; background: #fff;
        }
        button.btn-primary { background: #0056b3; color: white; border: none; padding: 10px 16px; border-radius: 6px; cursor: pointer; width: 100%; font-size: 14px; font-weight: 600; transition: 0.2s; }
        button.btn-primary:hover { background: #004494; }
        button.btn-success { background: #28a745; color: white; border: none; padding: 10px 16px; border-radius: 6px; cursor: pointer; width: 100%; font-size: 14px; font-weight: 600; transition: 0.2s; }
        button.btn-success:hover { background: #218838; }
        button.btn-warning { background: #f59e0b; color: white; border: none; padding: 10px 14px; border-radius: 6px; cursor: pointer; font-size: 14px; font-weight: 600; }
        button.btn-danger { background: #dc3545; color: white; border: none; padding: 10px 14px; border-radius: 6px; cursor: pointer; font-size: 14px; font-weight: 600; }
        .status-msg { text-align: center; font-weight: 600; margin-top: 8px; min-height: 20px; font-size: 13px; }
        .footer { text-align: center; margin-top: 24px; font-size: 12px; color: #666; }
        .tabcontent { display: none; }

        /* ====================================================================
           ASELSAN MV 4822 / 4826 NOMINAL ON PANEL KONSOLU (SEKIL 1-3)
           ==================================================================== */
        .radio-chassis {
            background: #23272b;
            border: 2px solid #14171a;
            border-radius: 10px;
            padding: 16px 18px;
            color: #e2e8f0;
            box-shadow: inset 0 1px 0 rgba(255,255,255,0.06), 0 4px 14px rgba(0,0,0,0.22);
            margin-bottom: 22px;
        }
        .radio-top-bar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
            padding-bottom: 6px;
            border-bottom: 1px solid #343a40;
        }
        .aselsan-brand {
            display: flex;
            align-items: baseline;
            gap: 10px;
        }
        .aselsan-logo {
            font-family: 'Arial Black', Impact, sans-serif;
            font-size: 18px;
            letter-spacing: 1.8px;
            color: #f8fafc;
        }
        .aselsan-model {
            font-family: 'Segoe UI', sans-serif;
            font-size: 12px;
            font-weight: 700;
            color: #94a3b8;
            letter-spacing: 0.6px;
        }
        .live-pill {
            display: inline-flex;
            align-items: center;
            gap: 6px;
            font-size: 11px;
            background: #1b2e28;
            color: #34d399;
            border: 1px solid #285446;
            padding: 3px 9px;
            border-radius: 4px;
            font-weight: 600;
            letter-spacing: 0.4px;
        }
        .live-dot {
            width: 7px;
            height: 7px;
            border-radius: 50%;
            background: #10b981;
        }

        .panel-main-grid {
            display: grid;
            grid-template-columns: 1fr auto;
            gap: 12px;
            align-items: stretch;
        }

        /* LCD Gosterge cercevesi */
        .lcd-bezel {
            background: #16191c;
            border: 2px solid #343a40;
            border-radius: 6px;
            padding: 8px 10px;
            box-shadow: inset 0 2px 5px rgba(0,0,0,0.7);
            display: flex;
            flex-direction: column;
            justify-content: center;
        }
        .lcd-glass {
            background: #8ca828;
            color: #151d04;
            border-radius: 4px;
            padding: 7px 14px 8px 14px;
            box-shadow: inset 0 1px 4px rgba(0,0,0,0.3);
            transition: background 0.15s;
        }
        .lcd-glass.tx-active {
            background: #d98218;
        }
        .lcd-symbols {
            display: flex;
            justify-content: space-between;
            align-items: center;
            font-size: 11px;
            font-weight: 800;
            border-bottom: 1px solid rgba(21, 29, 4, 0.22);
            padding-bottom: 3px;
            margin-bottom: 4px;
            font-family: 'Courier New', Consolas, monospace;
        }
        .sym-item {
            opacity: 0.22;
            padding: 1px 5px;
            border-radius: 2px;
            transition: opacity 0.12s;
        }
        .sym-item.on {
            opacity: 1;
            background: rgba(21, 29, 4, 0.12);
        }
        .lcd-readout-row {
            display: flex;
            justify-content: space-between;
            align-items: baseline;
            gap: 10px;
        }
        .lcd-mode-tag {
            font-family: 'Courier New', Consolas, monospace;
            font-size: 14px;
            font-weight: 900;
            background: rgba(21, 29, 4, 0.14);
            padding: 2px 6px;
            border-radius: 3px;
        }
        .lcd-digits {
            font-family: 'Courier New', Consolas, monospace;
            font-size: 36px;
            font-weight: 900;
            letter-spacing: 2.5px;
            line-height: 1.05;
            text-align: right;
            flex: 1;
        }
        .lcd-sub-info {
            display: flex;
            justify-content: space-between;
            font-family: 'Courier New', Consolas, monospace;
            font-size: 11.5px;
            font-weight: 700;
            margin-top: 4px;
            padding-top: 3px;
            border-top: 1px solid rgba(21, 29, 4, 0.18);
        }

        /* Sağdaki 3 durum LED'i + Yukarı/Aşağı Tuşları */
        .right-cluster {
            display: flex;
            gap: 10px;
            align-items: stretch;
        }
        .rocker-col {
            display: flex;
            flex-direction: column;
            gap: 6px;
            justify-content: space-between;
        }
        .btn-rocker {
            background: #343a40;
            color: #f1f5f9;
            border: 1px solid #495057;
            border-radius: 5px;
            width: 50px;
            flex: 1;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            box-shadow: 0 2px 0 #14171a;
            transition: background 0.1s, transform 0.08s;
        }
        .btn-rocker:hover { background: #3f474e; }
        .btn-rocker:active { transform: translateY(1px); box-shadow: none; }

        .led-col {
            background: #181b1f;
            border: 1px solid #343a40;
            border-radius: 6px;
            padding: 8px 12px;
            display: flex;
            flex-direction: column;
            justify-content: space-around;
            min-width: 110px;
        }
        .led-row {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 10.5px;
            font-weight: 700;
            color: #cbd5e1;
            letter-spacing: 0.4px;
        }
        .led-rect {
            width: 16px;
            height: 9px;
            border-radius: 2px;
            background: #2b3036;
            border: 1px solid #111;
            transition: all 0.12s;
        }
        .led-rect.red-on {
            background: #ef4444;
            box-shadow: 0 0 8px #ef4444;
        }
        .led-rect.green-on {
            background: #22c55e;
            box-shadow: 0 0 8px #22c55e;
        }
        .led-rect.amber-on {
            background: #f59e0b;
            box-shadow: 0 0 8px #f59e0b;
        }

        /* Ön Panel 6 Fonksiyon Tuşu (Nominal Endüstriyel Görünüm) */
        .panel-func-row {
            display: grid;
            grid-template-columns: repeat(6, 1fr);
            gap: 8px;
            margin-top: 10px;
        }
        .btn-panel {
            background: #31363c;
            color: #e2e8f0;
            border: 1px solid #454d55;
            border-radius: 5px;
            padding: 7px 4px;
            cursor: pointer;
            box-shadow: 0 2px 0 #14171a;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 2px;
            transition: all 0.1s;
        }
        .btn-panel:hover {
            background: #3b4148;
        }
        .btn-panel:active {
            transform: translateY(1px);
            box-shadow: none;
        }
        .btn-panel.active-func {
            background: #263849;
            border-color: #38bdf8;
            color: #ffffff;
        }
        .btn-panel .p-sym {
            font-family: 'Courier New', Consolas, monospace;
            font-size: 12px;
            font-weight: 800;
            color: #94a3b8;
            letter-spacing: 0.5px;
        }
        .btn-panel.active-func .p-sym {
            color: #38bdf8;
        }
        .btn-panel .p-lbl {
            font-size: 10.5px;
            font-weight: 700;
            letter-spacing: 0.3px;
        }

        /* Tek Satır Frekans & Adım Çubuğu */
        .quick-tune-bar {
            display: flex;
            flex-wrap: nowrap;
            gap: 6px;
            align-items: center;
            justify-content: space-between;
            background: #1b1e22;
            border: 1px solid #343a40;
            border-radius: 6px;
            padding: 7px 10px;
            margin-top: 10px;
        }
        .quick-tune-left {
            display: flex;
            align-items: center;
            gap: 6px;
        }
        .quick-tune-steps {
            display: flex;
            align-items: center;
            gap: 4px;
        }
        .quick-tune-bar input[type="text"] {
            width: 110px;
            margin: 0;
            padding: 5px 8px;
            font-family: 'Courier New', Consolas, monospace;
            font-size: 15px;
            font-weight: bold;
            text-align: center;
            background: #111315;
            color: #a3e635;
            border: 1px solid #495057;
            border-radius: 4px;
        }
        .btn-go {
            background: #0056b3;
            color: #fff;
            border: none;
            padding: 6px 12px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: 700;
            cursor: pointer;
            white-space: nowrap;
        }
        .btn-go:hover { background: #004494; }
        .btn-step {
            background: #2c3136;
            color: #cbd5e1;
            border: 1px solid #454d55;
            padding: 6px 8px;
            border-radius: 4px;
            font-size: 11.5px;
            font-weight: 700;
            cursor: pointer;
            white-space: nowrap;
        }
        .btn-step:hover { background: #3b4148; color: #fff; }

        /* Telsiz Gövdesi İçi Kompakt Alt Parametre Şeridi */
        .chassis-param-strip {
            display: grid;
            grid-template-columns: repeat(5, 1fr);
            gap: 8px;
            background: #1b1e22;
            border: 1px solid #343a40;
            border-radius: 6px;
            padding: 8px 10px;
            margin-top: 8px;
        }
        .cp-item label {
            display: block;
            font-size: 10.5px;
            font-weight: 700;
            color: #94a3b8;
            margin-bottom: 3px;
            letter-spacing: 0.2px;
        }
        .cp-item select, .cp-item input {
            width: 100%;
            margin: 0;
            padding: 5px 6px;
            font-size: 12px;
            font-weight: 600;
            background: #262b30;
            color: #f1f5f9;
            border: 1px solid #454d55;
            border-radius: 4px;
        }

        /* Canlı Web Ses (RX Hoparlör + VU Metre + Web PTT) Şeridi */
        .audio-trx-bar {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 10px;
            background: #181b1f;
            border: 1px solid #343a40;
            border-radius: 6px;
            padding: 7px 10px;
            margin-top: 8px;
            flex-wrap: wrap;
        }
        .btn-audio-rx {
            background: #2b3036;
            color: #e2e8f0;
            border: 1px solid #495057;
            border-radius: 5px;
            padding: 6px 12px;
            font-size: 12px;
            font-weight: 700;
            cursor: pointer;
            transition: 0.15s;
            white-space: nowrap;
        }
        .btn-audio-rx.listening {
            background: #065f46;
            border-color: #10b981;
            color: #fff;
            box-shadow: 0 0 8px rgba(16, 185, 129, 0.35);
        }
        .vu-box {
            flex: 1;
            min-width: 120px;
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .vu-lbl {
            font-size: 10.5px;
            font-weight: 700;
            color: #94a3b8;
            white-space: nowrap;
            font-family: monospace;
        }
        .vu-track {
            flex: 1;
            height: 10px;
            background: #111315;
            border: 1px solid #343a40;
            border-radius: 5px;
            overflow: hidden;
        }
        .vu-fill {
            height: 100%;
            width: 0%;
            background: linear-gradient(90deg, #22c55e 0%, #eab308 70%, #ef4444 100%);
            transition: width 0.12s ease-out;
        }
        .btn-web-ptt {
            background: #7f1d1d;
            color: #fecaca;
            border: 1px solid #b91c1c;
            border-radius: 5px;
            padding: 6px 14px;
            font-size: 12px;
            font-weight: 800;
            cursor: pointer;
            user-select: none;
            -webkit-user-select: none;
            transition: 0.1s;
            white-space: nowrap;
        }
        .btn-web-ptt.tx-down {
            background: #ef4444;
            color: #ffffff;
            border-color: #fca5a5;
            box-shadow: 0 0 12px #ef4444;
        }

        table { width: 100%; border-collapse: collapse; margin-top: 10px; font-size: 13px; }
        th, td { border: 1px solid #dee2e6; padding: 6px; text-align: center; }
        th { background-color: #f1f5f9; color: #1e293b; font-weight: 600; }
        td input, td select { width: 100%; padding: 5px; border: 1px solid #cbd5e1; border-radius: 4px; text-align: center; margin: 0; font-family: monospace; font-size: 13px; }

        .progress-wrap {
            width: 100%;
            background: #e2e8f0;
            border-radius: 8px;
            height: 22px;
            overflow: hidden;
            margin: 12px 0;
            display: none;
        }
        .progress-bar {
            height: 100%;
            width: 0%;
            background: linear-gradient(90deg, #0056b3, #28a745);
            color: white;
            font-size: 12px;
            font-weight: bold;
            text-align: center;
            line-height: 22px;
            transition: width 0.2s;
        }

        @media screen and (max-width: 720px) {
            .navbar { flex-direction: column; text-align: center; }
            .nav-links { justify-content: center; }
            .panel-main-grid { grid-template-columns: 1fr; }
            .right-cluster { justify-content: space-between; }
            .rocker-col { flex-direction: row; }
            .panel-func-row { grid-template-columns: repeat(3, 1fr); }
            .quick-tune-bar { flex-wrap: wrap; }
            .chassis-param-strip { grid-template-columns: 1fr 1fr; }
            .grid-2, .grid-3, .grid-4 { grid-template-columns: 1fr; gap: 6px; }
            .lcd-digits { font-size: 28px; }
            table { font-size: 11px; }
            th, td { padding: 3px; }
        }
    </style>
</head>
<body>
    <div class="navbar">
        <h2><span id="navModel">ASELSAN 4822</span> <span id="navCallsign" class="badge-ver">%CALLSIGN%</span> <span class="badge-ver">%VERSION%</span></h2>
        <div class="nav-links">
            <button class="tablinks active" onclick="openTab(event, 'Ayarlar')">1. Canlı Kontrol &amp; Ayarlar</button>
            <button class="tablinks" onclick="openTab(event, 'APRS')">2. APRS</button>
            <button class="tablinks" onclick="openTab(event, 'Kanallar')">3. Kanallar (CHIRP)</button>
            <button class="tablinks" onclick="openTab(event, 'Konsol')">4. Konsol</button>
            <button class="tablinks" onclick="openTab(event, 'OTA')">5. Sistem &amp; Güncelleme</button>
        </div>
    </div>

    <div class="container" id="mainContainer">

        <!-- ================================================================
             1. MENU: CANLI ON PANEL SIMULATORU & CIHAZ AYARLARI
             ================================================================ -->
        <div id="Ayarlar" class="tabcontent" style="display:block;">

            <!-- ÇİFT TELSİZ (VHF + UHF YAN YANA) OTOMATİK AĞ TARAMA & BAĞLANTI ÇUBUĞU -->
            <div class="dual-radio-bar">
                <div>
                    📡 <b>Bu Cihaz:</b> <code id="localIpBadge">192.168.4.1</code>
                    <span id="wifiModeBadge" style="margin-left:6px; font-size:12px; color:#0056b3; font-weight:600;">(AP Hotspot)</span>
                </div>
                <div style="display:flex; align-items:center; gap:8px; flex-wrap:wrap;">
                    <span id="peerFoundLabel" style="font-size:12.5px; color:#475569; font-weight:600;">🔗 Eş Telsiz: Taranmadı</span>
                    <button type="button" class="btn-primary" id="btnScanPeer" style="width:auto; padding:6px 12px; font-size:12px;" onclick="scanForPeerRadio(true)">🔍 Ağda Telsiz Ara &amp; Bağla</button>
                    <button type="button" class="btn-danger" id="btnCloseDual" style="display:none; width:auto; padding:6px 12px; font-size:12px;" onclick="toggleDualView(false)">✖ İkinci Telsizi Kapat</button>
                </div>
            </div>

            <div class="dual-rack" id="dualRack">
            <!-- 1. TELSİZ (YEREL) ASELSAN 4822/4826 NOMİNAL CANLI ÖN PANEL -->
            <div class="radio-chassis">
                <div class="radio-top-bar">
                    <div class="aselsan-brand">
                        <span class="aselsan-logo">aselsan</span>
                        <span class="aselsan-model" id="panelModelTitle">MV 4822 VHF / FM</span>
                    </div>
                    <div class="live-pill">
                        <span class="live-dot" id="syncDot"></span>
                        <span id="syncText">CANLI SENKRON</span>
                    </div>
                </div>

                <div class="panel-main-grid">
                    <div class="lcd-bezel">
                        <div class="lcd-glass" id="lcdGlass">
                            <div class="lcd-symbols">
                                <span class="sym-item on" title="ASELSAN Amblemi">[A]</span>
                                <span class="sym-item" id="symShift" title="Röle / Simpleks Durumu">⇄ SİMP</span>
                                <span class="sym-item" id="symSql" title="Susturma İptal (Monitör)">SQL AÇIK</span>
                                <span class="sym-item" id="symTone" title="CTCSS Ton Kodlu Susturma">♪ CTCSS</span>
                                <span class="sym-item on" id="symPwr" title="RF Çıkış Gücü">25W HI</span>
                            </div>
                            <div class="lcd-readout-row">
                                <span class="lcd-mode-tag" id="lcdModeTag">VFO</span>
                                <div class="lcd-digits" id="lcdMainText">433.500</div>
                            </div>
                            <div class="lcd-sub-info">
                                <span id="lcdSubLeft">RX: 433.500 | TX: 433.500 MHz</span>
                                <span id="lcdSubRight">SQL: 3 | TON: KAPALI</span>
                            </div>
                        </div>
                    </div>

                    <div class="right-cluster">
                        <div class="rocker-col">
                            <button type="button" class="btn-rocker" onclick="sendPanelAction('step_up')" title="Kanal / Frekans Yukarı (▲)">▲</button>
                            <button type="button" class="btn-rocker" onclick="sendPanelAction('step_down')" title="Kanal / Frekans Aşağı (▼)">▼</button>
                        </div>
                        <div class="led-col">
                            <div class="led-row">
                                <div class="led-rect" id="ledTx"></div>
                                <span>GÖNDERME</span>
                            </div>
                            <div class="led-row">
                                <div class="led-rect" id="ledBusy"></div>
                                <span>MEŞGUL</span>
                            </div>
                            <div class="led-row">
                                <div class="led-rect" id="ledTone"></div>
                                <span>TON / ÇAĞRI</span>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Ön Panel 6 Fonksiyon Tuşu (ŞEKİL 1-3 Nominal Görünüm) -->
                <div class="panel-func-row">
                    <button type="button" class="btn-panel" id="btnRole" onclick="sendPanelAction('toggle_shift')">
                        <span class="p-sym">[ ⇄ ]</span>
                        <span class="p-lbl">RÖLE / SİMP</span>
                    </button>
                    <button type="button" class="btn-panel" id="btnSust" onclick="sendPanelAction('toggle_sql')">
                        <span class="p-sym">[ SQL ]</span>
                        <span class="p-lbl">SUST. İPTAL</span>
                    </button>
                    <button type="button" class="btn-panel" id="btnTon" onclick="sendPanelAction('toggle_tone')">
                        <span class="p-sym">[ ♪ ]</span>
                        <span class="p-lbl">TON (CTCSS)</span>
                    </button>
                    <button type="button" class="btn-panel" id="btnGuc" onclick="sendPanelAction('toggle_pwr')">
                        <span class="p-sym">[ HI/LO ]</span>
                        <span class="p-lbl">YÜKSEK/DÜŞÜK</span>
                    </button>
                    <button type="button" class="btn-panel" id="btnVfoMr" onclick="sendPanelAction('toggle_mode')">
                        <span class="p-sym">[ VFO/MR ]</span>
                        <span class="p-lbl">VFO / HAFIZA</span>
                    </button>
                    <button type="button" class="btn-panel" onclick="sendPanelAction('beep_test')">
                        <span class="p-sym">[ SES ]</span>
                        <span class="p-lbl">SES / KONTROL</span>
                    </button>
                </div>

                <!-- Tek Satır Doğrudan Frekans ve Adım Çubuğu -->
                <div class="quick-tune-bar">
                    <div class="quick-tune-left">
                        <span style="font-size:11.5px; font-weight:700; color:#94a3b8;">Frekans:</span>
                        <input type="text" id="directFreqInput" placeholder="433.500" onkeydown="if(event.key==='Enter'){applyDirectFreq();}">
                        <button type="button" class="btn-go" onclick="applyDirectFreq()">Git</button>
                    </div>
                    <div class="quick-tune-steps">
                        <button type="button" class="btn-step" onclick="stepFreq(-1000)">-1 MHz</button>
                        <button type="button" class="btn-step" onclick="stepFreq(-100)">-100 kHz</button>
                        <button type="button" class="btn-step" onclick="stepFreq(-25)">-25 kHz</button>
                        <button type="button" class="btn-step" onclick="stepFreq(25)">+25 kHz</button>
                        <button type="button" class="btn-step" onclick="stepFreq(100)">+100 kHz</button>
                        <button type="button" class="btn-step" onclick="stepFreq(1000)">+1 MHz</button>
                    </div>
                </div>

                <!-- Telsiz Üzeri Kompakt Seçim Şeridi (Kanal, SQL Kademesi, Ton Hz, Shift, TOT) -->
                <div class="chassis-param-strip">
                    <div class="cp-item">
                        <label>Hafıza Kanalı:</label>
                        <select id="mem_ch" onchange="pushLiveParam('mem_ch', this.value)"></select>
                    </div>
                    <div class="cp-item">
                        <label>Susturma (SQL):</label>
                        <select id="sql_mode" onchange="pushLiveParam('sql_mode', this.value)">
                            <option value="0">0 - Açık (MON)</option>
                            <option value="1">1 - Çok Hassas</option>
                            <option value="2">2 - Hassas</option>
                            <option value="3">3 - Normal</option>
                            <option value="4">4 - Sıkı</option>
                            <option value="5">5 - Çok Sıkı</option>
                        </select>
                    </div>
                    <div class="cp-item">
                        <label>CTCSS Ton (Hz):</label>
                        <select id="vfo_tone_pos" onchange="pushLiveParam('vfo_tone_pos', this.value)"></select>
                    </div>
                    <div class="cp-item">
                        <label>Röle Shift (kHz):</label>
                        <input type="number" id="vfo_shift" step="25" min="0" max="20000" onchange="pushLiveParam('vfo_shift', this.value)">
                    </div>
                    <div class="cp-item">
                        <label>TOT Süresi (sn):</label>
                        <input type="number" id="tot_time" min="15" max="240" step="15" onchange="pushLiveParam('tot_time', this.value)">
                    </div>
                </div>

                <!-- Canlı Web Ses Dinleme (ADC0/SP) & Web Bas-Konuş (PWM0/MIC) Barı -->
                <div class="audio-trx-bar">
                    <button type="button" class="btn-audio-rx" id="btnAudioRx" onclick="toggleWebAudioRx(false)">🔊 Web Ses Dinle (RX)</button>
                    <div class="vu-box" title="SP (ADC0 / GP26) Canlı Ses Giriş Seviyesi">
                        <span class="vu-lbl" id="vuText">SES: %0</span>
                        <div class="vu-track"><div class="vu-fill" id="vuBar"></div></div>
                    </div>
                    <button type="button" class="btn-web-ptt" id="btnWebPtt"
                        onmousedown="startWebPtt(false)" onmouseup="stopWebPtt(false)" onmouseleave="stopWebPtt(false)"
                        ontouchstart="event.preventDefault(); startWebPtt(false);" ontouchend="event.preventDefault(); stopWebPtt(false);">
                        🎙️ BAS-KONUŞ (PTT)
                    </button>
                </div>
                <div id="liveActionStatus" class="status-msg" style="margin-top:4px; min-height:0;"></div>
            </div>

            <!-- 2. TELSİZ (EŞ TELSİZ / UZAK IP) ASELSAN 4822/4826 NOMİNAL CANLI ÖN PANEL -->
            <div class="radio-chassis" id="peerChassisWrap" style="display:none; border-color:#1e3a8a;">
                <div class="radio-top-bar">
                    <div class="aselsan-brand">
                        <span class="aselsan-logo">aselsan</span>
                        <span class="aselsan-model" id="peerModelTitle">İKİNCİ TELSİZ (EŞ CİHAZ)</span>
                    </div>
                    <div class="live-pill">
                        <span class="live-dot" id="peerSyncDot"></span>
                        <span id="peerSyncText">BAĞLANIYOR...</span>
                    </div>
                </div>

                <div class="panel-main-grid">
                    <div class="lcd-bezel">
                        <div class="lcd-glass" id="peerLcdGlass">
                            <div class="lcd-symbols">
                                <span class="sym-item on" title="ASELSAN Amblemi">[A]</span>
                                <span class="sym-item" id="peerSymShift">⇄ SİMP</span>
                                <span class="sym-item" id="peerSymSql">SQL AÇIK</span>
                                <span class="sym-item" id="peerSymTone">♪ CTCSS</span>
                                <span class="sym-item on" id="peerSymPwr">25W HI</span>
                            </div>
                            <div class="lcd-readout-row">
                                <span class="lcd-mode-tag" id="peerLcdModeTag">VFO</span>
                                <div class="lcd-digits" id="peerLcdMainText">145.500</div>
                            </div>
                            <div class="lcd-sub-info">
                                <span id="peerLcdSubLeft">RX: ---.--- | TX: ---.--- MHz</span>
                                <span id="peerLcdSubRight">SQL: - | TON: -</span>
                            </div>
                        </div>
                    </div>

                    <div class="right-cluster">
                        <div class="rocker-col">
                            <button type="button" class="btn-rocker" onclick="sendPeerPanelAction('step_up')" title="Kanal / Frekans Yukarı (▲)">▲</button>
                            <button type="button" class="btn-rocker" onclick="sendPeerPanelAction('step_down')" title="Kanal / Frekans Aşağı (▼)">▼</button>
                        </div>
                        <div class="led-col">
                            <div class="led-row">
                                <div class="led-rect" id="peerLedTx"></div>
                                <span>GÖNDERME</span>
                            </div>
                            <div class="led-row">
                                <div class="led-rect" id="peerLedBusy"></div>
                                <span>MEŞGUL</span>
                            </div>
                            <div class="led-row">
                                <div class="led-rect" id="peerLedTone"></div>
                                <span>TON / ÇAĞRI</span>
                            </div>
                        </div>
                    </div>
                </div>

                <div class="panel-func-row">
                    <button type="button" class="btn-panel" id="peerBtnRole" onclick="sendPeerPanelAction('toggle_shift')">
                        <span class="p-sym">[ ⇄ ]</span>
                        <span class="p-lbl">RÖLE / SİMP</span>
                    </button>
                    <button type="button" class="btn-panel" id="peerBtnSust" onclick="sendPeerPanelAction('toggle_sql')">
                        <span class="p-sym">[ SQL ]</span>
                        <span class="p-lbl">SUST. İPTAL</span>
                    </button>
                    <button type="button" class="btn-panel" id="peerBtnTon" onclick="sendPeerPanelAction('toggle_tone')">
                        <span class="p-sym">[ ♪ ]</span>
                        <span class="p-lbl">TON (CTCSS)</span>
                    </button>
                    <button type="button" class="btn-panel" id="peerBtnGuc" onclick="sendPeerPanelAction('toggle_pwr')">
                        <span class="p-sym">[ HI/LO ]</span>
                        <span class="p-lbl">YÜKSEK/DÜŞÜK</span>
                    </button>
                    <button type="button" class="btn-panel" id="peerBtnVfoMr" onclick="sendPeerPanelAction('toggle_mode')">
                        <span class="p-sym">[ VFO/MR ]</span>
                        <span class="p-lbl">VFO / HAFIZA</span>
                    </button>
                    <button type="button" class="btn-panel" onclick="sendPeerPanelAction('beep_test')">
                        <span class="p-sym">[ SES ]</span>
                        <span class="p-lbl">SES / KONTROL</span>
                    </button>
                </div>

                <div class="quick-tune-bar">
                    <div class="quick-tune-left">
                        <span style="font-size:11.5px; font-weight:700; color:#94a3b8;">Frekans:</span>
                        <input type="text" id="peerDirectFreqInput" placeholder="145.500" onkeydown="if(event.key==='Enter'){applyPeerDirectFreq();}">
                        <button type="button" class="btn-go" onclick="applyPeerDirectFreq()">Git</button>
                    </div>
                    <div class="quick-tune-steps">
                        <button type="button" class="btn-step" onclick="stepPeerFreq(-1000)">-1 MHz</button>
                        <button type="button" class="btn-step" onclick="stepPeerFreq(-100)">-100 kHz</button>
                        <button type="button" class="btn-step" onclick="stepPeerFreq(-25)">-25 kHz</button>
                        <button type="button" class="btn-step" onclick="stepPeerFreq(25)">+25 kHz</button>
                        <button type="button" class="btn-step" onclick="stepPeerFreq(100)">+100 kHz</button>
                        <button type="button" class="btn-step" onclick="stepPeerFreq(1000)">+1 MHz</button>
                    </div>
                </div>

                <div class="chassis-param-strip">
                    <div class="cp-item">
                        <label>Hafıza Kanalı:</label>
                        <select id="peer_mem_ch" onchange="pushPeerLiveParam('mem_ch', this.value)"></select>
                    </div>
                    <div class="cp-item">
                        <label>Susturma (SQL):</label>
                        <select id="peer_sql_mode" onchange="pushPeerLiveParam('sql_mode', this.value)">
                            <option value="0">0 - Açık (MON)</option>
                            <option value="1">1 - Çok Hassas</option>
                            <option value="2">2 - Hassas</option>
                            <option value="3">3 - Normal</option>
                            <option value="4">4 - Sıkı</option>
                            <option value="5">5 - Çok Sıkı</option>
                        </select>
                    </div>
                    <div class="cp-item">
                        <label>CTCSS Ton (Hz):</label>
                        <select id="peer_vfo_tone_pos" onchange="pushPeerLiveParam('vfo_tone_pos', this.value)"></select>
                    </div>
                    <div class="cp-item">
                        <label>Röle Shift (kHz):</label>
                        <input type="number" id="peer_vfo_shift" step="25" min="0" max="20000" onchange="pushPeerLiveParam('vfo_shift', this.value)">
                    </div>
                    <div class="cp-item">
                        <label>TOT Süresi (sn):</label>
                        <input type="number" id="peer_tot_time" min="15" max="240" step="15" onchange="pushPeerLiveParam('tot_time', this.value)">
                    </div>
                </div>

                <div class="audio-trx-bar">
                    <button type="button" class="btn-audio-rx" id="peerBtnAudioRx" onclick="toggleWebAudioRx(true)">🔊 Web Ses Dinle (RX)</button>
                    <div class="vu-box" title="Eş Telsiz SP (ADC0 / GP26) Canlı Ses Giriş Seviyesi">
                        <span class="vu-lbl" id="peerVuText">SES: %0</span>
                        <div class="vu-track"><div class="vu-fill" id="peerVuBar"></div></div>
                    </div>
                    <button type="button" class="btn-web-ptt" id="peerBtnWebPtt"
                        onmousedown="startWebPtt(true)" onmouseup="stopWebPtt(true)" onmouseleave="stopWebPtt(true)"
                        ontouchstart="event.preventDefault(); startWebPtt(true);" ontouchend="event.preventDefault(); stopWebPtt(true);">
                        🎙️ BAS-KONUŞ (PTT)
                    </button>
                </div>
                <div id="peerActionStatus" class="status-msg" style="margin-top:4px; min-height:0;"></div>
            </div>
            </div>

            <!-- KİMLİK, CİHAZ BANDI & ÇOKLU WIFI AĞ MODU AYARLARI -->
            <h3>Kimlik, Cihaz Bandı &amp; WiFi Ağ Modu Ayarları</h3>
            <div class="section-box">
                <div class="grid-4">
                    <div>
                        <label>Çağrı İşareti (Callsign):</label>
                        <input type="text" id="callsign" maxlength="6" style="text-transform:uppercase; font-family:monospace; font-weight:bold;">
                    </div>
                    <div>
                        <label>LCD Açılış Mesajı (Maks 6 krk):</label>
                        <input type="text" id="startup" maxlength="6" style="text-transform:uppercase; font-family:monospace;">
                    </div>
                    <div>
                        <label>Hotspot / Eş Şifresi (En az 8 krk):</label>
                        <input type="text" id="wifipass" minlength="8" maxlength="32">
                    </div>
                    <div>
                        <label>Cihaz Bandı (TAMSAT Kart Tipi):</label>
                        <select id="radio_type">
                            <option value="0">VHF (134 - 174 MHz - 4822)</option>
                            <option value="1">UHF (428.150 - 450.000 MHz - 4826)</option>
                        </select>
                    </div>
                </div>

                <div style="margin-top:6px;">
                    <label>WiFi Çalışma Modu (Hotspot / Eş Telsiz / Ev Ağı):</label>
                    <select id="wifi_mode" onchange="onWifiModeChange()">
                        <option value="0">Mod 0: Ana İstasyon / Kendi WiFi Hotspot'unu Aç (AP - 192.168.4.1)</option>
                        <option value="1">Mod 1: Arazide Eş Telsize Bağlan (Slave - Ana Telsizin Hotspot'una Otomatik Bağlanır)</option>
                        <option value="2">Mod 2: Ev / İstasyon WiFi Modemine Bağlan (STA - PC / Ev Ağı)</option>
                    </select>
                </div>

                <div id="staSettingsBox" class="grid-3" style="display:none; background:#eef4fb; padding:10px 12px; border-radius:6px; margin-top:4px; margin-bottom:8px; border:1px solid #cbd5e1;">
                    <div>
                        <label id="lblStaSsid">Bağlanılacak WiFi Adı (SSID):</label>
                        <input type="text" id="sta_ssid" maxlength="32" placeholder="Boşsa: ASELSAN_ÇAĞRI">
                    </div>
                    <div>
                        <label id="lblStaPass">Bağlanılacak WiFi Şifresi:</label>
                        <input type="text" id="sta_pass" maxlength="32" placeholder="Boşsa: Hotspot şifresi">
                    </div>
                    <div>
                        <label id="lblStaIp">Sabit IP (Boşsa Otomatik):</label>
                        <input type="text" id="sta_ip" maxlength="15" placeholder="Boş bırakabilirsiniz" style="font-family:monospace;">
                    </div>
                </div>
                <div style="font-size:12px; color:#475569; margin-bottom:10px;">
                    💡 <b>Not:</b> Mod 1 veya Mod 2 seçildiğinde hedef WiFi ağı 12 saniye içinde bulunamazsa telsiz <b>otomatik olarak Mod 0 (Hotspot AP - 192.168.4.1)</b> moduna döner; bağlantıyı asla kaybetmezsiniz. WiFi modu değişikliğini kaydettikten sonra <b>5. Sistem &amp; Güncelleme</b> sekmesinden cihazı yeniden başlatabilirsiniz.
                </div>

                <button type="button" class="btn-success" onclick="saveIdentitySettings()">💾 Kimlik ve WiFi Ayarlarını Kaydet</button>
                <div id="deviceSaveStatus" class="status-msg"></div>
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
                <span style="font-size:13px; color:#555;">Frekansları <b>KHz</b> cinsinden giriniz (Örn: <code>433500</code> veya <code>145575</code>).</span>
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
             4. MENU: KONSOL & AÇIK REST API (PC / ÇİFT TELSİZ KONTROL)
             ================================================================ -->
        <div id="Konsol" class="tabcontent">
            <h3>Seri Port Konsolu ve Canlı Loglar</h3>
            <textarea id="logArea" style="width:100%; height:240px; background:#0d1117; color:#3fb950; font-family:monospace; padding:10px; border-radius:6px; border:1px solid #30363d;" readonly></textarea>
            <form onsubmit="sendCommand(event)" style="display:flex; gap:8px; margin-top:8px; margin-bottom:20px;">
                <input type="text" id="cmdInput" placeholder="Seri komut yazın (örn: K, H, C V, C U, X) ve Gönder'e basın" autocomplete="off" required style="margin:0;">
                <button type="submit" class="btn-success" style="width:160px;">Gönder</button>
                <button type="button" class="btn-danger" style="width:130px;" onclick="clearLogs()">Temizle</button>
            </form>

            <h3>Açık REST API &amp; PC / İkinci Telsiz Bağlantı Özeti (CORS Aktif)</h3>
            <div class="section-box" style="font-size:13px; line-height:1.55;">
                <p style="margin-top:0;">
                    Tüm uç noktalar <code>Access-Control-Allow-Origin: *</code> (CORS) başlığı ile dış programlara (Python, C#, Masaüstü PC yazılımı veya diğer telsizin Web arayüzü) tamamen açıktır:
                </p>
                <ul style="margin:0; padding-left:18px; font-family:monospace; font-size:12.5px;">
                    <li><b>GET  /api/live</b> &rarr; Anlık frekans, kanal, VFO/MR, shift, CTCSS, SQL, TX/RX, güç, çağrı işareti ve IP durumunu JSON döner.</li>
                    <li><b>POST /api/live</b> &rarr; <code>action=step_up|step_down|toggle_shift|toggle_sql|toggle_tone|toggle_pwr|toggle_mode|beep_test</code> veya <code>vfo_freq=145500</code>, <code>mem_ch=1..50</code>, <code>sql_mode=0..5</code>, <code>vfo_tone_pos=0..38</code>, <code>vfo_shift=600</code>, <code>tot_time=60</code></li>
                    <li><b>GET  /getConfig</b> &amp; <b>POST /saveDevice</b> &rarr; Cihaz kimlik, bant ve WiFi ağ modu ayarları.</li>
                    <li><b>GET  /getChannels</b> &amp; <b>POST /saveChannels</b> &rarr; 50 hafıza kanalını JSON olarak okuma/yazma.</li>
                    <li><b>POST /sendAprsNow</b> &rarr; Anında APRS paketi gönderme.</li>
                </ul>
            </div>
        </div>

        <!-- ================================================================
             5. MENU: SISTEM & GUNCELLEME (OTA, REBOOT, FABRIKA AYARLARI)
             ================================================================ -->
        <div id="OTA" class="tabcontent">
            <h3>OTA Kablosuz Yazılım Güncelleme</h3>
            <div class="section-box">
                <p style="margin-top:0; font-size:14px; line-height:1.5;">
                    PlatformIO derlemesinden çıkan <b>.pio/build/rpipico2w/firmware.bin</b> dosyasını seçerek kablosuz yazılım yükleyebilirsiniz.<br>
                    <small style="color:#64748b;">Not: İlk kez OTA dosya sistemi (LittleFS 1.5MB) aktif edildiğinde bir defaya mahsus USB üzerinden yükleme yapılmış olmalıdır.</small>
                </p>
                <form onsubmit="uploadOtaFirmware(event)">
                    <input type="file" id="otaFileInput" name="update" accept=".bin" required>
                    <div class="progress-wrap" id="otaProgressWrap">
                        <div class="progress-bar" id="otaProgressBar">0%</div>
                    </div>
                    <button type="submit" class="btn-success" id="btnOtaSubmit">🚀 Yazılımı Yükle ve Güncelle (.bin)</button>
                </form>
                <div id="otaStatusMsg" class="status-msg"></div>
            </div>

            <h3>Sistem ve Sıfırlama İşlemleri</h3>
            <div class="section-box" style="border-color: #f5c2c7; background: #fff5f5;">
                <p style="margin-top:0; font-size:13px; color:#555;">
                    <b>Yeniden Başlat:</b> Telsizi ayarları silmeden yeniden başlatır (Çağrı işareti, WiFi şifresi veya WiFi ağ modu değiştirdiyseniz yeni ayarların aktif olması için kullanın).<br>
                    <b>Fabrika Ayarlarına Sıfırla:</b> Tüm ayarları ve kanalları siler, WiFi ağını tekrar <b>DLS_ASELSAN_KIT</b> (şifre: <b>deeplabstudio</b>) ilk kurulum moduna döndürür.
                </p>
                <div class="grid-2">
                    <button type="button" class="btn-warning" onclick="rebootDevice()">🔄 Cihazı Yeniden Başlat (Reboot)</button>
                    <button type="button" class="btn-danger" onclick="factoryResetDevice()">⚠️ Fabrika Ayarlarına Sıfırla (İlk Kurulum Modu)</button>
                </div>
            </div>
        </div>

        <div class="footer">
            TAMSAT &amp; <a href="https://deeplabstudio.com/" target="_blank" rel="noopener noreferrer" style="color:#0056b3; text-decoration:none; font-weight:600;">Deeplab Studio</a> (TA2DMX) &copy; 2026
        </div>
    </div>

    <script>
    const CTCSS_TONES = [
        67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5,
        94.8, 97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3,
        131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2, 165.5, 167.9,
        171.3, 173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 250.3
    ];

    let liveState = {
        freq: 433500,
        radio_type: 1,
        op_mode: 0,
        mem_ch: 1,
        shift_dir: 0,
        shift: 7600,
        tone_en: 0,
        tone_pos: 8,
        sql_mode: 3,
        rf_power: 0,
        trx_mode: 0,
        busy: 0
    };

    let peerLiveState = {
        freq: 145500,
        radio_type: 0,
        op_mode: 0,
        mem_ch: 1,
        shift_dir: 0,
        shift: 600,
        tone_en: 0,
        tone_pos: 8,
        sql_mode: 3,
        rf_power: 0,
        trx_mode: 0,
        busy: 0
    };
    let dualActive = false;

    function buildToneOptions(selectedIndex) {
        let html = '';
        for (let i = 0; i < CTCSS_TONES.length; i++) {
            let sel = (i === Number(selectedIndex)) ? ' selected' : '';
            html += `<option value="${i}"${sel}>${CTCSS_TONES[i].toFixed(1)} Hz</option>`;
        }
        return html;
    }

    function buildMemChOptions(selectedCh) {
        let html = '';
        for (let c = 1; c <= 50; c++) {
            let sel = (c === Number(selectedCh)) ? ' selected' : '';
            html += `<option value="${c}"${sel}>Kanal ${String(c).padStart(2, '0')}</option>`;
        }
        return html;
    }

    document.addEventListener("DOMContentLoaded", function() {
        document.getElementById('vfo_tone_pos').innerHTML = buildToneOptions(8);
        document.getElementById('mem_ch').innerHTML = buildMemChOptions(1);
        document.getElementById('peer_vfo_tone_pos').innerHTML = buildToneOptions(8);
        document.getElementById('peer_mem_ch').innerHTML = buildMemChOptions(1);
        loadConfig();
        pollLiveState();
        setInterval(pollLiveState, 500);
        setInterval(pollPeerLiveState, 650);
    });

    function formatFreqMhz(khz) {
        let mhz = (Number(khz) / 1000.0).toFixed(3);
        return mhz;
    }

    function updateFrontPanelUI(d) {
        liveState = d;
        const isUhf = (Number(d.radio_type) === 1);
        document.getElementById('navModel').innerText = isUhf ? 'ASELSAN 4826' : 'ASELSAN 4822';
        let csTag = d.callsign ? (' [' + d.callsign + ']') : '';
        document.getElementById('panelModelTitle').innerText = (isUhf
            ? 'MV 4826 UHF / FM (428.150 - 450.000 MHz)'
            : 'MV 4822 VHF / FM (134.000 - 174.000 MHz)') + csTag;
        if (d.ip) {
            document.getElementById('localIpBadge').innerText = d.ip;
        }

        const isMr = (Number(d.op_mode) === 1);
        document.getElementById('lcdModeTag').innerText = isMr
            ? ('CH ' + String(d.mem_ch).padStart(2, '0'))
            : 'VFO';

        let mainStr = (d.lcd && d.lcd.trim().length > 0) ? d.lcd.trim() : formatFreqMhz(d.freq);
        document.getElementById('lcdMainText').innerText = mainStr;

        let rxKhz = Number(d.freq);
        let txKhz = rxKhz + (Number(d.shift_dir) * Number(d.shift));
        document.getElementById('lcdSubLeft').innerText =
            `RX: ${formatFreqMhz(rxKhz)} | TX: ${formatFreqMhz(txKhz)} MHz`;

        let toneHz = (CTCSS_TONES[d.tone_pos] || 88.5).toFixed(1);
        let sqlTxt = (Number(d.sql_mode) === 0) ? 'AÇIK (MON)' : ('Kademe ' + d.sql_mode);
        document.getElementById('lcdSubRight').innerText =
            `SQL: ${sqlTxt} | TON: ${Number(d.tone_en) ? (toneHz + ' Hz ♪') : 'KAPALI'}`;

        let symShift = document.getElementById('symShift');
        if (Number(d.shift_dir) === 1) {
            symShift.innerText = '+ RÖLE';
            symShift.classList.add('on');
        } else if (Number(d.shift_dir) === -1) {
            symShift.innerText = '- RÖLE';
            symShift.classList.add('on');
        } else {
            symShift.innerText = '⇄ SİMP';
            symShift.classList.add('on');
        }

        document.getElementById('symSql').classList.toggle('on', Number(d.sql_mode) === 0);
        document.getElementById('symTone').classList.toggle('on', Number(d.tone_en) === 1);

        let symPwr = document.getElementById('symPwr');
        symPwr.innerText = (Number(d.rf_power) === 0) ? '25W HI' : '5W LO';

        const isTx = (Number(d.trx_mode) === 1);
        const isBusy = (Number(d.busy) === 1);
        document.getElementById('lcdGlass').classList.toggle('tx-active', isTx);
        document.getElementById('ledTx').classList.toggle('red-on', isTx);
        document.getElementById('ledBusy').classList.toggle('green-on', !isTx && isBusy);
        document.getElementById('ledTone').classList.toggle('amber-on', Number(d.tone_en) === 1);

        document.getElementById('btnRole').classList.toggle('active-func', Number(d.shift_dir) !== 0);
        document.getElementById('btnSust').classList.toggle('active-func', Number(d.sql_mode) === 0);
        document.getElementById('btnTon').classList.toggle('active-func', Number(d.tone_en) === 1);
        document.getElementById('btnGuc').classList.toggle('active-func', Number(d.rf_power) === 0);
        document.getElementById('btnVfoMr').classList.toggle('active-func', isMr);

        const activeEl = document.activeElement;
        if (activeEl !== document.getElementById('directFreqInput')) {
            document.getElementById('directFreqInput').value = formatFreqMhz(d.freq);
        }
        if (activeEl !== document.getElementById('mem_ch')) {
            document.getElementById('mem_ch').value = d.mem_ch;
        }
        if (activeEl !== document.getElementById('sql_mode')) {
            document.getElementById('sql_mode').value = d.sql_mode;
        }
        if (activeEl !== document.getElementById('vfo_shift')) {
            document.getElementById('vfo_shift').value = d.shift;
        }
        if (activeEl !== document.getElementById('vfo_tone_pos')) {
            document.getElementById('vfo_tone_pos').value = d.tone_pos;
        }
        if (activeEl !== document.getElementById('tot_time') && d.tot_time !== undefined) {
            document.getElementById('tot_time').value = d.tot_time;
        }
        let lvl = Number(d.audio_lvl || 0);
        document.getElementById('vuBar').style.width = lvl + '%';
        document.getElementById('vuText').innerText = 'SES: %' + lvl;
    }

    function updatePeerPanelUI(d) {
        peerLiveState = d;
        const isUhf = (Number(d.radio_type) === 1);
        let csTag = d.callsign ? (' [' + d.callsign + ']') : '';
        document.getElementById('peerModelTitle').innerText = (isUhf
            ? 'MV 4826 UHF / FM (428.150 - 450.000 MHz)'
            : 'MV 4822 VHF / FM (134.000 - 174.000 MHz)') + csTag;

        const isMr = (Number(d.op_mode) === 1);
        document.getElementById('peerLcdModeTag').innerText = isMr
            ? ('CH ' + String(d.mem_ch).padStart(2, '0'))
            : 'VFO';

        let mainStr = (d.lcd && d.lcd.trim().length > 0) ? d.lcd.trim() : formatFreqMhz(d.freq);
        document.getElementById('peerLcdMainText').innerText = mainStr;

        let rxKhz = Number(d.freq);
        let txKhz = rxKhz + (Number(d.shift_dir) * Number(d.shift));
        document.getElementById('peerLcdSubLeft').innerText =
            `RX: ${formatFreqMhz(rxKhz)} | TX: ${formatFreqMhz(txKhz)} MHz`;

        let toneHz = (CTCSS_TONES[d.tone_pos] || 88.5).toFixed(1);
        let sqlTxt = (Number(d.sql_mode) === 0) ? 'AÇIK (MON)' : ('Kademe ' + d.sql_mode);
        document.getElementById('peerLcdSubRight').innerText =
            `SQL: ${sqlTxt} | TON: ${Number(d.tone_en) ? (toneHz + ' Hz ♪') : 'KAPALI'}`;

        let symShift = document.getElementById('peerSymShift');
        if (Number(d.shift_dir) === 1) {
            symShift.innerText = '+ RÖLE';
            symShift.classList.add('on');
        } else if (Number(d.shift_dir) === -1) {
            symShift.innerText = '- RÖLE';
            symShift.classList.add('on');
        } else {
            symShift.innerText = '⇄ SİMP';
            symShift.classList.add('on');
        }

        document.getElementById('peerSymSql').classList.toggle('on', Number(d.sql_mode) === 0);
        document.getElementById('peerSymTone').classList.toggle('on', Number(d.tone_en) === 1);
        document.getElementById('peerSymPwr').innerText = (Number(d.rf_power) === 0) ? '25W HI' : '5W LO';

        const isTx = (Number(d.trx_mode) === 1);
        const isBusy = (Number(d.busy) === 1);
        document.getElementById('peerLcdGlass').classList.toggle('tx-active', isTx);
        document.getElementById('peerLedTx').classList.toggle('red-on', isTx);
        document.getElementById('peerLedBusy').classList.toggle('green-on', !isTx && isBusy);
        document.getElementById('peerLedTone').classList.toggle('amber-on', Number(d.tone_en) === 1);

        document.getElementById('peerBtnRole').classList.toggle('active-func', Number(d.shift_dir) !== 0);
        document.getElementById('peerBtnSust').classList.toggle('active-func', Number(d.sql_mode) === 0);
        document.getElementById('peerBtnTon').classList.toggle('active-func', Number(d.tone_en) === 1);
        document.getElementById('peerBtnGuc').classList.toggle('active-func', Number(d.rf_power) === 0);
        document.getElementById('peerBtnVfoMr').classList.toggle('active-func', isMr);

        const activeEl = document.activeElement;
        if (activeEl !== document.getElementById('peerDirectFreqInput')) {
            document.getElementById('peerDirectFreqInput').value = formatFreqMhz(d.freq);
        }
        if (activeEl !== document.getElementById('peer_mem_ch')) {
            document.getElementById('peer_mem_ch').value = d.mem_ch;
        }
        if (activeEl !== document.getElementById('peer_sql_mode')) {
            document.getElementById('peer_sql_mode').value = d.sql_mode;
        }
        if (activeEl !== document.getElementById('peer_vfo_shift')) {
            document.getElementById('peer_vfo_shift').value = d.shift;
        }
        if (activeEl !== document.getElementById('peer_vfo_tone_pos')) {
            document.getElementById('peer_vfo_tone_pos').value = d.tone_pos;
        }
        if (activeEl !== document.getElementById('peer_tot_time') && d.tot_time !== undefined) {
            document.getElementById('peer_tot_time').value = d.tot_time;
        }
        let plvl = Number(d.audio_lvl || 0);
        document.getElementById('peerVuBar').style.width = plvl + '%';
        document.getElementById('peerVuText').innerText = 'SES: %' + plvl;
    }

    // ========================================================================
    // WEB AUDIO RX (CANLI TELSİZ SESİ DİNLEME - 8 kHz 8-Bit PCM) & WEB PTT TX
    // ========================================================================
    let audioCtx = null;
    let localRxTimer = null;
    let peerRxTimer = null;
    let localNextPlayTime = 0;
    let peerNextPlayTime = 0;

    function ensureAudioCtx() {
        if (!audioCtx) {
            const AC = window.AudioContext || window.webkitAudioContext;
            audioCtx = new AC();
        }
        if (audioCtx.state === 'suspended') {
            audioCtx.resume();
        }
    }

    function playPcmChunk(u8, isPeer) {
        if (!u8 || u8.length < 16 || !audioCtx) return;
        const buf = audioCtx.createBuffer(1, u8.length, 8000);
        const ch = buf.getChannelData(0);
        let peak = 0;
        for (let i = 0; i < u8.length; i++) {
            let v = (u8[i] - 128) / 128.0;
            ch[i] = v;
            let av = Math.abs(v);
            if (av > peak) peak = av;
        }
        let pct = Math.min(100, Math.round(peak * 140));
        if (isPeer) {
            document.getElementById('peerVuBar').style.width = pct + '%';
            document.getElementById('peerVuText').innerText = 'SES: %' + pct;
        } else {
            document.getElementById('vuBar').style.width = pct + '%';
            document.getElementById('vuText').innerText = 'SES: %' + pct;
        }

        const src = audioCtx.createBufferSource();
        src.buffer = buf;
        src.connect(audioCtx.destination);
        let now = audioCtx.currentTime;
        if (isPeer) {
            if (peerNextPlayTime < now || peerNextPlayTime > now + 0.6) peerNextPlayTime = now + 0.06;
            src.start(peerNextPlayTime);
            peerNextPlayTime += buf.duration;
        } else {
            if (localNextPlayTime < now || localNextPlayTime > now + 0.6) localNextPlayTime = now + 0.06;
            src.start(localNextPlayTime);
            localNextPlayTime += buf.duration;
        }
    }

    function toggleWebAudioRx(isPeer) {
        ensureAudioCtx();
        if (isPeer) {
            let btn = document.getElementById('peerBtnAudioRx');
            if (peerRxTimer) {
                clearInterval(peerRxTimer);
                peerRxTimer = null;
                btn.classList.remove('listening');
                btn.innerText = '🔊 Web Ses Dinle (RX)';
            } else {
                btn.classList.add('listening');
                btn.innerText = '🔊 Dinleniyor (Kapat)';
                peerNextPlayTime = 0;
                peerRxTimer = setInterval(() => {
                    let base = getPeerBaseUrl();
                    if (!base) return;
                    fetch(base + '/api/audio_rx')
                        .then(r => r.arrayBuffer())
                        .then(ab => { if (ab.byteLength > 0) playPcmChunk(new Uint8Array(ab), true); })
                        .catch(() => {});
                }, 160);
            }
        } else {
            let btn = document.getElementById('btnAudioRx');
            if (localRxTimer) {
                clearInterval(localRxTimer);
                localRxTimer = null;
                btn.classList.remove('listening');
                btn.innerText = '🔊 Web Ses Dinle (RX)';
            } else {
                btn.classList.add('listening');
                btn.innerText = '🔊 Dinleniyor (Kapat)';
                localNextPlayTime = 0;
                localRxTimer = setInterval(() => {
                    fetch('/api/audio_rx')
                        .then(r => r.arrayBuffer())
                        .then(ab => { if (ab.byteLength > 0) playPcmChunk(new Uint8Array(ab), false); })
                        .catch(() => {});
                }, 160);
            }
        }
    }

    let webPttTimer = null;
    let webPttPeerMode = false;
    let micStream = null;
    let micProcessor = null;

    async function startWebPtt(isPeer) {
        if (webPttTimer) return;
        webPttPeerMode = isPeer;
        let btn = document.getElementById(isPeer ? 'peerBtnWebPtt' : 'btnWebPtt');
        btn.classList.add('tx-down');
        btn.innerText = '🔴 GÖNDERİLİYOR (TX)...';

        let base = isPeer ? getPeerBaseUrl() : '';
        fetch(base + '/api/audio_tx', { method: 'POST', body: 'action=start' }).catch(() => {});
        webPttTimer = setInterval(() => {
            fetch(base + '/api/audio_tx', { method: 'POST', body: 'action=keepalive' }).catch(() => {});
        }, 600);

        if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
            try {
                ensureAudioCtx();
                micStream = await navigator.mediaDevices.getUserMedia({ audio: true });
                const src = audioCtx.createMediaStreamSource(micStream);
                micProcessor = audioCtx.createScriptProcessor(2048, 1, 1);
                const ratio = audioCtx.sampleRate / 8000.0;
                micProcessor.onaudioprocess = function(e) {
                    if (!webPttTimer) return;
                    const input = e.inputBuffer.getChannelData(0);
                    const outLen = Math.floor(input.length / ratio);
                    let hex = '';
                    for (let i = 0; i < outLen; i++) {
                        let s = input[Math.floor(i * ratio)];
                        let b = Math.max(0, Math.min(255, Math.round((s + 1.0) * 127.5)));
                        hex += b.toString(16).padStart(2, '0');
                    }
                    fetch(base + '/api/audio_tx', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                        body: 'pcm=' + hex
                    }).catch(() => {});
                };
                src.connect(micProcessor);
                micProcessor.connect(audioCtx.destination);
            } catch (err) {
                // HTTP uzerinde mikrofon izni kapaliysa yalnizca tasiyici PTT aktif kalir
            }
        }
    }

    function stopWebPtt(isPeer) {
        if (!webPttTimer) return;
        clearInterval(webPttTimer);
        webPttTimer = null;
        if (micProcessor) { try { micProcessor.disconnect(); } catch(e){} micProcessor = null; }
        if (micStream) { try { micStream.getTracks().forEach(t => t.stop()); } catch(e){} micStream = null; }

        let btn = document.getElementById(isPeer ? 'peerBtnWebPtt' : 'btnWebPtt');
        btn.classList.remove('tx-down');
        btn.innerText = '🎙️ BAS-KONUŞ (PTT)';

        let base = isPeer ? getPeerBaseUrl() : '';
        fetch(base + '/api/audio_tx', { method: 'POST', body: 'action=stop' }).catch(() => {});
    }

    let activePeerIp = '';
    let scanningPeer = false;

    function getPeerBaseUrl() {
        if (!activePeerIp) return '';
        return 'http://' + activePeerIp;
    }

    function toggleDualView(forceState) {
        dualActive = (forceState !== undefined) ? forceState : !dualActive;
        document.getElementById('peerChassisWrap').style.display = dualActive ? 'block' : 'none';
        document.getElementById('dualRack').classList.toggle('active-dual', dualActive);
        document.getElementById('mainContainer').classList.toggle('dual-expanded', dualActive);
        document.getElementById('btnCloseDual').style.display = dualActive ? 'inline-block' : 'none';
        if (dualActive) {
            pollPeerLiveState();
        }
    }

    async function probeRadioIp(ip, timeoutMs) {
        const ctrl = new AbortController();
        const tid = setTimeout(() => ctrl.abort(), timeoutMs || 450);
        try {
            let res = await fetch('http://' + ip + '/api/live', { signal: ctrl.signal });
            clearTimeout(tid);
            if (!res.ok) return null;
            let d = await res.json();
            if (d && d.freq && d.radio_type !== undefined) return d;
        } catch (e) {
            clearTimeout(tid);
        }
        return null;
    }

    function onPeerConnected(ip, d) {
        activePeerIp = ip;
        try { localStorage.setItem('aselsan_peer_ip', ip); } catch(e) {}
        let band = (Number(d.radio_type) === 1) ? 'UHF 4826' : 'VHF 4822';
        let cs = d.callsign ? (' [' + d.callsign + ']') : '';
        let lbl = document.getElementById('peerFoundLabel');
        lbl.style.color = '#15803d';
        lbl.innerText = `🔗 Eş Telsiz: ${band}${cs} (${ip})`;
        toggleDualView(true);
        updatePeerPanelUI(d);
    }

    async function scanForPeerRadio(fullScan) {
        if (scanningPeer) return;
        scanningPeer = true;
        let btn = document.getElementById('btnScanPeer');
        let lbl = document.getElementById('peerFoundLabel');
        let myIp = document.getElementById('localIpBadge').innerText.trim() || window.location.hostname;

        btn.disabled = true;
        btn.innerText = '⏳ Ağ Taranıyor...';
        lbl.style.color = '#0056b3';
        lbl.innerText = '🔍 Ağdaki telsizler aranıyor...';

        // 1) Öncelikli adayları hızlı kontrol et (Son bilinen IP, 192.168.4.2, 192.168.4.1)
        let savedIp = '';
        try { savedIp = localStorage.getItem('aselsan_peer_ip') || ''; } catch(e) {}
        let fastList = [];
        if (activePeerIp && activePeerIp !== myIp) fastList.push(activePeerIp);
        if (savedIp && savedIp !== myIp && !fastList.includes(savedIp)) fastList.push(savedIp);
        ['192.168.4.2', '192.168.4.1', '192.168.4.3', '192.168.4.4'].forEach(ip => {
            if (ip !== myIp && !fastList.includes(ip)) fastList.push(ip);
        });

        for (let ip of fastList) {
            let d = await probeRadioIp(ip, 450);
            if (d) {
                onPeerConnected(ip, d);
                btn.disabled = false;
                btn.innerText = '🔍 Ağda Telsiz Ara & Bağla';
                scanningPeer = false;
                return;
            }
        }

        // 2) Kullanıcı "Ağda Telsiz Ara" butonuna bastıysa ve bir ev/istasyon ağındaysa tüm alt ağı (/24) paralel tara
        let parts = myIp.split('.');
        if (fullScan && parts.length === 4) {
            let prefix = `${parts[0]}.${parts[1]}.${parts[2]}.`;
            let foundIp = null;
            let foundData = null;
            const batchSize = 25;
            for (let start = 1; start <= 254 && !foundIp; start += batchSize) {
                let promises = [];
                let end = Math.min(254, start + batchSize - 1);
                lbl.innerText = `🔍 Ağ taranıyor: ${prefix}${start} - .${end}...`;
                for (let last = start; last <= end; last++) {
                    let cand = prefix + last;
                    if (cand === myIp || fastList.includes(cand)) continue;
                    promises.push(
                        probeRadioIp(cand, 420).then(res => {
                            if (res && !foundIp) {
                                foundIp = cand;
                                foundData = res;
                            }
                        })
                    );
                }
                await Promise.all(promises);
            }
            if (foundIp && foundData) {
                onPeerConnected(foundIp, foundData);
                // Bulunan eş telsiz IP'sini cihaza da kaydet
                let p = new URLSearchParams();
                p.append('peer_ip', foundIp);
                fetch('/saveDevice', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: p.toString() }).catch(()=>{});
                btn.disabled = false;
                btn.innerText = '🔍 Ağda Telsiz Ara & Bağla';
                scanningPeer = false;
                return;
            }
        }

        lbl.style.color = '#64748b';
        lbl.innerText = fullScan ? '⚠️ Ağda ikinci telsiz bulunamadı' : '🔗 Eş Telsiz: Bağlı Değil';
        btn.disabled = false;
        btn.innerText = '🔍 Ağda Telsiz Ara & Bağla';
        scanningPeer = false;
    }

    function pollLiveState() {
        if (document.getElementById('Ayarlar').style.display !== 'block') return;
        fetch('/api/live')
            .then(res => res.json())
            .then(data => {
                document.getElementById('syncDot').style.background = '#10b981';
                document.getElementById('syncText').innerText = 'CANLI SENKRON';
                updateFrontPanelUI(data);
            })
            .catch(() => {
                document.getElementById('syncDot').style.background = '#ef4444';
                document.getElementById('syncText').innerText = 'BAĞLANTI BEKLENİYOR';
            });
    }

    function pollPeerLiveState() {
        if (!dualActive || document.getElementById('Ayarlar').style.display !== 'block') return;
        let base = getPeerBaseUrl();
        if (!base) return;
        fetch(base + '/api/live')
            .then(res => res.json())
            .then(data => {
                document.getElementById('peerSyncDot').style.background = '#10b981';
                document.getElementById('peerSyncText').innerText = 'EŞ TELSİZ AKTİF';
                updatePeerPanelUI(data);
            })
            .catch(() => {
                document.getElementById('peerSyncDot').style.background = '#ef4444';
                document.getElementById('peerSyncText').innerText = 'EŞ TELSİZ YOK';
            });
    }

    function sendPanelAction(actionName) {
        let params = new URLSearchParams();
        params.append('action', actionName);
        fetch('/api/live', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: params.toString()
        })
        .then(res => res.json())
        .then(data => {
            updateFrontPanelUI(data);
        })
        .catch(e => console.error(e));
    }

    function sendPeerPanelAction(actionName) {
        let base = getPeerBaseUrl();
        if (!base) return;
        let params = new URLSearchParams();
        params.append('action', actionName);
        fetch(base + '/api/live', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: params.toString()
        })
        .then(res => res.json())
        .then(data => {
            updatePeerPanelUI(data);
        })
        .catch(e => console.error(e));
    }

    function pushLiveParam(key, val) {
        let params = new URLSearchParams();
        params.append(key, val);
        fetch('/api/live', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: params.toString()
        })
        .then(res => res.json())
        .then(data => {
            updateFrontPanelUI(data);
            showStatus('liveActionStatus', '✓ Telsize anında uygulandı', false);
        })
        .catch(() => {
            showStatus('liveActionStatus', 'Bağlantı hatası!', true);
        });
    }

    function pushPeerLiveParam(key, val) {
        let base = getPeerBaseUrl();
        if (!base) return;
        let params = new URLSearchParams();
        params.append(key, val);
        fetch(base + '/api/live', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: params.toString()
        })
        .then(res => res.json())
        .then(data => {
            updatePeerPanelUI(data);
            showStatus('peerActionStatus', '✓ Eş telsize uygulandı', false);
        })
        .catch(() => {
            showStatus('peerActionStatus', 'Eş telsiz bağlantı hatası!', true);
        });
    }

    function parseFreqInputToKhz(raw) {
        if (!raw) return 0;
        let s = String(raw).trim().replace(',', '.');
        let num = parseFloat(s);
        if (isNaN(num) || num <= 0) return 0;
        if (num < 1000) {
            return Math.round(num * 1000);
        }
        return Math.round(num);
    }

    function applyDirectFreq() {
        let khz = parseFreqInputToKhz(document.getElementById('directFreqInput').value);
        let minF = (Number(liveState.radio_type) === 1) ? 428150 : 134000;
        let maxF = (Number(liveState.radio_type) === 1) ? 450000 : 174000;
        if (khz < minF || khz > maxF) {
            showStatus('liveActionStatus', `Geçersiz frekans! (${formatFreqMhz(minF)} - ${formatFreqMhz(maxF)} MHz arası girin)`, true);
            return;
        }
        khz = Math.round(khz / 25) * 25;
        pushLiveParam('vfo_freq', khz);
    }

    function stepFreq(deltaKhz) {
        let targetKhz = Number(liveState.freq) + Number(deltaKhz);
        let minF = (Number(liveState.radio_type) === 1) ? 428150 : 134000;
        let maxF = (Number(liveState.radio_type) === 1) ? 450000 : 174000;
        if (targetKhz < minF) targetKhz = minF;
        if (targetKhz > maxF) targetKhz = maxF;
        pushLiveParam('vfo_freq', targetKhz);
    }

    function applyPeerDirectFreq() {
        let khz = parseFreqInputToKhz(document.getElementById('peerDirectFreqInput').value);
        let minF = (Number(peerLiveState.radio_type) === 1) ? 428150 : 134000;
        let maxF = (Number(peerLiveState.radio_type) === 1) ? 450000 : 174000;
        if (khz < minF || khz > maxF) {
            showStatus('peerActionStatus', `Geçersiz frekans! (${formatFreqMhz(minF)} - ${formatFreqMhz(maxF)} MHz arası girin)`, true);
            return;
        }
        khz = Math.round(khz / 25) * 25;
        pushPeerLiveParam('vfo_freq', khz);
    }

    function stepPeerFreq(deltaKhz) {
        let targetKhz = Number(peerLiveState.freq) + Number(deltaKhz);
        let minF = (Number(peerLiveState.radio_type) === 1) ? 428150 : 134000;
        let maxF = (Number(peerLiveState.radio_type) === 1) ? 450000 : 174000;
        if (targetKhz < minF) targetKhz = minF;
        if (targetKhz > maxF) targetKhz = maxF;
        pushPeerLiveParam('vfo_freq', targetKhz);
    }

    function onWifiModeChange() {
        let m = Number(document.getElementById('wifi_mode').value);
        let box = document.getElementById('staSettingsBox');
        if (m === 0) {
            box.style.display = 'none';
        } else {
            box.style.display = 'grid';
            if (m === 1) {
                document.getElementById('lblStaSsid').innerText = 'Ana Telsiz WiFi Adı (Boşsa: ASELSAN_ÇAĞRI):';
                document.getElementById('lblStaPass').innerText = 'Ana Telsiz Şifresi (Boşsa: Kendi Şifreniz):';
                document.getElementById('lblStaIp').innerText = 'Eş Telsiz IP (Boşsa Otomatik 192.168.4.2):';
                document.getElementById('sta_ip').placeholder = '192.168.4.2';
            } else {
                document.getElementById('lblStaSsid').innerText = 'Ev / İstasyon WiFi Adı (SSID):';
                document.getElementById('lblStaPass').innerText = 'Ev / İstasyon WiFi Şifresi:';
                document.getElementById('lblStaIp').innerText = 'Sabit IP (Boşsa Modemden DHCP Otomatik Alır):';
                document.getElementById('sta_ip').placeholder = 'Örn: 192.168.1.50 (İsteğe bağlı)';
            }
        }
    }

    function loadConfig() {
        fetch('/getConfig')
            .then(res => res.json())
            .then(data => {
                document.getElementById('callsign').value = data.callsign || '';
                document.getElementById('aprs_callsign').value = data.callsign || '';
                document.getElementById('navCallsign').innerText = data.callsign || '';
                document.getElementById('startup').value = data.startup || '';
                document.getElementById('wifipass').value = data.wifipass || '';
                document.getElementById('radio_type').value = data.radio_type;

                document.getElementById('wifi_mode').value = (data.wifi_mode !== undefined) ? data.wifi_mode : 0;
                document.getElementById('sta_ssid').value = data.sta_ssid || '';
                document.getElementById('sta_pass').value = data.sta_pass || '';
                document.getElementById('sta_ip').value = data.sta_ip || '';
                if (data.peer_ip && !activePeerIp) {
                    activePeerIp = data.peer_ip;
                }
                onWifiModeChange();

                if (data.ip) document.getElementById('localIpBadge').innerText = data.ip;
                let wm = Number(data.wifi_mode || 0);
                document.getElementById('wifiModeBadge').innerText =
                    wm === 1 ? '(Eş Telsiz / Slave Modu)' : (wm === 2 ? '(Ev WiFi İstemci Modu)' : '(Ana İstasyon / AP Hotspot)');

                document.getElementById('aprs_enable').value = data.aprs_enable;
                document.getElementById('aprs_ssid').value = data.aprs_ssid;
                document.getElementById('aprs_lat').value = data.aprs_lat || '';
                document.getElementById('aprs_lon').value = data.aprs_lon || '';
                document.getElementById('aprsmsg').value = data.aprsmsg || '';
                document.getElementById('aprstime').value = data.aprstime;
                document.getElementById('aprs_freq').value = data.aprs_freq;
                document.getElementById('iss_freq').value = data.iss_freq;

                // Sayfa açıldığında ağda eş telsiz varsa otomatik algıla ve bağla
                if (!dualActive) {
                    scanForPeerRadio(false);
                }
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
        } else if (tabName === 'Ayarlar') {
            pollLiveState();
        }
    }

    function showStatus(id, msg, isErr) {
        let el = document.getElementById(id);
        if (!el) return;
        el.style.color = isErr ? '#dc3545' : '#28a745';
        el.innerText = msg;
        setTimeout(() => { if (el.innerText === msg) el.innerText = ''; }, 3500);
    }

    function saveIdentitySettings() {
        let cs = document.getElementById('callsign').value.trim().toUpperCase();
        let pw = document.getElementById('wifipass').value;
        if (cs.length < 3) { showStatus('deviceSaveStatus', 'Çağrı işareti en az 3 karakter olmalıdır!', true); return; }
        if (pw.length < 8) { showStatus('deviceSaveStatus', 'WiFi şifresi en az 8 karakter olmalıdır!', true); return; }

        let params = new URLSearchParams();
        params.append('callsign', cs);
        params.append('startup', document.getElementById('startup').value.toUpperCase());
        params.append('wifipass', pw);
        params.append('radio_type', document.getElementById('radio_type').value);
        params.append('wifi_mode', document.getElementById('wifi_mode').value);
        params.append('sta_ssid', document.getElementById('sta_ssid').value.trim());
        params.append('sta_pass', document.getElementById('sta_pass').value.trim());
        params.append('sta_ip', document.getElementById('sta_ip').value.trim());

        fetch('/saveDevice', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: params.toString()
        }).then(res => {
            if (res.ok) {
                showStatus('deviceSaveStatus', 'Kimlik, Bant ve WiFi Ağ Modu ayarları kaydedildi! (WiFi modunu değiştirdiyseniz Yeniden Başlatın)', false);
                loadConfig();
                pollLiveState();
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
        return degStr + minStr + hemi;
    }

    function decimalToAprsLon(lon) {
        let hemi = lon >= 0 ? 'E' : 'W';
        lon = Math.abs(lon);
        let deg = Math.floor(lon);
        let min = (lon - deg) * 60.0;
        let degStr = String(deg).padStart(2, '0');
        let minStr = min.toFixed(2).padStart(5, '0');
        return degStr + minStr + hemi;
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
        }, function() {
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
            }).catch(() => {});
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
            <td><input type="number" id="f_${i}" step="25" value="${f}" placeholder="433500"></td>
            <td>
                <select id="sd_${i}">
                    <option value="0" ${sd == 0 ? 'selected' : ''}>Simplex</option>
                    <option value="-1" ${sd == -1 ? 'selected' : ''}>- Shift</option>
                    <option value="1" ${sd == 1 ? 'selected' : ''}>+ Shift</option>
                </select>
            </td>
            <td><input type="number" id="s_${i}" step="25" value="${s}" placeholder="7600"></td>
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
        a.download = 'aselsan_chirp_channels.csv';
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

    function uploadOtaFirmware(e) {
        e.preventDefault();
        let fileInput = document.getElementById('otaFileInput');
        if (!fileInput.files || !fileInput.files[0]) return;
        let file = fileInput.files[0];

        let btn = document.getElementById('btnOtaSubmit');
        let wrap = document.getElementById('otaProgressWrap');
        let bar = document.getElementById('otaProgressBar');
        let status = document.getElementById('otaStatusMsg');

        btn.disabled = true;
        wrap.style.display = 'block';
        bar.style.width = '0%';
        bar.innerText = '0%';
        status.style.color = '#0056b3';
        status.innerText = 'Yazılım telsize yükleniyor, lütfen bekleyin...';

        let formData = new FormData();
        formData.append('update', file, file.name);

        let xhr = new XMLHttpRequest();
        xhr.open('POST', '/update', true);
        xhr.upload.onprogress = function(evt) {
            if (evt.lengthComputable) {
                let pct = Math.round((evt.loaded / evt.total) * 100);
                bar.style.width = pct + '%';
                bar.innerText = pct + '%';
            }
        };
        xhr.onload = function() {
            btn.disabled = false;
            if (xhr.status === 200 && xhr.responseText.indexOf('BASARILI') !== -1) {
                bar.style.width = '100%';
                bar.innerText = '100%';
                status.style.color = '#28a745';
                status.innerText = xhr.responseText;
            } else {
                status.style.color = '#dc3545';
                status.innerText = xhr.responseText || 'Güncelleme başarısız oldu!';
            }
        };
        xhr.onerror = function() {
            btn.disabled = false;
            status.style.color = '#dc3545';
            status.innerText = 'Bağlantı kesildi veya cihaz yeniden başlıyor.';
        };
        xhr.send(formData);
    }
    </script>
</body>
</html>
)rawliteral";

static void applyLiveRadioState() {
    if (op_mode == MODE_MR) {
        int startCh = current_memory_channel;
        if (startCh < 1 || startCh > 50) startCh = 1;
        char chStr[3];
        sprintf(chStr, "%02d", startCh);
        GetMemoryChannel(chStr);
        if (strncmp(FRQ, "NULL", 4) == 0) {
            int c = startCh;
            do {
                c++;
                if (c > 50) c = 1;
                sprintf(chStr, "%02d", c);
                GetMemoryChannel(chStr);
            } while (strncmp(FRQ, "NULL", 4) == 0 && c != startCh);
            if (strncmp(FRQ, "NULL", 4) == 0) {
                op_mode = MODE_VFO;
                EEPROM.write(EEPROM_OP_MODE_ADDR, MODE_VFO);
                EEPROM.get(EEPROM_CURRCHNL_BLCKSTART, current_ch);
                numberToFrequency(current_ch.frequency, FRQ);
                strcpy(FRQ_old, FRQ);
            } else {
                current_memory_channel = c;
                EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
            }
        }
        validFRQ = true;
    } else {
        numberToFrequency(current_ch.frequency, FRQ);
        strcpy(FRQ_old, FRQ);
        validFRQ = Calculate_Frequency(FRQ);
    }
    SetTone(current_ch.tone_enabled);
    write_FRQ(current_ch.frequency);
    writeFRQToLcd(FRQ);
}

static void sendLiveStateJson() {
    sendCorsHeaders();
    String lcdClean = String(FRQ);
    lcdClean.trim();
    String cleanCall = trimStoredString(mycall);

    String json = "{";
    json += "\"callsign\":\"" + jsonEscape(cleanCall) + "\",";
    json += "\"ip\":\"" + getCurrentDeviceIp() + "\",";
    json += "\"freq\":" + String(current_ch.frequency) + ",";
    json += "\"lcd\":\"" + jsonEscape(lcdClean) + "\",";
    json += "\"op_mode\":" + String(op_mode) + ",";
    json += "\"mem_ch\":" + String(current_memory_channel) + ",";
    json += "\"shift_dir\":" + String(current_ch.shift_dir) + ",";
    json += "\"shift\":" + String(current_ch.shift) + ",";
    json += "\"tone_en\":" + String(current_ch.tone_enabled) + ",";
    json += "\"tone_pos\":" + String(current_ch.tone_pos) + ",";
    json += "\"sql_mode\":" + String(SQL_MODE == SQL_OFF ? 0 : sql_level) + ",";
    json += "\"rf_power\":" + String(RF_POWER_STATE) + ",";
    json += "\"trx_mode\":" + String(TRX_MODE == TX ? 1 : 0) + ",";
    json += "\"busy\":" + String((CHANNEL_BUSY == 0 || SQL_MODE == SQL_OFF) ? 1 : 0) + ",";
    json += "\"radio_type\":" + String(radio_type) + ",";
    uint8_t pk = audioRxPeak;
    audioRxPeak = 0;
    json += "\"tot_time\":" + String(tot_values.time_out_time) + ",";
    json += "\"audio_lvl\":" + String(pk);
    json += "}";
    server.send(200, "application/json", json);
}

void handleRoot() {
    sendCorsHeaders();
    String versionString = "v" + String(SW_MAJOR) + "." + String(SW_MINOR);
    if (!isSetupDone) {
        String page = String(setupHtmlPage);
        page.replace("%VERSION%", versionString);
        page.replace("%SEL_VHF%", radio_type == 0 ? "selected" : "");
        page.replace("%SEL_UHF%", radio_type == 1 ? "selected" : "");
        page.replace("%CHK_VHF%", radio_type == 0 ? "checked" : "");
        page.replace("%CHK_UHF%", radio_type == 1 ? "checked" : "");
        server.send(200, "text/html", page);
    } else {
        String page = String(htmlPage);
        page.replace("%VERSION%", versionString);
        page.replace("%CALLSIGN%", trimStoredString(mycall));
        server.send(200, "text/html", page);
    }
}

static void startHotspotAp(const String &ssid, const String &pass) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(ssid.c_str(), pass.c_str());
    Serial.printf("[WIFI] AP Aktif: SSID=%s IP=%s\r\n", ssid.c_str(), WiFi.softAPIP().toString().c_str());
}

void initWebUI() {
    if (wifiEnabled) return;

    String cleanCall = trimStoredString(mycall);
    if (cleanCall.length() == 0) cleanCall = "TA2DMX";
    String apSsid = isSetupDone ? ("ASELSAN_" + cleanCall) : String("DLS_ASELSAN_KIT");
    String apPass = (!isSetupDone || wifiPass.length() < 8) ? String("deeplabstudio") : wifiPass;

    if (!isSetupDone || wifiMode == 0) {
        startHotspotAp(apSsid, apPass);
    } else {
        String targetSsid = staSsid;
        targetSsid.trim();
        String targetPass = staPass;
        targetPass.trim();

        if (wifiMode == 1) {
            if (targetSsid.length() == 0) targetSsid = "ASELSAN_" + cleanCall;
            if (targetPass.length() == 0) targetPass = apPass;
        }

        if (targetSsid.length() == 0) {
            Serial.println("[WIFI] STA SSID bos, AP moduna donuluyor...");
            startHotspotAp(apSsid, apPass);
        } else {
            WiFi.mode(WIFI_STA);
            IPAddress customIp;
            if (wifiMode == 1) {
                if (staStaticIp.length() < 7 || !customIp.fromString(staStaticIp)) {
                    customIp = IPAddress(192, 168, 4, 2);
                }
                WiFi.config(customIp, IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
            } else if (wifiMode == 2 && staStaticIp.length() >= 7 && customIp.fromString(staStaticIp)) {
                IPAddress gw(customIp[0], customIp[1], customIp[2], 1);
                WiFi.config(customIp, gw, IPAddress(255, 255, 255, 0));
            }

            Serial.printf("[WIFI] STA Baglaniyor (Mod %u): SSID=%s\r\n", wifiMode, targetSsid.c_str());
            WiFi.begin(targetSsid.c_str(), targetPass.c_str());

            uint32_t t0 = millis();
            while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000UL) {
                delay(250);
            }

            if (WiFi.status() == WL_CONNECTED) {
                String myIp = WiFi.localIP().toString();
                Serial.printf("[WIFI] STA Baglandi! IP=%s\r\n", myIp.c_str());
                char lcdBuf[9];
                int lastDot = myIp.lastIndexOf('.');
                if (lastDot > 0) {
                    snprintf(lcdBuf, sizeof(lcdBuf), "IP  .%s", myIp.substring(lastDot + 1).c_str());
                } else {
                    snprintf(lcdBuf, sizeof(lcdBuf), "%-8.8s", myIp.c_str());
                }
                writeToLcd(lcdBuf);
                delay(1200);
                writeFRQToLcd(FRQ);
            } else {
                Serial.println("[WIFI] 12sn icinde STA baglanamadi! Otomatik AP Hotspot (192.168.4.1) aciliyor...");
                startHotspotAp(apSsid, apPass);
            }
        }
    }

    server.onNotFound([]() {
        sendCorsHeaders();
        if (server.method() == HTTP_OPTIONS) {
            server.send(204);
        } else {
            server.send(404, "text/plain", "Not Found");
        }
    });

    server.on("/", HTTP_GET, handleRoot);

    // ------------------------------------------------------------------------
    // CANLI WEB SES MOTORU (8 kHz 8-Bit PCM RX Dinleme & TX Bas-Konus)
    // ------------------------------------------------------------------------
    server.on("/api/audio_rx", HTTP_GET, []() {
        sendCorsHeaders();
        webAudioRxActive = true;
        lastAudioRxPollMs = millis();

        static uint8_t pcmOutBuf[2048];
        uint16_t count = 0;
        uint16_t head = audioRxHead;
        uint16_t tail = audioRxTail;

        while (tail != head && count < sizeof(pcmOutBuf)) {
            pcmOutBuf[count++] = audioRxRing[tail];
            tail = (tail + 1) & (AUDIO_RING_SIZE - 1);
        }
        audioRxTail = tail;

        server.sendHeader("Cache-Control", "no-store, no-cache");
        server.send_P(200, "application/octet-stream", (const char*)pcmOutBuf, count);
    });

    server.on("/api/audio_tx", HTTP_POST, []() {
        sendCorsHeaders();
        if (server.hasArg("action")) {
            String act = server.arg("action");
            if (act == "start" || act == "keepalive") {
                setWebPttState(true);
            } else if (act == "stop") {
                setWebPttState(false);
                server.send(200, "text/plain", "OK");
                return;
            }
        }
        if (server.hasArg("pcm")) {
            setWebPttState(true);
            const String &hexStr = server.arg("pcm");
            int hexLen = hexStr.length();
            uint16_t head = audioTxHead;
            uint16_t tail = audioTxTail;
            for (int i = 0; i + 1 < hexLen; i += 2) {
                char c1 = hexStr[i];
                char c2 = hexStr[i + 1];
                uint8_t n1 = (c1 >= '0' && c1 <= '9') ? (c1 - '0') : ((c1 >= 'a' && c1 <= 'f') ? (c1 - 'a' + 10) : ((c1 >= 'A' && c1 <= 'F') ? (c1 - 'A' + 10) : 0));
                uint8_t n2 = (c2 >= '0' && c2 <= '9') ? (c2 - '0') : ((c2 >= 'a' && c2 <= 'f') ? (c2 - 'a' + 10) : ((c2 >= 'A' && c2 <= 'F') ? (c2 - 'A' + 10) : 0));
                uint8_t sample = (n1 << 4) | n2;
                uint16_t nextHead = (head + 1) & (AUDIO_RING_SIZE - 1);
                if (nextHead == tail) {
                    tail = (tail + 1) & (AUDIO_RING_SIZE - 1);
                }
                audioTxRing[head] = sample;
                head = nextHead;
            }
            audioTxTail = tail;
            audioTxHead = head;
        }
        server.send(200, "text/plain", "OK");
    });

    // ------------------------------------------------------------------------
    // CANLI ON PANEL DURUM OKUMA VE ANLIK KONTROL (/api/live)
    // ------------------------------------------------------------------------
    server.on("/api/live", HTTP_GET, []() {
        sendLiveStateJson();
    });

    server.on("/api/live", HTTP_POST, []() {
        if (TRX_MODE == TX) {
            sendLiveStateJson();
            return;
        }
        bool changed = false;
        bool needChannelSave = false;

        if (server.hasArg("action")) {
            String act = server.arg("action");
            if (act == "toggle_shift") {
                if (current_ch.shift_dir == noSHIFT) current_ch.shift_dir = plusSHIFT;
                else if (current_ch.shift_dir == plusSHIFT) current_ch.shift_dir = minusSHIFT;
                else current_ch.shift_dir = noSHIFT;
                needChannelSave = true;
                changed = true;
            } else if (act == "toggle_sql") {
                SQL_MODE = (SQL_MODE == SQL_OFF) ? SQL_ON : SQL_OFF;
                EEPROM.write(EEPROM_SQLMODE_ADDR, (uint8_t)SQL_MODE);
                changed = true;
            } else if (act == "toggle_tone") {
                current_ch.tone_enabled = (current_ch.tone_enabled == CTCSS_OFF) ? CTCSS_ON : CTCSS_OFF;
                needChannelSave = true;
                changed = true;
            } else if (act == "toggle_pwr") {
                RF_POWER_STATE = (RF_POWER_STATE == HIGH_POWER) ? LOW_POWER : HIGH_POWER;
                EEPROM.write(EEPROM_RFPOWER_ADDR, RF_POWER_STATE);
                SetRFPower();
                changed = true;
            } else if (act == "toggle_mode") {
                op_mode = (op_mode == MODE_VFO) ? MODE_MR : MODE_VFO;
                EEPROM.write(EEPROM_OP_MODE_ADDR, op_mode);
                if (op_mode == MODE_VFO) {
                    EEPROM.get(EEPROM_CURRCHNL_BLCKSTART, current_ch);
                }
                changed = true;
            } else if (act == "step_up") {
                if (op_mode == MODE_VFO) {
                    uint32_t nextF = current_ch.frequency + freq_step;
                    uint32_t maxF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_max_25 * 25UL) : 450000UL;
                    if (nextF <= maxF) {
                        current_ch.frequency = nextF;
                        needChannelSave = true;
                        changed = true;
                    }
                } else {
                    int startCh = current_memory_channel;
                    char chStr[3];
                    do {
                        current_memory_channel++;
                        if (current_memory_channel > 50) current_memory_channel = 1;
                        sprintf(chStr, "%02d", current_memory_channel);
                        GetMemoryChannel(chStr);
                    } while (strncmp(FRQ, "NULL", 4) == 0 && current_memory_channel != startCh);
                    EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
                    changed = true;
                }
            } else if (act == "step_down") {
                if (op_mode == MODE_VFO) {
                    uint32_t minF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_min_25 * 25UL) : 428150UL;
                    if (current_ch.frequency >= minF + freq_step) {
                        current_ch.frequency -= freq_step;
                        needChannelSave = true;
                        changed = true;
                    }
                } else {
                    int startCh = current_memory_channel;
                    char chStr[3];
                    do {
                        current_memory_channel--;
                        if (current_memory_channel < 1) current_memory_channel = 50;
                        sprintf(chStr, "%02d", current_memory_channel);
                        GetMemoryChannel(chStr);
                    } while (strncmp(FRQ, "NULL", 4) == 0 && current_memory_channel != startCh);
                    EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
                    changed = true;
                }
            } else if (act == "beep_test") {
                Alert_Tone(SUCC_tone);
            }
        }

        if (server.hasArg("op_mode")) {
            uint8_t newMode = (server.arg("op_mode").toInt() == 1) ? MODE_MR : MODE_VFO;
            op_mode = newMode;
            EEPROM.write(EEPROM_OP_MODE_ADDR, op_mode);
            if (op_mode == MODE_VFO) {
                EEPROM.get(EEPROM_CURRCHNL_BLCKSTART, current_ch);
            }
            changed = true;
        }
        if (server.hasArg("mem_ch")) {
            int mc = server.arg("mem_ch").toInt();
            if (mc >= 1 && mc <= 50) {
                current_memory_channel = (uint8_t)mc;
                op_mode = MODE_MR;
                EEPROM.write(EEPROM_CURR_MEMCH_ADDR, current_memory_channel);
                EEPROM.write(EEPROM_OP_MODE_ADDR, op_mode);
                changed = true;
            }
        }
        if (server.hasArg("vfo_freq")) {
            uint32_t vf = (uint32_t)server.arg("vfo_freq").toInt();
            uint32_t minF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_min_25 * 25UL) : 428150UL;
            uint32_t maxF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_max_25 * 25UL) : 450000UL;
            if (vf >= minF && vf <= maxF) {
                op_mode = MODE_VFO;
                EEPROM.write(EEPROM_OP_MODE_ADDR, MODE_VFO);
                current_ch.frequency = (vf / 25UL) * 25UL;
                needChannelSave = true;
                changed = true;
            }
        }
        if (server.hasArg("rf_power")) {
            RF_POWER_STATE = (server.arg("rf_power").toInt() == 1) ? LOW_POWER : HIGH_POWER;
            EEPROM.write(EEPROM_RFPOWER_ADDR, RF_POWER_STATE);
            SetRFPower();
            changed = true;
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
            changed = true;
        }
        if (server.hasArg("vfo_shift_dir")) {
            int sd = server.arg("vfo_shift_dir").toInt();
            if (sd == -1 || sd == 0 || sd == 1) {
                current_ch.shift_dir = (int8_t)sd;
                needChannelSave = true;
                changed = true;
            }
        }
        if (server.hasArg("vfo_shift")) {
            int sh = server.arg("vfo_shift").toInt();
            if (sh >= 0 && sh <= 20000) {
                current_ch.shift = (int16_t)((sh / 25) * 25);
                needChannelSave = true;
                changed = true;
            }
        }
        if (server.hasArg("vfo_tone_en")) {
            current_ch.tone_enabled = (server.arg("vfo_tone_en").toInt() == 1) ? CTCSS_ON : CTCSS_OFF;
            needChannelSave = true;
            changed = true;
        }
        if (server.hasArg("vfo_tone_pos")) {
            int tp = server.arg("vfo_tone_pos").toInt();
            if (tp >= 0 && tp < TOTAL_TONES) {
                current_ch.tone_pos = (uint8_t)tp;
                needChannelSave = true;
                changed = true;
            }
        }
        if (server.hasArg("tot_time")) {
            int tt = server.arg("tot_time").toInt();
            if (tt >= 15 && tt <= 240) {
                tot_values.time_out_time = (uint8_t)tt;
                EEPROM.put(EEPROM_TOT, tot_values);
                changed = true;
            }
        }

        if (needChannelSave && op_mode == MODE_VFO) {
            EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
        }
        if (changed) {
            EEPROM.commit();
            applyLiveRadioState();
            Alert_Tone(OK_tone);
        }
        sendLiveStateJson();
    });

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

        if (server.hasArg("radio_type")) {
            radio_type = (server.arg("radio_type").toInt() == 1) ? 1 : 0;
            EEPROM.write(EEPROM_RADIOTP_ADDR, radio_type);
            current_ch.frequency = (radio_type == 0) ? 145575UL : 433500UL;
            current_ch.shift = (radio_type == 0) ? 600 : 7600;
            current_ch.shift_dir = noSHIFT;
            EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
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
        String bandStr = (radio_type == 1) ? "UHF (MV 4826)" : "VHF (MV 4822)";
        String resp = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
                      "<style>body{font-family:sans-serif;background:#0b2545;color:#fff;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0;padding:20px;}"
                      ".box{background:#fff;color:#222;padding:30px;border-radius:12px;max-width:420px;text-align:center;box-shadow:0 8px 24px rgba(0,0,0,0.3);}</style></head>"
                      "<body><div class='box'><h2 style='color:#28a745;margin-top:0;'>Kurulum Tamamlandı!</h2>"
                      "<p>Seçilen Model: <b>" + bandStr + "</b></p>"
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
        sendCorsHeaders();
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
        json += "\"ip\":\"" + getCurrentDeviceIp() + "\",";
        json += "\"wifi_mode\":" + String(wifiMode) + ",";
        json += "\"sta_ssid\":\"" + jsonEscape(staSsid) + "\",";
        json += "\"sta_pass\":\"" + jsonEscape(staPass) + "\",";
        json += "\"sta_ip\":\"" + jsonEscape(staStaticIp) + "\",";
        json += "\"peer_ip\":\"" + jsonEscape(peerIp) + "\",";
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
    // KIMLIK VE WIFI AYARLARINI KAYDET (/saveDevice)
    // ------------------------------------------------------------------------
    server.on("/saveDevice", HTTP_POST, []() {
        sendCorsHeaders();
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
        if (server.hasArg("radio_type")) {
            uint8_t rt = (server.arg("radio_type").toInt() == 1) ? 1 : 0;
            if (rt != radio_type) {
                radio_type = rt;
                EEPROM.write(EEPROM_RADIOTP_ADDR, radio_type);
                uint32_t minF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_min_25 * 25UL) : 428150UL;
                uint32_t maxF = (radio_type == 0) ? ((uint32_t)freqLimits.trx_max_25 * 25UL) : 450000UL;
                if (current_ch.frequency < minF || current_ch.frequency > maxF) {
                    current_ch.frequency = (radio_type == 0) ? 145575UL : 433500UL;
                    current_ch.shift = (radio_type == 0) ? 600 : 7600;
                    current_ch.shift_dir = noSHIFT;
                    EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, current_ch);
                }
            }
        }
        if (server.hasArg("wifi_mode")) {
            int wm = server.arg("wifi_mode").toInt();
            if (wm >= 0 && wm <= 2) {
                wifiMode = (uint8_t)wm;
                EEPROM.write(EEPROM_WIFI_MODE_ADDR, wifiMode);
            }
        }
        if (server.hasArg("sta_ssid")) {
            staSsid = server.arg("sta_ssid");
            staSsid.trim();
            if (staSsid.length() > 32) staSsid = staSsid.substring(0, 32);
            for (uint8_t i = 0; i <= 32; i++) {
                EEPROM.write(EEPROM_STA_SSID_ADDR + i, (i < staSsid.length()) ? (uint8_t)staSsid[i] : 0);
            }
        }
        if (server.hasArg("sta_pass")) {
            staPass = server.arg("sta_pass");
            staPass.trim();
            if (staPass.length() > 32) staPass = staPass.substring(0, 32);
            for (uint8_t i = 0; i <= 32; i++) {
                EEPROM.write(EEPROM_STA_PASS_ADDR + i, (i < staPass.length()) ? (uint8_t)staPass[i] : 0);
            }
        }
        if (server.hasArg("sta_ip")) {
            staStaticIp = server.arg("sta_ip");
            staStaticIp.trim();
            if (staStaticIp.length() > 15) staStaticIp = staStaticIp.substring(0, 15);
            for (uint8_t i = 0; i <= 15; i++) {
                EEPROM.write(EEPROM_STA_IP_ADDR + i, (i < staStaticIp.length()) ? (uint8_t)staStaticIp[i] : 0);
            }
        }
        if (server.hasArg("peer_ip")) {
            peerIp = server.arg("peer_ip");
            peerIp.trim();
            if (peerIp.length() == 0) peerIp = "192.168.4.2";
            if (peerIp.length() > 15) peerIp = peerIp.substring(0, 15);
            for (uint8_t i = 0; i <= 15; i++) {
                EEPROM.write(EEPROM_PEER_IP_ADDR + i, (i < peerIp.length()) ? (uint8_t)peerIp[i] : 0);
            }
        }
        EEPROM.write(EEPROM_NET_MAGIC_ADDR, NET_MAGIC_VALID);

        EEPROM.commit();
        applyLiveRadioState();
        Alert_Tone(OK_tone);
        server.send(200, "text/plain", "OK");
    });

    // ------------------------------------------------------------------------
    // 2. MENU: APRS AYARLARINI KAYDET (/saveAprs) VE TEST PAKETI (/sendAprsNow)
    // ------------------------------------------------------------------------
    server.on("/saveAprs", HTTP_POST, []() {
        sendCorsHeaders();
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
        sendCorsHeaders();
        server.send(200, "text/plain", "OK");
        send_packet(_FIXPOS_STATUS, (uint32_t)freqLimits.aprs_25 * 25UL);
    });

    // ------------------------------------------------------------------------
    // REBOOT VE FABRIKA AYARLARINA SIFIRLAMA (/reboot, /factoryReset)
    // ------------------------------------------------------------------------
    server.on("/reboot", HTTP_POST, []() {
        sendCorsHeaders();
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", "Rebooting...");
        delay(800);
        rp2040.reboot();
    });

    server.on("/factoryReset", HTTP_POST, []() {
        sendCorsHeaders();
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
        sendCorsHeaders();
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
        sendCorsHeaders();
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
            applyLiveRadioState();
        }
        Alert_Tone(OK_tone);
        server.send(200, "text/plain", "OK");
    });

    // ------------------------------------------------------------------------
    // 4. MENU: KONSOL LOGLARI VE KOMUT (/getLogs, /clearLogs, /sendCommand)
    // ------------------------------------------------------------------------
    server.on("/getLogs", HTTP_GET, []() {
        sendCorsHeaders();
        server.send(200, "text/plain", webLogBuffer);
    });

    server.on("/clearLogs", HTTP_POST, []() {
        sendCorsHeaders();
        webLogBuffer = "";
        server.send(200, "text/plain", "OK");
    });

    server.on("/sendCommand", HTTP_POST, []() {
        sendCorsHeaders();
        if (server.hasArg("cmd")) {
            commandString = server.arg("cmd") + "\n";
            commandComplete = true;
        }
        server.send(200, "text/plain", "OK");
    });

    // ------------------------------------------------------------------------
    // 5. MENU: OTA GUNCELLEME (/update) - LittleFS + PicoOTA Destekli
    // ------------------------------------------------------------------------
    server.on("/update", HTTP_POST, []() {
        sendCorsHeaders();
        server.sendHeader("Connection", "close");
        if (Update.hasError() || otaErrorMsg.length() > 0) {
            String msg = "Guncelleme BASARISIZ! " + otaErrorMsg;
            server.send(200, "text/plain", msg);
        } else {
            server.send(200, "text/plain", "Guncelleme BASARILI! Cihaz yeni yazilimla yeniden baslatiliyor...");
            delay(1000);
            rp2040.reboot();
        }
    }, []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            otaErrorMsg = "";
            Serial.printf("[OTA] Basliyor: %s\r\n", upload.filename.c_str());
            size_t fsSize = (size_t)(&_FS_end - &_FS_start);
            if (fsSize < 262144) {
                otaErrorMsg = "(Flash uzerinde OTA dosya sistemi ayrilmamis. Lutfen bu surumu bir kez USB kablo ile yukleyin).";
                Serial.println("[OTA] HATA: FS boyutu yetersiz!");
                return;
            }
            if (!LittleFS.begin()) {
                Serial.println("[OTA] LittleFS formatlaniyor...");
                LittleFS.format();
                LittleFS.begin();
            }
            LittleFS.remove("firmware.bin");
            size_t maxOtaSpace = fsSize - 65536;
            size_t beginSize = (upload.totalSize > 0 && upload.totalSize < maxOtaSpace) ? upload.totalSize : maxOtaSpace;
            if (!Update.begin(beginSize)) {
                Update.printError(Serial);
                otaErrorMsg = "(Update.begin hatasi - Yetersiz LittleFS alani)";
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (otaErrorMsg.length() == 0) {
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                    otaErrorMsg = "(Flash yazma hatasi)";
                }
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (otaErrorMsg.length() == 0) {
                if (Update.end(true)) {
                    Serial.printf("[OTA] Basarili: %u byte yazildi. Yeniden baslatiliyor...\r\n", upload.totalSize);
                } else {
                    Update.printError(Serial);
                    otaErrorMsg = "(Update.end dogrulama hatasi)";
                }
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
