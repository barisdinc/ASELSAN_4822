#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// KULLANICI AYARLARI (USER CONFIGURATION)
// ==========================================

// Cihaz açıldığında ekranda görünecek varsayılan mesaj (Maksimum 6 karakter)
#define DEFAULT_STARTUP_MSG   "TA2DMX"

// APRS Varsayılan Ayarları
#define APRS_DEFAULT_MYCALL   "TA2DMX" 
#define APRS_DEFAULT_SSID     "9"
#define APRS_DEFAULT_MESSAGE  "NOCALL" // Maksimum 28 karakter
#define APRS_DEFAULT_LAT      "3955.50N" // Enlem (Örn: Anıtkabir)
#define APRS_DEFAULT_LON      "3250.22E" // Boylam (Örn: Anıtkabir)

// ==========================================
// FREKANS LİMİTLERİ (KHz / 25)
// ==========================================
#define DEFAULT_VHF_MINIMUM_FREQ  5360 // 134.000 MHz (134000/25)
#define DEFAULT_UHF_MINIMUM_FREQ  16000 // 400.000 MHz (400000/25)
#define DEFAULT_VHF_MAXIMUM_FREQ  6960 // 174.000 MHz (174000/25)
#define DEFAULT_UHF_MAXIMUM_FREQ  18800 // 470.000 MHz (470000/25)

#define DEFAULT_VHF_SCAN_LOWER    5760 // 144.000 MHz
#define DEFAULT_VHF_SCAN_UPPER    5840 // 146.000 MHz
#define DEFAULT_UHF_SCAN_LOWER    17200 // 430.000 MHz
#define DEFAULT_UHF_SCAN_UPPER    17600 // 440.000 MHz

#define DEFAULT_APRS_VHF_FREQ     5792 // 144.800 MHz (APRS)
#define DEFAULT_ISS_APRS_FREQ     5833 // 145.825 MHz (ISS)

#define DEFAULT_VHF_VNA_MINIMUM_FREQ  5600 // 140.000 MHz (VNA Alt Limit)
#define DEFAULT_VHF_VNA_MAXIMUM_FREQ  6000 // 150.000 MHz (VNA Üst Limit)


#endif
