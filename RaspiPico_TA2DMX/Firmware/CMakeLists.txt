cmake_minimum_required(VERSION 3.13)

# Pico SDK'yı dahil et
include(pico_sdk_import.cmake)

project(Aselsan_Radio C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pico SDK'yı başlat
pico_sdk_init()

# --- ÇALIŞTIRILABİLİR DOSYA OLUŞTURMA ---
# Yeni eklediğimiz .cpp dosyalarını (helpers ve keypad) buraya ekledik:
add_executable(aselsan_radio
    src/main.cpp
    src/radio.cpp
    src/display.cpp
    src/storage.cpp
    src/web_server.cpp
    src/helpers.cpp     # YENİ EKLENDİ
    src/keypad.cpp      # YENİ EKLENDİ
)

# --- HEADER DOSYALARINI TANITMA ---
# src klasöründeki .h dosyalarını bulması için:
target_include_directories(aselsan_radio PRIVATE src)

# --- KÜTÜPHANE BAĞLANTILARI ---
target_link_libraries(aselsan_radio
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background # WiFi ve LwIP için
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_dma
    hardware_flash
    hardware_sync
)

# --- USB SERIAL AYARLARI ---
# printf çıktısını USB üzerinden görmek için 1 yapıyoruz
pico_enable_stdio_usb(aselsan_radio 1)
pico_enable_stdio_uart(aselsan_radio 0)

# --- ÇIKTI OLUŞTURMA ---
# .uf2, .hex, .bin vb. dosyaları oluştur
pico_add_extra_outputs(aselsan_radio)