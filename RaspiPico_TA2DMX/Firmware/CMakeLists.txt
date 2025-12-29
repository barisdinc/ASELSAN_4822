cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(Aselsan_Radio C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(aselsan_radio
    src/main.cpp
    src/radio.cpp
    src/display.cpp
    src/storage.cpp
    src/web_server.cpp
    src/helpers.cpp
    src/keypad.cpp
    src/ssb_rx.cpp
    src/adc_sampler.pio
)

pico_generate_pio_header(aselsan_radio ${CMAKE_CURRENT_LIST_DIR}/src/adc_sampler.pio)

target_include_directories(aselsan_radio PRIVATE src)

target_link_libraries(aselsan_radio
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background # WiFi ve LwIP için
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_dma
    hardware_flash
    hardware_sync
    pico_multicore
    hardware_interp
)

# printf çıktısını USB üzerinden görmek için 1 yapıyoruz
pico_enable_stdio_usb(aselsan_radio 1)
pico_enable_stdio_uart(aselsan_radio 0)

# .uf2, .hex, .bin vb. dosyaları oluştur
pico_add_extra_outputs(aselsan_radio)