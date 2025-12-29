#ifndef SSB_RX_H
#define SSB_RX_H

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

// 455 kHz civarını dinleyeceğiz
#define ADC_SAMPLE_RATE 200000 // 200 kHz (Undersampling ile 455 kHz -> 55 kHz'e düşer)
#define CENTER_FREQ_IF  55000  // 455 - (2 * 200) = 55 kHz (Aliasing sonucu)

class SsbRx {
public:
    void init(uint8_t adc_pin, uint8_t audio_pin);
    void start();
    void set_fine_tune(int16_t offset_hz); // Menüden gelen +/- ayar
    void set_mode(uint8_t mode); // 0: USB, 1: LSB, 2: AM
    void process_loop(); // Core 1'de dönecek

private:
    uint8_t _adc_pin;
    uint8_t _audio_pin;
    volatile int16_t _fine_tune = 0;
    uint8_t _mode = 0;
    
    // NCO (Osilatör) Değişkenleri
    int32_t _nco_phase = 0;
    int32_t _nco_step = 0;
    
    void update_nco();
    void process_sample(uint16_t adc_raw);
    void output_audio(int16_t sample);
};

// Global erişim için
extern SsbRx ssb_receiver;

#endif