#ifndef SSB_RX_H
#define SSB_RX_H

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

// Örnekleme Hızı: 200 kHz
// Pico ADC Clock (48MHz) / 240 = 200 kHz
#define CLK_DIV 240.0f  
#define ADC_SAMPLE_RATE 200000 
#define CENTER_FREQ_IF  55000  // 455 kHz alias frekansı

#define NUM_SAMPLES 256 

class SsbRx {
public:
    void init(uint8_t adc_pin, uint8_t audio_pin);
    void start();
    void set_fine_tune(int16_t offset_hz);
    void set_mode(uint8_t mode); // 0: USB, 1: LSB, 2: AM
    void process_loop(); // Core 1 döngüsü

    void dma_handler(); 

    uint16_t adc_buffer_a[NUM_SAMPLES];
    uint16_t adc_buffer_b[NUM_SAMPLES];
    
    volatile bool buffer_a_ready = false;
    volatile bool buffer_b_ready = false;

private:
    uint8_t _adc_pin;
    uint8_t _audio_pin;
    uint _dma_chan;
    
    volatile int16_t _fine_tune = 0;
    uint8_t _mode = 0;
    
    int32_t _nco_phase = 0;
    int32_t _nco_step = 0;
    
    int16_t sin_table[256];
    void update_nco();
    void process_block(uint16_t *buffer);
    void output_audio(int16_t sample);
};

extern SsbRx ssb_receiver;

#endif