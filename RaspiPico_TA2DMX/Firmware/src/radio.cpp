#include "radio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include <math.h>
#include <stdio.h>
#include "stdint-gcc.h"

//VHF BAND SELECTION  
//BS0 BS1
// 0  0     152.2   172.6
// 0  1     147.9   165.5
// 1  0     141.6   157.1
// 1  1     137.2   151.4
//+------------+---------------+-----+-----+
//| Radio Type |  Frequency    | BS0 | BS1 |
//+------------+---------------+-----+-----+
//| VHF        | 152.2   172.6 |   0 |   0 |
//|            | 147.9   165.5 |   0 |   1 |
//|            | 141.6   157.1 |   1 |   0 |
//|            | 137.2   151.4 |   1 |   1 |
//+------------+---------------+-----+-----+

//+------------+---------------+-----+-----+
//| Radio Type |  Frequency    | BS0 | BS1 |
//+------------+---------------+-----+-----+
//| UHF        | 406.0   418.0 |   1 |   1 |
//|            | 418.0   430.0 |   0 |   1 |
//|            | 440.0   455.0 |   1 |   0 |
//|            | 455.0   470.0 |   0 |   0 |
//+------------+---------------+-----+-----+

ChannelConfig current_channel;
bool is_transmitting = false;

// Sinüs Tablosu (8-bit)
#define SINE_TABLE_SIZE 256
const uint8_t sine_wave[SINE_TABLE_SIZE] = {
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173,
  176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 236, 237, 239, 240, 242, 243, 245,
  246, 247, 248, 249, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 255,
  255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 249, 248, 247, 246, 245,
  243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 220, 218, 215,
  213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 188, 185, 182, 179, 176, 173,
  170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131, 128, 125,
  122, 119, 116, 113, 110, 107, 104, 101, 98, 95, 92, 89, 86, 83, 80, 77,
  74, 71, 68, 65, 62, 60, 57, 54, 52, 49, 47, 44, 42, 39, 37, 35,
  32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 13, 11, 10, 8, 7, 5,
  4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 3,
  4, 5, 6, 7, 8, 10, 11, 13, 14, 16, 18, 20, 22, 24, 26, 28,
  30, 32, 35, 37, 39, 42, 44, 47, 49, 52, 54, 57, 60, 62, 65, 68,
  71, 74, 77, 80, 83, 86, 89, 92, 95, 98, 101, 104, 107, 110, 113, 116,
  119, 122, 125, 127, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128
};

// PWM Değişkenleri
uint slice_num;
volatile int phase_acc = 0;
struct repeating_timer tone_timer;

// Timer Callback (Kesme Fonksiyonu)
bool timer_callback(struct repeating_timer *t) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, sine_wave[phase_acc]);
    phase_acc = (phase_acc + 1) % SINE_TABLE_SIZE;
    return true;
}

void radio_init() {
    // GPIO Ayarları
    gpio_init(PLL_CLK_PIN); gpio_set_dir(PLL_CLK_PIN, GPIO_OUT);
    gpio_init(PLL_DATA_PIN); gpio_set_dir(PLL_DATA_PIN, GPIO_OUT);
    gpio_init(PLL_ENA_PIN); gpio_set_dir(PLL_ENA_PIN, GPIO_OUT);
    gpio_init(PLL_SEC_PIN); gpio_set_dir(PLL_SEC_PIN, GPIO_OUT);
    
    gpio_init(MUTE_PIN); gpio_set_dir(MUTE_PIN, GPIO_OUT);

    gpio_init(SQL_ACTIVE_PIN);
    gpio_set_dir(SQL_ACTIVE_PIN, GPIO_IN);
    

    gpio_init(PTT_OUT_PIN); gpio_set_dir(PTT_OUT_PIN, GPIO_OUT);
    gpio_init(PTT_IN_PIN);  gpio_set_dir(PTT_IN_PIN, GPIO_IN); gpio_pull_up(PTT_IN_PIN);
    
    gpio_init(BAND_SEL0_PIN); gpio_set_dir(BAND_SEL0_PIN, GPIO_OUT);
    gpio_init(BAND_SEL1_PIN); gpio_set_dir(BAND_SEL1_PIN, GPIO_OUT);

    // ADC Başlatma
    adc_init();
    adc_gpio_init(FWD_PWR_PIN);
    adc_gpio_init(REF_PWR_PIN);

    // PWM Başlatma (Ses)
    gpio_set_function(AUDIO_PWM_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(AUDIO_PWM_PIN);
    pwm_set_wrap(slice_num, 255);
    pwm_set_enabled(slice_num, true);
}

void set_pll_bits(int value, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
        gpio_put(PLL_DATA_PIN, (value >> i) & 1);
        sleep_us(10);
        gpio_put(PLL_CLK_PIN, 1);
        sleep_us(10);
        gpio_put(PLL_CLK_PIN, 0);
    }
}

void set_pll(uint32_t frequency) {
    uint32_t freqToSet = frequency;
    // ... Shift hesaplamaları buraya eklenecek ...
    
    int R_Counter = 12800 / 12.5; 
    int N_Counter = freqToSet / 12.5 / 80;
    int A_Counter = (freqToSet / 12.5) - (80 * N_Counter);
    printf("A : %d N: %d R: %d \n", A_Counter, N_Counter, R_Counter);

    gpio_put(PLL_SEC_PIN, 0);
    set_pll_bits(R_Counter, 14);
    set_pll_bits(1, 1);
    
    gpio_put(PLL_ENA_PIN, 1); sleep_us(10); gpio_put(PLL_ENA_PIN, 0);

    set_pll_bits(N_Counter, 10);
    set_pll_bits(A_Counter, 7);
    set_pll_bits(0, 1);

    gpio_put(PLL_ENA_PIN, 1); sleep_us(10); gpio_put(PLL_ENA_PIN, 0);
    gpio_put(PLL_SEC_PIN, 1);
}

void start_tone(float frequency) {
    float interval_us = 1000000.0f / (frequency * SINE_TABLE_SIZE);
    add_repeating_timer_us(-interval_us, timer_callback, NULL, &tone_timer);
}

void stop_tone() {
    cancel_repeating_timer(&tone_timer);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
}

void check_ptt() {
    if(!gpio_get(PTT_IN_PIN)) { // PTT Basılı (LOW)
        if(!is_transmitting) {
            is_transmitting = true;
            gpio_put(PTT_OUT_PIN, 1);
            if(current_channel.tone_enabled) start_tone(88.5); // Örnek ton
        }
    } else {
        if(is_transmitting) {
            is_transmitting = false;
            gpio_put(PTT_OUT_PIN, 0);
            stop_tone();
        }
    }
}

void set_FRQ(uint32_t Frequency) {
    static uint32_t last_freq = 0;
//VHF BAND SELECTION  
//BS0 BS1
// 0  0     152.2   172.6
// 0  1     147.9   165.5
// 1  0     141.6   157.1
// 1  1     137.2   151.4
//+------------+---------------+-----+-----+
//| Radio Type |  Frequency    | BS0 | BS1 |
//+------------+---------------+-----+-----+
//| VHF        | 152.2   172.6 |   0 |   0 |
//|            | 147.9   165.5 |   0 |   1 |
//|            | 141.6   157.1 |   1 |   0 |
//|            | 137.2   151.4 |   1 |   1 |
//+------------+---------------+-----+-----+

//+------------+---------------+-----+-----+
//| Radio Type |  Frequency    | BS0 | BS1 |
//+------------+---------------+-----+-----+
//| UHF        | 406.0   418.0 |   1 |   1 |
//|            | 418.0   430.0 |   0 |   1 |
//|            | 440.0   455.0 |   1 |   0 |
//|            | 455.0   470.0 |   0 |   0 |
//+------------+---------------+-----+-----+

//uint32_t UpdatedFrq = 0;
//float divider = 0;
//   if (validFRQ) {
     if(RADIO_TYPE == VHF)
     {
       if ((Frequency < 174000L) & (Frequency >= (166000L)))  { gpio_put(BAND_SEL0_PIN, 0); gpio_put(BAND_SEL1_PIN,0); }
       if ((Frequency < 166000L) & (Frequency >= (158000L)))  { gpio_put(BAND_SEL0_PIN, 1); gpio_put(BAND_SEL1_PIN,0); } 
       if ((Frequency < 158000L) & (Frequency >= (152000L)))  { gpio_put(BAND_SEL0_PIN, 0); gpio_put(BAND_SEL1_PIN,1); } 
       if ((Frequency < 152000L) & (Frequency >= (134000L)))  { gpio_put(BAND_SEL0_PIN, 1); gpio_put(BAND_SEL1_PIN,1); } 
     }
     else if(RADIO_TYPE == UHF)
     {
        if ((Frequency < 470000L) & (Frequency >= (455000L))) { gpio_put(BAND_SEL0_PIN, 0); gpio_put(BAND_SEL1_PIN, 0); }
        if ((Frequency < 455000L) & (Frequency >= (430000L))) { gpio_put(BAND_SEL0_PIN, 1); gpio_put(BAND_SEL1_PIN, 0); }
        if ((Frequency < 430000L) & (Frequency >= (418000L))) { gpio_put(BAND_SEL0_PIN, 0); gpio_put(BAND_SEL1_PIN, 1); }
        if ((Frequency < 418000L) & (Frequency >= (406000L))) { gpio_put(BAND_SEL0_PIN, 1); gpio_put(BAND_SEL1_PIN, 1); }
//     // Update EEPROM for last used Frequncy
     }
     // //BD1    current_ch.frequency = Frequency; //UpdatedFrq;
// //BD1    EEPROM.put(EEPROM_CURRCHNL_BLCKSTART,current_ch); 

     if (Frequency != last_freq) set_pll(Frequency);
     last_freq = Frequency;
     printf("freq = %d \n", Frequency);
//   } //validFRQ 
}

