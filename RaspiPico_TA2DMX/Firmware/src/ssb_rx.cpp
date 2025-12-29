#include "ssb_rx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

SsbRx ssb_receiver;

void dma_irq_handler_wrapper() {
    ssb_receiver.dma_handler();
}

void SsbRx::init(uint8_t adc_pin, uint8_t audio_pin) {
    _adc_pin = adc_pin;
    _audio_pin = audio_pin;

    for(int i=0; i<256; i++) {
        sin_table[i] = (int16_t)(127.0 * sinf(2.0 * M_PI * i / 256.0));
    }

    adc_init();
    adc_gpio_init(_adc_pin);
    adc_select_input(_adc_pin - 26);
    
    adc_fifo_setup(
        true,
        true,
        1,
        false,
        false
    );
    
    adc_set_clkdiv(CLK_DIV);

    gpio_set_function(_audio_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(_audio_pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, 255);
    pwm_init(slice, &config, true);

    _dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(_dma_chan);
    
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);
    
    dma_channel_configure(
        _dma_chan,
        &c,
        adc_buffer_a,
        &adc_hw->fifo,
        NUM_SAMPLES,
        false
    );

    dma_channel_set_irq0_enabled(_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler_wrapper);
    irq_set_enabled(DMA_IRQ_0, true);

    update_nco();
}

void SsbRx::start() {
    adc_run(false);
    adc_fifo_drain();
    dma_channel_set_write_addr(_dma_chan, adc_buffer_a, true);
    adc_run(true);
}

void SsbRx::dma_handler() {
    dma_hw->ints0 = 1u << _dma_chan;

    static bool filling_a = true;

    if (filling_a) {
        dma_channel_set_write_addr(_dma_chan, adc_buffer_b, true);
        buffer_a_ready = true;
        filling_a = false;
    } else {
        dma_channel_set_write_addr(_dma_chan, adc_buffer_a, true);
        buffer_b_ready = true;
        filling_a = true;
    }
}

void SsbRx::update_nco() {
    int32_t target_freq = CENTER_FREQ_IF + _fine_tune;
    // (Freq * 65536) / SampleRate
    _nco_step = (target_freq * 65536) / ADC_SAMPLE_RATE; 
}

void SsbRx::set_fine_tune(int16_t offset_hz) {
    _fine_tune = offset_hz;
    update_nco();
}

void SsbRx::set_mode(uint8_t mode) {
    _mode = mode;
}

void SsbRx::process_loop() {
    while(true) {
        if (buffer_a_ready) {
            process_block(adc_buffer_a);
            buffer_a_ready = false;
        }
        else if (buffer_b_ready) {
            process_block(adc_buffer_b);
            buffer_b_ready = false;
        }
        else {
            tight_loop_contents();
        }
    }
}

void SsbRx::process_block(uint16_t *buffer) {
    static int32_t i_filt = 0, q_filt = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        int16_t sample = buffer[i] - 2048;

        _nco_phase += _nco_step;
        uint8_t phase_idx = (_nco_phase >> 8) & 0xFF; 

        int16_t lo_i = sin_table[phase_idx]; 
        int16_t lo_q = sin_table[(phase_idx + 64) & 0xFF]; // +90 derece

        int32_t i_mix = sample * lo_i;
        int32_t q_mix = sample * lo_q;

        // IIR  filtre (FIR denenebilir)
        i_filt += (i_mix - i_filt) >> 3; 
        q_filt += (q_mix - q_filt) >> 3;

        int16_t audio_out = 0;
        if (_mode == 0)      audio_out = (i_filt - q_filt) >> 9; // USB
        else if (_mode == 1) audio_out = (i_filt + q_filt) >> 9; // LSB
        else                 audio_out = (abs(i_filt) + abs(q_filt)) >> 10; // AM

        output_audio(audio_out);
    }
}

void SsbRx::output_audio(int16_t sample) {
    if (sample > 127) sample = 127;
    if (sample < -128) sample = -128;
    
    pwm_set_gpio_level(_audio_pin, (uint8_t)(sample + 128));
}