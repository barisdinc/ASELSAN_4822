#include "ssb_rx.h"
#include <math.h>
#include <stdio.h>

SsbRx ssb_receiver;

// Basit Sinüs Tablosu (Hız için)
static int16_t sin_table[256]; // 8-bit çözünürlük yeterli olabilir

void SsbRx::init(uint8_t adc_pin, uint8_t audio_pin) {
    _adc_pin = adc_pin;
    _audio_pin = audio_pin;

    // Sinüs tablosunu doldur
    for(int i=0; i<256; i++) {
        sin_table[i] = (int16_t)(127.0 * sinf(2.0 * M_PI * i / 256.0));
    }

    // ADC Init
    adc_init();
    adc_gpio_init(_adc_pin);
    adc_select_input(_adc_pin - 26);
    
    // PWM Audio Init (Mevcut config.h'deki pin)
    gpio_set_function(_audio_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(_audio_pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f); 
    pwm_config_set_wrap(&config, 255); 
    pwm_init(slice, &config, true);

    update_nco();
}

void SsbRx::update_nco() {
    // Hedef frekans: 55 kHz (Aliased IF) + Fine Tune
    int32_t target_freq = CENTER_FREQ_IF + _fine_tune;
    
    // Phase Step hesaplama: (Freq * 2^32) / SampleRate
    // Basitleştirilmiş: (Freq * 65536) / SampleRate (16-bit hassasiyet için)
    _nco_step = (target_freq * 256) / (ADC_SAMPLE_RATE / 256); // Yaklaşık hesap
}

void SsbRx::set_fine_tune(int16_t offset_hz) {
    _fine_tune = offset_hz;
    update_nco();
}

void SsbRx::set_mode(uint8_t mode) {
    _mode = mode;
}

void SsbRx::process_loop() {
    // Gerçek zamanlı döngü (Core 1)
    // PicoRX'teki gibi DMA buffer beklenmeli, burada basitleştirilmiş halini yazıyorum.
    
    while(true) {
        // 1. ADC'den örnek al (Bloklayıcı veya FIFO kontrolü)
        // Gerçek uygulamada DMA kesmesi kullanılır.
        uint16_t raw = adc_read(); 
        
        // DC Offseti çıkar (12-bit ADC ortası 2048)
        int16_t sample = raw - 2048;

        // 2. Karıştırma (Mixer) - NCO ile çarpım
        _nco_phase += _nco_step;
        uint8_t phase_idx = (_nco_phase >> 24) & 0xFF; // Üst 8 biti al
        
        int16_t lo_i = sin_table[phase_idx];          // Cosine (yaklaşık)
        int16_t lo_q = sin_table[(phase_idx + 64) & 0xFF]; // Sine

        int32_t i_mix = sample * lo_i;
        int32_t q_mix = sample * lo_q;

        // 3. Filtreleme (Low Pass) - Basit bir IIR veya Moving Average
        // SSB için Yan bant bastırma burada yapılır.
        static int32_t i_filt = 0, q_filt = 0;
        i_filt += (i_mix - i_filt) >> 4; // Basit Low Pass
        q_filt += (q_mix - q_filt) >> 4;

        // 4. Demodülasyon (Phasing Method)
        int16_t audio_out = 0;
        if (_mode == 0) { // USB
            audio_out = (i_filt - q_filt) >> 8; 
        } else if (_mode == 1) { // LSB
            audio_out = (i_filt + q_filt) >> 8;
        } else { // AM (Genlik)
            // sqrt(I^2 + Q^2) yaklaşık hesabı
            audio_out = (abs(i_filt) + abs(q_filt)) >> 9; 
        }

        output_audio(audio_out);
        
        // Örnekleme hızını tutturmak için bekleme (ADC_SAMPLE_RATE'e göre)
        sleep_us(2); // Çok kaba bir zamanlama, DMA+PIO ile bu kısım otomatikleşir.
    }
}

void SsbRx::start() {
    // Basitleştirilmiş versiyonda (adc_read kullanan),
    // start fonksiyonu sadece ADC'nin hazır olduğundan emin olur.
    // İleride PIO/DMA kullanırsak state machine burada başlatılacak.
    
    // Şimdilik boş bırakabilir veya ADC'yi resetleyebilirsiniz.
    // Eğer free-running modu açıksa kapatalım ki adc_read() çakışmasın.
    adc_run(false); 
}

void SsbRx::output_audio(int16_t sample) {
    // 8-bit PWM çıkışı
    // sample -128 ile +127 arasındaysa 0-255'e çek
    uint8_t pwm_val = (uint8_t)(sample + 128);
    pwm_set_gpio_level(_audio_pin, pwm_val);
}
