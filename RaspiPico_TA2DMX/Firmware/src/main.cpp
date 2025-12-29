#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "config.h"
#include "radio.h"
#include "display.h"
#include "storage.h"
#include "helpers.h"
#include "keypad.h"
#include "stdint-gcc.h"
#include "pico/multicore.h"
#include "ssb_rx.h"
// IF Giriş pini  GP28/ADC2
#define IF_INPUT_PIN 28
int16_t ssb_fine_tune = 0;

// --- Global Değişkenler ---
char FRQ[9];
char FRQ_old[9];
char lcd_buf[16]; 

bool validFRQ = true;
uint8_t numChar = 0;
char pressedKEY = ' ';
char old_pressedKEY = ' ';

uint8_t trx_mode = 0; 
uint8_t scr_mode = 0; 
uint32_t scr_timer = 0;
uint8_t menu_index = 0; 
const int MAX_MENU = 3; 

bool sql_override = false; // Monitor
bool key_lock = false;     
bool light_latch = false;
bool is_scanning = false;  

uint32_t last_millis = 0;
uint32_t ptt_start_time = 0;
uint32_t key_press_time = 0; 

volatile bool keypad_event_flag = false; // Keypad kesmesi

void core1_entry() {
    // SSB alıcısı
    ssb_receiver.init(IF_INPUT_PIN, AUDIO_PWM_PIN);
    ssb_receiver.start();
    ssb_receiver.process_loop();
}



void alert_tone(int type) {
    if(type == 1) start_tone(1000); 
    if(type == 2) start_tone(400);  
    sleep_ms(100);
    stop_tone();
}

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == KEYPAD_INT_PIN) {
        keypad_event_flag = true; 
    }
}

void update_display() {
    memset(lcd_buf, 0, sizeof(lcd_buf));

    if (key_lock) { display_write_text("LOCKED  "); return; }
    if (scr_mode == 3) { display_write_text("SCAN... "); return; }

    if (scr_mode == 0) { 
        if (sql_override) display_write_text("MONITOR ");
        else { numberToFrequency(current_channel.frequency, FRQ); display_write_freq(FRQ); }
    } 
    else if (scr_mode == 1 || scr_mode == 2) { 
        if (menu_index == 0) strcpy(lcd_buf, scr_mode==1 ? "SET SHFT" : (current_channel.shift_dir==0 ? "SIMPLX" : (current_channel.shift_dir==1 ? "PLUS +" : "MINUS -")));
        else if (menu_index == 1) { if(scr_mode==1) strcpy(lcd_buf, "SET OFST"); else sprintf(lcd_buf, "S %02d.%03d", current_channel.shift/1000, current_channel.shift%1000); }
        else if (menu_index == 2) { if(scr_mode==1) strcpy(lcd_buf, "SET TONE"); else get_tone_name(current_channel.tone_pos, lcd_buf); }
        else if (menu_index == 3) { if(scr_mode==1) strcpy(lcd_buf, "TONE SW"); else strcpy(lcd_buf, current_channel.tone_enabled ? "TONE ON" : "TONE OFF"); }
        else if (menu_index == 4) { sprintf(lcd_buf, "FINE %d", ssb_fine_tune); display_write_text(lcd_buf); }
    }
}

void check_squelch() {
    bool speaker_on = false;
    if (gpio_get(SQL_ACTIVE_PIN) == 0) speaker_on = true;
    if (sql_override) speaker_on = true;

    if (speaker_on) {
        gpio_put(MUTE_PIN, 0); 
        set_led(LED_GREEN, true);
    } else {
        gpio_put(MUTE_PIN, 1); 
        set_led(LED_GREEN, false);
    }
}

void process_scan() {
    if (!is_scanning) return;
    if (gpio_get(SQL_ACTIVE_PIN) == 0) { 
        scr_mode = 3; 
        numberToFrequency(current_channel.frequency, FRQ);
        display_write_freq(FRQ);
        return; 
    }
    static uint32_t last_scan_time = 0;
    if (to_ms_since_boot(get_absolute_time()) - last_scan_time > 150) {
        last_scan_time = to_ms_since_boot(get_absolute_time());
        current_channel.frequency += 25; 
        if (current_channel.frequency > 440000 && current_channel.frequency < 400000) current_channel.frequency = 144000;
        if (current_channel.frequency > 470000) current_channel.frequency = 430000;
        set_FRQ(current_channel.frequency);
        display_write_text("SCANNING");
    }
}

void handle_keypad_input() {
    pressedKEY = keypad_read();
    
    // if (pressedKEY != 0) {
    //     printf("Key Processed: %c\n", pressedKEY);
    // }

    static bool long_press_handled = false;
    if (pressedKEY != 0 && pressedKEY == old_pressedKEY) {
        if (!long_press_handled && (to_ms_since_boot(get_absolute_time()) - key_press_time > 1500)) {
            if (pressedKEY == 'S') { 
                is_scanning = !is_scanning;
                if(is_scanning) { sql_override = false; scr_mode = 3; } else scr_mode = 0;
                alert_tone(is_scanning ? 1 : 2); update_display();
            }
            long_press_handled = true; 
        }
    } else {
        long_press_handled = false; 
        if (pressedKEY != 0) key_press_time = to_ms_since_boot(get_absolute_time());
    }

    if (pressedKEY != 0 && pressedKEY != old_pressedKEY) {
        alert_tone(1); scr_timer = 10000; 

        if (is_scanning) { is_scanning = false; scr_mode = 0; update_display(); old_pressedKEY = pressedKEY; return; }
        if (key_lock) { if (pressedKEY == '#') { key_lock = false; display_write_text("UNLOCKED"); sleep_ms(500); update_display(); } old_pressedKEY = pressedKEY; return; }

        if (scr_mode == 0) { 
            if (pressedKEY >= '0' && pressedKEY <= '9') {
                if (sql_override) sql_override = false; 
                if (numChar == 0) { strcpy(FRQ_old, FRQ); memset(FRQ, ' ', 8); FRQ[8]=0; }
                if (numChar <= 6) { FRQ[numChar++] = pressedKEY; if (numChar == 3) FRQ[numChar++] = '.'; display_write_freq(FRQ); }
                if (numChar == 7) {
                    uint32_t new_freq;
                    if (Calculate_Frequency(FRQ, &new_freq)) {
                        current_channel.frequency = new_freq; set_FRQ(current_channel.frequency); save_channel(&current_channel); update_display();
                    } else { alert_tone(2); strcpy(FRQ, FRQ_old); update_display(); }
                    numChar = 0;
                }
            }
            else if (pressedKEY == 'U') { current_channel.frequency += 25; set_FRQ(current_channel.frequency); update_display(); }
            else if (pressedKEY == 'D') { current_channel.frequency -= 25; set_FRQ(current_channel.frequency); update_display(); }
            else if (pressedKEY == 'M') { scr_mode = 1; menu_index = 0; update_display(); }
            
            else if (pressedKEY == 'S') { 
                sql_override = !sql_override; 
                update_display();
                check_squelch(); 
            }
            
            else if (pressedKEY == 'C') { light_latch = !light_latch; set_backlight(light_latch); }
            else if (pressedKEY == '*') { current_channel.tone_enabled = !current_channel.tone_enabled; update_display(); }
            else if (pressedKEY == '#') { key_lock = true; display_write_text("LOCKED  "); sleep_ms(500); update_display(); }
        }
        else if (scr_mode == 1) { 
            if (pressedKEY == 'U') { menu_index++; if (menu_index > MAX_MENU) menu_index = 0; update_display(); }
            else if (pressedKEY == 'D') { if (menu_index == 0) menu_index = MAX_MENU; else menu_index--; update_display(); }
            else if (pressedKEY == 'M' || pressedKEY == '*') { scr_mode = 2; update_display(); }
            else if (pressedKEY == '#') { scr_mode = 0; save_channel(&current_channel); update_display(); }
        }
        else if (scr_mode == 2) { 
            bool changed = false;
            if (pressedKEY == 'U') { changed=true; if(menu_index==0) current_channel.shift_dir = (current_channel.shift_dir==1)?-1:current_channel.shift_dir+1; else if(menu_index==1) current_channel.shift+=50; else if(menu_index==2) {current_channel.tone_pos++; if(current_channel.tone_pos>=tone_count)current_channel.tone_pos=0;} else if(menu_index==3) current_channel.tone_enabled=!current_channel.tone_enabled; }
            else if (pressedKEY == 'D') { changed=true; if(menu_index==0) current_channel.shift_dir = (current_channel.shift_dir==-1)?1:current_channel.shift_dir-1; else if(menu_index==1) {if(current_channel.shift>=50)current_channel.shift-=50;} else if(menu_index==2) {if(current_channel.tone_pos==0)current_channel.tone_pos=tone_count-1;else current_channel.tone_pos--;} else if(menu_index==3) current_channel.tone_enabled=!current_channel.tone_enabled; }
            else if (pressedKEY == 'M' || pressedKEY == '#' || pressedKEY == '*') { scr_mode = 1; update_display(); return; }
            else if (menu_index == 4) {
                if (pressedKEY == 'U') ssb_fine_tune += 10;// 10 Hz adımlarla ince ayar
                if (pressedKEY == 'D') ssb_fine_tune -= 10;
                ssb_receiver.set_fine_tune(ssb_fine_tune);
            }            
            if(changed) update_display();
        }
    }
    old_pressedKEY = pressedKEY;
}

int main() {
    stdio_init_all();
    multicore_launch_core1(core1_entry);

    sleep_ms(2000);

    storage_init();
    load_channel(&current_channel);
    
    if (current_channel.frequency < 10000 || current_channel.frequency > 500000) {
        current_channel.frequency = 145000; current_channel.shift = 600; 
        current_channel.tone_enabled = false; current_channel.shift_dir = 0;
    }

    radio_init();
    display_init();
    keypad_init();
   
    keypad_event_flag = false;
    gpio_set_irq_enabled_with_callback(KEYPAD_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    set_backlight(true);
    display_write_text("ASELSAN ");
    sleep_ms(1000);
    update_display(); 
    set_FRQ(current_channel.frequency);
    
    while (1) {
        uint32_t current_millis = to_ms_since_boot(get_absolute_time());

        if (gpio_get(PTT_IN_PIN) == 1) { 
            if (is_scanning) is_scanning = false;
            if (trx_mode == 0) {
                trx_mode = 1; ptt_start_time = current_millis;
                uint32_t tx_freq = current_channel.frequency;
                if(current_channel.shift_dir == 1) tx_freq += current_channel.shift;      
                else if(current_channel.shift_dir == -1) tx_freq -= current_channel.shift;
                set_FRQ(tx_freq);
                if(current_channel.tone_enabled) start_tone(tone_list[current_channel.tone_pos]);
            }
            if ((current_millis - ptt_start_time) > 180000) { 
                 trx_mode = 0; gpio_put(PTT_OUT_PIN, 0); stop_tone(); alert_tone(2);
            }
        } else { 
            // if (trx_mode == 1) {
                set_FRQ(current_channel.frequency + IF_FREQ); 
            // }
        }

        check_squelch(); 
        process_scan(); 
        if (keypad_event_flag) {
            gpio_set_irq_enabled(KEYPAD_INT_PIN, GPIO_IRQ_EDGE_FALL, false);
            sleep_ms(30); 
            handle_keypad_input(); 
            keypad_event_flag = false; 
            gpio_set_irq_enabled(KEYPAD_INT_PIN, GPIO_IRQ_EDGE_FALL, true);            
        }
        
        if (pressedKEY != 0) {
             handle_keypad_input();
             sleep_ms(50); // Çok hızlı döngüyü yavaşlat
        }

        if (scr_mode != 0 && scr_mode != 3 && scr_timer > 0) { 
            scr_timer--;
            if (scr_timer == 0) { scr_mode = 0; save_channel(&current_channel); update_display(); }
        }
        sleep_ms(5);
    }
    return 0;
}