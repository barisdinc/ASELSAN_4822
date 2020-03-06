#ifndef  PINCONFIG_H
#define  PINCONFIG_H

#include <Arduino.h> //For platformio compability

#define SQL_ACTIVE 2 //CHANNEL ACTIVE (SQUELCH) PIN

#define SQL_OFF 0
#define SQL_ON  1
int SQL_MODE = SQL_ON; //initial value for Squelch state

#define MIC_PIN  3  //D3 is our tone generation PIN (PWM)
#define CTCSS_OFF 0
#define CTCSS_ON  1
byte TONE_CTRL = CTCSS_ON; //we start without CTCSS Tone Control

//RF power control definitions
#define RF_POWER_PIN A0
#define HIGH_POWER 0
#define LOW_POWER  1
byte RF_POWER_STATE = HIGH_POWER; //Initial Power Level is Hight Power

//#define SQL_ACTIVE 2 //CHANNEL ACTIVE (SQUELCH) PIN
#define MUTE_PIN_1 6 //PIN for Audio Muting
//#define MUTE_PIN_2 5

#define BAND_SELECT_1  11
#define BAND_SELECT_0  8

//Receive/Transmit and PTT
#define PTT_OUTPUT_PIN 5
#define PTT_INPUT_PIN  12

#define FWD_POWER_PIN A6
#define REF_POWER_PIN A7

byte KeypadIntPin = 4;  //Interrupt Input PIN for MCU, D4 pin (PCINT20)

//MC145158 Programming
#define pll_clk_pin  9
#define pll_data_pin 10
#define pll_ena_pin  7

#define PLL_SEC A2


void configurePins();


#endif