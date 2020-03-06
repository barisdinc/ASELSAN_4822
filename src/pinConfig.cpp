#include "pinConfig.h"


void configurePins()
{
  //pin modes and initial states
  pinMode(SQL_ACTIVE, INPUT);
  digitalWrite(SQL_ACTIVE, SQL_ON); //Disable squelch for startup
  pinMode(MIC_PIN, OUTPUT);

  pinMode(RF_POWER_PIN, OUTPUT); //RF power control is output
  digitalWrite(RF_POWER_PIN, RF_POWER_STATE);
  //TODO: store last power state and restore on every boot 
  
  pinMode(MUTE_PIN_1, OUTPUT);
  digitalWrite(MUTE_PIN_1, HIGH); //Mute the Audio output

  pinMode(BAND_SELECT_0, OUTPUT);
  pinMode(BAND_SELECT_1, OUTPUT);

  pinMode(PTT_INPUT_PIN,  INPUT_PULLUP);
  pinMode(PTT_OUTPUT_PIN, OUTPUT);
  digitalWrite(PTT_OUTPUT_PIN, LOW); //No PTT at startup

  pinMode(FWD_POWER_PIN, INPUT);
  pinMode(REF_POWER_PIN, INPUT);

  //pinMode(KeypadIntPin,    INPUT);
  pinMode(KeypadIntPin, INPUT_PULLUP);
 // attachPinChangeInterrupt(KeypadIntPin, KeyPadInterrupt, RISING);						   
  pinMode(pll_clk_pin, OUTPUT);
  pinMode(pll_data_pin,OUTPUT);
  pinMode(pll_ena_pin, OUTPUT);
  pinMode(PLL_SEC, OUTPUT);

  digitalWrite(pll_clk_pin, LOW);
  digitalWrite(pll_data_pin,LOW);
  digitalWrite(pll_ena_pin, LOW);

}