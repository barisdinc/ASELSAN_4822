#ifndef ASELSAN_4822_H
#define ASELSAN_4822_H

#include "Arduino.h"

void calc_crc(bool in_bit);
void send_flag(unsigned char flag_len);
void send_header(void);
void send_payload(char type);
//void set_io(void);
void getGPSData();
//void StreamPrint_progmem(Print &out,PGM_P format,...);
void InitLCD() ;
void sendToLcd(uint8_t *data, uint8_t position) ;
void writeToLcd(const char text[]) ;
void writeFRQToLcd(const char frq[9]);
void scroll(const char *text, int speed) ;
void Greetings() ;
void send_SPIBit(int Counter, uint8_t length) ;
void send_SPIEnable() ;
void SetTone(int toneSTATE) ;
void Alert_Tone(int ToneType);
void SetPLLLock(uint32_t Frequency);
void write_FRQ(uint32_t Frequency) ;
void write_SHIFTtoLCD(uint16_t FRQshift) ;
void write_TONEtoLCD(unsigned long tone_pos) ;
//void SetRFPower(int rfpowerSTATE) ;
void SetRFPower() ;
void setRadioPower() ;
void readRfPower() ;
void numberToFrequency(uint32_t Freq, char *rFRQ) ;
void eeread_nbytes(String *dst, uint8_t sz, uint16_t addr);
void eeprom_readAPRS() ;
void eewrite_nbytes(String dst, uint8_t sz, uint16_t addr) ;
void eeprom_writeAPRS() ;
void initialize_eeprom() ;
void StoreFrequency(char mCHNL[9], char mFRQ[9]) ;
void GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg) ;
void GetMemoryChannel(char mFRQ[9]) ;
void PrintMenu() ;
void commandHelp(char komut) ;
void commandRadioType(char komut) ;
void commandStartupMSG() ;
void commandDumpConfig() ;
void commandMemoryDump() ;
void commandMemoryChannel() ;
void commandAPRSTimeout() ;
void commandAPRSMessage() ;
void commandFrequencyLowerLimit();
void commandFrequencyUpperLimit();
void commandScanLowerLimit();
void commandScanUpperLimit();
void commandAprsFrequency();
void commandISSFrequency();
void getEEPROMData() ;
void commandAPRSmycall() ;
void commandTogglePTT() ;
void StoreSpecialFrequency(char mCHNL[9], char mFRQ[9]) ;
void startScan() ;
void StreamPrint_progmem(Print &out,PGM_P format,...) ;
void serialEvent() ;
void readParam(char *szParam, int iMaxLen) ;
void set_nada_1200(void) ;
void set_nada_2400(void) ;
void set_nada(bool nada) ;
void calc_crc(bool in_bit) ;
void send_crc(void) ;
void send_header(void) ;
void send_payload(char type) ;
void send_char_NRZI(unsigned char in_byte, bool enBitStuff) ;
void send_string_len(String in_string, int len) ;
void send_flag(unsigned char flag_len) ;
void send_packet(char packet_type, uint32_t frequency) ;
void randomize(unsigned int &var, unsigned int low, unsigned int high) ;
void getGPSData() ;
void softResetDevice();

#endif
