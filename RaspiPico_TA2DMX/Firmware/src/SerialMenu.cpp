#include "main.h"

void PrintMenu()
{  
//  print_version();
  Serialprint("ASELSAN 48xx - TAMSAT Kit (v%d.%d-%s) \n\r",SW_MAJOR,SW_MINOR, __DATE__);
  Serialprint("-------------------------\n\r");
  //Serialprint("Y-Yardim\n\r");
  Serialprint("C-VHF/UHF Cevrimi Yap\n\r");
  Serialprint("A-Acilis Mesaji Degistir\n\r");
  Serialprint("F-Frekans Sinirlari\n\r");
  Serialprint("V-Analizor Sinirlari\n\r");
  Serialprint("H-Hafiza Islemleri\n\r");
  Serialprint("K-Konfigurasyon Yaz\n\r");
  Serialprint("S-APRS cagri isareti\n\r");
  Serialprint("T-APRS sessizlik suresi\n\r");
  Serialprint("M-APRS Mesaji\n\r");  
  Serialprint("G-GPS Oku\n\r");  
  Serialprint("W-WiFi/Web Arayuzu Baslat\n\r");
  Serialprint("J-Frekans Adimi Degistir (12.5/25)\n\r");
  Serialprint("Y-PLL Kalibrasyon Taramasi\n\r");
  Serialprint("m-PLL Yazim Modu Degistir (su an %d)\n\r", pllMode);
  Serialprint("z-A=0 Testi (145.000 MHz)\n\r");
  Serialprint("p-PTT Otomatik Testi (3 saniye)\n\r");
  Serialprint("t-PTT Polaritesi Degistir\n\r");
  Serialprint("r-R Sayaci Degistir (1024/512)\n\r");
  Serialprint("v-CTCSS Ton Modu Degistir (su an %d)\n\r", micTxMode);
  Serialprint("Seciminiz >\n\r");
}

void commandYardim(char komut)
{
  /*
  Serialprint("Yardim Menusu\r\n=====================\r\n");
  if (komut=='\n')
  {
    Serialprint("Yardim almak istediginiz komutu belirtiniz...\r\n");
    Serialprint("Ornek : Y C  veya Y F  veya Y A \r\n");
  } else if (komut == 'C')
  {
    Serialprint("Bu komut cihazinizi UHF ya da VHF olarak programlamanizi saglar. (sadece TAMSAT Kartini)\r\n");
    Serialprint("Kullanimi : \r\n      C V : VHF yap\r\n      C U : UHF yap");  
  } 
  else if (komut == 'A')
  {
    Serialprint("Kullanimi : \r\n      A [MESAJ]   Ornek:  A TA7W  (maksimum 6 karakter)\r\n"); 
  } else if (komut == 'H')
  {
    Serialprint("Kullanimi:   H [Kanal_No #2] [isim #6] [Frekans #6] [Shift #5] [Ton #4] \r\n"); // !!!CHANGED!!!!!
    Serialprint("Ornek:       H 01 ROLE-0 145600 +0600 0885 \r\n");
  } else if (komut == 'T')
  {
    Serialprint("Kullanimi:   T [2 basamakli olarak Sure (dakika) (00-99)] (00 iptal demektir) \r\n");
    Serialprint("Ornek:       T 300\r\n");
  } else if (komut == 'M')
  {
    Serialprint("Kullanimi:   M [MESAJ (maksimum 30 karakter) \r\n");
    Serialprint("Ornek:       M Merhaba, Ben BARIS DINC - OH2UDS  \r\n");
  }
  F [Alt-frekans] [ust-frekans]        Frekans araligi 
  V [Alt-frekans] [ust-frekans]        VNA Frekans araligi
  S [0125-5000]                        VNA Step
  H                                    Memory Dump as Json

*/  
}

void commandRadioType(char komut)
{
  komut == 'V' ? radio_type = 0 : radio_type = 1;
  initialize_eeprom();
  //Serialprint("OK\r\n");
  Alert_Tone(OK_tone);
}

void commandStartupMSG()
{
  String StartupMSG = "      ";
  for (uint8_t cn=0;cn<6;cn++) StartupMSG[cn] = ((commandString[cn+2] >= 32) and (commandString[cn+2] <= 126)) ?  commandString[cn+2] : ' ';
  eewrite_nbytes(StartupMSG,6,9);
  Alert_Tone(OK_tone);
}

void commandDumpConfig()
{
  uint8_t Check = EEPROM.read(0);
  uint8_t SW_major = EEPROM.read(1);
  uint8_t SW_minor = EEPROM.read(2);
  char CallSign[6];
  CallSign[0] = EEPROM.read(3);
  CallSign[1] = EEPROM.read(4);
  CallSign[2] = EEPROM.read(5);
  CallSign[3] = EEPROM.read(6);
  CallSign[4] = EEPROM.read(7);
  CallSign[5] = EEPROM.read(8);
  CallSign[6] = 0;
  char Message[8];
  Message[0] = EEPROM.read(9);
  Message[1] = EEPROM.read(10);
  Message[2] = EEPROM.read(11);
  Message[3] = EEPROM.read(12);
  Message[4] = EEPROM.read(13);
  Message[5] = EEPROM.read(14);
  Message[6] = EEPROM.read(15);
  Message[7] = EEPROM.read(16);
  Message[8] = 0;
  uint8_t RadioType = EEPROM.read(17);
  Serialprint("\n\r{\"CD\":{");    //CD Configuration Dump
  Serialprint("\"cs\":%d,\"smj\":%d,\"smn\":%d,\"c\":\"%s\",\"m\":\"%s\",\"r\":%d,",Check,SW_major,SW_minor,CallSign,Message,RadioType);
  Serialprint("\"ful\":%d,\"fll\":%d,\"sul\":%d,\"sll\":%d,",freqLimits.trx_max_25,freqLimits.trx_min_25,freqLimits.scn_max_25,freqLimits.scn_min_25);
  Serialprint("\"AT\":%d,\"AM\":\"HEBELE\",\"AF\":%d,\"IF\":%d",APRS_Timeout,            freqLimits.aprs_25,freqLimits.iss_25);
  Serialprint("}}\n\r");
  Alert_Tone(SUCC_tone);
}

void commandMemoryDump()
{
  Serialprint("\r\nMD[");    //Memory Dump
  for (int ch=0;ch<100;ch++)
    {
      //channelInfo_t chInfo = 
      GetPrintMemoryChannelInfo(ch,true);
      Serialprint(",");
    }
    Serialprint("{}]\r\n");
}

void commandMemoryChannel()
{
  
    //H [Kanal_No #2] [isim #6] [Frekans #6] [Shift #5] [Ton #4]
    //Ornek:       H 01 ROLE-0 145600 +0600 0885

}

void commandAPRSTimeout()
{
  //Serialprint("OK\r\n");
  APRS_Timeout = commandString.substring(2,4).toInt();
  eeprom_writeAPRS();
  Alert_Tone(OK_tone);

}

void commandAPRSMessage()
{
  //Serial.print(commandString.substring(2,30));
  for (uint8_t cn=0;cn<28;cn++) APRS_Message[cn] = ((commandString[cn+2] >= 32) and (commandString[cn+2] <= 126)) ?  commandString[cn+2] : ' ';
  //Serial.print(APRS_Message);
  //Serialprint("OK\r\n");
  eeprom_writeAPRS();
  Alert_Tone(OK_tone);
}

void commandFrequencyLowerLimit()
{
  freqLimits.trx_min_25 = commandString.substring(2,5).toInt();
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits);
  Alert_Tone(OK_tone);
}

void commandFrequencyUpperLimit()
{
  freqLimits.trx_max_25 = commandString.substring(2,5).toInt();
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits);
  Alert_Tone(OK_tone);
}

void commandScanLowerLimit()
{
  freqLimits.scn_min_25 = commandString.substring(2,5).toInt();
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits);
  Alert_Tone(OK_tone);
}

void commandScanUpperLimit()
{
  freqLimits.scn_max_25 = commandString.substring(2,5).toInt();
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits);
  Alert_Tone(OK_tone);
}

void commandAprsFrequency()
{
  freqLimits.aprs_25 = commandString.substring(2,5).toInt();
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits);
  Alert_Tone(OK_tone);
}

void commandISSFrequency()
{
  freqLimits.iss_25 = commandString.substring(2,5).toInt();
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits);
  Alert_Tone(OK_tone);
}

void commandVNALowerLimit()
{
  freqLimits.vna_min_25 = commandString.substring(2,5).toInt();
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits);
  Alert_Tone(OK_tone);
}

void commandVNAUpperLimit()
{
  freqLimits.vna_max_25 = commandString.substring(2,5).toInt();
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits);
  Alert_Tone(OK_tone);
}

void getEEPROMData()
{
  int addr = commandString.substring(2,4).toInt();
  uint8_t eeprom_val;
  //Serialprint("currentch %d %d %d %d %d\r\n", current_ch.frequency,current_ch.shift, current_ch.shift_dir, current_ch.tone_pos, current_ch.tone_enabled);
  for (int tt=0;tt<10;tt++)
    {
    eeprom_val = EEPROM.read(addr+tt);
    Serialprint("DATA: %d = %d \r\n", addr+tt, eeprom_val);
    }
}

void commandAPRSmycall()
{
  //Serial.print(commandString.substring(2,8));
  mycall = commandString.substring(2,8);  
  eeprom_writeAPRS();
  //Serialprint(" OK\r\n");
  Alert_Tone(OK_tone);
}

void commandTogglePTT()
{
  pttToggler = !pttToggler; 
  if (pttToggler) Serialprint("TX\r\n");
  else Serialprint("RX\r\n");
}

