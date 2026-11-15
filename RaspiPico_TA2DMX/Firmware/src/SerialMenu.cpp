#include "main.h"

void PrintMenu()
{
  Serialprint("\r\nASELSAN 48xx - TAMSAT Kit (v%d.%d)\r\n", SW_MAJOR, SW_MINOR);
  Serialprint("---------------------------------\r\n");
  Serialprint("C [V/U]   - VHF / UHF Mod Secimi\r\n");
  Serialprint("A [MESAJ] - Acilis Mesaji (Maks 6 krk)\r\n");
  Serialprint("S [CAGRI] - Cagri Isareti (Maks 6 krk)\r\n");
  Serialprint("M [MESAJ] - APRS Durum Mesaji\r\n");
  Serialprint("T [00-99] - APRS Gonderim Suresi (dk)\r\n");
  Serialprint("K         - Konfigurasyon Ozeti (JSON)\r\n");
  Serialprint("H         - Hafiza Kanallari Dokumu\r\n");
  Serialprint("X         - Cihazi Yeniden Baslat\r\n");
  Serialprint("Seciminiz > ");
}

void commandRadioType(char komut)
{
  radio_type = (komut == 'V' || komut == 'v') ? 0 : 1;
  initialize_eeprom();
  Alert_Tone(OK_tone);
}

void commandStartupMSG()
{
  String StartupMSG = "      ";
  for (uint8_t cn = 0; cn < 6; cn++) {
    StartupMSG[cn] = ((commandString[cn + 2] >= 32) && (commandString[cn + 2] <= 126)) ? commandString[cn + 2] : ' ';
  }
  eewrite_nbytes(StartupMSG, 6, 9);
  EEPROM.commit();
  Alert_Tone(OK_tone);
}

void commandDumpConfig()
{
  uint8_t Check = EEPROM.read(0);
  uint8_t SW_major = EEPROM.read(1);
  uint8_t SW_minor = EEPROM.read(2);
  char CallSign[7];
  for (int i = 0; i < 6; i++) CallSign[i] = EEPROM.read(3 + i);
  CallSign[6] = 0;

  char Message[9];
  for (int i = 0; i < 8; i++) Message[i] = EEPROM.read(9 + i);
  Message[8] = 0;

  uint8_t RadioType = EEPROM.read(17);
  Serialprint("\r\n{\"CD\":{");
  Serialprint("\"cs\":%d,\"smj\":%d,\"smn\":%d,\"c\":\"%s\",\"m\":\"%s\",\"r\":%d,",
              Check, SW_major, SW_minor, CallSign, Message, RadioType);
  Serialprint("\"ful\":%d,\"fll\":%d,\"sul\":%d,\"sll\":%d,",
              freqLimits.trx_max_25, freqLimits.trx_min_25, freqLimits.scn_max_25, freqLimits.scn_min_25);
  Serialprint("\"AT\":%d,\"AF\":%d,\"IF\":%d",
              APRS_Timeout, freqLimits.aprs_25, freqLimits.iss_25);
  Serialprint("}}\r\n");
  Alert_Tone(SUCC_tone);
}

void commandMemoryDump()
{
  Serialprint("\r\nMD[");
  for (int ch = 1; ch <= 50; ch++) {
    GetPrintMemoryChannelInfo(ch, true);
    if (ch < 50) Serialprint(",");
  }
  Serialprint("]\r\n");
}

void commandAPRSTimeout()
{
  APRS_Timeout = commandString.substring(2, 4).toInt();
  eeprom_writeAPRS();
  EEPROM.commit();
  Alert_Tone(OK_tone);
}

void commandAPRSMessage()
{
  for (uint8_t cn = 0; cn < 28; cn++) {
    APRS_Message[cn] = ((commandString[cn + 2] >= 32) && (commandString[cn + 2] <= 126)) ? commandString[cn + 2] : ' ';
  }
  eeprom_writeAPRS();
  EEPROM.commit();
  Alert_Tone(OK_tone);
}

void commandAPRSmycall()
{
  mycall = commandString.substring(2, 8);
  eeprom_writeAPRS();
  EEPROM.commit();
  Alert_Tone(OK_tone);
}

void commandTogglePTT()
{
  pttToggler = !pttToggler;
  Serialprint(pttToggler ? "TX\r\n" : "RX\r\n");
}
