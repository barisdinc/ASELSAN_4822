#include "main.h"

void eeread_nbytes(String *dst, uint8_t sz, uint16_t addr)
{
  String edata = "";
  for (uint8_t cn = 0; cn < sz; cn++) {
    char c = (char)EEPROM.read(addr + cn);
    if (c >= 32 && c <= 126) edata.concat(c);
    else edata.concat(' ');
  }
  *dst = edata;
}

void eeprom_readAPRS()
{
  eeread_nbytes(&APRS_Message, 28, 20);
  eeread_nbytes(&mycall, 6, 60);
  APRS_Timeout = EEPROM.read(66);
  if (APRS_Timeout > 99) APRS_Timeout = 0;
  eeread_nbytes(&lat, 8, 67);
  eeread_nbytes(&lon, 8, 75);

  uint8_t ssid_val = EEPROM.read(EEPROM_APRS_SSID_ADDR);
  myssid = (ssid_val <= 15) ? (char)ssid_val : 9;

  isSetupDone = (EEPROM.read(EEPROM_SETUP_DONE_ADDR) == SETUP_MAGIC_DONE);

  uint8_t pwr = EEPROM.read(EEPROM_RFPOWER_ADDR);
  if (pwr <= 1) {
    RF_POWER_STATE = pwr;
    SetRFPower();
  }
  uint8_t sql = EEPROM.read(EEPROM_SQLMODE_ADDR);
  if (sql <= 1) {
    SQL_MODE = sql;
  }
  uint8_t slev = EEPROM.read(EEPROM_SQLLEVEL_ADDR);
  sql_level = (slev >= 1 && slev <= 5) ? slev : 3;

  String wp = "";
  for (uint8_t i = 0; i < WIFI_PASS_MAX_LEN; i++) {
    uint8_t b = EEPROM.read(EEPROM_WIFI_PASS_ADDR + i);
    if (b == 0 || b == 0xFF) break;
    if (b >= 32 && b <= 126) wp += (char)b;
  }
  wifiPass = (wp.length() >= 8) ? wp : "deeplabstudio";

  if (EEPROM.read(EEPROM_NET_MAGIC_ADDR) != NET_MAGIC_VALID) {
    EEPROM.write(EEPROM_NET_MAGIC_ADDR, NET_MAGIC_VALID);
    EEPROM.write(EEPROM_WIFI_MODE_ADDR, 0);
    for (int i = 0; i < 33; i++) {
      EEPROM.write(EEPROM_STA_SSID_ADDR + i, 0);
      EEPROM.write(EEPROM_STA_PASS_ADDR + i, 0);
    }
    for (int i = 0; i < 17; i++) {
      EEPROM.write(EEPROM_STA_IP_ADDR + i, 0);
    }
    const char* defPeer = "192.168.4.2";
    for (int i = 0; i < 17; i++) {
      EEPROM.write(EEPROM_PEER_IP_ADDR + i, (i < 11) ? (uint8_t)defPeer[i] : 0);
    }
    EEPROM.commit();
  }

  uint8_t wm = EEPROM.read(EEPROM_WIFI_MODE_ADDR);
  wifiMode = (wm <= 2) ? wm : 0;

  staSsid = "";
  for (uint8_t i = 0; i < 32; i++) {
    uint8_t b = EEPROM.read(EEPROM_STA_SSID_ADDR + i);
    if (b == 0 || b == 0xFF) break;
    if (b >= 32 && b <= 126) staSsid += (char)b;
  }
  staPass = "";
  for (uint8_t i = 0; i < 32; i++) {
    uint8_t b = EEPROM.read(EEPROM_STA_PASS_ADDR + i);
    if (b == 0 || b == 0xFF) break;
    if (b >= 32 && b <= 126) staPass += (char)b;
  }
  staStaticIp = "";
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t b = EEPROM.read(EEPROM_STA_IP_ADDR + i);
    if (b == 0 || b == 0xFF) break;
    if (b >= 32 && b <= 126) staStaticIp += (char)b;
  }
  peerIp = "";
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t b = EEPROM.read(EEPROM_PEER_IP_ADDR + i);
    if (b == 0 || b == 0xFF) break;
    if (b >= 32 && b <= 126) peerIp += (char)b;
  }
  if (peerIp.length() == 0) peerIp = "192.168.4.2";
}

void eewrite_nbytes(String dst, uint8_t sz, uint16_t addr)
{
  for (uint8_t cn = 0; cn < sz; cn++) {
    char c = (cn < dst.length()) ? dst[cn] : ' ';
    if (c < 32 || c > 126) c = ' ';
    EEPROM.write(addr + cn, (uint8_t)c);
  }
}

void eeprom_writeAPRS()
{
  eewrite_nbytes(APRS_Message, 28, 20);
  eewrite_nbytes(mycall, 6, 3);
  eewrite_nbytes(mycall, 6, 60);
  EEPROM.write(66, APRS_Timeout);
  eewrite_nbytes(lat, 8, 67);
  eewrite_nbytes(lon, 8, 75);
  EEPROM.write(EEPROM_APRS_SSID_ADDR, (uint8_t)myssid);
}

void initialize_eeprom()
{
  if (radio_type > 1) radio_type = 0;
  EEPROM.write(0, 128);
  EEPROM.write(1, SW_MAJOR);
  EEPROM.write(2, SW_MINOR);
  EEPROM.write(EEPROM_LAYOUT_VER_ADDR, EEPROM_LAYOUT_VERSION);
  eewrite_nbytes(APRS_DEFAULT_MYCALL, 6, 3);
  eewrite_nbytes(DEFAULT_STARTUP_MSG, 6, 9);
  EEPROM.write(17, radio_type);
  EEPROM.write(EEPROM_OP_MODE_ADDR, MODE_VFO);
  EEPROM.write(EEPROM_CURR_MEMCH_ADDR, 1);

  APRS_Message = APRS_DEFAULT_MESSAGE;
  mycall = APRS_DEFAULT_MYCALL;
  APRS_Timeout = 0;
  myssid = 9;
  lat = APRS_DEFAULT_LAT;
  lon = APRS_DEFAULT_LON;
  eeprom_writeAPRS();

  isSetupDone = false;
  EEPROM.write(EEPROM_SETUP_DONE_ADDR, 0x00);
  RF_POWER_STATE = HIGH_POWER;
  EEPROM.write(EEPROM_RFPOWER_ADDR, RF_POWER_STATE);
  SQL_MODE = SQL_ON;
  EEPROM.write(EEPROM_SQLMODE_ADDR, SQL_MODE);
  sql_level = 3;
  EEPROM.write(EEPROM_SQLLEVEL_ADDR, sql_level);
  wifiPass = "deeplabstudio";
  for (uint8_t i = 0; i <= WIFI_PASS_MAX_LEN; i++) {
    uint8_t c = (i < wifiPass.length()) ? (uint8_t)wifiPass[i] : 0;
    EEPROM.write(EEPROM_WIFI_PASS_ADDR + i, c);
  }
  EEPROM.write(EEPROM_NET_MAGIC_ADDR, NET_MAGIC_VALID);
  wifiMode = 0;
  EEPROM.write(EEPROM_WIFI_MODE_ADDR, 0);
  staSsid = "";
  staPass = "";
  staStaticIp = "";
  peerIp = "192.168.4.2";
  for (int i = 0; i < 33; i++) {
    EEPROM.write(EEPROM_STA_SSID_ADDR + i, 0);
    EEPROM.write(EEPROM_STA_PASS_ADDR + i, 0);
  }
  for (int i = 0; i < 17; i++) {
    EEPROM.write(EEPROM_STA_IP_ADDR + i, 0);
    EEPROM.write(EEPROM_PEER_IP_ADDR + i, (i < (int)peerIp.length()) ? (uint8_t)peerIp[i] : 0);
  }

  channel_t default_channel;
  if (radio_type == 0) {
    default_channel.frequency = 145575;
    default_channel.shift = 600;
  } else {
    default_channel.frequency = 433500;
    default_channel.shift = 7600;
  }
  default_channel.shift_dir = noSHIFT;
  default_channel.tone_pos = 8;
  default_channel.tone_enabled = 0;
  EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, default_channel);
  current_ch = default_channel;

  freqLimits_t default_limits;
  EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART, default_limits);
  freqLimits = default_limits;

  tot_values.time_out_time = 60;
  tot_values.lock_time = 5;
  EEPROM.put(EEPROM_TOT, tot_values);

  memorych_t memch;
  memch.frequency25 = 65535;
  memch.shift25 = 0;
  memch.tone_position = 0;
  memch.SSTP = 0;
  memset(memch.ChannelName, 0x00, 7);
  for (int ch = 1; ch <= 50; ch++) {
    EEPROM.put(EEPROM_MEMDATA_BLCKSTART + ch * EEPROM_CHNNL_SIZE, memch);
  }
  EEPROM.commit();
}

void StoreFrequency(char mCHNL[9], char mFRQ[9])
{
  uint8_t ChannelNumber = ((mCHNL[0] - 48) * 10) + (mCHNL[1] - 48);
  uint16_t ChannelLocation = EEPROM_MEMDATA_BLCKSTART + ChannelNumber * EEPROM_CHNNL_SIZE;
  Calculate_Frequency(mFRQ);
  if (ChannelNumber > 90) { Alert_Tone(ERR_tone); return; }

  memorych_t memch;
  memch.frequency25 = (uint16_t)(current_ch.frequency / 25);
  memch.shift25 = (uint8_t)(current_ch.shift / 25);
  memch.tone_position = current_ch.tone_pos;
  memch.SSTP = (current_ch.shift_dir + 1) + (current_ch.tone_enabled * 4);
  EEPROM.put(ChannelLocation, memch);
  EEPROM.commit();
}

void GetMemoryChannel(char mFRQ[9])
{
  uint8_t ChannelNumber = ((mFRQ[0] - 48) * 10) + (mFRQ[1] - 48);
  uint16_t ChannelLocation = EEPROM_MEMDATA_BLCKSTART + ChannelNumber * EEPROM_CHNNL_SIZE;
  if (ChannelNumber > 90) { Alert_Tone(ERR_tone); return; }

  memorych_t l_memorych;
  EEPROM.get(ChannelLocation, l_memorych);
  current_ch.frequency = l_memorych.frequency25 * 25;
  current_ch.shift = l_memorych.shift25 * 25;
  current_ch.tone_pos = l_memorych.tone_position;
  current_ch.shift_dir = ((l_memorych.SSTP) & 0x03) - 1;
  current_ch.tone_enabled = ((l_memorych.SSTP) & 0x04) >> 2;

  if (l_memorych.ChannelName[0] != 0xFF && l_memorych.ChannelName[0] != 0x00 && l_memorych.ChannelName[0] != ' ') {
    memset(FRQ, ' ', 8);
    FRQ[8] = 0;
    for (int i = 0; i < 7; i++) {
      if (l_memorych.ChannelName[i] == 0xFF || l_memorych.ChannelName[i] == 0x00) break;
      FRQ[i] = l_memorych.ChannelName[i];
    }
  } else {
    strcpy(FRQ, "NULL    ");
    FRQ[8] = 0;
  }
}

void GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg)
{
  memorych_t l_memorych;
  uint16_t ChannelLocation = EEPROM_MEMDATA_BLCKSTART + channel_number * EEPROM_CHNNL_SIZE;
  EEPROM.get(ChannelLocation, l_memorych);
  if (dbg) {
    Serialprint("{c:%d,f:%d,s:%d,t:%d}", channel_number, l_memorych.frequency25, l_memorych.shift25, l_memorych.tone_position);
  }
}

void StoreSpecialFrequency(char mCHNL[9], char mFRQ[9])
{
  uint16_t ChannelNumber = (((mCHNL[0] - 48) * 100) + ((mCHNL[1] - 48) * 10) + (mCHNL[2] - 48));
  if (ChannelNumber == 666) { initialize_eeprom(); Alert_Tone(SUCC_tone); return; }
  if (ChannelNumber == 667) { softResetDevice(); return; }
  if (ChannelNumber == 998) { radio_type = 1; initialize_eeprom(); softResetDevice(); return; }
  if (ChannelNumber == 999) { radio_type = 0; initialize_eeprom(); softResetDevice(); return; }

  if (Calculate_Frequency(mFRQ)) {
    if (ChannelNumber == 101) freqLimits.trx_min_25 = current_ch.frequency / 25;
    if (ChannelNumber == 151) freqLimits.trx_min_25 = DEFAULT_VHF_MINIMUM_FREQ;
    if (ChannelNumber == 102) freqLimits.trx_max_25 = current_ch.frequency / 25;
    if (ChannelNumber == 152) freqLimits.trx_max_25 = DEFAULT_VHF_MAXIMUM_FREQ;
    if (ChannelNumber == 201) freqLimits.scn_min_25 = current_ch.frequency / 25;
    if (ChannelNumber == 202) freqLimits.scn_max_25 = current_ch.frequency / 25;
    if (ChannelNumber == 301) freqLimits.aprs_25    = current_ch.frequency / 25;
    if (ChannelNumber == 302) freqLimits.iss_25     = current_ch.frequency / 25;
    if (ChannelNumber == 401) freqLimits.vna_min_25 = current_ch.frequency / 25;
    if (ChannelNumber == 402) freqLimits.vna_max_25 = current_ch.frequency / 25;
    if (ChannelNumber <= 402) { EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART, freqLimits); EEPROM.commit(); }
    if (ChannelNumber == 600) { APRS_Timeout = current_ch.frequency % 1000; eeprom_writeAPRS(); EEPROM.commit(); }
    Alert_Tone(SUCC_tone);
  }
}
