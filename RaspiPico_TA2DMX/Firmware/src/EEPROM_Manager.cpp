#include "main.h"

void eeread_nbytes(String *dst, uint8_t sz, uint16_t addr)
{
  String edata = "";  
  for (uint8_t cn=0;cn<sz;cn++) {
    char c = (char)EEPROM.read(addr+cn);
    if (c >= 32 && c <= 126) edata.concat(c);
    else edata.concat(' ');
  }
  *dst = edata;
}

void eeprom_readAPRS()
{
  //APRS Settings
  eeread_nbytes(&APRS_Message,28,20);
  eeread_nbytes(&mycall,6,60);
  APRS_Timeout = EEPROM.read(66);
  if (APRS_Timeout > 99) APRS_Timeout = 0;
  eeread_nbytes(&lat,8,67);
  eeread_nbytes(&lon,8,75);

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
  if (wp.length() >= 8) wifiPass = wp;
  else wifiPass = "deeplabstudio";
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
    //APRS Settings
    eewrite_nbytes(APRS_Message,28,20);
    eewrite_nbytes(mycall,6,3);
    eewrite_nbytes(mycall,6,60);
    EEPROM.write(66,APRS_Timeout); //Aprs timeout in minutes
    eewrite_nbytes(lat,8,67);
    eewrite_nbytes(lon,8,75);
    EEPROM.write(EEPROM_APRS_SSID_ADDR, (uint8_t)myssid);
    Alert_Tone(OK_tone);
}

void initialize_eeprom() {  
    EEPROM.write(0, 128); // make eeprom initialized (128 = OK, 129 = Force Reset)
    EEPROM.write(1, SW_MAJOR);   //SW Version
    EEPROM.write(2, SW_MINOR);   //
    eewrite_nbytes(APRS_DEFAULT_MYCALL,6,3);
    eewrite_nbytes(DEFAULT_STARTUP_MSG,6,9);
    EEPROM.write(17,radio_type); // Program device as VHF=0 or UHF=1
    EEPROM.write(EEPROM_OP_MODE_ADDR, MODE_VFO);
    EEPROM.write(EEPROM_CURR_MEMCH_ADDR, 1);
    APRS_Message = APRS_DEFAULT_MESSAGE;
    mycall = APRS_DEFAULT_MYCALL;  
    APRS_Timeout = 0;
    myssid = 9;
    lat = APRS_DEFAULT_LAT;
    lon = APRS_DEFAULT_LON;
    eeprom_writeAPRS();

    // Ilk kurulum (Hotspot DLS_ASELSAN_KIT) moduna sifirla
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
    //for (int location=18;location < 300;location++) EEPROM.write(location,0); // Zeroise the rest of the memory
    channel_t default_channel;
    if (radio_type == 0)
    { 
      default_channel.frequency = 145500; // Simplex 145.500 default
      default_channel.shift = 600;
      default_channel.shift_dir = 0; // noSHIFT
    }
    else
    {
      default_channel.shift = 7600;
      default_channel.frequency = 433500;
    }
    default_channel.shift_dir = noSHIFT;
    default_channel.tone_pos = 8;
    default_channel.tone_enabled = 0; // CTCSS_OFF Default!
    EEPROM.put(EEPROM_CURRCHNL_BLCKSTART, default_channel);
    freqLimits_t default_limits;
    EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,default_limits);
    freqLimits = default_limits;


    // TOT EEPROM INIT
     EEPROM.put(EEPROM_TOT, tot_values);


    memorych_t memch;
    memch.frequency25 = 65535; // 1638375 / 25
    memch.shift25      = 0;
    memch.tone_position= 0;
    memch.SSTP         = 0;
    memch.ChannelName[0]  = ' ';
    memch.ChannelName[1]  = ' ';
    memch.ChannelName[2]  = ' ';
    memch.ChannelName[3]  = ' ';
    memch.ChannelName[4]  = ' ';
    memch.ChannelName[5]  = ' ';
    memch.ChannelName[6]  = ' ';


    memset(memch.ChannelName, 0x00, 7);
    for (int ch=1; ch<=50; ch++)
        EEPROM.put(EEPROM_MEMDATA_BLCKSTART + ch * EEPROM_CHNNL_SIZE, memch);
    EEPROM.commit();
}

void StoreFrequency(char mCHNL[9], char mFRQ[9]) {
    uint8_t ChannelNumber = ((mCHNL[0] - 48) * 10) + (mCHNL[1] - 48);
    uint16_t ChannelLocation = EEPROM_MEMDATA_BLCKSTART + ChannelNumber * EEPROM_CHNNL_SIZE;
    Calculate_Frequency(mFRQ); 
    if (ChannelNumber > 90 ) { Alert_Tone(ERR_tone); return;}
    
    memorych_t memch;
    memch.frequency25 = (uint16_t)(current_ch.frequency / 25);
    memch.shift25 = (uint8_t)(current_ch.shift / 25);
    memch.tone_position= current_ch.tone_pos; 
    memch.SSTP =  (current_ch.shift_dir + 1)  + (current_ch.tone_enabled * 4);//TODO: Power is missing + ( power * 8);
    EEPROM.put(ChannelLocation, memch); 
}

void GetMemoryChannel(char mFRQ[9]) {
    uint8_t ChannelNumber = ((mFRQ[0] - 48) * 10) + (mFRQ[1] - 48);
    uint16_t ChannelLocation = EEPROM_MEMDATA_BLCKSTART + ChannelNumber * EEPROM_CHNNL_SIZE;
    if (ChannelNumber > 90) { Alert_Tone(ERR_tone); return;};
    memorych_t l_memorych;
    EEPROM.get(ChannelLocation, l_memorych);
    current_ch.frequency = l_memorych.frequency25 * 25;
    current_ch.shift     = l_memorych.shift25 * 25;
    current_ch.tone_pos  = l_memorych.tone_position;
    current_ch.shift_dir = ((l_memorych.SSTP) & 0x03) - 1; //First 2 bits -1
    current_ch.tone_enabled= ((l_memorych.SSTP) & 0x04) >> 2;  //3rd bit is tone_enabled
                                                          //TODO read power as well
    if (l_memorych.ChannelName[0] != 0xFF && l_memorych.ChannelName[0] != 0x00 && l_memorych.ChannelName[0] != ' ') {
        memset(FRQ, ' ', 8);
        FRQ[8] = 0;
        for (int i=0; i<7; i++) {
            if (l_memorych.ChannelName[i] == 0xFF || l_memorych.ChannelName[i] == 0x00) break;
            FRQ[i] = l_memorych.ChannelName[i];
        }
    } else {
        strcpy(FRQ, "NULL    ");
        FRQ[8] = 0;
    }
 }

void GetPrintMemoryChannelInfo(int8_t channel_number, boolean dbg) {
      memorych_t l_memorych;
      uint16_t ChannelLocation = EEPROM_MEMDATA_BLCKSTART + channel_number * EEPROM_CHNNL_SIZE;
      EEPROM.get(ChannelLocation, l_memorych);
      //numberToFrequency(freq, FRQ);
      if (dbg) 
      {
        Serialprint("{c:%d,f:%d,s:%d,t:%d}",channel_number,l_memorych.frequency25,l_memorych.shift25,l_memorych.tone_position); //TODO: add name
      }

}

void StoreSpecialFrequency(char mCHNL[9], char mFRQ[9])
{
    uint16_t ChannelNumber = (((mCHNL[0] - 48) * 100) + ((mCHNL[1] - 48) * 10) + (mCHNL[2] - 48));
    if (Calculate_Frequency(mFRQ))
    {
      if (ChannelNumber == 101) { freqLimits.trx_min_25 = current_ch.frequency/25; } //TRX Lower Limit
      if (ChannelNumber == 151) { freqLimits.trx_min_25 = DEFAULT_VHF_MINIMUM_FREQ; }
      if (ChannelNumber == 102) { freqLimits.trx_max_25 = current_ch.frequency/25; } //TRX Upper Limit
      if (ChannelNumber == 152) { freqLimits.trx_max_25 = DEFAULT_VHF_MAXIMUM_FREQ; }
      if (ChannelNumber == 201) { freqLimits.scn_min_25 = current_ch.frequency/25; } //Scan lower Limit
      if (ChannelNumber == 202) { freqLimits.scn_max_25 = current_ch.frequency/25; } //Scan Upper Limit
      if (ChannelNumber == 301) { freqLimits.aprs_25    = current_ch.frequency/25; } //APRS Frequency
      if (ChannelNumber == 302) { freqLimits.iss_25     = current_ch.frequency/25; } //ISS APRS Frequency
      if (ChannelNumber == 401) { freqLimits.vna_min_25 = current_ch.frequency/25; } //VNA minimum frequency
      if (ChannelNumber == 402) { freqLimits.vna_max_25 = current_ch.frequency/25; } //VNA maximum frequency
      if (ChannelNumber <= 402) { EEPROM.put(EEPROM_SPECIALFRQ_BLCKSTART,freqLimits); }
      if (ChannelNumber == 600) { APRS_Timeout =  current_ch.frequency % 1000; eeprom_writeAPRS(); } //APRS Timeout
      if (ChannelNumber == 666) { initialize_eeprom(); } //initialize eeprom
      if (ChannelNumber == 667) { softResetDevice(); } //rest/reboot device
      if (ChannelNumber == 998) { radio_type = 1 ; initialize_eeprom(); } //Initiralize device
      if (ChannelNumber == 999) { radio_type = 0 ; initialize_eeprom(); } //Initiralize device
      if (ChannelNumber >= 998) { softResetDevice(); }
      Alert_Tone(SUCC_tone);
    }  

}

