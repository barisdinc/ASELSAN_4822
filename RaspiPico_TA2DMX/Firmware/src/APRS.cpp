#include "main.h"

void set_nada_1200(void)
{
  digitalWrite(MIC_PIN, HIGH);
  delayMicroseconds(tc1200);
  digitalWrite(MIC_PIN, LOW);
  delayMicroseconds(tc1200);
}

void set_nada_2400(void)
{
  digitalWrite(MIC_PIN, HIGH);
  delayMicroseconds(tc2400);
  digitalWrite(MIC_PIN, LOW);
  delayMicroseconds(tc2400);
  
  digitalWrite(MIC_PIN, HIGH);
  delayMicroseconds(tc2400);
  digitalWrite(MIC_PIN, LOW);
  delayMicroseconds(tc2400);
}

void set_nada(bool nada)
{
  if(nada)
    set_nada_1200();
  else
    set_nada_2400();
}

void calc_crc(bool in_bit)
{
  unsigned short xor_in;
  
  xor_in = crc ^ in_bit;
  crc >>= 1;

  if(xor_in & 0x01)
    crc ^= 0x8408;
}

void send_crc(void)
{
  unsigned char crc_lo = crc ^ 0xff;
  unsigned char crc_hi = (crc >> 8) ^ 0xff;

  send_char_NRZI(crc_lo, HIGH);
  send_char_NRZI(crc_hi, HIGH);
}

void send_header(void)
{
  char temp;

  /*
   * APRS AX.25 Header 
   * ........................................................
   * |   DEST   |  SOURCE  |   DIGI   | CTRL FLD |    PID   |
   * --------------------------------------------------------
   * |  7 bytes |  7 bytes |  7 bytes |   0x03   |   0xf0   |
   * --------------------------------------------------------
   * 
   * DEST   : 6 byte "callsign" + 1 byte ssid
   * SOURCE : 6 byte your callsign + 1 byte ssid
   * DIGI   : 6 byte "digi callsign" + 1 byte ssid
   * 
   * ALL DEST, SOURCE, & DIGI are left shifted 1 bit, ASCII format.
   * DIGI ssid is left shifted 1 bit + 1
   * 
   * CTRL FLD is 0x03 and not shifted.
   * PID is 0xf0 and not shifted.
   */

  /********* DEST ***********/
  temp = strlen(dest);
  for(int j=0; j<temp; j++)
    send_char_NRZI(dest[j] << 1, HIGH);
  if(temp < 6)
  {
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, HIGH);
  }
  send_char_NRZI('0' << 1, HIGH);

  
  /********* SOURCE *********/
  temp = mycall.length();
  for(int j=0; j<temp; j++)
    send_char_NRZI(mycall[j] << 1, HIGH);
  if(temp < 6)
  {
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, HIGH);
  }
  send_char_NRZI((myssid + '0') << 1, HIGH);

  
  /********* DIGI ***********/
  temp = strlen(digi);
  for(int j=0; j<temp; j++)
    send_char_NRZI(digi[j] << 1, HIGH);
  if(temp < 6)
  {
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, HIGH);
  }
  send_char_NRZI(((digissid + '0') << 1) + 1, HIGH);

  /***** CTRL FLD & PID *****/
  send_char_NRZI(_CTRL_ID, HIGH);
  send_char_NRZI(_PID, HIGH);
}

void send_payload(char type)
{
  /*
   * APRS AX.25 Payloads
   * 
   * TYPE : POSITION
   * ........................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|
   * --------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |
   * --------------------------------------------------------
   * 
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * 
   * 
   * TYPE : STATUS
   * ..................................
   * |DATA TYPE |    STATUS TEXT      |
   * ----------------------------------
   * |  1 byte  |       N bytes       |
   * ----------------------------------
   * 
   * DATA TYPE  : >
   * STATUS TEXT: Free form text
   * 
   * 
   * TYPE : POSITION & STATUS
   * ..............................................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|    STATUS TEXT      |
   * ------------------------------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |       N bytes       |
   * ------------------------------------------------------------------------------
   * 
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * STATUS TEXT: Free form text
   * 
   * 
   * All of the data are sent in the form of ASCII Text, not shifted.
   * 
   */
  if(type == _FIXPOS)
  {
    send_char_NRZI(_DT_POS, HIGH);
    send_string_len(lat, lat.length());
    send_char_NRZI(sym_ovl, HIGH);
    send_string_len(lon, lon.length());
    send_char_NRZI(sym_tab, HIGH);
  }
  else if(type == _STATUS)
  {
    send_char_NRZI(_DT_STATUS, HIGH);
    send_string_len(APRS_Message, APRS_Message.length());
  }
  else if(type == _FIXPOS_STATUS)
  {
    send_char_NRZI(_DT_POS, HIGH);
    send_string_len(lat, lat.length());
    send_char_NRZI(sym_ovl, HIGH);
    send_string_len(lon, lon.length());
    send_char_NRZI(sym_tab, HIGH);

    send_char_NRZI(' ', HIGH);
    send_string_len(APRS_Message, APRS_Message.length());
  }
}

void send_char_NRZI(unsigned char in_byte, bool enBitStuff)
{
  bool bits;
  
  for(int i = 0; i < 8; i++)
  {
    bits = in_byte & 0x01;

    calc_crc(bits);

    if(bits)
    {
      set_nada(nada);
      bit_stuff++;

      if((enBitStuff) && (bit_stuff == 5))
      {
        nada ^= 1;
        set_nada(nada);
        
        bit_stuff = 0;
      }
    }
    else
    {
      nada ^= 1;
      set_nada(nada);

      bit_stuff = 0;
    }

    in_byte >>= 1;
  }
}

void send_string_len(String in_string, int len)
{
  for(int j=0; j<len; j++)
    send_char_NRZI(in_string[j], HIGH);
}

void send_flag(unsigned char flag_len)
{
  for(int j=0; j<flag_len; j++)
    send_char_NRZI(_FLAG, LOW); 
}

void send_packet(char packet_type, uint32_t frequency)
{
  /*
   * AX25 FRAME
   * 
   * ........................................................
   * |  FLAG(s) |  HEADER  | PAYLOAD  | FCS(CRC) |  FLAG(s) |
   * --------------------------------------------------------
   * |  N bytes | 22 bytes |  N bytes | 2 bytes  |  N bytes |
   * --------------------------------------------------------
   * 
   * FLAG(s)  : 0x7e
   * HEADER   : see header
   * PAYLOAD  : 1 byte data type + N byte info
   * FCS      : 2 bytes calculated from HEADER + PAYLOAD
   */

   strcpy(FRQ_old,FRQ); //store old frequency for recall
   current_ch.shift_dir = noSHIFT; //get into SIMPLEX mode for caculations
   TRX_MODE = TX;
   numberToFrequency(frequency,FRQ);         //TODO: change for ISS
   validFRQ = Calculate_Frequency(FRQ);
   write_FRQ(current_ch.frequency);
   writeFRQToLcd(FRQ);
   delay(PLL_TX_SETTLE_MS);          // VCO kilitlenmeden PA acilmasin
   pttKey(true);
   //Serialprint("Sending packet..."); 
   noTone(MIC_PIN);
   delay(300); //TODO: put define for TXDELAY
    
  //prepare and send APRS message
  send_flag(100);
  crc = 0xffff;
  send_header();
  send_payload(packet_type);
  send_crc();
  send_flag(3);

  //APRS message transmission ended
  pttKey(false); // now stop transmitting
  //Restoring OLD values
  TRX_MODE = RX;
  strcpy(FRQ,FRQ_old);
  //numberToFrequency((highestFRQ+lowestFRQ)/2,FRQ);
  validFRQ = Calculate_Frequency(FRQ);
  write_FRQ(current_ch.frequency);              
}

