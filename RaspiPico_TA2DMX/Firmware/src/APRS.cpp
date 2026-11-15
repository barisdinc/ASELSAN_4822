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
  if (nada) set_nada_1200();
  else set_nada_2400();
}

void calc_crc(bool in_bit)
{
  unsigned short xor_in = crc ^ in_bit;
  crc >>= 1;
  if (xor_in & 0x01) crc ^= 0x8408;
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
  char temp = strlen(dest);
  for (int j = 0; j < temp; j++) send_char_NRZI(dest[j] << 1, HIGH);
  for (int j = temp; j < 6; j++) send_char_NRZI(' ' << 1, HIGH);
  send_char_NRZI('0' << 1, HIGH);

  String cleanCall = mycall;
  cleanCall.trim();
  temp = cleanCall.length();
  if (temp > 6) temp = 6;
  for (int j = 0; j < temp; j++) send_char_NRZI(cleanCall[j] << 1, HIGH);
  for (int j = temp; j < 6; j++) send_char_NRZI(' ' << 1, HIGH);
  send_char_NRZI(((myssid & 0x0F) + '0') << 1, HIGH);

  temp = strlen(digi);
  for (int j = 0; j < temp; j++) send_char_NRZI(digi[j] << 1, HIGH);
  for (int j = temp; j < 6; j++) send_char_NRZI(' ' << 1, HIGH);
  send_char_NRZI(((digissid + '0') << 1) + 1, HIGH);

  send_char_NRZI(_CTRL_ID, HIGH);
  send_char_NRZI(_PID, HIGH);
}

void send_payload(char type)
{
  if (type == _FIXPOS) {
    send_char_NRZI(_DT_POS, HIGH);
    send_string_len(lat, lat.length());
    send_char_NRZI(sym_ovl, HIGH);
    send_string_len(lon, lon.length());
    send_char_NRZI(sym_tab, HIGH);
  } else if (type == _STATUS) {
    send_char_NRZI(_DT_STATUS, HIGH);
    send_string_len(APRS_Message, APRS_Message.length());
  } else if (type == _FIXPOS_STATUS) {
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
  for (int i = 0; i < 8; i++) {
    bool bits = in_byte & 0x01;
    calc_crc(bits);

    if (bits) {
      set_nada(nada);
      bit_stuff++;
      if (enBitStuff && (bit_stuff == 5)) {
        nada ^= 1;
        set_nada(nada);
        bit_stuff = 0;
      }
    } else {
      nada ^= 1;
      set_nada(nada);
      bit_stuff = 0;
    }
    in_byte >>= 1;
  }
}

void send_string_len(String in_string, int len)
{
  for (int j = 0; j < len; j++) send_char_NRZI(in_string[j], HIGH);
}

void send_flag(unsigned char flag_len)
{
  for (int j = 0; j < flag_len; j++) send_char_NRZI(_FLAG, LOW);
}

void send_packet(char packet_type, uint32_t frequency)
{
  strcpy(FRQ_old, FRQ);
  int8_t saved_shift_dir = current_ch.shift_dir;
  uint32_t saved_freq = current_ch.frequency;
  current_ch.shift_dir = noSHIFT;
  TRX_MODE = TX;
  numberToFrequency(frequency, FRQ);
  validFRQ = Calculate_Frequency(FRQ);
  write_FRQ(current_ch.frequency);
  writeFRQToLcd(FRQ);
  delay(PLL_TX_SETTLE_MS);
  pttKey(true);
  noTone(MIC_PIN);
  pinMode(MIC_PIN, OUTPUT);
  delay(300);

  send_flag(100);
  crc = 0xffff;
  send_header();
  send_payload(packet_type);
  send_crc();
  send_flag(3);

  pttKey(false);
  TRX_MODE = RX;
  LST_MODE = RX;
  current_ch.shift_dir = saved_shift_dir;
  current_ch.frequency = saved_freq;
  strcpy(FRQ, FRQ_old);
  validFRQ = Calculate_Frequency(FRQ);
  write_FRQ(current_ch.frequency);
  writeFRQToLcd(FRQ);
  SetTone(current_ch.tone_enabled);
}
