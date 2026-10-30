#include "Calibration.h"
#include "Config.h"
#include <EEPROM.h>

static ChannelCal cals[NUM_CH];

uint16_t calibCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void calibDefaults() {
  for (uint8_t i = 0; i < NUM_CH; i++) {
    cals[i].mode = MODE_POS;
    cals[i].stopPulse = PULSE_STOP_DEFAULT;
    cals[i].minPulse = PULSE_MIN_DEFAULT;
    cals[i].maxPulse = PULSE_MAX_DEFAULT;
    cals[i].minDeg = 0;
    cals[i].maxDeg = 180;
    cals[i].homeDeg = 90;
    cals[i].invert = 0;
    cals[i].mirrorSrc = 255;
    cals[i].speedDps = 60.0f;
  }
  cals[CH_HOMBRO_DER].mirrorSrc = CH_HOMBRO_IZQ;
  cals[CH_HOMBRO_DER].invert = 1;
}

bool calibLoadEeprom() {
  EepromData d;
  EEPROM.get(0, d);
  if (d.magic[0]!='A' || d.magic[1]!='R' || d.magic[2]!='M' || d.magic[3]!='1') return false;
  if (d.version != 2) return false;
  // crc over header+channels (excluding crc field)
  size_t len = sizeof(EepromData) - sizeof(uint16_t);
  uint16_t calc = calibCrc16((const uint8_t*)&d, len);
  if (calc != d.crc) return false;
  for (uint8_t i = 0; i < NUM_CH; i++) cals[i] = d.ch[i];
  return true;
}

void calibSaveEeprom() {
  EepromData d;
  d.magic[0]='A'; d.magic[1]='R'; d.magic[2]='M'; d.magic[3]='1';
  d.version = 2;
  for (uint8_t i = 0; i < NUM_CH; i++) d.ch[i] = cals[i];
  size_t len = sizeof(EepromData) - sizeof(uint16_t);
  d.crc = calibCrc16((const uint8_t*)&d, len);
  EEPROM.put(0, d);
}

void calibInit() {
  calibDefaults();
  calibLoadEeprom(); // ignore failure, keep defaults
}

ChannelCal& calibCh(uint8_t ch) { return cals[ch]; }
