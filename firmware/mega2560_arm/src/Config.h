#pragma once
#include <Arduino.h>

#define NUM_CH 8
#define PCA_ADDR 0x40
#define PIN_OE 2

#define PCA_MODE1 0x00
#define PCA_PRESCALE 0xFE
#define PCA_LED0_ON_L 0x06

#define PULSE_STOP_DEFAULT 307
#define PULSE_MIN_DEFAULT 110
#define PULSE_MAX_DEFAULT 500

#define CH_PINZA 0
#define CH_GIRO_PINZA 1
#define CH_INCLINA_PINZA 2
#define CH_CODO 3
#define CH_HOMBRO_IZQ 4
#define CH_HOMBRO_DER 5
#define CH_BASE 6
#define CH_AUX 7

#define EEPROM_VERSION 2
#define MODE_POS 0
#define MODE_CONT 1

#pragma pack(push,1)
struct ChannelCal {
  uint8_t mode;
  uint16_t stopPulse;
  uint16_t minPulse;
  uint16_t maxPulse;
  int16_t minDeg;
  int16_t maxDeg;
  uint16_t homeDeg;
  uint8_t invert;
  uint8_t mirrorSrc; // 255 = none
  float speedDps;
};
struct EepromData {
  char magic[4];
  uint8_t version;
  ChannelCal ch[NUM_CH];
  uint16_t crc;
};
#pragma pack(pop)

// Motion axis runtime state
struct AxisState {
  float currentDeg;
  float targetDeg;
  uint16_t currentPulse;
  uint8_t moving;
  // positional move
  float moveStartDeg;
  float moveTargetDeg;
  unsigned long moveStartMs;
  unsigned long moveDurationMs;
  float moveSpeedDps;
  // timed continuous
  unsigned long timedEndMs;
  uint16_t timedRestorePulse;
  uint8_t timedActive;
};
