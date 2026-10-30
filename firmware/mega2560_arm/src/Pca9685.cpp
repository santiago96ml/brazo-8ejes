#include "Pca9685.h"
#include "Config.h"
#include <Wire.h>

static uint8_t lastCode = 0;

static void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

bool pcaProbe() {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(PCA_MODE1);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { lastCode = err; return false; }
  Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1);
  if (Wire.available()) { Wire.read(); lastCode = 0; return true; }
  lastCode = 4;
  return false;
}

void pcaInitDevice() {
  // sleep to change prescale
  uint8_t old = readReg(PCA_MODE1);
  writeReg(PCA_MODE1, (old & 0x7F) | 0x10);
  writeReg(PCA_PRESCALE, 121);
  writeReg(PCA_MODE1, old);
  delay(5);
  writeReg(PCA_MODE1, 0xA0); // auto-increment + restart
  // all channels off
  pcaSetAll(0);
}

void pcaSetPulse(uint8_t ch, uint16_t pulse) {
  if (ch >= 16) return;
  if (pulse > 4095) pulse = 4095;
  uint8_t base = PCA_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(base);
  Wire.write(0);          // ON_L
  Wire.write(0);          // ON_H
  Wire.write(pulse & 0xFF); // OFF_L
  Wire.write((pulse >> 8) & 0x0F); // OFF_H
  Wire.endTransmission();
}

void pcaSetAll(uint16_t pulse) {
  for (uint8_t i = 0; i < 16; i++) pcaSetPulse(i, pulse);
}

uint8_t pcaLastI2cCode() { return lastCode; }
