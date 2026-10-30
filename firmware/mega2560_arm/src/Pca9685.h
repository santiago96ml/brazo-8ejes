#pragma once
#include <Arduino.h>

bool pcaProbe();
void pcaInitDevice();
void pcaSetPulse(uint8_t ch, uint16_t pulse);
void pcaSetAll(uint16_t pulse);
uint8_t pcaLastI2cCode();
