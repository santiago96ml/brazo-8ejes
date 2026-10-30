#pragma once
#include "Config.h"

void calibInit();
void calibDefaults();
bool calibLoadEeprom();
void calibSaveEeprom();
ChannelCal& calibCh(uint8_t ch);
uint16_t calibCrc16(const uint8_t* data, size_t len);
