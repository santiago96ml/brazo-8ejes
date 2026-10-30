#pragma once
#include "Config.h"

void motionInit();
void motionTick();
void motionEstop();
void motionEnable();
void motionAllNeutral();
bool motionIsEstop();
bool motionIsEnabled();

void motionMovePos(uint8_t ch, float deg, unsigned long durationMs);
void motionMoveRaw(uint8_t ch, uint16_t pulse);
void motionTimed(uint8_t ch, int16_t offset, unsigned long ms);
void motionCalibrateStop(uint8_t ch, uint16_t pulse);
void motionSetRange(uint8_t ch, uint16_t mn, uint16_t mx);
void motionSetMode(uint8_t ch, uint8_t mode);
void motionSetLimits(uint8_t ch, int16_t mn, int16_t mx);
void motionSetMirror(uint8_t ch, uint8_t src);
void motionToggleInvert(uint8_t ch);
void motionCoordinated(uint8_t* chs, float* degs, uint8_t n);
void motionHomeAll();
void motionSetHome(uint8_t ch, uint16_t deg);
void motionRestoreDefaults();
void motionWatchdog(uint32_t ms);
void motionKeepalive();

float motionCurrentDeg(uint8_t ch);
uint16_t motionCurrentPulse(uint8_t ch);
float motionTargetDeg(uint8_t ch);
uint8_t motionMoving(uint8_t ch);
unsigned long motionMillisNow();
void motionSetWatchdogBroadcast(bool* flag); // noop placeholder
