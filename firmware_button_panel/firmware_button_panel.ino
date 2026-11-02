#include <Wire.h>
#include <EEPROM.h>

// Independent firmware for the physical 12-button panel. Do not merge with
// firmware_mega, whose serial protocol serves the kinematics controller.
#define PCA_ADDR 0x40
#define PCA_MODE1 0x00
#define PCA_MODE2 0x01
#define PCA_PRESCALE 0xFE
#define PCA_LED0_ON_L 0x06
#define PIN_OE 2

static const uint8_t AXIS_COUNT = 6;
static const uint8_t CHANNEL_COUNT = 7;
static const uint8_t buttonPins[AXIS_COUNT][2] = {
  {22, 23}, {24, 25}, {26, 27}, {28, 29}, {30, 31}, {32, 33}
};
static const uint8_t axisChannels[AXIS_COUNT][2] = {
  {0, 0}, {2, 2}, {6, 6}, {4, 5}, {3, 3}, {1, 1}
};
static const int8_t axisOffsets[AXIS_COUNT][2] = {
  {18, -18}, {22, -14}, {20, -20}, {28, -15}, {24, -14}, {18, -18}
};
static const char *const axisMessages[AXIS_COUNT][2] = {
  {"ABRIENDO PINZA", "CERRANDO PINZA"},
  {"SUBIENDO MUNECA", "BAJANDO MUNECA"},
  {"GIRANDO IZQUIERDA", "GIRANDO DERECHA"},
  {"SUBIENDO HOMBRO", "BAJANDO HOMBRO"},
  {"SUBIENDO CODO", "BAJANDO CODO"},
  {"GIRANDO ROTOR IZQUIERDA", "GIRANDO ROTOR DERECHA"}
};

static const uint8_t EEPROM_MAGIC_0 = 0xA6;
static const uint8_t EEPROM_MAGIC_1 = 0x5C;
static const uint8_t EEPROM_VERSION = 1;
static const uint8_t EEPROM_DATA_LENGTH = 19;
static const unsigned long STARTUP_GUARD_MS = 500;

uint16_t p_stop[CHANNEL_COUNT];
uint8_t estado_actual[AXIS_COUNT] = {0, 0, 0, 0, 0, 0};

static uint8_t debounceCandidate[AXIS_COUNT] = {0, 0, 0, 0, 0, 0};
static uint8_t debounceCount[AXIS_COUNT] = {0, 0, 0, 0, 0, 0};
static uint8_t heldSamples[AXIS_COUNT] = {0, 0, 0, 0, 0, 0};
static uint8_t appliedState[AXIS_COUNT] = {0, 0, 0, 0, 0, 0};
static bool outputsEnabled = false;
static unsigned long enabledAtMs = 0;
static unsigned long lastButtonSampleMs = 0;
static char commandBuffer[48];
static uint8_t commandLength = 0;
static bool commandOverflow = false;

static uint16_t clampPulse(int32_t pulse) {
  if (pulse < 102) return 102;
  if (pulse > 512) return 512;
  return (uint16_t)pulse;
}

static void pcaWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t pcaReadRegister(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1);
  if (!Wire.available()) return 0;
  return (uint8_t)Wire.read();
}

static void initPca9685() {
  uint8_t oldMode = pcaReadRegister(PCA_MODE1);
  uint8_t sleepMode = (uint8_t)((oldMode & 0x7F) | 0x10);
  pcaWriteRegister(PCA_MODE1, sleepMode);
  pcaWriteRegister(PCA_PRESCALE, 121); // 25 MHz oscillator, approximately 50 Hz
  pcaWriteRegister(PCA_MODE2, 0x04);   // Totem-pole outputs
  pcaWriteRegister(PCA_MODE1, (uint8_t)(oldMode & (uint8_t)~0x10));
  delay(5);
  pcaWriteRegister(PCA_MODE1, (uint8_t)((oldMode & (uint8_t)~0x10) | 0xA0));
}

static void setServoPulse(uint8_t channel, uint16_t pulseUs) {
  if (channel >= CHANNEL_COUNT) return;
  uint16_t boundedPulse = clampPulse(pulseUs);
  uint16_t ticks = (uint32_t)boundedPulse * 4096UL / 20000UL;
  uint8_t reg = (uint8_t)(PCA_LED0_ON_L + 4 * channel);
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)0);                // ON low
  Wire.write((uint8_t)0);                // ON high
  Wire.write((uint8_t)(ticks & 0xFF));   // OFF low
  Wire.write((uint8_t)(ticks >> 8));     // OFF high
  Wire.endTransmission();
}

static void writeAllStops() {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    setServoPulse(channel, p_stop[channel]);
  }
}

static uint16_t crc16Ccitt(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void loadDefaultStops() {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) p_stop[channel] = 307;
}

static bool loadStopsFromEeprom() {
  if (EEPROM.read(0) != EEPROM_MAGIC_0 || EEPROM.read(1) != EEPROM_MAGIC_1 ||
      EEPROM.read(2) != EEPROM_VERSION) return false;

  uint8_t payload[15]; // version plus seven little-endian uint16 values
  for (uint8_t i = 0; i < sizeof(payload); ++i) payload[i] = EEPROM.read((int)(2 + i));
  uint16_t expected = (uint16_t)EEPROM.read(17) | ((uint16_t)EEPROM.read(18) << 8);
  if (crc16Ccitt(payload, sizeof(payload)) != expected) return false;

  uint16_t loaded[CHANNEL_COUNT];
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    uint8_t address = (uint8_t)(3 + channel * 2);
    loaded[channel] = (uint16_t)EEPROM.read(address) |
                      ((uint16_t)EEPROM.read((uint8_t)(address + 1)) << 8);
    if (loaded[channel] < 290 || loaded[channel] > 325) return false;
  }
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) p_stop[channel] = loaded[channel];
  return true;
}

static void saveStopsToEeprom() {
  uint8_t payload[15];
  payload[0] = EEPROM_VERSION;
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    payload[1 + channel * 2] = (uint8_t)(p_stop[channel] & 0xFF);
    payload[2 + channel * 2] = (uint8_t)(p_stop[channel] >> 8);
  }
  uint16_t crc = crc16Ccitt(payload, sizeof(payload));
  EEPROM.update(0, EEPROM_MAGIC_0);
  EEPROM.update(1, EEPROM_MAGIC_1);
  for (uint8_t i = 0; i < sizeof(payload); ++i) EEPROM.update((int)(2 + i), payload[i]);
  EEPROM.update(17, (uint8_t)(crc & 0xFF));
  EEPROM.update(18, (uint8_t)(crc >> 8));
}

static void printStops() {
  Serial.print(F("p_stop=["));
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    if (channel) Serial.print(',');
    Serial.print(p_stop[channel]);
  }
  Serial.println(']');
}

static void resetButtonState() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    estado_actual[axis] = 0;
    debounceCandidate[axis] = 0;
    debounceCount[axis] = 0;
    heldSamples[axis] = 0;
    appliedState[axis] = 0;
  }
}

static void stopAxis(uint8_t axis) {
  setServoPulse(axisChannels[axis][0], p_stop[axisChannels[axis][0]]);
  if (axisChannels[axis][1] != axisChannels[axis][0]) {
    setServoPulse(axisChannels[axis][1], p_stop[axisChannels[axis][1]]);
  }
}

static void moveAxis(uint8_t axis, uint8_t direction) {
  uint8_t firstChannel = axisChannels[axis][0];
  int32_t firstPulse = (int32_t)p_stop[firstChannel] + axisOffsets[axis][direction - 1];
  setServoPulse(firstChannel, clampPulse(firstPulse));
  if (axisChannels[axis][1] != firstChannel) {
    uint8_t secondChannel = axisChannels[axis][1];
    int32_t secondPulse = (int32_t)p_stop[secondChannel] - axisOffsets[axis][direction - 1];
    setServoPulse(secondChannel, clampPulse(secondPulse));
  }
  Serial.println(axisMessages[axis][direction - 1]);
}

static uint8_t readRawPair(uint8_t axis) {
  bool firstHigh = digitalRead(buttonPins[axis][0]) == HIGH;
  bool secondHigh = digitalRead(buttonPins[axis][1]) == HIGH;
  if (firstHigh && secondHigh) return 0;
  if (!firstHigh && secondHigh) return 1;
  if (firstHigh && !secondHigh) return 2;
  return 3; // both buttons pressed: never command motion
}

static void acceptAxisState(uint8_t axis, uint8_t accepted) {
  if (accepted != estado_actual[axis]) {
    uint8_t previous = estado_actual[axis];
    estado_actual[axis] = accepted;
    heldSamples[axis] = (accepted == 1 || accepted == 2) ? 1 : 0;

    if (accepted == 0) {
      if (appliedState[axis] != 0 || previous != 0) {
        stopAxis(axis);
        Serial.println(F("DETENIDO (Freno)"));
      }
      appliedState[axis] = 0;
      return;
    }

    if (accepted == 3) {
      if (appliedState[axis] != 0) stopAxis(axis);
      appliedState[axis] = 0;
      Serial.println(F("BOTONES EN CONFLICTO; EJE DETENIDO"));
      return;
    }

    if (appliedState[axis] != 0) stopAxis(axis);
    appliedState[axis] = 0;
    return;
  }

  if (accepted != 1 && accepted != 2) return;
  if (heldSamples[axis] < 2) ++heldSamples[axis];
  if (heldSamples[axis] >= 2 && appliedState[axis] != accepted) {
    moveAxis(axis, accepted);
    appliedState[axis] = accepted;
  }
}

static void pollButtons() {
  unsigned long now = millis();
  if (now - lastButtonSampleMs < 10) return;
  lastButtonSampleMs = now;

  if (!outputsEnabled || now - enabledAtMs < STARTUP_GUARD_MS) {
    for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
      debounceCandidate[axis] = readRawPair(axis);
      debounceCount[axis] = 0;
    }
    return;
  }

  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    uint8_t raw = readRawPair(axis);
    if (raw == debounceCandidate[axis]) {
      if (debounceCount[axis] < 2) ++debounceCount[axis];
    } else {
      debounceCandidate[axis] = raw;
      debounceCount[axis] = 1;
    }
    if (debounceCount[axis] >= 2) acceptAxisState(axis, debounceCandidate[axis]);
  }
}

static bool parseUnsigned(const char *text, uint16_t *value) {
  if (!text || !*text) return false;
  uint32_t result = 0;
  while (*text) {
    if (*text < '0' || *text > '9') return false;
    result = result * 10 + (uint8_t)(*text - '0');
    if (result > 65535UL) return false;
    ++text;
  }
  *value = (uint16_t)result;
  return true;
}

static void printState() {
  Serial.print(F("{\"status\":\"STATE\",\"oe\":"));
  Serial.print(digitalRead(PIN_OE) == HIGH ? 1 : 0);
  Serial.print(F(",\"p_stop\":["));
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    if (channel) Serial.print(',');
    Serial.print(p_stop[channel]);
  }
  Serial.print(F("],\"estado\":["));
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    if (axis) Serial.print(',');
    Serial.print(estado_actual[axis]);
  }
  Serial.println(F("]}"));
}

static void handleCommand(char *line) {
  if (line[0] == 'C') {
    char *colon = line + 1;
    while (*colon && *colon != ':') ++colon;
    if (*colon != ':') {
      Serial.println(F("ERROR FORMATO: C<ch>:<pulse>"));
      return;
    }
    *colon = '\0';
    uint16_t channel;
    uint16_t pulse;
    if (!parseUnsigned(line + 1, &channel) || !parseUnsigned(colon + 1, &pulse) ||
        channel >= CHANNEL_COUNT || pulse < 290 || pulse > 325) {
      Serial.println(F("ERROR RANGO: canal 0..6, pulso 290..325"));
      return;
    }
    p_stop[channel] = pulse;
    setServoPulse((uint8_t)channel, pulse);
    Serial.print(F("P_STOP_ACTUALIZADO ch="));
    Serial.print(channel);
    Serial.print(F(" pulse="));
    Serial.println(pulse);
    return;
  }

  if (strcmp(line, "E") == 0) {
    saveStopsToEeprom();
    Serial.println(F("GUARDADO_EEPROM"));
  } else if (strcmp(line, "D") == 0) {
    loadDefaultStops();
    writeAllStops();
    Serial.println(F("P_STOP_RESTAURADOS_A_307 (RAM)"));
  } else if (strcmp(line, "Q") == 0) {
    printState();
  } else if (strcmp(line, "S") == 0) {
    writeAllStops();
    digitalWrite(PIN_OE, HIGH);
    outputsEnabled = false;
    resetButtonState();
    Serial.println(F("PARADA DE EMERGENCIA"));
  } else if (strcmp(line, "R") == 0) {
    writeAllStops();
    resetButtonState();
    digitalWrite(PIN_OE, LOW);
    outputsEnabled = true;
    enabledAtMs = millis();
    Serial.println(F("HABILITADO; ESPERANDO 500 ms"));
  } else if (strcmp(line, "I") == 0) {
    Wire.beginTransmission(PCA_ADDR);
    uint8_t result = Wire.endTransmission();
    if (result == 0) Serial.println(F("CONECTADO_0x40"));
    else Serial.println(F("ERROR_0x40"));
  } else if (strcmp(line, "V") == 0) {
    Serial.println(F("fw botonera 1.0.0"));
  } else if (*line) {
    Serial.println(F("COMANDO DESCONOCIDO"));
  }
}

static void serviceSerial() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      if (commandOverflow) {
        Serial.println(F("ERROR LINEA DEMASIADO LARGA"));
      } else {
        commandBuffer[commandLength] = '\0';
        handleCommand(commandBuffer);
      }
      commandLength = 0;
      commandOverflow = false;
    } else if (!commandOverflow) {
      if (commandLength < sizeof(commandBuffer) - 1) commandBuffer[commandLength++] = incoming;
      else commandOverflow = true;
    }
  }
}

void setup() {
  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, HIGH);
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    pinMode(buttonPins[axis][0], INPUT_PULLUP);
    pinMode(buttonPins[axis][1], INPUT_PULLUP);
  }

  Serial.begin(115200);
  loadDefaultStops();
  bool loaded = loadStopsFromEeprom();

  Wire.begin();
  Wire.setClock(400000);
  initPca9685();
  writeAllStops();
  delay(1000);
  digitalWrite(PIN_OE, LOW);
  outputsEnabled = true;
  enabledAtMs = millis();
  lastButtonSampleMs = enabledAtMs;
  resetButtonState();

  Serial.println(loaded ? F("EEPROM p_stop cargado") : F("p_stop por defecto (307)"));
  printStops();
  Serial.println(F("BOTONERA LISTA; antirrebote 20 ms, guarda inicial 500 ms"));
}

void loop() {
  serviceSerial();
  pollButtons();
  delay(1);
}
