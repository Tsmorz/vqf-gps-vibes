#include "board_power.h"

#include <Arduino.h>

#include "config.h"

namespace {

// Long enough for the PA1010D to be ready to answer on I2C. The BMP390 needs
// only a couple of milliseconds; the GPS receiver is what sets this.
constexpr uint32_t kRailSettleMs = 300;

// Long enough for the rail to discharge so the sensors genuinely reset, rather
// than merely dipping and holding their confused state.
constexpr uint32_t kRailOffMs = 200;

}  // namespace

void BoardPowerBegin() {
    pinMode(LDO2_ENABLE_PIN, OUTPUT);
    digitalWrite(LDO2_ENABLE_PIN, HIGH);
    delay(kRailSettleMs);
}

void BoardPowerCycleAux() {
    Serial.println("[pwr] power-cycling LDO2 (GPS + barometer)");
    digitalWrite(LDO2_ENABLE_PIN, LOW);
    delay(kRailOffMs);
    digitalWrite(LDO2_ENABLE_PIN, HIGH);
    delay(kRailSettleMs);
}
