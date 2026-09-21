#include "board_power.h"

#include <Arduino.h>
#include <driver/gpio.h>

#include "config.h"

namespace {

// Long enough for the PA1010D to be ready to answer on I2C. The BMP390 needs
// only a couple of milliseconds; the GPS receiver is what sets this.
constexpr uint32_t kRailSettleMs = kAuxRailSettleMs;

// Long enough for the rail to discharge so the sensors genuinely reset, rather
// than merely dipping and holding their confused state.
constexpr uint32_t kRailOffMs = 200;

}  // namespace

void BoardPowerBegin() {
    // After a deep sleep the pin is still latched low; release it first or the
    // write below is ignored and the aux sensors stay dark.
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(static_cast<gpio_num_t>(LDO2_ENABLE_PIN));
    pinMode(LDO2_ENABLE_PIN, OUTPUT);
    digitalWrite(LDO2_ENABLE_PIN, HIGH);
    delay(kRailSettleMs);
}

void BoardPowerAuxSet(bool on) {
    digitalWrite(LDO2_ENABLE_PIN, on ? HIGH : LOW);
}

void BoardPowerCycleAux() {
    Serial.println("[pwr] power-cycling LDO2 (GPS + barometer)");
    digitalWrite(LDO2_ENABLE_PIN, LOW);
    delay(kRailOffMs);
    digitalWrite(LDO2_ENABLE_PIN, HIGH);
    delay(kRailSettleMs);
}

void BoardPowerSleep() {
    digitalWrite(LDO2_ENABLE_PIN, LOW);
    gpio_hold_en(static_cast<gpio_num_t>(LDO2_ENABLE_PIN));
    gpio_deep_sleep_hold_en();
}
