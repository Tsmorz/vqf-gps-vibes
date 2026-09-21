#include "button.h"

#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>

#include "board_power.h"
#include "config.h"

namespace {

constexpr uint32_t kDebounceMs = 30;

bool pressed = false;  // debounced state
bool raw_last = false;
uint32_t raw_changed_ms = 0;
uint32_t pressed_since_ms = 0;
bool long_fired = false;

bool ReadRaw() {
    return digitalRead(BUTTON_PIN) == LOW;
}

}  // namespace

void ButtonBegin() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

ButtonEvent ButtonPoll() {
    const uint32_t now = millis();
    const bool raw = ReadRaw();
    if (raw != raw_last) {
        raw_last = raw;
        raw_changed_ms = now;
    }
    if (raw != pressed && now - raw_changed_ms >= kDebounceMs) {
        pressed = raw;
        if (pressed) {
            pressed_since_ms = now;
            long_fired = false;
        } else if (!long_fired) {
            return ButtonEvent::kShortPress;
        }
    }
    if (pressed && !long_fired && now - pressed_since_ms >= BUTTON_LONG_PRESS_MS) {
        long_fired = true;
        return ButtonEvent::kLongPress;
    }
    return ButtonEvent::kNone;
}

void ButtonSleepUntilPressed() {
    // Still held from the long press: arming a low-level wake now would end
    // the sleep the instant it began.
    while (ReadRaw()) {
        delay(10);
    }
    delay(kDebounceMs);

    BoardPowerSleep();
    // The RTC domain has its own pad control, so the pull-up has to be
    // requested again for the sleep; without it the line floats if the
    // board's external pull-up is ever absent.
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(BUTTON_PIN));
    rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(BUTTON_PIN));
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(BUTTON_PIN), 0);
    esp_deep_sleep_start();
}
