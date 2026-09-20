#include "status_led.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "config.h"

namespace {

Adafruit_NeoPixel pixel(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

SystemStatus current = SystemStatus::kInitialising;
uint32_t last_update_ms = 0;

// Refresh rate for the animation. 50 Hz is smooth to the eye and leaves the
// single-wire LED protocol using a negligible slice of the loop.
constexpr uint32_t kUpdateIntervalMs = 20;

// How each status looks: base colour, animation period, and whether it
// breathes smoothly or blinks hard on/off.
struct Pattern {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint16_t period_ms;
    bool breathe;
};

Pattern PatternFor(SystemStatus status) {
    switch (status) {
        case SystemStatus::kNormal:
            return {0, 255, 0, 3000, true};
        case SystemStatus::kWarning:
            return {255, 140, 0, 900, false};
        case SystemStatus::kError:
            return {255, 0, 0, 250, false};
        case SystemStatus::kInitialising:
        default:
            return {0, 40, 255, 600, true};
    }
}

// Returns the 0..255 intensity for this instant in the pattern's cycle.
// Breathing is a triangle wave rather than a sine -- visually equivalent here
// and far cheaper than a trig call at 50 Hz.
uint8_t IntensityAt(const Pattern& pattern, uint32_t now_ms) {
    const uint32_t phase = now_ms % pattern.period_ms;
    if (!pattern.breathe) {
        return phase < pattern.period_ms / 2 ? 255 : 0;
    }
    const uint32_t half = pattern.period_ms / 2;
    const uint32_t rising = phase < half ? phase : pattern.period_ms - phase;
    // Keep a dim floor so a breathing light never looks like it has gone out.
    return static_cast<uint8_t>(25 + (rising * 230) / half);
}

}  // namespace

void StatusLedBegin() {
    // The FeatherS3 gates the LED's 3.3 V rail behind a GPIO; without this the
    // LED stays dark no matter what is clocked into it.
    pinMode(RGB_LED_POWER_PIN, OUTPUT);
    digitalWrite(RGB_LED_POWER_PIN, HIGH);
    delay(10);

    pixel.begin();
    pixel.setBrightness(RGB_LED_BRIGHTNESS);
    pixel.clear();
    pixel.show();
    StatusLedSet(SystemStatus::kInitialising);
}

void StatusLedSet(SystemStatus status) {
    current = status;
}

SystemStatus StatusLedCurrent() {
    return current;
}

void StatusLedUpdate() {
    const uint32_t now = millis();
    if (now - last_update_ms < kUpdateIntervalMs) {
        return;
    }
    last_update_ms = now;

    const Pattern pattern = PatternFor(current);
    const uint16_t intensity = IntensityAt(pattern, now);
    pixel.setPixelColor(0, pixel.Color(static_cast<uint8_t>(pattern.red * intensity / 255),
                                       static_cast<uint8_t>(pattern.green * intensity / 255),
                                       static_cast<uint8_t>(pattern.blue * intensity / 255)));
    pixel.show();
}
