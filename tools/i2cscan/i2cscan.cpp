// Standalone bring-up probe for the FeatherS3 + STEMMA sensor stack.
//
// Answers two questions that decide the firmware's pin configuration:
//   1. Which of the board's two I2C buses each sensor is actually wired to.
//   2. Whether the onboard RGB LED lights (it needs its power rail enabled).
//
// Build and run with `task scan`. This file is NOT part of the firmware --
// platformio.ini compiles it only in the `i2cscan` environment.

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>

namespace {

// FeatherS3 pinout (variants/um_feathers3/pins_arduino.h).
constexpr int kBus0Sda = 8;    // SDA  -- broken out on the header
constexpr int kBus0Scl = 9;    // SCL
constexpr int kBus1Sda = 16;   // SDA1 -- the second, independent bus
constexpr int kBus1Scl = 15;   // SCL1
constexpr int kRgbData = 40;   // WS2812B data
constexpr int kRgbPower = 39;  // must be driven HIGH or the LED stays dark

Adafruit_NeoPixel pixel(1, kRgbData, NEO_GRB + NEO_KHZ800);
TwoWire bus1(1);

// Maps the I2C addresses this project expects to a human-readable name so the
// scan output can be read without a datasheet on hand.
const char* DeviceName(uint8_t address) {
    switch (address) {
        case 0x10:
            return "PA1010D Mini GPS";
        case 0x1C:
        case 0x1E:
            return "LIS3MDL magnetometer";
        case 0x6A:
        case 0x6B:
            return "LSM6DSOX accel+gyro";
        default:
            return "unknown";
    }
}

// Probes every valid 7-bit address on `wire` and prints whatever answers.
// Returns the number of devices found so the caller can flag an empty bus.
int ScanBus(TwoWire& wire, const char* label) {
    int found = 0;
    Serial.printf("\n[scan] %s\n", label);
    for (uint8_t address = 0x08; address < 0x78; address++) {
        wire.beginTransmission(address);
        if (wire.endTransmission() != 0) {
            continue;
        }
        Serial.printf("       0x%02X  %s\n", address, DeviceName(address));
        found++;
    }
    if (found == 0) {
        Serial.println("       (nothing responded -- check the STEMMA cable)");
    }
    return found;
}

}  // namespace

void setup() {
    Serial.begin(115200);
    // The native-USB CDC port enumerates after boot; give the host a moment so
    // the first lines of output are not lost.
    delay(2000);
    Serial.println("\n=== FeatherS3 I2C / LED probe ===");

    pinMode(kRgbPower, OUTPUT);
    digitalWrite(kRgbPower, HIGH);
    pixel.begin();
    pixel.setBrightness(40);

    Wire.begin(kBus0Sda, kBus0Scl);
    bus1.begin(kBus1Sda, kBus1Scl);
    Serial.printf("bus0 = SDA %d / SCL %d      bus1 = SDA %d / SCL %d\n", kBus0Sda, kBus0Scl,
                  kBus1Sda, kBus1Scl);
}

void loop() {
    // Cycle red -> green -> blue so a wrong LED pin or a dead power rail is
    // obvious at a glance while the scan runs.
    static const uint32_t kColors[] = {0xFF0000, 0x00FF00, 0x0000FF};
    static uint8_t color_index = 0;
    pixel.setPixelColor(0, kColors[color_index]);
    pixel.show();
    color_index = (color_index + 1) % 3;

    ScanBus(Wire, "bus0 (Wire)");
    ScanBus(bus1, "bus1 (Wire1)");
    delay(3000);
}
