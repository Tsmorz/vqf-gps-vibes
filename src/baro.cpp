#include "baro.h"

#include <Adafruit_BMP3XX.h>
#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "i2c_bus.h"

namespace {

Adafruit_BMP3XX bmp;

bool healthy = false;
uint8_t address = 0;
int consecutive_failures = 0;
uint32_t total_failures = 0;
uint32_t last_retry_ms = 0;
uint32_t last_read_ms = 0;
uint32_t initialised_bus_generation = 0;

// The standard atmosphere's sea-level pressure. Used as a *fixed* reference so
// that the altitude this driver reports is a consistent function of pressure.
// Its offset from true altitude is real but constant over a session, which is
// exactly the kind of error the filter's barometer bias state absorbs.
constexpr float kStandardSeaLevelHpa = 1013.25f;

// Plausible range for surface pressure: roughly 9 km up to a deep low at sea
// level. Anything outside this is a corrupt transfer, not weather.
constexpr float kMinPressurePa = 30000.0f;
constexpr float kMaxPressurePa = 110000.0f;
constexpr float kMinTemperatureC = -45.0f;
constexpr float kMaxTemperatureC = 90.0f;

// Barometric formula from the BMP180 datasheet, the same one Adafruit's
// readAltitude() uses -- reproduced here so the reading comes from a single
// performReading() rather than the two the library's helper would trigger.
float PressureAltitude(float pressure_pa) {
    const float hpa = pressure_pa / 100.0f;
    return 44330.0f * (1.0f - powf(hpa / kStandardSeaLevelHpa, 0.1903f));
}

// Oversampling chosen for a quiet altitude signal rather than speed: 8x on
// pressure with the IIR filter engaged, which is Bosch's "drone" preset. The
// resulting conversion takes about 20 ms, comfortably inside the 40 ms poll.
void Configure() {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

bool Begin() {
    for (uint8_t candidate : {BMP390_ADDR_PRIMARY, BMP390_ADDR_ALT}) {
        if (!bmp.begin_I2C(candidate, &I2cWire(I2cBus::kAux))) {
            continue;
        }
        Configure();
        address = candidate;
        healthy = true;
        consecutive_failures = 0;
        Serial.printf("[baro] BMP390 ready at 0x%02X\n", candidate);
        return true;
    }
    return false;
}

void NoteFailure() {
    total_failures++;
    if (++consecutive_failures < SENSOR_MAX_CONSEC_FAILS) {
        return;
    }
    if (healthy) {
        Serial.printf("[baro] BMP390 lost after %d consecutive failures\n", consecutive_failures);
    }
    healthy = false;
}

// Re-initialises after a bus restart, which invalidates the device handle.
void ReinitAfterBusRecovery() {
    const uint32_t generation = I2cBusGeneration(I2cBus::kAux);
    if (generation == initialised_bus_generation) {
        return;
    }
    initialised_bus_generation = generation;
    healthy = false;
    last_retry_ms = 0;
}

// Rate-limited re-initialisation, so an absent sensor cannot monopolise the
// shared bus. An unresponsive device is absent, not a wedged bus -- recovery
// of the bus itself is the watchdog's job.
void RetryIfOffline() {
    const uint32_t now = millis();
    if (healthy || now - last_retry_ms < SENSOR_RETRY_INTERVAL_MS) {
        return;
    }
    last_retry_ms = now;
    if (I2cDeviceResponds(I2cBus::kAux, address == 0 ? BMP390_ADDR_PRIMARY : address)) {
        Begin();
    }
}

}  // namespace

void BaroBegin() {
    initialised_bus_generation = I2cBusGeneration(I2cBus::kAux);
    if (!Begin()) {
        Serial.println("[baro] BMP390 not found -- will keep retrying");
    }
}

bool BaroRead(BaroSample& out) {
    ReinitAfterBusRecovery();
    RetryIfOffline();
    if (!healthy) {
        return false;
    }

    const uint32_t now = millis();
    if (now - last_read_ms < BARO_POLL_INTERVAL_MS) {
        return false;
    }
    last_read_ms = now;

    // Unlike the LSM6DS driver, performReading() does report transfer
    // failures, so its return value is worth trusting -- but the values are
    // still range-checked, because a corrupt transfer can succeed.
    if (!bmp.performReading()) {
        NoteFailure();
        return false;
    }

    const float pressure_pa = static_cast<float>(bmp.pressure);
    const float temperature_c = static_cast<float>(bmp.temperature);
    if (!isfinite(pressure_pa) || pressure_pa < kMinPressurePa || pressure_pa > kMaxPressurePa ||
        !isfinite(temperature_c) || temperature_c < kMinTemperatureC ||
        temperature_c > kMaxTemperatureC) {
        NoteFailure();
        return false;
    }

    out.pressure_pa = pressure_pa;
    out.temperature_c = temperature_c;
    out.pressure_altitude_m = PressureAltitude(pressure_pa);
    out.valid = true;

    consecutive_failures = 0;
    I2cNoteTransferOk(I2cBus::kAux);
    return true;
}

bool BaroHealthy() {
    return healthy;
}

uint32_t BaroFailureCount() {
    return total_failures;
}
