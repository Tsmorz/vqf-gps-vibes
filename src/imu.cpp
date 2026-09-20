#include "imu.h"

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "config.h"
#include "i2c_bus.h"
#include "mag_cal.h"

namespace {

Adafruit_LSM6DSOX lsm;
Adafruit_LIS3MDL lis;

// Per-chip health. `address` is remembered from a successful probe so a
// reconnect does not have to re-guess between the primary and alternate
// address the Adafruit breakouts can be strapped to.
struct SensorHealth {
    bool healthy = false;
    uint8_t address = 0;
    int consecutive_failures = 0;
    uint32_t total_failures = 0;
    uint32_t last_retry_ms = 0;
    bool reconnected = false;  // set on recovery, cleared once acted on
};

SensorHealth accel_gyro;
SensorHealth magnetometer;

// ── Magnetometer calibration ────────────────────────────────────────────────
// The fit is applied on core 1 as samples arrive; start/finish/clear come from
// the web handler on core 0. The spinlock covers both the request flags and
// the published status, and every critical section here is a few assignments.
MagCalibration mag_cal;
MagCalCollector mag_collector;
bool mag_collecting = false;
float mag_field_ut = 0.0f;
portMUX_TYPE mag_lock = portMUX_INITIALIZER_UNLOCKED;

constexpr const char* kMagCalNamespace = "magcal";

// Restores the calibration saved by a previous sweep, so the board comes up
// with a usable heading instead of needing a sweep after every reboot.
void LoadMagCalibration() {
    Preferences prefs;
    if (!prefs.begin(kMagCalNamespace, true)) {
        return;
    }
    const float offset_x = prefs.getFloat("ox", NAN);
    if (isfinite(offset_x)) {
        mag_cal.offset[0] = offset_x;
        mag_cal.offset[1] = prefs.getFloat("oy", 0.0f);
        mag_cal.offset[2] = prefs.getFloat("oz", 0.0f);
        mag_cal.scale[0] = prefs.getFloat("sx", 1.0f);
        mag_cal.scale[1] = prefs.getFloat("sy", 1.0f);
        mag_cal.scale[2] = prefs.getFloat("sz", 1.0f);
        mag_cal.valid = true;
        Serial.printf("[imu] mag calibration restored: offset %.1f %.1f %.1f uT\n",
                      mag_cal.offset[0], mag_cal.offset[1], mag_cal.offset[2]);
    }
    prefs.end();
}

void SaveMagCalibration(const MagCalibration& cal) {
    Preferences prefs;
    if (!prefs.begin(kMagCalNamespace, false)) {
        return;
    }
    prefs.putFloat("ox", cal.offset[0]);
    prefs.putFloat("oy", cal.offset[1]);
    prefs.putFloat("oz", cal.offset[2]);
    prefs.putFloat("sx", cal.scale[0]);
    prefs.putFloat("sy", cal.scale[1]);
    prefs.putFloat("sz", cal.scale[2]);
    prefs.end();
}

// How often to confirm each chip still acknowledges on the bus.
//
// This probe is the only reliable way to notice an unplugged sensor.
// Adafruit_LSM6DS::getEvent() ends in an unconditional `return true;` and the
// _read() behind it is declared void, so a failed I2C transfer never reaches
// the caller -- an unplugged LSM6DSOX reads as a clean (0, 0, 0), which the
// navigation filter would faithfully integrate as free fall. Asking the
// address whether anything is still there costs one tiny transaction and
// cannot be fooled the same way.
constexpr uint32_t kPresenceCheckIntervalMs = 250;
uint32_t last_presence_check_ms = 0;

// Applies the ranges and output data rates the filter expects.
//
// 833 Hz comfortably exceeds the 200 Hz estimator tick, so every read sees a
// fresh sample. The LIS3MDL tops out at 155 Hz, below the tick rate, so some
// ticks re-read the same magnetometer sample -- harmless, because VQF's
// magnetic correction is a heavily low-pass-filtered term with a ~9 s time
// constant and does not need gyro-rate updates.
void ConfigureAccelGyro() {
    lsm.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
    lsm.setAccelDataRate(LSM6DS_RATE_833_HZ);
    lsm.setGyroDataRate(LSM6DS_RATE_833_HZ);
}

void ConfigureMagnetometer() {
    lis.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}

// Tries the primary then the alternate address, recording whichever answers.
bool BeginAccelGyro() {
    for (uint8_t address : {LSM6DSOX_ADDR_PRIMARY, LSM6DSOX_ADDR_ALT}) {
        if (!lsm.begin_I2C(address, &Wire)) {
            continue;
        }
        ConfigureAccelGyro();
        accel_gyro.address = address;
        accel_gyro.healthy = true;
        accel_gyro.consecutive_failures = 0;
        Serial.printf("[imu] LSM6DSOX ready at 0x%02X\n", address);
        return true;
    }
    return false;
}

bool BeginMagnetometer() {
    for (uint8_t address : {LIS3MDL_ADDR_PRIMARY, LIS3MDL_ADDR_ALT}) {
        if (!lis.begin_I2C(address, &Wire)) {
            continue;
        }
        ConfigureMagnetometer();
        magnetometer.address = address;
        magnetometer.healthy = true;
        magnetometer.consecutive_failures = 0;
        Serial.printf("[imu] LIS3MDL ready at 0x%02X\n", address);
        return true;
    }
    return false;
}

// Records one failed read and takes the chip offline once failures pile up.
// A few failures on a flexing cable are normal and must not cause a reinit
// storm, so the chip is only declared lost after a run of them.
void NoteFailure(SensorHealth& health, const char* name) {
    health.total_failures++;
    if (++health.consecutive_failures < SENSOR_MAX_CONSEC_FAILS) {
        return;
    }
    if (health.healthy) {
        Serial.printf("[imu] %s lost after %d consecutive failures\n", name,
                      health.consecutive_failures);
    }
    health.healthy = false;
}

void NoteSuccess(SensorHealth& health) {
    health.consecutive_failures = 0;
}

// Takes a chip offline the moment it stops acknowledging. Unlike a failed
// read, a missing ACK is unambiguous, so there is no tolerance count here.
void CheckPresence(SensorHealth& health, const char* name) {
    if (!health.healthy || health.address == 0) {
        return;
    }
    if (I2cDeviceResponds(health.address)) {
        return;
    }
    Serial.printf("[imu] %s stopped acknowledging at 0x%02X -- offline\n", name, health.address);
    health.healthy = false;
    health.total_failures++;
}

// Probes both chips, rate-limited so the check never competes with sensor
// reads for bus time.
void CheckPresenceOfBothChips() {
    const uint32_t now = millis();
    if (now - last_presence_check_ms < kPresenceCheckIntervalMs) {
        return;
    }
    last_presence_check_ms = now;
    CheckPresence(accel_gyro, "LSM6DSOX");
    CheckPresence(magnetometer, "LIS3MDL");
}

// Re-initialises an offline chip, at most once per SENSOR_RETRY_INTERVAL_MS so
// a permanently unplugged sensor cannot monopolise the bus. The bus itself is
// recovered first, since a mid-transfer reset can leave SDA held low.
void RetryIfOffline(SensorHealth& health, const char* name, bool (*begin_fn)()) {
    const uint32_t now = millis();
    if (health.healthy || now - health.last_retry_ms < SENSOR_RETRY_INTERVAL_MS) {
        return;
    }
    health.last_retry_ms = now;

    if (!I2cDeviceResponds(health.address)) {
        I2cRecover();
    }
    if (begin_fn()) {
        health.reconnected = true;
        Serial.printf("[imu] %s reconnected\n", name);
    }
}

}  // namespace

void ImuBegin() {
    LoadMagCalibration();
    if (!BeginAccelGyro()) {
        Serial.println("[imu] LSM6DSOX not found -- will keep retrying");
    }
    if (!BeginMagnetometer()) {
        Serial.println("[imu] LIS3MDL not found -- will keep retrying");
    }
}

bool ImuRead(ImuSample& out) {
    CheckPresenceOfBothChips();
    RetryIfOffline(accel_gyro, "LSM6DSOX", BeginAccelGyro);
    RetryIfOffline(magnetometer, "LIS3MDL", BeginMagnetometer);

    if (accel_gyro.healthy) {
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t temperature_event;
        if (lsm.getEvent(&accel_event, &gyro_event, &temperature_event)) {
            out.accel[0] = accel_event.acceleration.x;
            out.accel[1] = accel_event.acceleration.y;
            out.accel[2] = accel_event.acceleration.z;
            out.gyro[0] = gyro_event.gyro.x;
            out.gyro[1] = gyro_event.gyro.y;
            out.gyro[2] = gyro_event.gyro.z;
            out.accel_gyro_valid = true;
            NoteSuccess(accel_gyro);
        } else {
            out.accel_gyro_valid = false;
            NoteFailure(accel_gyro, "LSM6DSOX");
        }
    } else {
        out.accel_gyro_valid = false;
    }

    if (magnetometer.healthy) {
        sensors_event_t mag_event;
        if (lis.getEvent(&mag_event)) {
            out.mag[0] = mag_event.magnetic.x;
            out.mag[1] = mag_event.magnetic.y;
            out.mag[2] = mag_event.magnetic.z;

            // A sweep collects raw readings; the correction is applied after,
            // so a sweep in progress never feeds on its own output.
            portENTER_CRITICAL(&mag_lock);
            if (mag_collecting) {
                mag_collector.Add(out.mag);
            }
            mag_cal.Apply(out.mag);
            mag_field_ut =
                sqrtf(out.mag[0] * out.mag[0] + out.mag[1] * out.mag[1] + out.mag[2] * out.mag[2]);
            portEXIT_CRITICAL(&mag_lock);

            out.mag_valid = true;
            NoteSuccess(magnetometer);
        } else {
            out.mag_valid = false;
            NoteFailure(magnetometer, "LIS3MDL");
        }
    } else {
        out.mag_valid = false;
    }

    return out.accel_gyro_valid;
}

bool ImuAccelGyroHealthy() {
    return accel_gyro.healthy;
}

bool ImuConsumeReconnectEvent() {
    if (!accel_gyro.reconnected) {
        return false;
    }
    accel_gyro.reconnected = false;
    return true;
}

bool ImuMagHealthy() {
    return magnetometer.healthy;
}

uint32_t ImuFailureCount() {
    return accel_gyro.total_failures + magnetometer.total_failures;
}

void ImuMagCalStart() {
    portENTER_CRITICAL(&mag_lock);
    mag_collector.Reset();
    mag_collecting = true;
    portEXIT_CRITICAL(&mag_lock);
    Serial.println("[imu] mag calibration started -- turn the board through all orientations");
}

bool ImuMagCalFinish() {
    MagCalibration fitted;
    bool solved = false;

    portENTER_CRITICAL(&mag_lock);
    mag_collecting = false;
    solved = mag_collector.Solve(fitted);
    if (solved) {
        mag_cal = fitted;
    }
    portEXIT_CRITICAL(&mag_lock);

    if (!solved) {
        Serial.println("[imu] mag calibration rejected -- the board was not turned far enough");
        return false;
    }
    SaveMagCalibration(fitted);
    Serial.printf("[imu] mag calibrated: offset %.1f %.1f %.1f uT  scale %.3f %.3f %.3f\n",
                  fitted.offset[0], fitted.offset[1], fitted.offset[2], fitted.scale[0],
                  fitted.scale[1], fitted.scale[2]);
    return true;
}

void ImuMagCalClear() {
    portENTER_CRITICAL(&mag_lock);
    mag_cal = MagCalibration();
    mag_collecting = false;
    mag_collector.Reset();
    portEXIT_CRITICAL(&mag_lock);

    Preferences prefs;
    if (prefs.begin(kMagCalNamespace, false)) {
        prefs.clear();
        prefs.end();
    }
    Serial.println("[imu] mag calibration cleared");
}

MagCalStatus ImuMagCalStatus() {
    MagCalStatus status;
    portENTER_CRITICAL(&mag_lock);
    status.calibrated = mag_cal.valid;
    status.collecting = mag_collecting;
    status.progress = mag_collector.progress();
    status.samples = mag_collector.sample_count();
    for (int axis = 0; axis < 3; axis++) {
        status.offset[axis] = mag_cal.offset[axis];
    }
    status.field_ut = mag_field_ut;
    portEXIT_CRITICAL(&mag_lock);
    return status;
}
