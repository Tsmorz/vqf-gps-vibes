#pragma once

#include <stdint.h>

// Driver for the Adafruit LSM6DSOX (accelerometer + gyroscope) and LIS3MDL
// (magnetometer) breakout, in sensor units ready to hand to VQF.
//
// The two chips are tracked independently: losing the magnetometer only costs
// absolute heading, so the filter keeps running on 6-DOF data, while losing
// the accelerometer/gyroscope stops orientation and propagation entirely.

struct ImuSample {
    float accel[3] = {0, 0, 0};  // m/s^2, body frame
    float gyro[3] = {0, 0, 0};   // rad/s, body frame, raw (VQF removes bias)
    float mag[3] = {0, 0, 0};    // microtesla, body frame, hard/soft-iron corrected
    bool accel_gyro_valid = false;
    bool mag_valid = false;
};

// State of the magnetometer calibration, for the dashboard.
struct MagCalStatus {
    bool calibrated = false;        // a fit is loaded and being applied
    bool collecting = false;        // a sweep is in progress
    float progress = 0.0f;          // 0..1 through the sweep
    float implied_field_ut = 0.0f;  // field strength the sweep implies so far
    int samples = 0;
    float offset[3] = {0, 0, 0};
    float field_ut = 0.0f;  // magnitude of the latest corrected reading
};

// Attempts to bring both chips up. Safe to call with nothing plugged in --
// missing sensors are simply reported as unhealthy and retried later.
void ImuBegin();

// Reads both chips into `out`. Returns true if accelerometer and gyroscope
// data is usable; `out.mag_valid` separately reports the magnetometer.
// Failures are counted internally and trigger re-initialisation, so this can
// be called every tick whatever the hardware is doing.
bool ImuRead(ImuSample& out);

// ── Full-scale ranges ────────────────────────────────────────────────────────
// Boot defaults come from IMU_ACCEL_RANGE_G / IMU_GYRO_RANGE_DPS /
// IMU_MAG_RANGE_GAUSS in config.h; the dashboard can change them at runtime.

// Applies new ranges, snapping each to the nearest value its chip supports.
// Issues I2C -- call only from the estimator task, which owns the bus.
//
// The requested ranges are remembered even while a chip is offline, so a
// sensor that reconnects comes back on the range the user selected rather
// than reverting to the compile-time default.
//
// Returns true if the accelerometer range changed, which invalidates the
// navigation filter's learned accelerometer bias: the part's offset and scale
// error are specific to the range it was measured on, so the old bias is no
// longer describing the same signal path. The caller is responsible for
// forgetting it -- see ApplyImuRanges() in estimator.cpp.
bool ImuSetRanges(int accel_g, int gyro_dps, int mag_gauss);

// The ranges actually in force, after snapping. The estimator publishes these
// so the dashboard's dropdowns show what the hardware is really running.
void ImuGetRanges(int& accel_g, int& gyro_dps, int& mag_gauss);

bool ImuAccelGyroHealthy();
bool ImuMagHealthy();

// True once after the accelerometer/gyroscope comes back from a dropout, then
// false until the next one. The estimator uses this to reset the orientation
// filter, whose internal state means nothing across a gap in the data.
bool ImuConsumeReconnectEvent();

// ── Magnetometer calibration ─────────────────────────────────────────────────
// See mag_cal.h for why this is needed. Requests are made from the web handler
// on core 0 and acted on by the estimator task on core 1.

// Begins a sweep. The user turns the board through all orientations; readings
// are corrected with the previous calibration until a new one is fitted.
void ImuMagCalStart();

// Ends a sweep, fitting and saving the result. Returns false if the board was
// not rotated enough, in which case the previous calibration is kept.
bool ImuMagCalFinish();

// Discards the stored calibration and goes back to raw readings.
void ImuMagCalClear();

MagCalStatus ImuMagCalStatus();

// Puts both chips into power-down (a few uA instead of continuous sampling).
// They sit on LDO1, which stays powered through deep sleep, so without this
// they would keep sampling for the whole sleep. Issues I2C -- call only from
// the estimator task. A chip that is absent is skipped. The chips are fully
// reconfigured by ImuBegin() on the next boot, so nothing has to undo this.
void ImuPowerDown();

// Total read failures since boot -- surfaced on the dashboard as an early
// warning that a STEMMA cable is intermittent.
uint32_t ImuFailureCount();
