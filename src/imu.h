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
    bool calibrated = false;  // a fit is loaded and being applied
    bool collecting = false;  // a sweep is in progress
    float progress = 0.0f;    // 0..1 through the sweep
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

// Total read failures since boot -- surfaced on the dashboard as an early
// warning that a STEMMA cable is intermittent.
uint32_t ImuFailureCount();
