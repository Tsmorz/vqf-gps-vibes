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
    float mag[3] = {0, 0, 0};    // microtesla, body frame
    bool accel_gyro_valid = false;
    bool mag_valid = false;
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

// Total read failures since boot -- surfaced on the dashboard as an early
// warning that a STEMMA cable is intermittent.
uint32_t ImuFailureCount();
