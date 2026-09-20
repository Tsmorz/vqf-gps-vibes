#pragma once

// Plausibility limits for the IMU, derived from the ranges the chips are
// configured for in imu.cpp.
//
// These exist because the Adafruit driver cannot report a failed transfer:
// Adafruit_LSM6DS::getEvent() ends in an unconditional `return true;` and the
// _read() behind it is declared void. When the I2C peripheral is in a bad
// state the transfer returns ESP_ERR_INVALID_STATE, the driver hands back
// whatever was in the buffer, and the filter integrates it -- on the bench
// that produced gyro readings of 20 rad/s and magnetometer readings of 479 uT
// from a board sitting still on a desk.
//
// So the numbers themselves are the check. A reading at or beyond the
// configured full scale is not a measurement the hardware could have produced.
//
// Pure math, no Arduino dependency -- unit tested on the host.

#include <math.h>

namespace sensor_limits {

// Full scale of the ranges selected in imu.cpp: +-4 g, +-1000 dps, +-4 gauss.
constexpr float kAccelFullScaleMps2 = 4.0f * 9.80665f;
constexpr float kGyroFullScaleRadS = 1000.0f * 3.14159265f / 180.0f;
constexpr float kMagFullScaleUt = 400.0f;

// An accelerometer in free fall reads zero, but this board is not in free
// fall: sustained exact zeros mean the transfer failed. The threshold is well
// under 1 g so genuine brief weightlessness is not what trips it.
constexpr float kMinPlausibleAccelMps2 = 0.5f;

// Samples within this fraction of full scale are treated as saturated.
//
// A saturated reading is not wrong so much as uninformative: all it says is
// that the true value is at least full scale, so integrating it is
// meaningless whether it came from genuine clipping or from a corrupt
// transfer. Rejecting a few samples during a hard knock costs nothing -- the
// filter coasts through them.
constexpr float kSaturationFraction = 0.99f;

// True if every axis is finite and comfortably inside `limit`.
inline bool WithinFullScale(const float value[3], float limit) {
    const float usable = limit * kSaturationFraction;
    for (int axis = 0; axis < 3; axis++) {
        if (!isfinite(value[axis]) || fabsf(value[axis]) >= usable) {
            return false;
        }
    }
    return true;
}

inline float Magnitude(const float value[3]) {
    return sqrtf(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

// True if this accelerometer and gyroscope pair could have come from the
// hardware as configured.
inline bool AccelGyroLooksValid(const float accel[3], const float gyro[3]) {
    if (!WithinFullScale(accel, kAccelFullScaleMps2) ||
        !WithinFullScale(gyro, kGyroFullScaleRadS)) {
        return false;
    }
    return Magnitude(accel) >= kMinPlausibleAccelMps2;
}

inline bool MagLooksValid(const float mag[3]) {
    return WithinFullScale(mag, kMagFullScaleUt);
}

}  // namespace sensor_limits
