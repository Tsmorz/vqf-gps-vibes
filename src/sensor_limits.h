#pragma once

// Plausibility limits for the IMU, derived from the ranges the chips are
// currently configured for.
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

#include "config.h"

namespace sensor_limits {

// Full scale of the ranges the chips are running right now.
//
// This is a parameter rather than a constant because the ranges are a runtime
// setting: the dashboard can widen the accelerometer mid-flight and the
// estimator re-applies it over I2C. A threshold left behind at the old range
// would reject every genuine reading past it, and a storm of rejected samples
// is indistinguishable on the dashboard from a failing sensor. Passing the
// active full scale in is what stops these two drifting apart.
struct FullScale {
    float accel_mps2;
    float gyro_rad_s;
    float mag_ut;
};

constexpr float kGravityMps2 = 9.80665f;
constexpr float kRadiansPerDegree = 3.14159265f / 180.0f;
constexpr float kMicroteslaPerGauss = 100.0f;

// Converts ranges expressed in the chips' own units into full-scale limits.
constexpr FullScale FullScaleFor(int accel_g, int gyro_dps, int mag_gauss) {
    return FullScale{accel_g * kGravityMps2, gyro_dps * kRadiansPerDegree,
                     mag_gauss * kMicroteslaPerGauss};
}

// The ranges the firmware boots with, before the dashboard changes anything.
constexpr FullScale kDefaultFullScale =
    FullScaleFor(IMU_ACCEL_RANGE_G, IMU_GYRO_RANGE_DPS, IMU_MAG_RANGE_GAUSS);

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
// filter coasts through them. Persistent rejections mean the range is too
// narrow for the platform, which is what the dashboard's range controls and
// the IMU_*_RANGE_* defaults in config.h are for.
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
// hardware as configured. `scale` must be the ranges currently on the chip.
inline bool AccelGyroLooksValid(const float accel[3], const float gyro[3], const FullScale& scale) {
    if (!WithinFullScale(accel, scale.accel_mps2) || !WithinFullScale(gyro, scale.gyro_rad_s)) {
        return false;
    }
    return Magnitude(accel) >= kMinPlausibleAccelMps2;
}

inline bool MagLooksValid(const float mag[3], const FullScale& scale) {
    return WithinFullScale(mag, scale.mag_ut);
}

}  // namespace sensor_limits
