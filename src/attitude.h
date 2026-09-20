#pragma once

// Quaternion helpers shared by the estimator and the telemetry encoder.
//
// VQF's 9D quaternion rotates a vector from the sensor (body) frame into its
// global frame. That frame has z opposite gravity and, once the magnetometer
// correction has converged, the horizontal magnetic field lying along +y --
// so it is ENU with y pointing at *magnetic* north. The navigation filter
// works in the same ENU convention, so the quaternion's rotation matrix can be
// used directly with no axis permutation.
//
// The one caveat is declination: y is magnetic north, not true north. Over the
// short baselines this filter operates on, that shows up as a fixed rotation
// of the whole trajectory, not as drift.
//
// Pure math, no Arduino dependency -- unit tested on the host.

#include <math.h>

namespace attitude {

// Builds the row-major 3x3 rotation matrix that takes a body-frame vector into
// the ENU frame, from a quaternion ordered (w, x, y, z) as VQF returns it.
inline void QuatToRotationMatrix(const float q[4], float out[9]) {
    const float w = q[0], x = q[1], y = q[2], z = q[3];

    out[0] = 1.0f - 2.0f * (y * y + z * z);
    out[1] = 2.0f * (x * y - w * z);
    out[2] = 2.0f * (x * z + w * y);

    out[3] = 2.0f * (x * y + w * z);
    out[4] = 1.0f - 2.0f * (x * x + z * z);
    out[5] = 2.0f * (y * z - w * x);

    out[6] = 2.0f * (x * z - w * y);
    out[7] = 2.0f * (y * z + w * x);
    out[8] = 1.0f - 2.0f * (x * x + y * y);
}

// Converts the same quaternion to roll/pitch/yaw in degrees, for the readouts
// on the dashboard. The pitch argument is clamped because accumulated rounding
// can push it a hair outside asin's domain and produce a NaN.
inline void QuatToEulerDegrees(const float q[4], float& roll_deg, float& pitch_deg,
                               float& yaw_deg) {
    const float w = q[0], x = q[1], y = q[2], z = q[3];

    const float roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));

    float sin_pitch = 2.0f * (w * y - z * x);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    const float pitch = asinf(sin_pitch);

    const float yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

    const float to_degrees = 180.0f / static_cast<float>(M_PI);
    roll_deg = roll * to_degrees;
    pitch_deg = pitch * to_degrees;
    yaw_deg = yaw * to_degrees;
}

}  // namespace attitude
