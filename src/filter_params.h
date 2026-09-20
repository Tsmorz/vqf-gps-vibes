#pragma once

// The tuning knobs the dashboard exposes.
//
// These are the terms that most visibly change the filter's behaviour: the two
// process-noise densities set how fast uncertainty grows between fixes, and
// the measurement sigmas set how hard each fix pulls the estimate back. They
// are shared between the web handler (core 0) and the estimator task (core 1)
// under a spinlock -- see estimator.cpp.

#include "config.h"

struct FilterParams {
    // ── Process noise -- how fast the 3-sigma envelope opens up ──────────
    float sigma_accel = DEFAULT_SIGMA_ACCEL;            // m/s^2/sqrt(Hz)
    float sigma_accel_bias = DEFAULT_SIGMA_ACCEL_BIAS;  // m/s^3/sqrt(Hz)

    // ── Measurement noise -- how hard a fix pulls the estimate back ──────
    float sigma_gps_pos_h = DEFAULT_SIGMA_GPS_POS_H;  // m
    float sigma_gps_pos_v = DEFAULT_SIGMA_GPS_POS_V;  // m
    float sigma_gps_vel = DEFAULT_SIGMA_GPS_VEL;      // m/s
    float sigma_zupt = DEFAULT_SIGMA_ZUPT;            // m/s

    // ── VQF orientation time constants ───────────────────────────────────
    float tau_acc = DEFAULT_TAU_ACC;  // s, gravity vs gyro in roll/pitch
    float tau_mag = DEFAULT_TAU_MAG;  // s, magnetometer vs gyro in yaw

    // ── Switches ─────────────────────────────────────────────────────────
    // Zero-velocity updates while VQF reports the board is at rest. On by
    // default: without them, indoor testing with no GPS fix diverges within
    // seconds as accelerometer bias double-integrates.
    bool zupt_enabled = true;
    // Whether GPS ground-speed/course is used as a velocity measurement.
    // Worth turning off when stationary, where course is pure noise.
    bool gps_vel_enabled = true;
};
