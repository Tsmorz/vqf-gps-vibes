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

    // Random walk of the barometer's altitude offset (m/sqrt(s)).
    float sigma_baro_bias = DEFAULT_SIGMA_BARO_BIAS;

    // ── Measurement noise -- how hard a fix pulls the estimate back ──────
    float sigma_gps_pos_h = DEFAULT_SIGMA_GPS_POS_H;  // m
    float sigma_gps_pos_v = DEFAULT_SIGMA_GPS_POS_V;  // m
    float sigma_gps_vel = DEFAULT_SIGMA_GPS_VEL;      // m/s
    float sigma_zupt = DEFAULT_SIGMA_ZUPT;            // m/s
    float sigma_baro = DEFAULT_SIGMA_BARO;            // m

    // ── VQF orientation time constants ───────────────────────────────────
    float tau_acc = DEFAULT_TAU_ACC;  // s, gravity vs gyro in roll/pitch
    float tau_mag = DEFAULT_TAU_MAG;  // s, magnetometer vs gyro in yaw

    // ── IMU full-scale ranges ────────────────────────────────────────────
    // Hardware settings rather than filter tuning, but they travel with the
    // knobs because they reach the device by the same path: the web handler
    // writes them here and the estimator task applies them over I2C, since it
    // is the only task allowed to touch the bus.
    //
    // Each is snapped to a value its chip supports -- see ImuSetRanges(). The
    // dashboard shows back what the hardware actually ended up on.
    //
    // Widen one when the dashboard reports samples rejected for saturation.
    // It is not free: full scale sets the quantisation step, so 16 g resolves
    // four times as coarsely as 4 g.
    int accel_range_g = IMU_ACCEL_RANGE_G;      // 2, 4, 8, 16
    int gyro_range_dps = IMU_GYRO_RANGE_DPS;    // 125, 250, 500, 1000, 2000
    int mag_range_gauss = IMU_MAG_RANGE_GAUSS;  // 4, 8, 12, 16

    // ── Switches ─────────────────────────────────────────────────────────
    // Zero-velocity updates while VQF reports the board is at rest. On by
    // default: without them, indoor testing with no GPS fix diverges within
    // seconds as accelerometer bias double-integrates.
    bool zupt_enabled = true;
    // Whether GPS reaches the filter at all. Off, the receiver is still
    // drained and still reported -- satellites, HDOP and fix state stay live --
    // but no fix is fused and none anchors the local frame, so the rig runs on
    // inertial dead reckoning plus whatever the barometer and zero-velocity
    // updates contribute. This is the switch for seeing what GPS is actually
    // buying, and for indoor work where every fix is multipath.
    bool gps_enabled = true;
    // Whether GPS ground-speed/course is used as a velocity measurement.
    // Has no effect when gps_enabled is false.
    // Worth turning off when stationary, where course is pure noise.
    bool gps_vel_enabled = true;
    // Whether the barometer contributes height. Turning it off is the clearest
    // way to see what it is buying: the up channel's 3-sigma envelope widens
    // immediately, since GPS altitude alone is roughly twice as noisy.
    bool baro_enabled = true;
};
