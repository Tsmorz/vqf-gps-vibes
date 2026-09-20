#pragma once

#include <stdint.h>

#include "filter_params.h"

// The real-time estimation half of the firmware.
//
// Runs in its own FreeRTOS task pinned to core 1, away from the WiFi stack, so
// the 200 Hz tick is not jittered by web traffic. That task is also the sole
// owner of the I2C bus: because the IMU and GPS share one STEMMA run, doing
// all transfers from a single task removes any chance of two cores
// interleaving transactions on the same peripheral.
//
// Everything the rest of the firmware needs is published as an immutable
// snapshot, copied out under a spinlock.

// One consistent view of the estimator's state, as of `timestamp_ms`.
struct EstimatorSnapshot {
    uint32_t timestamp_ms = 0;

    // ── Raw sensor data ──────────────────────────────────────────────────
    float accel[3] = {0, 0, 0};  // m/s^2
    float gyro[3] = {0, 0, 0};   // rad/s
    float mag[3] = {0, 0, 0};    // microtesla

    // ── Orientation, from VQF ────────────────────────────────────────────
    float quat[4] = {1, 0, 0, 0};  // (w, x, y, z), body -> ENU
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float gyro_bias[3] = {0, 0, 0};  // rad/s, VQF's online estimate
    bool rest_detected = false;
    bool mag_disturbed = false;

    // ── Navigation state, from the EKF ───────────────────────────────────
    float pos[3] = {0, 0, 0};         // m, local ENU
    float vel[3] = {0, 0, 0};         // m/s, local ENU
    float accel_bias[3] = {0, 0, 0};  // m/s^2, body frame
    float pos_sigma3[3] = {0, 0, 0};  // 3-sigma envelope, m
    float vel_sigma3[3] = {0, 0, 0};  // 3-sigma envelope, m/s

    // ── GPS ──────────────────────────────────────────────────────────────
    bool gps_fix = false;
    uint8_t gps_satellites = 0;
    float gps_hdop = 0.0f;
    double gps_lat = 0.0;
    double gps_lon = 0.0;
    float gps_alt_m = 0.0f;
    float gps_enu[3] = {0, 0, 0};  // the raw fix in the local frame
    bool gps_enu_valid = false;
    uint32_t gps_fix_age_ms = 0;
    uint32_t gps_update_count = 0;  // measurement updates applied since boot

    // ── Local frame origin (first fix) ───────────────────────────────────
    bool origin_valid = false;
    double origin_lat = 0.0;
    double origin_lon = 0.0;
    double origin_alt_m = 0.0;

    // ── Health ───────────────────────────────────────────────────────────
    bool imu_healthy = false;
    bool mag_healthy = false;
    bool gps_healthy = false;
    uint32_t imu_failures = 0;
    uint32_t gps_failures = 0;
    uint32_t filter_resets = 0;
    float estimator_hz = 0.0f;
};

// Initialises the bus and sensors, then starts the estimator task. Returns
// false if the task could not be created.
bool EstimatorBegin();

// Copies the latest snapshot. Safe to call from any core.
void EstimatorCopySnapshot(EstimatorSnapshot& out);

FilterParams EstimatorGetParams();
void EstimatorSetParams(const FilterParams& params);

// Clears the navigation state and the local-frame origin, so the next fix
// re-anchors the plot at the current position.
void EstimatorResetFilter();
