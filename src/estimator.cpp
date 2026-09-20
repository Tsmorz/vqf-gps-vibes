#include "estimator.h"

#include <Arduino.h>

#include "attitude.h"
#include "config.h"
#include "geo.h"
#include "gps.h"
#include "i2c_bus.h"
#include "imu.h"
#include "nav_filter.h"
#include "vqf.hpp"

namespace {

// Orientation filter. VQF's own rest detection also drives its online gyro
// bias estimation, so no start-up "hold still" calibration is needed here.
VQF vqf(ESTIMATOR_INTERVAL_MS / 1000.0f);

NavFilter nav;
geo::Origin origin;

FilterParams params;
EstimatorSnapshot snapshot;
uint32_t filter_resets = 0;
uint32_t gps_update_count = 0;

// Guards `params` (written from the web handler on core 0) and `snapshot`
// (written by the estimator task on core 1). Both are short, non-blocking
// critical sections, so a spinlock is the right primitive.
portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

// The VQF time constants last pushed into the filter. Applying them is not
// free -- it recomputes filter coefficients -- so it only happens on change.
float applied_tau_acc = -1.0f;
float applied_tau_mag = -1.0f;

FilterParams ReadParams() {
    portENTER_CRITICAL(&state_lock);
    const FilterParams copy = params;
    portEXIT_CRITICAL(&state_lock);
    return copy;
}

void ApplyVqfTuning(const FilterParams& active) {
    if (active.tau_acc != applied_tau_acc) {
        vqf.setTauAcc(active.tau_acc);
        applied_tau_acc = active.tau_acc;
    }
    if (active.tau_mag != applied_tau_mag) {
        vqf.setTauMag(active.tau_mag);
        applied_tau_mag = active.tau_mag;
    }
}

// Feeds one IMU sample to VQF. With a healthy magnetometer this is the full 9D
// update; without one, VQF still produces a 6D solution whose roll and pitch
// are just as good and whose yaw is free to drift.
void UpdateOrientation(const ImuSample& sample) {
    const vqf_real_t gyro[3] = {sample.gyro[0], sample.gyro[1], sample.gyro[2]};
    const vqf_real_t accel[3] = {sample.accel[0], sample.accel[1], sample.accel[2]};

    if (sample.mag_valid) {
        const vqf_real_t mag[3] = {sample.mag[0], sample.mag[1], sample.mag[2]};
        vqf.update(gyro, accel, mag);
    } else {
        vqf.update(gyro, accel);
    }
}

// Applies a GPS fix to the navigation filter.
//
// The very first fix defines the local ENU origin and is snapped to rather
// than blended in: an incremental update from the filter's arbitrary starting
// point would leave a large innovation and a long settling transient.
void ApplyGpsFix(const GpsSample& fix, const FilterParams& active) {
    if (!origin.valid) {
        origin.valid = true;
        origin.lat_deg = fix.lat_deg;
        origin.lon_deg = fix.lon_deg;
        origin.alt_m = fix.alt_m;
        Serial.printf("[est] local frame anchored at %.6f, %.6f (%.1f m)\n", fix.lat_deg,
                      fix.lon_deg, fix.alt_m);
    }

    float fix_enu[3];
    geo::ToEnu(origin, fix.lat_deg, fix.lon_deg, fix.alt_m, fix_enu);

    if (gps_update_count == 0) {
        nav.SetPosition(fix_enu, active.sigma_gps_pos_h);
    } else {
        nav.UpdateGpsPosition(fix_enu, active.sigma_gps_pos_h, active.sigma_gps_pos_v);
    }

    if (active.gps_vel_enabled) {
        float east_mps = 0.0f;
        float north_mps = 0.0f;
        geo::CourseToEnuVelocity(fix.speed_mps, fix.course_deg, east_mps, north_mps);
        nav.UpdateGpsVelocity(east_mps, north_mps, active.sigma_gps_vel);
    }
    gps_update_count++;
}

// Polls the GPS and folds any new fix into the filter. Runs at a fraction of
// the estimator rate -- the receiver only produces a fix once a second.
void ServiceGps(const FilterParams& active) {
    static uint32_t last_poll_ms = 0;
    const uint32_t now = millis();
    if (now - last_poll_ms < GPS_POLL_INTERVAL_MS) {
        return;
    }
    last_poll_ms = now;

    GpsPoll();

    GpsSample fix;
    if (GpsConsumeNewFix(fix)) {
        ApplyGpsFix(fix, active);
    }
}

// Copies the current state into the published snapshot.
void PublishSnapshot(const ImuSample& sample, const float quat[4], float loop_hz) {
    EstimatorSnapshot next;
    next.timestamp_ms = millis();

    for (int axis = 0; axis < 3; axis++) {
        next.accel[axis] = sample.accel[axis];
        next.gyro[axis] = sample.gyro[axis];
        next.mag[axis] = sample.mag[axis];
        next.pos[axis] = nav.state(NavFilter::kPosEast + axis);
        next.vel[axis] = nav.state(NavFilter::kVelEast + axis);
        next.accel_bias[axis] = nav.state(NavFilter::kBiasX + axis);
        next.pos_sigma3[axis] = nav.ThreeSigma(NavFilter::kPosEast + axis);
        next.vel_sigma3[axis] = nav.ThreeSigma(NavFilter::kVelEast + axis);
    }
    for (int i = 0; i < 4; i++) {
        next.quat[i] = quat[i];
    }
    attitude::QuatToEulerDegrees(quat, next.roll_deg, next.pitch_deg, next.yaw_deg);

    vqf_real_t bias[3];
    vqf.getBiasEstimate(bias);
    for (int axis = 0; axis < 3; axis++) {
        next.gyro_bias[axis] = static_cast<float>(bias[axis]);
    }
    next.rest_detected = vqf.getRestDetected();
    next.mag_disturbed = vqf.getMagDistDetected();

    const GpsSample fix = GpsLatest();
    next.gps_fix = fix.has_fix;
    next.gps_satellites = fix.satellites;
    next.gps_hdop = fix.hdop;
    next.gps_lat = fix.lat_deg;
    next.gps_lon = fix.lon_deg;
    next.gps_alt_m = fix.alt_m;
    next.gps_fix_age_ms = fix.fix_millis == 0 ? 0 : next.timestamp_ms - fix.fix_millis;
    next.gps_update_count = gps_update_count;
    if (fix.has_fix && origin.valid) {
        geo::ToEnu(origin, fix.lat_deg, fix.lon_deg, fix.alt_m, next.gps_enu);
        next.gps_enu_valid = true;
    }

    next.origin_valid = origin.valid;
    next.origin_lat = origin.lat_deg;
    next.origin_lon = origin.lon_deg;
    next.origin_alt_m = origin.alt_m;

    next.imu_healthy = ImuAccelGyroHealthy();
    next.mag_healthy = ImuMagHealthy();
    next.gps_healthy = GpsHealthy();
    next.imu_failures = ImuFailureCount();
    next.gps_failures = GpsFailureCount();
    next.filter_resets = filter_resets;
    next.estimator_hz = loop_hz;

    portENTER_CRITICAL(&state_lock);
    snapshot = next;
    portEXIT_CRITICAL(&state_lock);
}

// Measured tick rate, averaged over a second. Shown on the dashboard because a
// sagging rate is the first sign the bus is stalling on a bad cable.
float MeasureLoopRate() {
    static uint32_t window_start_ms = 0;
    static uint32_t ticks_in_window = 0;
    static float measured_hz = 0.0f;

    ticks_in_window++;
    const uint32_t now = millis();
    if (now - window_start_ms >= 1000) {
        measured_hz = ticks_in_window * 1000.0f / (now - window_start_ms);
        window_start_ms = now;
        ticks_in_window = 0;
    }
    return measured_hz;
}

// One estimator tick: read the IMU, update orientation, propagate the
// navigation filter, fold in any measurements, publish.
void Tick(float dt) {
    const FilterParams active = ReadParams();
    ApplyVqfTuning(active);

    ImuSample sample;
    const bool imu_ok = ImuRead(sample);

    // Orientation is integrated, so its state means nothing across a gap in
    // the data. Starting VQF over costs a few seconds of re-convergence and
    // avoids carrying a stale attitude into the navigation filter.
    if (ImuConsumeReconnectEvent()) {
        vqf.resetState();
        Serial.println("[est] IMU back -- orientation filter restarted");
    }

    float quat[4] = {1, 0, 0, 0};
    if (imu_ok) {
        UpdateOrientation(sample);

        vqf_real_t vqf_quat[4];
        vqf.getQuat9D(vqf_quat);
        for (int i = 0; i < 4; i++) {
            quat[i] = static_cast<float>(vqf_quat[i]);
        }

        // Without a valid attitude there is no way to resolve the
        // accelerometer into ENU, so propagation is skipped entirely rather
        // than run on a stale rotation.
        float rotation[9];
        attitude::QuatToRotationMatrix(quat, rotation);

        NavFilter::Params process_noise;
        process_noise.sigma_accel = active.sigma_accel;
        process_noise.sigma_accel_bias = active.sigma_accel_bias;
        nav.Predict(sample.accel, rotation, dt, process_noise);

        if (active.zupt_enabled && vqf.getRestDetected()) {
            nav.UpdateZeroVelocity(active.sigma_zupt);
        }
    } else {
        // No IMU: coast on the last velocity and let the envelope widen. The
        // alternative -- integrating whatever the driver returned -- is how an
        // unplugged sensor reading (0, 0, 0) turns into simulated free fall.
        NavFilter::Params process_noise;
        process_noise.sigma_accel = active.sigma_accel;
        process_noise.sigma_accel_bias = active.sigma_accel_bias;
        nav.PredictCoasting(dt, process_noise);
    }

    ServiceGps(active);

    // Numerical trouble here would publish NaNs to every plot on the
    // dashboard; starting over is both safer and easier to interpret.
    if (nav.IsDiverged()) {
        Serial.println("[est] filter diverged -- resetting");
        nav.Reset();
        filter_resets++;
    }

    PublishSnapshot(sample, quat, MeasureLoopRate());
}

void EstimatorTask(void* /*argument*/) {
    const TickType_t period = pdMS_TO_TICKS(ESTIMATOR_INTERVAL_MS);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_tick_us = micros();

    for (;;) {
        const uint32_t now_us = micros();
        const float dt = (now_us - last_tick_us) * 1e-6f;
        last_tick_us = now_us;

        Tick(dt);
        vTaskDelayUntil(&last_wake, period);
    }
}

}  // namespace

bool EstimatorBegin() {
    I2cBegin();
    ImuBegin();
    GpsBegin();

    const FilterParams initial = ReadParams();
    ApplyVqfTuning(initial);

    // Core 1 is left to the estimator; the Arduino loop and the WiFi stack run
    // on core 0 (see ARDUINO_RUNNING_CORE in platformio.ini). 8 kB of stack is
    // ample -- the largest thing on it is the EKF's 9x9 scratch matrices.
    const BaseType_t created =
        xTaskCreatePinnedToCore(EstimatorTask, "estimator", 8192, nullptr, 5, nullptr, 1);
    if (created != pdPASS) {
        Serial.println("[est] FATAL: could not start the estimator task");
        return false;
    }
    Serial.printf("[est] running at %d Hz on core 1\n", 1000 / ESTIMATOR_INTERVAL_MS);
    return true;
}

void EstimatorCopySnapshot(EstimatorSnapshot& out) {
    portENTER_CRITICAL(&state_lock);
    out = snapshot;
    portEXIT_CRITICAL(&state_lock);
}

FilterParams EstimatorGetParams() {
    return ReadParams();
}

void EstimatorSetParams(const FilterParams& updated) {
    portENTER_CRITICAL(&state_lock);
    params = updated;
    portEXIT_CRITICAL(&state_lock);
}

void EstimatorResetFilter() {
    // Touching the filter from core 0 while the estimator task is mid-tick
    // would corrupt it, so the reset is done inside the same critical section
    // the task uses. Both operations are a few microseconds of memset.
    portENTER_CRITICAL(&state_lock);
    nav.Reset();
    origin = geo::Origin();
    gps_update_count = 0;
    filter_resets++;
    portEXIT_CRITICAL(&state_lock);
    Serial.println("[est] filter and local frame reset");
}
