#include "estimator.h"

#include <Arduino.h>

#include "attitude.h"
#include "baro.h"
#include "board_power.h"
#include "config.h"
#include "geo.h"
#include "gps.h"
#include "gps_quality.h"
#include "i2c_bus.h"
#include "imu.h"
#include "nav_filter.h"
#include "recorder.h"
#include "vibration.h"
#include "vqf.hpp"

namespace {

// Orientation filter. VQF's own rest detection also drives its online gyro
// bias estimation, so no start-up "hold still" calibration is needed here.
VQF vqf(ESTIMATOR_INTERVAL_MS / 1000.0f);

NavFilter nav;
geo::Origin origin;
BaroSample latest_baro;

FilterParams params;
EstimatorSnapshot snapshot;
uint32_t filter_resets = 0;
uint32_t gps_update_count = 0;
uint32_t gps_rejected_count = 0;
// The sigma the last accepted fix was actually applied with, as opposed to the
// knob it was derived from. Published so the dashboard shows the trust the
// filter is really using rather than the one the slider claims.
float gps_applied_sigma_h = 0.0f;

// Whether the filter's position is tied to a fix at all. Distinct from
// gps_update_count, which is a tally for the dashboard: a filter that has just
// been reset has applied plenty of fixes historically and is still unanchored.
// Conflating the two is what let a divergence reset be followed by an
// incremental update against a filter that had just been zeroed.
bool position_anchored = false;
uint32_t gps_consecutive_rejects = 0;

// How long ticks take, as opposed to how often they run. A tick that outlasts
// its period is the direct evidence that core 1 is saturated (usually by I2C
// waits, not arithmetic). Only touched by the estimator task; the published
// values are the last completed one-second window.
struct TickStats {
    float avg_us = 0.0f;
    float max_us = 0.0f;
    uint32_t overruns = 0;
} tick_stats;

void RecordTickDuration(uint32_t busy_us) {
    static uint32_t window_start_ms = 0;
    static uint32_t sum_us = 0;
    static uint32_t count = 0;
    static uint32_t max_us = 0;

    sum_us += busy_us;
    count++;
    max_us = busy_us > max_us ? busy_us : max_us;
    if (busy_us > ESTIMATOR_INTERVAL_MS * 1000u) {
        tick_stats.overruns++;
    }

    const uint32_t now = millis();
    if (now - window_start_ms >= 1000) {
        tick_stats.avg_us = static_cast<float>(sum_us) / count;
        tick_stats.max_us = static_cast<float>(max_us);
        window_start_ms = now;
        sum_us = 0;
        count = 0;
        max_us = 0;
    }
}

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

// Pushes a range change from the dashboard through to the hardware.
//
// The web handler cannot do this itself -- only this task may touch the bus --
// so a change arrives as a new value in FilterParams and is applied on the
// first tick that sees it. ImuSetRanges() does its own change detection, so
// calling this every tick costs three integer comparisons.
void ApplyImuRanges(const FilterParams& active) {
    const bool accel_range_changed =
        ImuSetRanges(active.accel_range_g, active.gyro_range_dps, active.mag_range_gauss);

    // Publish what the chips actually ended up on. A request the hardware does
    // not offer is snapped to the nearest range it does, and the dashboard has
    // to show the real setting -- otherwise its dropdown sits on a value the
    // part is not running, and the saturation limits appear to disagree with
    // the range on screen.
    int accel_g = 0;
    int gyro_dps = 0;
    int mag_gauss = 0;
    ImuGetRanges(accel_g, gyro_dps, mag_gauss);
    if (accel_g != active.accel_range_g || gyro_dps != active.gyro_range_dps ||
        mag_gauss != active.mag_range_gauss) {
        portENTER_CRITICAL(&state_lock);
        params.accel_range_g = accel_g;
        params.gyro_range_dps = gyro_dps;
        params.mag_range_gauss = mag_gauss;
        portEXIT_CRITICAL(&state_lock);
    }

    // Unlike `params`, the filter is this task's alone -- see ResetNavState.
    if (accel_range_changed) {
        nav.ForgetAccelBias();
        Serial.println("[est] accel range changed -- learned accel bias forgotten");
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

// Returns the navigation state to its boot condition.
//
// Shared by the dashboard's reset button and the divergence guard, which used
// to differ: the guard called nav.Reset() on its own, leaving the filter
// anchored in its own bookkeeping, so the next fix arrived as an incremental
// correction to a filter that had just been zeroed. Far from the origin that is
// an enormous innovation and a long settling transient -- precisely what
// anchoring exists to avoid.
//
// Takes no lock. The estimator task calls this from inside its own tick, where
// it already has exclusive use of the filter; EstimatorResetFilter wraps it in
// the critical section instead, because that call arrives from core 0.
//
// `keep_frame` retains the ENU origin: after a divergence the dashboard should
// stay in the frame the user has been watching, whereas the reset button is
// asking for a fresh one. Either way the filter's position has stopped meaning
// anything in that frame, so the next fix must re-anchor.
void ResetNavState(bool keep_frame) {
    nav.Reset();
    latest_baro = BaroSample();
    position_anchored = false;
    gps_consecutive_rejects = 0;
    gps_applied_sigma_h = 0.0f;
    if (!keep_frame) {
        origin = geo::Origin();
        gps_update_count = 0;
        gps_rejected_count = 0;
    }
    filter_resets++;
}

// Applies a GPS fix to the navigation filter.
//
// The very first fix defines the local ENU origin and is snapped to rather
// than blended in: an incremental update from the filter's arbitrary starting
// point would leave a large innovation and a long settling transient.
//
// Returns false if the fix was refused as inconsistent with the filter.
bool ApplyGpsFix(const GpsSample& fix, const FilterParams& active, float sigma_scale) {
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

    // The knobs are what a good fix is worth; sigma_scale is how much worse
    // the receiver says this one is. See gps_quality.h.
    const float sigma_h = active.sigma_gps_pos_h * sigma_scale;
    const float sigma_v = active.sigma_gps_pos_v * sigma_scale;

    // Anchoring asserts the position rather than correcting it, so there is no
    // prior for the fix to disagree with and the gate below cannot apply.
    bool anchor = !position_anchored;

    if (!anchor) {
        const float nis = nav.GpsPositionNis(fix_enu, sigma_h);
        if (nis > GPS_MAX_POSITION_NIS) {
            if (++gps_consecutive_rejects < GPS_MAX_CONSECUTIVE_REJECTS) {
                return false;
            }
            // Several fixes running cannot all be wrong in the same direction.
            // What is wrong by then is the filter's own belief -- so stop
            // arguing with the receiver and re-anchor on it.
            Serial.printf("[est] %u fixes rejected in a row -- re-anchoring on GPS\n",
                          gps_consecutive_rejects);
            anchor = true;
        }
    }

    gps_consecutive_rejects = 0;
    gps_applied_sigma_h = sigma_h;

    if (anchor) {
        nav.SetPosition(fix_enu, sigma_h);
        position_anchored = true;
    } else {
        nav.UpdateGpsPosition(fix_enu, sigma_h, sigma_v);
    }

    if (active.gps_vel_enabled) {
        float east_mps = 0.0f;
        float north_mps = 0.0f;
        geo::CourseToEnuVelocity(fix.speed_mps, fix.course_deg, east_mps, north_mps);
        // Scaled by the same factor: speed and course come out of the same
        // least-squares solution at the same epoch, so the geometry that
        // degraded the position degraded these too.
        nav.UpdateGpsVelocity(east_mps, north_mps, active.sigma_gps_vel * sigma_scale);
    }
    gps_update_count++;
    return true;
}

// Reads the barometer and folds it into the filter.
//
// The first good sample anchors the offset state instead of being blended in:
// the barometer's datum starts out unknown to within the station's elevation,
// and that innovation would drag the height estimate hundreds of metres.
// After that the barometer supplies fine-grained height changes while GPS
// anchors the absolute value and slowly pins the offset down.
//
// Crucially, anchoring waits until the height estimate is worth anchoring to.
// At boot the filter free-runs: VQF has not converged, so gravity does not
// cancel and the height integrates away -- measured at -18 m within seconds of
// power-up. Anchoring against that bakes the error into the offset, and
// because the barometer is trusted an order of magnitude more than GPS
// altitude, it then takes minutes to undo. So the anchor waits for the first
// GPS fix, which snaps height to a known value; failing that, for a timeout,
// by which point VQF has converged and zero-velocity updates hold height
// steady. Indoors with no GPS the timeout is the path that runs, and the
// barometer is then the only height reference there is.
void ServiceBaro(const FilterParams& active) {
    BaroSample sample;
    if (!BaroRead(sample)) {
        return;
    }
    latest_baro = sample;

    if (!active.baro_enabled) {
        return;
    }
    if (!nav.baro_anchored()) {
        const bool height_is_meaningful = origin.valid || millis() > BARO_ANCHOR_FALLBACK_MS;
        if (!height_is_meaningful) {
            return;
        }
        nav.SetBaroBias(sample.pressure_altitude_m, BARO_ANCHOR_SIGMA);
        Serial.printf("[est] barometer anchored at %.1f m pressure altitude (%s)\n",
                      sample.pressure_altitude_m,
                      origin.valid ? "on the first GPS fix" : "no GPS -- using the timeout");
        return;
    }
    nav.UpdateBaroAltitude(sample.pressure_altitude_m, active.sigma_baro);
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
    if (!GpsConsumeNewFix(fix)) {
        return;
    }

    // With GPS fusion switched off the fix is drained and dropped here.
    //
    // It is deliberately not counted as a rejection -- nothing is wrong with
    // it -- and the poll above deliberately still runs: GpsLatest() keeps
    // feeding satellites, HDOP and fix state to the dashboard, so the receiver
    // still looks alive rather than looking like a hardware fault. Returning
    // before ApplyGpsFix also leaves the local frame unanchored, which is what
    // the dashboard reads to know the position panels have no source.
    if (!active.gps_enabled) {
        return;
    }

    // A fix describes where the board was when the receiver solved it, not
    // where it is now, and it is applied as though it were current. Normally
    // this tick drains and applies in one go, so the age is a millisecond or
    // two; it only grows if the estimator falls behind. Past GPS_STALE_MS the
    // board could have gone anywhere and the fix says more about the past than
    // the present.
    //
    // What this cannot see is the delay inside the receiver and on the wire,
    // between the epoch the fix describes and the sentence reaching us. That
    // needs the PA1010D's PPS output wired to an interrupt; until then the
    // sample carries the receiver's own UTC epoch so the lag is at least
    // measurable from a recording.
    if (millis() - fix.fix_millis > GPS_STALE_MS) {
        gps_rejected_count++;
        return;
    }

    // A fix the receiver has described as unusable is dropped here rather than
    // inside ApplyGpsFix, so that it cannot anchor the local frame either. The
    // origin is the reference every plotted position is relative to, and the
    // barometer's anchor waits on it -- neither should be defined by a fix the
    // receiver itself does not stand behind.
    const GpsTrust trust = GpsFixTrust(fix.satellites, fix.hdop);
    if (!trust.accepted) {
        gps_rejected_count++;
        return;
    }

    // Reported quality describes the geometry; this is the fix disagreeing
    // with the filter's own prediction, which is the only thing that catches
    // multipath. See NavFilter::GpsPositionNis().
    if (!ApplyGpsFix(fix, active, trust.sigma_scale)) {
        gps_rejected_count++;
    }
}

// Set by EstimatorSetAuxPower() from core 0, acted on by the estimator task.
volatile bool aux_power_requested = true;
bool aux_power = true;
uint32_t aux_ready_ms = 0;  // when a freshly powered rail has settled; 0 = settled

// Applies a pending change to the LDO2 rail. Never blocks: the sensors' boot
// time is waited out by holding back the aux drivers, not by delaying the tick
// that also carries the IMU.
void ServiceAuxPower() {
    const bool wanted = aux_power_requested;
    if (wanted != aux_power) {
        aux_power = wanted;
        BoardPowerAuxSet(wanted);
        if (wanted) {
            aux_ready_ms = millis() + kAuxRailSettleMs;
            Serial.println("[pwr] LDO2 on -- GPS and barometer powering up");
        } else {
            aux_ready_ms = 0;
            Serial.println("[pwr] LDO2 off -- GPS and barometer unpowered");
        }
    }
    if (aux_ready_ms != 0 && static_cast<int32_t>(millis() - aux_ready_ms) >= 0) {
        aux_ready_ms = 0;
        // The drivers' handles predate the power cut. Bumping the generation
        // makes both re-initialise, and restarting the transfer clock stops the
        // watchdog reading the silent off period as a dead bus and power-cycling.
        I2cRestart(I2cBus::kAux);
        I2cNoteTransferOk(I2cBus::kAux);
    }
}

// True when the GPS and barometer may be talked to.
bool AuxUsable() {
    return aux_power && aux_ready_ms == 0;
}

// Copies the current state into the published snapshot.
void PublishSnapshot(const ImuSample& sample, const float quat[4], float loop_hz, uint32_t now_us) {
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

    const MagCalStatus mag_status = ImuMagCalStatus();
    next.mag_calibrated = mag_status.calibrated;
    next.mag_collecting = mag_status.collecting;
    next.mag_cal_progress = mag_status.progress;
    next.mag_cal_implied_ut = mag_status.implied_field_ut;
    next.mag_field_ut = mag_status.field_ut;

    const GpsSample fix = GpsLatest();
    next.gps_fix = aux_power && fix.has_fix;
    next.gps_satellites = fix.satellites;
    next.gps_hdop = fix.hdop;
    next.gps_lat = fix.lat_deg;
    next.gps_lon = fix.lon_deg;
    next.gps_alt_m = fix.alt_m;
    next.gps_fix_age_ms = fix.fix_millis == 0 ? 0 : next.timestamp_ms - fix.fix_millis;
    next.gps_update_count = gps_update_count;
    next.gps_rejected_count = gps_rejected_count;
    next.gps_sigma_h_m = gps_applied_sigma_h;
    next.gps_epoch_tod_ms = fix.epoch_tod_ms;
    next.gps_epoch_valid = fix.epoch_valid;
    if (fix.has_fix && origin.valid) {
        geo::ToEnu(origin, fix.lat_deg, fix.lon_deg, fix.alt_m, next.gps_enu);
        next.gps_enu_valid = true;
    }

    next.origin_valid = origin.valid;
    next.origin_lat = origin.lat_deg;
    next.origin_lon = origin.lon_deg;
    next.origin_alt_m = origin.alt_m;

    next.aux_power = aux_power;
    next.baro_healthy = aux_power && BaroHealthy();
    next.baro_pressure_pa = latest_baro.pressure_pa;
    next.baro_temperature_c = latest_baro.temperature_c;
    next.baro_altitude_m = latest_baro.pressure_altitude_m;
    next.baro_bias_m = nav.state(NavFilter::kBaroBias);
    next.baro_bias_sigma3 = nav.ThreeSigma(NavFilter::kBaroBias);
    next.baro_height_m = latest_baro.pressure_altitude_m - next.baro_bias_m;
    next.baro_failures = BaroFailureCount();

    next.imu_healthy = ImuAccelGyroHealthy();
    next.mag_healthy = ImuMagHealthy();
    next.gps_healthy = aux_power && GpsHealthy();
    next.imu_failures = ImuFailureCount();
    next.gps_failures = GpsFailureCount();
    next.filter_resets = filter_resets;
    next.estimator_hz = loop_hz;
    next.tick_busy_avg_us = tick_stats.avg_us;
    next.tick_busy_max_us = tick_stats.max_us;
    next.tick_overruns = tick_stats.overruns;

    portENTER_CRITICAL(&state_lock);
    snapshot = next;
    portEXIT_CRITICAL(&state_lock);

    // The record is the snapshot just published, stamped with the tick's own
    // clock reading, so a recording and the dashboard can never disagree about
    // what a tick contained. Free while not recording: it returns on a flag.
    RecorderPushTick(next, now_us);
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
//
// `now_us` is the caller's own micros() reading, passed in rather than taken
// again here so the spectrum's sample timestamps agree exactly with the dt the
// filter integrated -- the frequency axis is derived from them.
void Tick(float dt, uint32_t now_us) {
    const FilterParams active = ReadParams();
    ApplyVqfTuning(active);

    ImuSample sample;
    const bool imu_ok = ImuRead(sample);

    // Strictly after ImuRead, which is what services the bus watchdog and
    // re-initialises the chips after a recovery. Writing a new range before
    // that runs would push it through a device handle whose bus has already
    // been torn down -- the ESP_ERR_INVALID_STATE case -- and the Adafruit
    // driver would report the write as successful.
    //
    // It also keeps the sample and the limits it was checked against in step:
    // `sample` was taken on the old range, so the new full scale must not
    // apply until the next read.
    ApplyImuRanges(active);

    // Orientation is integrated, so its state means nothing across a gap in
    // the data. Starting VQF over costs a few seconds of re-convergence and
    // avoids carrying a stale attitude into the navigation filter.
    if (ImuConsumeReconnectEvent()) {
        vqf.resetState();
        Serial.println("[est] IMU back -- orientation filter restarted");
    }

    // Raw, before VQF or the EKF touch it: the spectrum panel is looking for
    // what the structure did, not for what the filters made of it. A failed
    // read restarts the window rather than feeding it the driver's (0, 0, 0) --
    // that would be a step into silence and would read as broadband content
    // across the whole band.
    if (imu_ok) {
        VibrationPushSample(sample.accel, sample.gyro, now_us);
    } else {
        VibrationNoteGap();
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
        process_noise.sigma_baro_bias = active.sigma_baro_bias;
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
        process_noise.sigma_baro_bias = active.sigma_baro_bias;
        nav.PredictCoasting(dt, process_noise);
    }

    // GPS first: its first fix defines the local frame and snaps height, and
    // the barometer anchors against that same tick's value.
    ServiceAuxPower();
    if (AuxUsable()) {
        ServiceGps(active);
        ServiceBaro(active);
    }

    // Numerical trouble here would publish NaNs to every plot on the
    // dashboard; starting over is both safer and easier to interpret.
    if (nav.IsDiverged()) {
        Serial.println("[est] filter diverged -- resetting");
        ResetNavState(/*keep_frame=*/true);
    }

    PublishSnapshot(sample, quat, MeasureLoopRate(), now_us);
}

// Set by EstimatorPrepareSleep() from core 0, acted on by the estimator task,
// which alone may touch the buses.
volatile bool sleep_requested = false;
volatile bool sleep_ready = false;

// Powers the sensors down, then parks the task so nothing wakes them again.
[[noreturn]] void ParkForSleep() {
    ImuPowerDown();
    sleep_ready = true;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

void EstimatorTask(void* /*argument*/) {
    // Bring the bus up from here rather than from setup(). The I2C driver
    // allocates its interrupt on whichever core installs it, and setup() runs
    // on core 0 -- so every transfer completion would interrupt the WiFi core
    // and then wake this task across cores, adding jitter to the 200 Hz tick.
    I2cBeginAll();
    ImuBegin();
    GpsBegin();

    const TickType_t period = pdMS_TO_TICKS(ESTIMATOR_INTERVAL_MS);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_tick_us = micros();

    for (;;) {
        const uint32_t now_us = micros();
        const float dt = (now_us - last_tick_us) * 1e-6f;
        last_tick_us = now_us;

        if (sleep_requested) {
            ParkForSleep();
        }
        Tick(dt, now_us);
        RecordTickDuration(micros() - now_us);
        vTaskDelayUntil(&last_wake, period);
    }
}

}  // namespace

bool EstimatorBegin() {
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
    ResetNavState(/*keep_frame=*/false);
    portEXIT_CRITICAL(&state_lock);
    Serial.println("[est] filter and local frame reset");
}

void EstimatorSetAuxPower(bool on) {
    aux_power_requested = on;
}

void EstimatorPrepareSleep() {
    constexpr uint32_t kTimeoutMs = 500;
    sleep_requested = true;
    const uint32_t start_ms = millis();
    while (!sleep_ready && millis() - start_ms < kTimeoutMs) {
        delay(1);
    }
    if (!sleep_ready) {
        Serial.println("[est] estimator did not confirm IMU power-down in time; sleeping anyway");
    }
}
