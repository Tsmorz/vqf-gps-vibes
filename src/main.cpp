// VQF + GPS sensor fusion on an Unexpected Maker FeatherS3.
//
// Orientation comes from VQF (Laidig & Seel, Information Fusion 2023, vendored
// in lib/vqf/); position and velocity come from a 9-state EKF corrected by GPS
// (src/nav_filter.h). The two halves run on separate cores:
//
//   core 1  estimator task -- owns the I2C bus, 200 Hz (src/estimator.cpp)
//   core 0  this file      -- WiFi access point, dashboard, status LED
//
// Connect to the "vqf-gps" network and open http://192.168.4.1/.

#include <Arduino.h>

#include "config.h"
#include "estimator.h"
#include "status_led.h"
#include "web_server.h"

namespace {

// How far below the configured tick rate the estimator may sag before it is
// treated as a warning. Falling behind almost always means the I2C bus is
// stalling on a marginal cable.
constexpr float kMinHealthyEstimatorHz = 0.8f * (1000.0f / ESTIMATOR_INTERVAL_MS);

// Maps sensor health onto the four status-light states.
//
// Losing the accelerometer/gyroscope is fatal to the whole estimate, so that
// is the only error condition. A missing magnetometer (yaw drifts), a missing
// or unfixed GPS (position drifts), or a sagging tick rate are all degraded
// but still useful -- those are warnings.
SystemStatus EvaluateStatus(const EstimatorSnapshot& snapshot) {
    if (!snapshot.imu_healthy) {
        return SystemStatus::kError;
    }
    const bool degraded = !snapshot.mag_healthy || !snapshot.gps_healthy || !snapshot.gps_fix ||
                          snapshot.estimator_hz < kMinHealthyEstimatorHz;
    return degraded ? SystemStatus::kWarning : SystemStatus::kNormal;
}

const char* StatusName(SystemStatus status) {
    switch (status) {
        case SystemStatus::kNormal:
            return "normal";
        case SystemStatus::kWarning:
            return "warning";
        case SystemStatus::kError:
            return "error";
        case SystemStatus::kInitialising:
        default:
            return "init";
    }
}

// Iterations per second of loop() itself. The web server only gets serviced
// as often as this runs, so a sagging value shows up directly as a slow or
// failed WebSocket handshake.
float MeasureLoopRate() {
    static uint32_t window_start_ms = 0;
    static uint32_t iterations = 0;
    static float measured_hz = 0.0f;

    iterations++;
    const uint32_t now = millis();
    if (now - window_start_ms >= 1000) {
        measured_hz = iterations * 1000.0f / (now - window_start_ms);
        window_start_ms = now;
        iterations = 0;
    }
    return measured_hz;
}

float loop_hz = 0.0f;

// One line a second on the USB serial port, so the board can be watched
// without a browser attached.
void LogPeriodically(const EstimatorSnapshot& s) {
    static uint32_t last_log_ms = 0;
    const uint32_t now = millis();
    if (now - last_log_ms < 1000) {
        return;
    }
    last_log_ms = now;

    // The raw GPS position sits next to the estimate on purpose: when the two
    // march off together the receiver is wandering (multipath indoors), and
    // when only the estimate moves the fault is in the filter.
    Serial.printf(
        "[st] %.0fHz loop=%.0fHz rpy=%6.1f %6.1f %6.1f  "
        "pos=%7.2f %7.2f %7.2f  3s=%5.1f %5.1f %5.1f  "
        "vel=%6.2f %6.2f %6.2f  rest=%d  raw=%7.2f %7.2f %7.2f  gps=%s(%u)  "
        "imu=%s mag=%s  clients=%u\n",
        s.estimator_hz, loop_hz, s.roll_deg, s.pitch_deg, s.yaw_deg, s.pos[0], s.pos[1], s.pos[2],
        s.pos_sigma3[0], s.pos_sigma3[1], s.pos_sigma3[2], s.vel[0], s.vel[1], s.vel[2],
        s.rest_detected ? 1 : 0, s.gps_enu[0], s.gps_enu[1], s.gps_enu[2],
        s.gps_fix ? "fix" : (s.gps_healthy ? "searching" : "absent"), s.gps_satellites,
        s.imu_healthy ? "ok" : "LOST", s.mag_healthy ? "ok" : "LOST", WebServerClientCount());
}

}  // namespace

void setup() {
    Serial.begin(115200);

    // Never let logging block the loop.
    //
    // The USB-serial peripheral's write() waits for room in its ring buffer --
    // by default up to 100 ms, retried up to twenty times, so nearly two
    // seconds per call. Nothing drains that buffer unless a serial monitor is
    // open, so once a second the status line below stalled loop() for most of
    // a second and it fell to about 1 Hz. The web server is serviced from
    // loop(), so a WebSocket handshake that takes 20 ms with a monitor
    // attached took fifteen seconds without one -- exactly the untethered case
    // this board is built for. A zero timeout drops log output instead of
    // waiting, which is the right trade for a diagnostic channel.
    Serial.setTxTimeoutMs(0);
    // The native-USB CDC port enumerates after boot; this keeps the banner and
    // any start-up errors from being lost.
    delay(1500);
    Serial.printf("\n=== vqf-gps-vibes %s ===\n", FW_VERSION);

    StatusLedBegin();
    StatusLedSet(SystemStatus::kInitialising);

    if (!EstimatorBegin()) {
        StatusLedSet(SystemStatus::kError);
    }
    WebServerBegin();
}

void loop() {
    loop_hz = MeasureLoopRate();

    WebServerLoop();

    EstimatorSnapshot snapshot;
    EstimatorCopySnapshot(snapshot);

    const SystemStatus status = EvaluateStatus(snapshot);
    StatusLedSet(status);
    StatusLedUpdate();

    WebServerBroadcast(snapshot, EstimatorGetParams(), StatusName(status));
    LogPeriodically(snapshot);
}
