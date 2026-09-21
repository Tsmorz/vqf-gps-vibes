#include "vibration.h"

#include <Arduino.h>
#include <string.h>

#include "config.h"

namespace {

// Guards the ring against the two cores that touch it. The writer holds it for
// seven stores; the reader holds it for a 3 kB memcpy, four times a second,
// which is single-digit microseconds out of the estimator's 5000 us tick.
// Short and non-blocking either way, so a spinlock is the right primitive --
// the same reasoning as state_lock in estimator.cpp.
portMUX_TYPE ring_lock = portMUX_INITIALIZER_UNLOCKED;

// Both sensors are buffered even though only one is transformed, so that
// flipping the source shows the *same* excitation from the other sensor
// instead of starting a fresh 1.28 s window. Tapping something once and
// reading the accelerometer's answer against the gyroscope's is most of the
// point of having both.
float ring_accel[spectrum::kSamples][3];
float ring_gyro[spectrum::kSamples][3];
uint32_t ring_us[spectrum::kSamples];
uint32_t written = 0;  // samples since the window was last restarted

volatile bool enabled = false;
volatile SpectrumSource source = SpectrumSource::kAccel;

// Everything below is touched only from loop(). Static rather than local
// because together it is 7 kB, and the Arduino loop task's stack is 8 kB.
float window[spectrum::kSamples][3];
float axis_samples[spectrum::kSamples];
spectrum::Workspace workspace;
SpectrumSnapshot latest;
bool latest_pending = false;
uint32_t last_compute_ms = 0;

void RestartWindow() {
    portENTER_CRITICAL(&ring_lock);
    written = 0;
    portEXIT_CRITICAL(&ring_lock);
}

}  // namespace

void VibrationPushSample(const float accel[3], const float gyro[3], uint32_t now_us) {
    if (!enabled) {
        return;
    }
    portENTER_CRITICAL(&ring_lock);
    const int slot = static_cast<int>(written % spectrum::kSamples);
    for (int axis = 0; axis < 3; axis++) {
        ring_accel[slot][axis] = accel[axis];
        ring_gyro[slot][axis] = gyro[axis];
    }
    ring_us[slot] = now_us;
    written++;
    portEXIT_CRITICAL(&ring_lock);
}

void VibrationNoteGap() {
    if (!enabled) {
        return;
    }
    RestartWindow();
}

void VibrationService() {
    if (!enabled) {
        return;
    }
    const uint32_t now = millis();
    if (now - last_compute_ms < SPECTRUM_INTERVAL_MS) {
        return;
    }
    last_compute_ms = now;

    // Copy the ring out wholesale and do every bit of arithmetic afterwards,
    // so the estimator can only ever be blocked for the memcpy. Only two of
    // the timestamps are needed -- the window's first and last -- so they are
    // read here rather than copied along with it.
    constexpr uint32_t kWindow = static_cast<uint32_t>(spectrum::kSamples);
    uint32_t count = 0;
    uint32_t oldest_us = 0;
    uint32_t newest_us = 0;
    const SpectrumSource wanted = source;
    portENTER_CRITICAL(&ring_lock);
    count = written;
    if (count >= kWindow) {
        memcpy(window, wanted == SpectrumSource::kGyro ? ring_gyro : ring_accel, sizeof(window));
        oldest_us = ring_us[count % kWindow];
        newest_us = ring_us[(count - 1) % kWindow];
    }
    portEXIT_CRITICAL(&ring_lock);

    if (count < kWindow) {
        return;  // still filling after a restart or a dropout
    }

    // Unsigned subtraction, so a micros() rollover every 71 minutes lands on
    // the right answer rather than a negative span.
    const uint32_t span_us = newest_us - oldest_us;
    if (span_us == 0) {
        return;
    }
    const float sample_rate_hz = (spectrum::kSamples - 1) * 1e6f / static_cast<float>(span_us);

    for (int axis = 0; axis < 3; axis++) {
        // De-interleave into chronological order. The ring's oldest sample is
        // not at index 0, and the window has to line up with real time: a Hann
        // window applied in ring order would taper the middle of the record
        // and leave the wrap -- a step between two samples 1.28 s apart --
        // untouched at full amplitude, which is precisely the leakage the
        // window exists to remove. (An unwindowed transform would not care:
        // its magnitudes are invariant to a circular shift. A windowed one is
        // not, and that is easy to talk yourself out of.)
        const int oldest = static_cast<int>(count % kWindow);
        for (int i = 0; i < spectrum::kSamples; i++) {
            axis_samples[i] = window[(oldest + i) % spectrum::kSamples][axis];
        }
        spectrum::Compute(axis_samples, workspace, latest.bins[axis]);
    }

    latest.valid = true;
    latest.source = wanted;
    latest.timestamp_ms = now;
    latest.sample_rate_hz = sample_rate_hz;
    latest_pending = true;
}

bool VibrationTakeSpectrum(SpectrumSnapshot& out) {
    if (!latest_pending) {
        return false;
    }
    latest_pending = false;
    out = latest;
    return true;
}

void VibrationSetEnabled(bool on) {
    if (on == enabled) {
        return;
    }
    // Restart before enabling, not after: the ring still holds whatever was in
    // it when the analyser was last switched off, and publishing a spectrum of
    // minutes-old samples stamped with the current time would be a lie the
    // dashboard has no way to see through.
    RestartWindow();
    enabled = on;
    latest_pending = false;
    latest.valid = false;
    Serial.printf("[spec] spectral analysis %s\n", on ? "on" : "off");
}

void VibrationSetSource(SpectrumSource wanted) {
    if (wanted == source) {
        return;
    }
    source = wanted;
    latest_pending = false;
    latest.valid = false;
}

SpectrumConfig VibrationGetConfig() {
    SpectrumConfig config;
    config.enabled = enabled;
    config.source = source;
    return config;
}
