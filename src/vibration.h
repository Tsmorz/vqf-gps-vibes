#pragma once

#include <stdint.h>

#include "spectrum.h"

// The spectral analyser behind the dashboard's Spectrum panel.
//
// Off by default, and free while off: the estimator returns from
// VibrationPushSample() on a flag check, no transform runs, and no frame is
// encoded. Nothing here is on the path of a board nobody is looking at.
//
// The split across cores is the point of the module. The ring is filled from
// the estimator task on core 1, one sample per 200 Hz tick, because that task
// is the only thing that ever sees an IMU sample and the only thing allowed to
// touch the bus. The transform runs from loop() on core 0, where a few hundred
// microseconds of arithmetic cannot jitter the tick. The two meet at a
// spinlock held for a memcpy.

enum class SpectrumSource : uint8_t { kAccel, kGyro };

// What the analyser is currently doing, echoed in the telemetry frame so a
// browser connecting late shows the device's state instead of its own
// defaults -- the same reason the filter knobs are echoed.
struct SpectrumConfig {
    bool enabled = false;
    SpectrumSource source = SpectrumSource::kAccel;
};

// One computed spectrum: three axes of the selected sensor.
struct SpectrumSnapshot {
    bool valid = false;
    SpectrumSource source = SpectrumSource::kAccel;
    uint32_t timestamp_ms = 0;
    // Measured from the window's own timestamps rather than assumed to be the
    // nominal tick rate. The frequency axis is only as true as this number, so
    // an estimator running slow has to relabel every peak rather than quietly
    // report them at the wrong frequency.
    float sample_rate_hz = 0.0f;
    float bins[3][spectrum::kBins] = {};
};

// Appends one tick's sample to the window. Called from the estimator task,
// which must pass the same `now_us` it is already reading for its own dt.
void VibrationPushSample(const float accel[3], const float gyro[3], uint32_t now_us);

// Discards the window in progress. Called when the IMU read failed: a gap in
// the samples is a step the transform reads as broadband content, and the
// timestamps either side of it would understate the sample rate and so
// mislabel every frequency on the axis.
void VibrationNoteGap();

// Computes a spectrum if one is due and the window has filled. Called from
// loop(); does nothing while the analyser is off.
void VibrationService();

// Hands over the latest spectrum if one has been computed since the last call,
// so each is broadcast exactly once. Returns false when there is nothing new.
bool VibrationTakeSpectrum(SpectrumSnapshot& out);

// Switching on restarts the window -- a spectrum spanning the moment it was
// switched on would be half stale. Switching source deliberately does not:
// both sensors are buffered so that the next spectrum covers the same 1.28 s
// the previous one did, which is what makes it possible to tap something once
// and read the accelerometer's answer against the gyroscope's.
void VibrationSetEnabled(bool enabled);
void VibrationSetSource(SpectrumSource source);

SpectrumConfig VibrationGetConfig();
