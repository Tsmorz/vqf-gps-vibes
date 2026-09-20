#pragma once

// Magnetometer hard-iron and soft-iron calibration.
//
// A raw magnetometer does not measure the Earth's field; it measures the
// Earth's field plus whatever the board itself contributes. On this hardware
// that contribution is large -- the uncalibrated LIS3MDL reads about 86 uT
// where the true local field is about 48 uT, nearly all of the excess being a
// fixed offset on one axis. VQF cannot remove it: its magnetic disturbance
// rejection is designed for transient anomalies in a field it assumes is
// otherwise homogeneous, whereas a hard-iron offset is fixed in the sensor
// frame and rotates with it.
//
// Left uncorrected this does not affect roll and pitch, which come from
// gravity, but it distorts the horizontal component that heading is derived
// from -- and therefore shows up as a noisy, orientation-dependent yaw.
//
// The correction is the standard one: subtract the centre of the measurement
// ellipsoid (hard iron), then rescale each axis to equalise its span (soft
// iron). This is the per-axis approximation rather than a full ellipsoid fit;
// it captures the dominant error for a few lines of arithmetic, and the
// residual is well below the LIS3MDL's own noise.
//
// Pure math, no Arduino dependency -- unit tested on the host.

#include <math.h>

// A full rotation exposes each axis to +|B| and -|B|, so a complete sweep
// spans 2|B| on every axis -- roughly 100 uT in mid-latitudes, not 50. An
// earlier version required only 30 uT here and reported "100%" when barely a
// third of the needed range had been covered, so sweeps were being accepted
// that left the field strength 20% short.
//
// The Earth's field is between 25 and 65 uT everywhere on the surface, so a
// genuine sweep cannot imply less than 25 uT. That is the floor used here; it
// is a physical bound rather than a guess, and it is deliberately the *global*
// minimum so the check never rejects a valid sweep somewhere weak.
constexpr float kMagCalMinFieldUt = 25.0f;
constexpr float kMagCalMinSpanUt = 2.0f * kMagCalMinFieldUt;

// How isotropic the coverage must be. Rotating mostly about one axis leaves
// the others under-swept, and their spans give it away.
constexpr float kMagCalMinSpanRatio = 0.75f;

struct MagCalibration {
    bool valid = false;
    float offset[3] = {0.0f, 0.0f, 0.0f};  // hard iron, microtesla
    float scale[3] = {1.0f, 1.0f, 1.0f};   // soft iron, dimensionless

    // Corrects a reading in place. A no-op until a calibration has been fitted.
    void Apply(float mag[3]) const {
        if (!valid) {
            return;
        }
        for (int axis = 0; axis < 3; axis++) {
            mag[axis] = (mag[axis] - offset[axis]) * scale[axis];
        }
    }
};

// Collects the per-axis extremes seen during a calibration sweep.
//
// Only the bounding box is kept rather than the samples themselves: the fit
// below needs nothing more, and it costs six floats instead of an array that
// would have to be sized for however long the user keeps rotating.
class MagCalCollector {
   public:
    MagCalCollector() {
        Reset();
    }

    void Reset() {
        for (int axis = 0; axis < 3; axis++) {
            min_[axis] = INFINITY;
            max_[axis] = -INFINITY;
        }
        sample_count_ = 0;
    }

    void Add(const float mag[3]) {
        for (int axis = 0; axis < 3; axis++) {
            if (mag[axis] < min_[axis]) {
                min_[axis] = mag[axis];
            }
            if (mag[axis] > max_[axis]) {
                max_[axis] = mag[axis];
            }
        }
        sample_count_++;
    }

    int sample_count() const {
        return sample_count_;
    }

    float span(int axis) const {
        const float range = max_[axis] - min_[axis];
        return isfinite(range) && range > 0.0f ? range : 0.0f;
    }

    // The field strength this sweep implies so far, in microtesla.
    //
    // This is the number to watch while sweeping: it rises as the extremes are
    // reached and plateaus once the sphere is covered. Stopping before it
    // plateaus is what produces an offset that is confidently wrong, and no
    // progress bar can detect that on its own -- only the operator can see
    // that the number has stopped climbing.
    float implied_field_ut() const {
        return (span(0) + span(1) + span(2)) / 6.0f;
    }

    // How far along the sweep is, as 0..1. Driven by the worst axis against
    // the physical floor, and by how isotropic the coverage is -- both have to
    // be satisfied, and the lower of the two is the honest answer.
    float progress() const {
        float worst = span(0);
        float best = span(0);
        for (int axis = 1; axis < 3; axis++) {
            worst = span(axis) < worst ? span(axis) : worst;
            best = span(axis) > best ? span(axis) : best;
        }
        const float against_floor = worst / kMagCalMinSpanUt;
        const float isotropy = best > 0.0f ? (worst / best) / kMagCalMinSpanRatio : 0.0f;
        const float fraction = against_floor < isotropy ? against_floor : isotropy;
        return fraction > 1.0f ? 1.0f : fraction;
    }

    // Fits the calibration. Fails while any axis is still under-rotated,
    // which is what stops a sweep that only spun about one axis from being
    // accepted -- that would leave two axes uncorrected.
    bool Solve(MagCalibration& out) const {
        float worst = span(0);
        float best = span(0);
        for (int axis = 0; axis < 3; axis++) {
            if (span(axis) < kMagCalMinSpanUt) {
                return false;
            }
            worst = span(axis) < worst ? span(axis) : worst;
            best = span(axis) > best ? span(axis) : best;
        }
        // Reject a sweep that went round one axis far more than the others.
        if (worst < kMagCalMinSpanRatio * best) {
            return false;
        }

        const float mean_span = (span(0) + span(1) + span(2)) / 3.0f;
        for (int axis = 0; axis < 3; axis++) {
            out.offset[axis] = 0.5f * (min_[axis] + max_[axis]);
            out.scale[axis] = mean_span / span(axis);
        }
        out.valid = true;
        return true;
    }

   private:
    float min_[3];
    float max_[3];
    int sample_count_;
};
