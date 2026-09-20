// Host-side tests for the magnetometer hard/soft-iron calibration.
//
// These work from a synthetic magnetometer: a known Earth field, rotated
// through many orientations, then corrupted with the exact errors the
// calibration exists to remove. A correct fit recovers the original field
// strength no matter which way the sensor is pointing.

#include <math.h>
#include <unity.h>

#include "mag_cal.h"

namespace {

// Roughly the field strength in central Europe.
constexpr float kFieldUt = 48.6f;

// A hard-iron offset of the size actually measured on this board, plus a
// soft-iron scale error on one axis.
constexpr float kTrueOffset[3] = {2.6f, -6.9f, -42.0f};
constexpr float kTrueScale[3] = {1.0f, 1.15f, 0.9f};

// What the uncalibrated sensor would report for a field pointing in the
// direction (theta, phi): the true field distorted by the board's own iron.
void SimulateReading(float theta, float phi, float out[3]) {
    const float field[3] = {
        kFieldUt * sinf(theta) * cosf(phi),
        kFieldUt * sinf(theta) * sinf(phi),
        kFieldUt * cosf(theta),
    };
    for (int axis = 0; axis < 3; axis++) {
        out[axis] = field[axis] / kTrueScale[axis] + kTrueOffset[axis];
    }
}

// Sweeps the whole sphere, as a thorough calibration rotation would.
void SweepFullSphere(MagCalCollector& collector) {
    for (int i = 0; i <= 18; i++) {
        for (int j = 0; j < 36; j++) {
            float reading[3];
            SimulateReading(static_cast<float>(M_PI) * i / 18.0f,
                            2.0f * static_cast<float>(M_PI) * j / 36.0f, reading);
            collector.Add(reading);
        }
    }
}

}  // namespace

void setUp() {
}
void tearDown() {
}

// An unfitted calibration must leave readings untouched rather than apply
// garbage -- the board runs uncalibrated until the user sweeps it.
void test_uncalibrated_is_a_passthrough() {
    MagCalibration cal;
    float reading[3] = {1.0f, 2.0f, 3.0f};
    cal.Apply(reading);

    TEST_ASSERT_EQUAL_FLOAT(1.0f, reading[0]);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, reading[1]);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, reading[2]);
}

// The headline property: after a full sweep, a corrected reading has the true
// field strength whichever way the sensor points. That constant magnitude is
// exactly what a heading estimate needs.
void test_full_sweep_recovers_the_true_field() {
    MagCalCollector collector;
    SweepFullSphere(collector);

    MagCalibration cal;
    TEST_ASSERT_TRUE(collector.Solve(cal));
    TEST_ASSERT_TRUE(cal.valid);

    for (int i = 0; i <= 8; i++) {
        for (int j = 0; j < 12; j++) {
            float reading[3];
            SimulateReading(static_cast<float>(M_PI) * i / 8.0f,
                            2.0f * static_cast<float>(M_PI) * j / 12.0f, reading);
            cal.Apply(reading);
            const float magnitude =
                sqrtf(reading[0] * reading[0] + reading[1] * reading[1] + reading[2] * reading[2]);
            TEST_ASSERT_FLOAT_WITHIN(1.5f, kFieldUt, magnitude);
        }
    }
}

// The fitted offset must be the hard-iron offset that was injected.
void test_fit_recovers_the_hard_iron_offset() {
    MagCalCollector collector;
    SweepFullSphere(collector);

    MagCalibration cal;
    TEST_ASSERT_TRUE(collector.Solve(cal));
    for (int axis = 0; axis < 3; axis++) {
        TEST_ASSERT_FLOAT_WITHIN(1.0f, kTrueOffset[axis], cal.offset[axis]);
    }
}

// Spinning about one axis only leaves the other two unobserved. Accepting that
// would bake in a wrong correction, so it has to be refused.
void test_single_axis_rotation_is_refused() {
    MagCalCollector collector;
    for (int j = 0; j < 72; j++) {
        float reading[3];
        SimulateReading(static_cast<float>(M_PI) / 2.0f,
                        2.0f * static_cast<float>(M_PI) * j / 72.0f, reading);
        collector.Add(reading);
    }

    MagCalibration cal;
    TEST_ASSERT_FALSE_MESSAGE(collector.Solve(cal),
                              "a yaw-only spin must not be accepted as a calibration");
    TEST_ASSERT_FALSE(cal.valid);
}

// Progress has to reach 1.0 for a full sweep and stay near 0 for no data, so
// the dashboard's percentage means something.
void test_progress_tracks_the_sweep() {
    MagCalCollector collector;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, collector.progress());

    SweepFullSphere(collector);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, collector.progress());
    TEST_ASSERT_TRUE(collector.sample_count() > 0);

    collector.Reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, collector.progress());
    TEST_ASSERT_EQUAL_INT(0, collector.sample_count());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_uncalibrated_is_a_passthrough);
    RUN_TEST(test_full_sweep_recovers_the_true_field);
    RUN_TEST(test_fit_recovers_the_hard_iron_offset);
    RUN_TEST(test_single_axis_rotation_is_refused);
    RUN_TEST(test_progress_tracks_the_sweep);
    return UNITY_END();
}
