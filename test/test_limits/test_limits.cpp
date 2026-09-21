// Host-side tests for the IMU plausibility limits.
//
// The values below are taken from a real failure: with the I2C peripheral in a
// bad state after a bus teardown, a stationary board reported gyro rates of
// 20 rad/s, magnetometer readings of 479 uT, and frames of exact zeros -- all
// of which the Adafruit driver reported as successful reads.

#include <unity.h>

#include "sensor_limits.h"

namespace {

// A level, stationary board as it actually reads on this hardware.
const float kRestingAccel[3] = {1.568f, -0.017f, 9.989f};
const float kRestingGyro[3] = {0.002f, 0.012f, 0.004f};
const float kRestingMag[3] = {16.3f, -10.5f, -90.0f};

// The ranges the firmware boots with: +-4 g, +-1000 dps, +-4 gauss.
constexpr sensor_limits::FullScale kDefault = sensor_limits::kDefaultFullScale;

// The widest each chip offers, used to show a limit tracking its range.
constexpr sensor_limits::FullScale kWidest = sensor_limits::FullScaleFor(16, 2000, 16);

}  // namespace

void setUp() {
}
void tearDown() {
}

// Ordinary readings must pass, including brisk but physical motion.
void test_resting_and_moving_samples_are_accepted() {
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(kRestingAccel, kRestingGyro, kDefault));
    TEST_ASSERT_TRUE(sensor_limits::MagLooksValid(kRestingMag, kDefault));

    // A sharp but entirely physical manoeuvre: ~2 g and ~300 deg/s.
    const float brisk_accel[3] = {5.0f, -3.0f, 18.0f};
    const float brisk_gyro[3] = {5.2f, -2.0f, 1.0f};
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(brisk_accel, brisk_gyro, kDefault));
}

// The exact zeros a failed transfer produces. This is the case that sent the
// navigation filter into simulated free fall.
void test_all_zero_sample_is_rejected() {
    const float zeros[3] = {0.0f, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(zeros, kRestingGyro, kDefault));
}

// 20 rad/s is 1146 deg/s, past the +-1000 dps the gyroscope defaults to.
void test_gyro_beyond_full_scale_is_rejected() {
    const float impossible[3] = {0.0f, 19.874f, 18.921f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(kRestingAccel, impossible, kDefault));
}

// 479 uT is past the +-4 gauss the magnetometer defaults to.
void test_mag_beyond_full_scale_is_rejected() {
    const float impossible[3] = {15.9f, -10.4f, 478.87f};
    TEST_ASSERT_FALSE(sensor_limits::MagLooksValid(impossible, kDefault));

    const float also_impossible[3] = {15.9f, -10.4f, -449.14f};
    TEST_ASSERT_FALSE(sensor_limits::MagLooksValid(also_impossible, kDefault));
}

// A saturated axis says only "at least full scale", so it carries no
// magnitude to integrate -- whether it is genuine clipping or corruption.
// 38.898 m/s^2 is 3.97 g against a 4 g range: inside the range, but clipped.
void test_saturated_accel_is_rejected() {
    const float saturated[3] = {38.898f, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(saturated, kRestingGyro, kDefault));

    // Comfortably inside the range, and must still be accepted.
    const float strong_but_usable[3] = {30.0f, 0.0f, 9.8f};
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(strong_but_usable, kRestingGyro, kDefault));
}

// NaN must never reach the filter, where it would poison the covariance.
void test_non_finite_values_are_rejected() {
    const float nan_accel[3] = {NAN, 0.0f, 9.8f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(nan_accel, kRestingGyro, kDefault));

    const float inf_mag[3] = {INFINITY, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(sensor_limits::MagLooksValid(inf_mag, kDefault));
}

// The conversions from the chips' own units. These are the numbers the limits
// are built out of, and getting gauss -> uT or dps -> rad/s wrong would move
// every threshold at once without looking like an error anywhere.
void test_full_scale_is_derived_from_the_configured_ranges() {
    const sensor_limits::FullScale scale = sensor_limits::FullScaleFor(4, 1000, 4);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 39.2266f, scale.accel_mps2);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 17.4533f, scale.gyro_rad_s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 400.0f, scale.mag_ut);

    // The boot defaults must be exactly that, or config.h and the firmware
    // disagree about what the hardware is doing.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, scale.accel_mps2, kDefault.accel_mps2);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, scale.gyro_rad_s, kDefault.gyro_rad_s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, scale.mag_ut, kDefault.mag_ut);
}

// The point of making the ranges adjustable: the same readings that clip on
// the default ranges are ordinary measurements on the widest ones. If the
// limits did not track the range, widening it on the dashboard would change
// nothing except that the samples kept being thrown away.
void test_widening_the_range_accepts_what_the_default_range_clips() {
    const float hard_knock[3] = {38.898f, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(hard_knock, kRestingGyro, kDefault));
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(hard_knock, kRestingGyro, kWidest));

    const float fast_spin[3] = {0.0f, 19.874f, 18.921f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(kRestingAccel, fast_spin, kDefault));
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(kRestingAccel, fast_spin, kWidest));

    const float strong_field[3] = {15.9f, -10.4f, 478.87f};
    TEST_ASSERT_FALSE(sensor_limits::MagLooksValid(strong_field, kDefault));
    TEST_ASSERT_TRUE(sensor_limits::MagLooksValid(strong_field, kWidest));
}

// And the other way: narrowing a range has to start rejecting what it can no
// longer represent, or the filter would integrate clipped samples as real.
void test_narrowing_the_range_rejects_what_it_cannot_represent() {
    // 30 m/s^2 is 3.06 g -- fine at 4 g, past full scale at 2 g.
    const float strong[3] = {30.0f, 0.0f, 9.8f};
    constexpr sensor_limits::FullScale narrow = sensor_limits::FullScaleFor(2, 125, 4);
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(strong, kRestingGyro, kDefault));
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(strong, kRestingGyro, narrow));

    // 5.2 rad/s is 298 deg/s -- fine at 1000 dps, past full scale at 125.
    const float brisk_gyro[3] = {5.2f, -2.0f, 1.0f};
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(kRestingAccel, brisk_gyro, kDefault));
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(kRestingAccel, brisk_gyro, narrow));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_resting_and_moving_samples_are_accepted);
    RUN_TEST(test_all_zero_sample_is_rejected);
    RUN_TEST(test_gyro_beyond_full_scale_is_rejected);
    RUN_TEST(test_mag_beyond_full_scale_is_rejected);
    RUN_TEST(test_saturated_accel_is_rejected);
    RUN_TEST(test_non_finite_values_are_rejected);
    RUN_TEST(test_full_scale_is_derived_from_the_configured_ranges);
    RUN_TEST(test_widening_the_range_accepts_what_the_default_range_clips);
    RUN_TEST(test_narrowing_the_range_rejects_what_it_cannot_represent);
    return UNITY_END();
}
