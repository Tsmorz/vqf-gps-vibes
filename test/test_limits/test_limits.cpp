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

}  // namespace

void setUp() {
}
void tearDown() {
}

// Ordinary readings must pass, including brisk but physical motion.
void test_resting_and_moving_samples_are_accepted() {
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(kRestingAccel, kRestingGyro));
    TEST_ASSERT_TRUE(sensor_limits::MagLooksValid(kRestingMag));

    // A sharp but entirely physical manoeuvre: ~2 g and ~300 deg/s.
    const float brisk_accel[3] = {5.0f, -3.0f, 18.0f};
    const float brisk_gyro[3] = {5.2f, -2.0f, 1.0f};
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(brisk_accel, brisk_gyro));
}

// The exact zeros a failed transfer produces. This is the case that sent the
// navigation filter into simulated free fall.
void test_all_zero_sample_is_rejected() {
    const float zeros[3] = {0.0f, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(zeros, kRestingGyro));
}

// 20 rad/s is 1146 deg/s, past the +-1000 dps the gyroscope is configured for.
void test_gyro_beyond_full_scale_is_rejected() {
    const float impossible[3] = {0.0f, 19.874f, 18.921f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(kRestingAccel, impossible));
}

// 479 uT is past the +-4 gauss the magnetometer is configured for.
void test_mag_beyond_full_scale_is_rejected() {
    const float impossible[3] = {15.9f, -10.4f, 478.87f};
    TEST_ASSERT_FALSE(sensor_limits::MagLooksValid(impossible));

    const float also_impossible[3] = {15.9f, -10.4f, -449.14f};
    TEST_ASSERT_FALSE(sensor_limits::MagLooksValid(also_impossible));
}

// A saturated axis says only "at least full scale", so it carries no
// magnitude to integrate -- whether it is genuine clipping or corruption.
// 38.898 m/s^2 is 3.97 g against a 4 g range: inside the range, but clipped.
void test_saturated_accel_is_rejected() {
    const float saturated[3] = {38.898f, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(saturated, kRestingGyro));

    // Comfortably inside the range, and must still be accepted.
    const float strong_but_usable[3] = {30.0f, 0.0f, 9.8f};
    TEST_ASSERT_TRUE(sensor_limits::AccelGyroLooksValid(strong_but_usable, kRestingGyro));
}

// NaN must never reach the filter, where it would poison the covariance.
void test_non_finite_values_are_rejected() {
    const float nan_accel[3] = {NAN, 0.0f, 9.8f};
    TEST_ASSERT_FALSE(sensor_limits::AccelGyroLooksValid(nan_accel, kRestingGyro));

    const float inf_mag[3] = {INFINITY, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(sensor_limits::MagLooksValid(inf_mag));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_resting_and_moving_samples_are_accepted);
    RUN_TEST(test_all_zero_sample_is_rejected);
    RUN_TEST(test_gyro_beyond_full_scale_is_rejected);
    RUN_TEST(test_mag_beyond_full_scale_is_rejected);
    RUN_TEST(test_saturated_accel_is_rejected);
    RUN_TEST(test_non_finite_values_are_rejected);
    return UNITY_END();
}
