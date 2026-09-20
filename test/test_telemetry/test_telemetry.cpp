// Host-side tests for the telemetry frame encoder.
//
// The frame is the contract between the firmware and web/index.html. These
// tests pin down that contract: that a full snapshot encodes without
// truncation, that every field the dashboard reads is present, and that a
// too-small buffer fails safe rather than emitting half a frame.
//
// The encoded frame is also printed, so it can be fed to the dashboard's own
// JavaScript in a headless harness (see tools/check_dashboard.mjs).

#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "telemetry.h"

namespace {

// A snapshot with every field set to something distinctive, so a field that
// silently encodes as the wrong one is visible in the output.
EstimatorSnapshot MakeSnapshot() {
    EstimatorSnapshot s;
    s.timestamp_ms = 1234567;
    s.estimator_hz = 199.6f;

    s.accel[0] = 0.12f;
    s.accel[1] = -0.34f;
    s.accel[2] = 9.79f;
    s.gyro[0] = 0.0012f;
    s.gyro[1] = -0.0034f;
    s.gyro[2] = 0.0056f;
    s.mag[0] = 21.5f;
    s.mag[1] = -4.25f;
    s.mag[2] = -43.75f;

    s.quat[0] = 0.9239f;
    s.quat[1] = 0.0f;
    s.quat[2] = 0.0f;
    s.quat[3] = 0.3827f;
    s.roll_deg = 9.9f;
    s.pitch_deg = -15.4f;
    s.yaw_deg = 114.5f;
    s.gyro_bias[0] = 0.00123f;
    s.gyro_bias[1] = -0.00456f;
    s.gyro_bias[2] = 0.00789f;
    s.rest_detected = true;
    s.mag_disturbed = false;
    s.mag_calibrated = true;
    s.mag_collecting = false;
    s.mag_cal_progress = 1.0f;
    s.mag_field_ut = 48.6f;

    s.pos[0] = 12.5f;
    s.pos[1] = -3.25f;
    s.pos[2] = 1.75f;
    s.vel[0] = 0.5f;
    s.vel[1] = -0.25f;
    s.vel[2] = 0.125f;
    s.accel_bias[0] = 0.021f;
    s.accel_bias[1] = -0.013f;
    s.accel_bias[2] = 0.005f;
    s.pos_sigma3[0] = 4.5f;
    s.pos_sigma3[1] = 4.5f;
    s.pos_sigma3[2] = 9.0f;
    s.vel_sigma3[0] = 0.75f;
    s.vel_sigma3[1] = 0.75f;
    s.vel_sigma3[2] = 1.5f;

    s.baro_healthy = true;
    s.baro_pressure_pa = 95431.0f;
    s.baro_temperature_c = 22.4f;
    s.baro_altitude_m = 509.25f;
    s.baro_bias_m = 497.5f;
    s.baro_bias_sigma3 = 1.8f;
    s.baro_height_m = 11.75f;
    s.baro_failures = 0;

    s.gps_fix = true;
    s.gps_satellites = 9;
    s.gps_hdop = 1.2f;
    s.gps_lat = 52.5200123;
    s.gps_lon = 13.4050456;
    s.gps_alt_m = 41.5f;
    s.gps_enu[0] = 12.0f;
    s.gps_enu[1] = -3.0f;
    s.gps_enu[2] = 2.0f;
    s.gps_enu_valid = true;
    s.gps_fix_age_ms = 420;
    s.gps_update_count = 37;

    s.origin_valid = true;
    s.origin_lat = 52.5199000;
    s.origin_lon = 13.4049000;
    s.origin_alt_m = 39.5f;

    s.imu_healthy = true;
    s.mag_healthy = true;
    s.gps_healthy = true;
    s.imu_failures = 2;
    s.gps_failures = 1;
    s.filter_resets = 0;
    return s;
}

}  // namespace

void setUp() {
}
void tearDown() {
}

// A full snapshot must fit the advertised buffer with room to spare, and the
// returned length must match the string actually written.
void test_full_frame_encodes_within_the_buffer() {
    char buffer[kTelemetryBufferSize];
    const EstimatorSnapshot snapshot = MakeSnapshot();
    const FilterParams params;

    const size_t length = BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal");

    TEST_ASSERT_TRUE(length > 0);
    TEST_ASSERT_EQUAL_size_t(strlen(buffer), length);
    TEST_ASSERT_EQUAL_CHAR('{', buffer[0]);
    TEST_ASSERT_EQUAL_CHAR('}', buffer[length - 1]);

    // Printed so the headless dashboard harness can replay a real frame.
    printf("\nFRAME:%s\n", buffer);
}

// Every key web/index.html reads must be present. If a field is ever renamed
// on one side only, this is what catches it.
void test_frame_contains_every_key_the_dashboard_reads() {
    char buffer[kTelemetryBufferSize];
    const EstimatorSnapshot snapshot = MakeSnapshot();
    const FilterParams params;
    BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal");

    const char* required[] = {
        "\"t\":",
        "\"hz\":",
        "\"status\":",
        "\"imu\":",
        "\"a\":",
        "\"g\":",
        "\"m\":",
        "\"att\":",
        "\"q\":",
        "\"rpy\":",
        "\"rest\":",
        "\"magdist\":",
        "\"gbias\":",
        "\"nav\":",
        "\"p\":",
        "\"v\":",
        "\"ba\":",
        "\"p3s\":",
        "\"v3s\":",
        "\"gps\":",
        "\"fix\":",
        "\"sat\":",
        "\"hdop\":",
        "\"enuok\":",
        "\"enu\":",
        "\"n\":",
        "\"org\":",
        "\"health\":",
        "\"params\":",
        "\"sigma_accel\":",
        "\"sigma_accel_bias\":",
        "\"sigma_gps_pos_h\":",
        "\"sigma_gps_pos_v\":",
        "\"sigma_gps_vel\":",
        "\"sigma_zupt\":",
        "\"tau_acc\":",
        "\"tau_mag\":",
        "\"zupt_enabled\":",
        "\"gps_vel_enabled\":",
        // Magnetometer calibration state.
        "\"magcal\":",
        "\"done\":",
        "\"busy\":",
        "\"prog\":",
        "\"implied\":",
        "\"field\":",
        // Barometer readings, and the knobs that govern its fusion.
        "\"baro\":",
        "\"pa\":",
        "\"tc\":",
        "\"alt\":",
        "\"bias\":",
        "\"bias3s\":",
        "\"sigma_baro\":",
        "\"sigma_baro_bias\":",
        "\"baro_enabled\":",
    };
    for (const char* key : required) {
        TEST_ASSERT_NOT_NULL_MESSAGE(strstr(buffer, key), key);
    }
}

// Latitude and longitude need enough decimals to be worth plotting -- at
// 5 decimals a fix would quantise to about a metre.
void test_position_is_encoded_at_full_precision() {
    char buffer[kTelemetryBufferSize];
    const EstimatorSnapshot snapshot = MakeSnapshot();
    const FilterParams params;
    BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal");

    TEST_ASSERT_NOT_NULL(strstr(buffer, "52.5200123"));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "13.4050456"));
}

// A buffer too small to hold the whole frame must yield nothing at all --
// a truncated frame would be invalid JSON and the dashboard would drop it.
void test_short_buffer_emits_nothing() {
    char buffer[64];
    const EstimatorSnapshot snapshot = MakeSnapshot();
    const FilterParams params;

    TEST_ASSERT_EQUAL_size_t(
        0, BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal"));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_full_frame_encodes_within_the_buffer);
    RUN_TEST(test_frame_contains_every_key_the_dashboard_reads);
    RUN_TEST(test_position_is_encoded_at_full_precision);
    RUN_TEST(test_short_buffer_emits_nothing);
    return UNITY_END();
}
