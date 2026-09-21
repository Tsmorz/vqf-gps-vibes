// Host-side tests for the telemetry frame encoder.
//
// The frame is the contract between the firmware and web/index.html. These
// tests pin down that contract: that a full snapshot encodes without
// truncation, that every field the dashboard reads is present, and that a
// too-small buffer fails safe rather than emitting half a frame.
//
// The encoded frame is also printed, so it can be fed to the dashboard's own
// JavaScript in a headless harness (see tools/check_dashboard.mjs).

#include <math.h>
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
    s.gps_rejected_count = 4;
    s.gps_sigma_h_m = 7.5f;
    s.gps_epoch_tod_ms = 45296789;
    s.gps_epoch_valid = true;

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

// The analyser's state as the frame echoes it. Switched on and pointed at the
// gyroscope, so the dashboard harness exercises the panel-visible path and a
// source dropdown that has to sync to something other than its default.
const SpectrumConfig spec = {/*enabled=*/true, SpectrumSource::kGyro};

// A spectrum with one obvious peak, so a bin that lands in the wrong place is
// visible in the output rather than hidden in 129 similar numbers.
SpectrumSnapshot MakeSpectrum() {
    SpectrumSnapshot s;
    s.valid = true;
    s.source = SpectrumSource::kGyro;
    s.timestamp_ms = 1234567;
    s.sample_rate_hz = 199.6f;
    for (int axis = 0; axis < 3; axis++) {
        for (int bin = 0; bin < spectrum::kBins; bin++) {
            s.bins[axis][bin] = 0.001f;
        }
        // A distinct peak per axis: x at bin 20, y at 21, z at 22.
        s.bins[axis][20 + axis] = 0.25f + 0.1f * axis;
    }
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

    const size_t length =
        BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal", spec);

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
    BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal", spec);

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
        "\"rej\":",
        "\"sig\":",
        "\"ep\":",
        "\"epok\":",
        "\"enuok\":",
        "\"enu\":",
        "\"n\":",
        "\"org\":",
        "\"health\":",
        "\"aux\":",
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
        "\"gps_enabled\":",
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
        // IMU full-scale ranges. The dashboard seeds its dropdowns from these,
        // so a missing one leaves the UI showing a range the chip is not on.
        "\"accel_range_g\":",
        "\"gyro_range_dps\":",
        "\"mag_range_gauss\":",
        // Spectral analyser state. Not the spectrum itself -- that is its own
        // frame -- but what the dashboard needs to show the panel at all.
        "\"spec\":",
        "\"on\":",
        "\"src\":",
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
    BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal", spec);

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
        0, BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal", spec));
}

// Every boolean in the frame, in both states.
//
// The encoder writes these through `? 1 : 0` ternaries, and a flag dropped
// from a format string -- or wired to its neighbour -- still encodes cleanly
// as long as only one state is ever exercised. That is exactly how
// `baro_enabled` once went missing from the params block while the test that
// should have caught it had lost its assertion. Checking both states of every
// flag is what closes that gap.
void test_every_boolean_encodes_in_both_states() {
    char buffer[kTelemetryBufferSize];

    EstimatorSnapshot set = MakeSnapshot();
    set.rest_detected = true;
    set.mag_disturbed = true;
    set.mag_calibrated = true;
    set.mag_collecting = true;
    set.baro_healthy = true;
    set.gps_fix = true;
    set.gps_enu_valid = true;
    set.gps_epoch_valid = true;
    set.origin_valid = true;
    set.imu_healthy = true;
    set.mag_healthy = true;
    set.gps_healthy = true;
    FilterParams params_set;
    params_set.zupt_enabled = true;
    params_set.gps_enabled = true;
    params_set.gps_vel_enabled = true;
    params_set.baro_enabled = true;

    SpectrumConfig spec_set;
    spec_set.enabled = true;

    TEST_ASSERT_TRUE(
        BuildTelemetryFrame(buffer, sizeof(buffer), set, params_set, "normal", spec_set) > 0);
    const char* when_set[] = {"\"rest\":1",         "\"magdist\":1",
                              "\"done\":1",         "\"busy\":1",
                              "\"ok\":1",           "\"fix\":1",
                              "\"enuok\":1",        "\"epok\":1",
                              "\"imu\":1",          "\"mag\":1",
                              "\"gps\":1",          "\"zupt_enabled\":1",
                              "\"gps_enabled\":1",  "\"gps_vel_enabled\":1",
                              "\"baro_enabled\":1", "\"on\":1"};
    for (const char* needle : when_set) {
        TEST_ASSERT_NOT_NULL_MESSAGE(strstr(buffer, needle), needle);
    }

    EstimatorSnapshot clear = MakeSnapshot();
    clear.rest_detected = false;
    clear.mag_disturbed = false;
    clear.mag_calibrated = false;
    clear.mag_collecting = false;
    clear.baro_healthy = false;
    clear.gps_fix = false;
    clear.gps_enu_valid = false;
    clear.gps_epoch_valid = false;
    clear.origin_valid = false;
    clear.imu_healthy = false;
    clear.mag_healthy = false;
    clear.gps_healthy = false;
    FilterParams params_clear;
    params_clear.zupt_enabled = false;
    params_clear.gps_enabled = false;
    params_clear.gps_vel_enabled = false;
    params_clear.baro_enabled = false;

    SpectrumConfig spec_clear;
    spec_clear.enabled = false;

    TEST_ASSERT_TRUE(
        BuildTelemetryFrame(buffer, sizeof(buffer), clear, params_clear, "error", spec_clear) > 0);
    const char* when_clear[] = {"\"rest\":0",         "\"magdist\":0",
                                "\"done\":0",         "\"busy\":0",
                                "\"ok\":0",           "\"fix\":0",
                                "\"enuok\":0",        "\"epok\":0",
                                "\"imu\":0",          "\"mag\":0",
                                "\"gps\":0",          "\"zupt_enabled\":0",
                                "\"gps_enabled\":0",  "\"gps_vel_enabled\":0",
                                "\"baro_enabled\":0", "\"on\":0"};
    for (const char* needle : when_clear) {
        TEST_ASSERT_NOT_NULL_MESSAGE(strstr(buffer, needle), needle);
    }
}

// The ranges must survive as the integers they were set to. This is the
// failure `baro_enabled` already had once: the key present, the value wrong or
// the field quietly dropped, with the firmware and the dashboard each certain
// the other agreed with it.
void test_imu_ranges_are_encoded_as_set() {
    char buffer[kTelemetryBufferSize];
    const EstimatorSnapshot snapshot = MakeSnapshot();

    FilterParams params;
    params.accel_range_g = 16;
    params.gyro_range_dps = 2000;
    params.mag_range_gauss = 12;

    TEST_ASSERT_TRUE(BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, params, "normal", spec) >
                     0);
    const char* expected[] = {"\"accel_range_g\":16", "\"gyro_range_dps\":2000",
                              "\"mag_range_gauss\":12"};
    for (const char* needle : expected) {
        TEST_ASSERT_NOT_NULL_MESSAGE(strstr(buffer, needle), needle);
    }

    // And the boot defaults, which is what an untouched dashboard shows.
    const FilterParams defaults;
    TEST_ASSERT_TRUE(
        BuildTelemetryFrame(buffer, sizeof(buffer), snapshot, defaults, "normal", spec) > 0);
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"accel_range_g\":4"));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"gyro_range_dps\":1000"));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"mag_range_gauss\":4"));
}

// The encoder stops at whichever field overruns the buffer, and there is a
// separate early exit for every one of them -- a long `&&` chain, of which
// only the first link is exercised by a buffer that is obviously too small.
// Sweeping every size walks the failure through all of them. The contract at
// each: either nothing at all, or the complete frame, never a prefix of it.
void test_every_truncation_point_fails_safe() {
    const EstimatorSnapshot snapshot = MakeSnapshot();
    const FilterParams params;

    char full[kTelemetryBufferSize];
    const size_t complete =
        BuildTelemetryFrame(full, sizeof(full), snapshot, params, "normal", spec);
    TEST_ASSERT_TRUE(complete > 0);

    char buffer[kTelemetryBufferSize];
    for (size_t size = 1; size <= complete + 1; size++) {
        memset(buffer, 0x7f, sizeof(buffer));
        const size_t used = BuildTelemetryFrame(buffer, size, snapshot, params, "normal", spec);
        if (used == 0) {
            continue;
        }
        TEST_ASSERT_EQUAL_size_t(complete, used);
        TEST_ASSERT_EQUAL_STRING(full, buffer);
    }

    // The smallest buffer that can hold the frame does produce it, so the
    // sweep above is not simply refusing every size.
    TEST_ASSERT_EQUAL_size_t(
        complete, BuildTelemetryFrame(buffer, complete + 1, snapshot, params, "normal", spec));
}

// ── Spectrum frames ────────────────────────────────────────────────────────

// The spectrum frame is the larger half of the contract, and the buffer it has
// to fit in was sized by arithmetic rather than by measurement. This is the
// measurement.
void test_spectrum_frame_encodes_within_the_buffer() {
    char buffer[kSpectrumBufferSize];
    const size_t length = BuildSpectrumFrame(buffer, sizeof(buffer), MakeSpectrum());

    TEST_ASSERT_TRUE(length > 0);
    TEST_ASSERT_EQUAL_size_t(strlen(buffer), length);
    TEST_ASSERT_EQUAL_CHAR('{', buffer[0]);
    TEST_ASSERT_EQUAL_CHAR('}', buffer[length - 1]);

    // Printed so the headless dashboard harness can replay a real one.
    printf("\nSPECTRUM:%s\n", buffer);
}

// Worst case for the encoder is not the largest numbers but the longest ones:
// %.4g spends most characters on a small value in exponent form.
void test_a_worst_case_spectrum_still_fits() {
    SpectrumSnapshot worst = MakeSpectrum();
    for (int axis = 0; axis < 3; axis++) {
        for (int bin = 0; bin < spectrum::kBins; bin++) {
            worst.bins[axis][bin] = 1.2345e-11f;
        }
    }
    char buffer[kSpectrumBufferSize];
    TEST_ASSERT_TRUE(BuildSpectrumFrame(buffer, sizeof(buffer), worst) > 0);
}

void test_spectrum_frame_carries_what_the_dashboard_reads() {
    char buffer[kSpectrumBufferSize];
    BuildSpectrumFrame(buffer, sizeof(buffer), MakeSpectrum());

    // "type" is the only thing separating this from a telemetry frame.
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"type\":\"spec\""));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"src\":\"gyro\""));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"fs\":199.60"));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"n\":256"));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"bins\":[["));
    // Three axes, so two separators between them and a nested close at the end.
    TEST_ASSERT_NOT_NULL(strstr(buffer, "],["));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "]]}"));
    // The per-axis peaks, which is how a transposed or reused axis shows up.
    TEST_ASSERT_NOT_NULL(strstr(buffer, "0.25"));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "0.35"));
    TEST_ASSERT_NOT_NULL(strstr(buffer, "0.45"));
}

// Accelerometer is the other source name, and the one the dashboard defaults
// to -- a frame that always said "gyro" would still pass the test above.
void test_the_accelerometer_source_is_named_too() {
    SpectrumSnapshot from_accel = MakeSpectrum();
    from_accel.source = SpectrumSource::kAccel;

    char buffer[kSpectrumBufferSize];
    BuildSpectrumFrame(buffer, sizeof(buffer), from_accel);
    TEST_ASSERT_NOT_NULL(strstr(buffer, "\"src\":\"accel\""));
}

// Nothing has been computed yet: there is no spectrum to send, as opposed to
// an empty one. Sending 129 zeros would draw a floor that looks like a reading.
void test_an_invalid_spectrum_emits_nothing() {
    SpectrumSnapshot nothing_yet = MakeSpectrum();
    nothing_yet.valid = false;

    char buffer[kSpectrumBufferSize];
    TEST_ASSERT_EQUAL_size_t(0, BuildSpectrumFrame(buffer, sizeof(buffer), nothing_yet));
}

// A NaN would encode as "nan", which is not JSON -- JSON.parse throws and the
// dashboard loses the whole frame rather than the one bin.
void test_non_finite_bins_encode_as_zero() {
    SpectrumSnapshot poisoned = MakeSpectrum();
    poisoned.bins[0][5] = NAN;
    poisoned.bins[1][6] = INFINITY;
    poisoned.bins[2][7] = -INFINITY;

    char buffer[kSpectrumBufferSize];
    TEST_ASSERT_TRUE(BuildSpectrumFrame(buffer, sizeof(buffer), poisoned) > 0);
    TEST_ASSERT_NULL(strstr(buffer, "nan"));
    TEST_ASSERT_NULL(strstr(buffer, "inf"));
    TEST_ASSERT_NULL(strstr(buffer, "NAN"));
    TEST_ASSERT_NULL(strstr(buffer, "INF"));
}

// Same sweep as the telemetry frame: every buffer size either yields the whole
// frame or nothing, never a truncated one. The spectrum frame has far more
// places to run out, since it appends one number at a time.
void test_every_spectrum_truncation_point_fails_safe() {
    const SpectrumSnapshot reference = MakeSpectrum();

    char full[kSpectrumBufferSize];
    const size_t complete = BuildSpectrumFrame(full, sizeof(full), reference);
    TEST_ASSERT_TRUE(complete > 0);

    char buffer[kSpectrumBufferSize];
    for (size_t size = 1; size <= complete + 1; size++) {
        memset(buffer, 0x7f, sizeof(buffer));
        const size_t used = BuildSpectrumFrame(buffer, size, reference);
        if (used == 0) {
            continue;
        }
        TEST_ASSERT_EQUAL_size_t(complete, used);
        TEST_ASSERT_EQUAL_STRING(full, buffer);
    }

    TEST_ASSERT_EQUAL_size_t(complete, BuildSpectrumFrame(buffer, complete + 1, reference));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_full_frame_encodes_within_the_buffer);
    RUN_TEST(test_frame_contains_every_key_the_dashboard_reads);
    RUN_TEST(test_position_is_encoded_at_full_precision);
    RUN_TEST(test_imu_ranges_are_encoded_as_set);
    RUN_TEST(test_short_buffer_emits_nothing);
    RUN_TEST(test_every_boolean_encodes_in_both_states);
    RUN_TEST(test_every_truncation_point_fails_safe);
    RUN_TEST(test_spectrum_frame_encodes_within_the_buffer);
    RUN_TEST(test_a_worst_case_spectrum_still_fits);
    RUN_TEST(test_spectrum_frame_carries_what_the_dashboard_reads);
    RUN_TEST(test_the_accelerometer_source_is_named_too);
    RUN_TEST(test_an_invalid_spectrum_emits_nothing);
    RUN_TEST(test_non_finite_bins_encode_as_zero);
    RUN_TEST(test_every_spectrum_truncation_point_fails_safe);
    return UNITY_END();
}
