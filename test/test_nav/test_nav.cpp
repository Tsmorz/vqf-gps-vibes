// Host-side unit tests for the pure-math half of the firmware: the navigation
// EKF (nav_filter.h) and the geodetic projection (geo.h). Run with `task test`.

#include <unity.h>

#include "geo.h"
#include "nav_filter.h"

namespace {

// Body and ENU frames coincide, so the accelerometer's axes map straight
// through. Every test uses this unless it is specifically about rotation.
const float kIdentityRotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// What a level, stationary accelerometer reports: +1 g along its up axis.
const float kAccelAtRest[3] = {0.0f, 0.0f, NavFilter::kGravityMps2};

// Runs `seconds` of prediction at 200 Hz with the given body acceleration.
void PredictFor(NavFilter& filter, const float accel_body[3], float seconds) {
    const float dt = 0.005f;
    const NavFilter::Params params;
    for (int i = 0; i < static_cast<int>(seconds / dt); i++) {
        filter.Predict(accel_body, kIdentityRotation, dt, params);
    }
}

}  // namespace

void setUp() {
}
void tearDown() {
}

// A freshly reset filter sits at the origin with a positive-definite diagonal.
void test_reset_is_at_origin_with_positive_covariance() {
    NavFilter filter;
    for (int i = 0; i < NavFilter::kNumStates; i++) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, filter.state(i));
        TEST_ASSERT_TRUE(filter.variance(i) > 0.0f);
    }
    TEST_ASSERT_FALSE(filter.IsDiverged());
}

// Gravity must cancel exactly: a level board left alone must not accelerate.
void test_gravity_is_cancelled_when_level() {
    NavFilter filter;
    PredictFor(filter, kAccelAtRest, 10.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, filter.state(NavFilter::kVelUp));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, filter.state(NavFilter::kPosUp));
    TEST_ASSERT_FALSE(filter.IsDiverged());
}

// A 1 m/s^2 push along body x integrates into ENU east: v = at, p = at^2/2.
void test_constant_acceleration_integrates_to_kinematics() {
    NavFilter filter;
    const float accel[3] = {1.0f, 0.0f, NavFilter::kGravityMps2};
    PredictFor(filter, accel, 2.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.02f, 2.0f, filter.state(NavFilter::kVelEast));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, filter.state(NavFilter::kPosEast));
}

// With no measurements the position envelope must keep widening -- this is the
// behaviour the dashboard's 3-sigma band is there to show.
void test_uncertainty_grows_without_measurements() {
    NavFilter filter;
    const float initial = filter.ThreeSigma(NavFilter::kPosEast);
    PredictFor(filter, kAccelAtRest, 30.0f);

    TEST_ASSERT_TRUE(filter.ThreeSigma(NavFilter::kPosEast) > initial);
    TEST_ASSERT_TRUE(filter.ThreeSigma(NavFilter::kVelEast) > 0.0f);
}

// A scalar update must move the state toward the measurement and shrink that
// state's variance -- the two defining properties of a Kalman correction.
void test_scalar_update_moves_state_and_shrinks_variance() {
    NavFilter filter;
    const float before = filter.variance(NavFilter::kPosEast);
    filter.UpdateScalar(NavFilter::kPosEast, 10.0f, 1.0f);

    TEST_ASSERT_TRUE(filter.state(NavFilter::kPosEast) > 0.0f);
    TEST_ASSERT_TRUE(filter.state(NavFilter::kPosEast) < 10.0f);
    TEST_ASSERT_TRUE(filter.variance(NavFilter::kPosEast) < before);
}

// Repeated fixes at the same point should converge onto it and tighten the
// envelope well below the single-fix measurement noise.
void test_repeated_gps_fixes_converge() {
    NavFilter filter;
    const float fix[3] = {5.0f, -3.0f, 12.0f};
    for (int i = 0; i < 50; i++) {
        PredictFor(filter, kAccelAtRest, 1.0f);
        filter.UpdateGpsPosition(fix, 3.0f, 6.0f);
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5.0f, filter.state(NavFilter::kPosEast));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -3.0f, filter.state(NavFilter::kPosNorth));
    TEST_ASSERT_TRUE(filter.ThreeSigma(NavFilter::kPosEast) < 9.0f);
}

// The zero-velocity pseudo-measurement must pull velocity back to zero.
void test_zero_velocity_update_arrests_drift() {
    NavFilter filter;
    const float accel[3] = {2.0f, 0.0f, NavFilter::kGravityMps2};
    PredictFor(filter, accel, 1.0f);
    TEST_ASSERT_TRUE(filter.state(NavFilter::kVelEast) > 1.0f);

    for (int i = 0; i < 20; i++) {
        filter.UpdateZeroVelocity(0.02f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, filter.state(NavFilter::kVelEast));
}

// A stationary board with an uncorrected accel bias appears to accelerate.
// Held in place by GPS and zero-velocity updates, the filter should attribute
// that apparent motion to bias and estimate it -- the point of the bias states.
void test_accel_bias_is_observed_when_held_stationary() {
    NavFilter filter;
    const float kBias = 0.3f;
    const float accel[3] = {kBias, 0.0f, NavFilter::kGravityMps2};
    const float origin[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 400; i++) {
        PredictFor(filter, accel, 0.1f);
        filter.UpdateGpsPosition(origin, 1.0f, 2.0f);
        filter.UpdateZeroVelocity(0.02f);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.1f, kBias, filter.state(NavFilter::kBiasX));
}

// With no IMU the filter must coast on its last velocity and open its
// envelope faster than it would with a real accelerometer -- never freeze.
void test_coasting_widens_the_envelope_without_moving_wildly() {
    NavFilter filter;
    const NavFilter::Params params;
    filter.UpdateScalar(NavFilter::kVelEast, 1.0f, 0.01f);

    const float before = filter.ThreeSigma(NavFilter::kPosEast);
    for (int i = 0; i < 2000; i++) {
        filter.PredictCoasting(0.005f, params);
    }

    // Ten seconds at about 1 m/s -- coasting, not frozen and not diverging.
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 10.0f, filter.state(NavFilter::kPosEast));
    TEST_ASSERT_TRUE(filter.ThreeSigma(NavFilter::kPosEast) > before);
    TEST_ASSERT_FALSE(filter.IsDiverged());
}

// An unplugged accelerometer reads as a clean (0, 0, 0), which is
// indistinguishable from free fall. The state stays finite the whole way down,
// so the divergence check has to be about plausibility, not just NaN.
void test_free_fall_from_a_dead_sensor_is_caught() {
    NavFilter filter;
    const float dead_sensor[3] = {0.0f, 0.0f, 0.0f};
    const NavFilter::Params params;

    for (int i = 0; i < 200 * 200 && !filter.IsDiverged(); i++) {
        filter.Predict(dead_sensor, kIdentityRotation, 0.005f, params);
    }

    TEST_ASSERT_TRUE_MESSAGE(filter.IsDiverged(),
                             "a sensor reading zeros must eventually be caught");
}

// A nonsensical timestep must be ignored rather than poison the covariance.
void test_bad_timestep_is_rejected() {
    NavFilter filter;
    const NavFilter::Params params;
    filter.Predict(kAccelAtRest, kIdentityRotation, -1.0f, params);
    filter.Predict(kAccelAtRest, kIdentityRotation, 100.0f, params);

    TEST_ASSERT_FALSE(filter.IsDiverged());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, filter.state(NavFilter::kPosEast));
}

// One degree of latitude is close to 111 km; one degree of longitude shrinks
// by cos(latitude), so it is noticeably shorter at 52 degrees north.
void test_geo_degree_scales_are_physical() {
    TEST_ASSERT_FLOAT_WITHIN(400.0f, 111000.0f, geo::MetersPerDegreeLat(52.0));
    TEST_ASSERT_FLOAT_WITHIN(1000.0f, 68600.0f, geo::MetersPerDegreeLon(52.0));
}

// The origin projects to exactly zero, and a fix north-east of it lands in the
// positive quadrant with altitude carried straight through to up.
void test_geo_projection_is_relative_to_origin() {
    geo::Origin origin;
    origin.valid = true;
    origin.lat_deg = 52.5200;
    origin.lon_deg = 13.4050;
    origin.alt_m = 34.0;

    float enu[3];
    geo::ToEnu(origin, origin.lat_deg, origin.lon_deg, origin.alt_m, enu);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, enu[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, enu[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, enu[2]);

    geo::ToEnu(origin, origin.lat_deg + 0.001, origin.lon_deg + 0.001, origin.alt_m + 10.0, enu);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 68.0f, enu[0]);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 111.0f, enu[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 10.0f, enu[2]);
}

// Course is measured clockwise from north: 0 is due north, 90 is due east.
void test_course_maps_to_enu_velocity() {
    float east = 0.0f;
    float north = 0.0f;

    geo::CourseToEnuVelocity(10.0f, 0.0f, east, north);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, east);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, north);

    geo::CourseToEnuVelocity(10.0f, 90.0f, east, north);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, east);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, north);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.14444f, geo::KnotsToMps(10.0f));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_reset_is_at_origin_with_positive_covariance);
    RUN_TEST(test_gravity_is_cancelled_when_level);
    RUN_TEST(test_constant_acceleration_integrates_to_kinematics);
    RUN_TEST(test_uncertainty_grows_without_measurements);
    RUN_TEST(test_scalar_update_moves_state_and_shrinks_variance);
    RUN_TEST(test_repeated_gps_fixes_converge);
    RUN_TEST(test_zero_velocity_update_arrests_drift);
    RUN_TEST(test_accel_bias_is_observed_when_held_stationary);
    RUN_TEST(test_coasting_widens_the_envelope_without_moving_wildly);
    RUN_TEST(test_free_fall_from_a_dead_sensor_is_caught);
    RUN_TEST(test_bad_timestep_is_rejected);
    RUN_TEST(test_geo_degree_scales_are_physical);
    RUN_TEST(test_geo_projection_is_relative_to_origin);
    RUN_TEST(test_course_maps_to_enu_velocity);
    return UNITY_END();
}
