// Host-side unit tests for the pure-math half of the firmware: the navigation
// EKF (nav_filter.h) and the geodetic projection (geo.h). Run with `task test`.

#include <unity.h>

#include "geo.h"
#include "gps_quality.h"
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

// ── Barometer ───────────────────────────────────────────────────────────────

// Anchoring must not move the height estimate. The barometer's datum is
// unknown by hundreds of metres at first, and blending that in would drag the
// estimate with it -- which is the whole reason SetBaroBias exists.
void test_anchoring_the_barometer_does_not_move_height() {
    NavFilter filter;
    const float fix[3] = {0.0f, 0.0f, 12.0f};
    filter.SetPosition(fix, 1.0f);
    TEST_ASSERT_FALSE(filter.baro_anchored());

    // A pressure altitude 500 m off, as a station well above sea level reads.
    filter.SetBaroBias(512.0f, 2.0f);

    TEST_ASSERT_TRUE(filter.baro_anchored());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 12.0f, filter.state(NavFilter::kPosUp));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 500.0f, filter.state(NavFilter::kBaroBias));
}

// With its offset well known, a barometer that climbs ten metres must move
// the height estimate ten metres. This is the fine detail the sensor is for.
void test_barometer_tracks_a_climb() {
    NavFilter filter;
    const float ground[3] = {0.0f, 0.0f, 0.0f};
    filter.SetPosition(ground, 1.0f);
    filter.SetBaroBias(512.0f, 0.1f);  // offset confidently known

    for (int i = 0; i < 40; i++) {
        PredictFor(filter, kAccelAtRest, 0.05f);
        filter.UpdateBaroAltitude(522.0f, 0.3f);
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, 10.0f, filter.state(NavFilter::kPosUp));
}

// The barometer alone cannot separate a climb from a drifting offset -- it
// only ever observes their sum. The filter therefore splits an innovation
// between the two in proportion to how uncertain each currently is, and this
// pins that down, because it is the behaviour that makes the offset state
// safe: a badly-known offset soaks up the change instead of corrupting height.
void test_barometer_innovation_splits_by_prior_uncertainty() {
    NavFilter filter;
    const float ground[3] = {0.0f, 0.0f, 0.0f};
    filter.SetPosition(ground, 1.0f);  // height variance 1.0
    filter.SetBaroBias(500.0f, 1.0f);  // offset variance 1.0, equally unsure

    const float height_before = filter.state(NavFilter::kPosUp);
    const float offset_before = filter.state(NavFilter::kBaroBias);
    filter.UpdateBaroAltitude(510.0f, 0.001f);  // a 10 m step, trusted fully

    const float height_moved = filter.state(NavFilter::kPosUp) - height_before;
    const float offset_moved = filter.state(NavFilter::kBaroBias) - offset_before;

    // Equal priors, so the step is shared equally, and the two must still add
    // up to the full innovation.
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 5.0f, height_moved);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 5.0f, offset_moved);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, height_moved + offset_moved);
}

// A barometer whose absolute datum is badly wrong must still be usable: GPS
// pins the height, the offset state absorbs the error, and the two agree.
// This is the property that makes the sensor worth having at all.
void test_wrong_barometer_datum_is_absorbed_by_the_offset() {
    NavFilter filter;
    const float fix[3] = {0.0f, 0.0f, 0.0f};
    filter.SetPosition(fix, 3.0f);
    // Anchored 30 m wrong on purpose, and told the anchor is trustworthy, so
    // only the GPS updates below can correct it.
    filter.SetBaroBias(530.0f, 2.0f);

    for (int i = 0; i < 300; i++) {
        PredictFor(filter, kAccelAtRest, 0.05f);
        filter.UpdateBaroAltitude(500.0f, 0.3f);    // true offset is 500
        filter.UpdateGpsPosition(fix, 3.0f, 6.0f);  // truth: height 0
        filter.UpdateZeroVelocity(0.02f);
    }

    TEST_ASSERT_FLOAT_WITHIN(1.5f, 0.0f, filter.state(NavFilter::kPosUp));
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 500.0f, filter.state(NavFilter::kBaroBias));
}

// The point of adding the sensor: height should end up better known with the
// barometer than with GPS altitude alone.
void test_barometer_tightens_the_height_envelope() {
    const float fix[3] = {0.0f, 0.0f, 0.0f};

    NavFilter gps_only;
    gps_only.SetPosition(fix, 3.0f);
    NavFilter with_baro;
    with_baro.SetPosition(fix, 3.0f);
    with_baro.SetBaroBias(500.0f, 2.0f);

    for (int i = 0; i < 200; i++) {
        PredictFor(gps_only, kAccelAtRest, 0.05f);
        PredictFor(with_baro, kAccelAtRest, 0.05f);
        gps_only.UpdateGpsPosition(fix, 3.0f, 6.0f);
        with_baro.UpdateGpsPosition(fix, 3.0f, 6.0f);
        with_baro.UpdateBaroAltitude(500.0f, 0.3f);
    }

    TEST_ASSERT_TRUE_MESSAGE(
        with_baro.ThreeSigma(NavFilter::kPosUp) < gps_only.ThreeSigma(NavFilter::kPosUp),
        "adding the barometer must reduce the height uncertainty");
}

// A general-Jacobian update of a single state must match the single-state
// helper exactly -- the helper is now expressed in terms of the general form,
// and this pins that equivalence down.
void test_general_update_matches_single_state_update() {
    NavFilter a;
    NavFilter b;
    a.UpdateScalar(NavFilter::kPosEast, 7.0f, 4.0f);

    float h[NavFilter::kNumStates] = {};
    h[NavFilter::kPosEast] = 1.0f;
    b.UpdateScalarWithJacobian(h, 7.0f, 4.0f);

    for (int i = 0; i < NavFilter::kNumStates; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, a.state(i), b.state(i));
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, a.variance(i), b.variance(i));
    }
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

// The coasting path has its own timestep guard, and it is the one that matters
// most: coasting runs precisely when the IMU has gone, which is also when a
// stalled task is most likely to hand the filter a nonsense dt.
void test_coasting_rejects_a_bad_timestep() {
    NavFilter filter;
    NavFilter::Params params;
    filter.Reset();
    filter.UpdateScalar(NavFilter::kVelEast, 3.0f, 0.01f);

    const float position = filter.state(NavFilter::kPosEast);
    const float variance = filter.variance(NavFilter::kPosEast);
    const float bad_steps[] = {0.0f, -0.01f, 0.6f};
    for (const float dt : bad_steps) {
        filter.PredictCoasting(dt, params);
        TEST_ASSERT_EQUAL_FLOAT(position, filter.state(NavFilter::kPosEast));
        TEST_ASSERT_EQUAL_FLOAT(variance, filter.variance(NavFilter::kPosEast));
    }

    // A good step still moves it, so the guard is not rejecting everything.
    filter.PredictCoasting(0.1f, params);
    TEST_ASSERT_TRUE(filter.state(NavFilter::kPosEast) > position);
}

// A measurement carrying no information -- an all-zero Jacobian with zero
// variance -- leaves the innovation covariance at zero. Dividing by it would
// fill every state with NaN, so the update has to decline instead.
void test_degenerate_measurement_is_declined() {
    NavFilter filter;
    filter.Reset();
    const float here[3] = {4.0f, -2.0f, 1.0f};
    filter.SetPosition(here, 2.0f);

    float h[NavFilter::kNumStates] = {};
    filter.UpdateScalarWithJacobian(h, 5.0f, 0.0f);

    TEST_ASSERT_EQUAL_FLOAT(4.0f, filter.state(NavFilter::kPosEast));
    TEST_ASSERT_FALSE(filter.IsDiverged());
}

// The envelope reads zero rather than NaN when a diagonal is not positive.
// SetPosition with a zero sigma is the ordinary way that happens.
void test_three_sigma_is_zero_for_a_collapsed_variance() {
    NavFilter filter;
    filter.Reset();
    const float here[3] = {1.0f, 2.0f, 3.0f};
    filter.SetPosition(here, 0.0f);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, filter.ThreeSigma(NavFilter::kPosEast));
    TEST_ASSERT_TRUE(filter.ThreeSigma(NavFilter::kVelEast) > 0.0f);
}

// Divergence is about plausibility, not just NaN -- a runaway state stays
// perfectly finite all the way out. Each route to it is checked separately.
void test_divergence_catches_nan_position_and_speed() {
    NavFilter healthy;
    healthy.Reset();
    TEST_ASSERT_FALSE(healthy.IsDiverged());

    NavFilter not_a_number;
    not_a_number.Reset();
    not_a_number.UpdateScalar(NavFilter::kPosEast, NAN, 1.0f);
    TEST_ASSERT_TRUE(not_a_number.IsDiverged());

    NavFilter far_away;
    far_away.Reset();
    const float off_the_map[3] = {2.0f * NavFilter::kMaxPlausiblePositionM, 0.0f, 0.0f};
    far_away.SetPosition(off_the_map, 1.0f);
    TEST_ASSERT_TRUE(far_away.IsDiverged());

    NavFilter too_fast;
    too_fast.Reset();
    too_fast.UpdateScalar(NavFilter::kVelEast, 5.0f * NavFilter::kMaxPlausibleSpeedMps, 0.01f);
    TEST_ASSERT_TRUE(too_fast.IsDiverged());

    // A non-finite covariance, which is what a NaN tuning knob arriving from
    // the dashboard would produce: the states stay finite and plausible, and
    // only the variance gives it away.
    NavFilter bad_noise;
    bad_noise.Reset();
    NavFilter::Params nonsense;
    nonsense.sigma_accel = NAN;
    const float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float level[3] = {0.0f, 0.0f, NavFilter::kGravityMps2};
    bad_noise.Predict(level, identity, 0.01f, nonsense);
    TEST_ASSERT_TRUE(isfinite(bad_noise.state(NavFilter::kPosEast)));
    TEST_ASSERT_TRUE(bad_noise.IsDiverged());
}

// Snapping to a fix asserts the position, so every correlation the filter had
// built up against the old one is void. Leaving those cross terms behind next
// to a much smaller variance makes P indefinite -- not a slow decay but an
// immediate fault, and one the diagonal-only IsDiverged() cannot see until the
// damage is done.
//
// The path that exposes it is an ordinary moving cold start: VQF never reports
// rest, so no zero-velocity update bounds the pos/vel correlation, which
// reaches 0.996 in a minute of free-running.
void test_snapping_to_a_fix_leaves_a_consistent_covariance() {
    NavFilter filter;
    PredictFor(filter, kAccelAtRest, 60.0f);

    const float fix[3] = {0.0f, 0.0f, 0.0f};
    filter.SetPosition(fix, 5.0f);

    // The same fix's own ground speed, applied immediately afterwards exactly
    // as ApplyGpsFix does. With a stale cross-covariance this threw position
    // 30 m off the fix it had just been snapped to and drove its variance
    // negative -- from one self-consistent measurement.
    filter.UpdateGpsVelocity(1.0f, 0.0f, 0.5f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, filter.state(NavFilter::kPosEast));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, filter.state(NavFilter::kPosNorth));
    TEST_ASSERT_TRUE(filter.variance(NavFilter::kPosEast) > 0.0f);
    TEST_ASSERT_TRUE(filter.variance(NavFilter::kPosNorth) > 0.0f);
    TEST_ASSERT_FALSE(filter.IsDiverged());
}

// ── GPS innovation gate (NavFilter::GpsPositionNis) ──────────────────────────

// A fix where the filter already expects to be is unsurprising by definition.
void test_a_fix_where_the_filter_expects_one_scores_near_zero() {
    NavFilter filter;
    const float fix[3] = {0.0f, 0.0f, 0.0f};
    TEST_ASSERT_TRUE(filter.GpsPositionNis(fix, 5.0f) < 1.0f);
}

// The case the reported quality cannot catch: nine satellites, excellent HDOP,
// ranging off a reflection.
void test_a_multipath_jump_scores_far_past_the_gate() {
    NavFilter filter;
    // Converge first, so the filter has a tight belief to be surprised against.
    for (int i = 0; i < 30; i++) {
        const float here[3] = {0.0f, 0.0f, 0.0f};
        filter.UpdateGpsPosition(here, 5.0f, 6.0f);
    }
    const float jumped[3] = {60.0f, 0.0f, 0.0f};
    TEST_ASSERT_TRUE(filter.GpsPositionNis(jumped, 5.0f) > 13.8f);
}

// A wide prior has to forgive a distant fix, or the gate would reject exactly
// the fixes a lost filter needs most.
void test_an_uncertain_filter_accepts_a_distant_fix() {
    NavFilter filter;  // reset: position 1-sigma is 10 m
    PredictFor(filter, kAccelAtRest, 120.0f);
    const float far_away[3] = {200.0f, 0.0f, 0.0f};
    TEST_ASSERT_TRUE(filter.GpsPositionNis(far_away, 5.0f) < 13.8f);
}

// Widening the measurement sigma has to soften the gate: a fix the receiver
// has already told us is poor should not also be judged as if it were good.
void test_a_wider_sigma_softens_the_gate() {
    NavFilter filter;
    for (int i = 0; i < 30; i++) {
        const float here[3] = {0.0f, 0.0f, 0.0f};
        filter.UpdateGpsPosition(here, 5.0f, 6.0f);
    }
    const float jumped[3] = {60.0f, 0.0f, 0.0f};
    TEST_ASSERT_TRUE(filter.GpsPositionNis(jumped, 40.0f) < filter.GpsPositionNis(jumped, 5.0f));
}

// ── GPS fix quality (gps_quality.h) ──────────────────────────────────────────

// The measured sigmas were taken at open-sky geometry, so that is exactly the
// case the scale has to leave alone -- otherwise this change silently retunes
// a filter that was calibrated against real data.
void test_a_good_fix_is_trusted_exactly_as_measured() {
    const GpsTrust trust = GpsFixTrust(9, 1.0f);
    TEST_ASSERT_TRUE(trust.accepted);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, trust.sigma_scale);
}

void test_poor_geometry_widens_the_sigma_in_proportion_to_hdop() {
    const GpsTrust trust = GpsFixTrust(9, 3.5f);
    TEST_ASSERT_TRUE(trust.accepted);
    TEST_ASSERT_EQUAL_FLOAT(3.5f, trust.sigma_scale);
}

// The whole point of the floor: flattering geometry is not evidence that
// multipath went away, and multipath is what the measured scatter is made of.
void test_excellent_geometry_cannot_tighten_the_sigma() {
    const GpsTrust trust = GpsFixTrust(12, 0.5f);
    TEST_ASSERT_TRUE(trust.accepted);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, trust.sigma_scale);
}

// Four satellites is a valid 3D fix with no redundancy at all, so it is used
// but distrusted. Three is not a 3D fix: the altitude is assumed, not solved.
void test_thin_constellations_are_distrusted_then_rejected() {
    const GpsTrust four = GpsFixTrust(4, 1.0f);
    TEST_ASSERT_TRUE(four.accepted);
    TEST_ASSERT_TRUE(four.sigma_scale > 1.0f);

    const GpsTrust five = GpsFixTrust(5, 1.0f);
    TEST_ASSERT_TRUE(five.accepted);
    TEST_ASSERT_TRUE(five.sigma_scale > 1.0f);
    // Redundancy has to buy something: five must be trusted more than four.
    TEST_ASSERT_TRUE(five.sigma_scale < four.sigma_scale);

    TEST_ASSERT_FALSE(GpsFixTrust(3, 1.0f).accepted);
    TEST_ASSERT_FALSE(GpsFixTrust(0, 1.0f).accepted);
}

// Both degradations apply at once -- a four-satellite fix in bad geometry is
// worse than either fault alone.
void test_bad_geometry_and_few_satellites_compound() {
    const GpsTrust both = GpsFixTrust(4, 4.0f);
    TEST_ASSERT_TRUE(both.accepted);
    TEST_ASSERT_TRUE(both.sigma_scale > GpsFixTrust(9, 4.0f).sigma_scale);
    TEST_ASSERT_TRUE(both.sigma_scale > GpsFixTrust(4, 1.0f).sigma_scale);
}

// A missing HDOP reads as 0, and 0 must never be taken at face value: as a
// scale it would drive the sigma to zero and the Kalman gain to 1, handing the
// filter's entire position over to one unvetted fix.
void test_an_unreported_hdop_is_penalised_not_believed() {
    const GpsTrust trust = GpsFixTrust(9, 0.0f);
    TEST_ASSERT_TRUE(trust.accepted);
    TEST_ASSERT_TRUE(trust.sigma_scale > 1.0f);

    const GpsTrust nan_hdop = GpsFixTrust(9, NAN);
    TEST_ASSERT_TRUE(nan_hdop.accepted);
    TEST_ASSERT_TRUE(isfinite(nan_hdop.sigma_scale));
    TEST_ASSERT_TRUE(nan_hdop.sigma_scale > 1.0f);
}

// 99.99 is how a receiver spells "no value", and it must not be mistaken for
// the merely-poor geometry that an unreported HDOP gets.
void test_the_invalid_hdop_sentinel_is_rejected() {
    TEST_ASSERT_FALSE(GpsFixTrust(9, 99.99f).accepted);
    TEST_ASSERT_FALSE(GpsFixTrust(9, 50.0f).accepted);
}

// Whatever the receiver claims, the scale never argues a sigma below the
// measured one. Sweeping the whole plausible input space is the cheapest way
// to be sure no combination slips through.
void test_no_reported_quality_can_increase_trust() {
    for (int sats = 0; sats <= 20; sats++) {
        for (float hdop = -1.0f; hdop <= 25.0f; hdop += 0.25f) {
            const GpsTrust trust = GpsFixTrust(static_cast<uint8_t>(sats), hdop);
            if (!trust.accepted) {
                continue;
            }
            TEST_ASSERT_TRUE(isfinite(trust.sigma_scale));
            TEST_ASSERT_TRUE(trust.sigma_scale >= 1.0f);
        }
    }
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
    RUN_TEST(test_anchoring_the_barometer_does_not_move_height);
    RUN_TEST(test_barometer_tracks_a_climb);
    RUN_TEST(test_barometer_innovation_splits_by_prior_uncertainty);
    RUN_TEST(test_wrong_barometer_datum_is_absorbed_by_the_offset);
    RUN_TEST(test_barometer_tightens_the_height_envelope);
    RUN_TEST(test_general_update_matches_single_state_update);
    RUN_TEST(test_bad_timestep_is_rejected);
    RUN_TEST(test_coasting_rejects_a_bad_timestep);
    RUN_TEST(test_degenerate_measurement_is_declined);
    RUN_TEST(test_three_sigma_is_zero_for_a_collapsed_variance);
    RUN_TEST(test_divergence_catches_nan_position_and_speed);
    RUN_TEST(test_geo_degree_scales_are_physical);
    RUN_TEST(test_geo_projection_is_relative_to_origin);
    RUN_TEST(test_course_maps_to_enu_velocity);
    RUN_TEST(test_snapping_to_a_fix_leaves_a_consistent_covariance);
    RUN_TEST(test_a_fix_where_the_filter_expects_one_scores_near_zero);
    RUN_TEST(test_a_multipath_jump_scores_far_past_the_gate);
    RUN_TEST(test_an_uncertain_filter_accepts_a_distant_fix);
    RUN_TEST(test_a_wider_sigma_softens_the_gate);
    RUN_TEST(test_a_good_fix_is_trusted_exactly_as_measured);
    RUN_TEST(test_poor_geometry_widens_the_sigma_in_proportion_to_hdop);
    RUN_TEST(test_excellent_geometry_cannot_tighten_the_sigma);
    RUN_TEST(test_thin_constellations_are_distrusted_then_rejected);
    RUN_TEST(test_bad_geometry_and_few_satellites_compound);
    RUN_TEST(test_an_unreported_hdop_is_penalised_not_believed);
    RUN_TEST(test_the_invalid_hdop_sentinel_is_rejected);
    RUN_TEST(test_no_reported_quality_can_increase_trust);
    return UNITY_END();
}
