#pragma once

// How far to trust one GPS fix, from what the receiver says about it.
//
// Every fix used to enter the filter with the same sigma, which is wrong in
// both directions: a four-satellite fix wedged between buildings pulled the
// estimate exactly as hard as a nine-satellite fix under open sky. The
// receiver already publishes the two numbers that tell those apart, and both
// arrive in the GGA sentence the driver is already parsing:
//
//   satellites  how much redundancy the solution has. Four is the minimum for
//               a 3D fix and leaves none -- a single bad pseudorange is then
//               undetectable and goes straight into the answer. Below four the
//               receiver has stopped solving for height and is substituting an
//               assumed altitude, which is a fabricated number the filter must
//               never see as a measurement.
//   HDOP        how much the satellite geometry amplifies ranging error into
//               horizontal position error. This is the textbook scale factor,
//               sigma = UERE * HDOP, and it is the whole reason the receiver
//               reports it.
//
// So sigma_gps_pos_h is reinterpreted as the UERE -- the sigma at HDOP 1.0 --
// and its measured 5.0 m keeps its meaning, because the 538-fix session behind
// that number was ordinary open-sky geometry.
//
// The scale is deliberately clamped at 1.0 from below: reported quality can
// widen a sigma away from the measured value but can never tighten it. Good
// geometry is not evidence that multipath went away, and multipath is what
// dominates the measured scatter -- consecutive fixes differ by ~0.3 m while
// the total spread is 4.5 m (see DEFAULT_SIGMA_GPS_POS_H). HDOP says nothing
// about that error, so it does not get to argue the sigma down.
//
// Pure math, no Arduino dependency -- unit tested on the host in
// test/test_nav/.

#include <stdint.h>

#include "config.h"

struct GpsTrust {
    // False when the fix should not reach the filter at all.
    bool accepted = false;
    // Multiplies the configured GPS sigmas. Never below 1.0.
    float sigma_scale = 1.0f;
};

// Redundancy, as distinct from geometry. HDOP describes the shape of the
// constellation but says nothing about whether the solution can check itself,
// and at four satellites it cannot: there is one observation per unknown, so
// any ranging error is absorbed silently into the position instead of showing
// up as a residual. These multipliers are engineering judgement, not
// measurement -- unlike the sigmas they scale, which were measured.
inline float GpsSatelliteFactor(uint8_t satellites) {
    if (satellites >= 6) {
        return 1.0f;
    }
    return satellites == 5 ? 1.4f : 2.0f;
}

inline GpsTrust GpsFixTrust(uint8_t satellites, float hdop) {
    GpsTrust trust;

    if (satellites < GPS_MIN_SATELLITES) {
        return trust;
    }

    // Three readings of HDOP have to stay distinct, because collapsing any two
    // of them is a silent way to over-trust a bad fix. A value above the
    // ceiling is real but unusable geometry -- and is also how a receiver
    // spells "invalid", as 99.99 -- so the fix is dropped. Zero, negative or
    // NaN means the field was not reported at all, which is unknown geometry
    // rather than good geometry, so it gets a fixed penalty instead of the
    // benefit of the doubt. Everything else is taken at face value, floored so
    // that a flattering HDOP cannot tighten the sigma past what was measured.
    if (hdop > GPS_MAX_HDOP) {
        return trust;
    }
    const bool reported = hdop > 0.0f;  // false for NaN, which is the point
    const float hdop_factor =
        !reported ? GPS_UNKNOWN_HDOP : (hdop < GPS_HDOP_FLOOR ? GPS_HDOP_FLOOR : hdop);

    trust.accepted = true;
    trust.sigma_scale = GpsSatelliteFactor(satellites) * hdop_factor;
    return trust;
}
