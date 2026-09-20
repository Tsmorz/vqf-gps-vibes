#pragma once

// Geodetic (latitude/longitude/altitude) to local tangent-plane ENU
// conversion. Pure math with no Arduino dependency so it can be unit tested
// on the host -- see test/test_nav/.

#include <math.h>

namespace geo {

// The point the local ENU frame is anchored to: the first valid GPS fix after
// boot. Everything the dashboard plots is relative to this.
struct Origin {
    bool valid = false;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m = 0.0;
};

// Metres of northing per degree of latitude at `lat_deg`, and metres of
// easting per degree of longitude at the same latitude. These are the standard
// truncated series for the WGS84 ellipsoid; they are accurate to well under a
// metre per degree, which is far below GPS noise.
inline double MetersPerDegreeLat(double lat_deg) {
    const double phi = lat_deg * M_PI / 180.0;
    return 111132.92 - 559.82 * cos(2 * phi) + 1.175 * cos(4 * phi) - 0.0023 * cos(6 * phi);
}

inline double MetersPerDegreeLon(double lat_deg) {
    const double phi = lat_deg * M_PI / 180.0;
    return 111412.84 * cos(phi) - 93.5 * cos(3 * phi) + 0.118 * cos(5 * phi);
}

// Projects a fix onto the flat tangent plane at `origin`, writing
// [east, north, up] in metres. The flat-Earth approximation costs well under a
// centimetre of error per kilometre from the origin, which is negligible next
// to the few metres of GPS noise this filter is built around -- and it avoids
// the discontinuities a UTM projection would introduce at a zone boundary.
inline void ToEnu(const Origin& origin, double lat_deg, double lon_deg, double alt_m,
                  float out_enu[3]) {
    out_enu[0] =
        static_cast<float>((lon_deg - origin.lon_deg) * MetersPerDegreeLon(origin.lat_deg));
    out_enu[1] =
        static_cast<float>((lat_deg - origin.lat_deg) * MetersPerDegreeLat(origin.lat_deg));
    out_enu[2] = static_cast<float>(alt_m - origin.alt_m);
}

// Converts the PA1010D's ground speed and course-over-ground into horizontal
// ENU velocity components. Course is degrees clockwise from true north, so
// north is the cosine component and east the sine.
inline void CourseToEnuVelocity(float speed_mps, float course_deg, float& east_mps,
                                float& north_mps) {
    const float course_rad = course_deg * static_cast<float>(M_PI) / 180.0f;
    east_mps = speed_mps * sinf(course_rad);
    north_mps = speed_mps * cosf(course_rad);
}

// Knots to metres per second -- the NMEA RMC sentence reports speed in knots.
inline float KnotsToMps(float knots) {
    return knots * 0.514444f;
}

}  // namespace geo
