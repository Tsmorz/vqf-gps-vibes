#pragma once

#include <stdint.h>

// Driver for the Adafruit Mini GPS PA1010D on I2C.
//
// The GPS is the least reliable input in the system: it may be unplugged, and
// indoors it will sit for minutes with no satellite fix. Everything here is
// written so that "no GPS" is an ordinary state rather than a fault -- the
// navigation filter simply runs unaided and its uncertainty envelope grows.

struct GpsSample {
    bool has_fix = false;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    float alt_m = 0.0f;
    float speed_mps = 0.0f;
    float course_deg = 0.0f;
    uint8_t satellites = 0;
    float hdop = 0.0f;
    uint32_t fix_millis = 0;  // millis() when this fix was parsed
};

// Attempts to bring the receiver up and configure its NMEA output. Safe to
// call with no module attached.
void GpsBegin();

// Drains whatever NMEA the receiver has buffered and parses complete
// sentences. Non-blocking; call from the estimator tick.
void GpsPoll();

// Hands back the most recent fix exactly once, so the filter applies one
// measurement update per fix. Re-applying the same fix every tick would
// shrink the covariance without new information and make the filter
// overconfident. Returns false when no unconsumed fix is waiting.
bool GpsConsumeNewFix(GpsSample& out);

// The latest fix regardless of whether it has been consumed -- for telemetry.
GpsSample GpsLatest();

// True if the receiver is present on the bus. Independent of whether it has a
// satellite fix.
bool GpsHealthy();

uint32_t GpsFailureCount();
