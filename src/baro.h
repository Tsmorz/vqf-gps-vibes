#pragma once

#include <stdint.h>

// Driver for the Adafruit BMP390 pressure/temperature sensor.
//
// Worth being clear about what this part does and does not do: the BMP390 has
// no built-in altitude estimation. It reports pressure and temperature, and
// nothing else. Adafruit's readAltitude() is a barometric formula applied on
// the host, and it takes the current sea-level pressure as an argument -- a
// value you do not have. Supplying the 1013.25 hPa standard instead is wrong
// by however much the weather differs, which is routinely tens of metres.
//
// So this driver deliberately does not pretend to produce absolute altitude.
// It reports a *pressure altitude* against the fixed standard reference, whose
// absolute value is unreliable but whose changes are excellent, and leaves the
// navigation filter to carry the offset as a state it can estimate (see
// kBaroBias in nav_filter.h).

struct BaroSample {
    bool valid = false;
    float pressure_pa = 0.0f;
    float temperature_c = 0.0f;
    // Height from the barometric formula against the standard reference.
    // Treat differences as meaningful and the absolute value as arbitrary.
    float pressure_altitude_m = 0.0f;
};

// Attempts to bring the sensor up. Safe to call with nothing attached.
void BaroBegin();

// Reads the sensor if a new sample is due. Returns true and fills `out` only
// on a fresh, plausible reading; returns false in between polls and on error.
bool BaroRead(BaroSample& out);

bool BaroHealthy();

uint32_t BaroFailureCount();
