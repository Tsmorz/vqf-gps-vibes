#include "gps.h"

#include <Adafruit_GPS.h>
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "i2c_bus.h"

namespace {

Adafruit_GPS gps(&Wire);

bool healthy = false;
uint32_t total_failures = 0;
uint32_t last_retry_ms = 0;
uint32_t last_presence_check_ms = 0;

GpsSample latest;
bool fix_unconsumed = false;

// How often to confirm the receiver still acknowledges on the bus. Parsing
// alone cannot distinguish "unplugged" from "plugged in but no fix yet".
constexpr uint32_t kPresenceCheckIntervalMs = 2000;

// Asks for RMC (position, speed, course) and GGA (altitude, satellites, HDOP)
// once a second -- everything the filter needs and nothing it does not.
void ConfigureOutput() {
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

// Copies the library's parsed fields into our own struct. Called only after a
// sentence has parsed and reported a valid fix.
void CaptureFix() {
    latest.has_fix = true;
    latest.lat_deg = gps.latitudeDegrees;
    latest.lon_deg = gps.longitudeDegrees;
    latest.alt_m = gps.altitude;
    latest.speed_mps = gps.speed * 0.514444f;  // NMEA reports knots
    latest.course_deg = gps.angle;
    latest.satellites = gps.satellites;
    latest.hdop = gps.HDOP;
    latest.fix_millis = millis();
    fix_unconsumed = true;
}

// Re-initialises the receiver, rate-limited so an absent module does not
// monopolise the shared bus.
void RetryIfOffline() {
    const uint32_t now = millis();
    if (healthy || now - last_retry_ms < SENSOR_RETRY_INTERVAL_MS) {
        return;
    }
    last_retry_ms = now;

    if (!I2cDeviceResponds(PA1010D_ADDR)) {
        I2cRecover();
        return;
    }
    if (gps.begin(PA1010D_ADDR)) {
        ConfigureOutput();
        healthy = true;
        Serial.println("[gps] PA1010D reconnected");
    }
}

// Periodically confirms the module still answers. Catches an unplugged cable
// that parsing alone would show only as a fix that never arrives.
void CheckPresence() {
    const uint32_t now = millis();
    if (now - last_presence_check_ms < kPresenceCheckIntervalMs) {
        return;
    }
    last_presence_check_ms = now;

    if (I2cDeviceResponds(PA1010D_ADDR)) {
        return;
    }
    if (healthy) {
        Serial.println("[gps] PA1010D stopped responding");
        total_failures++;
    }
    healthy = false;
    latest.has_fix = false;
}

}  // namespace

void GpsBegin() {
    if (gps.begin(PA1010D_ADDR)) {
        ConfigureOutput();
        healthy = true;
        Serial.println("[gps] PA1010D ready at 0x10");
        return;
    }
    Serial.println("[gps] PA1010D not found -- will keep retrying");
}

void GpsPoll() {
    RetryIfOffline();
    CheckPresence();
    if (!healthy) {
        return;
    }

    // Drain against a clock, not a character count -- see GPS_POLL_BUDGET_US.
    // Stop at the first complete sentence so one tick never absorbs a whole
    // burst; the next poll, 200 ms later, picks up the rest.
    const uint32_t started_us = micros();
    while (micros() - started_us < GPS_POLL_BUDGET_US) {
        gps.read();
        if (!gps.newNMEAreceived()) {
            continue;
        }
        // parse() returns false for a checksum error or a sentence the library
        // does not handle -- both are routine, so just take the next one.
        if (!gps.parse(gps.lastNMEA())) {
            continue;
        }
        if (gps.fix) {
            CaptureFix();
        } else {
            latest.has_fix = false;
            latest.satellites = gps.satellites;
        }
        break;
    }
}

bool GpsConsumeNewFix(GpsSample& out) {
    if (!fix_unconsumed) {
        return false;
    }
    fix_unconsumed = false;
    out = latest;
    return true;
}

GpsSample GpsLatest() {
    return latest;
}

bool GpsHealthy() {
    return healthy;
}

uint32_t GpsFailureCount() {
    return total_failures;
}
