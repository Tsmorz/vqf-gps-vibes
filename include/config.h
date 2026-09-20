#pragma once

// Compile-time configuration for the VQF + GPS fusion firmware.
// Board: Unexpected Maker FeatherS3 (ESP32-S3). Every value can be overridden
// from platformio.ini with -D <NAME>=<value>.

// ── I2C ──────────────────────────────────────────────────────────────────────
// The FeatherS3 breaks out two independent I2C buses (SDA/SCL and SDA1/SCL1).
// `task scan` confirmed all three sensors answer on bus 0 only -- the LSM6DSOX,
// LIS3MDL and PA1010D are daisy-chained on a single STEMMA run:
//     bus0: 0x10 PA1010D, 0x1C LIS3MDL, 0x6A LSM6DSOX     bus1: empty
// Because they share one bus, all I2C traffic is issued from a single task
// (see estimator.cpp) so the GPS and IMU can never interleave transactions.
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#endif

// 400 kHz (fast mode). At the default 100 kHz a combined accel+gyro+mag read
// would consume a large share of each 5 ms estimator tick in transfer time.
#ifndef I2C_CLOCK_HZ
#define I2C_CLOCK_HZ 400000
#endif

// Sensor addresses. Each driver also tries the alternate address, since these
// depend on a solder jumper / SDO pin state on the Adafruit breakouts.
#define LSM6DSOX_ADDR_PRIMARY 0x6A
#define LSM6DSOX_ADDR_ALT 0x6B
#define LIS3MDL_ADDR_PRIMARY 0x1C
#define LIS3MDL_ADDR_ALT 0x1E
#define PA1010D_ADDR 0x10

// ── Onboard RGB status LED ───────────────────────────────────────────────────
// WS2812B on GPIO 40; GPIO 39 gates its power rail and must be driven HIGH.
// (The FeatherS3 LED is RGB, not RGBW -- there is no separate white element.)
#ifndef RGB_LED_PIN
#define RGB_LED_PIN 40
#endif
#ifndef RGB_LED_POWER_PIN
#define RGB_LED_POWER_PIN 39
#endif
#ifndef RGB_LED_BRIGHTNESS
#define RGB_LED_BRIGHTNESS 40  // 0-255; the onboard LED is uncomfortably bright
#endif

// ── Timing ───────────────────────────────────────────────────────────────────
// Estimator tick: IMU read + VQF update + EKF propagate, 200 Hz.
#ifndef ESTIMATOR_INTERVAL_MS
#define ESTIMATOR_INTERVAL_MS 5
#endif

// How often the estimator drains the GPS. The receiver emits two sentences a
// second, so 5 Hz picks them up promptly with margin to spare.
#ifndef GPS_POLL_INTERVAL_MS
#define GPS_POLL_INTERVAL_MS 200
#endif

// Wall-clock budget for one GPS drain, in microseconds.
//
// This bound matters more than it looks. The PA1010D pads its I2C output with
// 0x0A when it has nothing to say, and the Adafruit library discards that
// padding -- so every read() of an idle receiver triggers another 32-byte I2C
// transfer. Draining by character count therefore costs hundreds of
// transfers and stalls the tick; draining against a clock cannot.
#ifndef GPS_POLL_BUDGET_US
#define GPS_POLL_BUDGET_US 2500
#endif

// WebSocket telemetry rate. Deliberately far slower than the estimator: the
// browser cannot usefully render 200 Hz, and each frame costs a JSON encode.
#ifndef TELEMETRY_INTERVAL_MS
#define TELEMETRY_INTERVAL_MS 50
#endif

// An IMU sample older than this means the estimator task has stalled or the
// sensor dropped off the bus; telemetry reports the data as invalid.
#ifndef IMU_STALE_MS
#define IMU_STALE_MS 100
#endif

// A GPS fix older than this is no longer trusted for measurement updates.
#ifndef GPS_STALE_MS
#define GPS_STALE_MS 3000
#endif

// ── Sensor fault handling ────────────────────────────────────────────────────
// Consecutive failed reads before a driver declares the sensor lost and starts
// re-initialising it. A handful of failures is normal on a flexing cable.
#ifndef SENSOR_MAX_CONSEC_FAILS
#define SENSOR_MAX_CONSEC_FAILS 10
#endif

// Minimum gap between re-initialisation attempts, so a permanently unplugged
// sensor does not monopolise the bus retrying every tick.
#ifndef SENSOR_RETRY_INTERVAL_MS
#define SENSOR_RETRY_INTERVAL_MS 2000
#endif

// ── WiFi access point ────────────────────────────────────────────────────────
// The board hosts its own network; no router involved. Connect to this SSID
// and open http://192.168.4.1/.
#ifndef AP_SSID
#define AP_SSID "vqf-gps"
#endif
#ifndef AP_PASS
#define AP_PASS "vqfgps123"  // WPA2 requires at least 8 characters
#endif
#ifndef AP_CHANNEL
#define AP_CHANNEL 6
#endif

// ── Filter defaults ──────────────────────────────────────────────────────────
// Starting values for the runtime-tunable knobs exposed on the dashboard.
// See nav_filter.h for how each term enters the EKF.

// Accelerometer white noise density (m/s^2/sqrt(Hz)). This is the dominant
// process-noise term: it sets how fast the position/velocity 3-sigma envelope
// grows between GPS fixes.
#ifndef DEFAULT_SIGMA_ACCEL
#define DEFAULT_SIGMA_ACCEL 0.35f
#endif

// Accelerometer bias random walk (m/s^3/sqrt(Hz)). Governs how quickly the
// filter is willing to re-learn accel bias from GPS.
#ifndef DEFAULT_SIGMA_ACCEL_BIAS
#define DEFAULT_SIGMA_ACCEL_BIAS 0.008f
#endif

// GPS measurement noise, 1-sigma (m). Consumer GNSS horizontal accuracy is a
// few metres; vertical is roughly 2x worse.
#ifndef DEFAULT_SIGMA_GPS_POS_H
#define DEFAULT_SIGMA_GPS_POS_H 3.0f
#endif
#ifndef DEFAULT_SIGMA_GPS_POS_V
#define DEFAULT_SIGMA_GPS_POS_V 6.0f
#endif
#ifndef DEFAULT_SIGMA_GPS_VEL
#define DEFAULT_SIGMA_GPS_VEL 0.5f
#endif

// Pseudo-measurement noise for the zero-velocity update (m/s). Applied when
// VQF's rest detector says the device is stationary -- without it, indoor
// testing with no GPS fix diverges within seconds of double integration.
#ifndef DEFAULT_SIGMA_ZUPT
#define DEFAULT_SIGMA_ZUPT 0.02f
#endif

// VQF time constants (s). tauAcc sets how strongly gravity corrects gyro
// drift in roll/pitch; tauMag does the same for the magnetometer in yaw.
#ifndef DEFAULT_TAU_ACC
#define DEFAULT_TAU_ACC 3.0f
#endif
#ifndef DEFAULT_TAU_MAG
#define DEFAULT_TAU_MAG 9.0f
#endif

// ── Firmware identity ────────────────────────────────────────────────────────
#ifdef __has_include
#if __has_include("generated/version.h")
#include "generated/version.h"
#endif
#endif
#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif
