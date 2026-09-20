#pragma once

// Compile-time configuration for the VQF + GPS fusion firmware.
// Board: Unexpected Maker FeatherS3 (ESP32-S3). Every value can be overridden
// from platformio.ini with -D <NAME>=<value>.

// ── I2C ──────────────────────────────────────────────────────────────────────
// The FeatherS3 breaks out two independent I2C buses, and the sensors are
// split across them deliberately. See the comment on I2cBus in i2c_bus.h for
// the measurements behind the split; in short, the GPS's intermittent
// connector used to corrupt every sensor on a shared run, and it is also the
// heaviest consumer of bus time.
//
//   bus "imu" (Wire)   SDA 8  / SCL 9    0x6A LSM6DSOX, 0x1C LIS3MDL
//   bus "aux" (Wire1)  SDA 16 / SCL 15   0x10 PA1010D,  0x77 BMP390
//
// Run `task scan` after any rewiring -- it scans both buses and prints what
// answers on each.
#ifndef I2C_IMU_SDA_PIN
#define I2C_IMU_SDA_PIN 8
#define I2C_IMU_SCL_PIN 9
#endif

#ifndef I2C_AUX_SDA_PIN
#define I2C_AUX_SDA_PIN 16
#define I2C_AUX_SCL_PIN 15
#endif

// 100 kHz (standard mode) on both buses, chosen from measurement rather than
// preference.
//
// This started at 400 kHz on the reasoning that a combined accel+gyro+mag read
// should not eat a 5 ms tick. With four devices daisy-chained on one STEMMA
// run that turned out to be past the edge of reliable: the data came back
// inside the sensors' full scale but wrong, so nothing in software could tell
// it was corrupt. Measured over 20 s on a stationary board, GPS unplugged in
// both cases:
//
//                       400 kHz     100 kHz
//   accel sd            5.678       0.019   m/s^2
//   gyro max            21.8        0.053   rad/s
//   mag step-to-step    479         6.6     uT
//   roll sd             12.1        0.012   deg
//   yaw sd              26.4        0.097   deg
//   rejected samples    121         4
//
// The estimator still holds 200.0 Hz at 100 kHz, so the speed bought nothing
// and cost signal margin. Now that the IMU has a short bus to itself it would
// probably take 400 kHz again -- but only raise it with the same measurement
// in hand, not on the assumption that fewer devices must mean it is fine.
#ifndef I2C_IMU_CLOCK_HZ
#define I2C_IMU_CLOCK_HZ 100000
#endif
#ifndef I2C_AUX_CLOCK_HZ
#define I2C_AUX_CLOCK_HZ 100000
#endif

// Sensor addresses. Each driver also tries the alternate address, since these
// depend on a solder jumper / SDO pin state on the Adafruit breakouts.
#define LSM6DSOX_ADDR_PRIMARY 0x6A
#define LSM6DSOX_ADDR_ALT 0x6B
#define LIS3MDL_ADDR_PRIMARY 0x1C
#define LIS3MDL_ADDR_ALT 0x1E
#define PA1010D_ADDR 0x10
#define BMP390_ADDR_PRIMARY 0x77
#define BMP390_ADDR_ALT 0x76

// ── Power rails ──────────────────────────────────────────────────────────────
// LDO1 is always on and feeds the ESP32-S3 and the IMU. LDO2 is switchable,
// gated by GPIO 39, and feeds the 3V3 pin -- here the GPS and the barometer.
//
// The variant header defines LDO2 and RGB_PWR as the same pin 39, so the
// onboard LED shares that rail. Driving this pin low to darken the LED would
// cut power to the GPS and barometer; send a black pixel instead. See
// board_power.h.
#ifndef LDO2_ENABLE_PIN
#define LDO2_ENABLE_PIN 39
#endif

// ── Onboard RGB status LED ───────────────────────────────────────────────────
// WS2812B on GPIO 40, powered from LDO2 above.
// (The FeatherS3 LED is RGB, not RGBW -- there is no separate white element.)
#ifndef RGB_LED_PIN
#define RGB_LED_PIN 40
#endif
#ifndef RGB_LED_BRIGHTNESS
#define RGB_LED_BRIGHTNESS 40  // 0-255; the onboard LED is uncomfortably bright
#endif

// ── Timing ───────────────────────────────────────────────────────────────────
// Estimator tick: IMU read + VQF update + EKF propagate, 200 Hz.
#ifndef ESTIMATOR_INTERVAL_MS
#define ESTIMATOR_INTERVAL_MS 5
#endif

// How often the estimator drains the GPS.
//
// The receiver emits two sentences a second, about 150 bytes. At 100 kHz a
// 32-byte refill from the library's buffer takes nearly 3 ms, so each poll
// only moves one refill's worth -- polling at 20 Hz keeps throughput well
// ahead of what the receiver produces. (At 400 kHz, 5 Hz was enough; the
// slower bus is why this changed.)
#ifndef GPS_POLL_INTERVAL_MS
#define GPS_POLL_INTERVAL_MS 50
#endif

// Wall-clock budget for one GPS drain, in microseconds.
//
// This bound matters more than it looks. The PA1010D pads its I2C output with
// 0x0A when it has nothing to say, and the Adafruit library discards that
// padding -- so every read() of an idle receiver triggers another 32-byte I2C
// transfer. Draining by character count therefore costs hundreds of
// transfers and stalls the tick; draining against a clock cannot.
#ifndef GPS_POLL_BUDGET_US
#define GPS_POLL_BUDGET_US 3000
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

// ── WiFi ─────────────────────────────────────────────────────────────────────
// Credentials for your own network live in src/secrets.h, which is git-ignored.
// Copy src/secrets.example.h over it. With no secrets.h (or an empty SSID) the
// board goes straight to access-point mode.
#ifdef __has_include
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_PASS ""
#endif

// How long to wait for the configured network before giving up and starting
// the access point. Short enough not to be annoying when carrying the board
// outside, long enough for a router to answer a cold associate.
#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 8000
#endif

// Advertised over mDNS in station mode, so the dashboard has a stable address:
// http://vqf-gps.local/
#ifndef MDNS_HOSTNAME
#define MDNS_HOSTNAME "vqf-gps"
#endif

// Fallback access point, used when the network above is out of range. Connect
// to this SSID and open http://192.168.4.1/.
#ifndef AP_SSID
#define AP_SSID "vqf-gps"
#endif
#ifndef AP_PASS
#define AP_PASS "vqfgps123"  // WPA2 requires at least 8 characters
#endif
#ifndef AP_CHANNEL
#define AP_CHANNEL 6
#endif

// How often the estimator reads the barometer. The BMP390 is configured for
// 50 Hz with 8x pressure oversampling, so 25 Hz always sees a fresh sample
// and leaves the conversion comfortable time to finish.
#ifndef BARO_POLL_INTERVAL_MS
#define BARO_POLL_INTERVAL_MS 40
#endif

// ── Filter defaults ──────────────────────────────────────────────────────────
// Starting values for the runtime-tunable knobs exposed on the dashboard.
// See nav_filter.h for how each term enters the EKF.

// ── Measured on this hardware ────────────────────────────────────────────────
// The values below come from a 600 s stationary record at 92 Hz, analysed by
// Allan deviation (the slope -1/2 region gives the white-noise density N, the
// slope +1/2 region the random walk K). They replace the round numbers this
// project started with, several of which were off by more than an order of
// magnitude. Re-measure after any change to mounting or wiring.
//
//   accelerometer   N = 0.0076 m/s^2/sqrt(Hz)   K = 0.0014 m/s^2/sqrt(s)
//   gyroscope       N = 0.0021 rad/s/sqrt(Hz)   K = 0.00012 rad/s/sqrt(s)
//   barometer       per-sample sigma 0.030 m    drift K = 0.024 m/sqrt(s)
//   GPS scatter     east 6.05 m  north 1.86 m  up 3.09 m  (538 fixes)

// Accelerometer process noise (m/s^2/sqrt(Hz)). The dominant term: it sets how
// fast the position/velocity 3-sigma envelope grows between GPS fixes.
//
// Deliberately well above the measured sensor noise of 0.0076. This term has
// to cover more than the sensor: the accelerometer reads 10.076 m/s^2 at rest
// against a true 9.80665, a +2.74% scale error worth +0.269 m/s^2 -- some 35x
// the noise floor -- and that error scales with real acceleration, so it
// cannot all be absorbed by a constant bias state. Attitude error leaks in too
// (g * epsilon). Setting this to the sensor floor would make the filter
// confidently wrong the moment the rig moves.
//
// For reference, the free-run position 1-sigma this implies:
//   after  1 s   0.03 m
//   after  5 s   0.32 m
//   after 30 s   4.74 m
// Raise it for a dynamic platform; it is a dashboard knob for exactly that.
#ifndef DEFAULT_SIGMA_ACCEL
#define DEFAULT_SIGMA_ACCEL 0.05f
#endif

// Accelerometer bias random walk (m/s^3/sqrt(Hz)), measured at rest.
//
// A moving platform wants more than this: because the error above is a scale
// error rather than a true bias, the apparent bias shifts with acceleration --
// about 0.055 m/s^2 during a 2 m/s^2 manoeuvre -- and tracking that needs a
// far larger random walk than a stationary rig ever reveals.
#ifndef DEFAULT_SIGMA_ACCEL_BIAS
#define DEFAULT_SIGMA_ACCEL_BIAS 0.0015f
#endif

// GPS measurement noise, 1-sigma (m). Consumer GNSS horizontal accuracy is a
// few metres; vertical is roughly 2x worse.
// Measured per-axis scatter over 538 stationary fixes was 6.05 m east and
// 1.86 m north (4.5 m RMS); 5 m is that, rounded up. The error is dominated by
// slow multipath wander rather than per-fix jitter -- consecutive fixes differ
// by only ~0.3 m -- so this figure is deliberately the *total* scatter, which
// is what stops the filter chasing multipath.
#ifndef DEFAULT_SIGMA_GPS_POS_H
#define DEFAULT_SIGMA_GPS_POS_H 5.0f
#endif
// Measured 3.09 m, but left conservative: satellite geometry normally makes
// vertical worse than horizontal, and one favourable session is not evidence
// that it is better here. The barometer carries height now regardless.
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

// Barometer measurement noise, 1-sigma (m). The BMP390's own resolution is
// centimetres, but short-term pressure fluctuations -- a door opening, a gust,
// ventilation -- move the reading far more than that, and those are what this
// figure has to cover.
// Measured per-sample noise is 0.030 m (pressure sd 3.2 Pa). This is set
// higher because short-term pressure disturbance -- a door, a gust,
// ventilation -- moves the reading without the board moving, and that is what
// would otherwise be read as height.
#ifndef DEFAULT_SIGMA_BARO
#define DEFAULT_SIGMA_BARO 0.10f
#endif

// Random walk of the barometer's altitude offset (m/sqrt(s)). Weather shifts
// the sea-level reference by a few hPa over hours, which is millimetres per
// second of apparent altitude -- hence how small this is. Too large and the
// offset absorbs real climbs; too small and it cannot follow the weather.
// Measured drift of the pressure altitude was K = 0.024 m/sqrt(s), 2.02 m of
// excursion over ten minutes. The offset state has to be free enough to follow
// that, and no freer: at 0.024 m/sqrt(s) it can absorb the weather while a
// real climb of even 0.5 m/s goes almost entirely to height.
#ifndef DEFAULT_SIGMA_BARO_BIAS
#define DEFAULT_SIGMA_BARO_BIAS 0.024f
#endif

// Uncertainty assigned to the barometer offset when it is first anchored (m).
#ifndef BARO_ANCHOR_SIGMA
#define BARO_ANCHOR_SIGMA 2.0f
#endif

// How long to wait for a GPS fix before anchoring the barometer anyway.
// Long enough for VQF to converge and zero-velocity updates to settle the
// height, so the anchor is taken against a stable estimate rather than the
// boot transient. Indoors, where no fix ever arrives, this is the path that
// runs and the barometer becomes the only height reference.
#ifndef BARO_ANCHOR_FALLBACK_MS
#define BARO_ANCHOR_FALLBACK_MS 15000
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
