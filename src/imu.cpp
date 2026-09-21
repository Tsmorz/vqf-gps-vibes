#include "imu.h"

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "config.h"
#include "i2c_bus.h"
#include "mag_cal.h"
#include "sensor_limits.h"

namespace {

Adafruit_LSM6DSOX lsm;
Adafruit_LIS3MDL lis;

// Per-chip health. `address` is remembered from a successful probe so a
// reconnect does not have to re-guess between the primary and alternate
// address the Adafruit breakouts can be strapped to.
struct SensorHealth {
    bool healthy = false;
    uint8_t address = 0;
    int consecutive_failures = 0;
    uint32_t total_failures = 0;
    uint32_t last_retry_ms = 0;
    bool reconnected = false;  // set on recovery, cleared once acted on
};

SensorHealth accel_gyro;
SensorHealth magnetometer;

// ── Magnetometer calibration ────────────────────────────────────────────────
// The fit is applied on core 1 as samples arrive; start/finish/clear come from
// the web handler on core 0. The spinlock covers both the request flags and
// the published status, and every critical section here is a few assignments.
MagCalibration mag_cal;
MagCalCollector mag_collector;
bool mag_collecting = false;
float mag_field_ut = 0.0f;
portMUX_TYPE mag_lock = portMUX_INITIALIZER_UNLOCKED;

constexpr const char* kMagCalNamespace = "magcal";

// Restores the calibration saved by a previous sweep, so the board comes up
// with a usable heading instead of needing a sweep after every reboot.
void LoadMagCalibration() {
    Preferences prefs;
    if (!prefs.begin(kMagCalNamespace, true)) {
        return;
    }
    const float offset_x = prefs.getFloat("ox", NAN);
    if (isfinite(offset_x)) {
        mag_cal.offset[0] = offset_x;
        mag_cal.offset[1] = prefs.getFloat("oy", 0.0f);
        mag_cal.offset[2] = prefs.getFloat("oz", 0.0f);
        mag_cal.scale[0] = prefs.getFloat("sx", 1.0f);
        mag_cal.scale[1] = prefs.getFloat("sy", 1.0f);
        mag_cal.scale[2] = prefs.getFloat("sz", 1.0f);
        mag_cal.valid = true;
        Serial.printf("[imu] mag calibration restored: offset %.1f %.1f %.1f uT\n",
                      mag_cal.offset[0], mag_cal.offset[1], mag_cal.offset[2]);
    }
    prefs.end();
}

void SaveMagCalibration(const MagCalibration& cal) {
    Preferences prefs;
    if (!prefs.begin(kMagCalNamespace, false)) {
        return;
    }
    prefs.putFloat("ox", cal.offset[0]);
    prefs.putFloat("oy", cal.offset[1]);
    prefs.putFloat("oz", cal.offset[2]);
    prefs.putFloat("sx", cal.scale[0]);
    prefs.putFloat("sy", cal.scale[1]);
    prefs.putFloat("sz", cal.scale[2]);
    prefs.end();
}

// How often to confirm each chip still acknowledges on the bus.
//
// This probe is the only reliable way to notice an unplugged sensor.
// Adafruit_LSM6DS::getEvent() ends in an unconditional `return true;` and the
// _read() behind it is declared void, so a failed I2C transfer never reaches
// the caller -- an unplugged LSM6DSOX reads as a clean (0, 0, 0), which the
// navigation filter would faithfully integrate as free fall. Asking the
// address whether anything is still there costs one tiny transaction and
// cannot be fooled the same way.
constexpr uint32_t kPresenceCheckIntervalMs = 250;
uint32_t last_presence_check_ms = 0;

// Consecutive byte-identical magnetometer readings tolerated before the chip
// is presumed to have stopped converting.
//
// A live magnetometer always dithers in its low bits, so identical readings
// for this long cannot happen by chance. Repeats themselves are normal -- the
// part runs at 155 Hz and is read at 200 Hz -- which is why the threshold is
// over a second's worth rather than a handful. Without this a frozen chip
// reports healthy forever: it still acknowledges its address, and the driver
// still returns success, exactly as when its cable is pulled.
constexpr int kMagStaleSamples = 250;
float last_mag_reading[3] = {0.0f, 0.0f, 0.0f};
int identical_mag_samples = 0;

// Returns true if the magnetometer has produced a genuinely new reading.
bool MagIsLive(const float mag[3]) {
    const bool same = mag[0] == last_mag_reading[0] && mag[1] == last_mag_reading[1] &&
                      mag[2] == last_mag_reading[2];
    if (!same) {
        last_mag_reading[0] = mag[0];
        last_mag_reading[1] = mag[1];
        last_mag_reading[2] = mag[2];
        identical_mag_samples = 0;
        return true;
    }
    return ++identical_mag_samples < kMagStaleSamples;
}

// The bus generation these chips were initialised against. A bus recovery
// invalidates their device handles, and the Adafruit driver cannot tell us --
// it reports success while every transfer returns ESP_ERR_INVALID_STATE and
// hands back corrupted bytes. Re-initialising is the only way back.
uint32_t initialised_bus_generation = 0;

// ── Full-scale ranges ───────────────────────────────────────────────────────
// The ranges in force, and the full scale they imply.
//
// Touched only from the estimator task: ImuSetRanges() is called from Tick(),
// and every read of these happens in that same task. No lock is needed, and
// one should not be added casually -- the web handler deliberately does not
// call in here. It only moves the requested numbers into FilterParams, and
// Tick() applies them, because this task owns the bus.
int accel_range_g = IMU_ACCEL_RANGE_G;
int gyro_range_dps = IMU_GYRO_RANGE_DPS;
int mag_range_gauss = IMU_MAG_RANGE_GAUSS;
sensor_limits::FullScale full_scale = sensor_limits::kDefaultFullScale;

constexpr int kAccelRangesG[] = {2, 4, 8, 16};
constexpr int kGyroRangesDps[] = {125, 250, 500, 1000, 2000};
constexpr int kMagRangesGauss[] = {4, 8, 12, 16};

// Snaps a request to the nearest range the chip actually has. A value that is
// not offered has to become *something*, and silently picking the default
// would hide a typo in a build flag.
int SnapToSupported(int requested, const int* supported, int count) {
    int best = supported[0];
    int best_distance = requested > best ? requested - best : best - requested;
    for (int i = 1; i < count; i++) {
        const int distance =
            requested > supported[i] ? requested - supported[i] : supported[i] - requested;
        if (distance < best_distance) {
            best = supported[i];
            best_distance = distance;
        }
    }
    return best;
}

// The accelerometer enum is not in numeric order in the Adafruit header --
// 2 g = 0, 16 g = 1, 4 g = 2, 8 g = 3 -- so the mapping has to be spelled out.
// Deriving it arithmetically from the range would quietly select 16 g where
// 4 g was asked for.
lsm6ds_accel_range_t AccelRangeEnum(int range_g) {
    switch (range_g) {
        case 2:
            return LSM6DS_ACCEL_RANGE_2_G;
        case 8:
            return LSM6DS_ACCEL_RANGE_8_G;
        case 16:
            return LSM6DS_ACCEL_RANGE_16_G;
        default:
            return LSM6DS_ACCEL_RANGE_4_G;
    }
}

lsm6ds_gyro_range_t GyroRangeEnum(int range_dps) {
    switch (range_dps) {
        case 125:
            return LSM6DS_GYRO_RANGE_125_DPS;
        case 250:
            return LSM6DS_GYRO_RANGE_250_DPS;
        case 500:
            return LSM6DS_GYRO_RANGE_500_DPS;
        case 2000:
            return LSM6DS_GYRO_RANGE_2000_DPS;
        default:
            return LSM6DS_GYRO_RANGE_1000_DPS;
    }
}

lis3mdl_range_t MagRangeEnum(int range_gauss) {
    switch (range_gauss) {
        case 8:
            return LIS3MDL_RANGE_8_GAUSS;
        case 12:
            return LIS3MDL_RANGE_12_GAUSS;
        case 16:
            return LIS3MDL_RANGE_16_GAUSS;
        default:
            return LIS3MDL_RANGE_4_GAUSS;
    }
}

lsm6ds_data_rate_t AccelGyroOdrEnum(int hz) {
    switch (hz) {
        case 26:
            return LSM6DS_RATE_26_HZ;
        case 52:
            return LSM6DS_RATE_52_HZ;
        case 104:
            return LSM6DS_RATE_104_HZ;
        case 416:
            return LSM6DS_RATE_416_HZ;
        case 833:
            return LSM6DS_RATE_833_HZ;
        case 1666:
            return LSM6DS_RATE_1_66K_HZ;
        default:
            return LSM6DS_RATE_208_HZ;
    }
}

// Applies the ranges and output data rates the filter expects.
//
// Called on every successful begin_I2C() as well as on a range change, so a
// chip that drops off the bus and comes back returns on the range the user
// selected rather than reverting to the compile-time default.
//
// The rate comes from IMU_ACCEL_GYRO_ODR_HZ -- 208 Hz against the 200 Hz tick,
// chosen so the part's own anti-alias filter does the band-limiting the polled
// read path cannot. See the comment on that define for what it trades away.
//
// The LIS3MDL tops out at 155 Hz, below the tick rate, so some ticks re-read
// the same magnetometer sample -- harmless, because VQF's magnetic correction
// is a heavily low-pass-filtered term with a ~9 s time constant and does not
// need gyro-rate updates.
void ConfigureAccelGyro() {
    const lsm6ds_data_rate_t rate = AccelGyroOdrEnum(IMU_ACCEL_GYRO_ODR_HZ);
    lsm.setAccelRange(AccelRangeEnum(accel_range_g));
    lsm.setGyroRange(GyroRangeEnum(gyro_range_dps));
    lsm.setAccelDataRate(rate);
    lsm.setGyroDataRate(rate);
}

void ConfigureMagnetometer() {
    lis.setRange(MagRangeEnum(mag_range_gauss));
    // setDataRate() picks the performance mode itself -- 155 Hz is only
    // reachable in ultra-high-performance mode, so the library switches to it.
    // Calling setPerformanceMode() afterwards overwrites that into a
    // configuration the part cannot satisfy, and it stops converting: the
    // readings freeze at one value while the chip still acknowledges on the
    // bus and the driver still reports success. Do not add a
    // setPerformanceMode() call here.
    lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}

// Tries the primary then the alternate address, recording whichever answers.
bool BeginAccelGyro() {
    for (uint8_t address : {LSM6DSOX_ADDR_PRIMARY, LSM6DSOX_ADDR_ALT}) {
        if (!lsm.begin_I2C(address, &I2cWire(I2cBus::kImu))) {
            continue;
        }
        ConfigureAccelGyro();
        accel_gyro.address = address;
        accel_gyro.healthy = true;
        accel_gyro.consecutive_failures = 0;
        Serial.printf("[imu] LSM6DSOX ready at 0x%02X\n", address);
        return true;
    }
    return false;
}

bool BeginMagnetometer() {
    for (uint8_t address : {LIS3MDL_ADDR_PRIMARY, LIS3MDL_ADDR_ALT}) {
        if (!lis.begin_I2C(address, &I2cWire(I2cBus::kImu))) {
            continue;
        }
        ConfigureMagnetometer();
        magnetometer.address = address;
        magnetometer.healthy = true;
        magnetometer.consecutive_failures = 0;
        Serial.printf("[imu] LIS3MDL ready at 0x%02X\n", address);
        return true;
    }
    return false;
}

// Records one failed read and takes the chip offline once failures pile up.
// A few failures on a flexing cable are normal and must not cause a reinit
// storm, so the chip is only declared lost after a run of them.
void NoteFailure(SensorHealth& health, const char* name) {
    health.total_failures++;
    if (++health.consecutive_failures < SENSOR_MAX_CONSEC_FAILS) {
        return;
    }
    if (health.healthy) {
        Serial.printf("[imu] %s lost after %d consecutive failures\n", name,
                      health.consecutive_failures);
    }
    health.healthy = false;
}

void NoteSuccess(SensorHealth& health) {
    health.consecutive_failures = 0;
    // Evidence that the bus itself is alive, whatever other sensors are doing.
    I2cNoteTransferOk(I2cBus::kImu);
}

// Takes a chip offline the moment it stops acknowledging. Unlike a failed
// read, a missing ACK is unambiguous, so there is no tolerance count here.
void CheckPresence(SensorHealth& health, const char* name) {
    if (!health.healthy || health.address == 0) {
        return;
    }
    if (I2cDeviceResponds(I2cBus::kImu, health.address)) {
        return;
    }
    Serial.printf("[imu] %s stopped acknowledging at 0x%02X -- offline\n", name, health.address);
    health.healthy = false;
    health.total_failures++;
}

// Probes both chips, rate-limited so the check never competes with sensor
// reads for bus time.
void CheckPresenceOfBothChips() {
    const uint32_t now = millis();
    if (now - last_presence_check_ms < kPresenceCheckIntervalMs) {
        return;
    }
    last_presence_check_ms = now;
    CheckPresence(accel_gyro, "LSM6DSOX");
    CheckPresence(magnetometer, "LIS3MDL");
}

// Forces both chips to re-initialise if anything recovered the bus since they
// were set up, because their device handles no longer refer to a live bus.
void ReinitAfterBusRecovery() {
    const uint32_t generation = I2cBusGeneration(I2cBus::kImu);
    if (generation == initialised_bus_generation) {
        return;
    }
    initialised_bus_generation = generation;
    Serial.println("[imu] bus was recovered -- re-initialising both chips");
    accel_gyro.healthy = false;
    magnetometer.healthy = false;
    accel_gyro.last_retry_ms = 0;
    magnetometer.last_retry_ms = 0;
}

// Re-initialises an offline chip, at most once per SENSOR_RETRY_INTERVAL_MS so
// a permanently unplugged sensor cannot monopolise the bus. The bus itself is
// recovered first, since a mid-transfer reset can leave SDA held low.
void RetryIfOffline(SensorHealth& health, const char* name, bool (*begin_fn)()) {
    const uint32_t now = millis();
    if (health.healthy || now - health.last_retry_ms < SENSOR_RETRY_INTERVAL_MS) {
        return;
    }
    health.last_retry_ms = now;

    // A device that does not acknowledge is absent, not a wedged bus. Tearing
    // the bus down here is what used to take the IMU out whenever the GPS was
    // unplugged, so it only happens when SDA is genuinely stuck.
    if (!I2cDeviceResponds(I2cBus::kImu, health.address) && I2cBusIsWedged(I2cBus::kImu)) {
        I2cRecover(I2cBus::kImu);
    }
    if (begin_fn()) {
        health.reconnected = true;
        Serial.printf("[imu] %s reconnected\n", name);
    }
}

}  // namespace

void ImuBegin() {
    LoadMagCalibration();
    if (!BeginAccelGyro()) {
        Serial.println("[imu] LSM6DSOX not found -- will keep retrying");
    }
    if (!BeginMagnetometer()) {
        Serial.println("[imu] LIS3MDL not found -- will keep retrying");
    }
}

bool ImuRead(ImuSample& out) {
    I2cServiceWatchdog(I2cBus::kImu);
    ReinitAfterBusRecovery();
    CheckPresenceOfBothChips();
    RetryIfOffline(accel_gyro, "LSM6DSOX", BeginAccelGyro);
    RetryIfOffline(magnetometer, "LIS3MDL", BeginMagnetometer);

    if (accel_gyro.healthy) {
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t temperature_event;
        if (lsm.getEvent(&accel_event, &gyro_event, &temperature_event)) {
            out.accel[0] = accel_event.acceleration.x;
            out.accel[1] = accel_event.acceleration.y;
            out.accel[2] = accel_event.acceleration.z;
            out.gyro[0] = gyro_event.gyro.x;
            out.gyro[1] = gyro_event.gyro.y;
            out.gyro[2] = gyro_event.gyro.z;

            // The driver returns true whatever happened on the bus, so the
            // numbers themselves have to be the check.
            if (sensor_limits::AccelGyroLooksValid(out.accel, out.gyro, full_scale)) {
                out.accel_gyro_valid = true;
                NoteSuccess(accel_gyro);
            } else {
                out.accel[0] = out.accel[1] = out.accel[2] = 0.0f;
                out.gyro[0] = out.gyro[1] = out.gyro[2] = 0.0f;
                out.accel_gyro_valid = false;
                NoteFailure(accel_gyro, "LSM6DSOX");
            }
        } else {
            out.accel_gyro_valid = false;
            NoteFailure(accel_gyro, "LSM6DSOX");
        }
    } else {
        out.accel_gyro_valid = false;
    }

    if (magnetometer.healthy) {
        sensors_event_t mag_event;
        if (lis.getEvent(&mag_event)) {
            out.mag[0] = mag_event.magnetic.x;
            out.mag[1] = mag_event.magnetic.y;
            out.mag[2] = mag_event.magnetic.z;

            if (!MagIsLive(out.mag)) {
                Serial.println("[imu] LIS3MDL has stopped converting -- re-initialising");
                identical_mag_samples = 0;
                magnetometer.healthy = false;
                magnetometer.last_retry_ms = 0;
                magnetometer.total_failures++;
                out.mag[0] = out.mag[1] = out.mag[2] = 0.0f;
                out.mag_valid = false;
                return out.accel_gyro_valid;
            }

            if (!sensor_limits::MagLooksValid(out.mag, full_scale)) {
                // Clear the reading as well as flagging it. Otherwise the
                // rejected values still travel to the dashboard, where they
                // look exactly like a broken sensor.
                out.mag[0] = out.mag[1] = out.mag[2] = 0.0f;
                out.mag_valid = false;
                NoteFailure(magnetometer, "LIS3MDL");
                return out.accel_gyro_valid;
            }

            // A sweep collects raw readings; the correction is applied after,
            // so a sweep in progress never feeds on its own output. Only valid
            // samples get here, so corruption cannot poison a calibration.
            portENTER_CRITICAL(&mag_lock);
            if (mag_collecting) {
                mag_collector.Add(out.mag);
            }
            mag_cal.Apply(out.mag);
            mag_field_ut =
                sqrtf(out.mag[0] * out.mag[0] + out.mag[1] * out.mag[1] + out.mag[2] * out.mag[2]);
            portEXIT_CRITICAL(&mag_lock);

            out.mag_valid = true;
            NoteSuccess(magnetometer);
        } else {
            out.mag_valid = false;
            NoteFailure(magnetometer, "LIS3MDL");
        }
    } else {
        out.mag_valid = false;
    }

    return out.accel_gyro_valid;
}

bool ImuAccelGyroHealthy() {
    return accel_gyro.healthy;
}

bool ImuConsumeReconnectEvent() {
    if (!accel_gyro.reconnected) {
        return false;
    }
    accel_gyro.reconnected = false;
    return true;
}

bool ImuMagHealthy() {
    return magnetometer.healthy;
}

uint32_t ImuFailureCount() {
    return accel_gyro.total_failures + magnetometer.total_failures;
}

bool ImuSetRanges(int accel_g, int gyro_dps, int mag_gauss) {
    const int wanted_accel = SnapToSupported(accel_g, kAccelRangesG, 4);
    const int wanted_gyro = SnapToSupported(gyro_dps, kGyroRangesDps, 5);
    const int wanted_mag = SnapToSupported(mag_gauss, kMagRangesGauss, 4);

    const bool accel_changed = wanted_accel != accel_range_g;
    const bool gyro_changed = wanted_gyro != gyro_range_dps;
    const bool mag_changed = wanted_mag != mag_range_gauss;
    if (!accel_changed && !gyro_changed && !mag_changed) {
        return false;
    }

    accel_range_g = wanted_accel;
    gyro_range_dps = wanted_gyro;
    mag_range_gauss = wanted_mag;

    // The plausibility limits have to move with the range in the same step.
    // Leaving them behind would reject every reading past the old full scale
    // as corrupt, which on the dashboard looks exactly like a dying sensor.
    full_scale = sensor_limits::FullScaleFor(accel_range_g, gyro_range_dps, mag_range_gauss);

    // An offline chip is not written to -- the new range is already stored,
    // and BeginAccelGyro()/BeginMagnetometer() apply it when it comes back.
    if ((accel_changed || gyro_changed) && accel_gyro.healthy) {
        ConfigureAccelGyro();
    }
    if (mag_changed && magnetometer.healthy) {
        ConfigureMagnetometer();
        // The frozen-magnetometer detector compares consecutive readings, and
        // a range change puts them on a different quantisation step. Its
        // history is from the old scale, so start the run over.
        identical_mag_samples = 0;
    }

    Serial.printf("[imu] ranges now +-%d g, +-%d dps, +-%d gauss\n", accel_range_g, gyro_range_dps,
                  mag_range_gauss);
    return accel_changed;
}

void ImuGetRanges(int& accel_g, int& gyro_dps, int& mag_gauss) {
    accel_g = accel_range_g;
    gyro_dps = gyro_range_dps;
    mag_gauss = mag_range_gauss;
}

void ImuPowerDown() {
    if (accel_gyro.healthy) {
        lsm.setAccelDataRate(LSM6DS_RATE_SHUTDOWN);
        lsm.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);
    }
    if (magnetometer.healthy) {
        lis.setOperationMode(LIS3MDL_POWERDOWNMODE);
    }
    Serial.println("[imu] accel/gyro and magnetometer powered down");
}

void ImuMagCalStart() {
    portENTER_CRITICAL(&mag_lock);
    mag_collector.Reset();
    mag_collecting = true;
    portEXIT_CRITICAL(&mag_lock);
    Serial.println("[imu] mag calibration started -- turn the board through all orientations");
}

bool ImuMagCalFinish() {
    MagCalibration fitted;
    bool solved = false;
    float implied_field = 0.0f;

    portENTER_CRITICAL(&mag_lock);
    mag_collecting = false;
    implied_field = mag_collector.implied_field_ut();
    solved = mag_collector.Solve(fitted);
    if (solved) {
        mag_cal = fitted;
    }
    portEXIT_CRITICAL(&mag_lock);

    if (!solved) {
        Serial.printf(
            "[imu] mag calibration rejected -- the sweep implies only %.1f uT; turn the "
            "board through every orientation until that figure stops rising\n",
            implied_field);
        return false;
    }
    SaveMagCalibration(fitted);
    Serial.printf(
        "[imu] mag calibrated: offset %.1f %.1f %.1f uT  scale %.3f %.3f %.3f  "
        "implied field %.1f uT\n",
        fitted.offset[0], fitted.offset[1], fitted.offset[2], fitted.scale[0], fitted.scale[1],
        fitted.scale[2], implied_field);
    return true;
}

void ImuMagCalClear() {
    portENTER_CRITICAL(&mag_lock);
    mag_cal = MagCalibration();
    mag_collecting = false;
    mag_collector.Reset();
    portEXIT_CRITICAL(&mag_lock);

    Preferences prefs;
    if (prefs.begin(kMagCalNamespace, false)) {
        prefs.clear();
        prefs.end();
    }
    Serial.println("[imu] mag calibration cleared");
}

MagCalStatus ImuMagCalStatus() {
    MagCalStatus status;
    portENTER_CRITICAL(&mag_lock);
    status.calibrated = mag_cal.valid;
    status.collecting = mag_collecting;
    status.progress = mag_collector.progress();
    status.implied_field_ut = mag_collector.implied_field_ut();
    status.samples = mag_collector.sample_count();
    for (int axis = 0; axis < 3; axis++) {
        status.offset[axis] = mag_cal.offset[axis];
    }
    status.field_ut = mag_field_ut;
    portEXIT_CRITICAL(&mag_lock);
    return status;
}
