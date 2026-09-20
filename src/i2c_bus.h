#pragma once

#include <stdint.h>

class TwoWire;

// The board's two I2C buses, named for what each carries.
//
// The FeatherS3 breaks out two independent buses, and the sensors are split
// across them deliberately rather than for convenience. Everything used to
// share one STEMMA run, and that made the GPS's intermittent connector a
// system-wide fault: a single 20 s capture had 113 frames in which the
// accelerometer, gyroscope and magnetometer were all dead at once, because a
// glitch on the shared lines corrupts every transfer on them. The GPS is also
// the heaviest user of bus time -- a 32-byte refill costs ~2.9 ms at 100 kHz,
// against a 5 ms estimator tick.
//
// So the high-rate, latency-critical IMU gets a short bus to itself, and the
// slow and physically less reliable devices share the other one.
enum class I2cBus {
    kImu,  // LSM6DSOX + LIS3MDL
    kAux,  // PA1010D GPS + BMP390
};

// Brings both buses up on the pins in config.h.
void I2cBeginAll();

// The underlying Arduino bus object, for passing to a sensor library.
TwoWire& I2cWire(I2cBus bus);

// Restarts one bus. Every Adafruit_BusIO device handle registered against it
// becomes invalid, so this bumps that bus's generation (see below).
void I2cRestart(I2cBus bus);

// True if the bus is genuinely stuck, rather than merely empty.
//
// A sensor reset mid-transfer can be left holding SDA low, which locks the bus
// for every device on it. A device that simply does not acknowledge is a
// different thing entirely -- it is absent, the bus is fine, and tearing the
// bus down would only harm the sensors that still work.
//
// Deliberately passive: it reads the line level without reconfiguring the pin,
// because an earlier version reconfigured it and then re-began the bus, which
// restarted the peripheral every two seconds and corrupted every IMU read. A
// check for a broken bus must not break the bus.
bool I2cBusIsWedged(I2cBus bus);

// Clocks a wedged bus free, then restarts it. Bumps the generation.
bool I2cRecover(I2cBus bus);

// Increments whenever a bus is restarted. A driver that sees this change must
// call begin_I2C() again before trusting anything it reads -- the Adafruit
// drivers cannot tell it their handle went stale, they just return corrupt
// data and report success.
uint32_t I2cBusGeneration(I2cBus bus);

// True if `address` acknowledges an empty transmission. Answers "is the device
// there", not "is its data good" -- a chip can keep acknowledging while its
// data transfers fail.
bool I2cDeviceResponds(I2cBus bus, uint8_t address);

// ── Per-bus watchdog ─────────────────────────────────────────────────────────
// Recovers a peripheral that has wedged, without punishing a bus that is
// merely missing a device.
//
// The peripheral can enter ESP_ERR_INVALID_STATE -- after the wiring is
// disturbed, for instance -- and never leaves on its own: every transfer
// afterwards fails while the sensor drivers keep reporting success. Restarting
// it is the only way out. But restarting because one device is unresponsive is
// what used to break the IMU whenever the GPS was unplugged, so the two cases
// must be told apart. The signal is simple: if one device is absent, the others
// on that bus still complete transfers; if the peripheral is wedged, nothing
// does.

// Records that some device on `bus` completed a transfer.
void I2cNoteTransferOk(I2cBus bus);

// Restarts a bus on which nothing has succeeded recently. Cheap enough to call
// every estimator tick.
void I2cServiceWatchdog(I2cBus bus);
