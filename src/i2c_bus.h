#pragma once

// Shared ownership of the single I2C bus all three sensors sit on.
//
// The LSM6DSOX, LIS3MDL and PA1010D are daisy-chained on one STEMMA run
// (confirmed with `task scan`), so bus faults are shared: a flexing cable
// takes every sensor down at once. Recovery therefore belongs here rather
// than in the individual drivers.

// Brings up the bus on the pins in config.h at 400 kHz.
void I2cBegin();

// Recovers a wedged bus and returns true if it looks usable afterwards.
//
// A sensor that is reset mid-transfer can be left driving SDA low, which locks
// the bus for everyone. The fix is to release the peripheral, clock SCL
// manually until the slave finishes the byte it thinks it is sending, issue a
// STOP, then re-attach. Called by the drivers after repeated read failures.
bool I2cRecover();

// True if `address` acknowledges an empty transmission. Used by the drivers to
// tell "sensor unplugged" apart from "sensor present but returning garbage".
bool I2cDeviceResponds(unsigned char address);
