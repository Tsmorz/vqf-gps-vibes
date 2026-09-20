#pragma once

// Control of the FeatherS3's second LDO.
//
// The board has two regulators. LDO1 is always on and feeds the ESP32-S3 and
// the IMU. LDO2 is switchable and feeds the 3V3 pin -- on this build, the
// PA1010D GPS and the BMP390 -- and it is gated by GPIO 39.
//
// GPIO 39 is also RGB_PWR, the onboard LED's power gate: the variant header
// defines LDO2 and RGB_PWR as the same pin. So the LED and the aux sensors
// share a rail, with two consequences worth stating plainly:
//
//   * Never drive GPIO 39 low to "turn the LED off". It would cut power to the
//     GPS and barometer. To darken the LED, send it a black pixel instead.
//   * The rail must be up, and given time to settle, before those sensors are
//     initialised. The PA1010D in particular takes a while to answer after
//     power-up, which is why enabling it is a deliberate step in setup()
//     rather than a side effect of bringing the LED up.
//
// The switchable rail also buys a recovery option the IMU does not have: an
// aux sensor that has wedged badly enough to ignore its own bus can simply be
// power-cycled.

// Enables LDO2 and waits for the rail and its sensors to come up.
void BoardPowerBegin();

// Drops LDO2 briefly and brings it back, then waits for the sensors to boot.
// Blocks for a few hundred milliseconds. The onboard LED goes dark during it.
void BoardPowerCycleAux();
