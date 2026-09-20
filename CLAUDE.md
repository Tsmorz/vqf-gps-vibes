# CLAUDE.md

Guidance for Claude Code (claude.ai/code) when working in this repository.

## Project

Real-time VQF orientation estimation fused with GPS on an **Unexpected Maker
FeatherS3** (`board = um_feathers3`, PlatformIO env `feathers3`). Sensors:
LSM6DSOX + LIS3MDL (Adafruit breakout) and an Adafruit Mini GPS PA1010D, all on
I2C. The board runs its own WiFi access point and serves a dashboard. C++,
Arduino framework via PlatformIO, Google style.

See `README.md` for the hardware table, the dashboard panels and the filter
design. This file covers only what is easy to get wrong when editing.

## Things that will bite you

**Two buses, split on purpose.** IMU on bus 0 (`SDA 8 / SCL 9`), GPS + BMP390 on
bus 1 (`SDA1 16 / SCL1 15`). Not for convenience: on a shared run the GPS's
intermittent connector took the whole IMU down with it. `task scan` re-confirms
the layout against real hardware — run it before assuming a wiring problem is a
code problem.

**One task owns both buses.** All I2C traffic is issued from the estimator task
on core 1. Do not add an I2C read from `loop()` — use the snapshot instead.

**GPIO 39 is LDO2 *and* the LED's power gate.** Driving it low to darken the LED
cuts power to the GPS and barometer. Send a black pixel instead. `board_power.h`
owns this pin; `status_led.cpp` deliberately does not touch it.

**Never restart a bus because one device is silent.** An unresponsive device is
absent; the bus is fine. Tearing it down invalidates every Adafruit_BusIO handle
on it, and those drivers then return corrupt data while reporting success — this
is what broke the IMU every time the GPS was unplugged. `I2cServiceWatchdog()`
distinguishes the cases: if one device is absent the others still complete
transfers, so only a bus where *nothing* has succeeded for 3 s gets restarted.
Any restart bumps `I2cBusGeneration()`, and every driver must watch it and
re-`begin_I2C()`.

**I2C runs at 100 kHz, from measurement.** At 400 kHz the data came back inside
the sensors' full scale but wrong. See `I2C_CLOCK_HZ` in `include/config.h` for
the numbers. Raise it only with the same measurement in hand.

**Never drain the GPS by character count.** The PA1010D pads its I2C output with
`0x0A` when idle, and the Adafruit library discards that padding, so each
`read()` of an idle receiver triggers a fresh 32-byte I2C transfer. A 400-iteration
drain loop dropped the estimator from 200 Hz to 3 Hz. `GpsPoll()` is bounded by
`GPS_POLL_BUDGET_US` instead; keep it that way.

**The Adafruit IMU driver lies about failures.** `Adafruit_LSM6DS::getEvent()`
always returns `true` and its `_read()` is `void`, so an unplugged sensor reads
as `(0, 0, 0)` and the filter integrates it as free fall — this reached −103 km
on the bench before it was caught. `imu.cpp` probes the chip's I2C address every
250 ms instead; do not replace that with a check of the driver's return value.
The GPS has the same probe for the same reason.

**Never let `Serial` block the loop.** `Serial.setTxTimeoutMs(0)` in `setup()`
is load-bearing, not tidying. The USB-serial `write()` waits up to ~2 s when no
monitor is draining the port, and `loop()` — which services the web server —
then runs at about 1 Hz. Symptom: everything is fine over USB and the dashboard
is unusable untethered (a 15 ms WebSocket handshake becomes 15 s). If you add
logging to `loop()`, this is why it must stay non-blocking.

**VQF is vendored, not implemented here.** `lib/vqf/` is verbatim from
<https://github.com/dlaidig/vqf> (MIT). Don't edit it; treat it as a black box
with the API in `vqf.hpp`.

**VQF's earth frame is already ENU.** Its 9D quaternion puts z opposite gravity
and the horizontal magnetic field along +y, so the rotation matrix feeds the EKF
with no axis permutation. This was verified by reading the heading correction in
`vqf.cpp` (`atan2(magEarth[0], magEarth[1])`), not assumed — if you change frames,
verify it the same way.

**Core pinning is set in `platformio.ini`, not in code.** The board definition
pins the Arduino loop to core 1; `build_unflags`/`build_flags` move it to core 0
so core 1 belongs to the estimator. Removing those flags silently reintroduces
tick jitter.

## Quick file map

| To change... | Look at |
|---|---|
| Pins, rates, filter defaults, AP credentials | `include/config.h` |
| WiFi credentials (git-ignored) | `src/secrets.h`, template in `src/secrets.example.h` |
| Station vs access-point selection | `src/wifi_link.cpp` |
| The EKF itself (states, Jacobian, process noise) | `src/nav_filter.h` |
| How VQF and the EKF are driven, and the task loop | `src/estimator.cpp` |
| Which knobs the dashboard exposes | `src/filter_params.h` + `KNOBS` in `web/index.html` |
| The telemetry schema | `src/telemetry.cpp` **and** `ingest()` in `web/index.html` |
| Sensor reconnect behaviour | `src/imu.cpp`, `src/gps.cpp`, `src/i2c_bus.cpp` |
| Status LED colours and patterns | `src/status_led.cpp`, `EvaluateStatus()` in `src/main.cpp` |
| Behaviour when a sensor drops out | `PredictCoasting()`/`IsDiverged()` in `src/nav_filter.h` |
| Magnetometer hard/soft-iron calibration | `src/mag_cal.h`, driven from `src/imu.cpp` |
| Barometer fusion, and why it has its own state | `kBaroBias` in `src/nav_filter.h`, `ServiceBaro()` in `src/estimator.cpp` |
| LDO2 / power-cycle recovery | `src/board_power.{h,cpp}` |
| Dashboard panels and plotting | `web/index.html` (gzipped into flash by `scripts/embed_web.py`) |

## Commands

`task build`, `task flash`, `task monitor`, `task scan`, `task test`,
`task test-ui`, `task lint`, `task format`, `task ci`, `task erase`.
Run `task ci` before committing.

## Testing

Everything that can be tested without hardware, is:

- `test/test_nav/` — the EKF and the geodetic projection. `nav_filter.h`, `geo.h`
  and `attitude.h` are header-only and Arduino-free specifically so they can be.
- `test/test_telemetry/` — the frame encoder, built from the same `telemetry.cpp`
  the firmware ships.
- `task test-ui` — runs the dashboard's real JavaScript against a real frame from
  that encoder, under a stub DOM. This is what catches the firmware and the UI
  drifting apart on a field name.

**If you change the telemetry schema, change both sides and run `task test-ui`.**

**Verify that an edit actually landed.** These files get edited from more than one
place at once, and a search-and-replace whose anchor has drifted fails silently —
the build still succeeds and the field is simply missing at runtime. That is how
`baro_enabled` ended up absent from the telemetry params block while the test
that should have caught it had also lost its assertion. Grep for what you just
added.

## Style

- Google style, 4-space indent, 100-column limit (`.clang-format`).
- Short functions. Comments explain *why*, not *what* — especially the hardware
  quirks above, which are invisible from the code alone.
- Prefer status returns over C++ exceptions; this is a 200 Hz real-time task.
- Don't add abstraction layers that have exactly one implementation.
