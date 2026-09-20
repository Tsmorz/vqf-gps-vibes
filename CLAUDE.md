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

**Both I2C buses exist; only bus 0 is wired.** The FeatherS3 breaks out `SDA 8 /
SCL 9` *and* `SDA1 16 / SCL1 15`. All three sensors are on bus 0. `task scan`
re-confirms this against the real hardware — run it before assuming a wiring
problem is a code problem.

**One task owns the bus.** The IMU and GPS share one STEMMA run, so all I2C
traffic is issued from the estimator task on core 1. Do not add an I2C read from
`loop()` — use the snapshot instead.

**Never drain the GPS by character count.** The PA1010D pads its I2C output with
`0x0A` when idle, and the Adafruit library discards that padding, so each
`read()` of an idle receiver triggers a fresh 32-byte I2C transfer. A 400-iteration
drain loop dropped the estimator from 200 Hz to 3 Hz. `GpsPoll()` is bounded by
`GPS_POLL_BUDGET_US` instead; keep it that way.

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
| The EKF itself (states, Jacobian, process noise) | `src/nav_filter.h` |
| How VQF and the EKF are driven, and the task loop | `src/estimator.cpp` |
| Which knobs the dashboard exposes | `src/filter_params.h` + `KNOBS` in `web/index.html` |
| The telemetry schema | `src/telemetry.cpp` **and** `ingest()` in `web/index.html` |
| Sensor reconnect behaviour | `src/imu.cpp`, `src/gps.cpp`, `src/i2c_bus.cpp` |
| Status LED colours and patterns | `src/status_led.cpp`, `EvaluateStatus()` in `src/main.cpp` |
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

## Style

- Google style, 4-space indent, 100-column limit (`.clang-format`).
- Short functions. Comments explain *why*, not *what* — especially the hardware
  quirks above, which are invisible from the code alone.
- Prefer status returns over C++ exceptions; this is a 200 Hz real-time task.
- Don't add abstraction layers that have exactly one implementation.
