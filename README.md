# vqf-gps-vibes

Real-time orientation and position estimation on an **Unexpected Maker FeatherS3**
(ESP32-S3), streamed to a browser over the board's own WiFi access point.

Orientation comes from [VQF](https://github.com/dlaidig/vqf) (Laidig & Seel,
*Information Fusion*, 2023). Position and velocity come from a 9-state extended
Kalman filter driven by the accelerometer and corrected by GPS with ordinary
linear measurement updates — a loosely-coupled INS/GNSS filter sitting on top of
VQF's attitude solution.

## Hardware

| Part | Interface | Address |
|---|---|---|
| Unexpected Maker FeatherS3 (ESP32-S3, 16 MB flash) | — | — |
| Adafruit LSM6DSOX + LIS3MDL breakout | I2C bus 0 | `0x6A`, `0x1C` |
| Adafruit Mini GPS PA1010D | I2C bus 0 | `0x10` |

The FeatherS3 exposes **two** I2C buses — `SDA 8 / SCL 9` and `SDA1 16 / SCL1 15`.
All three sensors are daisy-chained on **bus 0** (`SDA 8 / SCL 9`); bus 1 is unused.
Run `task scan` to confirm this on your own wiring before anything else — it scans
both buses and prints what answers on each.

Because the sensors share one bus, every I2C transfer is issued from a single
FreeRTOS task, so the GPS and the IMU can never interleave transactions.

## Quick start

```bash
brew install go-task clang-format node   # node is only needed for `task test-ui`
pip install platformio
task init            # fetch toolchains, libraries, and create src/secrets.h
task flash           # build, flash, open the serial monitor
```

## Networking

The board picks its own mode on boot, so the same firmware works at a desk and
in a field:

- **Station** — if `WIFI_SSID` in `src/secrets.h` (git-ignored, created by
  `task init`) names a network that answers within 8 s, the board joins it and
  advertises itself over mDNS at **<http://vqf-gps.local/>**. Convenient at a
  desk: the dashboard is reachable from a machine that still has internet.
- **Access point** — otherwise, which is what happens the moment you carry the
  board out of range, it serves its own network **`vqf-gps`** (password
  `vqfgps123`) at **<http://192.168.4.1/>**.

Leave `WIFI_SSID` empty to always be an access point. The serial log prints
which mode won and the address to open.

## Commands

| Task | What it does |
|---|---|
| `task build` | Compile the firmware |
| `task flash` | Build, flash, open the serial monitor |
| `task upload` | Build and flash without the monitor |
| `task monitor` | Serial monitor only |
| `task scan` | Flash the I2C/LED bring-up probe — use when a cable is suspect |
| `task test` | Host unit tests (EKF, geodesy, telemetry encoder) |
| `task test-ui` | Run the dashboard's JavaScript headlessly against a real frame |
| `task lint` | cppcheck static analysis |
| `task format` / `format-check` | clang-format (Google style, 4-space indent) |
| `task ci` | Everything above, in CI order |
| `task erase` | Erase all flash |

## Dashboard

Six panels, all drawn with plain canvas — no CDN, since the board serves the page
from its own access point with no internet behind it.

- **Orientation** — the body's unit axes rotated by the VQF quaternion, against a
  faint ENU world frame. Drag to orbit.
- **IMU** — accelerometer, gyroscope and magnetometer time series.
- **Position (3D)** — the estimated trajectory in local ENU, raw GPS fixes, and
  the 3σ uncertainty ellipsoid. Drag to orbit.
- **Position vs time** — east/north/up, each with its ±3σ envelope shaded and raw
  fixes overlaid.
- **Velocity vs time** — same, for the velocity states.
- **Filter tuning** — the control knobs, below.

### Control knobs

Sliders are logarithmic, because these sigmas span orders of magnitude.

| Knob | Effect |
|---|---|
| `sigma_accel` | **The dominant process-noise term.** Sets how fast the position/velocity 3σ envelope opens between fixes. |
| `sigma_accel_bias` | Accelerometer bias random walk — how quickly bias is re-learned from GPS. |
| `sigma_gps_pos_h` / `_v` | Assumed 1σ GPS accuracy; lower values make fixes pull harder. |
| `sigma_gps_vel` | Assumed 1σ accuracy of GPS ground speed and course. |
| `sigma_zupt` | How firmly a detected rest pins velocity to zero. |
| `tau_acc` / `tau_mag` | VQF time constants for gravity (roll/pitch) and magnetometer (yaw) correction. |

Changes take effect on the next filter tick. Nothing is persisted — a reboot
returns to the defaults in `include/config.h`.

## Status LED

The onboard RGB LED (WS2812B on GPIO 40, power rail gated by GPIO 39) is the only
status channel before a browser is attached.

| Colour | Pattern | Meaning |
|---|---|---|
| Blue | fast pulse | Initialising — sensors and WiFi coming up |
| Green | slow breathe | Normal — estimator running, all sensors healthy, GPS has a fix |
| Amber | slow blink | Degraded — no GPS fix or module, magnetometer lost, or the tick rate has sagged |
| Red | fast blink | Error — the accelerometer/gyroscope is gone, so there is no estimate at all |

## Architecture

```
core 1   estimator task, 200 Hz — owns the I2C bus         src/estimator.cpp
core 0   Arduino loop — WiFi AP, dashboard, status LED     src/main.cpp
```

```
include/config.h      pins, rates, defaults — every value overridable with -D
lib/vqf/              vendored verbatim from github.com/dlaidig/vqf (MIT)
src/
  main.cpp            setup/loop, status evaluation, serial heartbeat
  estimator.cpp/.h    the 200 Hz task: IMU -> VQF -> EKF -> snapshot
  nav_filter.h        9-state EKF (pure math, unit tested)
  geo.h               lat/lon/alt -> local ENU (pure math, unit tested)
  attitude.h          quaternion -> rotation matrix and Euler angles
  filter_params.h     the runtime-tunable knobs
  imu.cpp/.h          LSM6DSOX + LIS3MDL, with independent reconnect
  gps.cpp/.h          PA1010D, with reconnect and presence detection
  i2c_bus.cpp/.h      shared bus init and stuck-bus recovery
  telemetry.cpp/.h    snapshot -> the JSON frame the dashboard reads
  wifi_link.cpp/.h    joins your network, falls back to the board's own AP
  web_server.cpp/.h   HTTP and WebSocket
  secrets.example.h   template for the git-ignored src/secrets.h
web/index.html        the dashboard, gzipped into flash at build time
tools/i2cscan/        the bring-up probe behind `task scan`
tools/check_dashboard.mjs   headless harness behind `task test-ui`
```

### The filter

State: `[pE pN pU | vE vN vU | bx by bz]` — position and velocity in the local ENU
tangent plane anchored at the first GPS fix, plus accelerometer bias in the body
frame.

Propagation rotates the bias-corrected accelerometer into ENU using VQF's
quaternion and adds gravity. Every measurement — GPS position, GPS velocity, the
zero-velocity pseudo-measurement — observes a single state directly, so updates
are applied one scalar at a time: the Kalman gain reduces to a single column of
`P` and no matrix inversion is needed.

VQF's 9D earth frame is already ENU (z opposite gravity, the horizontal magnetic
field along +y), so its rotation matrix is used with no axis permutation. The one
caveat is declination — +y is *magnetic* north, which rotates the whole trajectory
by a fixed angle rather than causing drift.

### Zero-velocity updates

On by default. While VQF's rest detector reports the board is stationary, all
three velocity components are measured as zero. Without this, indoor testing with
no GPS fix diverges within seconds as accelerometer bias double-integrates into
runaway position error. Turn it off from the dashboard to watch exactly that
happen — the 3σ envelope opening up is the point of the plot.

## Failure handling

The STEMMA cables are fragile and the GPS often has no fix, so neither is treated
as exceptional:

- Each chip tracks consecutive read failures and only goes offline after a run of
  them, so a flexing cable does not cause a re-init storm.
- Offline chips are re-initialised at most once every 2 s, and the bus is
  clock-pulsed free first in case a sensor was reset mid-transfer and is holding
  SDA low.
- The magnetometer and the accelerometer/gyroscope are tracked separately: losing
  the magnetometer costs absolute heading, and VQF keeps running on 6-DOF data.
- The GPS is probed for presence on the bus every 2 s, which distinguishes
  "unplugged" from "plugged in but still searching".
- Each GPS fix is applied exactly once. Re-applying the same fix every tick would
  shrink the covariance without new information and make the filter overconfident.
- The GPS drain is bounded by a wall-clock budget rather than a character count.
  The PA1010D pads its I2C output with `0x0A` when idle and the Adafruit library
  discards that padding, so draining by character count costs one 32-byte I2C
  transfer *per character* and collapses the tick rate.
- Divergence (a non-finite state or a negative variance) resets the filter rather
  than publishing NaNs to every plot.

Errors are reported as status returns, not C++ exceptions — exceptions in a 200 Hz
real-time task on an MCU would cost far more than they are worth here.

## Licence

`lib/vqf/` is vendored verbatim from <https://github.com/dlaidig/vqf> and is MIT
licensed — see `lib/vqf/LICENSE.txt`. Don't edit it; treat it as a black box with
the API documented in `vqf.hpp`.
