#pragma once

// Onboard RGB status light.
//
// The board is normally used untethered, on its own WiFi access point, so the
// LED is the only channel that works before a browser is connected. Four
// states, with conventional colours and distinct patterns so the state is
// readable in bright light where hue alone is hard to judge:
//
//   initialising  blue    fast pulse     sensors and WiFi coming up
//   normal        green   slow breathe   estimator running, sensors healthy
//   warning       amber   slow blink     degraded -- e.g. no GPS fix, no mag
//   error         red     fast blink     a required sensor is gone
//   calibrating   magenta fast pulse     magnetometer sweep in progress (BOOT button)
//
// (The FeatherS3's onboard LED is a WS2812B RGB part -- it has no separate
// white element, so white is produced by driving all three channels.)

enum class SystemStatus {
    kInitialising,
    kNormal,
    kWarning,
    kError,
    kCalibrating,
};

// Enables the LED's power rail and takes the light to the initialising state.
void StatusLedBegin();

void StatusLedSet(SystemStatus status);

// Turns the LED dark without touching its power rail (see board_power.h).
void StatusLedOff();

SystemStatus StatusLedCurrent();

// Advances the blink/breathe animation. Call often from loop(); it rate-limits
// itself so it will not flood the single-wire LED protocol.
void StatusLedUpdate();
