#pragma once

// Copy this file to src/secrets.h (which is git-ignored) and fill it in.
//
// On boot the board tries to join WIFI_SSID. If that network is not in range
// -- which is what happens the moment you carry the board outside to get a GPS
// fix -- it gives up after WIFI_CONNECT_TIMEOUT_MS and brings up its own access
// point instead. Leaving WIFI_SSID empty skips the attempt and always starts
// the access point.

#define WIFI_SSID ""
#define WIFI_PASS ""
