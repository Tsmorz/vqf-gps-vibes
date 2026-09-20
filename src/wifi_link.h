#pragma once

// Brings up networking, preferring your own WiFi and falling back to the
// board's access point.
//
// Station mode is the convenient one at a desk: the dashboard is reachable
// from a machine that still has internet. Access-point mode is the one that
// works in a field with no router, which is where the GPS is actually useful.
// Which one you get is decided by whether the configured network answers, so
// carrying the board outside switches it over without any reconfiguration.

enum class WifiMode {
    kStation,      // joined the network from src/secrets.h
    kAccessPoint,  // serving our own network
};

// Connects and returns the mode that ended up active. Falls back to the access
// point on any failure, so this always leaves the board reachable somehow.
WifiMode WifiLinkBegin();

WifiMode WifiLinkMode();

// The address the dashboard is served on, as a printable string.
const char* WifiLinkAddress();
