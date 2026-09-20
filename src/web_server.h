#pragma once

#include <stddef.h>

#include "estimator.h"
#include "filter_params.h"

// WiFi access point, dashboard HTTP server, and the WebSocket that carries
// telemetry out and control-knob changes back in.
//
// The board runs as its own access point rather than joining a network: it is
// meant to be taken outside for GPS, where there is no router to join. Connect
// to the AP_SSID network and open http://192.168.4.1/.

void WebServerBegin();

// Services HTTP and WebSocket traffic. Call from loop() as often as possible.
void WebServerLoop();

// Sends one telemetry frame to every connected browser. Rate-limits itself to
// TELEMETRY_INTERVAL_MS and does nothing when no one is connected.
void WebServerBroadcast(const EstimatorSnapshot& snapshot, const FilterParams& params,
                        const char* status_name);

size_t WebServerClientCount();
