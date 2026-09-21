#pragma once

#include <stdint.h>

#include "estimator.h"
#include "record_format.h"

// High-rate recording to a computer, over UDP.
//
// The estimator task pushes one record per 200 Hz tick into a ring; loop() on
// core 0 batches them into datagrams and sends them. The same split as the
// spectrum and for the same reason: the estimator's 5000 us budget is mostly
// I2C waits and must not gain a network call, whose latency it cannot bound.
// What core 1 pays is one memcpy under a spinlock, and only while recording.
//
// UDP rather than the dashboard's WebSocket because a lost datagram costs ten
// records, where TCP would stall the sender behind one retransmit -- and a
// stalled sender is a loop() that has stopped servicing the web server.
// The receiver is tools/vqf_record.py.

// Starts sending to `ip:port` (`ip` is the raw value an IPAddress converts
// to, so the header stays free of Arduino types). Restarting clears the counters and the ring.
void RecorderStart(uint32_t ip, uint16_t port);
void RecorderStop();

// Called from the estimator task with the snapshot it has just published.
// Returns on a flag check when not recording.
void RecorderPushTick(const EstimatorSnapshot& snapshot, uint32_t now_us);

// Sends any full batch. Called from loop(); does nothing when not recording.
void RecorderService();

RecorderStatus RecorderGetStatus();
