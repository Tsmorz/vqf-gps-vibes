#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "estimator.h"

// The wire format of the UDP recording stream.
//
// Header-only and Arduino-free so the host tests can build the exact bytes the
// firmware sends and hand them to the Python parser (tools/vqflog.py), which
// keeps its own copy of this layout. That is two copies of one contract, so
// `task test-py` feeds the parser a packet made here rather than one it built
// itself -- a field added on one side alone fails there instead of producing a
// plausible log with every column shifted.
//
// Little-endian throughout, which is what both the ESP32-S3 and any machine
// that will parse it are.
//
// One record per estimator tick, not per telemetry frame: the dashboard's
// 20 Hz is a decimation for the browser's sake, and the whole point of a
// recording is to keep what the browser cannot show. Records are batched into
// packets, because 200 UDP datagrams a second would spend the WiFi stack's
// time on headers.

namespace recfmt {

constexpr char kMagic[4] = {'V', 'Q', 'F', 'L'};
constexpr uint8_t kVersion = 1;

// Which bits of RecordSample::flags are set.
constexpr uint8_t kFlagImuOk = 1u << 0;
constexpr uint8_t kFlagMagOk = 1u << 1;
constexpr uint8_t kFlagGpsFix = 1u << 2;
constexpr uint8_t kFlagBaroOk = 1u << 3;
constexpr uint8_t kFlagRest = 1u << 4;
constexpr uint8_t kFlagMagDisturbed = 1u << 5;

#pragma pack(push, 1)
struct PacketHeader {
    char magic[4];
    uint8_t version;
    uint8_t count;         // records in this packet
    uint16_t record_size;  // lets a parser reject a layout it does not know
    uint32_t seq;          // per packet, so the receiver can count what the network lost
    uint32_t dropped;      // records overwritten in the ring before they could be sent, in total
};

struct RecordSample {
    uint32_t t_us;   // estimator clock, micros(); wraps every 71 min, the parser unwraps it
    float accel[3];  // m/s^2, body frame, raw
    float gyro[3];   // rad/s, raw
    float mag[3];    // uT, calibrated
    float quat[4];   // (w, x, y, z), body -> ENU
    float pos[3];    // m, local ENU
    float vel[3];    // m/s, local ENU
    float baro_pa;
    float baro_height_m;  // pressure altitude with the filter's offset removed
    double gps_lat;
    double gps_lon;
    float gps_alt_m;
    float gps_hdop;
    // Increments once per fix the filter accepted. Records repeat the last fix
    // between updates, and this is how the parser tells a new one from a repeat.
    uint32_t gps_update_count;
    uint8_t gps_satellites;
    uint8_t flags;
    uint16_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 16, "header layout is part of the wire format");
static_assert(sizeof(RecordSample) == 120, "record layout is part of the wire format");

// Ten records is 1216 bytes -- inside a 1472-byte payload, so a packet is never
// fragmented on a standard 1500-byte MTU.
constexpr size_t kRecordsPerPacket = 10;
constexpr size_t kMaxPacketBytes = sizeof(PacketHeader) + kRecordsPerPacket * sizeof(RecordSample);
static_assert(kMaxPacketBytes <= 1472, "a packet must fit one unfragmented datagram");

inline RecordSample MakeRecord(const EstimatorSnapshot& s, uint32_t t_us) {
    RecordSample r;
    memset(&r, 0, sizeof(r));
    r.t_us = t_us;
    memcpy(r.accel, s.accel, sizeof(r.accel));
    memcpy(r.gyro, s.gyro, sizeof(r.gyro));
    memcpy(r.mag, s.mag, sizeof(r.mag));
    memcpy(r.quat, s.quat, sizeof(r.quat));
    memcpy(r.pos, s.pos, sizeof(r.pos));
    memcpy(r.vel, s.vel, sizeof(r.vel));
    r.baro_pa = s.baro_pressure_pa;
    r.baro_height_m = s.baro_height_m;
    r.gps_lat = s.gps_lat;
    r.gps_lon = s.gps_lon;
    r.gps_alt_m = s.gps_alt_m;
    r.gps_hdop = s.gps_hdop;
    r.gps_update_count = s.gps_update_count;
    r.gps_satellites = s.gps_satellites;
    r.flags = (s.imu_healthy ? kFlagImuOk : 0) | (s.mag_healthy ? kFlagMagOk : 0) |
              (s.gps_fix ? kFlagGpsFix : 0) | (s.baro_healthy ? kFlagBaroOk : 0) |
              (s.rest_detected ? kFlagRest : 0) | (s.mag_disturbed ? kFlagMagDisturbed : 0);
    return r;
}

// Writes header + `count` records into `out` (at least kMaxPacketBytes).
// Returns the packet's length in bytes, or 0 for a count that will not fit.
inline size_t EncodePacket(uint8_t* out, uint32_t seq, uint32_t dropped,
                           const RecordSample* records, size_t count) {
    if (count == 0 || count > kRecordsPerPacket) {
        return 0;
    }
    PacketHeader header;
    memcpy(header.magic, kMagic, sizeof(header.magic));
    header.version = kVersion;
    header.count = static_cast<uint8_t>(count);
    header.record_size = sizeof(RecordSample);
    header.seq = seq;
    header.dropped = dropped;
    memcpy(out, &header, sizeof(header));
    memcpy(out + sizeof(header), records, count * sizeof(RecordSample));
    return sizeof(header) + count * sizeof(RecordSample);
}

}  // namespace recfmt

// What the recorder is doing, echoed in the telemetry frame so a browser that
// connects mid-recording shows the button in the right state.
struct RecorderStatus {
    bool on = false;
    uint32_t packets = 0;  // sent since this recording started
    uint32_t records = 0;
    uint32_t dropped = 0;  // lost inside the board: ring overflow or a failed send
    uint16_t port = 0;
};
