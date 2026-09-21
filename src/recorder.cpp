#include "recorder.h"

#include <Arduino.h>
#include <WiFiUdp.h>

namespace {

// Two seconds of ticks. loop() runs about every millisecond, so the ring only
// ever holds one batch in practice; the slack is for the moment WiFi stalls.
// 512 x 120 B is 60 kB of the S3's 512 kB, and is only touched while recording.
constexpr size_t kRingSize = 512;

portMUX_TYPE ring_lock = portMUX_INITIALIZER_UNLOCKED;
recfmt::RecordSample ring[kRingSize];
uint32_t head = 0;  // next slot to write; counts records since the recording started
uint32_t tail = 0;  // next slot to send

volatile bool recording = false;
uint32_t overwritten = 0;    // guarded by ring_lock
uint32_t send_failures = 0;  // records in datagrams the stack refused; loop() only
uint32_t packets_sent = 0;
uint32_t records_sent = 0;
uint32_t sequence = 0;
uint32_t dest_ip = 0;
uint16_t dest_port = 0;

WiFiUDP udp;

}  // namespace

void RecorderStart(uint32_t ip, uint16_t port) {
    portENTER_CRITICAL(&ring_lock);
    head = tail = 0;
    overwritten = 0;
    portEXIT_CRITICAL(&ring_lock);
    send_failures = 0;
    packets_sent = 0;
    records_sent = 0;
    sequence = 0;
    dest_ip = ip;
    dest_port = port;
    recording = true;
    Serial.printf("[rec] streaming to %s:%u\n", IPAddress(ip).toString().c_str(), port);
}

void RecorderStop() {
    if (!recording) {
        return;
    }
    recording = false;
    Serial.printf("[rec] stopped after %u records, %u dropped\n", records_sent,
                  overwritten + send_failures);
}

void RecorderPushTick(const EstimatorSnapshot& snapshot, uint32_t now_us) {
    if (!recording) {
        return;
    }
    const recfmt::RecordSample record = recfmt::MakeRecord(snapshot, now_us);
    portENTER_CRITICAL(&ring_lock);
    ring[head % kRingSize] = record;
    head++;
    // A full ring loses its oldest record, not the newest: after a stall the
    // recording resumes at the present rather than replaying old data.
    if (head - tail > kRingSize) {
        tail = head - kRingSize;
        overwritten++;
    }
    portEXIT_CRITICAL(&ring_lock);
}

void RecorderService() {
    if (!recording) {
        return;
    }
    static recfmt::RecordSample batch[recfmt::kRecordsPerPacket];
    static uint8_t packet[recfmt::kMaxPacketBytes];

    // Full batches only, so the stream is uniform. The tail end of a recording
    // is at most nine records (45 ms), and stopping is not the moment to care.
    //
    // One packet per call keeps loop() responsive to the web server; the next
    // call, a millisecond later, takes the next one. That is five times the
    // tick rate's worth of packets, so the ring drains with room to spare.
    uint32_t dropped_now = 0;
    portENTER_CRITICAL(&ring_lock);
    const bool ready = head - tail >= recfmt::kRecordsPerPacket;
    if (ready) {
        for (size_t i = 0; i < recfmt::kRecordsPerPacket; i++) {
            batch[i] = ring[(tail + i) % kRingSize];
        }
        tail += recfmt::kRecordsPerPacket;
    }
    dropped_now = overwritten;
    portEXIT_CRITICAL(&ring_lock);
    if (!ready) {
        return;
    }

    const size_t length = recfmt::EncodePacket(packet, sequence++, dropped_now + send_failures,
                                               batch, recfmt::kRecordsPerPacket);
    // A failed send is not retried: the point of UDP here is that nothing
    // waits. The sequence number has already advanced, so the receiver sees
    // the hole as well.
    if (udp.beginPacket(IPAddress(dest_ip), dest_port) && udp.write(packet, length) == length &&
        udp.endPacket()) {
        packets_sent++;
        records_sent += recfmt::kRecordsPerPacket;
    } else {
        send_failures += recfmt::kRecordsPerPacket;
    }
}

RecorderStatus RecorderGetStatus() {
    RecorderStatus status;
    status.on = recording;
    status.packets = packets_sent;
    status.records = records_sent;
    portENTER_CRITICAL(&ring_lock);
    status.dropped = overwritten;
    portEXIT_CRITICAL(&ring_lock);
    status.dropped += send_failures;
    status.port = dest_port;
    return status;
}
