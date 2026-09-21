// Host-side tests for the UDP recording wire format.
//
// The packet is the contract between the firmware and tools/vqflog.py, which
// keeps its own copy of the layout. Beyond checking sizes and offsets, this
// prints a packet built from a distinctive snapshot so `task test-py` can hand
// those exact bytes to the Python parser.

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "record_format.h"

namespace {

EstimatorSnapshot MakeSnapshot() {
    EstimatorSnapshot s;
    s.accel[0] = 0.25f;
    s.accel[1] = -0.5f;
    s.accel[2] = 9.75f;
    s.gyro[0] = 0.125f;
    s.gyro[1] = -0.0625f;
    s.gyro[2] = 0.03125f;
    s.mag[0] = 12.5f;
    s.mag[1] = -30.25f;
    s.mag[2] = 41.75f;
    s.quat[0] = 0.5f;
    s.quat[1] = 0.5f;
    s.quat[2] = 0.5f;
    s.quat[3] = 0.5f;
    s.pos[0] = 1.5f;
    s.pos[1] = -2.5f;
    s.pos[2] = 3.5f;
    s.vel[0] = 0.75f;
    s.vel[1] = -0.25f;
    s.vel[2] = 0.125f;
    s.baro_pressure_pa = 101325.0f;
    s.baro_height_m = 12.5f;
    s.gps_lat = 51.5007292;
    s.gps_lon = -0.1246254;
    s.gps_alt_m = 35.5f;
    s.gps_hdop = 0.9f;
    s.gps_update_count = 77;
    s.gps_satellites = 9;
    s.imu_healthy = true;
    s.mag_healthy = true;
    s.gps_fix = true;
    s.baro_healthy = false;
    s.rest_detected = true;
    s.mag_disturbed = false;
    return s;
}

// Not a tidy size by accident: 120 bytes is what the Python dtype is built on.
void test_layout_matches_the_documented_wire_format() {
    TEST_ASSERT_EQUAL_size_t(16, sizeof(recfmt::PacketHeader));
    TEST_ASSERT_EQUAL_size_t(120, sizeof(recfmt::RecordSample));
    TEST_ASSERT_EQUAL_size_t(0, offsetof(recfmt::RecordSample, t_us));
    TEST_ASSERT_EQUAL_size_t(4, offsetof(recfmt::RecordSample, accel));
    TEST_ASSERT_EQUAL_size_t(80, offsetof(recfmt::RecordSample, baro_pa));
    TEST_ASSERT_EQUAL_size_t(88, offsetof(recfmt::RecordSample, gps_lat));
    TEST_ASSERT_EQUAL_size_t(116, offsetof(recfmt::RecordSample, gps_satellites));
}

void test_a_record_carries_the_snapshot() {
    const EstimatorSnapshot s = MakeSnapshot();
    const recfmt::RecordSample r = recfmt::MakeRecord(s, 123456u);
    TEST_ASSERT_EQUAL_UINT32(123456u, r.t_us);
    TEST_ASSERT_EQUAL_FLOAT(9.75f, r.accel[2]);
    TEST_ASSERT_EQUAL_FLOAT(-0.0625f, r.gyro[1]);
    TEST_ASSERT_EQUAL_FLOAT(41.75f, r.mag[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, r.quat[3]);
    TEST_ASSERT_EQUAL_FLOAT(-2.5f, r.pos[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.125f, r.vel[2]);
    TEST_ASSERT_EQUAL_FLOAT(101325.0f, r.baro_pa);
    // Unity is built without double support; the coordinates must survive as doubles.
    TEST_ASSERT_TRUE(r.gps_lat == 51.5007292);
    TEST_ASSERT_TRUE(r.gps_lon == -0.1246254);
    TEST_ASSERT_EQUAL_UINT32(77, r.gps_update_count);
    TEST_ASSERT_EQUAL_UINT8(9, r.gps_satellites);
}

void test_every_flag_lands_on_its_own_bit() {
    EstimatorSnapshot s;
    TEST_ASSERT_EQUAL_UINT8(0, recfmt::MakeRecord(s, 0).flags);

    s.imu_healthy = true;
    TEST_ASSERT_EQUAL_UINT8(recfmt::kFlagImuOk, recfmt::MakeRecord(s, 0).flags);
    s = EstimatorSnapshot();
    s.mag_healthy = true;
    TEST_ASSERT_EQUAL_UINT8(recfmt::kFlagMagOk, recfmt::MakeRecord(s, 0).flags);
    s = EstimatorSnapshot();
    s.gps_fix = true;
    TEST_ASSERT_EQUAL_UINT8(recfmt::kFlagGpsFix, recfmt::MakeRecord(s, 0).flags);
    s = EstimatorSnapshot();
    s.baro_healthy = true;
    TEST_ASSERT_EQUAL_UINT8(recfmt::kFlagBaroOk, recfmt::MakeRecord(s, 0).flags);
    s = EstimatorSnapshot();
    s.rest_detected = true;
    TEST_ASSERT_EQUAL_UINT8(recfmt::kFlagRest, recfmt::MakeRecord(s, 0).flags);
    s = EstimatorSnapshot();
    s.mag_disturbed = true;
    TEST_ASSERT_EQUAL_UINT8(recfmt::kFlagMagDisturbed, recfmt::MakeRecord(s, 0).flags);
}

void test_a_full_packet_fits_one_datagram() {
    recfmt::RecordSample records[recfmt::kRecordsPerPacket];
    for (size_t i = 0; i < recfmt::kRecordsPerPacket; i++) {
        records[i] = recfmt::MakeRecord(MakeSnapshot(), 1000u * static_cast<uint32_t>(i));
    }
    uint8_t packet[recfmt::kMaxPacketBytes];
    const size_t length = recfmt::EncodePacket(packet, 9, 4, records, recfmt::kRecordsPerPacket);
    TEST_ASSERT_EQUAL_size_t(recfmt::kMaxPacketBytes, length);
    TEST_ASSERT_TRUE(length <= 1472);
    TEST_ASSERT_EQUAL_MEMORY("VQFL", packet, 4);
    TEST_ASSERT_EQUAL_UINT8(recfmt::kVersion, packet[4]);
    TEST_ASSERT_EQUAL_UINT8(recfmt::kRecordsPerPacket, packet[5]);
}

// A count the buffer was not sized for must be refused, not written past it.
void test_an_unrepresentable_count_is_refused() {
    recfmt::RecordSample records[recfmt::kRecordsPerPacket + 1] = {};
    uint8_t packet[recfmt::kMaxPacketBytes];
    TEST_ASSERT_EQUAL_size_t(0, recfmt::EncodePacket(packet, 0, 0, records, 0));
    TEST_ASSERT_EQUAL_size_t(
        0, recfmt::EncodePacket(packet, 0, 0, records, recfmt::kRecordsPerPacket + 1));
}

// Printed as hex for the Python parser test: two records, one second apart, so
// the parser's timestamp handling and field offsets are both exercised.
void test_print_reference_packet() {
    recfmt::RecordSample records[2];
    records[0] = recfmt::MakeRecord(MakeSnapshot(), 1000000u);
    records[1] = recfmt::MakeRecord(MakeSnapshot(), 1005000u);
    uint8_t packet[recfmt::kMaxPacketBytes];
    const size_t length = recfmt::EncodePacket(packet, 12, 5, records, 2);
    TEST_ASSERT_TRUE(length > 0);

    printf("\nRECORD_PACKET:");
    for (size_t i = 0; i < length; i++) {
        printf("%02x", packet[i]);
    }
    printf("\n");
}

}  // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_layout_matches_the_documented_wire_format);
    RUN_TEST(test_a_record_carries_the_snapshot);
    RUN_TEST(test_every_flag_lands_on_its_own_bit);
    RUN_TEST(test_a_full_packet_fits_one_datagram);
    RUN_TEST(test_an_unrepresentable_count_is_refused);
    RUN_TEST(test_print_reference_packet);
    return UNITY_END();
}
