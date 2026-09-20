#include "telemetry.h"

#include <stdio.h>

namespace {

// Formats a three-element array as a JSON list. Used for every vector in the
// frame so the precision stays consistent across them.
int WriteVector(char* out, size_t size, const char* key, const float value[3], int decimals) {
    return snprintf(out, size, "\"%s\":[%.*f,%.*f,%.*f]", key, decimals, value[0], decimals,
                    value[1], decimals, value[2]);
}

}  // namespace

size_t BuildTelemetryFrame(char* buffer, size_t buffer_size, const EstimatorSnapshot& s,
                           const FilterParams& p, const char* status_name) {
    size_t used = 0;

    // Appends through a cursor, bailing out the moment the buffer fills. Each
    // step checks before advancing so `used` can never run past the end.
    auto append = [&](int written) -> bool {
        if (written < 0 || static_cast<size_t>(written) >= buffer_size - used) {
            return false;
        }
        used += written;
        return true;
    };

    const bool complete =
        append(snprintf(buffer + used, buffer_size - used,
                        "{\"t\":%u,\"hz\":%.1f,\"status\":\"%s\",\"imu\":{", s.timestamp_ms,
                        s.estimator_hz, status_name)) &&
        append(WriteVector(buffer + used, buffer_size - used, "a", s.accel, 3)) &&
        append(snprintf(buffer + used, buffer_size - used, ",")) &&
        append(WriteVector(buffer + used, buffer_size - used, "g", s.gyro, 4)) &&
        append(snprintf(buffer + used, buffer_size - used, ",")) &&
        append(WriteVector(buffer + used, buffer_size - used, "m", s.mag, 2)) &&

        append(snprintf(buffer + used, buffer_size - used,
                        "},\"att\":{\"q\":[%.5f,%.5f,%.5f,%.5f],\"rpy\":[%.2f,%.2f,%.2f],"
                        "\"rest\":%d,\"magdist\":%d,",
                        s.quat[0], s.quat[1], s.quat[2], s.quat[3], s.roll_deg, s.pitch_deg,
                        s.yaw_deg, s.rest_detected ? 1 : 0, s.mag_disturbed ? 1 : 0)) &&
        append(WriteVector(buffer + used, buffer_size - used, "gbias", s.gyro_bias, 5)) &&

        append(snprintf(buffer + used, buffer_size - used,
                        "},\"magcal\":{\"done\":%d,\"busy\":%d,\"prog\":%.2f,\"field\":%.1f,"
                        "\"implied\":%.1f",
                        s.mag_calibrated ? 1 : 0, s.mag_collecting ? 1 : 0, s.mag_cal_progress,
                        s.mag_field_ut, s.mag_cal_implied_ut)) &&

        append(snprintf(buffer + used, buffer_size - used,
                        "},\"baro\":{\"ok\":%d,\"pa\":%.0f,\"tc\":%.1f,\"alt\":%.2f,"
                        "\"bias\":%.2f,\"bias3s\":%.2f,\"h\":%.2f,\"f\":%u",
                        s.baro_healthy ? 1 : 0, s.baro_pressure_pa, s.baro_temperature_c,
                        s.baro_altitude_m, s.baro_bias_m, s.baro_bias_sigma3, s.baro_height_m,
                        s.baro_failures)) &&

        append(snprintf(buffer + used, buffer_size - used, "},\"nav\":{")) &&
        append(WriteVector(buffer + used, buffer_size - used, "p", s.pos, 3)) &&
        append(snprintf(buffer + used, buffer_size - used, ",")) &&
        append(WriteVector(buffer + used, buffer_size - used, "v", s.vel, 3)) &&
        append(snprintf(buffer + used, buffer_size - used, ",")) &&
        append(WriteVector(buffer + used, buffer_size - used, "ba", s.accel_bias, 4)) &&
        append(snprintf(buffer + used, buffer_size - used, ",")) &&
        append(WriteVector(buffer + used, buffer_size - used, "p3s", s.pos_sigma3, 3)) &&
        append(snprintf(buffer + used, buffer_size - used, ",")) &&
        append(WriteVector(buffer + used, buffer_size - used, "v3s", s.vel_sigma3, 3)) &&

        append(snprintf(buffer + used, buffer_size - used,
                        "},\"gps\":{\"fix\":%d,\"sat\":%u,\"hdop\":%.1f,\"lat\":%.7f,"
                        "\"lon\":%.7f,\"alt\":%.1f,\"enuok\":%d,\"age\":%u,\"n\":%u,",
                        s.gps_fix ? 1 : 0, s.gps_satellites, s.gps_hdop, s.gps_lat, s.gps_lon,
                        s.gps_alt_m, s.gps_enu_valid ? 1 : 0, s.gps_fix_age_ms,
                        s.gps_update_count)) &&
        append(WriteVector(buffer + used, buffer_size - used, "enu", s.gps_enu, 3)) &&

        append(snprintf(buffer + used, buffer_size - used,
                        "},\"org\":{\"ok\":%d,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f},"
                        "\"health\":{\"imu\":%d,\"mag\":%d,\"gps\":%d,\"imuf\":%u,\"gpsf\":%u,"
                        "\"rst\":%u},",
                        s.origin_valid ? 1 : 0, s.origin_lat, s.origin_lon, s.origin_alt_m,
                        s.imu_healthy ? 1 : 0, s.mag_healthy ? 1 : 0, s.gps_healthy ? 1 : 0,
                        s.imu_failures, s.gps_failures, s.filter_resets)) &&

        // The knobs are echoed back so a newly connected browser can populate
        // its sliders from the device rather than from its own defaults.
        append(snprintf(buffer + used, buffer_size - used,
                        "\"params\":{\"sigma_accel\":%.4f,\"sigma_accel_bias\":%.5f,"
                        "\"sigma_gps_pos_h\":%.2f,\"sigma_gps_pos_v\":%.2f,"
                        "\"sigma_gps_vel\":%.3f,\"sigma_zupt\":%.4f,\"sigma_baro\":%.3f,"
                        "\"sigma_baro_bias\":%.5f,\"tau_acc\":%.2f,\"tau_mag\":%.2f,"
                        "\"zupt_enabled\":%d,\"gps_vel_enabled\":%d,\"baro_enabled\":%d}}",
                        p.sigma_accel, p.sigma_accel_bias, p.sigma_gps_pos_h, p.sigma_gps_pos_v,
                        p.sigma_gps_vel, p.sigma_zupt, p.sigma_baro, p.sigma_baro_bias, p.tau_acc,
                        p.tau_mag, p.zupt_enabled ? 1 : 0, p.gps_vel_enabled ? 1 : 0,
                        p.baro_enabled ? 1 : 0));

    return complete ? used : 0;
}
