#include "web_server.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "generated/web_index.h"
#include "imu.h"
#include "telemetry.h"
#include "vibration.h"
#include "wifi_link.h"

namespace {

WebServer http_server(80);
WebSocketsServer socket_server(81);
size_t connected_clients = 0;

// Locates the value of `key` in a small fixed-schema JSON object, returning a
// pointer to its first character or nullptr.
//
// The inbound messages have under a dozen known keys, so a full JSON parser
// would cost more flash than the protocol is worth. Whitespace around the
// colon is tolerated even though the dashboard's JSON.stringify never emits
// any: a protocol that only works for one particular encoder is a trap for
// anything else that ever talks to this board, including test tooling.
const char* FindValue(const char* json, const char* key) {
    char quoted[32];
    snprintf(quoted, sizeof(quoted), "\"%s\"", key);
    const char* found = strstr(json, quoted);
    if (found == nullptr) {
        return nullptr;
    }
    const char* cursor = found + strlen(quoted);
    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }
    if (*cursor != ':') {
        return nullptr;
    }
    cursor++;
    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }
    return cursor;
}

// True if `key` holds exactly the string `value`.
bool HasStringValue(const char* json, const char* key, const char* value) {
    const char* found = FindValue(json, key);
    if (found == nullptr || *found != '"') {
        return false;
    }
    found++;
    const size_t length = strlen(value);
    return strncmp(found, value, length) == 0 && found[length] == '"';
}

// Reads a numeric field. strtof skips any remaining whitespace itself.
bool ExtractNumber(const char* json, const char* key, float& out) {
    const char* value = FindValue(json, key);
    if (value == nullptr) {
        return false;
    }
    char* end = nullptr;
    const float parsed = strtof(value, &end);
    if (end == value) {
        return false;  // the key was present but the value was not a number
    }
    out = parsed;
    return true;
}

// Reads a flag, accepting both 0/1 and true/false, and leaving `out`
// untouched when the key is absent.
void ExtractFlag(const char* json, const char* key, bool& out) {
    const char* value = FindValue(json, key);
    if (value == nullptr) {
        return;
    }
    if (strncmp(value, "true", 4) == 0) {
        out = true;
    } else if (strncmp(value, "false", 5) == 0) {
        out = false;
    } else {
        float number = 0.0f;
        if (ExtractNumber(json, key, number)) {
            out = number != 0.0f;
        }
    }
}

// Applies a knob change from the dashboard. Every field is optional, so a
// slider can send just the one value it owns. Values are clamped to ranges
// that keep the filter well-conditioned -- a zero or negative sigma would
// make the covariance singular.
void HandleParamsCommand(const char* json) {
    FilterParams params = EstimatorGetParams();

    auto update_positive = [&](const char* key, float& field, float minimum, float maximum) {
        float value = 0.0f;
        if (!ExtractNumber(json, key, value)) {
            return;
        }
        field = constrain(value, minimum, maximum);
    };

    update_positive("sigma_accel", params.sigma_accel, 0.001f, 10.0f);
    update_positive("sigma_accel_bias", params.sigma_accel_bias, 0.00001f, 1.0f);
    update_positive("sigma_gps_pos_h", params.sigma_gps_pos_h, 0.1f, 100.0f);
    update_positive("sigma_gps_pos_v", params.sigma_gps_pos_v, 0.1f, 200.0f);
    update_positive("sigma_gps_vel", params.sigma_gps_vel, 0.01f, 20.0f);
    update_positive("sigma_zupt", params.sigma_zupt, 0.001f, 5.0f);
    update_positive("sigma_baro", params.sigma_baro, 0.02f, 20.0f);
    update_positive("sigma_baro_bias", params.sigma_baro_bias, 0.00001f, 1.0f);
    update_positive("tau_acc", params.tau_acc, 0.1f, 60.0f);
    update_positive("tau_mag", params.tau_mag, 0.1f, 120.0f);
    // Ranges are integers chosen from a fixed set, so they are only bounded
    // here -- ImuSetRanges() snaps them to a value the chip actually has, and
    // the estimator publishes the snapped result back. Doing the snapping
    // there rather than here keeps the list of supported ranges in one place,
    // next to the enum mapping that has to agree with it.
    auto update_range = [&](const char* key, int& field, int minimum, int maximum) {
        float value = 0.0f;
        if (!ExtractNumber(json, key, value)) {
            return;
        }
        field = static_cast<int>(
            constrain(value, static_cast<float>(minimum), static_cast<float>(maximum)));
    };

    update_range("accel_range_g", params.accel_range_g, 2, 16);
    update_range("gyro_range_dps", params.gyro_range_dps, 125, 2000);
    update_range("mag_range_gauss", params.mag_range_gauss, 4, 16);

    ExtractFlag(json, "zupt_enabled", params.zupt_enabled);
    ExtractFlag(json, "gps_enabled", params.gps_enabled);
    ExtractFlag(json, "gps_vel_enabled", params.gps_vel_enabled);
    ExtractFlag(json, "baro_enabled", params.baro_enabled);

    EstimatorSetParams(params);
}

// Starts, finishes or clears a magnetometer calibration sweep. See mag_cal.h
// for what the sweep is correcting and why VQF cannot do it on its own.
void HandleMagCalCommand(const char* json) {
    if (HasStringValue(json, "action", "start")) {
        ImuMagCalStart();
    } else if (HasStringValue(json, "action", "finish")) {
        ImuMagCalFinish();
    } else if (HasStringValue(json, "action", "clear")) {
        ImuMagCalClear();
    }
}

// Cuts or restores the GPS and barometer's rail. See EstimatorSetAuxPower().
void HandleAuxPowerCommand(const char* json) {
    if (HasStringValue(json, "state", "off")) {
        EstimatorSetAuxPower(false);
    } else if (HasStringValue(json, "state", "on")) {
        EstimatorSetAuxPower(true);
    }
}

// Switches the spectral analyser on or off and picks which sensor it looks at.
// Both fields are optional, so the source dropdown can send only the source.
void HandleSpectrumCommand(const char* json) {
    if (HasStringValue(json, "source", "gyro")) {
        VibrationSetSource(SpectrumSource::kGyro);
    } else if (HasStringValue(json, "source", "accel")) {
        VibrationSetSource(SpectrumSource::kAccel);
    }

    bool wanted = VibrationGetConfig().enabled;
    ExtractFlag(json, "enabled", wanted);
    VibrationSetEnabled(wanted);
}

void OnSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_CONNECTED) {
        connected_clients++;
        Serial.printf("[web] client %u connected (%u total)\n", client, connected_clients);
        return;
    }
    if (type == WStype_DISCONNECTED) {
        if (connected_clients > 0) {
            connected_clients--;
        }
        Serial.printf("[web] client %u disconnected (%u left)\n", client, connected_clients);
        return;
    }
    if (type != WStype_TEXT || length == 0) {
        return;
    }

    // The library does not null-terminate the payload, so copy it into a
    // bounded buffer before any string function touches it.
    char message[256];
    const size_t copied = length < sizeof(message) - 1 ? length : sizeof(message) - 1;
    memcpy(message, payload, copied);
    message[copied] = '\0';

    if (HasStringValue(message, "cmd", "params")) {
        HandleParamsCommand(message);
    } else if (HasStringValue(message, "cmd", "reset")) {
        EstimatorResetFilter();
    } else if (HasStringValue(message, "cmd", "magcal")) {
        HandleMagCalCommand(message);
    } else if (HasStringValue(message, "cmd", "auxpower")) {
        HandleAuxPowerCommand(message);
    } else if (HasStringValue(message, "cmd", "spectrum")) {
        HandleSpectrumCommand(message);
    }
}

// The dashboard is stored gzipped in flash and served as-is; every browser
// that can open a WebSocket can decompress it.
void HandleRoot() {
    http_server.sendHeader("Content-Encoding", "gzip");
    http_server.sendHeader("Cache-Control", "no-cache");
    http_server.send_P(200, "text/html", reinterpret_cast<const char*>(WEB_INDEX_GZ),
                       WEB_INDEX_GZ_LEN);
}

}  // namespace

void WebServerBegin() {
    const WifiMode mode = WifiLinkBegin();

    http_server.on("/", HandleRoot);
    if (mode == WifiMode::kAccessPoint) {
        // Phones probe these URLs to decide whether a network has internet. An
        // explicit 204 stops the "sign in to WiFi" pop-up from hijacking the
        // browser when you connect to the board outdoors. Only registered in
        // access-point mode -- on a real network these belong to the router.
        http_server.on("/generate_204", []() { http_server.send(204); });
        http_server.on("/hotspot-detect.html", HandleRoot);
    }
    http_server.onNotFound(HandleRoot);
    http_server.begin();

    socket_server.begin();
    socket_server.onEvent(OnSocketEvent);
}

void WebServerLoop() {
    http_server.handleClient();
    socket_server.loop();
}

void WebServerBroadcast(const EstimatorSnapshot& snapshot, const FilterParams& params,
                        const char* status_name) {
    static uint32_t last_send_ms = 0;
    const uint32_t now = millis();
    if (connected_clients == 0 || now - last_send_ms < TELEMETRY_INTERVAL_MS) {
        return;
    }
    last_send_ms = now;

    static char frame[kTelemetryBufferSize];
    const size_t length = BuildTelemetryFrame(frame, sizeof(frame), snapshot, params, status_name,
                                              VibrationGetConfig());
    if (length > 0) {
        socket_server.broadcastTXT(frame, length);
    }
}

void WebServerBroadcastSpectrum() {
    if (connected_clients == 0) {
        return;
    }
    // Both static: 6.5 kB between them, against the loop task's 8 kB stack.
    static SpectrumSnapshot spectrum;
    static char frame[kSpectrumBufferSize];

    if (!VibrationTakeSpectrum(spectrum)) {
        return;
    }
    const size_t length = BuildSpectrumFrame(frame, sizeof(frame), spectrum);
    if (length > 0) {
        socket_server.broadcastTXT(frame, length);
    }
}

size_t WebServerClientCount() {
    return connected_clients;
}
