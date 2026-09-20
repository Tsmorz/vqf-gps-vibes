#include "web_server.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "generated/web_index.h"
#include "telemetry.h"

namespace {

WebServer http_server(80);
WebSocketsServer socket_server(81);
size_t connected_clients = 0;

// Pulls a numeric field out of a small fixed-schema JSON object. The inbound
// messages have under a dozen known keys, so a full JSON parser would cost
// more flash than the protocol is worth.
bool ExtractNumber(const char* json, const char* key, float& out) {
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char* found = strstr(json, pattern);
    if (found == nullptr) {
        return false;
    }
    const char* value = found + strlen(pattern);
    char* end = nullptr;
    const float parsed = strtof(value, &end);
    if (end == value) {
        return false;  // the key was present but the value was not a number
    }
    out = parsed;
    return true;
}

// Reads a 0/1 flag, leaving `out` untouched when the key is absent.
void ExtractFlag(const char* json, const char* key, bool& out) {
    float value = 0.0f;
    if (ExtractNumber(json, key, value)) {
        out = value != 0.0f;
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
    update_positive("tau_acc", params.tau_acc, 0.1f, 60.0f);
    update_positive("tau_mag", params.tau_mag, 0.1f, 120.0f);
    ExtractFlag(json, "zupt_enabled", params.zupt_enabled);
    ExtractFlag(json, "gps_vel_enabled", params.gps_vel_enabled);

    EstimatorSetParams(params);
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

    if (strstr(message, "\"cmd\":\"params\"") != nullptr) {
        HandleParamsCommand(message);
    } else if (strstr(message, "\"cmd\":\"reset\"") != nullptr) {
        EstimatorResetFilter();
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
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL)) {
        Serial.println("[web] FATAL: could not start the access point");
        return;
    }
    Serial.printf("[web] access point \"%s\" up at http://%s/\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());

    http_server.on("/", HandleRoot);
    // Phones probe these URLs to decide whether a network has internet. An
    // explicit 204 stops the "sign in to WiFi" pop-up from hijacking the
    // browser when you connect to the board outdoors.
    http_server.on("/generate_204", []() { http_server.send(204); });
    http_server.on("/hotspot-detect.html", HandleRoot);
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
    const size_t length = BuildTelemetryFrame(frame, sizeof(frame), snapshot, params, status_name);
    if (length > 0) {
        socket_server.broadcastTXT(frame, length);
    }
}

size_t WebServerClientCount() {
    return connected_clients;
}
