#include "wifi_link.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include "config.h"

namespace {

WifiMode active_mode = WifiMode::kAccessPoint;
String address;

// Disables WiFi modem sleep.
//
// With power save on -- the default in station mode -- the radio sleeps
// between the access point's DTIM beacons and incoming packets sit buffered
// at the router until it wakes. On this network that showed up as 58-97 ms
// ping times on a LAN, an 8.7 kB page taking 1.9 s, and a WebSocket handshake
// needing 12 s to complete, because the library reads one header line per
// loop iteration and every one of them paid the wake-up latency.
//
// The cost is a noticeably higher idle current, which matters if the board is
// running from a battery. It is the right trade here: this is a real-time
// telemetry device whose whole purpose is a responsive live stream.
void DisableModemSleep() {
    WiFi.setSleep(false);
}

// True when secrets.h actually has a network configured. An empty SSID means
// "always be an access point".
bool HasConfiguredNetwork() {
    return strlen(WIFI_SSID) > 0;
}

// Tries to join the configured network, giving up after the timeout so a board
// carried out of range still comes up as an access point promptly.
bool JoinConfiguredNetwork() {
    Serial.printf("[net] joining \"%s\"", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    const uint32_t deadline = millis() + WIFI_CONNECT_TIMEOUT_MS;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[net] no answer -- falling back to the access point");
        WiFi.disconnect(true);
        return false;
    }
    address = WiFi.localIP().toString();
    DisableModemSleep();
    return true;
}

// Advertises the board as MDNS_HOSTNAME.local so the dashboard has a stable
// address even though the router hands out a different IP each time.
void StartMdns() {
    if (!MDNS.begin(MDNS_HOSTNAME)) {
        Serial.println("[net] mDNS failed to start (the IP still works)");
        return;
    }
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[net] also reachable at http://%s.local/\n", MDNS_HOSTNAME);
}

bool StartAccessPoint() {
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL)) {
        Serial.println("[net] FATAL: could not start the access point");
        return false;
    }
    address = WiFi.softAPIP().toString();
    DisableModemSleep();
    return true;
}

}  // namespace

WifiMode WifiLinkBegin() {
    if (HasConfiguredNetwork() && JoinConfiguredNetwork()) {
        active_mode = WifiMode::kStation;
        Serial.printf("[net] joined \"%s\" -- http://%s/\n", WIFI_SSID, address.c_str());
        StartMdns();
        return active_mode;
    }

    active_mode = WifiMode::kAccessPoint;
    if (StartAccessPoint()) {
        Serial.printf("[net] access point \"%s\" up -- http://%s/\n", AP_SSID, address.c_str());
    }
    return active_mode;
}

WifiMode WifiLinkMode() {
    return active_mode;
}

const char* WifiLinkAddress() {
    return address.c_str();
}
