/**
 * @file HubStatus.cpp
 * @brief Hub status broadcaster implementation
 * @author Philippe Hubert
 * @date 2026
 */

#include "HubStatus.h"

HubStatus::HubStatus() {
    memset(_buoyLastSeen, 0, sizeof(_buoyLastSeen));
    memset(_boatLastSeen, 0, sizeof(_boatLastSeen));
}

void HubStatus::begin(HubRelay* relay, uint32_t intervalMs) {
    _relay = relay;
    _intervalMs = intervalMs;
    _lastBroadcast = millis();
    Serial.printf("  ✓ Status broadcaster started (interval %lu ms)\n", intervalMs);
}

void HubStatus::update() {
    uint32_t now = millis();
    if (now - _lastBroadcast >= _intervalMs) {
        _lastBroadcast = now;
        broadcastStatus();
    }
}

void HubStatus::broadcastStatus() {
    if (_relay == nullptr) return;

    const RelayStats& stats = _relay->getStats();

    HubStatusPacket pkt = {};
    pkt.messageType         = MSG_TYPE_HUB_STATUS;
    pkt.hubId               = 0;
    pkt.uptimeMs            = millis();
    pkt.relayedCommands     = stats.relayedCommands;
    pkt.relayedStates       = stats.relayedStates;
    pkt.relayedGPS          = stats.relayedGPS;
    pkt.relayedAnemometer   = stats.relayedAnemometer;
    pkt.batteryVoltage      = 0.0f;     // TODO: read from PowerHub battery management
    pkt.rssiAvg             = 0;        // TODO: compute rolling average RSSI
    pkt.connectedBuoys      = getActiveBuoyCount();
    pkt.connectedBoats      = getActiveBoatCount();
    pkt.anemometerSeen      = isAnemometerActive();

    esp_now_send(_broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
}

// ─── Device tracking ───

void HubStatus::notifyBuoySeen(uint8_t buoyId) {
    if (buoyId < MAX_BUOYS) {
        _buoyLastSeen[buoyId] = millis();
    }
}

void HubStatus::notifyBoatSeen(const char* /*name*/) {
    // Simple tracking: mark any boat seen timestamp
    // For multiple boats, we'd track by name — simplified here
    if (_boatCount < MAX_BOATS) {
        _boatLastSeen[_boatCount] = millis();
        // Only increment if it's a genuinely new boat (approximation)
        if (_boatCount == 0 || (millis() - _boatLastSeen[0] > DEVICE_TIMEOUT_MS)) {
            _boatCount = 1;
        }
    }
    _boatLastSeen[0] = millis();  // At minimum, track "a boat was seen"
}

void HubStatus::notifyAnemometerSeen() {
    _anemoLastSeen = millis();
}

uint8_t HubStatus::getActiveBuoyCount() const {
    uint32_t now = millis();
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_BUOYS; i++) {
        if (_buoyLastSeen[i] != 0 && (now - _buoyLastSeen[i]) < DEVICE_TIMEOUT_MS) {
            count++;
        }
    }
    return count;
}

uint8_t HubStatus::getActiveBoatCount() const {
    uint32_t now = millis();
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_BOATS; i++) {
        if (_boatLastSeen[i] != 0 && (now - _boatLastSeen[i]) < DEVICE_TIMEOUT_MS) {
            count++;
        }
    }
    return count;
}

bool HubStatus::isAnemometerActive() const {
    return (_anemoLastSeen != 0) && (millis() - _anemoLastSeen < DEVICE_TIMEOUT_MS);
}
