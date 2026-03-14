/**
 * @file HubRelay.cpp
 * @brief ESP-NOW relay engine implementation
 * @author Philippe Hubert
 * @date 2026
 */

#include "HubRelay.h"
#include "HubStatus.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_timer.h>

// Singleton instance pointer (for static callback)
HubRelay* HubRelay::_instance = nullptr;

HubRelay::HubRelay() {
    _instance = this;
}

bool HubRelay::begin(uint32_t relayDelayUs, uint8_t queueSize) {
    _relayDelayUs = relayDelayUs;

    // ─── WiFi in STA mode (required for ESP-NOW) ───
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Set channel 1 (must match all other OpenSailingRC devices)
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    // Maximum TX power for best range
    esp_wifi_set_max_tx_power(84);  // 21 dBm max

    // Read our own MAC address
    esp_read_mac(_hubMac, ESP_MAC_WIFI_STA);
    Serial.printf("  Hub MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  _hubMac[0], _hubMac[1], _hubMac[2],
                  _hubMac[3], _hubMac[4], _hubMac[5]);

    // ─── Init ESP-NOW ───
    if (esp_now_init() != ESP_OK) {
        Serial.println("  ✗ esp_now_init() failed");
        return false;
    }

    // Register receive callback
    esp_now_register_recv_cb(onDataReceived);

    // Add broadcast peer for retransmission
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, _broadcastAddr, 6);
    peerInfo.channel = 0;   // Use current channel
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("  ✗ esp_now_add_peer(broadcast) failed");
        return false;
    }

    // ─── Create relay queue ───
    _relayQueue = xQueueCreate(queueSize, sizeof(RelayQueueItem));
    if (_relayQueue == nullptr) {
        Serial.println("  ✗ xQueueCreate failed");
        return false;
    }

    // ─── Start relay task on Core 0 (WiFi core) ───
    BaseType_t ret = xTaskCreatePinnedToCore(
        relayTaskEntry,
        "RelayTask",
        4096,           // Stack size (bytes)
        this,           // Parameter
        2,              // Priority (above idle, below WiFi)
        &_relayTaskHandle,
        0               // Core 0
    );
    if (ret != pdPASS) {
        Serial.println("  ✗ xTaskCreatePinnedToCore failed");
        return false;
    }

    // Print packet sizes for debugging/verification
    Serial.println("  Packet sizes (v2):");
    Serial.printf("    CommandPacket:      %u bytes\n", sizeof(CommandPacket));
    Serial.printf("    BuoyState:          %u bytes\n", sizeof(BuoyState));
    Serial.printf("    AckWithStatePacket: %u bytes\n", sizeof(AckWithStatePacket));
    Serial.printf("    GPSBroadcastPacket: %u bytes\n", sizeof(GPSBroadcastPacket));
    Serial.printf("    AnemometerPacket:   %u bytes\n", sizeof(AnemometerPacket));
    Serial.printf("    AnemometerPacketV1: %u bytes\n", sizeof(AnemometerPacketV1));
    Serial.printf("    HubStatusPacket:    %u bytes\n", sizeof(HubStatusPacket));

    Serial.println("  ✓ ESP-NOW relay engine started");
    return true;
}

// ═══════════════════════════════════════════════════════════════
// ESP-NOW Receive Callback (WiFi task context — keep fast!)
// ═══════════════════════════════════════════════════════════════

void HubRelay::onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (_instance == nullptr) return;
    if (len <= 0 || len > MAX_PACKET_SIZE) return;

    // Ignore our own packets (safety — shouldn't happen in broadcast)
    if (memcmp(mac, _instance->_hubMac, 6) == 0) {
        _instance->_stats.droppedOwn++;
        return;
    }

    _instance->_stats.totalReceived++;

    RelayQueueItem item;
    memcpy(item.data, data, len);
    item.length = static_cast<uint8_t>(len);
    memcpy(item.senderMac, mac, 6);
    item.receivedAtUs = esp_timer_get_time();

    // Non-blocking enqueue (drop if full — acceptable)
    if (xQueueSend(_instance->_relayQueue, &item, 0) != pdTRUE) {
        _instance->_stats.droppedQueueFull++;
    }
}

// ═══════════════════════════════════════════════════════════════
// Relay Task
// ═══════════════════════════════════════════════════════════════

void HubRelay::relayTaskEntry(void *param) {
    auto* self = static_cast<HubRelay*>(param);
    self->relayTaskLoop();
}

void HubRelay::relayTaskLoop() {
    RelayQueueItem item;

    for (;;) {
        // Block up to 100ms waiting for a packet
        if (xQueueReceive(_relayQueue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
            classifyAndRelay(item);
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Classification and Relay Logic
// ═══════════════════════════════════════════════════════════════

void HubRelay::classifyAndRelay(RelayQueueItem& item) {
    bool shouldRelay = false;
    uint8_t messageType = item.data[0];

    // ─── Packets with messageType as first byte ───
    if (messageType == MSG_TYPE_HUB_STATUS) {
        // Never relay other Hubs' status packets
        return;
    }

    if (messageType == MSG_TYPE_BOAT_GPS && item.length == sizeof(GPSBroadcastPacket)) {
        // GPS Boat data (messageType=1, v2 with TTL)
        auto *gps = reinterpret_cast<GPSBroadcastPacket*>(item.data);
        if (gps->ttl > 0) {
            gps->ttl--;
            shouldRelay = true;
            _stats.relayedGPS++;
            if (_statusTracker) _statusTracker->notifyBoatSeen(gps->name);
        } else {
            _stats.droppedTTL++;
        }
    }
    else if (messageType == MSG_TYPE_ANEMOMETER) {
        // Anemometer — handle both v1 (no TTL) and v2 (with TTL)
        shouldRelay = handleAnemometerRelay(item);
    }
    // ─── Packets classified by size (no messageType header) ───
    else if (item.length == sizeof(CommandPacket)) {
        auto *cmd = reinterpret_cast<CommandPacket*>(item.data);
        if (cmd->ttl > 0) {
            cmd->ttl--;
            shouldRelay = true;
            _stats.relayedCommands++;
        } else {
            _stats.droppedTTL++;
        }
    }
    else if (item.length == sizeof(AckWithStatePacket)) {
        auto *ack = reinterpret_cast<AckWithStatePacket*>(item.data);
        if (ack->ttl > 0) {
            ack->ttl--;  
            shouldRelay = true;
            _stats.relayedStates++;
            if (_statusTracker) _statusTracker->notifyBuoySeen(ack->buoyId);
        } else {
            _stats.droppedTTL++;
        }
    }
    else if (item.length == sizeof(BuoyState)) {
        auto *state = reinterpret_cast<BuoyState*>(item.data);
        if (state->ttl > 0) {
            state->ttl--;
            shouldRelay = true;
            _stats.relayedStates++;
            if (_statusTracker) _statusTracker->notifyBuoySeen(state->buoyId);
        } else {
            _stats.droppedTTL++;
        }
    }
    // Unknown packet — ignore (don't relay unknown formats)

    if (shouldRelay) {
        // Anti-collision delay: wait until at least _relayDelayUs after reception
        int64_t elapsed = esp_timer_get_time() - item.receivedAtUs;
        if (elapsed < (int64_t)_relayDelayUs) {
            delayMicroseconds(_relayDelayUs - elapsed);
        }

        esp_err_t result = esp_now_send(_broadcastAddr, item.data, item.length);
        if (result == ESP_OK) {
            _stats.totalRelayed++;
            _lastRelayMs = millis();
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Anemometer Relay (handles v1 and v2)
// ═══════════════════════════════════════════════════════════════

bool HubRelay::handleAnemometerRelay(RelayQueueItem& item) {
    // v2 anemometer (with TTL field)
    if (item.length == sizeof(AnemometerPacket)) {
        auto *anemo = reinterpret_cast<AnemometerPacket*>(item.data);
        if (anemo->ttl > 0) {
            anemo->ttl--;
            _stats.relayedAnemometer++;
            if (_statusTracker) _statusTracker->notifyAnemometerSeen();
            return true;
        }
        _stats.droppedTTL++;
        return false;
    }

    // v1 anemometer (no TTL field — passthrough with dedup)
    if (item.length == sizeof(AnemometerPacketV1)) {
        auto *anemo = reinterpret_cast<AnemometerPacketV1*>(item.data);

        // Deduplication: only relay if new sequenceNumber
        if (_anemoTracker.active &&
            anemo->sequenceNumber == _anemoTracker.lastSequenceNumber) {
            return false;   // Already relayed this one
        }

        _anemoTracker.lastSequenceNumber = anemo->sequenceNumber;
        _anemoTracker.lastRelayTimeMs = millis();
        _anemoTracker.active = true;

        _stats.relayedAnemometer++;
        if (_statusTracker) _statusTracker->notifyAnemometerSeen();
        return true;    // Relay as-is (no TTL to decrement)
    }

    return false;   // Unknown anemometer packet size
}
