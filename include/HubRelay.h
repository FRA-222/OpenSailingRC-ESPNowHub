/**
 * @file HubRelay.h
 * @brief ESP-NOW relay engine for the Hub
 * @author Philippe Hubert
 * @date 2026
 * 
 * Manages the receive callback, relay queue, packet classification,
 * TTL processing, and retransmission via FreeRTOS task.
 */

#ifndef HUB_RELAY_H
#define HUB_RELAY_H

#include <Arduino.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "PacketTypes.h"

/// Maximum ESP-NOW packet size
static constexpr size_t MAX_PACKET_SIZE = 250;

/// Item stored in the relay queue
struct RelayQueueItem {
    uint8_t data[MAX_PACKET_SIZE];
    uint8_t length;
    uint8_t senderMac[6];
    int64_t receivedAtUs;   ///< esp_timer_get_time() at reception
};

/// Relay statistics (atomic-safe via single-core relay task)
struct RelayStats {
    uint32_t relayedCommands    = 0;
    uint32_t relayedStates      = 0;
    uint32_t relayedGPS         = 0;
    uint32_t relayedAnemometer  = 0;
    uint32_t droppedTTL         = 0;    ///< Packets with TTL=0 (not relayed)
    uint32_t droppedOwn         = 0;    ///< Own packets ignored
    uint32_t droppedQueueFull   = 0;    ///< Queue overflow
    uint32_t totalReceived      = 0;
    uint32_t totalRelayed       = 0;
};

/// Anemometer passthrough tracker (for v1 packets without TTL)
struct AnemoTracker {
    uint32_t lastSequenceNumber = UINT32_MAX;
    uint32_t lastRelayTimeMs    = 0;
    bool     active             = false;
};

/**
 * @brief ESP-NOW relay engine
 * 
 * Creates a FreeRTOS task that drains the relay queue, classifies
 * packets, checks/decrements TTL, and rebroadcasts.
 */
class HubRelay {
public:
    HubRelay();

    /**
     * @brief Initialize ESP-NOW and start the relay task
     * @param relayDelayUs Anti-collision delay in microseconds
     * @param queueSize    Relay queue depth
     * @return true if initialization succeeds
     */
    bool begin(uint32_t relayDelayUs = HUB_RELAY_DELAY_US,
               uint8_t  queueSize   = HUB_RELAY_QUEUE_SIZE);

    /// Get a const reference to current relay statistics
    const RelayStats& getStats() const { return _stats; }

    /// Get the Hub MAC address (filled after begin())
    const uint8_t* getHubMac() const { return _hubMac; }

    /// Timestamp (millis) of the last relayed packet
    uint32_t lastRelayTimeMs() const { return _lastRelayMs; }

private:
    // ─── ESP-NOW callback (static, forwards to singleton) ───
    static void onDataReceived(const uint8_t *mac, const uint8_t *data, int len);
    static HubRelay* _instance;

    // ─── Relay task ───
    static void relayTaskEntry(void *param);
    void relayTaskLoop();
    void classifyAndRelay(RelayQueueItem& item);
    bool handleAnemometerRelay(RelayQueueItem& item);

    // ─── Members ───
    QueueHandle_t   _relayQueue     = nullptr;
    TaskHandle_t    _relayTaskHandle = nullptr;
    uint8_t         _hubMac[6]      = {};
    uint8_t         _broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint32_t        _relayDelayUs   = 3000;
    RelayStats      _stats;
    AnemoTracker    _anemoTracker;
    volatile uint32_t _lastRelayMs  = 0;
};

#endif // HUB_RELAY_H
