/**
 * @file HubStatus.h
 * @brief Hub status broadcaster and device tracker
 * @author Philippe Hubert
 * @date 2026
 * 
 * Periodically broadcasts HubStatusPacket and tracks which
 * devices (buoys, boats, anemometer) are actively communicating.
 */

#ifndef HUB_STATUS_H
#define HUB_STATUS_H

#include <Arduino.h>
#include <esp_now.h>
#include "PacketTypes.h"
#include "HubRelay.h"

/// Timeout for considering a device inactive
static constexpr uint32_t DEVICE_TIMEOUT_MS = 30000;  // 30 seconds

/**
 * @brief Tracks active devices and broadcasts Hub status
 */
class HubStatus {
public:
    HubStatus();

    /**
     * @brief Initialize the status broadcaster
     * @param relay Pointer to HubRelay (for reading stats)
     * @param intervalMs Broadcast interval in milliseconds
     */
    void begin(HubRelay* relay, uint32_t intervalMs = HUB_STATUS_INTERVAL_MS);

    /**
     * @brief Call from loop() — broadcasts status if interval elapsed
     */
    void update();

    /**
     * @brief Notify that a buoy was seen (call from relay classify)
     * @param buoyId Buoy ID (0-5)
     */
    void notifyBuoySeen(uint8_t buoyId);

    /**
     * @brief Notify that a boat GPS was seen
     * @param name Boat name (for tracking unique boats)
     */
    void notifyBoatSeen(const char* name);

    /// Notify that the anemometer was seen
    void notifyAnemometerSeen();

    /// Get count of active buoys (seen within timeout)
    uint8_t getActiveBuoyCount() const;

    /// Get count of active boats
    uint8_t getActiveBoatCount() const;

    /// Is the anemometer active?
    bool isAnemometerActive() const;

private:
    void broadcastStatus();

    HubRelay*   _relay          = nullptr;
    uint32_t    _intervalMs     = 5000;
    uint32_t    _lastBroadcast  = 0;

    // Device tracking
    static constexpr uint8_t MAX_BUOYS = 8;
    static constexpr uint8_t MAX_BOATS = 8;

    uint32_t _buoyLastSeen[MAX_BUOYS]   = {};
    uint32_t _boatLastSeen[MAX_BOATS]   = {};
    uint8_t  _boatCount                 = 0;
    uint32_t _anemoLastSeen             = 0;

    uint8_t _broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
};

#endif // HUB_STATUS_H
