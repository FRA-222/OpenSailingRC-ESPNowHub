/**
 * @file PacketTypes.h
 * @brief V2 packet structures for the ESP-NOW relay Hub
 * @author Philippe Hubert
 * @date 2026
 * 
 * These structures define the v2 wire format for ALL ESP-NOW packets
 * relayed by the Hub. Each structure includes a TTL field used to
 * prevent relay loops.
 * 
 * IMPORTANT: These structures MUST match exactly the ones defined
 * in each source project after the Phase 1 protocol update:
 *   - CommandPacket       → OpenSailingRC-BuoyJoystick / Autonomous-GPS-Buoy
 *   - BuoyState           → Autonomous-GPS-Buoy
 *   - AckWithStatePacket  → Autonomous-GPS-Buoy
 *   - GPSBroadcastPacket  → OpenSailingRC-BoatGPS
 *   - AnemometerPacket    → External anemometer / OpenSailingRC-Display
 *   - HubStatusPacket     → This project (OpenSailingRC-ESPNowHub)
 */

#ifndef PACKET_TYPES_H
#define PACKET_TYPES_H

#include <Arduino.h>

// ─── Message type constants (first byte for typed packets) ───
static constexpr uint8_t MSG_TYPE_BOAT_GPS     = 1;
static constexpr uint8_t MSG_TYPE_ANEMOMETER   = 2;
static constexpr uint8_t MSG_TYPE_HUB_STATUS   = 10;

// ─── Enums matching Autonomous-GPS-Buoy (ModeManagement.h) ───
// Values must match for struct layout; relay doesn't interpret them.
enum tEtatsGeneral {
    BUOY_INIT = 0,
    BUOY_READY,
    BUOY_MAINTENANCE,
    BUOY_HOME_DEFINITION,
    BUOY_NAV
};

enum tEtatsNav {
    NAV_NOTHING = 0,
    NAV_HOME,
    NAV_HOLD,
    NAV_STOP,
    NAV_BASIC,
    NAV_CAP,
    NAV_TARGET
};

// ─── BuoyCommand enum (matching Autonomous-GPS-Buoy) ───
enum BuoyCommand : uint8_t {
    CMD_INIT_HOME = 0,
    CMD_THROTTLE_INCREASE,
    CMD_THROTTLE_DECREASE,
    CMD_HEADING_INCREASE,
    CMD_HEADING_DECREASE,
    CMD_NAV_HOLD,
    CMD_NAV_CAP,
    CMD_NAV_HOME,
    CMD_NAV_STOP,
    CMD_HOME_VALIDATION,
    CMD_HEARTBEAT
};

// ═══════════════════════════════════════════════════════════════
// Packet structures — v2 with TTL field
// ═══════════════════════════════════════════════════════════════

/**
 * @brief Command packet (Joystick → Buoy) — v2 packed
 * TTL at last byte. sequenceNumber for deduplication.
 */
struct __attribute__((packed)) CommandPacket {
    uint8_t targetBuoyId;       ///< Target buoy ID (0-5), 0xFF = all
    uint8_t command;            ///< BuoyCommand enum value
    uint32_t timestamp;         ///< Command timestamp (for ACK matching)
    uint16_t sequenceNumber;    ///< Global sequence number (for deduplication)
    uint8_t ttl;                ///< Time-To-Live: 1=original, 0=relayed
};
// Expected size: 9 bytes

/**
 * @brief Buoy state (Buoy → Joystick/Display) — v2 NOT packed
 * Must match ESPNowDataLinkManagement.h BuoyState in Autonomous-GPS-Buoy
 */
struct BuoyState {
    uint8_t buoyId;
    uint32_t timestamp;

    tEtatsGeneral generalMode;
    tEtatsNav navigationMode;

    bool gpsOk;
    bool headingOk;
    bool yawRateOk;

    double latitude;
    double longitude;

    float temperature;
    float remainingCapacity;
    float distanceToCons;

    int8_t autoPilotThrottleCmde;
    float autoPilotTrueHeadingCmde;

    // v2 additions
    uint16_t sequenceNumber;
    uint8_t ttl;
};

/**
 * @brief ACK with buoy state (Buoy → Joystick) — v2 packed
 * Must match ESPNowDataLinkManagement.h AckWithStatePacket
 */
struct __attribute__((packed)) AckWithStatePacket {
    uint8_t buoyId;
    uint32_t commandTimestamp;
    uint8_t commandType;

    uint8_t generalMode;
    uint8_t navigationMode;
    bool gpsOk;
    bool headingOk;
    bool yawRateOk;

    float temperature;
    float remainingCapacity;
    float distanceToCons;

    int8_t autoPilotThrottleCmde;
    int16_t autoPilotTrueHeadingCmde;

    // v2 addition
    uint8_t ttl;
};
// Expected size: 27 bytes

/**
 * @brief GPS broadcast packet (BoatGPS → Display) — v2 NOT packed
 * Must match Communication.h GPSBroadcastPacket in OpenSailingRC-BoatGPS
 */
struct GPSBroadcastPacket {
    int8_t messageType;         ///< 1 = Boat GPS data
    char name[18];              ///< Boat name or MAC address
    uint32_t sequenceNumber;    ///< Sequence number (existing)
    uint32_t gpsTimestamp;      ///< GPS timestamp (ms)
    float latitude;
    float longitude;
    float speed;                ///< Speed in knots
    float heading;              ///< Heading in degrees
    uint8_t satellites;

    // v2 addition
    uint8_t ttl;
};

/**
 * @brief Anemometer data (Anemometer → Display) — v2 NOT packed
 * Must match struct_message_Anemometer in OpenSailingRC-Display
 */
struct AnemometerPacket {
    int8_t messageType;         ///< 2 = Anemometer
    char anemometerId[18];      ///< Anemometer ID string
    uint8_t macAddress[6];      ///< MAC address
    uint32_t sequenceNumber;    ///< Sequence number (existing)
    float windSpeed;            ///< Wind speed (m/s)
    unsigned long timestamp;    ///< Measurement timestamp

    // v2 addition
    uint8_t ttl;
};

/**
 * @brief Anemometer data — v1 (no TTL, firmware not modifiable)
 * Used for passthrough relay of legacy anemometer devices.
 */
struct AnemometerPacketV1 {
    int8_t messageType;
    char anemometerId[18];
    uint8_t macAddress[6];
    uint32_t sequenceNumber;
    float windSpeed;
    unsigned long timestamp;
};

/**
 * @brief Hub status packet (Hub → all devices)
 * Broadcast periodically by the Hub. Never relayed.
 */
struct __attribute__((packed)) HubStatusPacket {
    uint8_t messageType;        ///< MSG_TYPE_HUB_STATUS = 10
    uint8_t hubId;              ///< Hub ID (default 0)
    uint32_t uptimeMs;          ///< Uptime in milliseconds
    uint32_t relayedCommands;
    uint32_t relayedStates;
    uint32_t relayedGPS;
    uint32_t relayedAnemometer;
    float batteryVoltage;       ///< Battery voltage (V)
    int8_t rssiAvg;             ///< Average RSSI of received packets
    uint8_t connectedBuoys;     ///< Buoys seen in last 30s
    uint8_t connectedBoats;     ///< Boats seen in last 30s
    bool anemometerSeen;        ///< Anemometer seen in last 30s
};
// Expected size: 30 bytes

#endif // PACKET_TYPES_H
