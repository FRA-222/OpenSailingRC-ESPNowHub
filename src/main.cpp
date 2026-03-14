/**
 * @file main.cpp
 * @brief OpenSailingRC ESP-NOW Hub — Transparent relay
 * @author Philippe Hubert
 * @date 2026
 * 
 * Firmware for the M5Stack PowerHub (CoreS3 / ESP32-S3) acting as a
 * transparent ESP-NOW relay to extend the range of the OpenSailingRC
 * ecosystem. See ESPNOW_HUB_DESIGN.md for full architecture.
 * 
 * Relayed packet types:
 *   - CommandPacket       (Joystick → Buoys)
 *   - BuoyState           (Buoys → Joystick/Display)
 *   - AckWithStatePacket  (Buoys → Joystick)
 *   - GPSBroadcastPacket  (BoatGPS → Display)
 *   - AnemometerPacket    (Anemometer → Display)
 */

#include <Arduino.h>
#include <M5Unified.h>
#include "HubRelay.h"
#include "HubStatus.h"

// ─── Global instances ───
HubRelay  hubRelay;
HubStatus hubStatus;

// ─── Serial logging interval ───
static constexpr uint32_t LOG_INTERVAL_MS = 10000;  // 10 seconds
static uint32_t lastLogTime = 0;

// ─── LED activity indicator ───
static uint32_t lastLedUpdate = 0;
static constexpr uint32_t LED_BLINK_DURATION_MS = 50;

void printBanner() {
    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   OpenSailingRC ESP-NOW Hub Relay v1.0   ║");
    Serial.println("║   M5Stack PowerHub (CoreS3 / ESP32-S3)   ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
}

void setup() {
    // Initialize M5Stack
    auto cfg = M5.config();
    M5.begin(cfg);

    Serial.begin(115200);
    delay(500);
    printBanner();

    // ─── Initialize ESP-NOW relay engine ───
    Serial.println("→ Initializing ESP-NOW relay...");
    if (!hubRelay.begin(HUB_RELAY_DELAY_US, HUB_RELAY_QUEUE_SIZE)) {
        Serial.println("✗ FATAL: ESP-NOW relay init failed!");
        // Blink red indefinitely to signal error
        for (;;) {
            delay(1000);
        }
    }

    // ─── Initialize status broadcaster ───
    Serial.println("→ Initializing status broadcaster...");
    hubStatus.begin(&hubRelay, HUB_STATUS_INTERVAL_MS);

    Serial.println();
    Serial.println("════════════════════════════════════════════");
    Serial.println("  ✓ HUB READY — Relaying all ESP-NOW traffic");
    Serial.println("════════════════════════════════════════════");
    Serial.println();
}

void loop() {
    M5.update();
    uint32_t now = millis();

    // ─── Status broadcast (every 5s) ───
    hubStatus.update();

    // ─── LED activity indicator ───
    // Brief green flash on each relay, idle = dim blue
    if (now - hubRelay.lastRelayTimeMs() < LED_BLINK_DURATION_MS) {
        // Recent relay activity → green flash
        M5.Power.setLed(128);   // LED on (brightness)
    } else {
        M5.Power.setLed(0);     // LED off
    }

    // ─── Periodic serial statistics ───
    if (now - lastLogTime >= LOG_INTERVAL_MS) {
        lastLogTime = now;

        const RelayStats& s = hubRelay.getStats();
        if (s.totalReceived > 0) {
            Serial.println("─── Hub Statistics ───");
            Serial.printf("  Received:  %lu  |  Relayed: %lu\n",
                          s.totalReceived, s.totalRelayed);
            Serial.printf("  Commands:  %lu  |  States:  %lu\n",
                          s.relayedCommands, s.relayedStates);
            Serial.printf("  GPS:       %lu  |  Anémo:   %lu\n",
                          s.relayedGPS, s.relayedAnemometer);
            Serial.printf("  Dropped:   TTL=%lu  QueueFull=%lu  Own=%lu\n",
                          s.droppedTTL, s.droppedQueueFull, s.droppedOwn);
            Serial.printf("  Active:    Buoys=%u  Boats=%u  Anémo=%s\n",
                          hubStatus.getActiveBuoyCount(),
                          hubStatus.getActiveBoatCount(),
                          hubStatus.isAnemometerActive() ? "yes" : "no");
            Serial.printf("  Uptime:    %lu s\n", now / 1000);
            Serial.println("──────────────────────");
        } else {
            Serial.println("  (no traffic yet)");
        }
    }

    delay(10);  // Yield to FreeRTOS
}
