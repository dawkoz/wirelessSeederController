#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "dispenser_io.h"

// Fertilizer dispenser module. Meters fertilizer in proportion to the ground
// speed broadcast by the seeder, using the dose and calibration broadcast by
// the tractor. Cytron MD13S driver, Pololu 4752 motor with built-in encoder.
//
// This file is deliberately thin: the decision logic lives in
// dispenser_logic.h (hardware-free, tested by the bench image) and the I/O in
// dispenser_io.cpp. Only radio init, the receive callback and the status
// broadcast remain here.

static DispenserLogic  logic;
static DispenserOutputs output;

static uint32_t lastSendMs = 0;

static DispenserMode lastLoggedMode = DispenserMode::Normal;

static const char *modeName(DispenserMode mode)
{
    switch (mode) {
        case DispenserMode::Normal:          return "Normal";
        case DispenserMode::Calibrating:     return "Calibrating";
        case DispenserMode::CalibrationDone: return "CalibrationDone";
        case DispenserMode::Refused:         return "Refused";
        case DispenserMode::Clogged:         return "Clogged";
        case DispenserMode::Unclogging:      return "Unclogging";
        default:                             return "?";
    }
}

// Validate, copy, timestamp - nothing else. All the work goes through the
// inbox in dispenser_io.cpp; this runs in the WiFi task.
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    dispenserHandlePacket(incomingData, len);
}

static uint32_t sendFailures       = 0;
static uint32_t lastSendErrorLogMs = 0;

// Reports failures instead of discarding them. A channel mismatch between the
// radio and the registered peer makes every send fail, and without this the
// only symptom would be "no link" with nothing to explain it.
static void broadcast(const void *packet, size_t size, uint32_t now)
{
    esp_err_t result = esp_now_send(BROADCAST_ADDRESS, (const uint8_t *)packet, size);
    if (result == ESP_OK) return;

    sendFailures++;
    if (now - lastSendErrorLogMs >= 1000) {   // rate-limited, don't flood
        lastSendErrorLogMs = now;
        Serial.print("esp_now_send failed: ");
        Serial.print(esp_err_to_name(result));
        Serial.print("  total: ");
        Serial.println(sendFailures);
    }
}

static void sendStatus(uint32_t now)
{
    if (now - lastSendMs < SEND_INTERVAL_MS) return;
    lastSendMs = now;

    DispenserStatus status;
    fillHeader(status.header, MsgType::DispenserStatus, NodeId::Dispenser, dispenserLinkFlags());
    dispenserFillStatus(logic, status);

    broadcast(&status, sizeof(status), now);
}

void setup()
{
    // Motor stopped before anything else can go wrong. Note this only covers
    // the time from here onwards - the external pull-downs on PWM/DIR are
    // what hold the driver off during reset and boot.
    motorBegin();

    Serial.begin(115200);
    encoderBegin();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // No cast on the callback: if the signature ever stops matching (as it
    // does on Arduino-ESP32 3.x) this must fail to compile, not silently
    // misbehave.
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_ADDRESS, 6);
    // 0 means "whatever channel the radio is currently on". The radio is
    // pinned to ESPNOW_CHANNEL just above, so this resolves to the same thing
    // - but it can never disagree with it. Naming the channel explicitly here
    // risks the "peer channel is not equal to the home channel" error, which
    // makes every single send fail.
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
        return;
    }

    dispenserInit(logic, millis(), encoderEdges());
    Serial.println("Dispenser module ready");
}

void loop()
{
    uint32_t now = millis();

    if (dispenserControlTick(logic, now, output) && logic.mode != lastLoggedMode) {
        Serial.print('[');
        Serial.print(now);
        Serial.print("] mode ");
        Serial.print(modeName(lastLoggedMode));
        Serial.print(" -> ");
        Serial.println(modeName(logic.mode));
        lastLoggedMode = logic.mode;
    }

    sendStatus(now);
}
