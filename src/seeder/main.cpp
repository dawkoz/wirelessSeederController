#include <Arduino.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "wheel_speed.h"

// Seeder module. Reads the turbine and ground-wheel sensors, drives the
// tramline relay on command from the tractor, and broadcasts telemetry.

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

// The turbine ISR only ever increments this and it is never reset, so no
// pulse can be lost in a read-modify-write race with loop().
static volatile uint32_t turbinePulseTotal  = 0;
static volatile uint32_t lastTurbinePulseUs = 0;

static uint32_t turbineSnapshot   = 0;
static uint32_t turbineSnapshotMs = 0;
static uint16_t turbineRPM        = 0;

// Written by the wheel ISR. loop() holds wheelLock while it copies the times
// out, so it never sees a half-recorded pulse.
static WheelPulses  wheelPulses = {};
static portMUX_TYPE wheelLock   = portMUX_INITIALIZER_UNLOCKED;

static uint32_t lastSendMs = 0;

static LinkTracker links;

// Last command from the tractor. Deliberately held indefinitely on link loss:
// dropping the tramline relay mid-pass would leave an unmarked gap in the
// field, which is a permanent defect. Holding the last state is the lesser
// harm. See Implementation_Plan section 5 in CLAUDE.md.
static TractorCommand latestCommand;
static bool haveCommand = false;

// ---------------------------------------------------------------------------

void IRAM_ATTR turbinePulseISR()
{
    uint32_t now = micros();
    if (now - lastTurbinePulseUs < TURBINE_MIN_PULSE_GAP_US) return;
    lastTurbinePulseUs = now;
    turbinePulseTotal++;
}

void IRAM_ATTR wheelPulseISR()
{
    // 64-bit microseconds never wrap; micros() does after 71 minutes.
    uint64_t now = (uint64_t)esp_timer_get_time();

    portENTER_CRITICAL_ISR(&wheelLock);
    recordWheelPulse(wheelPulses, now);
    portEXIT_CRITICAL_ISR(&wheelLock);
}

// Does nothing but validate, copy and timestamp. Everything else happens in
// loop() - work done here runs in the WiFi task and stalls packet reception.
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (incomingData == nullptr || len < (int)sizeof(MessageHeader)) return;

    MessageHeader header;
    memcpy(&header, incomingData, sizeof(header));

    if (headerValid(incomingData, len, MsgType::TractorCommand, sizeof(TractorCommand))) {
        memcpy(&latestCommand, incomingData, sizeof(latestCommand));
        haveCommand = true;
        links.noteReceived(header.sender);
    } else if (headerValid(incomingData, len, MsgType::DispenserStatus, sizeof(DispenserStatus))) {
        // Not used here, but hearing it lets us tell the tractor that the
        // seeder and dispenser can see each other.
        links.noteReceived(header.sender);
    }
}

static void updateTurbineRPM(uint32_t nowMs)
{
    if (nowMs - turbineSnapshotMs < TURBINE_UPDATE_INTERVAL_MS) return;

    uint32_t total   = turbinePulseTotal;
    uint32_t pulses  = total - turbineSnapshot;
    uint32_t elapsed = nowMs - turbineSnapshotMs;

    turbineSnapshot   = total;
    turbineSnapshotMs = nowMs;

    if (elapsed == 0) return;

    // Divide by the measured elapsed time, and do the multiply first so the
    // integer division doesn't truncate away most of the answer.
    turbineRPM = (uint16_t)((pulses * 60000UL) / (elapsed * TURBINE_PULSES_PER_REV));
}

static void applyRelay()
{
    bool on = haveCommand && latestCommand.tramlineRelayOn;
    digitalWrite(RELAY_PIN, on ? RELAY_ON : RELAY_OFF);
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

static void sendTelemetry(uint32_t nowMs)
{
    if (nowMs - lastSendMs < SEND_INTERVAL_MS) return;
    lastSendMs = nowMs;

    uint64_t recentUs[WHEEL_RING_SIZE];

    portENTER_CRITICAL(&wheelLock);
    uint32_t wheelPulseTotal = wheelPulses.total;
    uint8_t  count           = copyWheelPulses(wheelPulses, recentUs);
    portEXIT_CRITICAL(&wheelLock);

    // Distance per pulse comes from the tractor: the sensor is on the metering
    // drive, so it depends on which seed-size gear the machine is in. Held
    // after the tractor goes quiet, like the relay state; the default covers
    // "never heard from it at all".
    uint16_t mmPerPulse = haveCommand ? validWheelMmPerPulse(latestCommand.wheelMmPerPulse)
                                      : WHEEL_MM_PER_PULSE_DEFAULT;

    // Clock read after the copy, so it is never earlier than the newest pulse.
    uint16_t speed = wheelSpeedMmS(recentUs, count, (uint64_t)esp_timer_get_time(), mmPerPulse);

    SeederTelemetry telemetry;
    fillHeader(telemetry.header, MsgType::SeederTelemetry, NodeId::Seeder, links.flags());

    telemetry.upTimeMs        = nowMs;
    telemetry.wheelPulses     = wheelPulseTotal;
    telemetry.turbineRPM      = turbineRPM;
    telemetry.womRPM          = 540;   // placeholder until a real WOM sensor exists
    telemetry.groundSpeedMmS  = speed;
    telemetry.wheelTurning    = (speed > 0) ? 1 : 0;
    telemetry.tramlineRelayOn = (digitalRead(RELAY_PIN) == RELAY_ON) ? 1 : 0;

    broadcast(&telemetry, sizeof(telemetry), nowMs);
}

void setup()
{
    Serial.begin(115200);

    // Relay off before anything else, so a reset can never leave it energised.
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);

    pinMode(TURBINE_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TURBINE_SENSOR_PIN), turbinePulseISR, RISING);

    pinMode(WHEEL_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WHEEL_SENSOR_PIN), wheelPulseISR, RISING);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    // Power save is a common cause of intermittently missed ESP-NOW frames.
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

    turbineSnapshotMs = millis();

    Serial.println("Seeder module ready");
}

void loop()
{
    uint32_t now = millis();

    updateTurbineRPM(now);
    applyRelay();
    sendTelemetry(now);
}
