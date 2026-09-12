#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "espnow_protocol.h"

// Seeder module. Reads the turbine and ground-wheel sensors, drives the
// tramline relay on command from the tractor, and broadcasts telemetry.

#define RELAY_PIN 12
#define TURBINE_INDUCTIVE_SENSOR_PIN 14
#define WHEEL_HALL_SENSOR_PIN 27          // metering unit, driven by the ground wheel

// The relay board is active LOW.
static constexpr uint8_t RELAY_ON  = LOW;
static constexpr uint8_t RELAY_OFF = HIGH;

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

// !!! PLACEHOLDER - MUST BE MEASURED BEFORE THE SPEED READING MEANS ANYTHING.
// The ground wheel drives the metering unit through a fixed coupling that sits
// BEFORE the seed-rate gearbox, so this value does not change when the seed
// rate is adjusted. To measure it: note wheelPulses, roll the seeder along a
// tape-measured 20 m, note wheelPulses again, then
//     MM_PER_PULSE = 20000 / (pulses after - pulses before)
static constexpr uint32_t MM_PER_PULSE = 300;

// One hole in the turbine disc per revolution.
static constexpr uint32_t TURBINE_PULSES_PER_REV = 1;

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

static constexpr uint32_t TURBINE_UPDATE_INTERVAL_MS = 2000;
static constexpr uint32_t SPEED_UPDATE_INTERVAL_MS   = 500;

// No wheel pulse for this long means the seeder really has stopped (or has
// been lifted at the headland) rather than just crawling between magnets.
static constexpr uint32_t WHEEL_STOP_TIMEOUT_MS = 2000;

// Shortest credible gap between real pulses. Anything faster is electrical
// noise on the sensor line, and left unfiltered it would inflate the reported
// speed - which, once the dispenser meters to speed, means over-applying.
static constexpr uint32_t MIN_TURBINE_PULSE_INTERVAL_US = 1000;
static constexpr uint32_t MIN_WHEEL_PULSE_INTERVAL_US   = 5000;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

// ISRs only ever increment these; they are never reset, so no pulse can be
// lost in a read-modify-write race with loop().
static volatile uint32_t turbinePulseTotal = 0;
static volatile uint32_t wheelPulseTotal   = 0;

static volatile uint32_t lastTurbinePulseUs = 0;
static volatile uint32_t lastWheelPulseUs   = 0;

static uint32_t turbineSnapshot     = 0;
static uint32_t turbineSnapshotMs   = 0;
static uint16_t turbineRPM          = 0;

static uint32_t wheelSnapshot       = 0;
static uint32_t wheelSnapshotMs     = 0;
static uint32_t lastWheelMovementMs = 0;
static uint16_t groundSpeedMmS      = 0;

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
    if (now - lastTurbinePulseUs < MIN_TURBINE_PULSE_INTERVAL_US) return;
    lastTurbinePulseUs = now;
    turbinePulseTotal++;
}

void IRAM_ATTR wheelPulseISR()
{
    uint32_t now = micros();
    if (now - lastWheelPulseUs < MIN_WHEEL_PULSE_INTERVAL_US) return;
    lastWheelPulseUs = now;
    wheelPulseTotal++;
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

static void updateGroundSpeed(uint32_t nowMs)
{
    if (nowMs - wheelSnapshotMs < SPEED_UPDATE_INTERVAL_MS) return;

    uint32_t total   = wheelPulseTotal;
    uint32_t pulses  = total - wheelSnapshot;
    uint32_t elapsed = nowMs - wheelSnapshotMs;

    wheelSnapshot   = total;
    wheelSnapshotMs = nowMs;

    if (pulses > 0) {
        lastWheelMovementMs = nowMs;
        if (elapsed > 0) {
            groundSpeedMmS = (uint16_t)((pulses * MM_PER_PULSE * 1000UL) / elapsed);
        }
    } else if (nowMs - lastWheelMovementMs >= WHEEL_STOP_TIMEOUT_MS) {
        // Genuinely stopped, or lifted at the headland.
        groundSpeedMmS = 0;
    }
    // Between those two cases the wheel is turning slower than one pulse per
    // window - hold the last speed rather than reporting a false zero.
}

static bool wheelIsTurning(uint32_t nowMs)
{
    return (lastWheelMovementMs != 0) &&
           (nowMs - lastWheelMovementMs < WHEEL_STOP_TIMEOUT_MS);
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

    SeederTelemetry telemetry;
    fillHeader(telemetry.header, MsgType::SeederTelemetry, NodeId::Seeder, links.flags());

    telemetry.upTimeMs        = nowMs;
    telemetry.wheelPulses     = wheelPulseTotal;
    telemetry.turbineRPM      = turbineRPM;
    telemetry.womRPM          = 540;   // placeholder until a real WOM sensor exists
    telemetry.groundSpeedMmS  = groundSpeedMmS;
    telemetry.wheelTurning    = wheelIsTurning(nowMs) ? 1 : 0;
    telemetry.tramlineRelayOn = (digitalRead(RELAY_PIN) == RELAY_ON) ? 1 : 0;

    broadcast(&telemetry, sizeof(telemetry), nowMs);
}

void setup()
{
    Serial.begin(115200);

    // Relay off before anything else, so a reset can never leave it energised.
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);

    pinMode(TURBINE_INDUCTIVE_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TURBINE_INDUCTIVE_SENSOR_PIN), turbinePulseISR, RISING);

    pinMode(WHEEL_HALL_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WHEEL_HALL_SENSOR_PIN), wheelPulseISR, RISING);

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

    uint32_t now = millis();
    turbineSnapshotMs = now;
    wheelSnapshotMs   = now;

    Serial.println("Seeder module ready");
}

void loop()
{
    uint32_t now = millis();

    updateTurbineRPM(now);
    updateGroundSpeed(now);
    applyRelay();
    sendTelemetry(now);
}
