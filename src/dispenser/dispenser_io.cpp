#include "dispenser_io.h"

#include "machine_settings.h"
#include "espnow_protocol.h"

// ---------------------------------------------------------------------------
// Motor output (MD13S: PWM + DIR)
// ---------------------------------------------------------------------------

static constexpr uint8_t  LEDC_CHANNEL    = 0;
static constexpr uint8_t  LEDC_RESOLUTION = 10;     // 0..1023
static constexpr uint16_t LEDC_MAX        = (1 << LEDC_RESOLUTION) - 1;

// The level currently on the DIR pin, so a direction change can drop the duty
// to 0 first.
static bool motorDirForward = true;

static uint8_t dirLevel(bool forward)
{
    return forward ? MOTOR_DIR_FORWARD : ((MOTOR_DIR_FORWARD == LOW) ? HIGH : LOW);
}

// DIR forward, LEDC attached, duty 0. Must stay the very first call in
// setup(): the pull-downs on the driver inputs only cover the time before it.
void motorBegin()
{
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIR_FORWARD);
    motorDirForward = true;
    ledcSetup(LEDC_CHANNEL, MOTOR_PWM_FREQUENCY_HZ, LEDC_RESOLUTION);
    ledcAttachPin(MOTOR_PWM_PIN, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, 0);
}

void motorApply(const DispenserOutputs &out)
{
    uint16_t permille = out.motorPermille;
    if (permille > 1000) permille = 1000;

    if (out.motorForward != motorDirForward) {
        // Duty 0 before flipping DIR, so the driver is never asked to brake
        // one way into full speed the other.
        ledcWrite(LEDC_CHANNEL, 0);
        digitalWrite(MOTOR_DIR_PIN, dirLevel(out.motorForward));
        motorDirForward = out.motorForward;
    }
    ledcWrite(LEDC_CHANNEL, (uint32_t)permille * LEDC_MAX / 1000);
}

// ---------------------------------------------------------------------------
// Encoder (channel A rising edges only; see the design record)
// ---------------------------------------------------------------------------

static volatile uint32_t encoderEdgeTotal  = 0;
static volatile uint32_t lastEncoderEdgeUs = 0;

void IRAM_ATTR encoderPulseISR()
{
    uint32_t now = micros();
    if (now - lastEncoderEdgeUs < ENCODER_MIN_PULSE_GAP_US) return;
    lastEncoderEdgeUs = now;
    encoderEdgeTotal++;
}

void encoderBegin()
{
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderPulseISR, RISING);
}

void encoderEnd()
{
    detachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN));
}

uint32_t encoderEdges()
{
    return encoderEdgeTotal;
}

// ---------------------------------------------------------------------------
// Receive inbox: the latest validated packets and the link tracker. The
// receive callback and loop() run in different contexts, so every write and
// read of the packet copies goes through one lock.
// ---------------------------------------------------------------------------

static SeederTelemetry latestTelemetry = {};
static bool haveTelemetry = false;

// Held indefinitely if the tractor goes quiet: an unfertilised strip is a
// permanent defect in the field, the same reasoning as the tramline relay on
// the seeder. Safe because the ground-speed interlock still stops the motor
// whenever the machine isn't moving.
static TractorCommand latestCommand = {};
static bool haveCommand = false;

static LinkTracker links;
static portMUX_TYPE inboxLock = portMUX_INITIALIZER_UNLOCKED;

// Validate, copy, timestamp - nothing else. This is the whole body of the
// receive callback; see the tractor and seeder for why nothing else may
// happen here.
void dispenserHandlePacket(const uint8_t *data, int len)
{
    if (data == nullptr || len < (int)sizeof(MessageHeader)) return;

    MessageHeader header;
    memcpy(&header, data, sizeof(header));

    if (headerValid(data, len, MsgType::SeederTelemetry, sizeof(SeederTelemetry))) {
        portENTER_CRITICAL(&inboxLock);
        memcpy(&latestTelemetry, data, sizeof(latestTelemetry));
        haveTelemetry = true;
        portEXIT_CRITICAL(&inboxLock);
        links.noteReceived(header.sender);
    } else if (headerValid(data, len, MsgType::TractorCommand, sizeof(TractorCommand))) {
        portENTER_CRITICAL(&inboxLock);
        memcpy(&latestCommand, data, sizeof(latestCommand));
        haveCommand = true;
        portEXIT_CRITICAL(&inboxLock);
        links.noteReceived(header.sender);
    }
}

void dispenserReadInputs(uint32_t nowMs, DispenserInputs &in)
{
    in.nowMs        = nowMs;
    in.encoderEdges = encoderEdges();

    portENTER_CRITICAL(&inboxLock);
    in.haveTelemetry = haveTelemetry;
    in.telemetry     = latestTelemetry;
    in.haveCommand   = haveCommand;
    in.command       = latestCommand;
    portEXIT_CRITICAL(&inboxLock);

    in.seederAlive  = links.isAlive(NodeId::Seeder);
    in.tractorAlive = links.isAlive(NodeId::Tractor);
}

void dispenserResetInbox()
{
    portENTER_CRITICAL(&inboxLock);
    latestTelemetry = {};
    latestCommand   = {};
    haveTelemetry   = false;
    haveCommand     = false;
    links = LinkTracker();
    portEXIT_CRITICAL(&inboxLock);
}

uint8_t dispenserLinkFlags()
{
    return links.flags();
}

bool dispenserControlTick(DispenserLogic &logic, uint32_t nowMs, DispenserOutputs &out)
{
    DispenserInputs in;
    dispenserReadInputs(nowMs, in);
    if (!dispenserStep(logic, in, out)) return false;
    motorApply(out);
    return true;
}
