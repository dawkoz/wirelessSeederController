#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "machine_settings.h"
#include "espnow_protocol.h"

// Fertilizer dispenser module. Meters fertilizer in proportion to the ground
// speed broadcast by the seeder, using the dose and calibration broadcast by
// the tractor. Cytron MD13S driver, Pololu 4752 motor with built-in encoder.

static constexpr uint8_t  LEDC_CHANNEL    = 0;
static constexpr uint8_t  LEDC_RESOLUTION = 10;     // 0..1023
static constexpr uint16_t LEDC_MAX        = (1 << LEDC_RESOLUTION) - 1;

static constexpr uint32_t CALIBRATION_TOTAL_EDGES =
    (uint32_t)CALIBRATION_REVOLUTIONS * ENCODER_EDGES_PER_REV;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static volatile uint32_t encoderEdgeTotal  = 0;
static volatile uint32_t lastEncoderEdgeUs = 0;

static uint32_t encoderSnapshot   = 0;
static uint32_t encoderSnapshotMs = 0;
static uint16_t measuredShaftRPM  = 0;

static uint16_t targetShaftRPM        = 0;
static uint16_t motorCommandedPermille = 0;
static float    integralTerm          = 0.0f;

static uint32_t motorCommandedSinceMs = 0;
static DispenserFault faultCode = DispenserFault::None;

static CalibrationState calibrationState  = CalibrationState::Idle;
static uint8_t          calibrationPercent = 0;
static uint32_t         calibrationStartEdges = 0;

static uint32_t lastSendMs = 0;

static LinkTracker links;

static SeederTelemetry latestTelemetry = {};
static bool haveTelemetry = false;

// Held indefinitely if the tractor goes quiet: an unfertilised strip is a
// permanent defect in the field, the same reasoning as the tramline relay on
// the seeder. Safe because the ground-speed interlock below still stops the
// motor whenever the machine isn't moving.
static TractorCommand latestCommand = {};
static bool haveCommand = false;

// ---------------------------------------------------------------------------

void IRAM_ATTR encoderPulseISR()
{
    uint32_t now = micros();
    if (now - lastEncoderEdgeUs < ENCODER_MIN_PULSE_GAP_US) return;
    lastEncoderEdgeUs = now;
    encoderEdgeTotal++;
}

// Validate, copy, timestamp - nothing else. See the tractor and seeder for why.
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (incomingData == nullptr || len < (int)sizeof(MessageHeader)) return;

    MessageHeader header;
    memcpy(&header, incomingData, sizeof(header));

    if (headerValid(incomingData, len, MsgType::SeederTelemetry, sizeof(SeederTelemetry))) {
        memcpy(&latestTelemetry, incomingData, sizeof(latestTelemetry));
        haveTelemetry = true;
        links.noteReceived(header.sender);
    } else if (headerValid(incomingData, len, MsgType::TractorCommand, sizeof(TractorCommand))) {
        memcpy(&latestCommand, incomingData, sizeof(latestCommand));
        haveCommand = true;
        links.noteReceived(header.sender);
    }
}

static void setMotorPermille(uint16_t permille)
{
    if (permille > 1000) permille = 1000;
    motorCommandedPermille = permille;
    ledcWrite(LEDC_CHANNEL, (uint32_t)permille * LEDC_MAX / 1000);
}

static void stopMotor()
{
    setMotorPermille(0);
    integralTerm          = 0.0f;
    targetShaftRPM        = 0;
    motorCommandedSinceMs = 0;
}

static void updateMeasuredRPM(uint32_t now, uint32_t elapsed)
{
    uint32_t total = encoderEdgeTotal;
    uint32_t edges = total - encoderSnapshot;
    encoderSnapshot   = total;
    encoderSnapshotMs = now;

    if (elapsed == 0) return;
    measuredShaftRPM = (uint16_t)((edges * 60000UL) / (elapsed * ENCODER_EDGES_PER_REV));
}

// Required output-shaft RPM to apply `dose` kg/ha at `speed` mm/s, given a
// calibration of `gramsPer100Rev` grams per 100 revolutions.
//
//   grams per metre  = (width_cm / 100) * dose / 10   = width_cm * dose / 1000
//   revs per metre   = gramsPerMetre * 100 / gramsPer100Rev
//                    = width_cm * dose / (10 * gramsPer100Rev)
//   revs per second  = revsPerMetre * speedMmS / 1000
//   RPM              = revsPerSecond * 60
//                    = width_cm * dose * speedMmS * 60 / (10000 * gramsPer100Rev)
//
// Folded into one expression to avoid intermediate rounding. 64-bit because
// the numerator reaches ~1.6e12 at the extremes of the input ranges.
//
// Check: 4 m width, 40 kg/ha, 500 g/100rev, 2 m/s
//        -> 2000*40*400*60 / (10000*500) = 384 RPM  (above the motor's 330,
//        which is exactly the over-speed case handled by the caller).
static uint32_t requiredShaftRPM(uint16_t speedMmS, uint16_t doseKgPerHa, uint32_t gramsPer100Rev)
{
    if (gramsPer100Rev == 0 || doseKgPerHa == 0 || speedMmS == 0) return 0;

    uint64_t numerator   = (uint64_t)speedMmS * doseKgPerHa * WORKING_WIDTH_CM * 60ULL;
    uint64_t denominator = (uint64_t)gramsPer100Rev * 10000ULL;

    return (uint32_t)(numerator / denominator);
}

// Feed-forward plus a PI trim, shared by normal metering and the calibration
// run. The feed-forward does essentially all the work; the PI only corrects
// for battery sag and auger load.
static uint16_t computeDuty(uint16_t target, uint32_t elapsed)
{
    float feedForward = (float)target * 1000.0f / (float)MOTOR_MAX_RPM;

    float error = (float)target - (float)measuredShaftRPM;
    integralTerm += error * ((float)elapsed / 1000.0f) * MOTOR_KI;
    if (integralTerm >  MOTOR_INTEGRAL_LIMIT) integralTerm =  MOTOR_INTEGRAL_LIMIT;
    if (integralTerm < -MOTOR_INTEGRAL_LIMIT) integralTerm = -MOTOR_INTEGRAL_LIMIT;

    float output = feedForward + MOTOR_KP * error + integralTerm;
    if (output < 0.0f)    output = 0.0f;
    if (output > 1000.0f) output = 1000.0f;

    uint16_t permille = (uint16_t)output;
    if (permille > 0 && permille < MOTOR_MIN_RUNNING_PERMILLE) permille = MOTOR_MIN_RUNNING_PERMILLE;
    return permille;
}

// Returns true when a calibration run owns the motor, so normal metering is
// skipped. Calibration deliberately ignores the ground-speed interlock - it
// is a stationary bench operation, and the seeder may not even be powered.
static bool runCalibration(uint32_t now, uint32_t elapsed)
{
    bool requested = haveCommand && latestCommand.calibrationRun;

    if (!requested) {
        if (calibrationState != CalibrationState::Idle) {
            calibrationState   = CalibrationState::Idle;
            calibrationPercent = 0;
            stopMotor();
        }
        return false;
    }

    // The tractor is what commands this run, so losing it aborts.
    if (!links.isAlive(NodeId::Tractor)) {
        calibrationState   = CalibrationState::Idle;
        calibrationPercent = 0;
        stopMotor();
        return false;
    }

    // Finished, or refused to start: hold with the motor off until the
    // tractor withdraws the request, so the operator sees the result.
    if (calibrationState == CalibrationState::Done ||
        calibrationState == CalibrationState::Refused) {
        stopMotor();
        return true;
    }

    // Never spin the auger while the machine is actually moving - whether
    // that is already true at the start or becomes true part-way through.
    if (links.isAlive(NodeId::Seeder) && haveTelemetry && latestTelemetry.wheelTurning) {
        calibrationState   = CalibrationState::Refused;
        calibrationPercent = 0;
        stopMotor();
        return true;
    }

    if (calibrationState != CalibrationState::Running) {
        calibrationState      = CalibrationState::Running;
        calibrationStartEdges = encoderEdgeTotal;
        calibrationPercent    = 0;
        integralTerm          = 0.0f;
    }

    uint32_t turned = encoderEdgeTotal - calibrationStartEdges;
    if (turned >= CALIBRATION_TOTAL_EDGES) {
        calibrationState   = CalibrationState::Done;
        calibrationPercent = 100;
        stopMotor();
        return true;
    }

    calibrationPercent = (uint8_t)(((uint64_t)turned * 100ULL) / CALIBRATION_TOTAL_EDGES);
    targetShaftRPM     = CALIBRATION_RPM;
    faultCode          = DispenserFault::None;
    setMotorPermille(computeDuty(CALIBRATION_RPM, elapsed));
    return true;
}

static void runControl(uint32_t now)
{
    uint32_t elapsed = now - encoderSnapshotMs;
    if (elapsed < MOTOR_CONTROL_INTERVAL_MS) return;

    updateMeasuredRPM(now, elapsed);

    if (runCalibration(now, elapsed)) return;

    // Without ground speed the dispenser cannot meter at all, so running would
    // apply an arbitrary unknown rate. This is the one link loss that stops it.
    if (!links.isAlive(NodeId::Seeder) || !haveTelemetry) {
        faultCode = DispenserFault::NoSpeedData;
        stopMotor();
        return;
    }

    bool     enabled = haveCommand && latestCommand.dispenserEnabled;
    uint16_t dose    = enabled ? latestCommand.doseKgPerHa    : 0;
    uint32_t calib   = haveCommand ? latestCommand.gramsPer100Rev : 0;

    // Not moving: headland turn, lifted, or standing still. Normal, not a fault.
    if (!latestTelemetry.wheelTurning || latestTelemetry.groundSpeedMmS == 0 || dose == 0) {
        faultCode = DispenserFault::None;
        stopMotor();
        return;
    }

    uint32_t required = requiredShaftRPM(latestTelemetry.groundSpeedMmS, dose, calib);

    bool overSpeed = (required > MOTOR_MAX_RPM);
    if (overSpeed) required = MOTOR_MAX_RPM;   // clamp and complain, don't hide it
    targetShaftRPM = (uint16_t)required;

    uint16_t permille = computeDuty(targetShaftRPM, elapsed);
    setMotorPermille(permille);

    // Stall detection: commanded to turn, but the shaft isn't. A blocked or
    // decoupled dispenser is otherwise completely silent, and the failure is
    // only discovered once a whole field has been done wrong.
    if (permille >= MOTOR_MIN_RUNNING_PERMILLE) {
        if (motorCommandedSinceMs == 0) motorCommandedSinceMs = now;
    } else {
        motorCommandedSinceMs = 0;
    }

    bool stalled = (motorCommandedSinceMs != 0) &&
                   (now - motorCommandedSinceMs >= STALL_TIMEOUT_MS) &&
                   (measuredShaftRPM < STALL_RPM_THRESHOLD);

    if (stalled)         faultCode = DispenserFault::Stalled;
    else if (overSpeed)  faultCode = DispenserFault::OverSpeed;
    else                 faultCode = DispenserFault::None;
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
    fillHeader(status.header, MsgType::DispenserStatus, NodeId::Dispenser, links.flags());

    status.targetShaftRPM         = targetShaftRPM;
    status.measuredShaftRPM       = measuredShaftRPM;
    status.motorCommandedPermille = motorCommandedPermille;
    status.motorRunning           = (motorCommandedPermille > 0) ? 1 : 0;
    status.faultCode              = faultCode;
    status.calibrationState       = calibrationState;
    status.calibrationPercent     = calibrationPercent;

    broadcast(&status, sizeof(status), now);
}

void setup()
{
    // Motor stopped before anything else can go wrong. Note this only covers
    // the time from here onwards - the external pull-downs on PWM/DIR are
    // what hold the driver off during reset and boot.
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIR_FORWARD);
    ledcSetup(LEDC_CHANNEL, MOTOR_PWM_FREQUENCY_HZ, LEDC_RESOLUTION);
    ledcAttachPin(MOTOR_PWM_PIN, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, 0);

    Serial.begin(115200);

    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderPulseISR, RISING);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);   // reserved for quadrature if ever needed

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

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

    encoderSnapshotMs = millis();
    Serial.println("Dispenser module ready");
}

void loop()
{
    uint32_t now = millis();

    runControl(now);
    sendStatus(now);
}
