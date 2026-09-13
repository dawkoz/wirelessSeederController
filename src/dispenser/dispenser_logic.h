#pragma once

#include "machine_settings.h"
#include "espnow_protocol.h"

// ---------------------------------------------------------------------------
// Dispenser decision logic, kept completely free of hardware so the bench
// test (src/dispenser_bench) can drive it with simulated time, encoder counts
// and packets.
// Nothing in here may read the clock, touch a pin, write serial or hold any
// global state - all state lives in DispenserLogic, and time and the encoder
// count come in through DispenserInputs.
// ---------------------------------------------------------------------------

static constexpr uint32_t CALIBRATION_TOTAL_EDGES =
    (uint32_t)CALIBRATION_REVOLUTIONS * ENCODER_EDGES_PER_REV;

struct DispenserInputs {
    uint32_t        nowMs;
    uint32_t        encoderEdges;    // channel A rising edges since boot, never reset
    bool            seederAlive;     // valid SeederTelemetry within LINK_TIMEOUT_MS
    bool            haveTelemetry;   // at least one ever received
    SeederTelemetry telemetry;       // the latest one (stale when !seederAlive)
    bool            tractorAlive;
    bool            haveCommand;
    TractorCommand  command;
};

struct DispenserOutputs {
    uint16_t motorPermille;          // 0..1000
    bool     motorForward;
};

struct DispenserLogic {
    // Step timing: prevStepMs is also set by dispenserInit, so the interval
    // check works from the very first step without a separate flag.
    uint32_t prevStepMs = 0;

    // Measured shaft speed: encoder snapshot and the RPM over the last step.
    uint32_t encoderSnapshot = 0;
    uint16_t measuredRPM     = 0;

    // Mode machine.
    DispenserMode  mode     = DispenserMode::Normal;
    DispenserFault fault    = DispenserFault::None;
    uint8_t        progress = 0;                  // see DispenserStatus.progressPercent

    // Speed controller state. Integral 0, target 0 and the clog timer stopped
    // is a "controller reset", done on every mode change and whenever metering
    // stops within Normal.
    uint16_t targetRPM      = 0;
    float    integralTerm   = 0.0f;

    // Clog timer. A running timer is marked by the bool, never by the start
    // time being nonzero.
    bool     clogTimerRunning = false;
    uint32_t clogTimerStartMs = 0;

    // Calibration run bookkeeping. calibrationArmed is what stops a run from
    // starting by itself after a reboot, a link gap, a clog or a cancel: it
    // is only set by a received command with calibrationRun == 0, and cleared
    // when a run actually starts (or is refused).
    bool     calibrationArmed = false;
    uint32_t calibrationStartEdges = 0;

    // Tractor command counters (see TractorCommand). lastCommandUpTimeMs is
    // how a tractor reboot is told apart from a fresh counter value.
    uint8_t  lastClearSeq        = 0;
    uint8_t  lastUnclogSeq       = 0;
    uint32_t lastCommandUpTimeMs = 0;
    bool     tractorWasAlive     = false;

    // Unclogging sequence start time.
    uint32_t unclogStartMs = 0;

    // The most recent step's output, after the reversal guard. It is what the
    // status reports and what the guard compares the next step against.
    DispenserOutputs prevOutput = {0, true};
};

// Motor off and controller reset in one step: integral 0, target 0, clog
// timer stopped.
inline void resetController(DispenserLogic &logic)
{
    logic.integralTerm     = 0.0f;
    logic.targetRPM        = 0;
    logic.clogTimerRunning = false;
    logic.clogTimerStartMs = 0;
}

inline void dispenserInit(DispenserLogic &logic, uint32_t nowMs, uint32_t edges)
{
    logic.prevStepMs        = nowMs;
    logic.encoderSnapshot   = edges;
    logic.measuredRPM       = 0;
    logic.mode              = DispenserMode::Normal;
    logic.fault             = DispenserFault::None;
    logic.progress          = 0;
    logic.calibrationArmed  = false;
    logic.calibrationStartEdges = 0;
    logic.lastClearSeq      = 0;
    logic.lastUnclogSeq     = 0;
    logic.lastCommandUpTimeMs = 0;
    logic.tractorWasAlive   = false;
    logic.unclogStartMs     = 0;
    logic.prevOutput.motorPermille = 0;
    logic.prevOutput.motorForward  = true;
    resetController(logic);
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
inline uint32_t requiredShaftRPM(uint16_t speedMmS, uint16_t doseKgPerHa, uint32_t gramsPer100Rev)
{
    if (gramsPer100Rev == 0 || doseKgPerHa == 0 || speedMmS == 0) return 0;

    uint64_t numerator   = (uint64_t)speedMmS * doseKgPerHa * WORKING_WIDTH_CM * 60ULL;
    uint64_t denominator = (uint64_t)gramsPer100Rev * 10000ULL;

    return (uint32_t)(numerator / denominator);
}

// Feed-forward plus a PI trim, shared by normal metering and the calibration
// run. The feed-forward does essentially all the work; the PI only corrects
// for battery sag and auger load. The anti-windup condition stops the integral
// from pushing further when the output is already saturated at either end -
// without it, a long spell at a low target (or full duty) would take seconds
// of oscillation to unwind after the target changes.
inline uint16_t computeDuty(DispenserLogic &logic, uint16_t target, uint32_t elapsed)
{
    float feedForward = (float)target * 1000.0f / (float)MOTOR_MAX_RPM;

    float error = (float)target - (float)logic.measuredRPM;

    float candidate = logic.integralTerm + error * ((float)elapsed / 1000.0f) * MOTOR_KI;
    if (candidate >  MOTOR_INTEGRAL_LIMIT) candidate =  MOTOR_INTEGRAL_LIMIT;
    if (candidate < -MOTOR_INTEGRAL_LIMIT) candidate = -MOTOR_INTEGRAL_LIMIT;

    float raw = feedForward + MOTOR_KP * error + candidate;
    if (!((raw > 1000.0f && error > 0.0f) ||
          (raw < (float)MOTOR_MIN_RUNNING_PERMILLE && error < 0.0f))) {
        logic.integralTerm = candidate;
    }

    // The trim may cancel up to all of the feed-forward, never more: a negative
    // integral left over from a higher target (a motor running faster than the
    // feed-forward expects) must not hold the duty at 0 once the target drops.
    if (logic.integralTerm < -feedForward) logic.integralTerm = -feedForward;

    float output = feedForward + MOTOR_KP * error + logic.integralTerm;
    if (output < 0.0f)    output = 0.0f;
    if (output > 1000.0f) output = 1000.0f;

    uint16_t permille = (uint16_t)output;
    if (permille > 0 && permille < MOTOR_MIN_RUNNING_PERMILLE) permille = MOTOR_MIN_RUNNING_PERMILLE;
    return permille;
}

// Clog check: the shaft turning slower than CLOG_MIN_SPEED_PERCENT of the
// commanded speed for CLOG_DETECT_MS, while the motor is actually being
// driven. A step that leaves the motor off is not a stall - the shaft is meant
// to be still - so it also stops the timer.
//
// The case this really covers: the integral goes negative when the motor turns
// faster than the feed-forward expects, which is normal on a tractor at about
// 14 V. Slow down while still seeding and a large negative integral could
// cancel the whole feed-forward, holding the duty at 0 with the shaft stopped.
// Without this guard that reads as a clog. The clamp in computeDuty() is what
// stops the duty reaching 0 in the first place; the guard covers every other
// way the duty can legitimately be 0. It hides no real clog, because a stopped
// shaft makes the error, and so the duty, positive.
//
// computeDuty never returns 1..MOTOR_MIN_RUNNING_PERMILLE, so "duty > 0" is
// the same test as "duty >= MOTOR_MIN_RUNNING_PERMILLE".
inline bool clogDetected(DispenserLogic &logic, uint32_t nowMs, uint16_t targetRPM, uint16_t duty)
{
    if (duty == 0) {
        logic.clogTimerRunning = false;
        return false;
    }

    bool slow = (uint32_t)logic.measuredRPM * 100 <
                (uint32_t)targetRPM * CLOG_MIN_SPEED_PERCENT;

    if (slow) {
        if (!logic.clogTimerRunning) {
            logic.clogTimerRunning = true;
            logic.clogTimerStartMs = nowMs;
        }
        return (nowMs - logic.clogTimerStartMs) >= CLOG_DETECT_MS;
    }
    logic.clogTimerRunning = false;
    return false;
}

// One control step. Returns false (with out untouched) until
// MOTOR_CONTROL_INTERVAL_MS has passed since the previous step or since
// dispenserInit; otherwise runs exactly one step with that elapsed time.
inline bool dispenserStep(DispenserLogic &logic, const DispenserInputs &in, DispenserOutputs &out)
{
    if (in.nowMs - logic.prevStepMs < MOTOR_CONTROL_INTERVAL_MS) return false;
    uint32_t elapsed = in.nowMs - logic.prevStepMs;
    logic.prevStepMs = in.nowMs;

    // 1. Measure: edges since the previous step, then the RPM over it. The
    //    64-bit maths keeps a long gap between steps from overflowing.
    uint32_t edges = in.encoderEdges - logic.encoderSnapshot;
    logic.encoderSnapshot = in.encoderEdges;
    uint64_t rpm = (uint64_t)edges * 60000ULL / ((uint64_t)elapsed * ENCODER_EDGES_PER_REV);
    logic.measuredRPM = (rpm > 65535) ? 65535 : (uint16_t)rpm;

    // 2. Tractor command counters. A change is only acted on once, and only
    //    if it did not arrive right after a link gap or a tractor reboot:
    //    acting then would replay an old operator decision (or worse, run the
    //    motor on a rebooted dispenser's own initiative).
    bool clearNow  = false;
    bool unclogNow = false;
    if (in.tractorAlive && in.haveCommand) {
        bool resync = !logic.tractorWasAlive || (in.command.upTimeMs < logic.lastCommandUpTimeMs);
        if (resync) {
            logic.lastClearSeq  = in.command.clogClearSeq;
            logic.lastUnclogSeq = in.command.unclogSeq;
        } else {
            // Compare with !=, never >: uint8_t counters wrap 255 -> 0.
            clearNow  = (in.command.clogClearSeq != logic.lastClearSeq);
            unclogNow = (in.command.unclogSeq != logic.lastUnclogSeq);
            logic.lastClearSeq  = in.command.clogClearSeq;
            logic.lastUnclogSeq = in.command.unclogSeq;
        }
        logic.lastCommandUpTimeMs = in.command.upTimeMs;
        if (in.command.calibrationRun == 0) logic.calibrationArmed = true;
    }
    logic.tractorWasAlive = in.tractorAlive && in.haveCommand;

    // 3. The mode table. next starts as "motor off" (duty 0, forward); the
    //    modes that run the motor overwrite it. A mode change resets the
    //    controller afterwards.
    DispenserMode    newMode  = logic.mode;
    DispenserFault   newFault = DispenserFault::None;
    DispenserOutputs next     = {0, true};

    switch (logic.mode) {
        case DispenserMode::Normal: {
            // Start calibration? Only on a request that was armed first (a
            // received calibrationRun == 0), so a run can never restart by
            // itself. Motor off this step either way.
            bool calibStart = in.tractorAlive && in.haveCommand &&
                              in.command.calibrationRun != 0 && logic.calibrationArmed;
            if (calibStart) {
                logic.calibrationArmed = false;
                if (in.seederAlive && in.haveTelemetry && in.telemetry.wheelTurning != 0) {
                    newMode = DispenserMode::Refused;
                } else {
                    newMode = DispenserMode::Calibrating;
                    logic.calibrationStartEdges = in.encoderEdges;
                    logic.progress = 0;
                }
                break;
            }

            // Without ground speed the dispenser cannot meter at all, so
            // running would apply an arbitrary unknown rate.
            if (!in.seederAlive || !in.haveTelemetry) {
                newFault = DispenserFault::NoSpeedData;
                resetController(logic);
                break;
            }

            // Tractor liveness is deliberately not required here: the last
            // dose is held on purpose, and the speed interlock above still
            // stops the motor whenever the machine isn't moving.
            bool     enabled = in.haveCommand && in.command.dispenserEnabled != 0;
            uint16_t dose    = enabled ? in.command.doseKgPerHa : 0;
            uint32_t calib   = in.haveCommand ? in.command.gramsPer100Rev : 0;

            // Not moving: headland turn, lifted, or standing still. Normal,
            // not a fault.
            if (in.telemetry.wheelTurning == 0 || in.telemetry.groundSpeedMmS == 0 || dose == 0) {
                resetController(logic);
                break;
            }

            uint32_t required = requiredShaftRPM(in.telemetry.groundSpeedMmS, dose, calib);
            if (required == 0) {
                resetController(logic);
                break;
            }

            bool overSpeed = (required > MOTOR_MAX_RPM);
            if (overSpeed) required = MOTOR_MAX_RPM;   // clamp and complain, don't hide it
            logic.targetRPM = (uint16_t)required;

            next.motorPermille = computeDuty(logic, logic.targetRPM, elapsed);
            next.motorForward  = true;

            if (clogDetected(logic, in.nowMs, logic.targetRPM, next.motorPermille)) {
                // Shaft blocked: motor off, controller reset, and wait for
                // the operator. The reset and progress happen via the mode
                // change below; the output is overridden here.
                newMode           = DispenserMode::Clogged;
                newFault          = DispenserFault::None;
                logic.progress    = 0;
                next.motorPermille = 0;
                next.motorForward  = true;
            } else {
                newFault = overSpeed ? DispenserFault::OverSpeed : DispenserFault::None;
            }
            break;
        }

        case DispenserMode::Calibrating: {
            // The tractor is what commands this run, so losing it aborts.
            if (!in.tractorAlive || !in.haveCommand || in.command.calibrationRun == 0) {
                newMode = DispenserMode::Normal;
                break;
            }
            // Never spin the auger while the machine is actually moving -
            // whether that is already true at the start or becomes true
            // part-way through.
            if (in.seederAlive && in.haveTelemetry && in.telemetry.wheelTurning != 0) {
                newMode = DispenserMode::Refused;
                break;
            }
            uint32_t turned = in.encoderEdges - logic.calibrationStartEdges;
            if (turned >= CALIBRATION_TOTAL_EDGES) {
                newMode = DispenserMode::CalibrationDone;
                logic.progress = 100;
                break;
            }
            logic.progress  = (uint8_t)(((uint64_t)turned * 100ULL) / CALIBRATION_TOTAL_EDGES);
            logic.targetRPM = CALIBRATION_RPM;
            next.motorPermille = computeDuty(logic, logic.targetRPM, elapsed);
            next.motorForward  = true;
            if (clogDetected(logic, in.nowMs, logic.targetRPM, next.motorPermille)) {
                newMode           = DispenserMode::Clogged;
                logic.progress    = 0;
                next.motorPermille = 0;
                next.motorForward  = true;
            }
            // Fault None throughout.
            break;
        }

        case DispenserMode::CalibrationDone:
        case DispenserMode::Refused: {
            // Hold with the motor off until the tractor withdraws the
            // request, so the operator sees the result. Nothing else leaves
            // these; the motor is off, so holding is safe.
            if (in.tractorAlive && in.haveCommand && in.command.calibrationRun == 0) {
                newMode = DispenserMode::Normal;
            }
            break;
        }

        case DispenserMode::Clogged: {
            // The motor stays off until the operator decides. Calibration
            // requests are ignored.
            if (clearNow) {
                newMode = DispenserMode::Normal;
            } else if (unclogNow) {
                newMode = DispenserMode::Unclogging;
                logic.unclogStartMs = in.nowMs;
                logic.progress = 0;
                // The first phase (reverse) goes out in this same step.
                next.motorPermille = UNCLOG_PERMILLE;
                next.motorForward  = false;
            }
            break;
        }

        case DispenserMode::Unclogging: {
            if (clearNow) {
                newMode = DispenserMode::Normal;
                break;
            }
            // Losing the tractor mid-sequence is not "operator decided" -
            // back to Clogged with the motor off, rather than holding the
            // last phase forever.
            if (!in.tractorAlive) {
                newMode = DispenserMode::Clogged;
                logic.progress = 0;
                break;
            }
            uint32_t t = in.nowMs - logic.unclogStartMs;
            if (t >= UNCLOG_TOTAL_MS) {
                newMode = DispenserMode::Clogged;
                logic.progress = 0;
                break;
            }
            uint32_t p = t % UNCLOG_CYCLE_MS;
            if (p < UNCLOG_REVERSE_MS) {
                next.motorPermille = UNCLOG_PERMILLE;
                next.motorForward  = false;
            } else if (p < UNCLOG_REVERSE_MS + UNCLOG_PAUSE_MS) {
                // pause: duty 0, forward
            } else if (p < UNCLOG_REVERSE_MS + UNCLOG_PAUSE_MS + UNCLOG_FORWARD_MS) {
                next.motorPermille = UNCLOG_PERMILLE;
                next.motorForward  = true;
            } else {
                // pause: duty 0, forward
            }
            logic.progress = (uint8_t)((uint64_t)t * 100ULL / UNCLOG_TOTAL_MS);
            // No clog check here: the sequence runs to completion whatever
            // the shaft does, and the encoder counts up in both directions.
            break;
        }
    }

    // Every mode change resets the controller (integral 0, target 0, clog
    // timer stopped). progressPercent is 0 in every mode except Calibrating
    // and Unclogging (set above) and CalibrationDone (100, set above).
    if (newMode != logic.mode) {
        logic.mode = newMode;
        resetController(logic);
        if (newMode == DispenserMode::Normal || newMode == DispenserMode::Refused ||
            newMode == DispenserMode::Clogged) {
            logic.progress = 0;
        }
    }
    logic.fault = newFault;

    out = next;

    // 4. Reversal guard: never switch the motor from turning one way straight
    //    into turning the other. The unclog sequence already pauses between
    //    moves; the guard makes a reversal at speed impossible whatever the
    //    timing.
    if (logic.prevOutput.motorPermille > 0 && out.motorPermille > 0 &&
        logic.prevOutput.motorForward != out.motorForward) {
        out.motorPermille = 0;
        out.motorForward  = logic.prevOutput.motorForward;
    }

    // 5. This step's output is what the status reports and what the guard
    //    compares the next step against.
    logic.prevOutput = out;
    return true;
}

// Everything but the header.
inline void dispenserFillStatus(const DispenserLogic &logic, DispenserStatus &status)
{
    status.targetShaftRPM         = logic.targetRPM;
    status.measuredShaftRPM       = logic.measuredRPM;
    status.motorCommandedPermille = logic.prevOutput.motorPermille;
    status.motorRunning           = (logic.prevOutput.motorPermille > 0) ? 1 : 0;
    status.faultCode              = logic.fault;
    status.mode                   = logic.mode;
    status.progressPercent        = logic.progress;
}
