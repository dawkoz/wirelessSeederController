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

// The continuous calibration run's length. The burst-mode run has its own,
// set by the dose and the calibration (calibrationBurstEdges).
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

// The push backwards after a burst (BURST_BACKLASH_MS): waiting out
// BURST_BACKLASH_DELAY_MS for the shaft to stop, then pushing.
enum class BacklashPush : uint8_t { None, Due, Pushing };

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

    // Distance ledger (see ledgerCatchupRPM). owedRevolutions is the debt in
    // output-shaft revolutions; the two snapshots are what it is measured
    // against, and ledgerValid is false until a metering step has taken them.
    float    owedRevolutions    = 0.0f;
    uint32_t ledgerPulses       = 0;
    uint32_t ledgerEdges        = 0;
    uint32_t ledgerSincePulseMs = 0;
    bool     ledgerValid        = false;

    // Calibration run bookkeeping. calibrationArmed is what stops a run from
    // starting by itself after a reboot, a link gap, a clog or a cancel: it
    // is only set by a received command with calibrationRun == 0, and cleared
    // when a run actually starts (or is refused). calibrationStartMs is when
    // the burst-mode run's replayed pass began (calibrationBurst).
    bool     calibrationArmed = false;
    uint32_t calibrationStartEdges = 0;
    uint32_t calibrationStartMs    = 0;

    // Tractor command counters (see TractorCommand). lastCommandUpTimeMs is
    // how a tractor reboot is told apart from a fresh counter value.
    uint8_t  lastClearSeq        = 0;
    uint8_t  lastUnclogSeq       = 0;
    uint32_t lastCommandUpTimeMs = 0;
    bool     tractorWasAlive     = false;

    // Unclogging sequence start time.
    uint32_t unclogStartMs = 0;

    // Burst metering (DISPENSER_BURST_MODE, see burstMeter). burstMode is the
    // switch as dispenserInit found it - a field, so the bench can drive both
    // kinds of metering from one build. A running burst ends when the encoder
    // reaches burstStopEdges; dispenserBurstStop() checks that between control
    // steps, so a burst stops on its count rather than up to a step later. The
    // same pair drives the calibration run's full-duty turns. burstLatePulses
    // counts pulses in a row that found the motor still busy with the earlier
    // ones (ZA SZYBKO). averageRPM is the shaft RPM the status reports in burst
    // mode, where the shaft alternates between full speed and standing still.
    // burstPermille is BURST_PERMILLE as dispenserInit found it, a field for
    // the same reason as burstMode.
    bool     burstMode       = DISPENSER_BURST_MODE;
    uint16_t burstPermille   = BURST_PERMILLE;
    bool     burstRunning    = false;
    uint32_t burstStopEdges  = 0;
    uint8_t  burstLatePulses = 0;
    float    averageRPM      = 0.0f;

    // Failed bursts (BURST_FAIL_MS): burstFailStreak counts them in a row - a
    // burst that reaches its count ends the run - and burstFailures all of
    // them since boot, for the status, wrapping. After a failure burstWaitPulse
    // holds the next attempt back until a new wheel pulse.
    uint8_t  burstFailStreak = 0;
    uint8_t  burstFailures   = 0;
    bool     burstWaitPulse  = false;

    // The push backwards after every burst: which phase it is in, and when
    // that phase ends. Timed on every call of dispenserBacklashTick(), not only
    // every control step - it is shorter than a step.
    BacklashPush backlash      = BacklashPush::None;
    uint32_t     backlashAtMs  = 0;

    // The seeding simulation (Simulating). simulationArmed works like
    // calibrationArmed. The counts are what the status reports, kept after a
    // run until the next one starts; simUnclogging marks the unclog sequence
    // running inside it, from unclogStartMs.
    bool     simulationArmed = false;
    uint32_t simStartMs      = 0;
    uint16_t simBursts       = 0;
    uint16_t simFailures     = 0;
    uint8_t  simUnclogs      = 0;
    bool     simUnclogging   = false;

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
    // The ledger dies with the controller. A debt must never survive a stop, a
    // clog or a mode change and then be paid off in one spot - the fertilizer
    // it stands for belongs to ground the machine has already left.
    logic.owedRevolutions  = 0.0f;
    logic.ledgerValid      = false;
    // So does a burst: it was counting against that ledger. A run of failed
    // bursts does not: it ends only when a burst gets through, or in the
    // unclog sequence it leads to.
    logic.burstRunning     = false;
    logic.burstLatePulses  = 0;
    logic.burstWaitPulse   = false;
    // And the push backwards after one: the motor stops where it is.
    logic.backlash         = BacklashPush::None;
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
    logic.calibrationStartMs    = 0;
    logic.lastClearSeq      = 0;
    logic.lastUnclogSeq     = 0;
    logic.lastCommandUpTimeMs = 0;
    logic.tractorWasAlive   = false;
    logic.unclogStartMs     = 0;
    logic.ledgerPulses      = 0;
    logic.ledgerEdges       = 0;
    logic.ledgerSincePulseMs = 0;
    logic.burstMode         = DISPENSER_BURST_MODE;
    logic.burstPermille     = BURST_PERMILLE;
    logic.burstStopEdges    = 0;
    logic.averageRPM        = 0.0f;
    logic.burstFailStreak   = 0;
    logic.burstFailures     = 0;
    logic.simulationArmed   = false;
    logic.simStartMs        = 0;
    logic.simBursts         = 0;
    logic.simFailures       = 0;
    logic.simUnclogs        = 0;
    logic.simUnclogging     = false;
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

// Revolutions of the output shaft per metre travelled - the same maths as
// requiredShaftRPM() one step earlier, and the two have to agree:
// revolutionsPerMetre * speed [m/s] * 60 == requiredShaftRPM(). At 4 m and
// 40 kg/ha with 500 g per 100 rev that is 3.2 rev/m, and 3.2 * 60 * 1 m/s is
// the same 192 RPM.
inline float revolutionsPerMetre(uint16_t doseKgPerHa, uint32_t gramsPer100Rev)
{
    if (doseKgPerHa == 0 || gramsPer100Rev == 0) return 0.0f;
    return (float)WORKING_WIDTH_CM * (float)doseKgPerHa / (10.0f * (float)gramsPer100Rev);
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

// The distance ledger: metering follows the ground rather than the speed
// estimate. Every wheel pulse is a fixed distance, a fixed distance is a fixed
// number of shaft turns, and the encoder says how many turns were really made.
// The difference is a debt in revolutions, and this returns the RPM to add to
// the rate so that it is paid off over LEDGER_CATCHUP_SECONDS.
//
// Why it is worth the state: one wheel pulse is about 0.8 m and the seeder
// averages its speed over a whole metering turn (~4.7 m), so the rate is right
// only on average and lags every change in speed - and the motor needs a few
// seconds to settle after each start. The ledger turns what that costs from
// fertilizer never applied into fertilizer applied a few metres later.
//
// Three cases add no debt and only take fresh snapshots:
//   - the first metering step, which has nothing to compare against;
//   - the seeder's cumulative counter going backwards, i.e. it rebooted (it
//     cannot wrap: about 1.3 pulses per metre against 2^32);
//   - over-speed, because a debt earned while the motor is already at its limit
//     cannot be repaid without double-dosing the strip that follows. ZA SZYBKO
//     is what the operator acts on there, not the ledger.
//
// Starting to count again credits the next whole pulse, part of which is ground
// covered before the ledger was looking: up to one pulse of debt that is not
// owed - about 12 g of fertilizer at the usual settings, always in the
// direction of applying slightly more. Small enough to leave alone; correcting
// it needs another piece of state and trades the bias for an under-application
// at the start of every pass instead.
inline float ledgerCatchupRPM(DispenserLogic &logic, const DispenserInputs &in,
                              uint32_t elapsed, float revsPerMetre, bool overSpeed)
{
    uint16_t mmPerPulse = validWheelMmPerPulse(in.command.wheelMmPerPulse);
    uint32_t pulses     = in.telemetry.wheelPulses;
    uint32_t edges      = in.encoderEdges;

    if (!logic.ledgerValid || pulses < logic.ledgerPulses || overSpeed) {
        logic.ledgerPulses       = pulses;
        logic.ledgerEdges        = edges;
        logic.ledgerSincePulseMs = 0;
        logic.ledgerValid        = true;
        return 0.0f;
    }

    uint32_t deltaPulses = pulses - logic.ledgerPulses;
    uint32_t deltaEdges  = edges  - logic.ledgerEdges;
    logic.ledgerPulses = pulses;
    logic.ledgerEdges  = edges;

    logic.owedRevolutions += ((float)deltaPulses * (float)mmPerPulse / 1000.0f) * revsPerMetre;
    logic.owedRevolutions -= (float)deltaEdges / (float)ENCODER_EDGES_PER_REV;

    // Cap the debt at a fixed distance of fertilizer: a spell at clamped duty
    // must never end as a heap on the ground.
    float cap = (float)LEDGER_MAX_METRES * revsPerMetre;
    if (logic.owedRevolutions >  cap) logic.owedRevolutions =  cap;
    if (logic.owedRevolutions < -cap) logic.owedRevolutions = -cap;

    // Where the ground has got to between two pulses, from the speed the seeder
    // reports. Without it the debt would step by a whole pulse - about 2.5
    // revolutions at the usual settings - and the target would saw up and down
    // at the pulse rate. Capped at one pulse's distance, so a stale speed can
    // never run away with it.
    //
    // A pulse is noticed at the end of the step it landed in, so the middle of
    // that step is the unbiased guess for when it really arrived. Restarting
    // from 0 would credit up to a step's worth of ground twice, which at the
    // usual settings holds the rate about 1 % high for as long as it meters.
    if (deltaPulses > 0) logic.ledgerSincePulseMs  = elapsed / 2;
    else                 logic.ledgerSincePulseMs += elapsed;

    float interpMm = (float)in.telemetry.groundSpeedMmS * (float)logic.ledgerSincePulseMs / 1000.0f;
    if (interpMm > (float)mmPerPulse) interpMm = (float)mmPerPulse;

    float effective = logic.owedRevolutions + (interpMm / 1000.0f) * revsPerMetre;
    float catchRPM  = effective * 60.0f / LEDGER_CATCHUP_SECONDS;

    if (catchRPM >  (float)LEDGER_MAX_CATCHUP_RPM) catchRPM =  (float)LEDGER_MAX_CATCHUP_RPM;
    if (catchRPM < -(float)LEDGER_MAX_CATCHUP_RPM) catchRPM = -(float)LEDGER_MAX_CATCHUP_RPM;
    return catchRPM;
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
//
// The timer itself: `slow` held for limitMs, one step at speed stops it.
// Shared by the continuous rule below and the burst rule after it.
inline bool clogTimerExpired(DispenserLogic &logic, uint32_t nowMs, bool slow, uint32_t limitMs)
{
    if (slow) {
        if (!logic.clogTimerRunning) {
            logic.clogTimerRunning = true;
            logic.clogTimerStartMs = nowMs;
        }
        return (nowMs - logic.clogTimerStartMs) >= limitMs;
    }
    logic.clogTimerRunning = false;
    return false;
}

inline bool clogDetected(DispenserLogic &logic, uint32_t nowMs, uint16_t targetRPM, uint16_t duty)
{
    if (duty == 0) {
        logic.clogTimerRunning = false;
        return false;
    }

    bool slow = (uint32_t)logic.measuredRPM * 100 <
                (uint32_t)targetRPM * CLOG_MIN_SPEED_PERCENT;

    return clogTimerExpired(logic, nowMs, slow, CLOG_DETECT_MS);
}

// What the burst PWM turns a free motor at: MOTOR_MAX_RPM at full PWM, and in
// proportion below it. Not a speed anything holds - there is no speed control
// in a burst - only the scale for the clog line and the target the status
// reports.
inline uint16_t burstNominalRPM(const DispenserLogic &logic)
{
    return (uint16_t)((uint32_t)logic.burstPermille * MOTOR_MAX_RPM / 1000);
}

// The stall check for burst metering and for the calibration run in burst mode.
// A burst's line is BURST_CLOG_MIN_RPM, or a third (CLOG_MIN_SPEED_PERCENT) of
// burstNominalRPM if that is lower: a gentler burst must not be called stalled
// for turning at the speed its PWM gives. The shaft has stalled when it stays
// under the line for limitMs while driven - BURST_FAIL_MS in work, where it
// fails the burst, CLOG_DETECT_MS in the calibration run, where it is a clog.
// The first step of every burst measures the standstill before it and starts
// the timer; the next one, at speed, stops it again. A step with the motor off
// stops it as above.
inline uint16_t burstClogLine(const DispenserLogic &logic)
{
    uint32_t third = (uint32_t)burstNominalRPM(logic) * CLOG_MIN_SPEED_PERCENT / 100;
    return (third < BURST_CLOG_MIN_RPM) ? (uint16_t)third : BURST_CLOG_MIN_RPM;
}

inline bool burstStalled(DispenserLogic &logic, uint32_t nowMs, uint16_t duty, uint32_t limitMs)
{
    if (duty == 0) {
        logic.clogTimerRunning = false;
        return false;
    }
    return clogTimerExpired(logic, nowMs, logic.measuredRPM < burstClogLine(logic), limitMs);
}

// The duty for one step of a burst - of burst metering or of the calibration
// run: burstPermille, BURST_PWM_FRACTION of full PWM. No speed control: the
// encoder counts the angle, so how fast it is turned changes nothing in the
// dose. One exception, because a gentler burst also has less torque: a shaft
// that has slowed under the clog line after a whole step of being driven gets
// full PWM at once - everything the motor has, to push through whatever holds
// it - and the burst's own PWM again as soon as it turns. Only if even full PWM
// does not move it does the burst fail (BURST_FAIL_MS; CLOG_DETECT_MS and a
// clog in the calibration run). The first step of a burst,
// after a step with the motor off (between bursts, or cut by
// dispenserBurstStop()), is left alone: what it measured is the standstill
// before the burst. At full PWM the exception changes nothing.
inline uint16_t burstDuty(const DispenserLogic &logic)
{
    bool firstStep = (logic.prevOutput.motorPermille == 0);
    if (!firstStep && logic.measuredRPM < burstClogLine(logic)) return 1000;
    return logic.burstPermille;
}

// The angle one wheel pulse's burst turns, in output-shaft revolutions: the
// tractor's angle factor, in thousandths of what the motor turns at full burst
// PWM - taken as BURST_ANGLE_REFERENCE_RPM at this logic's burstPermille - in
// the time between two pulses at BURST_ANGLE_REFERENCE_SPEED_MM_S. The factor
// is calibrated by weighing, so the reference only makes it scale the bursts.
// The distance per pulse is in both the reference and the ground a pulse
// stands for, so one factor holds for either seed size. At 1571 mm, full PWM
// and a factor of 500: 1.41 turns. Every wire value is taken, even past the
// tractor's 999.
inline float burstRevsPerPulse(const DispenserLogic &logic, const DispenserInputs &in)
{
    float mm       = (float)validWheelMmPerPulse(in.command.wheelMmPerPulse);
    float refRevs  = (float)BURST_ANGLE_REFERENCE_RPM * ((float)logic.burstPermille / 1000.0f) / 60.0f *
                     mm / (float)BURST_ANGLE_REFERENCE_SPEED_MM_S;
    return (float)in.command.burstAngleFactor / 1000.0f * refRevs;
}

// A burst has just ended - reached its count, or failed: the push backwards
// (BURST_BACKLASH_MS) is due once the shaft has had BURST_BACKLASH_DELAY_MS to
// stop. Nothing if it is switched off.
inline void backlashAfterBurst(DispenserLogic &logic, uint32_t nowMs)
{
    if (BURST_BACKLASH_MS == 0 || BURST_BACKLASH_PERMILLE == 0) return;
    logic.backlash     = BacklashPush::Due;
    logic.backlashAtMs = nowMs + BURST_BACKLASH_DELAY_MS;
}

// The push backwards on its own clock, called on every loop() through
// dispenserControlTick() - it is shorter than a control step - and by every step
// as well, so it runs even where nothing calls it between steps. Starts the push
// when due and ends it BURST_BACKLASH_MS later. Returns true when it has just
// changed the motor, with out set to what to apply.
inline bool dispenserBacklashTick(DispenserLogic &logic, uint32_t nowMs, DispenserOutputs &out)
{
    if (logic.backlash == BacklashPush::None) return false;
    if ((int32_t)(nowMs - logic.backlashAtMs) < 0) return false;

    if (logic.backlash == BacklashPush::Due) {
        logic.backlash     = BacklashPush::Pushing;
        logic.backlashAtMs = nowMs + BURST_BACKLASH_MS;
        out.motorPermille  = BURST_BACKLASH_PERMILLE;
        out.motorForward   = false;
    } else {
        logic.backlash     = BacklashPush::None;
        out.motorPermille  = 0;
        out.motorForward   = true;
    }
    logic.prevOutput = out;
    return true;
}

// Burst metering, for a motor that cannot turn the loaded auger slowly: every
// wheel pulse is dosed as one burst at the burst PWM (burstDuty), which ends
// when the encoder has counted that pulse's angle (burstRevsPerPulse). A pulse
// is a fixed distance, so a fixed angle per pulse is a fixed dose per metre -
// delivered in one go per pulse, just after the ground it belongs to. The PWM
// only sets how long each burst takes.
//
// It is the distance ledger with the rate taken out: owedRevolutions is the
// turns owed to the ground, each pulse adds its angle, the encoder takes off
// what was turned, and the motor runs flat out while anything worth a burst is
// owed. The overshoot of a burst - the motor braking to a stop past its count -
// is owed back by the next one, so it never adds up over a pass.
//
// No speed interlock: no pulse, no burst. The seeder needs two pulses after a
// stop before its speed says "moving", and waiting for that would lose the
// first pulse's ground at every start; the pulses themselves prove the ground
// moved. After the machine stops, only what the last pulses still owe is turned
// - ground already covered - and then nothing until the next pulse.
//
// Two cases add nothing and only take fresh snapshots, as in the ledger: the
// first step, and the seeder's counter going backwards (it rebooted). And what
// is owed never survives a stop for want of seeder or dose, a clog or a mode
// change (resetController), so it is never paid off in one spot afterwards.
//
// After a failed burst (BURST_FAIL_MS, handled in dispenserStep) nothing more
// is tried until a new pulse arrives - the pause is what lets a wedged granule
// settle, as it did when the operator pressed Anuluj - and a burst that reaches
// its count ends the run of failures.
//
// Returns the step's fault: OverSpeed once BURST_LATE_PULSES pulses in a row
// have found the motor still more than BURST_LATE_FRACTION of a pulse behind on
// the earlier ones.
inline DispenserFault burstMeter(DispenserLogic &logic, const DispenserInputs &in,
                                 float revsPerPulse, DispenserOutputs &next)
{
    uint32_t pulses       = in.telemetry.wheelPulses;
    uint32_t edges        = in.encoderEdges;

    if (!logic.ledgerValid || pulses < logic.ledgerPulses) {
        logic.ledgerPulses    = pulses;
        logic.ledgerEdges     = edges;
        logic.owedRevolutions = 0.0f;
        logic.ledgerValid     = true;
        logic.burstRunning    = false;
        logic.burstLatePulses = 0;
        logic.targetRPM       = 0;
        return DispenserFault::None;
    }

    uint32_t deltaPulses = pulses - logic.ledgerPulses;
    uint32_t deltaEdges  = edges  - logic.ledgerEdges;
    logic.ledgerPulses = pulses;
    logic.ledgerEdges  = edges;

    // What the earlier pulses still need, as of now.
    logic.owedRevolutions -= (float)deltaEdges / (float)ENCODER_EDGES_PER_REV;

    if (deltaPulses > 0) {
        // Not while the shaft is stalled under the stall line (the timer runs
        // from the step that measured it): a pulse finding the motor behind then
        // is behind because the auger is stuck, which is a failed burst's
        // business, not ZA SZYBKO's. A motor too slow for the machine turns.
        if (logic.clogTimerRunning) {
            // leave the count as it is
        } else if (logic.owedRevolutions > BURST_LATE_FRACTION * revsPerPulse) {
            if (logic.burstLatePulses < 255) logic.burstLatePulses++;
        } else {
            logic.burstLatePulses = 0;
        }
        logic.owedRevolutions += (float)deltaPulses * revsPerPulse;
        logic.burstWaitPulse   = false;      // after a failed burst: the next attempt
    }

    // Behind by more than BURST_MAX_BACKLOG_PULSES: the motor cannot keep up,
    // and the rest is dropped rather than dumped once the machine slows down.
    // Never less than a minimum burst and a pulse, though, or a dose too small
    // for a burst per pulse could never add up to one. Ahead by more than a
    // pulse - only an encoder counting edges that never happened gets there -
    // and the next pulses are not starved for it; by at least a minimum burst,
    // though, so a burst's braking is always owed back in full, however small
    // the dose.
    float minBurstRev = (float)BURST_MIN_EDGES / (float)ENCODER_EDGES_PER_REV;
    float maxOwed     = (float)BURST_MAX_BACKLOG_PULSES * revsPerPulse;
    if (maxOwed < minBurstRev + revsPerPulse) maxOwed = minBurstRev + revsPerPulse;
    float maxAhead    = (revsPerPulse > minBurstRev) ? revsPerPulse : minBurstRev;
    if (logic.owedRevolutions >  maxOwed)  logic.owedRevolutions =  maxOwed;
    if (logic.owedRevolutions < -maxAhead) logic.owedRevolutions = -maxAhead;

    // BURST_MIN_EDGES decides only whether a burst starts. One already running
    // goes on to its count, however little is left: stopping it short would
    // leave the remainder waiting for the next pulse, and after the last pulse
    // of a pass, for ever.
    float owedEdges = logic.owedRevolutions * (float)ENCODER_EDGES_PER_REV;
    bool  owing     = owedEdges >= (float)BURST_MIN_EDGES || (logic.burstRunning && owedEdges > 0.0f);
    bool  pushing   = logic.backlash != BacklashPush::None;   // the push after the last burst comes first
    if (owing && !logic.burstWaitPulse && !pushing) {
        // Rounded up, so a finished burst leaves the ledger at or below zero
        // rather than a sliver that would start another one. Never more than a
        // billion edges ahead (two million turns - only a dose far beyond what
        // the tractor sends asks for that): the count has to stay within reach
        // of the signed comparison in dispenserBurstStop(), and every step
        // extends it anyway.
        if (owedEdges > 1.0e9f) owedEdges = 1.0e9f;
        uint32_t toTurn = (uint32_t)owedEdges;
        if ((float)toTurn < owedEdges) toTurn++;

        logic.burstRunning   = true;
        logic.burstStopEdges = edges + toTurn;
        logic.targetRPM      = burstNominalRPM(logic);
        next.motorPermille   = burstDuty(logic);
        next.motorForward    = true;
    } else if (logic.burstWaitPulse || (owing && pushing)) {
        // A burst has just failed - motor off until the next pulse - or the
        // push backwards after the last one is still to finish. Nothing has
        // been caught up, so the late count stands.
        logic.burstRunning = false;
        logic.targetRPM    = 0;
    } else {
        // Caught up, so nothing is late any more - and a burst that was still
        // running has reached its count, which ends a run of failures and
        // brings its push backwards.
        if (logic.burstRunning) {
            logic.burstFailStreak = 0;
            backlashAfterBurst(logic, in.nowMs);
        }
        logic.burstRunning    = false;
        logic.burstLatePulses = 0;
        logic.targetRPM       = 0;
    }

    return (logic.burstLatePulses >= BURST_LATE_PULSES) ? DispenserFault::OverSpeed
                                                         : DispenserFault::None;
}

// Between control steps: a running burst ends the moment the encoder reaches
// its count, instead of up to MOTOR_CONTROL_INTERVAL_MS later - at full speed
// that is over half a turn. It only ever stops the motor, never starts it; the
// next step takes whatever is owed from there, including the few edges the
// shaft turns while it brakes. A burst that gets here did not fail, so it also
// ends a run of failed ones, and its push backwards is due. Returns true when it
// has just stopped the motor, with out set to what to apply.
inline bool dispenserBurstStop(DispenserLogic &logic, uint32_t nowMs, uint32_t encoderEdges, DispenserOutputs &out)
{
    if (!logic.burstRunning) return false;
    if ((int32_t)(encoderEdges - logic.burstStopEdges) < 0) return false;

    logic.burstRunning    = false;
    logic.burstFailStreak = 0;
    logic.targetRPM       = 0;
    out.motorPermille  = 0;
    out.motorForward   = true;
    logic.prevOutput   = out;
    backlashAfterBurst(logic, nowMs);
    return true;
}

// The encoder edges the first `pulses` pulses of the burst-mode calibration run
// ask for: what burstMeter() turns for that many pulses in work, at the angle
// factor and distance per pulse the tractor is sending now - 0 if the factor is
// 0. Worked out for the whole count at once, so the rounding of one pulse never
// adds up over the run; capped far past anything real, as in burstMeter(), to
// stay in range of dispenserBurstStop().
inline uint32_t calibrationBurstEdges(const DispenserLogic &logic, const DispenserInputs &in, uint32_t pulses)
{
    float edges = burstRevsPerPulse(logic, in) * (float)pulses * (float)ENCODER_EDGES_PER_REV;
    if (edges > 1.0e9f) edges = 1.0e9f;
    return (uint32_t)(edges + 0.5f);
}

// One step of the calibration run in burst mode, which fires real bursts
// without moving: CALIBRATION_PULSES pulses, one every distance-per-pulse at
// CALIBRATION_SPEED_MM_S - the first as the run starts - each asking for the
// angle one pulse asks for in work, and the motor doing with them what
// burstMeter() does with real ones: a burst at the burst PWM to each pulse's
// exact count (dispenserBurstStop()), carrying straight on when the next pulse
// is due before one is done, off in between. So what the operator weighs is
// what CALIBRATION_PULSES pulses deliver in work. Unlike burstMeter() nothing is
// ever dropped, however far behind the motor falls: every pulse is turned in
// full, or the weight would come out short. Each burst ends on the whole
// count so far, so the braking of one is taken off the next and the run ends
// on calibrationBurstEdges() for all the pulses. The pulses due are worked out
// from the clock and the turns done from the encoder, so no burst can go
// missing or run twice. Returns true when the shaft has clogged: stalled for
// CLOG_DETECT_MS. Unlike a failed burst in work, that ends the run as Clogged -
// the unclog sequence's reverse turns would count in the weighing.
inline bool calibrationBurst(DispenserLogic &logic, const DispenserInputs &in, uint32_t turned,
                             DispenserOutputs &next)
{
    uint32_t mmPerPulse = validWheelMmPerPulse(in.command.wheelMmPerPulse);
    uint32_t intervalMs = mmPerPulse * 1000 / CALIBRATION_SPEED_MM_S;
    uint32_t pulses     = 1 + (in.nowMs - logic.calibrationStartMs) / intervalMs;
    if (pulses > CALIBRATION_PULSES) pulses = CALIBRATION_PULSES;
    uint32_t due = calibrationBurstEdges(logic, in, pulses);

    // Between two pulses, or the push backwards after the last burst still to
    // finish (as in work): motor off - the push has its own output - which also
    // stops the clog timer. A burst that ends here, not in dispenserBurstStop(),
    // brings its push as well.
    if (turned >= due || logic.backlash != BacklashPush::None) {
        if (logic.burstRunning && turned >= due) backlashAfterBurst(logic, in.nowMs);
        logic.burstRunning = false;
        logic.targetRPM    = 0;
        return burstStalled(logic, in.nowMs, 0, CLOG_DETECT_MS);
    }

    logic.burstRunning   = true;
    logic.burstStopEdges = logic.calibrationStartEdges + due;
    logic.targetRPM      = burstNominalRPM(logic);
    next.motorPermille   = burstDuty(logic);
    next.motorForward    = true;
    return burstStalled(logic, in.nowMs, next.motorPermille, CLOG_DETECT_MS);
}

// One step of the reverse/forward unclog sequence, t ms into it: the phase's
// output into next. Shared by the operator's Odetkaj (Unclogging), burst
// metering's own (AutoUnclogging) and the simulation's. No stall check: the
// sequence runs to completion whatever the shaft does, and the encoder counts
// up in both directions.
inline void unclogOutput(uint32_t t, DispenserOutputs &next)
{
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
}

// A failed burst (BURST_FAIL_MS) - a granule wedged in the auger, most likely -
// and never a clog alarm. The motor stops, and what the burst still owed waits
// for the next pulse, whose burst tries again. One pulse's worth at most:
// enough to fill the gap this burst leaves, while a pulse arriving behind on it
// is not late, so the failure alone never sounds ZA SZYBKO. The push backwards
// follows it as it follows every burst - here it opens the backlash for the
// retry. Returns true when it completes a run of BURST_FAILS_BEFORE_UNCLOG: time
// for the unclog sequence instead, whose reverse starts next step - the
// reversal guard would stop this one anyway, straight from driving forward.
// Shared by work and the simulation.
inline bool failBurst(DispenserLogic &logic, float revsPerPulse, uint32_t nowMs, DispenserOutputs &next)
{
    if (logic.owedRevolutions > revsPerPulse) logic.owedRevolutions = revsPerPulse;
    logic.burstRunning     = false;
    logic.burstWaitPulse   = true;
    logic.burstLatePulses  = 0;
    logic.targetRPM        = 0;
    logic.clogTimerRunning = false;
    logic.burstFailures++;                       // wraps; the tractor blinks on every change
    if (logic.burstFailStreak < 255) logic.burstFailStreak++;
    next.motorPermille = 0;
    next.motorForward  = true;

    if (logic.burstFailStreak < BURST_FAILS_BEFORE_UNCLOG) {
        backlashAfterBurst(logic, nowMs);
        return false;
    }
    logic.burstFailStreak = 0;
    return true;
}

// One control step. Returns false (with out untouched) until
// MOTOR_CONTROL_INTERVAL_MS has passed since the previous step or since
// dispenserInit; otherwise runs exactly one step with that elapsed time.
inline bool dispenserStep(DispenserLogic &logic, const DispenserInputs &in, DispenserOutputs &out)
{
    if (in.nowMs - logic.prevStepMs < MOTOR_CONTROL_INTERVAL_MS) return false;
    uint32_t elapsed = in.nowMs - logic.prevStepMs;
    logic.prevStepMs = in.nowMs;

    // 0. The push backwards after a burst keeps its own time
    //    (dispenserBacklashTick() on every loop); moved on here too, so it never
    //    waits on a caller that does not tick it between steps.
    DispenserOutputs pushOut = logic.prevOutput;
    dispenserBacklashTick(logic, in.nowMs, pushOut);

    // 1. Measure: edges since the previous step, then the RPM over it. The
    //    64-bit maths keeps a long gap between steps from overflowing.
    uint32_t edges = in.encoderEdges - logic.encoderSnapshot;
    logic.encoderSnapshot = in.encoderEdges;
    uint64_t rpm = (uint64_t)edges * 60000ULL / ((uint64_t)elapsed * ENCODER_EDGES_PER_REV);
    logic.measuredRPM = (rpm > 65535) ? 65535 : (uint16_t)rpm;

    // The status's shaft RPM in burst mode (see averageRPM).
    float alpha = (float)elapsed / (float)BURST_RPM_AVERAGE_MS;
    if (alpha > 1.0f) alpha = 1.0f;
    logic.averageRPM += ((float)logic.measuredRPM - logic.averageRPM) * alpha;

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
        if (in.command.simulationRun == 0)  logic.simulationArmed  = true;
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
                    logic.calibrationStartMs    = in.nowMs;
                    logic.progress = 0;
                }
                break;
            }

            // Start the seeding simulation? Armed the same way, and refused the
            // same way while the machine moves - and in continuous metering,
            // which has no bursts to fail.
            bool simStart = in.tractorAlive && in.haveCommand &&
                            in.command.simulationRun != 0 && logic.simulationArmed;
            if (simStart) {
                logic.simulationArmed = false;
                if (!logic.burstMode || (in.seederAlive && in.haveTelemetry && in.telemetry.wheelTurning != 0)) {
                    newMode = DispenserMode::Refused;
                } else {
                    newMode = DispenserMode::Simulating;
                    logic.simStartMs    = in.nowMs;
                    logic.simBursts     = 0;
                    logic.simFailures   = 0;
                    logic.simUnclogs    = 0;
                    logic.simUnclogging = false;
                    logic.progress      = 0;
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

            if (logic.burstMode) {
                // Pulses, not speed: a machine standing still sends none, so
                // nothing is turned (see burstMeter). Only nothing to dose stops
                // it outright: the dispenser switched off, a dose of 0, or an
                // angle factor of 0. The dose's size is the tractor's business
                // here - it is in the angle factor - so only whether there is
                // one matters, and the grams-per-100 calibration not at all.
                if (dose == 0 || in.command.burstAngleFactor == 0) {
                    resetController(logic);
                    break;
                }
                float revsPerPulse = burstRevsPerPulse(logic, in);
                newFault = burstMeter(logic, in, revsPerPulse, next);
                if (burstStalled(logic, in.nowMs, next.motorPermille, BURST_FAIL_MS)) {
                    newFault = DispenserFault::None;
                    if (failBurst(logic, revsPerPulse, in.nowMs, next)) {
                        // Failing again and again: the unclog sequence, by
                        // itself, and back to metering after it.
                        newMode             = DispenserMode::AutoUnclogging;
                        logic.unclogStartMs = in.nowMs;
                        logic.progress      = 0;
                    }
                }
                break;
            }

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

            // The rate above is the feed-forward; the ledger corrects the total
            // applied. overSpeed is decided on the rate alone, just above, so
            // the catch-up can neither raise nor clear ZA SZYBKO.
            float   catchRPM = ledgerCatchupRPM(logic, in, elapsed,
                                                revolutionsPerMetre(dose, calib), overSpeed);
            int32_t target   = (int32_t)required +
                               (int32_t)(catchRPM + (catchRPM >= 0.0f ? 0.5f : -0.5f));

            // Repaying an over-application must never stop the auger while the
            // machine is moving: an unfertilised strip is a permanent defect
            // and nothing alarms on it, so a ledger that has gone negative -
            // through a real over-application, or through an encoder counting
            // edges that never happened - can take the rate down to half and no
            // further. Repayment then simply takes longer.
            int32_t floorRPM = (int32_t)(required / 2);
            if (target < floorRPM)               target = floorRPM;
            if (target > (int32_t)MOTOR_MAX_RPM) target = (int32_t)MOTOR_MAX_RPM;
            logic.targetRPM = (uint16_t)target;

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
            // Continuous: 100 revolutions. Burst mode: CALIBRATION_PULSES
            // pulses' worth at the angle factor being checked - done at once
            // if it is 0, since there is nothing to turn.
            uint32_t turned = in.encoderEdges - logic.calibrationStartEdges;
            uint32_t total  = logic.burstMode ? calibrationBurstEdges(logic, in, CALIBRATION_PULSES)
                                              : CALIBRATION_TOTAL_EDGES;
            if (turned >= total) {
                newMode = DispenserMode::CalibrationDone;
                logic.progress = 100;
                break;
            }
            logic.progress  = (uint8_t)(((uint64_t)turned * 100ULL) / total);
            bool clogged;
            if (logic.burstMode) {
                // Real bursts, the way the auger turns in work.
                clogged = calibrationBurst(logic, in, turned, next);
            } else {
                logic.targetRPM = CALIBRATION_RPM;
                next.motorPermille = computeDuty(logic, logic.targetRPM, elapsed);
                next.motorForward  = true;
                clogged = clogDetected(logic, in.nowMs, logic.targetRPM, next.motorPermille);
            }
            if (clogged) {
                newMode           = DispenserMode::Clogged;
                logic.progress    = 0;
                next.motorPermille = 0;
                next.motorForward  = true;
            }
            // Fault None throughout.
            break;
        }

        case DispenserMode::Simulating: {
            // The seeding simulation: burst metering exactly as in work, failed
            // bursts and their unclog included, standing still, on the pulses
            // SIMULATION_SPEED_MM_S would give. Like the calibration run, the
            // tractor asks for it and waits on it, so the request withdrawn or
            // the tractor lost ends it, and so does the machine really moving.
            if (!in.tractorAlive || !in.haveCommand || in.command.simulationRun == 0) {
                newMode = DispenserMode::Normal;
                break;
            }
            if (in.seederAlive && in.haveTelemetry && in.telemetry.wheelTurning != 0) {
                newMode = DispenserMode::Refused;
                break;
            }
            uint32_t t = in.nowMs - logic.simStartMs;
            if (t >= SIMULATION_DURATION_MS) {
                newMode = DispenserMode::SimulationDone;
                logic.progress = 100;
                break;
            }
            logic.progress = (uint8_t)((uint64_t)t * 100ULL / SIMULATION_DURATION_MS);

            // The unclog sequence a run of failures started, as AutoUnclogging
            // does in work - including metering afresh after it, the ground
            // covered meanwhile left undosed.
            if (logic.simUnclogging) {
                uint32_t u = in.nowMs - logic.unclogStartMs;
                if (u < UNCLOG_TOTAL_MS) {
                    unclogOutput(u, next);
                } else {
                    logic.simUnclogging = false;
                    resetController(logic);
                }
                break;
            }

            // The pulses the ground would have given since the start, at the
            // tractor's distance per pulse, in place of the seeder's.
            DispenserInputs sim = in;
            uint32_t mmPerPulse = validWheelMmPerPulse(in.command.wheelMmPerPulse);
            sim.telemetry.wheelPulses =
                (uint32_t)((uint64_t)t * SIMULATION_SPEED_MM_S / (1000ULL * mmPerPulse));

            float revsPerPulse = burstRevsPerPulse(logic, in);
            bool  wasRunning   = logic.burstRunning;
            newFault = burstMeter(logic, sim, revsPerPulse, next);
            if (!wasRunning && logic.burstRunning && logic.simBursts < 65535) logic.simBursts++;
            if (burstStalled(logic, in.nowMs, next.motorPermille, BURST_FAIL_MS)) {
                newFault = DispenserFault::None;
                if (logic.simFailures < 65535) logic.simFailures++;
                if (failBurst(logic, revsPerPulse, in.nowMs, next)) {
                    logic.simUnclogging = true;
                    logic.unclogStartMs = in.nowMs;
                    if (logic.simUnclogs < 255) logic.simUnclogs++;
                }
            }
            break;
        }

        case DispenserMode::CalibrationDone:
        case DispenserMode::SimulationDone:
        case DispenserMode::Refused: {
            // Hold with the motor off until the tractor withdraws the
            // request, so the operator sees the result. Nothing else leaves
            // these; the motor is off, so holding is safe. Refused can be
            // either run's, so both requests have to be down.
            if (in.tractorAlive && in.haveCommand && in.command.calibrationRun == 0 &&
                in.command.simulationRun == 0) {
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
            unclogOutput(t, next);
            logic.progress = (uint8_t)((uint64_t)t * 100ULL / UNCLOG_TOTAL_MS);
            break;
        }

        case DispenserMode::AutoUnclogging: {
            // Burst metering's own unclog, after BURST_FAILS_BEFORE_UNCLOG
            // failed bursts in a row: the operator's sequence, except that
            // nobody asked for it and nobody waits on it. It runs to the end
            // whatever the tractor does - metering does not need the tractor
            // either - and then metering simply goes on: Normal, which starts
            // afresh, leaving the ground covered meanwhile undosed. The
            // dispenser switched off, or nothing to dose, ends it at once.
            bool nothingToDose = in.haveCommand && (in.command.dispenserEnabled == 0 ||
                                                   in.command.doseKgPerHa == 0 ||
                                                   in.command.burstAngleFactor == 0);
            uint32_t t = in.nowMs - logic.unclogStartMs;
            if (nothingToDose || t >= UNCLOG_TOTAL_MS) {
                newMode = DispenserMode::Normal;
                break;
            }
            unclogOutput(t, next);
            logic.progress = (uint8_t)((uint64_t)t * 100ULL / UNCLOG_TOTAL_MS);
            break;
        }
    }

    // Every mode change resets the controller (integral 0, target 0, clog
    // timer stopped). progressPercent is 0 in every mode except Calibrating,
    // Unclogging, AutoUnclogging and Simulating (set above) and CalibrationDone
    // and SimulationDone (100, set above).
    if (newMode != logic.mode) {
        logic.mode = newMode;
        resetController(logic);
        if (newMode == DispenserMode::Normal || newMode == DispenserMode::Refused ||
            newMode == DispenserMode::Clogged) {
            logic.progress = 0;
        }
    }
    logic.fault = newFault;

    // A step that lands in the push backwards after a burst keeps it going:
    // the modes hold the motor off meanwhile, and the push has its own output.
    // A mode change has cancelled it above.
    if (logic.backlash == BacklashPush::Pushing) {
        next.motorPermille = BURST_BACKLASH_PERMILLE;
        next.motorForward  = false;
    }

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
    // In burst mode the shaft alternates between full speed and standing
    // still, so the tractor is sent the average - the rate a motor turning
    // continuously would have shown - rather than whichever it caught.
    status.measuredShaftRPM       = logic.burstMode ? (uint16_t)(logic.averageRPM + 0.5f)
                                                    : logic.measuredRPM;
    status.motorCommandedPermille = logic.prevOutput.motorPermille;
    status.motorRunning           = (logic.prevOutput.motorPermille > 0) ? 1 : 0;
    status.faultCode              = logic.fault;
    status.mode                   = logic.mode;
    status.progressPercent        = logic.progress;
    status.burstFailures          = logic.burstFailures;
    status.simBursts              = logic.simBursts;
    status.simFailures            = logic.simFailures;
    status.simUnclogs             = logic.simUnclogs;
}
