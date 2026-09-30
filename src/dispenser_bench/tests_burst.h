#pragma once

#include <math.h>

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_logic.h"
#include "../tractor/angle_factor.h"
#include "bench_report.h"
#include "bench_wheel.h"

// ---------------------------------------------------------------------------
// B tests: burst metering (DISPENSER_BURST_MODE) on its own. No hardware, a
// second or two for the lot, so they run with the D and L tests on 'd' -
// whatever DISPENSER_BURST_MODE is set to, since they set the mode themselves.
//
// Unlike the D harness, which steps the logic once per control interval, this
// one ticks every millisecond: the ground makes wheel pulses, the simulated
// seeder sends its cumulative count every SEND_INTERVAL_MS (or at random
// intervals, some lost), the shaft spins up at full duty and brakes when the
// motor is cut, dispenserBurstStop() runs on every tick exactly as
// dispenserControlTick() runs it on every loop, and dispenserStep() runs when
// it is due. What is checked is what burst metering promises: the shaft turns
// what the pulses ask for, the motor is at full duty or off and nothing in
// between, and the alarms say what they should.
//
// The defaults: the angle factor BENCH_ANGLE_FACTOR at 785 mm per pulse and
// 1000 mm/s, so a pulse asks for 2.512 turns and the ground for an average of
// 192 RPM - the D tests' 40 kg/ha at 500 g per 100 - against a simulated shaft
// that turns at 300 RPM at full duty. Bursts are at full PWM unless a test says
// otherwise, whatever BURST_PWM_FRACTION is set to.
// ---------------------------------------------------------------------------

static constexpr uint16_t B_FULL = 1000;   // full PWM, in permille

struct BHarness {
    DispenserLogic   logic;
    DispenserOutputs out = {0, true};

    uint32_t nowMs = 100000;

    // The shaft: at full duty it spins up to runRPM over spinUpMs, and at a
    // lower duty to that share of it, less droopRPM - a load slows a DC motor
    // by about the same amount at any voltage. Cut, it brakes to a stop over
    // brakeMs, and the few edges it turns then are the overshoot the next burst
    // owes back. blocked holds it still whatever the motor does.
    uint32_t edges     = 0;
    double   edgeCarry = 0.0;
    double   shaftRPM  = 0.0;
    double   runRPM    = 300.0;
    double   droopRPM  = 0.0;
    double   spinUpMs  = 40.0;
    double   brakeMs   = 8.0;
    bool     blocked   = false;
    bool     fastStop  = true;    // dispenserBurstStop() on every tick, as in production

    // The ground and the simulated seeder.
    double   speedMmS    = 1000.0;
    double   pulseCarry  = 0.0;
    uint32_t pulses      = 0;     // what the sensor has produced
    uint16_t mmPerPulse  = BENCH_WHEEL_MM_PER_PULSE;
    bool     seederOn    = true;  // sending at all
    uint32_t sendMinMs   = SEND_INTERVAL_MS;   // each interval drawn from [min, max]
    uint32_t sendMaxMs   = SEND_INTERVAL_MS;
    uint8_t  lossPercent = 0;     // packets lost on the way
    uint32_t rng         = 0x2468ACE1u;        // fixed seed: a failure repeats exactly
    uint32_t nextSendMs  = 0;
    uint32_t lastRxMs    = 0;

    // What the dispenser has received.
    bool            haveTelemetry = false;
    SeederTelemetry telemetry     = {};
    bool            tractorAlive  = true;
    bool            haveCommand   = true;
    TractorCommand  command       = {};

    // Watchers, over every control step since bFresh() or bWatch().
    uint32_t steps        = 0;
    uint32_t motorSteps   = 0;    // steps that left the motor driven
    uint32_t bursts       = 0;    // times the motor went from off to on
    bool     motorWasOn   = false;
    uint32_t pushes       = 0;    // times the motor went from off to backwards
    bool     pushWasOn    = false;
    uint32_t onMs         = 0;    // time with the motor driven...
    uint32_t onEdges      = 0;    // ...and the edges turned in it: the burst speed
    bool     oddDuty      = false; // a duty other than 0 and the burst PWM
    bool     lowDuty      = false; // a duty in the band where the motor only buzzes
    bool     sawClogged   = false;
    bool     sawOverSpeed = false;
    bool     sawNoSpeed   = false;
    bool     sawAutoUnclog = false;
    uint32_t failures     = 0;    // failed bursts (BURST_FAIL_MS)...
    uint8_t  lastFailures = 0;    // ...counted from the logic's wrapping counter
    double   worstOvershootEdges = 0.0;   // turned past what was owed, seen with the motor off
    double   worstOwedRev = 0.0;          // the most the ledger ever owed
    double   leastOwedRev = 0.0;          // the most it was ever ahead (negative)
};

// Sub-checks of B19, collected by bStep() over every step of every B test.
static bool     b19Ok    = true;
static uint32_t b19Count = 0;

static uint32_t bRand(BHarness &h, uint32_t span)
{
    h.rng = h.rng * 1664525u + 1013904223u;
    return (h.rng >> 16) % span;
}

// Everything at the defaults, burst metering on at full PWM, the seeder about
// to send and the tractor alive with its first command. Full PWM whatever
// BURST_PWM_FRACTION says: the tests that want a gentler burst set
// burstPermille themselves (B24 on).
static void bFresh(BHarness &h)
{
    h = BHarness();
    dispenserInit(h.logic, h.nowMs, h.edges);
    h.logic.burstMode     = true;
    h.logic.burstPermille = 1000;

    h.command.dispenserEnabled = 1;
    h.command.doseKgPerHa      = 40;
    h.command.gramsPer100Rev   = 500;
    h.command.calibrationRun   = 0;
    h.command.simulationRun    = 0;
    h.command.wheelMmPerPulse  = h.mmPerPulse;
    h.command.burstAngleFactor = BENCH_ANGLE_FACTOR;
    h.nextSendMs = h.nowMs;
}

// Clears the watchers, so a test can look at one part of its run.
static void bWatch(BHarness &h)
{
    h.steps = 0;
    h.motorSteps = 0;
    h.bursts = 0;
    h.pushes = 0;
    h.onMs = 0;
    h.onEdges = 0;
    h.oddDuty = false;
    h.lowDuty = false;
    h.sawClogged = false;
    h.sawOverSpeed = false;
    h.sawNoSpeed = false;
    h.sawAutoUnclog = false;
    h.failures = 0;
    h.lastFailures = h.logic.burstFailures;
    h.worstOvershootEdges = 0.0;
    h.worstOwedRev = 0.0;
    h.leastOwedRev = 0.0;
}

// Turns per pulse at the harness's current settings, worked out independently
// of the logic's own arithmetic: the angle factor, in thousandths of what
// BURST_ANGLE_REFERENCE_RPM at the harness's burst PWM turns between two pulses
// at BURST_ANGLE_REFERENCE_SPEED_MM_S.
static double bRevsPerPulse(const BHarness &h)
{
    double pulseSeconds = (double)h.mmPerPulse / (double)BURST_ANGLE_REFERENCE_SPEED_MM_S;
    double refRevs      = (double)BURST_ANGLE_REFERENCE_RPM * ((double)h.logic.burstPermille / 1000.0) / 60.0 *
                          pulseSeconds;
    return (double)h.command.burstAngleFactor / 1000.0 * refRevs;
}

static double bTurns(const BHarness &h, uint32_t sinceEdges)
{
    return (double)(h.edges - sinceEdges) / (double)ENCODER_EDGES_PER_REV;
}

// Turns delivered against turns owed, once the shaft has stopped: never short,
// and over by no more than the last burst's braking (a tenth of a turn is
// about four times what the simulated brake gives).
static bool bAddsUp(double got, double want)
{
    return got >= want - 0.005 && got <= want + 0.1;
}

// One control step's bookkeeping, and B19: the status must report what the
// step just did, with the averaged shaft RPM.
static void bStep(BHarness &h)
{
    h.steps++;
    if (h.out.motorPermille > 0 && h.out.motorForward) h.motorSteps++;
    bool push = !h.out.motorForward && h.out.motorPermille == BURST_BACKLASH_PERMILLE;   // after a burst
    if (h.out.motorPermille != 0 && h.out.motorPermille != h.logic.burstPermille && !push) h.oddDuty = true;
    if (h.out.motorPermille > 0 && h.out.motorPermille < MOTOR_MIN_RUNNING_PERMILLE) h.lowDuty = true;
    if (h.logic.mode == DispenserMode::Clogged) h.sawClogged = true;
    if (h.logic.mode == DispenserMode::AutoUnclogging) h.sawAutoUnclog = true;
    if (h.logic.fault == DispenserFault::OverSpeed)   h.sawOverSpeed = true;
    if (h.logic.fault == DispenserFault::NoSpeedData) h.sawNoSpeed = true;
    h.failures     += (uint8_t)(h.logic.burstFailures - h.lastFailures);
    h.lastFailures  = h.logic.burstFailures;

    if (h.logic.mode == DispenserMode::Normal && h.logic.ledgerValid) {
        double owed = (double)h.logic.owedRevolutions;
        if (owed > h.worstOwedRev) h.worstOwedRev = owed;
        if (owed < h.leastOwedRev) h.leastOwedRev = owed;
        if (!h.logic.burstRunning && -owed * (double)ENCODER_EDGES_PER_REV > h.worstOvershootEdges) {
            h.worstOvershootEdges = -owed * (double)ENCODER_EDGES_PER_REV;
        }
    }

    DispenserStatus status;
    dispenserFillStatus(h.logic, status);
    b19Count++;
    if (status.mode != h.logic.mode) b19Ok = false;
    if (status.faultCode != h.logic.fault) b19Ok = false;
    if (status.motorCommandedPermille != h.out.motorPermille) b19Ok = false;
    if (status.motorRunning != ((h.out.motorPermille > 0) ? 1 : 0)) b19Ok = false;
    if (status.targetShaftRPM != h.logic.targetRPM) b19Ok = false;
    if (status.progressPercent != h.logic.progress) b19Ok = false;
    if (status.burstFailures != h.logic.burstFailures) b19Ok = false;
    if (status.simBursts != h.logic.simBursts || status.simFailures != h.logic.simFailures) b19Ok = false;
    if (status.simUnclogs != h.logic.simUnclogs) b19Ok = false;
    uint16_t rpm = h.logic.burstMode ? (uint16_t)(h.logic.averageRPM + 0.5f) : h.logic.measuredRPM;
    if (status.measuredShaftRPM != rpm) b19Ok = false;
}

static void bTick(BHarness &h)
{
    h.nowMs++;

    // The ground makes pulses...
    if (h.speedMmS > 0.0) {
        h.pulseCarry += h.speedMmS / 1000.0 / (double)h.mmPerPulse;
        while (h.pulseCarry >= 1.0) {
            h.pulseCarry -= 1.0;
            h.pulses++;
        }
    }

    // ...and the seeder sends its cumulative count now and then, some lost.
    if (h.seederOn && (int32_t)(h.nowMs - h.nextSendMs) >= 0) {
        uint32_t span = h.sendMaxMs - h.sendMinMs + 1;
        h.nextSendMs = h.nowMs + h.sendMinMs + ((span > 1) ? bRand(h, span) : 0);
        if (h.lossPercent == 0 || bRand(h, 100) >= h.lossPercent) {
            h.telemetry.wheelPulses    = h.pulses;
            h.telemetry.groundSpeedMmS = (uint16_t)h.speedMmS;
            h.telemetry.wheelTurning   = (h.speedMmS > 0.0) ? 1 : 0;
            h.haveTelemetry = true;
            h.lastRxMs      = h.nowMs;
        }
    }

    // The shaft.
    double steady = 0.0;
    if (h.out.motorPermille > 0) {             // either way: the encoder counts up both ways
        steady = h.runRPM * (double)h.out.motorPermille / 1000.0 - h.droopRPM;
        if (steady < 0.0) steady = 0.0;
    }
    if (h.blocked) {
        h.shaftRPM = 0.0;
    } else if (h.shaftRPM < steady) {
        h.shaftRPM += h.runRPM / h.spinUpMs;
        if (h.shaftRPM > steady) h.shaftRPM = steady;
    } else if (h.shaftRPM > steady) {
        h.shaftRPM -= h.runRPM / h.brakeMs;
        if (h.shaftRPM < steady) h.shaftRPM = steady;
    }
    h.edgeCarry += h.shaftRPM * (double)ENCODER_EDGES_PER_REV / 60000.0;
    uint32_t whole = (uint32_t)h.edgeCarry;
    h.edgeCarry -= (double)whole;
    h.edges += whole;
    if (h.out.motorPermille > 0 && h.out.motorForward) {
        h.onMs++;
        h.onEdges += whole;
    }

    // dispenserControlTick(): the burst's own stop and the push backwards after
    // it on every call, then a step when one is due.
    if (h.fastStop) {
        dispenserBurstStop(h.logic, h.nowMs, h.edges, h.out);
        dispenserBacklashTick(h.logic, h.nowMs, h.out);
    }

    h.command.upTimeMs         = h.nowMs;
    h.command.wheelMmPerPulse  = h.mmPerPulse;

    DispenserInputs in;
    in.nowMs         = h.nowMs;
    in.encoderEdges  = h.edges;
    in.seederAlive   = h.haveTelemetry && (h.nowMs - h.lastRxMs) < LINK_TIMEOUT_MS;
    in.haveTelemetry = h.haveTelemetry;
    in.telemetry     = h.telemetry;
    in.tractorAlive  = h.tractorAlive;
    in.haveCommand   = h.haveCommand;
    in.command       = h.command;

    if (dispenserStep(h.logic, in, h.out)) bStep(h);

    bool on     = h.out.motorPermille > 0 && h.out.motorForward;
    bool pushOn = h.out.motorPermille > 0 && !h.out.motorForward;
    if (on && !h.motorWasOn) h.bursts++;
    h.motorWasOn = on;
    if (pushOn && !h.pushWasOn) h.pushes++;
    h.pushWasOn = pushOn;
}

static void bRun(BHarness &h, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) bTick(h);
}

// Runs until the dispenser has taken `count` more pulses into its ledger, or
// limitMs has passed. False on the limit.
static bool bRunPulses(BHarness &h, uint32_t count, uint32_t limitMs)
{
    uint32_t start = h.logic.ledgerPulses;
    for (uint32_t i = 0; i < limitMs; i++) {
        bTick(h);
        if (h.logic.ledgerPulses - start >= count) return true;
    }
    return false;
}

// The machine stops; runs until the shaft has been still for a second.
static void bFinish(BHarness &h)
{
    h.speedMmS = 0.0;
    uint32_t stillMs = 0;
    for (uint32_t i = 0; i < 30000 && stillMs < 1000; i++) {
        bTick(h);
        stillMs = (h.out.motorPermille == 0 && h.shaftRPM == 0.0) ? stillMs + 1 : 0;
    }
}

// Runs until the motor is off and has stopped - a moment between bursts.
// Idle means nothing is about to move either: no push backwards still to come,
// and nothing worth a burst owed - a pulse taken in during a push starts its
// burst only once the push is over - unless a failed burst waits for a pulse.
static bool bUntilIdle(BHarness &h, uint32_t limitMs)
{
    for (uint32_t i = 0; i < limitMs; i++) {
        bTick(h);
        bool nothingOwed = h.logic.burstWaitPulse ||
                           h.logic.owedRevolutions * (float)ENCODER_EDGES_PER_REV < (float)BURST_MIN_EDGES;
        if (h.out.motorPermille == 0 && h.shaftRPM == 0.0 && !h.logic.burstRunning &&
            h.logic.backlash == BacklashPush::None && nothingOwed) {
            return true;
        }
    }
    return false;
}

// The edges the model's shaft turns in one push backwards after a burst - the
// real encoder counts them as well, whichever way the shaft turns - and so the
// room the checks on counts give them, on top of a burst's braking.
static double bPushEdges(const BHarness &h)
{
    if (BURST_BACKLASH_MS == 0 || BURST_BACKLASH_PERMILLE == 0) return 0.0;
    double rpm = h.runRPM * (double)BURST_BACKLASH_PERMILLE / 1000.0 - h.droopRPM;
    if (rpm < 0.0) rpm = 0.0;
    return rpm * (double)ENCODER_EDGES_PER_REV / 60000.0 * (double)BURST_BACKLASH_MS + 2.0;
}

// Runs until a burst is under way with the shaft at full speed.
static bool bUntilBurstAtSpeed(BHarness &h, uint32_t limitMs)
{
    for (uint32_t i = 0; i < limitMs; i++) {
        bTick(h);
        if (h.out.motorPermille > 0 && h.shaftRPM >= h.runRPM) return true;
    }
    return false;
}

// The first step takes the ledger's snapshot; nothing is owed before it.
static void bStart(BHarness &h, uint32_t &edges0, uint32_t &pulses0)
{
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 20);
    edges0  = h.edges;
    pulses0 = h.logic.ledgerPulses;
}

// ---------------------------------------------------------------------------
// B01 - steady work: every pulse gets its own full-duty burst, and the turns
// add up to exactly what the pulses asked for
// ---------------------------------------------------------------------------

static void testB01()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);

    bool reached = bRunPulses(h, 100, 200000);
    bFinish(h);

    uint32_t pulses = h.logic.ledgerPulses - p0;
    double   want   = (double)pulses * bRevsPerPulse(h);
    double   got    = bTurns(h, e0);
    double   idle   = 1.0 - (double)h.motorSteps / (double)h.steps;

    bool ok = reached && bAddsUp(got, want)
           && h.bursts == pulses
           && !h.oddDuty && !h.sawClogged && h.failures == 0 && !h.sawOverSpeed && !h.sawNoSpeed
           && idle > 0.2
           && h.worstOvershootEdges <= 20.0 + bPushEdges(h)
           && h.pushes == ((BURST_BACKLASH_MS > 0 && BURST_BACKLASH_PERMILLE > 0) ? pulses : 0)
           && h.logic.mode == DispenserMode::Normal;

    // The overshoot includes the push backwards after each burst.
    reportCheck("B01", ok, "%lu pulses: %.2f turns (want %.2f +-0.1), %lu bursts and %lu pushes back, motor "
                "off %.0f %% of the steps, overshoot at most %.0f edges (limit %.0f), duty 0 or %u only",
                (unsigned long)pulses, got, want, (unsigned long)h.bursts, (unsigned long)h.pushes,
                100.0 * idle, h.worstOvershootEdges, 20.0 + bPushEdges(h), (unsigned)B_FULL);
}

// ---------------------------------------------------------------------------
// B02 - without the stop between steps a burst runs on to the next step, and
// the ledger still owes it back: the total stays right, only each burst is
// rougher. Shows what dispenserBurstStop() buys.
// ---------------------------------------------------------------------------

static void testB02()
{
    BHarness h;
    bFresh(h);
    h.fastStop = false;
    uint32_t e0, p0;
    bStart(h, e0, p0);

    bool reached = bRunPulses(h, 100, 200000);
    bFinish(h);

    uint32_t pulses = h.logic.ledgerPulses - p0;
    double   want   = (double)pulses * bRevsPerPulse(h);
    double   got    = bTurns(h, e0);

    // One step at full speed, plus the brake.
    double limit = h.runRPM * (double)MOTOR_CONTROL_INTERVAL_MS / 60000.0 + 0.1;
    bool ok = reached && fabs(got - want) <= limit && !h.sawClogged && h.failures == 0 && !h.sawOverSpeed;

    reportCheck("B02", ok, "no stop between steps: %.2f turns (want %.2f +-%.2f), overshoot up to %.0f edges",
                got, want, limit, h.worstOvershootEdges);
}

// ---------------------------------------------------------------------------
// B03 - no pulses, no motor: a machine standing still, even while the seeder's
// speed still says it moves
// ---------------------------------------------------------------------------

static void testB03()
{
    BHarness h;
    bFresh(h);
    h.speedMmS = 0.0;
    bRun(h, 10000);
    bool still = (h.motorSteps == 0) && (h.edges == 0);

    // The stale speed a stopping seeder reports for a few seconds, and no pulse.
    h.telemetry.groundSpeedMmS = 1500;
    h.telemetry.wheelTurning   = 1;
    h.seederOn = false;
    for (int i = 0; i < 800; i++) {          // under LINK_TIMEOUT_MS: the seeder is still alive
        bTick(h);
    }
    bool stillAgain = (h.motorSteps == 0) && (h.edges == 0) && !h.sawNoSpeed;

    reportCheck("B03", still && stillAgain, "10 s standing, then a stale 'moving' with no pulse: "
                "motor never on %s, %lu edges", (still && stillAgain) ? "yes" : "NO",
                (unsigned long)h.edges);
}

// ---------------------------------------------------------------------------
// B04 - the first pulse after a stop is dosed, although the seeder still says
// "stopped": its speed needs two pulses, the dose only one
// ---------------------------------------------------------------------------

static void testB04()
{
    BHarness h;
    bFresh(h);
    h.speedMmS = 0.0;
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRun(h, 2000);

    h.pulses++;                               // one pulse; speed 0, not turning
    bFinish(h);

    double want = bRevsPerPulse(h);
    double got  = bTurns(h, e0);
    bool   ok   = (h.telemetry.wheelTurning == 0) && bAddsUp(got, want) && h.bursts == 1;

    reportCheck("B04", ok, "one pulse reported with speed 0: %.2f turns in %lu burst (want %.2f)",
                got, (unsigned long)h.bursts, want);
}

// ---------------------------------------------------------------------------
// B05 - nothing to dose: seven cases, like D02, with the ground making pulses.
// The grams-per-100 calibration plays no part in burst mode - the angle factor
// takes its place - so a calibration of 0 must not stop anything (f).
// ---------------------------------------------------------------------------

struct B05Case {
    const char *id;
    const char *label;
    bool     command;
    bool     telemetry;
    uint8_t  enabled;
    uint16_t dose;
    uint16_t factor;
    uint32_t grams;
    bool     doses;         // the one case that does meter
    bool     noSpeedData;
};

static void testB05()
{
    const B05Case cases[] = {
        {"B05a", "(a) no packets",           false, false, 1, 40, BENCH_ANGLE_FACTOR, 500, false, true },
        {"B05b", "(b) command only",         true,  false, 1, 40, BENCH_ANGLE_FACTOR, 500, false, true },
        {"B05c", "(c) telemetry only",       false, true,  1, 40, BENCH_ANGLE_FACTOR, 500, false, false},
        {"B05d", "(d) enabled 0",            true,  true,  0, 40, BENCH_ANGLE_FACTOR, 500, false, false},
        {"B05e", "(e) dose 0",               true,  true,  1,  0, BENCH_ANGLE_FACTOR, 500, false, false},
        {"B05f", "(f) 0 g per 100: doses",   true,  true,  1, 40, BENCH_ANGLE_FACTOR,   0, true,  false},
        {"B05g", "(g) angle factor 0",       true,  true,  1, 40, 0,                  500, false, false},
    };

    for (uint8_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        BHarness h;
        bFresh(h);
        h.haveCommand  = cases[c].command;
        h.tractorAlive = cases[c].command;
        h.seederOn     = cases[c].telemetry;
        h.command.dispenserEnabled = cases[c].enabled;
        h.command.doseKgPerHa      = cases[c].dose;
        h.command.burstAngleFactor = cases[c].factor;
        h.command.gramsPer100Rev   = cases[c].grams;

        if (cases[c].doses) {
            bRun(h, 5000);
            bool ok = h.bursts > 0 && h.logic.fault == DispenserFault::None;
            reportCheck(cases[c].id, ok, "%s: %lu bursts in 5 s of pulses", cases[c].label,
                        (unsigned long)h.bursts);
            continue;
        }

        bool ok = true;
        for (int i = 0; i < 5000; i++) {
            bTick(h);
            if (h.out.motorPermille != 0) ok = false;
            if (h.logic.mode != DispenserMode::Normal) ok = false;
        }
        DispenserFault want = cases[c].noSpeedData ? DispenserFault::NoSpeedData : DispenserFault::None;
        if (h.logic.fault != want) ok = false;
        if (h.sawOverSpeed) ok = false;

        reportCheck(cases[c].id, ok, "%s: motor off for 5 s of pulses, fault %s", cases[c].label,
                    cases[c].noSpeedData ? "NoSpeedData" : "None");
    }
}

// ---------------------------------------------------------------------------
// B06 - over speed: the machine needs more than the motor has. ZA SZYBKO within
// a few pulses and steady while it lasts, the backlog capped, and after slowing
// down the alarm clears and no more than the cap is left to catch up
// ---------------------------------------------------------------------------

static void testB06()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);

    h.speedMmS = 3000.0;                      // asks for 576 RPM against 300
    uint32_t tFast = h.nowMs;
    uint32_t tRaised = 0;
    bool     steady  = true;
    bool     started = false;
    bool     fullOk  = true;
    for (int i = 0; i < 20000; i++) {
        bTick(h);
        if (tRaised == 0 && h.logic.fault == DispenserFault::OverSpeed) tRaised = h.nowMs;
        if (tRaised != 0 && h.logic.fault != DispenserFault::OverSpeed) steady = false;
        if (h.out.motorPermille > 0) started = true;
        if (started && h.out.motorPermille != B_FULL) fullOk = false;   // flat out from the first burst
    }
    double cap     = (double)BURST_MAX_BACKLOG_PULSES * bRevsPerPulse(h);
    bool   capOk   = h.worstOwedRev <= cap + 0.01;

    h.speedMmS = 500.0;
    uint32_t tSlow = h.nowMs;
    uint32_t tCleared = 0;
    for (int i = 0; i < 10000; i++) {
        bTick(h);
        if (tCleared == 0 && h.logic.fault == DispenserFault::None) tCleared = h.nowMs;
    }

    bool ok = (tRaised != 0) && (tRaised - tFast <= 3000) && steady && capOk && fullOk
           && (tCleared != 0) && (tCleared - tSlow <= 3000) && !h.sawClogged && h.failures == 0;

    reportCheck("B06", ok, "3000 mm/s: ZA SZYBKO after %lu ms (limit 3000) and steady %s, backlog at most "
                "%.2f turns (cap %.2f), flat out %s; 500 mm/s: cleared after %lu ms (limit 3000)",
                (unsigned long)(tRaised ? tRaised - tFast : 0), steady ? "yes" : "NO", h.worstOwedRev, cap,
                fullOk ? "yes" : "NO", (unsigned long)(tCleared ? tCleared - tSlow : 0));
}

// ---------------------------------------------------------------------------
// B07 - close to the motor's limit on a bad radio: packets at random 100-300 ms
// intervals and one in ten lost. The motor is busy most of the time, but it
// keeps up, so ZA SZYBKO must never sound - and the dose still adds up.
// ---------------------------------------------------------------------------

static void testB07()
{
    BHarness h;
    bFresh(h);
    h.sendMinMs   = 100;
    h.sendMaxMs   = 300;
    h.lossPercent = 10;
    uint32_t e0, p0;
    bStart(h, e0, p0);

    h.speedMmS = 1250.0;                      // 240 RPM of the 300: 80 % busy, spin-up included
    bool reached = bRunPulses(h, 300, 400000);
    bFinish(h);

    uint32_t pulses = h.logic.ledgerPulses - p0;
    double   want   = (double)pulses * bRevsPerPulse(h);
    double   got    = bTurns(h, e0);
    double   busy   = (double)h.motorSteps / (double)h.steps;

    bool ok = reached && !h.sawOverSpeed && !h.sawNoSpeed && bAddsUp(got, want);

    reportCheck("B07", ok, "%lu pulses at 80 %% of the motor on a lossy radio: busy %.0f %% of the steps, "
                "ZA SZYBKO never %s, %.2f turns (want %.2f +-0.1)",
                (unsigned long)pulses, 100.0 * busy, h.sawOverSpeed ? "NO" : "yes", got, want);
}

// ---------------------------------------------------------------------------
// B08 - two, then three, pulses in one packet, as after a radio gap: every one
// dosed, one burst after the other. Two raise no alarm; three at this speed
// may for a moment, since the motor really is that far behind.
// ---------------------------------------------------------------------------

// Delivers `count` pulses in the next packet and no regular one with them, then
// carries on at 1000 mm/s for ten more.
static void bGapPulses(BHarness &h, uint32_t count)
{
    bUntilIdle(h, 5000);
    h.pulses    += count;
    h.pulseCarry = 0.0;                       // the next regular pulse is a whole pulse away
    bRunPulses(h, 10, 20000);
}

static void testB08()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRunPulses(h, 10, 20000);

    bGapPulses(h, 2);
    bool twoQuiet = !h.sawOverSpeed;
    bGapPulses(h, 3);
    bFinish(h);

    uint32_t pulses = h.logic.ledgerPulses - p0;
    double   want   = (double)pulses * bRevsPerPulse(h);
    double   got    = bTurns(h, e0);

    bool ok = twoQuiet && bAddsUp(got, want) && h.worstOwedRev <= 3.0 * bRevsPerPulse(h) + 0.01;

    reportCheck("B08", ok, "2 and then 3 pulses in one packet: %.2f turns (want %.2f +-0.1), none dropped; "
                "ZA SZYBKO never for two %s",
                got, want, twoQuiet ? "yes" : "NO");
}

// ---------------------------------------------------------------------------
// B09 - the seeder goes quiet in the middle of a burst: motor off with
// NoSpeedData; when it is back, the ground covered in the gap is not dumped
// ---------------------------------------------------------------------------

static void testB09()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRunPulses(h, 5, 20000);
    bool inBurst = bUntilBurstAtSpeed(h, 5000);

    h.seederOn = false;
    uint32_t t0 = h.nowMs;
    bool stopped = false;
    for (uint32_t i = 0; i < LINK_TIMEOUT_MS + 500; i++) {
        bTick(h);
        if (h.logic.fault == DispenserFault::NoSpeedData) {
            stopped = (h.out.motorPermille == 0);
            break;
        }
    }
    uint32_t tStop = h.nowMs - t0;

    bRun(h, 3000);                            // the machine keeps moving, unheard
    uint32_t edgesGap  = h.edges;
    uint32_t pulsesGap = h.pulses;

    h.seederOn   = true;
    h.nextSendMs = h.nowMs;
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 20);  // the first step back only takes a snapshot
    bool noDump = (h.logic.owedRevolutions == 0.0f) && (h.out.motorPermille == 0);
    uint32_t pBack = h.logic.ledgerPulses;

    bRunPulses(h, 5, 20000);
    bFinish(h);

    double want = (double)(h.logic.ledgerPulses - pBack) * bRevsPerPulse(h);
    double got  = bTurns(h, edgesGap);
    bool   ok   = inBurst && stopped && (tStop <= LINK_TIMEOUT_MS + MOTOR_CONTROL_INTERVAL_MS)
               && noDump && bAddsUp(got, want) && (pBack >= pulsesGap);

    reportCheck("B09", ok, "seeder lost mid-burst: NoSpeedData with the motor off after %lu ms %s; back: "
                "nothing dumped %s, %.2f turns for the new pulses (want %.2f)",
                (unsigned long)tStop, stopped ? "yes" : "NO", noDump ? "yes" : "NO", got, want);
}

// ---------------------------------------------------------------------------
// B10 - the seeder reboots and its cumulative counter restarts: no burst for
// the jump, and dosing goes on from the new count
// ---------------------------------------------------------------------------

static void testB10()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRunPulses(h, 20, 40000);
    bUntilIdle(h, 5000);

    h.pulses = 0;                             // restarted
    uint32_t eReboot = h.edges;
    bWatch(h);
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 250);
    bool noBurst = (h.edges == eReboot);
    uint32_t pBack = h.logic.ledgerPulses;

    bRunPulses(h, 10, 20000);
    bFinish(h);

    double want = (double)(h.logic.ledgerPulses - pBack) * bRevsPerPulse(h);
    double got  = bTurns(h, eReboot);
    bool   ok   = noBurst && bAddsUp(got, want) && h.worstOwedRev <= bRevsPerPulse(h) + 0.01;

    reportCheck("B10", ok, "seeder counter restarted: no burst for it %s, then %.2f turns (want %.2f)",
                noBurst ? "yes" : "NO", got, want);
}

// ---------------------------------------------------------------------------
// B11 - a blocked auger in the middle of a burst, the machine moving on: the
// burst fails after BURST_FAIL_MS - motor off in that step, no clog, no fault,
// the failure counted for the tractor's LED - keeping at most one pulse's
// worth owed, and nothing more is tried until the next pulse. Freed by then,
// that pulse's burst makes up the failed one's portion with its own, and
// reaching its count ends the run of failures.
// ---------------------------------------------------------------------------

static void testB11()
{
    if (BURST_FAILS_BEFORE_UNCLOG < 2) {
        reportSkipped("B11", "BURST_FAILS_BEFORE_UNCLOG is 1: every failure unclogs at once (B31)");
        return;
    }
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRunPulses(h, 5, 20000);
    bool inBurst = bUntilBurstAtSpeed(h, 5000);
    bWatch(h);

    h.blocked = true;
    uint32_t t0 = h.nowMs;
    for (int i = 0; i < (int)BURST_FAIL_MS + 1000 && h.failures == 0; i++) bTick(h);
    uint32_t tFail  = h.nowMs - t0;
    bool     timeOk = h.failures == 1 && tFail >= BURST_FAIL_MS &&
                      tFail <= BURST_FAIL_MS + 2 * MOTOR_CONTROL_INTERVAL_MS;
    bool     quiet  = h.out.motorPermille == 0 && h.logic.mode == DispenserMode::Normal &&
                      h.logic.fault == DispenserFault::None && h.logic.burstFailStreak == 1;
    double   keptRev = (double)h.logic.owedRevolutions;
    bool     kept    = keptRev > 0.0 && keptRev <= bRevsPerPulse(h) + 0.001;

    // Freed at once - the granule dropped - but only the next pulse tries again.
    h.blocked = false;
    uint32_t pFail = h.logic.ledgerPulses;
    uint32_t eFail = h.edges;
    bool     waits = true;
    for (int i = 0; i < 5000 && h.logic.ledgerPulses == pFail; i++) {
        bTick(h);
        if (h.logic.ledgerPulses == pFail && h.out.motorPermille != 0 && h.out.motorForward) waits = false;
    }
    bool retried = h.logic.ledgerPulses == pFail + 1 && h.out.motorPermille > 0;

    // That burst: the kept portion and its own pulse's - and those of any pulse
    // that comes while it makes them up - then the run is over.
    bUntilIdle(h, 5000);
    double want   = keptRev + (double)(h.logic.ledgerPulses - pFail) * bRevsPerPulse(h);
    double got    = bTurns(h, eFail);
    bool   madeUp = bAddsUp(got, want) && h.logic.burstFailStreak == 0;

    bool ok = inBurst && timeOk && quiet && kept && waits && retried && madeUp &&
              !h.sawClogged && !h.sawOverSpeed && !h.sawAutoUnclog;

    reportCheck("B11", ok, "blocked mid-burst: failed after %lu ms (want %lu..%lu), quiet %s, %.2f turns kept; "
                "waits for the next pulse %s; freed: %.2f turns (want %.2f), run over %s",
                (unsigned long)tFail, (unsigned long)BURST_FAIL_MS,
                (unsigned long)(BURST_FAIL_MS + 2 * MOTOR_CONTROL_INTERVAL_MS), quiet ? "yes" : "NO", keptRev,
                waits ? "yes" : "NO", got, want, (h.logic.burstFailStreak == 0) ? "yes" : "NO");
}

// ---------------------------------------------------------------------------
// B12 - a heavy auger that still turns fails no burst; one that barely turns
// fails them all, and is never called clogged
// ---------------------------------------------------------------------------

static void testB12()
{
    BHarness h;
    bFresh(h);
    h.runRPM   = (double)BURST_CLOG_MIN_RPM + 20.0;   // heavy, but turning
    h.speedMmS = 300.0;                               // slow enough for it to keep up
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRun(h, 60000);
    bFinish(h);
    double want = (double)(h.logic.ledgerPulses - p0) * bRevsPerPulse(h);
    double got  = bTurns(h, e0);
    bool heavyOk = h.failures == 0 && !h.sawClogged && bAddsUp(got, want);

    BHarness g;
    bFresh(g);
    g.runRPM = (double)BURST_CLOG_MIN_RPM - 20.0;     // barely turning at full duty
    bRun(g, 10000);
    bool failsOk = g.failures >= 1 && !g.sawClogged;

    reportCheck("B12", heavyOk && failsOk, "%u RPM at full duty for 60 s: no burst failed %s, %.2f of %.2f "
                "turns; %u RPM: %lu failed in 10 s, never Clogged %s",
                (unsigned)(BURST_CLOG_MIN_RPM + 20), heavyOk ? "yes" : "NO", got, want,
                (unsigned)(BURST_CLOG_MIN_RPM - 20), (unsigned long)g.failures, g.sawClogged ? "NO" : "yes");
}

// What the model's shaft settles at for a duty, load included.
static double bSteadyRPM(const BHarness &h, uint16_t permille)
{
    double rpm = h.runRPM * (double)permille / 1000.0 - h.droopRPM;
    return (rpm > 0.0) ? rpm : 0.0;
}

// Into a burst at speed, with a step at speed behind it: every burst's first
// step measures the standstill before it and starts the clog timer, and a hold
// before the next step would be timed from there, not from the hold.
static bool bUntilBurstSettled(BHarness &h, uint32_t limitMs)
{
    for (uint32_t i = 0; i < limitMs; i++) {
        bTick(h);
        if (h.out.motorPermille > 0 && h.shaftRPM >= 0.9 * bSteadyRPM(h, h.out.motorPermille) &&
            !h.logic.clogTimerRunning) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// B13 - the calibration run fires CALIBRATION_PULSES real bursts, spaced as at
// CALIBRATION_SPEED_MM_S. With 2.5 m per pulse a pulse is due every 1.5 s, and
// an angle factor of 889 asks for 4.0 turns a pulse - 0.8 s at 300
// RPM - so every burst is separate: each starts within a control step of its
// pulse, each turns one pulse's angle, and the run stops on the exact count
// ---------------------------------------------------------------------------

// Arms and starts a run with no seeder, as D20 does. True once it is running.
static bool bStartCalibration(BHarness &h)
{
    h.seederOn = false;
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 10);  // calibrationRun 0 arms it
    h.command.calibrationRun = 1;
    for (int i = 0; i < 500; i++) {
        bTick(h);
        if (h.logic.mode == DispenserMode::Calibrating) return true;
    }
    return false;
}

// The calibration run's pulse interval for a distance per pulse, as the logic
// works it out.
static uint32_t bCalibIntervalMs(uint16_t mmPerPulse)
{
    return (uint32_t)mmPerPulse * 1000 / CALIBRATION_SPEED_MM_S;
}

// What the calibration run turns in all: CALIBRATION_PULSES pulses at the
// harness's angle factor, burst PWM and distance, worked out independently of
// the logic.
static double bCalibWantEdges(const BHarness &h)
{
    return (double)CALIBRATION_PULSES * bRevsPerPulse(h) * (double)ENCODER_EDGES_PER_REV;
}

// Runs a started calibration to its end. True when it ended in CalibrationDone.
static bool bCalibrateToEnd(BHarness &h, uint32_t limitMs)
{
    for (uint32_t i = 0; i < limitMs; i++) {
        bTick(h);
        if (h.logic.mode != DispenserMode::Calibrating) break;
    }
    return h.logic.mode == DispenserMode::CalibrationDone;
}

static void testB13()
{
    BHarness h;
    bFresh(h);
    h.mmPerPulse            = 2500;
    h.command.burstAngleFactor = 889;       // 4.0 turns a pulse at 2.5 m
    bool started = bStartCalibration(h);
    uint32_t interval = bCalibIntervalMs(h.mmPerPulse);
    uint32_t pulseEdges = (uint32_t)(bRevsPerPulse(h) * (double)ENCODER_EDGES_PER_REV + 0.5);   // 1920

    bool     ok = started;
    bool     sawDone = false;
    uint8_t  lastProgress = 0;
    uint32_t bursts = 0;
    bool     wasOn = false;
    bool     onTime = true;                   // each burst within a step of its pulse
    uint32_t prevStartEdges = h.edges;
    uint32_t worstShare = 0, leastShare = 0xFFFFFFFFu;
    for (int i = 0; i < 120000; i++) {
        bTick(h);
        bool on = h.out.motorPermille > 0 && h.out.motorForward;
        if (on && !wasOn) {
            uint32_t since = h.nowMs - h.logic.calibrationStartMs;
            uint32_t pulse = bursts * interval;
            if (since < pulse || since > pulse + MOTOR_CONTROL_INTERVAL_MS + 1) onTime = false;
            if (bursts > 0) {                 // the edges since the last burst began: one share
                uint32_t share = h.edges - prevStartEdges;
                if (share > worstShare) worstShare = share;
                if (share < leastShare) leastShare = share;
            }
            prevStartEdges = h.edges;
            bursts++;
        }
        wasOn = on;
        if (h.logic.mode == DispenserMode::Calibrating) {
            if (h.logic.progress < lastProgress) ok = false;
            lastProgress = h.logic.progress;
        } else if (h.logic.mode == DispenserMode::CalibrationDone) {
            sawDone = true;
            break;
        } else {
            ok = false;
            break;
        }
    }
    uint32_t took = h.nowMs - h.logic.calibrationStartMs;
    bRun(h, 1000);                            // let it brake, and hold

    uint32_t turned  = h.edges - h.logic.calibrationStartEdges;
    double   want    = bCalibWantEdges(h);
    // Each burst's share holds its braking and its push backwards; each ends
    // on the whole count so far, so the next one is that much shorter.
    uint32_t pushRoom = (uint32_t)bPushEdges(h);
    bool     edgesOk = (double)turned >= want - 1.0 && (double)turned <= want + 20.0 + (double)pushRoom;
    bool     sharesOk = (CALIBRATION_PULSES < 2) ||
                        (leastShare + 20 + pushRoom >= pulseEdges && worstShare <= pulseEdges + 20 + pushRoom);
    if (leastShare > worstShare) leastShare = worstShare = 0;   // one burst: no share between two to measure
    ok = ok && sawDone && edgesOk && onTime && sharesOk && bursts == CALIBRATION_PULSES && !h.oddDuty
            && (h.logic.progress == 100) && (h.out.motorPermille == 0)
            && (h.logic.mode == DispenserMode::CalibrationDone);

    reportCheck("B13", ok, "calibration, %u pulses %lu ms apart: %lu bursts, each within a step of its pulse %s, "
                "%lu..%lu edges each (want %lu +-20), Done after %lu ms, %lu edges (want %.0f)",
                (unsigned)CALIBRATION_PULSES, (unsigned long)interval,
                (unsigned long)bursts, onTime ? "yes" : "NO", (unsigned long)leastShare,
                (unsigned long)worstShare, (unsigned long)pulseEdges, (unsigned long)took,
                (unsigned long)turned, want);
}

// ---------------------------------------------------------------------------
// B14 - a clog during the calibration run: blocked in the middle of a burst
// ---------------------------------------------------------------------------

static void testB14()
{
    BHarness h;
    bFresh(h);
    h.mmPerPulse          = 2500;             // separate bursts, as in B13
    h.command.burstAngleFactor = 889;
    bool started = bStartCalibration(h);
    bool inBurst = bUntilBurstSettled(h, 5000);   // the first burst: there is one whatever the pulse count

    h.blocked = true;
    uint32_t t0 = h.nowMs;
    bool clogged = false;
    for (int i = 0; i < 3000; i++) {
        bTick(h);
        if (h.logic.mode == DispenserMode::Clogged) {
            clogged = (h.out.motorPermille == 0);
            break;
        }
    }
    uint32_t tClog = h.nowMs - t0;

    bool ok = started && inBurst && clogged && tClog >= CLOG_DETECT_MS
           && tClog <= CLOG_DETECT_MS + 2 * MOTOR_CONTROL_INTERVAL_MS;
    reportCheck("B14", ok, "blocked in a calibration burst: Clogged with the motor off after %lu ms (want %lu..%lu) %s",
                (unsigned long)tClog, (unsigned long)CLOG_DETECT_MS,
                (unsigned long)(CLOG_DETECT_MS + 2 * MOTOR_CONTROL_INTERVAL_MS), clogged ? "yes" : "NO");
}

// ---------------------------------------------------------------------------
// B15 - an angle so small that a pulse asks for less than BURST_MIN_EDGES: it
// waits for a few pulses and then goes, and nothing is lost
// ---------------------------------------------------------------------------

static void testB15()
{
    BHarness h;
    bFresh(h);
    h.command.burstAngleFactor = 4;           // 2.7 edges a pulse
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bool reached = bRunPulses(h, 200, 400000);
    bFinish(h);

    uint32_t pulses = h.logic.ledgerPulses - p0;
    double   want   = (double)pulses * bRevsPerPulse(h);
    double   got    = bTurns(h, e0);
    double   perPulseEdges = bRevsPerPulse(h) * (double)ENCODER_EDGES_PER_REV;

    // Up to a minimum burst can stay owed after the last pulse; nothing more.
    double minBurstRev = (double)BURST_MIN_EDGES / (double)ENCODER_EDGES_PER_REV;
    bool ok = reached && got >= want - minBurstRev - 0.005 && got <= want + 0.1
           && h.bursts < pulses / 4 && h.bursts > 0;

    reportCheck("B15", ok, "%.1f edges a pulse: %lu bursts for %lu pulses, %.3f turns (want %.3f)",
                perPulseEdges, (unsigned long)h.bursts, (unsigned long)pulses, got, want);
}

// ---------------------------------------------------------------------------
// B16 - the angle factor changes mid-pass, as the tractor changes it when the
// dose is doubled: each pulse is dosed at the factor in force when it came.
// The dose itself changes nothing here - in burst mode it only says whether
// to dose at all - which the second half checks.
// ---------------------------------------------------------------------------

static void testB16()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRunPulses(h, 30, 60000);
    bUntilIdle(h, 5000);

    double   perPulseA = bRevsPerPulse(h);
    uint32_t pMid      = h.logic.ledgerPulses;
    h.command.burstAngleFactor = 2 * BENCH_ANGLE_FACTOR;   // at 500 mm/s that asks for 192 RPM, as before
    h.speedMmS = 500.0;
    double   perPulseB = bRevsPerPulse(h);

    bRunPulses(h, 30, 120000);
    bFinish(h);

    double want = (double)(pMid - p0) * perPulseA + (double)(h.logic.ledgerPulses - pMid) * perPulseB;
    double got  = bTurns(h, e0);
    bool   ok   = bAddsUp(got, want) && !h.sawOverSpeed;

    // The dose doubled, the factor not: the same angle a pulse.
    BHarness d;
    bFresh(d);
    d.command.doseKgPerHa = 80;
    uint32_t de0, dp0;
    bStart(d, de0, dp0);
    bRunPulses(d, 20, 60000);
    bFinish(d);
    double dWant = (double)(d.logic.ledgerPulses - dp0) * bRevsPerPulse(d);
    bool   doseOk = bAddsUp(bTurns(d, de0), dWant);

    reportCheck("B16", ok && doseOk, "angle factor doubled mid-pass: %.2f turns (want %.2f +-0.1); the dose "
                "doubled with the factor unchanged: the same angle a pulse %s", got, want, doseOk ? "yes" : "NO");
}

// ---------------------------------------------------------------------------
// B17 - switched off in the middle of a burst: motor off in that step. Back
// on, the pulses that went by while it was off are not dosed
// ---------------------------------------------------------------------------

static void testB17()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRunPulses(h, 5, 20000);
    bool inBurst = bUntilBurstAtSpeed(h, 5000);

    h.command.dispenserEnabled = 0;
    bool offInStep = false;
    for (int i = 0; i < 200; i++) {
        uint32_t stepsBefore = h.steps;
        bTick(h);
        if (h.steps != stepsBefore) {
            offInStep = (h.out.motorPermille == 0) && !h.logic.burstRunning;
            break;
        }
    }
    bRun(h, 5000);                            // pulses go by, switched off
    bool stayedOff = (h.out.motorPermille == 0);

    h.command.dispenserEnabled = 1;
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 10);
    bool noDump = (h.logic.owedRevolutions == 0.0f) && (h.out.motorPermille == 0);
    uint32_t eOn = h.edges;
    uint32_t pOn = h.logic.ledgerPulses;
    bRunPulses(h, 5, 20000);
    bFinish(h);

    double want = (double)(h.logic.ledgerPulses - pOn) * bRevsPerPulse(h);
    double got  = bTurns(h, eOn);
    bool   ok   = inBurst && offInStep && stayedOff && noDump && bAddsUp(got, want);

    reportCheck("B17", ok, "switched off mid-burst: off in that step %s, stays off %s; on again: nothing "
                "dumped %s, %.2f turns (want %.2f)",
                offInStep ? "yes" : "NO", stayedOff ? "yes" : "NO", noDump ? "yes" : "NO", got, want);
}

// ---------------------------------------------------------------------------
// B18 - the tractor goes quiet: dosing goes on with the last dose
// ---------------------------------------------------------------------------

static void testB18()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRunPulses(h, 5, 20000);

    h.tractorAlive = false;                   // the inbox keeps its last command
    bWatch(h);
    bool reached = bRunPulses(h, 30, 60000);
    bFinish(h);

    double want = (double)(h.logic.ledgerPulses - p0) * bRevsPerPulse(h);
    double got  = bTurns(h, e0);
    bool   ok   = reached && bAddsUp(got, want) && h.bursts >= 30 && !h.sawOverSpeed
               && h.logic.mode == DispenserMode::Normal;

    reportCheck("B18", ok, "tractor silent: %lu bursts, %.2f turns (want %.2f)", (unsigned long)h.bursts,
                got, want);
}

// ---------------------------------------------------------------------------
// B20 - the RPM the tractor is sent: the average, close to what the ground
// asks for while working, and 0 once the machine has stood for a while
// ---------------------------------------------------------------------------

static void testB20()
{
    BHarness h;
    bFresh(h);
    bRun(h, 10000);

    double want  = 192.0;
    double worst = 0.0;
    bool   ok    = true;
    for (int i = 0; i < 20; i++) {            // sampled every 500 ms, as a tractor would see it
        bRun(h, 500);
        DispenserStatus status;
        dispenserFillStatus(h.logic, status);
        double dev = fabs((double)status.measuredShaftRPM - want);
        if (dev > worst) worst = dev;
        if (dev > 0.25 * want) ok = false;
    }

    h.speedMmS = 0.0;
    bRun(h, 20000);
    DispenserStatus stopped;
    dispenserFillStatus(h.logic, stopped);
    if (stopped.measuredShaftRPM != 0) ok = false;

    reportCheck("B20", ok, "working: the status's RPM within %.0f of %.0f (limit 25 %%); standing 20 s: %u",
                worst, want, (unsigned)stopped.measuredShaftRPM);
}

// ---------------------------------------------------------------------------
// B21 - dispenserBurstStop() only ever stops a running burst, on its count,
// across the encoder counter's wrap; and continuous metering never starts one
// ---------------------------------------------------------------------------

static void testB21()
{
    bool ok = true;

    DispenserLogic   logic;
    DispenserOutputs out = {0, true};
    dispenserInit(logic, 0, 0);
    out.motorPermille = 777;                  // must come back untouched when nothing is due
    if (dispenserBurstStop(logic, 1000, 123456, out) || out.motorPermille != 777) ok = false;

    logic.burstRunning   = true;
    logic.burstStopEdges = 0x00000010u;       // the count wraps before the burst ends
    out.motorPermille    = B_FULL;
    if (dispenserBurstStop(logic, 1000, 0xFFFFFFF0u, out)) ok = false;
    if (dispenserBurstStop(logic, 1000, 0x0000000Fu, out)) ok = false;
    if (!dispenserBurstStop(logic, 1000, 0x00000010u, out)) ok = false;
    if (out.motorPermille != 0 || !out.motorForward || logic.burstRunning) ok = false;
    if (logic.prevOutput.motorPermille != 0) ok = false;
    if (dispenserBurstStop(logic, 1000, 0x00000020u, out)) ok = false;   // once only

    // Continuous metering at the D defaults never marks a burst.
    BHarness h;
    bFresh(h);
    h.logic.burstMode = false;
    bool never = true;
    for (int i = 0; i < 5000; i++) {
        bTick(h);
        if (h.logic.burstRunning) never = false;
    }
    ok = ok && never;

    // And a real pass across the wrap.
    BHarness w;
    bFresh(w);
    w.edges = 0xFFFFFF00u;
    dispenserInit(w.logic, w.nowMs, w.edges);
    w.logic.burstMode     = true;
    w.logic.burstPermille = 1000;
    uint32_t e0, p0;
    bStart(w, e0, p0);
    bRunPulses(w, 10, 20000);
    bFinish(w);
    double want = (double)(w.logic.ledgerPulses - p0) * bRevsPerPulse(w);
    double got  = bTurns(w, e0);
    bool   wrapOk = bAddsUp(got, want);

    reportCheck("B21", ok && wrapOk, "the stop acts once, on its count, across the wrap; continuous metering "
                "never starts a burst %s; a pass across the wrap gives %.2f of %.2f turns",
                never ? "yes" : "NO", got, want);
}

// ---------------------------------------------------------------------------
// B22 - an encoder counting edges that never happened makes the ledger run
// ahead; that costs at most one pulse's burst, never a stretch of bare ground
// ---------------------------------------------------------------------------

static void testB22()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRunPulses(h, 5, 20000);
    bUntilIdle(h, 5000);

    h.edges += 5000;                          // ten turns' worth of noise
    bWatch(h);
    uint32_t pNoise = h.logic.ledgerPulses;
    bRunPulses(h, 3, 20000);
    bUntilIdle(h, 5000);

    double minBurstRev = (double)BURST_MIN_EDGES / (double)ENCODER_EDGES_PER_REV;
    double floorRev    = -((bRevsPerPulse(h) > minBurstRev) ? bRevsPerPulse(h) : minBurstRev) - 0.01;
    bool   ok = (h.leastOwedRev >= floorRev) && (h.bursts >= 1)
             && (h.logic.ledgerPulses - pNoise >= 3);

    reportCheck("B22", ok, "10 turns of phantom edges: the ledger at most %.2f turns ahead (limit %.2f), "
                "%lu burst(s) within the next 3 pulses",
                -h.leastOwedRev, -floorRev, (unsigned long)h.bursts);
}

// ---------------------------------------------------------------------------
// B23 - the far ends of what the wire can carry: an angle factor and a dose no
// tractor sends, at the longest distance per pulse - 590 turns a pulse. The
// motor must simply run flat out with ZA SZYBKO - never stop because a count
// overflowed or wrapped.
// ---------------------------------------------------------------------------

static void testB23()
{
    BHarness h;
    bFresh(h);
    h.command.burstAngleFactor = 65535;
    h.command.doseKgPerHa      = 65535;
    h.command.gramsPer100Rev   = 1;
    h.mmPerPulse               = WHEEL_MM_PER_PULSE_MAX;
    h.speedMmS                 = 2000.0;

    // A pulse every 2.5 s, and ZA SZYBKO needs two of them to find the motor a
    // whole pulse behind: the fourth pulse, at about 10 s.
    bool started = false;
    bool flatOut = true;
    for (int i = 0; i < 20000; i++) {
        bTick(h);
        if (h.out.motorPermille > 0) started = true;
        if (started && h.out.motorPermille != B_FULL) flatOut = false;
    }

    // The stop count stays ahead of the shaft, within the signed comparison's
    // reach, and the backlog within its cap.
    int32_t ahead = (int32_t)(h.logic.burstStopEdges - h.edges);
    double  cap   = (double)BURST_MAX_BACKLOG_PULSES * bRevsPerPulse(h);
    bool ok = started && flatOut && h.logic.fault == DispenserFault::OverSpeed
           && h.logic.burstRunning && ahead > 0 && ahead <= 1000000001
           && h.worstOwedRev <= cap * 1.0001;

    reportCheck("B23", ok, "angle factor 65535 at 5 m a pulse: flat out from the first pulse %s, ZA SZYBKO %s, "
                "stop count %ld edges ahead (limit 1e9)",
                flatOut ? "yes" : "NO", (h.logic.fault == DispenserFault::OverSpeed) ? "yes" : "NO",
                (long)ahead);
}

// ---------------------------------------------------------------------------
// B24-B28 - gentler bursts (BURST_PWM_FRACTION below 1): the same bursts, one
// per pulse and to the same count, only at half PWM, so each turns slower and
// lasts longer. The shaft would turn 347 RPM at full PWM (what the bench
// measured on a free shaft), so about 173 at half; 600 mm/s asks for about 115
// RPM of work, two thirds of that.
// ---------------------------------------------------------------------------

static constexpr uint16_t B_HALF = 500;

static void bHalfPwm(BHarness &h)
{
    bFresh(h);
    h.logic.burstPermille = B_HALF;
    h.runRPM   = 347.0;
    h.speedMmS = 600.0;
}

// The shaft's speed while the motor was driven - spin-up included.
static double bBurstRPM(const BHarness &h)
{
    if (h.onMs == 0) return 0.0;
    return (double)h.onEdges * 60000.0 / ((double)h.onMs * (double)ENCODER_EDGES_PER_REV);
}

// B24 - half PWM on a free shaft: every burst at exactly half PWM - a free
// shaft never needs the stall push - one per pulse, the turns exact, at about
// half the free speed.
static void testB24()
{
    BHarness h;
    bHalfPwm(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);

    bool reached = bRunPulses(h, 60, 200000);
    bFinish(h);

    uint32_t pulses = h.logic.ledgerPulses - p0;
    double   want   = (double)pulses * bRevsPerPulse(h);
    double   got    = bTurns(h, e0);
    double   rpm    = bBurstRPM(h);
    double   steady = bSteadyRPM(h, B_HALF);

    bool ok = reached && bAddsUp(got, want) && h.bursts == pulses && !h.oddDuty
           && fabs(rpm - steady) <= 0.1 * steady && !h.sawClogged && h.failures == 0 && !h.sawOverSpeed;

    reportCheck("B24", ok, "half PWM: every burst at %u permille %s, at %.0f RPM (the model's %.0f), %lu bursts "
                "for %lu pulses, %.2f turns (want %.2f)",
                (unsigned)B_HALF, h.oddDuty ? "NO" : "yes", rpm, steady, (unsigned long)h.bursts,
                (unsigned long)pulses, got, want);
}

// B25 - half PWM with a heavy auger that takes 100 RPM off at any duty: nothing
// holds the speed, so the bursts simply turn slower - 74 RPM - and last longer,
// and the dose is still exact. That is above the clog line, so no stall push
// and no alarm. 300 mm/s asks for 58 RPM of work.
static void testB25()
{
    BHarness h;
    bHalfPwm(h);
    h.droopRPM = 100.0;
    h.speedMmS = 300.0;
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bRun(h, 60000);
    double rpm = bBurstRPM(h);
    bFinish(h);

    double want   = (double)(h.logic.ledgerPulses - p0) * bRevsPerPulse(h);
    double got    = bTurns(h, e0);
    double steady = bSteadyRPM(h, B_HALF);

    bool ok = bAddsUp(got, want) && !h.oddDuty && fabs(rpm - steady) <= 0.1 * steady
           && !h.sawClogged && h.failures == 0 && !h.sawOverSpeed;

    reportCheck("B25", ok, "heavy auger at half PWM: bursts at %.0f RPM (the model's %.0f), never pushed %s, "
                "never Clogged %s, %.2f of %.2f turns",
                rpm, steady, h.oddDuty ? "NO" : "yes", h.sawClogged ? "NO" : "yes", got, want);
}

// B26 - an auger too heavy for half PWM: it takes 140 RPM off, leaving 34 RPM,
// under the stall line of 54. The stall push gives it full PWM whenever it slows
// under the line, so it keeps turning, no burst fails, and the dose is exact.
// Held for good, full PWM is tried and the burst fails after BURST_FAIL_MS.
static void testB26()
{
    BHarness h;
    bHalfPwm(h);
    h.droopRPM = 140.0;
    h.speedMmS = 300.0;
    uint32_t e0, p0;
    bStart(h, e0, p0);

    bool pushed = false;
    for (int i = 0; i < 60000; i++) {
        bTick(h);
        if (h.out.motorPermille == B_FULL) pushed = true;
    }
    double rpm = bBurstRPM(h);
    bFinish(h);
    double want    = (double)(h.logic.ledgerPulses - p0) * bRevsPerPulse(h);
    double got     = bTurns(h, e0);
    bool   heavyOk = pushed && h.failures == 0 && !h.sawClogged && !h.lowDuty && bAddsUp(got, want);

    // And held for good: full PWM tried, then the burst fails, motor off.
    BHarness g;
    bHalfPwm(g);
    bRunPulses(g, 3, 30000);
    bool gInBurst = bUntilBurstSettled(g, 5000);
    bWatch(g);
    g.blocked = true;
    uint32_t t0 = g.nowMs;
    bool gPushed = false;
    for (int i = 0; i < (int)BURST_FAIL_MS + 1000 && g.failures == 0; i++) {
        bTick(g);
        if (g.out.motorPermille == B_FULL) gPushed = true;
    }
    uint32_t tFail  = g.nowMs - t0;
    DispenserMode after = (BURST_FAILS_BEFORE_UNCLOG > 1) ? DispenserMode::Normal : DispenserMode::AutoUnclogging;
    bool     failOk = gInBurst && gPushed && g.failures == 1 && g.out.motorPermille == 0 &&
                      g.logic.mode == after && tFail >= BURST_FAIL_MS &&
                      tFail <= BURST_FAIL_MS + 2 * MOTOR_CONTROL_INTERVAL_MS;

    bool ok = heavyOk && failOk;
    reportCheck("B26", ok, "too heavy for half PWM: pushed to full %s, bursts at %.0f RPM on average, none "
                "failed %s, %.2f of %.2f turns; held for good: full PWM tried %s, failed after %lu ms %s",
                pushed ? "yes" : "NO", rpm, (h.failures == 0) ? "yes" : "NO", got, want,
                gPushed ? "yes" : "NO", (unsigned long)tFail, failOk ? "yes" : "NO");
}

// B27 - the calibration run at half PWM: every step at half PWM, the same
// exact count.
static void testB27()
{
    BHarness h;
    bHalfPwm(h);
    bool started = bStartCalibration(h);
    bWatch(h);
    bool done = bCalibrateToEnd(h, 120000);
    double rpm = bBurstRPM(h);
    bRun(h, 1000);

    uint32_t turned  = h.edges - h.logic.calibrationStartEdges;
    double   want    = bCalibWantEdges(h);
    bool     edgesOk = (double)turned >= want - 1.0 && (double)turned <= want + 20.0;
    bool     ok      = started && done && edgesOk && !h.oddDuty && h.logic.progress == 100;

    reportCheck("B27", ok, "calibration at half PWM: every step at %u permille %s, %.0f RPM, %lu edges "
                "(want %.0f), Done %s",
                (unsigned)B_HALF, h.oddDuty ? "NO" : "yes", rpm, (unsigned long)turned, want,
                done ? "yes" : "NO");
}

// B28 - the switch itself: dispenserInit takes BURST_PERMILLE; full PWM stays
// full PWM whatever the shaft does; below it, the stall push comes only after
// a step of being driven, never on a burst's first step; and the clog line is
// 60 RPM, or a third of what the PWM turns a free motor at if that is lower.
static void testB28()
{
    DispenserLogic logic;
    dispenserInit(logic, 0, 0);
    bool ok = (logic.burstPermille == BURST_PERMILLE);

    logic.burstPermille = B_FULL;
    logic.measuredRPM   = 0;
    logic.prevOutput    = DispenserOutputs{0, true};        // a burst's first step
    if (burstDuty(logic) != B_FULL) ok = false;
    logic.prevOutput    = DispenserOutputs{B_FULL, true};   // driven through the last step
    if (burstDuty(logic) != B_FULL) ok = false;
    logic.measuredRPM   = MOTOR_MAX_RPM + 50;
    if (burstDuty(logic) != B_FULL) ok = false;
    if (burstClogLine(logic) != BURST_CLOG_MIN_RPM) ok = false;

    logic.burstPermille = B_HALF;
    logic.measuredRPM   = 0;
    logic.prevOutput    = DispenserOutputs{0, true};
    if (burstDuty(logic) != B_HALF) ok = false;              // first step: no push
    logic.prevOutput    = DispenserOutputs{B_HALF, true};
    if (burstDuty(logic) != B_FULL) ok = false;              // stalled after a driven step: push
    logic.measuredRPM   = 170;
    if (burstDuty(logic) != B_HALF) ok = false;              // turning: its own PWM again
    uint16_t lineHalf = burstClogLine(logic);
    if (lineHalf != (uint16_t)(165 * CLOG_MIN_SPEED_PERCENT / 100)) ok = false;
    logic.burstPermille = 364;                               // about 120 RPM on a free motor
    uint16_t line364 = burstClogLine(logic);
    if (line364 != (uint16_t)(120 * CLOG_MIN_SPEED_PERCENT / 100)) ok = false;

    reportCheck("B28", ok, "BURST_PWM_FRACTION %.2f gives %u permille; full PWM always full; at half PWM the push "
                "only after a driven step; clog line %u RPM at full PWM, %u at half, %u at 364 permille",
                (double)BURST_PWM_FRACTION, (unsigned)BURST_PERMILLE, (unsigned)BURST_CLOG_MIN_RPM,
                (unsigned)lineHalf, (unsigned)line364);
}

// B29 - a calibration run far beyond the motor: 785 mm per pulse at 6 km/h is
// a pulse every 471 ms, and at the tractor's largest angle factor, 999, a pulse
// asks for 1.41 turns - 0.85 s for an auger so heavy that the motor turns it at
// 100 RPM. In work the backlog would hit its cap after a few pulses and the
// rest be dropped (ZA SZYBKO); here every pulse's angle is turned in full, in
// one burst from start to finish, or the weight would come out short.
static void testB29()
{
    BHarness h;
    bFresh(h);                                 // 785 mm per pulse
    h.command.burstAngleFactor = 999;
    h.runRPM = 100.0;
    bool started = bStartCalibration(h);
    bWatch(h);
    bool done = bCalibrateToEnd(h, 120000);
    uint32_t took = h.nowMs - h.logic.calibrationStartMs;
    bRun(h, 1000);

    uint32_t turned  = h.edges - h.logic.calibrationStartEdges;
    double   want    = bCalibWantEdges(h);
    uint32_t fastest = (uint32_t)(want / (double)ENCODER_EDGES_PER_REV * 60000.0 / h.runRPM);
    bool ok = started && done && h.bursts == 1 && !h.oddDuty && took <= fastest + 500
           && (double)turned >= want - 1.0 && (double)turned <= want + 20.0;

    reportCheck("B29", ok, "a pulse every %lu ms, %.1f turns each - far more than the motor can: %lu burst(s) "
                "(want 1), all %.1f turns turned (%lu edges, want %.0f), Done after %lu ms (motor alone: %lu)",
                (unsigned long)bCalibIntervalMs(h.mmPerPulse), bRevsPerPulse(h), (unsigned long)h.bursts,
                want / (double)ENCODER_EDGES_PER_REV, (unsigned long)turned, want, (unsigned long)took,
                (unsigned long)fastest);
}

// B30 - the procedure, with the tractor's own arithmetic
// (src/tractor/angle_factor.h): calibrate, weigh, correct the angle factor,
// calibrate again. The run is always CALIBRATION_PULSES bursts and the factor
// sets their angle, so what the run turns - and weighs - is in proportion to
// the factor. With an auger that gives B_AUGER_GRAMS_PER_TURN, at 40 kg/ha and
// the machine's 1571 mm a pulse, the first-boot factor weighs short; one
// correction must bring the next run onto the expected mass, and halving the
// dose - which halves the factor - must halve it.
static constexpr double B_AUGER_GRAMS_PER_TURN = 15.0;

// One whole calibration run at this factor and dose, 1571 mm a pulse: the
// turns, and in `grams` what the scale reads - whole grams.
static double bCalibrationRun(uint16_t factor, uint16_t dose, uint32_t &grams, bool &ok)
{
    BHarness h;
    bFresh(h);
    h.mmPerPulse               = 1571;
    h.command.burstAngleFactor = factor;
    h.command.doseKgPerHa      = dose;
    ok = bStartCalibration(h) && bCalibrateToEnd(h, 120000) && h.bursts >= 1;
    bRun(h, 1000);
    double edges = (double)(h.edges - h.logic.calibrationStartEdges);
    ok = ok && edges >= bCalibWantEdges(h) - 1.0 && edges <= bCalibWantEdges(h) + 20.0;
    double turns = edges / (double)ENCODER_EDGES_PER_REV;
    grams = (uint32_t)(turns * B_AUGER_GRAMS_PER_TURN + 0.5);
    return turns;
}

static void testB30()
{
    const uint16_t mm = 1571;
    bool okA, okB, okC;
    uint32_t gramsA, gramsB, gramsC;

    uint32_t expected = expectedCalibrationGrams(40, mm);                      // 503 g
    uint16_t factorA  = DEFAULT_ANGLE_FACTOR;
    double   turnsA   = bCalibrationRun(factorA, 40, gramsA, okA);
    uint16_t factorB  = angleFactorFromWeighing(factorA, expected, gramsA);
    double   turnsB   = bCalibrationRun(factorB, 40, gramsB, okB);

    uint16_t factorC   = angleFactorForDose(factorB, 40, 20);
    uint32_t expectedC = expectedCalibrationGrams(20, mm);                     // 251 g
    bCalibrationRun(factorC, 20, gramsC, okC);

    // The factor is whole numbers and the scale and the expected mass whole
    // grams, so the corrected run is allowed half a unit of each it went
    // through - 0.2 % or less apiece in a 20-pulse run of 500 g, several % in
    // a run of a few pulses - and a little for each run's final braking.
    double errB  = ((double)gramsB - (double)expected) / (double)expected;
    double errC  = ((double)gramsC - (double)expectedC) / (double)expectedC;
    double tolB  = 0.002 + 0.5 / (double)expected + 0.5 / (double)gramsA + 0.5 / (double)gramsB +
                   0.5 / (double)factorB;
    double tolC  = tolB + 0.5 / (double)expectedC + 0.5 / (double)gramsC + 0.5 / (double)factorC;
    double ratio = turnsB / turnsA;
    double want  = (double)factorB / (double)factorA;

    // The ends of the range: capped at three digits, never 0, and left alone
    // with nothing to go by; and the largest run the wire allows, worked out
    // here the long way.
    uint32_t largest = (uint32_t)((double)CALIBRATION_PULSES * WHEEL_MM_PER_PULSE_MAX * WORKING_WIDTH_CM * 999.0 /
                                  1000000.0 + 0.5);
    bool ends = angleFactorFromWeighing(500, 503, 141) == ANGLE_FACTOR_MAX
             && angleFactorFromWeighing(5, 100, 100000) == 1
             && angleFactorFromWeighing(500, 503, 0) == 500
             && angleFactorFromWeighing(500, 0, 503) == 500
             && angleFactorForDose(500, 0, 40) == 500
             && angleFactorForDose(0, 40, 80) == 0
             && angleFactorForDose(500, 40, 50) == 625
             && expectedCalibrationGrams(999, WHEEL_MM_PER_PULSE_MAX) == largest;

    // The turns follow the factor exactly, but for the one burst's braking at
    // the end of each run - the same few edges whatever its length.
    double tolRatio = 0.002 + 2.0 * 20.0 / (turnsA * (double)ENCODER_EDGES_PER_REV);

    bool ok = okA && okB && okC && ends && gramsA < expected && gramsB > 0 && gramsC > 0
           && fabs(errB) <= tolB && fabs(errC) <= tolC && fabs(ratio - want) <= tolRatio;

    reportCheck("B30", ok, "%.0f g a turn, 40 kg/ha: factor %u weighs %lu g of %lu; corrected to %u, %lu g "
                "(%+.1f %%); at 20 kg/ha factor %u, %lu g of %lu (%+.1f %%); ends of range %s",
                B_AUGER_GRAMS_PER_TURN, (unsigned)factorA, (unsigned long)gramsA, (unsigned long)expected,
                (unsigned)factorB, (unsigned long)gramsB, 100.0 * errB, (unsigned)factorC,
                (unsigned long)gramsC, (unsigned long)expectedC, 100.0 * errC, ends ? "right" : "WRONG");
}

// ---------------------------------------------------------------------------
// B31 - held for good with the machine moving on: every burst fails, and the
// BURST_FAILS_BEFORE_UNCLOG-th in a row starts the unclog sequence by itself -
// reverse first, then forward, UNCLOG_TOTAL_MS in all - after which metering
// goes on, with no alarm at any point. Still blocked, it goes round again. At
// this speed, a pulse every 785 ms (like 8 km/h on the machine's 1571 mm), the
// failures never sound ZA SZYBKO either.
// ---------------------------------------------------------------------------

static void testB31()
{
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bWatch(h);
    h.blocked = true;

    for (int i = 0; i < 30000 && h.logic.mode != DispenserMode::AutoUnclogging; i++) bTick(h);
    bool     reached    = (h.logic.mode == DispenserMode::AutoUnclogging);
    uint32_t firstRound = h.failures;

    // Through the sequence, and back to Normal.
    uint32_t tStart       = h.nowMs;
    bool     sawReverse   = false;
    bool     sawForward   = false;
    bool     reverseFirst = false;
    for (int i = 0; i < (int)UNCLOG_TOTAL_MS + 1000 && h.logic.mode == DispenserMode::AutoUnclogging; i++) {
        bTick(h);
        if (h.logic.mode != DispenserMode::AutoUnclogging || h.out.motorPermille != UNCLOG_PERMILLE) continue;
        if (!h.out.motorForward) {
            if (!sawForward) reverseFirst = true;
            sawReverse = true;
        } else {
            sawForward = true;
        }
    }
    uint32_t took   = h.nowMs - tStart;
    bool     back   = (h.logic.mode == DispenserMode::Normal);
    bool     timeOk = took >= UNCLOG_TOTAL_MS && took <= UNCLOG_TOTAL_MS + 2 * MOTOR_CONTROL_INTERVAL_MS;

    // Still blocked: round again.
    for (int i = 0; i < 30000 && h.logic.mode != DispenserMode::AutoUnclogging; i++) bTick(h);
    bool again = (h.logic.mode == DispenserMode::AutoUnclogging);

    bool ok = reached && firstRound == BURST_FAILS_BEFORE_UNCLOG && reverseFirst && sawReverse && sawForward &&
              back && timeOk && again && h.failures == 2u * BURST_FAILS_BEFORE_UNCLOG && !h.sawClogged &&
              !h.sawOverSpeed;

    reportCheck("B31", ok, "held for good: unclogs by itself after %lu failures (want %u), reverse first %s, "
                "forward %s, metering again after %lu ms %s; round again %s; never Clogged %s, no ZA SZYBKO %s",
                (unsigned long)firstRound, (unsigned)BURST_FAILS_BEFORE_UNCLOG, reverseFirst ? "yes" : "NO",
                sawForward ? "yes" : "NO", (unsigned long)took, back ? "yes" : "NO", again ? "yes" : "NO",
                h.sawClogged ? "NO" : "yes", h.sawOverSpeed ? "NO" : "yes");
}

// ---------------------------------------------------------------------------
// B32 - "in a row" means nothing got through in between: one failure short of
// an unclog, a burst that reaches its count, as many again - and no unclog.
// Then the unclog sequence's own rules: the tractor going quiet does not stop
// it, the dispenser switched off does.
// ---------------------------------------------------------------------------

static void testB32()
{
    if (BURST_FAILS_BEFORE_UNCLOG < 2) {
        reportSkipped("B32", "BURST_FAILS_BEFORE_UNCLOG is 1: every failure unclogs at once (B31)");
        return;
    }
    const uint32_t shortOf = BURST_FAILS_BEFORE_UNCLOG - 1;

    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    bWatch(h);

    h.blocked = true;
    for (int i = 0; i < 30000 && h.failures < shortOf; i++) bTick(h);
    bool failed = h.failures == shortOf && h.logic.burstFailStreak == shortOf;

    // One gets through.
    h.blocked = false;
    for (int i = 0; i < 5000 && h.logic.burstFailStreak != 0; i++) bTick(h);
    bool cleared = (h.logic.burstFailStreak == 0);

    // As many again: never BURST_FAILS_BEFORE_UNCLOG in a row.
    h.blocked = true;
    for (int i = 0; i < 30000 && h.failures < 2 * shortOf; i++) bTick(h);
    bool noUnclog = failed && h.failures == 2 * shortOf && h.logic.burstFailStreak == shortOf && !h.sawAutoUnclog;

    // The next one in a row unclogs; the tractor going quiet does not stop it.
    for (int i = 0; i < 20000 && h.logic.mode != DispenserMode::AutoUnclogging; i++) bTick(h);
    h.tractorAlive = false;
    bRun(h, 1000);
    bool noTractor = (h.logic.mode == DispenserMode::AutoUnclogging);

    // Switched off: over at the next step.
    h.tractorAlive = true;
    h.command.dispenserEnabled = 0;
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 10);
    bool offStops = h.logic.mode == DispenserMode::Normal && h.out.motorPermille == 0;

    bool ok = cleared && noUnclog && noTractor && offStops && !h.sawClogged;
    reportCheck("B32", ok, "%lu failures, one through, %lu more: run cleared %s, no unclog %s; the unclog goes on "
                "without the tractor %s, stops when switched off %s",
                (unsigned long)shortOf, (unsigned long)shortOf, cleared ? "yes" : "NO", noUnclog ? "yes" : "NO",
                noTractor ? "yes" : "NO", offStops ? "yes" : "NO");
}

// ---------------------------------------------------------------------------
// B33 - the seeding simulation, standing still: one burst per pulse that
// SIMULATION_SPEED_MM_S would give at the tractor's distance per pulse, added
// up as in work and counted; after SIMULATION_DURATION_MS it is SimulationDone,
// motor off, holding its counts until the request is withdrawn. Withdrawn
// early, or the tractor gone, it simply stops.
// ---------------------------------------------------------------------------

// Arms and starts a simulation with the machine standing. True once it runs.
static bool bStartSimulation(BHarness &h)
{
    h.speedMmS = 0.0;                         // the seeder says stopped, and sends no pulses
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 10);  // simulationRun 0 arms it
    h.command.simulationRun = 1;
    for (int i = 0; i < 500; i++) {
        bTick(h);
        if (h.logic.mode == DispenserMode::Simulating) return true;
    }
    return false;
}

// The pulses the simulation has given by now.
static uint32_t bSimPulses(const BHarness &h)
{
    uint32_t t = h.nowMs - h.logic.simStartMs;
    return (uint32_t)((uint64_t)t * SIMULATION_SPEED_MM_S / (1000ULL * h.mmPerPulse));
}

static void testB33()
{
    // The machine's own numbers - the bench's factor of 1778 at 7 km/h would be
    // beyond the motor, and the tractor never sends more than 999.
    BHarness h;
    bFresh(h);
    h.mmPerPulse               = 1571;
    h.command.burstAngleFactor = DEFAULT_ANGLE_FACTOR;
    bool started = bStartSimulation(h);
    uint32_t e0 = h.edges;
    bWatch(h);

    // A minute of it, as in work.
    bRun(h, 60000);
    bUntilIdle(h, 5000);
    uint32_t pulses  = bSimPulses(h);
    double   want    = (double)pulses * bRevsPerPulse(h);
    double   got     = bTurns(h, e0);
    uint8_t  percent = h.logic.progress;
    uint32_t tNow    = h.nowMs - h.logic.simStartMs;
    bool asWork = started && bAddsUp(got, want) && h.logic.simBursts == pulses && h.logic.simFailures == 0 &&
                  h.failures == 0 && !h.sawOverSpeed && !h.sawClogged &&
                  percent == (uint8_t)((uint64_t)tNow * 100ULL / SIMULATION_DURATION_MS);

    // The rest of it, skipped: the end is the clock's, not the shaft's.
    uint16_t bursts = h.logic.simBursts;
    h.nowMs = h.logic.simStartMs + SIMULATION_DURATION_MS;
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 10);
    bool done = h.logic.mode == DispenserMode::SimulationDone && h.logic.progress == 100 &&
                h.out.motorPermille == 0;
    bRun(h, 2000);
    bool holds = h.logic.mode == DispenserMode::SimulationDone && h.logic.simBursts == bursts;
    h.command.simulationRun = 0;
    bRun(h, MOTOR_CONTROL_INTERVAL_MS + 10);
    bool back = (h.logic.mode == DispenserMode::Normal) && h.logic.simBursts == bursts;

    // Withdrawn early, and the tractor gone: over at once.
    BHarness w;
    bFresh(w);
    bool wStarted = bStartSimulation(w);
    bRun(w, 3000);
    w.command.simulationRun = 0;
    bRun(w, MOTOR_CONTROL_INTERVAL_MS + 10);
    bool withdrawn = wStarted && w.logic.mode == DispenserMode::Normal;
    bStartSimulation(w);
    bRun(w, 3000);
    w.tractorAlive = false;
    bRun(w, MOTOR_CONTROL_INTERVAL_MS + 10);
    bool noTractor = (w.logic.mode == DispenserMode::Normal);

    bool ok = asWork && done && holds && back && withdrawn && noTractor;
    reportCheck("B33", ok, "simulation, a minute: %u bursts for %lu pulses, %.2f turns (want %.2f), %u %%; done "
                "at the end %s, holds %s, back to Normal %s; withdrawn %s, tractor gone %s",
                (unsigned)bursts, (unsigned long)pulses, got, want, (unsigned)percent, done ? "yes" : "NO",
                holds ? "yes" : "NO", back ? "yes" : "NO", withdrawn ? "yes" : "NO", noTractor ? "yes" : "NO");
}

// ---------------------------------------------------------------------------
// B34 - failed bursts in the simulation act as in work: held for good, every
// burst fails, and BURST_FAILS_BEFORE_UNCLOG in a row run the unclog sequence -
// inside the simulation, which goes on after it - all of it counted, and the
// tractor's LED count too. Refused with the machine moving, and in continuous
// metering, which has no bursts to fail.
// ---------------------------------------------------------------------------

static void testB34()
{
    BHarness h;
    bFresh(h);
    bool started = bStartSimulation(h);
    bWatch(h);
    h.blocked = true;
    for (int i = 0; i < 30000 && h.logic.simUnclogs == 0; i++) bTick(h);
    bool unclogs = started && h.logic.simUnclogs == 1 && h.logic.simFailures == BURST_FAILS_BEFORE_UNCLOG &&
                   h.logic.mode == DispenserMode::Simulating && h.failures == h.logic.simFailures;

    bool reversed = false;
    for (int i = 0; i < (int)UNCLOG_TOTAL_MS + 500 && h.logic.simUnclogging; i++) {
        bTick(h);
        if (h.out.motorPermille == UNCLOG_PERMILLE && !h.out.motorForward) reversed = true;
    }
    bool resumed = !h.logic.simUnclogging && h.logic.mode == DispenserMode::Simulating;

    // Still blocked: failing again, inside the same simulation.
    for (int i = 0; i < 30000 && h.logic.simUnclogs < 2; i++) bTick(h);
    bool again = h.logic.simUnclogs == 2 && h.logic.simFailures == 2u * BURST_FAILS_BEFORE_UNCLOG &&
                 !h.sawClogged && !h.sawAutoUnclog && !h.sawOverSpeed;

    BHarness m;
    bFresh(m);
    bRun(m, MOTOR_CONTROL_INTERVAL_MS + 10);
    m.command.simulationRun = 1;
    bRun(m, 300);
    bool refusedMoving = (m.logic.mode == DispenserMode::Refused);
    m.command.simulationRun = 0;
    bRun(m, MOTOR_CONTROL_INTERVAL_MS + 10);
    bool refusedClears = (m.logic.mode == DispenserMode::Normal);

    BHarness c;
    bFresh(c);
    c.logic.burstMode = false;
    bool continuousRefused = !bStartSimulation(c) && c.logic.mode == DispenserMode::Refused;

    bool ok = unclogs && reversed && resumed && again && refusedMoving && refusedClears && continuousRefused;
    reportCheck("B34", ok, "held for good in the simulation: unclogs inside it after %u failures %s, reverse %s, "
                "goes on %s, again %s; refused moving %s (clears %s), in continuous metering %s",
                (unsigned)BURST_FAILS_BEFORE_UNCLOG, unclogs ? "yes" : "NO", reversed ? "yes" : "NO",
                resumed ? "yes" : "NO", again ? "yes" : "NO", refusedMoving ? "yes" : "NO",
                refusedClears ? "yes" : "NO", continuousRefused ? "yes" : "NO");
}

// ---------------------------------------------------------------------------
// B35 - the push backwards after every burst: BURST_BACKLASH_DELAY_MS after a
// burst stops, BURST_BACKLASH_PERMILLE backwards for BURST_BACKLASH_MS, then
// off - and no burst in the meantime. In work, in the calibration run, in the
// simulation, and after a failed burst; not when the failure unclogs instead,
// and cut short when the dispenser is switched off.
// ---------------------------------------------------------------------------

// Runs until a burst stops, then times what follows: the gap to the push, its
// duty and length, and whether anything drove forward before it was over.
static bool bTimePush(BHarness &h, uint32_t &gapMs, uint32_t &lenMs, uint16_t &duty, bool &forwardInside)
{
    bool wasForward = false;
    uint32_t tStop = 0, tPush = 0;
    forwardInside = false;
    duty = 0;
    for (int i = 0; i < 20000; i++) {
        bTick(h);
        bool forward = h.out.motorPermille > 0 && h.out.motorForward;
        bool back    = h.out.motorPermille > 0 && !h.out.motorForward;
        if (tStop == 0) {
            if (wasForward && !forward) tStop = h.nowMs;
            wasForward = forward;
            continue;
        }
        if (tPush == 0) {
            if (forward) forwardInside = true;
            if (back) {
                tPush = h.nowMs;
                duty  = h.out.motorPermille;
            }
            continue;
        }
        if (forward) forwardInside = true;
        if (!back) {
            gapMs = tPush - tStop;
            lenMs = h.nowMs - tPush;
            return true;
        }
    }
    return false;
}

static void testB35()
{
    if (BURST_BACKLASH_MS == 0 || BURST_BACKLASH_PERMILLE == 0) {
        reportSkipped("B35", "the push backwards is switched off (BURST_BACKLASH_MS or _PERMILLE 0)");
        return;
    }

    // In work.
    BHarness h;
    bFresh(h);
    uint32_t e0, p0;
    bStart(h, e0, p0);
    uint32_t gap = 0, len = 0;
    uint16_t duty = 0;
    bool     forwardInside = true;
    bool     timed = bTimePush(h, gap, len, duty, forwardInside);
    bool     workOk = timed && duty == BURST_BACKLASH_PERMILLE && !forwardInside &&
                      gap >= BURST_BACKLASH_DELAY_MS && gap <= BURST_BACKLASH_DELAY_MS + 1 &&
                      len >= BURST_BACKLASH_MS && len <= BURST_BACKLASH_MS + 1;

    // In the calibration run: one after every burst but the last, which Done cuts.
    BHarness c;
    bFresh(c);
    c.mmPerPulse               = 2500;             // separate bursts, as in B13
    c.command.burstAngleFactor = 889;
    bStartCalibration(c);
    bWatch(c);
    bCalibrateToEnd(c, 120000);
    bool calibOk = c.bursts == CALIBRATION_PULSES && c.pushes + 1 >= c.bursts && c.pushes <= c.bursts;

    // In the simulation.
    BHarness s;
    bFresh(s);
    s.mmPerPulse               = 1571;
    s.command.burstAngleFactor = DEFAULT_ANGLE_FACTOR;
    bStartSimulation(s);
    bWatch(s);
    bRun(s, 30000);
    bool simOk = s.bursts >= 30 && s.pushes + 1 >= s.bursts && s.pushes <= s.bursts;

    // After failed bursts: a push after each but the one that unclogs.
    BHarness f;
    bFresh(f);
    bStart(f, e0, p0);
    bWatch(f);
    f.blocked = true;
    for (int i = 0; i < 30000 && f.logic.mode != DispenserMode::AutoUnclogging; i++) bTick(f);
    bool failOk = f.failures == BURST_FAILS_BEFORE_UNCLOG &&
                  f.pushes == (uint32_t)(BURST_FAILS_BEFORE_UNCLOG - 1) + f.bursts - f.failures;

    // Switched off in the middle of one: over at the next step.
    BHarness o;
    bFresh(o);
    bStart(o, e0, p0);
    bool gotPush = false;
    for (int i = 0; i < 20000 && !gotPush; i++) {
        bTick(o);
        gotPush = o.logic.backlash == BacklashPush::Pushing;
    }
    o.command.dispenserEnabled = 0;
    bRun(o, MOTOR_CONTROL_INTERVAL_MS + 10);
    bool offOk = gotPush && o.out.motorPermille == 0 && o.logic.backlash == BacklashPush::None;

    bool ok = workOk && calibOk && simOk && failOk && offOk;
    reportCheck("B35", ok, "push back: %lu ms after the stop, %u permille for %lu ms, no burst inside %s; in "
                "calibration %lu for %lu bursts, simulation %lu for %lu; after failures %s; off stops it %s",
                (unsigned long)gap, (unsigned)duty, (unsigned long)len, forwardInside ? "NO" : "yes",
                (unsigned long)c.pushes, (unsigned long)c.bursts, (unsigned long)s.pushes,
                (unsigned long)s.bursts, failOk ? "yes" : "NO", offOk ? "yes" : "NO");
}

// ---------------------------------------------------------------------------

static void runBurstTests()
{
    Serial.println("--- B: burst metering (no hardware) ---");

    b19Ok    = true;
    b19Count = 0;

    testB01();
    testB02();
    testB03();
    testB04();
    testB05();
    testB06();
    testB07();
    testB08();
    testB09();
    testB10();
    testB11();
    testB12();
    testB13();
    testB14();
    testB15();
    testB16();
    testB17();
    testB18();
    testB20();
    testB21();
    testB22();
    testB23();
    testB24();
    testB25();
    testB26();
    testB27();
    testB28();
    testB29();
    testB30();
    testB31();
    testB32();
    testB33();
    testB34();
    testB35();

    reportCheck("B19", b19Ok && b19Count > 0,
                "dispenserFillStatus matched the step, with the averaged RPM, over %lu checks",
                (unsigned long)b19Count);
}
