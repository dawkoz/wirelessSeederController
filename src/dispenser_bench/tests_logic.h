#pragma once

#include <math.h>

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_logic.h"
#include "bench_report.h"

// ---------------------------------------------------------------------------
// D tests: the decision logic on its own, driven with made-up time, encoder
// counts and packets. No hardware, milliseconds per test, so timing edges can
// be checked exactly (slow for 1400 ms vs 1500 ms).
//
// Time starts at 100000 ms, one step is MOTOR_CONTROL_INTERVAL_MS unless a
// test says otherwise, and "shaft at R RPM" adds R x edges-per-rev x stepMs /
// 60000 edges per step through a fractional accumulator so a long run cannot
// drift.
// ---------------------------------------------------------------------------

struct DHarness {
    DispenserLogic   logic;
    DispenserOutputs out = {0, true};

    uint32_t nowMs     = 100000;
    uint32_t edges     = 0;
    double   edgeCarry = 0.0;
    double   shaftRPM  = 0.0;

    bool            seederAlive   = false;
    bool            haveTelemetry = false;
    SeederTelemetry telemetry     = {};
    bool            tractorAlive  = false;
    bool            haveCommand   = false;
    TractorCommand  command       = {};
    int32_t         upTimeOffset  = 0;
};

// Sub-checks of D32, collected while D03, D04, D10 and D20 run.
static bool    d32Ok    = true;
static uint8_t d32Count = 0;

// The expected RPM, computed independently of requiredShaftRPM():
//   speed[m/s] x 60 x (working width [m] x dose / 10) / (grams / 100)
static double independentRPM(uint16_t speedMmS, uint16_t doseKgPerHa, uint32_t gramsPer100Rev)
{
    if (speedMmS == 0 || doseKgPerHa == 0 || gramsPer100Rev == 0) return 0.0;

    double metresPerSecond = (double)speedMmS / 1000.0;
    double gramsPerMetre   = ((double)WORKING_WIDTH_CM / 100.0) * (double)doseKgPerHa / 10.0;
    double revsPerMetre    = gramsPerMetre / ((double)gramsPer100Rev / 100.0);

    return metresPerSecond * 60.0 * revsPerMetre;
}

static void dFresh(DHarness &h)
{
    dispenserInit(h.logic, 100000, 0);

    h.out = {0, true};
    h.nowMs     = 100000;
    h.edges     = 0;
    h.edgeCarry = 0.0;
    h.shaftRPM  = 0.0;

    h.seederAlive   = false;
    h.haveTelemetry = false;
    h.telemetry     = SeederTelemetry();
    h.tractorAlive  = false;
    h.haveCommand   = false;
    h.command       = TractorCommand();
    h.upTimeOffset  = 0;
}

// "defaults": seeder alive with telemetry (1000 mm/s, turning) and tractor
// alive with a command (enabled, 40 kg/ha, 500 g/100 rev, calibrationRun 0,
// counters 0, upTimeMs growing with time) - which asks for 192 RPM.
static void dDefaults(DHarness &h)
{
    h.seederAlive   = true;
    h.haveTelemetry = true;
    h.telemetry.groundSpeedMmS = 1000;
    h.telemetry.wheelTurning   = 1;

    h.tractorAlive  = true;
    h.haveCommand   = true;
    h.command.dispenserEnabled = 1;
    h.command.doseKgPerHa      = 40;
    h.command.gramsPer100Rev   = 500;
    h.command.calibrationRun   = 0;
    h.command.clogClearSeq     = 0;
    h.command.unclogSeq        = 0;
    h.upTimeOffset             = 0;
}

static void dSetCounters(DHarness &h, uint8_t clear, uint8_t unclog)
{
    h.command.clogClearSeq = clear;
    h.command.unclogSeq    = unclog;
}

// The command's upTimeMs is nowMs + offset, so a test can make it jump
// backwards (a tractor reboot) by setting an explicit value.
static void dSetUpTime(DHarness &h, uint32_t value)
{
    h.upTimeOffset = (int32_t)((int64_t)value - (int64_t)h.nowMs);
}

// "following": the shaft turns at whatever the previous step asked for.
static void dFollow(DHarness &h)
{
    h.shaftRPM = (double)h.logic.targetRPM;
}

static bool dStep(DHarness &h, uint32_t stepMs = MOTOR_CONTROL_INTERVAL_MS)
{
    h.edgeCarry += h.shaftRPM * (double)ENCODER_EDGES_PER_REV * (double)stepMs / 60000.0;
    uint32_t whole = (uint32_t)h.edgeCarry;
    h.edgeCarry -= (double)whole;
    h.edges += whole;

    h.nowMs += stepMs;
    h.command.upTimeMs = (uint32_t)((int64_t)h.nowMs + h.upTimeOffset);

    DispenserInputs in;
    in.nowMs        = h.nowMs;
    in.encoderEdges = h.edges;
    in.seederAlive  = h.seederAlive;
    in.haveTelemetry = h.haveTelemetry;
    in.telemetry    = h.telemetry;
    in.tractorAlive = h.tractorAlive;
    in.haveCommand  = h.haveCommand;
    in.command      = h.command;

    return dispenserStep(h.logic, in, h.out);
}

// D32: the status must report what the step just did.
static void dCheckStatus(const DHarness &h)
{
    DispenserStatus status;
    dispenserFillStatus(h.logic, status);

    d32Count++;
    if (status.mode != h.logic.mode) d32Ok = false;
    if (status.faultCode != h.logic.fault) d32Ok = false;
    if (status.motorCommandedPermille != h.out.motorPermille) d32Ok = false;
    if (status.motorRunning != ((h.out.motorPermille > 0) ? 1 : 0)) d32Ok = false;
    if (status.targetShaftRPM != h.logic.targetRPM) d32Ok = false;
    if (status.progressPercent != h.logic.progress) d32Ok = false;
}

// "reach Clogged": defaults, following for 10 steps, then the shaft stops and
// stays stopped until the mode is Clogged.
static bool dReachClogged(DHarness &h)
{
    dFresh(h);
    dDefaults(h);
    for (int i = 0; i < 10; i++) {
        dStep(h);
        dFollow(h);
    }
    h.shaftRPM = 0;
    for (int i = 0; i < 40; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) return true;
    }
    return false;
}

// "Calibrating for N steps": no seeder, calibrationRun 0 for one step then 1,
// shaft at CALIBRATION_RPM. Returns true when the run was still going after
// N steps.
static bool dCalibrateFor(DHarness &h, int steps)
{
    dFresh(h);
    h.tractorAlive = true;
    h.haveCommand  = true;
    h.command.dispenserEnabled = 1;
    h.command.doseKgPerHa      = 40;
    h.command.gramsPer100Rev   = 500;
    h.command.calibrationRun   = 0;      // armed by this step
    h.command.upTimeMs         = h.nowMs;

    dStep(h);

    h.command.calibrationRun = 1;
    h.shaftRPM = CALIBRATION_RPM;

    for (int i = 0; i < steps; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Calibrating) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// D01 - the rate maths
// ---------------------------------------------------------------------------

static void testD01()
{
    struct Case { uint16_t speed; uint16_t dose; uint32_t grams; };
    const Case cases[] = {
        {1000, 40, 500}, {500, 40, 500}, {1667, 40, 800},
        {2778, 40, 800}, {65535, 999, 1}, {1, 1, 99999},
    };

    bool ok = true;
    char detail[96];
    detail[0] = '\0';

    for (uint8_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint32_t got    = requiredShaftRPM(cases[i].speed, cases[i].dose, cases[i].grams);
        double   expect = independentRPM(cases[i].speed, cases[i].dose, cases[i].grams);

        if (fabs((double)got - expect) > 1.0) {
            ok = false;
            snprintf(detail, sizeof(detail), "(%u,%u,%lu) got %lu want %.1f",
                     (unsigned)cases[i].speed, (unsigned)cases[i].dose,
                     (unsigned long)cases[i].grams, (unsigned long)got, expect);
        }
    }

    uint32_t zeros = requiredShaftRPM(0, 40, 500)
                   | requiredShaftRPM(1000, 0, 500)
                   | requiredShaftRPM(1000, 40, 0);
    if (zeros != 0) {
        ok = false;
        snprintf(detail, sizeof(detail), "a zero input did not give 0 (%lu)", (unsigned long)zeros);
    }

    if (ok) reportCheck("D01", true, "6 cases within 1 of the independent formula, 0 on any zero input");
    else    reportCheck("D01", false, "%s", detail);
}

// ---------------------------------------------------------------------------
// D02 - nothing turns the motor: nine input combinations
// ---------------------------------------------------------------------------

struct D02Case {
    const char *id;        // D02a .. D02i
    const char *label;
    bool     command;      // tractor packet present
    bool     telemetry;    // seeder packet present
    uint8_t  enabled;
    uint16_t dose;
    uint32_t grams;
    uint16_t speed;
    uint8_t  turning;
    bool     noSpeedData;  // expected fault
};

static void testD02()
{
    const D02Case cases[] = {
        {"D02a", "(a) no packets",            false, false, 1,   40, 500, 1000, 1, true },
        {"D02b", "(b) command only",          true,  false, 1,   40, 500, 1000, 1, true },
        {"D02c", "(c) telemetry only",        false, true,  1,   40, 500, 1500, 1, false},
        {"D02d", "(d) defaults, enabled 0",   true,  true,  0,   40, 500, 1000, 1, false},
        {"D02e", "(e) dose 0",                true,  true,  1,    0, 500, 1000, 1, false},
        {"D02f", "(f) 0 g/100 rev",           true,  true,  1,   40,   0, 1000, 1, false},
        {"D02g", "(g) speed 0, not turning",  true,  true,  1,   40, 500,    0, 0, false},
        {"D02h", "(h) turning, speed 0",      true,  true,  1,   40, 500,    0, 1, false},
        {"D02i", "(i) speed 1500, not turning", true, true,  1,   40, 500, 1500, 0, false},
    };

    for (uint8_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        DHarness h;
        dFresh(h);

        if (cases[c].command) {
            h.tractorAlive = true;
            h.haveCommand  = true;
            h.command.dispenserEnabled = cases[c].enabled;
            h.command.doseKgPerHa      = cases[c].dose;
            h.command.gramsPer100Rev   = cases[c].grams;
            h.command.calibrationRun   = 0;
        }
        if (cases[c].telemetry) {
            h.seederAlive   = true;
            h.haveTelemetry = true;
            h.telemetry.groundSpeedMmS = cases[c].speed;
            h.telemetry.wheelTurning   = cases[c].turning;
        }

        bool ok = true;
        for (int i = 0; i < 30; i++) {
            dStep(h);
            if (h.out.motorPermille != 0) ok = false;
            if (h.logic.mode != DispenserMode::Normal) ok = false;
            DispenserFault want = cases[c].noSpeedData ? DispenserFault::NoSpeedData
                                                       : DispenserFault::None;
            if (h.logic.fault != want) ok = false;
        }

        reportCheck(cases[c].id, ok, "%s duty 0 and Normal for 30 steps, fault %s",
                    cases[c].label,
                    cases[c].noSpeedData ? "NoSpeedData" : "None");
    }
}

// ---------------------------------------------------------------------------
// D03 - steady metering at the default setting (192 RPM)
// ---------------------------------------------------------------------------

static void testD03()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);

    // The catalogue's own expectation: the duty a correct feed-forward gives at
    // 192 RPM, +-30 permille once the loop has settled.
    const double expectedDuty = 192.0 * 1000.0 / (double)MOTOR_MAX_RPM;
    bool ok = true;
    double worst = 0.0;

    for (int i = 0; i < 30; i++) {
        dStep(h);
        dFollow(h);

        if (i >= 10) {
            if (h.logic.mode != DispenserMode::Normal) ok = false;
            if (h.logic.fault != DispenserFault::None) ok = false;
            if (h.logic.targetRPM != 192) ok = false;
            if (!h.out.motorForward) ok = false;

            double deviation = fabs((double)h.out.motorPermille - expectedDuty);
            if (deviation > worst) worst = deviation;
            if (deviation > 30.0) ok = false;
        }
        dCheckStatus(h);
    }

    reportCheck("D03", ok, "step 10+: target 192, duty %.0f +-30 (worst %.0f), forward, fault None",
                expectedDuty, worst);
}

// ---------------------------------------------------------------------------
// D04 - over speed: the machine outruns the motor
// ---------------------------------------------------------------------------

static void testD04()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    h.telemetry.groundSpeedMmS = 3000;
    h.shaftRPM = 300;

    bool ok = true;
    for (int i = 0; i < 50; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Normal) ok = false;
        if (h.logic.fault != DispenserFault::OverSpeed) ok = false;
        if (h.logic.targetRPM != MOTOR_MAX_RPM) ok = false;
        if (i >= 1 && h.out.motorPermille != 1000) ok = false;
        if (h.logic.mode == DispenserMode::Clogged) ok = false;
        dCheckStatus(h);
    }

    reportCheck("D04", ok, "target %u, fault OverSpeed, duty 1000 from step 2, never Clogged",
                (unsigned)MOTOR_MAX_RPM);
}

// ---------------------------------------------------------------------------
// D05 - a stall takes CLOG_DETECT_MS to raise the clog
// ---------------------------------------------------------------------------

static void testD05()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    for (int i = 0; i < 20; i++) {
        dStep(h);
        dFollow(h);
    }

    h.shaftRPM = 0;

    bool ok = true;
    uint32_t tSlow = 0, tClog = 0;
    bool sawSlow = false, sawClog = false;
    uint16_t dutyAtClog = 0xFFFF;
    bool forwardAtClog = false;
    uint8_t progressAtClog = 255;

    for (int i = 0; i < 40; i++) {
        dStep(h);
        if (!sawSlow) {
            sawSlow = true;
            tSlow = h.nowMs;
        }
        if (h.logic.mode == DispenserMode::Clogged) {
            if (!sawClog) {
                sawClog = true;
                tClog = h.nowMs;
                dutyAtClog = h.out.motorPermille;
                forwardAtClog = h.out.motorForward;
                progressAtClog = h.logic.progress;
            }
        } else if (sawClog) {
            ok = false;   // must not leave Clogged by itself
        }
    }

    if (!sawClog) ok = false;
    if (sawClog && (tClog - tSlow) != CLOG_DETECT_MS) ok = false;
    if (dutyAtClog != 0 || !forwardAtClog) ok = false;
    if (progressAtClog != 0) ok = false;
    if (h.logic.fault != DispenserFault::None) ok = false;

    for (int i = 0; i < 50; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Clogged) ok = false;
        if (h.out.motorPermille != 0) ok = false;
    }

    reportCheck("D05", ok, "Clogged %lu ms after the shaft stopped (want %lu), duty 0 forward, "
                "stays Clogged for 50 more steps",
                (unsigned long)(tClog - tSlow), (unsigned long)CLOG_DETECT_MS);
}

// ---------------------------------------------------------------------------
// D06 - the threshold is a third of the commanded speed
// ---------------------------------------------------------------------------

static void testD06()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    h.shaftRPM = 69;   // 36 % of 192

    bool ok = true;
    for (int i = 0; i < 50; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) ok = false;
    }
    reportCheck("D06a", ok, "69 RPM (36 %%) never Clogged over 50 steps");

    DHarness g;
    dFresh(g);
    dDefaults(g);
    g.shaftRPM = 57;   // 30 % of 192

    bool sawSlow = false, sawClog = false;
    bool okB = true;
    uint32_t tSlow = 0, tClog = 0;
    for (int i = 0; i < 40; i++) {
        dStep(g);
        if (!sawSlow) { sawSlow = true; tSlow = g.nowMs; }
        if (g.logic.mode == DispenserMode::Clogged && !sawClog) {
            sawClog = true;
            tClog = g.nowMs;
        }
    }
    okB = sawClog && ((tClog - tSlow) == CLOG_DETECT_MS);
    reportCheck("D06b", okB, "57 RPM (30 %%) Clogged after %lu ms (want %lu)",
                (unsigned long)(tClog - tSlow), (unsigned long)CLOG_DETECT_MS);
}

// ---------------------------------------------------------------------------
// D07 - the timer restarts: 14 slow steps then one at speed
// ---------------------------------------------------------------------------

static void testD07()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    for (int i = 0; i < 10; i++) {
        dStep(h);
        dFollow(h);
    }

    bool ok = true;
    for (int cycle = 0; cycle < 3; cycle++) {
        h.shaftRPM = 0;
        for (int i = 0; i < 14; i++) {
            dStep(h);
            if (h.logic.mode == DispenserMode::Clogged) ok = false;
        }
        h.shaftRPM = 192;
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) ok = false;
    }

    reportCheck("D07", ok, "3 x (14 stopped steps + 1 at speed) never Clogged");
}

// ---------------------------------------------------------------------------
// D08 - a clog during the calibration run, then clear, then re-arm
// ---------------------------------------------------------------------------

static void testD08()
{
    DHarness h;
    dFresh(h);
    h.tractorAlive = true;
    h.haveCommand  = true;
    h.command.dispenserEnabled = 1;
    h.command.doseKgPerHa      = 40;
    h.command.gramsPer100Rev   = 500;
    h.command.calibrationRun   = 0;
    h.command.upTimeMs         = h.nowMs;

    dStep(h);                       // arms the run
    h.command.calibrationRun = 1;
    h.shaftRPM = CALIBRATION_RPM;

    bool ok = true;
    for (int i = 0; i < 10; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Calibrating) ok = false;
        // The step that enters Calibrating resets the controller (target 0,
        // motor off); the run drives from the next step.
        if (i > 0 && h.logic.targetRPM != CALIBRATION_RPM) ok = false;
    }

    h.shaftRPM = 0;
    bool sawSlow = false, sawClog = false;
    uint32_t tSlow = 0, tClog = 0;
    for (int i = 0; i < 40; i++) {
        dStep(h);
        if (!sawSlow) { sawSlow = true; tSlow = h.nowMs; }
        if (h.logic.mode == DispenserMode::Clogged && !sawClog) {
            sawClog = true;
            tClog = h.nowMs;
        }
    }
    if (!sawClog || (tClog - tSlow) != CLOG_DETECT_MS) ok = false;

    // calibrationRun is still 1: Clogged holds.
    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Clogged) ok = false;
    }

    // Anuluj clears it.
    dSetCounters(h, 1, 0);
    dStep(h);
    if (h.logic.mode != DispenserMode::Normal) ok = false;
    if (h.out.motorPermille != 0) ok = false;

    // The request is still raised, but the run is not armed any more.
    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Calibrating) ok = false;
        if (h.out.motorPermille != 0) ok = false;
    }

    // Lowering and raising it again starts a new run.
    h.command.calibrationRun = 0;
    h.shaftRPM = CALIBRATION_RPM;
    dStep(h);
    h.command.calibrationRun = 1;
    dStep(h);
    if (h.logic.mode != DispenserMode::Calibrating) ok = false;

    reportCheck("D08", ok, "calibration clogged after %lu ms, clear works, run re-arms only "
                "after the request drops", (unsigned long)(tClog - tSlow));
}

// ---------------------------------------------------------------------------
// D09 - Anuluj goes straight back to metering
// ---------------------------------------------------------------------------

static void testD09()
{
    DHarness h;
    bool ok = dReachClogged(h);

    dSetCounters(h, 1, 0);
    dStep(h);
    if (h.logic.mode != DispenserMode::Normal) ok = false;
    if (h.out.motorPermille != 0) ok = false;
    if (h.logic.fault != DispenserFault::None) ok = false;

    bool dutyUp = false;
    for (int i = 0; i < 2; i++) {
        dStep(h);
        if (h.out.motorPermille > 0) dutyUp = true;
    }
    if (!dutyUp) ok = false;

    reportCheck("D09", ok, "clear gives Normal with duty 0, then metering within 2 steps");
}

// ---------------------------------------------------------------------------
// D10 - Odetkaj: the reverse/forward sequence, phase by phase
// ---------------------------------------------------------------------------

// Walks the unclog sequence from the step that entered it, checking every
// phase against the mode table. Returns true when every step matched.
static bool dRunUnclogSequence(DHarness &h, uint32_t stepMs, uint32_t startMs,
                               uint32_t *cloggedAtMs)
{
    bool ok = true;
    uint8_t lastProgress = h.logic.progress;

    for (int i = 0; i < 60; i++) {
        dStep(h, stepMs);

        uint32_t t = h.nowMs - startMs;

        if (h.logic.mode == DispenserMode::Clogged) {
            if (cloggedAtMs != NULL) *cloggedAtMs = t;
            if (h.out.motorPermille != 0) ok = false;
            if (!h.out.motorForward) ok = false;
            return ok;
        }

        if (h.logic.mode != DispenserMode::Unclogging) {
            ok = false;
            return ok;
        }

        uint32_t p = t % UNCLOG_CYCLE_MS;
        uint16_t wantDuty;
        bool     wantForward;
        if (p < UNCLOG_REVERSE_MS) {
            wantDuty = UNCLOG_PERMILLE; wantForward = false;
        } else if (p < UNCLOG_REVERSE_MS + UNCLOG_PAUSE_MS) {
            wantDuty = 0; wantForward = true;
        } else if (p < UNCLOG_REVERSE_MS + UNCLOG_PAUSE_MS + UNCLOG_FORWARD_MS) {
            wantDuty = UNCLOG_PERMILLE; wantForward = true;
        } else {
            wantDuty = 0; wantForward = true;
        }

        if (h.out.motorPermille != wantDuty) ok = false;
        if (h.out.motorForward != wantForward) ok = false;
        if (h.logic.progress < lastProgress) ok = false;
        if (h.logic.mode == DispenserMode::Unclogging && h.logic.progress >= 100) ok = false;
        lastProgress = h.logic.progress;
    }

    return false;   // never finished the sequence
}

static void testD10()
{
    DHarness h;
    bool ok = dReachClogged(h);

    dSetCounters(h, 0, 1);
    dStep(h);
    uint32_t start = h.nowMs;

    if (h.logic.mode != DispenserMode::Unclogging) ok = false;
    if (h.out.motorPermille != UNCLOG_PERMILLE) ok = false;
    if (h.out.motorForward) ok = false;
    dCheckStatus(h);

    uint32_t cloggedAt = 0;
    if (!dRunUnclogSequence(h, MOTOR_CONTROL_INTERVAL_MS, start, &cloggedAt)) ok = false;
    if (cloggedAt != UNCLOG_TOTAL_MS) ok = false;

    reportCheck("D10", ok, "first step reverse at %u permille, every phase matched, "
                "Clogged again at %lu ms (want %lu)",
                (unsigned)UNCLOG_PERMILLE, (unsigned long)cloggedAt, (unsigned long)UNCLOG_TOTAL_MS);
}

// ---------------------------------------------------------------------------
// D11 - the reversal guard, with uneven step timing
// ---------------------------------------------------------------------------

static void testD11()
{
    DHarness h;
    bool ok = dReachClogged(h);

    dSetCounters(h, 0, 1);

    uint32_t start = 0;
    bool first = true;
    uint16_t prevDuty = 0;
    bool     prevForward = true;
    bool     sawClog = false;
    uint32_t cloggedAt = 0;

    for (int i = 0; i < 80; i++) {
        uint32_t stepMs = (i % 2 == 0) ? 100 : 170;
        dStep(h, stepMs);

        if (first) {
            first = false;
            start = h.nowMs;
        }

        if (h.out.motorPermille > 0 && prevDuty > 0 && h.out.motorForward != prevForward) {
            ok = false;   // duty > 0 in the opposite direction of the previous step
        }
        prevDuty    = h.out.motorPermille;
        prevForward = h.out.motorForward;

        if (h.logic.mode == DispenserMode::Clogged && !sawClog) {
            sawClog = true;
            cloggedAt = h.nowMs - start;
        }
    }

    if (!sawClog) ok = false;
    if (sawClog && cloggedAt < UNCLOG_TOTAL_MS) ok = false;

    reportCheck("D11", ok, "no reversal at duty > 0, Clogged at %lu ms (want %lu)",
                (unsigned long)cloggedAt, (unsigned long)UNCLOG_TOTAL_MS);
}

// ---------------------------------------------------------------------------
// D12 - Anuluj while the sequence runs
// ---------------------------------------------------------------------------

static void testD12()
{
    DHarness h;
    bool ok = dReachClogged(h);

    dSetCounters(h, 0, 1);
    dStep(h);
    uint32_t start = h.nowMs;

    bool cleared = false;
    for (int i = 0; i < 60; i++) {
        uint32_t t = h.nowMs - start;
        if (t >= 1200 && !cleared) {
            dSetCounters(h, 1, 1);
            cleared = true;
        }
        dStep(h);
        if (cleared) {
            if (h.logic.mode != DispenserMode::Normal) ok = false;
            if (h.out.motorPermille != 0) ok = false;
            break;
        }
    }
    if (!cleared) ok = false;

    reportCheck("D12", ok, "Anuluj during Unclogging gives Normal and duty 0 in that step");
}

// ---------------------------------------------------------------------------
// D13 - the tractor disappears mid-sequence
// ---------------------------------------------------------------------------

static void testD13()
{
    DHarness h;
    bool ok = dReachClogged(h);

    dSetCounters(h, 0, 1);
    dStep(h);
    uint32_t start = h.nowMs;

    for (int i = 0; i < 60; i++) {
        uint32_t t = h.nowMs - start;
        if (t >= 500) {
            h.tractorAlive = false;
            dStep(h);
            if (h.logic.mode != DispenserMode::Clogged) ok = false;
            if (h.out.motorPermille != 0) ok = false;
            break;
        }
        dStep(h);
    }

    // The tractor comes back with the same counters: the sequence must not
    // resume by itself.
    h.tractorAlive = true;
    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Clogged) ok = false;
        if (h.out.motorPermille != 0) ok = false;
    }

    reportCheck("D13", ok, "lost tractor gives Clogged and duty 0, and it stays there");
}

// ---------------------------------------------------------------------------
// D14 - a counter change while the tractor is silent is not replayed
// ---------------------------------------------------------------------------

static void testD14()
{
    DHarness h;
    bool ok = dReachClogged(h);

    h.tractorAlive = false;
    dSetCounters(h, 0, 1);          // sent while nobody is listening
    for (int i = 0; i < 20; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Clogged) ok = false;
    }

    h.tractorAlive = true;
    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Clogged) ok = false;
        if (h.out.motorPermille != 0) ok = false;
    }

    dSetCounters(h, 0, 2);
    dStep(h);
    if (h.logic.mode != DispenserMode::Unclogging) ok = false;

    reportCheck("D14", ok, "the change seen during the gap is not replayed; a new one acts");
}

// ---------------------------------------------------------------------------
// D15 - a tractor reboot (upTimeMs going backwards) is a resync
// ---------------------------------------------------------------------------

static void testD15()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    dSetCounters(h, 3, 5);

    for (int i = 0; i < 10; i++) {
        dStep(h);
        dFollow(h);
    }
    dSetUpTime(h, 500000);

    h.shaftRPM = 0;
    bool ok = true;
    for (int i = 0; i < 40; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) break;
    }
    if (h.logic.mode != DispenserMode::Clogged) ok = false;

    // Rebooted tractor: counters back to 0 and upTimeMs far below the last one.
    dSetUpTime(h, 1200);
    dSetCounters(h, 0, 0);
    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Clogged) ok = false;
        if (h.out.motorPermille != 0) ok = false;
    }

    dSetCounters(h, 0, 1);
    dStep(h);
    if (h.logic.mode != DispenserMode::Unclogging) ok = false;

    reportCheck("D15", ok, "reboot resyncs, then a real change acts");
}

// ---------------------------------------------------------------------------
// D16 - the first command after boot never acts on its counters
// ---------------------------------------------------------------------------

static void testD16()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    dSetCounters(h, 7, 9);

    bool ok = true;
    for (int i = 0; i < 10; i++) {
        dStep(h);
        dFollow(h);
        if (h.logic.mode != DispenserMode::Normal) ok = false;
    }

    h.shaftRPM = 0;
    for (int i = 0; i < 40; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) break;
    }
    if (h.logic.mode != DispenserMode::Clogged) ok = false;
    if (h.logic.lastClearSeq != 7) ok = false;

    dSetCounters(h, 7, 10);
    dStep(h);
    if (h.logic.mode != DispenserMode::Unclogging) ok = false;

    reportCheck("D16", ok, "first command (7,9) acted on nothing, then 9 -> 10 unclogs");
}

// ---------------------------------------------------------------------------
// D17 - changes seen while metering are not replayed later
// ---------------------------------------------------------------------------

static void testD17()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    dSetCounters(h, 1, 1);

    bool ok = true;
    for (int i = 0; i < 10; i++) {
        dStep(h);
        dFollow(h);
        if (h.logic.mode != DispenserMode::Normal) ok = false;
        if (h.out.motorPermille == 0) ok = false;
        if (!h.out.motorForward) ok = false;
    }

    h.shaftRPM = 0;
    for (int i = 0; i < 40; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) break;
    }
    if (h.logic.mode != DispenserMode::Clogged) ok = false;

    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Clogged) ok = false;
    }

    reportCheck("D17", ok, "metering is unaffected, and the clog is not cleared by the old change");
}

// ---------------------------------------------------------------------------
// D18 - the unclog counter wraps 255 -> 0
// ---------------------------------------------------------------------------

static void testD18()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    dSetCounters(h, 0, 255);          // the value the counter sits at

    for (int i = 0; i < 10; i++) {
        dStep(h);
        dFollow(h);
    }
    h.shaftRPM = 0;

    bool ok = true;
    for (int i = 0; i < 40; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) break;
    }
    if (h.logic.mode != DispenserMode::Clogged) ok = false;
    if (h.logic.lastUnclogSeq != 255) ok = false;

    dSetCounters(h, 0, 0);
    dStep(h);
    if (h.logic.mode != DispenserMode::Unclogging) ok = false;

    reportCheck("D18", ok, "255 -> 0 counts as a change");
}

// ---------------------------------------------------------------------------
// D19 - Anuluj wins when both counters change in the same packet
// ---------------------------------------------------------------------------

static void testD19()
{
    DHarness h;
    bool ok = dReachClogged(h);

    dSetCounters(h, 1, 1);
    dStep(h);
    if (h.logic.mode != DispenserMode::Normal) ok = false;

    for (int i = 0; i < 10; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Unclogging) ok = false;
    }

    reportCheck("D19", ok, "both at once gives Normal, never Unclogging");
}

// ---------------------------------------------------------------------------
// D20 - a calibration run counts the revolutions and stops at the end
// ---------------------------------------------------------------------------

static void testD20()
{
    DHarness h;
    dFresh(h);
    h.tractorAlive = true;
    h.haveCommand  = true;
    h.command.dispenserEnabled = 1;
    h.command.doseKgPerHa      = 40;
    h.command.gramsPer100Rev   = 500;
    h.command.calibrationRun   = 0;
    h.command.upTimeMs         = h.nowMs;

    dStep(h);                                  // arms it
    h.command.calibrationRun = 1;
    h.shaftRPM = CALIBRATION_RPM;

    bool ok = true;
    bool started = false;
    uint8_t lastProgress = 0;
    bool sawDone = false;
    uint32_t edgesAtDone = 0;
    uint16_t dutyAtDone = 0xFFFF;
    uint8_t progressAtDone = 0;

    for (int i = 0; i < 700; i++) {
        dStep(h);

        if (h.logic.mode == DispenserMode::Calibrating) {
            // The step that enters Calibrating resets the controller (target 0,
            // motor off); the run drives from the next step.
            if (started && h.logic.targetRPM != CALIBRATION_RPM) ok = false;
            started = true;
            if (!h.out.motorForward) ok = false;
            if (h.logic.progress < lastProgress) ok = false;
            lastProgress = h.logic.progress;

            uint32_t turned = h.edges - h.logic.calibrationStartEdges;
            if (turned >= CALIBRATION_REVOLUTIONS * ENCODER_EDGES_PER_REV) ok = false;  // should be Done
        } else if (h.logic.mode == DispenserMode::CalibrationDone) {
            sawDone = true;
            edgesAtDone    = h.edges - h.logic.calibrationStartEdges;
            dutyAtDone     = h.out.motorPermille;
            progressAtDone = h.logic.progress;
            break;
        }
        dCheckStatus(h);
    }

    if (!started || !sawDone) ok = false;
    if (edgesAtDone < CALIBRATION_REVOLUTIONS * ENCODER_EDGES_PER_REV) ok = false;
    if (dutyAtDone != 0) ok = false;
    if (progressAtDone != 100) ok = false;

    reportCheck("D20", ok, "Calibrating at target %u, Done after %lu edges (want %lu), duty 0, 100 %%",
                (unsigned)CALIBRATION_RPM, (unsigned long)edgesAtDone,
                (unsigned long)(CALIBRATION_REVOLUTIONS * ENCODER_EDGES_PER_REV));
}

// ---------------------------------------------------------------------------
// D21 - a finished run waits for the tractor to withdraw the request
// ---------------------------------------------------------------------------

static void testD21()
{
    DHarness h;
    bool ok = dCalibrateFor(h, 20);
    h.command.calibrationRun = 1;

    // Run it to the end.
    for (int i = 0; i < 700 && h.logic.mode == DispenserMode::Calibrating; i++) {
        dStep(h);
    }
    if (h.logic.mode != DispenserMode::CalibrationDone) ok = false;

    for (int i = 0; i < 50; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::CalibrationDone) ok = false;
        if (h.out.motorPermille != 0) ok = false;
    }

    h.command.calibrationRun = 0;
    dStep(h);
    if (h.logic.mode != DispenserMode::Normal) ok = false;

    reportCheck("D21", ok, "CalibrationDone holds with the motor off until the request drops");
}

// ---------------------------------------------------------------------------
// D22 - a run is refused while the machine moves
// ---------------------------------------------------------------------------

static void testD22()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);

    dStep(h);                       // calibrationRun 0 arms it
    h.command.calibrationRun = 1;
    dStep(h);
    bool ok = (h.logic.mode == DispenserMode::Refused);

    for (int i = 0; i < 50; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Refused) ok = false;
        if (h.out.motorPermille != 0) ok = false;
    }

    h.command.calibrationRun = 0;
    dStep(h);
    if (h.logic.mode != DispenserMode::Normal) ok = false;

    reportCheck("D22", ok, "Refused while the seeder reports the wheel turning, duty 0, then Normal");
}

// ---------------------------------------------------------------------------
// D23/D24/D25 - what ends a running calibration
// ---------------------------------------------------------------------------

static void testD23()
{
    DHarness h;
    bool ok = dCalibrateFor(h, 20);

    h.seederAlive   = true;
    h.haveTelemetry = true;
    h.telemetry.groundSpeedMmS = 1000;
    h.telemetry.wheelTurning   = 1;
    dStep(h);
    if (h.logic.mode != DispenserMode::Refused) ok = false;
    if (h.out.motorPermille != 0) ok = false;

    reportCheck("D23", ok, "the machine starts moving: Refused with duty 0 in that step");
}

static void testD24()
{
    DHarness h;
    bool ok = dCalibrateFor(h, 20);

    h.command.calibrationRun = 0;
    dStep(h);
    if (h.logic.mode != DispenserMode::Normal) ok = false;
    if (h.out.motorPermille != 0) ok = false;

    reportCheck("D24", ok, "the request drops: Normal with duty 0 in that step");
}

static void testD25()
{
    DHarness h;
    bool ok = dCalibrateFor(h, 20);

    h.tractorAlive = false;
    dStep(h);
    if (h.logic.mode != DispenserMode::Normal) ok = false;
    if (h.out.motorPermille != 0) ok = false;

    // The tractor comes back with the request still raised: no run, because
    // the request never went away.
    h.tractorAlive = true;
    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Normal) ok = false;
    }

    h.command.calibrationRun = 0;
    dStep(h);
    h.command.calibrationRun = 1;
    dStep(h);
    if (h.logic.mode != DispenserMode::Calibrating) ok = false;

    reportCheck("D25", ok, "lost tractor aborts; the still-raised request does not restart it");
}

// ---------------------------------------------------------------------------
// D26 - a request already raised at boot starts nothing
// ---------------------------------------------------------------------------

static void testD26()
{
    DHarness h;
    dFresh(h);
    h.tractorAlive = true;
    h.haveCommand  = true;
    h.command.dispenserEnabled = 1;
    h.command.doseKgPerHa      = 40;
    h.command.gramsPer100Rev   = 500;
    h.command.calibrationRun   = 1;      // never seen as 0
    h.command.upTimeMs         = h.nowMs;

    bool ok = true;
    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Normal) ok = false;
        if (h.out.motorPermille != 0) ok = false;
    }

    h.command.calibrationRun = 0;
    dStep(h);
    h.command.calibrationRun = 1;
    h.shaftRPM = CALIBRATION_RPM;
    dStep(h);
    if (h.logic.mode != DispenserMode::Calibrating) ok = false;

    reportCheck("D26", ok, "a request raised at boot does nothing until it drops and rises");
}

// ---------------------------------------------------------------------------
// D27 - a calibration request is ignored while clogged
// ---------------------------------------------------------------------------

static void testD27()
{
    DHarness h;
    bool ok = dReachClogged(h);

    h.command.calibrationRun = 0;
    dStep(h);
    h.command.calibrationRun = 1;

    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Clogged) ok = false;
    }

    reportCheck("D27", ok, "Clogged ignores the calibration request");
}

// ---------------------------------------------------------------------------
// D28/D29 - anti-windup
// ---------------------------------------------------------------------------

static void testD28()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    h.telemetry.groundSpeedMmS = 3000;
    h.shaftRPM = 200;

    bool ok = true;
    for (int i = 0; i < 100; i++) {
        dStep(h);
        if (h.out.motorPermille != 1000) ok = false;
        if (h.logic.mode == DispenserMode::Clogged) ok = false;
    }

    h.telemetry.groundSpeedMmS = 500;
    h.shaftRPM = 96;

    uint16_t limit = (uint16_t)(96 * 1000 / MOTOR_MAX_RPM + 100);
    uint16_t worst = 0;
    for (int i = 0; i < 2; i++) {
        dStep(h);
        if (h.out.motorPermille > worst) worst = h.out.motorPermille;
    }
    if (worst > limit) ok = false;

    reportCheck("D28", ok, "1000 permille at the high target, %u permille after the change (limit %u)",
                (unsigned)worst, (unsigned)limit);
}

static void testD29()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    h.telemetry.groundSpeedMmS = 200;
    h.command.doseKgPerHa      = 10;
    h.command.gramsPer100Rev   = 1000;
    h.shaftRPM = 25;

    bool ok = true;
    for (int i = 0; i < 600; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) ok = false;
    }

    dDefaults(h);          // back to 192 RPM, shaft stopped
    h.shaftRPM = 0;
    dStep(h);

    // The row's limit is min(1000, 192 x 1000 / MOTOR_MAX_RPM + MOTOR_KP x 192) - 50.
    // Starting from MOTOR_MAX_RPM instead of 1000 makes the limit 280 permille,
    // which a controller with no anti-windup (~343) would pass - exactly what
    // this test exists to catch.
    double floorValue = 1000.0;
    double byFormula = 192.0 * 1000.0 / (double)MOTOR_MAX_RPM + MOTOR_KP * 192.0;
    if (byFormula < floorValue) floorValue = byFormula;
    floorValue -= 50.0;

    if ((double)h.out.motorPermille < floorValue) ok = false;

    reportCheck("D29", ok, "after 60 s at a 4 RPM target the duty recovers to %u (want >= %.0f)",
                (unsigned)h.out.motorPermille, floorValue);
}

// ---------------------------------------------------------------------------
// D30/D31 - link losses
// ---------------------------------------------------------------------------

static void testD30()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    for (int i = 0; i < 10; i++) {
        dStep(h);
        dFollow(h);
    }

    bool ok = true;
    h.seederAlive = false;
    dStep(h);
    if (h.logic.fault != DispenserFault::NoSpeedData) ok = false;
    if (h.out.motorPermille != 0) ok = false;

    for (int i = 0; i < 4; i++) dStep(h);

    h.seederAlive = true;
    bool dutyUp = false;
    for (int i = 0; i < 2; i++) {
        dStep(h);
        if (h.out.motorPermille > 0) dutyUp = true;
    }
    if (!dutyUp) ok = false;
    if (h.logic.fault != DispenserFault::None) ok = false;

    reportCheck("D30", ok, "no seeder: NoSpeedData and duty 0; back: metering within 2 steps");
}

static void testD31()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    for (int i = 0; i < 10; i++) {
        dStep(h);
        dFollow(h);
    }

    bool ok = true;
    h.tractorAlive = false;
    for (int i = 0; i < 100; i++) {
        dStep(h);
        dFollow(h);
        if (h.logic.mode != DispenserMode::Normal) ok = false;
        if (h.logic.fault != DispenserFault::None) ok = false;
        if (h.logic.targetRPM != 192) ok = false;
        if (h.out.motorPermille == 0) ok = false;
    }

    reportCheck("D31", ok, "the last dose keeps metering with the tractor gone");
}

// ---------------------------------------------------------------------------
// D33 - a clog resets the controller
// ---------------------------------------------------------------------------

static void testD33()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    h.shaftRPM = 70;

    bool ok = true;
    for (int i = 0; i < 80; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) ok = false;
    }

    h.shaftRPM = 0;
    for (int i = 0; i < 40; i++) {
        dStep(h);
        if (h.logic.mode == DispenserMode::Clogged) break;
    }
    if (h.logic.mode != DispenserMode::Clogged) ok = false;

    dSetCounters(h, 1, 0);
    dStep(h);
    if (h.logic.mode != DispenserMode::Normal) ok = false;

    h.shaftRPM = 192;
    bool first = true;
    uint16_t firstDuty = 0;
    uint16_t limit = (uint16_t)(192 * 1000 / MOTOR_MAX_RPM + 60);
    for (int i = 0; i < 5; i++) {
        dStep(h);
        if (first && h.out.motorPermille > 0) {
            first = false;
            firstDuty = h.out.motorPermille;
        }
    }
    if (first) ok = false;
    if (firstDuty > limit) ok = false;

    reportCheck("D33", ok, "first duty after the clear is %u (limit %u) - the integral was reset",
                (unsigned)firstDuty, (unsigned)limit);
}

// ---------------------------------------------------------------------------
// D34 - a target of 0 RPM is not a fault
// ---------------------------------------------------------------------------

static void testD34()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    h.telemetry.groundSpeedMmS = 1;
    h.command.doseKgPerHa      = 1;
    h.command.gramsPer100Rev   = 99999;
    h.shaftRPM = 0;

    bool ok = true;
    for (int i = 0; i < 30; i++) {
        dStep(h);
        if (h.out.motorPermille != 0) ok = false;
        if (h.logic.fault != DispenserFault::None) ok = false;
        if (h.logic.mode == DispenserMode::Clogged) ok = false;
    }

    reportCheck("D34", ok, "a 0 RPM target gives duty 0, fault None, never Clogged");
}

// ---------------------------------------------------------------------------
// D35 - a negative integral must not hold the motor off after slowing down
// ---------------------------------------------------------------------------

static void testD35()
{
    DHarness h;
    dFresh(h);
    dDefaults(h);
    h.telemetry.groundSpeedMmS = 1719;   // asks for 330 RPM, the motor's maximum
    h.shaftRPM = 363;                    // ... and the motor runs faster than asked

    bool ok = true;
    for (int i = 0; i < 300; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Normal) ok = false;
    }
    if (h.logic.targetRPM != MOTOR_MAX_RPM) ok = false;

    // The stimulus has to have happened: turning faster than asked drives the
    // integral negative, and that is what a slow-down then has to survive.
    float integralBefore = h.logic.integralTerm;

    // Slow down while still seeding - 104 mm/s asks for 19 RPM - and the shaft
    // is stopped. Without the clamp in computeDuty() the leftover negative
    // integral cancels the whole feed-forward and the duty stays at 0, so the
    // motor never restarts.
    h.telemetry.groundSpeedMmS = 104;
    h.shaftRPM = 0;

    uint16_t worst = 0xFFFF;
    for (int i = 0; i < 3; i++) {
        dStep(h);
        if (h.logic.mode != DispenserMode::Normal) ok = false;
        if (h.out.motorPermille < worst) worst = h.out.motorPermille;
    }

    if (worst < MOTOR_MIN_RUNNING_PERMILLE) ok = false;

    reportCheck("D35", ok, "integral %.0f before the change; at a %u RPM target the duty is at least "
                "%u permille in each of the next 3 steps (lowest %u)",
                (double)integralBefore, (unsigned)h.logic.targetRPM,
                (unsigned)MOTOR_MIN_RUNNING_PERMILLE, (unsigned)worst);
}

// ---------------------------------------------------------------------------

static void runLogicTests()
{
    Serial.println();
    Serial.println("--- D: logic tests (no hardware) ---");

    d32Ok = true;
    d32Count = 0;

    testD01();
    testD02();
    testD03();
    testD04();
    testD05();
    testD06();
    testD07();
    testD08();
    testD09();
    testD10();
    testD11();
    testD12();
    testD13();
    testD14();
    testD15();
    testD16();
    testD17();
    testD18();
    testD19();
    testD20();
    testD21();
    testD22();
    testD23();
    testD24();
    testD25();
    testD26();
    testD27();
    testD28();
    testD29();
    testD30();
    testD31();
    testD33();
    testD34();
    testD35();

    reportCheck("D32", d32Ok && d32Count > 0,
                "dispenserFillStatus matched the step over %u checks", (unsigned)d32Count);
}
