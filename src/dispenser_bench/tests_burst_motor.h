#pragma once

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_io.h"
#include "bench_inject.h"
#include "bench_report.h"

// ---------------------------------------------------------------------------
// U tests: burst metering with the real motor and encoder - what the B tests
// can only assume: how fast the motor really turns in a burst, how far it runs
// on after a burst is cut, the calibration run, and a stuck auger - a failed
// burst, then the unclog it leads to. Real time and simulated packets, like the
// M tests, with burst metering switched on whatever DISPENSER_BURST_MODE says,
// at the production burst PWM (BURST_PWM_FRACTION). The angle factor's angle
// follows the PWM, so the ground speeds below keep the motor as busy at any
// PWM, and the time limits follow it.
//
// Coupling off and a free shaft, like the M tests. Run on the fitted module
// with the auger coupled and fertilizer in the hopper (bench image flashed, a
// bucket under the outlet), U01 is the one worth repeating: its burst RPM is
// what the loaded motor really turns at, which is what decides how fast the
// machine can go (CLAUDE.md, Dispenser behaviour -> Burst metering).
// ---------------------------------------------------------------------------

// Watched by uPump(): bursts, and what the shaft does while one runs.
struct UWatch {
    uint32_t bursts        = 0;
    bool     motorOn       = false;
    uint32_t onMs          = 0;       // time with the motor driven...
    uint32_t onEdges       = 0;       // ...and the edges turned in it
    double   worstOvershoot = 0.0;    // edges past the count, seen between bursts
    uint32_t lastEdges     = 0;
    uint32_t lastMs        = 0;
};

// benchPump() for the given time, keeping the watch. False on an abort.
static bool uPump(uint32_t ms, UWatch &w)
{
    uint32_t start = millis();
    w.lastEdges = encoderEdges();
    w.lastMs    = start;

    while (millis() - start < ms) {
        if (!benchPump()) return false;

        uint32_t now   = millis();
        uint32_t edges = encoderEdges();
        if (w.motorOn) {
            w.onMs    += now - w.lastMs;
            w.onEdges += edges - w.lastEdges;
        }
        w.lastEdges = edges;
        w.lastMs    = now;

        bool on = out.motorPermille > 0 && out.motorForward;   // a burst, not the push back after one
        if (on && !w.motorOn) w.bursts++;
        w.motorOn = on;

        if (!logic.burstRunning && logic.mode == DispenserMode::Normal && logic.ledgerValid) {
            double over = -(double)logic.owedRevolutions * (double)ENCODER_EDGES_PER_REV;
            if (over > w.worstOvershoot) w.worstOvershoot = over;
        }
        delay(1);
    }
    return true;
}

// The logic's failed-burst count when U04 cuts the encoder. A file static,
// because pollUntil() takes plain functions.
static uint8_t uFailBase = 0;

static double uBurstRPM(const UWatch &w)
{
    if (w.onMs == 0) return 0.0;
    return (double)w.onEdges * 60000.0 / ((double)w.onMs * (double)ENCODER_EDGES_PER_REV);
}

// Turns per pulse at the injector's settings, independent of the logic: the
// angle factor, in thousandths of what BURST_ANGLE_REFERENCE_RPM at the burst
// PWM turns between two pulses at BURST_ANGLE_REFERENCE_SPEED_MM_S.
static double uRevsPerPulse()
{
    double pulseSeconds = (double)injector.tractor.wheelMmPerPulse / (double)BURST_ANGLE_REFERENCE_SPEED_MM_S;
    double refRevs      = (double)BURST_ANGLE_REFERENCE_RPM * ((double)BURST_PERMILLE / 1000.0) / 60.0 *
                          pulseSeconds;
    return (double)injector.tractor.burstAngleFactor / 1000.0 * refRevs;
}

// What the burst PWM turns a free motor at, near enough: MOTOR_MAX_RPM at full
// PWM, in proportion below it.
static double uBurstNominalRPM()
{
    return (double)BURST_PERMILLE * (double)MOTOR_MAX_RPM / 1000.0;
}

// The ground speed that keeps the motor busy for about `share` of the time at
// the burst PWM, with the injector's settings: the pulses a second times the
// turns each, against the turns a second the motor makes.
static uint16_t uGroundFor(double share)
{
    return (uint16_t)(share * uBurstNominalRPM() / 60.0 / uRevsPerPulse() *
                      (double)injector.tractor.wheelMmPerPulse);
}

static void runBurstMotorTests()
{
    Serial.println();
    Serial.println("--- U: burst metering (motor, free shaft) ---");

    // U01 - 20 s of ground that keeps the motor busy about 60 % of the time
    // (1031 mm/s at full PWM), then the machine stops: every pulse's turns
    // delivered, one burst each, and the shaft still once the last burst is
    // done. The RPM the bursts turn at is printed, not judged: the PWM is not a
    // speed anything holds.
    if (abortRequested) return;
    {
        TestMarker tm("U01");
        MotorGuard guard;

        if (!benchPrepare(true)) { reportLine("U01", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.seeder.groundSpeedMmS = uGroundFor(0.6);
        injectorStart(injector, millis());
        uint32_t edges0 = encoderEdges();

        UWatch w;
        if (!uPump(20000, w)) { reportLine("U01", Outcome::Aborted, "aborted by key"); return; }

        // The machine stops. What the last pulse owes is turned, then nothing.
        // Idle means the logic has the final count as well: a pulse the seeder
        // counted just before the stop is still owed a burst.
        injector.seeder.groundSpeedMmS = 0;
        injector.seeder.wheelTurning   = 0;
        injectorSchedule(injector, millis());
        uint32_t tStop = millis();
        bool idle = pollUntil(3000, []() {
            return logic.ledgerPulses == injector.seeder.wheelPulses &&
                   out.motorPermille == 0 && !logic.burstRunning;
        });
        if (abortRequested) { reportLine("U01", Outcome::Aborted, "aborted by key"); return; }
        uint32_t idleAfter = millis() - tStop;
        if (!runFor(500)) { reportLine("U01", Outcome::Aborted, "aborted by key"); return; }   // the brake

        Window still;
        if (!runWindow(2000, still)) { reportLine("U01", Outcome::Aborted, "aborted by key"); return; }

        uint32_t pulses = injector.seeder.wheelPulses;
        double   want   = (double)pulses * uRevsPerPulse();
        double   turns  = (double)(encoderEdges() - edges0) / (double)ENCODER_EDGES_PER_REV;
        double   rpm    = uBurstRPM(w);

        // Never short; over by at most what the last burst runs on as it brakes.
        bool addsUp = turns >= want - 0.01 && turns <= want + 0.5;
        bool ok = addsUp && idle && (still.aEdges <= 2) && (w.bursts + 1 >= pulses) && (w.bursts <= pulses)
               && !gSawClogged && !gSawBuzzingDuty && logic.fault == DispenserFault::None;

        reportCheck("U01", ok, "%lu pulses at %u mm/s: %.2f turns (want %.2f, up to +0.5), %lu bursts at %.0f RPM, "
                    "overshoot up to %.0f edges; stopped: motor off after %lu ms, %lu edges in the next 2 s",
                    (unsigned long)pulses, (unsigned)uGroundFor(0.6), turns, want, (unsigned long)w.bursts, rpm,
                    w.worstOvershoot, (unsigned long)idleAfter, (unsigned long)still.aEdges);
        if (rpm > 0.0) {
            Serial.printf("     bursts at %u permille (BURST_PWM_FRACTION %.2f) turn this shaft at %.0f RPM: with "
                          "the auger loaded, that is the most the dose can ask for\n",
                          (unsigned)BURST_PERMILLE, (double)BURST_PWM_FRACTION, rpm);
        }
    }

    // U02 - faster than the motor, then slower again: the fast speed asks for
    // twice the turns the burst PWM makes, the slow one keeps it busy half the
    // time.
    if (abortRequested) return;
    {
        TestMarker tm("U02");
        MotorGuard guard;

        if (!benchPrepare(true)) { reportLine("U02", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        uint16_t fast = uGroundFor(2.0);
        injector.seeder.groundSpeedMmS = fast;
        injectorStart(injector, millis());

        uint32_t t0 = millis();
        bool raised = pollUntil(6000, []() { return logic.fault == DispenserFault::OverSpeed; });
        if (abortRequested) { reportLine("U02", Outcome::Aborted, "aborted by key"); return; }
        uint32_t tRaised = millis() - t0;

        // Once raised, it holds while the machine stays too fast.
        bool steady = true;
        uint32_t tHold = millis();
        while (raised && millis() - tHold < 3000) {
            if (!benchPump()) { reportLine("U02", Outcome::Aborted, "aborted by key"); return; }
            if (logic.fault != DispenserFault::OverSpeed) steady = false;
            delay(1);
        }

        // The backlog left over is worked off with half the motor to spare;
        // the alarm clears at the first pulse that finds it caught up.
        uint16_t slow = uGroundFor(0.5);
        injector.seeder.groundSpeedMmS = slow;
        injectorSchedule(injector, millis());
        uint32_t t1 = millis();
        bool cleared = pollUntil(15000, []() { return logic.fault == DispenserFault::None; });
        if (abortRequested) { reportLine("U02", Outcome::Aborted, "aborted by key"); return; }
        uint32_t tCleared = millis() - t1;

        bool ok = raised && steady && cleared && !gSawClogged;
        reportCheck("U02", ok, "%u mm/s: OverSpeed after %lu ms (limit 6000), held for 3 s %s; %u mm/s: "
                    "cleared after %lu ms (limit 15000); never Clogged %s",
                    (unsigned)fast, (unsigned long)tRaised, steady ? "yes" : "NO", (unsigned)slow,
                    (unsigned long)tCleared, gSawClogged ? "NO" : "yes");
    }

    // U03 - the calibration run: CALIBRATION_PULSES real bursts spaced as at
    // CALIBRATION_SPEED_MM_S, each one pulse's angle, stopped on the exact
    // count. 2.5 m per pulse puts the pulses 1.5 s apart and an angle factor of
    // 889 asks for 4 turns each at full PWM (2 at half), so a free shaft makes
    // that many separate bursts with pauses between them.
    if (abortRequested) return;
    {
        TestMarker tm("U03");
        MotorGuard guard;

        if (!benchPrepare(true)) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.wheelMmPerPulse  = 2500;
        injector.tractor.burstAngleFactor = 889;
        injector.tractor.calibrationRun   = 0;
        injector.tractor.sending         = true;
        injectorSchedule(injector, millis());
        if (!runFor(1000)) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool started = pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }

        // The run takes CALIBRATION_PULSES pulse intervals, or as long as all
        // its turns take if the motor cannot keep up; the limit allows for a
        // motor at half the burst PWM's nominal speed.
        double   wantTurns = (double)CALIBRATION_PULSES * uRevsPerPulse();
        uint32_t wantEdges = (uint32_t)(wantTurns * (double)ENCODER_EDGES_PER_REV + 0.5);
        uint32_t interval  = (uint32_t)injector.tractor.wheelMmPerPulse * 1000 / CALIBRATION_SPEED_MM_S;
        uint32_t limit     = (uint32_t)CALIBRATION_PULSES * interval +
                             (uint32_t)(2.0 * wantTurns * 60000.0 / uBurstNominalRPM()) + 5000;
        uint32_t start    = millis();
        uint32_t bursts   = 0;
        bool     wasOn    = false;
        bool     done     = false;
        while (millis() - start < limit) {
            if (!benchPump()) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }
            bool on = out.motorPermille > 0 && out.motorForward;
            if (on && !wasOn) bursts++;
            wasOn = on;
            if (logic.mode != DispenserMode::Calibrating) {
                done = true;
                break;
            }
            delay(1);
        }
        uint32_t took = millis() - start;
        done = done && (logic.mode == DispenserMode::CalibrationDone);

        if (!runFor(1000)) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }

        uint32_t turned  = encoderEdges() - logic.calibrationStartEdges;
        bool     edgesOk = turned + 1 >= wantEdges && turned <= wantEdges + ENCODER_EDGES_PER_REV / 2;
        bool     ok      = started && done && edgesOk && logic.progress == 100 && out.motorPermille == 0
                        && !gSawBuzzingDuty && bursts >= 1 && bursts <= CALIBRATION_PULSES;

        reportCheck("U03", ok, "calibration as %u pulses, one every %lu ms: started %s, %lu bursts (%u on a free "
                    "shaft that keeps up), Done after %lu ms (limit %lu), %lu edges (want %lu, up to +%lu), "
                    "progress 100 %s",
                    (unsigned)CALIBRATION_PULSES, (unsigned long)interval, started ? "yes" : "NO",
                    (unsigned long)bursts, (unsigned)CALIBRATION_PULSES, (unsigned long)took,
                    (unsigned long)limit, (unsigned long)turned, (unsigned long)wantEdges,
                    (unsigned long)(ENCODER_EDGES_PER_REV / 2), (logic.progress == 100) ? "yes" : "NO");
    }

    // U04 - a stuck auger: channel A detached while the motor runs, so the
    // logic sees a shaft that has stopped while driven (below full PWM, full
    // PWM first - the stall push). The burst fails after BURST_FAIL_MS - motor
    // off, never Clogged - and with the shaft still "stuck" and the ground
    // moving, BURST_FAILS_BEFORE_UNCLOG failures in a row start the unclog
    // sequence by itself. Channel A back, metering goes on after it.
    if (abortRequested) return;
    {
        TestMarker tm("U04");
        MotorGuard guard;

        if (!benchPrepare(true)) { reportLine("U04", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.seeder.groundSpeedMmS = uGroundFor(0.6);
        injectorStart(injector, millis());

        if (!runFor(2000)) { reportLine("U04", Outcome::Aborted, "aborted by key"); return; }
        bool inBurst = pollUntil(3000, []() { return out.motorPermille > 0; });
        if (abortRequested) { reportLine("U04", Outcome::Aborted, "aborted by key"); return; }
        if (!runFor(150)) { reportLine("U04", Outcome::Aborted, "aborted by key"); return; }

        uFailBase = logic.burstFailures;
        uint32_t t0 = millis();
        encoderEnd();
        bool failed = pollUntil(BURST_FAIL_MS + 400, []() { return logic.burstFailures != uFailBase; });
        uint32_t msToFail = millis() - t0;
        bool dutyZero = (out.motorPermille == 0);
        if (abortRequested) { encoderBegin(); reportLine("U04", Outcome::Aborted, "aborted by key"); return; }

        // Still stuck: the rest of the run of failures, then the unclog.
        uint32_t interval = (uint32_t)injector.tractor.wheelMmPerPulse * 1000 / injector.seeder.groundSpeedMmS;
        uint32_t limit    = (uint32_t)BURST_FAILS_BEFORE_UNCLOG * (BURST_FAIL_MS + interval) + 1000;
        bool unclogging = pollUntil(limit, []() { return logic.mode == DispenserMode::AutoUnclogging; });
        uint8_t failures = (uint8_t)(logic.burstFailures - uFailBase);
        encoderBegin();
        if (abortRequested) { reportLine("U04", Outcome::Aborted, "aborted by key"); return; }

        bool back = pollUntil(UNCLOG_TOTAL_MS + 500, []() { return logic.mode == DispenserMode::Normal; });
        if (abortRequested) { reportLine("U04", Outcome::Aborted, "aborted by key"); return; }

        bool ok = inBurst && failed && msToFail >= BURST_FAIL_MS && msToFail <= BURST_FAIL_MS + 400 && dutyZero
               && unclogging && failures == BURST_FAILS_BEFORE_UNCLOG && back && !gSawClogged;
        reportCheck("U04", ok, "encoder cut mid-burst: failed after %lu ms (want %lu..%lu), duty 0 %s; unclogging "
                    "by itself after %u failures (want %u) %s; metering after it %s; never Clogged %s",
                    (unsigned long)msToFail, (unsigned long)BURST_FAIL_MS, (unsigned long)(BURST_FAIL_MS + 400),
                    dutyZero ? "yes" : "NO", (unsigned)failures, (unsigned)BURST_FAILS_BEFORE_UNCLOG,
                    unclogging ? "yes" : "NO", back ? "yes" : "NO", gSawClogged ? "NO" : "yes");
    }
}
