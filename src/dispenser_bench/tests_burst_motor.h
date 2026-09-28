#pragma once

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_io.h"
#include "bench_inject.h"
#include "bench_report.h"

// ---------------------------------------------------------------------------
// U tests: burst metering with the real motor and encoder - what the B tests
// can only assume: how fast the motor really turns in a burst, how far it runs
// on after a burst is cut, the calibration run, and a clog in the middle of a
// burst. Real time and simulated packets, like the M tests, with burst metering
// switched on whatever DISPENSER_BURST_MODE says, at the production burst PWM
// (BURST_PWM_FRACTION): the ground speeds and time limits below follow it.
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

        bool on = out.motorPermille > 0;
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

static double uBurstRPM(const UWatch &w)
{
    if (w.onMs == 0) return 0.0;
    return (double)w.onEdges * 60000.0 / ((double)w.onMs * (double)ENCODER_EDGES_PER_REV);
}

// Turns per pulse at the injector's settings, independent of the logic.
static double uRevsPerPulse()
{
    return (double)WORKING_WIDTH_CM * (double)injector.tractor.doseKgPerHa /
           (10.0 * (double)injector.tractor.gramsPer100Rev) *
           (double)injector.tractor.wheelMmPerPulse / 1000.0;
}

// What the burst PWM turns a free motor at, near enough: MOTOR_MAX_RPM at full
// PWM, in proportion below it.
static double uBurstNominalRPM()
{
    return (double)BURST_PERMILLE * (double)MOTOR_MAX_RPM / 1000.0;
}

// The ground speed that keeps the motor busy for about `share` of the time at
// the burst PWM, with the injector's default dose and calibration: at 1000 mm/s
// they ask for independentRPM(1000, 40, 500) = 192 RPM.
static uint16_t uGroundFor(double share)
{
    return (uint16_t)(share * uBurstNominalRPM() / independentRPM(1000, 40, 500) * 1000.0);
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

    // U02 - faster than the motor, then slower again: 3000 mm/s asks for 576 RPM,
    // more than any burst speed; the slow speed keeps it busy half the time.
    if (abortRequested) return;
    {
        TestMarker tm("U02");
        MotorGuard guard;

        if (!benchPrepare(true)) { reportLine("U02", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.seeder.groundSpeedMmS = 3000;
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
        reportCheck("U02", ok, "3000 mm/s: OverSpeed after %lu ms (limit 6000), held for 3 s %s; %u mm/s: "
                    "cleared after %lu ms (limit 15000); never Clogged %s",
                    (unsigned long)tRaised, steady ? "yes" : "NO", (unsigned)slow, (unsigned long)tCleared,
                    gSawClogged ? "NO" : "yes");
    }

    // U03 - the calibration run at the burst PWM, stopped on the exact count.
    if (abortRequested) return;
    {
        TestMarker tm("U03");
        MotorGuard guard;

        if (!benchPrepare(true)) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.calibrationRun = 0;
        injector.tractor.sending        = true;
        injectorSchedule(injector, millis());
        if (!runFor(1000)) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool started = pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }

        // 100 turns at the burst PWM - 18 s at 330 RPM, 36 s at half PWM - and
        // the limit allows for a motor at half that.
        uint32_t limit = (uint32_t)(2.0 * (double)CALIBRATION_REVOLUTIONS * 60000.0 / uBurstNominalRPM()) + 5000;
        uint32_t start = millis();
        bool done = pollUntil(limit, []() { return logic.mode != DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }
        uint32_t took = millis() - start;
        done = done && (logic.mode == DispenserMode::CalibrationDone);

        if (!runFor(1000)) { reportLine("U03", Outcome::Aborted, "aborted by key"); return; }

        uint32_t turned  = encoderEdges() - logic.calibrationStartEdges;
        bool     edgesOk = turned >= CALIBRATION_TOTAL_EDGES && turned <= CALIBRATION_TOTAL_EDGES + ENCODER_EDGES_PER_REV / 2;
        bool     ok      = started && done && edgesOk && logic.progress == 100 && out.motorPermille == 0
                        && !gSawBuzzingDuty;

        reportCheck("U03", ok, "calibration at the burst PWM: started %s, Done after %lu ms (limit %lu), "
                    "%lu edges (want %lu..%lu), progress 100 %s",
                    started ? "yes" : "NO", (unsigned long)took, (unsigned long)limit, (unsigned long)turned,
                    (unsigned long)CALIBRATION_TOTAL_EDGES,
                    (unsigned long)(CALIBRATION_TOTAL_EDGES + ENCODER_EDGES_PER_REV / 2),
                    (logic.progress == 100) ? "yes" : "NO");
    }

    // U04 - a clog in the middle of a burst: channel A detached while the motor
    // runs, so the logic sees a shaft that has stopped while driven. Below full
    // PWM that first brings full PWM (the stall push), then the clog.
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

        uint32_t t0 = millis();
        encoderEnd();
        bool clogged = pollUntil(CLOG_DETECT_MS + 400, []() { return logic.mode == DispenserMode::Clogged; });
        uint32_t msToClog = millis() - t0;
        encoderBegin();
        if (abortRequested) { reportLine("U04", Outcome::Aborted, "aborted by key"); return; }

        bool dutyZero = (out.motorPermille == 0);
        if (!runFor(2000)) { reportLine("U04", Outcome::Aborted, "aborted by key"); return; }
        bool stays = (logic.mode == DispenserMode::Clogged) && (out.motorPermille == 0);

        bool ok = inBurst && clogged && msToClog >= CLOG_DETECT_MS && msToClog <= CLOG_DETECT_MS + 400
               && dutyZero && stays;
        reportCheck("U04", ok, "encoder cut mid-burst: Clogged after %lu ms (want %lu..%lu), duty 0 %s, "
                    "stays Clogged with the ground moving %s",
                    (unsigned long)msToClog, (unsigned long)CLOG_DETECT_MS,
                    (unsigned long)(CLOG_DETECT_MS + 400), dutyZero ? "yes" : "NO", stays ? "yes" : "NO");
    }
}
