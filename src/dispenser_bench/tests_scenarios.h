#pragma once

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_io.h"
#include "bench_inject.h"
#include "bench_report.h"

// ---------------------------------------------------------------------------
// M, C and K tests: real time, real motor and encoder, simulated packets, and
// automatic PASS/FAIL from the shaft measured independently of the logic's own
// reporting (encoderEdges() deltas, and channel B for the direction).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// M - metering and links
// ---------------------------------------------------------------------------

struct M01Case {
    const char *id;
    const char *label;
    bool     cmd;
    bool     tel;
    uint8_t  enabled;
    uint16_t dose;
    uint32_t grams;
    uint16_t speed;
    uint8_t  turning;
};

static void m01Run(const M01Case &c)
{
    TestMarker tm(c.id);
    MotorGuard guard;

    if (!benchPrepare()) { reportLine(c.id, Outcome::Aborted, "aborted by key"); return; }
    injectorDefaults(injector);

    if (c.cmd) {
        injector.tractor.dispenserEnabled = c.enabled;
        injector.tractor.doseKgPerHa      = c.dose;
        injector.tractor.gramsPer100Rev   = c.grams;
        injector.tractor.sending          = true;
    }
    if (c.tel) {
        injector.seeder.groundSpeedMmS = c.speed;
        injector.seeder.wheelTurning   = c.turning;
        injector.seeder.sending        = true;
    }
    injectorSchedule(injector, millis());

    // Reset here too: this test watches only part of the run.
    gRunMaxDuty = 0;

    if (!runFor(1000)) { reportLine(c.id, Outcome::Aborted, "aborted by key"); return; }

    Window w;
    if (!runWindow(2000, w)) { reportLine(c.id, Outcome::Aborted, "aborted by key"); return; }

    bool ok = (w.aEdges <= 2) && (gRunMaxDuty == 0);

    reportCheck(c.id, ok, "%s: %lu A edges after the first second (want <= 2), highest duty %u",
                c.label, (unsigned long)w.aEdges, (unsigned)gRunMaxDuty);
}

static void runMeteringTests()
{
    Serial.println();
    Serial.println("--- M: metering and links (motor, free shaft) ---");

    const M01Case cases[] = {
        {"M01a", "(a) no packets",            false, false, 1, 40, 500, 1000, 1},
        {"M01b", "(b) command only",          true,  false, 1, 40, 500, 1000, 1},
        {"M01c", "(c) telemetry only",        false, true,  1, 40, 500, 1500, 1},
        {"M01d", "(d) defaults, enabled 0",   true,  true,  0, 40, 500, 1000, 1},
        {"M01e", "(e) dose 0",                true,  true,  1,  0, 500, 1000, 1},
        {"M01f", "(f) 0 g/100 rev",           true,  true,  1, 40,   0, 1000, 1},
        {"M01g", "(g) speed 0, not turning",  true,  true,  1, 40, 500,    0, 0},
        {"M01h", "(h) turning, speed 0",      true,  true,  1, 40, 500,    0, 1},
        {"M01i", "(i) speed 1500, not turning", true, true, 1, 40, 500, 1500, 0},
    };

    for (uint8_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        if (abortRequested) return;      // an abort ends the whole command
        m01Run(cases[i]);
    }
    if (abortRequested) return;

    // M02 - the headline metering check.
    {
        TestMarker tm("M02");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M02", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injectorStart(injector, millis());

        if (!runFor(2000)) { reportLine("M02", Outcome::Aborted, "aborted by key"); return; }
        Window w;
        if (!runWindow(1000, w)) { reportLine("M02", Outcome::Aborted, "aborted by key"); return; }

        double rpm  = windowRPM(w);
        double want = independentRPM(1000, 40, 500);
        double tol  = want * 0.10;
        if (tol < 5.0) tol = 5.0;

        int  dir   = windowDirection(w, 0.9);
        bool dirOk = (dir >= 0) && (!report.haveForwardLevel || dir == report.forwardALevel);

        bool ok = fabs(rpm - want) <= tol
               && logic.mode == DispenserMode::Normal
               && logic.fault == DispenserFault::None
               && dirOk;

        reportCheck("M02", ok, "mean %.1f RPM, target %.1f +-%.1f; %s, fault %s; direction %s",
                    rpm, want, tol, modeName(logic.mode), faultName(logic.fault),
                    (dir < 0) ? "unclear" : (dir == 1 ? "A high" : "A low"));
    }

    // M03 - the target moving up and down.
    if (abortRequested) return;
    {
        TestMarker tm("M03");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M03", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injectorStart(injector, millis());

        const uint16_t speeds[] = {500, 1500, 500};
        bool ok = true;
        double worst = 0.0;

        for (uint8_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++) {
            injector.seeder.groundSpeedMmS = speeds[i];
            injector.seeder.wheelTurning   = 1;
            injectorSchedule(injector, millis());   // measured from here, not from the send

            if (!runFor(500)) { reportLine("M03", Outcome::Aborted, "aborted by key"); return; }
            Window w;
            if (!runWindow(1000, w)) { reportLine("M03", Outcome::Aborted, "aborted by key"); return; }

            double rpm  = windowRPM(w);
            double want = independentRPM(speeds[i], 40, 500);
            double tol  = want * 0.15;
            double dev  = fabs(rpm - want);
            if (dev > worst) worst = dev;
            if (dev > tol) ok = false;
        }

        if (gSawClogged) ok = false;

        reportCheck("M03", ok, "500 -> 1500 -> 500 mm/s, worst error %.1f RPM (limit 15 %%), never Clogged",
                    worst);
    }

    // M04 - stop and go, ten times.
    if (abortRequested) return;
    {
        TestMarker tm("M04");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M04", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injectorStart(injector, millis());

        bool ok = true;
        bool dutyZeroInTime = true;
        bool stoppedInTime  = true;

        for (int cycle = 0; cycle < 10 && ok; cycle++) {
            injector.seeder.groundSpeedMmS = 1500;
            injector.seeder.wheelTurning   = 1;
            injectorSchedule(injector, millis());
            if (!runFor(2000)) { reportLine("M04", Outcome::Aborted, "aborted by key"); return; }

            injector.seeder.groundSpeedMmS = 0;
            injector.seeder.wheelTurning   = 0;
            injectorSchedule(injector, millis());

            uint32_t t0 = millis();
            if (!pollUntil(300, []() { return out.motorPermille == 0; })) {
                if (abortRequested) { reportLine("M04", Outcome::Aborted, "aborted by key"); return; }
                dutyZeroInTime = false;
            }

            uint32_t elapsed = millis() - t0;
            if (elapsed < 1000) {
                if (!runFor(1000 - elapsed)) { reportLine("M04", Outcome::Aborted, "aborted by key"); return; }
            }
            Window w;
            if (!runWindow(500, w)) { reportLine("M04", Outcome::Aborted, "aborted by key"); return; }
            if (windowRPM(w) >= 5.0) stoppedInTime = false;
        }

        ok = ok && dutyZeroInTime && stoppedInTime && !gSawClogged;

        reportCheck("M04", ok, "10 cycles: duty 0 within 300 ms %s, below 5 RPM within 1 s %s, never Clogged",
                    dutyZeroInTime ? "yes" : "NO", stoppedInTime ? "yes" : "NO");
    }

    // M05 - over speed.
    if (abortRequested) return;
    {
        TestMarker tm("M05");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M05", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injectorStart(injector, millis());
        injector.seeder.groundSpeedMmS = 3000;
        injectorSchedule(injector, millis());

        uint32_t t0 = millis();
        bool overSeen = false;
        while (millis() - t0 < 300) {
            if (!benchPump()) { reportLine("M05", Outcome::Aborted, "aborted by key"); return; }
            if (logic.fault == DispenserFault::OverSpeed && logic.targetRPM == MOTOR_MAX_RPM) {
                overSeen = true;
                break;
            }
            delay(1);
        }
        uint32_t elapsed = millis() - t0;
        if (elapsed < 5000) {
            if (!runFor(5000 - elapsed)) { reportLine("M05", Outcome::Aborted, "aborted by key"); return; }
        }

        injector.seeder.groundSpeedMmS = 1000;
        injectorSchedule(injector, millis());
        bool noneSeen = pollUntil(500, []() { return logic.fault == DispenserFault::None; });
        if (abortRequested) { reportLine("M05", Outcome::Aborted, "aborted by key"); return; }

        bool ok = overSeen && noneSeen && !gSawClogged;

        reportCheck("M05", ok, "OverSpeed and target %u within 300 ms %s, fault None within 500 ms %s, never Clogged",
                    (unsigned)MOTOR_MAX_RPM, overSeen ? "yes" : "NO", noneSeen ? "yes" : "NO");
    }

    // M06 - the settings changing while it meters.
    if (abortRequested) return;
    {
        TestMarker tm("M06");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M06", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        // 500 mm/s, so a 96 RPM target - doubling the dose asks for 192, which
        // is inside what the motor can do. At 1000 mm/s the doubled target is
        // 384 RPM, above MOTOR_MAX_RPM, so it would be clamped and the check
        // could never pass.
        injector.seeder.groundSpeedMmS = 500;
        injectorStart(injector, millis());

        if (!runFor(2000)) { reportLine("M06", Outcome::Aborted, "aborted by key"); return; }

        bool offOk   = false;
        bool onOk    = false;
        bool doseOk  = false;
        bool gramsOk = false;

        injector.tractor.dispenserEnabled = 0;
        injectorSchedule(injector, millis());
        offOk = pollUntil(300, []() { return out.motorPermille == 0; });
        if (abortRequested) { reportLine("M06", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.dispenserEnabled = 1;
        injectorSchedule(injector, millis());
        onOk = pollUntil(300, []() { return out.motorPermille > 0; });
        if (abortRequested) { reportLine("M06", Outcome::Aborted, "aborted by key"); return; }

        // Each target within +-1 of the independent formula, as the row says.
        injector.tractor.doseKgPerHa = 80;
        injectorSchedule(injector, millis());
        doseOk = pollUntil(300, []() {
            double want = independentRPM(500, 80, 500);
            return ((double)logic.targetRPM + 1.0 >= want) && ((double)logic.targetRPM <= want + 1.0);
        });
        if (abortRequested) { reportLine("M06", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.gramsPer100Rev = 1000;
        injectorSchedule(injector, millis());
        gramsOk = pollUntil(300, []() {
            double want = independentRPM(500, 80, 1000);
            return ((double)logic.targetRPM + 1.0 >= want) && ((double)logic.targetRPM <= want + 1.0);
        });
        if (abortRequested) { reportLine("M06", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("M06", offOk && onOk && doseOk && gramsOk,
                    "at 500 mm/s (target %.0f): enabled 0 ->1 %s/%s, dose 40 -> 80 -> target %.0f %s, "
                    "500 -> 1000 g -> target %.0f %s",
                    independentRPM(500, 40, 500), offOk ? "ok" : "NO", onOk ? "ok" : "NO",
                    independentRPM(500, 80, 500), doseOk ? "ok" : "NO",
                    independentRPM(500, 80, 1000), gramsOk ? "ok" : "NO");
    }

    // M07 - the seeder goes quiet.
    if (abortRequested) return;
    {
        TestMarker tm("M07");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M07", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injectorStart(injector, millis());

        if (!runFor(2000)) { reportLine("M07", Outcome::Aborted, "aborted by key"); return; }

        injector.seeder.sending = false;
        bool stopped = pollUntil(LINK_TIMEOUT_MS + 300, []() {
            return out.motorPermille == 0 && logic.fault == DispenserFault::NoSpeedData;
        });
        if (abortRequested) { reportLine("M07", Outcome::Aborted, "aborted by key"); return; }

        if (!runFor(2000)) { reportLine("M07", Outcome::Aborted, "aborted by key"); return; }

        injector.seeder.sending = true;
        injectorSchedule(injector, millis());
        bool back = pollUntil(500, []() { return out.motorPermille > 0; });
        if (abortRequested) { reportLine("M07", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("M07", stopped && back, "duty 0 and NoSpeedData within %lu ms %s, metering again %s",
                    (unsigned long)(LINK_TIMEOUT_MS + 300), stopped ? "yes" : "NO", back ? "yes" : "NO");
    }

    // M08 - the tractor goes quiet.
    if (abortRequested) return;
    {
        TestMarker tm("M08");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M08", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injectorStart(injector, millis());

        if (!runFor(2000)) { reportLine("M08", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.sending = false;
        if (!runFor(4000)) { reportLine("M08", Outcome::Aborted, "aborted by key"); return; }

        Window w;
        if (!runWindow(1000, w)) { reportLine("M08", Outcome::Aborted, "aborted by key"); return; }

        double rpm  = windowRPM(w);
        double want = independentRPM(1000, 40, 500);
        bool ok = fabs(rpm - want) <= 0.10 * want
               && logic.fault == DispenserFault::None
               && logic.mode == DispenserMode::Normal;

        reportCheck("M08", ok, "tractor silent for 5 s: %.1f RPM (target %.1f), fault %s, %s",
                    rpm, want, faultName(logic.fault), modeName(logic.mode));
    }

    // M09 - jittery packet timing, then a gap longer than the link timeout.
    if (abortRequested) return;
    {
        TestMarker tm("M09");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M09", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.seeder.jitter = true;      // 100 - 800 ms
        injectorStart(injector, millis());

        // This test watches only part of the run, so it resets for itself.
        gSawZeroDuty = false;
        if (!runFor(10000)) { reportLine("M09", Outcome::Aborted, "aborted by key"); return; }
        bool jitterOk = !gSawZeroDuty;

        injector.seeder.sending = false;
        bool gapOk = pollUntil(1300, []() { return out.motorPermille == 0; });
        if (abortRequested) { reportLine("M09", Outcome::Aborted, "aborted by key"); return; }
        if (!runFor(300)) { reportLine("M09", Outcome::Aborted, "aborted by key"); return; }

        injector.seeder.sending = true;
        injectorSchedule(injector, millis());
        bool back = pollUntil(500, []() { return out.motorPermille > 0; });
        if (abortRequested) { reportLine("M09", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("M09", jitterOk && gapOk && back,
                    "duty never 0 during 10 s of jitter %s, 0 during the gap %s, metering again %s",
                    jitterOk ? "yes" : "NO", gapOk ? "yes" : "NO", back ? "yes" : "NO");
    }

    // M10 - a target too low for the motor to hold, then a normal one.
    if (abortRequested) return;
    {
        TestMarker tm("M10");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M10", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.seeder.groundSpeedMmS   = 200;
        injector.tractor.doseKgPerHa     = 10;
        injector.tractor.gramsPer100Rev  = 1000;
        injectorStart(injector, millis());

        // Reset here too: this test watches only part of the run.
        gSawClogged     = false;
        gSawBuzzingDuty = false;

        if (!runFor(20000)) { reportLine("M10", Outcome::Aborted, "aborted by key"); return; }
        bool lowOk = !gSawClogged && !gSawBuzzingDuty;

        injector.seeder.groundSpeedMmS  = 1500;
        injector.tractor.doseKgPerHa    = 40;
        injector.tractor.gramsPer100Rev = 500;
        injectorSchedule(injector, millis());

        if (!runFor(1000)) { reportLine("M10", Outcome::Aborted, "aborted by key"); return; }
        Window w;
        if (!runWindow(500, w)) { reportLine("M10", Outcome::Aborted, "aborted by key"); return; }

        double rpm  = windowRPM(w);
        double want = independentRPM(1500, 40, 500);
        bool highOk = rpm >= 0.5 * want;

        reportCheck("M10", lowOk && highOk,
                    "at a 4 RPM target: never Clogged %s, duty 0 or >= %u %s; after the change %.1f of %.1f RPM",
                    lowOk ? "yes" : "NO", (unsigned)MOTOR_MIN_RUNNING_PERMILLE,
                    gSawBuzzingDuty ? "NO" : "yes", rpm, want);
    }

    // M11 - the distance ledger with the real motor. Everything else here
    // measures a speed at an instant; this measures what the shaft actually
    // delivered over a stretch of simulated ground, which is what the dose is.
    if (abortRequested) return;
    {
        TestMarker tm("M11");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("M11", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injectorStart(injector, millis());

        // Up to speed first: the start transient is the ledger's to correct,
        // but it belongs to the metres before the measurement, not inside it.
        if (!runFor(3000)) { reportLine("M11", Outcome::Aborted, "aborted by key"); return; }

        uint32_t edges0  = encoderEdges();
        uint32_t pulses0 = injector.seeder.wheelPulses;

        if (!runFor(20000)) { reportLine("M11", Outcome::Aborted, "aborted by key"); return; }

        double turns  = (double)(encoderEdges() - edges0) / (double)ENCODER_EDGES_PER_REV;
        double metres = (double)(injector.seeder.wheelPulses - pulses0) *
                        (double)injector.tractor.wheelMmPerPulse / 1000.0;
        double want   = metres * (double)WORKING_WIDTH_CM * (double)injector.tractor.doseKgPerHa /
                        (10.0 * (double)injector.tractor.gramsPer100Rev);

        bool ok = (want > 0.0) && (fabs(turns - want) <= 0.03 * want);

        reportCheck("M11", ok, "%.1f m of ground: %.1f shaft turns (want %.1f +-3 %%)",
                    metres, turns, want);
    }
}

// ---------------------------------------------------------------------------
// C - the calibration run
// ---------------------------------------------------------------------------

// Brings a run to CalibrationDone with no seeder packets. False means it did
// not get there, or was aborted - the caller must check abortRequested.
static bool cRunToDone()
{
    if (abortRequested) return false;

    injectorDefaults(injector);
    injector.tractor.calibrationRun = 0;
    injector.tractor.sending        = true;
    injectorSchedule(injector, millis());

    if (!runFor(1000)) return false;

    injector.tractor.calibrationRun = 1;
    injectorSchedule(injector, millis());   // the change goes out now, not at the next send
    if (!pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; })) return false;
    if (abortRequested) return false;

    uint32_t limit = (uint32_t)(1.3 * (double)CALIBRATION_REVOLUTIONS * 60000.0
                                / (double)CALIBRATION_RPM) + 2000;
    uint32_t start = millis();

    while (millis() - start < limit) {
        if (!runFor(200)) return false;
        if (logic.mode == DispenserMode::CalibrationDone) return true;
        if (logic.mode != DispenserMode::Calibrating) return false;
    }
    return false;
}

static void runCalibrationTests()
{
    Serial.println();
    Serial.println("--- C: calibration run (motor, free shaft) ---");

    // C01 - a full run.
    {
        TestMarker tm("C01");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("C01", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.calibrationRun = 0;
        injector.tractor.sending        = true;
        injectorSchedule(injector, millis());

        if (!runFor(1000)) { reportLine("C01", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool started = pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("C01", Outcome::Aborted, "aborted by key"); return; }

        if (!runFor(2000)) { reportLine("C01", Outcome::Aborted, "aborted by key"); return; }
        Window w;
        if (!runWindow(1000, w)) { reportLine("C01", Outcome::Aborted, "aborted by key"); return; }

        double rpm   = windowRPM(w);
        double want  = (double)CALIBRATION_RPM;
        bool   rpmOk = fabs(rpm - want) <= 0.15 * want;

        uint32_t limit = (uint32_t)(1.3 * (double)CALIBRATION_REVOLUTIONS * 60000.0
                                    / (double)CALIBRATION_RPM);
        uint32_t start = millis();
        bool  done = false;
        bool  progressOk = true;
        uint8_t lastProgress = 0;

        while (millis() - start < limit) {
            if (!runFor(200)) { reportLine("C01", Outcome::Aborted, "aborted by key"); return; }
            if (logic.mode == DispenserMode::Calibrating) {
                if (logic.progress < lastProgress) progressOk = false;
                lastProgress = logic.progress;
            } else if (logic.mode == DispenserMode::CalibrationDone) {
                done = true;
                break;
            } else {
                break;
            }
        }

        uint32_t settled = millis() - start;

        if (!runFor(1000)) { reportLine("C01", Outcome::Aborted, "aborted by key"); return; }

        uint32_t turned     = encoderEdges() - logic.calibrationStartEdges;
        double   wantEdges  = (double)CALIBRATION_REVOLUTIONS * (double)ENCODER_EDGES_PER_REV;
        bool     edgesOk    = ((double)turned >= wantEdges) && ((double)turned <= 1.01 * wantEdges);
        bool     progressOk100 = (logic.progress == 100);

        bool ok = started && rpmOk && progressOk && done && edgesOk && progressOk100;

        reportCheck("C01", ok, "started %s, %.1f RPM (want %.0f +-15%%), Done after %lu ms (limit %lu), "
                    "%lu edges (want %.0f..%.0f), progress 100 %s",
                    started ? "yes" : "NO", rpm, want, (unsigned long)settled, (unsigned long)limit,
                    (unsigned long)turned, wantEdges, 1.01 * wantEdges, progressOk100 ? "yes" : "NO");
    }

    // C02 - a finished run holds until the request drops.
    if (abortRequested) return;
    {
        TestMarker tm("C02");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("C02", Outcome::Aborted, "aborted by key"); return; }
        bool done = cRunToDone();
        if (abortRequested) { reportLine("C02", Outcome::Aborted, "aborted by key"); return; }

        bool held = true;
        for (int i = 0; i < 15; i++) {
            if (!runFor(200)) { reportLine("C02", Outcome::Aborted, "aborted by key"); return; }
            if (logic.mode != DispenserMode::CalibrationDone) held = false;
            if (out.motorPermille != 0) held = false;
        }

        injector.tractor.calibrationRun = 0;
        injectorSchedule(injector, millis());
        bool normal = pollUntil(300, []() { return logic.mode == DispenserMode::Normal; });
        if (abortRequested) { reportLine("C02", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("C02", done && held && normal, "Done holds with duty 0 %s, then Normal %s",
                    held ? "yes" : "NO", normal ? "yes" : "NO");
    }

    // C03 - cancelled part way through.
    if (abortRequested) return;
    {
        TestMarker tm("C03");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("C03", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.calibrationRun = 0;
        injector.tractor.sending        = true;
        injectorSchedule(injector, millis());

        if (!runFor(1000)) { reportLine("C03", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool started = pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("C03", Outcome::Aborted, "aborted by key"); return; }

        uint32_t start = millis();
        while (logic.progress < 20 && (millis() - start) < 60000) {
            if (!runFor(100)) { reportLine("C03", Outcome::Aborted, "aborted by key"); return; }
        }

        injector.tractor.calibrationRun = 0;
        injectorSchedule(injector, millis());
        bool normal = pollUntil(300, []() {
            return out.motorPermille == 0 && logic.mode == DispenserMode::Normal;
        });
        if (abortRequested) { reportLine("C03", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("C03", started && normal, "cancelled at %u %%: duty 0 and Normal within 300 ms %s",
                    (unsigned)logic.progress, normal ? "yes" : "NO");
    }

    // C04 - refused while the machine moves.
    if (abortRequested) return;
    {
        TestMarker tm("C04");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("C04", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.seeder.groundSpeedMmS = 1000;
        injector.seeder.wheelTurning   = 1;
        injector.seeder.sending        = true;
        injector.tractor.calibrationRun = 0;
        injector.tractor.sending        = true;
        injectorSchedule(injector, millis());

        if (!runFor(1000)) { reportLine("C04", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool refused = pollUntil(300, []() { return logic.mode == DispenserMode::Refused; });
        if (abortRequested) { reportLine("C04", Outcome::Aborted, "aborted by key"); return; }

        // The machine was moving with the dispenser on, so it metered until
        // Refused stopped it: let the shaft stop before counting.
        if (!runFor(500)) { reportLine("C04", Outcome::Aborted, "aborted by key"); return; }

        Window w;
        if (!runWindow(3000, w)) { reportLine("C04", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("C04", refused && w.aEdges <= 2,
                    "Refused within 300 ms %s, %lu edges over 3 s (want <= 2)",
                    refused ? "yes" : "NO", (unsigned long)w.aEdges);
    }

    // C05 - the machine starts moving during a run.
    if (abortRequested) return;
    {
        TestMarker tm("C05");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("C05", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.calibrationRun = 0;
        injector.tractor.sending        = true;
        injectorSchedule(injector, millis());

        if (!runFor(1000)) { reportLine("C05", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool started = pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("C05", Outcome::Aborted, "aborted by key"); return; }
        if (!runFor(5000)) { reportLine("C05", Outcome::Aborted, "aborted by key"); return; }

        injector.seeder.groundSpeedMmS = 1000;
        injector.seeder.wheelTurning   = 1;
        injector.seeder.sending        = true;
        injectorSchedule(injector, millis());

        bool refused = pollUntil(300, []() {
            return logic.mode == DispenserMode::Refused && out.motorPermille == 0;
        });
        if (abortRequested) { reportLine("C05", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("C05", started && refused, "5 s into the run the seeder reports movement: Refused %s",
                    refused ? "yes" : "NO");
    }

    // C06 - the tractor goes quiet during a run.
    if (abortRequested) return;
    {
        TestMarker tm("C06");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.calibrationRun = 0;
        injector.tractor.sending        = true;
        injectorSchedule(injector, millis());

        if (!runFor(1000)) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool started = pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }
        if (!runFor(5000)) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.sending = false;
        if (!runFor(2000)) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injector.tractor.sending        = true;
        injectorSchedule(injector, millis());

        bool stopped = pollUntil(LINK_TIMEOUT_MS + 300, []() {
            return out.motorPermille == 0 && logic.mode == DispenserMode::Normal;
        });
        if (abortRequested) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }

        if (!runFor(3000)) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }
        bool notCalibrating = (logic.mode != DispenserMode::Calibrating);

        injector.tractor.calibrationRun = 0;
        injectorSchedule(injector, millis());
        if (!runFor(1000)) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }
        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool again = pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("C06", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("C06", started && stopped && notCalibrating && again,
                    "tractor lost: duty 0 and Normal %s; request still raised does not restart it %s; "
                    "lowered and raised again does %s",
                    stopped ? "yes" : "NO", notCalibrating ? "yes" : "NO", again ? "yes" : "NO");
    }
}

// ---------------------------------------------------------------------------
// K - clog and unclogging, with a real motor
// ---------------------------------------------------------------------------

// Meters, then detaches channel A: the logic sees a stopped shaft while the
// motor keeps turning. The caller has already prepared the bench and started
// the injector. bEdgesWhileTurning reports what channel B counted between the
// detach and the clog - proving the shaft really was turning.
static bool kClogFromMetering(uint32_t *msToClog, uint32_t *bEdgesWhileTurning)
{
    if (!runFor(2000)) return false;

    uint32_t t0 = millis();
    uint32_t b0 = bEdges();
    encoderEnd();

    bool clogged = pollUntil(CLOG_DETECT_MS + 400, []() {
        return logic.mode == DispenserMode::Clogged;
    });

    if (msToClog != NULL) *msToClog = millis() - t0;
    if (bEdgesWhileTurning != NULL) *bEdgesWhileTurning = bEdges() - b0;
    return clogged;
}

// "Clog as in K01": meter as in M02, detach A, wait for Clogged, re-attach A.
// Always re-attaching is essential - every later edge check would otherwise
// pass without measuring anything. False means it never clogged, or the test
// was aborted - the caller must check abortRequested.
static bool kCauseClog(uint32_t *msToClog)
{
    if (!benchPrepare()) return false;
    injectorDefaults(injector);
    injectorStart(injector, millis());

    bool ok = kClogFromMetering(msToClog, NULL);
    encoderBegin();
    return ok;
}

static void runClogTests()
{
    Serial.println();
    Serial.println("--- K: clog and unclogging (motor) ---");

    // K01 - the clog itself.
    if (abortRequested) return;
    {
        TestMarker tm("K01");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("K01", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injectorStart(injector, millis());

        uint32_t msToClog = 0;
        uint32_t bTurning = 0;
        bool clogged = kClogFromMetering(&msToClog, &bTurning);
        if (abortRequested) { reportLine("K01", Outcome::Aborted, "aborted by key"); return; }

        bool timingOk = clogged && (msToClog >= CLOG_DETECT_MS) && (msToClog <= CLOG_DETECT_MS + 400);
        bool turned   = bTurning > 0;
        bool dutyZero = (out.motorPermille == 0);

        encoderBegin();

        if (!runFor(3000)) { reportLine("K01", Outcome::Aborted, "aborted by key"); return; }
        bool staysClogged = (logic.mode == DispenserMode::Clogged) && (out.motorPermille == 0);

        reportCheck("K01", timingOk && turned && dutyZero && staysClogged,
                    "Clogged %lu ms after detaching A (want %lu..%lu), %lu B edges while it turned, "
                    "duty 0 %s, stays Clogged %s",
                    (unsigned long)msToClog, (unsigned long)CLOG_DETECT_MS,
                    (unsigned long)(CLOG_DETECT_MS + 400), (unsigned long)bTurning,
                    dutyZero ? "yes" : "NO", staysClogged ? "yes" : "NO");
    }

    // K02 - Anuluj.
    if (abortRequested) return;
    {
        TestMarker tm("K02");
        MotorGuard guard;

        uint32_t msToClog = 0;
        bool clogged = kCauseClog(&msToClog);
        if (abortRequested) { reportLine("K02", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.clogClearSeq++;
        injectorSchedule(injector, millis());
        bool normal = pollUntil(300, []() { return logic.mode == DispenserMode::Normal; });
        if (abortRequested) { reportLine("K02", Outcome::Aborted, "aborted by key"); return; }

        if (!runFor(1000)) { reportLine("K02", Outcome::Aborted, "aborted by key"); return; }
        Window w;
        if (!runWindow(500, w)) { reportLine("K02", Outcome::Aborted, "aborted by key"); return; }

        double rpm  = windowRPM(w);
        double want = independentRPM(1000, 40, 500);
        bool   back = rpm >= 0.5 * want;

        reportCheck("K02", clogged && normal && back,
                    "Normal within 300 ms %s, %.1f of %.1f RPM within 1 s %s",
                    normal ? "yes" : "NO", rpm, want, back ? "yes" : "NO");
    }

    // K03 - Odetkaj, watching the direction of every move.
    if (abortRequested) return;
    {
        TestMarker tm("K03");
        MotorGuard guard;

        uint32_t msToClog = 0;
        bool clogged = kCauseClog(&msToClog);
        if (abortRequested) { reportLine("K03", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.unclogSeq++;
        injectorSchedule(injector, millis());
        bool entered = pollUntil(300, []() { return logic.mode == DispenserMode::Unclogging; });
        if (abortRequested) { reportLine("K03", Outcome::Aborted, "aborted by key"); return; }

        // Every window and the return to Clogged are timed from the moment the
        // sequence really started, not from the counter change: the packet can
        // wait for the next send, and the logic acts on its next control step.
        uint32_t tEnter = millis();

        int  levelReverse = -1;
        int  levelForward = -1;
        bool directionOk  = true;
        bool windowOk     = true;

        for (int cycle = 0; cycle < (int)UNCLOG_CYCLES; cycle++) {
            // The middle 400 ms of each move, from the mode table's own timing.
            const uint32_t windowMs = 400;
            const uint32_t reverseAt = (uint32_t)cycle * UNCLOG_CYCLE_MS
                                     + (UNCLOG_REVERSE_MS - windowMs) / 2;
            const uint32_t forwardAt = (uint32_t)cycle * UNCLOG_CYCLE_MS
                                     + UNCLOG_REVERSE_MS + UNCLOG_PAUSE_MS
                                     + (UNCLOG_FORWARD_MS - windowMs) / 2;

            for (int half = 0; half < 2; half++) {
                const uint32_t at      = (half == 0) ? reverseAt : forwardAt;
                const bool     forward = (half == 1);

                if (!pumpUntil(tEnter, at)) { reportLine("K03", Outcome::Aborted, "aborted by key"); return; }
                Window w;
                if (!runWindow(windowMs, w)) { reportLine("K03", Outcome::Aborted, "aborted by key"); return; }

                int level = windowDirection(w, 0.8);
                if (level < 0) {
                    windowOk = false;
                } else if (forward) {
                    if (levelForward < 0) levelForward = level;
                    else if (levelForward != level) directionOk = false;
                } else {
                    if (levelReverse < 0) levelReverse = level;
                    else if (levelReverse != level) directionOk = false;
                }
            }
        }

        if (levelReverse < 0 || levelForward < 0 || levelReverse == levelForward) directionOk = false;
        if (report.haveForwardLevel && levelForward != report.forwardALevel) directionOk = false;

        if (!pumpUntil(tEnter, UNCLOG_TOTAL_MS - 300)) {
            reportLine("K03", Outcome::Aborted, "aborted by key");
            return;
        }
        bool backToClogged = pollUntil(700, []() { return logic.mode == DispenserMode::Clogged; });
        if (abortRequested) { reportLine("K03", Outcome::Aborted, "aborted by key"); return; }
        uint32_t tClog = millis() - tEnter;

        bool timingOk = backToClogged
                     && (tClog >= UNCLOG_TOTAL_MS - 300)
                     && (tClog <= UNCLOG_TOTAL_MS + 300);
        bool stoppedOk = (out.motorPermille == 0) && out.motorForward;

        reportCheck("K03", clogged && entered && windowOk && directionOk && timingOk && stoppedOk,
                    "Unclogging within 300 ms %s, directions separate %s, back to Clogged at %lu ms "
                    "(want %lu +-300), duty 0 forward %s",
                    entered ? "yes" : "NO", directionOk ? "yes" : "NO",
                    (unsigned long)tClog, (unsigned long)UNCLOG_TOTAL_MS, stoppedOk ? "yes" : "NO");
    }

    // K04 - a clog during the calibration run.
    if (abortRequested) return;
    {
        TestMarker tm("K04");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("K04", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.calibrationRun = 0;
        injector.tractor.sending        = true;
        injectorSchedule(injector, millis());

        if (!runFor(1000)) { reportLine("K04", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.calibrationRun = 1;
        injectorSchedule(injector, millis());
        bool started = pollUntil(300, []() { return logic.mode == DispenserMode::Calibrating; });
        if (abortRequested) { reportLine("K04", Outcome::Aborted, "aborted by key"); return; }
        if (!runFor(5000)) { reportLine("K04", Outcome::Aborted, "aborted by key"); return; }

        uint32_t t0 = millis();
        encoderEnd();
        bool clogged = pollUntil(CLOG_DETECT_MS + 400, []() {
            return logic.mode == DispenserMode::Clogged;
        });
        if (abortRequested) { reportLine("K04", Outcome::Aborted, "aborted by key"); return; }
        uint32_t msToClog = millis() - t0;
        encoderBegin();

        injector.tractor.clogClearSeq++;
        injectorSchedule(injector, millis());
        bool normal = pollUntil(300, []() { return logic.mode == DispenserMode::Normal; });
        if (abortRequested) { reportLine("K04", Outcome::Aborted, "aborted by key"); return; }

        Window w;
        if (!runWindow(3000, w)) { reportLine("K04", Outcome::Aborted, "aborted by key"); return; }

        bool notCalibrating = (logic.mode != DispenserMode::Calibrating) && (w.aEdges <= 2);

        reportCheck("K04", started && clogged && (msToClog <= CLOG_DETECT_MS + 400) && normal && notCalibrating,
                    "calibration clogged after %lu ms (limit %lu), clear gives Normal %s, "
                    "the still-raised request does not restart it %s",
                    (unsigned long)msToClog, (unsigned long)(CLOG_DETECT_MS + 400),
                    normal ? "yes" : "NO", notCalibrating ? "yes" : "NO");
    }

    // K05 - a tractor reboot while clogged.
    if (abortRequested) return;
    {
        TestMarker tm("K05");
        MotorGuard guard;

        if (!benchPrepare()) { reportLine("K05", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.clogClearSeq = 1;
        injector.tractor.unclogSeq    = 1;
        injectorStart(injector, millis());

        uint32_t msToClog = 0;
        bool clogged = kClogFromMetering(&msToClog, NULL);
        if (abortRequested) { reportLine("K05", Outcome::Aborted, "aborted by key"); return; }
        encoderBegin();

        injectorFakeReboot(injector, 1500);
        injector.tractor.clogClearSeq = 0;
        injector.tractor.unclogSeq    = 0;

        Window w;
        if (!runWindow(3000, w)) { reportLine("K05", Outcome::Aborted, "aborted by key"); return; }

        bool stays = (logic.mode == DispenserMode::Clogged) && (w.aEdges <= 2);

        reportCheck("K05", clogged && stays, "after the reboot and (0,0): Clogged %s, %lu edges over 3 s",
                    stays ? "yes" : "NO", (unsigned long)w.aEdges);
    }

    // K06 - a counter change that arrives after a gap is ignored.
    if (abortRequested) return;
    {
        TestMarker tm("K06");
        MotorGuard guard;

        uint32_t msToClog = 0;
        bool clogged = kCauseClog(&msToClog);
        if (abortRequested) { reportLine("K06", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.sending = false;
        if (!runFor(2000)) { reportLine("K06", Outcome::Aborted, "aborted by key"); return; }

        injector.tractor.unclogSeq++;
        injector.tractor.sending = true;
        injectorSchedule(injector, millis());

        Window w;
        if (!runWindow(3000, w)) { reportLine("K06", Outcome::Aborted, "aborted by key"); return; }
        bool stillClogged = (logic.mode == DispenserMode::Clogged) && (w.aEdges <= 2);

        injector.tractor.unclogSeq++;
        injectorSchedule(injector, millis());
        bool unclogging = pollUntil(300, []() { return logic.mode == DispenserMode::Unclogging; });
        if (abortRequested) { reportLine("K06", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("K06", clogged && stillClogged && unclogging,
                    "the change across the gap is ignored %s, a later one unclogs %s",
                    stillClogged ? "yes" : "NO", unclogging ? "yes" : "NO");
    }

    // K07 - a real stall, with the lever.
    if (abortRequested) return;
    {
        TestMarker tm("K07");
        MotorGuard guard;

        Serial.println();
        Serial.println(">> REAL STALL TEST. Clamp locking pliers on the coupling, handle against");
        Serial.println(">> a fixed stop in the FORWARD direction, so the shaft cannot move at all.");
        Serial.println(">> Fit it before answering. Press any key when it is fitted ('s' skips).");

        int key = waitForKey();
        if (key == 's' || key == 'S') {
            reportSkipped("K07", "skipped by the operator");
            return;
        }

        if (!benchPrepare()) { reportLine("K07", Outcome::Aborted, "aborted by key"); return; }
        injectorDefaults(injector);
        injector.tractor.doseKgPerHa = 25;      // 1000 mm/s at 25 kg/ha asks for 120 RPM
        injectorStart(injector, millis());

        uint32_t t0 = millis();
        bool clogged = pollUntil(CLOG_DETECT_MS + 700, []() {
            return logic.mode == DispenserMode::Clogged;
        });
        if (abortRequested) { reportLine("K07", Outcome::Aborted, "aborted by key"); return; }
        uint32_t msToClog = millis() - t0;

        bool dutyZero = (out.motorPermille == 0);

        // Stop the simulated machine before the lever comes off: on a running
        // tractor Anuluj would go straight back to metering, and the operator's
        // hands are still on the coupling.
        injector.seeder.groundSpeedMmS = 0;
        injector.seeder.wheelTurning   = 0;
        injectorSchedule(injector, millis());
        if (!runFor(300)) { reportLine("K07", Outcome::Aborted, "aborted by key"); return; }

        Serial.println(">> Remove the lever, keep clear, then press a key. The motor stays off.");
        waitForKey();

        injector.tractor.clogClearSeq++;
        injectorSchedule(injector, millis());
        bool normal = pollUntil(300, []() {
            return logic.mode == DispenserMode::Normal && out.motorPermille == 0;
        });
        if (abortRequested) { reportLine("K07", Outcome::Aborted, "aborted by key"); return; }

        reportCheck("K07", clogged && dutyZero && normal,
                    "Clogged %lu ms after starting (limit %lu), duty 0 %s, clear gives Normal with duty 0 %s",
                    (unsigned long)msToClog, (unsigned long)(CLOG_DETECT_MS + 700),
                    dutyZero ? "yes" : "NO", normal ? "yes" : "NO");
    }
}
