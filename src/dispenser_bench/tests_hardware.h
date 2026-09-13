#pragma once

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_io.h"
#include "bench_report.h"

// ---------------------------------------------------------------------------
// H tests: the module's own hardware - supply, MD13S, motor, both encoder
// channels, the DIR line and the radio. The motor is driven directly here, so
// nothing else is allowed to touch it: these tests use motorWait() /
// motorWindow(), which keep the radio alive but never run the control tick.
//
// A few of them need the operator. Every operator step can be skipped with
// 's', and a skipped step is a SKIP, never a PASS.
// ---------------------------------------------------------------------------

static void testH00()
{
    TestMarker tm("H00");

    bool ok = bootInfo.reasonOk && !bootInfo.hadMarker && bootInfo.espnowOk;

    reportCheck("H00", ok, "reset %s, marker %s, ESP-NOW %s",
                resetReasonName(bootInfo.reason),
                bootInfo.hadMarker ? "WAS SET" : "clear",
                bootInfo.espnowOk ? "ok" : "FAILED");
}

static void testH01()
{
    TestMarker tm("H01");
    MotorGuard guard;

    if (!benchPrepare()) { reportLine("H01", Outcome::Aborted, "aborted by key"); return; }

    Window w;
    if (!motorWindow(3000, w)) {
        reportLine("H01", Outcome::Aborted, "aborted by key");
        return;
    }

    bool ok = (w.aEdges == 0) && (w.bEdges == 0);

    reportCheck("H01", ok, "motor off for 3 s: %lu edges on A, %lu on B (want 0, 0)",
                (unsigned long)w.aEdges, (unsigned long)w.bEdges);
}

static void testH02()
{
    TestMarker tm("H02");
    MotorGuard guard;

    if (!benchPrepare()) { reportLine("H02", Outcome::Aborted, "aborted by key"); return; }

    Serial.println();
    Serial.println(">> Coupling DISCONNECTED from the auger. Mark the output shaft,");
    Serial.println(">> turn it by hand exactly 10 revolutions in the dispensing direction,");
    Serial.println(">> then press any key. ('s' skips this step.)");

    uint32_t aStart = encoderEdges();
    uint32_t bStart = bEdges();

    int key = waitForKey();
    if (key == 's' || key == 'S') {
        reportSkipped("H02", "skipped by the operator");
        return;
    }

    uint32_t aDelta = encoderEdges() - aStart;
    uint32_t bDelta = bEdges()      - bStart;

    double perRev  = (double)aDelta / 10.0;
    double want    = (double)ENCODER_EDGES_PER_REV;
    double tolerance = 0.03 * want;

    bool countOk = fabs(perRev - want) <= tolerance;
    bool bothOk  = (aDelta > 0) && (fabs((double)bDelta - (double)aDelta) <= 0.03 * (double)aDelta);

    if (countOk) {
        report.haveEdgesPerRev = true;
        report.edgesPerRev     = perRev;
    }

    reportCheck("H02", countOk && bothOk,
                "10 turns: %lu A edges (%.1f per rev, want %.0f +-3%%), %lu B edges",
                (unsigned long)aDelta, perRev, want, (unsigned long)bDelta);
}

static void testH03()
{
    TestMarker tm("H03");
    MotorGuard guard;

    if (!benchPrepare()) { reportLine("H03", Outcome::Aborted, "aborted by key"); return; }

    const uint16_t duties[] = {250, 500, 750, 1000};
    const uint8_t  count    = sizeof(duties) / sizeof(duties[0]);

    bool   rising   = true;
    double previous = -1.0;
    double fullDuty = 0.0;
    bool   ok       = true;

    Serial.println(">> Free shaft, no auger load. Forward, 1.5 s at each level:");

    for (uint8_t i = 0; i < count; i++) {
        DispenserOutputs drive = {duties[i], true};
        motorApply(drive);

        if (!motorWait(500)) { reportLine("H03", Outcome::Aborted, "aborted by key"); return; }

        Window w;
        if (!motorWindow(1000, w)) { reportLine("H03", Outcome::Aborted, "aborted by key"); return; }

        double rpm = windowRPM(w);
        Serial.printf("     %4u permille -> %6.1f RPM\n", (unsigned)duties[i], rpm);

        if (previous >= 0.0 && rpm < previous - 1.0) rising = false;
        previous = rpm;

        if (duties[i] == 1000) fullDuty = rpm;
    }

    safeStop();

    double floorRPM = 0.7 * (double)MOTOR_MAX_RPM;
    if (fullDuty < floorRPM) ok = false;
    if (!rising) ok = false;

    report.haveFullDutyRPM = true;
    report.rpmAtFullDuty   = (int)(fullDuty + 0.5);

    reportCheck("H03", ok, "RPM never fell as duty rose: %s; %.1f RPM at 1000 permille (want >= %.1f)",
                rising ? "yes" : "NO", fullDuty, floorRPM);
}

static void testH04()
{
    TestMarker tm("H04");
    MotorGuard guard;

    if (!benchPrepare()) { reportLine("H04", Outcome::Aborted, "aborted by key"); return; }

    Serial.println(">> Watch the shaft. Forward 300 permille for 1.5 s, stop, then");
    Serial.println(">> reverse 300 permille for 1.5 s. It will not move far.");

    DispenserOutputs forward = {300, true};
    DispenserOutputs reverse = {300, false};
    DispenserOutputs off     = {0, true};

    motorApply(forward);
    if (!motorWait(500)) { reportLine("H04", Outcome::Aborted, "aborted by key"); return; }
    Window fwd;
    if (!motorWindow(1000, fwd)) { reportLine("H04", Outcome::Aborted, "aborted by key"); return; }

    motorApply(off);
    if (!motorWait(700)) { reportLine("H04", Outcome::Aborted, "aborted by key"); return; }

    motorApply(reverse);
    if (!motorWait(500)) { reportLine("H04", Outcome::Aborted, "aborted by key"); return; }
    Window rev;
    if (!motorWindow(1000, rev)) { reportLine("H04", Outcome::Aborted, "aborted by key"); return; }

    motorApply(off);
    safeStop();

    int forwardLevel = windowDirection(fwd, 0.9);
    int reverseLevel = windowDirection(rev, 0.9);

    bool levelsOk = (forwardLevel >= 0) && (reverseLevel >= 0) && (forwardLevel != reverseLevel);

    Serial.printf("     forward: %lu of %lu B edges saw A high;  reverse: %lu of %lu\n",
                  (unsigned long)fwd.bHigh, (unsigned long)fwd.bEdges,
                  (unsigned long)rev.bHigh, (unsigned long)rev.bEdges);

    if (forwardLevel >= 0) {
        report.haveForwardLevel = true;
        report.forwardALevel    = forwardLevel;
    }

    Serial.println(">> Did the FIRST (forward) run turn the shaft the way the auger dispenses? y/n ('s' skips)");
    int answer = askYesNo();
    if (answer < 0) {
        reportSkipped("H04", "operator skipped the direction question");
        return;
    }

    bool turnedRightWay = (answer == 1);

    if (!turnedRightWay) {
        reportCheck("H04", false,
                    "the forward run turned the wrong way - flip MOTOR_DIR_FORWARD or swap the motor leads");
        return;
    }

    reportCheck("H04", levelsOk && turnedRightWay,
                "channel B separates the directions (forward = A %s), and the forward run dispenses",
                (forwardLevel == 1) ? "high" : "low");
}

static void testH05()
{
    TestMarker tm("H05");
    MotorGuard guard;

    if (!benchPrepare()) { reportLine("H05", Outcome::Aborted, "aborted by key"); return; }

    Serial.println(">> Auger loaded as it would be in work, if you can. Ramping forward");
    Serial.println(">> from 0 by 10 permille every 300 ms, stopping at the first level that turns it.");

    int breakaway = -1;
    for (int duty = 10; duty <= 400; duty += 10) {
        DispenserOutputs drive = {(uint16_t)duty, true};
        motorApply(drive);

        Window w;
        if (!motorWindow(300, w)) { reportLine("H05", Outcome::Aborted, "aborted by key"); return; }

        if (windowRPM(w) >= 5.0) {
            breakaway = duty;
            break;
        }
    }

    safeStop();
    if (!motorWait(1000)) { reportLine("H05", Outcome::Aborted, "aborted by key"); return; }

    Serial.println(">> Now starting from standstill at MOTOR_MIN_RUNNING_PERMILLE for 1 s.");

    DispenserOutputs minimum = {MOTOR_MIN_RUNNING_PERMILLE, true};
    motorApply(minimum);

    if (!motorWait(500)) { reportLine("H05", Outcome::Aborted, "aborted by key"); return; }
    Window w;
    if (!motorWindow(500, w)) { reportLine("H05", Outcome::Aborted, "aborted by key"); return; }

    safeStop();

    double standstillRPM = windowRPM(w);
    bool   startOk = standstillRPM >= 5.0;
    bool   foundOk = (breakaway > 0) && (breakaway <= 400);

    if (foundOk) {
        report.haveBreakaway     = true;
        report.breakawayPermille = breakaway;
    }

    if (foundOk && startOk) {
        reportCheck("H05", true, "breakaway at %d permille; MOTOR_MIN_RUNNING_PERMILLE (%u) reaches %.1f RPM",
                    breakaway, (unsigned)MOTOR_MIN_RUNNING_PERMILLE, standstillRPM);
        return;
    }

    if (!foundOk) {
        reportCheck("H05", false, "nothing turned the shaft up to 400 permille");
    } else {
        reportCheck("H05", false, "standstill start at %u permille only reached %.1f RPM - "
                    "raise MOTOR_MIN_RUNNING_PERMILLE to at least %d",
                    (unsigned)MOTOR_MIN_RUNNING_PERMILLE, standstillRPM, breakaway + 20);
    }
}

static void testH06()
{
    TestMarker tm("H06");
    MotorGuard guard;

    if (!benchPrepare()) { reportLine("H06", Outcome::Aborted, "aborted by key"); return; }

    Serial.println();
    Serial.println(">> OPTIONAL stall test. Clamp locking pliers on the coupling with the");
    Serial.println(">> handle against a fixed stop so the shaft CANNOT move. Fit it before");
    Serial.println(">> answering. Press any key when it is fitted ('s' skips).");

    int key = waitForKey();
    if (key == 's' || key == 'S') {
        reportSkipped("H06", "skipped by the operator");
        return;
    }

    DispenserOutputs drive = {1000, true};
    motorApply(drive);

    uint32_t startMs = millis();
    if (!motorWait(1000)) { reportLine("H06", Outcome::Aborted, "aborted by key"); return; }
    Window w;
    if (!motorWindow(1000, w)) { reportLine("H06", Outcome::Aborted, "aborted by key"); return; }

    safeStop();

    double rpm      = windowRPM(w);
    bool   complete = (millis() - startMs) >= 2000;   // a reset would have restarted millis()

    Serial.println(">> Remove the lever now.");
    waitForKey();

    if (rpm >= 5.0) {
        reportSkipped("H06", "shaft was not held: %.1f RPM with the lever fitted", rpm);
        return;
    }

    reportCheck("H06", complete, "held shaft: %.1f RPM at 1000 permille, board kept running", rpm);
}

static void testH07()
{
    TestMarker tm("H07");

    reportCheck("H07", heartbeatFailures == 0,
                "heartbeat sends this session: %lu failed",
                (unsigned long)heartbeatFailures);
}

static void runHardwareTests()
{
    Serial.println();
    Serial.println("--- H: hardware ---");

    // An abort ends the whole command: nothing may start the motor again.
    if (abortRequested) return; testH00();
    if (abortRequested) return; testH01();
    if (abortRequested) return; testH02();
    if (abortRequested) return; testH03();
    if (abortRequested) return; testH04();
    if (abortRequested) return; testH05();
    if (abortRequested) return; testH06();
    if (abortRequested) return; testH07();
}
