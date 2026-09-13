#pragma once

#include <Arduino.h>
#include <stdarg.h>

// ---------------------------------------------------------------------------
// PASS/FAIL/SKIP/ABORTED recording for the bench test. Counters accumulate for
// the whole session, so `r` shows everything done since the board came up.
// A skipped operator step is a SKIP, never a PASS.
// ---------------------------------------------------------------------------

enum class Outcome : uint8_t { Pass, Fail, Skip, Aborted };

struct Report {
    uint16_t pass    = 0;
    uint16_t fail    = 0;
    uint16_t skip    = 0;
    uint16_t aborted = 0;

    static constexpr uint8_t MAX_FAILED = 48;
    char    failedIds[MAX_FAILED][10] = {};
    uint8_t failedCount = 0;

    // Measured values worth copying into machine_settings.h.
    bool   haveEdgesPerRev   = false;
    double edgesPerRev       = 0.0;
    bool   haveBreakaway     = false;
    int    breakawayPermille = 0;
    bool   haveFullDutyRPM   = false;
    int    rpmAtFullDuty     = 0;
    bool   haveForwardLevel  = false;
    int    forwardALevel     = 0;   // 1 = channel A high at the B edges, 0 = low
};

static Report report;

static const char *modeName(DispenserMode mode)
{
    switch (mode) {
        case DispenserMode::Normal:          return "Normal";
        case DispenserMode::Calibrating:     return "Calibrating";
        case DispenserMode::CalibrationDone: return "CalibrationDone";
        case DispenserMode::Refused:         return "Refused";
        case DispenserMode::Clogged:         return "Clogged";
        case DispenserMode::Unclogging:      return "Unclogging";
        default:                             return "?";
    }
}

static const char *faultName(DispenserFault fault)
{
    switch (fault) {
        case DispenserFault::None:        return "None";
        case DispenserFault::NoSpeedData: return "NoSpeedData";
        case DispenserFault::OverSpeed:   return "OverSpeed";
        default:                          return "?";
    }
}

static const char *outcomeName(Outcome outcome)
{
    switch (outcome) {
        case Outcome::Pass:    return "PASS";
        case Outcome::Fail:    return "FAIL";
        case Outcome::Skip:    return "SKIP";
        default:               return "ABORTED";
    }
}

static void reportV(const char *id, Outcome outcome, const char *fmt, va_list args)
{
    char text[192];
    vsnprintf(text, sizeof(text), fmt, args);

    switch (outcome) {
        case Outcome::Pass:    report.pass++;    break;
        case Outcome::Skip:    report.skip++;    break;
        case Outcome::Aborted: report.aborted++; break;
        case Outcome::Fail:
            report.fail++;
            if (report.failedCount < Report::MAX_FAILED) {
                strncpy(report.failedIds[report.failedCount], id, 9);
                report.failedIds[report.failedCount][9] = '\0';
                report.failedCount++;
            }
            break;
    }

    Serial.printf("[%-7s] %s %s\n", outcomeName(outcome), id, text);
}

static void reportLine(const char *id, Outcome outcome, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    reportV(id, outcome, fmt, args);
    va_end(args);
}

// The common case: one measured value against an expectation.
static void reportCheck(const char *id, bool ok, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    reportV(id, ok ? Outcome::Pass : Outcome::Fail, fmt, args);
    va_end(args);
}

static void reportSkipped(const char *id, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    reportV(id, Outcome::Skip, fmt, args);
    va_end(args);
}

// Always ends with the reflash line - a bench image must never be left on the
// module that goes on the machine.
static void reportSummary()
{
    Serial.println();
    Serial.println("=== BENCH SUMMARY ===");
    Serial.printf("  PASS %u   FAIL %u   SKIP %u   ABORTED %u\n",
                  (unsigned)report.pass, (unsigned)report.fail,
                  (unsigned)report.skip, (unsigned)report.aborted);

    if (report.failedCount > 0) {
        Serial.print("  failed:");
        for (uint8_t i = 0; i < report.failedCount; i++) {
            Serial.print(' ');
            Serial.print(report.failedIds[i]);
        }
        Serial.println();
    }

    Serial.println("  --- measured values for include/machine_settings.h ---");
    if (report.haveEdgesPerRev) {
        Serial.printf("  ENCODER_EDGES_PER_REV         = %.0f   (is  %u)\n",
                      report.edgesPerRev, (unsigned)ENCODER_EDGES_PER_REV);
    }
    if (report.haveBreakaway) {
        Serial.printf("  MOTOR_MIN_RUNNING_PERMILLE    = %d   (is  %u)\n",
                      report.breakawayPermille, (unsigned)MOTOR_MIN_RUNNING_PERMILLE);
    }
    if (report.haveFullDutyRPM) {
        Serial.printf("  MOTOR_MAX_RPM                 = %d   (is  %u)\n",
                      report.rpmAtFullDuty, (unsigned)MOTOR_MAX_RPM);
    }
    if (report.haveForwardLevel) {
        Serial.printf("  forward = channel A %s at the channel B edges (MOTOR_DIR_FORWARD is %s)\n",
                      report.forwardALevel ? "HIGH" : "LOW",
                      (MOTOR_DIR_FORWARD == LOW) ? "LOW" : "HIGH");
    }

    Serial.println("  Reflash the production firmware before fitting the module: pio run -e dispenser -t upload");
    Serial.println("=====================");
}
