#pragma once

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_io.h"
#include "bench_inject.h"
#include "bench_report.h"

// ---------------------------------------------------------------------------
// V tests: the receive path, with real time and no motor. Every packet goes
// through dispenserHandlePacket() - the function the production receive
// callback calls - so validation, the inbox lock and the link timing are all
// exercised, not just the decision logic.
// ---------------------------------------------------------------------------

// Sends one broken packet and returns true if the inbox accepted it anyway.
static bool vSendBroken(bool seederPacket, int variant)
{
    dispenserResetInbox();

    Injector inj;
    injectorDefaults(inj);
    inj.seeder.groundSpeedMmS = 1234;
    inj.seeder.wheelTurning   = 1;
    inj.seeder.wheelPulses    = 5678;

    uint8_t buf[80] = {};
    int len = seederPacket ? buildSeederPacket(inj, millis(), buf)
                           : buildTractorPacket(inj, millis(), buf);

    switch (variant) {
        case 0:  buf[0] = 'X'; break;                                  // magic0
        case 1:  buf[1] = 'X'; break;                                  // magic1
        case 2:  buf[2] = PROTOCOL_VERSION - 1; break;
        case 3:  buf[2] = PROTOCOL_VERSION + 1; break;
        case 4:  buf[3] = NETWORK_ID + 1; break;
        case 5:  buf[4] = seederPacket ? (uint8_t)MsgType::TractorCommand
                                       : (uint8_t)MsgType::DispenserStatus; break;
        case 6:  buf[5] = seederPacket ? (uint8_t)NodeId::Tractor
                                       : (uint8_t)NodeId::Seeder; break;
        case 7:  len -= 1; break;
        case 8:  len += 1; break;      // buf[] holds a full packet plus a zero byte
        case 9:  len = 8; break;       // header only
        case 10: len = 0; break;
        case 11: len = -1; break;
        case 12:
            // Null data pointer with the length the type would have.
            dispenserHandlePacket(NULL, seederPacket ? (int)sizeof(SeederTelemetry)
                                                     : (int)sizeof(TractorCommand));
            break;
        default: break;
    }

    if (variant != 12) dispenserHandlePacket(buf, len);

    DispenserInputs in;
    dispenserReadInputs(millis(), in);
    bool accepted = seederPacket ? in.haveTelemetry : in.haveCommand;
    if (accepted) return true;
    if (dispenserLinkFlags() != 0) return true;   // must not mark a board alive either
    return false;
}

static void testV03()
{
    TestMarker tm("V03");

    int firstAccepted = -1;
    for (int v = 0; v <= 12; v++) {
        if (vSendBroken(true, v)) { firstAccepted = v; break; }
    }

    reportCheck("V03", firstAccepted < 0,
                "13 broken SeederTelemetry variants (first accepted, if any: %d)", firstAccepted);
}

static void testV04()
{
    TestMarker tm("V04");

    int firstAccepted = -1;
    for (int v = 0; v <= 12; v++) {
        if (vSendBroken(false, v)) { firstAccepted = v; break; }
    }

    reportCheck("V04", firstAccepted < 0,
                "13 broken TractorCommand variants (first accepted, if any: %d)", firstAccepted);
}

static void testV01()
{
    TestMarker tm("V01");

    dispenserResetInbox();

    Injector inj;
    injectorDefaults(inj);
    inj.seeder.groundSpeedMmS = 1234;
    inj.seeder.wheelTurning   = 1;
    inj.seeder.wheelPulses    = 5678;

    uint8_t buf[64];
    int len = buildSeederPacket(inj, millis(), buf);
    dispenserHandlePacket(buf, len);

    DispenserInputs in;
    dispenserReadInputs(millis(), in);

    bool ok = in.haveTelemetry && in.seederAlive
           && in.telemetry.groundSpeedMmS == 1234
           && in.telemetry.wheelTurning == 1
           && in.telemetry.wheelPulses == 5678;

    reportCheck("V01", ok, "accepted, alive, speed %u, turning %u, pulses %lu",
                (unsigned)in.telemetry.groundSpeedMmS,
                (unsigned)in.telemetry.wheelTurning,
                (unsigned long)in.telemetry.wheelPulses);
}

static void testV02()
{
    TestMarker tm("V02");

    dispenserResetInbox();

    Injector inj;
    injectorDefaults(inj);
    inj.tractor.dispenserEnabled = 1;
    inj.tractor.doseKgPerHa      = 77;
    inj.tractor.gramsPer100Rev   = 444;
    inj.tractor.calibrationRun   = 0;
    inj.tractor.clogClearSeq     = 3;
    inj.tractor.unclogSeq        = 9;

    uint8_t buf[64];
    int len = buildTractorPacket(inj, millis(), buf);
    dispenserHandlePacket(buf, len);

    DispenserInputs in;
    dispenserReadInputs(millis(), in);

    bool ok = in.haveCommand && in.tractorAlive
           && in.command.doseKgPerHa == 77
           && in.command.gramsPer100Rev == 444
           && in.command.dispenserEnabled == 1
           && in.command.clogClearSeq == 3
           && in.command.unclogSeq == 9;

    reportCheck("V02", ok, "accepted, alive, dose %u, grams %lu, counters (%u,%u)",
                (unsigned)in.command.doseKgPerHa,
                (unsigned long)in.command.gramsPer100Rev,
                (unsigned)in.command.clogClearSeq, (unsigned)in.command.unclogSeq);
}

static void testV05()
{
    TestMarker tm("V05");

    dispenserResetInbox();

    DispenserStatus other;
    memset(&other, 0, sizeof(other));
    fillHeader(other.header, MsgType::DispenserStatus, NodeId::Dispenser,
               LINK_HEARD_TRACTOR | LINK_HEARD_SEEDER);
    other.targetShaftRPM = 200;

    dispenserHandlePacket((const uint8_t *)&other, sizeof(other));

    DispenserInputs in;
    dispenserReadInputs(millis(), in);

    bool ok = !in.haveTelemetry && !in.haveCommand && dispenserLinkFlags() == 0;

    reportCheck("V05", ok, "another dispenser's status sets nothing and marks no board alive");
}

static void testV06()
{
    TestMarker tm("V06");

    dispenserResetInbox();

    uint32_t rng = 0xC0FFEEu;
    bool acceptedAnything = false;

    for (int i = 0; i < 2000; i++) {
        uint8_t buf[64];
        rng = rng * 1664525u + 1013904223u;
        int len = (int)((rng >> 16) % 65);
        for (int j = 0; j < len; j++) {
            rng = rng * 1664525u + 1013904223u;
            buf[j] = (uint8_t)(rng >> 24);
        }

        dispenserHandlePacket(buf, len);

        DispenserInputs in;
        dispenserReadInputs(millis(), in);
        if (in.haveTelemetry || in.haveCommand) { acceptedAnything = true; break; }
    }

    // And the board must still be working afterwards.
    Injector inj;
    injectorDefaults(inj);
    uint8_t good[64];
    int goodLen = buildSeederPacket(inj, millis(), good);
    dispenserHandlePacket(good, goodLen);

    DispenserInputs in;
    dispenserReadInputs(millis(), in);
    bool stillWorking = in.haveTelemetry && in.seederAlive;

    reportCheck("V06", !acceptedAnything && stillWorking,
                "2000 random packets all dropped, a valid one still accepted afterwards");
}

static void testV07()
{
    TestMarker tm("V07");

    dispenserResetInbox();

    Injector inj;
    injectorDefaults(inj);
    uint8_t buf[64];
    int len = buildSeederPacket(inj, millis(), buf);
    dispenserHandlePacket(buf, len);

    if (!silentWait(LINK_TIMEOUT_MS + 200)) {
        reportLine("V07", Outcome::Aborted, "aborted by key");
        return;
    }

    DispenserInputs in;
    dispenserReadInputs(millis(), in);

    bool ok = !in.seederAlive && in.haveTelemetry;

    reportCheck("V07", ok, "after %lu ms of silence: alive %d, haveTelemetry %d",
                (unsigned long)(LINK_TIMEOUT_MS + 200), in.seederAlive ? 1 : 0,
                in.haveTelemetry ? 1 : 0);
}

static void testV08()
{
    TestMarker tm("V08");

    dispenserResetInbox();

    Injector inj;
    injectorDefaults(inj);
    uint8_t buf[64];

    dispenserHandlePacket(buf, buildSeederPacket(inj, millis(), buf));
    dispenserHandlePacket(buf, buildTractorPacket(inj, millis(), buf));

    uint8_t wanted = LINK_HEARD_SEEDER | LINK_HEARD_TRACTOR;
    uint8_t got    = dispenserLinkFlags();

    if (!silentWait(LINK_TIMEOUT_MS + 200)) {
        reportLine("V08", Outcome::Aborted, "aborted by key");
        return;
    }

    uint8_t after = dispenserLinkFlags();

    bool ok = (got == wanted) && (after == 0);

    reportCheck("V08", ok, "flags %u with both talking (want %u), %u after silence",
                (unsigned)got, (unsigned)wanted, (unsigned)after);
}

static void runPacketTests()
{
    Serial.println();
    Serial.println("--- V: packet path (no motor) ---");

    // An abort ends the whole command, here too.
    if (abortRequested) return; testV01();
    if (abortRequested) return; testV02();
    if (abortRequested) return; testV03();
    if (abortRequested) return; testV04();
    if (abortRequested) return; testV05();
    if (abortRequested) return; testV06();
    if (abortRequested) return; testV07();
    if (abortRequested) return; testV08();
}
