#pragma once

#include <Arduino.h>

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_io.h"

// ---------------------------------------------------------------------------
// The simulated seeder and tractor. Packets are built with the real structs
// and fillHeader(), then handed to dispenserHandlePacket() - the same function
// the production receive callback calls - so validation, the inbox and the
// link timing are exercised exactly as in the field.
//
// Nothing here ever registers a receive callback: a real tractor or seeder
// nearby must not be able to interfere with (or be commanded by) the bench.
// ---------------------------------------------------------------------------

struct SimSeeder {
    bool     sending  = false;
    uint32_t interval = SEND_INTERVAL_MS;
    uint32_t nextDue  = 0;
    uint16_t groundSpeedMmS = 1000;
    uint8_t  wheelTurning   = 1;

    // Jitter mode: the interval is drawn from [jitterMin, jitterMax] with a
    // fixed seed, so a failing run can be repeated exactly.
    bool     jitter    = false;
    uint32_t jitterMin = 100;
    uint32_t jitterMax = 800;
    uint32_t rng       = 0x12345678u;

    // Cumulative wheel pulses, advanced from the speed actually sent, over the
    // time since the previous seeder packet.
    double   distanceMm  = 0.0;
    uint32_t wheelPulses = 0;
    uint32_t lastSentMs  = 0;
};

struct SimTractor {
    bool     sending  = false;
    uint32_t interval = SEND_INTERVAL_MS;
    uint32_t nextDue  = 0;

    uint8_t  dispenserEnabled = 1;
    uint16_t doseKgPerHa      = 40;
    uint32_t gramsPer100Rev   = 500;
    uint8_t  calibrationRun   = 0;
    uint8_t  clogClearSeq     = 0;
    uint8_t  unclogSeq        = 0;

    // The simulated seeder turns its speed into wheelPulses with this same
    // value, so the two sides of the dispenser's distance ledger agree - as
    // they do on the machine, where the tractor sends the number both boards
    // meter with.
    uint16_t wheelMmPerPulse  = WHEEL_MM_PER_PULSE_DEFAULT;

    // upTimeMs = bench time + this offset, so a test can make it go backwards
    // (a tractor reboot) by lowering it.
    int32_t  upTimeOffset = 0;
};

struct Injector {
    SimSeeder  seeder;
    SimTractor tractor;
};

static void injectorDefaults(Injector &inj)
{
    inj.seeder  = SimSeeder();
    inj.tractor = SimTractor();
    inj.seeder.sending  = false;
    inj.tractor.sending = false;
    inj.seeder.lastSentMs = millis();
}

static void injectorSilence(Injector &inj)
{
    inj.seeder.sending  = false;
    inj.tractor.sending = false;
}

// Makes the next packets due immediately, without changing who is talking.
static void injectorSchedule(Injector &inj, uint32_t now)
{
    inj.seeder.nextDue    = now;
    inj.tractor.nextDue   = now;
    inj.seeder.lastSentMs = now;   // the next packet measures its distance from here
}

// Both boards talking, from now on.
static void injectorStart(Injector &inj, uint32_t now)
{
    inj.seeder.sending  = true;
    inj.tractor.sending = true;
    injectorSchedule(inj, now);
}

// The tractor's upTimeMs jumps to `value` - a reboot, as far as the dispenser
// can tell.
static void injectorFakeReboot(Injector &inj, uint32_t value)
{
    inj.tractor.upTimeOffset = (int32_t)((int64_t)value - (int64_t)millis());
}

// One packet per call, no rate limiting - used by the packet tests.
static int buildSeederPacket(const Injector &inj, uint32_t now, uint8_t *buf)
{
    SeederTelemetry packet;
    memset(&packet, 0, sizeof(packet));
    fillHeader(packet.header, MsgType::SeederTelemetry, NodeId::Seeder,
               LINK_HEARD_TRACTOR | LINK_HEARD_DISPENSER);
    packet.upTimeMs       = now;
    packet.wheelPulses    = inj.seeder.wheelPulses;
    packet.turbineRPM     = 3000;
    packet.womRPM         = 540;
    packet.groundSpeedMmS = inj.seeder.groundSpeedMmS;
    packet.wheelTurning   = inj.seeder.wheelTurning;

    memcpy(buf, &packet, sizeof(packet));
    return (int)sizeof(packet);
}

static int buildTractorPacket(const Injector &inj, uint32_t now, uint8_t *buf)
{
    TractorCommand packet;
    memset(&packet, 0, sizeof(packet));
    fillHeader(packet.header, MsgType::TractorCommand, NodeId::Tractor,
               LINK_HEARD_SEEDER | LINK_HEARD_DISPENSER);
    packet.tramlineNumber   = 0;
    packet.tramlineRelayOn  = 1;
    packet.dispenserEnabled = inj.tractor.dispenserEnabled;
    packet.calibrationRun   = inj.tractor.calibrationRun;
    packet.doseKgPerHa      = inj.tractor.doseKgPerHa;
    packet.gramsPer100Rev   = inj.tractor.gramsPer100Rev;
    packet.upTimeMs         = (uint32_t)((int64_t)now + inj.tractor.upTimeOffset);
    packet.clogClearSeq     = inj.tractor.clogClearSeq;
    packet.unclogSeq        = inj.tractor.unclogSeq;
    packet.wheelMmPerPulse  = inj.tractor.wheelMmPerPulse;

    memcpy(buf, &packet, sizeof(packet));
    return (int)sizeof(packet);
}

static uint32_t nextJitterInterval(SimSeeder &seeder)
{
    seeder.rng = seeder.rng * 1664525u + 1013904223u;
    uint32_t span = seeder.jitterMax - seeder.jitterMin + 1;
    return seeder.jitterMin + ((seeder.rng >> 16) % span);
}

// Sends whatever is due. Called from every loop of the hardware-in-the-loop
// tests, so the rate the real boards would produce is reproduced.
static void injectorPump(Injector &inj, uint32_t now)
{
    if (inj.seeder.sending && (int32_t)(now - inj.seeder.nextDue) >= 0) {
        // Distance accumulates over the time since the previous seeder packet,
        // so wheelPulses behaves like the real cumulative counter - it is the
        // distance covered, not the number of pumps.
        uint32_t sinceLast = now - inj.seeder.lastSentMs;
        inj.seeder.lastSentMs = now;

        inj.seeder.distanceMm += (double)inj.seeder.groundSpeedMmS * (double)sinceLast / 1000.0;
        while (inj.seeder.distanceMm >= (double)inj.tractor.wheelMmPerPulse) {
            inj.seeder.distanceMm -= (double)inj.tractor.wheelMmPerPulse;
            inj.seeder.wheelPulses++;
        }

        uint8_t buf[64];
        int len = buildSeederPacket(inj, now, buf);
        dispenserHandlePacket(buf, len);

        inj.seeder.nextDue = now + (inj.seeder.jitter ? nextJitterInterval(inj.seeder)
                                                      : inj.seeder.interval);
    }

    if (inj.tractor.sending && (int32_t)(now - inj.tractor.nextDue) >= 0) {
        uint8_t buf[64];
        int len = buildTractorPacket(inj, now, buf);
        dispenserHandlePacket(buf, len);
        inj.tractor.nextDue = now + inj.tractor.interval;
    }
}

// One deliberately malformed packet: a valid one with a single byte or the
// length broken. Used by the packet tests.
static int buildBrokenSeederPacket(const Injector &inj, uint32_t now, uint8_t *buf, uint8_t which)
{
    int len = buildSeederPacket(inj, now, buf);
    switch (which) {
        case 0:  buf[0] = 'X'; break;                       // magic0
        case 1:  buf[1] = 'X'; break;                       // magic1
        case 2:  buf[2] = PROTOCOL_VERSION - 1; break;
        case 3:  buf[2] = PROTOCOL_VERSION + 1; break;
        case 4:  buf[3] = NETWORK_ID + 1; break;
        case 5:  buf[4] = (uint8_t)MsgType::TractorCommand; break;
        case 6:  buf[5] = (uint8_t)NodeId::Tractor; break;
        default: break;
    }
    return len;
}
