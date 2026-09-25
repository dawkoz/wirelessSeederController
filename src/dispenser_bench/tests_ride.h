#pragma once

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_io.h"
#include "bench_inject.h"
#include "bench_report.h"

// ---------------------------------------------------------------------------
// P01 - one simulated pass, for watching the dispenser rather than for
// measuring it: the tractor and the seeder talk as they would on the machine,
// which stands, pulls away, seeds, slows down and stands again. A readout line
// twice a second shows what the dispenser made of it.
//
// The speed is what the seeder *reports*, ramped smoothly. The real sensor
// lags by about a turn of the metering drive and needs a couple of pulses
// before it reports anything - that is the W tests' subject, not this one's.
// Below WHEEL_MIN_SPEED_MM_S the simulated seeder says "stopped", as the real
// one does.
//
// Deliberately not part of 'a': it proves nothing the M tests don't, and it
// is meant to be run on its own, as often as you like. Change the pass here.
// ---------------------------------------------------------------------------

static constexpr uint16_t RIDE_SPEED_MM_S  = 2222;   // 8 km/h, a normal seeding speed
static constexpr uint16_t RIDE_DOSE_KG_HA  = 40;
static constexpr uint32_t RIDE_GRAMS       = 1000;   // per 100 revolutions: 213 RPM at 8 km/h
static constexpr uint32_t RIDE_STANDING_MS = 2000;   // before pulling away
static constexpr uint32_t RIDE_ACCEL_MS    = 3000;   // standstill to full speed
static constexpr uint32_t RIDE_SEED_MS     = 6000;   // at full speed
static constexpr uint32_t RIDE_DECEL_MS    = 3000;   // full speed to standstill
static constexpr uint32_t RIDE_STOPPED_MS  = 2000;   // after the stop
static constexpr uint32_t RIDE_PRINT_MS    = 500;    // one readout line this often

static constexpr uint32_t RIDE_ACCEL_END_MS = RIDE_STANDING_MS + RIDE_ACCEL_MS;
static constexpr uint32_t RIDE_SEED_END_MS  = RIDE_ACCEL_END_MS + RIDE_SEED_MS;
static constexpr uint32_t RIDE_DECEL_END_MS = RIDE_SEED_END_MS + RIDE_DECEL_MS;
static constexpr uint32_t RIDE_TOTAL_MS     = RIDE_DECEL_END_MS + RIDE_STOPPED_MS;

static_assert(RIDE_SPEED_MM_S > WHEEL_MIN_SPEED_MM_S, "the pass must actually move");
// The stop is checked 1 s after the seeder says "stopped", over at least 0.5 s.
static_assert(RIDE_STOPPED_MS >= 1500, "RIDE_STOPPED_MS is too short for the stop check");

// The seeder's reported speed, t ms into the pass.
static uint16_t rideSpeedAt(uint32_t t)
{
    if (t < RIDE_STANDING_MS)  return 0;
    if (t < RIDE_ACCEL_END_MS) return (uint16_t)((uint32_t)RIDE_SPEED_MM_S * (t - RIDE_STANDING_MS) / RIDE_ACCEL_MS);
    if (t < RIDE_SEED_END_MS)  return RIDE_SPEED_MM_S;
    if (t < RIDE_DECEL_END_MS) return (uint16_t)((uint32_t)RIDE_SPEED_MM_S * (RIDE_DECEL_END_MS - t) / RIDE_DECEL_MS);
    return 0;
}

static const char *ridePhaseAt(uint32_t t)
{
    if (t < RIDE_STANDING_MS)  return "standing";
    if (t < RIDE_ACCEL_END_MS) return "pulling away";
    if (t < RIDE_SEED_END_MS)  return "seeding";
    if (t < RIDE_DECEL_END_MS) return "stopping";
    return "stopped";
}

static void runRideTest()
{
    Serial.println();
    Serial.println("--- P: one simulated pass (motor, free shaft) ---");
    Serial.printf(" %.1f km/h, %u kg/ha, %lu g per 100 rev: %.0f RPM while seeding\n",
                  (double)RIDE_SPEED_MM_S * 0.0036, (unsigned)RIDE_DOSE_KG_HA, (unsigned long)RIDE_GRAMS,
                  independentRPM(RIDE_SPEED_MM_S, RIDE_DOSE_KG_HA, RIDE_GRAMS));
    Serial.printf(" %.0f s standing, %.0f s pulling away, %.0f s seeding, %.0f s stopping, %.0f s standing\n",
                  RIDE_STANDING_MS / 1000.0, RIDE_ACCEL_MS / 1000.0, RIDE_SEED_MS / 1000.0,
                  RIDE_DECEL_MS / 1000.0, RIDE_STOPPED_MS / 1000.0);
    Serial.println(" rate = what the reported speed asks for; target = the dispenser's, with the ledger's");
    Serial.println(" trim; shaft = measured over the last half second; duty in permille. Any key aborts.");

    TestMarker tm("P01");
    MotorGuard guard;

    if (!benchPrepare()) { reportLine("P01", Outcome::Aborted, "aborted by key"); return; }
    injectorDefaults(injector);
    injector.tractor.doseKgPerHa    = RIDE_DOSE_KG_HA;
    injector.tractor.gramsPer100Rev = RIDE_GRAMS;
    injector.seeder.groundSpeedMmS  = 0;
    injector.seeder.wheelTurning    = 0;
    injectorStart(injector, millis());

    Serial.println("      t  phase           ground  rate  target  shaft  duty");

    uint32_t start     = millis();
    uint32_t edges0    = encoderEdges();
    uint32_t pulses0   = injector.seeder.wheelPulses;
    double   distance0 = injector.seeder.distanceMm;

    uint32_t nextPrint      = 0;          // ms into the pass
    uint32_t lastPrintMs    = start;
    uint32_t lastPrintEdges = edges0;

    // Before pulling away: the shaft must not move.
    bool     standingTaken   = false;
    uint32_t standingEdges   = 0;
    uint16_t standingMaxDuty = 0;

    // After the stop: duty 0 within 300 ms and nothing after that, and the
    // shaft still a second later.
    bool     moved          = false;
    bool     stopSent       = false;
    uint32_t stopMs         = 0;
    bool     dutyZeroSeen   = false;
    uint32_t dutyZeroAfter  = 0;
    uint16_t dutyAfterStop  = 0;
    bool     stillTaken     = false;
    uint32_t stillEdges0    = 0;
    uint32_t stillMs0       = 0;

    while (true) {
        if (!benchPump()) { reportLine("P01", Outcome::Aborted, "aborted by key"); return; }

        uint32_t now = millis();
        uint32_t t   = now - start;
        if (t >= RIDE_TOTAL_MS) break;

        uint16_t speed  = rideSpeedAt(t);
        bool     moving = speed >= WHEEL_MIN_SPEED_MM_S;
        injector.seeder.groundSpeedMmS = moving ? speed : 0;
        injector.seeder.wheelTurning   = moving ? 1 : 0;

        if (moving) moved = true;
        if (moved && !moving && !stopSent) {
            // The seeder says "stopped" now, not at its next regular send, so
            // the stop is timed from the packet that carries it.
            injectorSchedule(injector, now);
            stopSent = true;
            stopMs   = now;
        }

        if (t < RIDE_STANDING_MS && out.motorPermille > standingMaxDuty) standingMaxDuty = out.motorPermille;
        if (!standingTaken && t >= RIDE_STANDING_MS) {
            standingEdges = encoderEdges() - edges0;
            standingTaken = true;
        }

        if (stopSent) {
            uint32_t since = now - stopMs;
            if (!dutyZeroSeen && out.motorPermille == 0) {
                dutyZeroSeen  = true;
                dutyZeroAfter = since;
            }
            if (since >= 300 && out.motorPermille > dutyAfterStop) dutyAfterStop = out.motorPermille;
            if (!stillTaken && since >= 1000) {
                stillTaken  = true;
                stillEdges0 = encoderEdges();
                stillMs0    = now;
            }
        }

        if (t >= nextPrint) {
            uint32_t edgesNow = encoderEdges();
            uint32_t span     = now - lastPrintMs;
            double   shaft    = (span == 0) ? 0.0
                              : (double)(edgesNow - lastPrintEdges) * 60000.0 /
                                ((double)span * (double)ENCODER_EDGES_PER_REV);
            lastPrintMs    = now;
            lastPrintEdges = edgesNow;
            nextPrint     += RIDE_PRINT_MS;

            const char *flag = "";
            if (logic.mode != DispenserMode::Normal)       flag = modeName(logic.mode);
            else if (logic.fault != DispenserFault::None)  flag = faultName(logic.fault);

            Serial.printf(" %5.1fs  %-12s %4.1f km/h %5.0f %7u %6.0f %5u%s%s\n",
                          t / 1000.0, ridePhaseAt(t), (double)injector.seeder.groundSpeedMmS * 0.0036,
                          independentRPM(injector.seeder.groundSpeedMmS, RIDE_DOSE_KG_HA, RIDE_GRAMS),
                          (unsigned)logic.targetRPM, shaft, (unsigned)out.motorPermille,
                          flag[0] ? "  " : "", flag);
        }

        delay(1);
    }

    uint32_t endMs = millis();

    // What the shaft delivered against the ground the seeder reported. For
    // information only: the ledger drops whatever it still owes at a stop, so
    // a short pass with ramps is not held to M11's 3 %.
    double turns  = (double)(encoderEdges() - edges0) / (double)ENCODER_EDGES_PER_REV;
    double metres = ((double)(injector.seeder.wheelPulses - pulses0) * (double)injector.tractor.wheelMmPerPulse
                     + (injector.seeder.distanceMm - distance0)) / 1000.0;
    double want   = metres * (double)WORKING_WIDTH_CM * (double)RIDE_DOSE_KG_HA / (10.0 * (double)RIDE_GRAMS);

    Serial.printf("  %.1f m of ground, %.1f shaft turns - the dose asks for %.1f (%+.1f %%), for information\n",
                  metres, turns, want, (want > 0.0) ? 100.0 * (turns - want) / want : 0.0);

    double stillRPM = 0.0;
    if (stillTaken && endMs > stillMs0) {
        stillRPM = (double)(encoderEdges() - stillEdges0) * 60000.0 /
                   ((double)(endMs - stillMs0) * (double)ENCODER_EDGES_PER_REV);
    }

    bool standingOk = standingTaken && (standingEdges <= 2) && (standingMaxDuty == 0);
    bool zeroOk     = stopSent && dutyZeroSeen && (dutyZeroAfter <= 300);
    bool stayedOk   = stopSent && (dutyAfterStop == 0);
    bool stillOk    = stillTaken && (stillRPM < 5.0);

    reportCheck("P01", standingOk && zeroOk && stayedOk && stillOk && !gSawClogged,
                "still while standing %s; duty 0 within 300 ms of the stop %s (%lu ms), stayed 0 %s; "
                "%.1f RPM 1 s later %s; never Clogged %s",
                standingOk ? "yes" : "NO", zeroOk ? "yes" : "NO",
                (unsigned long)(dutyZeroSeen ? dutyZeroAfter : endMs - stopMs),
                stayedOk ? "yes" : "NO", stillRPM, stillOk ? "yes" : "NO",
                gSawClogged ? "NO" : "yes");
}
