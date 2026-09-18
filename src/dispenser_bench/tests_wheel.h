#pragma once

#include <math.h>

#include "machine_settings.h"
#include "../seeder/wheel_speed.h"
#include "bench_report.h"

// ---------------------------------------------------------------------------
// W tests: the seeder's ground-speed logic (src/seeder/wheel_speed.h), driven
// with made-up pulse times. It has no hardware code in it, and this image is
// the only place in the project that can run it at all - so the dispenser's
// bench carries it even though the code belongs to the other board.
//
// The numbers behind the rules: the sensor is on the metering drive, so one
// pulse is about 0.8 m with 6 magnets. That is a pulse every 0.35 s at 8 km/h
// but only every 1.4 s at 2 km/h, which is why "stopped" is a speed
// (WHEEL_MIN_SPEED_MM_S) and not a fixed timeout, and why the average spans a
// whole turn of the drive.
// ---------------------------------------------------------------------------

struct WHarness {
    WheelPulses pulses;
    uint64_t    nowUs;
};

static void wFresh(WHarness &h)
{
    h.pulses = WheelPulses();
    h.nowUs  = 1000000ULL;
}

static void wAdvance(WHarness &h, uint64_t us) { h.nowUs += us; }
static void wPulse(WHarness &h)                { recordWheelPulse(h.pulses, h.nowUs); }

static uint16_t wSpeed(const WHarness &h, uint16_t mmPerPulse)
{
    uint64_t recent[WHEEL_RING_SIZE];
    uint8_t  count = copyWheelPulses(h.pulses, recent);
    return wheelSpeedMmS(recent, count, h.nowUs, mmPerPulse);
}

// Time between pulses at a given speed.
static uint64_t wGapUs(uint16_t mmPerPulse, uint32_t speedMmS)
{
    return (uint64_t)mmPerPulse * 1000000ULL / speedMmS;
}

// Feeds `count` pulses as if the machine had run at speedMmS the whole time,
// and leaves the clock on the last one.
static void wRun(WHarness &h, uint16_t mmPerPulse, uint32_t speedMmS, uint8_t count)
{
    uint64_t gap = wGapUs(mmPerPulse, speedMmS);
    for (uint8_t i = 0; i < count; i++) {
        wAdvance(h, gap);
        wPulse(h);
    }
}

static bool wWithin(uint16_t got, uint32_t want, double fraction)
{
    return fabs((double)got - (double)want) <= fraction * (double)want;
}

// How long a missing pulse takes to mean "stopped" at a given distance.
static uint64_t wStopUs(uint16_t mmPerPulse)
{
    return (uint64_t)mmPerPulse * 1000000ULL / WHEEL_MIN_SPEED_MM_S;
}

static void runWheelTests()
{
    const uint16_t mm = WHEEL_MM_PER_PULSE_DEFAULT;

    Serial.println();
    Serial.println("--- W: ground speed from wheel pulses (no hardware) ---");

    // W01 - a steady 8 km/h.
    {
        WHarness h; wFresh(h);
        wRun(h, mm, 2222, 10);
        uint16_t speed = wSpeed(h, mm);
        reportCheck("W01", wWithin(speed, 2222, 0.02),
                    "steady 8 km/h: %u mm/s (want 2222 +-2 %%)", (unsigned)speed);
    }

    // W02 - the speed halves and the average follows within a turn.
    {
        WHarness h; wFresh(h);
        wRun(h, mm, 2222, 10);
        wRun(h, mm, 1111, 8);
        uint16_t speed = wSpeed(h, mm);
        reportCheck("W02", wWithin(speed, 1111, 0.05),
                    "8 -> 4 km/h: %u mm/s one turn later (want 1111 +-5 %%)", (unsigned)speed);
    }

    // W03 - a late pulse brings the speed down smoothly, not in a step.
    {
        WHarness h; wFresh(h);
        wRun(h, mm, 2222, 10);
        uint16_t a = wSpeed(h, mm);
        wAdvance(h, 500000);  uint16_t b = wSpeed(h, mm);
        wAdvance(h, 500000);  uint16_t c = wSpeed(h, mm);
        wAdvance(h, 500000);  uint16_t d = wSpeed(h, mm);
        bool ok = (a > b) && (b > c) && (c > d) && (d > 0);
        reportCheck("W03", ok, "pulse late: %u -> %u -> %u -> %u mm/s, none 0 before the stop rule",
                    (unsigned)a, (unsigned)b, (unsigned)c, (unsigned)d);
    }

    // W04 - the stop rule fires exactly where it should, and the speed slides
    // into it: the last value before 0 is WHEEL_MIN_SPEED_MM_S.
    {
        WHarness h; wFresh(h);
        wRun(h, mm, 2222, 10);
        uint64_t stopUs = wStopUs(mm);
        wAdvance(h, stopUs - 20000);
        uint16_t justBefore = wSpeed(h, mm);
        wAdvance(h, 40000);
        uint16_t justAfter = wSpeed(h, mm);
        bool ok = (justBefore > 0) && (justAfter == 0) &&
                  wWithin(justBefore, WHEEL_MIN_SPEED_MM_S, 0.10);
        reportCheck("W04", ok, "at the stop rule (%lu ms): %u mm/s, then 0 (want %u +-10 %%)",
                    (unsigned long)(stopUs / 1000), (unsigned)justBefore,
                    (unsigned)WHEEL_MIN_SPEED_MM_S);
    }

    // W05 - starting again. The first pulse after a stop is not movement (one
    // pulse is not a speed), and the second must not be measured across the stop.
    {
        WHarness h; wFresh(h);
        wRun(h, mm, 2222, 10);
        wAdvance(h, 10000000);                       // 10 s standing still
        uint16_t stopped = wSpeed(h, mm);
        wPulse(h);
        uint16_t first = wSpeed(h, mm);
        wAdvance(h, wGapUs(mm, 1111));
        wPulse(h);
        uint16_t second = wSpeed(h, mm);
        bool ok = (stopped == 0) && (first == 0) && wWithin(second, 1111, 0.05);
        reportCheck("W05", ok, "restart: stopped %u, first pulse %u, second %u mm/s (want 0, 0, 1111)",
                    (unsigned)stopped, (unsigned)first, (unsigned)second);
    }

    // W06 - one stray pulse while stopped is not movement.
    {
        WHarness h; wFresh(h);
        wPulse(h);
        uint16_t atOnce = wSpeed(h, mm);
        wAdvance(h, 1000000);
        uint16_t later = wSpeed(h, mm);
        reportCheck("W06", (atOnce == 0) && (later == 0),
                    "one stray pulse: %u then %u mm/s (want 0, 0)",
                    (unsigned)atOnce, (unsigned)later);
    }

    // W07 - magnets glued on by hand are not evenly spaced. Averaging whole
    // turns is what cancels that, so the deviations here sum to one turn.
    {
        WHarness h; wFresh(h);
        const double dev[6] = {1.10, 0.90, 1.05, 0.95, 1.00, 1.00};
        uint64_t base = wGapUs(mm, 2222);
        bool ok = true;
        uint16_t last = 0;
        for (int turn = 0; turn < 4; turn++) {
            for (int i = 0; i < 6; i++) {
                wAdvance(h, (uint64_t)(base * dev[i]));
                wPulse(h);
                if (turn >= 2) {
                    last = wSpeed(h, mm);
                    if (!wWithin(last, 2222, 0.02)) ok = false;
                }
            }
        }
        reportCheck("W07", ok, "magnets +-10 %% uneven: %u mm/s at every pulse of the last turns "
                    "(want 2222 +-2 %%)", (unsigned)last);
    }

    // W08 - the ISR's noise gate.
    {
        WHarness h; wFresh(h);
        wPulse(h);
        wAdvance(h, WHEEL_MIN_PULSE_GAP_US / 2);
        wPulse(h);
        uint32_t afterNoise = h.pulses.total;
        wAdvance(h, WHEEL_MIN_PULSE_GAP_US * 2);
        wPulse(h);
        uint32_t afterReal = h.pulses.total;
        reportCheck("W08", (afterNoise == 1) && (afterReal == 2),
                    "noise gate %lu us: %lu pulse after a half-gap edge, %lu after a real one",
                    (unsigned long)WHEEL_MIN_PULSE_GAP_US,
                    (unsigned long)afterNoise, (unsigned long)afterReal);
    }

    // W09 - the two seed-size settings. The same pulses, a different distance
    // per pulse, so the speeds are in the same ratio as the two values.
    {
        const uint16_t large = 980;
        WHarness h; wFresh(h);
        wRun(h, mm, 2222, 10);
        uint16_t small = wSpeed(h, mm);
        uint16_t big   = wSpeed(h, large);
        double   ratio = (small > 0) ? (double)big / (double)small : 0.0;
        bool     ok    = fabs(ratio - (double)large / (double)mm) < 0.02;
        reportCheck("W09", ok, "%u mm vs %u mm per pulse: %u and %u mm/s, ratio %.3f (want %.3f)",
                    (unsigned)mm, (unsigned)large, (unsigned)small, (unsigned)big,
                    ratio, (double)large / (double)mm);
    }

    // W10 - 1.5 km/h, a pulse every 1.9 s. The old fixed 2 s timeout called
    // this stopped; the speed rule must not.
    {
        WHarness h; wFresh(h);
        wRun(h, mm, 417, 10);
        uint16_t speed = wSpeed(h, mm);
        reportCheck("W10", (speed > 0) && wWithin(speed, 417, 0.10),
                    "1.5 km/h (a pulse every %lu ms): %u mm/s (want 417 +-10 %%)",
                    (unsigned long)(wGapUs(mm, 417) / 1000), (unsigned)speed);
    }

    // W11 - an impossible speed clamps instead of wrapping. Built by hand: the
    // ISR's noise gate would never let gaps this short through.
    {
        uint64_t times[WHEEL_RING_SIZE];
        for (uint8_t i = 0; i < WHEEL_RING_SIZE; i++) {
            times[i] = 10000000ULL - (uint64_t)i * 10000ULL;   // newest first, 10 ms apart
        }
        uint16_t speed = wheelSpeedMmS(times, WHEEL_RING_SIZE, 10000000ULL, WHEEL_MM_PER_PULSE_MAX);
        reportCheck("W11", speed == UINT16_MAX,
                    "absurd speed clamps to %u mm/s (want %u)", (unsigned)speed, (unsigned)UINT16_MAX);
    }
}
