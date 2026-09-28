#pragma once

#include "machine_settings.h"

// ---------------------------------------------------------------------------
// Ground speed from the ground-wheel pulses.
//
// The speed is the distance covered by the last WHEEL_AVERAGE_INTERVALS gaps
// between pulses - five turns of the drive - divided by the time they took, so
// it changes smoothly rather than in steps. Nothing in here touches hardware,
// so it can be tested on a PC.
// ---------------------------------------------------------------------------

static constexpr uint8_t WHEEL_RING_SIZE = WHEEL_AVERAGE_INTERVALS + 1;

// Times of the latest pulses in microseconds, kept in a ring.
struct WheelPulses {
    uint64_t timesUs[WHEEL_RING_SIZE];
    uint8_t  newest;   // index of the newest time
    uint32_t total;    // pulses since boot, never reset
};

// Called from the pulse interrupt. Ignores a pulse too soon after the last
// one to be real.
inline void recordWheelPulse(WheelPulses &pulses, uint64_t nowUs)
{
    if (pulses.total > 0 && nowUs - pulses.timesUs[pulses.newest] < WHEEL_MIN_PULSE_GAP_US) return;

    pulses.newest = (pulses.newest + 1 == WHEEL_RING_SIZE) ? 0 : pulses.newest + 1;
    pulses.timesUs[pulses.newest] = nowUs;
    pulses.total++;
}

// Copies the times out, newest first. Returns how many there are.
inline uint8_t copyWheelPulses(const WheelPulses &pulses, uint64_t recentUs[])
{
    uint8_t count = (pulses.total < WHEEL_RING_SIZE) ? (uint8_t)pulses.total : WHEEL_RING_SIZE;
    for (uint8_t i = 0; i < count; i++) {
        recentUs[i] = pulses.timesUs[(pulses.newest + WHEEL_RING_SIZE - i) % WHEEL_RING_SIZE];
    }
    return count;
}

// Speed in mm/s from times given newest first, as copyWheelPulses returns
// them. nowUs must not be earlier than the newest pulse. mmPerPulse is the
// distance covered between two pulses for the seed-size gear in use, which the
// tractor sends in every command - the sensor is on the metering drive, so it
// is not a constant of the machine.
inline uint16_t wheelSpeedMmS(const uint64_t recentUs[], uint8_t count, uint64_t nowUs, uint16_t mmPerPulse)
{
    // No pulse for this long means stopped: at WHEEL_MIN_SPEED_MM_S one would
    // already have arrived. It scales with the calibration instead of being a
    // fixed timeout, and it does both jobs - "stopped now", and "there was a
    // stop inside the averaging window".
    const uint64_t stopUs = (uint64_t)mmPerPulse * 1000000ULL / WHEEL_MIN_SPEED_MM_S;

    if (count < 2) return 0;

    uint64_t sinceLastUs = nowUs - recentUs[0];
    if (sinceLastUs > stopUs) return 0;

    // Count the gaps since the machine last stopped. A gap longer than the
    // stop timeout is a stop, so nothing before it is averaged in: after a
    // stop, the speed comes from the second pulse onwards.
    uint8_t gaps = 0;
    while (gaps < WHEEL_AVERAGE_INTERVALS && gaps + 1 < count &&
           recentUs[gaps] - recentUs[gaps + 1] <= stopUs) {
        gaps++;
    }
    if (gaps == 0) return 0;

    // Whole turns of the drive only - the newest ones - as soon as there is
    // one, so uneven spacing between the magnets cancels out while the average
    // builds up after a start too, not just once it spans all
    // WHEEL_AVERAGE_INTERVALS (itself whole turns).
    if (gaps >= WHEEL_MAGNETS) gaps -= gaps % WHEEL_MAGNETS;

    uint64_t spanUs = recentUs[0] - recentUs[gaps];
    if (spanUs == 0) return 0;

    uint64_t speed = (uint64_t)gaps * mmPerPulse * 1000000ULL / spanUs;

    // If the next pulse is late, the wheel has slowed: lower the speed now
    // rather than holding it until the stop rule fires. Late means later than
    // the longest gap in the average. Every pair of magnets is in there, so
    // uneven spacing can't cause a false drop - and neither can the ordinary
    // scatter of one gap against the next, which with a single gap as the
    // reference pulled the speed down for a moment in about every other gap,
    // and the number on the work screen twitched with it.
    //
    // The two rules meet exactly: at a steady speed the decayed value reaches
    // WHEEL_MIN_SPEED_MM_S at the moment sinceLastUs reaches stopUs, so the
    // speed slides down to the stop instead of stepping to 0 from wherever it
    // happened to be.
    uint64_t longestUs = 0;
    for (uint8_t i = 0; i < gaps; i++) {
        uint64_t gapUs = recentUs[i] - recentUs[i + 1];
        if (gapUs > longestUs) longestUs = gapUs;
    }

    if (sinceLastUs > longestUs) {
        speed = speed * longestUs / sinceLastUs;
    }

    return (speed > UINT16_MAX) ? UINT16_MAX : (uint16_t)speed;
}
