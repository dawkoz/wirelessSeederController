#pragma once

#include <stdint.h>

#include "machine_settings.h"

// ---------------------------------------------------------------------------
// Burst metering's angle factor, the tractor's side of it: what the calibration
// run should weigh, and the factor that follows from a weighing or a new dose.
//
// The dispenser turns the factor into the angle each wheel pulse's burst turns
// (burstRevsPerPulse() in src/dispenser/dispenser_logic.h), and every angle is
// in proportion to it - so all the tractor ever does with it is scale it.
// Nothing in here touches hardware, so the dispenser bench runs the whole
// procedure through it (B30).
// ---------------------------------------------------------------------------

// Three digits on the Kalibracja screen. 1000 would be the motor turning flat
// out the whole way from one pulse to the next at BURST_ANGLE_REFERENCE_SPEED_MM_S
// (10 km/h), so 999 is also about the most a pulse can ask for at that speed.
static constexpr uint16_t ANGLE_FACTOR_MAX = 999;

// What CALIBRATION_PULSES wheel pulses' worth of ground should get at this
// dose, in grams: the mass the calibration run's bursts should weigh. A pulse
// covers mmPerPulse by WORKING_WIDTH_CM, so it gets mm x cm x kg/ha / 1,000,000
// grams - 25.1 g at 1571 mm, 4 m and 40 kg/ha, 503 g for the run. Rounded to the
// gram, which is what the screens show and what the weighing is compared with.
// 64-bit on the way: 20 pulses of 5 m at 999 kg/ha do not fit in 32 bits.
inline uint32_t expectedCalibrationGrams(uint16_t doseKgPerHa, uint16_t mmPerPulse)
{
    uint64_t product = (uint64_t)CALIBRATION_PULSES * mmPerPulse * WORKING_WIDTH_CM * doseKgPerHa;
    return (uint32_t)((product + 500000ULL) / 1000000ULL);
}

// factor x num / den, rounded, and kept within 1..ANGLE_FACTOR_MAX: never 0,
// which would stop the dosing altogether, and never past what the screen shows.
// A factor of 0, or nothing to scale by, stays as it is.
inline uint16_t scaleAngleFactor(uint16_t factor, uint32_t num, uint32_t den)
{
    if (factor == 0 || num == 0 || den == 0) return factor;
    uint64_t scaled = ((uint64_t)factor * num * 2 + den) / (2ULL * den);
    if (scaled < 1)                scaled = 1;
    if (scaled > ANGLE_FACTOR_MAX) scaled = ANGLE_FACTOR_MAX;
    return (uint16_t)scaled;
}

// After the calibration run: the factor that would have made the weighed mass
// the expected one. The run's angle is in proportion to the factor, and so is
// what it delivers - 548 g weighed for 503 g expected takes 500 to 459.
inline uint16_t angleFactorFromWeighing(uint16_t factor, uint32_t expectedGrams, uint32_t weighedGrams)
{
    return scaleAngleFactor(factor, expectedGrams, weighedGrams);
}

// A new dose: the angle a pulse needs is in proportion to it, so the factor
// follows - 40 to 50 kg/ha takes 500 to 625. A dose of 0 on either side leaves
// it alone, since there is no proportion to keep.
inline uint16_t angleFactorForDose(uint16_t factor, uint16_t oldDose, uint16_t newDose)
{
    return scaleAngleFactor(factor, newDose, oldDose);
}
