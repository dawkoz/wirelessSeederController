#pragma once

#include <stdint.h>

// The distance per wheel pulse every bench test simulates: the tractor the
// bench fakes sends it, and the seeder it fakes turns ground speed into pulses
// with it. It is the value the tests were written and checked against, when
// six magnets were planned for the metering drive. It stayed when the machine
// kept its three (WHEEL_MM_PER_PULSE_DEFAULT is now 1571): the logic under
// test takes whatever distance the tractor sends, so this only fixes the
// numbers the tests were checked with, and the machine's default no longer
// moves them.
static constexpr uint16_t BENCH_WHEEL_MM_PER_PULSE = 785;

// The burst-mode angle factor the bench's simulated tractor sends. 1778 at
// 785 mm per pulse and full PWM is 2.512 turns a pulse - what the tests' 40
// kg/ha at 500 g per 100 asked for before the factor existed, so every number
// the burst tests were checked with still holds. It is above the tractor's 999,
// which only the tractor limits: the dispenser takes whatever the wire carries.
static constexpr uint16_t BENCH_ANGLE_FACTOR = 1778;
