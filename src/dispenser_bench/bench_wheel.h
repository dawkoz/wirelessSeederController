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
