#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Machine settings
//
// Every value you might want to change for this machine, grouped by the board
// that uses it. After changing a value, reflash the boards its section names.
//
// MEASURE marks an estimate that has to be measured on the machine.
// ---------------------------------------------------------------------------

// ===========================================================================
// SHARED: values for the whole machine. Reflash all three boards after a change.
// ===========================================================================

// --- Machine ----------------------------------------------------------------

static constexpr uint32_t WORKING_WIDTH_CM = 400;        // 4.00 m
static constexpr uint16_t CALIBRATION_REVOLUTIONS = 100; // dispenser shaft turns per calibration run (continuous)
// Wheel pulses the burst-mode calibration run doses: the dispenser fires that
// many bursts, and the tractor weighs them against what that much ground should
// get (see the burst metering section below).
static constexpr uint8_t CALIBRATION_PULSES = 20;

// --- Serial -----------------------------------------------------------------

// Serial monitor speed on every board, and what the tractor's Ustawienia screen
// tells the operator to set on the laptop. Keep it equal to `monitor_speed` in
// platformio.ini.
static constexpr uint32_t SERIAL_BAUD = 115200;

// --- Radio link -------------------------------------------------------------

static constexpr uint8_t NETWORK_ID = 1; // change only if a machine nearby runs this firmware
static constexpr uint8_t ESPNOW_CHANNEL = 1;
static constexpr uint32_t SEND_INTERVAL_MS = 200; // every board sends this often
static constexpr uint32_t LINK_TIMEOUT_MS = 1000; // a board silent this long counts as disconnected

// ===========================================================================
// SEEDER
// ===========================================================================

// --- Pins -------------------------------------------------------------------

// All three are plain GPIOs: no strapping role, nothing driven during boot,
// an input with no pull-up or pull-down while in reset, and internal pull-ups
// and interrupts once running. During reset the relay board's own input circuit
// decides: an active-HIGH board stays off only if that input has a pull-down
// (most have one; if the relay clicks at power-up, fit 10k from the pin to
// GND). setup() drives the pin low - off - before anything else.
static constexpr uint8_t RELAY_PIN = 18;          // relay board input, active HIGH
static constexpr uint8_t TURBINE_SENSOR_PIN = 19; // inductive sensor
static constexpr uint8_t WHEEL_SENSOR_PIN = 22;   // Hall sensor on the metering drive (WHEEL_MAGNETS magnets)

// The fitted relay board closes on a high input: with the LOW/HIGH of the
// board it replaced, tramlines were on where they should be off and the other
// way round (found in the field, 29 September 2026).
static constexpr uint8_t RELAY_ON = HIGH;
static constexpr uint8_t RELAY_OFF = LOW;

// --- Ground wheel -----------------------------------------------------------

// Distance travelled between two wheel-sensor pulses. The sensor is on the
// metering drive, not on the ground wheel, so this depends on which seed-size
// gear the machine is in: the tractor keeps one measured value per setting and
// sends the active one in every packet (TractorCommand.wheelMmPerPulse). The
// default below is only used until the first command arrives, and as the
// fallback for a value outside the limits.
// MEASURE: Nasiona -> Kalibracja on the tractor does it, once per seed size.
// A 600 mm wheel turns the metering drive about 0.4 of a turn per turn, so one
// drive turn is ~4.71 m of ground, split between the magnets.
static constexpr uint16_t WHEEL_MM_PER_PULSE_DEFAULT = 1571; // 3 magnets, ratio 0.4, 600 mm wheel
static constexpr uint16_t WHEEL_MM_PER_PULSE_MIN = 100;
static constexpr uint16_t WHEEL_MM_PER_PULSE_MAX = 5000;

// Magnets on the metering drive. Six were planned, to halve the distance per
// pulse, but that close together the sensor could not tell one from the next
// (27 September 2026), so the drive keeps its three.
static constexpr uint8_t WHEEL_MAGNETS = 3;

// The speed is averaged over this many gaps between pulses: five turns of the
// drive, about 24 m, so the number on the work screen holds steady (the user's
// choice, 28 September 2026 - one turn, 3 gaps, made it twitch). A stop still
// shows at once: the late-pulse rule in src/seeder/wheel_speed.h and
// WHEEL_MIN_SPEED_MM_S do not wait for the average. Burst metering doses by the
// pulses and never reads the speed; continuous metering takes it as its
// feed-forward, and the distance ledger makes up the lag after a start. The
// price is a slower number: a change of speed takes 10-20 s to settle on the
// screen (8 -> 4 km/h about 19 s). 9 or 6 react faster and twitch a little
// more; 3 is the twitch this replaced.
static constexpr uint8_t WHEEL_AVERAGE_INTERVALS = 15;

// The slowest speed this sensor still calls "moving". A pulse that has not
// arrived within (distance per pulse / this) means the machine has stopped: at
// any higher speed it would already be here. One number with a physical
// meaning, in place of the old fixed 2 s timeout - which, on the metering drive,
// called a machine moving at 1.4 km/h stopped even with six magnets. The cost
// of lowering it is that a real stop takes (distance per pulse / this) to
// notice: 5.2 s at the default distance with three magnets.
static constexpr uint16_t WHEEL_MIN_SPEED_MM_S = 300; // ~1.1 km/h

static constexpr uint32_t WHEEL_MAX_SPEED_MM_S = 11000; // ~40 km/h, above any road speed
// A gap shorter than this is electrical noise, not a pulse. Derived from the
// default distance per pulse rather than the calibrated one: it is read in the
// ISR, so it has to be a compile-time constant.
static constexpr uint32_t WHEEL_MIN_PULSE_GAP_US =
    (uint32_t)((uint64_t)WHEEL_MM_PER_PULSE_DEFAULT * 1000000ULL / WHEEL_MAX_SPEED_MM_S);

// Whole wheel turns only, so uneven spacing between the magnets cancels out.
static_assert(WHEEL_AVERAGE_INTERVALS > 0 && WHEEL_AVERAGE_INTERVALS % WHEEL_MAGNETS == 0,
              "WHEEL_AVERAGE_INTERVALS must be a multiple of WHEEL_MAGNETS");

// --- Turbine ----------------------------------------------------------------

static constexpr uint32_t TURBINE_PULSES_PER_REV = 1; // holes in the turbine disc
static constexpr uint32_t TURBINE_UPDATE_INTERVAL_MS = 2000;
static constexpr uint32_t TURBINE_MIN_PULSE_GAP_US = 1000; // shorter gaps are electrical noise

// ===========================================================================
// TRACTOR
// ===========================================================================

// --- Pins -------------------------------------------------------------------

// The button closes to GND and has a 470 R pull-up to 3V3 (plus 10-100 nF to GND)
// at the board. It used to be on GPIO12, where only the internal ~45k pull-up
// was allowed - a strapping pin must never be pulled high - and ~0.07 mA is too
// little for a car-style switch: its contacts film over and miss the first
// press after a pause. 470 R puts ~7 mA through them. GPIO32 has no boot role.
static constexpr uint8_t BUTTON_PIN = 32;
static constexpr uint8_t GREEN_LED_PIN = 14;  // all links healthy; flickers during boot (GPIO14 does), harmless
static constexpr uint8_t BLUE_LED_PIN = 27;   // blinks while a link is down
static constexpr uint8_t YELLOW_LED_PIN = 13; // tramline relay on; two quick blinks: a burst failed
static constexpr uint8_t BUZZER_PIN = 19;
static constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;

// --- Button -----------------------------------------------------------------

// How the button is read (src/tractor/main.cpp, buttonSample()):
//  1. A timer samples it every BUTTON_SAMPLE_MS, whatever loop() is doing.
//  2. A press starts once BUTTON_PRESS_CONFIRM_MS of samples in a row read it
//     closed, and counts from the first of them.
//  3. A press ends only once BUTTON_RELEASE_CONFIRM_MS of samples in a row read
//     it open. Any shorter opening - bounce, a spark under a finger that is
//     still holding it - is part of the press.
//  4. A press that reaches BUTTON_LONG_PRESS_MS (sparks included) is a long
//     press, sent right then with the finger still down; letting go afterwards
//     sends nothing. One that ends earlier is a short press, sent when its end
//     is confirmed.
//  5. A press that starts less than BUTTON_MIN_GAP_MS after the finger last let
//     go - of any press, counted or not - is ignored whole: the switch bouncing
//     as it is let go, or a finger resting on it in a shaking cab. This also
//     keeps any two clicks at least BUTTON_MIN_GAP_MS apart.
// Checked by simulation against bouncy switches, noise and a shaking cab
// (CLAUDE.md, Design record -> Tractor UI).
static constexpr uint32_t BUTTON_SAMPLE_MS = 5;
// Two samples in a row, so a single spike on the line can never be a press.
// 5 would start a press on any closed sample. Keep it short: a quick tap on
// this switch makes only 15-60 ms of contact - at 50, every tap under 50 ms
// was thrown away as a "blip" (seen in the BUTTON_DEBUG log, 27 September 2026).
static constexpr uint32_t BUTTON_PRESS_CONFIRM_MS = 50;
// Must be longer than any opening the switch makes on its own: this one clicked
// twice when openings of 20 ms already ended a press. At 50 ms a press lasts
// through anything up to ~45 ms.
static constexpr uint32_t BUTTON_RELEASE_CONFIRM_MS = 50;
static constexpr uint32_t BUTTON_LONG_PRESS_MS = 750;
// Taps with the finger lifted for less than this count only the first one.
// 0 turns the rule off.
static constexpr uint32_t BUTTON_MIN_GAP_MS = 80;
static_assert(BUTTON_PRESS_CONFIRM_MS % BUTTON_SAMPLE_MS == 0 && BUTTON_PRESS_CONFIRM_MS >= BUTTON_SAMPLE_MS,
              "BUTTON_PRESS_CONFIRM_MS must be a whole number of samples, at least one");
static_assert(BUTTON_RELEASE_CONFIRM_MS % BUTTON_SAMPLE_MS == 0 && BUTTON_RELEASE_CONFIRM_MS >= 2 * BUTTON_SAMPLE_MS,
              "BUTTON_RELEASE_CONFIRM_MS must be a whole number of samples, at least two");
// A press counts only if its screen was already showing this long before the
// press began - faster than a person can react to a new screen.
static constexpr uint32_t BUTTON_SCREEN_SETTLE_MS = 100;

// --- Tramlines --------------------------------------------------------------

static constexpr uint8_t TRAMLINE_RHYTHM = 6;                        // passes per cycle
static constexpr uint8_t TRAMLINE_ACTIVE_MASK = (1 << 2) | (1 << 3); // relay on for passes 3 and 4 (bit 0 = pass 1)

// --- Alarms -----------------------------------------------------------------

// The fan has to turn while the machine is seeding - no air, no seed at the
// coulters - so this alarm is on at every power-up. Screen 21 (Dmuchawa) turns
// it off until the next one, for testing a stationary machine with no fan
// running; it is never stored. Below this the turbine counts as
// stopped; in work it runs at around 3000 RPM, so the threshold only catches a
// fan that has actually stopped, not one that is merely slow.
static constexpr uint16_t TURBINE_RUNNING_MIN_RPM = 50;

// ...and the condition has to hold this long before the screen takes over. The
// fan takes a moment to come up when you move off, and the seeder only
// recomputes its RPM every TURBINE_UPDATE_INTERVAL_MS, so without this every
// start from a standstill would beep.
static constexpr uint32_t TURBINE_ALARM_DELAY_MS = 3000;

static constexpr uint32_t LINK_BUZZER_DELAY_MS = 5000; // a lost link beeps only after this long

// The clog alarm (ZATKANIE!, screen 12) is silent - the user's call, 29
// September 2026: in the field it distracted more than it helped. Burst
// metering never raises it in work any more (see BURST_FAIL_MS); the screen can
// still come up, without the buzzer, for a clog in the calibration run or in
// continuous metering.
static constexpr bool CLOG_ALARM_BUZZER = false;

// A failed burst on the dispenser blinks the yellow LED BURST_FAIL_BLINKS
// times, BURST_FAIL_BLINK_MS on and as long off: two blinks in 0.4 s. While the
// tramline relay holds the LED lit, the same blinks go dark instead.
static constexpr uint8_t BURST_FAIL_BLINKS = 2;
static constexpr uint32_t BURST_FAIL_BLINK_MS = 100;

// --- Calibration run --------------------------------------------------------

// After START, the dispenser not calibrating for this long means the run was
// interrupted.
static constexpr uint32_t CALIBRATION_START_TIMEOUT_MS = 2000;

// --- Wheel calibration (Nasiona -> Kalibracja) ------------------------------

// Driven in the field with the machine working, so wheel slip is part of the
// number. 100 m is about 128 pulses at the default distance; 20 m would be 25,
// where one pulse either way is a 4 % error.
static constexpr uint16_t WHEEL_CALIB_DISTANCE_M = 100;
static constexpr uint16_t WHEEL_CALIB_MIN_PULSES = 50; // below this the result is refused

// --- First-boot values, and the restore path --------------------------------
//
// What a tractor board with an empty NVS starts from: a new board, or one after
// `pio run -t erase`. A normal upload keeps NVS, so a board that has been set up
// never reads these.
//
// This is also how a calibration is restored. The tractor's Ustawienia screen
// prints exactly this block over USB; paste it over these lines and flash it,
// and a blank board comes up with the machine's own numbers. Keep the printouts
// in docs/calibration-settings.txt - nothing can write settings back into the
// tractor over the radio or the serial line, on purpose.
static constexpr uint16_t DEFAULT_DOSE_KG_PER_HA = 40;
static constexpr uint32_t DEFAULT_GRAMS_PER_100REV = 500;
static constexpr bool DEFAULT_DISPENSER_ENABLED = false;
static constexpr bool DEFAULT_TRAMLINES_ENABLED = false;
static constexpr bool DEFAULT_SEED_LARGE = false;
static constexpr uint16_t DEFAULT_WHEEL_MM_SMALL = WHEEL_MM_PER_PULSE_DEFAULT;
static constexpr uint16_t DEFAULT_WHEEL_MM_LARGE = WHEEL_MM_PER_PULSE_DEFAULT;
static constexpr uint16_t DEFAULT_ANGLE_FACTOR = 500; // burst metering: the motor half busy at 10 km/h

// ===========================================================================
// DISPENSER  (wiring and parts: docs/dispenser_module_hardware.md)
// ===========================================================================

// --- Pins -------------------------------------------------------------------

static constexpr uint8_t MOTOR_PWM_PIN = 25; // needs a 10k pull-down to GND
static constexpr uint8_t MOTOR_DIR_PIN = 26; // needs a 10k pull-down to GND
static constexpr uint8_t ENCODER_A_PIN = 32; // encoder fed 5 V, through the BSS138 level shifter: 12 V here destroys the board
static constexpr uint8_t ENCODER_B_PIN = 33; // wired, not used

static constexpr uint8_t MOTOR_DIR_FORWARD = LOW; // change to HIGH if the auger turns the wrong way

// --- Motor and encoder ------------------------------------------------------

// MEASURE: turn the output shaft 10 times by hand; the edge count should grow
// by 10 times this (channel A rising edges: 64 CPR / 4 x 30:1 gearbox).
static constexpr uint32_t ENCODER_EDGES_PER_REV = 480;

// MEASURE: the lowest duty, in per mille, that still turns a loaded auger.
static constexpr uint16_t MOTOR_MIN_RUNNING_PERMILLE = 80;

static constexpr uint16_t MOTOR_MAX_RPM = 330;            // Pololu 4752 at 12 V
static constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 16000; // MD13S accepts up to 20 kHz
static constexpr uint32_t ENCODER_MIN_PULSE_GAP_US = 100; // shorter gaps are electrical noise

// --- Speed control: feed-forward plus a gentle PI trim ----------------------

static constexpr uint32_t MOTOR_CONTROL_INTERVAL_MS = 100;
static constexpr float MOTOR_KP = 0.8f;
static constexpr float MOTOR_KI = 0.4f;
static constexpr float MOTOR_INTEGRAL_LIMIT = 400.0f;

// --- Distance ledger --------------------------------------------------------

// The dispenser meters to distance, not to speed: every wheel pulse is a fixed
// distance and therefore a fixed number of shaft turns, and the encoder says
// how many it has actually made. The difference - the debt - is paid off over
// LEDGER_CATCHUP_SECONDS. The speed-based rate is still the feed-forward; this
// only removes the error it leaves behind (a speed average that lags a whole
// metering turn, and the first seconds after every start).
static constexpr float LEDGER_CATCHUP_SECONDS = 5.0f;
static constexpr uint16_t LEDGER_MAX_CATCHUP_RPM = 60; // never more than this above or below the rate
static constexpr uint16_t LEDGER_MAX_METRES = 10;      // debt cap, in metres of travel

// --- Clog alarm -------------------------------------------------------------

// Clogged when the shaft turns slower than CLOG_MIN_SPEED_PERCENT of the
// commanded speed for CLOG_DETECT_MS while the motor is meant to be turning.
static constexpr uint32_t CLOG_DETECT_MS = 1500;
static constexpr uint32_t CLOG_MIN_SPEED_PERCENT = 33;

// The tractor sounds the alarm again only after it has seen the dispenser out
// of Clogged, or lost contact with it. A new clog takes at least CLOG_DETECT_MS
// of normal running, so it can never slip in between two packets unseen.
static_assert(CLOG_DETECT_MS > LINK_TIMEOUT_MS + SEND_INTERVAL_MS,
              "CLOG_DETECT_MS must be longer than LINK_TIMEOUT_MS + SEND_INTERVAL_MS");

// --- Unclogging (Odetkaj on the tractor) ------------------------------------

static constexpr uint16_t UNCLOG_PERMILLE = 1000; // full torque; lower it if the gearbox or the supply suffers
static constexpr uint32_t UNCLOG_REVERSE_MS = 800;
static constexpr uint32_t UNCLOG_FORWARD_MS = 800;
static constexpr uint32_t UNCLOG_PAUSE_MS = 200; // motor off after each move, so it never reverses at speed
static constexpr uint32_t UNCLOG_CYCLES = 2;     // one cycle: reverse, pause, forward, pause
static constexpr uint32_t UNCLOG_CYCLE_MS = UNCLOG_REVERSE_MS + UNCLOG_PAUSE_MS + UNCLOG_FORWARD_MS + UNCLOG_PAUSE_MS;
static constexpr uint32_t UNCLOG_TOTAL_MS = UNCLOG_CYCLES * UNCLOG_CYCLE_MS; // 4000

static_assert(UNCLOG_PAUSE_MS >= 2 * MOTOR_CONTROL_INTERVAL_MS,
              "UNCLOG_PAUSE_MS must span at least two control steps");

// --- Calibration run --------------------------------------------------------

static constexpr uint16_t CALIBRATION_RPM = 120; // moderate, so the auger fills as it does in work

// --- Burst metering (a motor that cannot turn the auger slowly) -------------

// The fitted motor cannot turn the loaded auger at a low duty: it stalls and
// the clog alarm fires. So instead of a slow continuous rate, every wheel pulse
// is dosed as one burst at BURST_PWM_FRACTION of full PWM until the encoder has
// counted that pulse's angle, then stop (burstMeter() in
// src/dispenser/dispenser_logic.h). The angle is set by the angle factor below,
// calibrated on the tractor. The tractor and the dispenser both read this
// switch - the tractor for which calibration screens to show - so flash both
// after changing it.
// false = the continuous rate control above, for a motor that can hold a speed.
static constexpr bool DISPENSER_BURST_MODE = true;

// How hard a burst drives the motor, as a share of full PWM: 1.0 is flat out,
// 0.75, 0.5 and so on less. Only the power: a burst still comes with every
// pulse and still ends on that pulse's count from the encoder, so a gentler one
// simply turns slower and lasts longer, and the dose is the same. There is no
// speed control - how fast the shaft turns changes nothing the encoder counts.
// Three things do follow from it:
//  - grams per revolution can depend on how fast the auger turns, which is why
//    the calibration run uses this same PWM; calibrate with the engine running,
//    as in work, since the battery voltage moves the speed too;
//  - the torque falls with it, so a shaft that slows under the clog line in the
//    middle of a burst gets full PWM until it moves again (burstDuty() in
//    src/dispenser/dispenser_logic.h), and only a shaft that full PWM cannot
//    move either is a clog;
//  - the top working speed falls in step, because a burst has to finish before
//    the next pulse. The angle factor below is scaled on this same PWM, so
//    whatever the fraction, F / 1000 is roughly how busy the motor is at the
//    reference speed, 10 km/h: the top speed is about 10 km/h x 1000 / F, times
//    the loaded motor's real full-PWM RPM over 300. Faster than that, ZA SZYBKO
//    sounds. Change the fraction and the same F turns a proportionally smaller
//    angle, so recalibrate.
static constexpr float BURST_PWM_FRACTION = 1.0f;
static constexpr uint16_t BURST_PERMILLE = (uint16_t)(BURST_PWM_FRACTION * 1000.0f + 0.5f);

// The angle factor F (0-999, set on the tractor's Kalibracja screens, sent in
// every command) is the angle each pulse's burst turns, in thousandths of a
// reference: what the motor turns at full burst PWM - taken as
// BURST_ANGLE_REFERENCE_RPM x BURST_PWM_FRACTION - in the time between two
// pulses at BURST_ANGLE_REFERENCE_SPEED_MM_S. So 1000 would keep the motor busy
// all the time at 10 km/h, and F = 500 at 1571 mm per pulse is 1.41 turns a
// pulse. An estimate, not a measurement: the factor is calibrated by weighing,
// so all the reference does is make F scale the bursts. The distance per pulse
// is in both the reference and the ground, so F holds for either seed size.
//
// 999 is the most the tractor takes - about all the motor turns between two
// pulses at 10 km/h - and that is the point: the machine is never driven
// faster (8 km/h in practice) and the motor can do no more than full PWM, so
// 999 is the machine's physical limit (the user's reference, 28 September
// 2026). At full PWM a dose needs F = 6 x width [cm] x dose [kg/ha] x 2778 /
// (300 x grams per 100 turns): at 4 m, past 999 when the auger gives less than
// about 890 g per 100 turns at 40 kg/ha (445 g at 20). The calibration run then
// still weighs short at 999, and the remedy is mechanical - more grams per turn,
// a wider dispenser opening - not a different reference, which would only hide
// the limit. The 300 RPM is assumed: a loaded motor that turns slower lowers
// the real limit in proportion, and ZA SZYBKO sounds when it is reached.
static constexpr uint16_t BURST_ANGLE_REFERENCE_RPM = 300;
static constexpr uint16_t BURST_ANGLE_REFERENCE_SPEED_MM_S = 2778; // 10 km/h

// The calibration run in burst mode fires real bursts: CALIBRATION_PULSES wheel
// pulses as if driving at CALIBRATION_SPEED_MM_S - one every distance-per-pulse
// at that speed, with the tractor's distance for the seed size in use (0.94 s
// at 1571 mm) - each dosed exactly as in work: one burst, to the angle the
// angle factor sets, at the burst PWM, carrying straight on if the next pulse
// is due before one is done. Every pulse's angle is turned in full even then:
// nothing is dropped the way it is in work when the motor falls hopelessly
// behind, or the weight would come out short. The tractor shows what that
// ground should get, CALIBRATION_PULSES x distance per pulse [m] x dose
// [kg/ha] x 0.4 g at 4 m - 503 g at 1571 mm and 40 kg/ha - and after the run
// takes the weighed mass and sets F = F x expected / weighed.
static constexpr uint16_t CALIBRATION_SPEED_MM_S = 1667; // 6 km/h

// What is owed below this waits for the next pulse, so the sliver of rounding
// left over from a burst never twitches the motor.
static constexpr uint32_t BURST_MIN_EDGES = ENCODER_EDGES_PER_REV / 10; // a tenth of a turn

// How far the motor may fall behind the ground, in pulses. Reached only when it
// cannot keep up at all (ZA SZYBKO): whatever would go past it is dropped, so
// slowing down afterwards never dumps more than this in one place. Not below 3:
// a radio gap just short of LINK_TIMEOUT_MS can deliver two or three pulses in
// one packet at speed, and every one of them is owed.
static constexpr uint8_t BURST_MAX_BACKLOG_PULSES = 3;

// ZA SZYBKO. A pulse is late when it arrives with the motor still more than this
// many pulses' turns behind on the earlier ones; BURST_LATE_PULSES late pulses in
// a row mean it cannot keep up with the machine, and the alarm clears when a
// pulse finds it caught up again. A single late pulse is usually the radio: a
// packet lost or delayed starts one burst late. Simulated on a bad radio
// (packets 100-300 ms apart, one in ten lost): no alarm at 80 % of the motor's
// speed and one short one in 10 minutes at 90 %; driving 15 % too fast raises it
// after about 4 s, 40 % too fast after 2 s. At half a pulse, the first value
// tried, it sounded twice in 10 minutes already at 80 %.
static constexpr float BURST_LATE_FRACTION = 1.0f;
static constexpr uint8_t BURST_LATE_PULSES = 2;

// A stalled burst: the shaft stays under this - or under a third
// (CLOG_MIN_SPEED_PERCENT) of what the burst PWM turns a free motor at, if that
// is lower - while a burst drives it: 60 RPM at full PWM, 54 at half. A blocked
// auger reads close to 0; a heavy one that still turns, well above it.
static constexpr uint16_t BURST_CLOG_MIN_RPM = 60;

// Stalled for BURST_FAIL_MS in work, a burst has failed - a granule wedged in
// the auger, most likely. It never raises the clog alarm (the user's rules, 29
// September 2026: in the field the auger always went on after Anuluj, about four
// times in 700 m): the burst stops, what it still owed - one pulse's worth at
// most - waits for the next wheel pulse, and that pulse's burst tries again.
// BURST_FAILS_BEFORE_UNCLOG failures in a row, with no burst reaching its count
// in between, run the unclog sequence by themselves (AutoUnclogging), and
// metering goes on after it - for as long as it takes, never Clogged. The only
// sign on the tractor is the yellow LED blinking twice per failure. The
// calibration run still stops as Clogged after CLOG_DETECT_MS: the unclog's
// reverse turns would count in its weighing.
static constexpr uint32_t BURST_FAIL_MS = 1500;
static constexpr uint8_t BURST_FAILS_BEFORE_UNCLOG = 3;

// After every burst - in work, in the calibration run and in the simulation,
// and after a failed one too - a short push backwards: BURST_BACKLASH_PERMILLE
// of full PWM for BURST_BACKLASH_MS, starting BURST_BACKLASH_DELAY_MS after the
// motor stopped, when the brake has the shaft still (sooner, the push would
// only brake harder). It is not meant to turn the auger, only to open the
// gearbox's backlash, so the next burst's motor gets a little free travel to
// gain speed in before the teeth take the load: a burst that starts with the
// teeth already pressed together starts from a dead point (the user's idea,
// 29 September 2026). Not measured - and the encoder, which cannot tell the
// direction, counts it as a few more edges turned; the calibration run pushes
// back after its bursts too, so the weighing takes that in. The next burst waits
// until the push is over. BURST_BACKLASH_MS 0 turns it off.
static constexpr uint16_t BURST_BACKLASH_PERMILLE = 100; // 10 %
static constexpr uint32_t BURST_BACKLASH_MS = 50;
static constexpr uint32_t BURST_BACKLASH_DELAY_MS = 30;

// The seeding simulation (tractor menu -> Symulacja): burst metering exactly as
// in work - failed bursts, retries and the unclog sequence included - standing
// still, on the pulses SIMULATION_SPEED_MM_S would give with the tractor's
// distance per pulse, for SIMULATION_MINUTES. The endurance test the user asked
// for (29 September 2026): the clogs came after a few passes, not at once. The
// tractor shows the failed bursts as a share of all of them. Both boards read
// these; reflash both after a change.
static constexpr uint16_t SIMULATION_SPEED_MM_S = 1944; // 7 km/h
static constexpr uint8_t SIMULATION_MINUTES = 15;
static constexpr uint32_t SIMULATION_DURATION_MS = (uint32_t)SIMULATION_MINUTES * 60UL * 1000UL;

// The shaft RPM the tractor shows is averaged over about this long: the shaft
// itself alternates between the burst's speed and standing still.
static constexpr uint32_t BURST_RPM_AVERAGE_MS = 2000;

static_assert(BURST_PWM_FRACTION > 0.0f && BURST_PWM_FRACTION <= 1.0f,
              "BURST_PWM_FRACTION is a share of full PWM: above 0, at most 1.0");
static_assert(BURST_PERMILLE >= MOTOR_MIN_RUNNING_PERMILLE && BURST_PERMILLE <= 1000,
              "BURST_PWM_FRACTION is too low to turn the motor at all");
static_assert(BURST_MIN_EDGES > 0, "BURST_MIN_EDGES must be at least one edge");
// Once the backlog sits at its cap, a new pulse must still find more than
// BURST_LATE_FRACTION to do, or ZA SZYBKO could never be raised.
static_assert((float)BURST_MAX_BACKLOG_PULSES >= 1.0f + BURST_LATE_FRACTION,
              "BURST_MAX_BACKLOG_PULSES must leave room for a late pulse");
static_assert(BURST_CLOG_MIN_RPM > 0 && BURST_CLOG_MIN_RPM < MOTOR_MAX_RPM,
              "BURST_CLOG_MIN_RPM must be between 0 and the motor's speed");
// Every burst's first step measures the standstill before it and starts the
// timer; it takes a later step at speed to stop it again.
static_assert(BURST_FAIL_MS >= 3 * MOTOR_CONTROL_INTERVAL_MS,
              "BURST_FAIL_MS must span several control steps");
static_assert(BURST_FAILS_BEFORE_UNCLOG >= 1, "BURST_FAILS_BEFORE_UNCLOG must be at least one failure");
static_assert(BURST_BACKLASH_PERMILLE <= 1000, "BURST_BACKLASH_PERMILLE is a share of full PWM, in per mille");
// The push has to be over well within a control step's worth of the next
// burst's wait, and a step that lands in it must still see it running.
static_assert(BURST_BACKLASH_DELAY_MS + BURST_BACKLASH_MS < 2 * MOTOR_CONTROL_INTERVAL_MS,
              "the push backwards after a burst must be short");
static_assert(SIMULATION_SPEED_MM_S > 0 && SIMULATION_MINUTES >= 1 && SIMULATION_MINUTES <= 99,
              "the simulation needs a speed, and a length the tractor can show as mm:ss");
static_assert(CALIBRATION_PULSES >= 1, "CALIBRATION_PULSES must be at least one pulse");
static_assert(BURST_ANGLE_REFERENCE_RPM > 0 && BURST_ANGLE_REFERENCE_SPEED_MM_S > 0,
              "the angle factor's reference must be a speed of the motor and of the machine");
static_assert(DEFAULT_ANGLE_FACTOR <= 999, "the angle factor has three digits on the tractor");
// The pulses must come at least a millisecond apart even at the shortest distance.
static_assert(CALIBRATION_SPEED_MM_S > 0 && (uint32_t)WHEEL_MM_PER_PULSE_MIN * 1000 >= CALIBRATION_SPEED_MM_S,
              "CALIBRATION_SPEED_MM_S must be a speed, in mm/s");
