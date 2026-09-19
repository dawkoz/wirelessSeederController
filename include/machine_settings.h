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

static constexpr uint32_t WORKING_WIDTH_CM        = 400;   // 4.00 m
static constexpr uint16_t CALIBRATION_REVOLUTIONS = 100;   // dispenser shaft turns per calibration run

// --- Serial -----------------------------------------------------------------

// Serial monitor speed on every board, and what the tractor's Ustawienia screen
// tells the operator to set on the laptop. Keep it equal to `monitor_speed` in
// platformio.ini.
static constexpr uint32_t SERIAL_BAUD = 115200;

// --- Radio link -------------------------------------------------------------

static constexpr uint8_t  NETWORK_ID       = 1;      // change only if a machine nearby runs this firmware
static constexpr uint8_t  ESPNOW_CHANNEL   = 1;
static constexpr uint32_t SEND_INTERVAL_MS = 200;    // every board sends this often
static constexpr uint32_t LINK_TIMEOUT_MS  = 1000;   // a board silent this long counts as disconnected


// ===========================================================================
// SEEDER
// ===========================================================================

// --- Pins -------------------------------------------------------------------

static constexpr uint8_t RELAY_PIN          = 12;   // strapping pin: never add a pull-up to this line
static constexpr uint8_t TURBINE_SENSOR_PIN = 14;   // inductive sensor
static constexpr uint8_t WHEEL_SENSOR_PIN   = 27;   // Hall sensor on the ground wheel, before the seed-rate gearbox

static constexpr uint8_t RELAY_ON  = LOW;           // the relay board is active LOW
static constexpr uint8_t RELAY_OFF = HIGH;

// --- Ground wheel -----------------------------------------------------------

// Distance travelled between two wheel-sensor pulses. The sensor is on the
// metering drive, not on the ground wheel, so this depends on which seed-size
// gear the machine is in: the tractor keeps one measured value per setting and
// sends the active one in every packet (TractorCommand.wheelMmPerPulse). The
// default below is only used until the first command arrives, and as the
// fallback for a value outside the limits.
// MEASURE: Nasiona -> Kalibracja on the tractor does it, once per seed size.
static constexpr uint16_t WHEEL_MM_PER_PULSE_DEFAULT = 785;   // 6 magnets, ratio 0.4, 600 mm wheel
static constexpr uint16_t WHEEL_MM_PER_PULSE_MIN     = 100;
static constexpr uint16_t WHEEL_MM_PER_PULSE_MAX     = 5000;

static constexpr uint8_t  WHEEL_MAGNETS           = 6;       // three more fitted to halve the gap between pulses
static constexpr uint8_t  WHEEL_AVERAGE_INTERVALS = 6;       // speed is averaged over this many gaps between pulses

// The slowest speed this sensor still calls "moving". A pulse that has not
// arrived within (distance per pulse / this) means the machine has stopped: at
// any higher speed it would already be here. One number with a physical
// meaning, in place of the old fixed 2 s timeout - which, on the metering drive
// with 6 magnets, called a machine moving at 1.4 km/h stopped. The cost of
// lowering it is that a real stop takes (distance per pulse / this) to notice:
// 2.6 s at the default distance.
static constexpr uint16_t WHEEL_MIN_SPEED_MM_S = 300;        // ~1.1 km/h

static constexpr uint32_t WHEEL_MAX_SPEED_MM_S = 11000;      // ~40 km/h, above any road speed
// A gap shorter than this is electrical noise, not a pulse. Derived from the
// default distance per pulse rather than the calibrated one: it is read in the
// ISR, so it has to be a compile-time constant.
static constexpr uint32_t WHEEL_MIN_PULSE_GAP_US =
    (uint32_t)((uint64_t)WHEEL_MM_PER_PULSE_DEFAULT * 1000000ULL / WHEEL_MAX_SPEED_MM_S);

// Whole wheel turns only, so uneven spacing between the magnets cancels out.
static_assert(WHEEL_AVERAGE_INTERVALS > 0 && WHEEL_AVERAGE_INTERVALS % WHEEL_MAGNETS == 0,
              "WHEEL_AVERAGE_INTERVALS must be a multiple of WHEEL_MAGNETS");

// --- Turbine ----------------------------------------------------------------

static constexpr uint32_t TURBINE_PULSES_PER_REV     = 1;      // holes in the turbine disc
static constexpr uint32_t TURBINE_UPDATE_INTERVAL_MS = 2000;
static constexpr uint32_t TURBINE_MIN_PULSE_GAP_US   = 1000;   // shorter gaps are electrical noise


// ===========================================================================
// TRACTOR
// ===========================================================================

// --- Pins -------------------------------------------------------------------

static constexpr uint8_t BUTTON_PIN       = 12;     // to GND. Strapping pin: never add a pull-up to this line
static constexpr uint8_t GREEN_LED_PIN    = 14;     // all links healthy
static constexpr uint8_t BLUE_LED_PIN     = 27;     // blinks while a link is down
static constexpr uint8_t YELLOW_LED_PIN   = 13;     // tramline relay on
static constexpr uint8_t BUZZER_PIN       = 19;
static constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;

// --- Button -----------------------------------------------------------------

static constexpr uint32_t BUTTON_DEBOUNCE_MS   = 50;
static constexpr uint32_t BUTTON_LONG_PRESS_MS = 1500;
// A press counts only if its screen was already showing this long before the
// press began - faster than a person can react to a new screen.
static constexpr uint32_t BUTTON_SCREEN_SETTLE_MS = 250;

// --- Tramlines --------------------------------------------------------------

static constexpr uint8_t TRAMLINE_RHYTHM      = 6;                     // passes per cycle
static constexpr uint8_t TRAMLINE_ACTIVE_MASK = (1 << 2) | (1 << 3);   // relay on for passes 3 and 4 (bit 0 = pass 1)

// --- Alarms -----------------------------------------------------------------

// The fan has to turn while the machine is seeding - no air, no seed at the
// coulters - so this alarm is always on. Below this the turbine counts as
// stopped; in work it runs at around 3000 RPM, so the threshold only catches a
// fan that has actually stopped, not one that is merely slow.
static constexpr uint16_t TURBINE_RUNNING_MIN_RPM = 50;

// ...and the condition has to hold this long before the screen takes over. The
// fan takes a moment to come up when you move off, and the seeder only
// recomputes its RPM every TURBINE_UPDATE_INTERVAL_MS, so without this every
// start from a standstill would beep.
static constexpr uint32_t TURBINE_ALARM_DELAY_MS = 3000;

static constexpr uint32_t LINK_BUZZER_DELAY_MS    = 5000;    // a lost link beeps only after this long

// --- Calibration run --------------------------------------------------------

// After START, the dispenser not calibrating for this long means the run was
// interrupted.
static constexpr uint32_t CALIBRATION_START_TIMEOUT_MS = 2000;

// --- Wheel calibration (Nasiona -> Kalibracja) ------------------------------

// Driven in the field with the machine working, so wheel slip is part of the
// number. 100 m is about 128 pulses at the default distance; 20 m would be 25,
// where one pulse either way is a 4 % error.
static constexpr uint16_t WHEEL_CALIB_DISTANCE_M = 100;
static constexpr uint16_t WHEEL_CALIB_MIN_PULSES = 50;    // below this the result is refused

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
static constexpr uint16_t DEFAULT_DOSE_KG_PER_HA    = 40;
static constexpr uint32_t DEFAULT_GRAMS_PER_100REV  = 500;
static constexpr bool     DEFAULT_DISPENSER_ENABLED = false;
static constexpr bool     DEFAULT_TRAMLINES_ENABLED = false;
static constexpr bool     DEFAULT_SEED_LARGE        = false;
static constexpr uint16_t DEFAULT_WHEEL_MM_SMALL    = WHEEL_MM_PER_PULSE_DEFAULT;
static constexpr uint16_t DEFAULT_WHEEL_MM_LARGE    = WHEEL_MM_PER_PULSE_DEFAULT;


// ===========================================================================
// DISPENSER  (wiring and parts: docs/dispenser_module_hardware.md)
// ===========================================================================

// --- Pins -------------------------------------------------------------------

static constexpr uint8_t MOTOR_PWM_PIN = 25;   // needs a 10k pull-down to GND
static constexpr uint8_t MOTOR_DIR_PIN = 26;   // needs a 10k pull-down to GND
static constexpr uint8_t ENCODER_A_PIN = 32;   // encoder fed 5 V, through the BSS138 level shifter: 12 V here destroys the board
static constexpr uint8_t ENCODER_B_PIN = 33;   // wired, not used

static constexpr uint8_t MOTOR_DIR_FORWARD = LOW;   // change to HIGH if the auger turns the wrong way

// --- Motor and encoder ------------------------------------------------------

// MEASURE: turn the output shaft 10 times by hand; the edge count should grow
// by 10 times this (channel A rising edges: 64 CPR / 4 x 30:1 gearbox).
static constexpr uint32_t ENCODER_EDGES_PER_REV = 480;

// MEASURE: the lowest duty, in per mille, that still turns a loaded auger.
static constexpr uint16_t MOTOR_MIN_RUNNING_PERMILLE = 80;

static constexpr uint16_t MOTOR_MAX_RPM            = 330;     // Pololu 4752 at 12 V
static constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ   = 16000;   // MD13S accepts up to 20 kHz
static constexpr uint32_t ENCODER_MIN_PULSE_GAP_US = 100;     // shorter gaps are electrical noise

// --- Speed control: feed-forward plus a gentle PI trim ----------------------

static constexpr uint32_t MOTOR_CONTROL_INTERVAL_MS = 100;
static constexpr float    MOTOR_KP                  = 0.8f;
static constexpr float    MOTOR_KI                  = 0.4f;
static constexpr float    MOTOR_INTEGRAL_LIMIT      = 400.0f;

// --- Distance ledger --------------------------------------------------------

// The dispenser meters to distance, not to speed: every wheel pulse is a fixed
// distance and therefore a fixed number of shaft turns, and the encoder says
// how many it has actually made. The difference - the debt - is paid off over
// LEDGER_CATCHUP_SECONDS. The speed-based rate is still the feed-forward; this
// only removes the error it leaves behind (a speed average that lags a whole
// metering turn, and the first seconds after every start).
static constexpr float    LEDGER_CATCHUP_SECONDS = 5.0f;
static constexpr uint16_t LEDGER_MAX_CATCHUP_RPM = 60;    // never more than this above or below the rate
static constexpr uint16_t LEDGER_MAX_METRES      = 10;    // debt cap, in metres of travel

// --- Clog alarm -------------------------------------------------------------

// Clogged when the shaft turns slower than CLOG_MIN_SPEED_PERCENT of the
// commanded speed for CLOG_DETECT_MS while the motor is meant to be turning.
static constexpr uint32_t CLOG_DETECT_MS         = 1500;
static constexpr uint32_t CLOG_MIN_SPEED_PERCENT = 33;

// The tractor sounds the alarm again only after it has seen the dispenser out
// of Clogged, or lost contact with it. A new clog takes at least CLOG_DETECT_MS
// of normal running, so it can never slip in between two packets unseen.
static_assert(CLOG_DETECT_MS > LINK_TIMEOUT_MS + SEND_INTERVAL_MS,
              "CLOG_DETECT_MS must be longer than LINK_TIMEOUT_MS + SEND_INTERVAL_MS");

// --- Unclogging (Odetkaj on the tractor) ------------------------------------

static constexpr uint16_t UNCLOG_PERMILLE   = 1000;   // full torque; lower it if the gearbox or the supply suffers
static constexpr uint32_t UNCLOG_REVERSE_MS = 800;
static constexpr uint32_t UNCLOG_FORWARD_MS = 800;
static constexpr uint32_t UNCLOG_PAUSE_MS   = 200;    // motor off after each move, so it never reverses at speed
static constexpr uint32_t UNCLOG_CYCLES     = 2;      // one cycle: reverse, pause, forward, pause
static constexpr uint32_t UNCLOG_CYCLE_MS   = UNCLOG_REVERSE_MS + UNCLOG_PAUSE_MS + UNCLOG_FORWARD_MS + UNCLOG_PAUSE_MS;
static constexpr uint32_t UNCLOG_TOTAL_MS   = UNCLOG_CYCLES * UNCLOG_CYCLE_MS;   // 4000

static_assert(UNCLOG_PAUSE_MS >= 2 * MOTOR_CONTROL_INTERVAL_MS,
              "UNCLOG_PAUSE_MS must span at least two control steps");

// --- Calibration run --------------------------------------------------------

static constexpr uint16_t CALIBRATION_RPM = 120;   // moderate, so the auger fills as it does in work
