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

// MEASURE: push the seeder along a measured 100 m and count the wheel pulses,
// then WHEEL_MM_PER_PULSE = 100000 / pulses.
// Until then, an estimate: 600 mm wheel x 3.14 / 3 magnets.
static constexpr uint32_t WHEEL_MM_PER_PULSE = 628;

static constexpr uint8_t  WHEEL_MAGNETS           = 3;
static constexpr uint8_t  WHEEL_AVERAGE_INTERVALS = 6;       // speed is averaged over this many gaps between pulses
static constexpr uint32_t WHEEL_STOP_TIMEOUT_MS   = 2000;    // no pulse for this long = stopped
static constexpr uint32_t WHEEL_MIN_PULSE_GAP_US  = 40000;   // shorter gaps are electrical noise (40 ms is over 50 km/h)

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

static constexpr bool     ENABLE_TURBINE_ALARM    = false;
static constexpr bool     ENABLE_WOM_ALARM        = false;   // no WOM sensor yet: the seeder sends a fixed 540
static constexpr uint16_t TURBINE_RUNNING_MIN_RPM = 50;      // below this the turbine counts as stopped
static constexpr uint16_t WOM_RUNNING_MIN_RPM     = 50;      // below this the WOM counts as stopped
static constexpr uint32_t LINK_BUZZER_DELAY_MS    = 5000;    // a lost link beeps only after this long

// --- Calibration run --------------------------------------------------------

// After START, the dispenser not calibrating for this long means the run was
// interrupted.
static constexpr uint32_t CALIBRATION_START_TIMEOUT_MS = 2000;

// --- First-boot values, until changed on the Dawka and Kalibracja screens ---

static constexpr uint16_t DEFAULT_DOSE_KG_PER_HA   = 40;
static constexpr uint32_t DEFAULT_GRAMS_PER_100REV = 500;


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
