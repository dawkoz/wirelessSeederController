#include <Arduino.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Preferences.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "angle_factor.h"

// ===========================================================================
// TEMPORARY - button debugging on serial, at SERIAL_BAUD. 1 = on, 0 = off:
// every "#if BUTTON_DEBUG" block then compiles to nothing and the firmware is
// exactly the production one. Can also be set from platformio.ini with
// -D BUTTON_DEBUG=0. Search for BUTTON_DEBUG to delete the blocks once the
// button is settled. Must be 0 before the tractor goes to the field.
// ===========================================================================
#ifndef BUTTON_DEBUG
#define BUTTON_DEBUG 0
#endif

// Tractor module. Operator interface: OLED, one button, three LEDs, buzzer.
// Owns the tramline selection and the dispenser settings, and broadcasts them.

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

static Adafruit_SH1106 oled((int8_t)OLED_RESET);

// ---------------------------------------------------------------------------
// Display and editor limits
// ---------------------------------------------------------------------------

static constexpr uint16_t MAX_DOSE_KG_PER_HA   = 999;     // 3 digits
static constexpr uint32_t MAX_GRAMS_PER_100REV = 99999;   // 5 digits
static constexpr uint32_t MAX_CALIB_MASS_GRAMS = 99999;   // 5 digits: any run fits, at most 39960 g expected

static constexpr uint32_t DISPLAY_INTERVAL_MS = 200;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

enum class Screen : uint8_t {
    Menu,
    Work,
    EditDose,
    EditCalibration,  // grams per 100 turns - continuous metering's Kalibracja
    EditAngle,        // the angle factor - burst metering's Kalibracja, screen 5
    CalibMass,        // what the burst calibration run weighed -> the factor, screen 5b
    CalibConfirm,     // "Start kalibracji?" -> Anuluj / START
    CalibRunning,     // progress bar while the dispenser turns
    Tramlines,        // the tramline on/off switch, screen 15
    Seeds,            // seed size and its wheel calibration, screen 16
    SeedCalibAsk,     // "Kalibracja kola?" -> Anuluj / OK, screen 17
    SeedCalibRun,     // driving the measured distance, screen 18
    SeedCalibResult,  // the measured value, or why there isn't one, screen 19
    Settings,         // everything stored, and the USB export, screen 20
    Blower,           // the blower alarm switch, screen 21
    Tests,            // the tests, one per row, screen 22
    SimConfirm,       // "Symulacja siewu" -> Anuluj / START, screen 23
    SimRunning,       // the simulation, and how it ended, screens 24-27
    SimResult,        // stopped with a long press: how far it got, screen 28
};

// What is actually on the display. One Screen can show as several Views
// (CalibRunning splits into progress/done/refused/no-dispenser/interrupted)
// and the clog alarm is an overlay that covers any Screen. Both the button
// handlers and redraw() go by the View, never by Screen alone - otherwise a
// press on the clog alarm would act on the screen underneath it.
enum class View : uint8_t {
    Menu,
    Work,
    WorkFault,        // a fault replaces the work screen entirely
    EditDose,
    EditCalibration,
    EditAngle,
    CalibMass,
    Tramlines,
    CalibConfirm,
    CalibProgress,
    CalibDone,
    CalibRefused,
    CalibNoDispenser,
    CalibInterrupted,
    ClogAlert,
    ClogChoice,
    Unclogging,
    Seeds,
    SeedCalibAsk,
    SeedCalibRun,
    SeedCalibResult,
    Settings,
    Blower,
    Tests,
    SimConfirm,
    SimProgress,
    SimDone,
    SimRefused,
    SimNoDispenser,
    SimStopped,       // stopped early, by the operator or by the dispenser
};

enum class MenuItem : uint8_t {
    Praca      = 0,
    Dawka      = 1,
    Kalibracja = 2,
    Sciezki    = 3,
    Nasiona    = 4,
    Ustawienia = 5,
    Dmuchawa   = 6,
    Testy      = 7,
    Count      = 8,
};

// More items than rows, so the menu scrolls: four are drawn from menuTop, which
// follows the cursor.
static const char *const MENU_LABELS[] = {"Praca", "Dawka", "Kalibracja", "Sciezki", "Nasiona", "Ustawienia",
                                          "Dmuchawa", "Testy"};
static_assert(sizeof(MENU_LABELS) / sizeof(MENU_LABELS[0]) == (size_t)MenuItem::Count,
              "MENU_LABELS must name every MenuItem, in the enum's order");
static constexpr uint8_t MENU_VISIBLE_ROWS = 4;

// What can take over the work screen. LinkLost is the exception: it is a letter
// in the corner and the buzzer, never a full screen.
enum class FaultCode : uint8_t {
    None,
    TurbineOff,          // the fan has stopped while the machine is seeding
    DispenserOverSpeed,  // driving faster than the dispenser can meter
    LinkLost,
};

enum class ButtonEvent : uint8_t { None, Short, Long };

static Screen   screen   = Screen::Menu;
static uint8_t  menuIndex = 0;
static uint8_t  menuTop   = 0;   // first of the four menu rows on screen

// Numeric editor. digits[] holds one decimal digit each, most significant
// first. The cursor runs across the digits and then onto the trailing action
// fields, so the positions are:
//     0 .. digitCount-1   the digits
//     digitCount          the screen's extra action (WL./WYL., TEST or WROC)
//     digitCount+1        ZAPISZ
// except on the angle factor's screen, which has one action more:
//     digitCount          TEST
//     digitCount+1        MASA
//     digitCount+2        ZAPISZ
// editingDigit means short presses change the digit rather than move on.
static uint8_t digits[5]    = {0};
static uint8_t digitCount   = 0;
static uint8_t cursor       = 0;
static bool    editingDigit = false;

static uint8_t  calibConfirmIndex = 0;   // 0 = Anuluj, 1 = START
static bool     calibrationRequested = false;

static uint16_t doseKgPerHa      = DEFAULT_DOSE_KG_PER_HA;
static uint32_t gramsPer100Rev   = DEFAULT_GRAMS_PER_100REV;
static bool     dispenserEnabled = false;

// Burst metering's calibration (DISPENSER_BURST_MODE), 0-999: the angle each
// wheel pulse's burst turns, in thousandths of what the motor turns flat out
// between two pulses at 10 km/h (see angle_factor.h). Set by weighing the
// calibration run or by hand on screen 5, and scaled with the dose whenever a
// new one is saved. Stored in NVS; sent in every command in both kinds of
// metering, though only burst metering reads it.
static uint16_t angleFactor      = DEFAULT_ANGLE_FACTOR;

static uint8_t tramlineNumber = 0;

// The machine is usually used without tramlines at all, so the switch turns
// the relay off for every pass until the operator turns them back on. Flipping
// it never touches tramlineNumber: switching off and on again mid-field must
// not lose the pass. Stored in NVS, written only when it is flipped.
static bool    tramlinesEnabled = false;
static uint8_t tramlineCursor   = 0;      // screen 15: 0 = the switch, 1 = ZAPISZ

// The blower alarm can be switched off for testing a stationary machine, where
// the wheel is turned by hand and no fan runs. Never stored, on purpose: every
// power-up starts with it on, so an alarm switched off in the yard cannot follow
// the machine into the field, where a stopped fan ruins the pass unnoticed.
static bool    blowerAlarmEnabled = true;
static uint8_t blowerCursor       = 0;    // screen 21: 0 = the switch, 1 = ZAPISZ

// Distance covered between two wheel pulses, one value per seed-size gear: the
// pin-27 sensor is on the metering drive, so it turns at a different rate to
// the ground wheel and the ratio depends on the gear. Both values are measured
// by driving WHEEL_CALIB_DISTANCE_M (screens 16-19) and kept in NVS; the active
// one goes out in every command, and the seeder and dispenser meter with it.
static uint16_t wheelMmSmall = DEFAULT_WHEEL_MM_SMALL;
static uint16_t wheelMmLarge = DEFAULT_WHEEL_MM_LARGE;
static bool     seedLarge    = DEFAULT_SEED_LARGE;   // false = small seeds

// Screen 20: the settings, and the one way they leave the board - printed over
// USB. settingsSentMs only drives the "sent" message on the screen.
static uint8_t  settingsCursor = 0;       // 0 = send, 1 = Wroc
static uint32_t settingsSentMs = 0;

static uint8_t  seedCursor      = 0;      // screen 16: 0 Male, 1 Duze, 2 Kalibracja, 3 Wroc
static uint8_t  seedAskIndex    = 0;      // screen 17: 0 = Anuluj, 1 = OK
static uint8_t  seedResultIndex = 0;      // screen 19: 0 = Anuluj, 1 = ZAPISZ

// Screen 22 (Testy): tests of the machine standing still, a row each, then
// Wroc - room for more to come. The seeding simulation only in burst metering,
// which is where there are bursts to fail.
enum class TestRow : uint8_t { Simulation, Back };
static const TestRow TEST_ROWS_BURST[]      = {TestRow::Simulation, TestRow::Back};
static const TestRow TEST_ROWS_CONTINUOUS[] = {TestRow::Back};
static constexpr uint8_t TEST_ROW_COUNT     = DISPENSER_BURST_MODE ? 2 : 1;
static uint8_t testsCursor = 0;

static TestRow testRow(uint8_t index)
{
    return DISPENSER_BURST_MODE ? TEST_ROWS_BURST[index] : TEST_ROWS_CONTINUOUS[index];
}

// The seeding simulation (screens 23-28): simulationRequested goes out in every
// command, like calibrationRequested. While the dispenser runs it the tractor
// copies its counts and times it, so a run that ends early - a long press, or
// the dispenser dropping it (simInterrupted, as for the calibration run) - can
// still show how far it got.
static uint8_t  simConfirmIndex      = 0;       // screen 23: 0 = Anuluj, 1 = START
static bool     simulationRequested  = false;
static bool     simInterrupted       = false;
static bool     simStartWatchRunning = false;
static uint32_t simStartWatchMs      = 0;
static bool     simSeen              = false;   // the dispenser has reported it running since START
static uint32_t simSeenAtMs          = 0;       // when it first did: the clock on screen 24
static uint32_t simElapsedMs         = 0;
static uint16_t simBursts            = 0;
static uint16_t simFailures          = 0;
static uint8_t  simUnclogs           = 0;

// Why a calibration run produced no usable number. The result screen shows the
// reason and offers nothing but Anuluj.
enum class WheelCalibError : uint8_t { None, NoSeeder, SeederReset, TooFew, OutOfRange };

static uint32_t calibWheelStartPulses = 0;
static uint32_t calibWheelStartUpTime = 0;   // the seeder's, to catch its counter restarting
static uint32_t calibWheelPulses      = 0;   // pulses over the finished run
static uint16_t calibWheelResultMm    = 0;
static WheelCalibError calibWheelError = WheelCalibError::None;

static uint16_t activeWheelMmPerPulse()
{
    return seedLarge ? wheelMmLarge : wheelMmSmall;
}

static Preferences prefs;
static LinkTracker links;

// The receive callback writes only the rx copies, under rxLock, so loop()
// can never read a half-copied struct. loop() snapshots them under the same
// lock at the top of every iteration; everything below reads the snapshot.
static SeederTelemetry rxSeederData     = {};
static DispenserStatus rxDispenserData  = {};
static bool rxSeederEverSeen    = false;
static bool rxDispenserEverSeen = false;
static portMUX_TYPE rxLock = portMUX_INITIALIZER_UNLOCKED;

static SeederTelemetry seederData     = {};
static DispenserStatus dispenserData  = {};
static bool seederEverSeen    = false;
static bool dispenserEverSeen = false;

static FaultCode faultCode = FaultCode::None;
static uint32_t  linkDownSinceMs   = 0;
static uint32_t  turbineBadSinceMs = 0;   // when the fan was first seen stopped

// Clog overlay: the alarm covers any screen. acknowledged means the operator
// has seen the alarm and moved on to the Anuluj / Odetkaj choice.
static bool     clogAcknowledged = false;
static uint8_t  clogChoiceIndex  = 0;      // 0 = Anuluj, 1 = Odetkaj
static uint8_t  clogClearSeq     = 0;      // +1 each time the operator picks Anuluj
static uint8_t  unclogSeq        = 0;      // +1 each time the operator picks Odetkaj

// Failed bursts on the dispenser (burst metering): its status counts them, and
// every change blinks the yellow LED (BURST_FAIL_BLINKS) - the only sign of
// them, by the user's choice. burstFailSynced is false until a status has been
// taken as the starting point, at boot and after every loss of contact, so a
// link gap or a dispenser reboot never blinks.
static uint8_t  lastBurstFailures = 0;
static bool     burstFailSynced   = false;
static bool     burstBlinking     = false;
static uint32_t burstBlinkStartMs = 0;

// Calibration: interrupted means the dispenser sat in Normal after START
// instead of calibrating (its side dropped the run), so the screen must tell
// the operator instead of sitting at 0 % forever.
static bool     calibrationInterrupted = false;
static bool     calibStartWatchRunning = false;
static uint32_t calibStartWatchMs      = 0;

// View tracking - also the reference for the button rule: a press counts only
// if the view it acts on was already showing BUTTON_SCREEN_SETTLE_MS before
// the press began. shownSinceMs is when loop() first saw the current view.
static View      shownView     = View::Menu;
static FaultCode shownFault    = FaultCode::None;
static uint32_t  shownSinceMs  = 0;

static uint32_t lastSendMs    = 0;
static uint32_t lastDisplayMs = 0;
static bool     displayDirty  = true;

// ---------------------------------------------------------------------------
// ESP-NOW
// ---------------------------------------------------------------------------

// Validate, copy, timestamp. Nothing else - this runs in the WiFi task, and
// touching the display or the buzzer from here would both stall packet
// reception and race with loop() over the I2C bus. The copies land in the rx
// buffers under rxLock; loop() picks them up at the top of every iteration.
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (incomingData == nullptr || len < (int)sizeof(MessageHeader)) return;

    MessageHeader header;
    memcpy(&header, incomingData, sizeof(header));

    if (headerValid(incomingData, len, MsgType::SeederTelemetry, sizeof(SeederTelemetry))) {
        portENTER_CRITICAL(&rxLock);
        memcpy(&rxSeederData, incomingData, sizeof(rxSeederData));
        rxSeederEverSeen = true;
        portEXIT_CRITICAL(&rxLock);
        links.noteReceived(header.sender);
    } else if (headerValid(incomingData, len, MsgType::DispenserStatus, sizeof(DispenserStatus))) {
        portENTER_CRITICAL(&rxLock);
        memcpy(&rxDispenserData, incomingData, sizeof(rxDispenserData));
        rxDispenserEverSeen = true;
        portEXIT_CRITICAL(&rxLock);
        links.noteReceived(header.sender);
    }
}

// First thing loop() does: take a consistent snapshot of the received data.
static void snapshotReceived()
{
    portENTER_CRITICAL(&rxLock);
    seederData        = rxSeederData;
    dispenserData     = rxDispenserData;
    seederEverSeen    = rxSeederEverSeen;
    dispenserEverSeen = rxDispenserEverSeen;
    portEXIT_CRITICAL(&rxLock);
}

static uint32_t sendFailures       = 0;
static uint32_t lastSendErrorLogMs = 0;

// Reports failures instead of discarding them. A channel mismatch between the
// radio and the registered peer makes every send fail, and without this the
// only symptom would be "no link" with nothing to explain it.
static void broadcast(const void *packet, size_t size, uint32_t now)
{
    esp_err_t result = esp_now_send(BROADCAST_ADDRESS, (const uint8_t *)packet, size);
    if (result == ESP_OK) return;

    sendFailures++;
    if (now - lastSendErrorLogMs >= 1000) {   // rate-limited, don't flood
        lastSendErrorLogMs = now;
        Serial.print("esp_now_send failed: ");
        Serial.print(esp_err_to_name(result));
        Serial.print("  total: ");
        Serial.println(sendFailures);
    }
}

static void sendCommand(uint32_t now)
{
    if (now - lastSendMs < SEND_INTERVAL_MS) return;
    lastSendMs = now;

    TractorCommand command;
    fillHeader(command.header, MsgType::TractorCommand, NodeId::Tractor, links.flags());

    command.tramlineNumber   = tramlineNumber;
    command.tramlineRelayOn  = (tramlinesEnabled &&
                                ((TRAMLINE_ACTIVE_MASK >> tramlineNumber) & 1)) ? 1 : 0;
    command.dispenserEnabled = dispenserEnabled ? 1 : 0;
    command.calibrationRun   = calibrationRequested ? 1 : 0;
    command.doseKgPerHa      = doseKgPerHa;
    command.gramsPer100Rev   = gramsPer100Rev;
    command.upTimeMs         = now;
    command.clogClearSeq     = clogClearSeq;
    command.unclogSeq        = unclogSeq;
    command.wheelMmPerPulse  = activeWheelMmPerPulse();
    command.burstAngleFactor = angleFactor;
    command.simulationRun    = simulationRequested ? 1 : 0;

    broadcast(&command, sizeof(command), now);
}

// ---------------------------------------------------------------------------
// Button debugging - TEMPORARY, compiled only with BUTTON_DEBUG 1
// ---------------------------------------------------------------------------

// Everything in this section exists only to print. The button code below
// never reads any of it: with BUTTON_DEBUG 0 the section, the one-line hooks
// that feed it and the prints all disappear, and the button works exactly the
// same, only silently. It has its own lock, so it does not even share that.
//
// What the sampler saw and decided is recorded by the timer task and printed
// by loop(): printing from the timer task would stall the very sampling it is
// meant to show. Each record is made before the press it explains is queued,
// so its line always comes out before loop()'s line about that press. Every
// time is millis().
#if BUTTON_DEBUG
static portMUX_TYPE buttonDebugLock = portMUX_INITIALIZER_UNLOCKED;

enum class ButtonDebugKind : uint8_t {
    Blip,         // contact too short to start a press    a = when it began, b = how long
    PressStart,   // a press confirmed                     a = when it began, b = ms since the last let-go
    Spark,        // an opening too short to end a press   a = when it began, b = how long
    Release,      // a press confirmed over                a = when the finger let go, b = how long it was held
    Long,         // held for BUTTON_LONG_PRESS_MS         a = when the press began
};

struct ButtonDebugRecord {
    ButtonDebugKind kind;
    uint8_t         outcome;   // PressStart: 1 = too soon. Release: 0 = short sent, 1 = it was a long
                               // press, 2 = too soon. Long: 0 = sent, 1 = too soon, not sent.
    uint32_t        ms;        // the sample that decided it
    uint32_t        a;
    uint32_t        b;
};

static constexpr uint8_t BUTTON_DEBUG_QUEUE_SIZE = 32;
static ButtonDebugRecord buttonDebugQueue[BUTTON_DEBUG_QUEUE_SIZE];   // all three under buttonDebugLock
static uint8_t           buttonDebugFirst = 0;
static uint8_t           buttonDebugCount = 0;
static uint32_t          buttonDebugLost  = 0;
static bool              debugLongIgnoredShown = false;   // the sampler's own: one "too soon" long line per press

static void debugButton(ButtonDebugKind kind, uint8_t outcome, uint32_t ms, uint32_t a, uint32_t b)
{
    portENTER_CRITICAL(&buttonDebugLock);
    if (buttonDebugCount < BUTTON_DEBUG_QUEUE_SIZE) {
        ButtonDebugRecord &r = buttonDebugQueue[(buttonDebugFirst + buttonDebugCount) % BUTTON_DEBUG_QUEUE_SIZE];
        r.kind    = kind;
        r.outcome = outcome;
        r.ms      = ms;
        r.a       = a;
        r.b       = b;
        buttonDebugCount++;
    } else {
        buttonDebugLost++;
    }
    portEXIT_CRITICAL(&buttonDebugLock);
}

static const char *const VIEW_DEBUG_NAMES[] = {
    "Menu", "Work", "WorkFault", "EditDose", "EditCalibration", "EditAngle", "CalibMass", "Tramlines",
    "CalibConfirm", "CalibProgress", "CalibDone", "CalibRefused", "CalibNoDispenser", "CalibInterrupted",
    "ClogAlert", "ClogChoice", "Unclogging", "Seeds", "SeedCalibAsk", "SeedCalibRun",
    "SeedCalibResult", "Settings", "Blower", "Tests", "SimConfirm", "SimProgress", "SimDone",
    "SimRefused", "SimNoDispenser", "SimStopped",
};
static_assert(sizeof(VIEW_DEBUG_NAMES) / sizeof(VIEW_DEBUG_NAMES[0]) == (size_t)View::SimStopped + 1,
              "VIEW_DEBUG_NAMES must name every View, in the enum's order");

static const char *viewDebugName(View v)
{
    return VIEW_DEBUG_NAMES[(uint8_t)v];
}

// Prints and empties the sampler's records. Called by loop() every iteration,
// and by takeButtonEvent() before it says what it did with a press.
static void printButtonDebug()
{
    for (;;) {
        ButtonDebugRecord r    = {ButtonDebugKind::Blip, 0, 0, 0, 0};
        bool              have = false;
        uint32_t          lost = 0;

        portENTER_CRITICAL(&buttonDebugLock);
        if (buttonDebugCount > 0) {
            r = buttonDebugQueue[buttonDebugFirst];
            buttonDebugFirst = (uint8_t)((buttonDebugFirst + 1) % BUTTON_DEBUG_QUEUE_SIZE);
            buttonDebugCount--;
            have = true;
        }
        lost            = buttonDebugLost;
        buttonDebugLost = 0;
        portEXIT_CRITICAL(&buttonDebugLock);

        if (lost != 0) Serial.printf("[btn        ] %lu lines lost, the debug queue was full\n", (unsigned long)lost);
        if (!have) return;

        unsigned long t = r.ms, a = r.a, b = r.b;
        switch (r.kind) {
            case ButtonDebugKind::Blip:
                Serial.printf("[btn %7lu] blip     contact at %lu for %lu ms - too short for a press\n", t, a, b);
                break;
            case ButtonDebugKind::PressStart:
                if (r.outcome != 0) {
                    Serial.printf("[btn %7lu] press    began %lu, %lu ms after the last let-go -> TOO SOON (needs %lu), ignored whole\n",
                                  t, a, b, (unsigned long)BUTTON_MIN_GAP_MS);
                } else {
                    Serial.printf("[btn %7lu] press    began %lu, %lu ms after the last let-go\n", t, a, b);
                }
                break;
            case ButtonDebugKind::Spark:
                Serial.printf("[btn %7lu] spark    open at %lu for %lu ms - the press goes on\n", t, a, b);
                break;
            case ButtonDebugKind::Release:
                Serial.printf("[btn %7lu] let go   at %lu, held %lu ms -> %s\n", t, a, b,
                              r.outcome == 0 ? "SHORT sent to loop" :
                              r.outcome == 1 ? "nothing, it was a long press" :
                                               "nothing, the press was too soon");
                break;
            case ButtonDebugKind::Long:
                Serial.printf("[btn %7lu] long     press began %lu, held %lu ms -> %s\n", t, a, t - a,
                              r.outcome == 0 ? "LONG sent to loop" : "not sent, the press was too soon");
                break;
        }
    }
}
// ---- end of button debugging ---------------------------------------------
#endif

// ---------------------------------------------------------------------------
// Button
// ---------------------------------------------------------------------------

// The button is sampled every BUTTON_SAMPLE_MS by an esp_timer, from the timer
// task - never by loop(). loop() cannot see the pin while it redraws (~30 ms a
// frame, every 200 ms on the live screens and after every press), and a button
// read from loop() lost quick taps there and merged fast repeated ones into one.
// The callback does only the part that has to keep time - when a press starts
// and ends, short or long - and queues the press with the moment it began.
// Everything a press does happens in loop(), the same split as OnDataRecv.
//
// The rules are listed with the constants in machine_settings.h. The shape of
// them: a press starts quickly (BUTTON_PRESS_CONFIRM_MS) but ends slowly
// (BUTTON_RELEASE_CONFIRM_MS), so the switch bouncing or sparking under a
// finger never splits one press into two; and a press starting within
// BUTTON_MIN_GAP_MS of the last release is that release still bouncing, so it
// is ignored whole. The gap is timed from the release, not from the previous
// click: a long click is sent while the finger is still down, and a bounce when
// it finally lets go, more than BUTTON_MIN_GAP_MS later, would otherwise click
// again.
//
// A level, not an edge: a pin-change interrupt fires on every bounce and on
// every spike picked up by the cable, and a spike taken as a press on the work
// screen would move the tramline pass unseen.
//
// A long press fires the moment the hold time is reached, while the button is
// still down, so the operator gets feedback without having to watch the
// screen. The release that follows is swallowed.

struct ButtonPress {
    ButtonEvent type;
    uint32_t    startMs;   // when the finger went down, to within one sample
};

static constexpr uint32_t BUTTON_PRESS_SAMPLES   = BUTTON_PRESS_CONFIRM_MS / BUTTON_SAMPLE_MS;
static constexpr uint32_t BUTTON_RELEASE_SAMPLES = BUTTON_RELEASE_CONFIRM_MS / BUTTON_SAMPLE_MS;
static constexpr uint8_t  BUTTON_QUEUE_SIZE      = 8;

// Shared between the timer task and loop(): only ever touched under buttonLock.
static ButtonPress  buttonQueue[BUTTON_QUEUE_SIZE];
static uint8_t      buttonQueueFirst = 0;
static uint8_t      buttonQueueCount = 0;
static portMUX_TYPE buttonLock = portMUX_INITIALIZER_UNLOCKED;

// The sampler's own state: read and written by buttonSample() alone.
static bool     sampledDown      = false;   // a press is in progress
static uint32_t sampledRun       = 0;       // samples in a row against that: closed ones between
                                            // presses, open ones during a press
static uint32_t sampledRunStart  = 0;       // when that run of samples began
static uint32_t sampledPressMs   = 0;       // when the current press began
static uint32_t sampledReleaseMs = 0;       // when the finger last let go, of any press
static bool     sampledTooSoon   = false;   // the current press began inside BUTTON_MIN_GAP_MS of that
static bool     sampledLongSent  = false;

static esp_timer_handle_t buttonTimer = nullptr;

static void queueButtonPress(ButtonEvent type, uint32_t startMs)
{
    portENTER_CRITICAL(&buttonLock);
    // A full queue means loop() has been stuck for seconds. The newest press is
    // the one dropped, so the ones already waiting still run in order.
    if (buttonQueueCount < BUTTON_QUEUE_SIZE) {
        uint8_t slot = (uint8_t)((buttonQueueFirst + buttonQueueCount) % BUTTON_QUEUE_SIZE);
        buttonQueue[slot].type    = type;
        buttonQueue[slot].startMs = startMs;
        buttonQueueCount++;
    }
    portEXIT_CRITICAL(&buttonLock);
}

// The esp_timer callback. It runs in the esp_timer task, not in an interrupt,
// so millis() and digitalRead() are safe here.
static void buttonSample(void *)
{
    uint32_t now  = millis();
    bool     down = (digitalRead(BUTTON_PIN) == LOW);

    if (down == sampledDown) {
#if BUTTON_DEBUG
        // A run that came back before it counted: an opening during a press,
        // or contact between presses, each too short to change anything.
        if (sampledRun > 0) {
            debugButton(sampledDown ? ButtonDebugKind::Spark : ButtonDebugKind::Blip, 0,
                        now, sampledRunStart, now - sampledRunStart);
        }
#endif
        sampledRun = 0;                        // bounce, a spark or a spike: the level came back
    } else {
        if (sampledRun == 0) sampledRunStart = now;
        sampledRun++;
        // Quick to start a press, slow to end one.
        if (sampledRun >= (down ? BUTTON_PRESS_SAMPLES : BUTTON_RELEASE_SAMPLES)) {
            sampledDown = down;
            sampledRun  = 0;
            if (down) {
                // Where the finger actually pressed, not where the confirmation
                // finished: the button rule compares this with the view.
                sampledPressMs  = sampledRunStart;
                sampledLongSent = false;
                sampledTooSoon  = (sampledPressMs - sampledReleaseMs < BUTTON_MIN_GAP_MS);
#if BUTTON_DEBUG
                debugLongIgnoredShown = false;
                debugButton(ButtonDebugKind::PressStart, sampledTooSoon ? 1 : 0,
                            now, sampledPressMs, sampledPressMs - sampledReleaseMs);
#endif
            } else {
                sampledReleaseMs = sampledRunStart;   // where the finger let go
#if BUTTON_DEBUG
                debugButton(ButtonDebugKind::Release, sampledLongSent ? 1 : (sampledTooSoon ? 2 : 0),
                            now, sampledReleaseMs, sampledReleaseMs - sampledPressMs);
#endif
                if (!sampledLongSent && !sampledTooSoon) {
                    queueButtonPress(ButtonEvent::Short, sampledPressMs);
                }
            }
        }
    }

    // Only on a closed sample: an open one may be the press ending, and a press
    // that ends before BUTTON_LONG_PRESS_MS is a short one.
    if (sampledDown && down && !sampledLongSent && !sampledTooSoon &&
        (now - sampledPressMs >= BUTTON_LONG_PRESS_MS)) {
        sampledLongSent = true;                // the release after it is swallowed
#if BUTTON_DEBUG
        debugButton(ButtonDebugKind::Long, 0, now, sampledPressMs, 0);
#endif
        queueButtonPress(ButtonEvent::Long, sampledPressMs);
    }

#if BUTTON_DEBUG
    // Held long enough for a long press, but the press was too soon: said once,
    // so the log shows the long press that did not happen.
    if (sampledDown && down && sampledTooSoon && !debugLongIgnoredShown &&
        (now - sampledPressMs >= BUTTON_LONG_PRESS_MS)) {
        debugLongIgnoredShown = true;
        debugButton(ButtonDebugKind::Long, 1, now, sampledPressMs, 0);
    }
#endif
}

static void startButtonSampling()
{
    esp_timer_create_args_t args = {};
    args.callback        = &buttonSample;
    args.arg             = nullptr;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name            = "button";

    if (esp_timer_create(&args, &buttonTimer) != ESP_OK ||
        esp_timer_start_periodic(buttonTimer, (uint64_t)BUTTON_SAMPLE_MS * 1000ULL) != ESP_OK) {
        Serial.println("Button timer failed to start - the button will not respond");
    }
}

// The next press waiting, if any, after the button rule: a press counts only
// if the view it would act on was already showing BUTTON_SCREEN_SETTLE_MS
// before the press began. shownSinceMs is when that view appeared, so a view
// that changed at any point during the press - or between the release and
// loop() getting to it - is later than the press start and fails the test as
// well. That covers "the view changed while the button was held" and "the press
// started just before a new view appeared": either way the press would act on a
// screen the operator was not looking at. An ignored press makes no beep.
static ButtonEvent takeButtonEvent()
{
    ButtonPress press = {ButtonEvent::None, 0};

    portENTER_CRITICAL(&buttonLock);
    if (buttonQueueCount > 0) {
        press = buttonQueue[buttonQueueFirst];
        buttonQueueFirst = (uint8_t)((buttonQueueFirst + 1) % BUTTON_QUEUE_SIZE);
        buttonQueueCount--;
    }
    portEXIT_CRITICAL(&buttonLock);

    if (press.type == ButtonEvent::None) return ButtonEvent::None;
    bool counts = (int32_t)(press.startMs - shownSinceMs) >= (int32_t)BUTTON_SCREEN_SETTLE_MS;

#if BUTTON_DEBUG
    printButtonDebug();   // the sampler's line about this press comes first
    Serial.printf("[btn %7lu] loop     takes %s (began %lu) on %s, up %ld ms when it began -> ",
                  (unsigned long)millis(), press.type == ButtonEvent::Long ? "LONG" : "SHORT",
                  (unsigned long)press.startMs, viewDebugName(shownView),
                  (long)(int32_t)(press.startMs - shownSinceMs));
    if (counts) {
        Serial.println("COUNTED");
    } else {
        Serial.printf("IGNORED, screen rule: it must be up %lu ms first\n", (unsigned long)BUTTON_SCREEN_SETTLE_MS);
    }
#endif

    return counts ? press.type : ButtonEvent::None;
}

// ---------------------------------------------------------------------------
// Numeric editor
// ---------------------------------------------------------------------------

static void loadDigits(uint32_t value, uint8_t count)
{
    digitCount   = count;
    cursor       = 0;
    editingDigit = false;
    for (int8_t i = count - 1; i >= 0; i--) {
        digits[i] = value % 10;
        value /= 10;
    }
}

static uint32_t digitsToValue()
{
    uint32_t value = 0;
    for (uint8_t i = 0; i < digitCount; i++) {
        value = value * 10 + digits[i];
    }
    return value;
}

// What the burst calibration run should weigh at the dose and seed size in use -
// so it follows both by itself.
static uint32_t calibExpectedGrams()
{
    return expectedCalibrationGrams(doseKgPerHa, activeWheelMmPerPulse());
}

static void storeAngleFactor(uint16_t factor)
{
    if (factor > ANGLE_FACTOR_MAX) factor = ANGLE_FACTOR_MAX;
    angleFactor = factor;
    prefs.putUShort("angle", angleFactor);
}

static void saveEditedValue()
{
    uint32_t value = digitsToValue();

    switch (screen) {
        case Screen::EditDose: {
            if (value > MAX_DOSE_KG_PER_HA) value = MAX_DOSE_KG_PER_HA;
            // The burst angle a pulse needs is in proportion to the dose, so the
            // angle factor follows it: a new dose needs no new calibration run to
            // be about right - though grams per turn need not be quite constant,
            // so after a large change one is worth doing.
            uint16_t factor = angleFactorForDose(angleFactor, doseKgPerHa, (uint16_t)value);
            doseKgPerHa = (uint16_t)value;
            prefs.putUShort("dose", doseKgPerHa);
            prefs.putBool("disp_on", dispenserEnabled);
            if (factor != angleFactor) storeAngleFactor(factor);
            break;
        }
        case Screen::EditAngle:
            storeAngleFactor((uint16_t)value);
            break;
        case Screen::CalibMass:
            // The weighed mass: the factor that would have made it the expected
            // one. A mass of 0 leaves the factor as it was.
            storeAngleFactor(angleFactorFromWeighing(angleFactor, calibExpectedGrams(), value));
            break;
        default:
            if (value > MAX_GRAMS_PER_100REV) value = MAX_GRAMS_PER_100REV;
            gramsPer100Rev = value;
            prefs.putULong("calib", gramsPer100Rev);
            break;
    }
}

// Screen 5 in burst metering: the angle factor, with the cursor on the first
// digit, or on TEST when coming back to it for another run.
static void openAngleEditor(bool onTest)
{
    screen = Screen::EditAngle;
    loadDigits(angleFactor, 3);
    if (onTest) cursor = digitCount;
}

// Screen 5b: the weighed mass, starting from what the run should weigh. Left
// as it is, ZAPISZ then changes nothing.
static void openMassEditor()
{
    uint32_t expected = calibExpectedGrams();
    if (expected > MAX_CALIB_MASS_GRAMS) expected = MAX_CALIB_MASS_GRAMS;
    screen = Screen::CalibMass;
    loadDigits(expected, 5);
}

// Where the calibration run's screens return to: the editor TEST was on.
static Screen calibEditorScreen()
{
    return DISPENSER_BURST_MODE ? Screen::EditAngle : Screen::EditCalibration;
}

// ---------------------------------------------------------------------------
// Screen logic
// ---------------------------------------------------------------------------

static void handleMenu(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        menuIndex = (menuIndex + 1) % (uint8_t)MenuItem::Count;
        // Keep the cursor on screen. Wrapping back to the first item pulls the
        // window back to the top by itself.
        if (menuIndex < menuTop)                          menuTop = menuIndex;
        if (menuIndex > menuTop + (MENU_VISIBLE_ROWS - 1)) menuTop = menuIndex - (MENU_VISIBLE_ROWS - 1);
    } else if (event == ButtonEvent::Long) {
        switch ((MenuItem)menuIndex) {
            case MenuItem::Praca:
                screen = Screen::Work;
                break;
            case MenuItem::Dawka:
                screen = Screen::EditDose;
                loadDigits(doseKgPerHa, 3);
                break;
            case MenuItem::Kalibracja:
                if (DISPENSER_BURST_MODE) {
                    openAngleEditor(false);
                } else {
                    screen = Screen::EditCalibration;
                    loadDigits(gramsPer100Rev, 5);
                }
                break;
            case MenuItem::Sciezki:
                screen = Screen::Tramlines;
                tramlineCursor = 0;   // cursor starts on the switch
                break;
            case MenuItem::Nasiona:
                screen = Screen::Seeds;
                seedCursor = 0;       // cursor starts on the first size, never on Wroc
                break;
            case MenuItem::Ustawienia:
                screen = Screen::Settings;
                settingsCursor = 0;   // cursor starts on the export
                settingsSentMs = 0;
                break;
            case MenuItem::Dmuchawa:
                screen = Screen::Blower;
                blowerCursor = 0;     // cursor starts on the switch
                break;
            case MenuItem::Testy:
                screen = Screen::Tests;
                testsCursor = 0;      // cursor starts on the first test
                break;
            default:
                break;
        }
    }
}

static void handleWork(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        // Nothing to change while tramlines are off - and the pass number is
        // not even on the screen then, so a blind change would be worse.
        if (tramlinesEnabled) tramlineNumber = (tramlineNumber + 1) % TRAMLINE_RHYTHM;
    } else if (event == ButtonEvent::Long) {
        screen = Screen::Menu;
    }
}

// Screen 15: two fields, 0 = the switch and 1 = ZAPISZ. As on the editors,
// ZAPISZ is the only way out, so leaving always leaves the value as drawn -
// and the switch itself is stored the moment it is flipped.
static void handleTramlines(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        tramlineCursor = (tramlineCursor + 1) % 2;
    } else if (event == ButtonEvent::Long) {
        if (tramlineCursor == 0) {
            tramlinesEnabled = !tramlinesEnabled;
            prefs.putBool("tram_on", tramlinesEnabled);
        } else {
            screen = Screen::Menu;
        }
    }
}

// Screen 21: works like screen 15, except that the switch is not stored - see
// blowerAlarmEnabled.
static void handleBlower(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        blowerCursor = (blowerCursor + 1) % 2;
    } else if (event == ButtonEvent::Long) {
        if (blowerCursor == 0) {
            blowerAlarmEnabled = !blowerAlarmEnabled;
        } else {
            screen = Screen::Menu;
        }
    }
}

// Screen 16: the seed-size gear and its wheel calibration. Picking a size
// stores it at once, like the tramline switch - there is nothing pending to
// save, so the way out is the Wroc row rather than a ZAPISZ field.
static void handleSeeds(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        seedCursor = (seedCursor + 1) % 4;
        return;
    }
    if (event != ButtonEvent::Long) return;

    switch (seedCursor) {
        case 0:
        case 1: {
            bool large = (seedCursor == 1);
            if (large != seedLarge) {
                seedLarge = large;
                prefs.putBool("seed_l", seedLarge);
            }
            break;
        }
        case 2:
            seedAskIndex = 0;                  // Anuluj preselected
            screen = Screen::SeedCalibAsk;
            break;
        default:
            screen = Screen::Menu;
            break;
    }
}

// Start of a wheel calibration run: remember where the seeder's cumulative
// pulse counter stood, and its uptime, so a reboot of the seeder mid-run can be
// told from a simple link gap. A gap loses nothing - the counter is cumulative.
static void startWheelCalibration()
{
    calibWheelError  = WheelCalibError::None;
    seedResultIndex  = 0;

    if (!seederEverSeen || !links.isAlive(NodeId::Seeder)) {
        calibWheelError = WheelCalibError::NoSeeder;
        screen = Screen::SeedCalibResult;
        return;
    }

    calibWheelStartPulses = seederData.wheelPulses;
    calibWheelStartUpTime = seederData.upTimeMs;
    calibWheelPulses      = 0;
    screen = Screen::SeedCalibRun;
}

static void finishWheelCalibration()
{
    calibWheelError = WheelCalibError::None;
    seedResultIndex = 0;

    if (!links.isAlive(NodeId::Seeder)) {
        calibWheelError = WheelCalibError::NoSeeder;
    } else if (seederData.upTimeMs < calibWheelStartUpTime) {
        // Its counter restarted, so the difference means nothing.
        calibWheelError = WheelCalibError::SeederReset;
    } else {
        calibWheelPulses = seederData.wheelPulses - calibWheelStartPulses;
        if (calibWheelPulses < WHEEL_CALIB_MIN_PULSES) {
            calibWheelError = WheelCalibError::TooFew;
        } else {
            // Rounded, not truncated: one millimetre matters over 100 m.
            uint32_t mm = ((uint32_t)WHEEL_CALIB_DISTANCE_M * 1000UL + calibWheelPulses / 2) / calibWheelPulses;
            if (mm < WHEEL_MM_PER_PULSE_MIN || mm > WHEEL_MM_PER_PULSE_MAX) {
                calibWheelError = WheelCalibError::OutOfRange;
            } else {
                calibWheelResultMm = (uint16_t)mm;
            }
        }
    }
    screen = Screen::SeedCalibResult;
}

static void handleSeedCalibAsk(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        seedAskIndex = (seedAskIndex + 1) % 2;
    } else if (event == ButtonEvent::Long) {
        if (seedAskIndex == 1) startWheelCalibration();
        else                   screen = Screen::Seeds;
    }
}

// Screen 18: only a long press ends the run. A stray short press must not throw
// away a 100 m drive - the same reasoning as the dispenser's calibration run.
static void handleSeedCalibRun(ButtonEvent event)
{
    if (event == ButtonEvent::Long) finishWheelCalibration();
}

static void handleSeedCalibResult(ButtonEvent event)
{
    if (calibWheelError != WheelCalibError::None) {
        if (event != ButtonEvent::None) screen = Screen::Seeds;
        return;
    }

    if (event == ButtonEvent::Short) {
        seedResultIndex = (seedResultIndex + 1) % 2;
    } else if (event == ButtonEvent::Long) {
        if (seedResultIndex == 1) {
            // Into the slot for the size being calibrated, never the other one.
            if (seedLarge) {
                wheelMmLarge = calibWheelResultMm;
                prefs.putUShort("wheel_l", wheelMmLarge);
            } else {
                wheelMmSmall = calibWheelResultMm;
                prefs.putUShort("wheel_s", wheelMmSmall);
            }
        }
        screen = Screen::Seeds;
    }
}

// Everything the tractor keeps in NVS, printed over USB in one block: the
// operator reads it in a serial monitor at 115200 and keeps it somewhere safe
// (docs/calibration-settings.txt). The second half is the same values written
// as the first-boot constants, so restoring a blank board is a paste into
// include/machine_settings.h and a flash.
//
// There is deliberately no way in: no serial command and no packet can write a
// setting. A calibration costs a drive across the field to measure, and the
// only thing that may overwrite one is the operator, on the screen that
// measured it.
static void exportSettings()
{
    Serial.println();
    Serial.println("=== TRACTOR SETTINGS ===");
    Serial.printf("  protocol            %u\n",            (unsigned)PROTOCOL_VERSION);
    Serial.printf("  uptime              %lu s\n",          (unsigned long)(millis() / 1000));
    Serial.printf("  dose                %u kg/ha\n",       (unsigned)doseKgPerHa);
    Serial.printf("  dispenser calib     %lu g per 100 rev\n", (unsigned long)gramsPer100Rev);
    Serial.printf("  angle factor        %u (burst metering %s)\n", (unsigned)angleFactor,
                  DISPENSER_BURST_MODE ? "ON" : "OFF");
    Serial.printf("  dispenser           %s\n",             dispenserEnabled ? "ON" : "OFF");
    Serial.printf("  tramlines           %s\n",             tramlinesEnabled ? "ON" : "OFF");
    Serial.printf("  seed size           %s\n",             seedLarge ? "LARGE" : "SMALL");
    Serial.printf("  wheel mm per pulse  small %u, large %u\n",
                  (unsigned)wheelMmSmall, (unsigned)wheelMmLarge);
    Serial.println("  --- paste over the first-boot values in include/machine_settings.h ---");
    Serial.printf("  static constexpr uint16_t DEFAULT_DOSE_KG_PER_HA    = %u;\n",  (unsigned)doseKgPerHa);
    Serial.printf("  static constexpr uint32_t DEFAULT_GRAMS_PER_100REV  = %lu;\n", (unsigned long)gramsPer100Rev);
    Serial.printf("  static constexpr bool     DEFAULT_DISPENSER_ENABLED = %s;\n",  dispenserEnabled ? "true" : "false");
    Serial.printf("  static constexpr bool     DEFAULT_TRAMLINES_ENABLED = %s;\n",  tramlinesEnabled ? "true" : "false");
    Serial.printf("  static constexpr bool     DEFAULT_SEED_LARGE        = %s;\n",  seedLarge ? "true" : "false");
    Serial.printf("  static constexpr uint16_t DEFAULT_WHEEL_MM_SMALL    = %u;\n",  (unsigned)wheelMmSmall);
    Serial.printf("  static constexpr uint16_t DEFAULT_WHEEL_MM_LARGE    = %u;\n",  (unsigned)wheelMmLarge);
    Serial.printf("  static constexpr uint16_t DEFAULT_ANGLE_FACTOR      = %u;\n",  (unsigned)angleFactor);
    Serial.println("=== END, keep this with the date in docs/calibration-settings.txt ===");
}

// Screen 20: two rows, send and leave. Sending takes about 50 ms of serial
// writing, which is why it is a screen of its own and not something the work
// screen can trigger.
static void handleSettings(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        settingsCursor = (settingsCursor + 1) % 2;
    } else if (event == ButtonEvent::Long) {
        if (settingsCursor == 0) {
            exportSettings();
            settingsSentMs = millis();
        } else {
            screen = Screen::Menu;
        }
    }
}

// The editors' action fields after the digits: the screen's own action and
// ZAPISZ, or TEST, MASA and ZAPISZ on the angle factor's.
static uint8_t editActionCount()
{
    return (screen == Screen::EditAngle) ? 3 : 2;
}

static void handleEdit(ButtonEvent event)
{
    const uint8_t fields     = digitCount + editActionCount();
    const uint8_t extraField = digitCount;        // WL./WYL., TEST or WROC
    const uint8_t saveField  = fields - 1;        // ZAPISZ, always the last

    if (event == ButtonEvent::Short) {
        if (editingDigit) {
            digits[cursor] = (digits[cursor] + 1) % 10;
        } else {
            cursor = (cursor + 1) % fields;       // wraps through the actions
        }
        return;
    }

    if (event != ButtonEvent::Long) return;

    if (editingDigit) {
        editingDigit = false;                     // commit this digit
    } else if (cursor == saveField) {
        saveEditedValue();
        // The mass is only a way to the factor, so it goes back to it, ready
        // for the next run.
        if (screen == Screen::CalibMass) openAngleEditor(true);
        else                             screen = Screen::Menu;
    } else if (cursor == extraField) {
        switch (screen) {
            case Screen::EditDose:
                dispenserEnabled = !dispenserEnabled;
                break;
            case Screen::CalibMass:
                openAngleEditor(true);            // WROC: back, the factor untouched
                break;
            default:
                // TEST. A burst run goes by the angle factor, so the one on
                // screen is stored first: the run tests what the operator sees.
                // The continuous run is 100 turns whatever the calibration.
                if (screen == Screen::EditAngle) saveEditedValue();
                calibConfirmIndex = 0;            // default to Anuluj
                screen = Screen::CalibConfirm;
                break;
        }
    } else if (cursor > extraField) {
        // MASA, on the angle factor's screen only. Stores the factor on screen
        // first, for the same reason as TEST: the mass corrects that one.
        saveEditedValue();
        openMassEditor();
    } else {
        editingDigit = true;
    }
}

static void handleCalibConfirm(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        calibConfirmIndex = (calibConfirmIndex + 1) % 2;
    } else if (event == ButtonEvent::Long) {
        if (calibConfirmIndex == 1) {
            calibrationRequested    = true;
            calibrationInterrupted  = false;
            calibStartWatchRunning  = false;
            calibStartWatchMs       = 0;
            screen = Screen::CalibRunning;
        } else {
            screen = calibEditorScreen();       // cursor is still on TEST
        }
    }
}

// Cancel (while turning) or dismiss (done / refused / interrupted) a
// calibration run; also how the run is withdrawn when the dispenser's side
// dropped it.
static void cancelCalibration()
{
    calibrationRequested   = false;
    calibrationInterrupted = false;
    calibStartWatchRunning = false;
    calibStartWatchMs      = 0;
    screen = calibEditorScreen();       // cursor is still on TEST
}

// While the shaft is turning only a long press cancels, so a stray short
// press can't stop a run half way through and spoil the weighing.
static void handleCalibTurning(ButtonEvent event)
{
    if (event == ButtonEvent::Long) cancelCalibration();
}

// Finished, refused or interrupted: any press dismisses it.
static void handleCalibResult(ButtonEvent event)
{
    if (event != ButtonEvent::None) cancelCalibration();
}

// Finished. In burst metering the press goes straight on to entering what the
// run weighed.
static void handleCalibDone(ButtonEvent event)
{
    if (event == ButtonEvent::None) return;
    cancelCalibration();
    if (DISPENSER_BURST_MODE) openMassEditor();
}

// Screen 22: a short press moves down the tests, a long one opens the one the
// cursor is on.
static void handleTests(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        testsCursor = (uint8_t)((testsCursor + 1) % TEST_ROW_COUNT);
    } else if (event == ButtonEvent::Long) {
        switch (testRow(testsCursor)) {
            case TestRow::Simulation:
                simConfirmIndex = 0;                  // default to Anuluj
                screen = Screen::SimConfirm;
                break;
            default:
                screen = Screen::Menu;
                break;
        }
    }
}

// Screen 23: Anuluj (preselected) or START.
static void handleSimConfirm(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        simConfirmIndex = (simConfirmIndex + 1) % 2;
    } else if (event == ButtonEvent::Long) {
        if (simConfirmIndex == 1) {
            simulationRequested  = true;
            simInterrupted       = false;
            simStartWatchRunning = false;
            simSeen              = false;
            simElapsedMs         = 0;
            simBursts            = 0;
            simFailures          = 0;
            simUnclogs           = 0;
            screen = Screen::SimRunning;
        } else {
            screen = Screen::Tests;
        }
    }
}

// Withdraws the request, whatever state the run is in.
static void stopSimulation()
{
    simulationRequested  = false;
    simStartWatchRunning = false;
}

// Running, or waiting for the dispenser: only a long press stops it - a stray
// short one must not throw away a quarter of an hour - and the screen then
// shows how far it got.
static void handleSimRunning(ButtonEvent event)
{
    if (event != ButtonEvent::Long) return;
    stopSimulation();
    screen = Screen::SimResult;
}

// Finished, refused or stopped: any press, back to the tests.
static void handleSimFinished(ButtonEvent event)
{
    if (event == ButtonEvent::None) return;
    stopSimulation();
    simInterrupted = false;
    screen = Screen::Tests;
}

static void handleWorkFault(ButtonEvent event)
{
    // The pass number is hidden behind the fault screen, so a short press
    // must not change it blind.
    if (event == ButtonEvent::Long) screen = Screen::Menu;
}

// The alarm itself: any press acknowledges it and reveals the choice. The
// cursor is reset here too, so the first frame of the choice screen cannot show
// the previous clog's highlight.
static void handleClogAlert(ButtonEvent event)
{
    if (event != ButtonEvent::None) {
        clogAcknowledged = true;
        clogChoiceIndex  = 0;
    }
}

// Anuluj / Odetkaj. The counters are what the dispenser acts on once; the
// view stays here until the dispenser reports its new mode (~0.2-0.4 s), and
// a second pick in that window is harmless because the dispenser acts once
// per change and ignores changes outside Clogged.
static void handleClogChoice(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        clogChoiceIndex = (clogChoiceIndex + 1) % 2;
    } else if (event == ButtonEvent::Long) {
        if (clogChoiceIndex == 0) {
            clogClearSeq++;
        } else {
            unclogSeq++;
        }
    }
}

// Dispatch on the view, never on screen alone - the clog overlay covers the
// screen underneath, and its presses must not leak through to it.
static void handleView(View view, ButtonEvent event)
{
    switch (view) {
        case View::Menu:              handleMenu(event);          break;
        case View::Work:              handleWork(event);          break;
        case View::WorkFault:         handleWorkFault(event);     break;
        case View::EditDose:
        case View::EditCalibration:
        case View::EditAngle:
        case View::CalibMass:         handleEdit(event);          break;
        case View::Tramlines:         handleTramlines(event);     break;
        case View::Blower:            handleBlower(event);        break;
        case View::Seeds:             handleSeeds(event);         break;
        case View::SeedCalibAsk:      handleSeedCalibAsk(event);  break;
        case View::SeedCalibRun:      handleSeedCalibRun(event);  break;
        case View::SeedCalibResult:   handleSeedCalibResult(event); break;
        case View::Settings:          handleSettings(event);      break;
        case View::CalibConfirm:      handleCalibConfirm(event);  break;
        case View::CalibProgress:
        case View::CalibNoDispenser:  handleCalibTurning(event);  break;
        case View::CalibDone:         handleCalibDone(event);     break;
        case View::CalibRefused:
        case View::CalibInterrupted:  handleCalibResult(event);   break;
        case View::ClogAlert:         handleClogAlert(event);     break;
        case View::ClogChoice:        handleClogChoice(event);    break;
        case View::Unclogging:        break;   // nothing responds while the sequence runs
        case View::Tests:             handleTests(event);         break;
        case View::SimConfirm:        handleSimConfirm(event);    break;
        case View::SimProgress:
        case View::SimNoDispenser:    handleSimRunning(event);    break;
        case View::SimDone:
        case View::SimRefused:
        case View::SimStopped:        handleSimFinished(event);   break;
    }
}

// ---------------------------------------------------------------------------
// Faults, LEDs, buzzer
// ---------------------------------------------------------------------------

// "Down" drives the LEDs and the corner letters: it includes never having
// been heard from, which is exactly the state the blue LED has always shown
// while waiting for the seeder to come up.
static bool seederLinkDown()
{
    return !links.isAlive(NodeId::Seeder);
}

// The dispenser only counts as missing once it has been seen at least once,
// so the system still runs cleanly as a two-module setup until that board
// actually exists.
static bool dispenserLinkDown()
{
    return dispenserEverSeen && !links.isAlive(NodeId::Dispenser);
}

// "Lost" is the stricter notion that drives the buzzer: a peer that was
// talking to us and then went quiet. Powering up the tractor before the
// seeder is not a fault, so it must not make noise.
static bool anyLinkLost()
{
    if (seederEverSeen && !links.isAlive(NodeId::Seeder))       return true;
    if (dispenserEverSeen && !links.isAlive(NodeId::Dispenser)) return true;
    return false;
}

// True when both peers are talking to us but cannot hear each other. Only
// visible thanks to the linkFlags byte each of them sends.
static bool crossLinkDown()
{
    if (!links.isAlive(NodeId::Seeder) || !links.isAlive(NodeId::Dispenser)) return false;
    return !(seederData.header.linkFlags & LINK_HEARD_DISPENSER);
}

static bool anyLinkDown()
{
    return seederLinkDown() || dispenserLinkDown() || crossLinkDown();
}

static void updateFaults(uint32_t now)
{
    // Timed from when contact was LOST, not from boot - see anyLinkLost().
    if (anyLinkLost() || crossLinkDown()) {
        if (linkDownSinceMs == 0) linkDownSinceMs = now;
    } else {
        linkDownSinceMs = 0;
    }

    // The fan, but only on telemetry the seeder is actually still sending: once
    // it goes quiet, the last thing it said is no evidence about the fan (the
    // same rule the dispenser fault below follows). The condition also has to
    // hold for TURBINE_ALARM_DELAY_MS, or moving off before the fan is up to
    // speed would beep every time. Switched off on screen 21 it never fires, and
    // switching it back on starts the delay afresh.
    bool fanStopped = blowerAlarmEnabled &&
                      links.isAlive(NodeId::Seeder) && seederData.wheelTurning &&
                      seederData.turbineRPM < TURBINE_RUNNING_MIN_RPM;
    if (fanStopped) {
        if (turbineBadSinceMs == 0) turbineBadSinceMs = now;
    } else {
        turbineBadSinceMs = 0;
    }

    // Highest priority first: a stopped fan ruins the pass outright, an
    // over-running dispenser only gets the rate wrong.
    if (fanStopped && (now - turbineBadSinceMs) >= TURBINE_ALARM_DELAY_MS) {
        faultCode = FaultCode::TurbineOff;
    } else if (dispenserEverSeen && links.isAlive(NodeId::Dispenser) &&
               dispenserData.faultCode == DispenserFault::OverSpeed) {
        faultCode = FaultCode::DispenserOverSpeed;
    } else if (linkDownSinceMs != 0 && (now - linkDownSinceMs) >= LINK_BUZZER_DELAY_MS) {
        faultCode = FaultCode::LinkLost;
    } else {
        faultCode = FaultCode::None;
    }
}

// Everything except LinkLost replaces the work screen entirely. Defined in
// the drawing section; declared here because currentView() needs it.
static bool faultTakesOverScreen();

// Clog overlay and calibration-run bookkeeping. Must run after updateFaults
// and before the view is worked out, so the button rule sees the same view
// the operator sees.
static void updateScreenBookkeeping(uint32_t now)
{
    bool dispenserAlive = links.isAlive(NodeId::Dispenser);
    bool clogActive = dispenserAlive &&
                      (dispenserData.mode == DispenserMode::Clogged ||
                       dispenserData.mode == DispenserMode::Unclogging);

    // The acknowledgement is per clog, not forever.
    if (!clogActive) clogAcknowledged = false;

    // A clog during a calibration run drops the run on the dispenser's side.
    if (clogActive && screen == Screen::CalibRunning) {
        calibrationRequested   = false;
        calibrationInterrupted = false;
        calibStartWatchRunning = false;
        calibStartWatchMs      = 0;
        screen = calibEditorScreen();       // cursor is still on TEST
    }

    // After START the dispenser must be Calibrating. If it reports Normal
    // instead, its side dropped the run (reboot, link gap, clog, cancel) and
    // it will never start by itself - so tell the operator instead of sitting
    // at 0 % forever.
    if (screen == Screen::CalibRunning && !calibrationInterrupted) {
        if (dispenserAlive && dispenserData.mode == DispenserMode::Normal) {
            if (!calibStartWatchRunning) {
                calibStartWatchRunning = true;
                calibStartWatchMs      = now;
            }
            if (now - calibStartWatchMs >= CALIBRATION_START_TIMEOUT_MS) {
                calibrationInterrupted = true;
                calibrationRequested   = false;
                calibStartWatchRunning = false;
                calibStartWatchMs      = 0;
            }
        } else {
            calibStartWatchRunning = false;
        }
    }

    // The seeding simulation: copy its counts and time it while it runs, so a
    // run that stops early can still show them. And, as for the calibration
    // run, a dispenser that sits in Normal after START - or drops back to it
    // part-way - has dropped the run.
    if (screen == Screen::SimRunning && !simInterrupted) {
        bool running = dispenserData.mode == DispenserMode::Simulating;
        if (dispenserAlive && (running || dispenserData.mode == DispenserMode::SimulationDone)) {
            if (!simSeen) {
                simSeen     = true;
                simSeenAtMs = now;
            }
            simElapsedMs = running ? now - simSeenAtMs : SIMULATION_DURATION_MS;
            if (simElapsedMs > SIMULATION_DURATION_MS) simElapsedMs = SIMULATION_DURATION_MS;
            simBursts   = dispenserData.simBursts;
            simFailures = dispenserData.simFailures;
            simUnclogs  = dispenserData.simUnclogs;
        }
        if (dispenserAlive && dispenserData.mode == DispenserMode::Normal) {
            if (!simStartWatchRunning) {
                simStartWatchRunning = true;
                simStartWatchMs      = now;
            }
            if (now - simStartWatchMs >= CALIBRATION_START_TIMEOUT_MS) {
                simInterrupted = true;
                stopSimulation();
            }
        } else {
            simStartWatchRunning = false;
        }
    }
}

// What the display currently shows. The clog overlay covers every screen
// while the dispenser reports a clog (and only while it is alive - see the
// trap about stale data). Burst metering's own unclog after failed bursts
// (AutoUnclogging) shows nothing at all: the yellow LED's blinks are its only
// sign, by the user's choice.
static View currentView()
{
    bool dispenserAlive = links.isAlive(NodeId::Dispenser);

    if (dispenserAlive && (dispenserData.mode == DispenserMode::Clogged ||
                           dispenserData.mode == DispenserMode::Unclogging)) {
        if (dispenserData.mode == DispenserMode::Unclogging) return View::Unclogging;
        return clogAcknowledged ? View::ClogChoice : View::ClogAlert;
    }

    switch (screen) {
        case Screen::Menu:            return View::Menu;
        case Screen::Work:            return faultTakesOverScreen() ? View::WorkFault : View::Work;
        case Screen::EditDose:        return View::EditDose;
        case Screen::EditCalibration: return View::EditCalibration;
        case Screen::EditAngle:       return View::EditAngle;
        case Screen::CalibMass:       return View::CalibMass;
        case Screen::Tramlines:       return View::Tramlines;
        case Screen::Seeds:           return View::Seeds;
        case Screen::SeedCalibAsk:    return View::SeedCalibAsk;
        case Screen::SeedCalibRun:    return View::SeedCalibRun;
        case Screen::SeedCalibResult: return View::SeedCalibResult;
        case Screen::Settings:        return View::Settings;
        case Screen::Blower:          return View::Blower;
        case Screen::CalibConfirm:    return View::CalibConfirm;
        case Screen::CalibRunning:
            if (calibrationInterrupted)            return View::CalibInterrupted;
            if (!dispenserAlive)                   return View::CalibNoDispenser;
            switch (dispenserData.mode) {
                case DispenserMode::Calibrating:     return View::CalibProgress;
                case DispenserMode::CalibrationDone: return View::CalibDone;
                case DispenserMode::Refused:         return View::CalibRefused;
                default:                             return View::CalibProgress;   // 0 % while starting
            }
        case Screen::Tests:           return View::Tests;
        case Screen::SimConfirm:      return View::SimConfirm;
        case Screen::SimResult:       return View::SimStopped;
        case Screen::SimRunning:
            if (simInterrupted)                    return View::SimStopped;
            if (!dispenserAlive)                   return View::SimNoDispenser;
            switch (dispenserData.mode) {
                case DispenserMode::SimulationDone:  return View::SimDone;
                case DispenserMode::Refused:         return View::SimRefused;
                default:                             return View::SimProgress;     // running, or starting
            }
    }
    return View::Menu;
}

// A failed burst on the dispenser starts the yellow LED's blinks. Compared
// only while the dispenser is heard; the first status after contact was lost,
// or ever made, is only taken as the starting point.
static void updateBurstFailBlink(uint32_t now)
{
    if (!dispenserEverSeen || !links.isAlive(NodeId::Dispenser)) {
        burstFailSynced = false;
        return;
    }
    if (!burstFailSynced) {
        lastBurstFailures = dispenserData.burstFailures;
        burstFailSynced   = true;
        return;
    }
    if (dispenserData.burstFailures != lastBurstFailures) {
        lastBurstFailures = dispenserData.burstFailures;
        burstBlinking     = true;
        burstBlinkStartMs = now;
    }
}

// Serviced every loop iteration, so the buzzer can never be left stuck on by
// a packet that stopped arriving mid-beep.
static void updateOutputs(uint32_t now)
{
    // Faults beep over every screen. The clog alarm did too, over screens that
    // don't show it, until CLOG_ALARM_BUZZER turned that off; the choice screen
    // is quiet either way.
    bool buzzing = (faultCode != FaultCode::None) || (CLOG_ALARM_BUZZER && currentView() == View::ClogAlert);
    digitalWrite(BUZZER_PIN, (buzzing && (now % 500 < 250)) ? HIGH : LOW);

    // Yellow: the tramline relay, as the seeder last reported it - turned the
    // other way BURST_FAIL_BLINKS times, BURST_FAIL_BLINK_MS each, after a
    // failed burst, so the blinks show whether the relay holds it lit or not.
    updateBurstFailBlink(now);
    bool yellow = seederEverSeen && seederData.tramlineRelayOn;
    if (burstBlinking) {
        uint32_t t = now - burstBlinkStartMs;
        if (t >= 2UL * BURST_FAIL_BLINKS * BURST_FAIL_BLINK_MS) {
            burstBlinking = false;
        } else if ((t / BURST_FAIL_BLINK_MS) % 2 == 0) {
            yellow = !yellow;
        }
    }
    digitalWrite(YELLOW_LED_PIN, yellow ? HIGH : LOW);

    if (anyLinkDown()) {
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, (now % 1000 < 500) ? HIGH : LOW);
    } else {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(BLUE_LED_PIN, LOW);
    }
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

static void drawSelectableLine(int16_t y, const char *text, bool selected)
{
    if (selected) {
        oled.fillRect(0, y - 2, SCREEN_WIDTH, 20, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setCursor(2, y);
    oled.print(text);
}

// Menu rows are 16 px apart, tops at y = 0, 16, 32, 48: the classic font's
// glyphs use 7 of their 8 rows, 14 px at size 2, which leaves 1 px above and
// below, and none of the four words has a descender. drawSelectableLine()
// keeps its 20 px rows for the confirm screens.
static void drawMenuRow(int16_t rowTop, const char *text, bool selected)
{
    if (selected) {
        oled.fillRect(0, rowTop, SCREEN_WIDTH, 16, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setTextSize(2);
    oled.setCursor(2, rowTop + 1);
    oled.print(text);
}

static void drawMenu()
{
    for (uint8_t row = 0; row < MENU_VISIBLE_ROWS; row++) {
        uint8_t item = menuTop + row;
        if (item >= (uint8_t)MenuItem::Count) break;
        drawMenuRow(row * 16, MENU_LABELS[item], menuIndex == item);
    }
}

// Screen 16. Two different things have to be visible at once: the cursor (the
// inverted row, as everywhere else) and which setting is actually in use (the
// square on the right). One button cannot show both by highlighting alone.
// "Kalibracja" is 10 characters, exactly the full width at text size 2, which
// is why the marker sits on the right instead of in front of the text.
static void drawSeedRow(int16_t rowTop, const char *text, bool selected, bool active)
{
    drawMenuRow(rowTop, text, selected);
    if (active) oled.fillRect(118, rowTop + 5, 6, 6, selected ? BLACK : WHITE);
}

static void drawSeeds()
{
    drawSeedRow(0,  "Male nas.",  seedCursor == 0, !seedLarge);
    drawSeedRow(16, "Duze nas.",  seedCursor == 1, seedLarge);
    drawSeedRow(32, "Kalibracja", seedCursor == 2, false);
    drawSeedRow(48, "Wroc",       seedCursor == 3, false);
}

// Screen 17, on the 20 px grid the other confirm screens use.
static void drawSeedCalibAsk()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Kalibracja kola");
    oled.setCursor(0, 16);
    oled.print(seedLarge ? "duze nasiona" : "male nasiona");

    oled.setTextSize(2);
    drawSelectableLine(26, "Anuluj", seedAskIndex == 0);
    drawSelectableLine(46, "OK",     seedAskIndex == 1);
}

// Screen 18: the pulse count, in the biggest digits that fit. How far that is
// in metres is deliberately not shown - it could only be worked out with the
// value being replaced, which on a first calibration is exactly the number that
// is wrong. The distance is the one thing the operator measures on the ground.
static void drawSeedCalibRun()
{
    uint32_t pulses = (seederData.wheelPulses >= calibWheelStartPulses)
                      ? (seederData.wheelPulses - calibWheelStartPulses) : 0;

    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("Przejedz ");
    oled.print(WHEEL_CALIB_DISTANCE_M);
    oled.print(" m");

    oled.setTextSize(2);
    oled.setCursor(4, 14);
    oled.print("Impulsy:");

    oled.setTextSize(3);
    oled.setCursor(4, 32);
    oled.print(pulses);

    oled.setTextSize(1);
    oled.setCursor(0, 56);
    oled.print("Dlugi klik = koniec");
}

// Screen 19: the measured value against the one it would replace, or why there
// is no value. A failed run offers nothing but Anuluj - it must never look like
// something worth saving.
static void drawSeedCalibResult()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);

    if (calibWheelError != WheelCalibError::None) {
        oled.setCursor(0, 0);
        oled.print("Kalibracja kola");
        oled.setCursor(0, 12);
        switch (calibWheelError) {
            case WheelCalibError::NoSeeder:    oled.print("Brak siewnika");       break;
            case WheelCalibError::SeederReset: oled.print("Reset siewnika");      break;
            case WheelCalibError::TooFew:      oled.print("Za malo impulsow");    break;
            default:                           oled.print("Wynik poza zakresem"); break;
        }
        oled.setTextSize(2);
        drawSelectableLine(36, "Anuluj", true);
        return;
    }

    oled.setCursor(0, 0);
    oled.print("Wynik: ");
    oled.print(calibWheelPulses);
    oled.print(" imp");
    oled.setCursor(0, 8);
    oled.print("1 imp = ");
    oled.print(calibWheelResultMm);
    oled.print(" mm");
    oled.setCursor(0, 16);
    oled.print("bylo ");
    oled.print(activeWheelMmPerPulse());
    oled.print(" mm");

    oled.setTextSize(2);
    drawSelectableLine(26, "Anuluj", seedResultIndex == 0);
    drawSelectableLine(46, "ZAPISZ", seedResultIndex == 1);
}

// Sets the highlight and leaves the cursor where the caller's text goes: the
// two rows on screen 20 are 12 px, like the editors' action fields, because the
// four value lines above them need the rest of the panel.
static void drawSettingsRow(int16_t rowTop, bool selected)
{
    if (selected) {
        oled.fillRect(0, rowTop, SCREEN_WIDTH, 12, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setCursor(2, rowTop + 2);
}

// Screen 20: everything the board keeps, and the row that sends it over USB.
// The baud rate is on the row itself, so the operator has it in front of them
// when they open the serial monitor on the laptop.
static void drawSettings()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);

    // Worst case each of these lines is exactly 21 characters, which is the
    // full width at text size 1. The calibration shown is the one the metering
    // in use goes by: the angle factor in burst metering.
    oled.setCursor(0, 0);
    oled.print("Dawka:");
    oled.print(doseKgPerHa);
    if (DISPENSER_BURST_MODE) {
        oled.print(" Kat:");
        oled.print(angleFactor);
    } else {
        oled.print(" Kalib:");
        oled.print(gramsPer100Rev);
    }

    oled.setCursor(0, 10);
    oled.print("Doz:");
    oled.print(dispenserEnabled ? "WL." : "WYL.");
    oled.print(" Sciezki:");
    oled.print(tramlinesEnabled ? "WL." : "WYL.");

    oled.setCursor(0, 20);
    oled.print("Nasiona:");
    oled.print(seedLarge ? "DUZE" : "MALE");

    oled.setCursor(0, 30);
    oled.print("Kolo M:");
    oled.print(wheelMmSmall);
    oled.print(" D:");
    oled.print(wheelMmLarge);
    oled.print("mm");

    bool sent = (settingsSentMs != 0) && (millis() - settingsSentMs < 2000);

    drawSettingsRow(40, settingsCursor == 0);
    if (sent) {
        oled.print("Wyslano!");
    } else {
        oled.print("Wyslij USB ");
        oled.print(SERIAL_BAUD);
    }

    drawSettingsRow(52, settingsCursor == 1);
    oled.print("Wroc");
}

// Every full-screen fault is drawn the same way: what has gone wrong on two
// large lines, on the grid the clog alarm uses, and the way out underneath.
// They are not acknowledged like the clog alarm - they clear themselves when
// the machine does - so the only thing to say is how to leave the screen.
// Two lines because 10 characters is the width at text size 2.
static void drawFaultLines(const char *what, const char *state)
{
    oled.setTextColor(WHITE);
    oled.setTextSize(2);
    oled.setCursor(0, 4);
    oled.print(what);
    oled.setCursor(0, 22);
    oled.print(state);

    oled.setTextSize(1);
    oled.setCursor(0, 54);
    oled.print("Dlugi klik = menu");
}

static void drawFaultScreen()
{
    switch (faultCode) {
        case FaultCode::TurbineOff:         drawFaultLines("DMUCHAWA", "STOI");      break;
        case FaultCode::DispenserOverSpeed: drawFaultLines("DOZOWNIK", "ZA SZYBKO"); break;
        // FaultCode::LinkLost deliberately draws nothing - a lost link is
        // shown as a small letter in the corner of the normal work screen
        // (and on the LEDs), never as a full-screen takeover. It only sounds
        // the buzzer, and only after LINK_BUZZER_DELAY_MS.
        default:
            break;
    }
}

// Everything except LinkLost replaces the work screen entirely.
static bool faultTakesOverScreen()
{
    return faultCode != FaultCode::None && faultCode != FaultCode::LinkLost;
}

// The view dispatcher already routed a fault to View::WorkFault, so this
// draws only the normal work screen.
static void drawWork()
{
    oled.setTextColor(WHITE);

    // The two numbers that come from the seeder are only worth reading while it
    // is still talking. With it unheard they are whatever arrived last, so the
    // screen says so instead of showing something that looks live.
    //
    // Display only: the stored telemetry is left exactly as it was, so a short
    // gap - which is the usual kind - changes nothing about the alarms, the
    // relay or what the dispenser is doing with the last speed it had.
    bool seederStale = seederLinkDown();

    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print("RPM ");
    if (seederStale) {
        oled.print("???");
    } else {
        oled.print(seederData.turbineRPM);
    }

    oled.setTextSize(1);
    oled.setCursor(0, 20);
    if (seederStale) {
        oled.print("???km/h");      // as wide as "6.4km/h", so nothing else moves
    } else {
        // mm/s -> km/h with one decimal: mm/s * 3.6 / 1000
        uint32_t kmhTenths = ((uint32_t)seederData.groundSpeedMmS * 36UL) / 1000UL;
        oled.print(kmhTenths / 10);
        oled.print('.');
        oled.print(kmhTenths % 10);
        oled.print("km/h");
    }

    // Which seed-size gear the dose is being metered for. One letter, because
    // the line is full at two digits of speed - but it has to be somewhere: the
    // wrong setting silently changes the rate by about a fifth.
    oled.setCursor(54, 20);
    oled.print(seedLarge ? 'D' : 'M');

    // The dose, and only when the dispenser is actually set to apply one: a
    // number here means it is on, and nothing here means it is off. Worst case
    // "kg/ha: 999" ends on the last pixel column.
    bool dosing = dispenserEnabled && doseKgPerHa > 0;
    if (dosing) {
        oled.setCursor(66, 20);
        oled.print("kg/ha: ");
        oled.print(doseKgPerHa);
    }

    oled.setCursor(0, 34);
    if (tramlinesEnabled) {
        oled.print("Przejazd:");

        // Same size as the rest of the screen, on the same row as the label.
        // It used to be drawn at text size 4 from (86, 30), which reaches down
        // to y = 57 - into the bottom row, where it shared pixels with the
        // animation marker (x 93-102, y 51-56) whenever tramlines were on and
        // the auger was turning. The screen mockups in CLAUDE.md could not show
        // that, because they draw every size at one character per cell.
        oled.setCursor(96, 34);
        oled.print(tramlineNumber + 1);
    } else {
        // The pass number would be meaningless: the relay can never come on.
        oled.print("Sciezki: WYL.");
    }

    // The bottom row says one of two things, and an empty row means all is
    // well and the dispenser is off. A board that has gone missing comes first:
    // it matters more than the rate, and with the seeder or the dispenser gone
    // the rate under it would be stale anyway.
    oled.setTextSize(1);
    oled.setCursor(0, 50);

    if (anyLinkDown()) {
        oled.print("BRAK:");
        if (seederLinkDown())     oled.print(" S");
        if (dispenserLinkDown())  oled.print(" D");
        if (crossLinkDown())      oled.print(" S-D");
    } else if (dosing && dispenserEverSeen) {
        oled.print("Doz:");
        oled.print(dispenserData.measuredShaftRPM);
        oled.print(" RPM");

        // Four frames, half a second each, and only while the auger is really
        // being driven: a machine standing still, or a dispenser with nothing
        // to do, leaves this blank rather than pretending to work.
        if (dispenserData.motorRunning) {
            static const char *const FRAMES[4] = {"[*   ]", "[ *  ]", "[  * ]", "[   *]"};
            oled.setCursor(92, 50);
            oled.print(FRAMES[(millis() / 500) % 4]);
        }
    }
}

// One action field on an editor's bottom row, label at textX.
static void drawEditField(int16_t x, int16_t width, int16_t textX, const char *label, bool selected)
{
    if (selected) {
        oled.fillRect(x, 50, width, 12, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setCursor(textX, 52);
    oled.print(label);
}

// The line under the digits of the two burst calibration screens, at text size
// 1: what the run should weigh - 20 pulses' worth of ground at the dose and
// seed size in use - and, on the mass screen, the angle factor now and what
// ZAPISZ would make it. The two halves there are drawn apart, so the widest
// numbers (a 5-digit mass) still leave a gap: "Ocz.39960g" ends at x = 59 and
// "Kat 999>999" starts at x = 62.
static void drawCalibInfo(int16_t y)
{
    uint32_t expected = calibExpectedGrams();
    char     text[24];

    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, y);

    if (screen == Screen::EditAngle) {
        snprintf(text, sizeof(text), "%u porcji = %lu g", (unsigned)CALIBRATION_PULSES, (unsigned long)expected);
        oled.print(text);
        return;
    }

    snprintf(text, sizeof(text), "Ocz.%lug", (unsigned long)expected);
    oled.print(text);

    uint16_t after = angleFactorFromWeighing(angleFactor, expected, digitsToValue());
    snprintf(text, sizeof(text), "Kat %u>%u", (unsigned)angleFactor, (unsigned)after);
    oled.setCursor(SCREEN_WIDTH - 6 * (int16_t)strlen(text), y);
    oled.print(text);
}

static void drawEdit()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    switch (screen) {
        case Screen::EditDose:  oled.print("DAWKA kg/ha"); break;
        case Screen::EditAngle: oled.print("WSP. KATA");   break;
        case Screen::CalibMass: oled.print("MASA g");      break;
        default:
            // Kept short on purpose: the built-in font is 6 px per character, so
            // a longer title runs into the mode label drawn at x = 92.
            oled.print("KALIBR. g/100");
            break;
    }

    // Textual mode indicator: which of the two press meanings is currently
    // active must never be ambiguous.
    oled.setCursor(92, 0);
    oled.print(editingDigit ? "ZMIEN" : "WYBOR");

    // The burst calibration screens have a line of their own under the digits
    // (drawCalibInfo), so their digits sit 3 px higher to make room for it.
    bool          infoLine  = (screen == Screen::EditAngle || screen == Screen::CalibMass);
    const int16_t top       = infoLine ? 11 : 14;   // of the highlight; the digit is 3 px lower
    const int16_t charWidth = 18;   // text size 3
    int16_t totalWidth = digitCount * charWidth;
    int16_t x0 = (SCREEN_WIDTH - totalWidth) / 2;

    oled.setTextSize(3);
    for (uint8_t i = 0; i < digitCount; i++) {
        int16_t x = x0 + i * charWidth;
        bool selected = (cursor == i);
        if (selected) {
            oled.fillRect(x - 1, top, charWidth, 28, WHITE);
            oled.setTextColor(BLACK);
        } else {
            oled.setTextColor(WHITE);
        }
        oled.setCursor(x, top + 3);
        oled.print(digits[i]);
    }

    oled.setTextSize(1);
    if (infoLine) drawCalibInfo(40);

    // The action fields sit side by side on the bottom row: three on the angle
    // factor's screen, two everywhere else.
    if (screen == Screen::EditAngle) {
        drawEditField(0,  40, 8,  "TEST",   cursor == digitCount);
        drawEditField(43, 40, 51, "MASA",   cursor == digitCount + 1);
        drawEditField(86, 42, 89, "ZAPISZ", cursor == digitCount + 2);
        return;
    }

    const char *extraLabel;
    switch (screen) {
        case Screen::EditDose:  extraLabel = dispenserEnabled ? "WL." : "WYL."; break;
        case Screen::CalibMass: extraLabel = "WROC";                            break;
        default:                extraLabel = "TEST";                            break;
    }
    drawEditField(2,  52, 6,  extraLabel, cursor == digitCount);
    drawEditField(62, 62, 70, "ZAPISZ",   cursor == digitCount + 1);
}

// Screens 15 and 21: a title, the switch (cursor 0) and ZAPISZ (cursor 1). The
// switch takes effect the moment it is flipped, so ZAPISZ only leaves the
// screen. note, if any, goes bottom left: at most 9 characters, so it ends
// before the ZAPISZ field.
static void drawSwitchScreen(const char *title, bool on, uint8_t switchCursor, const char *note)
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(title);

    const char *value = on ? "WL." : "WYL.";
    const int16_t charWidth = 18;   // text size 3
    int16_t length = (int16_t)strlen(value);
    int16_t x = (SCREEN_WIDTH - charWidth * length) / 2;

    if (switchCursor == 0) {
        oled.fillRect(x - 3, 14, charWidth * length + 6, 28, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setTextSize(3);
    oled.setCursor(x, 17);
    oled.print(value);

    oled.setTextSize(1);
    if (note != nullptr) {
        oled.setTextColor(WHITE);
        oled.setCursor(0, 52);
        oled.print(note);
    }

    // The bottom-right field sits exactly where the editors draw ZAPISZ.
    if (switchCursor == 1) {
        oled.fillRect(62, 50, 62, 12, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setCursor(70, 52);
    oled.print("ZAPISZ");
}

// Screen 15: the tramline switch, stored the moment it is flipped.
static void drawTramlines()
{
    drawSwitchScreen("SCIEZKI", tramlinesEnabled, tramlineCursor, nullptr);
}

// Screen 21: the blower alarm switch. Off, it says for how long - it is back on
// at the next power-up.
static void drawBlower()
{
    drawSwitchScreen("ALARM DMUCHAWY", blowerAlarmEnabled, blowerCursor,
                     blowerAlarmEnabled ? nullptr : "do resetu");
}

static void drawCalibConfirm()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Start kalibracji?");
    oled.setCursor(0, 16);
    if (DISPENSER_BURST_MODE) {
        // What the run is and what it should weigh.
        oled.print(CALIBRATION_PULSES);
        oled.print(" porcji = ");
        oled.print(calibExpectedGrams());
        oled.print(" g");
    } else {
        oled.print(CALIBRATION_REVOLUTIONS);
        oled.print(" obrotow");
    }

    // The 20 px grid of drawSelectableLine(). At y = 50 the highlight bar and
    // the bottom two pixel rows of the glyphs would fall off the 64 px panel.
    oled.setTextSize(2);
    drawSelectableLine(26, "Anuluj", calibConfirmIndex == 0);
    drawSelectableLine(46, "START",  calibConfirmIndex == 1);
}

// Shared by CalibProgress and Unclogging: a title, a progress bar and the
// percentage underneath.
static void drawProgress(const char *title, uint8_t percent)
{
    if (percent > 100) percent = 100;

    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print(title);

    // Progress bar
    oled.drawRect(2, 20, 124, 16, WHITE);
    uint16_t fill = (uint16_t)((120UL * percent) / 100UL);
    if (fill > 0) oled.fillRect(4, 22, fill, 12, WHITE);

    // Percentage underneath, centred
    oled.setTextSize(2);
    int16_t x = (percent == 100) ? 40 : (percent >= 10 ? 46 : 52);
    oled.setCursor(x, 42);
    oled.print(percent);
    oled.print('%');
}

static void drawCalibProgress()
{
    if (DISPENSER_BURST_MODE) {
        // The mass the bursts should add up to stays in sight while they run.
        // At most 21 characters, the full width: "Kalibracja... 39960 g".
        char title[24];
        snprintf(title, sizeof(title), "Kalibracja... %lu g", (unsigned long)calibExpectedGrams());
        drawProgress(title, dispenserData.progressPercent);
    } else {
        drawProgress("Kalibracja...", dispenserData.progressPercent);
    }
}

// Screens 10 and 26: the run the title names cannot reach the dispenser.
static void drawNoDispenser(const char *title)
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print(title);
    oled.setTextSize(2);
    oled.setCursor(0, 24);
    oled.print("BRAK");
    oled.setCursor(0, 44);
    oled.print("DOZOWNIKA");
}

// Screens 9 and 27: the run the title names was refused or stopped, because
// the seeder reports the wheel turning.
static void drawRefused(const char *title)
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print(title);
    oled.setTextSize(2);
    oled.setCursor(0, 22);
    oled.print("MASZYNA");
    oled.setCursor(0, 42);
    oled.print("W RUCHU");
}

static void drawCalibDone()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(2);
    oled.setCursor(0, 6);
    oled.print("GOTOWE");
    oled.setTextSize(1);
    if (DISPENSER_BURST_MODE) {
        // Any press opens the mass screen (5b).
        oled.setCursor(0, 30);
        oled.print("Ocz. masa: ");
        oled.print(calibExpectedGrams());
        oled.print(" g");
        oled.setCursor(0, 42);
        oled.print("Zwaz nawoz.");
        oled.setCursor(0, 54);
        oled.print("Nacisnij: wpisz mase");
        return;
    }
    oled.setCursor(0, 30);
    oled.print("Zwaz nawoz i wpisz");
    oled.setCursor(0, 42);
    oled.print("wynik w gramach.");
    oled.setCursor(0, 54);
    oled.print("Nacisnij aby wrocic");
}

static void drawCalibInterrupted()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Kalibracja");
    oled.setTextSize(2);
    oled.setCursor(0, 22);
    oled.print("PRZERWANA");
    oled.setTextSize(1);
    oled.setCursor(0, 54);
    oled.print("Nacisnij aby wrocic");
}

static void drawClogAlert()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(2);
    oled.setCursor(0, 4);
    oled.print("ZATKANIE!");
    oled.setCursor(0, 22);
    oled.print("DOZOWNIKA");
    drawSelectableLine(46, "OK", true);
}

static void drawClogChoice()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Zatkanie dozownika");

    // Same 20 px grid the calibration confirm screen uses (see drawCalibConfirm).
    oled.setTextSize(2);
    drawSelectableLine(26, "Anuluj", clogChoiceIndex == 0);
    drawSelectableLine(46, "Odetkaj", clogChoiceIndex == 1);
}

static void drawUnclogging()
{
    drawProgress("Odtykanie...", dispenserData.progressPercent);
}

// The simulation's speed in tenths of km/h, rounded: 1944 mm/s is 7.0.
static constexpr uint32_t SIMULATION_KMH_TENTHS = ((uint32_t)SIMULATION_SPEED_MM_S * 36UL + 500UL) / 1000UL;

static void printSimSpeed()
{
    oled.print(SIMULATION_KMH_TENTHS / 10);
    oled.print('.');
    oled.print(SIMULATION_KMH_TENTHS % 10);
    oled.print(" km/h");
}

// Screen 22: the tests, a row each at text size 1 - the rows of screen 20 - so
// longer names fit and more can follow; then Wroc.
static void drawTests()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("TESTY");

    for (uint8_t i = 0; i < TEST_ROW_COUNT; i++) {
        drawSettingsRow((int16_t)(14 + 12 * i), testsCursor == i);
        if (testRow(i) == TestRow::Simulation) {
            oled.print("Symulacja ");
            oled.print(SIMULATION_MINUTES);
        } else {
            oled.print("Wroc");
        }
    }
}

// Screen 23, on the grid of the other confirm screens.
static void drawSimConfirm()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Symulacja siewu");
    oled.setCursor(0, 16);
    printSimSpeed();
    oled.print(", ");
    oled.print(SIMULATION_MINUTES);
    oled.print(" min");

    oled.setTextSize(2);
    drawSelectableLine(26, "Anuluj", simConfirmIndex == 0);
    drawSelectableLine(46, "START",  simConfirmIndex == 1);
}

// mm:ss of a time in ms. Minutes stay under 100 (SIMULATION_MINUTES <= 99).
static void printMinSec(uint32_t ms)
{
    uint32_t s = ms / 1000;
    uint32_t m = s / 60;
    s %= 60;
    if (m < 10) oled.print('0');
    oled.print(m);
    oled.print(':');
    if (s < 10) oled.print('0');
    oled.print(s);
}

// The failed bursts as a share of all of them, to a tenth of a percent -
// "--" before there is a burst to go by.
static void printFailShare()
{
    if (simBursts == 0) {
        oled.print("--");
        return;
    }
    uint32_t tenths = ((uint32_t)simFailures * 1000UL + simBursts / 2) / simBursts;
    oled.print(tenths / 10);
    oled.print('.');
    oled.print(tenths % 10);
    oled.print('%');
}

// Screen 24: the simulation running - its clock, and the counts as the
// dispenser reports them. The widest line, "Nieudane: 450 (40.4%)", is 21
// characters: a run has room for no more failures than that, each taking at
// least BURST_FAIL_MS and a pulse.
static void drawSimProgress()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);

    oled.setCursor(0, 0);
    oled.print("Symulacja ");
    printSimSpeed();

    oled.setCursor(0, 11);
    oled.print("Czas ");
    printMinSec(simElapsedMs);
    oled.print(" / ");
    printMinSec(SIMULATION_DURATION_MS);

    oled.setCursor(0, 22);
    oled.print("Porcje: ");
    oled.print(simBursts);

    oled.setCursor(0, 33);
    oled.print("Nieudane: ");
    oled.print(simFailures);
    oled.print(" (");
    printFailShare();
    oled.print(')');

    oled.setCursor(0, 44);
    oled.print("Odtykania: ");
    oled.print(simUnclogs);

    oled.setCursor(0, 55);
    oled.print("Dlugi klik = stop");
}

// Screens 25 and 28: the result - the share of failed bursts, large, the counts
// under it - after the whole run, or after as much of it as there was.
static void drawSimResult(bool stopped)
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(stopped ? "Symulacja przerwana" : "Symulacja - koniec");

    oled.setCursor(0, 10);
    oled.print("Czas ");
    printMinSec(simElapsedMs);
    oled.print(", ");
    printSimSpeed();

    oled.setTextSize(2);
    oled.setCursor(0, 20);
    printFailShare();

    oled.setTextSize(1);
    oled.setCursor(0, 38);
    oled.print("nieudane: ");
    oled.print(simFailures);
    oled.print(" z ");
    oled.print(simBursts);

    oled.setCursor(0, 47);
    oled.print("odtykania: ");
    oled.print(simUnclogs);

    oled.setCursor(0, 56);
    oled.print("Nacisnij aby wrocic");
}

static void redraw()
{
    oled.clearDisplay();
    switch (currentView()) {
        case View::Menu:              drawMenu();             break;
        case View::Work:              drawWork();             break;
        case View::WorkFault:         drawFaultScreen();      break;
        case View::EditDose:
        case View::EditCalibration:
        case View::EditAngle:
        case View::CalibMass:         drawEdit();             break;
        case View::Tramlines:         drawTramlines();        break;
        case View::Blower:            drawBlower();           break;
        case View::Seeds:             drawSeeds();            break;
        case View::SeedCalibAsk:      drawSeedCalibAsk();     break;
        case View::SeedCalibRun:      drawSeedCalibRun();     break;
        case View::SeedCalibResult:   drawSeedCalibResult();  break;
        case View::Settings:          drawSettings();         break;
        case View::CalibConfirm:      drawCalibConfirm();     break;
        case View::CalibProgress:     drawCalibProgress();    break;
        case View::CalibDone:         drawCalibDone();        break;
        case View::CalibRefused:      drawRefused("Kalibracja"); break;
        case View::CalibNoDispenser:  drawNoDispenser("Kalibracja"); break;
        case View::CalibInterrupted:  drawCalibInterrupted(); break;
        case View::ClogAlert:         drawClogAlert();        break;
        case View::ClogChoice:        drawClogChoice();       break;
        case View::Unclogging:        drawUnclogging();       break;
        case View::Tests:             drawTests();            break;
        case View::SimConfirm:        drawSimConfirm();       break;
        case View::SimProgress:       drawSimProgress();      break;
        case View::SimDone:           drawSimResult(false);   break;
        case View::SimStopped:        drawSimResult(true);    break;
        case View::SimRefused:        drawRefused("Symulacja"); break;
        case View::SimNoDispenser:    drawNoDispenser("Symulacja"); break;
    }
    oled.display();
}

// ---------------------------------------------------------------------------

void setup()
{
#if BUTTON_DEBUG
    // Without a buffer every print waits for the 128-byte UART FIFO, so a few
    // debug lines would hold loop() up for tens of ms - and shift the screen
    // rule's timing, which is part of what the log is meant to show.
    Serial.setTxBufferSize(4096);
#endif
    Serial.begin(SERIAL_BAUD);
#if BUTTON_DEBUG
    Serial.printf("\nBUTTON_DEBUG on - times in ms since boot. Sample every %lu, a press after %lu of contact, "
                  "let go after %lu without, long at %lu, next press at least %lu after a let-go, "
                  "screen up %lu before a press counts\n",
                  (unsigned long)BUTTON_SAMPLE_MS, (unsigned long)BUTTON_PRESS_CONFIRM_MS,
                  (unsigned long)BUTTON_RELEASE_CONFIRM_MS, (unsigned long)BUTTON_LONG_PRESS_MS,
                  (unsigned long)BUTTON_MIN_GAP_MS, (unsigned long)BUTTON_SCREEN_SETTLE_MS);
#endif

    // The internal pull-up stays on alongside the 470 R one: should that
    // resistor ever come off, the button still works, just with the old
    // contact current.
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Before anything below can return early: without it the button is dead.
    // The rule's clock starts here, so a button already held at power-up - its
    // press begins at the first sample - falls inside BUTTON_SCREEN_SETTLE_MS
    // and is ignored, however long the boot took.
    shownSinceMs = millis();
    startButtonSampling();

    prefs.begin("tractor", false);
    // Stored in the ESP32's NVS partition in flash - no battery involved, so
    // these survive being unplugged indefinitely (see CLAUDE.md).
    // No saved value means a new board or an erased one: it starts from the
    // first-boot constants, which are also the restore path - see the Ustawienia
    // screen and docs/calibration-settings.txt.
    doseKgPerHa      = prefs.getUShort("dose",    DEFAULT_DOSE_KG_PER_HA);
    gramsPer100Rev   = prefs.getULong("calib",    DEFAULT_GRAMS_PER_100REV);
    dispenserEnabled = prefs.getBool("disp_on",   DEFAULT_DISPENSER_ENABLED);
    tramlinesEnabled = prefs.getBool("tram_on",   DEFAULT_TRAMLINES_ENABLED);
    wheelMmSmall     = prefs.getUShort("wheel_s", DEFAULT_WHEEL_MM_SMALL);
    wheelMmLarge     = prefs.getUShort("wheel_l", DEFAULT_WHEEL_MM_LARGE);
    seedLarge        = prefs.getBool("seed_l",    DEFAULT_SEED_LARGE);
    angleFactor      = prefs.getUShort("angle",   DEFAULT_ANGLE_FACTOR);
    // Three digits on screen 5, which would show and store only the last three
    // of anything larger.
    if (angleFactor > ANGLE_FACTOR_MAX) angleFactor = ANGLE_FACTOR_MAX;

    oled.begin(SH1106_SWITCHCAPVCC, OLED_I2C_ADDRESS);
    // oled.begin() calls Wire.begin(), which leaves the bus at the Arduino
    // default of 100 kHz. A full framebuffer push is ~1150 bytes, so at
    // 100 kHz every redraw blocks loop() for ~100 ms; 400 kHz brings that to
    // ~30 ms. The button no longer depends on it - a timer samples it - but
    // everything a press does, the screen, the LEDs and the buzzer's beat all
    // wait for loop(). The original AVR library set the same speed via TWBR;
    // that register write had to be removed for the ESP32 port, so it is done
    // here instead.
    Wire.setClock(400000);
    oled.display();
    oled.clearDisplay();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_ADDRESS, 6);
    // 0 means "whatever channel the radio is currently on". The radio is
    // pinned to ESPNOW_CHANNEL just above, so this resolves to the same thing
    // - but it can never disagree with it. Naming the channel explicitly here
    // risks the "peer channel is not equal to the home channel" error, which
    // makes every single send fail.
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
        return;
    }

    // Buzzer self-test, as before.
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("Tractor module ready");
}

void loop()
{
    uint32_t now = millis();

    // 1. Receive snapshot: everything below reads these consistent copies.
    snapshotReceived();

    // 2. Faults must come before the view is worked out, or the fault screen
    //    lags a loop behind and the button rule sees a stale view.
    updateFaults(now);

    // 3. Clog overlay and calibration bookkeeping.
    updateScreenBookkeeping(now);

    // 4. View change detection - a view appearing is what arms the button
    //    rule and marks the display dirty.
    View v = currentView();
    if (v != shownView || (v == View::WorkFault && faultCode != shownFault)) {
        shownView    = v;
        shownFault   = faultCode;   // two different full-screen faults are two different screens
        shownSinceMs = now;
        displayDirty = true;
        if (v == View::ClogChoice) clogChoiceIndex = 0;   // Anuluj preselected every time it appears
#if BUTTON_DEBUG
        Serial.printf("[btn %7lu] screen   -> %s\n", (unsigned long)now, viewDebugName(v));
#endif
    }

    // 5. Button, dispatched on the current view. At most one press per
    //    iteration: if it changes the view, step 4 sees that before the next
    //    queued press is judged.
#if BUTTON_DEBUG
    printButtonDebug();
#endif
    ButtonEvent event = takeButtonEvent();
    if (event != ButtonEvent::None) {
        handleView(v, event);
        // Confirm a counted long press audibly so the operator needn't watch
        // the screen. An ignored press (button rule) makes no beep.
        if (event == ButtonEvent::Long) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(30);
            digitalWrite(BUZZER_PIN, LOW);
        }
        displayDirty = true;
    }

    // 6. Outputs: buzzer, LEDs.
    updateOutputs(now);

    // 7. Command.
    sendCommand(now);

    // 8. Redraw when dirty, or periodically on the views showing live data.
    //    The menu and the editors change solely in response to a button, and
    //    repainting them 5x a second would just burn loop() time on an
    //    identical image.
    View liveView = currentView();
    bool live = (liveView == View::Work || liveView == View::WorkFault ||
                 liveView == View::CalibProgress || liveView == View::Unclogging ||
                 liveView == View::SeedCalibRun || liveView == View::SimProgress ||
                 liveView == View::Settings);   // so the "sent" message clears itself

    if (displayDirty || (live && (now - lastDisplayMs >= DISPLAY_INTERVAL_MS))) {
        lastDisplayMs = now;
        displayDirty  = false;
        redraw();
    }
}
