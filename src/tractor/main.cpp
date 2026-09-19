#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Preferences.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#include "machine_settings.h"
#include "espnow_protocol.h"

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

static constexpr uint32_t DISPLAY_INTERVAL_MS = 200;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

enum class Screen : uint8_t {
    Menu,
    Work,
    EditDose,
    EditCalibration,
    CalibConfirm,     // "Start kalibracji?" -> Anuluj / START
    CalibRunning,     // progress bar while the dispenser turns
    Tramlines,        // the tramline on/off switch, screen 15
    Seeds,            // seed size and its wheel calibration, screen 16
    SeedCalibAsk,     // "Kalibracja kola?" -> Anuluj / OK, screen 17
    SeedCalibRun,     // driving the measured distance, screen 18
    SeedCalibResult,  // the measured value, or why there isn't one, screen 19
    Settings,         // everything stored, and the USB export, screen 20
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
};

enum class MenuItem : uint8_t {
    Praca      = 0,
    Dawka      = 1,
    Kalibracja = 2,
    Sciezki    = 3,
    Nasiona    = 4,
    Ustawienia = 5,
    Count      = 6,
};

// More items than rows, so the menu scrolls: four are drawn from menuTop, which
// follows the cursor.
static const char *const MENU_LABELS[] = {"Praca", "Dawka", "Kalibracja", "Sciezki", "Nasiona", "Ustawienia"};
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
// first. The cursor runs across the digits and then onto two trailing action
// fields, so the positions are:
//     0 .. digitCount-1   the digits
//     digitCount          the screen's extra action (WL./WYL. or TEST)
//     digitCount+1        ZAPISZ
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

static uint8_t tramlineNumber = 0;

// The machine is usually used without tramlines at all, so the switch turns
// the relay off for every pass until the operator turns them back on. Flipping
// it never touches tramlineNumber: switching off and on again mid-field must
// not lose the pass. Stored in NVS, written only when it is flipped.
static bool    tramlinesEnabled = false;
static uint8_t tramlineCursor   = 0;      // screen 15: 0 = the switch, 1 = ZAPISZ

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

// Calibration: interrupted means the dispenser sat in Normal after START
// instead of calibrating (its side dropped the run), so the screen must tell
// the operator instead of sitting at 0 % forever.
static bool     calibrationInterrupted = false;
static bool     calibStartWatchRunning = false;
static uint32_t calibStartWatchMs      = 0;

// View tracking - also the reference for the button rule: a press counts only
// if the view it started on was already showing BUTTON_SCREEN_SETTLE_MS
// before the press began.
static View      shownView     = View::Menu;
static FaultCode shownFault    = FaultCode::None;
static uint32_t  shownSinceMs  = 0;
static uint8_t   viewGeneration = 0;

static uint32_t lastSendMs    = 0;
static uint32_t lastDisplayMs = 0;
static bool     displayDirty  = true;

// Button debounce / press classification
static bool     buttonStable   = false;   // true = pressed
static bool     buttonFlicker  = false;
static uint32_t lastDebounceMs = 0;
static bool     buttonHeld     = false;
static uint32_t buttonDownMs   = 0;
static bool     longAlreadyFired = false;
static uint8_t  pressGeneration = 0;      // viewGeneration when the press began
static uint32_t pressStartMs    = 0;      // last raw change, i.e. when the finger actually pressed

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

    broadcast(&command, sizeof(command), now);
}

// ---------------------------------------------------------------------------
// Button
// ---------------------------------------------------------------------------

// A long press fires the moment the hold time is reached, while the button is
// still down, so the operator gets feedback without having to watch the
// screen. The release that follows is swallowed.
//
// Button rule: a press counts only if the same view was showing for the whole
// press. That covers both "the view changed while the button was held" and
// "the press started just before a new view appeared" - without it, a press
// that straddles a screen change would act on the screen it no longer sees.
static bool pressCounts()
{
    return pressGeneration == viewGeneration &&
           (int32_t)(pressStartMs - shownSinceMs) >= (int32_t)BUTTON_SCREEN_SETTLE_MS;
}

static ButtonEvent readButton(uint32_t now)
{
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);

    if (pressed != buttonFlicker) {
        buttonFlicker  = pressed;
        lastDebounceMs = now;
    }
    if (now - lastDebounceMs < BUTTON_DEBOUNCE_MS) return ButtonEvent::None;

    buttonStable = buttonFlicker;

    if (buttonStable && !buttonHeld) {
        buttonHeld       = true;
        buttonDownMs     = now;
        longAlreadyFired = false;
        // Where the finger actually pressed, i.e. the last raw change.
        pressGeneration  = viewGeneration;
        pressStartMs     = lastDebounceMs;
    } else if (buttonStable && buttonHeld && !longAlreadyFired &&
               (now - buttonDownMs >= BUTTON_LONG_PRESS_MS)) {
        longAlreadyFired = true;   // even an ignored long press swallows the release
        if (!pressCounts()) return ButtonEvent::None;
        return ButtonEvent::Long;
    } else if (!buttonStable && buttonHeld) {
        buttonHeld = false;
        if (!longAlreadyFired) {
            if (!pressCounts()) return ButtonEvent::None;
            return ButtonEvent::Short;
        }
    }

    return ButtonEvent::None;
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

static void saveEditedValue()
{
    uint32_t value = digitsToValue();

    if (screen == Screen::EditDose) {
        if (value > MAX_DOSE_KG_PER_HA) value = MAX_DOSE_KG_PER_HA;
        doseKgPerHa = (uint16_t)value;
        prefs.putUShort("dose", doseKgPerHa);
        prefs.putBool("disp_on", dispenserEnabled);
    } else {
        if (value > MAX_GRAMS_PER_100REV) value = MAX_GRAMS_PER_100REV;
        gramsPer100Rev = value;
        prefs.putULong("calib", gramsPer100Rev);
    }
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
                screen = Screen::EditCalibration;
                loadDigits(gramsPer100Rev, 5);
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

static void handleEdit(ButtonEvent event)
{
    const uint8_t extraField = digitCount;        // WL./WYL. or TEST
    const uint8_t saveField  = digitCount + 1;    // ZAPISZ

    if (event == ButtonEvent::Short) {
        if (editingDigit) {
            digits[cursor] = (digits[cursor] + 1) % 10;
        } else {
            cursor = (cursor + 1) % (digitCount + 2);   // wraps through both actions
        }
        return;
    }

    if (event != ButtonEvent::Long) return;

    if (editingDigit) {
        editingDigit = false;                     // commit this digit
    } else if (cursor == saveField) {
        saveEditedValue();
        screen = Screen::Menu;
    } else if (cursor == extraField) {
        if (screen == Screen::EditDose) {
            dispenserEnabled = !dispenserEnabled;
        } else {
            calibConfirmIndex = 0;                // default to Anuluj
            screen = Screen::CalibConfirm;
        }
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
            screen = Screen::EditCalibration;
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
    screen = Screen::EditCalibration;   // cursor is still on TEST
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
        case View::EditCalibration:   handleEdit(event);          break;
        case View::Tramlines:         handleTramlines(event);     break;
        case View::Seeds:             handleSeeds(event);         break;
        case View::SeedCalibAsk:      handleSeedCalibAsk(event);  break;
        case View::SeedCalibRun:      handleSeedCalibRun(event);  break;
        case View::SeedCalibResult:   handleSeedCalibResult(event); break;
        case View::Settings:          handleSettings(event);      break;
        case View::CalibConfirm:      handleCalibConfirm(event);  break;
        case View::CalibProgress:
        case View::CalibNoDispenser:  handleCalibTurning(event);  break;
        case View::CalibDone:
        case View::CalibRefused:
        case View::CalibInterrupted:  handleCalibResult(event);   break;
        case View::ClogAlert:         handleClogAlert(event);     break;
        case View::ClogChoice:        handleClogChoice(event);    break;
        case View::Unclogging:        break;   // nothing responds while the sequence runs
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
    // speed would beep every time.
    bool fanStopped = links.isAlive(NodeId::Seeder) && seederData.wheelTurning &&
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
        screen = Screen::EditCalibration;   // cursor is still on TEST
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
}

// What the display currently shows. The clog overlay covers every screen
// while the dispenser reports a clog (and only while it is alive - see the
// trap about stale data).
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
        case Screen::Tramlines:       return View::Tramlines;
        case Screen::Seeds:           return View::Seeds;
        case Screen::SeedCalibAsk:    return View::SeedCalibAsk;
        case Screen::SeedCalibRun:    return View::SeedCalibRun;
        case Screen::SeedCalibResult: return View::SeedCalibResult;
        case Screen::Settings:        return View::Settings;
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
    }
    return View::Menu;
}

// Serviced every loop iteration, so the buzzer can never be left stuck on by
// a packet that stopped arriving mid-beep.
static void updateOutputs(uint32_t now)
{
    // The clog alarm beeps over every screen, including ones that don't show
    // it; the choice screen is quiet on purpose.
    bool buzzing = (faultCode != FaultCode::None) || (currentView() == View::ClogAlert);
    digitalWrite(BUZZER_PIN, (buzzing && (now % 500 < 250)) ? HIGH : LOW);

    digitalWrite(YELLOW_LED_PIN, (seederEverSeen && seederData.tramlineRelayOn) ? HIGH : LOW);

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
    // full width at text size 1.
    oled.setCursor(0, 0);
    oled.print("Dawka:");
    oled.print(doseKgPerHa);
    oled.print(" Kalib:");
    oled.print(gramsPer100Rev);

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

        oled.setTextSize(4);
        oled.setCursor(86, 30);
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

static void drawEdit()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    if (screen == Screen::EditDose) {
        oled.print("DAWKA kg/ha");
    } else {
        // Kept short on purpose: the built-in font is 6 px per character, so
        // a longer title runs into the mode label drawn at x = 92.
        oled.print("KALIBR. g/100");
    }

    // Textual mode indicator: which of the two press meanings is currently
    // active must never be ambiguous.
    oled.setCursor(92, 0);
    oled.print(editingDigit ? "ZMIEN" : "WYBOR");

    const int16_t charWidth = 18;   // text size 3
    int16_t totalWidth = digitCount * charWidth;
    int16_t x0 = (SCREEN_WIDTH - totalWidth) / 2;

    oled.setTextSize(3);
    for (uint8_t i = 0; i < digitCount; i++) {
        int16_t x = x0 + i * charWidth;
        bool selected = (cursor == i);
        if (selected) {
            oled.fillRect(x - 1, 14, charWidth, 28, WHITE);
            oled.setTextColor(BLACK);
        } else {
            oled.setTextColor(WHITE);
        }
        oled.setCursor(x, 17);
        oled.print(digits[i]);
    }

    // The two action fields sit side by side on the bottom row.
    oled.setTextSize(1);

    const char *extraLabel;
    if (screen == Screen::EditDose) {
        extraLabel = dispenserEnabled ? "WL." : "WYL.";
    } else {
        extraLabel = "TEST";
    }

    bool extraSelected = (cursor == digitCount);
    if (extraSelected) {
        oled.fillRect(2, 50, 52, 12, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setCursor(6, 52);
    oled.print(extraLabel);

    bool saveSelected = (cursor == digitCount + 1);
    if (saveSelected) {
        oled.fillRect(62, 50, 62, 12, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setCursor(70, 52);
    oled.print("ZAPISZ");
}

// Screen 15: the tramline switch. The value is stored the moment it is
// flipped, so ZAPISZ only leaves the screen.
static void drawTramlines()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("SCIEZKI");

    const char *value = tramlinesEnabled ? "WL." : "WYL.";
    const int16_t charWidth = 18;   // text size 3
    int16_t length = (int16_t)strlen(value);
    int16_t x = (SCREEN_WIDTH - charWidth * length) / 2;

    if (tramlineCursor == 0) {
        oled.fillRect(x - 3, 14, charWidth * length + 6, 28, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setTextSize(3);
    oled.setCursor(x, 17);
    oled.print(value);

    // The bottom-right field sits exactly where the editors draw ZAPISZ.
    oled.setTextSize(1);
    if (tramlineCursor == 1) {
        oled.fillRect(62, 50, 62, 12, WHITE);
        oled.setTextColor(BLACK);
    } else {
        oled.setTextColor(WHITE);
    }
    oled.setCursor(70, 52);
    oled.print("ZAPISZ");
}

static void drawCalibConfirm()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Start kalibracji?");
    oled.setCursor(0, 16);
    oled.print(CALIBRATION_REVOLUTIONS);
    oled.print(" obrotow");

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
    drawProgress("Kalibracja...", dispenserData.progressPercent);
}

static void drawCalibNoDispenser()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Kalibracja");
    oled.setTextSize(2);
    oled.setCursor(0, 24);
    oled.print("BRAK");
    oled.setCursor(0, 44);
    oled.print("DOZOWNIKA");
}

static void drawCalibRefused()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Kalibracja");
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

static void redraw()
{
    oled.clearDisplay();
    switch (currentView()) {
        case View::Menu:              drawMenu();             break;
        case View::Work:              drawWork();             break;
        case View::WorkFault:         drawFaultScreen();      break;
        case View::EditDose:
        case View::EditCalibration:   drawEdit();             break;
        case View::Tramlines:         drawTramlines();        break;
        case View::Seeds:             drawSeeds();            break;
        case View::SeedCalibAsk:      drawSeedCalibAsk();     break;
        case View::SeedCalibRun:      drawSeedCalibRun();     break;
        case View::SeedCalibResult:   drawSeedCalibResult();  break;
        case View::Settings:          drawSettings();         break;
        case View::CalibConfirm:      drawCalibConfirm();     break;
        case View::CalibProgress:     drawCalibProgress();    break;
        case View::CalibDone:         drawCalibDone();        break;
        case View::CalibRefused:      drawCalibRefused();     break;
        case View::CalibNoDispenser:  drawCalibNoDispenser(); break;
        case View::CalibInterrupted:  drawCalibInterrupted(); break;
        case View::ClogAlert:         drawClogAlert();        break;
        case View::ClogChoice:        drawClogChoice();       break;
        case View::Unclogging:        drawUnclogging();       break;
    }
    oled.display();
}

// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(SERIAL_BAUD);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

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

    oled.begin(SH1106_SWITCHCAPVCC, OLED_I2C_ADDRESS);
    // oled.begin() calls Wire.begin(), which leaves the bus at the Arduino
    // default of 100 kHz. A full framebuffer push is ~1150 bytes, so at
    // 100 kHz every redraw blocks loop() for ~100 ms - long enough that the
    // button, which is only sampled once per loop, starts dropping presses.
    // 400 kHz brings that to ~26 ms. The original AVR library set the same
    // speed via TWBR; that register write had to be removed for the ESP32
    // port, so it is done here instead.
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
        viewGeneration++;
        displayDirty = true;
        if (v == View::ClogChoice) clogChoiceIndex = 0;   // Anuluj preselected every time it appears
    }

    // 5. Button, dispatched on the current view.
    ButtonEvent event = readButton(now);
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
                 liveView == View::SeedCalibRun ||
                 liveView == View::Settings);   // so the "sent" message clears itself

    if (displayDirty || (live && (now - lastDisplayMs >= DISPLAY_INTERVAL_MS))) {
        lastDisplayMs = now;
        displayDirty  = false;
        redraw();
    }
}
