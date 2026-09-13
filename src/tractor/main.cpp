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
};

enum class MenuItem : uint8_t {
    Praca      = 0,
    Dawka      = 1,
    Kalibracja = 2,
    Sciezki    = 3,
    Count      = 4,
};

enum class FaultCode : uint8_t {
    None,
    TurbineOff,
    WomOff,
    WomWhileLifted,
    LinkLost,
    DispenserOverSpeed,
};

enum class ButtonEvent : uint8_t { None, Short, Long };

static Screen   screen   = Screen::Menu;
static uint8_t  menuIndex = 0;

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
static uint32_t  linkDownSinceMs = 0;

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

    // Highest priority first.
    if (ENABLE_WOM_ALARM && seederData.wheelTurning && seederData.womRPM < WOM_RUNNING_MIN_RPM) {
        faultCode = FaultCode::WomOff;
    } else if (ENABLE_TURBINE_ALARM && seederData.wheelTurning && seederData.turbineRPM < TURBINE_RUNNING_MIN_RPM) {
        faultCode = FaultCode::TurbineOff;
    } else if (ENABLE_WOM_ALARM && !seederData.wheelTurning && seederData.womRPM >= WOM_RUNNING_MIN_RPM) {
        faultCode = FaultCode::WomWhileLifted;
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
    drawMenuRow(0,  "Praca",      menuIndex == 0);
    drawMenuRow(16, "Dawka",      menuIndex == 1);
    drawMenuRow(32, "Kalibracja", menuIndex == 2);
    drawMenuRow(48, "Sciezki",    menuIndex == 3);
}

static void drawFaultScreen()
{
    oled.setTextColor(WHITE);
    switch (faultCode) {
        case FaultCode::WomOff:
        case FaultCode::WomWhileLifted:
            oled.setTextSize(3);
            oled.setCursor(10, 15);
            oled.print("WOM");
            break;
        case FaultCode::TurbineOff:
            oled.setTextSize(2);
            oled.setCursor(10, 20);
            oled.print("Dmuchawa");
            break;
        case FaultCode::DispenserOverSpeed:
            oled.setTextSize(2);
            oled.setCursor(4, 10);
            oled.print("DOZOWNIK");
            oled.setCursor(4, 32);
            oled.print("ZA SZYBKO");
            break;
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

    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print("RPM ");
    oled.print(seederData.turbineRPM);

    // mm/s -> km/h with one decimal: mm/s * 3.6 / 1000
    uint32_t kmhTenths = ((uint32_t)seederData.groundSpeedMmS * 36UL) / 1000UL;
    oled.setTextSize(1);
    oled.setCursor(0, 20);
    oled.print(kmhTenths / 10);
    oled.print('.');
    oled.print(kmhTenths % 10);
    oled.print("km/h");

    oled.setCursor(66, 20);
    if (!dispenserEnabled || doseKgPerHa == 0) {
        oled.print("DOZ:WYL.");
    } else {
        oled.print("DOZ:");
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

    // Only shown when something is wrong - a clean screen means all good.
    if (anyLinkDown()) {
        oled.setTextSize(1);
        oled.setCursor(0, 50);
        oled.print("BRAK:");
        if (seederLinkDown())     oled.print(" S");
        if (dispenserLinkDown())  oled.print(" D");
        if (crossLinkDown())      oled.print(" S-D");
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
    Serial.begin(115200);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    prefs.begin("tractor", false);
    // Stored in the ESP32's NVS partition in flash - no battery involved, so
    // these survive being unplugged indefinitely (see CLAUDE.md).
    doseKgPerHa      = prefs.getUShort("dose",  DEFAULT_DOSE_KG_PER_HA);
    gramsPer100Rev   = prefs.getULong("calib",  DEFAULT_GRAMS_PER_100REV);
    dispenserEnabled = prefs.getBool("disp_on", false);
    tramlinesEnabled = prefs.getBool("tram_on", false);   // no saved value = off

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
                 liveView == View::CalibProgress || liveView == View::Unclogging);

    if (displayDirty || (live && (now - lastDisplayMs >= DISPLAY_INTERVAL_MS))) {
        lastDisplayMs = now;
        displayDirty  = false;
        redraw();
    }
}
