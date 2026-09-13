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
};

enum class MenuItem : uint8_t {
    Praca      = 0,
    Dawka      = 1,
    Kalibracja = 2,
    Count      = 3,
};

enum class FaultCode : uint8_t {
    None,
    TurbineOff,
    WomOff,
    WomWhileLifted,
    LinkLost,
    DispenserStalled,
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

static Preferences prefs;
static LinkTracker links;

static SeederTelemetry seederData  = {};
static DispenserStatus dispenserData = {};
static bool seederEverSeen    = false;
static bool dispenserEverSeen = false;

static FaultCode faultCode = FaultCode::None;
static uint32_t  linkDownSinceMs = 0;

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

// ---------------------------------------------------------------------------
// ESP-NOW
// ---------------------------------------------------------------------------

// Validate, copy, timestamp. Nothing else - this runs in the WiFi task, and
// touching the display or the buzzer from here would both stall packet
// reception and race with loop() over the I2C bus.
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (incomingData == nullptr || len < (int)sizeof(MessageHeader)) return;

    MessageHeader header;
    memcpy(&header, incomingData, sizeof(header));

    if (headerValid(incomingData, len, MsgType::SeederTelemetry, sizeof(SeederTelemetry))) {
        memcpy(&seederData, incomingData, sizeof(seederData));
        seederEverSeen = true;
        links.noteReceived(header.sender);
    } else if (headerValid(incomingData, len, MsgType::DispenserStatus, sizeof(DispenserStatus))) {
        memcpy(&dispenserData, incomingData, sizeof(dispenserData));
        dispenserEverSeen = true;
        links.noteReceived(header.sender);
    }
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
    command.tramlineRelayOn  = (TRAMLINE_ACTIVE_MASK >> tramlineNumber) & 1;
    command.dispenserEnabled = dispenserEnabled ? 1 : 0;
    command.calibrationRun   = calibrationRequested ? 1 : 0;
    command.doseKgPerHa      = doseKgPerHa;
    command.gramsPer100Rev   = gramsPer100Rev;

    broadcast(&command, sizeof(command), now);
}

// ---------------------------------------------------------------------------
// Button
// ---------------------------------------------------------------------------

// A long press fires the moment the hold time is reached, while the button is
// still down, so the operator gets feedback without having to watch the
// screen. The release that follows is swallowed.
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
    } else if (buttonStable && buttonHeld && !longAlreadyFired &&
               (now - buttonDownMs >= BUTTON_LONG_PRESS_MS)) {
        longAlreadyFired = true;
        return ButtonEvent::Long;
    } else if (!buttonStable && buttonHeld) {
        buttonHeld = false;
        if (!longAlreadyFired) return ButtonEvent::Short;
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
            default:
                break;
        }
    }
}

static void handleWork(ButtonEvent event)
{
    if (event == ButtonEvent::Short) {
        tramlineNumber = (tramlineNumber + 1) % TRAMLINE_RHYTHM;
    } else if (event == ButtonEvent::Long) {
        screen = Screen::Menu;
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
            calibrationRequested = true;
            screen = Screen::CalibRunning;
        } else {
            screen = Screen::EditCalibration;
        }
    }
}

static void handleCalibRunning(ButtonEvent event)
{
    bool finished = dispenserEverSeen &&
                    (dispenserData.calibrationState == CalibrationState::Done ||
                     dispenserData.calibrationState == CalibrationState::Refused);

    // Once it has finished (or was refused) any press dismisses it. While it
    // is still turning, only a long press aborts - so a stray short press
    // can't stop a run half way through and spoil the weighing.
    if (finished ? (event != ButtonEvent::None) : (event == ButtonEvent::Long)) {
        calibrationRequested = false;
        screen = Screen::EditCalibration;
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
    } else if (dispenserEverSeen && dispenserData.faultCode == DispenserFault::Stalled) {
        faultCode = FaultCode::DispenserStalled;
    } else if (dispenserEverSeen && dispenserData.faultCode == DispenserFault::OverSpeed) {
        faultCode = FaultCode::DispenserOverSpeed;
    } else if (linkDownSinceMs != 0 && (now - linkDownSinceMs) >= LINK_BUZZER_DELAY_MS) {
        faultCode = FaultCode::LinkLost;
    } else {
        faultCode = FaultCode::None;
    }
}

// Serviced every loop iteration, so the buzzer can never be left stuck on by
// a packet that stopped arriving mid-beep.
static void updateOutputs(uint32_t now)
{
    bool buzzing = (faultCode != FaultCode::None);
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

static void drawMenu()
{
    oled.setTextSize(2);
    drawSelectableLine(4,  "Praca",      menuIndex == 0);
    drawSelectableLine(24, "Dawka",      menuIndex == 1);
    drawSelectableLine(44, "Kalibracja", menuIndex == 2);
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
        case FaultCode::DispenserStalled:
            oled.setTextSize(2);
            oled.setCursor(4, 10);
            oled.print("DOZOWNIK");
            oled.setCursor(4, 32);
            oled.print("BLOKADA");
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

static void drawWork()
{
    if (faultTakesOverScreen()) {
        drawFaultScreen();
        return;
    }

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
    oled.print("Przejazd:");

    oled.setTextSize(4);
    oled.setCursor(86, 30);
    oled.print(tramlineNumber + 1);

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

static void drawCalibConfirm()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("Start kalibracji?");
    oled.setCursor(0, 16);
    oled.print(CALIBRATION_REVOLUTIONS);
    oled.print(" obrotow");

    // Same 20 px grid the main menu uses. At y = 50 the highlight bar and the
    // bottom two pixel rows of the glyphs would fall off the 64 px panel.
    oled.setTextSize(2);
    drawSelectableLine(26, "Anuluj", calibConfirmIndex == 0);
    drawSelectableLine(46, "START",  calibConfirmIndex == 1);
}

static void drawCalibRunning()
{
    oled.setTextColor(WHITE);
    oled.setTextSize(1);

    if (!links.isAlive(NodeId::Dispenser)) {
        oled.setCursor(0, 4);
        oled.print("Kalibracja");
        oled.setTextSize(2);
        oled.setCursor(0, 24);
        oled.print("BRAK");
        oled.setCursor(0, 44);
        oled.print("DOZOWNIKA");
        return;
    }

    if (dispenserData.calibrationState == CalibrationState::Refused) {
        oled.setCursor(0, 4);
        oled.print("Kalibracja");
        oled.setTextSize(2);
        oled.setCursor(0, 22);
        oled.print("MASZYNA");
        oled.setCursor(0, 42);
        oled.print("W RUCHU");
        return;
    }

    if (dispenserData.calibrationState == CalibrationState::Done) {
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
        return;
    }

    uint8_t percent = dispenserData.calibrationPercent;
    if (percent > 100) percent = 100;

    oled.setCursor(0, 4);
    oled.print("Kalibracja...");

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

static void redraw()
{
    oled.clearDisplay();
    switch (screen) {
        case Screen::Menu:            drawMenu();          break;
        case Screen::Work:            drawWork();          break;
        case Screen::EditDose:
        case Screen::EditCalibration: drawEdit();          break;
        case Screen::CalibConfirm:    drawCalibConfirm();  break;
        case Screen::CalibRunning:    drawCalibRunning();  break;
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

    ButtonEvent event = readButton(now);
    if (event != ButtonEvent::None) {
        switch (screen) {
            case Screen::Menu:            handleMenu(event);         break;
            case Screen::Work:            handleWork(event);         break;
            case Screen::EditDose:
            case Screen::EditCalibration: handleEdit(event);         break;
            case Screen::CalibConfirm:    handleCalibConfirm(event); break;
            case Screen::CalibRunning:    handleCalibRunning(event); break;
        }
        // Confirm a long press audibly so the operator needn't watch the screen.
        if (event == ButtonEvent::Long) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(30);
            digitalWrite(BUZZER_PIN, LOW);
        }
        displayDirty = true;
    }

    updateFaults(now);
    updateOutputs(now);
    sendCommand(now);

    // Only the screens showing live data need repainting on a timer. The menu
    // and the editors change solely in response to a button, and repainting
    // them 5x a second would just burn loop() time on an identical image.
    bool liveScreen = (screen == Screen::Work || screen == Screen::CalibRunning);

    if (displayDirty || (liveScreen && (now - lastDisplayMs >= DISPLAY_INTERVAL_MS))) {
        lastDisplayMs = now;
        displayDirty  = false;
        redraw();
    }
}
