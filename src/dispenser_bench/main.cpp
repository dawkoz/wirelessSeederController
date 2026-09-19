#include <Arduino.h>
#include <esp_attr.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "machine_settings.h"
#include "espnow_protocol.h"
#include "../dispenser/dispenser_io.h"
#include "bench_report.h"
#include "bench_inject.h"

// ---------------------------------------------------------------------------
// Dispenser bench test. NEVER fit a module running this image: it runs the
// production decision logic and the production I/O, but its own setup(), its
// own menu and its own packet injection.
//
// The logic it drives is src/dispenser/dispenser_logic.h, unchanged - a bench
// that re-implements the logic would prove nothing about what gets flashed.
// The tractor and seeder are simulated at the receive boundary: packets are
// built with the real structs and fillHeader() and handed to
// dispenserHandlePacket(), the function the production receive callback calls.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Interrupted-test marker: a reset during a test leaves this behind, so the
// next boot can say which test died. It survives software, panic, watchdog and
// most brownout resets - not a full power loss.
// ---------------------------------------------------------------------------

static constexpr uint32_t MARKER_MAGIC = 0xB0BA0000u + PROTOCOL_VERSION;

struct RtcMarker {
    uint32_t magic;
    char     testId[8];
};

RTC_NOINIT_ATTR static RtcMarker rtcMarker;

static void markerSet(const char *id)
{
    rtcMarker.magic = MARKER_MAGIC;
    strncpy(rtcMarker.testId, id, 7);
    rtcMarker.testId[7] = '\0';
}

static void markerClear()
{
    rtcMarker.magic     = 0;
    rtcMarker.testId[0] = '\0';
}

// Marks the test in progress, and clears it however the test ends.
struct TestMarker {
    explicit TestMarker(const char *id) { markerSet(id); }
    ~TestMarker() { markerClear(); }
};

// Duty 0 and forward on every exit path, whatever the outcome.
struct MotorGuard {
    ~MotorGuard() { motorApply(DispenserOutputs{0, true}); }
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static DispenserLogic   logic;
static DispenserOutputs out = {0, true};
static Injector         injector;

static uint32_t heartbeatFailures = 0;
static uint32_t lastHeartbeatMs   = 0;
static bool     abortRequested    = false;
static bool     motorChecklistOk  = false;

static DispenserMode lastPrintedMode = DispenserMode::Normal;

// Watched during the hardware-in-the-loop runs.
static uint16_t gRunMaxDuty     = 0;
static bool     gSawZeroDuty    = false;
static bool     gSawBuzzingDuty = false;
static bool     gSawClogged     = false;

// ---------------------------------------------------------------------------
// Channel B: bench only. It counts its own edges and how many of them saw
// channel A high, which is the only way to tell the two directions apart -
// the encoder counts up whichever way the shaft turns.
// ---------------------------------------------------------------------------

static volatile uint32_t bEdgeTotal = 0;
static volatile uint32_t bHighTotal = 0;

void IRAM_ATTR encoderBPulseISR()
{
    bEdgeTotal++;
    if (digitalRead(ENCODER_A_PIN) == HIGH) bHighTotal++;
}

static uint32_t bEdges()     { return bEdgeTotal; }
static uint32_t bHighEdges() { return bHighTotal; }

struct Window {
    uint32_t ms     = 0;
    uint32_t aEdges = 0;
    uint32_t bEdges = 0;
    uint32_t bHigh  = 0;
};

// RPM from the encoder edges measured over a window - never from the logic's
// own measuredShaftRPM, which is what is being checked.
static double windowRPM(const Window &w)
{
    if (w.ms == 0) return 0.0;
    return (double)w.aEdges * 60000.0 / ((double)w.ms * (double)ENCODER_EDGES_PER_REV);
}

// 1 = channel A sat high at the channel B edges, 0 = low, -1 = no clear
// majority. Forward and reverse must give opposite answers.
static int windowDirection(const Window &w, double minAgree)
{
    if (w.bEdges == 0) return -1;

    double ratio = (double)w.bHigh / (double)w.bEdges;
    if (ratio >= minAgree)         return 1;
    if (ratio <= 1.0 - minAgree)   return 0;
    return -1;
}

// ---------------------------------------------------------------------------
// Radio: as in production, except that no receive callback is ever registered.
// A real tractor or seeder nearby must not be able to interfere with the bench
// (or be commanded by it). The heartbeat keeps the radio's current draw
// realistic while the motor runs, and every real board drops it at the magic
// byte check.
// ---------------------------------------------------------------------------

static void heartbeat(uint32_t now)
{
    if (now - lastHeartbeatMs < SEND_INTERVAL_MS) return;
    lastHeartbeatMs = now;

    const uint8_t beat[8] = {'B', 'T', 0, 0, 0, 0, 0, 0};
    if (esp_now_send(BROADCAST_ADDRESS, beat, sizeof(beat)) != ESP_OK) heartbeatFailures++;
}

static bool radioBegin()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) return false;

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_ADDRESS, 6);
    peerInfo.channel = 0;      // "whatever channel the radio is on" - see the production notes
    peerInfo.encrypt = false;

    return (esp_now_add_peer(&peerInfo) == ESP_OK);
}

// ---------------------------------------------------------------------------
// The pump every hardware-in-the-loop test runs on: never prints per control
// step (serial writes block, and that would disturb the timing being tested).
// ---------------------------------------------------------------------------

static void noteStep()
{
    if (logic.mode != lastPrintedMode) {
        Serial.printf("[%lu] mode %s -> %s\n", (unsigned long)millis(),
                      modeName(lastPrintedMode), modeName(logic.mode));
        lastPrintedMode = logic.mode;
    }

    if (out.motorPermille > gRunMaxDuty) gRunMaxDuty = out.motorPermille;
    if (out.motorPermille == 0) gSawZeroDuty = true;
    if (out.motorPermille > 0 && out.motorPermille < MOTOR_MIN_RUNNING_PERMILLE) gSawBuzzingDuty = true;
    if (logic.mode == DispenserMode::Clogged) gSawClogged = true;
}

// Duty 0 and forward. Defined before abortPending(), which calls it.
static void safeStop()
{
    DispenserOutputs stop = {0, true};
    motorApply(stop);
}

// True once an abort has been requested. The first key stops the motor on the
// spot, and from then on every wait below returns false immediately - so an
// abort ends the whole command, not just the wait it happened to land in.
static bool abortPending()
{
    if (abortRequested) return true;

    if (Serial.available() > 0) {
        int c = Serial.read();
        if (c >= 0 && c != '\r' && c != '\n') {
            safeStop();
            Serial.println(">> aborted by key - motor off");
            abortRequested = true;
            return true;
        }
    }
    return false;
}

// Returns false when the operator asked to abort.
static bool benchPump()
{
    if (abortPending()) return false;

    uint32_t now = millis();

    injectorPump(injector, now);
    if (dispenserControlTick(logic, now, out)) noteStep();
    heartbeat(now);

    return true;
}

static bool runFor(uint32_t ms)
{
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (!benchPump()) return false;
        delay(1);
    }
    return true;
}

static bool runWindow(uint32_t ms, Window &w)
{
    w.ms = ms;

    uint32_t a0 = encoderEdges();
    uint32_t b0 = bEdges();
    uint32_t h0 = bHighEdges();

    if (!runFor(ms)) return false;

    w.aEdges = encoderEdges() - a0;
    w.bEdges = bEdges()      - b0;
    w.bHigh  = bHighEdges()  - h0;
    return true;
}

// For the H tests: the motor is driven directly, so nothing else may touch it.
// This keeps the radio alive and checks for the abort key, and nothing more.
static bool motorWait(uint32_t ms)
{
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (abortPending()) return false;
        heartbeat(millis());
        delay(1);
    }
    return true;
}

static bool motorWindow(uint32_t ms, Window &w)
{
    w.ms = ms;

    uint32_t a0 = encoderEdges();
    uint32_t b0 = bEdges();
    uint32_t h0 = bHighEdges();

    if (!motorWait(ms)) return false;

    w.aEdges = encoderEdges() - a0;
    w.bEdges = bEdges()      - b0;
    w.bHigh  = bHighEdges()  - h0;
    return true;
}

// Waits with the radio alive and nothing sent - used by the packet tests.
static bool silentWait(uint32_t ms)
{
    return motorWait(ms);
}

typedef bool (*Pred)();

// Pumps until the condition holds or the timeout expires. False means the
// timeout expired *or* the operator aborted - callers must check
// abortRequested before using that as a result.
static bool pollUntil(uint32_t timeoutMs, Pred pred)
{
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (!benchPump()) return false;
        if (pred()) return true;
        delay(1);
    }
    return pred();
}

// Pumps until the given time has elapsed since startMs.
static bool pumpUntil(uint32_t startMs, uint32_t offsetMs)
{
    while ((millis() - startMs) < offsetMs) {
        if (!benchPump()) return false;
        delay(1);
    }
    return true;
}

// Every hardware-in-the-loop test starts here: motor off, a known inbox, a
// silent injector, a fresh logic, and a still shaft. False means the operator
// asked to abort, and the test must not run at all.
static bool benchPrepare()
{
    if (abortRequested) return false;

    safeStop();
    encoderBegin();             // a K test detaches channel A; always re-attach it
    dispenserResetInbox();
    injectorSilence(injector);

    // The watch flags describe the test that set them, never the one running
    // now: M03-M05 read gSawClogged, and a flag left over from a K test would
    // fail a healthy module.
    gRunMaxDuty     = 0;
    gSawZeroDuty    = false;
    gSawBuzzingDuty = false;
    gSawClogged     = false;

    if (!motorWait(1000)) return false;

    // The logic starts after the wait, so its first control step covers one
    // normal interval, as in production. Started before the wait, that step
    // spanned the whole second, and its integral kick held M02 about 10 % over
    // the target for the next few seconds.
    dispenserInit(logic, millis(), encoderEdges());
    lastPrintedMode = logic.mode;
    return true;
}

// ---------------------------------------------------------------------------
// Operator input. A prompt is the only place a key is an answer rather than an
// abort, and every operator step can be skipped with 's'.
// ---------------------------------------------------------------------------

static int waitForKey()
{
    while (true) {
        heartbeat(millis());
        if (Serial.available() > 0) {
            int c = Serial.read();
            if (c == '\r' || c == '\n' || c < 0) continue;
            return c;
        }
        delay(10);
    }
}

// y -> 1, n -> 0, s -> -1 (skip). Anything else is ignored.
static int askYesNo()
{
    while (true) {
        int c = waitForKey();
        if (c == 'y' || c == 'Y') return 1;
        if (c == 'n' || c == 'N') return 0;
        if (c == 's' || c == 'S') return -1;
    }
}

// ---------------------------------------------------------------------------
// Boot report
// ---------------------------------------------------------------------------

struct BootInfo {
    esp_reset_reason_t reason   = ESP_RST_UNKNOWN;
    bool               reasonOk = false;
    bool               hadMarker = false;
    char               markerId[8] = {};
    bool               espnowOk = false;
};

static BootInfo bootInfo;

static const char *resetReasonName(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_EXT:       return "external pin";
        case ESP_RST_SW:        return "software";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "interrupt watchdog";
        case ESP_RST_TASK_WDT:  return "task watchdog";
        case ESP_RST_WDT:       return "other watchdog";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_DEEPSLEEP: return "deep sleep wake";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "unknown";
    }
}

// One line for the resets that mean something went wrong.
static const char *resetReasonAdvice(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_BROWNOUT:
            return "the 12 V supply sagged - check the wiring, the capacitor and the supply current";
        case ESP_RST_PANIC:
            return "firmware crashed - report the test that was running";
        case ESP_RST_INT_WDT:
            return "interrupt watchdog - something blocked interrupts for too long";
        case ESP_RST_TASK_WDT:
            return "task watchdog - the loop stopped running";
        case ESP_RST_WDT:
            return "watchdog reset";
        default:
            return "";
    }
}

static void bootReport()
{
    Serial.println();
    Serial.println("=======================================================");
    Serial.println(" DISPENSER BENCH TEST - NOT FIELD FIRMWARE");
    Serial.printf( " built %s %s\n", __DATE__, __TIME__);
    Serial.println("=======================================================");

    Serial.printf(" reset reason: %s\n", resetReasonName(bootInfo.reason));
    if (!bootInfo.reasonOk) {
        Serial.printf(" FAIL H00: %s - %s\n", resetReasonName(bootInfo.reason),
                      resetReasonAdvice(bootInfo.reason));
    }

    if (bootInfo.hadMarker) {
        Serial.printf(" RESET DURING %s - the board died while that test was running\n",
                      bootInfo.markerId);
    } else {
        Serial.println(" no interrupted-test marker");
    }

    Serial.println(" settings:");
    Serial.printf("   pins            PWM %u  DIR %u  encoder A %u  encoder B %u\n",
                  (unsigned)MOTOR_PWM_PIN, (unsigned)MOTOR_DIR_PIN,
                  (unsigned)ENCODER_A_PIN, (unsigned)ENCODER_B_PIN);
    Serial.printf("   encoder         %lu edges per output revolution\n",
                  (unsigned long)ENCODER_EDGES_PER_REV);
    Serial.printf("   motor           max %u RPM, min running %u permille, PWM %lu Hz\n",
                  (unsigned)MOTOR_MAX_RPM, (unsigned)MOTOR_MIN_RUNNING_PERMILLE,
                  (unsigned long)MOTOR_PWM_FREQUENCY_HZ);
    Serial.printf("   control         every %lu ms, Kp %.2f, Ki %.2f, integral limit %.0f\n",
                  (unsigned long)MOTOR_CONTROL_INTERVAL_MS, (double)MOTOR_KP,
                  (double)MOTOR_KI, (double)MOTOR_INTEGRAL_LIMIT);
    Serial.printf("   clog            slower than %lu %% of target for %lu ms\n",
                  (unsigned long)CLOG_MIN_SPEED_PERCENT, (unsigned long)CLOG_DETECT_MS);
    Serial.printf("   unclog          %lu ms total, %u permille (%lu ms reverse, %lu ms pause, "
                  "%lu ms forward, %u cycles)\n",
                  (unsigned long)UNCLOG_TOTAL_MS, (unsigned)UNCLOG_PERMILLE,
                  (unsigned long)UNCLOG_REVERSE_MS, (unsigned long)UNCLOG_PAUSE_MS,
                  (unsigned long)UNCLOG_FORWARD_MS, (unsigned)UNCLOG_CYCLES);
    Serial.printf("   calibration     %u revolutions at %u RPM\n",
                  (unsigned)CALIBRATION_REVOLUTIONS, (unsigned)CALIBRATION_RPM);

    Serial.printf(" ESP-NOW: %s\n", bootInfo.espnowOk ? "initialised" : "FAILED TO INITIALISE");
}

static void printMenu()
{
    Serial.println();
    Serial.println("--- menu ------------------------------------------------------------------");
    Serial.println("  d  logic tests (D01-D35 and the ledger, L01-L12, no hardware)");
    Serial.println("  w  ground speed from wheel pulses (W01-W11, no hardware)");
    Serial.println("  v  packet tests (V01-V08, no motor)");
    Serial.println("  h  hardware (H00-H07, motor: read the checklist first)");
    Serial.println("  m  metering and links (M01-M10, motor)");
    Serial.println("  c  calibration run (C01-C06, motor)");
    Serial.println("  k  clog and unclogging (K01-K07, motor)");
    Serial.println("  a  all of the above, in that order");
    Serial.println("  r  report so far");
    Serial.println("  x  motor off now");
    Serial.println("  ?  this menu and the boot report");
    Serial.println("Any key pressed while a motor test runs aborts it.");
    Serial.println("---------------------------------------------------------------------------");
}

// Printed once per session, before the first test that turns the motor.
static bool askMotorChecklist()
{
    if (motorChecklistOk) return true;

    Serial.println();
    Serial.println("=== BEFORE THE FIRST MOTOR TEST ===");
    Serial.println("  1. the motor is clamped to the bench");
    Serial.println("  2. the coupling is off the auger and the hopper is empty (unless a test says otherwise)");
    Serial.println("  3. the 12 V supply gives at least 6 A, through the 10 A fuse");
    Serial.println("  4. the encoder Vcc is on 5 V, not 12 V");
    Serial.println("  5. USB powers the ESP32: the regulator's 5 V is disconnected from its 5V pin");
    Serial.println("Type y if all five are true ('s' skips this check and the motor tests).");

    int c = waitForKey();
    if (c == 'y' || c == 'Y') {
        motorChecklistOk = true;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// The tests. Only main.cpp is a .cpp file; each header is included exactly once
// from here, in the order their helpers are needed.
// ---------------------------------------------------------------------------

#include "tests_logic.h"
#include "tests_wheel.h"
#include "tests_packets.h"
#include "tests_hardware.h"
#include "tests_scenarios.h"

// ---------------------------------------------------------------------------

void setup()
{
    // Motor stopped before anything else can go wrong, exactly as production.
    motorBegin();

    encoderBegin();
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderBPulseISR, RISING);

    Serial.begin(SERIAL_BAUD);
    delay(300);   // let the host open the port, so the banner is not lost

    bootInfo.reason    = esp_reset_reason();
    bootInfo.reasonOk  = (bootInfo.reason == ESP_RST_POWERON) ||
                         (bootInfo.reason == ESP_RST_EXT) ||
                         (bootInfo.reason == ESP_RST_SW);
    bootInfo.hadMarker = (rtcMarker.magic == MARKER_MAGIC) && (rtcMarker.testId[0] != '\0');
    if (bootInfo.hadMarker) {
        strncpy(bootInfo.markerId, rtcMarker.testId, 7);
        bootInfo.markerId[7] = '\0';
        markerClear();
    }

    if (!bootInfo.reasonOk) {
        reportLine("H00", Outcome::Fail, "boot reset reason: %s (%s)",
                   resetReasonName(bootInfo.reason), resetReasonAdvice(bootInfo.reason));
    }
    if (bootInfo.hadMarker) {
        reportLine(bootInfo.markerId, Outcome::Fail, "RESET DURING %s (%s)",
                   bootInfo.markerId, resetReasonName(bootInfo.reason));
    }

    bootInfo.espnowOk = radioBegin();
    if (!bootInfo.espnowOk) {
        reportLine("H00", Outcome::Fail, "ESP-NOW did not initialise");
    }

    bootReport();
    printMenu();

    Serial.println();
    Serial.println("Ready. Press ? for the menu.");
}

void loop()
{
    heartbeat(millis());

    if (Serial.available() <= 0) return;

    int c = Serial.read();
    if (c == '\r' || c == '\n' || c < 0) return;

    // Only the commands that turn the motor need the setup checklist.
    bool motorCommand = (c == 'h' || c == 'm' || c == 'c' || c == 'k' || c == 'a');
    if (motorCommand && !askMotorChecklist()) {
        Serial.println(">> motor tests skipped");
        return;
    }

    switch (c) {
        case 'd':
            runLogicTests();
            break;
        case 'w':
            runWheelTests();
            break;
        case 'v':
            runPacketTests();
            break;
        case 'h':
            runHardwareTests();
            break;
        case 'm':
            runMeteringTests();
            break;
        case 'c':
            runCalibrationTests();
            break;
        case 'k':
            runClogTests();
            break;
        case 'a':
            abortRequested = false;
            runLogicTests();
            if (!abortRequested) runWheelTests();
            if (!abortRequested) runPacketTests();
            if (!abortRequested) runHardwareTests();
            if (!abortRequested) runMeteringTests();
            if (!abortRequested) runCalibrationTests();
            if (!abortRequested) runClogTests();
            break;
        case 'r':
            reportSummary();
            break;
        case 'x':
            Serial.println(">> motor off");
            break;
        case '?':
            bootReport();
            printMenu();
            break;
        default:
            break;
    }

    // Nothing may leave the motor running.
    safeStop();
    abortRequested = false;
}
