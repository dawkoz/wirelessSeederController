# CLAUDE.md

Guidance for working on this repo. Read this before making changes.

## What this project is

A wireless controller for a Kverneland Accord-style seeder, mimicking the Kverneland FGS Rhythmus functionality. Two (soon three) ESP32 modules talk to each other over **ESP-NOW** (not WiFi/MQTT — no router, no internet, no phone app):

- **Seeder module** (mounted on the seeder): reads sensors (turbine RPM, metering unit RPM via Hall sensor), drives the tramline relay.
- **Tractor module** (mounted in the tractor cab): OLED display, button for tramline selection, LED/buzzer fault alerts.
- **Dispenser module** (new, not yet implemented): will drive a fertilizer dispenser motor via a Cytron motor controller. Physically separate board from the seeder module for one reason only — the seeder's enclosure has no room left. It gets its own 12V power cable; everything else (commands/telemetry) is wireless like the other two boards.

## Philosophy — read this before changing anything

- **This runs on moving farm equipment, often unattended, sometimes far from a workbench.** Reliability and predictability beat cleverness. Prefer boring, explicit code over abstractions.
- **ESP-NOW, not WiFi infrastructure.** There is no router in a field. Don't suggest MQTT/HTTP/cloud-anything for board-to-board communication.
- **The wire protocol is the most fragile part of this system.** The tractor and seeder `memcpy` raw struct bytes at each other. Both boards **must** agree byte-for-byte on struct layout. That's why the structs live in one shared file — see below. Never let a board-specific struct definition exist outside it.
- **Hardware constraints drive architecture decisions**, not the other way around — e.g., the dispenser is a separate module purely because of physical enclosure space, not for software reasons.
- **Minimal dependencies.** Only Adafruit GFX + the (older, `SH1106_SWITCHCAPVCC`-API) Adafruit_SH1106 OLED library are used, and only by the tractor board.

## Repo / build structure

This is a **single PlatformIO project** (not Arduino IDE, not separate per-board projects) with **one build environment per physical board**:

```
platformio.ini              defines envs: tractor, seeder, dispenser
include/espnow_protocol.h   struct_seeder / struct_tractor — the shared wire protocol, single source of truth
src/tractor/main.cpp        tractor board firmware
src/seeder/main.cpp         seeder board firmware
src/dispenser/main.cpp      dispenser board firmware (currently a blank stub)
```

Why one project with multiple envs, instead of three separate PlatformIO projects: it lets `include/espnow_protocol.h` be physically the same file for every board that needs it, so the protocol can't silently drift between boards the way it could with copy-pasted struct definitions (which is how this repo worked before the PlatformIO migration — each `.ino` had its own copy).

**To build/upload a specific board:**
```bash
pio run -e tractor -t upload
pio run -e seeder -t upload
pio run -e dispenser -t upload
```
Or in VS Code: pick the environment from the PlatformIO status bar at the bottom, or the Project Tasks tree in the PlatformIO sidebar.

**Platform version is pinned** in `platformio.ini` (`espressif32@7.0.1`, Arduino-ESP32 core 2.0.17) deliberately. Arduino-ESP32 3.x (ESP-IDF 5) changes the ESP-NOW receive-callback signature (`OnDataRecv` gains an `esp_now_recv_info_t*` parameter) — bumping past the 2.x line will break both `tractor` and `seeder` builds until the callbacks are rewritten. Don't bump this without checking ESP-NOW API changes first.

## Hardware / pin reference

### Seeder ESP32 (`src/seeder/main.cpp`)
| Pin | Function |
|---|---|
| 12 | Relay control (tramline), active LOW |
| 14 | Turbine inductive sensor (interrupt, RISING) |
| 27 | Ground-wheel / metering unit Hall sensor, 3 magnets (interrupt, RISING) |

The pin 27 sensor sits on the ground wheel **before** the seed-rate gearbox, so it measures distance travelled regardless of the seed rate setting. It is the source for both "is the seeder moving" and ground speed — see `MM_PER_PULSE` in `src/seeder/main.cpp`, which is **a placeholder until measured**.

### Tractor ESP32 (`src/tractor/main.cpp`)
| Pin | Function |
|---|---|
| 12 | Tramline selection button |
| 14 | Green LED (connected) |
| 27 | Blue LED (connecting/blinking) |
| 13 | Yellow LED (tramline active) |
| 19 | Buzzer |

OLED: SH1106 128x64 over I2C, address `0x3C`. Single button drives the whole menu (Implementation_Plan §6): short press < 1500 ms, long press fires *at* 1500 ms while still held.

### Dispenser ESP32 (`src/dispenser/main.cpp`)
Firmware written, hardware not yet built. **Cytron MD13S** driver (PWM + DIR, 13 A continuous) and a **Pololu 4752** motor (37Dx68L, 30:1, 12 V, 330 RPM, 14 kg·cm, 5.5 A stall) with a built-in quadrature encoder (64 CPR motor shaft → 1920 CPR output shaft). Powered by its own 12 V line; control/telemetry wireless like the other two boards.

| Pin | Function |
|---|---|
| 25 | MD13S PWM (LEDC, 20 kHz) |
| 26 | MD13S DIR |
| 32 | Encoder channel A (interrupt, RISING) |
| 33 | Encoder channel B (wired, unused — quadrature reserved) |

`ENCODER_EDGES_PER_REV` is set to 480 — confirmed against Pololu's documentation: "64 CPR" counts both edges of both channels, so one channel's rising edges give 64 ÷ 4 = 16 per motor revolution, × 30:1 = 480 per output revolution. Still **verify by hand-turning the output shaft 10 revolutions** before trusting the rate control.

MD13S accepts 3.3 V logic directly, so PWM/DIR need no level shifter. Its PWM limit is 20 kHz; firmware runs at 16 kHz for margin.

Pins verified against the Espressif GPIO reference: none are strapping pins, none touch the SPI flash, all support interrupts and internal pull-ups, and none output PWM during boot the way GPIO 0/5/14/15 do. GPIO 32/33 double as `XTAL_32K_P/N`, but the 32.768 kHz crystal is not fitted on ESP32-WROOM-32, so they are free.

**Two hardware requirements that firmware cannot substitute for:**

- **10 kΩ pull-downs from MD13S PWM and DIR to GND.** ESP32 pins are high-impedance during reset and until `setup()` runs, so without them the driver's inputs float and the motor can run before any code executes.
- **The encoder's outputs sit at whatever its Vcc is fed** (spec range 3.5–20 V). Feed it 5 V and divide A/B down to 3.3 V. 12 V on that wire puts 12 V on a GPIO and destroys the board.

**Full parts list, pin verification and wiring: [docs/dispenser_module_hardware.md](docs/dispenser_module_hardware.md).**

## Completed

- ESP-NOW comms between tractor and seeder boards.
- Turbine RPM + metering unit activity sensing on the seeder board.
- Tramline relay control, manual tramline selection via button on the tractor board.
- OLED display + LED/buzzer fault alerting on the tractor board.
- Migration from Arduino IDE (two separate `.ino` sketches) to a single PlatformIO project with per-board build environments and a shared protocol header.
- **Protocol v2** — broadcast addressing (no MACs), magic/version/network-id validation, length-checked receives, per-peer link tracking with second-hand `linkFlags`.
- **Firmware hardening** — all display/buzzer/LED/relay work moved out of the ESP-NOW receive callback into `loop()`, `enum` fault codes replacing `String`, monotonic debounced pulse counters, fail-safe rules per Implementation_Plan §5.
- **Ground speed** derived from the existing ground-wheel sensor (no new hardware).
- **Single-button menu system** on the tractor (`Praca` / `Dawka` / `Kalibracja`) with NVS persistence.
- **Dispenser firmware** — MD13S drive, encoder feedback, closed-loop rate control, stall and over-speed detection.

## Current task

All of Implementation_Plan Phases 0–5 are written and compile cleanly (`pio run`, three environments, zero errors). **None of it has run on physical hardware yet.** Before field use, work through the Implementation_Plan §11 verification list — in particular the bench test with all three boards, and measuring `MM_PER_PULSE` and the dispenser calibration, both of which are placeholders in the source today.

## Future tasks

- **Real WOM (power take-off) RPM sensing** — currently hardcoded to `540` as a placeholder in `src/seeder/main.cpp` (`seederData.WOMRPM = 540; // Example value`). README lists this as TODO. Note the WOM fault alarms in `updateFaultStatus()` are consequently dead code today (`enableWOMAlarm` is `false`), and stay dead until a real sensor exists.
- **Wiring diagram** — README "Installation" and "Wiring Diagram" sections are still TODO.
- **Demonstration video** — README "Video" section is still TODO.

## Implementation_Plan

Working document for: hardening the existing firmware, protocol v2, and the fertilizer dispenser module. Written before implementation. Ideas that were considered and rejected are recorded in §8 with reasons, so they don't get re-proposed later.

**Guiding constraint for all of it:** the current two-module system works and is trusted in the field. Nothing here is a rewrite for its own sake. Each change below is justified either by "the dispenser can't be built without it" or "this is a real failure mode on farm equipment". Anything that was merely stylistic got dropped.

### 1. Review findings on the current firmware

Severity: **[P0]** blocks the dispenser · **[P1]** real risk, fix while we're in here · **[P2]** cleanup.

**F1 [P0] — Display, buzzer and LEDs are driven from inside the ESP-NOW receive callback.**
`OnDataRecv()` in `src/tractor/main.cpp:73` calls `updateDisplay()` and `updateFaultStatus()`. That callback runs in the WiFi stack's task, not in `loop()`. Three concrete consequences:

- `updateDisplay()` pushes the entire 1 KB framebuffer over I2C. Blocking that long inside the RX callback stalls the WiFi task and drops incoming packets. A third broadcasting node makes it worse.
- `loop()` *also* calls `updateDisplay()` from the button handler (`src/tractor/main.cpp:172`). So two different tasks can call into `Wire` and the library's shared `static uint8_t buffer[]` concurrently. Neither is thread-safe → torn frames or a wedged I2C bus. A latent race that testing rarely reproduces and more traffic makes more likely.
- **Live bug, worth fixing on its own merits:** the buzzer is toggled *only* inside `updateFaultStatus()`, which is *only* reached on packet receipt. If the link drops mid-beep while the pin happens to be HIGH, **the buzzer sounds continuously until a packet arrives again.** The display has the same shape of problem — it freezes on the last received frame with nothing telling the operator the reading is stale.

Fix: the RX callback does nothing but validate, `memcpy` into a buffer, and stamp `lastRxMs`. All display/buzzer/LED/relay work moves into `loop()`.

**F2 [P0] — `memcpy` with no length or type validation.**
Both boards do `memcpy(&x, incomingData, sizeof(x))` with no check on `len`. That is safe today *only* because exactly one peer exists and it sends exactly one struct type. The moment a third node broadcasts a different-sized packet, this reads past the received buffer and silently corrupts state. Every receive path must check magic + version + network id + msgType + `len == sizeof(expected)` before copying.

**F3 [P0] — No fail-safe on link loss.**
`tractorData.tramlineActive` holds its last received value forever. If the tractor module dies or drives out of range, the seeder keeps the relay wherever it last was. For a relay that is merely wrong; for the dispenser's **motor** it means it keeps running, unattended, indefinitely. Every actuator needs an explicit receive-timeout → safe state.

**F4 [P1] — Pulse counters lose counts.**
`pulseCountTurbine` is read and then set to `0` (`src/seeder/main.cpp:135-137`); a pulse arriving between those two statements is lost. Tolerable for an RPM readout, not for speed/distance where the error accumulates over a field. Fix: ISR counters become monotonic (`count++`, never reset), and the reader takes a snapshot and subtracts the previous snapshot. Bonus: this is also packet-loss tolerant over the radio (§4) and yields cumulative distance for free.

**F5 [P1] — Arduino `String` used for state.**
`faultStatus` and `success` are heap-allocated `String`s, reassigned at 5 Hz and compared with `==`. On a device expected to run unattended for a full working day that is needless heap churn and fragmentation risk. Replace with `enum class FaultCode : uint8_t`; that also removes the string comparisons from `updateDisplay()`.

**F6 [P1] — Fault status is computed *after* the display is drawn.**
In `OnDataRecv()`, `updateDisplay()` runs before `updateFaultStatus()`, so every frame renders the *previous* packet's fault state — faults appear one cycle (~200 ms) late. Resolves itself once both move into `loop()` in the right order.

**F7 [P2] — `calculateRPM()` truncates.**
`pulses * (60000 / intervalMs)` evaluates the integer division first. Exact at 2000 ms (= 30), but silently wrong at other intervals (7000 → 8 instead of 8.57, a 7% error) — a trap for whoever next tunes the interval. Use `(pulses * 60000UL) / elapsedMs`, divide by *measured* elapsed time rather than the nominal interval, and fold in a `PULSES_PER_REV` constant instead of assuming 1.

**F8 [P2] — `esp_now_recv_cb_t(OnDataRecv)` cast.**
The C-style cast suppresses precisely the signature check that would warn us on an Arduino-ESP32 3.x bump — the exact breakage the version pin in `platformio.ini` exists to guard against. Drop the cast and let the compiler enforce it.

**F9 [P2] — Dead/shadowed variable.** Global `int currentState;` (`src/tractor/main.cpp:28`) is shadowed by a local of the same name in `loop()`. Remove the global.

**F10 [P2] — Magic numbers that the README advertises as configurable.** `%6` (tramline rhythm) and `tramlineNumber == 2 || tramlineNumber == 3` (which passes fire the relay) are hardcoded. Promote to named constants at the top of the file.

**F11 [P2] — WiFi channel not pinned, power save left enabled.** Peer `channel = 0` means "whatever channel we happen to be on". With three nodes, be deterministic: `WiFi.mode(WIFI_STA); WiFi.disconnect(); esp_wifi_set_channel(...); esp_wifi_set_ps(WIFI_PS_NONE);` identically on all three boards. Leaving power save on is a well-known cause of intermittently missed ESP-NOW frames.

**F12 — Not a bug: transmitting unconditionally every 200 ms. Keep it.**
Flagging this explicitly because it looks wasteful and it isn't. The fixed-rate packet *is* the heartbeat. If a node only transmitted on change, silence would be ambiguous — "nothing changed" and "the other board is dead" would be indistinguishable, and the F3 fail-safe becomes impossible to implement. Periodic send is the correct design here. ESP-NOW at 5 Hz with ~30-byte payloads is nowhere near any limit; there is nothing to win by optimising it.

### 2. Addressing: drop the hardcoded MACs

Answering the question directly: **today, yes — replacing one ESP32 means editing and reflashing *both* boards** (the new module's MAC must be written into the other board's source). With three modules that becomes six edit points and an easy way to brick a working system in the middle of a field.

Fix: send to the ESP-NOW **broadcast address** `FF:FF:FF:FF:FF:FF` — registered once as a peer, an identical constant on every board — and put a `senderId` in the packet header. Every node hears every packet and filters by `msgType`/`senderId`. Swapping any module then needs **zero** firmware changes anywhere. This is the single highest-value change in the plan.

Consequences, all handled elsewhere here:

- The unicast MAC-layer ACK disappears, so `lastMessageDelivered` (which drives the green/blue "connected" LED) stops meaning anything. Replace it with *time since last received packet*, tracked **per sender** (§2.1). That is a strictly better liveness signal anyway: an ACK only proved the peer's radio responded, whereas a received packet proves the other board is alive and running its loop.
- ESP-NOW encryption requires per-peer keys and is incompatible with broadcast. `encrypt` is already `false`, so nothing is lost — see §8.3.
- Two identical systems in adjacent fields would hear each other. One `NETWORK_ID` byte in the header, checked on receive, removes that whole class of problem for free.

**Does a board receive its own broadcasts?** No. The WiFi hardware does not loop its own transmissions back into its own receive path, so a node never sees its own packets. The `sender` field in the header makes this robust regardless — every receive path filters on it anyway, so even if that behaviour ever changed, nothing would break.

#### 2.1 Link status — per-peer, and second-hand

Two layers, both cheap:

1. **Direct:** each board keeps `lastRxMs[NodeId]` — one timestamp per sender. A peer counts as present if heard within `LINK_TIMEOUT_MS`. At 5 Hz, a 1000 ms window means 5 consecutive misses before a link is declared down; comfortably tolerant of the occasional dropped frame.
2. **Second-hand:** every message carries a `linkFlags` byte — one bit per node, set if *this sender* has heard that node recently. So the tractor learns whether the **seeder and dispenser can hear each other**, even though the tractor has no direct visibility into that link. Costs one byte and no extra traffic.

That gives the tractor the full 3×3 connectivity picture from packets it was already receiving. Display treatment in §6.

Deliberately **not** doing mesh relaying/redundancy (a node forwarding packets on behalf of an unreachable peer): the link has never dropped once in the field, all three modules are within a few metres of each other, and relaying would add store-and-forward state, loop-prevention and packet-age handling to solve a problem that does not exist. The `linkFlags` byte means that if this assumption ever stops holding, the operator sees it rather than being surprised by it.

### 3. Protocol v2 — `include/espnow_protocol.h`

One tagged format: a common header plus one payload struct per message type. Fixed-width types throughout, `packed`, with `static_assert`s so a layout change can never silently reach the air.

**Naming note:** names are spelled out rather than abbreviated. `PROTOCOL_*`, not `PROTO_*`.

**What the "magic" bytes are:** a fixed two-byte signature (`'K'`,`'S'` — Kverneland Seeder) at the very start of every packet. ESP-NOW will hand us *any* frame that lands on our channel, including packets from an unrelated ESP-NOW project someone else built. Checking two known bytes first is the cheapest possible way to discard anything that isn't ours before it's interpreted as data. It costs 2 bytes per packet and two comparisons, and it turns "garbage silently memcpy'd into our state" (F2) into "packet dropped". The version and network-id bytes right after it do the same job for two other cases: firmware that has drifted out of step, and an identical machine working the next field over.

```c
static constexpr uint8_t PROTOCOL_MAGIC_0 = 'K';   // Kverneland
static constexpr uint8_t PROTOCOL_MAGIC_1 = 'S';   // Seeder
static constexpr uint8_t PROTOCOL_VERSION = 2;
static constexpr uint8_t NETWORK_ID       = 1;     // bump only if a neighbouring machine runs this firmware

enum class NodeId  : uint8_t { Tractor = 1, Seeder = 2, Dispenser = 3 };
enum class MsgType : uint8_t { SeederTelemetry = 1, TractorCommand = 2, DispenserStatus = 3 };

// linkFlags: which nodes THIS sender has heard from recently (§2.1)
static constexpr uint8_t LINK_HEARD_TRACTOR   = 1 << 0;
static constexpr uint8_t LINK_HEARD_SEEDER    = 1 << 1;
static constexpr uint8_t LINK_HEARD_DISPENSER = 1 << 2;

struct __attribute__((packed)) MessageHeader {
    uint8_t  magic0, magic1;
    uint8_t  version;
    uint8_t  networkId;
    MsgType  type;
    NodeId   sender;
    uint8_t  linkFlags;
    uint8_t  reserved;         // keeps the header at 8 bytes and leaves room to grow
};
static_assert(sizeof(MessageHeader) == 8, "MessageHeader layout changed - reflash ALL boards");
```

Payloads (field list is the intent; exact widths settled during implementation):

- **`SeederTelemetry`** (Seeder → all, 5 Hz): `upTimeMs`, `wheelPulses` (cumulative — see below), `turbineRPM`, `womRPM`, `groundSpeed_mm_s`, `wheelTurning`, `tramlineRelayOn`.
- **`TractorCommand`** (Tractor → all, 5 Hz): `tramlineNumber`, `tramlineRelayOn`, `doseKgPerHa`, `gramsPer100Rev` (so the dispenser doesn't hold its own copy of calibration — §7).
- **`DispenserStatus`** (Dispenser → all, 5 Hz): `targetShaftRPM`, `measuredShaftRPM`, `motorCommandedPermille`, `motorRunning`, `faultCode`.

Plus one shared `inline bool headerValid(const uint8_t* data, int len, MsgType expected, size_t expectedSize)` helper in this header — the F2 validation written once, called by all three boards.

**One wheel counter, not two.** Correction from the original draft: the existing metering-unit hall sensor **is** the ground-wheel sensor — the metering unit is ground-driven, so its rotation already measures distance travelled. There is no second sensor and no new seeder hardware. So there is exactly one cumulative counter, `wheelPulses` (3 magnets, already fitted), and both the existing "is it turning" boolean and the new ground speed are derived from it (§4).

A useful free consequence: when the seeder is lifted at the headland the ground wheel stops, so `wheelPulses` stops, so speed reads 0, so the dispenser stops (§5). The lifted-at-headland interlock needs no extra sensor and no extra logic.

Why cumulative counters rather than per-interval counts is in §8.7 — it is the non-obvious part of this design.

### 4. Ground speed — derived from the sensor already fitted

**No new seeder hardware.** The metering-unit hall sensor (GPIO 27, 3 magnets, already mounted) is ground-driven, so it already measures distance. Everything below is firmware-only.

- **One calibration constant: `MM_PER_PULSE`.** Do *not* model it as wheel-diameter × drive-ratio ÷ magnets — that needs three numbers, each of which can be measured wrong, to compute one number. Instead measure it directly: roll the seeder along a tape-measured distance, read the cumulative pulse count, divide. The whole drivetrain collapses into one empirically-correct constant, and the measurement takes five minutes. Hardcode it as a named constant (these parameters don't change in service).
- `speed_mm_s = (Δpulses × MM_PER_PULSE × 1000) / Δms`, over a ~1000 ms window, using *measured* elapsed ms (F7).
- **Zero detection:** no pulse for > 2000 ms → speed 0, `wheelTurning` false. This replaces the current `pulses >= 2` per-3-s boolean, so one mechanism serves both purposes.
- **ISR debounce:** reject pulses closer together than `MIN_PULSE_INTERVAL_US`, derived from the maximum plausible working speed. Inductive noise would otherwise inflate reported speed and — once the dispenser is speed-driven — over-apply fertilizer.
- **Low-speed resolution:** with 3 magnets on a ground-driven shaft, pulses are sparse at creep speed and a fixed window quantises badly. Since the hardware is fixed, handle it in firmware: widen the averaging window at low pulse rates (or hold the last valid speed until the next pulse), and treat "no pulse for 2 s" as a genuine stop. Accept that metering accuracy degrades below walking pace — that is also true of the mechanical seeder itself.
- The seeder sends **both** raw cumulative `wheelPulses` *and* its computed `groundSpeed_mm_s`, so `MM_PER_PULSE` lives in exactly one place — the board that owns the sensor. See §8.6.

> **⚠ Open question that gates this whole section — §10.** If the sensor sits *after* the seeder's variable seed-rate gearbox, then `MM_PER_PULSE` silently changes every time the seed rate is adjusted, and both the speed readout and the dispenser dose would be wrong in a way that looks fine on the display. If it sits on the ground-wheel side of that gearbox, the constant holds forever. **Confirm before implementing Phase 2.**

### 5. Fail-safe model

`LINK_TIMEOUT_MS = 1000` (5 missed packets at 5 Hz). The governing principle, per your call: **a gap in the field is a permanent defect; a machine that keeps doing what it was last told is usually the lesser harm.** So actuators hold their last command on link loss, *except* where holding it would be meaningless or unsafe.

| Board | Condition | Behaviour |
|---|---|---|
| Seeder | no `TractorCommand` for any duration | **hold last tramline relay state.** Deliberately no timeout — dropping the relay mid-pass would leave an unmarked gap in the tramline, which is a permanent, visible defect in the field. Holding state is benign by comparison. |
| Seeder | boot, before any command | relay **off** (`HIGH`, de-energised) — current behaviour, unchanged |
| Dispenser | no `SeederTelemetry` for 1000 ms | **motor stopped.** Without ground speed it cannot meter at all, so running would apply an arbitrary, unknown rate. |
| Dispenser | no `TractorCommand` for 1000 ms | **hold last dose + calibration, keep metering to measured speed** — same reasoning as the tramline: an unfertilised strip is a permanent defect. Safe because the speed interlock below still applies. ⚠ *Confirm — see §10.* |
| Dispenser | ground speed 0 / wheel not turning | **motor stopped.** Covers headland turns, lifting, and standing still, with no extra sensor. |
| Dispenser | boot, before any packet | PWM 0 written **first** in `setup()`, before anything else |
| Tractor | any peer not heard for 1000 ms | blue LED + a single-letter indicator in the corner of the display (§6.4). **No buzzer yet.** |
| Tractor | a peer that *had been heard* goes quiet for **5000 ms** | buzzer sounds. Timed from loss of contact, not from boot — powering the tractor up before the seeder is not a fault and must stay silent. |
| Dispenser | calibration run active, tractor link lost | run aborts, motor stopped (§6.5) — the opposite of normal metering, because the tractor is what commanded the run |
| Tractor | existing turbine / WOM / metering faults | unchanged from current firmware |

The buzzer must be serviced **every** `loop()` iteration so it can never stick on (F1). Also enable the ESP32 task watchdog — cheap insurance on an unattended machine.

### 6. UX — one button, menu-driven

**Decided: keep the tractor module, keep the single existing button, no hardware changes.** The phone/SoftAP config mode from the earlier draft is **dropped** — see §8.4. Only two numbers ever need entering (calibration weight and dose), which a button menu handles perfectly well, so the whole SoftAP/ESP-NOW coexistence problem and the iOS flakiness simply never arise. Removing it is a strict simplification.

Press classification, from the existing debounced input:

- **Short press** — released before 1500 ms.
- **Long press** — fires **at** 1500 ms while still held, *not* on release. Firing at the threshold gives immediate feedback with gloves on; the subsequent release is swallowed. Confirm with a short buzzer blip so the operator doesn't have to look at the screen.

#### 6.1 Screen / state machine

```
BOOT
 └─> MENU ──────────────────────────────────────────────┐
      │  short = next item (wraps, selection shown in    │
      │          inverted contrast: black on white)      │
      │  long  = enter selected item                     │
      │                                                  │
      ├─ [Praca]      long ─> WORK ──── long ────────────┤
      ├─ [Dawka]      long ─> EDIT(3 digits, kg/ha) ─────┤
      └─ [Kalibracja] long ─> EDIT(5 digits, g/100 obr.) ┘
                                        (ZAPISZ / long)

WORK:
   short = advance tramline 1..6   (existing behaviour, unchanged)
   long  = back to MENU
   shows: turbine RPM, tramline number, ground speed, dispenser state,
          link indicators (§6.4), fault screens (existing behaviour)

EDIT (n digits + two trailing action fields):
   cursor order: digit 0 .. digit n-1, then ACTION, then ZAPISZ
       ACTION on Dawka      = [Wl./Wyl.]  toggles the dispenser (§6.3)
       ACTION on Kalibracja = [TEST]      starts a calibration run (§6.5)

   cursor mode:  short = move cursor right, wrapping through every field
                 long  = on a digit  -> enter digit mode
                         on ACTION   -> toggle / start calibration
                         on ZAPISZ   -> commit to NVS, return to MENU
   digit mode:   short = increment this digit 0..9, wrapping
                 long  = commit digit, return to cursor mode

   The selected field is drawn in inverted contrast, same visual language
   as the menu. A corner label reads WYBOR or ZMIEN so the current press
   meaning is never ambiguous.

CALIBRATION (from Kalibracja / TEST):
   confirm screen: "Start kalibracji?"  [Anuluj] [START]
                   short = move between them, long = choose
   running screen: progress bar + percent underneath
                   long  = abort (short is ignored mid-run on purpose)
                   when finished: "GOTOWE", any press returns
```

Every state is reachable and escapable with one button, and every transition is a single deterministic press class. No timeouts anywhere in the UI — nothing changes unless the operator presses something.

#### 6.2 Persistence — and how durable it actually is

`Dawka` (kg/ha), `Kalibracja` (g per 100 dispenser revolutions) and the dispenser on/off state persist via `Preferences`, which writes to the **NVS partition in the ESP32's SPI flash**. Concretely:

- **No battery is involved.** Flash keeps its contents with the board completely unpowered.
- **Retention is rated in the order of 10–20 years** at room temperature. A year on a shelf is nothing — the settings will still be there.
- **Endurance is ~100,000 erase cycles per sector**, and NVS wear-levels across the partition. The firmware only writes on `ZAPISZ` (and when toggling Wł./Wył.), i.e. at human speed, so this limit is unreachable in practice. This is *why* settings are not written every loop.
- **The one thing that erases them:** a full-chip erase (`pio run -t erase`, `esptool erase_flash`) or changing the partition table. A normal `pio run -t upload` leaves NVS intact, so reflashing firmware does **not** lose the calibration.

Everything else — working width 4 m, tramline rhythm, `MM_PER_PULSE` — stays a named compile-time constant (§10: these don't change in service). The tractor broadcasts the stored values in every `TractorCommand`, so the dispenser never holds its own copy and never needs reflashing when they change.

#### 6.3 Dispenser on/off

An explicit **`Wł./Wył.` field on the `Dawka` screen**, selected with short presses like any other field and toggled with a long press. The motor runs only when `dispenserEnabled && doseKgPerHa > 0`, so a zero dose still means off — belt and braces. The state persists in NVS and the WORK screen shows `DOZ:WYL.` or `DOZ:<dose>`, so it is never ambiguous.

#### 6.5 Automated calibration run

Turning the dispenser shaft 100 times by hand is miserable, so the `Kalibracja` screen has a **`TEST`** field alongside `ZAPISZ`. Long-pressing it opens a confirmation screen (`Start kalibracji?` → `Anuluj` / `START`, selected the same way as everything else). On `START` the tractor raises `calibrationRun` in its command packet and the dispenser turns its output shaft exactly `CALIBRATION_REVOLUTIONS` (100) times at a fixed 120 RPM, reporting progress as a percentage that the tractor draws as a progress bar.

Design points worth keeping:

- **The request is level-triggered**, repeated in every 5 Hz command packet, so it survives a dropped packet with no acknowledgement protocol. The tractor lowers it to cancel, or once it has seen `Done`.
- **Calibration deliberately bypasses the ground-speed interlock** — it is a stationary bench operation and the seeder may not even be powered. It therefore does *not* require `SeederTelemetry`.
- **But it refuses to run if the machine is actually moving.** If the seeder is heard and reports `wheelTurning`, the dispenser answers `Refused` and the tractor shows `MASZYNA W RUCHU`. This also aborts a run that was already going when the machine started moving.
- **Losing the tractor link aborts the run** and stops the motor — unlike normal metering, which holds. The tractor is what commanded it, so its silence means abort.
- A short press cannot stop a run in progress (it would spoil the weighing); only a long press aborts. Once finished, any press dismisses.

#### 6.4 Link indicators on the WORK screen

Per §2.1, the tractor knows both who *it* can hear and who the *others* can hear. Display in a corner, small, only when something is wrong (a clean screen means everything is fine):

- `S` — seeder not heard by the tractor
- `D` — dispenser not heard by the tractor
- `S-D` — both are talking to the tractor, but they cannot hear **each other** (the one failure mode invisible without §2.1's `linkFlags`, and the one that silently stops fertilizer being metered correctly)

Blue LED blinks whenever any link is down; buzzer only after 5 s (§5).

### 7. Dispenser module

**Hardware (confirmed):** Cytron **MD13S** driver + Pololu **4752** motor — 37Dx68L, 30:1 gearbox, 12 V, 330 RPM output, 14 kg·cm, ~200 mA typical / 5.5 A stall, with a **built-in quadrature encoder** (64 CPR at the motor shaft = 1920 CPR at the output shaft).

**The encoder changes the plan for the better:** the "add a feedback sensor" recommendation from the earlier draft is already satisfied by hardware you own. Closed-loop control and blockage detection are available from day one, so there is no open-loop-only interim design to build and then throw away.

#### 7.1 Control topology

The dispenser subscribes directly to `SeederTelemetry` (ground speed) and `TractorCommand` (dose + calibration) and computes its own motor setpoint locally. Rationale in §8.8.

#### 7.2 Rate maths

With working width **4 m** hardcoded, dose **D** kg/ha and calibration **C** grams per 100 output-shaft revolutions:

```
grams per metre travelled  = width[m] × D[kg/ha] / 10        = 0.4 · D
revolutions per metre      = (0.4 · D) / (C / 100)           = 40 · D / C
target output-shaft RPM    = 60 · v[m/s] · 40 · D / C        = 2400 · v · D / C
```

Sanity check at D = 40 kg/ha: 16 g of fertilizer per metre travelled.

#### 7.3 ⚠ Motor speed is the binding constraint — check this before building

The motor tops out at **330 RPM**, which caps working speed:

```
v_max [m/s] = 330 · C / (2400 · D)
```

At the target 40 kg/ha:

| Calibration C (g per 100 rev) | Max working speed |
|---|---|
| 300 | 3.7 km/h |
| 500 | 6.2 km/h |
| 800 | 9.9 km/h |
| 1000 | 12.4 km/h |

So the dispenser must deliver roughly **≥ 800 g per 100 revolutions** to sustain 10 km/h at 40 kg/ha. If the auger meters less than that per revolution, the motor simply cannot keep up and the machine will silently under-apply at working speed. **This is a mechanical sizing question to settle before the dispenser is built** — it is decided by auger/rotor geometry, not by firmware. Firmware's job is to make the failure loud, not to hide it: when the required RPM exceeds what the motor can deliver, clamp to maximum, raise a fault, and alarm on the tractor (`Za szybko` / over-speed). Slowing down then genuinely fixes it.

The 14 kg·cm torque figure is likewise unverifiable from here — whether it turns a loaded fertilizer auger is a bench test, not a calculation.

#### 7.4 Encoder wiring — the one way to destroy the ESP32 here

Encoder pinout: red/black = **motor power** (goes to the MD13S, not the ESP32), green = encoder GND, blue = encoder Vcc, yellow = channel A, white = channel B.

**The encoder outputs sit at whatever voltage you feed its Vcc.** Its spec range is 3.5–20 V, so feeding it from the 12 V rail would put **12 V straight onto a GPIO** and destroy the board. Note 3.3 V is marginally *below* the 3.5 V minimum, so the safe options are: power the encoder from 5 V and divide A/B down to 3.3 V (the same divider technique already used in the seeder's sensor box), or verify 3.3 V operation on the bench first.

**Counting:** direction is fixed (the auger only runs one way), so full quadrature decoding is unnecessary — count rising edges on **channel A only**. That is ≈480 edges per output-shaft revolution (verify empirically: Pololu's "64 CPR" is quadrature counts, so ÷4 per channel, ×30 gearbox). At the 330 RPM maximum that is ~2.6 kHz of interrupts — comfortable for a minimal `IRAM_ATTR` ISR on a 240 MHz ESP32, and half the load of decoding both channels. If it ever does become a problem, the ESP32's PCNT peripheral counts in hardware for free; not worth the complexity up front.

#### 7.5 Motor drive

MD13S is sign-magnitude: **PWM** + **DIR**. Hardware PWM via `ledcSetup`/`ledcAttachPin` (Arduino-ESP32 2.x API — 3.x renamed this to `ledcAttach`, another reason the version pin in `platformio.ini` matters). Run PWM at ~20 kHz to stay out of the audible range and within the MD13S spec. Keep DIR on a real GPIO rather than strapping it, so the auger can be reversed to clear a blockage.

Suggested pins (avoiding strapping pins 0/2/12/15): `PWM = 25`, `DIR = 26`, `ENC_A = 32`, `ENC_B = 33` (B wired but unused initially).

**Control law:** feed-forward from §7.2 (which is accurate as soon as C is calibrated) plus a PI trim on measured shaft RPM. Feed-forward does the work; PI only corrects for battery sag and load variation, so it can be gentle and is hard to destabilise. Note that a DC motor will not start below some minimum duty (stiction) — clamp the output to either 0 or ≥ that minimum rather than letting it sit in a buzzing, non-rotating band.

**Fault detection**, all from the encoder: commanded but `measuredShaftRPM ≈ 0` → blockage/stall/decoupled shaft; measured persistently below target → over-speed (§7.3) or slipping. Both go into `DispenserStatus.faultCode` and alarm on the tractor. This is the thing that stops a whole field being done wrong without anyone noticing.

#### 7.6 Power

Own 12 V feed. Tractor 12 V is electrically filthy — load dumps, alternator noise, and a motor drawing up to 5.5 A on stall right next to a microcontroller. Needs a proper buck converter, generous input filtering, and TVS protection, with motor and logic grounds joined at a single point. Brownout resets from a naively shared supply are the most likely real-world failure mode of the finished module, and it is a wiring problem, not a firmware one.

### 8. Rejected ideas, and why

1. **Event-driven / on-change-only transmission** — kills the heartbeat, makes silence ambiguous, and makes the §5 fail-safe impossible. See F12.
2. **MQTT / HTTP / cloud for board-to-board comms** — no router in a field. Already project philosophy; recorded here so it stays rejected.
3. **ESP-NOW encryption** — incompatible with broadcast addressing, and the threat model (someone parked in your field with an ESP32 and this repo) does not justify giving up zero-reflash module swapping.
4. **Phone / SoftAP config mode** — proposed in the first draft, now **dropped entirely**. It only ever existed to make entering calibration numbers bearable, and the §6 menu does that with hardware that already exists. Keeping it would have meant SoftAP/ESP-NOW radio coexistence, iOS's hostility to networks without internet, an HTTP server, and an HTML page to maintain — all to type two numbers. The menu is strictly less code and strictly more reliable.
5. **Automatic MAC pairing/discovery** (nodes learn each other's MACs at boot, store in NVS) — solves the same problem as broadcast but adds a pairing state machine, persistent state that can go stale, and a "how do I re-pair after a swap" field procedure — i.e. reintroduces exactly the manual step we set out to delete. Broadcast achieves the goal with less code and no state.
6. **Sending only raw pulses and computing speed on each receiver** — would duplicate `MM_PER_PULSE` onto every board, so recalibrating would mean reflashing all three. The seeder owns the sensor, so the seeder owns the constant and publishes a finished speed. Raw cumulative pulses are still sent alongside, for distance and diagnostics.
7. **Sending per-interval pulse counts instead of cumulative counters** — a dropped packet would permanently lose those counts, biasing distance and average speed low. Cumulative counters self-heal across gaps: a receiver that misses packets still computes the correct delta once the next one lands. This is why §3 sends cumulative values.
8. **Tractor computes the dispenser's motor setpoint and sends it** — adds a hop and a dependency: a tractor-module failure would take the dispenser down even though the seeder is still broadcasting perfectly good speed data. Local computation on the dispenser also keeps the control loop next to the motor.
9. **One shared struct for all three nodes** — any field change would force all three boards to be reflashed in lockstep, which is the exact coupling this refactor removes. Tagged per-type messages let a node ignore types it doesn't care about.
10. **Rewriting around FreeRTOS tasks / an event framework** — the `loop()` + timestamps pattern is entirely sufficient at 5 Hz, and "boring beats clever" is the stated philosophy. The one genuine concurrency problem (F1) is fixed by *removing* work from another context, not by adding more contexts.
11. **A second seeder sensor for ground speed** — proposed in the first draft on a misreading. The metering unit is ground-driven, so the sensor already fitted measures distance; adding another would be duplicate hardware measuring the same shaft. Firmware-only (§4).
12. **Open-loop dispenser as an interim phase** — proposed before the motor was known. The Pololu 4752 has a built-in encoder, so closed-loop is available immediately and an open-loop stage would be throwaway work.
13. **Full quadrature decoding of the dispenser encoder** — the auger turns one way only, so channel A alone gives ≈480 edges per output revolution, which is far more resolution than the control loop needs, at half the interrupt load. Channel B gets wired anyway so the option stays open.
14. **Deriving `MM_PER_PULSE` from wheel diameter × drive ratio ÷ magnets** — three measurable quantities, three chances to be wrong, to produce one number that can simply be measured directly with a tape measure in five minutes. See §4.
15. **Mesh relaying / redundant paths between modules** — §2.1. Solves a failure that has never occurred at a range of a few metres, and adds real complexity. `linkFlags` makes the assumption observable instead.

### 9. Phases

Each phase compiles, is independently field-testable, and is a sensible commit boundary.

- **Phase 0** — protocol v2 header + broadcast addressing + per-peer link tracking, tractor/seeder only, **no behaviour change**. Verify the existing link still works on real hardware before building anything on top.
- **Phase 1** — restructure the callbacks (F1, F2, F6), `enum` fault codes (F5), §5 fail-safe rules, link indicators + 5 s buzzer rule.
- **Phase 2** — counter rework (F4, F7), ground speed from the existing sensor, speed on the display. **Gated on the gearbox question in §10.**
- **Phase 3** — cleanups F8–F11.
- **Phase 4** — menu system (§6) + NVS persistence. Deliverable on its own: `Dawka`/`Kalibracja` are enterable and stored before any motor exists.
- **Phase 5** — dispenser firmware: encoder counting, closed-loop control, fault detection.

Phase 4 before Phase 5 deliberately — it means the dispenser's inputs are already proven and adjustable by the time the motor is first energised.

### 10. Open questions

**Answered:** tractor hardware stays as-is (single button, no encoder) · sensor box does have a divider · metering sensor *is* the ground-wheel sensor, 3 magnets, already fitted · Cytron MD13S · Pololu 4752 with built-in encoder · 4 m working width · 40 kg/ha · room exists for dispenser shaft feedback (and the motor's own encoder already provides it).

**✔ The metering sensor is BEFORE the seed-rate gearbox** — it is fixed to the ground wheel, so seed-rate adjustment does not affect it. `MM_PER_PULSE` is therefore a genuine constant and §4 is unblocked.

**✔ Dispenser holds its last dose on tractor link loss** (does *not* stop) — confirmed. Only loss of `SeederTelemetry`, or zero ground speed, stops the motor.

**Still open (neither blocks firmware):**

1. **Auger output per revolution** — §7.3: does it deliver ≥ ~800 g per 100 revolutions? Decides whether 40 kg/ha at 10 km/h is achievable at all. Mechanical, not firmware; firmware clamps and alarms either way.
2. **Menu wording** — `Praca` / `Dawka` / `Kalibracja` assumed; `Dawka = 000` as the dispenser off-switch (§6.3) implemented as proposed.
3. **`MM_PER_PULSE` value** — placeholder in `src/seeder/main.cpp` until the 20 m roll-out measurement is done (§11). Everything else works; only the speed number is wrong until then.

### 11. Verification

- `pio run` (all three environments) after every phase.
- `static_assert`s on every wire struct — a layout change fails the build instead of corrupting data over the air.
- **Bench test all three boards on a desk before mounting anything.** Pull power from each in turn and confirm §5 behaviour: seeder *holds* its relay, dispenser *stops* on seeder loss, and the right corner letters appear on the tractor.
- **`MM_PER_PULSE` calibration:** mark a 20 m run, push the seeder along it, compare reported distance from `wheelPulses` against the tape measure. Repeat at two different seed-rate settings to close out §10.1.
- **Dispenser calibration:** use the built-in `Kalibracja` → `TEST` run (§6.5) — it turns the shaft exactly 100 revolutions on its own. Catch and weigh the output → `Kalibracja`. Verify `ENCODER_EDGES_PER_REV` first by hand-turning 10 revolutions (expect ≈4800 edges), since every dose depends on it.
- **Blockage detection:** with the dispenser commanded and running, physically stall the shaft; the tractor must alarm.
- **Buzzer stuck-on regression (F1):** trigger a fault, then cut power to the seeder mid-beep. The buzzer must stop, not sing until the battery dies.

## Important notes

- **`GPIO12` is the MTDI strapping pin** — its level at reset selects the flash voltage, and held high at reset the chip configures 1.8 V flash and **fails to boot**. Both existing boards use it (seeder relay, tractor button) and both work today because the pin sits low at reset. Nothing needs changing, but **never add an external pull-up to GPIO12**, and if the relay module is ever swapped for one with a pull-up on its input, expect a board that no longer boots.
- **No MAC addresses anywhere.** All three boards transmit to the ESP-NOW broadcast address and filter incoming packets by the `type`/`sender` fields in `MessageHeader`. A physical ESP32 can be swapped for a new one with **no firmware change on any board**.
- ESP-NOW runs on a fixed `ESPNOW_CHANNEL` (1) with power save disabled, set identically on all three boards in `setup()`. `encrypt` is `false` — required, since ESP-NOW encryption needs per-peer keys and is incompatible with broadcast addressing (Implementation_Plan §8.3). The registered peer uses `channel = 0` ("current radio channel") deliberately: naming the channel on both the radio and the peer creates a way for them to disagree, and a mismatch makes *every* `esp_now_send()` fail. Send failures are counted and logged on serial rather than discarded.
- **The tractor calls `Wire.setClock(400000)` after `oled.begin()` — leave it there.** The vendored SH1106 library set the same speed via the AVR `TWBR` register, which had to be removed for the ESP32 port. Without it the bus runs at Arduino's default 100 kHz, a full redraw takes ~103 ms instead of ~26 ms, and since the button is polled once per `loop()` it starts dropping presses. See `lib/Adafruit_SH1106/README.md`.
- **Any change to `include/espnow_protocol.h` means reflashing all three boards.** Bump `PROTOCOL_VERSION` when you do — receivers drop mismatched packets, so a half-updated set of boards fails loudly instead of quietly misreading each other. The `static_assert`s on every struct turn an accidental layout change into a build error.
- The Adafruit_SH1106 library used by the tractor board is the older community library (class `Adafruit_SH1106`, `begin(SH1106_SWITCHCAPVCC, addr)` API), originally from [wonho-maker/Adafruit_SH1106](https://github.com/wonho-maker/Adafruit_SH1106) — not Adafruit's newer `Adafruit_SH110X` library, which has a different, incompatible API. It's **vendored** (not pulled via `lib_deps`) at `lib/Adafruit_SH1106/`, with a small ESP32-portability patch — upstream targets AVR only and doesn't compile as-is against the ESP32 Arduino core. See `lib/Adafruit_SH1106/README.md` for exactly what was changed and why (no behavioral change, both patched spots are on the SPI code path this project doesn't use — only I2C).
