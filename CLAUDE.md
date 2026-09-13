# CLAUDE.md

Guidance for working on this repo. Read this before making changes.

## Contents

1. [What this project is](#what-this-project-is)
2. [Philosophy](#philosophy)
3. [Important notes](#important-notes)
4. [Repo and build structure](#repo-and-build-structure)
5. [Hardware and pins](#hardware-and-pins)
6. [Tractor screens](#tractor-screens)
7. [Current tasks](#current-tasks)
   - [How to work on a task](#how-to-work-on-a-task)
   - [Task 1: Clog alarm and unclogging](#task-1-clog-alarm-and-unclogging)
   - [Task 2: Tramline switch](#task-2-tramline-switch)
   - [Task 3: Dispenser bench test](#task-3-dispenser-bench-test)
8. [Future tasks](#future-tasks)
9. [Completed](#completed)
   - [Design record](#design-record)

## What this project is

A wireless controller for a Kverneland Accord-style seeder, mimicking the Kverneland FGS Rhythmus functionality. Three ESP32 modules talk to each other over **ESP-NOW** (not WiFi/MQTT — no router, no internet, no phone app):

- **Seeder module** (on the seeder): turbine RPM sensor, ground-wheel sensor (ground speed and "is the seeder moving"), tramline relay.
- **Tractor module** (in the cab): OLED display, one button that drives every screen, LEDs and buzzer. Owns the tramline selection and the dispenser settings.
- **Dispenser module** (on the seeder): fertilizer dispenser motor driven through a Cytron motor driver, metered from the seeder's ground speed. A separate board for one reason only — the seeder's enclosure has no room left. It gets its own 12 V power cable; everything else is wireless like the other two boards. **Firmware written, hardware not built yet.**

## Philosophy

Read this before changing anything.

- **This runs on moving farm equipment, often unattended, sometimes far from a workbench.** Reliability and predictability beat cleverness. Prefer boring, explicit code over abstractions.
- **ESP-NOW, not WiFi infrastructure.** There is no router in a field. Don't suggest MQTT/HTTP/cloud-anything for board-to-board communication.
- **The wire protocol is the most fragile part of this system.** The boards `memcpy` raw struct bytes at each other, so all three **must** agree byte-for-byte on struct layout. That's why the structs live in one shared file. Never let a board-specific struct definition exist outside it.
- **Hardware constraints drive architecture decisions**, not the other way around — e.g., the dispenser is a separate module purely because of physical enclosure space, not for software reasons.
- **Minimal dependencies.** Only Adafruit GFX + the (older, `SH1106_SWITCHCAPVCC`-API) Adafruit_SH1106 OLED library are used, and only by the tractor board.

## Important notes

- **`GPIO12` is the MTDI strapping pin** — its level at reset selects the flash voltage, and held high at reset the chip configures 1.8 V flash and **fails to boot**. Both existing boards use it (seeder relay, tractor button) and both work today because the pin sits low at reset. Nothing needs changing, but **never add an external pull-up to GPIO12**, and if the relay module is ever swapped for one with a pull-up on its input, expect a board that no longer boots.
- **No MAC addresses anywhere.** All three boards transmit to the ESP-NOW broadcast address and filter incoming packets by the `type`/`sender` fields in `MessageHeader`. A physical ESP32 can be swapped for a new one with **no firmware change on any board**.
- ESP-NOW runs on a fixed `ESPNOW_CHANNEL` (1) with power save disabled, set identically on all three boards in `setup()`. `encrypt` is `false` — required, since ESP-NOW encryption needs per-peer keys and is incompatible with broadcast addressing (see Rejected ideas). The registered peer uses `channel = 0` ("current radio channel") deliberately: naming the channel on both the radio and the peer creates a way for them to disagree, and a mismatch makes *every* `esp_now_send()` fail. Send failures are counted and logged on serial rather than discarded.
- **Any change to `include/espnow_protocol.h` means reflashing all three boards.** Bump `PROTOCOL_VERSION` when you do — receivers drop mismatched packets, so a half-updated set of boards fails loudly instead of quietly misreading each other. The `static_assert`s on every struct turn an accidental layout change into a build error.
- **The dispenser writes PWM 0 before anything else in `setup()`** — keep it the first thing. The 10 kΩ pull-downs on the driver inputs cover the time before that (reset and boot).
- **The tractor calls `Wire.setClock(400000)` after `oled.begin()` — leave it there.** The vendored SH1106 library set the same speed via the AVR `TWBR` register, which had to be removed for the ESP32 port. Without it the bus runs at Arduino's default 100 kHz, a full redraw takes ~103 ms instead of ~26 ms, and since the button is polled once per `loop()` it starts dropping presses.
- The Adafruit_SH1106 library used by the tractor board is the older community library (class `Adafruit_SH1106`, `begin(SH1106_SWITCHCAPVCC, addr)` API), originally from [wonho-maker/Adafruit_SH1106](https://github.com/wonho-maker/Adafruit_SH1106) — not Adafruit's newer `Adafruit_SH110X` library, which has a different, incompatible API. It's **vendored** (not pulled via `lib_deps`) at `lib/Adafruit_SH1106/`, with three small ESP32-portability patches, because upstream targets AVR only. Two are on the SPI path this project doesn't use; the third removed the AVR I2C clock setting, which is why the `Wire.setClock` call above exists. Details: `lib/Adafruit_SH1106/README.md`.

## Repo and build structure

This is a **single PlatformIO project** (not Arduino IDE, not separate per-board projects) with **one build environment per physical board**:

```
platformio.ini              defines envs: tractor, seeder, dispenser
include/espnow_protocol.h   message structs — the shared wire protocol, single source of truth
include/machine_settings.h  every machine value (pins, calibration, timings, alarms), grouped by board
src/tractor/main.cpp        tractor board firmware
src/seeder/main.cpp         seeder board firmware
src/seeder/wheel_speed.h    ground speed from wheel pulse timing (no hardware code, so it can be tested on a PC)
src/dispenser/main.cpp      dispenser board firmware
docs/dispenser_module_hardware.md   dispenser parts list, pin verification, wiring
```

Why one project with multiple envs, instead of three separate PlatformIO projects: it lets `include/espnow_protocol.h` be physically the same file for every board that needs it, so the protocol can't silently drift between boards the way it could with copy-pasted struct definitions (which is how this repo worked before the PlatformIO migration — each `.ino` had its own copy).

**To build/upload a specific board:**
```bash
pio run -e tractor -t upload
pio run -e seeder -t upload
pio run -e dispenser -t upload
```
Or in VS Code: pick the environment from the PlatformIO status bar at the bottom, or the Project Tasks tree in the PlatformIO sidebar. On the development PC `pio` is not on PATH from a plain shell — see [How to work on a task](#how-to-work-on-a-task) for the full path.

**Platform version is pinned** in `platformio.ini` (`espressif32@7.0.1`, Arduino-ESP32 core 2.0.17) deliberately. Arduino-ESP32 3.x (ESP-IDF 5) changes the ESP-NOW receive-callback signature (`OnDataRecv` gains an `esp_now_recv_info_t*` parameter) — bumping past the 2.x line will break the builds until the callbacks are rewritten. Don't bump this without checking ESP-NOW API changes first. The core compiles C++ as **`-std=gnu++11`** (GCC 8.4).

## Hardware and pins

All pin numbers below are set in `include/machine_settings.h`.

### Seeder ESP32 (`src/seeder/main.cpp`)
| Pin | Function |
|---|---|
| 12 | Relay control (tramline), active LOW |
| 14 | Turbine inductive sensor (interrupt, RISING) |
| 27 | Ground-wheel Hall sensor, 3 magnets (interrupt, RISING) |

The pin 27 sensor sits on the ground wheel **before** the seed-rate gearbox, so it measures distance travelled regardless of the seed rate setting. It is the source for both "is the seeder moving" and ground speed — see `WHEEL_MM_PER_PULSE` in `include/machine_settings.h`, which is **an estimate until measured** (628 mm: a 60 cm wheel with 3 magnets).

### Tractor ESP32 (`src/tractor/main.cpp`)
| Pin | Function |
|---|---|
| 12 | The button (to GND) |
| 14 | Green LED (connected) |
| 27 | Blue LED (connecting/blinking) |
| 13 | Yellow LED (tramline active) |
| 19 | Buzzer |

OLED: SH1106 128x64 over I2C, address `0x3C`. The single button drives every screen (see [Tractor screens](#tractor-screens)): short press < 1500 ms, long press fires *at* 1500 ms while still held.

### Dispenser ESP32 (`src/dispenser/main.cpp`)
Firmware written, hardware not yet built. **Cytron MD13S** driver (PWM + DIR, 13 A continuous) and a **Pololu 4752** motor (37Dx68L, 30:1, 12 V, 330 RPM, 14 kg·cm, 5.5 A stall) with a built-in quadrature encoder (64 CPR motor shaft → 1920 CPR output shaft). Powered by its own 12 V line; control/telemetry wireless like the other two boards.

| Pin | Function |
|---|---|
| 25 | MD13S PWM (LEDC, 16 kHz) |
| 26 | MD13S DIR |
| 32 | Encoder channel A (interrupt, RISING) |
| 33 | Encoder channel B (wired, unused by the production firmware) |

`ENCODER_EDGES_PER_REV` is set to 480 — confirmed against Pololu's documentation: "64 CPR" counts both edges of both channels, so one channel's rising edges give 64 ÷ 4 = 16 per motor revolution, × 30:1 = 480 per output revolution. Still **verify by hand-turning the output shaft 10 revolutions** before trusting the rate control. Counting one channel cannot tell direction: the count goes **up** whichever way the shaft turns.

MD13S accepts 3.3 V logic directly, so PWM/DIR need no level shifter. Its PWM limit is 20 kHz; firmware runs at 16 kHz for margin.

Pins verified against the Espressif GPIO reference: none are strapping pins, none touch the SPI flash, all support interrupts and internal pull-ups, and none output PWM during boot the way GPIO 0/5/14/15 do. GPIO 32/33 double as `XTAL_32K_P/N`, but the 32.768 kHz crystal is not fitted on ESP32-WROOM-32, so they are free.

**Two hardware requirements that firmware cannot substitute for:**

- **10 kΩ pull-downs from MD13S PWM and DIR to GND.** ESP32 pins are high-impedance during reset and until `setup()` runs, so without them the driver's inputs float and the motor can run before any code executes.
- **The encoder's outputs sit at whatever its Vcc is fed** (spec range 3.5–20 V). Feed it 5 V and divide A/B down to 3.3 V. 12 V on that wire puts 12 V on a GPIO and destroys the board.

**Full parts list, pin verification and wiring: [docs/dispenser_module_hardware.md](docs/dispenser_module_hardware.md).**

## Tractor screens

What the tractor's OLED shows and what the button does on each screen, as built in `src/tractor/main.cpp`. **Update this section in the same commit whenever a screen or a button action changes.**

- **Short press:** released before 1.5 s. **Long press:** fires at 1.5 s while the button is still held, with a 30 ms beep. The release after it does nothing.
- In the mockups, `[ ]` marks the highlighted (inverted) field. Text and positions follow the code, but large fonts are drawn at normal size.

### Map

```
Power on: Adafruit logo while starting up, 0.5 s beep
└── 1 MENU
    ├── Praca ────── 2 WORK
    │                └── 3 FAULT, replaces WORK while active
    ├── Dawka ────── 4 DOSE EDITOR
    └── Kalibracja ─ 5 CALIBRATION EDITOR
                     └── TEST ─ 6 CONFIRM
                                └── START ─ 7 RUNNING
                                            ├── 8 DONE
                                            ├── 9 MACHINE MOVING
                                            └── 10 NO DISPENSER
```

Screens 7–10 are one screen. Its content changes by itself as the dispenser reports.

### Mockups

```
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│[Praca              ]│  │RPM 3150             │  │                     │
│                     │  │6.4km/h    DOZ:40    │  │ DOZOWNIK            │
│ Dawka               │  │                     │  │                     │
│                     │  │Przejazd:       3    │  │ BLOKADA             │
│ Kalibracja          │  │BRAK: D              │  │                     │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
        1. Menu                  2. Work                  3. Fault

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│DAWKA kg/ha     WYBOR│  │KALIBR. g/100   WYBOR│  │Start kalibracji?    │
│                     │  │                     │  │100 obrotow          │
│      [0] 4  0       │  │   [0] 0  5  0  0    │  │                     │
│                     │  │                     │  │[Anuluj             ]│
│ WL.        ZAPISZ   │  │ TEST       ZAPISZ   │  │ START               │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
     4. Dose editor       5. Calibration editor          6. Confirm

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│Kalibracja...        │  │GOTOWE               │  │Kalibracja           │
│                     │  │                     │  │                     │
│ |##########-------| │  │Zwaz nawoz i wpisz   │  │MASZYNA              │
│                     │  │wynik w gramach.     │  │                     │
│         60%         │  │Nacisnij aby wrocic  │  │W RUCHU              │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
       7. Running                8. Done             9. Machine moving

┌─────────────────────┐
│Kalibracja           │
│                     │
│BRAK                 │
│                     │
│DOZOWNIKA            │
└─────────────────────┘
    10. No dispenser
```

### What each screen does

| Screen | Content | Short press | Long press |
|---|---|---|---|
| 1 Menu | `Praca`, `Dawka`, `Kalibracja`. Starts on `Praca`, then stays on the item last opened | next item | open it |
| 2 Work | turbine RPM, ground speed, dispenser setting (`DOZ:WYL.` or the dose), pass number 1–6 in large digits. The `BRAK:` line appears only while a link is down: `S` seeder, `D` dispenser (only after it has been heard once), `S-D` seeder and dispenser can't hear each other | next pass, 6 → 1 | back to 1 |
| 3 Fault | `DOZOWNIK BLOKADA` (shaft not turning) or `DOZOWNIK ZA SZYBKO` (driving faster than the dispenser can keep up with) | next pass, although the pass number is hidden | back to 1 |
| 4 Dose editor | dose in kg/ha, `WL./WYL.`, `ZAPISZ` | see Editors | see Editors |
| 5 Calibration editor | grams per 100 dispenser revolutions, `TEST`, `ZAPISZ` | see Editors | see Editors |
| 6 Confirm | `Anuluj` (preselected) or `START` | switch | `Anuluj` → 5, `START` → 7 |
| 7 Running | progress of the 100-revolution run | ignored | cancel the run → 5 |
| 8 Done | weigh the output and enter it in grams | → 5 | → 5 |
| 9 Machine moving | run refused or stopped, because the seeder reports the wheel turning | → 5 | → 5 |
| 10 No dispenser | dispenser not heard | ignored | cancel the run → 5 |

### Editors (4 and 5)

- Open with the saved value, cursor on the first digit.
- The top-right label says what a short press does: `WYBOR` moves the cursor, `ZMIEN` changes the digit.
- The cursor goes through each digit, then `WL./WYL.` or `TEST`, then `ZAPISZ`, then back to the first digit.
- Long press on a digit switches to `ZMIEN`, where a short press adds 1 (9 → 0). Long press again to go back to `WYBOR`.
- Long press on `WL./WYL.` switches the dispenser on or off straight away. The setting is stored with `ZAPISZ`.
- Long press on `TEST` opens 6. Coming back from 6–10 keeps the digits as they were, with the cursor on `TEST`.
- Long press on `ZAPISZ` stores the value and returns to 1. It's the only way out, so leaving an editor always saves.

### On every screen

- **Buzzer:** beeps ¼ s on, ¼ s off while a dispenser fault is active, including on screens that don't show the fault. It also beeps, with nothing on screen, when a board that was heard has been silent for about 6 s, or when the seeder and dispenser haven't heard each other for about 6 s.
- **LEDs:** green = all links fine. Blue blinking = a link is down, including the seeder not heard since power-on. Yellow = the seeder's last report says the tramline relay is on.
- Coded but switched off: the `WOM` and `Dmuchawa` fault screens (`ENABLE_WOM_ALARM` and `ENABLE_TURBINE_ALARM` are `false`).

## Current tasks

Three tasks, each doable on its own in a separate session. The user will say which one to do; do only that one.

| Task | Boards to reflash afterwards | Depends on |
|---|---|---|
| [Task 1: Clog alarm and unclogging](#task-1-clog-alarm-and-unclogging) | all three (protocol v4) | — |
| [Task 2: Tramline switch](#task-2-tramline-switch) | tractor | — (small extra step if Task 1 is already done) |
| [Task 3: Dispenser bench test](#task-3-dispenser-bench-test) | none (it is a separate image for the bench) | **Task 1** |

Recommended order: 1, 2, 3.

### How to work on a task

1. **Read the whole task section first**, then read every file it touches **completely** (not excerpts). The existing code has comments explaining why things are the way they are — keep that reasoning intact.
2. **Build commands.** `pio` is not on PATH on this PC. Use the full path:
   - PowerShell: `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e dispenser`
   - Git Bash: `"$HOME/.platformio/penv/Scripts/pio.exe" run -e dispenser`
   - All environments: the same without `-e ...`.

   Build only. Never run `-t upload`, `-t erase` or the serial monitor unless the user asks — no board is connected during these sessions, so **the compiler is the only automatic check you have**. Everything else depends on careful reading.
3. **C++11 only** (`-std=gnu++11`, GCC 8.4): no `std::make_unique`, no inline variables, no structured bindings, no generic lambdas; a `constexpr` function may contain only a single `return` statement.
4. **Match the existing style:** `static constexpr` constants in `include/machine_settings.h` (grouped by board, with a comment when the value needs explaining), `enum class ... : uint8_t`, comments that say *why*, no `String`, no dynamic allocation, no new libraries, fixed-width integer types in anything that goes on the wire.
5. **UI text is Polish without diacritics** (the built-in font has no Polish letters: `WYL.` not `WYŁ.`). Use the strings exactly as the task gives them. The built-in font is 6×8 px per character × text size, and text wraps to the next line silently: from x = 0 a line holds 21 characters at size 1, 10 at size 2, 7 at size 3, 5 at size 4.
6. **Don't touch** what the task doesn't list: the platform pin in `platformio.ini`, `lib/Adafruit_SH1106/`, pin numbers, the seeder's behaviour, other tasks' sections.
7. **Git:** don't commit or push unless the user asks. The working tree may contain the user's uncommitted work — never `git checkout`, `git restore`, `git reset` or `git stash` files. Run `git status` and `git diff --stat` **before you start** and keep the output: the "only these files changed" checks below mean *changed by you in this session*, compared with that starting point.
8. **If the spec looks wrong, contradicts the code, or can't be built as written, stop and ask the user** — describe the conflict precisely. Don't improvise a different design. Decisions marked as agreed with the user are not open for change.
9. **Final check — required before saying the task is done.** Run the full build (all environments, zero errors, no new warnings). Then re-read the task section from the top and go through its *Final verification* list one item at a time: open the code, confirm the item, and write a table `ID | done? | file:line | note`. Fix anything missing. Report to the user: the files changed, that table, every deviation from the spec and why, questions for the user, and which boards need reflashing.
10. **Documentation is part of the task:** each task ends by updating the sections it names, moving itself to [Completed](#completed) (two or three bullets) and deleting its own section here, and updating the [Contents](#contents) list and the table above.

### Task 1: Clog alarm and unclogging

**Goal.** When the dispenser shaft can't turn, the dispenser stops the motor and reports a clog. The tractor shows an alarm on top of whatever screen is up; the operator either resumes (`Anuluj`) or runs an automatic back-and-forth unclogging sequence (`Odetkaj`). This replaces today's `DOZOWNIK BLOKADA` stall alarm.

**Files:** `include/espnow_protocol.h`, `include/machine_settings.h`, `src/dispenser/` (split into three files, see 1.4), `src/tractor/main.cpp`, `CLAUDE.md`. `src/seeder/main.cpp` needs no edit but gets rebuilt and must be reflashed with the others.

#### 1.1 Decisions agreed with the user — implement exactly, don't re-ask

- **Clog** = the shaft turns slower than 1/3 of the commanded speed for 1.5 s. Checked only while the motor is meant to turn: normal metering and the calibration run. Never while unclogging.
- On a clog the dispenser switches the motor off, sets direction forward, resets the speed controller, drops a running calibration and enters mode `Clogged`. The motor stays off until the operator decides.
- **The tractor alarm covers any screen:** `ZATKANIE!` / `DOZOWNIKA` / `[OK]` with the buzzer beeping. A short **or** long press on it opens the choice `Anuluj` / `Odetkaj`, with `Anuluj` preselected and the buzzer silent.
- **`Anuluj` clears the clog:** the dispenser goes back to normal metering (which still needs the machine moving). If the shaft is still blocked, the clog is detected again and the alarm comes back.
- **`Odetkaj` runs the reverse/forward sequence (4 s) once**, then the tractor is back on the choice screen. There is no check of whether it worked.
- **Commands are counters:** the tractor adds 1 to a counter for `Anuluj` or for `Odetkaj`, repeats the counter in every packet, and the dispenser acts once per change.
- **Button rule:** a press counts only if the same screen was showing for the whole press. The tramline pass changes only on a visible work screen.
- **`DOZOWNIK ZA SZYBKO` stays a full-screen alarm with the buzzer** (the user considers it critical), but the button must not change the tramline while it shows.
- **Not wanted:** an alarm for a calibration value of 0, or a check after 50 calibration turns.

Added in this spec for robustness (implement them too): the sender check in `headerValid()`, the locked receive snapshots, reboot-safe counters (tractor `upTimeMs`), the calibration "armed" rule and the `PRZERWANA` screen that it needs.

#### 1.2 Protocol v4 (`include/espnow_protocol.h`)

- `PROTOCOL_VERSION` 3 → 4, with a `// v4: ...` line under the v3 comment line.
- **Delete `enum class CalibrationState`** and every use of it. Add:

  ```cpp
  // What the dispenser is doing. DispenserStatus.faultCode is only meaningful in Normal.
  enum class DispenserMode : uint8_t {
      Normal          = 0,   // metering to ground speed (motor off while not moving, off, or no seeder)
      Calibrating     = 1,   // automated calibration run turning the shaft
      CalibrationDone = 2,   // run finished, motor off, until the tractor withdraws the request
      Refused         = 3,   // run refused or stopped because the machine is moving
      Clogged         = 4,   // shaft blocked, motor off, waiting for Anuluj or Odetkaj
      Unclogging      = 5,   // running the reverse/forward sequence
  };
  ```
- `DispenserFault`: remove `Stalled`. Values become `None = 0`, `NoSpeedData = 1`, `OverSpeed = 2`.
- `TractorCommand`: append after `gramsPer100Rev`, in this order:

  ```cpp
  uint32_t upTimeMs;       // tractor millis(); a smaller value than last time means it rebooted
  uint8_t  clogClearSeq;   // +1 each time the operator picks Anuluj on the clog screen
  uint8_t  unclogSeq;      // +1 each time the operator picks Odetkaj
  ```
  and change its `static_assert` to `== 24`. Comment why these are counters: a flag held high would repeat the action, and a one-packet pulse is lost with one dropped packet; a counter repeated in every packet survives loss and is acted on once. Also say that the dispenser ignores a change that arrives right after a link gap or a tractor reboot (1.4, step 2).
- `DispenserStatus`: `CalibrationState calibrationState` → `DispenserMode mode`; `uint8_t calibrationPercent` → `uint8_t progressPercent` (0–100 while `Calibrating` or `Unclogging`, 100 in `CalibrationDone`, 0 otherwise). The size stays 18, keep `static_assert(... == 18 ...)`.
- `headerValid()` must also require that `header.sender` matches the message type: `SeederTelemetry` only from `NodeId::Seeder`, `TractorCommand` only from `NodeId::Tractor`, `DispenserStatus` only from `NodeId::Dispenser`. A mislabelled packet must not mark a board alive. Write the mapping as a plain `inline` function.

#### 1.3 Settings (`include/machine_settings.h`)

DISPENSER section: delete `STALL_TIMEOUT_MS`, `STALL_RPM_THRESHOLD` and their "Stall alarm" block. After the "Speed control" block add:

```cpp
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
```

TRACTOR section: in "Button" add `BUTTON_SCREEN_SETTLE_MS = 250` (comment: a press counts only if its screen was already showing this long before the press began — faster than a person can react to a new screen). Add a "Calibration run" block with `CALIBRATION_START_TIMEOUT_MS = 2000` (comment: after START, the dispenser not calibrating for this long means the run was interrupted).

#### 1.4 Dispenser: logic, I/O and main in separate files

**Why the split:** Task 3 must run exactly this decision logic on the bench with simulated packets. So the logic can't live next to `setup()`/`loop()` or call hardware functions. `src/seeder/wheel_speed.h` is the precedent.

| File | Contains | Rules |
|---|---|---|
| `src/dispenser/dispenser_logic.h` | mode machine, rate maths, speed controller, clog check, unclog sequence, command counters, status fill | header-only, `inline` functions, all state inside `struct DispenserLogic`. **No** `millis()`, `micros()`, `digitalWrite`, `ledcWrite`, `attachInterrupt`, `Serial`, global or `static` variables. Time and encoder count come in through `DispenserInputs`. |
| `src/dispenser/dispenser_io.h` + `dispenser_io.cpp` | encoder interrupt and counter, motor PWM/DIR output, the receive inbox (latest validated packets + `LinkTracker`), one-call control tick | the only code that touches pins; no ESP-NOW init or send |
| `src/dispenser/main.cpp` | `setup()`, `loop()`, ESP-NOW init, one-line receive callback, status broadcast, serial log of mode changes | thin |

`platformio.ini` needs no change (`+<dispenser/*>` compiles both `.cpp` files).

**Public interface — use these exact names, Task 3 depends on them:**

```cpp
// dispenser_logic.h
struct DispenserInputs {
    uint32_t        nowMs;
    uint32_t        encoderEdges;    // channel A rising edges since boot, never reset
    bool            seederAlive;     // valid SeederTelemetry within LINK_TIMEOUT_MS
    bool            haveTelemetry;   // at least one ever received
    SeederTelemetry telemetry;       // the latest one (stale when !seederAlive)
    bool            tractorAlive;
    bool            haveCommand;
    TractorCommand  command;
};

struct DispenserOutputs {
    uint16_t motorPermille;          // 0..1000
    bool     motorForward;
};

struct DispenserLogic { /* members are up to you */ };

inline void     dispenserInit(DispenserLogic &logic, uint32_t nowMs, uint32_t encoderEdges);
inline bool     dispenserStep(DispenserLogic &logic, const DispenserInputs &in, DispenserOutputs &out);
inline void     dispenserFillStatus(const DispenserLogic &logic, DispenserStatus &status);   // everything but the header
inline uint32_t requiredShaftRPM(uint16_t speedMmS, uint16_t doseKgPerHa, uint32_t gramsPer100Rev);

// dispenser_io.h
void     motorBegin();                                  // DIR forward, LEDC attached, duty 0
void     motorApply(const DispenserOutputs &out);
void     encoderBegin();                                // INPUT_PULLUP + RISING interrupt on ENCODER_A_PIN
void     encoderEnd();                                  // detach it (bench fault injection only)
uint32_t encoderEdges();
void     dispenserHandlePacket(const uint8_t *data, int len);   // the whole body of the receive callback
void     dispenserReadInputs(uint32_t nowMs, DispenserInputs &in);
void     dispenserResetInbox();                         // forget every received packet (bench only)
uint8_t  dispenserLinkFlags();
bool     dispenserControlTick(DispenserLogic &logic, uint32_t nowMs, DispenserOutputs &out);
```

**`dispenserStep` contract.** If less than `MOTOR_CONTROL_INTERVAL_MS` has passed since the previous step (or since `dispenserInit`), return `false` and leave `out` untouched. Otherwise run exactly one step with `elapsed = in.nowMs − previous step time`, fill `out`, return `true`. All interval maths is unsigned subtraction (`now - since`, wrap-safe). A running timer is marked by its own `bool`, never by `since == 0`. `dispenserInit` sets mode `Normal`, fault `None`, all timers stopped, controller reset, previous output duty 0 forward, `calibrationArmed = false`, `tractorWasAlive = false`.

**One step, in this order:**

1. **Measure.** `edges = in.encoderEdges − snapshot`, then `snapshot = in.encoderEdges`. `measuredRPM = edges × 60000 / (elapsed × ENCODER_EDGES_PER_REV)`, computed in 64-bit and clamped to 65535 (today's 32-bit expression overflows after a long gap between steps).
2. **Tractor command counters.** Only when `in.tractorAlive && in.haveCommand`:
   - `resync = !tractorWasAlive || in.command.upTimeMs < lastCommandUpTimeMs`
   - `resync` → copy both counters into `lastClearSeq` / `lastUnclogSeq` and act on nothing.
   - otherwise → `clearNow = (in.command.clogClearSeq != lastClearSeq)`, `unclogNow = (in.command.unclogSeq != lastUnclogSeq)`, then copy both. Compare with `!=`, never `>`: a `uint8_t` wraps 255 → 0.
   - `lastCommandUpTimeMs = in.command.upTimeMs`
   - `in.command.calibrationRun == 0` → `calibrationArmed = true`

   Then, in every step: `tractorWasAlive = in.tractorAlive && in.haveCommand`.
3. **Mode** — the table below.
4. **Reversal guard.** If the previous step's output had duty > 0 and this step's output has duty > 0 in the other direction, output duty 0 with the previous direction instead. The sequence already has pauses; the guard makes a reversal at speed impossible whatever the timing.
5. Store this step's output as the previous output (it is also what the status reports).

**Mode table.** "Motor off" = duty 0, direction forward. "Controller reset" = integral 0, target 0, clog timer stopped — also done on every mode change.

| Mode | What one step does |
|---|---|
| `Normal` | **Start calibration?** If `tractorAlive && haveCommand && command.calibrationRun && calibrationArmed`: `calibrationArmed = false`; if `seederAlive && haveTelemetry && telemetry.wheelTurning` → `Refused`, else → `Calibrating` with `startEdges = in.encoderEdges`, progress 0. Motor off this step either way.<br>**Otherwise meter:** (a) `!seederAlive` or `!haveTelemetry` → fault `NoSpeedData`, motor off, controller reset. (b) `enabled = haveCommand && command.dispenserEnabled` — tractor liveness is **not** required, the last dose is held on purpose; `dose = enabled ? command.doseKgPerHa : 0`; `calib = haveCommand ? command.gramsPer100Rev : 0`. If `!telemetry.wheelTurning`, `telemetry.groundSpeedMmS == 0` or `dose == 0` → fault `None`, motor off, controller reset. (c) `required = requiredShaftRPM(speed, dose, calib)`; 0 → as (b). (d) `overSpeed = required > MOTOR_MAX_RPM`, clamp to `MOTOR_MAX_RPM`; target = required; duty from the controller, forward; clog check; unless the clog check just entered `Clogged`, fault = `OverSpeed` or `None`. |
| `Calibrating` | `!tractorAlive`, `!haveCommand` or `!command.calibrationRun` → `Normal`, motor off. Else seeder alive, have telemetry and `wheelTurning` → `Refused`, motor off. Else `turned = in.encoderEdges − startEdges`; `turned ≥ CALIBRATION_REVOLUTIONS × ENCODER_EDGES_PER_REV` → `CalibrationDone`, progress 100, motor off. Else progress = `turned × 100 / total`, target `CALIBRATION_RPM`, duty from the controller, forward, clog check. Fault `None` throughout. |
| `CalibrationDone`, `Refused` | Motor off. `tractorAlive && haveCommand && !command.calibrationRun` → `Normal`. Nothing else leaves these (the motor is off, so holding is safe). |
| `Clogged` | `clearNow` → `Normal`, motor off. Else `unclogNow` → `Unclogging` with `unclogStartMs = in.nowMs`, outputting the first phase (reverse) in this same step. Otherwise motor off. Calibration requests are ignored. |
| `Unclogging` | `clearNow` → `Normal`, motor off. Else `!tractorAlive` → `Clogged`, motor off. Else `t = in.nowMs − unclogStartMs`; `t ≥ UNCLOG_TOTAL_MS` → `Clogged`, motor off. Else phase from `p = t % UNCLOG_CYCLE_MS`: `p < REVERSE` → `UNCLOG_PERMILLE` reverse; `p < REVERSE + PAUSE` → duty 0 forward; `p < REVERSE + PAUSE + FORWARD` → `UNCLOG_PERMILLE` forward; else duty 0 forward. progress = `t × 100 / UNCLOG_TOTAL_MS`. No clog check. |

**Clog check** (in `Normal` step (d) and in `Calibrating`, after the duty is computed): `slow = measuredRPM × 100 < target × CLOG_MIN_SPEED_PERCENT` (32-bit maths). Slow and timer stopped → start it at `in.nowMs`. Slow and `in.nowMs − timerStart ≥ CLOG_DETECT_MS` → **enter `Clogged`**: motor off (this step's output becomes duty 0 forward), controller reset, progress 0, fault `None`. Not slow → stop the timer.

**Controller** — today's `computeDuty()` plus anti-windup (the only new part is the condition on the integral):

```
feedForward = target × 1000 / MOTOR_MAX_RPM
error       = target − measuredRPM
candidate   = clamp(integral + error × MOTOR_KI × elapsed / 1000, −MOTOR_INTEGRAL_LIMIT, +MOTOR_INTEGRAL_LIMIT)
raw         = feedForward + MOTOR_KP × error + candidate
keep the old integral if (raw > 1000 and error > 0) or (raw < MOTOR_MIN_RUNNING_PERMILLE and error < 0),
otherwise integral = candidate
duty        = clamp(feedForward + MOTOR_KP × error + integral, 0, 1000)
if 0 < duty < MOTOR_MIN_RUNNING_PERMILLE: duty = MOTOR_MIN_RUNNING_PERMILLE
```

`requiredShaftRPM`: move today's function with its comment, unchanged. `dispenserFillStatus`: `targetShaftRPM`, `measuredShaftRPM`, `motorCommandedPermille` = the duty of the most recent step's output (after the reversal guard), `motorRunning` = that duty > 0, `faultCode`, `mode`, `progressPercent`.

**`dispenser_io.cpp`:**
- Encoder: today's `encoderPulseISR` (`IRAM_ATTR`, `ENCODER_MIN_PULSE_GAP_US` debounce, `volatile uint32_t` total that only ever increments).
- `motorBegin()`: today's first lines of `setup()` — DIR pin output at `MOTOR_DIR_FORWARD`, `ledcSetup`, `ledcAttachPin`, `ledcWrite(channel, 0)`. The LEDC channel/resolution constants move here.
- `motorApply(out)`: if `out.motorForward` differs from the level currently on DIR → `ledcWrite(0)` first, then write DIR (forward = `MOTOR_DIR_FORWARD`, reverse = the other level), then write the duty. Otherwise just write the duty.
- Inbox: `static` copies of the latest `SeederTelemetry` and `TractorCommand`, their have-flags, `static LinkTracker links`, and `static portMUX_TYPE inboxLock`. `dispenserHandlePacket()` does today's `OnDataRecv` checks, then `memcpy` + have-flag inside `portENTER_CRITICAL(&inboxLock)`, then `links.noteReceived(...)`. `dispenserReadInputs()` copies packets and flags under the same lock and fills `seederAlive`/`tractorAlive` from `links.isAlive()`, `encoderEdges` from the counter, `nowMs` from the argument. `dispenserResetInbox()` clears packets and flags and does `links = LinkTracker();` under the lock. `dispenserLinkFlags()` returns `links.flags()`.
- `dispenserControlTick()`: `dispenserReadInputs` → `dispenserStep` → if it stepped, `motorApply(out)` → return whether it stepped.

**`main.cpp`:** `setup()` calls `motorBegin()` first (before `Serial.begin`), then `encoderBegin()`, WiFi/ESP-NOW exactly as today, then `dispenserInit(logic, millis(), encoderEdges())`. `OnDataRecv(mac, data, len)` only calls `dispenserHandlePacket(data, len)`; register it without a cast. `loop()`: `dispenserControlTick(logic, now, out)`; when a step ran and the mode changed, print one line like `[12345] mode Normal -> Clogged`; then `sendStatus(now)` with `fillHeader(..., dispenserLinkFlags())` and `dispenserFillStatus()`. Nothing else is printed per step.

#### 1.5 Tractor (`src/tractor/main.cpp`)

**a) Receive snapshot.** Today the receive callback `memcpy`s into `seederData`/`dispenserData` while `loop()` reads them, so a half-copied struct can be read. The callback now writes `rxSeederData`, `rxDispenserData` and the ever-seen flags inside `portENTER_CRITICAL(&rxLock)`. The first thing `loop()` does is copy them, under the same lock, into `seederData`/`dispenserData`/the flags that the drawing code already uses.

**b) `loop()` order:**

```
1  receive snapshot                          (a)
2  updateFaults(now)                         must come before the view is worked out
3  clog and calibration bookkeeping          (c), (d)
4  view change detection                     (e)
5  readButton → handler for the CURRENT VIEW → 30 ms beep for a counted long press
6  updateOutputs(now)                        buzzer, LEDs
7  sendCommand(now)
8  redraw the current view when dirty, or periodically on Work, WorkFault, CalibProgress, Unclogging
```

**c) Clog overlay.**

```cpp
bool dispenserAlive = links.isAlive(NodeId::Dispenser);
bool clogActive = dispenserAlive &&
                  (dispenserData.mode == DispenserMode::Clogged || dispenserData.mode == DispenserMode::Unclogging);
if (!clogActive) clogAcknowledged = false;
if (clogActive && screen == Screen::CalibRunning) {
    calibrationRequested   = false;          // the dispenser dropped the run
    calibrationInterrupted = false;
    screen = Screen::EditCalibration;        // cursor is still on TEST
}
```

While `clogActive` the view is `Unclogging` when the mode is `Unclogging`, otherwise `ClogChoice` when `clogAcknowledged`, otherwise `ClogAlert`. The `screen` underneath is kept; when the clog ends the operator is back where they were.

| View | Short press | Long press |
|---|---|---|
| `ClogAlert` | `clogAcknowledged = true` | same |
| `ClogChoice` | toggle `clogChoiceIndex` | index 0 (`Anuluj`): `clogClearSeq++`; index 1 (`Odetkaj`): `unclogSeq++` |
| `Unclogging` | ignored | ignored |

After a pick the view stays `ClogChoice` until the dispenser reports its new mode (~0.2–0.4 s). A second pick in that window is harmless: the dispenser acts once per change and ignores changes outside `Clogged`.

**d) Calibration screen.** Add `bool calibrationInterrupted` and a timer. While `screen == CalibRunning && !calibrationInterrupted`: when the dispenser is alive and reports `Normal`, keep the timer running (start it if stopped); otherwise stop it. When it reaches `CALIBRATION_START_TIMEOUT_MS`: `calibrationInterrupted = true`, `calibrationRequested = false`. START on the confirm screen also resets `calibrationInterrupted` and stops the timer.

View for `CalibRunning`: `calibrationInterrupted` → `CalibInterrupted`; dispenser not alive → `CalibNoDispenser`; mode `Calibrating` → `CalibProgress`; `CalibrationDone` → `CalibDone`; `Refused` → `CalibRefused`; anything else → `CalibProgress` (0 % while starting). Presses: `CalibProgress` and `CalibNoDispenser` — short ignored, long cancels. `CalibDone`, `CalibRefused`, `CalibInterrupted` — any press. Cancel or dismiss = `calibrationRequested = false`, `calibrationInterrupted = false`, `screen = EditCalibration`.

Why: the dispenser now starts a run only on a request it has *seen* go from 0 to 1, so it never restarts one by itself after its own reboot, a link gap or a clog. Without this screen the tractor would sit at 0 % forever in those cases.

**e) Views and the button rule.** Add `enum class View : uint8_t { Menu, Work, WorkFault, EditDose, EditCalibration, CalibConfirm, CalibProgress, CalibDone, CalibRefused, CalibNoDispenser, CalibInterrupted, ClogAlert, ClogChoice, Unclogging };` and `View currentView()`: the clog overlay first (c), then by `screen` — `Work` is `WorkFault` while `faultTakesOverScreen()`, `CalibRunning` resolves as in (d). **Both the button handlers and `redraw()` go by the view, never by `screen` alone** — otherwise a press on the clog alarm would act on the screen underneath it.

In step 4 of the loop:

```cpp
View v = currentView();
if (v != shownView || (v == View::WorkFault && faultCode != shownFault)) {
    shownView    = v;
    shownFault   = faultCode;      // two different full-screen faults are two different screens
    shownSinceMs = now;
    viewGeneration++;
    displayDirty = true;
    if (v == View::ClogChoice) clogChoiceIndex = 0;   // Anuluj preselected every time it appears
}
```

In `readButton()`: at the debounced press-down store `pressGeneration = viewGeneration` and `pressStartMs = lastDebounceMs` (the last raw change, i.e. when the finger actually pressed). Before returning `Short` or `Long`, require `pressGeneration == viewGeneration && (int32_t)(pressStartMs - shownSinceMs) >= (int32_t)BUTTON_SCREEN_SETTLE_MS`. If that fails return `None`; for the long-press case still set `longAlreadyFired` so the release is swallowed too. An ignored press makes no beep. This covers both "the screen changed while the button was held" and "the press started just before a new screen appeared".

**f) Faults and buzzer.** Remove `FaultCode::DispenserStalled` and its drawing. `DispenserOverSpeed` stays: full screen on the work screen, buzzer on every screen. On `WorkFault` a short press does nothing; long press → menu. Buzzer: `faultCode != FaultCode::None || currentView() == View::ClogAlert`, same ¼ s pattern.

**g) Command.** `sendCommand()` also sets `upTimeMs = now`, `clogClearSeq`, `unclogSeq` (`uint8_t`, start at 0).

**h) Drawing.** Move the progress bar and percentage out of `drawCalibRunning()` into `drawProgress(const char *title, uint8_t percent)`, used by `CalibProgress` and `Unclogging`. New screens (numbers continue the Tractor screens list):

```
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│Kalibracja           │  │ZATKANIE!            │  │Zatkanie dozownika   │  │Odtykanie...         │
│                     │  │                     │  │                     │  │                     │
│PRZERWANA            │  │DOZOWNIKA            │  │[Anuluj             ]│  │ |######-----------| │
│                     │  │                     │  │                     │  │                     │
│Nacisnij aby wrocic  │  │[OK                 ]│  │ Odetkaj             │  │         35%         │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
   11. Interrupted          12. Clog alarm           13. Clog choice          14. Unclogging
```

| Screen | Exact drawing |
|---|---|
| 11 | size 1 `Kalibracja` at (0, 4); size 2 `PRZERWANA` at (0, 22); size 1 `Nacisnij aby wrocic` at (0, 54) |
| 12 | size 2 `ZATKANIE!` at (0, 4); size 2 `DOZOWNIKA` at (0, 22); size 2 `drawSelectableLine(46, "OK", true)` |
| 13 | size 1 `Zatkanie dozownika` at (0, 4); size 2 `drawSelectableLine(26, "Anuluj", index == 0)` and `drawSelectableLine(46, "Odetkaj", index == 1)` — the same grid as the calibration confirm screen |
| 14 | `drawProgress("Odtykanie...", dispenserData.progressPercent)` |
| 3 | only `DOZOWNIK` / `ZA SZYBKO` remains |

#### 1.6 Documentation (part of the task)

- **Tractor screens:** remove `BLOKADA` from screen 3 and change its short press to "nothing"; add screens 11–14 to the map, mockups and table (11 under 7–10 as another state of the running screen; 12–14 as "over any screen while the dispenser reports a clog": `12 ── OK ──> 13 ──Anuluj──> back to the screen underneath`, `13 ──Odetkaj──> 14 ──> 13`); add the button rule as a bullet at the top; the buzzer bullet gains the clog alarm.
- **New section `## Dispenser behaviour`** right after Tractor screens (and in Contents): the mode table and the counter rules from 1.4, condensed but with every transition kept, naming constants rather than copying values. Task 3's expectations rely on it.
- **Repo and build structure:** the three dispenser files.
- **Design record → Fail-safe rules:** rows for clog → motor off until the operator decides; tractor lost while unclogging → back to `Clogged`; a calibration request still raised after a dispenser reboot, link gap, clog or cancel → no run until the tractor lowers and raises it again; dispenser reports `Clogged` → tractor alarm over any screen.
- Move this task to Completed and delete this section (see How to work on a task, point 10).

#### 1.7 Traps

1. **The encoder counts up in both directions.** During unclogging the count keeps growing; don't use it for progress or direction.
2. **Counters must never act on boot or after a gap.** "First packet after boot and the counters differ, so act" is exactly the bug this design prevents — a rebooted dispenser would run the motor by itself.
3. **`calibrationArmed` is what stops a run restarting by itself** after a reboot, a link gap, a clog or a cancel. Don't simplify it away and don't set it `true` in `dispenserInit`.
4. **Handlers and drawing go by the view.** Dispatching on `screen` makes a press on the clog alarm act on the screen underneath.
5. **No overlay from stale data:** `clogActive` needs `links.isAlive(NodeId::Dispenser)`.
6. **`dispenser_logic.h` stays hardware-free.** Search it for `millis`, `micros`, `digital`, `ledc`, `Serial`, `Interrupt`, `static ` — comments aside, nothing may match except `static_assert` or `static constexpr` constants.
7. Deleting `CalibrationState` and `Stalled` breaks the tractor build until 1.5 is done. Expected — the finished task builds all three environments.
8. `updateFaults()` before `currentView()`, or the fault screen lags a loop behind and the button rule sees a stale view.
9. `motorBegin()` stays the very first call in the dispenser's `setup()`.
10. Keep `drawSelectableLine()` as it is; the new screens use its existing 20 px rows at y = 26 and y = 46 (a row at y = 50 would fall off the panel).

#### 1.8 Final verification

Go through every item; cite `file:line` for each.

- **P1** `PROTOCOL_VERSION == 4` with a v4 comment.
- **P2** `DispenserMode` with the six values 0–5 as in 1.2.
- **P3** `DispenserFault` is `None 0`, `NoSpeedData 1`, `OverSpeed 2`; `Stalled` and `CalibrationState` appear nowhere in `include/` or `src/` (search).
- **P4** `TractorCommand` field order as in 1.2, `static_assert == 24`; `DispenserStatus` has `mode` and `progressPercent`, `static_assert == 18`.
- **P5** `headerValid()` checks the sender against the type.
- **S1** No `STALL_` constant left in `include/` or `src/`. Clog and unclog constants and both `static_assert`s as in 1.3; `UNCLOG_TOTAL_MS` works out to 4000.
- **S2** `BUTTON_SCREEN_SETTLE_MS` and `CALIBRATION_START_TIMEOUT_MS` in the TRACTOR section.
- **D1** The three dispenser files exist with the exact public names from 1.4; the search from trap 6 finds nothing.
- **D2** `dispenserStep` returns `false` before `MOTOR_CONTROL_INTERVAL_MS` has passed; measured RPM is 64-bit and clamped.
- **D3** Counter handling: resync on `!tractorWasAlive` and on `upTimeMs` going down; `!=` comparisons; `tractorWasAlive` updated every step.
- **D4** `calibrationArmed`: `false` at init, set only by a received `calibrationRun == 0` from an alive tractor, cleared when a run starts or is refused.
- **D5** Every row of the mode table, one by one, including: metering keeps the last dose while the tractor is gone; `Clogged` starts the first unclog phase in the same step; unclogging stops on tractor loss and on `clearNow`.
- **D6** Clog check only in `Normal` (d) and `Calibrating`, exact formula, bool timer, entering `Clogged` gives duty 0 in that same step.
- **D7** Reversal guard present.
- **D8** Anti-windup condition exactly as in 1.4.
- **D9** `motorApply` writes duty 0 before changing DIR; the inbox uses one lock for write and read; `dispenserResetInbox` resets the `LinkTracker`.
- **D10** `main.cpp`: `motorBegin()` first; one-line callback without a cast; mode changes logged, nothing else per step.
- **T1** Receive snapshot under `rxLock`.
- **T2** `loop()` order as in 1.5 b.
- **T3** `View` enum and `currentView()`; handlers and `redraw()` use the view.
- **T4** Button rule in `readButton()`: generation and settle time checked for short and long; an ignored long press still swallows the release; no beep.
- **T5** Overlay condition, acknowledgement reset, calibration dropped when a clog appears on `CalibRunning`.
- **T6** Press actions of 12, 13, 14; `Anuluj` preselected each time 13 appears.
- **T7** Calibration views including `CalibInterrupted` and its timer; press actions as in 1.5 d.
- **T8** `DispenserStalled` gone; short press on the ZA SZYBKO screen does nothing; buzzer includes `ClogAlert`.
- **T9** `sendCommand()` fills `upTimeMs`, `clogClearSeq`, `unclogSeq`.
- **T10** Screens 11–14 drawn exactly as in 1.5 h; `drawProgress()` shared.
- **X1** Documentation updated as in 1.6, Contents and the task table updated, this section removed.
- **X2** Full build: tractor, seeder and dispenser succeed with no new warnings.

### Task 2: Tramline switch

**Goal.** The user usually seeds **without** tramlines. Add a saved on/off switch for tramlines, reached from a fourth main-menu item `Sciezki`. While it is off, the tramline relay never switches on and the work screen shows `Sciezki: WYL.` instead of the pass number.

**Files:** `src/tractor/main.cpp`, `CLAUDE.md`. Nothing else: the seeder already switches the relay by `tramlineRelayOn`, so **no protocol change** — do not edit `include/espnow_protocol.h`, `src/seeder/` or `src/dispenser/`. Only the tractor needs reflashing.

#### 2.1 Decisions agreed with the user

- The main menu becomes `Praca`, `Dawka`, `Kalibracja`, `Sciezki`, the new item last. Four items fit by removing the blank space between the rows.
- The switch lives on a screen opened from `Sciezki` (a menu row `Sciezki: WYL.` would be 13 characters, too long at text size 2).
- It starts **off** (no saved value = off). Flipping it saves straight away.
- While off, the work screen shows `Sciezki: WYL.` where `Przejazd:` and the pass number are.

#### 2.2 Behaviour and layout

- `static bool tramlinesEnabled`, loaded in `setup()` with `prefs.getBool("tram_on", false)`.
- `sendCommand()`: `command.tramlineRelayOn = tramlinesEnabled && ((TRAMLINE_ACTIVE_MASK >> tramlineNumber) & 1)`. `tramlineNumber` is still sent as it is.
- Flipping the switch never changes `tramlineNumber` — switching off and on again mid-field must not lose the pass.
- Work screen while off: short press does nothing (also on the full-screen fault); long press → menu as before.
- The yellow LED needs no change: it shows the relay state the seeder reports.

**Menu** (`MenuItem::Sciezki = 3`, `Count = 4`): rows 16 px apart, tops at y = 0, 16, 32, 48. Selected row: `fillRect(0, rowTop, 128, 16, WHITE)` and black text; text size 2 at `(2, rowTop + 1)`. The classic font's glyphs use 7 of their 8 rows, 14 px at size 2, so this leaves 1 px above and below, and none of the four words has a descender. Leave `drawSelectableLine()` as it is — the confirm screens still use its 20 px rows.

**Tramlines screen** (`Screen::Tramlines`, screen 15):

```
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│[Praca              ]│  │SCIEZKI              │  │RPM 3150             │
│ Dawka               │  │                     │  │6.4km/h    DOZ:40    │
│ Kalibracja          │  │       [WYL.]        │  │                     │
│ Sciezki             │  │                     │  │Sciezki: WYL.        │
│                     │  │            ZAPISZ   │  │BRAK: D              │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
        1. Menu              15. Tramlines          2. Work, tramlines off
```

- Opened by a long press on `Sciezki` in the menu, with the cursor on the switch. The menu stays on `Sciezki` afterwards, like the other items.
- Two fields: 0 = the switch, 1 = `ZAPISZ`. Short press moves the cursor between them.
- Long press on the switch: `tramlinesEnabled = !tramlinesEnabled;` then `prefs.putBool("tram_on", tramlinesEnabled);` straight away.
- Long press on `ZAPISZ`: back to the menu. As on the editors, it's the only way out (the value is already stored).
- Drawing: size 1 `SCIEZKI` at (0, 0). The value `WL.` or `WYL.` at size 3, y = 17, centred: `x = (128 − 18 × length) / 2`. Selected: `fillRect(x − 3, 14, 18 × length + 6, 28, WHITE)` with black text. `ZAPISZ` exactly like the editors: size 1 at (70, 52), selected box `fillRect(62, 50, 62, 12, WHITE)`.

**Work screen while off:** size 1 `Sciezki: WYL.` at (0, 34) and no large digit. While on: unchanged.

#### 2.3 If Task 1 is already done

Task 1 added `enum class View` and the button rule (look for `currentView()` in `src/tractor/main.cpp`). Then also: add `View::Tramlines`, map `Screen::Tramlines` to it in `currentView()`, handle it in the view-based button dispatch and in `redraw()`. The work-screen short-press change applies to `View::Work` (on `WorkFault` short press already does nothing).

#### 2.4 Documentation (part of the task)

- **Tractor screens:** map gets `└── Sciezki ──── 15 TRAMLINES`; replace mockup 1 with the four-row menu; add mockup 15 and the tramlines-off work screen; update table row 1 (four items) and row 2 (`Sciezki: WYL.`, short press ignored while off); if Task 1 isn't done yet, row 3's short press becomes "next pass while tramlines are on, nothing while off"; add row 15.
- **Future tasks → Statistics screen:** the menu is now full.
- Move this task to Completed and delete this section (How to work on a task, point 10).

#### 2.5 Traps

1. `MenuItem::Count` becomes 4; the menu's wrap-around uses it.
2. No protocol, seeder or dispenser change: compared with the starting `git diff --stat` (How to work on a task, point 7), only `src/tractor/main.cpp` and `CLAUDE.md` may have changed.
3. Don't reset the pass number when switching.
4. NVS keys are at most 15 characters (`tram_on` is fine). Write only on a long press, never in `loop()`.
5. At size 3 a line holds 7 characters; `WYL.` is 4.

#### 2.6 Final verification

- **R1** `MenuItem::Sciezki = 3`, `Count = 4`; menu short press cycles through four items.
- **R2** Menu drawing exactly as in 2.2; `drawSelectableLine()` unchanged.
- **R3** `Screen::Tramlines` opened by a long press on `Sciezki`, cursor on the switch.
- **R4** Short press moves the cursor; long press on the switch flips it and calls `putBool("tram_on", ...)` immediately; long press on `ZAPISZ` returns to the menu.
- **R5** Loaded with default `false`.
- **R6** `tramlineRelayOn` gated by the switch; `tramlineNumber` unchanged.
- **R7** Work screen while off: `Sciezki: WYL.` at (0, 34), no large digit; short press ignored there and on the fault screen; long press → menu.
- **R8** Flipping never touches `tramlineNumber`.
- **R9** Tramlines screen drawn exactly as in 2.2.
- **R10** If Task 1 is done: `View::Tramlines` handled in `currentView()`, the dispatch and `redraw()`.
- **R11** Compared with the starting `git diff --stat`, only the two allowed files changed.
- **R12** Documentation updated as in 2.4, Contents and the task table updated, this section removed.
- **R13** Full build succeeds with no new warnings.

### Task 3: Dispenser bench test

**Goal.** A separate firmware image, flashed onto the dispenser module on the workbench, that proves (1) the module's hardware works together — supply, MD13S, motor, both encoder channels, the direction line, the radio — and (2) the real dispenser control code does the right thing for every input the other two boards can send, including broken, missing, late and contradictory packets. The tractor and seeder are not needed; their packets are simulated.

**Depends on Task 1.** If `src/dispenser/dispenser_logic.h` doesn't exist, stop and tell the user.

**Files:** `platformio.ini` (new environment), new folder `src/dispenser_bench/`, `docs/dispenser_module_hardware.md`, `CLAUDE.md`. **Don't change `src/dispenser/*`.** If the bench can't be written without changing it, or a test expectation contradicts the code, check both against [Dispenser behaviour](#dispenser-behaviour) (added by Task 1) and ask the user. Never bend an expectation to match the code.

#### 3.1 Approach — decided, with reasons

- **A separate PlatformIO environment, `dispenser_bench`** — the user's proposal, and the right one: test code can never end up in the field image.
- **It runs the production code, not a copy.** It includes `src/dispenser/dispenser_logic.h` and compiles `src/dispenser/dispenser_io.cpp`. A bench that re-implements the logic proves nothing about what's flashed. Only `setup()`/`loop()`, radio reception and status sending differ from production.
- **The other boards are simulated at the receive boundary.** The bench builds packets with the real structs and `fillHeader()` and passes the raw bytes to `dispenserHandlePacket()` — the function the production receive callback calls — so validation, the inbox and link timing are tested as well. In industry terms: a hardware-in-the-loop bench test with rest-of-network simulation and fault injection.
- **Three kinds of test:**
  1. **Deterministic logic tests (D):** call `dispenserStep()` directly with made-up time, encoder counts and packets. No hardware, milliseconds per test, exact timing edges (slow for 1400 ms vs 1500 ms). These are unit tests; they run on the ESP32 because this PC has no host C++ compiler.
  2. **Packet tests (V):** through `dispenserHandlePacket()` with real time.
  3. **Hardware-in-the-loop (H, M, C, K):** real time, real motor and encoder, simulated packets, automatic PASS/FAIL from independently measured shaft speed, direction, mode and timing. A few steps need the operator.
- **Radio reception stays off:** the bench never registers a receive callback, so a real tractor or seeder nearby can't interfere and can't be commanded by the bench. The radio still transmits a heartbeat so its current draw is realistic while the motor runs (brownouts are this module's most likely failure): every 200 ms, 8 bytes starting `'B','T'`, which every real board drops at the magic-byte check.
- **Rejected:** `pio test`/Unity on the target (the test runner owns the serial port and can't prompt an operator, while half the value here is physical); host tests with `pio test -e native` (no host compiler installed, and no hardware tested — possible later); a second ESP32 simulating the other boards over the radio (needs another board; the radio code is the same as on the two boards already working); `#ifdef` test hooks in the production firmware.

#### 3.2 Safety rules — the bench drives a motor with a 5.5 A stall current

1. The motor moves only while a test that needs it runs. The idle menu keeps duty 0.
2. **Any key pressed while a motor test runs aborts it:** motor off at once, test marked `ABORTED`, back to the menu (an `a` run stops entirely). Operator prompts are the only time a key is an answer.
3. Every test ends through one function that sets duty 0 and direction forward, whatever the outcome.
4. Before the first motor test of a session, print this checklist and wait for `y`: motor clamped to the bench; coupling disconnected from the auger and hopper empty unless the test says otherwise; 12 V supply of at least 6 A through the 10 A fuse; encoder Vcc on 5 V (not 12 V).
5. **Stall tests use a lever, never fingers:** locking pliers clamped on the coupling with the handle resting against a fixed stop in the **forward** direction, fitted before the test starts, so the shaft can't move at all. Never run a reverse move with the lever fitted — prompt to remove it first.
6. The summary always ends with: `Reflash the production firmware before fitting the module: pio run -e dispenser -t upload`.

#### 3.3 Build environment and files

In `platformio.ini`, add the environment to `default_envs` (so a plain `pio run` proves the bench still compiles after dispenser changes) and add:

```ini
[env:dispenser_bench]
; Bench test image for the dispenser module (CLAUDE.md, Task 3). Never fit a module running this.
build_src_filter = +<dispenser_bench/*> +<dispenser/dispenser_io.cpp>
```

This filter was checked with a scratch project on this toolchain: it compiles the bench folder plus the shared file, and not `src/dispenser/main.cpp`.

```
src/dispenser_bench/main.cpp            setup/loop, menu, serial input, radio heartbeat, boot report
src/dispenser_bench/bench_report.h      PASS/FAIL/SKIP/ABORTED recording, measured values, summary
src/dispenser_bench/bench_inject.h      simulated seeder and tractor, packet builders
src/dispenser_bench/tests_logic.h       D tests
src/dispenser_bench/tests_packets.h     V tests
src/dispenser_bench/tests_hardware.h    H tests
src/dispenser_bench/tests_scenarios.h   M, C and K tests
```

Only `main.cpp` is a `.cpp` file; the headers are included once, from it. No subfolders. Include the shared code as `#include "../dispenser/dispenser_logic.h"` and `#include "../dispenser/dispenser_io.h"`.

Running it: `pio run -e dispenser_bench -t upload`, then `pio device monitor -e dispenser_bench` (115200 baud). Typed keys arrive one at a time; ignore `\r` and `\n`. The monitor doesn't reset the board on connect (`monitor_rts/dtr = 0`), so `?` must reprint the menu and the boot report.

#### 3.4 Program structure

- **Boot report:** banner `DISPENSER BENCH TEST - NOT FIELD FIRMWARE`; build date and time; reset reason from `esp_reset_reason()` — power-on, external pin and software are normal, anything else (brownout, panic, watchdogs) is printed as `FAIL H00` with a one-line explanation; key settings (pins, `ENCODER_EDGES_PER_REV`, `MOTOR_MAX_RPM`, `MOTOR_MIN_RUNNING_PERMILLE`, PWM frequency, the clog and unclog constants); ESP-NOW init result.
- **Interrupted-test marker:** a struct `{ uint32_t magic; char testId[8]; }` declared `RTC_NOINIT_ATTR`. Set it when a test starts, clear it when the test ends. A valid marker at boot means the board reset during that test: print `RESET DURING <id>` with the reset reason and count it as a FAIL. It survives software, panic, watchdog and most brownout resets, not a full power loss.
- **Menu:** `?` help, `d` logic tests, `v` packet tests, `h` hardware, `m` metering, `c` calibration, `k` clog and unclog, `a` all of these in that order, `r` report so far, `x` motor off.
- **Hardware in `setup()`:** `motorBegin()` first, then `encoderBegin()`. Channel B: `INPUT_PULLUP` and a bench-only `IRAM_ATTR` RISING interrupt that counts B edges and counts how many of them saw channel A high (`digitalRead(ENCODER_A_PIN)`). That ratio tells the direction; production never uses channel B.
- **Radio:** `WiFi.mode(WIFI_STA)`, disconnect, power save off, channel, `esp_now_init()`, broadcast peer — as in production, but **no `esp_now_register_recv_cb()`**. Heartbeat every 200 ms in the menu and during tests; count failed sends; any failure fails H07.
- **`runFor(ms)`** is the core of every hardware-in-the-loop test. Until the time is up, looping with `delay(1)`: check for the abort key; let the injector send due packets; call `dispenserControlTick(logic, now, out)`; send the heartbeat when due; print mode changes; record whatever the test is sampling. **Never print per control step** — serial writes block and would disturb the timing being tested. Print mode changes, results, and at most one status line per second.
- **Before each hardware-in-the-loop test:** motor off, `dispenserResetInbox()`, injector silent, `dispenserInit(logic, millis(), encoderEdges())`, wait 1000 ms for the shaft to stop.
- **Measuring the shaft independently:** checks use `encoderEdges()` deltas over a time window (RPM = edges × 60000 / (window ms × `ENCODER_EDGES_PER_REV`)), not the logic's own `measuredShaftRPM`. Direction = the share of B edges that saw A high in a window; forward and reverse must give opposite majorities.
- **Reporting:** one line per check, always with the measured and expected values, e.g. `[PASS] M02 mean 188 RPM, target 192 +-10%`. The summary lists the counts, the failed IDs, the measured values worth copying into `machine_settings.h` (edges per revolution, breakaway duty, RPM at full duty, which A level means forward) and the reflash reminder.
- **Operator prompts** say what to do and what will happen next. Answers are `y`/`n` or any key; `s` skips (counted as SKIP, never PASS).
- **Expectations are computed from the constants** where a formula is given, so a settings change doesn't break the bench. Expected shaft RPM uses an independent `double` formula — `speed[m/s] × 60 × (WORKING_WIDTH_CM / 100 × dose / 10) / (grams / 100)` — never `requiredShaftRPM()` itself.

#### 3.5 Injector (`bench_inject.h`)

One state per simulated board: sending on or off, interval (default `SEND_INTERVAL_MS`), next due time, and its fields.

- **Seeder:** `groundSpeedMmS`, `wheelTurning`; `upTimeMs` = bench time; `wheelPulses` advancing with speed and `WHEEL_MM_PER_PULSE`; `linkFlags` = tractor and dispenser heard.
- **Tractor:** `dispenserEnabled`, `doseKgPerHa`, `gramsPer100Rev`, `calibrationRun`, `clogClearSeq`, `unclogSeq`; `upTimeMs` = bench time + an offset a test can lower to fake a reboot; `linkFlags` = seeder and dispenser heard.
- Packets are built with the structs and `fillHeader()` and passed as bytes to `dispenserHandlePacket()`.
- A helper sends one malformed packet: copy a valid packet into a byte buffer, change a byte or the length.
- Jitter mode: the interval is drawn from [min, max] with a fixed-seed pseudo-random generator, so a failing run can be repeated.

#### 3.6 Test catalogue

Implement every row. In the D tests: a fresh `DispenserLogic` per test; time starts at 100000 ms; one step = time + `MOTOR_CONTROL_INTERVAL_MS` unless stated; **"shaft at R RPM"** adds `R × ENCODER_EDGES_PER_REV × stepMs / 60000` edges per step through a fractional accumulator (no truncation drift); **"defaults"** = seeder alive with telemetry (1000 mm/s, `wheelTurning` 1) and tractor alive with a command (enabled, 40 kg/ha, 500 g/100 rev, `calibrationRun` 0, counters 0, `upTimeMs` growing with time), which gives a target of 192 RPM; **"following"** = shaft at the previous step's target RPM; **"reach Clogged"** = defaults, following for 10 steps, then shaft 0 until the mode is `Clogged`; **"Calibrating for N steps"** = no seeder, `calibrationRun` 0 for one step, then 1, shaft at `CALIBRATION_RPM`, N steps after the mode became `Calibrating`; **"`Unclogging`"** as a starting point = reach Clogged, then `unclogSeq` +1.

**D — logic, no hardware (`d`)**

| ID | Stimulus | PASS when |
|---|---|---|
| D01 | `requiredShaftRPM` for (1000, 40, 500), (500, 40, 500), (1667, 40, 800), (2778, 40, 800), (65535, 999, 1), (1, 1, 99999), and each of the three inputs 0 | within 1 of the independent formula: 192, 96, 200, 333, 157126716, 0; 0 whenever an input is 0 |
| D02 | 30 steps each, shaft 0: (a) no packets; (b) command only; (c) telemetry at 1500 mm/s turning, no command; (d) defaults but enabled 0; (e) dose 0; (f) 0 g/100 rev; (g) speed 0 and `wheelTurning` 0; (h) `wheelTurning` 1, speed 0; (i) `wheelTurning` 0, speed 1500 | duty 0 at every step; mode `Normal`; fault `NoSpeedData` in (a) and (b), `None` otherwise |
| D03 | defaults, following, 30 steps | from step 10: `Normal`, fault `None`, target 192, forward, duty within ±30 of `192 × 1000 / MOTOR_MAX_RPM` |
| D04 | defaults at 3000 mm/s, shaft 300 RPM, 50 steps | target `MOTOR_MAX_RPM`, fault `OverSpeed`, duty 1000 from step 2, never `Clogged` |
| D05 | defaults, following 20 steps, then shaft 0 | `Normal` while `now − tSlow < CLOG_DETECT_MS` (tSlow = the first step that measured 0); `Clogged` at the first step with `now − tSlow ≥ CLOG_DETECT_MS`; that step outputs duty 0 forward, fault `None`, progress 0; `Clogged` with duty 0 for 50 more steps although the inputs still ask for 192 RPM |
| D06 | defaults, shaft at 69 RPM (36 %) for 50 steps; then a fresh logic with the shaft at 57 RPM (30 %) | 69 RPM: never `Clogged`. 57 RPM: `Clogged`, timed as in D05 |
| D07 | defaults, following 10 steps; then three times: shaft 0 for 14 steps, one step at 192 RPM | never `Clogged` |
| D08 | no seeder; `calibrationRun` 0 for one step, then 1; shaft 120 RPM for 10 steps, then 0 | `Calibrating` (target `CALIBRATION_RPM`), then `Clogged` timed as in D05; with `calibrationRun` still 1: `Clogged` for 30 steps; `clogClearSeq` +1 → `Normal` in that step; `calibrationRun` still 1: not `Calibrating` for 30 steps, duty 0; `calibrationRun` 0 for one step, then 1 → `Calibrating` |
| D09 | reach Clogged; `clogClearSeq` +1, following | `Normal` and duty 0 in that step; duty > 0 within 2 steps; fault `None` |
| D10 | reach Clogged; `unclogSeq` +1 at time S; 45 steps | at S: `Unclogging`, duty `UNCLOG_PERMILLE`, reverse. Every later step matches the phase of `t = now − S` from the mode table (duty-0 phases forward). First step with `t ≥ UNCLOG_TOTAL_MS`: `Clogged`, duty 0, forward. Progress never decreases and is < 100 while `Unclogging` |
| D11 | as D10 with step intervals alternating 100 ms and 170 ms | no step has duty > 0 in the opposite direction of an immediately preceding step with duty > 0; `Clogged` at the first step with `t ≥ UNCLOG_TOTAL_MS` |
| D12 | `Unclogging`; at the first step with t ≥ 1200 `clogClearSeq` +1 | `Normal` and duty 0 in that step |
| D13 | `Unclogging`; at the first step with t ≥ 500 tractor not alive | `Clogged` and duty 0 in that step; tractor alive again, counters unchanged, 30 steps: `Clogged`, duty 0 |
| D14 | reach Clogged; tractor not alive for 20 steps while `unclogSeq` +1; then alive | `Clogged` with duty 0 for 30 steps; then `unclogSeq` +1 → `Unclogging` |
| D15 | counters (3, 5) from the start, tractor alive throughout; reach Clogged at tractor `upTimeMs` ≈ 500000; then `upTimeMs` 1200 and counters (0, 0) | `Clogged` for 30 steps; then `unclogSeq` 0 → 1 with `upTimeMs` growing → `Unclogging` |
| D16 | fresh logic, first command has counters (7, 9); reach Clogged; `unclogSeq` 9 → 10 | nothing acted on at the start; `Unclogging` after the change; `clogClearSeq` 7 never acted on |
| D17 | defaults metering; both counters +1; 10 steps; then reach Clogged | `Normal`, duty > 0, forward during the 10 steps; then `Clogged` for 30 steps (the earlier changes are not replayed) |
| D18 | reach Clogged with `unclogSeq` 255; then 0 | `Unclogging` |
| D19 | reach Clogged; both counters +1 in the same step | `Normal` in that step; never `Unclogging` |
| D20 | no seeder; `calibrationRun` 0 for one step, then 1; shaft at 120 RPM | `Calibrating`, target `CALIBRATION_RPM`, forward, progress never decreasing; `CalibrationDone` at the first step where edges since the run started ≥ `CALIBRATION_REVOLUTIONS × ENCODER_EDGES_PER_REV`; that step duty 0, progress 100 |
| D21 | D20 continued with `calibrationRun` 1 for 50 steps, then 0 | `CalibrationDone` with duty 0 for 50 steps; `Normal` in the step with 0 |
| D22 | seeder alive, turning, 1000 mm/s; `calibrationRun` 0 for one step, then 1 | `Refused` in the step with 1; duty 0 while `Refused` (50 steps); `Normal` when `calibrationRun` returns to 0 |
| D23 | `Calibrating` for 20 steps; then seeder alive and turning | `Refused`, duty 0 in that step |
| D24 | `Calibrating` for 20 steps; then `calibrationRun` 0 | `Normal`, duty 0 in that step |
| D25 | `Calibrating` for 20 steps; tractor not alive | `Normal`, duty 0 in that step; tractor alive with `calibrationRun` still 1: `Normal` for 30 steps; 0 for one step, then 1 → `Calibrating` |
| D26 | fresh logic, first command has `calibrationRun` 1, no seeder | `Normal`, duty 0 for 30 steps; 0 for one step, then 1 → `Calibrating` |
| D27 | reach Clogged; `calibrationRun` 0 for one step, then 1 | `Clogged` for 30 steps |
| D28 | defaults at 3000 mm/s (target 330), shaft 200 RPM for 100 steps; then 500 mm/s (target 96), shaft 96 RPM | duty 1000 and never `Clogged` during the first part; within 2 steps of the change duty ≤ `96 × 1000 / MOTOR_MAX_RPM + 100` (fails without anti-windup) |
| D29 | 200 mm/s, 10 kg/ha, 1000 g (target 4), shaft 25 RPM for 600 steps; then defaults with shaft 0 for one step | in that step duty ≥ `min(1000, 192 × 1000 / MOTOR_MAX_RPM + MOTOR_KP × 192) − 50` (fails without anti-windup) |
| D30 | defaults, following; seeder not alive for 5 steps; alive again | fault `NoSpeedData` and duty 0 in the first step without the seeder; fault `None` and duty > 0 within 2 steps after it's back |
| D31 | defaults, following; tractor not alive for 100 steps, command contents unchanged | every step: `Normal`, fault `None`, target 192, duty > 0 |
| D32 | during D03, D04, D10 and D20 | `dispenserFillStatus()`: `mode` and `faultCode` as expected, `motorCommandedPermille` = output duty, `motorRunning` = duty > 0, `targetShaftRPM` and `progressPercent` as in that test |
| D33 | defaults, shaft 70 RPM for 80 steps (builds a large integral without a clog); then shaft 0 until `Clogged`; `clogClearSeq` +1; shaft 192 RPM | first step with duty > 0 after the clear: duty ≤ `192 × 1000 / MOTOR_MAX_RPM + 60` (the clog reset the integral) |
| D34 | 1 mm/s, 1 kg/ha, 99999 g (required 0), turning, 30 steps | duty 0, fault `None`, never `Clogged` |

**V — packet path, no motor (`v`).** Call `dispenserResetInbox()` before each case.

| ID | Stimulus | PASS when |
|---|---|---|
| V01 | valid `SeederTelemetry` | `haveTelemetry` and `seederAlive` true; fields read back equal the sent ones |
| V02 | valid `TractorCommand` | `haveCommand` and `tractorAlive` true; fields equal |
| V03 | `SeederTelemetry` broken one way at a time: magic0; magic1; version −1; version +1; network id +1; type `TractorCommand`; sender `Tractor`; `len` = size − 1; `len` = size + 1 (a buffer one byte longer); `len` = 8 (header only); `len` = 0; `len` = −1; null data pointer with the right `len` | never accepted: `haveTelemetry` and `seederAlive` false, `dispenserLinkFlags()` 0 |
| V04 | the same list for `TractorCommand` | never accepted |
| V05 | valid `DispenserStatus` (another dispenser) | no have-flag set, link flags 0 |
| V06 | 2000 packets of random length 0–64 with random bytes, fixed seed | none accepted; the board keeps responding |
| V07 | valid telemetry, then silence for `LINK_TIMEOUT_MS + 200` ms | `seederAlive` false while `haveTelemetry` stays true |
| V08 | valid telemetry and command | link flags = seeder and tractor bits; after `LINK_TIMEOUT_MS + 200` ms of silence, 0 |

**H — hardware (`h`)**

| ID | Stimulus | PASS when |
|---|---|---|
| H00 | boot | normal reset reason, no interrupted-test marker, ESP-NOW initialised |
| H01 | motor off 3 s, nobody touching the shaft | 0 new edges on A and on B |
| H02 | operator, coupling off: mark the output shaft, turn it by hand exactly 10 revolutions forward, press a key | A edges within ±3 % of `10 × ENCODER_EDGES_PER_REV`; B edges within ±3 % of A; print edges per revolution |
| H03 | free shaft: forward at 250, 500, 750, 1000 ‰, 1.5 s each | RPM over the last 1 s of each step never lower than the step before; RPM at 1000 ‰ ≥ 70 % of `MOTOR_MAX_RPM`; print the table |
| H04 | operator watches: forward 300 ‰ 1.5 s, off 0.7 s, reverse 300 ‰ 1.5 s | in each run ≥ 90 % of B edges agree on A's level and the two runs give opposite levels (DIR line and channel B work); operator answers `y` to "the first run turned the dispensing way" — `n` fails with "flip `MOTOR_DIR_FORWARD` or swap the motor leads"; print the forward level |
| H05 | prompt suggests the auger loaded: ramp forward from 0 by 10 ‰ every 300 ms up to 400 ‰, stopping at the first 300 ms window with ≥ 5 RPM; motor off 1 s; then `MOTOR_MIN_RUNNING_PERMILLE` from standstill for 1 s | a breakaway duty found at ≤ 400 ‰; the standstill start reaches ≥ 5 RPM over its last 500 ms. On failure suggest `MOTOR_MIN_RUNNING_PERMILLE` ≥ breakaway + 20 |
| H06 | operator, optional: lever fitted against the stop (3.2 rule 5); forward 1000 ‰ for 2 s | < 5 RPM measured (otherwise SKIP "shaft was not held"); the board did not reset; prompt to remove the lever |
| H07 | whole session | every heartbeat send returned `ESP_OK` |

**M — metering and links: motor, free shaft (`m`)**

| ID | Stimulus | PASS when |
|---|---|---|
| M01 | each D02 case (a)–(i) as packets, 3 s each | ≤ 2 edges after the first second; duty 0 throughout |
| M02 | seeder 1000 mm/s turning; tractor enabled, 40 kg/ha, 500 g | after 2 s, mean RPM over 1 s within ±10 % (at least ±5 RPM) of the expected 192; `Normal`, fault `None`; ≥ 90 % of B edges agree on one A level, and if H04 ran this session it is H04's forward level |
| M03 | speed 500 → 1500 → 500 mm/s, 3 s each | within 1.5 s of each change, RPM within ±15 % of the new target; never `Clogged` |
| M04 | 10 cycles: 1500 mm/s for 2 s, then 0 mm/s not turning for 2 s | never `Clogged`; duty 0 within 300 ms of the first stop packet; < 5 RPM within 1 s |
| M05 | 3000 mm/s for 5 s, then 1000 mm/s | fault `OverSpeed` within 300 ms; target `MOTOR_MAX_RPM`; never `Clogged`; fault `None` within 500 ms of going back |
| M06 | while metering: enabled 0 then 1; dose 40 → 80; 500 g → 1000 g | duty 0 within 300 ms of enabled 0, > 0 within 300 ms of enabled 1; target doubles, then halves (±1) within 300 ms |
| M07 | stop the seeder packets for 3 s, then resume | duty 0 and fault `NoSpeedData` within `LINK_TIMEOUT_MS + 300` ms of the last packet; metering again within 500 ms of resuming |
| M08 | stop the tractor packets for 5 s | RPM within ±10 % of target throughout; fault `None`; `Normal` |
| M09 | seeder intervals random 100–800 ms for 10 s; then one 1300 ms gap | duty never 0 during the jitter; duty 0 during the gap; metering again after it |
| M10 | 200 mm/s, 10 kg/ha, 1000 g (target 4) for 20 s; then 1500 mm/s, 40 kg/ha, 500 g (target 288) | during the low target never `Clogged`, duty always 0 or ≥ `MOTOR_MIN_RUNNING_PERMILLE`; after the change ≥ 50 % of 288 RPM within 1 s |

**C — calibration run: motor, free shaft (`c`)**

| ID | Stimulus | PASS when |
|---|---|---|
| C01 | no seeder packets; tractor `calibrationRun` 0 for 1 s, then 1 | `Calibrating` within 300 ms; RPM within ±15 % of `CALIBRATION_RPM` after 2 s; progress never decreases; `CalibrationDone` within `1.3 × CALIBRATION_REVOLUTIONS × 60000 / CALIBRATION_RPM` ms (65 s today); edges since the start, counted 1 s after `Done`, between 100 % and 101 % of `CALIBRATION_REVOLUTIONS × ENCODER_EDGES_PER_REV`; progress 100 |
| C02 | C01 continued with `calibrationRun` 1 for 3 s, then 0 | `CalibrationDone` with duty 0 while 1; `Normal` within 300 ms of 0 |
| C03 | start a run; at ≥ 20 % progress `calibrationRun` 0 | duty 0 within 300 ms; `Normal` |
| C04 | seeder at 1000 mm/s turning; `calibrationRun` 0 → 1 | `Refused` within 300 ms; ≤ 2 edges over 3 s |
| C05 | run for 5 s; then the seeder reports turning | `Refused` and duty 0 within 300 ms |
| C06 | run for 5 s; stop the tractor packets for 2 s; resume with `calibrationRun` 1 | duty 0 within `LINK_TIMEOUT_MS + 300` ms; `Normal`; not `Calibrating` for 3 s after resuming; 0 for 1 s, then 1 → `Calibrating` |

**K — clog and unclogging: motor (`k`).** K01–K06 simulate the clog by detaching the channel-A interrupt with `encoderEnd()`: the logic sees a stopped shaft while the motor keeps turning, and channel B proves it really was turning. **"Clog as in K01"** = meter as in M02, `encoderEnd()`, wait for `Clogged`, then `encoderBegin()` again — always re-attach, or every later edge check passes without measuring anything.

| ID | Stimulus | PASS when |
|---|---|---|
| K01 | metering as in M02; `encoderEnd()` | `Clogged` between `CLOG_DETECT_MS` and `CLOG_DETECT_MS + 400` ms after `encoderEnd()`; duty 0 from then on; B edges kept increasing between `encoderEnd()` and `Clogged` (the shaft really turned); after `encoderBegin()` still `Clogged` with duty 0 for 3 s, metering inputs unchanged |
| K02 | K01 continued: `clogClearSeq` +1 | `Normal` within 300 ms; ≥ 50 % of target RPM within 1 s |
| K03 | clog as in K01; `unclogSeq` +1 | `Unclogging` within 300 ms; in the middle 400 ms of every move, ≥ 80 % of B edges agree on one A level; the moves the logic drives in reverse all give one level, the forward moves the other level (and if H04 ran this session, forward matches H04); `Clogged` again at `UNCLOG_TOTAL_MS` ±300 ms; duty 0 and forward afterwards |
| K04 | calibration run (no seeder) for 5 s; `encoderEnd()` | `Clogged` within `CLOG_DETECT_MS + 400` ms; after `encoderBegin()` and `clogClearSeq` +1: `Normal`; `calibrationRun` still 1 → not `Calibrating` and ≤ 2 edges for 3 s |
| K05 | tractor counters (1, 1) from the start; clog as in K01; then tractor `upTimeMs` 1500 and counters (0, 0) | `Clogged`, ≤ 2 edges over 3 s |
| K06 | clog as in K01; stop the tractor packets for 2 s; resume with `unclogSeq` +1 | `Clogged` and ≤ 2 edges for 3 s; another +1 with packets flowing → `Unclogging` |
| K07 | operator, real stall: lever fitted against the stop; metering for 120 RPM (1000 mm/s, 25 kg/ha, 500 g) | `Clogged` within `CLOG_DETECT_MS + 700` ms of starting; duty 0; the board did not reset; prompt to remove the lever before anything else; `clogClearSeq` +1 → `Normal` |

#### 3.7 Documentation (part of the task)

- `docs/dispenser_module_hardware.md`: a new section "Bench test" — how to flash and open the monitor, the setup checklist from 3.2, the recommended order (`d` and `v` with nothing connected; `h` with the coupling off; then `m`, `c`, `k`), which measured values go into `machine_settings.h`, and the reflash reminder. Point to it from section 4 "Before powering up".
- CLAUDE.md: the bench files and the `dispenser_bench` environment in Repo and build structure (plus its upload and monitor commands); a one-line pointer in Hardware and pins → Dispenser; Future tasks → Hardware verification says the dispenser part is covered by the bench test.
- Move this task to Completed and delete this section (How to work on a task, point 10).

#### 3.8 Traps

1. **Don't copy dispenser logic into the bench.** Search `src/dispenser_bench/` for `requiredShaftRPM`, `feedForward`, `CLOG_MIN_SPEED_PERCENT`, `UNCLOG_REVERSE_MS`: only calls into the shared code, printing, and the independent expectation formulas may match.
2. Exactly one `.cpp` in `src/dispenser_bench/`.
3. No `esp_now_register_recv_cb` anywhere in the bench.
4. D tests build `DispenserInputs` themselves and never touch hardware.
5. Reset the inbox and the logic before each hardware test and wait for the shaft to stop, or one test's packets and motion leak into the next.
6. The tolerances follow from the latency budget — packets every 200 ms, a control step every 100 ms, a 1000 ms link timeout. Don't tighten them. If one looks wrong, ask.
7. No printing in the per-step path.
8. The abort key must work in every motor phase, including the H tests' own loops — route them through the same check.
9. The encoder counts up in both directions; direction comes only from channel B.
10. Every operator step can be skipped with `s`, and a skipped step is SKIP, never PASS.

#### 3.9 Final verification

- **B1** `dispenser_bench` environment with exactly the `build_src_filter` from 3.3, listed in `default_envs`.
- **B2** Files as in 3.3; only `main.cpp` is a `.cpp`; no subfolders.
- **B3** Trap 1 and trap 3 searches clean; `src/dispenser/*` not changed in this session (compare with the starting `git diff --stat`).
- **B4** `motorBegin()` first in `setup()`; duty 0 in the menu; one safe-stop function used on every exit path.
- **B5** Abort on any key in every motor loop; `ABORTED` recorded.
- **B6** Setup checklist confirmation before the first motor test of a session.
- **B7** Boot report with reset reason and the `RTC_NOINIT_ATTR` marker.
- **B8** Heartbeat: 8 bytes starting `'B','T'`, every 200 ms, failures counted.
- **B9** Injector sends through `dispenserHandlePacket()`, built with `fillHeader()`; fixed-seed jitter.
- **B10** Channel B interrupt and direction measurement; shaft RPM from `encoderEdges()`.
- **B11** Go through the catalogue row by row: D01–D34, V01–V08, H00–H07, M01–M10, C01–C06, K01–K07 each implemented with the stimulus and PASS condition of its row.
- **B12** Report lines show measured and expected values; the summary has the counts, failed IDs, suggested settings values and the reflash reminder.
- **B13** Documentation updated as in 3.7, Contents and the task table updated, this section removed.
- **B14** Full build: tractor, seeder, dispenser and dispenser_bench succeed with no new warnings.

## Future tasks

- **Hardware verification before field use.** Nothing has run on the machine yet.
  - Dispenser module on the bench: [Task 3](#task-3-dispenser-bench-test).
  - All three boards on a desk: pull power from each in turn — the seeder holds its relay, the dispenser stops when the seeder goes, the right `BRAK:` letters appear. With a fault beeping, cut the seeder's power: the buzzer must stop (regression check for the old stuck-buzzer bug).
  - `WHEEL_MM_PER_PULSE`: push the seeder along a measured 100 m and count `wheelPulses`; the value is 100000 / pulses. Use 100 m, not 20 m: 20 m is only ~32 pulses, so one pulse either way is a 3 % error. Repeat at two seed-rate settings to confirm the sensor really is before the gearbox.
  - Dispenser calibration: check `ENCODER_EDGES_PER_REV` by hand-turning 10 revolutions (≈ 4800 edges), then `Kalibracja` → `TEST`, catch and weigh the output, enter it.
  - Blockage: stall the running dispenser; the tractor must alarm.
- **Auger output per revolution** — mechanical, not firmware: 40 kg/ha at 10 km/h needs roughly ≥ 800 g per 100 revolutions (Design record → Dispenser). Firmware clamps and alarms either way.
- **Statistics screen** — agreed with the user, not designed in detail: hectares, kg applied, average kg/ha, wheel pulses, dispenser revolutions, and `ZERUJ` with a confirmation. The main menu holds four rows at text size 2, so once `Sciezki` is added a fifth item needs scrolling. The encoder counts reverse turns too, so leave `Unclogging` out of the revolution total.
- **Task watchdog** on all three boards — planned in the fail-safe design, never added.
- **README is out of date** — it still says to write MAC addresses into the source and describes two boards.
- **Real WOM (power take-off) RPM sensing** — hardcoded to `540` in `src/seeder/main.cpp` (`telemetry.womRPM = 540`). The WOM alarms in `updateFaults()` stay dead code (`ENABLE_WOM_ALARM` is `false`) until a real sensor exists.
- **Wiring diagram** and **demonstration video** — README TODOs.
- **Host-side unit tests (optional)** — `src/seeder/wheel_speed.h` (and `dispenser_logic.h` after Task 1) have no hardware code, so they could run under `pio test -e native`. That needs a host C++ compiler such as MinGW-w64, which this PC doesn't have.

## Completed

Everything below is written and builds with `pio run` (three environments, zero errors). **None of it has run on the physical machine yet** — see Future tasks → Hardware verification.

- ESP-NOW link between the boards; turbine RPM and ground-wheel sensing on the seeder; tramline relay with manual pass selection; OLED, LEDs and buzzer alarms on the tractor.
- Migration from two Arduino IDE sketches to one PlatformIO project with an environment per board and a shared protocol header.
- **Protocol v2/v3** — broadcast addressing (no MACs), magic/version/network-id validation, length-checked receives, per-peer link tracking with second-hand `linkFlags`; v3 added dispenser on/off and the automated calibration run.
- **Firmware hardening** — all display/buzzer/LED/relay work moved out of the ESP-NOW receive callback into `loop()`, `enum` fault codes instead of `String`, monotonic debounced pulse counters, the fail-safe rules below.
- **Ground speed** from the existing ground-wheel sensor: the average of the last 6 gaps between pulses, lowered as soon as a pulse is late, 0 after 2 s without one (`src/seeder/wheel_speed.h`).
- **Settings file** — every machine value in `include/machine_settings.h`, grouped by board.
- **Single-button menu** on the tractor (`Praca` / `Dawka` / `Kalibracja`) with NVS persistence and the automated 100-revolution calibration run.
- **Dispenser firmware** — MD13S drive, encoder feedback, feed-forward + PI rate control, stall and over-speed alarms.

### Design record

Condensed from the Implementation_Plan that produced the firmware above (its phases 0–5 are all done). It keeps the reasons behind decisions so they don't get re-proposed; the code is the reference for details.

#### Review findings that were fixed (Implementation_Plan §1)

| | Problem in the original two-board firmware | Fix |
|---|---|---|
| F1 | Display, buzzer and LEDs driven from the ESP-NOW receive callback: stalled packet reception, raced `loop()` on the I2C bus, and the buzzer could stay on when packets stopped mid-beep | callback only validates, copies and timestamps; all output work in `loop()`, serviced every iteration |
| F2 | `memcpy` without checking length or type | `headerValid()`: magic, version, network id, type, exact length |
| F3 | no fail-safe on link loss | fail-safe rules below |
| F4 | pulse counters read then reset, losing pulses in between | ISR counters only increment; readers subtract snapshots |
| F5 | Arduino `String` for state (heap churn all day) | `enum class` |
| F6 | faults computed after drawing, so one packet late | fixed by ordering in `loop()` |
| F7 | RPM maths divided before multiplying | multiply first, divide by measured elapsed time, pulses-per-rev constant |
| F8 | cast on the receive callback hid signature changes | cast removed, the compiler checks it |
| F9 | shadowed global variable | removed |
| F10 | tramline rhythm and active passes hardcoded | named constants |
| F11 | WiFi channel not pinned, power save on | fixed channel, power save off, identical on every board |
| F12 | (not a bug) sending every 200 ms even when nothing changes | kept on purpose: the periodic packet is the heartbeat that makes silence mean "board gone" |

#### Addressing and link status (§2)

- Everything goes to the broadcast address with the sender in the header, so any board can be swapped without touching firmware anywhere. MAC pairing and ESP-NOW encryption were rejected (see below).
- A peer is alive if a valid packet from it arrived within `LINK_TIMEOUT_MS` (1000 ms = 5 missed packets). That is better evidence than the old unicast ACK: it proves the other board's loop runs, not just its radio.
- Each packet's `linkFlags` byte says whom the sender hears, which is how the tractor learns whether the seeder and dispenser hear each other.
- `NETWORK_ID` separates two machines with this firmware working side by side. A board never receives its own broadcasts, and filtering by sender makes that irrelevant anyway.

#### Protocol (§3)

- One 8-byte header plus one packed payload struct per message type, fixed-width fields, a `static_assert` on every size, names spelled out (`PROTOCOL_*`, not `PROTO_*`).
- Magic bytes `'K','S'` drop unrelated ESP-NOW traffic; the version drops out-of-step firmware; the network id drops the neighbour's machine.
- Counters on the wire are cumulative (`wheelPulses`), so a lost packet loses no counts: the next one gives the right difference.
- The ground-wheel sensor sits before the seed-rate gearbox (confirmed), so `WHEEL_MM_PER_PULSE` is a true constant — measured directly with a roll-out, not computed from diameter, ratio and magnets. Lifting the seeder stops the wheel, so speed goes to 0 and the dispenser stops without any extra sensor.

#### Fail-safe rules (§5)

Governing principle, agreed with the user: **a gap in the field is a permanent defect**, so an actuator holds its last command on link loss unless holding would be meaningless or unsafe.

| Board | Condition | Behaviour |
|---|---|---|
| Seeder | boot, before any command | relay off |
| Seeder | tractor not heard, for any time | holds the last relay state — dropping it mid-pass would leave an unmarked gap in the tramline |
| Dispenser | boot | PWM 0 before anything else in `setup()`; the pull-down resistors cover reset and boot before that |
| Dispenser | seeder not heard for `LINK_TIMEOUT_MS` | motor off, fault `NoSpeedData` — without ground speed any rate is a guess |
| Dispenser | tractor not heard | keeps metering with the last dose and calibration (an unfertilised strip is a permanent defect); the speed interlock still applies |
| Dispenser | wheel not turning, speed 0, dispenser off or dose 0 | motor off — covers headland turns, lifting and standing still |
| Dispenser | calibration run and the tractor is lost | run aborts, motor off — the tractor commanded it |
| Dispenser | calibration run and the seeder reports the wheel turning | refused or stopped, motor off |
| Tractor | a peer not heard for `LINK_TIMEOUT_MS` | blue LED blinks, `BRAK:` letters on the work screen, no buzzer yet |
| Tractor | a peer that had been heard stays silent `LINK_BUZZER_DELAY_MS` longer, or seeder and dispenser can't hear each other | buzzer — timed from the loss of contact, so powering the tractor up first stays silent |

#### Tractor UI (§6)

- One existing button and no hardware change; a phone/SoftAP configuration mode was rejected (see below).
- A long press fires at 1.5 s while still held, confirmed by a short beep, so it works with gloves and without looking; the release is swallowed. A screen changes only on a press or because another board reported something — there are no idle timeouts.
- Dose, calibration and dispenser on/off are stored with `Preferences` in the NVS partition of the ESP32's flash: no battery; retention in the order of 10–20 years; ~100,000 erase cycles per sector with wear levelling, written only on an operator action. Only a full-chip erase or a partition-table change clears them — a normal upload keeps them.
- Values that never change in service (working width, tramline rhythm, `WHEEL_MM_PER_PULSE`) stay compile-time constants. The tractor broadcasts dose and calibration in every packet, so the dispenser stores nothing and never needs reflashing when they change.
- Calibration run: the request is level-triggered (survives lost packets without an acknowledgement protocol); it bypasses the ground-speed interlock because it's a stationary job and the seeder may be off, but is refused while the machine moves; tractor loss aborts it; a short press can't stop it (that would spoil the weighing).

#### Dispenser (§7)

- **Topology:** the dispenser computes its own motor setpoint from `SeederTelemetry` and `TractorCommand`, so a tractor-module failure doesn't stop metering, and the control loop sits next to the motor.
- **Rate maths** with working width W m, dose D kg/ha and calibration C g per 100 output-shaft revolutions: grams per metre = W × D / 10; revolutions per metre = (W × D / 10) / (C / 100); shaft RPM = 60 × speed [m/s] × revolutions per metre. At 4 m and 40 kg/ha that's 16 g per metre.
- **The motor's 330 RPM caps the working speed:** v_max [m/s] = 330 × C / (2400 × D) at 4 m. At 40 kg/ha: C = 300 → 3.7 km/h, 500 → 6.2 km/h, 800 → 9.9 km/h, 1000 → 12.4 km/h. So the auger must deliver about ≥ 800 g per 100 revolutions for 10 km/h. That is decided by auger geometry; firmware's job is to make the shortfall loud (clamp and raise `ZA SZYBKO`), not to hide it.
- **Encoder:** channel A rising edges only, 480 per output revolution, ~2.6 kHz at full speed; channel B is wired but unused (the ESP32's PCNT peripheral is the upgrade path if interrupts ever become a problem).
- **Control:** feed-forward from the rate maths does the work; a gentle PI trim corrects for battery sag and auger load. The duty is either 0 or at least `MOTOR_MIN_RUNNING_PERMILLE`, never in the buzzing band where the motor doesn't turn.
- **Power:** tractor 12 V is dirty (load dumps, alternator noise) and the motor draws up to 5.5 A on stall next to a microcontroller. Brownouts are the most likely field failure, and they are a wiring problem, not a firmware one — see `docs/dispenser_module_hardware.md`.

#### Rejected ideas (§8)

1. **Sending only on change** — kills the heartbeat; silence would no longer mean "board gone", so link-loss fail-safes become impossible.
2. **MQTT / HTTP / cloud between boards** — there is no router in a field.
3. **ESP-NOW encryption** — incompatible with broadcast addressing; the threat (someone in the field with an ESP32 and this repo) doesn't justify losing zero-reflash board swaps.
4. **Phone / SoftAP configuration mode** — only existed to type two numbers; would need radio coexistence, an HTTP server and a web page, and phones distrust networks without internet. The button menu does it with existing hardware.
5. **Automatic MAC pairing** — a pairing state machine and stored state that goes stale, i.e. brings back the manual re-pairing step broadcast removes.
6. **Receivers computing speed from raw pulses** — would copy `WHEEL_MM_PER_PULSE` to every board; the seeder owns the sensor and publishes finished speed (raw pulses are still sent).
7. **Per-interval pulse counts on the wire** — a dropped packet loses counts for good; cumulative counters heal themselves.
8. **The tractor computing the dispenser setpoint** — adds a hop and makes metering depend on the tractor module.
9. **One shared struct for all messages** — every field change would force all boards to update in lockstep.
10. **FreeRTOS tasks or an event framework** — `loop()` with timestamps is enough at 5 Hz; the one real concurrency bug (F1) was fixed by removing work from another context.
11. **A second seeder sensor for ground speed** — the fitted ground-wheel sensor already measures distance.
12. **An open-loop dispenser as an interim step** — the motor's built-in encoder makes closed loop available from day one.
13. **Full quadrature decoding in production** — one direction while metering; channel A gives far more resolution than needed at half the interrupt load.
14. **Deriving `WHEEL_MM_PER_PULSE` from wheel diameter, ratio and magnets** — three measurements to get wrong instead of one roll-out.
15. **Mesh relaying between modules** — solves a failure never seen at a few metres; `linkFlags` makes the assumption visible instead.
