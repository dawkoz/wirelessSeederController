# CLAUDE.md

Guidance for working on this repo. Read this before making changes.

## Contents

1. [What this project is](#what-this-project-is)
2. [Philosophy](#philosophy)
3. [Important notes](#important-notes)
4. [Repo and build structure](#repo-and-build-structure)
5. [Hardware and pins](#hardware-and-pins)
6. [Tractor screens](#tractor-screens)
7. [Dispenser behaviour](#dispenser-behaviour)
8. [Current tasks](#current-tasks)
   - [How to work on a task](#how-to-work-on-a-task)
9. [Future tasks](#future-tasks)
10. [Completed](#completed)
    - [Design record](#design-record)

## What this project is

A wireless controller for a Kverneland Accord-style seeder, mimicking the Kverneland FGS Rhythmus functionality. Three ESP32 modules talk to each other over **ESP-NOW** (not WiFi/MQTT — no router, no internet, no phone app):

- **Seeder module** (on the seeder): turbine RPM sensor, ground-wheel sensor (ground speed and "is the seeder moving"), tramline relay.
- **Tractor module** (in the cab): OLED display, one button that drives every screen, LEDs and buzzer. Owns the tramline selection and the dispenser settings.
- **Dispenser module** (on the seeder): fertilizer dispenser motor driven through a Cytron motor driver, metered from the seeder's ground speed. A separate board for one reason only — the seeder's enclosure has no room left. It gets its own 12 V power cable; everything else is wireless like the other two boards. **Built and bench-tested, not yet fitted to the machine.**

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
- ESP-NOW runs on a fixed `ESPNOW_CHANNEL` (1) with power save disabled, set identically on all three boards in `setup()`. `encrypt` is `false` — required, since ESP-NOW encryption needs per-peer keys and is incompatible with broadcast addressing (see Rejected ideas). The registered peer uses `channel = 0` ("current radio channel") deliberately: naming the channel on both the radio and the peer creates a way for them to disagree, and a mismatch makes _every_ `esp_now_send()` fail. Send failures are counted and logged on serial rather than discarded.
- **Any change to `include/espnow_protocol.h` means reflashing all three boards.** Bump `PROTOCOL_VERSION` when you do — receivers drop mismatched packets, so a half-updated set of boards fails loudly instead of quietly misreading each other. The `static_assert`s on every struct turn an accidental layout change into a build error.
- **The dispenser writes PWM 0 before anything else in `setup()`** — keep it the first thing. The 10 kΩ pull-downs on the driver inputs cover the time before that (reset and boot).
- **The tractor calls `Wire.setClock(400000)` after `oled.begin()` — leave it there.** The vendored SH1106 library set the same speed via the AVR `TWBR` register, which had to be removed for the ESP32 port. Without it the bus runs at Arduino's default 100 kHz, a full redraw takes ~103 ms instead of ~26 ms, and since the button is polled once per `loop()` it starts dropping presses.
- The Adafruit_SH1106 library used by the tractor board is the older community library (class `Adafruit_SH1106`, `begin(SH1106_SWITCHCAPVCC, addr)` API), originally from [wonho-maker/Adafruit_SH1106](https://github.com/wonho-maker/Adafruit_SH1106) — not Adafruit's newer `Adafruit_SH110X` library, which has a different, incompatible API. It's **vendored** (not pulled via `lib_deps`) at `lib/Adafruit_SH1106/`, with three small ESP32-portability patches, because upstream targets AVR only. Two are on the SPI path this project doesn't use; the third removed the AVR I2C clock setting, which is why the `Wire.setClock` call above exists. Details: `lib/Adafruit_SH1106/README.md`.

## Repo and build structure

This is a **single PlatformIO project** (not Arduino IDE, not separate per-board projects) with **one build environment per physical board**:

```
platformio.ini              defines envs: tractor, seeder, dispenser, dispenser_bench
include/espnow_protocol.h   message structs — the shared wire protocol, single source of truth
include/machine_settings.h  every machine value (pins, calibration, timings, alarms), grouped by board
src/tractor/main.cpp        tractor board firmware
src/seeder/main.cpp         seeder board firmware
src/seeder/wheel_speed.h    ground speed from wheel pulse timing (no hardware code, so it can be tested on a PC)
src/dispenser/main.cpp      dispenser board firmware - thin: radio init, receive callback, status broadcast
src/dispenser/dispenser_logic.h   the dispenser's decision logic, header-only and hardware-free (see Dispenser behaviour)
src/dispenser/dispenser_io.h/.cpp encoder, motor output and the packet inbox - the only dispenser code that touches pins
src/dispenser_bench/        bench test image for the dispenser module - its own environment, never fitted (see docs/dispenser_module_hardware.md)
docs/dispenser_module_hardware.md   dispenser parts list, pin verification, wiring, bench test
docs/calibration-settings.txt       what the tractor has stored, exported over USB - the backup for an erased or replaced board
```

Why one project with multiple envs, instead of three separate PlatformIO projects: it lets `include/espnow_protocol.h` be physically the same file for every board that needs it, so the protocol can't silently drift between boards the way it could with copy-pasted struct definitions (which is how this repo worked before the PlatformIO migration — each `.ino` had its own copy).

**To build/upload a specific board:**

```bash
pio run -e tractor -t upload
pio run -e seeder -t upload
pio run -e dispenser -t upload
```

The bench test image is a fourth environment, uploaded to the dispenser module on the workbench only:

```bash
pio run -e dispenser_bench -t upload
pio device monitor -e dispenser_bench
```

Or in VS Code: pick the environment from the PlatformIO status bar at the bottom, or the Project Tasks tree in the PlatformIO sidebar. On the development PC `pio` is not on PATH from a plain shell — see [How to work on a task](#how-to-work-on-a-task) for the full path.

**Platform version is pinned** in `platformio.ini` (`espressif32@7.0.1`, Arduino-ESP32 core 2.0.17) deliberately. Arduino-ESP32 3.x (ESP-IDF 5) changes the ESP-NOW receive-callback signature (`OnDataRecv` gains an `esp_now_recv_info_t*` parameter) — bumping past the 2.x line will break the builds until the callbacks are rewritten. Don't bump this without checking ESP-NOW API changes first. The core compiles C++ as **`-std=gnu++11`** (GCC 8.4).

## Hardware and pins

All pin numbers below are set in `include/machine_settings.h`.

### Seeder ESP32 (`src/seeder/main.cpp`)

| Pin | Function                                                |
| --- | ------------------------------------------------------- |
| 12  | Relay control (tramline), active LOW                    |
| 14  | Turbine inductive sensor (interrupt, RISING)            |
| 27  | Metering-drive Hall sensor, 3 magnets (6 being fitted)  |

The pin 27 sensor is on the **metering drive, not on the ground wheel** (found on the machine on 18 September 2026). The two do not turn at the same rate: about 0.4 of a metering turn per ground-wheel turn for small seeds and a little more for large ones, selected by the machine's own two-speed gear — and confirmed independent of the seed-rate setting. **6 magnets**, so a pulse is roughly 0.8 m: about 0.35 s apart at 8 km/h but 1.4 s at 2 km/h, which is why "stopped" is a speed (`WHEEL_MIN_SPEED_MM_S`) rather than a fixed timeout, and why the speed is averaged over a whole turn of the drive. The distance per pulse is therefore **one measured value per gear**, kept on the tractor and sent in every packet (`TractorCommand.wheelMmPerPulse`); the operator measures each one by driving 100 m on the `Nasiona` screens. It is also the source for "is the seeder moving".

### Tractor ESP32 (`src/tractor/main.cpp`)

| Pin | Function                       |
| --- | ------------------------------ |
| 12  | The button (to GND)            |
| 14  | Green LED (connected)          |
| 27  | Blue LED (connecting/blinking) |
| 13  | Yellow LED (tramline active)   |
| 19  | Buzzer                         |

OLED: SH1106 128x64 over I2C, address `0x3C`. The single button drives every screen (see [Tractor screens](#tractor-screens)): short press < 1500 ms, long press fires _at_ 1500 ms while still held.

### Dispenser ESP32 (`src/dispenser/main.cpp`)

Built and bench-tested, not yet fitted to the machine. **Cytron MD13S** driver (PWM + DIR, 13 A continuous) and a **Pololu 4752** motor (37Dx68L, 30:1, 12 V, 330 RPM, 14 kg·cm, 5.5 A stall) with a built-in quadrature encoder (64 CPR motor shaft → 1920 CPR output shaft). Powered by its own 12 V line; control/telemetry wireless like the other two boards.

| Pin | Function                                                     |
| --- | ------------------------------------------------------------ |
| 25  | MD13S PWM (LEDC, 16 kHz)                                     |
| 26  | MD13S DIR                                                    |
| 32  | Encoder channel A (interrupt, RISING)                        |
| 33  | Encoder channel B (wired, unused by the production firmware) |

`ENCODER_EDGES_PER_REV` is set to 480 — confirmed against Pololu's documentation: "64 CPR" counts both edges of both channels, so one channel's rising edges give 64 ÷ 4 = 16 per motor revolution, × 30:1 = 480 per output revolution. Still **verify by hand-turning the output shaft 10 revolutions** before trusting the rate control. Counting one channel cannot tell direction: the count goes **up** whichever way the shaft turns.

MD13S accepts 3.3 V logic directly, so PWM/DIR need no level shifter. Its PWM limit is 20 kHz; firmware runs at 16 kHz for margin.

Pins verified against the Espressif GPIO reference: none are strapping pins, none touch the SPI flash, all support interrupts and internal pull-ups, and none output PWM during boot the way GPIO 0/5/14/15 do. GPIO 32/33 double as `XTAL_32K_P/N`, but the 32.768 kHz crystal is not fitted on ESP32-WROOM-32, so they are free.

**Two hardware requirements that firmware cannot substitute for:**

- **10 kΩ pull-downs from MD13S PWM and DIR to GND.** ESP32 pins are high-impedance during reset and until `setup()` runs, so without them the driver's inputs float and the motor can run before any code executes.
- **The encoder's outputs are pulled up to its own Vcc on the encoder board** (spec range 3.5–20 V; Pololu confirms the pull-ups). Feed it 5 V and translate A/B to 3.3 V with a **BSS138 level-shifter module** — a plain resistor divider sits at ~2.5 V because of those pull-ups, right at the ESP32's logic threshold. 12 V on the encoder Vcc puts 12 V on a GPIO and destroys the board.
- **The surge suppressor must clamp below the MD13S's 30 V absolute maximum**: 1.5KE20A or SMBJ18A, not a "24 V" part (SMBJ24A clamps at 38.9 V).

**Full parts list with Polish shops (Botland, Allegro), the traps found when checking it, pin verification and wiring: [docs/dispenser_module_hardware.md](docs/dispenser_module_hardware.md). Its §6 covers the bench test image** — flash that before the module goes anywhere near the machine.

## Tractor screens

What the tractor's OLED shows and what the button does on each screen, as built in `src/tractor/main.cpp`. **Update this section in the same commit whenever a screen or a button action changes.**

- **Short press:** released before 1.5 s. **Long press:** fires at 1.5 s while the button is still held, with a 30 ms beep. The release after it does nothing.
- **Button rule:** a press counts only if the same screen was already showing for the whole press — and for at least `BUTTON_SCREEN_SETTLE_MS` (250 ms) before it began. A press that straddles a screen change is ignored silently (no beep), so a press on a screen that just appeared can never act on it.
- In the mockups, `[ ]` marks the highlighted (inverted) field. Text and positions follow the code, but large fonts are drawn at normal size.

### Map

```
Power on: Adafruit logo while starting up, 0.5 s beep
└── 1 MENU
    ├── Praca ────── 2 WORK
    │                └── 3a / 3b FAULTS, replace WORK while active
    ├── Dawka ────── 4 DOSE EDITOR
    ├── Kalibracja ─ 5 CALIBRATION EDITOR
    │                └── TEST ─ 6 CONFIRM
    │                           └── START ─ 7 RUNNING
    │                                       ├── 8 DONE
    │                                       ├── 9 MACHINE MOVING
    │                                       ├── 10 NO DISPENSER
    │                                       └── 11 INTERRUPTED
    ├── Sciezki ──── 15 TRAMLINES
    ├── Nasiona ──── 16 SEEDS (size, and the wheel calibration for it)
    │                └── Kalibracja ─ 17 CONFIRM
    │                                 └── OK ─ 18 DRIVING
    │                                          └── 19 RESULT
    └── Ustawienia ─ 20 SETTINGS, and the USB export

The menu has six items and four rows: the window follows the cursor, and
wrapping past the last item brings it back to the top.

Over any screen while the dispenser reports a clog:
12 CLOG ALARM ── OK ──> 13 CLOG CHOICE ──Anuluj──> back to the screen underneath
                       13 CLOG CHOICE ──Odetkaj──> 14 UNCLOGGING ──> 13
```

Screens 7–11 are one screen. Its content changes by itself as the dispenser reports. 12–14 cover whatever is underneath; when the clog ends, the operator is back on that screen.

### Mockups

```
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│[Praca              ]│  │RPM 3150             │  │DMUCHAWA             │  │DOZOWNIK             │
│ Dawka               │  │6.4km/h  M kg/ha: 40 │  │                     │  │                     │
│ Kalibracja          │  │                     │  │STOI                 │  │ZA SZYBKO            │
│ Sciezki             │  │Przejazd:       3    │  │                     │  │                     │
│                     │  │Doz:42 RPM     [*   ]│  │Dlugi klik = menu    │  │Dlugi klik = menu    │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
        1. Menu                  2. Work              3a. Blower stopped       3b. Dispenser too fast

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

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│Kalibracja           │  │ZATKANIE!            │  │Zatkanie dozownika   │  │Odtykanie...         │
│                     │  │                     │  │                     │  │                     │
│PRZERWANA            │  │DOZOWNIKA            │  │[Anuluj             ]│  │ |######-----------| │
│                     │  │                     │  │                     │  │                     │
│Nacisnij aby wrocic  │  │[OK                 ]│  │ Odetkaj             │  │         35%         │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
   11. Interrupted          12. Clog alarm           13. Clog choice          14. Unclogging

┌─────────────────────┐
│SCIEZKI              │
│                     │
│       [WYL.]        │
│                     │
│            ZAPISZ   │
└─────────────────────┘
     15. Tramlines

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│[Male nas.        ]■ │  │Kalibracja kola      │  │Przejedz 100 m       │  │Wynik: 128 imp       │
│ Duze nas.           │  │male nasiona         │  │ Impulsy:            │  │1 imp = 781 mm       │
│ Kalibracja          │  │                     │  │ 128                 │  │bylo 785 mm          │
│ Wroc                │  │[Anuluj             ]│  │                     │  │[Anuluj             ]│
│                     │  │ OK                  │  │Dlugi klik = koniec  │  │ ZAPISZ              │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
     16. Nasiona            17. Calibrate?            18. Driving              19. Result

┌─────────────────────┐
│Dawka:40 Kalib:500   │
│Doz:WL. Sciezki:WYL. │
│Nasiona:MALE         │
│Kolo M:781 D:627mm   │
│[Wyslij USB 115200  ]│
│ Wroc                │
└─────────────────────┘
     20. Settings

┌─────────────────────┐
│Kalibracja kola      │
│Za malo impulsow     │
│                     │
│[Anuluj             ]│
│                     │
└─────────────────────┘
    19b. Failed run
```

On 16 the inverted row is the cursor and the small square marks the size in
use — with one button the two cannot both be shown by highlighting. Screen 19
replaces the result with the reason and offers only `Anuluj` when the run gave
nothing usable: `Brak siewnika`, `Reset siewnika` (its cumulative counter
restarted, so the difference means nothing), `Za malo impulsow` (under
`WHEEL_CALIB_MIN_PULSES`) or `Wynik poza zakresem`.

#### The work screen in every state

```
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│RPM 3150             │  │RPM 3150             │  │RPM 3150             │  │RPM 3150             │
│6.4km/h  M kg/ha: 40 │  │0.0km/h  M kg/ha: 40 │  │6.4km/h  M           │  │6.4km/h  M kg/ha: 40 │
│                     │  │                     │  │                     │  │                     │
│Przejazd:       3    │  │Przejazd:       3    │  │Przejazd:       3    │  │Sciezki: WYL.        │
│Doz:42 RPM     [*   ]│  │Doz:0 RPM            │  │                     │  │Doz:42 RPM     [ *  ]│
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
   a. dispensing            b. standing still        c. dispenser off         d. tramlines off

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│RPM 3150             │  │RPM ???              │  │RPM 3150             │  │RPM 3150             │
│6.4km/h  M kg/ha: 40 │  │???km/h  M kg/ha: 40 │  │6.4km/h  M kg/ha: 40 │  │12.4km/h D kg/ha: 999│
│                     │  │                     │  │                     │  │                     │
│Przejazd:       3    │  │Przejazd:       3    │  │Przejazd:       3    │  │Przejazd:       6    │
│BRAK: D              │  │BRAK: S              │  │BRAK: S-D            │  │Doz:330 RPM    [   *]│
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
   e. dispenser unheard     f. seeder unheard        g. deaf to each other     h. widest case
```

- **Top line:** the turbine's RPM, as the seeder last reported it.
- **Second line:** ground speed; `M` or `D` for the seed-size gear; and the dose, drawn **only while the dispenser is switched on** — a blank there means off, so there is never a number to misread (b shows the machine stopped with the dispenser still on, c shows it switched off). Case h is the widest the line can get and it exactly fills the display.
- **Third line:** `Przejazd:` and the pass number in large digits, or `Sciezki: WYL.` when tramlines are off, because the pass number would then mean nothing.
- **Bottom line:** `BRAK:` and the letters of whatever is missing, if anything is (e, f, g). Otherwise the dispenser's measured shaft RPM, and the four-frame animation while the auger is actually being driven — half a second a frame, blank when the machine stands still or the dispenser has nothing to do. An empty line means all is well and the dispenser is off.
- **The seeder's two numbers go to `???` while it is unheard** (f). What is stored is the last packet it sent, and a frozen number looks live. Only the drawing changes: the telemetry itself is left alone, so a gap of a second or two — which is the usual kind — disturbs neither the alarms, nor the tramline relay, nor the dispenser, which keeps metering on the last speed it had. `???km/h` is exactly as wide as a real speed, so nothing else on the line moves.

### What each screen does

| Screen               | Content                                                                                                                                                                                                                                                                                                                                                               | Short press                                                      | Long press                                                |
| -------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- | --------------------------------------------------------- |
| 1 Menu               | `Praca`, `Dawka`, `Kalibracja`, `Sciezki`, `Nasiona`, `Ustawienia` — four rows at a time, the window following the cursor. Starts on `Praca`, then stays on the item last opened                                                                                                                                                                                                                                                                      | next item                                                        | open it                                                   |
| 2 Work               | turbine RPM, ground speed, the seed-size gear (`M` or `D`) and the dose in `kg/ha` — the dose is drawn only while the dispenser is switched on, so a blank there means it is off. Pass number 1–6 in large digits; while tramlines are off, `Sciezki: WYL.` takes that place. The bottom row is `BRAK:` while a link is down (`S` seeder, `D` dispenser once it has been heard, `S-D` they cannot hear each other), otherwise the dispenser's measured shaft RPM (`Doz:42 RPM`) and a four-frame animation that runs only while the auger is actually being driven. An empty bottom row means all is well and the dispenser is off. While the seeder is unheard the turbine RPM and the speed are drawn as `???`, the stored values untouched | next pass, 6 → 1 (nothing while tramlines are off)               | back to 1                                                 |
| 3a Blower            | `DMUCHAWA STOI` — the machine is moving and the seeder reports the turbine below `TURBINE_RUNNING_MIN_RPM`, held for `TURBINE_ALARM_DELAY_MS` so that moving off before the fan is up to speed does not beep. Only on telemetry still arriving: a silent seeder is `BRAK: S`, not a fan fault |                                                    nothing (the pass number is hidden, so it must not change blind) | back to 1                                                 |
| 3b Dispenser         | `DOZOWNIK ZA SZYBKO` (driving faster than the dispenser can keep up with; only while the dispenser is heard — a silent dispenser is announced by `BRAK: D` instead)                                                                                                                                                                                                   | nothing (the pass number is hidden, so it must not change blind) | back to 1                                                 |
| 4 Dose editor        | dose in kg/ha, `WL./WYL.`, `ZAPISZ`                                                                                                                                                                                                                                                                                                                                   | see Editors                                                      | see Editors                                               |
| 5 Calibration editor | grams per 100 dispenser revolutions, `TEST`, `ZAPISZ`                                                                                                                                                                                                                                                                                                                 | see Editors                                                      | see Editors                                               |
| 6 Confirm            | `Anuluj` (preselected) or `START`                                                                                                                                                                                                                                                                                                                                     | switch                                                           | `Anuluj` → 5, `START` → 7                                 |
| 7 Running            | progress of the 100-revolution run                                                                                                                                                                                                                                                                                                                                    | ignored                                                          | cancel the run → 5                                        |
| 8 Done               | weigh the output and enter it in grams                                                                                                                                                                                                                                                                                                                                | → 5                                                              | → 5                                                       |
| 9 Machine moving     | run refused or stopped, because the seeder reports the wheel turning                                                                                                                                                                                                                                                                                                  | → 5                                                              | → 5                                                       |
| 10 No dispenser      | dispenser not heard                                                                                                                                                                                                                                                                                                                                                   | ignored                                                          | cancel the run → 5                                        |
| 11 Interrupted       | dispenser stayed in Normal after START instead of calibrating, for `CALIBRATION_START_TIMEOUT_MS` — its side dropped the run (reboot, link gap, clog, cancel)                                                                                                                                                                                                         | → 5                                                              | → 5                                                       |
| 12 Clog alarm        | `ZATKANIE!` / `DOZOWNIKA` / `[OK]`, over any screen, with the buzzer                                                                                                                                                                                                                                                                                                  | acknowledge → 13                                                 | acknowledge → 13                                          |
| 13 Clog choice       | `Anuluj` (preselected) / `Odetkaj`, buzzer silent                                                                                                                                                                                                                                                                                                                     | switch                                                           | `Anuluj` → screen underneath, `Odetkaj` → 14              |
| 14 Unclogging        | progress of the reverse/forward sequence, over any screen                                                                                                                                                                                                                                                                                                             | ignored                                                          | ignored                                                   |
| 15 Tramlines         | `SCIEZKI`, the switch `WL.`/`WYL.`, `ZAPISZ`                                                                                                                                                                                                                                                                                                                          | move the cursor                                                  | switch: flip it and store it straight away; `ZAPISZ`: → 1 |
| 16 Nasiona | `Male nas.` / `Duze nas.` (a square marks the one in use) / `Kalibracja` / `Wroc` | next row | a size: select and store it at once; `Kalibracja` → 17; `Wroc` → 1 |
| 17 Calibrate? | `Kalibracja kola` and which size, `Anuluj` (preselected) / `OK` | switch | `Anuluj` → 16; `OK` → 18, or → 19 with `Brak siewnika` if the seeder is not heard |
| 18 Driving | the pulses counted since START, in large digits. No distance: it could only be shown using the value being replaced, and the 100 m is measured on the ground. Can be driven during normal work — it only reads the seeder's cumulative counter | ignored, so a stray press cannot throw away a 100 m drive | finish → 19 |
| 20 Ustawienia | everything the board keeps in NVS, and two rows: send it over USB (the baud rate is on the row) or leave. The only way settings leave the board — nothing can write them back in, see docs/calibration-settings.txt | switch | send, printing the block and showing `Wyslano!`; `Wroc` → 1 |
| 19 Result | the measured mm per pulse against the one it would replace, or why the run gave nothing | switch (nothing to switch on a failure) | `Anuluj` → 16; `ZAPISZ` stores it for the size being calibrated → 16 |

### Editors (4 and 5)

- Open with the saved value, cursor on the first digit.
- The top-right label says what a short press does: `WYBOR` moves the cursor, `ZMIEN` changes the digit.
- The cursor goes through each digit, then `WL./WYL.` or `TEST`, then `ZAPISZ`, then back to the first digit.
- Long press on a digit switches to `ZMIEN`, where a short press adds 1 (9 → 0). Long press again to go back to `WYBOR`.
- Long press on `WL./WYL.` switches the dispenser on or off straight away. The setting is stored with `ZAPISZ`.
- Long press on `TEST` opens 6. Coming back from 6–11 keeps the digits as they were, with the cursor on `TEST`.
- Long press on `ZAPISZ` stores the value and returns to 1. It's the only way out, so leaving an editor always saves.

### On every screen

- **Buzzer:** beeps ¼ s on, ¼ s off while a dispenser fault is active, including on screens that don't show the fault, and while the clog alarm (12) shows — also on screens that don't show it. It also beeps, with nothing on screen, when a board that was heard has been silent for about 6 s, or when the seeder and dispenser haven't heard each other for about 6 s.
- **LEDs:** green = all links fine. Blue blinking = a link is down, including the seeder not heard since power-on. Yellow = the seeder's last report says the tramline relay is on.
- **Two faults can take over the work screen**, both always on: the blower (3a) and the dispenser over-speed (3b). Nothing is acknowledged - they clear themselves when the machine does, and until then the buzzer runs and a long press is the only thing that works. A lost link is not one of them: it is the `BRAK:` letters, the blue LED and, after `LINK_BUZZER_DELAY_MS`, the buzzer. There was a WOM alarm; it was removed on 18 September 2026 because no WOM sensor exists.

## Dispenser behaviour

The dispenser's decision logic lives in `src/dispenser/dispenser_logic.h` — a header-only, hardware-free module (no clock reads, no pins, no serial, no globals; all state inside `DispenserLogic`), so the bench test drives it with simulated time, encoder counts and packets. `dispenserStep()` runs at most every `MOTOR_CONTROL_INTERVAL_MS`, measures the shaft RPM from the encoder delta first (64-bit maths, clamped), then does the counter handling, the mode table, the reversal guard, and stores the output — which is what `dispenserFillStatus()` reports. Every mode change resets the controller (integral 0, target 0, clog timer stopped).

**Tractor command counters.** `clogClearSeq` (Anuluj) and `unclogSeq` (Odetkaj) are counters repeated in every packet. The dispenser compares them with `!=` (never `>`, they wrap), acts once per change, and only outside a resync: the first command after boot or a link gap (`tractorWasAlive` was false) or after `upTimeMs` went backwards (tractor reboot) just copies the counters and acts on nothing. `calibrationArmed` is set only by a received command with `calibrationRun == 0` from an alive tractor, and cleared when a run starts or is refused — that is what stops a run from restarting by itself after a dispenser reboot, a link gap, a clog or a cancel.

**Modes.** `DispenserStatus.faultCode` is only meaningful in Normal:

| Mode                     | What one step does                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Leaves when                                                                                                                    |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| Normal                   | A calibration request (alive tractor, `calibrationRun` 1, armed) → `Refused` if the seeder reports the wheel turning, else `Calibrating`, motor off either way. Otherwise meters: no seeder → fault `NoSpeedData`, motor off. Not moving, speed 0, dispenser off or dose 0 → fault `None`, motor off. Else the rate = `requiredShaftRPM(speed, dose, calib)`, clamped to `MOTOR_MAX_RPM` (fault `OverSpeed`), plus the distance ledger's trim, never below half the rate; duty from the controller, clog check. The tractor being gone is ignored on purpose — the last dose is held | clog detected → `Clogged`                                                                                                      |
| Calibrating              | Tractor lost, no command or `calibrationRun` 0 → `Normal`, motor off. Seeder reports the wheel turning → `Refused`, motor off. `CALIBRATION_TOTAL_EDGES` edges turned → `CalibrationDone`, progress 100, motor off. Else progress from the edge count, target `CALIBRATION_RPM`, duty from the controller, clog check; fault `None` throughout                                                                                                                                                                         | clog detected → `Clogged`                                                                                                      |
| CalibrationDone, Refused | Motor off                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | tractor alive and `calibrationRun` 0 → `Normal`. Nothing else leaves; the motor is off, so holding is safe                     |
| Clogged                  | Motor off until the operator decides; calibration requests ignored                                                                                                                                                                                                                                                                                                                                                                                                                                                     | `clogClearSeq` change → `Normal`, motor off; `unclogSeq` change → `Unclogging`, first phase (reverse) output in that same step |
| Unclogging               | Phase from `t % UNCLOG_CYCLE_MS`: reverse `UNCLOG_PERMILLE` for `UNCLOG_REVERSE_MS`, pause (duty 0, forward) for `UNCLOG_PAUSE_MS`, forward `UNCLOG_PERMILLE` for `UNCLOG_FORWARD_MS`, pause; progress from `t / UNCLOG_TOTAL_MS`. No clog check — the encoder counts up in both directions, so it cannot be used here                                                                                                                                                                                                 | `clogClearSeq` change → `Normal`, motor off; tractor lost or `UNCLOG_TOTAL_MS` elapsed → `Clogged`, motor off                  |

**Clog check** (Normal and Calibrating only, and only while the controller is actually driving the motor — a step that leaves the duty at 0 is not a stall, so it stops the timer): shaft slower than `CLOG_MIN_SPEED_PERCENT` of the target for a continuous `CLOG_DETECT_MS` → `Clogged`: duty 0 forward in that same step, controller reset, progress 0, fault `None`. One step at speed stops the timer. The guard covers the case where a large negative integral (a motor running faster than the feed-forward expects, i.e. a tractor at ~14 V) has cancelled the feed-forward and the duty sits at 0 with the shaft stopped — that is not a clog, and the clamp in the controller is what stops it happening at all. It hides no real clog: a stopped shaft makes the error, and so the duty, positive.

**Distance ledger.** Metering follows the ground, not the speed estimate. One wheel pulse is a fixed distance (`TractorCommand.wheelMmPerPulse`, about 0.8 m), a fixed distance is a fixed number of shaft turns (`revolutionsPerMetre()`), and the encoder says how many turns were really made — the difference is a debt in revolutions, and `ledgerCatchupRPM()` returns the RPM added to the rate to pay it off over `LEDGER_CATCHUP_SECONDS`. It is worth the state because the speed estimate is coarse: the seeder averages a whole turn of the metering drive (~4.7 m), and the motor needs a few seconds to settle after every start. The ledger turns what that costs from fertilizer never applied into fertilizer applied a few metres later. Between pulses the ground is interpolated from the reported speed, capped at one pulse's distance — without that the debt would step by a whole pulse (~2.5 revolutions) and the target would saw up and down at the pulse rate. Four rules keep it safe:

- the debt is capped at `LEDGER_MAX_METRES` of travel and the trim at `LEDGER_MAX_CATCHUP_RPM`;
- `resetController()` wipes it, so a debt never survives a stop, a clog or a mode change and gets paid off in one spot afterwards;
- nothing is recorded while the rate is clamped (`OverSpeed`): a debt earned with the motor already at its limit cannot be repaid without double-dosing the strip that follows, and `ZA SZYBKO` is what the operator acts on;
- it can never take the target below half the rate, so a ledger that has gone negative — a real over-application, or an encoder counting edges that never happened — cannot stop the auger while the machine is moving.

**Controller.** Feed-forward (`target × 1000 / MOTOR_MAX_RPM`) plus a gentle PI trim with anti-windup: the integral only updates when the raw output is not saturated — `raw > 1000` with positive error, or `raw < MOTOR_MIN_RUNNING_PERMILLE` with negative error — otherwise a long spell at full duty or at a low target would take seconds to unwind. The integral is then clamped to `−feedForward`: the trim may cancel up to all of the feed-forward, never more, or a negative integral left over from a higher target (a motor running faster than the feed-forward expects) would hold the duty at 0 and the motor simply never restart after slowing down, with no alarm. Duty is 0 or at least `MOTOR_MIN_RUNNING_PERMILLE`, never in the buzzing band. A reversal guard drops to duty 0 whenever two consecutive steps would otherwise drive in opposite directions at speed.

## Current tasks

**No open tasks.** Tasks 1–7 were reviewed and moved to [Completed](#completed). The dispenser module is built and has passed the bench test; the seeder needs its 6 magnets fitted and both seed-size gears calibrated (`Nasiona` → `Kalibracja`) before the machine meters correctly. Remaining work is in [Future tasks](#future-tasks).

**Before any board goes on the machine:** protocol v5 changed the packet layout, so the tractor, the seeder and the dispenser must all be flashed from the same build. A board left on older firmware silently ignores the others.

The rules below apply to every new task written into this section.

### How to work on a task

1. **Read the whole task section first**, then read every file it touches **completely** (not excerpts). The existing code has comments explaining why things are the way they are — keep that reasoning intact.
2. **Build commands.** `pio` is not on PATH on this PC. Use the full path:
   - PowerShell: `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e dispenser`
   - Git Bash: `"$HOME/.platformio/penv/Scripts/pio.exe" run -e dispenser`
   - All environments: the same without `-e ...`.

   Build only. Never run `-t upload`, `-t erase` or the serial monitor unless the user asks — no board is connected during these sessions, so **the compiler is the only automatic check you have**. Everything else depends on careful reading.

3. **C++11 only** (`-std=gnu++11`, GCC 8.4): no `std::make_unique`, no inline variables, no structured bindings, no generic lambdas; a `constexpr` function may contain only a single `return` statement.
4. **Match the existing style:** `static constexpr` constants in `include/machine_settings.h` (grouped by board, with a comment when the value needs explaining), `enum class ... : uint8_t`, comments that say _why_, no `String`, no dynamic allocation, no new libraries, fixed-width integer types in anything that goes on the wire.
5. **UI text is Polish without diacritics** (the built-in font has no Polish letters: `WYL.` not `WYŁ.`). Use the strings exactly as the task gives them. The built-in font is 6×8 px per character × text size, and text wraps to the next line silently: from x = 0 a line holds 21 characters at size 1, 10 at size 2, 7 at size 3, 5 at size 4.
6. **Don't touch** what the task doesn't list: the platform pin in `platformio.ini`, `lib/Adafruit_SH1106/`, pin numbers, the seeder's behaviour, other tasks' sections.
7. **Git:** don't commit or push unless the user asks. The working tree may contain the user's uncommitted work — never `git checkout`, `git restore`, `git reset` or `git stash` files. Run `git status` and `git diff --stat` **before you start** and keep the output: the "only these files changed" checks below mean _changed by you in this session_, compared with that starting point.
8. **If the spec looks wrong, contradicts the code, or can't be built as written, stop and ask the user** — describe the conflict precisely. Don't improvise a different design. Decisions marked as agreed with the user are not open for change.
9. **Final check — required before saying the task is done.** Run the full build (all environments, zero errors, no new warnings). Then re-read the task section from the top and go through its _Final verification_ list one item at a time: open the code, confirm the item, and write a table `ID | done? | file:line | note`. Fix anything missing. Report to the user: the files changed, that table, every deviation from the spec and why, questions for the user, and which boards need reflashing.
10. **Documentation is part of the task:** update the sections the task names and the [Contents](#contents) list. When the task is done, mark it **DONE** in the task table and add your report (files changed, the final-verification table, deviations, reflash list) under a `## Current grunt work report` section after Current tasks (create it if it isn't there). **Don't move tasks to Completed and don't delete task sections** — the larger model does that after it has reviewed the work.

## Future tasks

- **Hardware verification before field use.** Nothing has run on the machine yet.
  - Dispenser module on the bench: **the bench test image ran on 17 September 2026** ([docs/dispenser_module_hardware.md](docs/dispenser_module_hardware.md) §6). Supply, driver, motor, both encoder channels, direction, radio, metering, link loss and the calibration run passed. **Not run yet:** the K tests (clog and unclogging with the motor; only K07 needs the lever) and the stall tests H06 and K07 (no fuse on the bench). The W, L and M11 checks added on 18 September 2026 have never run on the board either - they need no hardware beyond the module itself (`w` and `d`), so run them with the next bench session. That run's four failures were bugs in the tests, fixed afterwards: D08 and D20 expected the calibration target on the step that enters Calibrating, which resets the controller; C04 counted the shaft coasting down from metering; and `benchPrepare()` started the logic before its 1 s wait, so the first control step of every motor test integrated a whole second of error — M02 measured 215 RPM for a 192 RPM target (a simulation of the controller reproduces 214).
  - All three boards on a desk: pull power from each in turn — the seeder holds its relay, the dispenser stops when the seeder goes, the right `BRAK:` letters appear. With a fault beeping, cut the seeder's power: the buzzer must stop (regression check for the old stuck-buzzer bug).
  - **Fit the 6 magnets and calibrate both gears.** `Nasiona` → `Kalibracja` on the tractor, once in each seed-size gear, driving `WHEEL_CALIB_DISTANCE_M` in the field with the machine working so that wheel slip is part of the number. Until then every board meters on `WHEEL_MM_PER_PULSE_DEFAULT` (785 mm), which is an estimate. Check the counts differ between the gears by roughly the ratio you measured, and that the seed-rate setting really does not change them. Export the result afterwards (`Ustawienia` → `Wyslij`) and keep the block in [docs/calibration-settings.txt](docs/calibration-settings.txt). Then set `WHEEL_MM_PER_PULSE_DEFAULT` to the measured value for the gear used most and reflash: it is the fallback before the first command arrives, and `WHEEL_MIN_PULSE_GAP_US` - the interrupt's noise filter, which has to be a compile-time constant - is derived from it. At 785 mm it ignores pulses closer together than ~40 km/h; a measured value much below 400 mm would make that filter start clipping real pulses at working speed.
  - Dispenser calibration: check `ENCODER_EDGES_PER_REV` by hand-turning 10 revolutions (≈ 4800 edges), then `Kalibracja` → `TEST`, catch and weigh the output, enter it.
  - Blockage: stall the running dispenser; the tractor must alarm.
- **Auger output per revolution** — mechanical, not firmware: 40 kg/ha at 10 km/h needs roughly ≥ 800 g per 100 revolutions (Design record → Dispenser). Firmware clamps and alarms either way.
- **Statistics screen** — agreed with the user, not designed in detail: hectares, kg applied, average kg/ha, wheel pulses, dispenser revolutions, and `ZERUJ` with a confirmation. The main menu is now full (four rows at text size 2), so a fifth item needs scrolling. The encoder counts reverse turns too, so leave `Unclogging` out of the revolution total.
- **Task watchdog on the three production boards — do this before field use, dispenser first.** Planned in the fail-safe design, never added. Without it, a hung `loop()` on the dispenser leaves the motor PWM at its last duty until the power is cut; a crash or brownout is already safe (the reset runs `motorBegin()`). Arduino-ESP32 2.0.17 has it built in: call `enableLoopWDT()` as the last line of `setup()` in `src/dispenser`, `src/tractor` and `src/seeder` (5 s timeout, the core feeds it after every `loop()`). All three loops return within milliseconds; the logic already treats a reboot safely (motor off, counters resync, calibration not re-armed). **Not in the bench image** — its operator prompts block inside a test for minutes.
- **README is out of date** — it still says to write MAC addresses into the source and describes two boards.
- **WOM (power take-off) RPM is fabricated** — `src/seeder/main.cpp` sends a fixed `540` and nothing reads it. The alarm that used to (and could only ever have fired on a machine standing still, given the fixed value) was removed on 18 September 2026, with the WOM constants. If a sensor is ever fitted the field is still there on the wire; if not, it can go at the next protocol bump.
- **Wiring diagram** and **demonstration video** — README TODOs.
- **Motor constants from the machine — the one tweak that matters for dose accuracy.** The bench run confirmed `ENCODER_EDGES_PER_REV` = 480 (481 by hand; the calibration run stopped at 100.2 revolutions) and `MOTOR_DIR_FORWARD` = `LOW`. It measured only a free shaft on the bench supply: 347 RPM at full duty against `MOTOR_MAX_RPM` = 330, breakaway at 70 permille against `MOTOR_MIN_RUNNING_PERMILLE` = 80.
  - Why `MOTOR_MAX_RPM` matters: the feed-forward assumes the motor reaches it at full duty, and the integral resets whenever metering stops (headland, lifting), so every start runs on the feed-forward alone. Simulated with a motor 15 % faster than assumed (a tractor at ~14 V), a 192 RPM target runs at ~221 RPM for the first 2 s, 210 after 4 s, 197 after 16 s; with `MOTOR_MAX_RPM` equal to the real speed, 193 from the start. It is also the `ZA SZYBKO` limit, so it must be a speed the **loaded** motor really reaches.
  - How: flash the bench image on the fitted module and run `h` with the auger coupled, fertilizer in the hopper, a bucket under the outlet and the engine running (skip H02 and H06). Set `MOTOR_MAX_RPM` to H03's full-duty RPM and `MOTOR_MIN_RUNNING_PERMILLE` a margin above H05's breakaway, rebuild, reflash the production firmware.
- **Host-side unit tests (optional)** — `src/seeder/wheel_speed.h` and `src/dispenser/dispenser_logic.h` have no hardware code, so they could run under `pio test -e native`. That needs a host C++ compiler such as MinGW-w64, which this PC doesn't have. The bench image's D tests already cover the dispenser logic on the ESP32.

## Completed

Everything below is written and builds with `pio run` (four environments, zero errors, zero warnings). **None of it has run on the physical machine yet** — see Future tasks → Hardware verification.

- ESP-NOW link between the boards; turbine RPM and ground-wheel sensing on the seeder; tramline relay with manual pass selection; OLED, LEDs and buzzer alarms on the tractor.
- Migration from two Arduino IDE sketches to one PlatformIO project with an environment per board and a shared protocol header.
- **Protocol v2/v3** — broadcast addressing (no MACs), magic/version/network-id validation, length-checked receives, per-peer link tracking with second-hand `linkFlags`; v3 added dispenser on/off and the automated calibration run.
- **Firmware hardening** — all display/buzzer/LED/relay work moved out of the ESP-NOW receive callback into `loop()`, `enum` fault codes instead of `String`, monotonic debounced pulse counters, the fail-safe rules below.
- **Ground speed** from the existing ground-wheel sensor: the average of the last 6 gaps between pulses, lowered as soon as a pulse is late, 0 after 2 s without one (`src/seeder/wheel_speed.h`).
- **Settings file** — every machine value in `include/machine_settings.h`, grouped by board.
- **Single-button menu** on the tractor (`Praca` / `Dawka` / `Kalibracja`) with NVS persistence and the automated 100-revolution calibration run.
- **Dispenser firmware** — MD13S drive, encoder feedback, feed-forward + PI rate control, over-speed alarm.
- **Protocol v4 + clog alarm and unclogging** — `DispenserMode` replaces the calibration-state enum; `TractorCommand` carries `upTimeMs` and the Anuluj/Odetkaj counters; the dispenser detects a clog (shaft under `CLOG_MIN_SPEED_PERCENT` of target for `CLOG_DETECT_MS`) and either resumes on Anuluj or runs the reverse/forward unclog sequence on Odetkaj; the tractor alarms over any screen (`ZATKANIE!`); the calibration run can no longer restart by itself.
- **Dispenser split** — decision logic in `src/dispenser/dispenser_logic.h` (hardware-free, so the bench test can drive it), I/O in `dispenser_io.cpp`, thin `main.cpp`; PI anti-windup, locked receive inbox, reboot-safe command counters.
- **Tractor views** — an explicit `View` drives both the button handlers and the drawing, with the button rule (a press counts only if its screen was already showing) and a locked receive snapshot.
- **Tramline switch** — a fourth menu item `Sciezki` (screen 15) with a saved on/off switch, off by default; while off the relay never switches on, the work screen shows `Sciezki: WYL.` instead of the pass number, and a short press there is ignored.
- **Dispenser bench test** — a separate `dispenser_bench` environment that runs the production logic and I/O with simulated tractor and seeder packets: 35 logic checks, 8 packet checks, 8 hardware checks and 23 motor-in-the-loop scenarios, with automatic PASS/FAIL, a `RTC_NOINIT_ATTR` marker that names the test a reset happened in, and an operator checklist before the first motor test. Any key stops the motor at once and ends the whole command. **Never fitted to the machine** — the summary always prints the reflash reminder. Usage: `docs/dispenser_module_hardware.md` §6.
- **Review fixes** — the bench abort became sticky and ends the whole command; test deadlines are timed from the packet that carries the change; D29 and M06 corrected (D29 couldn't fail, M06 couldn't pass) and D35 added; K07 no longer restarts the motor after the lever comes off; the dispenser's PI integral is clamped to `−feedForward`, so a negative integral left from a higher target can't hold the motor off after slowing down; the tractor shows `ZA SZYBKO` only while the dispenser is heard.
- **Protocol v5 + distance per pulse as a setting** — the wheel sensor turned out to be on the metering drive, whose ratio to the ground wheel depends on the machine's seed-size gear, so `WHEEL_MM_PER_PULSE` could not be a constant: `TractorCommand` now carries `wheelMmPerPulse` and the seeder meters with what it is sent. The fixed 2 s stop timeout became a speed (`WHEEL_MIN_SPEED_MM_S`), so a slow pulse rate can no longer read as "stopped", and the noise gate is derived from a top speed instead of being a literal. 11 bench checks (W01–W11) drive `wheel_speed.h` with simulated pulse trains.
- **Seed-size screen and wheel calibration** — a fifth menu item `Nasiona` (so the menu scrolls), the size stored on the tractor and shown on the work screen, and a wizard that measures the distance per pulse by driving 100 m (screens 16–19). It refuses a run that a seeder reboot or too few pulses made meaningless, and a radio gap during the drive costs nothing, because the counter it reads is cumulative.
- **Settings export** — an `Ustawienia` screen (screen 20) listing everything the tractor keeps in NVS, with one row that prints it over USB at `SERIAL_BAUD` and confirms with `Wyslano!`. The printout doubles as the first-boot constants, so an erased or replaced board is restored by pasting into `include/machine_settings.h`. Export only: nothing can write a setting into the board.
- **Distance ledger** — the dispenser meters to the ground rather than to the speed estimate: a debt in shaft revolutions, paid off over `LEDGER_CATCHUP_SECONDS`, with the ground interpolated between pulses. 12 logic checks (L01–L12) and a motor-in-the-loop check (M11) that compares shaft turns against simulated ground; D03, D06, D29, D33 and D35 were re-expressed against the commanded target, which now includes the ledger's trim.

### Design record

Condensed from the Implementation_Plan that produced the firmware above (its phases 0–5 are all done). It keeps the reasons behind decisions so they don't get re-proposed; the code is the reference for details.

#### Review findings that were fixed (Implementation_Plan §1)

|     | Problem in the original two-board firmware                                                                                                                                             | Fix                                                                                                   |
| --- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| F1  | Display, buzzer and LEDs driven from the ESP-NOW receive callback: stalled packet reception, raced `loop()` on the I2C bus, and the buzzer could stay on when packets stopped mid-beep | callback only validates, copies and timestamps; all output work in `loop()`, serviced every iteration |
| F2  | `memcpy` without checking length or type                                                                                                                                               | `headerValid()`: magic, version, network id, type, exact length                                       |
| F3  | no fail-safe on link loss                                                                                                                                                              | fail-safe rules below                                                                                 |
| F4  | pulse counters read then reset, losing pulses in between                                                                                                                               | ISR counters only increment; readers subtract snapshots                                               |
| F5  | Arduino `String` for state (heap churn all day)                                                                                                                                        | `enum class`                                                                                          |
| F6  | faults computed after drawing, so one packet late                                                                                                                                      | fixed by ordering in `loop()`                                                                         |
| F7  | RPM maths divided before multiplying                                                                                                                                                   | multiply first, divide by measured elapsed time, pulses-per-rev constant                              |
| F8  | cast on the receive callback hid signature changes                                                                                                                                     | cast removed, the compiler checks it                                                                  |
| F9  | shadowed global variable                                                                                                                                                               | removed                                                                                               |
| F10 | tramline rhythm and active passes hardcoded                                                                                                                                            | named constants                                                                                       |
| F11 | WiFi channel not pinned, power save on                                                                                                                                                 | fixed channel, power save off, identical on every board                                               |
| F12 | (not a bug) sending every 200 ms even when nothing changes                                                                                                                             | kept on purpose: the periodic packet is the heartbeat that makes silence mean "board gone"            |

#### Addressing and link status (§2)

- Everything goes to the broadcast address with the sender in the header, so any board can be swapped without touching firmware anywhere. MAC pairing and ESP-NOW encryption were rejected (see below).
- A peer is alive if a valid packet from it arrived within `LINK_TIMEOUT_MS` (1000 ms = 5 missed packets). That is better evidence than the old unicast ACK: it proves the other board's loop runs, not just its radio.
- Each packet's `linkFlags` byte says whom the sender hears, which is how the tractor learns whether the seeder and dispenser hear each other.
- `NETWORK_ID` separates two machines with this firmware working side by side. A board never receives its own broadcasts, and filtering by sender makes that irrelevant anyway.

#### Protocol (§3)

- One 8-byte header plus one packed payload struct per message type, fixed-width fields, a `static_assert` on every size, names spelled out (`PROTOCOL_*`, not `PROTO_*`).
- Magic bytes `'K','S'` drop unrelated ESP-NOW traffic; the version drops out-of-step firmware; the network id drops the neighbour's machine.
- Counters on the wire are cumulative (`wheelPulses`), so a lost packet loses no counts: the next one gives the right difference.
- ~~The ground-wheel sensor sits before the seed-rate gearbox (confirmed), so `WHEEL_MM_PER_PULSE` is a true constant.~~ **Wrong — corrected on the machine on 18 September 2026:** the sensor is on the metering drive, which turns about 0.4 of a turn per ground-wheel turn for small seeds and a little more for large ones. `WHEEL_MM_PER_PULSE` is therefore one measured value **per seed-size setting**, not a constant of the machine. It is still measured by driving a known distance rather than computed from diameter, ratio and magnets. Lifting the seeder stops the drive, so speed goes to 0 and the dispenser stops without any extra sensor. Protocol v5 carries the active value in every `TractorCommand`, so the tractor owns both numbers - as it already does for the dose and the dispenser calibration - and the seeder stores no setting of its own. The two gears are measured separately rather than one derived from the other by tooth count (agreed with the user, 18 September 2026).

#### Fail-safe rules (§5)

Governing principle, agreed with the user: **a gap in the field is a permanent defect**, so an actuator holds its last command on link loss unless holding would be meaningless or unsafe.

| Board     | Condition                                                                                                            | Behaviour                                                                                                                          |
| --------- | -------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| Seeder    | boot, before any command                                                                                             | relay off                                                                                                                          |
| Seeder    | tractor not heard, for any time                                                                                      | holds the last relay state — dropping it mid-pass would leave an unmarked gap in the tramline                                      |
| Dispenser | boot                                                                                                                 | PWM 0 before anything else in `setup()`; the pull-down resistors cover reset and boot before that                                  |
| Dispenser | seeder not heard for `LINK_TIMEOUT_MS`                                                                               | motor off, fault `NoSpeedData` — without ground speed any rate is a guess                                                          |
| Dispenser | tractor not heard                                                                                                    | keeps metering with the last dose and calibration (an unfertilised strip is a permanent defect); the speed interlock still applies |
| Dispenser | wheel not turning, speed 0, dispenser off or dose 0                                                                  | motor off — covers headland turns, lifting and standing still                                                                      |
| Dispenser | calibration run and the tractor is lost                                                                              | run aborts, motor off — the tractor commanded it                                                                                   |
| Dispenser | calibration run and the seeder reports the wheel turning                                                             | refused or stopped, motor off                                                                                                      |
| Dispenser | clog detected                                                                                                        | motor off until the operator decides — neither metering nor a calibration run restarts by itself                                   |
| Dispenser | tractor lost while unclogging                                                                                        | back to `Clogged`, motor off — losing the tractor is not an operator decision                                                      |
| Dispenser | a calibration request still raised after a dispenser reboot, link gap, clog or cancel                                | no run until the tractor lowers and raises it again (the "armed" rule)                                                             |
| Tractor   | dispenser reports `Clogged`                                                                                          | `ZATKANIE!` alarm over any screen, buzzer included                                                                                 |
| Tractor   | a peer not heard for `LINK_TIMEOUT_MS`                                                                               | blue LED blinks, `BRAK:` letters on the work screen, no buzzer yet                                                                 |
| Tractor   | a peer that had been heard stays silent `LINK_BUZZER_DELAY_MS` longer, or seeder and dispenser can't hear each other | buzzer — timed from the loss of contact, so powering the tractor up first stays silent                                             |

#### Tractor UI (§6)

- One existing button and no hardware change; a phone/SoftAP configuration mode was rejected (see below).
- A long press fires at 1.5 s while still held, confirmed by a short beep, so it works with gloves and without looking; the release is swallowed. A screen changes only on a press or because another board reported something — there are no idle timeouts.
- Dose, calibration and dispenser on/off are stored with `Preferences` in the NVS partition of the ESP32's flash: no battery; retention in the order of 10–20 years; ~100,000 erase cycles per sector with wear levelling, written only on an operator action. Only a full-chip erase or a partition-table change clears them — a normal upload keeps them. The `Ustawienia` screen prints everything stored over USB, both in words and as the first-boot constants, so restoring an erased or replaced board is a paste into `machine_settings.h` and a flash ([docs/calibration-settings.txt](docs/calibration-settings.txt)). That printout is the only way settings leave a board, and there is deliberately no way in: no packet and no serial command can write a setting, because a calibration costs a drive across the field and only the operator, on the screen that measured it, may replace one.
- Values that never change in service (working width, tramline rhythm) stay compile-time constants. The distance per wheel pulse used to be one and is not any more: the machine's seed-size gear changes it, so it is a measured setting per gear, stored on the tractor and sent on the wire like the dose. The tractor broadcasts dose and calibration in every packet, so the dispenser stores nothing and never needs reflashing when they change.
- Calibration run: the request is level-triggered (survives lost packets without an acknowledgement protocol); it bypasses the ground-speed interlock because it's a stationary job and the seeder may be off, but is refused while the machine moves; tractor loss aborts it; a short press can't stop it (that would spoil the weighing). The dispenser starts a run only on a request it saw go from 0 to 1, so it never restarts one by itself after its own reboot, a link gap or a clog — the tractor shows `PRZERWANA` instead of sitting at 0 %.
- Clog alarm (agreed with the user): clog = shaft under 1/3 of the commanded speed for 1.5 s; the alarm covers any screen with the buzzer; `OK` → `Anuluj` (preselected, resumes metering) / `Odetkaj` (4 s reverse/forward sequence, no success check). Commands are counters repeated in every packet, acted on once, ignored right after a link gap or tractor reboot. A press counts only if its screen was already showing for the whole press, so an alarm appearing mid-press can't be acknowledged blind.
- Tramline switch (agreed with the user): the machine is usually used without tramlines, so the switch starts off; flipping it saves straight away and never resets the pass number.

#### Dispenser (§7)

- **Topology:** the dispenser computes its own motor setpoint from `SeederTelemetry` and `TractorCommand`, so a tractor-module failure doesn't stop metering, and the control loop sits next to the motor.
- **Rate maths** with working width W m, dose D kg/ha and calibration C g per 100 output-shaft revolutions: grams per metre = W × D / 10; revolutions per metre = (W × D / 10) / (C / 100); shaft RPM = 60 × speed [m/s] × revolutions per metre. At 4 m and 40 kg/ha that's 16 g per metre.
- **The motor's 330 RPM caps the working speed:** v_max [m/s] = 330 × C / (2400 × D) at 4 m. At 40 kg/ha: C = 300 → 3.7 km/h, 500 → 6.2 km/h, 800 → 9.9 km/h, 1000 → 12.4 km/h. So the auger must deliver about ≥ 800 g per 100 revolutions for 10 km/h. That is decided by auger geometry; firmware's job is to make the shortfall loud (clamp and raise `ZA SZYBKO`), not to hide it.
- **Metering to distance, not to speed** (18 September 2026): the wheel sensor gives one pulse per 0.8 m and the speed is averaged over a whole turn of the metering drive, so the rate alone is a lagging, quantised input. The dispenser keeps a ledger of the shaft turns it owes the ground and trims the target to pay it off - see Dispenser behaviour. The speed-based rate stays as the feed-forward; the ledger is what makes the total right.
- **Encoder:** channel A rising edges only, 480 per output revolution, ~2.6 kHz at full speed; channel B is wired but unused (the ESP32's PCNT peripheral is the upgrade path if interrupts ever become a problem).
- **Control:** feed-forward from the rate maths does the work; a gentle PI trim corrects for battery sag and auger load. The duty is either 0 or at least `MOTOR_MIN_RUNNING_PERMILLE`, never in the buzzing band where the motor doesn't turn.
- **Power:** tractor 12 V is dirty (load dumps, alternator noise) and the motor draws up to 5.5 A on stall next to a microcontroller. Brownouts are the most likely field failure, and they are a wiring problem, not a firmware one — see `docs/dispenser_module_hardware.md`.

#### Rejected ideas (§8)

1. **Sending only on change** — kills the heartbeat; silence would no longer mean "board gone", so link-loss fail-safes become impossible.
2. **MQTT / HTTP / cloud between boards** — there is no router in a field.
3. **ESP-NOW encryption** — incompatible with broadcast addressing; the threat (someone in the field with an ESP32 and this repo) doesn't justify losing zero-reflash board swaps.
4. **Phone / SoftAP configuration mode** — only existed to type two numbers; would need radio coexistence, an HTTP server and a web page, and phones distrust networks without internet. The button menu does it with existing hardware.
5. **Automatic MAC pairing** — a pairing state machine and stored state that goes stale, i.e. brings back the manual re-pairing step broadcast removes.
6. **Receivers computing speed from raw pulses** — the seeder owns the sensor and publishes finished speed; raw pulses are still sent, and the dispenser counts them for its distance ledger (with the distance per pulse the tractor sends), but no board other than the seeder turns them into a speed.
7. **Per-interval pulse counts on the wire** — a dropped packet loses counts for good; cumulative counters heal themselves.
8. **The tractor computing the dispenser setpoint** — adds a hop and makes metering depend on the tractor module.
9. **One shared struct for all messages** — every field change would force all boards to update in lockstep.
10. **FreeRTOS tasks or an event framework** — `loop()` with timestamps is enough at 5 Hz; the one real concurrency bug (F1) was fixed by removing work from another context.
11. **A second seeder sensor for ground speed** — the fitted ground-wheel sensor already measures distance.
12. **An open-loop dispenser as an interim step** — the motor's built-in encoder makes closed loop available from day one.
13. **Full quadrature decoding in production** — one direction while metering; channel A gives far more resolution than needed at half the interrupt load.
14. **Deriving `WHEEL_MM_PER_PULSE` from wheel diameter, ratio and magnets** — three measurements to get wrong instead of one roll-out.
15. **Mesh relaying between modules** — solves a failure never seen at a few metres; `linkFlags` makes the assumption visible instead.
16. **An alarm for a calibration value of 0, and a check after 50 calibration turns** — declined by the user.
17. **PlatformIO `pio test`/Unity for the bench test** — the test runner owns the serial port and can't prompt an operator, and half the bench test is physical steps; the bench is its own firmware image instead, running the production dispenser code.
18. **A second ESP32 simulating the tractor and seeder over the radio for the bench test** — needs another board, and the radio code is the same as on the two boards already working; packets are injected at the receive function instead.
19. **A pure pulse-lock loop for the dispenser, with no speed term** - the electronic-gearbox form of the same idea. It would have meant rewriting the RPM loop the bench had already proved, and most of the D tests, to arrive in the same place; the distance ledger sits on top of them instead.
20. **Deriving the large-seed distance per pulse from the small-seed one by the gear ratio** - declined by the user: both gears are measured, so a wrong tooth count cannot quietly halve a dose.
21. **Correcting the half-pulse of debt the ledger takes on when it starts counting** - the first pulse it credits is partly ground covered before it was looking, about 12 g of fertilizer and always in the direction of applying more. Removing that bias needs another piece of state and trades it for an under-application at the start of every pass.
