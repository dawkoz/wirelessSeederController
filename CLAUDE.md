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
   - [Burst metering](#burst-metering)
8. [Current tasks](#current-tasks)
   - [How to work on a task](#how-to-work-on-a-task)
9. [Future tasks](#future-tasks)
10. [Completed](#completed)
    - [Design record](#design-record)

## What this project is

A wireless controller for a Kverneland Accord-style seeder, mimicking the Kverneland FGS Rhythmus functionality. Three ESP32 modules talk to each other over **ESP-NOW** (not WiFi/MQTT — no router, no internet, no phone app):

- **Seeder module** (on the seeder): turbine RPM sensor, ground-wheel sensor (ground speed and "is the seeder moving"), tramline relay.
- **Tractor module** (in the cab): OLED display, one button that drives every screen, LEDs and buzzer. Owns the tramline selection and the dispenser settings.
- **Dispenser module** (on the seeder): fertilizer dispenser motor driven through a Cytron motor driver, metered from the seeder's ground speed. A separate board for one reason only — the seeder's enclosure has no room left. It gets its own 12 V power cable, from the power pins of the tractor's ISOBUS socket; everything else is wireless like the other two boards. **Built and bench-tested, not yet fitted to the machine.**

## Philosophy

Read this before changing anything.

- **This runs on moving farm equipment, often unattended, sometimes far from a workbench.** Reliability and predictability beat cleverness. Prefer boring, explicit code over abstractions.
- **ESP-NOW, not WiFi infrastructure.** There is no router in a field. Don't suggest MQTT/HTTP/cloud-anything for board-to-board communication.
- **The wire protocol is the most fragile part of this system.** The boards `memcpy` raw struct bytes at each other, so all three **must** agree byte-for-byte on struct layout. That's why the structs live in one shared file. Never let a board-specific struct definition exist outside it.
- **Hardware constraints drive architecture decisions**, not the other way around — e.g., the dispenser is a separate module purely because of physical enclosure space, not for software reasons.
- **Minimal dependencies.** Only Adafruit GFX + the (older, `SH1106_SWITCHCAPVCC`-API) Adafruit_SH1106 OLED library are used, and only by the tractor board.

## Important notes

- **`GPIO12` is the MTDI strapping pin** — its level at reset selects the flash voltage, and held high at reset the chip configures 1.8 V flash and **fails to boot**. Neither board uses it any more: the tractor's button moved off it to GPIO32 on 27 September 2026, because it needed a strong pull-up, and the seeder's relay to GPIO18 in the user's commit `a3b032a`. **Never put anything that can hold it high on GPIO12** (see [Hardware and pins](#hardware-and-pins)).
- **No MAC addresses anywhere.** All three boards transmit to the ESP-NOW broadcast address and filter incoming packets by the `type`/`sender` fields in `MessageHeader`. A physical ESP32 can be swapped for a new one with **no firmware change on any board**.
- ESP-NOW runs on a fixed `ESPNOW_CHANNEL` (1) with power save disabled, set identically on all three boards in `setup()`. `encrypt` is `false` — required, since ESP-NOW encryption needs per-peer keys and is incompatible with broadcast addressing (see Rejected ideas). The registered peer uses `channel = 0` ("current radio channel") deliberately: naming the channel on both the radio and the peer creates a way for them to disagree, and a mismatch makes _every_ `esp_now_send()` fail. Send failures are counted and logged on serial rather than discarded.
- **Any change to `include/espnow_protocol.h` means reflashing all three boards.** Bump `PROTOCOL_VERSION` when you do — receivers drop mismatched packets, so a half-updated set of boards fails loudly instead of quietly misreading each other. The `static_assert`s on every struct turn an accidental layout change into a build error.
- **The dispenser writes PWM 0 before anything else in `setup()`** — keep it the first thing. The 10 kΩ pull-downs on the driver inputs cover the time before that (reset and boot).
- **The tractor calls `Wire.setClock(400000)` after `oled.begin()` — leave it there.** The vendored SH1106 library set the same speed via the AVR `TWBR` register, which had to be removed for the ESP32 port. Without it the bus runs at Arduino's default 100 kHz and a full redraw takes ~103 ms instead of ~30 ms. The button no longer suffers from that — a timer samples it — but everything `loop()` does waits for every frame: the response to a press, the screen itself, the LEDs, the buzzer's beat.
- The Adafruit_SH1106 library used by the tractor board is the older community library (class `Adafruit_SH1106`, `begin(SH1106_SWITCHCAPVCC, addr)` API), originally from [wonho-maker/Adafruit_SH1106](https://github.com/wonho-maker/Adafruit_SH1106) — not Adafruit's newer `Adafruit_SH110X` library, which has a different, incompatible API. It's **vendored** (not pulled via `lib_deps`) at `lib/Adafruit_SH1106/`, with three small ESP32-portability patches, because upstream targets AVR only. Two are on the SPI path this project doesn't use; the third removed the AVR I2C clock setting, which is why the `Wire.setClock` call above exists. Details: `lib/Adafruit_SH1106/README.md`.

## Repo and build structure

This is a **single PlatformIO project** (not Arduino IDE, not separate per-board projects) with **one build environment per physical board**:

```
platformio.ini              defines envs: tractor, seeder, dispenser, dispenser_bench
include/espnow_protocol.h   message structs — the shared wire protocol, single source of truth
include/machine_settings.h  every machine value (pins, calibration, timings, alarms), grouped by board
src/tractor/main.cpp        tractor board firmware
src/tractor/angle_factor.h  burst metering's angle factor: the calibration run's expected mass and the corrections (no hardware code, the bench tests it)
src/seeder/main.cpp         seeder board firmware
src/seeder/wheel_speed.h    ground speed from wheel pulse timing, averaged over 15 pulses (no hardware code, so it can be tested on a PC)
src/dispenser/main.cpp      dispenser board firmware - thin: radio init, receive callback, status broadcast
src/dispenser/dispenser_logic.h   the dispenser's decision logic, header-only and hardware-free (see Dispenser behaviour)
src/dispenser/dispenser_io.h/.cpp encoder, motor output and the packet inbox - the only dispenser code that touches pins
src/dispenser_bench/        bench test image for the dispenser module - its own environment, never fitted (see docs/dispenser_module_hardware.md)
docs/dispenser_module_hardware.md   dispenser parts list, pin verification, wiring, bench test
docs/dispenser_wiring.svg           the same wiring drawn out: every pin, the power input, a connection list
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
| 18  | Relay control (tramline), active HIGH                   |
| 19  | Turbine inductive sensor (interrupt, RISING)            |
| 22  | Metering-drive Hall sensor, 3 magnets                   |

The pins moved from 12/14/27 in the user's commit `a3b032a`. The relay board fitted closes on a **high** input: it was driven active-LOW until 29 September 2026, and the tramlines came on where they should be off and the other way round (`RELAY_ON`/`RELAY_OFF` in `include/machine_settings.h`). During reset the pin floats, so the board's input needs a pull-down to stay off then (most have one; fit 10 kΩ to GND if the relay clicks at power-up).

The pin 22 sensor is on the **metering drive, not on the ground wheel** (found on the machine on 18 September 2026). The two do not turn at the same rate: about 0.4 of a metering turn per ground-wheel turn for small seeds and a little more for large ones, selected by the machine's own two-speed gear — and confirmed independent of the seed-rate setting. **3 magnets** (`WHEEL_MAGNETS`; six were tried, and that close together the sensor could not tell one from the next — 27 September 2026), so a pulse is roughly 1.6 m: about 0.7 s apart at 8 km/h but 2.8 s at 2 km/h, which is why "stopped" is a speed (`WHEEL_MIN_SPEED_MM_S`) rather than a fixed timeout, and why the speed is averaged over whole turns of the drive — five of them since 28 September 2026, 15 pulses (`WHEEL_AVERAGE_INTERVALS`), because one turn made the number on the work screen twitch. The distance per pulse is therefore **one measured value per gear**, kept on the tractor and sent in every packet (`TractorCommand.wheelMmPerPulse`); the operator measures each one by driving 100 m on the `Nasiona` screens. It is also the source for "is the seeder moving".

### Tractor ESP32 (`src/tractor/main.cpp`)

| Pin | Function                                                              |
| --- | --------------------------------------------------------------------- |
| 32  | The button (to GND), 470 Ω to 3V3 and 10–100 nF to GND at the board   |
| 14  | Green LED (connected)                                                 |
| 27  | Blue LED (connecting/blinking)                                        |
| 13  | Yellow LED (tramline active)                                          |
| 19  | Buzzer                                                                |
| 21  | OLED SDA (the `Wire` default)                                         |
| 22  | OLED SCL (the `Wire` default)                                         |
| 12  | Nothing — a strapping pin, left free                                  |

OLED: SH1106 128x64 over I2C, address `0x3C`. The single button drives every screen (see [Tractor screens](#tractor-screens)): short press < 750 ms, long press fires _at_ 750 ms (`BUTTON_LONG_PRESS_MS`) while still held.

**The button's wiring** (since 27 September 2026, when it moved off GPIO12): the button between GPIO32 and GND, a **470 Ω resistor from GPIO32 to 3V3** and a **ceramic capacitor of 10–100 nF from GPIO32 to GND** (the one fitted is 10 nF), both at the ESP32 board. The resistor puts ~7 mA through the contacts while the button is pressed. It is a car-style panel switch, and those need real current to conduct through the film that grows on their contacts: car electronics feed their switch inputs 1–15 mA, while the ESP32's internal ~45 kΩ pull-up gives ~0.07 mA — and the first press after a pause went missing. Anything from 220 Ω to 1 kΩ works: higher gives the contacts less current, lower gives more than car electronics use and lowers the contact resistance that still reads as pressed (155 Ω at 470 Ω, against well under 1 Ω for a healthy switch and its cable). GPIO12 could not take the resistor, being a strapping pin. The capacitor absorbs fast spikes picked up by the cab wiring before they reach the pin, and its discharge through the contacts at each press helps keep them clean. It is insurance, not a requirement: the 470 Ω already makes the line hard to disturb, and the firmware needs two samples of contact in a row (10 ms) before anything counts as a press. The internal pull-up stays enabled as a fallback, so the button still works, weakly, if the resistor ever comes off.

**Every tractor pin, checked against Espressif's GPIO reference** (27 September 2026):

- **32 (button):** not a strapping pin, does nothing at boot, has an internal pull-up. It doubles as `XTAL_32K_P`, free because ESP32-WROOM modules fit no 32.768 kHz crystal — worth a glance that the board has none near the pin.
- **14 (green LED):** not a strapping pin, but it outputs a PWM signal during boot, so the LED flickers at power-up. Harmless for an LED; never put anything that moves on it.
- **27 (blue LED), 19 (buzzer), 21/22 (I2C):** no strapping or boot function. Like every GPIO, 19 floats during reset: a buzzer driven through a transistor stays silent then only if the base has a pull-down resistor.
- **13 (yellow LED):** JTAG `MTCK`, which this firmware never enables; not a strapping pin.
- **12:** the `MTDI` strapping pin — now unused, and it must stay free of anything that could hold it high.

### Dispenser ESP32 (`src/dispenser/main.cpp`)

Built and bench-tested, not yet fitted to the machine. **Cytron MD13S** driver (PWM + DIR, 13 A continuous) and a **Pololu 4752** motor (37Dx68L, 30:1, 12 V, 330 RPM, 14 kg·cm, 5.5 A stall) with a built-in quadrature encoder (64 CPR motor shaft → 1920 CPR output shaft). Powered by its own 12 V line from the ISOBUS socket's PWR / PWR_GND pins, through a 10 A fuse at the plug; control/telemetry wireless like the other two boards.

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

- **Short press:** let go before 0.75 s. **Long press:** fires at 0.75 s (`BUTTON_LONG_PRESS_MS`) while the button is still held, with a 30 ms beep. Letting go after it does nothing.
- **How a press is read** (the rules, with their constants, are listed in `include/machine_settings.h`): a timer reads the button every 5 ms whatever the screen is doing. A press **starts** after 10 ms of contact (`BUTTON_PRESS_CONFIRM_MS`) and **ends** only after 50 ms without it (`BUTTON_RELEASE_CONFIRM_MS`), so the switch bouncing, or sparking under a finger that is still holding it, never splits one press into two — and a hold that sparks all the way through is still one long press. A short press is sent when its end is confirmed, 50 ms after the finger lets go. **A press that starts less than `BUTTON_MIN_GAP_MS` (250 ms) after the finger last let go is ignored whole** — timed from the release of any press, counted or not, so bounce after letting go, even after a long press, and a finger resting on the button in a shaking cab make no extra clicks. Taps with the finger up for less than 250 ms count only the first, silently, until the operator pauses; a long press started within 250 ms of letting go is ignored too.
- **Button rule:** a press counts only if the same screen was already showing for the whole press — and for at least `BUTTON_SCREEN_SETTLE_MS` (250 ms) before it began. A press that straddles a screen change is ignored silently (no beep), so a press on a screen that just appeared can never act on it.
- In the mockups, `[ ]` marks the highlighted (inverted) field. Text and positions follow the code, but large fonts are drawn at normal size - one character per cell, whatever the size the code asks for. That is a real limitation of drawing a 128x64 panel in text: a mockup cannot show that a size-4 glyph is 20x28 pixels and covers four rows of cells. It hid a collision on the work screen for months (see the third-line note below), so when a screen uses a large font, check its footprint against the panel rather than against the mockup: the classic font draws a 5x7 glyph in a 6x8 cell, scaled by the text size.

### Map

```
Power on: Adafruit logo while starting up, 0.5 s beep
└── 1 MENU
    ├── Praca ────── 2 WORK
    │                └── 3a / 3b FAULTS, replace WORK while active
    ├── Dawka ────── 4 DOSE EDITOR
    ├── Kalibracja ─ 5 CALIBRATION EDITOR: the angle factor in burst metering,
    │                │                     grams per 100 turns in continuous
    │                ├── MASA ─ 5b MASS (burst metering only)
    │                └── TEST ─ 6 CONFIRM
    │                           └── START ─ 7 RUNNING
    │                                       ├── 8 DONE ── (burst) 5b MASS
    │                                       ├── 9 MACHINE MOVING
    │                                       ├── 10 NO DISPENSER
    │                                       └── 11 INTERRUPTED
    ├── Sciezki ──── 15 TRAMLINES
    ├── Nasiona ──── 16 SEEDS (size, and the wheel calibration for it)
    │                └── Kalibracja ─ 17 CONFIRM
    │                                 └── OK ─ 18 DRIVING
    │                                          └── 19 RESULT
    ├── Ustawienia ─ 20 SETTINGS, and the USB export
    ├── Dmuchawa ─── 21 BLOWER ALARM switch, back on at every power-up
    └── Testy ────── 22 TESTS, a row each (burst metering: Symulacja 15), Wroc
                     └── Symulacja 15 ─ 23 CONFIRM
                                        └── START ─ 24 RUNNING
                                                    ├── 25 DONE, the result
                                                    ├── 26 NO DISPENSER
                                                    ├── 27 MACHINE MOVING
                                                    └── 28 STOPPED, how far it got
                                                        (long press on 24 or 26,
                                                        or the dispenser dropped it)

The menu has eight items and four rows: the window follows the cursor, and
wrapping past the last item brings it back to the top.

Over any screen while the dispenser reports a clog (silent; in burst metering
only from the calibration run):
12 CLOG ALARM ── OK ──> 13 CLOG CHOICE ──Anuluj──> back to the screen underneath
                       13 CLOG CHOICE ──Odetkaj──> 14 UNCLOGGING ──> 13
```

Screens 7–11 are one screen. Its content changes by itself as the dispenser reports. 12–14 cover whatever is underneath; when the clog ends, the operator is back on that screen. In burst metering, work never brings them up: a stalled burst is tried again, and a run of them unclogs by itself with nothing on screen — two quick blinks of the yellow LED per failed burst are the only sign ([Burst metering](#burst-metering) → Failed bursts).

The tractor shows screens 5–8 for whichever metering `DISPENSER_BURST_MODE` selects — the same constant the dispenser reads, so the two boards must be flashed from the same build. With burst metering (on since 28 September 2026) screen 5 is the angle factor and 5b the weighed mass; with continuous metering 5 is grams per 100 turns and there is no 5b.

### Mockups

```
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│                     │  │[Praca              ]│  │ Sciezki             │
│  Adafruit logo      │  │ Dawka               │  │ Nasiona             │
│  (the library's     │  │ Kalibracja          │  │ Ustawienia          │
│  start-up image)    │  │ Sciezki             │  │[Dmuchawa           ]│
│                     │  │                     │  │                     │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
  0. Power-up, ~0.5 s            1. Menu             1b. Menu, scrolled

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│RPM 3150             │  │DMUCHAWA             │  │DOZOWNIK             │
│6.4km/h  M kg/ha: 40 │  │                     │  │                     │
│                     │  │STOI                 │  │ZA SZYBKO            │
│Przejazd:       3    │  │                     │  │                     │
│Doz:42 RPM     [*   ]│  │Dlugi klik = menu    │  │Dlugi klik = menu    │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
        2. Work             3a. Blower stopped     3b. Dispenser too fast

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│DAWKA kg/ha     WYBOR│  │DAWKA kg/ha     ZMIEN│  │KALIBR. g/100   WYBOR│  │Start kalibracji?    │
│                     │  │                     │  │                     │  │100 obrotow          │
│      [0] 4  0       │  │       0 [4] 0       │  │   [0] 0  5  0  0    │  │                     │
│                     │  │                     │  │                     │  │[Anuluj             ]│
│ WL.        ZAPISZ   │  │ WL.        ZAPISZ   │  │ TEST       ZAPISZ   │  │ START               │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
     4. Dose editor        4b. Changing a digit    5. Calibration editor          6. Confirm

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│Kalibracja...        │  │GOTOWE               │  │Kalibracja           │  │Kalibracja           │
│                     │  │                     │  │                     │  │                     │
│ |##########-------| │  │Zwaz nawoz i wpisz   │  │MASZYNA              │  │BRAK                 │
│                     │  │wynik w gramach.     │  │                     │  │                     │
│         60%         │  │Nacisnij aby wrocic  │  │W RUCHU              │  │DOZOWNIKA            │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
       7. Running                8. Done             9. Machine moving         10. No dispenser

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│WSP. KATA       WYBOR│  │MASA g          WYBOR│  │Start kalibracji?    │
│                     │  │                     │  │20 porcji = 503 g    │
│      [5] 0  0       │  │   [0] 0  5  4  8    │  │                     │
│20 porcji = 503 g    │  │Ocz.503g  Kat 500>459│  │[Anuluj             ]│
│ TEST   MASA   ZAPISZ│  │ WROC       ZAPISZ   │  │ START               │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
 5. Angle factor (burst)    5b. Weighed mass         6. Confirm (burst)

┌─────────────────────┐  ┌─────────────────────┐
│Kalibracja... 503 g  │  │GOTOWE               │
│                     │  │                     │
│ |##########-------| │  │Ocz. masa: 503 g     │
│                     │  │Zwaz nawoz.          │
│         60%         │  │Nacisnij: wpisz mase │
└─────────────────────┘  └─────────────────────┘
   7. Running (burst)        8. Done (burst)

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│Kalibracja           │  │ZATKANIE!            │  │Zatkanie dozownika   │  │Odtykanie...         │
│                     │  │                     │  │                     │  │                     │
│PRZERWANA            │  │DOZOWNIKA            │  │[Anuluj             ]│  │ |######-----------| │
│                     │  │                     │  │                     │  │                     │
│Nacisnij aby wrocic  │  │[OK                 ]│  │ Odetkaj             │  │         35%         │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
    11. Interrupted           12. Clog alarm          13. Clog choice           14. Unclogging

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│SCIEZKI              │  │ALARM DMUCHAWY       │  │ALARM DMUCHAWY       │
│                     │  │                     │  │                     │
│       [WYL.]        │  │       [WL.]         │  │       [WYL.]        │
│                     │  │                     │  │                     │
│            ZAPISZ   │  │            ZAPISZ   │  │do resetu   ZAPISZ   │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
     15. Tramlines         21a. Blower alarm on    21b. Blower alarm off

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│[Male nas.        ]■ │  │Kalibracja kola      │  │Przejedz 100 m       │  │Wynik: 128 imp       │
│ Duze nas.           │  │male nasiona         │  │ Impulsy:            │  │1 imp = 781 mm       │
│ Kalibracja          │  │                     │  │ 128                 │  │bylo 785 mm          │
│ Wroc                │  │[Anuluj             ]│  │                     │  │[Anuluj             ]│
│                     │  │ OK                  │  │Dlugi klik = koniec  │  │ ZAPISZ              │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
      16. Nasiona             17. Calibrate?            18. Driving               19. Result

┌─────────────────────┐
│Kalibracja kola      │
│Za malo impulsow     │
│                     │
│[Anuluj             ]│
│                     │
└─────────────────────┘
    19b. Failed run

┌─────────────────────┐  ┌─────────────────────┐
│Dawka:40 Kalib:500   │  │Dawka:40 Kalib:500   │
│Doz:WL. Sciezki:WYL. │  │Doz:WL. Sciezki:WYL. │
│Nasiona:MALE         │  │Nasiona:MALE         │
│Kolo M:781 D:627mm   │  │Kolo M:781 D:627mm   │
│[Wyslij USB 115200  ]│  │[Wyslano!           ]│
│ Wroc                │  │ Wroc                │
└─────────────────────┘  └─────────────────────┘
      20. Settings         20b. Just sent, 2 s

┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│TESTY                │  │Symulacja siewu      │  │Symulacja 7.0 km/h   │
│[Symulacja 15       ]│  │7.0 km/h, 15 min     │  │Czas 03:25 / 15:00   │
│ Wroc                │  │                     │  │Porcje: 254          │
│                     │  │[Anuluj             ]│  │Nieudane: 3 (1.2%)   │
│                     │  │ START               │  │Odtykania: 0         │
│                     │  │                     │  │Dlugi klik = stop    │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
        22. Testy               23. Confirm              24. Running

┌─────────────────────┐  ┌─────────────────────┐
│Symulacja - koniec   │  │Symulacja przerwana  │
│Czas 15:00, 7.0 km/h │  │Czas 08:12, 7.0 km/h │
│1.2%                 │  │0.9%                 │
│nieudane: 13 z 1112  │  │nieudane: 5 z 546    │
│odtykania: 1         │  │odtykania: 0         │
│Nacisnij aby wrocic  │  │Nacisnij aby wrocic  │
└─────────────────────┘  └─────────────────────┘
  25. Done (% at size 2)     28. Stopped early
```

The menu shows four of its eight items at a time and the window follows the
cursor, so `Dmuchawa` and `Testy` only come into view once the cursor gets
there (1b is the window before the last). 4b is the editor with a digit being changed: the label says
`ZMIEN` and a short press adds 1; screen 5 changes the same way. 20b shows for
2 s after a send. 21 comes up as 21a at every power-up; 21b's `do resetu` says
the alarm stays off only until the next one. On the burst screens the line
under the digits is what the calibration run should weigh — 20 pulses' worth of
ground at the dose and seed size in use, so it follows both — and on 5b also the
angle factor now and what `ZAPISZ` would make it: `Kat 500>459` is 548 g
weighed for 503 g expected.

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
- **Second line:** ground speed, which the seeder averages over the last 15 pulses (five drive turns, about 24 m) — steady, but a change of speed takes 10–20 s to settle on it (8 → 4 km/h about 19 s, 4 → 8 about 10 s, simulated), while a stop still shows within the stop rule's ~5 s; `M` or `D` for the seed-size gear; and the dose, drawn **only while the dispenser is switched on** — a blank there means off, so there is never a number to misread (b shows the machine stopped with the dispenser still on, c shows it switched off). Case h is the widest the line can get and it exactly fills the display.
- **Third line:** `Przejazd:` and the pass number, or `Sciezki: WYL.` when tramlines are off, because the pass number would then mean nothing. The pass number is drawn at the same text size as everything else on this screen: at size 4 it reached down into the bottom row and shared pixels with the animation marker (x 93-102, y 51-56) whenever tramlines were on and the auger was turning.
- **Bottom line:** `BRAK:` and the letters of whatever is missing, if anything is (e, f, g). Otherwise the dispenser's measured shaft RPM, and the four-frame animation while the auger is actually being driven — half a second a frame, blank when the machine stands still or the dispenser has nothing to do. An empty line means all is well and the dispenser is off. In burst metering the tractor's code is the same but the dispenser sends a 2 s average as the RPM, and the marker shows only while a burst drives the auger, so it comes and goes with the bursts ([Burst metering](#burst-metering)).
- **The seeder's two numbers go to `???` while it is unheard** (f). What is stored is the last packet it sent, and a frozen number looks live. Only the drawing changes: the telemetry itself is left alone, so a gap of a second or two — which is the usual kind — disturbs neither the alarms, nor the tramline relay, nor the dispenser, which keeps metering on the last speed it had. `???km/h` is exactly as wide as a real speed, so nothing else on the line moves.

### What each screen does

| Screen               | Content                                                                                                                                                                                                                                                                                                                                                               | Short press                                                      | Long press                                                |
| -------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- | --------------------------------------------------------- |
| 1 Menu               | `Praca`, `Dawka`, `Kalibracja`, `Sciezki`, `Nasiona`, `Ustawienia`, `Dmuchawa` — four rows at a time, the window following the cursor. Starts on `Praca`, then stays on the item last opened                                                                                                                                                                                                                                                                      | next item                                                        | open it                                                   |
| 2 Work               | turbine RPM, ground speed, the seed-size gear (`M` or `D`) and the dose in `kg/ha` — the dose is drawn only while the dispenser is switched on, so a blank there means it is off. Pass number 1–6 on the same row as the label; while tramlines are off, `Sciezki: WYL.` takes that place. The bottom row is `BRAK:` while a link is down (`S` seeder, `D` dispenser once it has been heard, `S-D` they cannot hear each other), otherwise the dispenser's measured shaft RPM (`Doz:42 RPM`) and a four-frame animation that runs only while the auger is actually being driven. An empty bottom row means all is well and the dispenser is off. While the seeder is unheard the turbine RPM and the speed are drawn as `???`, the stored values untouched | next pass, 6 → 1 (nothing while tramlines are off)               | back to 1                                                 |
| 3a Blower            | `DMUCHAWA STOI` — the machine is moving and the seeder reports the turbine below `TURBINE_RUNNING_MIN_RPM`, held for `TURBINE_ALARM_DELAY_MS` so that moving off before the fan is up to speed does not beep. Only on telemetry still arriving: a silent seeder is `BRAK: S`, not a fan fault. Never while switched off on 21 |                                                    nothing (the pass number is hidden, so it must not change blind) | back to 1                                                 |
| 3b Dispenser         | `DOZOWNIK ZA SZYBKO` (driving faster than the dispenser can keep up with; only while the dispenser is heard — a silent dispenser is announced by `BRAK: D` instead)                                                                                                                                                                                                   | nothing (the pass number is hidden, so it must not change blind) | back to 1                                                 |
| 4 Dose editor        | dose in kg/ha, `WL./WYL.`, `ZAPISZ`                                                                                                                                                                                                                                                                                                                                   | see Editors                                                      | see Editors                                               |
| 5 Calibration editor | burst metering: the angle factor, 3 digits (0–999), and under them what the calibration run should weigh; `TEST`, `MASA`, `ZAPISZ`. Continuous metering: grams per 100 dispenser revolutions, `TEST`, `ZAPISZ`                                                                                                                                                                                                                                                 | see Editors                                                      | see Editors                                               |
| 5b Mass (burst only) | the weighed mass in grams, 5 digits, opened at what the run should weigh; under them the expected mass, the angle factor now and what `ZAPISZ` would make it; `WROC`, `ZAPISZ`                                                                                                                                                                                         | see Editors                                                      | see Editors                                               |
| 6 Confirm            | `Anuluj` (preselected) or `START`; what the run is: `20 porcji = 503 g` (burst), `100 obrotow` (continuous)                                                                                                                                                                                                                                                                                                                                     | switch                                                           | `Anuluj` → 5, `START` → 7                                 |
| 7 Running            | progress of the run; in burst metering the title carries the mass it should weigh                                                                                                                                                                                                                                                                                                                                    | ignored                                                          | cancel the run → 5                                        |
| 8 Done               | burst: the expected mass, weigh the output. Continuous: weigh the output and enter it in grams                                                                                                                                                                                                                                                                                                                                | burst → 5b, continuous → 5                                       | the same                                                  |
| 9 Machine moving     | run refused or stopped, because the seeder reports the wheel turning                                                                                                                                                                                                                                                                                                  | → 5                                                              | → 5                                                       |
| 10 No dispenser      | dispenser not heard                                                                                                                                                                                                                                                                                                                                                   | ignored                                                          | cancel the run → 5                                        |
| 11 Interrupted       | dispenser stayed in Normal after START instead of calibrating, for `CALIBRATION_START_TIMEOUT_MS` — its side dropped the run (reboot, link gap, clog, cancel)                                                                                                                                                                                                         | → 5                                                              | → 5                                                       |
| 12 Clog alarm        | `ZATKANIE!` / `DOZOWNIKA` / `[OK]`, over any screen. Silent since 29 September 2026 (`CLOG_ALARM_BUZZER`); in burst metering only a clog in the calibration run brings it up                                                                                                                                                                                                  | acknowledge → 13                                                 | acknowledge → 13                                          |
| 13 Clog choice       | `Anuluj` (preselected) / `Odetkaj`, buzzer silent                                                                                                                                                                                                                                                                                                                     | switch                                                           | `Anuluj` → screen underneath, `Odetkaj` → 14              |
| 14 Unclogging        | progress of the reverse/forward sequence the operator asked for with `Odetkaj`, over any screen. Burst metering's own unclog after failed bursts (`AutoUnclogging`) shows nothing                                                                                                                                                                                                                                                                                                             | ignored                                                          | ignored                                                   |
| 15 Tramlines         | `SCIEZKI`, the switch `WL.`/`WYL.`, `ZAPISZ`                                                                                                                                                                                                                                                                                                                          | move the cursor                                                  | switch: flip it and store it straight away; `ZAPISZ`: → 1 |
| 16 Nasiona | `Male nas.` / `Duze nas.` (a square marks the one in use) / `Kalibracja` / `Wroc` | next row | a size: select and store it at once; `Kalibracja` → 17; `Wroc` → 1 |
| 17 Calibrate? | `Kalibracja kola` and which size, `Anuluj` (preselected) / `OK` | switch | `Anuluj` → 16; `OK` → 18, or → 19 with `Brak siewnika` if the seeder is not heard |
| 18 Driving | the pulses counted since START, in large digits. No distance: it could only be shown using the value being replaced, and the 100 m is measured on the ground. Can be driven during normal work — it only reads the seeder's cumulative counter | ignored, so a stray press cannot throw away a 100 m drive | finish → 19 |
| 20 Ustawienia | everything the board keeps in NVS, and two rows: send it over USB (the baud rate is on the row) or leave. The only way settings leave the board — nothing can write them back in, see docs/calibration-settings.txt | switch | send, printing the block and showing `Wyslano!`; `Wroc` → 1 |
| 19 Result | the measured mm per pulse against the one it would replace, or why the run gave nothing | switch (nothing to switch on a failure) | `Anuluj` → 16; `ZAPISZ` stores it for the size being calibrated → 16 |
| 21 Dmuchawa | `ALARM DMUCHAWY`, the switch `WL.`/`WYL.`, `ZAPISZ`; `do resetu` while it is off. For testing a stationary machine: never stored, so every power-up starts with the alarm on | move the cursor | switch: flip it, in force at once; `ZAPISZ`: → 1 |
| 22 Testy | `TESTY`: the tests, a row each at text size 1, then `Wroc`. `Symulacja 15` only in burst metering. Room for more tests (`TestRow` in `src/tractor/main.cpp`) | next row | a test: open it; `Wroc` → 1 |
| 23 Simulation? | `Symulacja siewu`, `7.0 km/h, 15 min` (`SIMULATION_SPEED_MM_S`, `SIMULATION_MINUTES`), `Anuluj` (preselected) / `START` | switch | `Anuluj` → 22; `START` → 24 |
| 24 Simulating | the clock by the tractor's own time since the dispenser started, the bursts, the failed ones with their share, the unclog sequences, as the dispenser reports them. The yellow LED blinks for each failure as in work | ignored, so a stray press cannot throw away a quarter of an hour | stop → 28 |
| 25 Done | the share of failed bursts at text size 2, the counts under it | → 22 | → 22 |
| 26 No dispenser, 27 Machine moving | as 10 and 9, titled `Symulacja` | 26: ignored; 27: → 22 | 26: stop → 28; 27: → 22 |
| 28 Stopped | 25 for as far as it got, titled `Symulacja przerwana` — after a long press on 24 or 26, or when the dispenser dropped the run (in Normal for `CALIBRATION_START_TIMEOUT_MS`) | → 22 | → 22 |

**The burst calibration, step by step** ([Burst metering](#burst-metering) → The calibration run): set the dose on 4 first; `Kalibracja` opens 5 at the stored angle factor, with what 20 bursts should weigh under it. `TEST` → `START` fires them; any press on `GOTOWE` opens 5b at the expected mass. Enter what the scale says and `ZAPISZ`: the factor becomes factor × expected ÷ weighed, and 5 comes back with the cursor on `TEST` for the next run. Or change the factor by hand on 5 — a little more if the run weighed short.

### Editors (4, 5 and 5b)

- Open with the saved value, cursor on the first digit. 5b opens at what the run should weigh instead, so its `ZAPISZ` changes nothing until a digit does.
- The top-right label says what a short press does: `WYBOR` moves the cursor, `ZMIEN` changes the digit.
- The cursor goes through each digit, then the actions, then back to the first digit: `WL./WYL.` or `TEST`, then `ZAPISZ` — on 5 in burst metering `TEST`, `MASA`, `ZAPISZ`, and on 5b `WROC`, `ZAPISZ`.
- Long press on a digit switches to `ZMIEN`, where a short press adds 1 (9 → 0). Long press again to go back to `WYBOR`.
- Long press on `WL./WYL.` switches the dispenser on or off straight away. The setting is stored with `ZAPISZ`.
- Long press on `TEST` opens 6. Coming back from 6–11 keeps the digits as they were, with the cursor on `TEST`. In burst metering `TEST` first stores the angle factor on screen — the run tests what the operator sees — and so does `MASA`, which opens 5b.
- Long press on `ZAPISZ` stores the value and returns to 1. It's the only way out, so leaving an editor always saves. 5b is the exception, being only a way to the factor: its `ZAPISZ` sets the factor to factor × expected ÷ weighed (rounded, kept within 1–999; a mass of 0 leaves it alone) and `WROC` leaves it as it was, and both go back to 5 with the cursor on `TEST`.
- Saving a new dose on 4 scales the angle factor with it (40 → 50 kg/ha takes 500 to 625, kept within 1–999; a dose of 0 on either side leaves it alone), because the angle a pulse needs is in proportion to the dose. It is done in both kinds of metering, so the factor stays right if the mode is switched.

### On every screen

- **Buzzer:** beeps ¼ s on, ¼ s off while a dispenser fault is active, including on screens that don't show the fault. It also beeps, with nothing on screen, when a board that was heard has been silent for about 6 s, or when the seeder and dispenser haven't heard each other for about 6 s. Nothing about clogs beeps: the clog alarm (12) is silent since 29 September 2026 (`CLOG_ALARM_BUZZER`, the user's call — it distracted more than it helped).
- **LEDs:** green = all links fine. Blue blinking = a link is down, including the seeder not heard since power-on. Yellow = the seeder's last report says the tramline relay is on; **two quick blinks** (`BURST_FAIL_BLINKS` × `BURST_FAIL_BLINK_MS`, 0.4 s in all) = a burst failed on the dispenser — the only sign of one, by the user's choice. While the relay holds the LED lit, the blinks go dark instead. The tractor takes the dispenser's failure count as it finds it at boot and after every loss of contact, so a link gap or a dispenser reboot never blinks.
- **Two faults can take over the work screen**: the blower (3a), on at every power-up and off only while screen 21 says so, and the dispenser over-speed (3b), always on. Nothing is acknowledged - they clear themselves when the machine does, and until then the buzzer runs and a long press is the only thing that works. A lost link is not one of them: it is the `BRAK:` letters, the blue LED and, after `LINK_BUZZER_DELAY_MS`, the buzzer. There was a WOM alarm; it was removed on 18 September 2026 because no WOM sensor exists.

## Dispenser behaviour

The dispenser's decision logic lives in `src/dispenser/dispenser_logic.h` — a header-only, hardware-free module (no clock reads, no pins, no serial, no globals; all state inside `DispenserLogic`), so the bench test drives it with simulated time, encoder counts and packets. `dispenserStep()` runs at most every `MOTOR_CONTROL_INTERVAL_MS`, measures the shaft RPM from the encoder delta first (64-bit maths, clamped), then does the counter handling, the mode table, the reversal guard, and stores the output — which is what `dispenserFillStatus()` reports. Every mode change resets the controller (integral 0, target 0, clog timer stopped).

**Tractor command counters.** `clogClearSeq` (Anuluj) and `unclogSeq` (Odetkaj) are counters repeated in every packet. The dispenser compares them with `!=` (never `>`, they wrap), acts once per change, and only outside a resync: the first command after boot or a link gap (`tractorWasAlive` was false) or after `upTimeMs` went backwards (tractor reboot) just copies the counters and acts on nothing. `calibrationArmed` is set only by a received command with `calibrationRun == 0` from an alive tractor, and cleared when a run starts or is refused — that is what stops a run from restarting by itself after a dispenser reboot, a link gap, a clog or a cancel.

**Modes.** `DispenserStatus.faultCode` is only meaningful in Normal:

| Mode                     | What one step does                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Leaves when                                                                                                                    |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| Normal                   | A calibration request (alive tractor, `calibrationRun` 1, armed) → `Refused` if the seeder reports the wheel turning, else `Calibrating`, motor off either way. A simulation request (`simulationRun` 1, armed the same way) likewise → `Refused` or `Simulating` - `Refused` in continuous metering too. Otherwise meters: no seeder → fault `NoSpeedData`, motor off. Not moving, speed 0, dispenser off or dose 0 → fault `None`, motor off. Else the rate = `requiredShaftRPM(speed, dose, calib)`, clamped to `MOTOR_MAX_RPM` (fault `OverSpeed`), plus the distance ledger's trim, never below half the rate; duty from the controller, clog check. In burst metering, bursts instead of the rate ([Burst metering](#burst-metering)), and a stalled burst fails and is tried again with the next pulse — no clog. The tractor being gone is ignored on purpose — the last dose is held | clog detected → `Clogged` (continuous); `BURST_FAILS_BEFORE_UNCLOG` failed bursts in a row → `AutoUnclogging` (burst), motor off in that step |
| Calibrating              | Tractor lost, no command or `calibrationRun` 0 → `Normal`, motor off. Seeder reports the wheel turning → `Refused`, motor off. `CALIBRATION_TOTAL_EDGES` edges turned → `CalibrationDone`, progress 100, motor off. Else progress from the edge count, target `CALIBRATION_RPM`, duty from the controller, clog check; fault `None` throughout                                                                                                                                                                         | clog detected → `Clogged`                                                                                                      |
| CalibrationDone, SimulationDone, Refused | Motor off                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | tractor alive with `calibrationRun` and `simulationRun` both 0 → `Normal` (a refusal can be either run's). Nothing else leaves; the motor is off, so holding is safe |
| Clogged                  | Motor off until the operator decides; calibration requests ignored                                                                                                                                                                                                                                                                                                                                                                                                                                                     | `clogClearSeq` change → `Normal`, motor off; `unclogSeq` change → `Unclogging`, first phase (reverse) output in that same step |
| Unclogging               | Phase from `t % UNCLOG_CYCLE_MS`: reverse `UNCLOG_PERMILLE` for `UNCLOG_REVERSE_MS`, pause (duty 0, forward) for `UNCLOG_PAUSE_MS`, forward `UNCLOG_PERMILLE` for `UNCLOG_FORWARD_MS`, pause; progress from `t / UNCLOG_TOTAL_MS` (phases: `unclogOutput()`). No clog check — the encoder counts up in both directions, so it cannot be used here                                                                                                                                                                                                 | `clogClearSeq` change → `Normal`, motor off; tractor lost or `UNCLOG_TOTAL_MS` elapsed → `Clogged`, motor off                  |
| AutoUnclogging           | Burst metering's own unclog after failed bursts: the same phases (`unclogOutput()`), nobody asked for it and nobody waits on it — the tractor shows nothing. Runs to the end whatever the tractor does                                                                                                                                                                                                                                                                                                  | `UNCLOG_TOTAL_MS` elapsed → `Normal` (metering starts afresh); the dispenser off, a dose or angle factor of 0 → `Normal` at once |
| Simulating               | The seeding simulation, standing still: burst metering exactly as in work (`burstMeter()`, failed bursts, `failBurst()`), on the pulses `SIMULATION_SPEED_MM_S` would give at the tractor's distance per pulse; `BURST_FAILS_BEFORE_UNCLOG` failures in a row run the unclog phases inside it (`simUnclogging`), metering afresh after them. Counts the bursts, the failed ones and the unclogs for the status; progress from the time; fault as in work | tractor lost, no command or `simulationRun` 0 → `Normal`; the seeder reports the wheel turning → `Refused`; `SIMULATION_DURATION_MS` elapsed → `SimulationDone`, progress 100, motor off |

**Clog check** (continuous metering's Normal, and Calibrating; burst metering's work has failed bursts instead — [Burst metering](#burst-metering). Only while the controller is actually driving the motor — a step that leaves the duty at 0 is not a stall, so it stops the timer): shaft slower than `CLOG_MIN_SPEED_PERCENT` of the target for a continuous `CLOG_DETECT_MS` → `Clogged`: duty 0 forward in that same step, controller reset, progress 0, fault `None`. One step at speed stops the timer. The guard covers the case where a large negative integral (a motor running faster than the feed-forward expects, i.e. a tractor at ~14 V) has cancelled the feed-forward and the duty sits at 0 with the shaft stopped — that is not a clog, and the clamp in the controller is what stops it happening at all. It hides no real clog: a stopped shaft makes the error, and so the duty, positive.

**Distance ledger.** Metering follows the ground, not the speed estimate. One wheel pulse is a fixed distance (`TractorCommand.wheelMmPerPulse`, about 1.6 m with the 3 magnets), a fixed distance is a fixed number of shaft turns (`revolutionsPerMetre()`), and the encoder says how many turns were really made — the difference is a debt in revolutions, and `ledgerCatchupRPM()` returns the RPM added to the rate to pay it off over `LEDGER_CATCHUP_SECONDS`. It is worth the state because the speed estimate is coarse: the seeder averages five turns of the metering drive (~24 m; one turn, ~4.7 m, until 28 September 2026), and the motor needs a few seconds to settle after every start. The ledger turns what that costs from fertilizer never applied into fertilizer applied a few metres later. Between pulses the ground is interpolated from the reported speed, capped at one pulse's distance — without that the debt would step by a whole pulse (~5 revolutions at 40 kg/ha and 500 g per 100 turns) and the target would saw up and down at the pulse rate. Four rules keep it safe:

- the debt is capped at `LEDGER_MAX_METRES` of travel and the trim at `LEDGER_MAX_CATCHUP_RPM`;
- `resetController()` wipes it, so a debt never survives a stop, a clog or a mode change and gets paid off in one spot afterwards;
- nothing is recorded while the rate is clamped (`OverSpeed`): a debt earned with the motor already at its limit cannot be repaid without double-dosing the strip that follows, and `ZA SZYBKO` is what the operator acts on;
- it can never take the target below half the rate, so a ledger that has gone negative — a real over-application, or an encoder counting edges that never happened — cannot stop the auger while the machine is moving.

**Controller.** Feed-forward (`target × 1000 / MOTOR_MAX_RPM`) plus a gentle PI trim with anti-windup: the integral only updates when the raw output is not saturated — `raw > 1000` with positive error, or `raw < MOTOR_MIN_RUNNING_PERMILLE` with negative error — otherwise a long spell at full duty or at a low target would take seconds to unwind. The integral is then clamped to `−feedForward`: the trim may cancel up to all of the feed-forward, never more, or a negative integral left over from a higher target (a motor running faster than the feed-forward expects) would hold the duty at 0 and the motor simply never restart after slowing down, with no alarm. Duty is 0 or at least `MOTOR_MIN_RUNNING_PERMILLE`, never in the buzzing band. A reversal guard drops to duty 0 whenever two consecutive steps would otherwise drive in opposite directions at speed.

### Burst metering

**On (`DISPENSER_BURST_MODE = true`) on the branch `feature/fertilizer-dispenser-bursts`, since 28 September 2026.** The fitted motor cannot turn the loaded auger at a low duty: at the low targets of slow driving it stalls and the clog alarm fires — though the continuous calibration run's 120 RPM turned it well. Part of that is the controller above: at a low target the PI integral climbs a couple of permille a step, so the clog detector gives up long before the duty gets near full. With the switch on, Normal mode never runs the rate controller — every wheel pulse is dosed as one burst at `BURST_PWM_FRACTION` of full PWM (1.0, flat out, unless changed). Everything else in the mode table holds; only the metering in Normal and the calibration run's drive change. `false` brings back exactly the continuous metering above, for a motor that can hold a speed. The logic reads the switch and the PWM through `DispenserLogic.burstMode` and `.burstPermille`, set by `dispenserInit()`, so the bench can drive every kind from one build.

- **The portion is set by distance, never by time, and its angle by the tractor's angle factor.** Each wheel pulse (`TractorCommand.wheelMmPerPulse` of ground) gets one burst of `burstRevsPerPulse()` shaft turns: the angle factor F (`TractorCommand.burstAngleFactor`, protocol v6; 0–999 from the tractor) in thousandths of what the motor turns at the burst PWM — taken as `BURST_ANGLE_REFERENCE_RPM` (300) × `BURST_PWM_FRACTION` — in the time between two pulses at `BURST_ANGLE_REFERENCE_SPEED_MM_S` (10 km/h). So F/1000 is about how busy the motor is at 10 km/h, and F = 500 is 1.41 turns a pulse at 1571 mm. The reference is only a scale — F is calibrated by weighing — and the distance per pulse is in both the reference and the ground, so one F holds for either seed size. The dispenser no longer works the angle out from the dose and the grams-per-100 calibration (the user's design, 28 September 2026): the tractor keeps F in proportion to the dose, the dispenser only checks that there is a dose, and grams per 100 turns is continuous metering's alone. No speed and no RPM enter into it: slower driving brings the pulses, and so the bursts, less often. The pulse is the finest distance the machine measures, so a portion is one pulse's worth — with the 3 magnets, about 1.6 m. More magnets would make the portions smaller and more frequent with no firmware change beyond a new wheel calibration (six were tried and could not be told apart, 27 September 2026).
- **A burst runs at `BURST_PERMILLE` until the encoder has counted the portion, then stops.** `BURST_PERMILLE` is `BURST_PWM_FRACTION` × 1000 — 1000 flat out, 750, 500 and so on for gentler bursts. There is no speed control: the encoder counts the angle, so how fast it is turned changes nothing in the dose, and a gentler burst simply turns slower and lasts longer — one burst per pulse, the portion and every alarm stay the same (agreed with the user, 28 September 2026: a share of the PWM, not a speed). The speed matters in only three places, none of which needs a controller: grams per revolution can depend on it, which is why the calibration run uses the same PWM (calibrate with the engine running, since the battery voltage moves an open-loop speed by about 15 %); the top working speed (below); and torque, which falls with the PWM — so `burstDuty()` gives a shaft that slows under the stall line after a whole step of being driven full PWM at once, and the burst's own PWM again as soon as it turns. Only a shaft that full PWM cannot move either fails the burst. At full PWM that exception changes nothing. `burstMeter()` is the distance ledger without the rate: `owedRevolutions` gains each pulse's portion, the encoder takes off what was turned, and the motor runs while there is at least `BURST_MIN_EDGES` to start a burst, and to its count once it has started. `dispenserBurstStop()` ends a burst on its exact count on every call of `dispenserControlTick()` — every `loop()` — rather than at the next 100 ms step, which at full speed is over half a turn. The few edges the shaft turns while braking are owed back by the next burst, so they never add up over a pass. The one exception is a dose so small that a pulse asks for less than `BURST_MIN_EDGES`: that waits for a few pulses, and after the last pulse of a pass up to that much stays owed.
- **No speed interlock**: no pulse, no burst. The seeder needs two pulses after every stop before its speed says "moving", and waiting for that would lose the first pulse's ground at every start. After the machine stops only what the last pulse still owes is turned — ground it has already covered — then nothing.
- **Behind, `ZA SZYBKO`**: a pulse arriving while the motor is still more than `BURST_LATE_FRACTION` (one) pulse's turns behind on the earlier ones is late, and `BURST_LATE_PULSES` (2) late pulses in a row raise `OverSpeed`. It holds while the motor stays behind and clears as soon as a pulse finds it caught up. A single late pulse is usually the radio: a lost or late packet starts one burst late. The backlog is capped at `BURST_MAX_BACKLOG_PULSES` (3): what would go past it is dropped, so slowing down never dumps more than that. It is 3 rather than 2 because a radio gap just under `LINK_TIMEOUT_MS` can deliver two or three pulses in one packet, and all of them are owed. The negative side is capped at one pulse (or a minimum burst, if that is more, so a burst's braking is always owed back in full), so an encoder counting edges that never happened can cost one burst at most. A pulse that arrives while the shaft is stalled is not counted as late either way: the auger is stuck, which is a failed burst's business, not the speed's — without that, a stall long enough for two pulses flashed `ZA SZYBKO` (found by B31 with other `BURST_FAILS_BEFORE_UNCLOG` values).
- **Failed bursts, and no clog alarm in work** (the user's rules, 29 September 2026: in the field the alarm came about four times in 700 m, and after `Anuluj` the auger always went on). A burst has failed when the shaft stays under `BURST_CLOG_MIN_RPM` (60), or under a third of what the burst PWM turns a free motor at (`burstNominalRPM()`) if that is lower — 54 at half PWM — for `BURST_FAIL_MS` (1.5 s) while the burst drives it. Then, in that step: motor off; what the burst still owed kept, but one pulse's worth at most — enough to fill the gap it leaves, while a pulse arriving behind on it is not late; nothing more tried until the next wheel pulse, whose burst takes both (the pause is what `Anuluj` gave the jam); and the failure counted, `DispenserStatus.burstFailures`, on which the tractor blinks its yellow LED twice. A burst that reaches its count ends the run of failures. `BURST_FAILS_BEFORE_UNCLOG` (3) in a row start the unclog sequence by themselves — `AutoUnclogging`, the same reverse/forward phases as `Odetkaj`, 4 s — and then metering simply goes on, for as long as it takes: burst metering never goes `Clogged` in work, and the tractor shows nothing for any of it. The ground covered during the unclog is not dosed (the mode change resets the ledger). The calibration run still stops as `Clogged` after `CLOG_DETECT_MS`, silently: the unclog's reverse turns would count in its weighing.
- **A push backwards after every burst** (the user's idea, 29 September 2026): with the gearbox's teeth left pressed together by the last burst, the next one starts from a dead point, so the motor gets no run-up to push through a stiff spot. `BURST_BACKLASH_DELAY_MS` (30) after a burst stops — the brake has the shaft still by then; sooner, the push would only brake harder — the motor is driven backwards at `BURST_BACKLASH_PERMILLE` (100, 10 %) for `BURST_BACKLASH_MS` (50), then off. It is not measured and is not meant to turn the auger, only to open the backlash so the next burst's motor spins freely for a moment before the teeth take the load. After every burst in work, in the calibration run and in the simulation, and after a failed one too — then it also gives the retry its run-up — but not after the failure that starts the unclog sequence, which reverses anyway. No burst starts until the push is over (`burstMeter()` and `calibrationBurst()` hold for it, and a step that lands in it keeps it going); a mode change or the dispenser switched off ends it at once (`resetController()`). The push runs on its own clock (`dispenserBacklashTick()`, every `loop()`, and every step as well), being shorter than a control step. The encoder cannot tell direction, so the push's few edges count as turned — a slight shortfall per burst, taken in by the calibration run, which pushes back after its bursts too; recalibrate after changing the push. `BURST_BACKLASH_MS` 0 turns it off.
- **What throws the owed turns away**, as for the ledger: no seeder, the dispenser off, a dose or angle factor of 0, any mode change (the automatic unclog included). A failed burst keeps at most a pulse's worth. A seeder reboot only takes fresh snapshots. Losing the tractor changes nothing — the last command is held.
- **The calibration run fires real bursts, and the tractor turns the weighing into the angle factor** (`calibrationBurst()`; the user's method, 28 September 2026). There is always one burst per pulse, its angle set by F. So the run is `CALIBRATION_PULSES` (20) pulses as if driving at `CALIBRATION_SPEED_MM_S` (1667, 6 km/h) — one every distance-per-pulse at that speed, with the tractor's distance for the seed size in use (0.94 s at 1571 mm) — each dosed exactly as in work: one burst, to the angle F sets, at the burst PWM, carrying straight on when the next pulse is due before one is done. Every pulse's angle is turned in full even when the motor falls far behind: the field's backlog cap does not apply here, or the weight would come out short. What 20 pulses of ground should get is **expected [g] = 8 × distance per pulse [m] × dose [kg/ha]** at 4 m — 503 g at 1571 mm and 40 kg/ha (`expectedCalibrationGrams()` in `src/tractor/angle_factor.h`) — and the tractor shows it on screens 5, 6, 7, 8 and 5b, following the dose and the seed size by itself. After the run, 5b takes the weighed grams and sets **F = F × expected ÷ weighed** (rounded, 1–999): the run's angle, and so its weight, is in proportion to F. Run it again until the weight is close, or nudge F by hand on 5. F = 0 leaves nothing to turn, and the run ends at once. A new dose scales F in proportion by itself, so it needs no new run to be about right, though one is worth doing after a large change. **Redo the check after switching the mode, in either direction, and after changing `BURST_PWM_FRACTION`, the seed size or the reference constants.**
- **F stops at 999, on purpose** — about all the motor turns at full PWM between two pulses at 10 km/h. The user chose that reference so 999 is the machine's physical limit: it is never driven faster than 10 km/h (8 in practice), and the motor can do no more than full PWM (28 September 2026). At full PWM a dose needs F = 6 × width [cm] × dose [kg/ha] × 2778 / (300 × grams per 100 turns): at 4 m, more than 999 whenever the auger gives less than about 890 g per 100 turns at 40 kg/ha (445 g at 20). The run then still weighs short at 999, and 5b shows the correction capped (`Kat 500>999`). **The remedy is mechanical** — more grams per turn, e.g. a wider dispenser opening — not a different `BURST_ANGLE_REFERENCE_SPEED_MM_S`, which would only hide the limit (agreed with the user). The 300 RPM is an assumption: if U01 shows the loaded motor turning slower, the real limit is lower in proportion, and `ZA SZYBKO` sounds when it is reached.
- **The status**: `measuredShaftRPM` is the shaft's average over about `BURST_RPM_AVERAGE_MS` (2 s), which is what the work screen shows — the shaft itself alternates between the burst's speed and standing. `motorRunning` is the motor as it is, so the marker on the work screen runs during bursts only. `targetShaftRPM` is `burstNominalRPM()` — `MOTOR_MAX_RPM` at full PWM — while a burst runs, and 0 between. `burstFailures` counts the failed bursts, wrapping. Protocol v6 carries the angle factor, v7 the failure count and `AutoUnclogging`, v8 the simulation, so all three boards need the same build.
- **The seeding simulation** (`Simulating`; the user's endurance test, 29 September 2026 — the clogs came after a few passes, not at once). Asked for from the tractor (`Testy` → `Symulacja 15`) with `TractorCommand.simulationRun`, armed and refused exactly like the calibration run (and refused in continuous metering). For `SIMULATION_MINUTES` (15) the dispenser meters as in work — `burstMeter()`, the angle factor, failed bursts retried, `BURST_FAILS_BEFORE_UNCLOG` in a row unclogged — standing still, on the pulses `SIMULATION_SPEED_MM_S` (7 km/h) would give at the tractor's distance per pulse: 1.24 a second at 1571 mm, 1114 in all. The unclog runs inside the simulation (`simUnclogging`) rather than as `AutoUnclogging`, so the tractor's screen stays on it, and metering starts afresh after it as in work. `DispenserStatus` carries the bursts, the failed ones and the unclogs, kept after the run until the next one starts; the tractor shows the failed share at the end, and the yellow LED blinks for every failure as in work. A burst that reaches its count clears the run of failures here too, so the result is what 15 minutes of work at 7 km/h would have given — the auger, the fertilizer and the motor are the real ones; only the ground is not.
- **The limit is still the motor.** A portion has to be turned before the next pulse comes, so the top working speed is about 10 km/h × 1000 ÷ F × (the loaded motor's real full-PWM RPM ÷ 300) — whatever the PWM fraction, since F's reference scales with it: F = 900 gives about 11 km/h at 300 RPM. In grams it is the design record's arithmetic, v_max [m/s] = RPM × C / (60 × 40 × D) at 4 m, with C in g per 100 revolutions and D in kg/ha: at 300 RPM, 500 g and 40 kg/ha, 1.56 m/s or 5.6 km/h. Bench test U01 prints the RPM the bursts really turn at.
- **Simulated before use.** The B tests (`src/dispenser_bench/tests_burst.h`, `d` on the bench) were run on a PC against this header, with a 1 ms model of the ground, the seeder's packets, a shaft that spins up and brakes — at a speed set by the duty, less what the load takes off — and `dispenserBurstStop()` on every tick. All 41 checks pass, with `BURST_PWM_FRACTION` at 1.0 and at 0.5 (the tests set their own PWM), and with `BURST_FAILS_BEFORE_UNCLOG` at 2 and 5 and `BURST_FAIL_MS` at 0.7 and 3 s (at 1, B11 and B32 are skipped): 100 pulses at 1000 mm/s turn 251.23 of the 251.21 revolutions asked for, one burst per pulse, at most 11 edges past each count (246 without the stop between steps). The calibration run at 2.5 m per pulse and an angle factor of 889 fires 20 separate bursts, each within a control step of its pulse and each one pulse's angle (1920–1929 edges for 1920), 38 411 edges for 38 402 in all; at the tractor's top factor, 999, with an auger so heavy the motor turns it at 100 RPM — 1.41 turns a pulse every 0.47 s, beyond it — it turns all 28.2 turns as one burst. B30 runs the whole procedure through the tractor's own arithmetic (`src/tractor/angle_factor.h`): with an auger giving 15 g a turn, the first-boot factor 500 weighs 424 g for 503 expected, the correction takes it to 593 and the next run weighs 503 g; halving the dose takes the factor to 297 and the run to 252 g for 251. At half PWM every burst runs at exactly 500 permille and the dose is exact; a heavy auger simply turns slower (73 RPM) with no alarm; one too heavy for half PWM gets the full-PWM push whenever it slows under the stall line and keeps turning, no burst failed; held for good the burst fails after 1.6 s. Failed bursts (B11, B31, B32): blocked mid-burst, the burst fails after 1.66 s, keeps one pulse's 2.51 turns, waits for the next pulse, and freed makes them up (7.56 turns for 7.54); held for good at a pulse every 785 ms (8 km/h on the machine), the third failure unclogs by itself — reverse first, 4.0 s — metering resumes, and still blocked it goes round again, never `Clogged` and no `ZA SZYBKO`; two failures, one burst through and two more do not unclog; the unclog goes on without the tractor and stops when the dispenser is switched off. The simulation (B33, B34), on the machine's 1571 mm and factor 500: a minute of it fires 74 bursts for 74 pulses, 104.64 turns for 104.62, and ends as `SimulationDone` on the clock; held for good, it unclogs inside itself after three failures and goes on. The push backwards (B35): 30 ms after each stop, 100 permille for 50 ms, no burst inside it; 20 pushes for the calibration run's 20 bursts, one after each failed burst but the one that unclogs, cut short by switching off. Writing them found two faults, both fixed: a burst stopped short once less than `BURST_MIN_EDGES` was left, so the last pulse of every pass came up short (B01), and for a small dose the backlog cap sat below a minimum burst, so nothing was ever dosed (B15). `ZA SZYBKO`'s thresholds come from a sweep across the motor's limit on a bad radio (the numbers are in `include/machine_settings.h`). The same PC run gives the D and L tests byte-identical output with and without this change, so continuous metering is untouched.

## Current tasks

**No open tasks.** Tasks 1–7 were reviewed and moved to [Completed](#completed). The dispenser module is built and has passed the bench test; the seeder needs both seed-size gears calibrated (`Nasiona` → `Kalibracja`) before the machine meters correctly. It keeps its 3 magnets. Remaining work is in [Future tasks](#future-tasks).

**Burst metering is on — branch `feature/fertilizer-dispenser-bursts` (28 September 2026).** The fitted motor cannot turn the loaded auger slowly, so until it is replaced the dispenser doses each wheel pulse in one burst ([Burst metering](#burst-metering)), at full PWM by default; `BURST_PWM_FRACTION` in `include/machine_settings.h` makes the bursts gentler (0.75, 0.5, …) without changing anything else. The angle of each burst is the tractor's **angle factor** (0–999, first boot 500), set on the `Kalibracja` screens. **Flash all three boards from this build** — protocol v6 carries the factor. The seeder's own change is the smoother speed below. **Calibrate before sowing:** set the dose, then `Kalibracja` → `TEST` → `START` fires 20 real bursts spaced as at 6 km/h; weigh the output, enter the grams on the mass screen that follows, `ZAPISZ` corrects the factor; repeat until close (step by step under [Tractor screens](#tractor-screens), after the table). Redo it after changing `BURST_PWM_FRACTION` or the seed size; a new dose rescales the factor by itself. If the correction stops at 999 and the run still weighs short, the dose is beyond what the motor delivers at 10 km/h — the intended limit; the remedy is more grams per turn, e.g. a wider dispenser opening ([Burst metering](#burst-metering)). When the new motor is fitted: `DISPENSER_BURST_MODE = false` in `include/machine_settings.h`, reflash the tractor and the dispenser (the tractor reads it for its calibration screens), recalibrate.

**The speed on the work screen is averaged over 15 pulses (28 September 2026)**, asked for by the user because it twitched: the seeder now averages five turns of the metering drive instead of one (`WHEEL_AVERAGE_INTERVALS`), and a pulse only counts as late once it is later than the longest gap in that average. The second half matters as much as the first: measured against a single gap, the late-pulse rule pulled the number down for a moment in about every other gap, and with the 15-gap average alone it still dipped 8 %. Simulated at 8 km/h with every gap ±5 % at random, the number stays within 7.8–8.1 km/h (7.4–8.3 before) and changes about a third as often; a stop shows as quickly as before, but a change of speed takes 10–20 s to settle on screen. Burst metering never reads the speed, so the dosing is unaffected. If it reacts too slowly, lower `WHEEL_AVERAGE_INTERVALS` to 9 or 6 (a multiple of 3; W12 then allows a little more scatter) — 3 brings the twitch back, and W12 fails on it.

**No clog alarm in burst metering's work (29 September 2026, in the field)**, the user's rules: the alarm came about four times in 700 m and `Anuluj` always let the auger go on. A burst that stalls for `BURST_FAIL_MS` (1.5 s) is stopped and tried again with the next pulse, keeping one pulse's worth to make up; `BURST_FAILS_BEFORE_UNCLOG` (3) in a row run the unclog sequence by themselves, and metering goes on after it. Nothing on the screen and no buzzer — the yellow LED blinks twice per failed burst, and that is all; the clog alarm that the calibration run can still raise is silent too (`CLOG_ALARM_BUZZER`). **Protocol v7** (the failure count, `AutoUnclogging`): flash all three boards again; the seeder has no change of its own. Worth watching in the field: how often the LED blinks, and whether the unclog cures a run of failures — if single failures that the next pulse gets through are common, a shorter `BURST_FAIL_MS` would cut the gap each one leaves ([Burst metering](#burst-metering) → Failed bursts).

**The seeding simulation and the tramline relay (29 September 2026, in the field).** `Testy` → `Symulacja 15` on the tractor runs 15 minutes of work at 7 km/h standing still, failures and unclogs included, and ends with the share of failed bursts — the user's test for clogs that come after a few passes ([Burst metering](#burst-metering) → The seeding simulation). And the seeder drives its relay active-HIGH now: the fitted board closes on a high input, and the tramlines were the wrong way round. **Protocol v8: flash all three boards** — the seeder has the relay change of its own.

**A push backwards after every burst (29 September 2026)** — the user's theory: the last burst leaves the gear teeth pressed together, so the next starts from a dead point. 10 % of full PWM backwards for 50 ms, 30 ms after every burst stops, everywhere bursts run ([Burst metering](#burst-metering) → A push backwards). Dispenser only, no protocol change. **Recalibrate after flashing it**: the encoder counts the push's few edges as turned, and the calibration run pushes back too, so the weighing takes that in. Worth comparing: the simulation's share of failed bursts with the push and without it (`BURST_BACKLASH_MS = 0`).

**Before any board goes on the machine:** protocol v8 (v7, v6 and v5 before it) changed the packet layout, so the tractor, the seeder and the dispenser must all be flashed from the same build. A board left on older firmware silently ignores the others.

**`BUTTON_DEBUG` is on in `src/tractor/main.cpp` (temporary, 27 September 2026).** Every button event — each press, spark, let-go, long press and what `loop()` did with it, with times — goes to serial at 115200, to pin down presses dropped after a long press and taps faster than about three a second. Set it to 0, or delete the `#if BUTTON_DEBUG` blocks, before the tractor goes to the field; with 0 the firmware is exactly the production one.

**The tractor's button moved from GPIO12 to GPIO32 on 27 September 2026.** Rewire it and fit the 470 Ω pull-up and the 10–100 nF capacitor ([Hardware and pins](#tractor-esp32-srctractormaincpp)) in the same session as flashing that firmware: new firmware on the old wiring, or the other way round, leaves a button that does nothing. No protocol change — only the tractor needs reflashing for it.

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
  - Dispenser module on the bench: **the bench test image ran on 17 September 2026** ([docs/dispenser_module_hardware.md](docs/dispenser_module_hardware.md) §6). Supply, driver, motor, both encoder channels, direction, radio, metering, link loss and the calibration run passed. **Not run yet:** the K tests (clog and unclogging with the motor; only K07 needs the lever) and the stall tests H06 and K07 (no fuse on the bench). The W, L and M11 checks added on 18 September 2026 have never run on the board either - they need no hardware beyond the module itself (`w` and `d`), so run them with the next bench session. Nor has P01, the simulated pass on its own key `p` (added 25 September 2026). That run's four failures were bugs in the tests, fixed afterwards: D08 and D20 expected the calibration target on the step that enters Calibrating, which resets the controller; C04 counted the shaft coasting down from metering; and `benchPrepare()` started the logic before its 1 s wait, so the first control step of every motor test integrated a whole second of error — M02 measured 215 RPM for a 192 RPM target (a simulation of the controller reproduces 214).
  - All three boards on a desk: pull power from each in turn — the seeder holds its relay, the dispenser stops when the seeder goes, the right `BRAK:` letters appear. With a fault beeping, cut the seeder's power: the buzzer must stop (regression check for the old stuck-buzzer bug).
  - **The tractor button after its rewiring to GPIO32:** on the menu, where every short press moves the cursor, leave it alone for a few minutes, then press slowly and firmly. Repeat several times: it must move every time. Then tap ten times, lifting the finger for a good quarter of a second between taps: ten steps, never more (faster taps are dropped on purpose, `BUTTON_MIN_GAP_MS`). Press and let go slowly and hesitantly a few times: one step each. Open something with a long press and let go slowly: it opens, and nothing else happens. Hold with a light, wobbly finger for a second: one long press. If a slow press after a pause still goes missing, first swap the 470 Ω for 220 Ω (~15 mA, the level car electronics use for a moment at closure to clean contacts). If that doesn't cure it, the switch's contacts are too far gone for 3.3 V — replace it with one that has gold contacts.
  - **Calibrate both gears.** `Nasiona` → `Kalibracja` on the tractor, once in each seed-size gear, driving `WHEEL_CALIB_DISTANCE_M` in the field with the machine working so that wheel slip is part of the number. Until then every board meters on `WHEEL_MM_PER_PULSE_DEFAULT` (1571 mm, worked out from a 600 mm wheel, a 0.4 ratio and 3 magnets), which is an estimate. With 3 magnets 100 m is only ~64 pulses, so one pulse either way is ~1.6 %; driving 200 m halves that. Check the counts differ between the gears by roughly the ratio you measured, and that the seed-rate setting really does not change them. Export the result afterwards (`Ustawienia` → `Wyslij`) and keep the block in [docs/calibration-settings.txt](docs/calibration-settings.txt). Then set `WHEEL_MM_PER_PULSE_DEFAULT` to the measured value for the gear used most and reflash: it is the fallback before the first command arrives, and `WHEEL_MIN_PULSE_GAP_US` - the interrupt's noise filter, which has to be a compile-time constant - is derived from it. At 1571 mm it ignores pulses closer together than ~40 km/h; a measured value much below 800 mm would make that filter start clipping real pulses at 20 km/h.
  - Dispenser calibration: check `ENCODER_EDGES_PER_REV` by hand-turning 10 revolutions (≈ 4800 edges), then `Kalibracja` → `TEST`, catch and weigh the output, enter it.
  - **Stationary dose check** (after the dispenser calibration, with the same fertilizer): machine on stands so the wheel turns the metering drive, `Dmuchawa` → `WYL.`, dispenser `WL.` with a dose. Count pulses on screen 18 (`Nasiona` → `Kalibracja` → `OK`; leave with `Anuluj`, never `ZAPISZ`), turn the wheel steadily and without pauses for 100+ pulses, weigh the output. Expected grams = pulses × distance per pulse [mm] / 1000 × `WORKING_WIDTH_CM` / 100 × dose / 10 — with the distance per pulse the tractor sends for the gear in use (`Ustawienia`, `Kolo M/D`). Each stop and start costs up to about one pulse's worth, and a run that showed `ZA SZYBKO` is not valid.
  - Blockage: stall the running dispenser; the tractor must alarm.
- **Replace the dispenser motor** — the fitted one cannot turn the loaded auger slowly, which is why burst metering is on ([Burst metering](#burst-metering)). With a motor that holds a low speed under load, set `DISPENSER_BURST_MODE = false`, reflash the tractor and the dispenser (the tractor reads the switch for its calibration screens) and recalibrate. Worth knowing when choosing it: in continuous metering the clog detector fires after 1.5 s at a third of the target, long before the PI integral can raise a low duty to full, so a motor that is only marginal at low duty will read as clogged there.
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
- **Ground speed** from the existing ground-wheel sensor: the average of the last 15 gaps between pulses — five turns of the drive, since 28 September 2026, when one turn made the work screen twitch — lowered as soon as a pulse is later than the longest of them, 0 once one is later than `WHEEL_MIN_SPEED_MM_S` allows (`src/seeder/wheel_speed.h`).
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
- **Blower alarm switch** — a seventh menu item `Dmuchawa` (screen 21) turns the blower alarm off until the next power-up, for testing a stationary machine; it shares its drawing with the tramline screen.
- **Distance ledger** — the dispenser meters to the ground rather than to the speed estimate: a debt in shaft revolutions, paid off over `LEDGER_CATCHUP_SECONDS`, with the ground interpolated between pulses. 12 logic checks (L01–L12) and a motor-in-the-loop check (M11) that compares shaft turns against simulated ground; D03, D06, D29, D33 and D35 were re-expressed against the commanded target, which now includes the ledger's trim.
- **Burst metering** (branch `feature/fertilizer-dispenser-bursts`) — for a motor that cannot turn the auger slowly, `DISPENSER_BURST_MODE` doses each wheel pulse in one burst at `BURST_PWM_FRACTION` of full PWM that stops on the encoder count for the angle the tractor's angle factor sets; the calibration run fires 20 of those bursts spaced as at 6 km/h, and the tractor turns the weighed mass into a corrected factor (screens 5 and 5b), scaling it with the dose as well. Protocol v6 (`TractorCommand.burstAngleFactor`). 32 B tests (38 checks) with no hardware, simulated on a PC against the real logic and the tractor's arithmetic, all passing; 4 U tests with the motor, never run on the board yet. The existing D, L, M, C, K and P tests keep testing continuous metering ([Burst metering](#burst-metering)).
- **A push backwards after every burst** (29 September 2026, the user's idea) — 10 % of full PWM backwards for 50 ms, 30 ms after each burst stops, in work, in the calibration run and in the simulation, to open the gearbox's backlash so the next burst gets a run-up; the next burst waits for it. Dispenser only. B35, and the harness's idle and burst counters taught about it.
- **The seeding simulation** (29 September 2026) — `Testy` → `Symulacja 15` on the tractor (a new menu item, room for more tests) runs burst metering as in work for 15 minutes at 7 km/h on simulated pulses, standing still, and reports the share of failed bursts; `DispenserMode::Simulating` / `SimulationDone`, protocol v8 (`TractorCommand.simulationRun`, the counts in `DispenserStatus`). B33 and B34. And the seeder's tramline relay driven active-HIGH, for the board fitted.
- **Failed bursts instead of the clog alarm** (29 September 2026, the user's rules after a day in the field) — in burst metering's work a stalled burst is stopped and tried again with the next pulse, making up one pulse's worth; three in a row run the unclog sequence by themselves (`AutoUnclogging`) and metering goes on; the tractor's only sign is two blinks of the yellow LED per failure, and the clog alarm's buzzer is off everywhere. Pulses arriving during a stall no longer count towards `ZA SZYBKO`. Protocol v7 (`DispenserStatus.burstFailures`, `DispenserMode::AutoUnclogging`). B11, B12 and B26 rewritten, B31 and B32 added, U04 rewritten for the motor.
- **Smoother speed** — the seeder averages 15 pulse gaps (five drive turns) instead of 3, whole turns only while the average builds up after a start, and a pulse counts as late only past the longest gap in it. W02 and W07 re-checked for the longer average (W07 now at every pulse from the first whole turn, which caught the average going off by 2.5 % while it built up, before the whole-turns rule), W12 added: two and a half minutes of 8 km/h with every gap ±5 % at random stay within −1.7 %…+2.1 % (the old rules: −7.9 %…+4.2 %; the long average with the old late-pulse rule: −8.3 %…+2.1 %).
- **Tractor button on a timer, and on GPIO32** — sampled every 5 ms by an `esp_timer` instead of once per `loop()`, where every redraw hid it: a press starts on 10 ms of contact and ends only after 50 ms without it, a long press fires at 750 ms, and a press starting within 250 ms of the last release is dropped, so the bouncing car-style switch clicks exactly once (the user's rules, simulated before coding); presses are queued with their start time; and moved off the GPIO12 strapping pin so it could have a 470 Ω pull-up, because the car-style switch missed the first press after a pause at the internal pull-up's ~0.07 mA. Needs the rewiring in [Current tasks](#current-tasks) (Design record → Tractor UI).

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
| Dispenser | calibration run or simulation, and the tractor is lost                                                                           | run aborts, motor off — the tractor commanded it                                                                                   |
| Dispenser | calibration run and the seeder reports the wheel turning                                                             | refused or stopped, motor off                                                                                                      |
| Dispenser | clog detected                                                                                                        | motor off until the operator decides — neither metering nor a calibration run restarts by itself                                   |
| Dispenser | burst metering, in work: a burst stalled for `BURST_FAIL_MS`                                                         | that burst stopped, tried again with the next pulse; `BURST_FAILS_BEFORE_UNCLOG` in a row → the unclog sequence by itself, then metering again. Never `Clogged`, never an alarm (the user's rules, 29 September 2026) |
| Dispenser | tractor lost while unclogging                                                                                        | back to `Clogged`, motor off — losing the tractor is not an operator decision. Burst metering's own unclog runs to the end: nobody asked for it |
| Dispenser | a calibration request still raised after a dispenser reboot, link gap, clog or cancel                                | no run until the tractor lowers and raises it again (the "armed" rule)                                                             |
| Tractor   | dispenser reports `Clogged`                                                                                          | `ZATKANIE!` alarm over any screen, buzzer included                                                                                 |
| Tractor   | a peer not heard for `LINK_TIMEOUT_MS`                                                                               | blue LED blinks, `BRAK:` letters on the work screen, no buzzer yet                                                                 |
| Tractor   | a peer that had been heard stays silent `LINK_BUZZER_DELAY_MS` longer, or seeder and dispenser can't hear each other | buzzer — timed from the loss of contact, so powering the tractor up first stays silent                                             |

#### Tractor UI (§6)

- One existing button; a phone/SoftAP configuration mode was rejected (see below). The button itself needed one hardware change later (the last bullet below).
- A long press fires at 0.75 s while still held (1.5 s until 27 September 2026, when the user shortened it), confirmed by a short beep, so it works with gloves and without looking; the release is swallowed. A screen changes only on a press or because another board reported something — there are no idle timeouts.
- Dose, calibration and dispenser on/off are stored with `Preferences` in the NVS partition of the ESP32's flash: no battery; retention in the order of 10–20 years; ~100,000 erase cycles per sector with wear levelling, written only on an operator action. Only a full-chip erase or a partition-table change clears them — a normal upload keeps them. The `Ustawienia` screen prints everything stored over USB, both in words and as the first-boot constants, so restoring an erased or replaced board is a paste into `machine_settings.h` and a flash ([docs/calibration-settings.txt](docs/calibration-settings.txt)). That printout is the only way settings leave a board, and there is deliberately no way in: no packet and no serial command can write a setting, because a calibration costs a drive across the field and only the operator, on the screen that measured it, may replace one.
- Values that never change in service (working width, tramline rhythm) stay compile-time constants. The distance per wheel pulse used to be one and is not any more: the machine's seed-size gear changes it, so it is a measured setting per gear, stored on the tractor and sent on the wire like the dose. The tractor broadcasts dose and calibration in every packet, so the dispenser stores nothing and never needs reflashing when they change.
- Calibration run: the request is level-triggered (survives lost packets without an acknowledgement protocol); it bypasses the ground-speed interlock because it's a stationary job and the seeder may be off, but is refused while the machine moves; tractor loss aborts it; a short press can't stop it (that would spoil the weighing). The dispenser starts a run only on a request it saw go from 0 to 1, so it never restarts one by itself after its own reboot, a link gap or a clog — the tractor shows `PRZERWANA` instead of sitting at 0 %.
- Clog alarm (agreed with the user): clog = shaft under 1/3 of the commanded speed for 1.5 s; the alarm covers any screen with the buzzer; `OK` → `Anuluj` (preselected, resumes metering) / `Odetkaj` (4 s reverse/forward sequence, no success check). Commands are counters repeated in every packet, acted on once, ignored right after a link gap or tractor reboot. A press counts only if its screen was already showing for the whole press, so an alarm appearing mid-press can't be acknowledged blind. **Changed on 29 September 2026, in the field, by the user:** the alarm came about four times in 700 m and `Anuluj` always let the auger go on, so burst metering no longer raises it in work — a stalled burst is simply tried again with the next pulse, and three in a row unclog by themselves — and the buzzer is off for it everywhere (`CLOG_ALARM_BUZZER`). The only sign left is two blinks of the yellow LED per failed burst; the user found the buzzer too distracting and the LED visible enough.
- Tramline switch (agreed with the user): the machine is usually used without tramlines, so the switch starts off; flipping it saves straight away and never resets the pass number.
- The work screen's pass number was drawn at text size 4 until 26 September 2026. The glyph is 20x28 pixels from (86, 30), so it reached y = 57 - into the bottom row, where it overwrote part of the animation marker at (92, 50): 14 shared pixels at pass 3, and between 6 and 16 for every other pass and frame except pass 4 in frames 2-4. Nothing broke and the screen stayed readable, which is why it went unnoticed; the ASCII mockups could not show it. It is now drawn at size 1 at (96, 34), on the same row as its label, exactly as the mockup has always shown it.
- Blower alarm switch (added 25 September 2026, asked for by the user to test a stationary machine with the wheel turned by hand and no fan running): screen 21. Unlike every other switch it is **never stored** — each power-up starts with the alarm on. An alarm switched off in the yard must not follow the machine into the field, where a stopped fan ruins the pass with nothing to show for it. The dispenser never reads the turbine, so the alarm is a tractor matter only.
- Button reading and wiring (27 September 2026, after the user found presses going missing). Two separate faults:
  - **Timing.** The button was read once per `loop()`, needed 50 ms of steady level for both press and release, and could not be read at all during a redraw (~30 ms, every 200 ms on the live screens and after every press). Simulated against that timing: a 70 ms tap on the work screen counted 88 % of the time, and ten quick taps of 60 ms with 70 ms gaps gave about six. It is now sampled every 5 ms by an `esp_timer`; with the 20 ms debounce it first had, the same simulation counted every press of 20 ms or more, and all ten taps (the rules below have since replaced the debounce). The callback only decides when a press starts and ends, tells short from long and queues the press with its start time; everything else stays in `loop()` — the same split as the radio's receive callback, and the one deliberate exception to rejected idea 10. The button rule is unchanged: judged at the moment `loop()` takes the press, against when the current view appeared.
  - **Contacts.** The first press after a pause went missing even with the timing right. The button is a car-style panel switch, and at the ~0.07 mA of the internal pull-up the film on its contacts is enough to stop it conducting; car electronics feed switch inputs 1–15 mA, some with a stronger pulse at closure precisely to clean the contacts. It needed a strong pull-up, which GPIO12 — a strapping pin — cannot have, so it moved to GPIO32 with 470 Ω (~7 mA) and 10–100 nF. Every other tractor pin was checked at the same time ([Hardware and pins](#tractor-esp32-srctractormaincpp)); none needed changing.
  - **Bounce.** With the contacts fixed, the switch clicked twice, sometimes more: its contacts open for longer than the 20 ms debounce while it is held or let go, and each opening read as the end of a press (the old 50 ms debounce had hidden most of it). A 250 ms minimum gap between presses (`BUTTON_MIN_GAP_MS`) stopped the doubles but left a symmetric debounce underneath, and that has two faults of its own: an opening of 20 ms or more early in a hold turns a long press into a short one, and a hold that sparks every few milliseconds never registers at all, because 20 ms of unbroken contact never comes.
  - **The rules now in force** are the user's design (27 September 2026), checked by simulation before any code: a press starts at once and ends slowly — 10 ms of contact to start (`BUTTON_PRESS_CONFIRM_MS`), 50 ms without it to end (`BUTTON_RELEASE_CONFIRM_MS`) — a long press fires at `BUTTON_LONG_PRESS_MS` with the finger still down, and presses closer than `BUTTON_MIN_GAP_MS` are dropped. Simulated on a switch sampled every 5 ms at a random phase:
    - 18 hand-written cases (bouncy taps, bouncy releases after short and long presses, make bounce, sparks before and after a long press fires, holds chattering 15/5 and 10/10 ms, a finger resting in a shaking cab): all right. The symmetric debounce got four wrong; the user's gap as first worded got seven wrong (next bullet).
    - 36 000 random presses with hard but physical bounce — openings up to 44 ms at make, during the hold and at release: none wrong, and no two clicks ever under 250 ms apart. With the release confirmation at 20 ms instead of 50, 2952 of the 3000 sessions went wrong: it has to be longer than the longest opening the switch makes.
    - Edges: a clean press of 750 ms is short and 755 ms long; taps 250 ms apart both count, 240 ms apart only the first; an opening under ~45 ms never ends a press, one of 50 ms always does.
    - Noise: 20 000 microsecond spikes gave 8 phantom clicks with a press starting on a single closed sample, and none with two, which is why `BUTTON_PRESS_CONFIRM_MS` is 10 and not 5. The second sample costs nothing measurable: it lost only taps under ~70 ms so broken up that no two samples in a row saw contact (5 in 36 000 presses of a deliberately absurd generator).
  - **Why the gap is timed from the release.** The user's first wording kept `BUTTON_MIN_GAP_MS` between clicks. A long click is sent while the finger is still down, though, so the bounce when it finally lets go comes more than 250 ms after that click and passes as an extra short one; and a spark mid-hold longer than the release confirmation gave a short click and then, 750 ms later, a long one. Timed from the moment the finger let go to the start of the next press, both are dropped, and any two clicks still end up at least `BUTTON_MIN_GAP_MS` apart (agreed with the user).
  - Not done, deliberately (offered, not chosen by the user): dropping `BUTTON_SCREEN_SETTLE_MS` for screens the operator opened with their own press — a tap starting less than ~310 ms after a long press's beep is still ignored — and a click on every counted press.

#### Dispenser (§7)

- **Topology:** the dispenser computes its own motor setpoint from `SeederTelemetry` and `TractorCommand`, so a tractor-module failure doesn't stop metering, and the control loop sits next to the motor.
- **Rate maths** with working width W m, dose D kg/ha and calibration C g per 100 output-shaft revolutions: grams per metre = W × D / 10; revolutions per metre = (W × D / 10) / (C / 100); shaft RPM = 60 × speed [m/s] × revolutions per metre. At 4 m and 40 kg/ha that's 16 g per metre.
- **The motor's 330 RPM caps the working speed:** v_max [m/s] = 330 × C / (2400 × D) at 4 m. At 40 kg/ha: C = 300 → 3.7 km/h, 500 → 6.2 km/h, 800 → 9.9 km/h, 1000 → 12.4 km/h. So the auger must deliver about ≥ 800 g per 100 revolutions for 10 km/h. That is decided by auger geometry; firmware's job is to make the shortfall loud (clamp and raise `ZA SZYBKO`), not to hide it.
- **Metering to distance, not to speed** (18 September 2026): the wheel sensor gives one pulse per ~1.6 m (3 magnets) and the speed is averaged over a whole turn of the metering drive, so the rate alone is a lagging, quantised input. The dispenser keeps a ledger of the shaft turns it owes the ground and trims the target to pay it off - see Dispenser behaviour. The speed-based rate stays as the feed-forward; the ledger is what makes the total right.
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
10. **FreeRTOS tasks or an event framework** — `loop()` with timestamps is enough at 5 Hz; the one real concurrency bug (F1) was fixed by removing work from another context. The tractor button's `esp_timer` sampler is the one exception, kept to the same pattern as the receive callback: it only records, and `loop()` acts (Design record → Tractor UI).
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
22. **Powering the dispenser from the 7-pin lighting socket** - its position-light pins are live only with the lights on, share one 7.5–10 A fuse with the tractor's own lights (a stall could put the position lights out on the road, and no fuse of ours can blow first), and run on 1 mm² wiring with a ground shared by the indicators and brake lights. The ISOBUS socket's power pins are the feed (agreed with the user, 21 September 2026).
23. **A series Schottky for reverse polarity** - the ISOBUS plug is keyed, and the diode would cost ~0.5 V and up to ~1 W of heat in a sealed plastic box for a case the connector already prevents. A P-FET is the option if protection is ever wanted (`docs/dispenser_module_hardware.md`, item 15).
24. **A pin-change interrupt for the tractor button** - it fires on every bounce and on every spike the cab wiring picks up, and the usual "take the first edge, then ignore the pin for a while" debounce turns one spike into a press; on the work screen that moves the tramline pass without anyone seeing it. Making it safe means checking the level again some milliseconds later, which is a sampler with extra steps. Jack Ganssle's "A Guide to Debouncing": an undebounced switch goes to a polled pin, never to an interrupt. The button is sampled by a timer instead.
25. **Keeping the button on GPIO12 by burning the flash-voltage eFuse** (`espefuse.py set_flash_voltage 3.3V`, after which GPIO12 no longer straps) - permanent, it has to be repeated on every replacement ESP32, and a board where it was forgotten does not boot with the pull-up fitted. That would break the rule that any board can be swapped as it comes. The button moved to GPIO32 instead.
26. **Timing the button's minimum gap between clicks instead of from the release** - a long click is sent while the finger is still down, so the bounce when it lets go comes more than the gap after it and clicks again; simulated, it got seven of 18 bounce cases wrong. The gap runs from the moment the finger let go to the start of the next press (Design record → Tractor UI).
27. **A symmetric debounce for the button** (the same time to start a press as to end one) - an opening mid-hold longer than it turns a long press into a short one, and a hold that sparks every few milliseconds never registers at all. A press now starts on 10 ms of contact and ends only after 50 ms without it.
28. **Six magnets on the metering drive** - planned to halve the distance per pulse, but that close together the Hall sensor could not tell one magnet from the next (27 September 2026). The drive keeps three: `WHEEL_MAGNETS` = 3, the speed is averaged over those three gaps (one drive turn, ~4.7 m as before; 15 gaps, five turns, since 28 September 2026), and the price is coarser distance - 1.6 m per pulse, so a stop takes up to 5.2 s to register at `WHEEL_MIN_SPEED_MM_S`. Simulated (seeder estimator and dispenser ledger, 80 m runs), the net error of one start and stop rises by about 40 %: at 8 km/h +0.84 m of fertilizer instead of +0.60 m, about 13 g instead of 10 g at 40 kg/ha on 4 m.
