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
| 12 | Relay control (tramline) |
| 14 | Turbine inductive sensor (interrupt, RISING) |
| 27 | Metering unit Hall sensor (interrupt, RISING) |

Peer MAC address: `broadcastAddressTractor` at the top of the file — must match the tractor board's actual MAC.

### Tractor ESP32 (`src/tractor/main.cpp`)
| Pin | Function |
|---|---|
| 12 | Tramline selection button |
| 14 | Green LED (connected) |
| 27 | Blue LED (connecting/blinking) |
| 13 | Yellow LED (tramline active) |
| 19 | Buzzer |

OLED: SH1106 128x64 over I2C, address `0x3C`. Peer MAC address: `broadcastAddressSeeder` at the top of the file — must match the seeder board's actual MAC.

### Dispenser ESP32 (`src/dispenser/main.cpp`)
Not yet designed. Will have a Cytron motor controller driving the dispenser motor. Powered by its own 12V line; control/telemetry planned to be wireless (ESP-NOW, consistent with the other two boards).

## Completed

- ESP-NOW comms between tractor and seeder boards.
- Turbine RPM + metering unit activity sensing on the seeder board.
- Tramline relay control, manual tramline selection via button on the tractor board.
- OLED display + LED/buzzer fault alerting on the tractor board.
- Migration from Arduino IDE (two separate `.ino` sketches) to a single PlatformIO project with per-board build environments and a shared protocol header.

## Current task

The PlatformIO migration above — done, verified by compiling all three environments with the PlatformIO CLI (`pio run`), zero errors, `.pio/build` artifacts produced for `tractor`, `seeder`, and `dispenser`. Not yet verified on physical hardware (upload) — do that before relying on it in the field.

## Future tasks

- **Design and implement the fertilizer dispenser module**: Cytron motor controller wiring, pin selection, ESP-NOW protocol extension (new struct + peer registration, likely modeled after the existing seeder/tractor pattern), real `src/dispenser/main.cpp` logic (currently a blank stub).
- **Real WOM (power take-off) RPM sensing** — currently hardcoded to `540` as a placeholder in `src/seeder/main.cpp` (`seederData.WOMRPM = 540; // Example value`). README lists this as TODO.
- **Wiring diagram** — README "Installation" and "Wiring Diagram" sections are still TODO.
- **Demonstration video** — README "Video" section is still TODO.

## Important notes

- MAC addresses are hardcoded per-device in each board's source file. If you swap a physical ESP32 unit, you must update the corresponding `broadcastAddress*` array (and the *other* board's copy of that MAC) before it will pair.
- ESP-NOW peer `channel` is set to `0` (defaults to the current WiFi channel) and `encrypt` is `false` on both boards — keep both sides consistent if this ever changes.
- The Adafruit_SH1106 library used by the tractor board is the older community library (class `Adafruit_SH1106`, `begin(SH1106_SWITCHCAPVCC, addr)` API), originally from [wonho-maker/Adafruit_SH1106](https://github.com/wonho-maker/Adafruit_SH1106) — not Adafruit's newer `Adafruit_SH110X` library, which has a different, incompatible API. It's **vendored** (not pulled via `lib_deps`) at `lib/Adafruit_SH1106/`, with a small ESP32-portability patch — upstream targets AVR only and doesn't compile as-is against the ESP32 Arduino core. See `lib/Adafruit_SH1106/README.md` for exactly what was changed and why (no behavioral change, both patched spots are on the SPI code path this project doesn't use — only I2C).
