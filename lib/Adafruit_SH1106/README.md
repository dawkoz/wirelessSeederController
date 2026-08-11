# Adafruit_SH1106 (vendored, ESP32-patched)

Vendored from [wonho-maker/Adafruit_SH1106](https://github.com/wonho-maker/Adafruit_SH1106) (BSD license, see `LICENSE.txt`), used by `src/tractor/main.cpp` for the tractor board's OLED.

Upstream targets AVR only and unconditionally includes AVR-only headers (`avr/pgmspace.h`) and touches an AVR-only hardware register (`TWBR`, for I2C bus speed) that don't exist on ESP32 — it doesn't compile against the ESP32 Arduino core as-is. Both are guarded to `__AVR__` here; neither is on the I2C code path this project actually uses (`sid == -1`), so this is a pure portability fix with no behavior change. No other edits were made — same class, same API (`Adafruit_SH1106`, `begin(SH1106_SWITCHCAPVCC, addr)`), same public methods `src/tractor/main.cpp` already relies on.
