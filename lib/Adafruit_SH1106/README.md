# Adafruit_SH1106 (vendored, ESP32-patched)

Vendored from [wonho-maker/Adafruit_SH1106](https://github.com/wonho-maker/Adafruit_SH1106) (BSD license, see `LICENSE.txt`), used by `src/tractor/main.cpp` for the tractor board's OLED.

Upstream targets AVR only and doesn't compile against the ESP32 Arduino core as-is. Three things were patched, all guarded so the AVR behaviour is untouched:

1. **`#include <avr/pgmspace.h>` and `<util/delay.h>`** — AVR-only headers, guarded to `__AVR__`. Nothing in this library actually needs them on ESP32.
2. **`PortReg` / `PortMask` typedefs** — upstream assumes 8-bit GPIO port registers. ESP32's are 32-bit (like the SAM3X8E case upstream already special-cases), so an `ESP32`/`ESP8266` branch was added alongside it. Only used on the SPI code path, which this project doesn't use (`sid == -1`), but it has to type-check to compile.
3. **`TWBR = 12` inside `display()`** — an AVR I2C-clock register write, guarded to `__AVR__`.

**Point 3 matters and is easy to miss.** Unlike points 1 and 2, that register write *is* on the I2C path this project uses, and it was not cosmetic: `TWBR = 12` sets 400 kHz on a 16 MHz AVR. Guarding it out for ESP32 silently dropped the bus back to the Arduino default of 100 kHz, which quadruples the time of every full-screen redraw (~103 ms instead of ~26 ms) and is long enough to start dropping button presses. The tractor firmware therefore calls **`Wire.setClock(400000)`** right after `oled.begin()` to restore the speed the library author intended. Don't remove it.

Apart from those three, no edits were made — same class, same API (`Adafruit_SH1106`, `begin(SH1106_SWITCHCAPVCC, addr)`), same public methods `src/tractor/main.cpp` relies on.
