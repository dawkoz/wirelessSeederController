# Dispenser module — parts list and wiring

Everything needed to build the third (fertilizer dispenser) module. The motor
driver and motor are already chosen; the rest is the supporting hardware.

Firmware pin assignments come from `src/dispenser/main.cpp` — if you change a
pin here, change it there too.

---

## 1. Parts to order

Grouped by how much you actually need them, because the previous draft of this
list did not distinguish and that is a fair thing to ask about.

### 1.1 Required — it will not work (or will be damaged) without these

| # | Part | Qty | Why it's required |
|---|---|---|---|
| 1 | **Cytron MD13S** motor driver | 1 | 6–30 V, 13 A continuous / 30 A peak. Accepts 3.3 V logic directly, so no level shifter on PWM/DIR. |
| 2 | **Pololu 4752** gearmotor | 1 | 37Dx68L, 30:1, 12 V, 330 RPM, 14 kg·cm, 5.5 A stall, 64 CPR quadrature encoder. |
| 3 | **ESP32 DevKit** (38-pin, WROOM-32, 4 MB) | 1 | Same board as the tractor and seeder modules. |
| 4 | DC-DC buck converter, 12 V → 5 V, ≥ 2 A, **wide input** | 1 | The ESP32 cannot run from 12 V. **Do not use a bare LM2596 module.** Tractor 12 V sees load dumps far above 12 V. Use something rated to at least 24–40 V in, ideally automotive-grade (Pololu D24V22F5, Traco TSR/TEN). |
| 5 | Resistors **2 × 10 kΩ + 2 × 20 kΩ** *or* a 4-channel **BSS138** level-shifter module | 1 set | **Not optional.** The encoder's outputs swing to whatever its Vcc is, and its spec minimum is 3.5 V — so it runs at 5 V and its outputs must be dropped to 3.3 V. Connecting them raw destroys the ESP32. |
| 6 | Resistors **2 × 10 kΩ** (pull-downs) | 1 set | See §2.3 — keeps the motor stopped while the ESP32 is in reset. Cheap, and the alternative is a motor that can run unattended. |
| 7 | Inline blade fuse holder + **10 A** fuse | 1 | Safety, not convenience. An unfused 12 V feed to a 5.5 A motor on a vehicle is a fire risk. Sized above stall current, below the wiring limit. |
| 8 | Pololu **#1084** 37D mounting bracket | 1 | Nothing holds the motor otherwise. |
| 9 | 6 mm shaft coupling or hub | 1 | Motor shaft is **6 mm**; match the other half to the auger shaft. A flexible/jaw coupling is far more forgiving of misalignment than a rigid one. |
| 10 | IP65+ junction box, cable glands | 1 + 3 | Same style as the existing seeder boxes. Must fit ESP32 + MD13S (61 × 33 mm) + buck converter. |
| 11 | 2-core power cable, **≥ 1.5 mm²** (2.5 mm² for a long run) | as needed | Must carry 5.5 A stall without dropping voltage. |
| 12 | Terminal blocks / WAGO, ferrules, heat shrink | — | |

### 1.2 Strongly recommended — it will work on the bench without these and fail in the field

| # | Part | Qty | What it protects against |
|---|---|---|---|
| 13 | Electrolytic **1000 µF / 35–50 V**, low ESR, across the MD13S power terminals | 1 | **This is the capacitor that earns its place.** Several metres of supply cable have enough inductance that 5.5 A current steps at 16 kHz PWM cause real voltage dips at the driver. The buck converter shares that rail, so those dips brown out the ESP32 and reset it mid-work. With short bench leads you will never see the problem; on the machine you will. |
| 14 | TVS diode, **SMBJ24A** or **1.5KE24A**, across 12 V after the fuse | 1 | Alternator load dump. A 12 V tractor system can spike well above what even a 40 V-rated buck survives. Costs pennies. |
| 15 | Reverse-polarity protection: Schottky ≥ 10 A (**SR1040**) or a P-channel MOSFET circuit | 1 | One reversed connection during field servicing destroys the MD13S and likely everything downstream. The MOSFET version wastes far less voltage than the diode. |
| 16 | ESP32 screw-terminal breakout, or solder everything to perfboard | 1 | Vibration on farm equipment works dupont jumpers and breadboards loose. This is the most common cause of intermittent faults on machine-mounted electronics. |

### 1.3 Optional — buy only if your layout needs it

| # | Part | Notes |
|---|---|---|
| 17 | 6-core **shielded** cable | Only if the motor is not right at the enclosure. 2 cores motor power + 4 encoder. Shielding matters because the encoder lines would run alongside switched motor current. If the motor mounts on the box, plain cable is fine. |
| 18 | ~~100 nF ceramic decoupling capacitors~~ | **Skip these.** The earlier draft listed them; that was over-specification. The ESP32 DevKit and the MD13S both carry their own local decoupling, and adding more at the signal header achieves nothing measurable here. |
| 19 | Heatsink for the MD13S | Not needed. The MD13S is a full NMOS H-bridge rated 13 A continuous; this motor draws 200 mA typical and 5.5 A only at stall. Cytron explicitly states no heatsink is required. |

---

## 2. ESP32 pin verification

Checked against the Espressif GPIO documentation and the ESP32-WROOM-32
datasheet, because a pin that looks free can still misbehave at boot.

### 2.1 The chosen pins

| Pin | Use | Verdict |
|---|---|---|
| **25** | MD13S PWM (LEDC) | Safe. `GPIO25 / DAC_1 / ADC2_CH8 / RTC_GPIO6`. Not a strapping pin, not connected to flash, full output and LEDC capability. Its ADC2 channel is unusable while WiFi runs — irrelevant, we only drive it as a digital output. |
| **26** | MD13S DIR | Safe. `GPIO26 / DAC_2 / ADC2_CH9 / RTC_GPIO7`. Same reasoning as 25. |
| **32** | Encoder channel A (interrupt) | Safe, with one check — see §2.2. `GPIO32 / XTAL_32K_P / ADC1_CH4 / TOUCH9`. Has an internal pull-up and full interrupt support. |
| **33** | Encoder channel B (wired, unused) | Safe, same check as 32. `GPIO33 / XTAL_32K_N / ADC1_CH5 / TOUCH8`. |

All four are in Espressif's "no special boot behaviour, no internal peripheral
conflict" group. None are the input-only pins (34–39), which matters because
those have **no internal pull-up** and cannot be used the way this project uses
sensor inputs.

### 2.2 The one thing to confirm on your specific board

GPIO32 and GPIO33 double as `XTAL_32K_P` / `XTAL_32K_N`, the pins for an
optional 32.768 kHz RTC crystal. That crystal is **not fitted on the
ESP32-WROOM-32 module** — it does not appear in the module schematic, and the
module runs its RTC from an internal oscillator. So both pins are free on a
standard DevKit.

Worth a ten-second look at your actual board anyway: if you can see a small
crystal near those two pins, move the encoder to GPIO 18/19 and update
`ENCODER_A_PIN` / `ENCODER_B_PIN` in `src/dispenser/main.cpp`.

### 2.3 Boot-time floating — the reason for the pull-down resistors

This is the part worth taking seriously. **ESP32 GPIOs are high-impedance
during reset and for the first few hundred milliseconds of boot**, before
firmware runs and configures them. During that window the MD13S's PWM input is
not being driven by anything.

The firmware writes PWM = 0 as the very first thing in `setup()`, but it
physically cannot cover the window before it starts executing. A floating input
on a 13 A motor driver, on unattended machinery, is not something to leave to
chance.

So: **10 kΩ from MD13S PWM to GND, and 10 kΩ from MD13S DIR to GND.** This
guarantees the motor is stopped whenever the ESP32 is in reset, unprogrammed,
crashed, or physically removed from its socket.

A related note on why the pins were chosen: **GPIO 0, 5, 14 and 15 actually
output a PWM signal during boot.** Had the motor PWM been on one of those, the
motor would briefly spin every single time the board reset. GPIO 25 and 26 do
not do this.

### 2.4 Timing headroom

- **LEDC at 16 kHz, 10-bit:** needs 1024 × 16 kHz = 16.4 MHz against the 80 MHz
  APB clock, so roughly 12 bits are available at this frequency. 10-bit has
  comfortable margin.
- **Encoder interrupt rate:** 480 edges/rev × 330 RPM ÷ 60 = **2.6 kHz** on a
  240 MHz core, from a minimal `IRAM_ATTR` handler. Not close to a problem.
  Decoding both channels would have doubled it for resolution the control loop
  does not need.

### 2.5 A note that applies to the *existing* boards

`GPIO12` is the **MTDI strapping pin**: its level at reset selects the flash
voltage (VDD_SDIO). Held high at reset, the chip configures 1.8 V flash and
**fails to boot**. Both existing modules use GPIO12 — the seeder for the relay,
the tractor for the button. Both work today because the pin sits low at reset.

Nothing needs changing. But **never add an external pull-up to GPIO12** on
either board, and if you ever swap the relay module for one with a pull-up on
its input, expect a board that no longer boots.

---

## 3. Wiring

### 3.1 Power distribution

```
Tractor 12 V ──[ 10 A fuse ]──[ reverse-polarity diode/MOSFET ]──┬── 12 V rail
                                                                  │
                          TVS diode (SMBJ24A) ────────────────────┤
                                                                  │
                                    1000 µF ─────────────────────┤
                                                                  │
                         ┌────────────────────────────────────────┤
                         │                                        │
                  MD13S power in                        Buck converter in
                  (green terminal block)                 12 V → 5 V
                         │                                        │
                         │                                        ├── ESP32 "VIN" (5 V)
                         │                                        └── Encoder BLUE (Vcc)
                         │
                  Motor windings
```

The **buck converter feeds only the ESP32 and the encoder**. Motor current
never passes through it.

### 3.2 MD13S

| MD13S | Connects to |
|---|---|
| Green terminal block `+` | 12 V rail (after fuse and protection) |
| Green terminal block `−` | Ground |
| Black terminal block `MA` | Motor **red** wire |
| Black terminal block `MB` | Motor **black** wire |
| Signal header `PWM` | ESP32 **GPIO 25**, plus **10 kΩ to GND** (§2.3) |
| Signal header `DIR` | ESP32 **GPIO 26**, plus **10 kΩ to GND** (§2.3) |
| Signal header `GND` | ESP32 **GND** — required; without a shared ground the logic inputs float |

The MD13S is specified for 3.3 V and 5 V logic, so PWM and DIR connect straight
to ESP32 pins with no level shifter. Firmware drives PWM at 16 kHz; the
driver's stated limit is 20 kHz.

### 3.3 Motor and encoder

Six leads. The two thick ones are motor power and go to the **driver**; the
four thin ones are the encoder and go to the **ESP32**.

| Motor wire | Function | Connects to |
|---|---|---|
| **Red** | Motor power | MD13S `MA` |
| **Black** | Motor power | MD13S `MB` |
| **Green** | Encoder GND | Common ground |
| **Blue** | Encoder Vcc | **5 V** from the buck converter |
| **Yellow** | Encoder channel A | Divider → ESP32 **GPIO 32** |
| **White** | Encoder channel B | Divider → ESP32 **GPIO 33** |

Divider for each of yellow and white (if not using a BSS138 module):

```
encoder output ──[ 10 kΩ ]──┬── ESP32 GPIO
                            │
                        [ 20 kΩ ]
                            │
                           GND
```

5 V × 20/(10+20) = 3.33 V. Channel B is wired but unused by the firmware — the
auger turns one way, so rising edges on A alone give ample resolution at half
the interrupt load. It's connected so full quadrature stays available without
rewiring.

### 3.4 Grounding

Join motor ground and logic ground at **one single point**, at the MD13S power
terminal. Do not daisy-chain the ESP32's ground through the motor current path
— the volt-drop along that wire during a 5.5 A stall appears as ground noise on
the encoder inputs and corrupts the count.

---

## 4. Before powering up

1. **Check the encoder's blue wire goes to 5 V, not 12 V.** This is the one
   mistake that destroys the ESP32 — the encoder's outputs sit at its Vcc, so
   12 V there puts 12 V straight onto a GPIO.
2. Verify each divider output measures ≈3.3 V, not 5 V, with the encoder
   powered and the shaft held so that output is high.
3. Confirm the two 10 kΩ pull-downs are present on PWM and DIR (§2.3).
4. Confirm continuity from ESP32 GND to MD13S GND before applying 12 V.
5. First power-up: hopper empty, coupling disconnected from the auger. Confirm
   the motor turns and the encoder counts before loading it.
6. If the auger runs the wrong way, either swap the motor's red and black leads
   or flip `MOTOR_DIR_FORWARD` in `src/dispenser/main.cpp`.

## 5. First calibration

1. Bench-verify `ENCODER_EDGES_PER_REV` (firmware assumes **480** rising edges
   per output-shaft revolution): turn the output shaft 10 revolutions by hand
   and check the count is ≈4800.
2. On the tractor: `Kalibracja` → cursor to `TEST` → long press → `START`. The
   dispenser turns exactly 100 revolutions and shows a progress bar. It refuses
   to start if the seeder reports the machine is moving.
3. Catch and weigh everything it delivers, in grams.
4. Enter that number as `Kalibracja` and save.

Per Implementation_Plan §7.3: to sustain 10 km/h at 40 kg/ha the auger needs to
deliver roughly **≥ 800 g per 100 revolutions**. If calibration comes out much
below that, the motor cannot keep up at working speed — firmware clamps and
raises the `ZA SZYBKO` alarm rather than silently under-applying, but the real
fix is auger geometry.

## Sources

- [Pololu 4752 — 30:1 Metal Gearmotor 37Dx68L mm with 64 CPR encoder](https://www.pololu.com/product/4752)
- [Cytron MD13S — 13 A 6–30 V DC motor driver](https://fluxelectronix.com/shop/md13s-13amp-6-30v-dc-motor-driver-cytron/)
- [ESP-IDF GPIO & RTC GPIO reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
- [ESP32-WROOM-32 datasheet](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.html)
- [Espressif ESP32 hardware design guidelines](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/schematic-checklist.html)
