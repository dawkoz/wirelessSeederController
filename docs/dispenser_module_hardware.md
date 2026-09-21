# Dispenser module — parts list and wiring

Everything needed to build the third (fertilizer dispenser) module. The motor
driver and motor are already chosen; the rest is the supporting hardware.

Firmware pin assignments come from `include/machine_settings.h` — if you change
a pin here, change it there too.

**The wiring is drawn in [dispenser_wiring.svg](dispenser_wiring.svg)** — every
pin on every board, the power input, a connection list and what gets soldered
onto the MD13S. This document is the reasoning behind it.

---

## 1. Parts to order

Grouped by how much you actually need them. Checked against the manufacturers'
documentation and against the Polish shops in September 2026: §1.4 says where
to buy each part, and §1.5 lists the traps that check turned up — read it
before ordering.

### 1.1 Required — it will not work (or will be damaged) without these

| # | Part | Qty | Why it's required |
|---|---|---|---|
| 1 | **Cytron MD13S** motor driver | 1 | 6–30 V (**30 V is its absolute maximum**), 13 A continuous / 30 A peak for 10 s. Accepts 3.3 V logic directly (high ≥ 3 V), so PWM/DIR need no level shifter. The control input is a 4-pin **Grove** connector (cable included), and the two terminal blocks come **loose — you solder them on**. |
| 2 | **Pololu 4752** gearmotor — the genuine Pololu part | 1 | 37Dx68L, 30:1, 12 V, 330 RPM, 14 kg·cm and 5.5 A at stall, 64 CPR quadrature encoder. Pololu rates the gearbox for **10 kg·cm continuous** (25 kg·cm instantaneous) — see §1.5. |
| 3 | **ESP32-DevKitC-32E V4** (38 pins, ESP32-WROOM-32E module, PCB antenna) | 1 | Any board with a classic ESP32 works — the firmware is built for `esp32dev`. **Not** an ESP32-S2/S3/C3/C6 board, and **not** a `-32U`/`-32UE` module, which needs an external antenna. |
| 4 | **Pololu D24V22F5** step-down regulator, 5 V 2.5 A, 5.3–36 V in | 1 | The ESP32 cannot run from 12 V. Reverse-voltage protected, and its 36 V limit is safe behind the surge suppressor in item 14. **Do not use a bare LM2596 module.** For more margin: Pololu **D36V28F5** (5 V 3.2 A, up to 50 V in). |
| 5 | **4-channel BSS138 logic-level converter** module | 1 | **Not optional.** The encoder runs at 5 V (its minimum is 3.5 V), and **its A/B outputs are pulled up to that 5 V on the encoder board** — connected raw they put 5 V on the ESP32. A resistor divider does not work here: with the on-board pull-up in the chain it only reaches about 2.5 V, right at the ESP32's logic threshold. The BSS138 module doesn't mind the pull-ups. **Not a TXB0104 converter** — that type misbehaves with pull-up resistors. |
| 6 | Resistors **2 × 10 kΩ** (pull-downs) | 1 set | See §2.3 — keeps the motor stopped while the ESP32 is in reset. Soldered straight onto the MD13S's Grove pins (§3.2). |
| 7 | **Waterproof** inline blade fuse holder + **10 A** blade fuses, with spares | 1 | Safety, not convenience: an unfused 12 V feed to a 5.5 A motor on a vehicle is a fire risk. Fit it **right at the ISOBUS plug** — the tractor end of the cable — not at the dispenser box: a fuse only protects the cable after it, and the socket's own fuse is rated far above what 1.5 mm² cable can carry. Keep spares: item 14 blows it on purpose if someone jump-starts with 24 V. |
| 8 | Pololu **#1084** 37D mounting bracket (pair) | 1 | Nothing holds the motor otherwise. Motor screws are included. |
| 9 | **Jaw coupling** ("sprzęgło kłowe"), 6 mm bore on the motor side, rated **≥ 3 N·m** (L050 size or bigger) | 1 | The motor shaft is **6 mm, D-shaped, 16 mm long**; the other bore matches the auger shaft. Tighten the set screw on the flat. Small "encoder" couplings and aluminium helical/beam couplings are rated around 1 N·m or less — below this motor's 1.4 N·m stall torque — and slip or crack. |
| 10 | **Plastic** IP65+ box (never metal — it blocks the radio), cable glands sized to the cables | 1 + 3 | Must fit the ESP32 on its terminal adapter, the MD13S (61 × 33 mm) with C1 on it, the regulator, the two distribution blocks and the TVS. The fuse is not in the box — it sits at the ISOBUS plug. Glands grip only their clamping range: a 2 × 1.5 mm² rubber cable is about 9–10 mm across, which needs PG11 or M20, not PG7. |
| 11 | Power cable **2 × 1.5 mm²** (2 × 2.5 mm² for runs over a few metres), **rubber, oil- and UV-resistant**: H05RR-F / H07RN-F ("OnPD") | as needed | Must carry 5.5 A stall without dropping voltage. Ordinary PVC "OMY" cable stiffens in the cold and degrades in oil and sunlight on a farm machine. |
| 12 | Terminal blocks / WAGO, ferrules, heat shrink | — | Two **4-way WAGO 221** (32 A, 0.2–4 mm²) are the `+` and `−` distribution points inside the box (§3.1). |
| 20 | **ISOBUS implement-side plug** (ISO 11783-2 breakaway connector, 9-pin), sealed | 1 | The dispenser takes its 12 V from the power pins of the tractor's ISOBUS socket (§1.5 trap 12). Ask the John Deere dealer for the implement-side part number, or buy a sealed plug or a pre-made tail from an agricultural-electronics supplier. It must seal — the socket has a sealed cap for a reason. Only PWR and PWR_GND are wired. |

### 1.2 Strongly recommended — it will work on the bench without these and fail in the field

| # | Part | Qty | What it protects against |
|---|---|---|---|
| 13 | Electrolytic **C1: 1000 µF, ≥ 35 V (50 V preferred), low ESR, 105 °C**, **soldered straight onto the MD13S `+`/`−` power pads** | 1 | The MD13S chops the motor current at 16 kHz, so the current it draws is a square wave at the motor current — up to ~4 A, 16,000 times a second. Its own 2 × 330 µF already supply most of that; C1 takes the local impedance from about 50 mΩ to 20 mΩ, so the switching current stays inside the box instead of running up and down the cable beside the encoder wires, and the ripple is shared by three capacitors instead of two. It is margin, not a load-bearing part — a 5.5 A stall sags a proper feed by well under a volt, which the 5 V regulator never notices, so it does not stand between the ESP32 and a brownout. Fit it anyway; it costs a few złoty. **Low ESR and 105 °C** because it carries 1–2 A RMS of ripple whenever the motor runs, which dries a general-purpose 85 °C part out in a season. **Never a 25 V part**: the rail can reach the suppressor's ~28–29 V clamp. Mounting: §3.2. |
| 14 | Surge suppressor (TVS diode): **1.5KE20A** (through-hole, 1500 W) or **SMBJ18A** (SMD, 600 W), from +12 V to GND on the distribution blocks (§3.1), band (cathode) to +12 V | 1 | Spikes on the tractor's 12 V. Below its stand-off voltage it leaks a few microamps — electrically invisible until something goes wrong. It must clamp **below the MD13S's 30 V**: 1.5KE20A clamps at 27.7 V, SMBJ18A at 29.2 V. **Not SMBJ24A (38.9 V) or 1.5KE24A (33.2 V)** — and the two families name parts differently (SMBJ by stand-off voltage, 1.5KE by breakdown voltage), so the "24" means different things. Both recommended parts start conducting around 19–20 V: normal charging (≤ 14.8 V) doesn't touch them; a 24 V jump start makes them conduct hard and blows the fuse, which is the point. A TVS absorbs spikes — a full alternator load dump with the battery disconnected can still destroy it, and then it fails short and the fuse opens. |
| 16 | ESP32 screw-terminal adapter **matching the board** | 1 | Vibration on farm equipment works dupont jumpers and breadboards loose — the most common cause of intermittent faults on machine-mounted electronics. The DevKitC V4 needs a **38-pin adapter with 25.4 mm (1″) between the rows** (Allegro lists them as "38pin 25.5mm … DevKitC-v4"); a 30-pin board needs the 30-pin adapter. |

### 1.3 Optional — buy only if your layout needs it

| # | Part | Notes |
|---|---|---|
| 17 | 6-core **shielded** cable (e.g. LiYCY 6 × 0.25 mm²) | Only if the motor is not right at the enclosure. 2 cores motor power (use heavier cable if the run is long) + 4 encoder. Shielding matters because the encoder lines would run alongside switched motor current. If the motor mounts on the box, the motor's own leads are enough. |
| 18 | ~~100 nF ceramic decoupling capacitors~~ | **Skip these.** The ESP32 DevKit and the MD13S both carry their own local decoupling, and adding more at the signal header achieves nothing measurable here. |
| 19 | Heatsink for the MD13S | Not needed. The MD13S is a full NMOS H-bridge rated 13 A continuous without one; this motor draws 200 mA typical and 5.5 A only at stall. |
| 15 | Reverse-polarity protection — **not needed on the ISOBUS feed** | The ISOBUS plug is keyed and cannot go in reversed, which was the case this guarded against; a keyed connector where the cable enters the box covers servicing. A series **Schottky (MBR1045) is the wrong part here**: it drops ~0.5 V and dissipates 0.4–1 W the whole time the auger is metering (about 3 W in a stall), inside a sealed plastic box with no metal to sink the heat into. If you want protection anyway, use a **P-channel MOSFET** (e.g. IRF4905, ~20 mΩ: about 40 mV and 0.08 W at 2 A). Fit it so its **body diode points in the normal current direction** — for a P-channel in the `+` line that is drain to the supply, source to the load, deliberately "backwards" — with the gate to GND through 10 kΩ and a 15 V zener from gate to source (cathode at the source), because the TVS clamps at ~28 V and V<sub>GS</sub> max is ±20 V. With neither, a reversed supply forward-biases the MD13S's MOSFET body diodes and the TVS: roughly a dead short, which blows the 10 A fuse within milliseconds — whether the driver survives those milliseconds is a coin flip. The regulator is reverse-protected, so the ESP32 is safe either way. |

### 1.4 Where to buy in Poland

Checked on 14 September 2026. Botland prices are from botland.store in EUR
including VAT (botland.com.pl shows the same items in PLN), and every Botland
item below showed "Available — shipping in 24 hours". TME (tme.eu, Łódź) sells
to individuals too and is the fallback for any discrete part Botland and
Allegro don't have.

| # | Part | Botland | Allegro / others |
|---|---|---|---|
| 1 | MD13S | "Cytron MD13S – single-channel 30V / 13A motor controller", code CTN-12412, ~€13.50 | Kamami |
| 2 | Pololu 4752 | "30:1 Metal Gearmotor 12V 37Dx68Lmm with 64 CPR Encoder – Pololu 4752", code PLL-16115, ~€55.90 | Kamami, TME (POLOLU-4752). **Skip "zamiennik Pololu 4752" listings** — §1.5 |
| 3 | ESP32 board | "ESP32-DevKitC-32E V4 WiFi + BT 4.2 – platform with the module ESP-WROOM-32E" (Espressif, micro-USB), ~€16.90 | many — check chip and module, §1.5 |
| 4 | Regulator | "D24V22F5 – przetwornica step-down – 5V 2,5A – Pololu 2858", ~€14.90, pins unsoldered. D36V28F5 is Pololu 3782 | Allegro, TME (POLOLU-2858) |
| 5 | Level shifter | "Konwerter poziomów logicznych dwukierunkowy, 4-kanałowy" (BSS138), ~€2.50, pins unsoldered | Allegro, Nettigo |
| 6 | 10 kΩ resistors | resistor assortments | Allegro |
| 7 | Fuse holder, fuses | — | Allegro: "oprawka bezpiecznika wodoodporna", "bezpiecznik samochodowy 10A" |
| 8 | Bracket | "Mocowania aluminiowe do silników 37D – 2szt. – Pololu 1084", ~€10.00 | Allegro, Kamami |
| 9 | Coupling | — | Allegro: "sprzęgło kłowe" with both bores; check the rated torque |
| 10 | Box, glands | Kradex hermetic enclosures (IP65/IP67) and hermetic cable glands | Allegro |
| 11 | Cable | — | Allegro: "przewód OnPD 2x1,5" or "H07RN-F 2x1,5" |
| 12 | Distribution blocks | — | Allegro or any electrical wholesaler: "WAGO 221-414" |
| 13 | 1000 µF 50 V low ESR | not in Botland's low-ESR range when checked | Allegro, TME, a-hobby.pl (Jamicon TL, 16 × 25 mm) |
| 14 | TVS | — | Allegro: "SMBJ18A"; TME: "1.5KE20A" (Littelfuse or Diotec); eltron.pl: "P6KE20A" (600 W, also 27.7 V) |
| 15 | P-FET, only if you fit reverse protection | — | Allegro, TME: "IRF4905", plus a 15 V zener and a 10 kΩ resistor |
| 16 | Terminal adapter | — | Allegro: "Adapter ESP32 38pin 25.5mm terminal shield DevKitC-v4" (~17 zł) |
| 20 | ISOBUS plug | — | the John Deere dealer (the part for your socket); agricultural-electronics suppliers and Allegro: "wtyczka ISOBUS 9 pin", "ISOBUS implement connector" — roughly 50–150 zł for a sealed one |

### 1.5 Traps found when checking this list

1. **"24 V" surge suppressors don't protect the MD13S.** Its 30 V is an
   absolute maximum; SMBJ24A clamps at 38.9 V and 1.5KE24A at 33.2 V. Use
   1.5KE20A or SMBJ18A (item 14). An earlier draft of this document listed the
   24 V parts.
2. **The encoder has its own pull-up resistors.** Pololu confirms pull-ups on
   the 37D encoder board (10 kΩ on their comparable 25D encoders). The 10 k /
   20 k divider in the earlier draft then gives about 2.5 V, which the ESP32
   may or may not read as high — miscounted edges, and so a wrong dose. Use the
   BSS138 module (item 5) and check it with §4 step 6.
3. **Motor power arrives on a 0.1″ header.** All six 20 cm leads, the red and
   black motor power included, end in one 1×6 female 0.1″ header. That is fine
   for the encoder, not for up to 5.5 A: cut the red and black leads out of it
   and put them straight into the MD13S terminal block with ferrules.
4. **MD13S details:**
   - the control input is a Grove connector — cut one end off the supplied
     cable rather than looking for 0.1″ pins. Cytron's manual numbers its pins
     1 GND, 2 PWM, 3 DIR, 4 NC; an earlier draft of this document listed them
     the other way round. Don't settle it from a photo — find GND by
     continuity (§3.2);
   - the terminal blocks come loose — solder them on;
   - Cytron's manual and the shop description disagree on which colour block
     is power and which is motor — **wire by the silkscreen `+`, `−`, `MA`,
     `MB`**, never by colour;
   - the two test buttons (MA, MB) spin the motor whatever the ESP32 does —
     never press them with the auger connected or hands near it.
5. **ESP32 look-alikes.** Many "ESP32" boards on Allegro are ESP32-S3 or C3
   (different chips — this firmware won't run on them) or carry a -32U/-32UE
   module that needs an external antenna. Buy a classic ESP32-WROOM-32/32D/32E
   board with a PCB antenna and a matching terminal adapter (item 16), and put
   it in a **plastic** box.
6. **USB and the 5 V regulator must never power the ESP32 at the same time.**
   Espressif: power the DevKitC from one and only one source — USB, the 5V pin
   or the 3V3 pin — or the board or the supply can be damaged. On the bench,
   with the laptop's USB plugged in, disconnect the regulator's 5 V from the
   ESP32's 5V pin; keep the grounds joined, and keep the encoder and the level
   shifter's HV side on the regulator.
7. **No series diode for reverse polarity.** An earlier draft put an MBR1045
   Schottky in the `+` line. On the keyed ISOBUS feed it guards against nothing
   the plug doesn't already prevent, and it costs half a volt and up to a watt
   of heat in a sealed box. Item 15 has the P-FET if you want protection anyway.
8. **Small couplings.** The motor's stall torque (1.4 N·m) and the gearbox's
   instantaneous limit (2.5 N·m) are beyond what encoder couplings and small
   helical couplings take. Use a jaw coupling rated ≥ 3 N·m (item 9).
9. **The gearbox is rated 10 kg·cm continuous.** Stall is 14 kg·cm, and Pololu
   warns that stalling and overloading shorten the gearmotor's life. If the
   loaded auger needs close to that to turn, this motor will wear out — so
   before fitting it, run the bench test's H05 with the auger loaded and watch
   the breakaway duty. A higher gear ratio gives more torque but less speed,
   and the speed table in CLAUDE.md (Design record → Dispenser) then asks for
   even more fertilizer per revolution.
10. **Clones.** Several Polish shops sell "zamiennik Pololu 4752" motors. Their
    encoder resolution, wire colours and encoder supply range are not
    guaranteed to match, and the firmware constants assume the genuine part
    (480 edges per output revolution, 330 RPM).
11. **Cable and glands.** PVC cable doesn't last on a farm machine, and a gland
    that doesn't grip the cable's outer diameter isn't IP65 at all (items 10,
    11).
12. **Where the power comes from — the ISOBUS socket, not the lighting
    socket.** Take the dispenser's 12 V from the **PWR and PWR_GND pins of the
    tractor's ISOBUS socket** (item 20), through the 10 A fuse right at the plug
    (item 7).

    The 7-pin lighting socket (ISO 1724) looks like the obvious source because
    it powers a whole trailer's lights, but it is the wrong one for a motor.
    Its position-light pins are live only while the lights are on. They share
    one 7.5–10 A fuse with the tractor's own lights, so a 5.5 A stall on top of
    them can blow that fuse and put the position lights out on the road — and a
    10 A fuse of our own can never blow first on a 7.5 A circuit. And 1 mm²
    wiring with a ground pin shared by the indicators and brake lights costs
    ~0.5 V at 2 A and ~1.3 V at a stall, more as the plug corrodes. The lighting
    socket is fine for the seeder module's logic; the motor needs a real feed.

    The ISOBUS power pins are fused far higher on the tractor side, so the
    10 A fuse at the plug is the one that blows — the fuse coordination a
    lighting circuit cannot give. Wire **only PWR and PWR_GND**: never ECU_PWR
    (a few amps, meant for control units), never the CAN pins, and never the
    TBC pins, which are part of the bus termination. Take the pin numbering
    from the tractor's operator's manual and confirm it with a meter before
    crimping — those pins can source tens of amps, so mind the probes.

    Two checks on the tractor, once:
    - **Key out, meter PWR to PWR_GND.** If it is still live, the module's
      ~50 mA idle draw is on the battery around the clock — about 100 Ah over a
      winter. Unplug the seeder when it is parked for weeks; that happens
      anyway when it is unhitched.
    - **Load it with a bulb for a few minutes, key on.** Some tractors switch the
      implement power off when no ISOBUS device answers on the bus; this module
      never talks CAN. If the power drops out, that is a question for the
      dealer.

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
`ENCODER_A_PIN` / `ENCODER_B_PIN` in `include/machine_settings.h`.

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

**What is documented and what isn't.** The ESP32 half is Espressif's own:
the *ESP32 Pin List* says that during reset all pins are output-disabled, and
its "At reset" column for GPIO 25 and 26 is empty — no input enable, no
internal pull-up, no internal pull-down. The MD13S half is not documented:
Cytron's manual gives the truth table and the logic levels but not the input
circuit, so whether the board already pulls PWM low is unknown. Measure it
before anything is connected: board unpowered, meter on resistance, PWM to
GND. Tens of kΩ means Cytron fitted a pull-down; megohms means the pin floats.
Driver makers who do document it fit one on purpose — Pololu puts 100 kΩ on
the PWM and EN inputs of its MAX14870 carrier, for example.

**Only the PWM one is a safety part.** The truth table: PWM low → both outputs
low → motor stopped, whatever DIR does. The DIR pull-down only keeps the
direction deterministic. Fit both anyway; they cost nothing.

**Solder them straight onto the MD13S's Grove pins** (§3.2), not at the ESP32
end: there they also hold the motor off if the Grove cable breaks or shakes
loose, which on a vibrating machine is the likelier failure.

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
Tractor ISOBUS socket
  PWR ─────[ F1 10 A, right at the plug ]── cable 2 × 1.5 mm² ── gland ──► + block (WAGO 221, 4-way)
  PWR_GND ───────────────────────────────── same cable ────────── gland ──► − block (WAGO 221, 4-way)

+ block:  cable in · MD13S `+` · D24V22F5 VIN · TVS band (cathode)
− block:  cable in · MD13S `−` · D24V22F5 GND · TVS plain end      ← the single ground point (§3.4)

C1 1000 µF:  soldered straight onto the MD13S `+` / `−` pads, stripe to `−`, body glued down (§3.2)
D24V22F5 5 V ──► ESP32 `5V` (not while USB is plugged in) · encoder BLUE (Vcc) · level shifter HV
```

Everything meets on **two distribution blocks right beside the driver**, one
for `+` and one for `−`, with one short wire from each to the MD13S's screw
terminal. Landing four conductors of mixed gauge under one screw of the MD13S's
terminal block is not a joint that survives a season of vibration; a 4-way
WAGO 221 takes them cleanly and lets the driver come out for service without
taking the node apart.

**C1 is the one part that must be closer still** — on the MD13S's own pads,
because its whole job is to supply the switching current locally (item 13).
**The TVS sits on the blocks**: in a box this small the difference between the
cable gland and the driver is a fraction of a microhenry, and one junction is
more reliable than two.

The **regulator feeds only the ESP32, the encoder and the level shifter's HV
side**, and takes its 12 V from the same blocks, so C1 sits between the cable
and both loads. Motor current never passes through it. The level shifter's LV
side takes 3.3 V from the ESP32's `3V3` pin.

There is no series diode — the ISOBUS plug is keyed (item 15).

### 3.2 MD13S

**Wire by the silkscreen labels, not by the terminal colours** — Cytron's manual
and the shop listing disagree on which colour is which. Solder both terminal
blocks on first; they ship loose.

| MD13S | Connects to |
|---|---|
| Terminal `+` | the `+` distribution block (§3.1); C1's `+` leg soldered to its pad |
| Terminal `−` | the `−` distribution block — the single ground point (§3.4); C1's `−` leg (stripe) soldered to its pad |
| Terminal `MA` | Motor **red** wire |
| Terminal `MB` | Motor **black** wire |
| Grove `PWM` | ESP32 **GPIO 25**, plus **10 kΩ to GND** (§2.3) |
| Grove `DIR` | ESP32 **GPIO 26**, plus **10 kΩ to GND** (§2.3) |
| Grove `GND` | ESP32 **GND** — required; without a shared ground the logic inputs float |
| Grove `NC` | nothing |

The control input is the 4-pin Grove connector: cut one end off the supplied
cable and take the wires to the terminal adapter. The MD13S accepts 3.3 V and
5 V logic (high ≥ 3 V), so PWM and DIR connect straight to ESP32 pins with no
level shifter. Firmware drives PWM at 16 kHz; the driver's limit is 20 kHz.
With PWM low, both motor outputs go low, which brakes the motor.

The two test buttons on the board (MA, MB) drive the motor directly, whatever
the ESP32 is doing. Never press them with the auger connected or hands near it.

**Which Grove pin is which.** Cytron's manual numbers them 1 GND, 2 PWM,
3 DIR, 4 NC. Don't trust a pin order read off a photo, or this document's
earlier drafts: with the board unpowered, find GND by continuity to the `−`
power terminal, then read PWM and DIR off the silkscreen.

**Fitting the pull-downs.** Solder the two 10 kΩ straight onto the Grove pins,
PWM → GND and DIR → GND (§2.3 says why there). 0805 SMD resistors laid flat
across the pads, or through-hole parts with the leads cut short and sleeved.
Then check them before anything else is connected: about 10 kΩ from each pin
to GND, or less if the board turns out to have its own pull-down in parallel.

**Fitting C1.** Across the **power** block, `+` and `−` — never `MA`/`MB`,
where it would sit on a bridge reversing at 16 kHz and destroy itself and
probably the driver. Solder it to the pads under the terminal block, stripe
(`−`) to `−`: with no series diode it has no reverse protection, and a
reversed electrolytic vents or bursts. Keep its leads to a few centimetres at
most — lead inductance is exactly what it is there to beat. Then **fix the
body down** with RTV, hot glue or a cable tie: a 16 × 25 mm can shaken for a
season on its own two legs work-hardens them until they snap at the seal. If
you would rather not solder to the board, crimp C1's leg and the feed wire into
one ferrule so the screw clamps a single bundle. The first plug-in may spark
slightly as about 1660 µF charges (roughly 0.16 J) — normal.

### 3.3 Motor and encoder

Six 20 cm leads ending in one 1×6 female 0.1″ header. The two motor-power
leads carry up to 5.5 A, too much for that header: **cut red and black out of
it** and take them to the MD13S terminal block with ferrules. The four encoder
wires can stay on the header or go to a terminal block.

| Motor wire | Function | Connects to |
|---|---|---|
| **Red** | Motor power | MD13S `MA` |
| **Black** | Motor power | MD13S `MB` |
| **Green** | Encoder GND | Common ground |
| **Blue** | Encoder Vcc | **5 V** from the regulator — **never 12 V** |
| **Yellow** | Encoder channel A | Level shifter `HV1` |
| **White** | Encoder channel B | Level shifter `HV2` |

The encoder's A/B outputs are pulled up to its Vcc on the encoder board, so
they swing between 0 and 5 V. The BSS138 level shifter turns that into 0–3.3 V:

| Level shifter pin | Connects to |
|---|---|
| `HV` | 5 V from the regulator |
| `LV` | ESP32 `3V3` |
| `GND` (either side) | common ground |
| `HV1` / `LV1` | encoder yellow (A) / ESP32 **GPIO 32** |
| `HV2` / `LV2` | encoder white (B) / ESP32 **GPIO 33** |

Don't replace the level shifter with a resistor divider: the encoder's own
pull-up sits in series with the divider and the high level ends up around
2.5 V, right at the ESP32's threshold. The firmware also enables the ESP32's
internal pull-ups on GPIO 32/33, which is harmless alongside the module's.

**Channel A is the heart of the production firmware** — cut it and nothing
works. Every 100 ms the dispenser turns its edge count into the measured shaft
RPM, and the feed-forward + PI controller corrects the duty against it; that is
what absorbs a 12.6 V battery versus a 14.4 V alternator, and auger load. The
clog detector, the 100-revolution calibration run and the distance ledger all
count the same edges. With no encoder the measured RPM stays 0, the clog
detector fires after 1.5 s and the dispenser never meters.

Channel B is wired but unused by the production firmware — the auger turns one
way, so rising edges on A alone give ample resolution at half the interrupt
load. The bench test uses it to tell the two directions apart, so keep it
connected: the bench image is needed again on the fitted machine (§6, "What to
write down").

### 3.4 Grounding

Join motor ground and logic ground at **one single point**: the `−`
distribution block beside the MD13S (§3.1). Do not daisy-chain the ESP32's
ground through the motor current path — the volt-drop along that wire during a
5.5 A stall appears as ground noise on the encoder inputs and corrupts the
count.

**The return to the tractor is PWR_GND in the supply cable — never the seeder
frame.** Check there is no continuity from the single ground point to the
frame. If the motor case, the bracket or the `−` ends up bonded to the machine,
the frame and the three-point linkage become a second return path, and part of
every motor current — a 5.5 A stall included — comes back through the hitch.

---

## 4. Before the first motor run

1. **Check the encoder's blue wire goes to 5 V, not 12 V.** This is the one
   mistake that destroys the ESP32 — the encoder's outputs sit at its Vcc.
2. Confirm the two 10 kΩ pull-downs are present on PWM and DIR (§2.3, §3.2).
3. Confirm continuity from ESP32 GND to MD13S GND before applying 12 V — and
   **no** continuity from that ground to the seeder frame (§3.4).
4. Check the polarity with the multimeter's diode test before the first 12 V:
   the TVS's band faces +12 V — a reversed TVS shorts the rail and blows the
   fuse — and C1's stripe goes to `−`.
5. Power the ESP32 from **one source only** (§1.5 trap 6): with USB plugged in,
   the regulator's 5 V stays off the ESP32 `5V` pin.
6. **Check the encoder signals with a multimeter before `LV1`/`LV2` go to
   GPIO 32/33.** Coupling off the auger, ESP32 on USB, 12 V on — the
   pull-downs keep the motor still. The high level on the LV side comes from
   the module's `LV` pin, not from the encoder: with no 3.3 V on `LV`, `LV1`
   never goes high, and a meter reads it as about 0–0.1 V.
   - `LV` to GND reads 3.3 V, `HV` to GND 5 V, and blue to green at the motor's
     header 5 V. Measure on the module's own pins — its header is soldered by
     hand.
   - Turn the output shaft a few degrees one way, let go, read `HV1`; repeat
     about ten times. The level changes every 0.4° of the output shaft, so even
     slow turning is far too fast for a meter — read only with the shaft still.
     About half the stops read ~5 V and the rest ~0 V.
   - At each stop `LV1` reads ~3.3 V where `HV1` read ~5 V, and ~0 V where it
     read ~0 V — never 5 V, never ~2.5 V. Then the same for `HV2`/`LV2`.
   - `HV1` never high: take yellow off `HV1` and measure the bare wire the same
     way. If it now changes, the fault is on the module side — a solder joint,
     a short, or something holding `LV1` low (the module passes a low in both
     directions). If it still never goes high, the encoder has no supply, the
     wires don't match §3.3, or the encoder is damaged.
7. First motor run: hopper empty, coupling off the auger. **The bench test
   image in §6 does this automatically** — flash it first. Its H02 counts every
   encoder edge while you turn the shaft by hand, a far better check than a
   meter.
8. If the auger runs the wrong way, either swap the motor's red and black leads
   or flip `MOTOR_DIR_FORWARD` in `include/machine_settings.h`.
9. Once, on the tractor: the two ISOBUS checks in §1.5 trap 12 — whether PWR is
   still live with the key out, and whether it stays up under a bulb's load.

## 5. First calibration

1. Bench-verify `ENCODER_EDGES_PER_REV` (firmware assumes **480** rising edges
   per output-shaft revolution): turn the output shaft 10 revolutions by hand
   and check the count is ≈4800.
2. On the tractor: `Kalibracja` → cursor to `TEST` → long press → `START`. The
   dispenser turns exactly 100 revolutions and shows a progress bar. It refuses
   to start if the seeder reports the machine is moving.
3. Catch and weigh everything it delivers, in grams.
4. Enter that number as `Kalibracja` and save.

Per the speed table in CLAUDE.md (Design record → Dispenser): to sustain 10 km/h at 40 kg/ha the auger needs to
deliver roughly **≥ 800 g per 100 revolutions**. If calibration comes out much
below that, the motor cannot keep up at working speed — firmware clamps and
raises the `ZA SZYBKO` alarm rather than silently under-applying, but the real
fix is auger geometry.

---

## 6. Bench test

A separate firmware image (`src/dispenser_bench/`, built as the
`dispenser_bench` environment) that proves the module works before it goes
anywhere near the machine. It runs the **production** decision logic and the
production I/O — only `setup()`, the menu, radio reception and the packet
injection are its own — and it drives the real motor from simulated tractor and
seeder packets.

**Never fit a module that is running this image.** The summary always says so.

```bash
pio run -e dispenser_bench -t upload
pio device monitor -e dispenser_bench          # 115200 baud
```

The monitor does not reset the board on connect, so press `?` to reprint the
boot report and the menu. Keys are read one at a time.

### Before the first motor test

The board prints this and waits for `y` — once per session, before any test
that turns the motor:

1. the motor is clamped to the bench;
2. the coupling is off the auger and the hopper is empty (unless a test says
   otherwise);
3. the 12 V supply gives at least 6 A, through the 10 A fuse;
4. the encoder Vcc is on 5 V, not 12 V;
5. USB powers the ESP32: **the regulator's 5 V is disconnected from the ESP32
   `5V` pin** for the whole bench session (§1.5 trap 6). Keep the grounds joined
   and the encoder and the level shifter's HV side on the regulator.

**Any key pressed while a motor test runs aborts it** — the motor stops at
once, the test is recorded `ABORTED` and you are back at the menu. Operator
prompts are the only time a key is an answer, and every one of them can be
skipped with `s` (recorded as `SKIP`, never as a pass).

Stall tests use a lever, never fingers: locking pliers clamped on the coupling
with the handle against a fixed stop in the **forward** direction. K07 asks you
to fit it, and asks you to remove it before it does anything else.

### Recommended order

| Keys | What you need | What it proves |
|---|---|---|
| `d` | nothing connected | the decision logic and the distance ledger, 35 + 12 checks, milliseconds |
| `w` | nothing connected | ground speed from wheel pulses (the seeder's own logic), 11 checks |
| `v` | nothing connected | packet validation, the inbox and the link timing |
| `h` | coupling off, shaft free to turn | supply, driver, motor, both encoder channels, DIR, radio |
| `m` | coupling off, shaft free | metering and every link-loss case, with the motor running; M11 checks the turns delivered over simulated ground |
| `c` | coupling off, shaft free | the 100-revolution calibration run |
| `k` | coupling off; K07 needs the lever | clog detection and the unclog sequence |

`a` runs all of them in that order, `r` prints the summary so far, `x` stops the
motor, `?` reprints the menu and the boot report.

### What to write down

The summary prints the measured values worth copying into
`include/machine_settings.h`:

- **`ENCODER_EDGES_PER_REV`** — H02 has you turn the shaft 10 revolutions by
  hand; the count should be 4800 (480 per revolution).
- **`MOTOR_MIN_RUNNING_PERMILLE`** — H05 ramps up to find the duty that breaks
  the shaft away, and checks the configured value can start it from standstill.
- **`MOTOR_MAX_RPM`** — H03 measures the shaft RPM at full duty.
- **which channel A level means forward** — H04 separates the directions with
  channel B and asks you to confirm the forward run dispenses.

On the bench, H03 and H05 turn a free shaft, but `MOTOR_MAX_RPM` and
`MOTOR_MIN_RUNNING_PERMILLE` belong to the loaded auger at the tractor's
voltage. Measure those two again on the machine: bench image flashed, auger
coupled, fertilizer in the hopper, a bucket under the outlet, engine running,
then `h` with H02 and H06 skipped. The dose after every stop depends on
`MOTOR_MAX_RPM` being right (CLAUDE.md, Future tasks).

### Afterwards

```bash
pio run -e dispenser -t upload
```

Reflash the production firmware before the module is fitted. The bench image
never sends a status packet and never listens for one, so a machine left with
it flashed would look dead to the tractor.

---

## Sources

- [Pololu 4752 — 30:1 Metal Gearmotor 37Dx68L mm with 64 CPR encoder](https://www.pololu.com/product/4752) (leads, shaft, encoder supply, torque limits)
- [Pololu forum — pull-up resistors on the 37D encoder board](https://forum.pololu.com/t/d37-gearmotor-encoder-pull-up-resistors/6499) and [10 kΩ pull-ups on the 25D encoder](https://forum.pololu.com/t/encoder-pull-up-resistors/23784)
- [Cytron MD13S product page](https://www.cytron.io/p-13amp-6v-30v-dc-motor-driver) and [MD13S User's Manual V1.1](https://download.kamami.pl/p576759-MD13S%20Users%20Manual.pdf) (30 V absolute maximum, Grove input pinout, truth table, test buttons)
- [Pololu D24V22F5 5 V 2.5 A step-down regulator](https://www.pololu.com/product/2858)
- [Espressif ESP32-DevKitC user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html) (power from one source only)
- [Littelfuse SMBJ series datasheet](https://www.littelfuse.com/assetdocs/tvs-diodes-smbj-series-datasheet?assetguid=ba555e99-a12d-4f72-a0b6-86b06c67171e) and [1.5KE series datasheet](https://www.digikey.com/htmldatasheets/production/99296/0/0/1/1n6267a-1-5kexxa-series.html) (clamping voltages)
- Botland product pages checked for §1.4: [MD13S](https://botland.store/motor-drivers-modules/12412-cytron-md13s-single-channel-30v-13a-motor-controller-5904422377090.html), [Pololu 4752](https://botland.store/dc-motors-with-gearbox-and-encoders/16115-301metal-gearmotor-12v-37dx68lmm-with-64-cpr-encoder-pololu-4752-5904422325213.html), [ESP32-DevKitC-32E](https://botland.store/esp32-wifi-and-bt-modules/8306-esp32-devkitc-32e-v4-wifi-bt-42-platform-with-the-module-esp-wroom-32e-5904422336394.html), [D24V22F5](https://botland.store/converters-step-down/4978-step-down-voltage-converter-d24v22f5-5v-25a-pololu-2858-5904422365769.html), [BSS138 level shifter](https://botland.store/voltage-converters/6117-4-channel-logic-level-converter-5904422365189.html), [Pololu 1084 bracket](https://botland.store/bracket/2343-aluminium-motor-mount-37d-2pcs-pololu-1084-5904422300258.html)
- [ESP-IDF GPIO & RTC GPIO reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
- [Espressif ESP32 Pin List](https://download.kamami.pl/p563391-esp32_chip_pin_list_en.pdf) (§2.3: every pin output-disabled during reset; no pull-up or pull-down on GPIO 25/26 at reset)
- [Pololu MAX14870 motor driver carrier](https://www.pololu.com/product/2961) (§2.3: pull-downs on a driver's control inputs as a deliberate design choice)
- [Arduino Forum — pull-downs on output pins during power-up and reset](https://forum.arduino.cc/t/pull-down-resistor-on-digital-io-pin-used-as-output-low-level-on-this-pin-during-power-up-restart/917732) (§2.3: not needed on opto-isolated inputs, worth it on critical control lines)
- [ESP32-WROOM-32 datasheet](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.html)
- [Espressif ESP32 hardware design guidelines](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/schematic-checklist.html)
