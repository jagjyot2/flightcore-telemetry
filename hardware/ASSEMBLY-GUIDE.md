# FlightCore Hardware Assembly & Bench Bring-Up Guide

> **Pre-Assembly Reminder**: FlightCore is strictly a telemetry/sensing module. Never route ESC PWM wires to FlightCore or fly without securing all loose harnesses away from propellers.

---

## 1. Mechanical & PCB Preparation
- **PCB Specs**: `96.0 mm × 68.0 mm`, 2-Layer `1.6 mm` FR4, `1 oz` (or `2 oz` recommended for >35 A builds) copper, HASL/ENIG finish.
- **3D Printed Tray**: Print `benchtwin/enclosure_mount.stl` in PETG or TPU-damped PETG (`0.2 mm` layer height, `40%` infill). Verify the central downward ToF cutout window (`60..72 mm × 22..38 mm`) aligns directly beneath `U3` (`VL53L0X`).
- **Mounting Hardware**: 4 × M3 × 10 mm nylon/aluminum standoffs on the `89.0 mm × 61.0 mm` hole pattern (`H1..H4`).

---

## 2. Recommended Soldering Order

1. **Low-Profile SMD Passives & ICs (Top Side)**:
   - Solder `U5` (`INA180A2` SOT-23-5), `D2` (`BAT54S` SOT-23), `U6` (`AMS1117-3.3` SOT-223).
   - Solder `R1..R10`, `R12` (0805), `C2..C13` (0805), `F1` (1206), `D1` (SMA), `D3` (SMB), `D4` (0805 LED — align cathode mark toward `R12`), `SW1`.
   - Solder `R11` (`1 mΩ` 2512 shunt) with generous solder fillet and optionally reinforce the `VBAT_IN` / `VBAT_LOAD` copper pads with 14 AWG solder bridge if operating above 35 A continuous.
2. **RF & SMA Edge Stage**:
   - Solder `U4` (`SX1278 Ra-02`) castellations cleanly using flux. Inspect for solder bridges on `NSS/MOSI/MISO/SCK`.
   - Solder `J5` edge-mount SMA connector. **Attach the 433 MHz antenna (`ANT1`) immediately** — never power `U4` in TX mode without a 50 Ω load!
3. **Through-Hole Modules, Headers & Bulk Electrolytic**:
   - Install `C1` (`100 µF` 35 V — verify `+` long lead toward `VBAT_PROT` square pad).
   - Install female pin headers for `U1` (2 × 15-pin 2.54 mm) so ESP32 DevKit v1 stays removable/replaceable.
   - Install `U7` (5 V / 3 A Buck module), `U2` (`MPU6050` 8-pin, keep level with PCB!), `U3` (`VL53L0X` 6-pin, facing downward through tray cutout window if mounted inverted, or angled outward/downward per airframe bay).
   - Install `J3`, `J4`, `J6` JST-XH headers, `JP1` 2-pin jumper header, and `J1`/`J2` XT60 connectors.

---

## 3. Crucial Jumper & Module Checks Before Power-On

- **Check `JP1` Rule**:
  - **Bench USB-only test (NO battery connected)**: Install `JP1` jumper cap so DevKit `3V3` powers `+3V3_BUS`.
  - **Battery / Buck BEC mode (J1 connected)**: **REMOVE `JP1` jumper cap** so `U6` (`AMS1117-3.3`) powers `+3V3_BUS` cleanly without fighting the DevKit's onboard LDO!
- **MPU6050 (`U2`) LDO Check**:
  - If using a `GY-521` breakout fitted with a 3.3 V onboard LDO (`RT9193`), feeding `3.3 V` into `VCC` runs the IMU at ~3.15 V (within `2.375–3.46 V` spec). For nominal `3.30 V`, bridge the GY-521 LDO or use a 3.3 V-direct carrier.
- **A3144 Hall Sensor (`U8` via `J3`)**:
  - Mount `U8` (`A3144EUA-T`) within `2–4 mm` of a small neodymium magnet glued/balanced onto the motor bell/hub. Keep wires twisted (`+5V`, `SIG`, `GND`) and away from ESC phase wires.

---

## 4. Safe Smoke-Test & Bring-Up Checklist

1. **Continuity Check (Power OFF)**:
   - Verify resistance between `VBAT_IN` and `GND` > `50 kΩ`, `+5V_BUS` and `GND` > `1 kΩ`, `+3V3_BUS` and `GND` > `500 Ω`.
2. **USB Bench Bring-Up (`JP1` CLOSED, NO Battery)**:
   - Flash firmware: `pio run -e esp32dev -t upload`
   - Open serial monitor at `115200` baud; verify boot banner reports:
     `[INIT] MPU6050 (0x68): OK`, `[INIT] VL53L0X (0x29): OK`, `[INIT] SX1278 433MHz (ver=0x12): OK`.
   - Type `status` and `json` in CLI to confirm live attitude, ToF distance, and NTC room temperature (`~20–30 °C`).
3. **Current-Limited Battery Test (`JP1` OPEN, Antenna `ANT1` Attached)**:
   - Connect current-limited bench PSU (`12.0 V`, `0.5 A` limit) to `J1`.
   - Probe `+5V_BUS` (`4.95–5.05 V`) and `+3V3_BUS` (`3.27–3.33 V`).
   - Verify CLI `status` reads `vbat ≈ 12.00 V` within `±0.15 V`.
   - Run ground station decoder against USB binary stream (`bin` CLI command) or paired 433 MHz LoRa receiver:
     `python -m ground_station.receiver_cli --serial COM3 --format table`.
