# FlightCore BenchTwin Simulation & Staging Notes

This directory (`benchtwin/`) contains the prepared export and local staging bundle for **BenchTwin** circuit visualization, wiring verification, BOM cross-checking, and 3D enclosure inspection.

---

## 1. Staged Files in `benchtwin/`

| File | Purpose |
|---|---|
| `flightcore-benchtwin.json` | Structured `benchtwin/circuit-v1` component, pin, net, and wire graph matching `hardware/flightcore.net` |
| `flightcore.btw` | BenchTwin project archive bundle linking circuit graph, BOM, STL, SVG, and nominal/fault simulation presets |
| `flightcore.net` | KiCad E-format netlist synchronized 1:1 with `hardware/flightcore.net` |
| `BOM.csv` | Synchronized component bill of materials (`hardware/BOM.csv`) |
| `enclosure_mount.stl` | Watertight closed-manifold 3D-printable tray (`102 × 74 mm`, `89 × 61 mm` M3 standoffs, downward ToF window) |
| `schematic_preview.svg` | Vector schematic block & pin-bus wiring diagram |

---

## 2. Simulation Fidelity Matrix (Simulated vs. Behavioral Stubs)

Because **BenchTwin** natively simulates linear/discrete electrical networks, power dividers, and basic GPIO logic—while complex MEMS/RF ICs lack full SPICE/digital transistor-level models—the table below explicitly documents how each subsystem behaves in simulation vs. physical hardware:

| Component | Designator | BenchTwin Mode | Exact Fidelity & Limitations |
|---|---|---|---|
| **Resistors & Voltage Dividers** (`R1..R12`, `R11` 1 mΩ) | `R1..R12` | **Native Electrical Simulation** | Exact Ohm's-law division (`100k/10k` = `11:1` VBAT scale; `10k` NTC pull-up; `4.7k` I2C pull-ups). |
| **Capacitors & RC Filters** (`C1..C13`) | `C1..C13` | **Native Electrical Simulation** | Exact RC time constants (`R6/C5` Hall debounce; `R10/C7` current anti-alias; decoupling bypass). |
| **Protection Diodes & Switches** (`D1..D4`, `SW1`, `JP1`, `F1`) | `D1..D4`, `SW1`, `JP1`, `F1` | **Native Electrical Simulation** | Forward voltage drops (`SS34`, `BAT54S` clamping), TVS knee, LED current (`R12`), button pull-low. |
| **10 kΩ NTC Thermistor** (`RT1`) | `RT1` | **Native Variable Resistor Model** | Swept via resistance presets (`10 kΩ` @ `25 °C`, `2.488 kΩ` @ `60 °C`) matching Steinhart-Hart math. |
| **ESP32 DevKit v1** (`U1`) | `U1` | **MCU Pin-Bus + Firmware Math Harness** | Physical GPIO pin continuity and ADC1 channel mapping verified; full Xtensa firmware validated via `pio run -e esp32dev` + C++ native test binary (`firmware/test/test_flightcore_native.cpp`). |
| **INA180A2 Current Sense Amp** (`U5`) | `U5` | **Behavioral Gain Stub (`50 V/V`)** | Approximated as ideal `V_OUT = 0.050 * I_BAT` voltage source on `GPIO35`; common-mode transient saturation above `26 V` is documented analytically rather than transistor-modeled. |
| **A3144 Hall-Effect Sensor** (`U8` via `J3`) | `U8` | **Behavioral Open-Collector Pulse Generator** | Simulated as `0–3.3 V` open-collector square-wave pulse train (via `R5` pull-up); magnetic flux hysteresis threshold (`B_OP / B_RP`) is not magnetically modeled. |
| **MPU6050 6-Axis IMU** (`U2`) | `U2` | **I2C Behavioral Stub (Pending Live Session)** | I2C bus wiring (`GPIO21/22`, `0x68`) and pull-ups verified electrically; 6-axis register traffic, startup bias calibration, complementary filter, and vibration RMS are verified in C++/pytest harnesses rather than live BenchTwin MEMS simulation. |
| **VL53L0X ToF Ranging** (`U3`) | `U3` | **I2C Behavioral Stub (Pending Live Session)** | Optical time-of-flight laser propagation is not ray-traced; I2C (`0x29`) + `XSHUT` (`GPIO26`) wiring is verified, and median-5 + EMA glitch filter math is verified in C++/pytest. |
| **SX1278 Ra-02 433 MHz LoRa** (`U4`) | `U4` | **SPI Register Mock / RF Stub** | 433 MHz RF propagation, path loss, and antenna impedance (`J5`) are NOT simulated electromagnetically; SPI register sequencing, timeout retries, and 37-byte CRC-16-CCITT framing are verified via `MockSx1278Transport` and `ground_station/protocol.py`. |
| **ESP32-CAM Bridge** (`MOD_CAM` via `J6`) | `J6` | **UART2 Pin-Header Stub** | UART2 pin routing (`TX2=17`, `RX2=16`, `5V`, `GND`) is staged; no live MJPEG camera stream simulation. |

---

## 3. Importing into a Live BenchTwin Session

1. Open BenchTwin and load `benchtwin/flightcore.btw` (or import `benchtwin/flightcore-benchtwin.json` + `benchtwin/flightcore.net`).
2. Select preset **`nominal_4s_hover`** (`15.6 V`, `16.5 A`, `6.8 kΩ` NTC, `130 Hz` Hall) to inspect analog node voltages:
   - `VBAT_ADC` (`GPIO34`) = `15.6 / 11.0 = 1.418 V`
   - `IBAT_ADC` (`GPIO35`) = `16.5 * 0.050 = 0.825 V`
   - `NTC_ADC` (`GPIO36`) = `3.3 * 6800 / (10000 + 6800) = 1.336 V` (`~33.9 °C`)
3. Confirm **`JP1`** state matches power mode (`OPEN` when simulating `VBAT_IN` battery supply).
