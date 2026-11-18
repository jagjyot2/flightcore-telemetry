# FlightCore Power Distribution & Protection Architecture

> **Design Objective**: Deliver noise-isolated `5.0 V` and `3.3 V` rails for ESP32, IMU, ToF laser, Hall sensor, and +17 dBm LoRa TX bursts from a `2S–6S LiPo` (`7.4 V – 25.2 V`) propulsion battery while maintaining zero back-feed hazard when connected via USB.

---

## 1. Power Tree Diagram

```
  J1 (XT60 Battery IN: 7.4V – 25.2V, up to 60A)
   │
   ├──► [R11: 1 mΩ 3W 2512 Shunt] ──► J2 (XT60 PDB/ESC Pass-Through)
   │       │         │
   │       IN+      IN-
   │       └──► U5 (INA180A2, 50 V/V) ──► R10 (100Ω) / C7 (100nF) ──► GPIO35 (IBAT_ADC)
   │
   └──► [F1: 1.5A Polyfuse] ──► [D1: SS34 Reverse Polarity] ──► VBAT_PROT
                                                                   │
          ┌────────────────────────────────────────────────────────┤
          ▼                                                        ▼
   [D3: SMBJ28A TVS] + [C1: 100µF 35V]                   [R7: 100kΩ / R8: 10kΩ]
          │                                               + [D2: BAT54S] + [C6: 100nF]
          ▼                                                        │
   [U7: MP1584 5V / 3A Buck BEC]                                   ▼
          │ (+5V_BUS, C2 10µF)                              GPIO34 (VBAT_ADC)
          ├──► U1 ESP32 DevKit VIN (Internal ESP32 3.3V LDO)
          ├──► J3.1 (A3144 Hall VCC, 4.5V–24V rated)
          ├──► J6.1 (ESP32-CAM 5V optional rail)
          └──► [U6: AMS1117-3.3 LDO] (+3V3_BUS, C3 10µF in / C4 22µF out)
                     ├──► U4 SX1278 LoRa (+3V3_BUS, C11 10µF + C12 100nF)
                     ├──► U2 MPU6050 + U3 VL53L0X + I2C/CS/XSHUT/Hall Pull-Ups
                     ├──► U5 INA180A2 VS (+3V3_BUS, C13 100nF)
                     └──► R9 10kΩ NTC Reference Pull-Up
```

---

## 2. Current Budget & Rail Sizing

| Rail | Regulator | Max Capacity | Typical Load | Peak Burst Load | Margin |
|---|---|---|---|---|---|
| `VBAT_IN -> J2` | Direct 2.0 mm copper + `R11` | `54 A` cont (`1mΩ` @ `3W`: $I=\sqrt{3/0.001}=54.7\,\text{A}$) | `12–30 A` (quad hover) | `50 A` | Safe for 45A–50A 4S/6S builds |
| `+5V_BUS` | `U7` MP1584 Buck | `3000 mA` | `~480 mA` (ESP32 + CAM + LDO) | `~850 mA` | > 3.5× headroom |
| `+3V3_BUS` | `U6` AMS1117-3.3 | `1000 mA` | `~45 mA` (Sensors + RX LoRa) | `~145 mA` (+17 dBm TX burst) | > 6.8× headroom |
| `ESP32_3V3` | Onboard DevKit LDO | `~500 mA` | `~110 mA` (ESP32 SoC active) | `~240 mA` (RF startup) | Isolated from `+3V3_BUS` |

### Why Split `ESP32_3V3` and `+3V3_BUS`?
1. **LDO Contention Prevention**: Connecting two independent 3.3 V regulators (`U1` onboard AMS1117 and `U6` carrier AMS1117) directly in parallel causes back-feed current and thermal oscillation. Jumper `JP1` isolates them during battery operation.
2. **LoRa PA Burst Immunity**: `U4` (SX1278) draws ~95–120 mA pulsed current during LoRa transmissions; dedicated `U6` + `C11 (10µF)` + `C12 (100nF)` isolates ADC reference (`+3V3_BUS` feeding `R9` NTC and `U5`) from ESP32 CPU switching transients.
3. **USB-Only Bench Mode**: When running exclusively from USB (`VBAT_IN` disconnected), close `JP1` so DevKit `3V3` powers `+3V3_BUS` for bench sensor verification (keep LoRa at `<= +14 dBm` or short bursts on USB!).

---

## 3. High-Side Current & Voltage Sensing Calculations

### Battery Voltage Divider (`R7 = 100 kΩ`, `R8 = 10 kΩ`, 1%)
$$V_{\text{ADC}} = V_{\text{BAT\_PROT}} \cdot \frac{10\,\text{k}\Omega}{100\,\text{k}\Omega + 10\,\text{k}\Omega} = \frac{V_{\text{BAT\_PROT}}}{11.0}$$
- At `6S` max charge (`25.2 V`): `V_ADC = 2.291 V` (inside ESP32 `ADC_11db` linear region).
- Clamped by `D2 (BAT54S)` to `[GND - 0.3V, +3V3_BUS + 0.3V]` and low-pass filtered at $f_c \approx \frac{1}{2\pi (9.09\text{k}\Omega)(100\text{nF})} \approx 175\,\text{Hz}$.

### Current Shunt Front-End (`R11 = 1 mΩ`, `U5 = INA180A2`, Gain = `50 V/V`)
$$V_{\text{OUT}} = I_{\text{BAT}} \cdot R_{\text{SHUNT}} \cdot G = I_{\text{BAT}} \cdot 0.001\,\Omega \cdot 50 = 0.050\,\text{V/A}$$
- At `30 A`: `V_OUT = 1.500 V`, shunt power dissipation $P = 30^2 \cdot 0.001 = 0.90\,\text{W}$ (30% of `3 W` 2512 rating).
- Maximum measurable current before `2.6 V` ADC saturation: `52.0 A` (`3.3 V` rail headroom allows up to `66 A`).
- **Common-Mode Check**: `INA180A2` supports `V_CM = -0.2 V .. +26.0 V` (`30 V` absolute max), safely covering `2S–6S LiPo` (`<= 25.2 V`).

### Fault & Transient Protection
- **`F1` (1.5 A PTC Polyfuse)**: Trips on buck/sensor rail fault without interrupting main ESC battery loop (`J1 -> J2`).
- **`D1` (SS34 Schottky)**: Blocks reverse battery connection on FlightCore electronics.
- **`D3` (SMBJ28A TVS)**: Clamps hot-plug LiPo inductive spikes above `28 V` (`V_c < 45 V`).
