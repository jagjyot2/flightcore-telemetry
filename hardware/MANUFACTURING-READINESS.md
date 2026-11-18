# Manufacturing Readiness Report — `04-FLIGHTCORE` (`flightcore`)

- **Board Reviewed**: [`flightcore.kicad_pcb`](flightcore.kicad_pcb) / [`flightcore.kicad_sch`](flightcore.kicad_sch)
- **Audit Date**: `2026-11-18`
- **Release Decision**: **`HOLD — DO NOT FABRICATE`** (critical `60 A` power-path trace undersizing and tight corner mounting clearance)

---

## 1. Board Specifications & DRC/ERC Summary (Observed Live)

| Parameter | Measured Value | Status / Observation |
|---|---|---|
| Dimensions (`Edge.Cuts`) | `96.0 x 68.0 mm`, thickness `1.6 mm`, `ENIG` | `VERIFIED` |
| Copper Stackup | `2` Layers (`F.Cu`, `B.Cu`), `1 oz` Cu | `VERIFIED` |
| Placed Symbols / Footprints | `49` / `49` (`45` components + `H1..H4`); `BOM.csv`: `49` rows | `VERIFIED` |
| KiCad `10.0.8` ERC (`--severity-all`) | `0` Errors, `0` Warnings (exit `0`) | `VERIFIED WITH LIMITATIONS` |
| KiCad `10.0.8` DRC (`--schematic-parity`) | `0` Violations, `0` Unconnected Pads, `0` Parity Issues (exit `0`) | `VERIFIED WITH LIMITATIONS` (`courtyard` & `filter` checks ignored by KiCad default) |
| Mounting Holes (`H1..H4`) | `(3.5, 3.5)`, `(92.5, 3.5)`, `(92.5, 64.5)`, `(3.5, 64.5)` (`3.2 mm` drill, `5.2 mm` pad) | **`VERIFIED WITH LIMITATIONS`** (**`0.90 mm` copper pad-to-edge clearance** — tight for V-cut depaneling; recommend `>= 1.2 mm` / move holes to `(4.5, 4.5)`) |
| Vias & Drills (`flightcore-PTH.drl`) | `93` vias (`0.70 / 0.35 mm`); component drills: `0.90`, `1.00`, `1.20`, `3.20`, `3.80 mm` | `VERIFIED` |
| `B.Cu` Ground Pour (`GND`) | `0.25 mm` clearance, `0.35 mm` thermal bridge, **split into `6` `filled_polygon` islands**; `GND` traces down to `0.22 mm` | **`FAILED` / `VERIFIED WITH LIMITATIONS`** (fragmented ground return plane under `ESP32` + `SX1278` RF section) |
| Gerber / Drill Files (`hardware/gerbers/`) | Complete 2-layer Gerber set (`9` Gerber + `.gbrjob` + `2` Excellon `.drl` + PDF drill maps) + `drc_report.txt` (stale) & updated `flightcore_{erc,drc}.{txt,json}` | `VERIFIED WITH LIMITATIONS` |

---

## 2. Critical Electrical & Trace Ampacity Findings

| Net / Path | Measured Widths (mm) | Length (mm) | Spec / Expected Current | Engineering Assessment | Status |
|---|---|---|---|---|---|
| `VBAT_IN` (`J1 XT60` -> `R11`) | `[0.22, 0.70]` | `24.7` | Up to **`60 A`** design spec (`XT60` + `1 mOhm` 3W shunt) | **`0.70 mm` `1 oz` Cu carries `~2.2 A` (`20 °C` rise) and fuses well below `60 A`**. Even `0.22 mm` segments exist on `VBAT_IN`. Completely unsafe for pass-through PDB load. | **`FAILED`** |
| `VBAT_LOAD` (`R11` -> `J2 XT60`) | `[0.60]` | `42.4` | Up to **`60 A`** | `0.60 mm` `1 oz` Cu (`~1.9 A` @ `20 °C` rise) will delaminate/vaporize under `20–60 A` propulsion load. Must use full-width `2 oz` copper polygon pours + exposed solder-mask busbar reinforcement. | **`FAILED`** |
| `VBAT_FUSED`, `VBAT_PROT` | `[0.60]` | `9.0`, `48.0` | `<= 1.5 A` (`F1` polyfuse) | Adequate for `<= 1.5 A` avionics input (`F1`) | `VERIFIED` |
| `+5V_BUS`, `+3V3_BUS` | `[0.45]`, `[0.40]` | `144.3`, `282.6` | `~0.6–1.5 A` (`ESP32` + `SX1278` TX bursts) | Marginal under `ESP32` + `SX1278` peak TX (`~650 mA`); widen `+3V3_BUS` / `+5V_BUS` to `>= 0.8–1.0 mm`. Notice `U6 AMS1117-3.3` drops `5.0 -> 3.3 V` (`0.85 W` at `0.5 A`). | **`VERIFIED WITH LIMITATIONS`** |
| `LORA_RF_ANT` (`U4` -> `J5 SMA`) | `[0.25, 0.95]` | `14.8` | `433 MHz` RF | Trace step from `0.25 mm` to `0.95 mm` on `1.6 mm` FR4 is not a matched `50 Ohm` CPWG (`~3.0 mm` microstrip on `1.6 mm` FR4 without coplanar ground). | **`VERIFIED WITH LIMITATIONS`** |

---

## 3. Footprint & Measurement Review

- **`R11` (`0.001R-3W-Shunt`) — `FAILED`**: Placed as 2-pad `FlightCore:R_2512_Shunt` rather than 4-wire Kelvin (`IBAT_SENSE_RAW` taps straight off high-current pad), introducing heavy copper resistance error into `INA180A2` (`G=50 V/V`).
- **`J1`, `J2` (`AMASS_XT60_Vertical`) — `PROVISIONAL`**: `7.2 mm` pitch (`-3.6` to `+3.6 mm`), `3.8 mm` circular drill (`5.6 mm` pad = `0.9 mm` annular ring), lacks explicit silkscreen `+ / -` polarity markers inside the footprint.
- **`J3`, `J4`, `J6` (`JST-XH`) — `PROVISIONAL`**: Placed on `2.54 mm` pin headers instead of genuine `2.50 mm` JST `B3B/B2B/B4B-XH-A` shrouded housings (`0.12 mm` cumulative pitch offset across 4 pins, plus missing mechanical key polarization).
- **`U2` (`MPU6050-GY521`), `U3` (`VL53L0X`), `U7` (`MP1584` module) — `PROVISIONAL`**: Breakout module pin headers (`1x08`, `1x06`, `Buck_5V_3A_Module`) assume third-party module pin order/mechanical clearance.
- **Strict Telemetry-Only Boundary — `VERIFIED`**: No flight control or actuator outputs on `flightcore` hardware (`ESP32` logs/transmits LoRa telemetry only).

## 4. Required Actions Before Fabrication
1. Convert `J1 -> R11 -> J2` (`VBAT_IN` / `VBAT_LOAD`) into dual-layer (`F.Cu + B.Cu`) wide copper polygons (`>= 8–10 mm` equivalent width on `2 oz` Cu with stitched via arrays and solder-mask-free tinning strips) AND replace `R11` with a 4-wire Kelvin shunt (or explicitly cap `FlightCore` to `< 3 A` bench sensing).
2. Unify the `B.Cu` `GND` plane into `1` contiguous region (currently `6` islands) and pull `H1..H4` inward to `(4.5, 4.5)`+ (`>= 1.5 mm` pad-to-edge clearance).
3. Replace `2.54 mm` headers on `J3/J4/J6` with keyed `2.50 mm` JST-XH footprints and add silkscreen polarity markings on `J1/J2 XT60`.
