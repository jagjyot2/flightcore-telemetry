# Manufacturing Readiness Report — `04-FLIGHTCORE` (`flightcore`)

> **Verdict (`2026-11-19`, Phase 3 Remediated):** **`READY FOR FAB WITH 2 oz COPPER SPEC FOR > 25 A`** *(or use default `1 oz` copper strictly capped `≤ 20–25 A` continuous / bench telemetry)*
> All Phase 2 electrical/mechanical fabrication blockers (`ISS-007`, `ISS-013`) have been remediated and verified in KiCad `10.0.8` (`0` ERC / `0` DRC / `0` Unconnected / `0` Parity) and `20/20` `pytest` checks.

---

## 1. Board Specifications & KiCad `10.0.8` DRC/ERC Summary (Observed Live)

| Parameter | Verified Measurement |
|---|---|
| **Board Dimensions & Finish** | `96.0 mm × 68.0 mm × 1.6 mm` (2-Layer FR4, ENIG finish) |
| **Mounting Pattern** | **`87.0 mm × 59.0 mm`** (`H1..H4` at `(4.5, 4.5)`, `(91.5, 4.5)`, `(91.5, 63.5)`, `(4.5, 63.5)` — **`1.90 mm` copper-to-edge clearance**, up from `0.90 mm`) |
| **BOM / Footprints** | **`49`** footprints/holes (`R11` 4-wire Kelvin shunt, `2.50 mm` JST-XH `J3/J4/J6`) |
| **Routed Nets / Segments** | **`33`** nets, **`4,579`** track segments (`AutoRouter` failed: `[]`) |
| **KiCad ERC (`run-erc.ps1`)** | **`0` errors, `0` warnings** |
| **KiCad DRC (`run-drc.ps1 --schematic-parity --severity-all`)** | **`0` violations, `0` unconnected pads, `0` footprint errors, `0` schematic parity issues** |
| **Test Suite (`04-FLIGHTCORE/tests`)** | **`20/20 PASSED`** (`test_cpp_native_suite`, `test_hardware_and_benchtwin_artifacts`, `test_protocol_and_ground_station`, `test_sensor_math`, `test_simulation_architecture_and_wokwi`) |

---

## 2. Phase 3 Electrical & Trace Ampacity Remediation (`ISS-007`)

- **4-Wire Kelvin Shunt (`R11` `CSS4J-4026R-2L00F`)**: Replaced 2-pad `R_2512_Shunt` with true 4-terminal Kelvin shunt; inner-tab `SHUNT_SENSE_P` / `SHUNT_SENSE_N` sense traces route cleanly (`0.22–0.25 mm`) straight to `U5` (`INA180A2IDBVR`).
- **Dual-Layer High-Current Buses (`VBAT_IN`, `VBAT_LOAD`)**: Routed across `J1 (XT60)` -> `R11` -> `J2 (XT60)` using **`3.5 mm` dual-layer `F.Cu + B.Cu`** copper bars (`7.0 mm` equivalent copper width, stitched with `1.1 mm / 0.5 mm` vias); `VBAT_IN` branch to `F1` (`1.5 A` polyfuse) uses `1.2 mm` (`zero` `< 1.2 mm` neckdowns on main current path).
- **Sub-Rail & RF Widths**: `VBAT_FUSED`/`VBAT_PROT` (`0.8 mm`), `+5V_BUS` (`0.6 mm`), `+3V3_BUS` (`0.5 mm`), `LORA_RF_ANT` (`0.95 mm` `50 Ω` feed to `J5` SMA).
- **`B.Cu GND` Plane**: `4` connected grounded polygons with `0` DRC `[isolated_copper]` warnings (`island_removal_mode 0`).

---

## 3. Footprint & Fabrication Spec Notes (`ISS-013`)

- **JST-XH Sensor Connectors (`J3` Hall, `J4` NTC, `J6` CAM UART)**: Replaced generic `2.54 mm` headers with keyed **`2.50 mm` JST-XH** footprints (`fp_jst_xh_1xN`).
- **Fab House Note**: Because KiCad's default board stackup leaves outer copper thickness unpopulated (`1 oz` default at standard fabs), explicitly select **`2 oz` (`0.070 mm`)** outer copper at order checkout if sustaining `30–60 A` pack draw (and match benchtop enclosure standoff CAD to `87.0 × 59.0 mm` before 3D printing).
