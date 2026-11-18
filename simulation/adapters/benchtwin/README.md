# BenchTwin Preserved Adapter (`04-FLIGHTCORE`)

BenchTwin is preserved intact in `benchtwin/` without modifying `firmware/` or `hardware/`.
Wokwi is the active runtime simulator, while this adapter manifest keeps BenchTwin ready for future reactivation (circuit visualization, Ohm's-law/RC analog node inspection, KiCad netlist cross-checking, and 3D `enclosure_mount.stl` inspection).

## Reactivation Steps

1. Open BenchTwin and load `benchtwin/flightcore.btw` (or import `benchtwin/flightcore-benchtwin.json` + `benchtwin/flightcore.net`).
2. Select a preset from `benchtwin/flightcore.btw` (e.g., `nominal_4s_hover`).
3. Verify analog node voltages (`VBAT_ADC` on `GPIO34`, `IBAT_ADC` on `GPIO35`, `NTC_ADC` on `GPIO36`) and confirm `JP1` is `OPEN` when simulating `VBAT_IN`.
4. See [../../../benchtwin/BENCHTWIN-SIMULATION-NOTES.md](../../../benchtwin/BENCHTWIN-SIMULATION-NOTES.md) for the BenchTwin fidelity matrix.
