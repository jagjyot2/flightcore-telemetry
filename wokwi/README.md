# FlightCore Wokwi Simulation Setup

This directory (`wokwi/`) configures **Wokwi** as the active runtime simulator for `04-FLIGHTCORE`, consuming the PlatformIO-compiled ESP32 ELF/BIN directly without modifying `firmware/`, `hardware/`, or `benchtwin/`.

---

## 1. Build Firmware with PlatformIO

```powershell
cd firmware
pio run -e esp32dev
```

This produces:
- `firmware/.pio/build/esp32dev/firmware.bin`
- `firmware/.pio/build/esp32dev/firmware.elf`

Both paths are referenced in [wokwi.toml](wokwi.toml).

---

## 2. Run in VS Code Wokwi Extension

1. Install the **Wokwi Simulator** extension in VS Code.
2. Open the `04-FLIGHTCORE/wokwi/` folder (or select `wokwi/wokwi.toml` as the active Wokwi project).
3. Press **F1 -> Wokwi: Start Simulator**.
4. Interact with:
   - `imu_mpu6050`: Adjust accel/gyro/temp sliders.
   - `ntc_probe`: Adjust NTC thermistor temperature.
   - `pot_vbat` / `pot_ibat`: Adjust simulated `VBAT_ADC` (`GPIO34`) and `IBAT_ADC` (`GPIO35`).
   - `btn_hall` (`GPIO33`) / `btn_cal` (`GPIO4`): Inject Hall RPM edges or trigger IMU calibration.
   - Serial Monitor (`115200 baud`): Send `status`, `json`, `hex`, `cal imu`, `interval 200`.

---

## 3. Run Headlessly with Wokwi CLI

Headless CI/CLI runs require a Wokwi token set in the environment:

```powershell
$env:WOKWI_CLI_TOKEN = "<your-wokwi-cli-token>"
wokwi-cli wokwi --timeout 10000
```

> **Note on Custom Chips**: `chip-vl53l0x` and `chip-sx1278` are defined in `wokwi/chips/` (`.chip.json` + `.chip.c`) and compiled to `.chip.wasm` by the Wokwi CLI / Wokwi build server. See [../simulation/documentation/COMPONENT-SUPPORT-MATRIX.md](../simulation/documentation/COMPONENT-SUPPORT-MATRIX.md) for native vs. stubbed peripheral fidelity.
