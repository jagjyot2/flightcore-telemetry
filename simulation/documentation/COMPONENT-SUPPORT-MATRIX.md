# FlightCore Component Support Matrix (Wokwi vs. Physical Hardware vs. BenchTwin)

> **Transparency Rule**: Unsupported peripherals are **never silently substituted**. Every non-native peripheral in Wokwi is explicitly backed by either a custom Wokwi chip stub (`.chip.json` + `.chip.c`), a documented analog/digital stand-in, or a deterministic software mock in `tests/` and `simulation/`.

---

## 1. Wokwi Simulation Support Matrix

| Subsystem | Physical Part (`hardware/BOM.csv`) | ESP32 GPIO / Bus | Wokwi Status | Wokwi Representation (`wokwi/diagram.json`) | Verification & Limitations |
|---|---|---|---|---|---|
| **MCU** | DOIT ESP32 DevKit v1 (30-pin `U1`) | All GPIOs | **Natively Supported** | `board-esp32-devkit-c-v4` (`esp32`) | Full Xtensa ELF execution from `firmware/.pio/build/esp32dev/firmware.elf`. |
| **6-Axis IMU** | MPU6050 (`U2`, `I2C 0x68`) | `SDA=21`, `SCL=22`, `INT=32` | **Natively Supported** | `wokwi-mpu6050` (`imu_mpu6050`) | Native I2C `0x68` register emulation; interactive accel/gyro/temp sliders. |
| **Thermal Probe** | 10 kΩ B3950 NTC (`RT1`) + 10k `R9` | `VP` (`GPIO36` / `ADC1_CH0`) | **Natively Supported** | `wokwi-ntc-temperature-sensor` (`ntc_probe`) | Native Steinhart-Hart temperature slider (`-24 °C` to `+80 °C`). |
| **Calibration Button** | Tactile Switch (`SW1`) | `GPIO4` (`INPUT_PULLUP`) | **Natively Supported** | `wokwi-pushbutton` (`btn_cal`) | Triggers runtime `250`-sample IMU zero-bias recalibration. |
| **Status LED** | Green LED (`D4`) + 330 Ω (`R12`) | `GPIO2` | **Natively Supported** | `wokwi-led` (`led_status`) + `wokwi-resistor` | Visualizes boot, calibration, and telemetry TX heartbeat. |
| **Battery Voltage Divider** | `100k/10k` Divider (`R7/R8`) + `BAT54S` | `GPIO34` (`ADC1_CH6`) | **Analog Stand-In** | `wokwi-potentiometer` (`pot_vbat`) | Sweeps `0–3.3 V` ADC1 input (`0–36.3 V` scaled); clamping diode `D2` not SPICE-modeled in Wokwi. |
| **Current Sense Amp** | `INA180A2` (`U5`) + `1 mΩ` (`R11`) | `GPIO35` (`ADC1_CH7`) | **Analog Stand-In** | `wokwi-potentiometer` (`pot_ibat`) | Sweeps `0–3.3 V` (`0–66 A` @ `50 mV/A`); high-side common-mode limit (`26 V`) not modeled in Wokwi. |
| **ToF Laser Ranging** | VL53L0X (`U3`, `I2C 0x29`) | `SDA=21`, `SCL=22`, `XSHUT=26`, `INT=25` | **Unsupported Natively -> Custom Wokwi Chip Stub** | `chip-vl53l0x` (`wokwi/chips/chip-vl53l0x.chip.json`, `.chip.c`) | Responds on I2C `0x29`, honors `XSHUT` reset, returns deterministic ranging profile. Optical laser propagation is not ray-traced. |
| **433 MHz LoRa Radio** | SX1278 Ra-02 (`U4`, `VSPI`) | `SCK=18`, `MISO=19`, `MOSI=23`, `CS=5`, `RST=14`, `DIO0=27` | **Unsupported Natively -> Custom Wokwi Chip Stub** | `chip-sx1278` (`wokwi/chips/chip-sx1278.chip.json`, `.chip.c`) | Implements SX1278 SPI `RegVersion` (`0x12`), mode/FIFO registers, and `DIO0` `TxDone` pulse. RF propagation & antenna impedance (`J5`) are mocked via `ground_station/protocol.py` & `simulation/scenarios/`. |
| **Hall-Effect RPM** | A3144EUA-T (`U8` via `J3`) | `GPIO33` (`FALLING` ISR) | **Unsupported Natively -> Manual Pulse Stub** | `wokwi-pushbutton` (`btn_hall`) | Manual edge pulses test ISR & debounce; high-RPM (`40,000 RPM`) timing is verified in `tests/test_sensor_math.py` and `firmware/test/test_flightcore_native.cpp`. |
| **ESP32-CAM Bridge** | ESP32-CAM (`J6`, `UART2`) | `RX2=16`, `TX2=17` | **Unsupported Natively -> Software Stub** | Unwired `UART2` (`FC:TELEM` / `FC:PING` protocol) | Verified via firmware UART2 non-blocking state machine; no live MJPEG camera simulation. |

---

## 2. Preserved BenchTwin Support (`benchtwin/`)

BenchTwin remains preserved in `benchtwin/` and indexed by `simulation/adapters/benchtwin/benchtwin_adapter_manifest.json` for electrical netlist/BOM/RC/3D-STL inspection. See `benchtwin/BENCHTWIN-SIMULATION-NOTES.md` for the BenchTwin fidelity matrix.
