# FlightCore Pin Mapping & Electrical Interface Specification

> **SAFETY CONSTRAINT**: FlightCore is strictly a **sensor acquisition & LoRa telemetry expansion system**. No pins are allocated to ESC PWM, DShot, servo actuation, or motor output control.

---

## 1. Master ESP32 DevKit v1 (30-Pin `U1`) GPIO Assignment

| DevKit Pin | ESP32 GPIO | Hardware Bus / Peripheral | Net Name | External Circuit / Pull-Up / Conditioning | Safety & Boot Notes |
|---|---|---|---|---|---|
| 1 | `EN` | Chip Reset | `NC_EN` | Internal DevKit RC reset | Unconnected on carrier |
| 2 | `VP` (`GPIO36`) | `ADC1_CH0` (Input-only) | `NTC_ADC` | 10 kΩ 1% pull-up `R9` to `+3V3_BUS`; 100 nF `C8` to GND; `J4` to 10k NTC `RT1` | Input-only ADC1 (Wi-Fi safe) |
| 3 | `VN` (`GPIO39`) | Spare | `NC_VN` | — | Reserved input-only |
| 4 | `D34` (`GPIO34`) | `ADC1_CH6` (Input-only) | `VBAT_ADC` | `R7` 100 kΩ / `R8` 10 kΩ (11:1); `C6` 100 nF; `D2` `BAT54S` clamp | Max 25.2 V (6S LiPo) -> 2.29 V ADC |
| 5 | `D35` (`GPIO35`) | `ADC1_CH7` (Input-only) | `IBAT_ADC` | `U5` `INA180A2` (50 V/V) across `R11` 1 mΩ + `R10` 100 Ω / `C7` 100 nF | 50 mV/A sensitivity (0–60 A) |
| 6 | `D32` (`GPIO32`) | Digital Input | `MPU_INT` | Connected to `U2.8` (`MPU6050 INT`) | Push-pull data-ready IRQ |
| 7 | `D33` (`GPIO33`) | External ISR (`FALLING`) | `HALL_SIG` | `R5` 10 kΩ pull-up to `+3V3_BUS`; `R6` 1 kΩ + `C5` 100 nF from `J3.2` (`A3144`) | Protects GPIO from 5 V supply on `A3144` |
| 8 | `D25` (`GPIO25`) | Digital Input | `TOF_INT` | Connected to `U3.5` (`VL53L0X GPIO1`) | Active-LOW data-ready flag |
| 9 | `D26` (`GPIO26`) | Digital Output | `TOF_XSHUT` | `R3` 10 kΩ pull-up to `+3V3_BUS` -> `U3.6` (`XSHUT`) | Software hard-reset on I2C hang |
| 10 | `D27` (`GPIO27`) | Digital Input / ISR | `LORA_DIO0` | Connected to `U4.5` (`SX1278 DIO0`) | Signals `TxDone` / `RxDone` |
| 11 | `D14` (`GPIO14`) | Digital Output | `LORA_RST` | Connected to `U4.4` (`SX1278 RST`, active LOW) | Reset pulse during `begin()` |
| 12 | `D12` (`GPIO12`) | **UNUSED (MTDI Strap)** | `NC_D12` | Left floating intentionally | **Never pull HIGH** (sets flash voltage!) |
| 13 | `D13` (`GPIO13`) | Spare | `NC_D13` | — | Reserved |
| 14 | `GND` | Power Ground | `GND` | Shared ground plane (`B.Cu`) | Star-tied near `U7` buck GND |
| 15 | `VIN` | 5 V System Input | `+5V_BUS` | Fed by `U7` 5 V / 3 A Buck converter | Feeds onboard DevKit LDO |
| 16 | `3V3` | DevKit 3.3 V Output | `ESP32_3V3` | Bridged to `+3V3_BUS` **ONLY** via `JP1` in USB bench mode | Keep `JP1` **OPEN** on battery |
| 17 | `GND` | Power Ground | `GND` | Shared ground plane (`B.Cu`) | — |
| 18 | `D15` (`GPIO15`) | **UNUSED (Strap)** | `NC_D15` | Left floating intentionally | Strapping pin avoided |
| 19 | `D2` (`GPIO2`) | Digital Output | `STATUS_LED` | Drives `D4` LED via `R12` 330 Ω to `GND` | Heartbeat / calibration blink |
| 20 | `D4` (`GPIO4`) | Digital Input (`INPUT_PULLUP`) | `CAL_BTN` | Tactile `SW1` momentary switch to `GND` | Ground-level IMU recalibration trigger |
| 21 | `RX2` (`GPIO16`) | `UART2 RX` | `CAM_UART_RX` | `J6.4` (`ESP32-CAM TX` @ 115200 8N1) | Non-blocking UART2 telemetry stub |
| 22 | `TX2` (`GPIO17`) | `UART2 TX` | `CAM_UART_TX` | `J6.3` (`ESP32-CAM RX` @ 115200 8N1) | Sends `FC:TELEM,...` & `FC:PING` |
| 23 | `D5` (`GPIO5`) | `VSPI CS0` | `LORA_CS` | `R4` 10 kΩ pull-up to `+3V3_BUS` -> `U4.15` (`NSS`) | Pull-up ensures idle HIGH during boot |
| 24 | `D18` (`GPIO18`) | `VSPI SCK` | `VSPI_SCK` | Connected to `U4.12` (`SX1278 SCK`, 4 MHz) | SPI Mode 0, MSB first |
| 25 | `D19` (`GPIO19`) | `VSPI MISO` | `VSPI_MISO` | Connected to `U4.13` (`SX1278 MISO`) | — |
| 26 | `D21` (`GPIO21`) | `I2C SDA` | `I2C_SDA` | `R1` 4.7 kΩ pull-up to `+3V3_BUS` -> `U2.4`, `U3.4` | Shared 400 kHz Fast Mode I2C |
| 27 | `RX0` (`GPIO3`) | `UART0 RX` | `NC_RX0` | Internal CP2102/CH340 USB bridge | Serial CLI RX |
| 28 | `TX0` (`GPIO1`) | `UART0 TX` | `NC_TX0` | Internal CP2102/CH340 USB bridge | Serial CLI TX |
| 29 | `D22` (`GPIO22`) | `I2C SCL` | `I2C_SCL` | `R2` 4.7 kΩ pull-up to `+3V3_BUS` -> `U2.3`, `U3.3` | Shared 400 kHz Fast Mode I2C |
| 30 | `D23` (`GPIO23`) | `VSPI MOSI` | `VSPI_MOSI` | Connected to `U4.14` (`SX1278 MOSI`) | — |

---

## 2. I2C Bus Address Map (`GPIO21` SDA / `GPIO22` SCL @ 400 kHz)

| Device | Designator | 7-Bit I2C Address | Address Selection | Local Decoupling |
|---|---|---|---|---|
| **MPU6050 6-Axis IMU** | `U2` | `0x68` | `U2.7 (AD0)` tied to `GND` | `C9` 100 nF near `U2.1` |
| **VL53L0X ToF Laser** | `U3` | `0x29` | Fixed factory address (isolated via `XSHUT`) | `C10` 100 nF near `U3.1` |

---

## 3. External Harness Pinouts

- **`J1` (XT60-M Battery Input)** & **`J2` (XT60-F PDB Pass-Through)**: `Pin 1 = VBAT+` (`VBAT_IN` / `VBAT_LOAD`), `Pin 2 = GND`
- **`J3` (JST-XH 3-Pin Hall RPM)**: `1 = +5V_BUS`, `2 = HALL_SIG_EXT` (open-collector `A3144`), `3 = GND`
- **`J4` (JST-XH 2-Pin NTC Probe)**: `1 = NTC_ADC` (`10k pull-up R9` to `+3V3_BUS`), `2 = GND`
- **`J5` (Edge SMA 433 MHz)**: Center pin = `LORA_RF_ANT` (0.95 mm 50 Ω track to `U4.17`), outer tabs = `GND`
- **`J6` (JST-XH 4-Pin ESP32-CAM)**: `1 = +5V_BUS`, `2 = GND`, `3 = CAM_UART_TX` (`GPIO17`), `4 = CAM_UART_RX` (`GPIO16`)
