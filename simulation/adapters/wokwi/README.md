# Wokwi FlightCore Adapter

This adapter maps FlightCore's canonical pin definitions (`firmware/include/config.h` and `hardware/PIN-MAPPING.md`) to **Wokwi** simulator parts and custom chip stubs in `wokwi/diagram.json`.

## Pin & Peripheral Mapping

| Canonical Signal | ESP32 GPIO | Hardware Part | Wokwi Part / Stub | Fidelity |
|---|---|---|---|---|
| `I2C_SDA` / `I2C_SCL` / `MPU_INT` | `21` / `22` / `32` | MPU6050 (`U2`) | `wokwi-mpu6050` | **Native Wokwi** |
| `NTC_ADC` | `36` (`VP`) | 10k B3950 NTC (`RT1`) | `wokwi-ntc-temperature-sensor` | **Native Wokwi** |
| `VBAT_ADC` | `34` | 100k/10k Divider (`R7/R8`) | `wokwi-potentiometer` (`pot_vbat`) | **Analog Stand-in** |
| `IBAT_ADC` | `35` | INA180A2 (`U5`) + 1 mΩ (`R11`) | `wokwi-potentiometer` (`pot_ibat`) | **Analog Stand-in** |
| `HALL_SIG` | `33` | A3144 Hall (`U8`) | `wokwi-pushbutton` (`btn_hall`) | **Manual Pulse Stub** |
| `CAL_BTN` | `4` | Tactile Switch (`SW1`) | `wokwi-pushbutton` (`btn_cal`) | **Native Wokwi** |
| `STATUS_LED` | `2` | Status LED (`D4`) | `wokwi-led` (`led_status`) | **Native Wokwi** |
| `TOF_I2C` / `XSHUT` / `INT` | `21/22/26/25` | VL53L0X (`U3`) | `chip-vl53l0x` (`.chip.json`/`.c`) | **Custom Wokwi Chip Stub** |
| `VSPI` / `CS` / `RST` / `DIO0` | `18/19/23/5/14/27` | SX1278 Ra-02 (`U4`) | `chip-sx1278` (`.chip.json`/`.c`) | **Custom Wokwi Chip Stub** |
| `CAM_UART_RX` / `TX` | `16` / `17` | ESP32-CAM (`J6`) | UART2 Serial Stub (unwired / monitor) | **Documented Software Stub** |

See [../../documentation/COMPONENT-SUPPORT-MATRIX.md](../../documentation/COMPONENT-SUPPORT-MATRIX.md) for full details.
