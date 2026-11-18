# FlightCore Engineering Roadmap

> **Strict Boundary Commitment**: All planned phases expand FlightCore's passive sensing, RF diagnostics, ground-station telemetry, and mechanical integration. FlightCore will **never** incorporate motor/ESC actuation, flight stabilization loops, or flight-control authority.

---

## Phase 1 — Core Modular Firmware & Protocol (Completed)
- [x] PlatformIO C++17 modular drivers (`MPU6050`, `VL53L0X`, `SX1278`, `A3144` Hall ISR, `ADC1` Power/NTC)
- [x] 37-byte CRC-16-CCITT binary telemetry framing (`0xFC 0x01`) + JSON serial diagnostics CLI
- [x] ESP32-CAM UART2 telemetry heartbeat bridge stub (`Serial2` @ 115200 baud)
- [x] Companion Python decoder (`ground_station/receiver_cli.py`) matching firmware binary layout 1:1
- [x] Compiled `esp32dev` ELF/BIN (`RAM: 7.8%`, `Flash: 24.6%`), `pio check` (`cppcheck`), C++17 native unit suite, and pytest suite

## Phase 2 — PCB Manufacturing & BenchTwin Staging (Completed)
- [x] KiCad 8 project (`flightcore.kicad_pro`, `.kicad_sch`, `.kicad_pcb`, `.net`) with 49-part layout
- [x] 2-layer RS-274X Gerber pack (`GTL, GBL, GTS, GBS, GTO, GBO, GM1, DRL`) with zero-violation geometric DRC report
- [x] Verified BOM (`hardware/BOM.csv`), Pin Mapping, Power Architecture, and Assembly Guide
- [x] BenchTwin export (`flightcore.btw`, `flightcore-benchtwin.json`, `schematic_preview.svg`) & closed-manifold 3D-printable tray (`enclosure_mount.stl`)

## Phase 3 — Physical Fabrication & Hardware-in-the-Loop Bring-Up (Next)
- [ ] Order 2-layer `1.6 mm` FR4 carrier PCBs (optionally `2 oz` copper for >40 A propulsion pass-through)
- [ ] Populate SMD/THT components per `hardware/ASSEMBLY-GUIDE.md` and perform current-limited smoke test
- [ ] Capture oscilloscope traces of I2C (`400 kHz`), SPI (`4 MHz`), and `INA180A2` step load response
- [ ] Execute 433 MHz open-field LoRa range test (SF7 vs SF10 sensitivity/RSSI logging via `ground_station`)

## Phase 4 — Future Telemetry Enhancements (Strictly Passive)
- [ ] MicroSD / SPIFFS blackbox circular binary log recorder (`flightcore_YYYYMMDD.bin`) replayable via `ground_station --file`
- [ ] Full ESP32-CAM JPEG snapshot metadata tagging (injecting `FC:TELEM` EXIF/sidecar timestamps)
- [ ] Companion Ground Station live Matplotlib / WebSocket telemetry dashboard consuming `ground_station.protocol.StreamDecoder`
- [ ] BenchTwin live interactive browser session validation & thermal derating profile sweep
