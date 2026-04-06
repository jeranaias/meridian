# Meridian Full Parity Audit — ArduPilot 1.5M Line Port

## Wave 1: Foundation (HAL + Sensors) — COMPLETE

### Critical Porting Risks Identified

1. **Recursive Semaphores** — ArduPilot uses recursive mutexes everywhere (ChibiOS mutex_t). Rust's Mutex is NOT recursive. Must either redesign locking or build counting recursive wrapper with thread ID tracking. HIGHEST RISK ITEM.

2. **DMA Memory Regions (STM32H7)** — DMA cannot access DTCM or SRAM4. Every SPI/UART buffer must be in SRAM1/2/3. Need separate linker sections and pool allocators per memory region. Silent data corruption if wrong.

3. **ICM-42688 AFSR Bug** — MUST disable Anti-Alias Filter Spectral Rejection (`INTF_CONFIG1[6:5] = 0b10`) on ALL IxM42xxx devices. Gyro stalls for 2ms near 100 deg/s if not disabled. Flight-critical.

4. **BMI270 Firmware Upload** — Requires uploading 4KB firmware blob over SPI before sensor functions. Not documented in most datasheets.

5. **IOMCU Failsafe** — IO coprocessor runs its own failsafe mixing. If main CPU crashes, IO MCU continues outputting the last-known failsafe PWM pattern. Must implement this redundancy.

### HAL Interface — What Meridian Needs

| Interface | Methods | Meridian Equivalent |
|-----------|---------|-------------------|
| UARTDriver | begin/end/write/read/available/txspace/set_flow_control | embedded-hal Serial + DMA |
| SPIDevice | transfer/transfer_fullduplex/set_speed/get_semaphore | embedded-hal SPI + DMA |
| I2CDevice | transfer/read_registers/write_register/get_semaphore | embedded-hal I2C |
| GPIO | pinMode/read/write/toggle/attach_interrupt | embedded-hal GPIO |
| RCOutput | set_freq/write/push/set_dshot_rate/send_dshot_command | Timer PWM + DShot DMA |
| RCInput | new_input/num_channels/read/set_override | UART/timer capture |
| Storage | read_block/write_block (16KB virtual EEPROM) | Flash sectors with wear leveling |
| Scheduler | register_timer_process/register_io_process/delay | RTIC tasks + priorities |
| AnalogIn | channel/board_voltage/servorail_voltage | ADC DMA |

### Thread Model (ChibiOS → RTIC mapping)

| ChibiOS Thread | Priority | Rate | RTIC Equivalent |
|---------------|----------|------|-----------------|
| monitor | 183 | 0.1Hz | Software task, lowest |
| timer | 181 | 1kHz | Hardware timer ISR |
| rcout | 181 | 1kHz | DMA completion ISR |
| main | 180 | 400Hz | Main loop task |
| rcin | 177 | varies | UART/timer ISR |
| io | 58 | 100Hz | Software task |
| storage | 59 | async | Flash write task |

### Boot Sequence (ChibiOS → Meridian)
1. Clock init (8MHz xtal → 400MHz PLL)
2. Watchdog enable
3. USB init
4. I2C bus clear (20 SCL pulses)
5. DMA init
6. Serial(0) begin
7. Scheduler init (spawn threads)
8. GPIO → RCIn → RCOut → Board detect → SD card → safety
9. setup() — sensor probe, EKF init
10. loop() — main flight loop

### Sensor Drivers Summary

**IMU (19K lines):**
- ICM-42688: 12-reg init, FIFO 16-byte packets, header 0x68 validation, DMA SPI
- BMI270: 4KB firmware upload, separate accel/gyro FIFO frames
- Vibration: 5Hz + 2Hz LP envelope on raw accel, clipping at 152 m/s²
- Temp cal: 3rd-order polynomial, online learning with double-precision 4th-order fit
- Multi-IMU: angle diff for gyro, vector distance for accel, IMU3 Z-axis 6x relaxed

**Baro (9K lines):**
- BMP280: integer Bosch algorithm, t_fine intermediate, 50Hz
- BMP388/390: float 11-coefficient compensation, SPI dummy byte discard
- Ground cal: 10s settle + 5-sample average

**Compass (14K lines):**
- Calibration: Levenberg-Marquardt sphere+ellipsoid (9 params), 300 samples, fitness < 5 mGauss
- Motor comp: throttle/current × 3D vector, or per-motor with power^0.65 scaling
- Consistency: 3D angle < 90°, XY angle < 60°, XY length diff < 200 mGauss
- Declination: IGRF bilinear interpolation, one-time GPS lookup

**GPS (18K lines):**
- uBlox: 22-step config, 8-baud auto-detect (9600→460800), NAV-PVT preferred
- NMEA: RMC+GGA within 150ms for fix
- Health: delta_time EMA, delayed_count, 4x rate threshold
- Blending: inverse-variance weighting, health counter 10/-1
- RTK: VALSET binary config, RELPOSNED for heading, moving baseline

**Rangefinder (14K lines):** 47 driver types, 5-state status enum
**Optical Flow (3K lines):** 8 drivers, EKF fuses flowRate-bodyRate
**Airspeed (5K lines):** 19 types, 3-state Kalman for in-flight cal
**External AHRS (9K lines):** 7 backends, replaces or supplements EKF

---

## Wave 2: Comms + Vehicles + Motors (169K lines) — COMPLETE

### Critical Findings

1. **GCS MAVLink** — 90 message handlers, 10 stream groups, mission upload state machine with 8s timeout. Signing uses HMAC-SHA256, timestamp +60s on boot. USB always unsigned. Statustext: 30-entry queue, 50-char chunks. 50 serial protocol IDs.

2. **RC Protocols** — SBUS failsafe has BOTH explicit flag (bit 3) AND implicit check (ch1-4 <= 875us). CRSF failsafe is timeout-only (500ms TX, 150ms RX). CRSFv3 baud negotiation via 0x70/0x71 general commands. ELRS bootstrap baud 420000 vs CRSF 416666. FrSky passthrough packs complex data into 32-bit Sport frames with 10+ app IDs.

3. **Vehicle Main Code** — Rate controller runs BEFORE EKF in fast loop (gyro-only, minimum latency). Land detector uses 8 conditions, specifically excludes baro/EKF/rangefinder. RTL has 6 states with terrain/rally/cone-slope path building. QuadPlane transition has Q_ASSIST triggering conditions.

4. **Motors** — Full 12-step mixing pipeline in output_armed_stabilizing(). 38 frame presets (11 Quad + 5 Hex + 7 Octa + 8 OctaQuad + 3 Y6 + 2 DodecaHex + 2 Deca). Thrust linearization: quadratic formula with voltage LP filter. Battery sag compensation: resistance estimator with TC1=0.5s. 190 servo function slots. 200+ RC aux functions. BLHeli passthrough: 4-way protocol with 8-state machine.

### Detailed docs: `docs/audit_wave2_*.md` (4,458 lines total)

---

## Wave 3: Advanced Nav + Payload + Utilities (207K lines) — COMPLETE

### Critical Findings

1. **AutoTune** — Multirotors use "twitch" method (180 deg/s step commands, 5-step sequence, 4 consecutive passes per step, 25% gain backoff). Helis use frequency-sweep chirp with gain/phase extraction. This is complex — needs dedicated implementation.

2. **GyroFFT** — Dedicated IO-priority thread, tracks 3 simultaneous noise peaks with distance-matrix anti-swap. Harmonic matching: peak is harmonic if freq differs from N*fundamental by <10%. Notch center frequency updated every control loop. On CMSIS-DSP ARM FFT underneath.

3. **NotchFilter critical detail** — Has a 5% per-update slew limiter on center frequency. Without this, the filter rings. HarmonicNotchFilter supports up to 16 harmonics with 5 dynamic tracking modes (throttle/RPM/BLHeli/FFT/RPM2).

4. **SCurve trajectory** — 7-phase jerk-limited path (not simple trapezoidal). WPNav uses this for straight segments and Hermite/Catmull-Rom splines for corners. Default jerk 1.0 m/s³.

5. **SmartRTL loop pruning** — Beyond Douglas-Peucker, has loop detection that excises non-adjacent segments within ~2m. Two background algorithms with 200us time budgets each. Thorough cleanup must complete before vehicle starts returning.

6. **Siyi gimbal** — Does NOT accept direct angle commands. FC runs P-controller (P=1.5, max 90 deg/s) to convert angles to -100..+100 rate scalars. 14 mount backend types total.

7. **Precision Landing** — Has its own 2-state Kalman filter per axis, completely separate from main EKF. Inertial history ring buffer compensates for sensor lag.

8. **OpenDroneID** — Pre-arm requires arm_status OK handshake from transponder. 5 message types broadcast over WiFi/BLE/MAVLink relay.

9. **AP_Math sqrt_controller** — Kinematic shapers are the core of all flight mode pos/vel tracking. Not trivial to re-derive — must port carefully.

10. **AP_Param group tree** — 18-bit group element (6 bits × 3 levels = 64 entries per level). Dynamic params from Lua use keys 300+. Flag system allows one param to reference another's value as default.

11. **Logger** — ~150-200 distinct message types across ~20 library LogStructure.h files. Binary sync 0xA3/0x95. Rate limiting via 256-element timestamp array. 1KB reserved for critical messages.

12. **Scripting** — Lua 5.4, 43-200KB heap, ~922 binding entries, 49 shipped applets. CRC32 tracked per script. Environment isolation.

13. **Proximity** — 8 sectors × 5 layers = 40 faces in 3D boundary. 3-sample median + IIR filter per face. 1s expiry.

14. **Terrain** — 32×28 grid blocks, 12-block LRU cache (~22KB RAM), MAVLink request/data protocol, bilinear interpolation.

15. **OSD** — Character-cell based (30×16 PAL), 56+ item types, 4+2 screens, 107 symbols. MAX7456 SPI + MSP DisplayPort backends.

### Detailed docs: `docs/audit_wave3_*.md` (4,114 lines total)

---

## FULL AUDIT COMPLETE — 12,108 lines of documentation across 12 agents

### Summary Statistics
- **Wave 1 (Foundation):** 166K C++ lines audited → 3,427 lines of notes
- **Wave 2 (Comms+Vehicles):** 169K C++ lines audited → 4,458 lines of notes
- **Wave 3 (Nav+Payload+Utilities):** 207K C++ lines audited → 4,114 lines of notes
- **Total:** 542K C++ lines audited, 12,108 lines of surgical notes
- **Source files read:** 250+ across all 154 ArduPilot libraries

### Top 10 Porting Risks (ordered by severity)

1. **Recursive semaphores** — ArduPilot uses them everywhere. Rust Mutex is not recursive. Redesign or wrap.
2. **DMA memory regions (STM32H7)** — Wrong memory region = silent data corruption. Need linker sections.
3. **ICM-42688 AFSR bug** — Disable or gyro stalls at 100 deg/s. Flight-critical.
4. **NotchFilter slew limiter** — Missing 5% slew = filter ringing. Subtle and dangerous.
5. **sqrt_controller kinematic shapers** — Core of all position/velocity control. Must port exactly.
6. **SBUS dual failsafe** — Both explicit flag AND implicit channel check needed. Missing either = false-safe.
7. **H7 SPI v2 peripheral** — Completely different register layout from older STM32. Can't reuse F4/F7 drivers.
8. **BMI270 firmware upload** — 4KB blob required before sensor works. Easy to miss.
9. **Siyi angle→rate conversion** — Gimbal doesn't accept angles. FC must run P-controller.
10. **Parameter group tree 18-bit encoding** — EEPROM-stable format. Must match exactly for config compatibility.
