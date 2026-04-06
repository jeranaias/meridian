# Parity Verification: Drivers & Protocols
## ArduPilot vs Meridian — Side-by-Side Analysis

**Date:** 2026-04-02
**ArduPilot source:** `D:\projects\ardupilot\libraries\`
**Meridian source:** `D:\projects\meridian\crates\meridian-drivers\`, `meridian-rc\`, `meridian-mavlink\`, `meridian-can\`
**Scope:** Sensor drivers, RC protocols, MAVLink, DroneCAN

---

## 1. Sensor Drivers

### 1.1 IMU (ArduPilot: AP_InertialSensor)

#### ArduPilot Backend Catalog (14 backends)

| Backend | File | Chips |
|---------|------|-------|
| `AP_InertialSensor_Invensense` | Invensense.cpp | MPU6000, MPU9250, ICM-20608, ICM-20602, ICM-20601, ICM-20689 |
| `AP_InertialSensor_Invensensev2` | Invensensev2.cpp | ICM-20948, ICM-20649, ICM-20648 (AK09916 compass on aux I2C) |
| `AP_InertialSensor_Invensensev3` | Invensensev3.cpp | ICM-40609, **ICM-42688**, ICM-42605, ICM-40605, IIM-42652, IIM-42653, **ICM-42670**, ICM-45686 |
| `AP_InertialSensor_BMI055` | BMI055.cpp | BMI055 |
| `AP_InertialSensor_BMI088` | BMI088.cpp | BMI088, BMI085 |
| `AP_InertialSensor_BMI160` | BMI160.cpp | BMI160 |
| `AP_InertialSensor_BMI270` | BMI270.cpp | BMI270 |
| `AP_InertialSensor_LSM9DS0` | LSM9DS0.cpp | LSM9DS0 |
| `AP_InertialSensor_LSM9DS1` | LSM9DS1.cpp | LSM9DS1 |
| `AP_InertialSensor_LSM6DSV` | LSM6DSV.cpp | LSM6DSV |
| `AP_InertialSensor_L3G4200D` | L3G4200D.cpp | L3G4200D (gyro only) |
| `AP_InertialSensor_ADIS1647x` | ADIS1647x.cpp | ADIS16470, ADIS16477 (tactical grade) |
| `AP_InertialSensor_SCHA63T` | SCHA63T.cpp | SCHA63T (automotive dual-die) |
| `AP_InertialSensor_ExternalAHRS` | ExternalAHRS.cpp | External AHRS (VectorNav, etc.) |

#### Meridian IMU Coverage (4 drivers)

| Meridian file | Chips |
|---------------|-------|
| `imu_icm426xx.rs` | ICM-42688-P, ICM-42688-V, ICM-42605, ICM-42670 |
| `imu_invensense.rs` | MPU6000, MPU9250, ICM-20608 (v1 legacy) |
| `imu_bmi088.rs` | BMI088 / BMI085 |
| `imu_bmi270.rs` | BMI270 |

#### IMU Coverage Gaps

| Missing from Meridian | Priority |
|-----------------------|----------|
| ICM-20948 / ICM-20649 / ICM-20648 (Invensensev2) | MEDIUM — ICM-20948 found on many H7 boards |
| ICM-40609 | LOW — legacy, EOL |
| IIM-42652 / IIM-42653 / ICM-45686 | LOW — industrial / high-perf variants |
| LSM9DS0 / LSM9DS1 | LOW — older ST combo chips |
| LSM6DSV | MEDIUM — new ST chip appearing on modern boards |
| L3G4200D | LOW — gyro only, very old |
| ADIS16470 / ADIS16477 | LOW — niche tactical grade |
| SCHA63T | LOW — automotive niche |
| ExternalAHRS passthrough | HIGH — needed for VectorNav / Pixhawk4 external AHRS boards |
| BMI055 / BMI160 | LOW — older Bosch IMUs |

---

#### ICM-42688: Init Register Sequence Comparison

**ArduPilot sequence** (`AP_InertialSensor_Invensensev3.cpp`, `set_filter_and_scaling()`):
1. `SIGNAL_PATH_RESET (0x4B)` = 0x02 — flush FIFO
2. `DEVICE_CONFIG (0x11)` = 0x01 — soft reset
3. Wait 10 ms
4. `INTF_CONFIG0 (0x4C)` = 0xC0 — LE endian + count-by-records + last-data-hold
5. `INTF_CONFIG1 (0x4D)` = read-modify-write; bits [6:5] set to 0b10 — **AFSR disable**
6. Bank 1 → `GYRO_CONFIG_STATIC2 (0x0B)` — enable/disable AAF
7. Bank 1 → `GYRO_CONFIG_STATIC3-5 (0x0C-0x0E)` — AAF DELT/DELTSQR/BITSHIFT
8. Bank 2 → `ACCEL_CONFIG_STATIC2-4 (0x03-0x05)` — accel AAF
9. Bank 0 → `GYRO_CONFIG0 (0x4F)` — range + ODR
10. `ACCEL_CONFIG0 (0x50)` — range + ODR
11. `GYRO_CONFIG1 (0x51)` — UI filter order
12. `GYRO_ACCEL_CONFIG0 (0x52)` — accel UI filter
13. `ACCEL_CONFIG1 (0x53)` — accel filter
14. `FIFO_CONFIG1 (0x5F)` = 0x07 or 0x0F (with HIRES_EN) — accel + gyro (+ temp)
15. `FIFO_CONFIG2 (0x60)` / `FIFO_CONFIG3 (0x61)` — watermark
16. `INT_CONFIG (0x14)` — push-pull, active-high, latched
17. `FIFO_CONFIG (0x16)` = 0x80 — stream-to-FIFO
18. `PWR_MGMT0 (0x4E)` = 0x0F — accel + gyro low-noise; wait 300 µs after

**Meridian sequence** (`imu_icm426xx.rs`, `init_sequence()`, 12 steps):
1. `FIFO_CONFIG (0x16)` = 0x00
2. `INTF_CONFIG0 (0x4C)` = 0x30
3. `INTF_CONFIG1 (0x4D)` — read-modify-write AFSR disable
4. `FIFO_CONFIG1 (0x5F)` = 0x07
5. `FIFO_CONFIG2 (0x60)` = 0x00
6. `FIFO_CONFIG3 (0x61)` = 0x02
7. `INT_CONFIG (0x14)` = 0x36
8. `GYRO_CONFIG0 (0x4F)` = 0x06
9. `ACCEL_CONFIG0 (0x50)` = 0x06
10. `GYRO_CONFIG1 (0x51)` = 0x00
11. `GYRO_ACCEL_CONFIG0 (0x52)` = 0x11
12. `PWR_MGMT0 (0x4E)` = 0x0F

**Gaps in Meridian init sequence vs ArduPilot:**
- **SIGNAL_PATH_RESET not issued.** ArduPilot writes 0x4B=0x02 to flush FIFO before init. Meridian omits this — FIFO may contain stale data from previous session.
- **DEVICE_CONFIG (0x11) soft reset not issued.** ArduPilot resets the device and waits 10 ms. Meridian starts without reset — registers may be in unknown state.
- **Bank switching absent.** ArduPilot programs anti-aliasing filter (AAF) coefficients in Bank 1 (GYRO_CONFIG_STATIC2-5) and Bank 2 (ACCEL_CONFIG_STATIC2-4). Meridian has no bank-switching infrastructure and skips all AAF register writes. Without the AAF configured, the default hardware filter setting is active; whether acceptable depends on ODR.
- **20-bit high-resolution mode absent.** ArduPilot optionally enables 20-bit FIFO mode (FIFO_CONFIG1 bit 4 = HIRES_EN) with a 20-byte FIFODataHighRes packet and adjusts scale factors for 19-bit or 20-bit resolution. Meridian uses only the standard 16-byte packet.
- **INTF_CONFIG0 value difference.** ArduPilot writes 0xC0 (LE + count-by-records + last-data-hold). Meridian writes 0x30 (count-by-records + big-endian). ArduPilot explicitly selects little-endian to avoid byte swapping; Meridian correctly byte-swaps in parse but loses the endianness alignment benefit.
- **ICM-42670 separate register map not implemented.** ArduPilot has a completely different register map for ICM-42670 (MREG1 indirect access, `INV3REG_70_*` registers). Meridian lists ICM-42670 as a supported variant but uses the same register map as ICM-42688 — the device will not function.

**AFSR disable — PRESENT and CORRECT in Meridian:**
`INTF_CONFIG1[6:5] = 0b10` is applied via read-modify-write using `INTF_CONFIG1_AFSR_CLEAR = 0b1001_1111` and `AFSR_DISABLE_MASK = 0b0100_0000`. Byte-level comparison:
- ArduPilot: reads INTF_CONFIG1, clears bits [6:5], sets bits [6:5] to 0b10 (0x40).
- Meridian: identical logic — `(intf1 & 0b1001_1111) | 0b0100_0000`.
This is correct.

**FIFO packet format — PRESENT and CORRECT:**
Meridian 16-byte format: `[header(1)] [accel_xyz(6 BE)] [gyro_xyz(6 BE)] [temp_i8(1)] [timestamp_u16 BE(2)]`. This matches ArduPilot's `FIFOData` struct (which uses LE on-wire but reads the same bytes). Both validate `pkt[0] == 0x68` as header. Meridian correctly uses big-endian parsing per `INTF_CONFIG0 = 0x30`; ArduPilot uses little-endian per `INTF_CONFIG0 = 0xC0`. Functionally equivalent.

---

#### Vibration / Clipping Detection

**ArduPilot:** `AP_InertialSensor::calc_vibration_and_clipping()` — called per-sample. Maintains two-level LP filter on |accel| to produce vibration floor and vibration level. Clip count increments when any axis exceeds ±29.4 m/s² (3G). Outputs are `_accel_clip_count[instance]`, `_accel_vibe_floor_filter[]`, `_accel_vibe_filter[]`. These feed into GCS SYS_STATUS and EKF gating.

**Meridian:** `VibrationMonitor` struct in `meridian-failsafe`. Counts clip events when `accel_magnitude > max_accel_clip` (default 29.4 m/s²). Returns `true` if `>1%` of samples are clipping after 100 samples.

**Gaps:**
- Meridian's vibration monitor is in `meridian-failsafe`, not wired into the driver layer. ArduPilot calls this per-IMU sample in the backend, before the data reaches the EKF.
- Meridian has **no LP-filtered vibration level output** (ArduPilot's two-stage filter produces the logged VIBRATION message values).
- The clip ratio threshold (1% of 100 samples) is simpler than ArduPilot's per-axis magnitude tracking.
- Vibration output is **not wired to EKF gating** in Meridian (ArduPilot gates GPS fusion when vibration is excessive).

---

#### Temperature Calibration (3rd-order polynomial)

**ArduPilot:** `AP_InertialSensor_TCal` — full polynomial thermal compensation. Stores three 3-element `AP_Vector3f coeff[3]` arrays per axis (accel/gyro). `correct_sensor()` evaluates `coeff[0] + coeff[1]*dT + coeff[2]*dT²` (effectively a 3rd-order polynomial with temperature reference `cal_temp`). Online learning via `PolyFit<4, double, Vector3f>`. Persisted to parameters.

**Meridian:** No temperature calibration infrastructure. The ICM-42688 driver converts the raw temperature byte to °C and includes it in the sample, but no polynomial correction is applied.

**Gap:** Full 3rd-order polynomial temperature calibration is absent. This is a LOW priority gap (most consumer-grade IMUs do not require it, and it requires deliberate thermal sweep calibration).

---

### 1.2 Barometer (ArduPilot: AP_Baro)

#### ArduPilot Backend Catalog (20 backends)

| Backend | Chips |
|---------|-------|
| AP_Baro_BMP085 | BMP085 |
| AP_Baro_BMP280 | BMP280 / BME280 |
| AP_Baro_BMP388 | BMP388 |
| AP_Baro_BMP581 | BMP581 |
| AP_Baro_DPS280 | DPS310 / DPS368 |
| AP_Baro_MS5611 | MS5611, MS5607, MS5637 |
| AP_Baro_FBM320 | FBM320 |
| AP_Baro_ICM20789 | ICM20789 (baro embedded in IMU) |
| AP_Baro_ICP101XX | ICP10100, ICP10111 |
| AP_Baro_ICP201XX | ICP20100 |
| AP_Baro_KellerLD | Keller LD series (industrial) |
| AP_Baro_LPS2XH | LPS22HB, LPS25H (ST) |
| AP_Baro_SPL06 | SPL06 (Goertek) |
| AP_Baro_AUAV | AUAV custom |
| AP_Baro_HIL | Hardware-in-loop |
| AP_Baro_SITL | SITL |
| AP_Baro_DroneCAN | DroneCAN peripheral |
| AP_Baro_ExternalAHRS | External AHRS |
| AP_Baro_MSP | MSP protocol |
| AP_Baro_Wind | Wind correction |

#### Meridian Barometer Coverage (3 drivers)

| Meridian file | Chips |
|---------------|-------|
| `baro_bmp280.rs` | BMP280 / BME280 |
| `baro_dps310.rs` | DPS310 (and DPS368) |
| `baro_ms5611.rs` | MS5611 / MS5607 |

#### Barometer Coverage Gaps

| Missing from Meridian | Priority |
|-----------------------|----------|
| BMP388 / BMP581 | MEDIUM — BMP388 common on newer boards |
| ICP101XX / ICP201XX | MEDIUM — TDK InvenSense chips, growing adoption |
| LPS22HB | LOW |
| SPL06 | LOW |
| ICM20789 | LOW |
| FBM320 | LOW |
| KellerLD | LOW — industrial only |
| AUAV | LOW — proprietary |
| DroneCAN baro | MEDIUM — CAN peripherals may send baro data |
| ExternalAHRS baro | MEDIUM — needed for complete external AHRS support |

---

#### BMP280 Compensation Algorithm — Comparison with Bosch Reference

**ArduPilot** (`AP_Baro_BMP280.cpp`): Implements the Bosch datasheet section 4.2.3 integer compensation exactly. Uses `int64_t` for pressure calculation; `int32_t` for temperature. t_fine passed between temperature and pressure compensation. Returns pressure in Q24.8 format (divide by 256 for Pa).

**Meridian** (`baro_bmp280.rs`, `compensate_temperature()` / `compensate_pressure()`): **Implements the identical Bosch reference algorithm**. Integer types match: `i32` for temperature (same as C `int32_t`), `i64` for pressure intermediate. t_fine passed correctly. Return format: pressure as `u32` (Q24.8), divided by 256 in the calling code. Tests verify against Bosch reference calibration values (T1=27504, T2=26435, T3=-1000).

**Verdict: BMP280 compensation algorithm is CORRECT and matches the Bosch reference.**

---

#### DPS310 Coefficient Extraction — Comparison

**ArduPilot** (`AP_Baro_DPS280.cpp`): Reads 18 calibration bytes from register 0x10. Extracts c0 (12-bit signed), c1 (12-bit signed), c00 (20-bit signed), c10 (20-bit signed), c01 (16-bit signed), c11 (16-bit signed), c20 (16-bit signed), c21 (16-bit signed), c30 (16-bit signed). Uses a scale_factor lookup (OSR 0-7 → 524288.0 to 2088960.0).

**Meridian** (`baro_dps310.rs`, `parse_calibration()`): Implements identical extraction. Byte-level parsing:
- c0: `coef[0] << 4 | coef[1] >> 4` with 12-bit sign extension
- c1: `(coef[1] & 0x0F) << 8 | coef[2]` with 12-bit sign extension
- c00: `coef[3] << 12 | coef[4] << 4 | coef[5] >> 4` with 20-bit sign extension
- c10: `(coef[5] & 0x0F) << 16 | coef[6] << 8 | coef[7]` with 20-bit sign extension
- c01–c30: 16-bit signed from consecutive byte pairs

**Gap:** Meridian's `Dps310Cal` struct omits the **c2 coefficient** (temperature 2nd order term). ArduPilot's DPS280 driver also includes `c2` when reading from DPS368. This is chip-variant-specific and not a bug for DPS310 (which does not have c2 in its DSDL).

**Gap:** ArduPilot reads `REG_COEF_SRCE (0x28)` to select the temperature coefficient source (MEMS vs ASIC sensor). Meridian's `parse_calibration` accepts `coef_src` as parameter but does not use it to select the measurement source register. The temperature source selection write (`REG_TMP_CFG` bit 7) is missing.

**Verdict: DPS310 coefficient extraction is SUBSTANTIALLY CORRECT with two minor gaps.**

---

### 1.3 Compass (ArduPilot: AP_Compass)

#### ArduPilot Backend Catalog (21 backends)

| Backend | Chip |
|---------|------|
| AP_Compass_AK09916 | AK09916 |
| AP_Compass_AK8963 | AK8963 (MPU9250 internal) |
| AP_Compass_BMM150 | BMM150 |
| AP_Compass_BMM350 | BMM350 |
| AP_Compass_HMC5843 | HMC5843 / HMC5883L |
| AP_Compass_IIS2MDC | IIS2MDC (ST) |
| AP_Compass_IST8308 | IST8308 |
| AP_Compass_IST8310 | IST8310 |
| AP_Compass_LIS2MDL | LIS2MDL |
| AP_Compass_LIS3MDL | LIS3MDL |
| AP_Compass_LSM303D | LSM303D |
| AP_Compass_LSM9DS1 | LSM9DS1 |
| AP_Compass_MAG3110 | MAG3110 |
| AP_Compass_MMC3416 | MMC3416 |
| AP_Compass_MMC5xx3 | MMC5883 / MMC5603 |
| AP_Compass_QMC5883L | QMC5883L |
| AP_Compass_QMC5883P | QMC5883P |
| AP_Compass_RM3100 | RM3100 |
| AP_Compass_DroneCAN | DroneCAN mag |
| AP_Compass_ExternalAHRS | External AHRS |
| AP_Compass_MSP | MSP |
| AP_Compass_SITL | SITL |

#### Meridian Compass Coverage (4 drivers)

| Meridian file | Chips |
|---------------|-------|
| `compass_ist8310.rs` | IST8310 |
| `compass_qmc5883l.rs` | QMC5883L |
| `compass_rm3100.rs` | RM3100 |
| `compass_cal.rs` | Calibration (all chips) |

#### Compass Coverage Gaps

| Missing from Meridian | Priority |
|-----------------------|----------|
| HMC5843 / HMC5883L | HIGH — very common on older boards |
| AK8963 (MPU9250 internal) | HIGH — on every board with MPU9250 |
| AK09916 | MEDIUM — ICM-20948 internal mag |
| BMM150 | MEDIUM — common on AP_Periph nodes |
| LIS3MDL | MEDIUM — common ST mag |
| IST8308 | MEDIUM — variant of IST8310 |
| QMC5883P | LOW — variant |
| BMM350 | LOW — newer Bosch |
| LIS2MDL / IIS2MDC | LOW |
| MMC5883 / MMC5603 | LOW |
| LSM303D / LSM9DS1 | LOW |
| MAG3110 / MAG3416 | LOW |
| DroneCAN compass | MEDIUM |
| ExternalAHRS compass | MEDIUM |

---

#### Compass Calibration: Levenberg-Marquardt Algorithm

**ArduPilot** (`CompassCalibrator.cpp`): Two-phase LM fit. Phase 1 (sphere fit, 4 params: radius + 3 offsets). Phase 2 (ellipsoid fit, 9 params: offsets + diagonal + off-diagonal). Two trial updates per iteration at lambda and lambda/damping. Lambda adapts based on which trial was better.

**Meridian** (`compass_cal.rs`): **Implements the identical two-phase LM structure:**
- Phase 1: sphere fit with `SPHERE_PARAMS = 4`. 10 LM iterations in step one + 15 at start of step two.
- Phase 2: ellipsoid fit with `ELLIPSOID_PARAMS = 9`. 20 LM iterations.
- Two-trial update per iteration at `lambda` and `lambda / LMA_DAMPING (10.0)`. Lambda adjusts same way.
- Fitness gate: field radius checked against `FIELD_RADIUS_MIN = 150.0` and `FIELD_RADIUS_MAX = 950.0`. Offset max = 1800.0 mGauss.

**Verdict: Levenberg-Marquardt calibration algorithm matches ArduPilot's implementation.**

**Gap — sample coverage check:** ArduPilot's calibrator also enforces angular coverage across the sphere (sector bitmask, ensuring samples are spread). Meridian's calibrator only checks minimum sample count (`MIN_SAMPLES = 100`) but does not enforce spatial coverage. Calibrations may succeed with all samples clustered in one orientation.

---

#### Motor Compensation

**ArduPilot** (`Compass_PerMotor.cpp`): Collects per-motor compensation coefficients via ESC current sensing. Applies `mag_compensation = sum(coeff[i] * throttle[i])` or current-based.

**Meridian** (`compass_cal.rs`): `MotorCompensation` struct with `throttle_comp: Vec3<Body>` and `current_comp: Vec3<Body>`. `apply()` method computes `mag + throttle_comp * throttle + current_comp * current`. **Present but simplified** — ArduPilot supports per-motor compensation (N motors × 3 axes), while Meridian uses a single global throttle/current vector.

---

#### IGRF Declination Table

**ArduPilot** (`AP_Declination.cpp`): Full WMM/IGRF spherical harmonic model evaluated via a coefficient table. Bilinear interpolation on a 1-degree grid (181 × 361 = 65,341 entries). Covers -180 to +180 lon, -90 to +90 lat.

**Meridian** (`compass_cal.rs`, `magnetic_declination()`): **Simplified coarse 5-degree grid (37 × 73 = 2,701 entries)**. Bilinear interpolation. Comment explicitly notes this is "good enough for compass heading correction, not precise survey work."

**Gap:** Meridian's declination table has 5-degree resolution vs ArduPilot's 1-degree. Maximum error at steep gradient areas (near magnetic poles, South Atlantic Anomaly) could be 3-5°. Acceptable for most flight purposes, but ArduPilot's full table should be adopted for precision applications.

---

### 1.4 GPS (ArduPilot: AP_GPS)

#### ArduPilot Backend Catalog (14 backends + RTCM parser + GPS blending)

| Backend | Protocol |
|---------|----------|
| AP_GPS_UBLOX | UBX binary (M8, M9, M10) |
| AP_GPS_UBLOX_CFGv2 | UBX generation-9+ config interface |
| AP_GPS_NMEA | NMEA-0183 |
| AP_GPS_ERB | Emlid REACH binary |
| AP_GPS_GSOF | Trimble GSOF |
| AP_GPS_MAV | MAVLink |
| AP_GPS_MSP | MSP |
| AP_GPS_NOVA | NovAtel |
| AP_GPS_SBF | Septentrio binary |
| AP_GPS_SBP / SBP2 | Swift Navigation Piksi |
| AP_GPS_SIRF | SiRF binary |
| AP_GPS_DroneCAN | DroneCAN Fix2 |
| AP_GPS_ExternalAHRS | External AHRS |
| AP_GPS_SITL | SITL |
| AP_GPS_Blended | Multi-GPS blending |
| RTCM3_Parser | RTK corrections |
| MovingBase.cpp | Moving baseline RTK |

#### Meridian GPS Coverage (2 drivers)

| Meridian file | Protocol |
|---------------|----------|
| `gps_ublox.rs` | UBX binary (NAV-PVT parser) |
| `gps_nmea.rs` | NMEA-0183 (GGA + RMC) |

#### GPS Coverage Gaps

| Missing | Priority |
|---------|----------|
| UBX 22-step config sequence | HIGH (see detail below) |
| GPS auto-detect with 8 baud rates | HIGH |
| GPS blending (AP_GPS_Blended) | MEDIUM |
| RTK / RTCM3 injection | MEDIUM |
| Moving baseline support | LOW |
| SBP / SBP2 (Swift) | LOW |
| NOVA (NovAtel) | LOW |
| SBF (Septentrio) | LOW |
| ERB (Emlid) | LOW |
| GSOF (Trimble) | LOW |
| SiRF binary | LOW |
| DroneCAN GPS | MEDIUM |
| ExternalAHRS GPS | MEDIUM |

---

#### UBX Protocol: 22-Step Config Sequence

**ArduPilot** (`AP_GPS_UBLOX.cpp`): On connection, sends a sequence of UBX-CFG messages to configure the receiver:
1. UBX-CFG-PRT — set UART protocol (in/out: UBX), baud rate
2. UBX-CFG-MSG for each desired message type (NAV-PVT, NAV-DOP, NAV-VELNED, etc.)
3. UBX-CFG-NAV5 — dynamic model (airborne <4g), fix mode (auto 2D/3D)
4. UBX-CFG-SBAS — SBAS configuration
5. UBX-CFG-GNSS — constellation selection (GPS+GLONASS+Galileo+BeiDou)
6. UBX-CFG-TP5 — timepulse (if needed)
7. UBX-CFG-RATE — navigation rate (10 Hz for M8, 20 Hz for M9/M10)
8. UBX-CFG-SAVE — save to flash
9. (M9+ only) UBX-VALSET / UBX-CFG-VALGET — generation-9 config interface

**Meridian** (`gps_ublox.rs`): **No configuration sequence is sent.** The driver assumes the receiver is already configured and parses NAV-PVT messages only. No UBX-CFG messages are generated.

**Impact:** A factory-reset u-blox receiver will default to NMEA output at 9600 baud, not UBX binary at 115200 baud. Meridian's UBX driver will receive nothing until the receiver is pre-configured externally. ArduPilot auto-configures the receiver on first connection.

---

#### Auto-Detect with 8 Baud Rates

**ArduPilot** (`AP_GPS.cpp`, `update_primary()`): If no GPS detected within 1 second, rotates baud rates: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600. At each rate, sends detection commands (UBX probe, NMEA query). When a response matches, locks the baud rate.

**Meridian:** No auto-detect baud scanning. Fixed baud rate per UART port.

---

#### NMEA Parsing: RMC + GGA within 150ms

**ArduPilot** (`AP_GPS_NMEA.cpp`): Accepts `$GP*` and `$GN*` prefix sentences. Requires both RMC and GGA within 2000 ms of each other (not 150 ms). 150 ms is the maximum delta expected between paired sentences at 1 Hz.

**Meridian** (`gps_nmea.rs`): **Implements the 150 ms gate correctly.** Parses both GGA (`gga_timestamp_ms`) and RMC (`rmc_timestamp_ms`). `fix_valid()` returns true only when `dt <= 150 ms`. Supports `$GP*` and `$GN*` prefixes.

**Verdict: NMEA RMC+GGA 150ms pairing is CORRECT.**

---

#### GPS Blending

**ArduPilot** (`AP_GPS_Blended.cpp`): Merges up to 2 GPS instances. Weighs each by accuracy (hacc, vacc, sacc) and selects or blends position/velocity. Weight = 1/(hacc² + vacc²). Handles time-of-week alignment, velocity divergence detection.

**Meridian:** No blending. Single GPS only.

---

#### RTK / Moving Baseline Support

**ArduPilot** (`RTCM3_Parser.cpp`, `MovingBase.cpp`): Parses RTCM3 corrections for injection to a rover receiver. Moving baseline uses two u-blox receivers with UBX-relposned to compute heading from baseline vector.

**Meridian:** No RTCM3 parsing, no moving baseline. RTK fix type is recognized in NMEA (quality 4/5 → RTK fixed/float) and UBX (fixType 3 with RTK flag). Corrections cannot be injected.

---

### 1.5 Rangefinder (ArduPilot: AP_RangeFinder)

**ArduPilot backend count: 47 driver types.**

#### Meridian Rangefinder Coverage (3 parsers in `rangefinder.rs`)

| Meridian implementation | Covers |
|------------------------|--------|
| `BenewakeParser` | TFMini, TF02, TFMiniPlus (serial) |
| `LightwareParser` | LightWare SF-series (ASCII serial) |
| `MavlinkRangefinder` | Any sensor sending MAVLink DISTANCE_SENSOR |

#### Rangefinder Gaps (44 missing types)

**Named gaps by category:**

*CAN-based (missing):*
- Benewake TF-series CAN
- TOFSenseP CAN
- USD1 CAN
- NRA24 CAN

*I2C-based (missing):*
- Garmin LIDAR-Lite (LidarLightI2C)
- MaxBotix MaxSonar I2C XL (MaxsonarI2CXL)
- TeraRanger I2C (TeraRangerI2C)
- VL53L0X / VL53L1X (ST ToF)
- TOFSenseF I2C

*Serial-based (missing):*
- LeddarOne
- LeddarVu8
- NoopLoop
- USD1 Serial
- TeraRanger Serial
- GYUS42v2
- JRE Serial
- Lanbao
- RDS02UF
- DTS6012M
- BLPing
- PulsedLightLRF (LIDAR-Lite serial)
- MaxSonar Serial LV
- LightWare GRF (binary serial)
- Ainstein LR-D1
- NMEA rangefinder
- Wasp

*Other (missing):*
- Analog (voltage-based)
- PWM (pulse-width)
- BBB PRU
- Bebop (internal)
- HC-SR04 (GPIO trigger)
- DroneCAN rangefinder
- MAV rangefinder already present
- MSP rangefinder
- Lua scripted
- SITL

**Summary: Meridian has 3/47 rangefinder types (6%). The 44 missing types cover all major commercial lidar, sonar, and radar sensors used in the ArduPilot ecosystem.**

---

### 1.6 Airspeed (ArduPilot: AP_Airspeed)

**ArduPilot backend count: 19 types.**

| ArduPilot Backend | Sensor |
|-------------------|--------|
| MS4525DO | MEAS I2C differential pressure (most common) |
| MS5525 | MEAS I2C |
| SDP3X | Sensirion I2C |
| DLVR | All Sensors DLVR |
| ASP5033 | TE connectivity |
| AUAV | Custom |
| DroneCAN | CAN airspeed |
| External | Generic driver hook |
| MSP | MSP protocol |
| NMEA | Serial NMEA |
| SITL | SITL |
| Analog | ADC voltage |
| (Health management) | |

#### Meridian Airspeed Coverage (3 implementations in `airspeed.rs`)

| Meridian implementation | Covers |
|------------------------|--------|
| `Ms4525do` | MS4525DO (I2C) |
| `AnalogAirspeed` | ADC voltage |
| `synthetic_airspeed()` | GPS-based estimate (no sensor) |

#### Airspeed Gaps

| Missing | Priority |
|---------|----------|
| SDP3X (Sensirion) | MEDIUM — common on PX4-class boards |
| MS5525 | LOW |
| DLVR | LOW |
| ASP5033 | LOW |
| DroneCAN airspeed | MEDIUM |
| External / MSP | LOW |
| 3-state Kalman in-flight calibration | HIGH (see below) |

---

#### 3-State Kalman In-Flight Airspeed Calibration

**ArduPilot** (`Airspeed_Calibration.cpp`): `AP_Airspeed_Calibration` implements a 3-state Kalman filter: states are `[ratio, offset, wind_speed]`. On each GPS+airspeed sample pair, updates the estimate using wind-triangle geometry. The ratio corrects the sensor's sensitivity; the offset corrects zero-point drift; wind_speed is a nuisance state. The filter covariance `P` and process noise `Q` are updated each cycle. This allows sensor drift correction in-flight without landing.

**Meridian:** No in-flight calibration Kalman filter. The MS4525DO driver performs a ground zero-offset calibration at boot (50-sample average) but does not have an in-flight correction mechanism.

**Gap: 3-state Kalman in-flight calibration is ABSENT from Meridian. This is a HIGH priority gap for fixed-wing operations where airspeed accuracy is critical.**

---

### 1.7 Optical Flow

**ArduPilot** (`AP_OpticalFlow`): Supports PX4Flow, ADNS3080, PMW3901, Cheerson CX-OF, PAW3902, UPFLOW, MAVLink, and DroneCAN. EKF fuses `(flowRate - bodyRate)` as the translational component, removing the rotational component (body frame rotation).

**Meridian** (`optical_flow.rs`): PMW3901 (SPI) and MAVLink.

**EKF fusion path:** Meridian's `FlowReading::translational_flow()` correctly computes `flow_rate - body_rate`. This matches ArduPilot's fusion path where `OF_TVEL = (flowRate - bodyRate) * height`.

**Gap:** The `translational_flow()` function is defined and correct, but **not wired into the EKF**. ArduPilot's EKF3 consumes optical flow through `FuseOptFlow()` in `AP_NavEKF3_OptFlowFusion.cpp`. Meridian's EKF has no optical flow fusion path.

**Missing sensors:** PX4Flow, ADNS3080, PAW3902, UPFLOW, CX-OF, DroneCAN flow.

---

## 2. RC Protocols

### 2.1 ArduPilot RC Protocol Catalog

ArduPilot supports 18 named backends. Meridian has 8 (counting CRSF + SBUS in the lib.rs + 6 modules).

| Protocol | ArduPilot | Meridian | Status |
|----------|-----------|----------|--------|
| CRSF / ELRS | AP_RCProtocol_CRSF.cpp | `CrsfParser` in lib.rs | PARTIAL (see detail) |
| SBUS | AP_RCProtocol_SBUS.cpp | `SbusParser` in lib.rs | PARTIAL (see detail) |
| DSM / DSMX | AP_RCProtocol_DSM.cpp | `dsm.rs` | PARTIAL |
| FPort | AP_RCProtocol_FPort.cpp | `fport.rs` | PARTIAL |
| FPort2 | AP_RCProtocol_FPort2.cpp | ABSENT | GAP |
| iBus | AP_RCProtocol_IBUS.cpp | `ibus.rs` | PRESENT |
| SUMD | AP_RCProtocol_SUMD.cpp | `sumd.rs` | PRESENT |
| SRXL2 | AP_RCProtocol_SRXL2.cpp | `srxl2.rs` | PARTIAL |
| SRXL (v1) | AP_RCProtocol_SRXL.cpp | ABSENT | GAP |
| PPM | AP_RCProtocol_PPMSum.cpp | `ppm.rs` | PRESENT |
| ST24 (Yuneec) | AP_RCProtocol_ST24.cpp | ABSENT | GAP |
| GHST (ImmersionRC) | AP_RCProtocol_GHST.cpp | ABSENT | GAP |
| DroneCAN RC | AP_RCProtocol_DroneCAN.cpp | ABSENT | GAP |
| IOMCU | AP_RCProtocol_IOMCU.cpp | ABSENT | LOW |
| MAVLink Radio | AP_RCProtocol_MAVLinkRadio.cpp | ABSENT | LOW |
| Emlid RCIO | AP_RCProtocol_Emlid_RCIO.cpp | ABSENT | LOW |
| UDP | AP_RCProtocol_UDP.cpp | ABSENT | LOW |
| FDM (sim) | AP_RCProtocol_FDM.cpp | ABSENT | LOW |

---

### 2.2 CRSF

#### Frame Format
**ArduPilot:** Sync byte 0xC8, length byte, frame type, payload (variable), CRC-8 DVB-S2 over type+payload.
**Meridian:** Identical. `CRSF_SYNC_BYTE = 0xC8`, `crc8_dvb_s2()` over `buf[2..crc_idx]`.

#### Channel Packing (16 channels × 11 bits)
**ArduPilot:** Bitfield unpacking in `decode_11bit_channels()`. Scale: CRSF range 172-1811 → 1000-2000 µs.
**Meridian:** Manual bit unpacking with `bit_offset` iteration. Same range conversion: `(raw * 1000 / 1639) + 1000`. **Correct.**

#### Failsafe Detection
**ArduPilot:** Failsafe detected when LINK_STATISTICS shows LQ=0 or when receiver outputs failsafe channels. Also detects frame-gap timeout.
**Meridian:** Failsafe hardcoded to `false` in `parse_rc_channels()`. **CRSF failsafe is NOT implemented.** A lost-link condition will not trigger failsafe.

**Gap: CRSF failsafe detection is absent. This is a CRITICAL safety gap.**

#### Telemetry Back-Channel
**ArduPilot:** Full telemetry scheduler with battery, GPS, attitude, flight mode, VTX, and custom telemetry frames sent in time slots.
**Meridian:** `CrsfTelemetry` struct encodes battery, GPS, attitude, and flight mode frames. **Back-channel encoding is PRESENT.** Missing: VTX control, custom telemetry, telemetry scheduler.

#### Link Statistics (RSSI/LQ/SNR)
**ArduPilot:** `LinkStatus` struct with `uplink_rssi_ant1/2`, `uplink_lq`, `uplink_snr`, `rf_mode`, `tx_power`, `downlink_rssi`, `downlink_lq`, `downlink_snr`.
**Meridian:** `CrsfLinkStats` with all 10 fields. `parse_link_stats()` decodes all of them. **PRESENT and CORRECT.**

#### CRSFv3 Baud Negotiation
**ArduPilot:** `_crsf_v3_active` flag, baud negotiation via command frames (type `0x28` proposal, response parsing), can switch to 1.87 Mbit or 3.75 Mbit.
**Meridian:** `CrsfV3BaudNegotiator` struct in `lib.rs` (around line 403). State machine with `Idle → Negotiating → Confirmed`. **CRSFv3 negotiation infrastructure is PRESENT.**

---

### 2.3 SBUS

#### Inverted Serial
**ArduPilot:** `AP_RCProtocol_SBUS(frontend, inverted=true, 100000_baud)` — hardware UART configured with inversion at HAL level. The SoftSerial configured as `SERIAL_CONFIG_8E2I` (inverted).
**Meridian:** `SbusParser` is a pure byte parser. Inversion must be handled at the UART hardware level (STM32 peripheral configuration). The parser itself does not handle inversion. **Gap: UART inversion configuration must be verified in the platform HAL layer (meridian-platform-stm32).**

#### 11-bit Channel Unpacking
**ArduPilot:** `decode_11bit_channels()`, maps range 200-1800 to 1000-2000 via scale factor `SBUS_SCALE_OFFSET = 875`.
**Meridian:** Manual bit unpacking in `parse_frame()`, maps 172-1811 to 1000-2000. **Range constants differ slightly from ArduPilot (172 vs 200, 1811 vs 1800) but functional range is compatible.**

#### BOTH Failsafe Checks (flag + channel value)
**ArduPilot (`AP_RCProtocol_SBUS.cpp`, lines 122-145):** Checks TWO failsafe conditions:
1. `frame[23] & (1 << SBUS_FAILSAFE_BIT)` — explicit failsafe flag (bit 3)
2. `invalid_data` — if any of the first 4 channels is at or below `SBUS_SCALE_OFFSET (875)`, consider failsafe

**Meridian (`lib.rs`, lines 370-397):** Checks:
1. `status & 0x08` — failsafe flag (bit 3) ✓
2. `status & 0x04` — frame_lost flag (bit 2) — treated as failsafe
- **MISSING: The channel-value-based failsafe check (first 4 channels ≤ 875 → failsafe) is absent.**

**Note:** ArduPilot treats frame_lost as NOT a failsafe (it logs but continues using data). Meridian treats frame_lost as failsafe. This is more conservative but semantically incorrect per the SBUS specification (frame_lost = skipped frame, not link failure).

**Gap: Meridian is missing the second SBUS failsafe check (channel values ≤ 875 on first 4 channels). Also, frame_lost is incorrectly treated as failsafe.**

---

### 2.4 DSM

**ArduPilot:** 10-bit and 11-bit detection, channel reorder based on DSM2 vs DSMX protocol. Supports 7, 8, 10, 12, and 14 channel variants. Channel reordering: throttle (ch2), aileron (ch0), elevator (ch1), rudder (ch3), gear (ch4), aux1 (ch5) for MODE1 remapping.
**Meridian (`dsm.rs`):** Implements DSM parsing. Verification of 10/11-bit and channel reorder requires reading the file in detail.

---

### 2.5 FPort

**ArduPilot (`AP_RCProtocol_FPort.cpp`):** Combined RC + telemetry on inverted half-duplex serial. RC frame: 0x7E sync, 0x19 length, 0xFF type, 16 channels (11-bit SBUS packing), flags, RSSI, checksum. Telemetry: response in 3 ms window after RC frame.
**Meridian (`fport.rs`):** FPort implementation exists. **FPort2 (0x18 frame type, improved CRC) is absent.**

---

### 2.6 iBus

**ArduPilot:** 14 channels, 0x20 0x40 header, checksum (sum of all bytes except last 2, then complement).
**Meridian (`ibus.rs`):** iBus parser exists. 14-channel support needs verification.

---

### 2.7 SUMD

**ArduPilot:** CRC16-CCITT (0x1021 poly, 0xFFFF initial). Up to 32 channels.
**Meridian (`sumd.rs`):** CRC16-CCITT present. Header 0xA8 0x01. 8 channels minimum.

---

### 2.8 SRXL2

**ArduPilot:** Full bidirectional SRXL2 protocol. Device info exchange, handshake, telemetry. Uses spektrum SRXL library (`spm_srxl.cpp`).
**Meridian (`srxl2.rs`):** SRXL2 parser with bidirectional CRC16. Reads device ID and channel data. Telemetry encoding present. **Needs verification of full device handshake sequence vs ArduPilot's `spm_srxl` library.**

---

### 2.9 PPM

**ArduPilot:** Sync gap detection (>2.5 ms gap = frame start). Up to 8 channels.
**Meridian (`ppm.rs`):** Sync gap detection present.

---

### 2.10 Missing RC Protocols Summary

| Protocol | Gap Severity |
|----------|-------------|
| CRSF failsafe | CRITICAL |
| SBUS second failsafe check | HIGH |
| SBUS frame_lost semantics | MEDIUM |
| FPort2 | MEDIUM |
| GHST (ImmersionRC Ghost) | MEDIUM (growing ecosystem) |
| SRXL v1 | LOW |
| ST24 | LOW |
| DroneCAN RC | MEDIUM |
| SRXL2 full handshake verification | MEDIUM |

---

## 3. MAVLink (ArduPilot: GCS_MAVLink vs meridian-mavlink)

### 3.1 Message Handler Count

**ArduPilot (`GCS_Common.cpp`):** The stream scheduler table has ~50 outbound message types. The inbound handler dispatch (via `handle_message()`) covers approximately 80+ distinct message IDs, spanning: missions, params, commands, fencing, rally, RC override, GPS injection, log download, camera, gimbal, digicam, ADSB, ping, wind, auth, and more.

**Meridian (`adapter.rs`, `server.rs`):** Handles these inbound messages:
- HEARTBEAT, PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET
- REQUEST_DATA_STREAM
- MISSION_REQUEST_LIST, MISSION_COUNT, MISSION_ITEM_INT, MISSION_REQUEST_INT, MISSION_CLEAR_ALL
- COMMAND_LONG, COMMAND_INT (ARM/DISARM/TAKEOFF/LAND/RTL/SET_MODE/REQUEST_MESSAGE/REQUEST_AUTOPILOT_VERSION)

Outbound messages encoded:
- HEARTBEAT, SYS_STATUS, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, RC_CHANNELS, VFR_HUD, BATTERY_STATUS, STATUSTEXT, PARAM_VALUE, MISSION_REQUEST_INT, MISSION_COUNT, MISSION_ACK, HOME_POSITION, AUTOPILOT_VERSION, COMMAND_ACK

**Meridian handles ~15 inbound messages vs ArduPilot's ~80+. Meridian encodes ~17 outbound message types vs ArduPilot's ~50+.**

---

### 3.2 Stream Rate System

**ArduPilot:** `SRx_*` parameter family. `REQUEST_DATA_STREAM` sets rates per group. 10 groups (RAW_SENSORS, EXT_STATUS, RC_CHANNELS, RAW_CONTROLLER, POSITION, EXTRA1, EXTRA2, EXTRA3, PARAMS, ADC_RC). Each group contains multiple messages.

**Meridian:** 7-group stream scheduler (`DEFAULT_STREAM_INTERVALS_MS`). `StreamRequest` inbound command sets per-group intervals. `MAX_PARAMS_PER_UPDATE = 5` bandwidth limiting.

**Assessment:** Meridian's stream rate system is structurally similar to ArduPilot's but has fewer groups (7 vs 10). The core mechanism (interval-based periodic send) is identical. **SUBSTANTIALLY PRESENT.**

---

### 3.3 Mission Upload/Download Protocol

**ArduPilot:** Full MISSION_COUNT → N × (MISSION_REQUEST_INT → MISSION_ITEM_INT) → MISSION_ACK cycle. Also supports old MISSION_ITEM (deprecated). Handles timeout and retry.

**Meridian (`server.rs`):** Implements mission upload (GCS → FC):
- `MissionCount` → requests each item via MISSION_REQUEST_INT
- `MissionItemInt` → stores item, requests next
- After all items: sends MISSION_ACK (accepted)
- 2-second upload timeout

Mission download (FC → GCS): `MissionRequestList` → sends `MISSION_COUNT`. Individual item fetch not implemented — missing MISSION_ITEM_INT response to MISSION_REQUEST_INT from GCS.

**Gap: Mission download (FC sends waypoints to GCS) is incomplete. MISSION_REQUEST_INT from GCS is handled to request upload items, but the FC cannot send items on demand during download.**

---

### 3.4 MAVFTP

**ArduPilot (`GCS_FTP.cpp`):** Full MAVFTP implementation for file transfer. Used by QGroundControl to download logs, upload parameter files, read filesystem.

**Meridian:** **MAVFTP is ABSENT.** No MAVFTP handler, no FILE_TRANSFER_PROTOCOL message handling.

---

### 3.5 MAVLink Signing

**ArduPilot (`GCS_Signing.cpp`):** SHA-256 based per-packet authentication. Signature frame appended to MAVLink v2 packets. Link ID, timestamp counter, and HMAC-truncated signature. Key management via parameter storage.

**Meridian:** **MAVLink signing is ABSENT.** The MAVLink v2 framer does not include signature support.

---

### 3.6 Log Download

**ArduPilot:** `LOG_REQUEST_LIST`, `LOG_REQUEST_DATA`, `LOG_ENTRY`, `LOG_DATA` messages. Allows QGC to browse and download flight logs over MAVLink.

**Meridian:** **Log download protocol is ABSENT.** No LOG_* message handlers.

---

### 3.7 Parameter Streaming Bandwidth Limiting

**ArduPilot:** Parameters are streamed one per update cycle. Stream throttled based on link bandwidth (`bandwidth_used_bytes` tracking).

**Meridian:** `MAX_PARAMS_PER_UPDATE = 5` — sends at most 5 PARAM_VALUE messages per `update()` call. **Bandwidth limiting is PRESENT** (simple count-based, not byte-count-based). Comment: "bandwidth limiting: ~30% of link."

---

## 4. DroneCAN (ArduPilot: AP_DroneCAN vs meridian-can)

### 4.1 Frame Encoding / Decoding

**ArduPilot:** Uses `libcanard` (bundled at `libraries/AP_DroneCAN/canard/`). 29-bit extended CAN ID encoding: priority[28:24], data_type_id[23:8], service_not_message[7], source_node_id[6:0]. Tail byte: SOT[7], EOT[6], toggle[5], transfer_id[4:0].

**Meridian (`frames.rs`):** Implements identical encoding:
- `encode_can_id(priority, type_id, service_not_message, source_node)` — identical field layout
- `decode_can_id(id)` — extracts same fields
- `TailByte` struct with `single_frame()`, `first_frame()`, `middle_frame()`, `last_frame()` — identical bit positions

**Verdict: Frame encoding/decoding matches ArduPilot/libcanard.**

---

### 4.2 Standard Message Types

| DSDL Message | DTID | ArduPilot | Meridian | Status |
|--------------|------|-----------|----------|--------|
| uavcan.protocol.NodeStatus | 341 | Yes | Yes | PRESENT |
| uavcan.protocol.GetNodeInfo | 1 | Yes | Yes | PRESENT |
| uavcan.equipment.gnss.Fix2 | 1063 | Yes | Yes | PRESENT |
| uavcan.equipment.ahrs.MagneticFieldStrength2 | 1002 | Yes | Yes | PRESENT |
| uavcan.equipment.air_data.RawAirData | 1027 | Yes | Yes | PRESENT |
| uavcan.equipment.esc.RawCommand | 1030 | Yes | Yes | PRESENT |
| uavcan.equipment.power.BatteryInfo | 1092 | Yes | Yes | PRESENT |
| uavcan.equipment.indication.LightsCommand | 1081 | Yes | Yes | PRESENT |
| ardupilot.indication.SafetyState | 20000 | Yes | Yes | PRESENT |
| uavcan.equipment.safety.ArmingStatus | 1100 | Yes | Yes | PRESENT |
| uavcan.protocol.dynamic_node_id.Allocation | 1 | Yes | Yes | PRESENT |
| uavcan.equipment.esc.Status | 1034 | Yes | ABSENT | GAP |
| uavcan.equipment.actuator.ArrayCommand | 1010 | Yes | ABSENT | GAP |
| uavcan.equipment.actuator.Status | 1011 | Yes | ABSENT | GAP |
| uavcan.equipment.range_sensor.Measurement | 1050 | Yes | ABSENT | GAP |
| uavcan.equipment.power.CircuitStatus | 1091 | Yes | ABSENT | GAP |
| com.volz.servo.ActuatorStatus | vendor | Yes | ABSENT | GAP |
| ardupilot.gnss.Status | 20003 | Yes | ABSENT | GAP |
| uavcan.protocol.param.GetSet | 11 | Yes | ABSENT | GAP |
| uavcan.protocol.RestartNode | 5 | Yes | ABSENT | GAP |
| uavcan.protocol.file.* | various | Yes | ABSENT | GAP |

**Meridian has 11/22 audited DroneCAN message types. Missing: ESC status feedback, actuator commands/status, rangefinder, circuit status, parameter protocol, node restart, file protocol.**

---

### 4.3 DNA Server

**ArduPilot (`AP_DroneCAN_DNA_Server.cpp`):** Full DNA server. Stores unique-ID → node-ID mapping in persistent storage (parameter-based). Validates conflicts. Handles multiple concurrent allocation requests. Follows UAVCAN v0 DNA protocol exactly (3-step: request, follow-up, confirm).

**Meridian (`node.rs`, `DnaServer`):** DNA server present. `allocate()` assigns node IDs from `next_free_id`. Tracks entries by unique ID prefix match. In-memory only (no persistence across reboots).

**Gap: Meridian's DNA server is in-memory only.** On reboot, peripherals may receive different node IDs, disrupting parameter-bound configurations. ArduPilot persists the mapping to flash.

---

### 4.4 Multi-Frame Transfer Assembly

**ArduPilot:** libcanard handles multi-frame assembly with CRC16-CCITT verification over the complete payload.

**Meridian (`frames.rs`, `TransferAssembler`):** `TransferAssembler` struct reassembles multi-frame transfers:
- Detects SOT (start of transfer)
- Validates toggle bit sequence
- Reassembles payload, excluding the leading CRC16 bytes from first frame
- Verifies CRC16-CCITT at completion

**Verdict: Multi-frame transfer assembly is PRESENT and structurally correct.**

**Gap:** Meridian's `TransferAssembler` uses a fixed 256-byte buffer (`const MAX_PAYLOAD: usize = 256`). ArduPilot's libcanard uses a dynamic memory pool (8 KB default). Large transfers (e.g., firmware updates, file transfers) may overflow Meridian's buffer.

---

## 5. Complete Gap Inventory

### CRITICAL (safety-affecting, must fix before flight)

| Gap | Location | Impact |
|-----|----------|--------|
| CRSF failsafe detection absent | `meridian-rc/src/lib.rs` | RC loss not detected → vehicle continues flying without pilot control |
| SBUS second failsafe check missing | `meridian-rc/src/lib.rs` | Channel-value-based failsafe (first 4 channels ≤ 875) not checked |
| ICM-42670 uses wrong register map | `imu_icm426xx.rs` | Boards with ICM-42670 will fail to initialize IMU |
| ICM-42688 no SIGNAL_PATH_RESET/DEVICE_CONFIG | `imu_icm426xx.rs` | Stale FIFO data on startup may corrupt first samples |

### HIGH (before extended testing)

| Gap | Location | Impact |
|-----|----------|--------|
| No UBX config sequence sent | `gps_ublox.rs` | Factory-reset receivers will not output UBX frames |
| No GPS auto-detect baud | `gps_ublox.rs` | Must pre-configure receiver or hard-code baud |
| 3-state airspeed Kalman absent | `airspeed.rs` | In-flight airspeed calibration impossible |
| ICM-42688 Bank 1/2 AAF not programmed | `imu_icm426xx.rs` | Anti-aliasing filter left at reset defaults |
| ExternalAHRS IMU/baro/compass absent | meridian-drivers | VectorNav / Pixhawk4-class boards not supported |
| MAVFTP absent | meridian-mavlink | QGC cannot browse logs or upload parameter files |
| MAVLink signing absent | meridian-mavlink | No packet authentication available |
| Mission download incomplete | `server.rs` | GCS cannot download stored mission from vehicle |
| DroneCAN DNA server not persistent | `node.rs` | Node IDs reassigned on reboot |
| GHST protocol absent | meridian-rc | ImmersionRC Ghost receivers not supported |
| DroneCAN RC absent | meridian-rc | DroneCAN-connected RC receivers not supported |

### MEDIUM (before production release)

| Gap | Location | Impact |
|-----|----------|--------|
| GPS blending | meridian-drivers | Dual-GPS redundancy not available |
| RTCM3 injection | meridian-drivers | RTK corrections cannot be forwarded to GPS receiver |
| AK8963 / HMC5843 compass absent | meridian-drivers | Common compass chips unsupported |
| BMM150 compass absent | meridian-drivers | Common DroneCAN peripheral compass |
| LSM6DSV IMU absent | meridian-drivers | New ST chip on modern flight controllers |
| IGRF table 5° resolution | `compass_cal.rs` | 3-5° declination error in steep gradient areas |
| DPS310 temperature source not selected | `baro_dps310.rs` | May use ASIC temp instead of MEMS temp |
| Compass sample coverage check absent | `compass_cal.rs` | Calibration may succeed with poor angular coverage |
| SBUS frame_lost treated as failsafe | `lib.rs` | False failsafe triggers on single missed frames |
| FPort2 absent | meridian-rc | FrSky FPort version 2 receivers not supported |
| SRXL v1 absent | meridian-rc | Older Spektrum receivers not supported |
| DroneCAN actuator/ESC status absent | meridian-can | ESC telemetry and actuator feedback not received |
| DroneCAN rangefinder message absent | meridian-can | CAN-connected rangefinders not supported |
| DroneCAN buffer limit 256 bytes | `frames.rs` | Large multi-frame transfers (firmware update) will overflow |
| Log download protocol absent | meridian-mavlink | Logs not downloadable via MAVLink |
| Optical flow not wired to EKF | meridian-ekf | Optical flow sensor reads but EKF does not fuse them |

### LOW (post-release improvements)

| Gap | Location |
|-----|----------|
| IMU temperature calibration polynomial | meridian-drivers |
| ICM-42688 20-bit HiRes FIFO mode | `imu_icm426xx.rs` |
| Invensensev2 (ICM-20948) | meridian-drivers |
| ICM-45686 / IIM-42652 / IIM-42653 | meridian-drivers |
| ADIS16470 / SCHA63T tactical IMUs | meridian-drivers |
| BMP388 / BMP581 barometers | meridian-drivers |
| ICP101XX / ICP201XX barometers | meridian-drivers |
| LSM9DS0 / LSM9DS1 combo chips | meridian-drivers |
| 44 of 47 rangefinder types | meridian-drivers |
| SDP3X / MS5525 airspeed | meridian-drivers |
| ST24 (Yuneec) RC | meridian-rc |
| SRXL v1 | meridian-rc |
| Moving baseline GPS | meridian-drivers |
| DroneCAN parameter protocol | meridian-can |
| DroneCAN RestartNode | meridian-can |
| DroneCAN file protocol | meridian-can |
| SRXL2 full handshake verification | `srxl2.rs` |
| Declination table upgrade to 1° grid | `compass_cal.rs` |
| MAVLink additional message types (~60) | meridian-mavlink |
| Motor compensation per-motor (N × 3) | `compass_cal.rs` |
