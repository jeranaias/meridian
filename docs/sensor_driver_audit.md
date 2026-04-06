# Sensor Driver Port Audit: ArduPilot → Meridian

**Source**: `D:\projects\ardupilot\libraries\AP_InertialSensor\`, `AP_Baro\`, `AP_Compass\`
**Target**: `D:\projects\meridian\crates\meridian-drivers\`
**Date**: 2026-04-02
**Scope**: ~42K lines of C++ sensor driver code

---

## 1. AP_InertialSensor (IMU)

### 1.1 Driver Architecture

The frontend class `AP_InertialSensor` owns up to `INS_MAX_INSTANCES` (typically 3) backends. Each backend is a subclass of `AP_InertialSensor_Backend`. The lifecycle is:

1. `AP_InertialSensor::detect_backends()` calls board-specific probe lists defined in HAL headers (e.g., `HAL_INS_PROBE_LIST`). Each probe entry calls `BackendClass::probe(imu, dev, rotation)`.
2. `probe()` does a hardware ID check (WHOAMI register), instantiates the object, and calls `hardware_init()`. On failure it returns `nullptr`.
3. On success the backend is added to `_backends[]` array via `_add_backend()`.
4. `AP_InertialSensor::init()` calls `_start_backends()` which calls `start()` on each, where the backend registers its gyro and accel instance slots via `_imu.register_gyro(instance, rate_hz, id)` and `_imu.register_accel(...)`.
5. Each backend registers a periodic callback (via `dev->register_periodic_callback(period_us, fn)`) that runs at the sensor's ODR rate on its bus thread.
6. The main loop calls `AP_InertialSensor::update()`, which clears healthy flags, calls `backend->update()` on each backend, then sets healthy flags for backends that successfully published data and re-selects `_first_usable_gyro/accel`.

**Key registration data**: `_gyro_raw_sample_rates[]`, `_accel_raw_sample_rates[]`, `_gyro_over_sampling[]`, `_accel_over_sampling[]`, per-instance device IDs.

---

### 1.2 IMU Backend Catalog

| Backend class | File | Chips covered | Notes |
|---|---|---|---|
| `AP_InertialSensor_Invensense` | `AP_InertialSensor_Invensense.cpp` | MPU6000, MPU9250, ICM-20608, ICM-20602, ICM-20601, ICM-20689 | Legacy 8kHz FIFO; manual anti-alias filtering needed |
| `AP_InertialSensor_Invensensev2` | `AP_InertialSensor_Invensensev2.cpp` | ICM-20948, ICM-20649, ICM-20648 | Supports AuxiliaryBus for AK09916 magnetometer |
| `AP_InertialSensor_Invensensev3` | `AP_InertialSensor_Invensensev3.cpp` | ICM-40609, ICM-42688, ICM-42605, ICM-40605, IIM-42652, IIM-42653, ICM-42670, ICM-45686 | 32kHz internal AAF; simpler driver; FIFO-only |
| `AP_InertialSensor_BMI055` | `AP_InertialSensor_BMI055.cpp` | BMI055 | Separate accel+gyro chips |
| `AP_InertialSensor_BMI088` | `AP_InertialSensor_BMI088.cpp` | BMI088, BMI085 | Industrial-grade; high shock tolerance |
| `AP_InertialSensor_BMI160` | `AP_InertialSensor_BMI160.cpp` | BMI160 | FIFO; 1600Hz max ODR |
| `AP_InertialSensor_BMI270` | `AP_InertialSensor_BMI270.cpp` | BMI270 | Requires 4KB config file upload at init; FIFO |
| `AP_InertialSensor_LSM9DS0` | `AP_InertialSensor_LSM9DS0.cpp` | LSM9DS0 | Accel+gyro+mag combo |
| `AP_InertialSensor_LSM9DS1` | `AP_InertialSensor_LSM9DS1.cpp` | LSM9DS1 | |
| `AP_InertialSensor_LSM6DSV` | `AP_InertialSensor_LSM6DSV.cpp` | LSM6DSV | ST MEMS; FIFO |
| `AP_InertialSensor_L3G4200D` | `AP_InertialSensor_L3G4200D.cpp` | L3G4200D | Gyro only |
| `AP_InertialSensor_ADIS1647x` | `AP_InertialSensor_ADIS1647x.cpp` | ADIS16470, ADIS16477 | High-precision tactical-grade |
| `AP_InertialSensor_SCHA63T` | `AP_InertialSensor_SCHA63T.cpp` | SCHA63T | Automotive-grade dual-die |
| `AP_InertialSensor_ExternalAHRS` | `AP_InertialSensor_ExternalAHRS.cpp` | External AHRS (e.g. VectorNav) | Receives data via UART |

**For Meridian priority**: ICM-42688 (Matek H743 Slim), BMI270 (common on flight controllers), and BMI088 (vibration-tolerant boards).

---

### 1.3 ICM-42688 Driver (Invensensev3) — Complete Detail

**File**: `D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_Invensensev3.cpp`

#### WHOAMI
```
Register 0x75 → 0x47 (ICM-42688-P) or 0xDB (ICM-42688-V)
Register 0x72 → checked only for ICM-45686 (0xE9)
```

#### Register Map (Main Bank 0)
```
0x11  DEVICE_CONFIG       — soft reset via bit 0
0x16  FIFO_CONFIG         — FIFO mode: 0x80 = stop-on-full
0x2D  INT_STATUS          — interrupt status
0x2E  FIFO_COUNTH         — FIFO sample count (2 bytes, LE)
0x30  FIFO_DATA           — FIFO read burst
0x4B  SIGNAL_PATH_RESET   — 0x02 = flush FIFO
0x4C  INTF_CONFIG0        — 0xC0 = little-endian + count-by-records + last-data-hold
0x4D  INTF_CONFIG1        — AFSR bits [5:6], RTC mode bit 2
0x4E  PWR_MGMT0           — 0x00 = off, 0x0F = gyro+accel low-noise
0x4F  GYRO_CONFIG0        — [7:5]=range, [3:0]=ODR
0x50  ACCEL_CONFIG0       — [7:5]=range, [3:0]=ODR
0x51  GYRO_CONFIG1        — UI filter order
0x52  GYRO_ACCEL_CONFIG0  — accel UI filter order
0x53  ACCEL_CONFIG1       — accel filter
0x5F  FIFO_CONFIG1        — 0x07 = accel+gyro+temp; bit 4 = HIRES_EN
0x60  FIFO_CONFIG2        — watermark low byte
0x61  FIFO_CONFIG3        — watermark high byte
0x75  WHOAMI
0x76  BANK_SEL            — selects register bank 0/1/2/3/4
```

#### Bank 1 Registers (gyro AAF)
```
0x0B  GYRO_CONFIG_STATIC2 — AAF enable bits [0:1]
0x0C  GYRO_CONFIG_STATIC3 — GYRO_AAF_DELT
0x0D  GYRO_CONFIG_STATIC4 — GYRO_AAF_DELTSQR[7:0]
0x0E  GYRO_CONFIG_STATIC5 — [7:4]=GYRO_AAF_BITSHIFT, [3:0]=GYRO_AAF_DELTSQR[11:8]
0x7B  INTF_CONFIG5        — pin9 function config (for RTC/CLKIN on IIM-42652)
```

#### Bank 2 Registers (accel AAF)
```
0x03  ACCEL_CONFIG_STATIC2 — [7:1]=ACCEL_AAF_DELT, [0]=enable
0x04  ACCEL_CONFIG_STATIC3 — ACCEL_AAF_DELTSQR[7:0]
0x05  ACCEL_CONFIG_STATIC4 — [7:4]=ACCEL_AAF_BITSHIFT, [3:0]=ACCEL_AAF_DELTSQR[11:8]
```

#### Initialization Sequence (`set_filter_and_scaling()` → called from `start()`)

1. `register_write(PWR_MGMT0, 0x00)` — power off sensors (required before config change)
2. `register_write(GYRO_CONFIG0, odr_config)` — set ODR (0x06=1kHz, 0x05=2kHz, 0x04=4kHz, 0x03=8kHz)
3. `register_write(ACCEL_CONFIG0, odr_config)` — same ODR
4. `register_write_bank(1, GYRO_CONFIG_STATIC2, aaf_enable & ~0x03)` — enable gyro AAF
5. `register_write_bank(1, GYRO_CONFIG_STATIC3, aaf_delt)` — AAF delta parameter
6. `register_write_bank(1, GYRO_CONFIG_STATIC4, aaf_deltsqr & 0xFF)`
7. `register_write_bank(1, GYRO_CONFIG_STATIC5, (aaf_bitshift<<4) | (aaf_deltsqr>>8))`
8. `register_write_bank(2, ACCEL_CONFIG_STATIC2, accel_aaf_delt<<1)` — enable accel AAF + delta
9. `register_write_bank(2, ACCEL_CONFIG_STATIC3, accel_aaf_deltsqr & 0xFF)`
10. `register_write_bank(2, ACCEL_CONFIG_STATIC4, (accel_aaf_bitshift<<4) | (accel_aaf_deltsqr>>8))`
11. **AFSR fix**: read `INTF_CONFIG1`, write back with bits [6:5] = 0b10 (disable AFSR). This is critical: AFSR switches noise floor at ~100°/s and causes 2ms gyro stall/"stuck gyro". Applied to ALL IxM42xxx devices.
12. `register_write(PWR_MGMT0, 0x0F)` — enable gyro+accel low-noise mode
13. `hal.scheduler->delay_microseconds(300)` — mandatory 300μs after power-on

#### AAF Parameters for ICM-42688 at various ODRs
```
1kHz  (default):  gyro aaf_delt=6,  aaf_deltsqr=36,  aaf_bitshift=10  → ~258Hz cutoff
                  accel aaf_delt=5, aaf_deltsqr=25,  aaf_bitshift=10  → ~213Hz cutoff
2kHz:             gyro aaf_delt=12, aaf_deltsqr=144, aaf_bitshift=8   → ~536Hz cutoff
4kHz/8kHz:        gyro aaf_delt=21, aaf_deltsqr=440, aaf_bitshift=6   → ~997Hz cutoff
```

#### FIFO Configuration
- Mode: stop-on-full (`FIFO_CONFIG = 0x80`)
- Contents: accel + gyro + temperature (`FIFO_CONFIG1 = 0x07`), optionally HIRES bit 4
- Interface: little-endian, count-by-records, last-data-hold (`INTF_CONFIG0 = 0xC0`)
- FIFO flush: write `SIGNAL_PATH_RESET = 0x02`
- Buffer size: 8 samples normal / 24 samples fast-sample window mode

#### FIFO Packet Format (16 bytes normal, 20 bytes HIRES)
```c
struct FIFOData {         // 16 bytes, header must be 0x68 for valid accel+gyro+timestamp
    uint8_t  header;      // 0x68 = ACCEL_EN | GYRO_EN | TMST_FIELD_EN
    int16_t  accel[3];    // little-endian, LSB/g = 32768/16 = 2048
    int16_t  gyro[3];     // little-endian, LSB/dps = 32768/2000 = 16.4
    int8_t   temperature; // temp = val / 2.07 + 25.0
    uint16_t timestamp;   // 16-bit internal counter
};

struct FIFODataHighRes {  // 20 bytes, header must be 0x78 (adds HIRES_EN bit)
    uint8_t  header;
    uint8_t  accel[6];    // 3x 16-bit MSBs + nibbles in trailing byte
    uint8_t  gyro[6];
    int16_t  temperature; // temp = val / 132.48 + 25.0
    uint16_t timestamp;
    uint8_t  gx:4, ax:4, gy:4, ay:4, gz:4, az:4;  // 4 LSBs for 20-bit values
};
```

#### Scale Factors
```
Normal mode:
  Gyro:  ±2000dps,  16.4 LSB/dps,  GYRO_SCALE  = radians(1)/(32768/2000)
  Accel: ±16g,      2048 LSB/g,    ACCEL_SCALE = 9.80665/(32768/16)

HiRes mode (19-bit, ICM-42688):
  Gyro:  ±2000dps,  GYRO_SCALE_HIGHRES  = radians(1)/(524288/2000)
  Accel: ±16g,      ACCEL_SCALE_HIGHRES = 9.80665/(524288/16)

Temperature: val * (1/2.07) + 25.0°C  (normal mode)
             val * (1/132.48) + 25.0°C (HiRes mode)
```

#### Sample Rate / Timing
- Default ODR: 1000 Hz (odr_config = 0x06)
- Fast sampling: 2x/4x/8x loop rate; constrained to powers-of-2, max 8kHz
- Periodic callback runs at `backend_period_us = 1,000,000 / backend_rate_hz`
- Dynamic adjustment: primary IMU adjusts callback to stay synchronous with incoming data; non-primary IMUs reduced to 2x loop rate

#### SPI Transfer for FIFO Read
```
tfr_buffer[0] = FIFO_DATA | 0x80   // read flag
memset(tfr_buffer+1, 0, n*16)
dev->transfer_fullduplex(tfr_buffer, n*16+1)
// samples start at tfr_buffer+1
```

#### Hardware Quirks
- **AFSR bug** (all IxM42xxx): AFSR feature causes 2ms gyro stall near 100°/s. Fixed by setting `INTF_CONFIG1[6:5] = 0b10`.
- **ICM-40605 fifo_config1**: uses 0x0F instead of 0x07 (different bit meaning for temp).
- **SPI read flag**: 0x80 OR'd onto register address for reads.
- **Bank selection**: write `BANK_SEL = bank_num`, do register access, write `BANK_SEL = 0`. The ICM-42670 uses a completely different block-select mechanism (BLK_SEL_R/W + MADDR + M_R/W registers with 10μs delays).
- **Header validation**: on corrupt FIFO, header != 0x68, driver calls `fifo_reset()` and discards all samples.
- **Register checking**: after each FIFO read, at low SPI speed, `dev->check_next_register()` validates one register against expected value. Mismatch increments error counts.

---

### 1.4 BMI270 Driver

**File**: `D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_BMI270.cpp`

#### Init Sequence (requires firmware upload)
1. SPI: dummy read CHIP_ID to initialize SPI mode
2. Write `CMD = 0xB6` (soft reset), delay 2ms
3. Another dummy SPI read
4. Read CHIP_ID, expect 0x24
5. Write `PWR_CONF = 0x00` (disable power save), delay 1ms
6. Write `INIT_CTRL = 0x00` (start loading config)
7. Burst transfer `maximum_fifo_config_file` (4096-byte microcode blob) to register 0x5E
8. Write `INIT_CTRL = 0x01` (end config load)
9. Delay 20ms, read `INTERNAL_STATUS`, bit 0 must be 1 for success
10. Max 5 retries total

#### Register Map Highlights
```
0x00  CHIP_ID             = 0x24
0x02  ERR_REG
0x03  STATUS
0x0C  ACC_DATA_X_LSB      (6 bytes, then 6 gyro bytes follow)
0x12  GYR_DATA_X_LSB
0x24  FIFO_LENGTH_LSB/MSB
0x26  FIFO_DATA
0x40  ACC_CONF            — bandwidth + ODR
0x41  ACC_RANGE           — ±2/4/8/16g
0x42  GYRO_CONF           — filter + ODR  
0x43  GYRO_RANGE          — ±125/250/500/1000/2000 dps
0x48  FIFO_CONFIG_0       — FIFO mode
0x49  FIFO_CONFIG_1       — enable accel/gyro in FIFO
0x7C  PWR_CONF
0x7D  PWR_CTRL            — enable sensors
0x7E  CMD                 — 0xB6 soft reset
```

#### Scale Factors
- Accel: configured for ±16g; scale = 9.80665 * 16 / 32768 = `(1/32768) * 9.80665 * 16`
- Gyro: configured for ±2000dps; scale = radians(2000) / 32767
- Temperature: `klsb * 0.002 + 23.0°C` (updated every 100 FIFO reads ~= every 100ms)

#### FIFO Frame Format
Each frame has a 1-byte header indicating content (accel frame, gyro frame, or skip frame):
- Accel frame: header = 0x84, followed by 6 bytes
- Gyro frame: header = 0xC4, followed by 6 bytes  
- Header 0x80 = invalid frame → `fifo_reset()`

#### Quirks
- Must upload 4096-byte config blob or sensor is non-functional.
- SPI: set read flag 0x80.
- Temperature register is NOT in FIFO; polled separately.
- SPI speed must be LOW during init, HIGH during normal operation.

---

### 1.5 IMU Health Monitoring

**File**: `D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor.cpp`

#### `_gyro_healthy[]` / `_accel_healthy[]`

These flags are **cleared at the start of every `update()` call** (main loop rate, ~400Hz):
```cpp
for i in 0..INS_MAX_INSTANCES:
    _gyro_healthy[i] = false
    _accel_healthy[i] = false
```

They are re-set to `true` inside `_publish_gyro()` / `_publish_accel()`, which are called from `update_gyro()` / `update_accel()` inside each backend's `update()`. So a sensor that fails to deliver a sample in any given loop cycle is immediately unhealthy.

#### Error Count Arbitration
After all backends update, the system performs a cross-IMU sanity check:
```
if any IMU has zero error count since startup:
    mark healthy any IMU that has accumulated errors since startup as unhealthy
    (i.e., prefer clean IMUs over noisy ones)
```
This uses `_gyro_error_count[]` vs `_gyro_startup_error_count[]` snapshots.

#### Health for `_first_usable_gyro/accel`
Scans IMU 0..N in order; first that is both `_gyro_healthy[i] == true` AND `_use(i) == true` becomes `_first_usable_gyro`. The `_use` parameter is the INS_USE/INS_USE2/INS_USE3 parameter.

#### Delta Time Monitoring
In `_notify_new_accel_raw_sample()`:
```cpp
if (sample_us - last_sample_us > 100000U):  // 100ms gap
    zero delta_velocity accumulator, dt = 0
```
This effectively resets the integrator if a sensor goes silent for 100ms.

#### Stuck Sensor Detection (AFSR bug)
The IMU itself does not expose a "stuck" flag. The AFSR workaround (disabling AFSR on init) prevents the most common cause of stuck gyro output. For application-level stuck detection, ArduPilot relies on the gyros_consistent() prearm check.

---

### 1.6 Vibration and Clipping

**File**: `D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor.cpp`, line 2216

```cpp
void AP_InertialSensor::calc_vibration_and_clipping(uint8_t instance, const Vector3f &accel, float dt)
```
Called on every raw accel sample from `_notify_new_accel_raw_sample()`.

#### Clipping Detection
```cpp
if |accel.x| > clip_limit OR |accel.y| > clip_limit OR |accel.z| > clip_limit:
    _accel_clip_count[instance]++
```
Where `clip_limit = (16.0 - 0.5) * 9.80665 ≈ 152.0 m/s²` (backend default; backend can override via `_clip_limit`).

This is a simple threshold check — no hysteresis, no time-domain filtering. Count is monotonically increasing and can be queried via `get_accel_clip_count()`.

#### Vibration Measurement
Two-stage filter pipeline per axis:
1. **Floor filter** (5Hz lowpass, `_accel_vibe_floor_filter`): tracks the slowly-varying mean
2. **Compute deviation**: `diff = (accel - floor_filtered)²` per axis (squared deviation)
3. **Vibration filter** (2Hz lowpass, `_accel_vibe_filter`): smooths the squared deviations

Output via `get_vibration_levels()`:
```cpp
vibe.x = sqrt(_accel_vibe_filter[instance].x)   // RMS-like vibration level
vibe.y = sqrt(_accel_vibe_filter[instance].y)
vibe.z = sqrt(_accel_vibe_filter[instance].z)
```

Stillness check (`is_still()`): all three axes below `_still_threshold` (default: copter=2.5, fixed-wing=0.1).

Constants:
```
AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ  = 5.0 Hz
AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ        = 2.0 Hz
AP_INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS = 500
```

---

### 1.7 Temperature Calibration

**File**: `D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_tempcal.cpp`

**Model**: 3rd-order polynomial correction applied to both accel and gyro, separately. Reference temperature is a fixed `TEMP_REFERENCE = 35.0°C`.

#### Correction Formula
```
correction(T) = polynomial_eval(T - TEMP_REFERENCE, coeff)
             = (c0 + (c1 + c2*(T-35))*((T-35)) * (T-35)) * 1e-6
```
(Scale factor 1e-6 is embedded in stored coefficients to keep param values manageable in GCS.)

#### Application
```cpp
void correct_sensor(temperature, cal_temp, coeff[], v):
    temperature = clamp(temperature, temp_min, temp_max)
    cal_temp = clamp(cal_temp, temp_min, temp_max)
    v -= polynomial_eval(temperature - TEMP_REFERENCE, coeff)
    v += polynomial_eval(cal_temp - TEMP_REFERENCE, coeff)
```
The double evaluation accounts for the fact that calibration offsets were measured at `cal_temp`, not at `TEMP_REFERENCE`. Applied before publishing.

#### Online Learning
When `INS_TCAL_ENABLE = 2` (LearnCalibration):
- Accumulates 100-sample averages at each 0.5°C temperature step.
- Uses `PolyFit<4, double, Vector3f>` (4th-order double-precision polynomial fit).
- Saves partial calibration every 15 seconds.
- Completes when temperature rises ≥ `TEMP_RANGE_MIN` (10°C) from start AND reaches `TMAX` target.
- Timeout: 10 minutes after TEMP_RANGE_MIN is exceeded.
- Reference: always 35°C internally; stored coefficients are delta from this reference.

**Parameters per IMU instance**: ENABLE, TMIN, TMAX, ACC1/ACC2/ACC3 (3D vectors), GYR1/GYR2/GYR3 (3D vectors) = 8+9+9 = 20 floats per instance.

---

### 1.8 Multi-IMU Consistency (Pre-arm)

**File**: `D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor.cpp`, line 1435 / 1523

#### `gyros_consistent(threshold_dps)`
Compares every enabled gyro against the primary:
```
for each gyro i (if use_gyro(i)):
    diff = |gyro[i] - gyro[primary]|
    if diff > radians(threshold): return false
return true
```
Typical threshold: 10 deg/s for prearm. Returns `true` if only one gyro.

#### `accels_consistent(error_threshold_ms2)`
Compares every enabled accel against the primary:
```
for each accel i (if use_accel(i)):
    diff = accel[i] - accel[primary]
    threshold = error_threshold (IMU3 gets 3x threshold due to temp differences)
    Z-axis relaxed: z component threshold *= 2
    if diff.length > threshold: return false
return true
```
Typical threshold: `AP_INERTIAL_SENSOR_ACCEL_ERROR_THRESHOLD` (3.5 m/s²). IMU3 Z-axis gets 2×3×3.5 = 21 m/s² threshold.

---

### 1.9 Sample Timing and FIFO Batch System

The FIFO delivers samples at the hardware ODR (1–8kHz). The periodic callback reads all pending FIFO samples in one SPI transaction per batch, calling `_notify_new_gyro_raw_sample()` / `_notify_new_accel_raw_sample()` for each sample.

Inside `_notify_new_gyro_raw_sample()`:
- `_update_sensor_rate()`: adaptive rate estimation using elapsed time over N samples
- Gyro sample is integrated into `_delta_angle_acc` with coning correction
- After integration, `apply_gyro_filters()` runs the harmonic notch chain and LPF
- Sets `_new_gyro_data[instance] = true`
- Main loop `update_gyro()` then publishes the filtered result and clears the flag

**Oversampling**: backends can declare they oversample N:1 (e.g., if running at 8kHz but providing 1kHz output). The frontend scales the published sample rate accordingly.

**Backend rate validation** (prearm check): each backend's `get_gyro_backend_rate_hz()` must be ≥ 1.8 × loop_rate_hz or the prearm fails.

**BatchSampler**: optional, captures raw sensor-rate or post-filter data to a ring buffer for dataflash logging. Configurable via `INS_LOG_BAT_MASK`, `INS_LOG_BAT_CNT`.

---

### 1.10 Harmonic Notch Filter

**File**: `D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_Backend.cpp`, `apply_gyro_filters()`

The harmonic notch filter is applied in the backend's interrupt-context callback, not the main loop:

```
for each notch in harmonic_notches[]:
    if not enabled: continue
    if not primary IMU AND not EnableOnAllIMUs option: reset + skip
    gyro_filtered = notch.filter[instance].apply(gyro_filtered)
gyro_filtered = gyro_lowpass_filter[instance].apply(gyro_filtered)
```

Up to `HAL_INS_NUM_HARMONIC_NOTCH_FILTERS` (typically 2) independent notch filter groups exist, each with per-instance `HarmonicNotchFilterVector3f` objects.

Frequency input (`update_freq_hz()`) comes from:
- Throttle-based: `sqrt(throttle_ratio) * center_freq_hz`
- RPM sensor: direct RPM reading
- FFT: `AP_GyroFFT` library computes noise peak from the `_gyro_window` ring buffers; calls `harmonic_notch.update_frequencies_hz()` at ~20Hz

FFT data path:
- `save_gyro_window()` pushes raw (or post-filter) gyro samples into `_gyro_window[instance][axis]` ring buffers
- `AP_GyroFFT::update()` runs Welch periodogram on these windows
- Detected frequency fed back to notch filter update

---

## 2. AP_Baro (Barometer)

### 2.1 Driver Architecture

`AP_Baro` owns up to `BARO_MAX_DRIVERS` (3) backend drivers and `BARO_MAX_INSTANCES` (3) sensor instances. A single driver can expose multiple sensor instances (e.g., ICM-20789 includes an internal baro).

**Registration flow**:
1. `AP_Baro::init()` calls board-specific probe list.
2. `probe()` detects chip via WHO_AM_I, reads calibration, calls `register_sensor()` which returns an instance index and stores device ID.
3. Backend registers a periodic callback (50Hz default = 20ms interval).
4. Main loop calls `AP_Baro::update()` which calls `backend_update(i)` for each driver, then recalculates altitudes for all healthy sensors.

**Health definition** (non-periph):
```cpp
healthy(i) = sensors[i].healthy && sensors[i].alt_ok && sensors[i].calibrated
```
Where `sensors[i].healthy` is set true if `last_update_ms < 500ms` ago. `alt_ok` is false if the altitude calculation produces NaN/Inf. `calibrated` requires `calibrate()` to have been called at boot.

---

### 2.2 BMP280 Driver

**File**: `D:\projects\ardupilot\libraries\AP_Baro\AP_Baro_BMP280.cpp`

#### Init Sequence
1. Check WHO_AM_I (reg 0xD0): 0x58 = BMP280, 0x60 = BME280
2. Read 24-byte calibration block from reg 0x88 (T1..T3, P1..P9)
3. Write `CTRL_MEAS (0xF4)`: oversampling T=2x, P=16x, mode=normal (continuous)
4. Write `CONFIG (0xF5)`: IIR filter coefficient 2

#### Calibration Registers
```
0x88-0x9F: T1(u16), T2(s16), T3(s16), P1(u16), P2-P9(s16)
```

#### Oversampling (hardcoded)
```
BMP280_OVERSAMPLING_T = 2  (BMP280_OVERSAMPLING_2)
BMP280_OVERSAMPLING_P = 5  (BMP280_OVERSAMPLING_16)
BMP280_FILTER_COEFFICIENT = 2
```

#### Compensation Algorithm (integer, from Bosch datasheet p22)
Temperature (returns t_fine for use in pressure):
```cpp
var1 = (((adc_T >> 3) - (T1 << 1)) * T2) >> 11
var2 = ((((adc_T >> 4) - T1)^2 >> 12) * T3) >> 14
t_fine = var1 + var2
T = (t_fine * 5 + 128) >> 8   [in 0.01°C units]
```

Pressure (using t_fine):
```cpp
var1 = t_fine - 128000
var2 = var1^2 * P6; var2 += (var1*P5)<<17; var2 += P4<<35
var1 = ((var1^2*P3)>>8) + ((var1*P2)<<12)
var1 = (((1<<47)+var1) * P1) >> 33
p = 1048576 - adc_P
p = ((p<<31 - var2) * 3125) / var1
var1 = (P9 * (p>>13)^2) >> 25
var2 = (P8 * p) >> 19
p = ((p + var1 + var2) >> 8) + (P7 << 4)   [in Pa * 256]
pressure_Pa = p / 256.0
```

#### Sample Rate
- 50Hz (20ms periodic callback)
- Raw data read from registers 0xF7..0xFC (6 bytes: pressure MSB/LSB/XLSB, temp MSB/LSB/XLSB)
- Accumulates sum/count, publishes on `update()` call from main loop

#### SPI Quirk
SPI write uses masked address (AND with 0x7F to clear bit 7); I2C uses address directly.

---

### 2.3 BMP388/BMP390 Driver

**File**: `D:\projects\ardupilot\libraries\AP_Baro\AP_Baro_BMP388.cpp`

#### Init Sequence
1. Write `PWR_CTRL (0x1B) = 0x33` — enable pressure+temp, normal mode (oddly done before WHO_AM_I read)
2. Read WHO_AM_I (reg 0x00): 0x50 = BMP388, 0x60 = BMP390
3. Read pressure calibration (20 bytes from 0x36) and temperature calibration (5 bytes from 0x31)
4. Call `scale_calibration_data()` to convert NVM integers to float coefficients
5. Write `PWR_CTRL = 0x33` again (checked register)
6. Register 50Hz periodic callback

#### Calibration Scaling (float conversion from raw NVM)
```
par_t1 = nvm_par_t1 * 256.0
par_t2 = nvm_par_t2 / 2^30
par_t3 = nvm_par_t3 / 2^48
par_p1 = (nvm_par_p1 - 16384) / 2^20
par_p2 = (nvm_par_p2 - 16384) / 2^29
par_p3 = nvm_par_p3 / 2^32
par_p4 = nvm_par_p4 / 2^37
par_p5 = nvm_par_p5 * 8.0
par_p6 = nvm_par_p6 / 64.0
par_p7 = nvm_par_p7 / 256.0
par_p8 = nvm_par_p8 / 32768.0
par_p9 = nvm_par_p9 / 2^48
par_p10 = nvm_par_p10 / 2^48
par_p11 = nvm_par_p11 / 2^65
```

#### Compensation Algorithm (floating point)
Temperature:
```
partial1 = adc_T - par_t1
partial2 = partial1 * par_t2
T_comp = partial2 + partial1^2 * par_t3    [stored as member, used in pressure comp]
```

Pressure:
```
partial_out1 = par_p5 + par_p6*T + par_p7*T^2 + par_p8*T^3
partial_out2 = adc_P * (par_p1 + par_p2*T + par_p3*T^2 + par_p4*T^3)
partial4 = adc_P^2 * (par_p9 + par_p10*T) + adc_P^3 * par_p11
P_Pa = partial_out1 + partial_out2 + partial4
```

#### SPI Quirk
BMP388 SPI requires discarding the first byte after the address byte. Read uses a `len+2` buffer: `buf[0] = reg|0x80`, transfer, copy from `buf[2]`.

#### Register Map Highlights
```
0x00  CHIP_ID
0x04  PRESS_DATA   (3 bytes, 24-bit)
0x07  TEMP_DATA    (3 bytes, 24-bit)
0x1B  PWR_CTRL     = 0x33 → normal mode, temp+press enabled
0x1C  OSR          — oversampling config (not set by driver → uses POR defaults)
0x1D  ODR          — output data rate
0x1F  CONFIG       — IIR filter
0x31  CAL_T        (5 bytes)
0x36  CAL_P        (20 bytes)
0x7E  CMD          — 0xB6 = soft reset
```

---

### 2.4 Barometer Calibration

**File**: `D:\projects\ardupilot\libraries\AP_Baro\AP_Baro.cpp`, `calibrate()`

1. Mark all sensors `calibrated=true`, `alt_ok=true` (optimistic start)
2. Let sensors settle: 10 × (wait-until-healthy + 100ms) delay = ~1 second
3. Collect `num_samples` (typically 5) readings with 100ms gaps
4. Average pressure per sensor instance
5. Convert to sea-level pressure: `ground_pressure = get_sealevel_pressure(avg_pressure, field_elevation)`
6. Save to parameter storage: `sensors[i].ground_pressure`
7. Panic (config_error) if all sensors remain uncalibrated

`update_calibration()` (called pre-arm in flight): continuously updates `ground_pressure` to current pressure, slewing slowly via LP filter, so the altitude reference stays fresh.

---

### 2.5 Altitude Calculation

**File**: `D:\projects\ardupilot\libraries\AP_Baro\AP_Baro_atmosphere.cpp`

#### Simple model (troposphere, good to ~11km)
```cpp
float get_altitude_difference_simple(base_pressure, pressure):
    temp_K = get_ground_temperature() + 273.15
    scaling = pressure / base_pressure
    return 153.8462 * temp_K * (1.0 - exp(0.190259 * log(scaling)))
```
This is the standard barometric formula: `h = T/L * (1 - (P/P0)^(R*L/g))` where L=0.0065 K/m.

#### Full 1976 US Standard Atmosphere model
Available when `AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED`. Uses 8 atmospheric layers with gradient/isothermal transitions. Used for SITL.

#### Field Elevation Correction
The `_field_elevation_active` value (set from GPS before arming) shifts the reference so altitude is above ground, not above the calibration point. Calibration now stores `P0 = sealevel_equivalent(actual_P, field_elevation)`.

#### Multi-sensor Primary Selection
`update()` tries `_primary_baro` parameter first; falls back to first healthy sensor. Altitude from primary published as `get_altitude()`.

---

## 3. AP_Compass (Magnetometer)

### 3.1 Driver Architecture

`Compass` owns up to `COMPASS_MAX_INSTANCES` (3) sensors and `COMPASS_MAX_BACKEND` (3) backends. Similar probe-and-register pattern.

- Each backend calls `register_compass(dev_id, instance)` during init.
- Backends implement `read()` which updates `_state[id].field` (a Vector3f in body frame, milligauss).
- Health: `_state[i].healthy = (now - _state[i].last_update_ms < 500ms)`
- Main `Compass::read()` calls all backends, updates healthy flags, tries auto-declination if location available.

#### Rotation Handling
Each compass instance has an `orientation` parameter (one of `ROTATION_*` enum values). The backend applies the board rotation; the frontend applies the compass-specific orientation. External compasses have their own rotation parameter.

---

### 3.2 Compass Backend Catalog

| Class | Chip | Notes |
|---|---|---|
| `AP_Compass_HMC5843` | HMC5843, HMC5883L | Classic I2C; 75Hz; 1-3 axis |
| `AP_Compass_AK8963` | AK8963 | Internal to MPU9250; read via AuxiliaryBus |
| `AP_Compass_AK09916` | AK09916 | Internal to ICM-20948; 100Hz |
| `AP_Compass_BMM150` | BMM150 | Bosch; I2C/SPI |
| `AP_Compass_BMM350` | BMM350 | Newer Bosch; I2C |
| `AP_Compass_LIS3MDL` | LIS3MDL | ST MEMS; ±4 to ±16 gauss |
| `AP_Compass_LIS2MDL` | LIS2MDL | ST low-power |
| `AP_Compass_IIS2MDC` | IIS2MDC | ST industrial |
| `AP_Compass_LSM303D` | LSM303D | Accel+mag combo |
| `AP_Compass_QMC5883L` | QMC5883L | Pin-compatible with HMC5883 |
| `AP_Compass_QMC5883P` | QMC5883P | |
| `AP_Compass_RM3100` | RM3100 | PNI; high accuracy; SPI/I2C |
| `AP_Compass_IST8308` | IST8308 | ISENTEK; I2C |
| `AP_Compass_IST8310` | IST8310 | ISENTEK; I2C |
| `AP_Compass_MMC3416` | MMC3416 | MEMSIC |
| `AP_Compass_MMC5xx3` | MMC5603, etc. | MEMSIC new generation |
| `AP_Compass_MAG3110` | MAG3110 | NXP |
| `AP_Compass_LSM9DS1` | LSM9DS1 | ST combo IMU+mag |

---

### 3.3 Compass Calibration System

**Files**: `D:\projects\ardupilot\libraries\AP_Compass\CompassCalibrator.cpp/.h`, `AP_Compass_Calibration.cpp`

#### Overview
Fits an offset ellipsoid to a set of 300 compass samples collected while rotating the vehicle through all orientations. The fitting converts the ellipsoid into a sphere, yielding offsets (hard iron), diagonal scale factors (soft iron X/Y/Z), and off-diagonal terms (cross-axis sensitivity).

#### State Machine
```
NOT_STARTED → WAITING_TO_START → RUNNING_STEP_ONE → RUNNING_STEP_TWO → SUCCESS/FAILED/BAD_ORIENTATION/BAD_RADIUS
```

**WAITING_TO_START**: waits for `delay` seconds and until sample buffer memory is successfully allocated.

**RUNNING_STEP_ONE** (sphere fit, 10 LM iterations):
1. `calc_initial_offset()`: simple centroid of all samples as initial offset estimate
2. `run_sphere_fit()` × 10: Levenberg-Marquardt fitting 4 parameters (radius, offset x/y/z)
3. If fitness diverges → FAILED

**RUNNING_STEP_TWO** (sphere + ellipsoid fit, 35 LM iterations):
4. `thin_samples()`: removes samples that no longer meet minimum-distance criterion after Step 1 refines the radius
5. Continues collecting up to 300 samples
6. `run_sphere_fit()` × 15 more iterations (sphere params only)
7. `run_ellipsoid_fit()` × 20 iterations: 9 parameters (offset 3D + diagonal 3D + offdiagonal 3D)
8. `fit_acceptable()`: checks `sqrt(_fitness) < tolerance` (default 5.0 for external primary, 10.0 for internal/secondary)
9. `fix_radius()`: adjusts scale factor to normalize sphere to expected Earth field magnitude
10. `calculate_orientation()`: tries all `ROTATION_*` orientations, finds best match to expected NED field direction
11. → SUCCESS or FAILED/BAD_ORIENTATION/BAD_RADIUS

#### Sample Acceptance (`accept_sample()`)
New sample accepted only if it is at least `min_distance` from all existing samples. `min_distance` is calculated from the current sphere radius to ensure geometric coverage of the sphere surface. Uses `AP_GeodesicGrid::section()` (geodesic grid) to track coverage by direction.

#### Result Structure
```
ofs:          Vector3f  — hard iron offsets (milligauss)
diag:         Vector3f  — diagonal scale corrections
offdiag:      Vector3f  — off-diagonal corrections (xy, xz, yz)
scale_factor: float     — overall scale normalization
orientation:  Rotation  — detected rotation enum
fitness:      float     — sqrt(mean_squared_residual) in milligauss
```

#### Fitness / Tolerance
- External primary compass: `_calibration_threshold` (default `AP_COMPASS_CALIBRATION_FITNESS_DEFAULT` = 16.0, stored as `sqrt(fitness)` threshold = 5.0? — see `set_tolerance()`)
- Internal/secondary: threshold × 2

#### Runs in a dedicated thread
The calibration loop (`_update_calibration_trampoline`) runs in a 2KB IO-priority thread, not the main loop. Main loop only calls `cal_update()` which checks state and saves results.

#### Number of Samples
`COMPASS_CAL_NUM_SAMPLES = 300` (defined in `CompassCalibrator.h`).

---

### 3.4 Motor Interference Compensation

Two modes exist:

#### Global Motor Compensation (`COMPASS_MOT_COMP_THROTTLE` / `COMPASS_MOT_COMP_CURRENT`)
```cpp
// In read() path:
corrected_field = raw_field + motor_compensation * throttle_or_current
```
`motor_compensation` is a Vector3f parameter per compass instance. Learned during a dedicated motor test where the operator spins motors while ArduPilot records the delta compass reading vs. throttle/current.

#### Per-Motor Compensation (`COMPASS_MOT_COMP_PER_MOTOR = 0x03`)
**File**: `D:\projects\ardupilot\libraries\AP_Compass\Compass_PerMotor.cpp`

4 motors, each with a 3D compensation vector. Each motor's contribution scaled by:
```cpp
output = PWM_to_0_1(hal.rcout->read_last_sent(motor_map[i]))
output *= voltage_scale
output = pow(output, expo)   // expo default 0.65 (nonlinear power scaling)
compass_correction += compensation[i] * output
```

Calibration procedure: start each motor individually, accumulate `(field_change, output)` pairs for 0.5s settling, compute `compensation[i] = field_change / output`.

---

### 3.5 Multi-Compass Consistency

**File**: `D:\projects\ardupilot\libraries\AP_Compass\AP_Compass.cpp`, line 2240

```cpp
bool Compass::consistent() const
```

All compasses configured for yaw use (`use_for_yaw(i)`) are compared against the primary:

| Check | Threshold constant | Value |
|---|---|---|
| XYZ angle difference | `AP_COMPASS_MAX_XYZ_ANG_DIFF` | 90° |
| XY plane angle difference | `AP_COMPASS_MAX_XY_ANG_DIFF` | 60° |
| XY plane length difference | `AP_COMPASS_MAX_XY_LENGTH_DIFF` | 200 milligauss |

The XY checks are more stringent because heading accuracy depends on the horizontal components. The XYZ check catches gross flips (e.g., compass reading upside-down).

Additional check: if `mag_field_xy.is_zero()`, immediately returns false (saturated or dead sensor).

---

### 3.6 Declination Handling

**File**: `D:\projects\ardupilot\libraries\AP_Compass\AP_Compass.cpp`

#### Manual declination
Parameter `COMPASS_DEC` stores declination in radians. Applied in `calculate_heading()`:
```cpp
heading = wrap_PI(atan2(-headY, headX) + _declination)
```

#### Auto-declination
When `COMPASS_AUTODEC = 1` (default) and GPS has a fix:
```cpp
void Compass::try_set_initial_location():
    if !_initial_location_set && ahrs.get_location(loc):
        _declination.set(radians(AP_Declination::get_declination(lat/1e7, lon/1e7)))
        _initial_location_set = true
```
`AP_Declination::get_declination()` uses a 90×90 grid lookup table of IGRF data, interpolating bilinearly.

Called once per `read()` loop until a valid location is obtained. After that, declination is static until reboot (location doesn't update during flight).

---

## 4. Port Priority Summary for Meridian

### Immediate need (Matek H743 Slim)
1. **ICM-42688 via Invensensev3** — most detailed section above; must implement register-accurate init with AFSR fix
2. **BMP388/BMP390** — onboard baro on many H743 builds
3. Basic health infrastructure: `_gyro_healthy[]`, `_accel_healthy[]`, error count tracking

### Second priority
4. **BMI270** — widespread on modern boards; requires firmware blob upload
5. **BMP280** — still common on older boards
6. Vibration/clipping calculation
7. Temperature calibration (3rd-order polynomial apply; online learning can be deferred)

### Third priority
8. Harmonic notch filter integration
9. Compass calibration system (LM sphere+ellipsoid fit)
10. Multi-IMU consistency checks (prearm)
11. Per-motor compass compensation
12. Auto-declination

### Architecture differences to plan for in Rust
- ArduPilot's `register_periodic_callback` model maps to async tasks or RTIC tasks in Meridian
- The "clear healthy flag then re-set it" pattern requires care with Rust's ownership model — suggest a health status enum with timestamp rather than a bare bool
- The `_notify_new_gyro_raw_sample()` hot path with inline coning correction should be a `#[inline(always)]` function called from interrupt context
- The temperature calibration `PolyFit<4>` uses double precision — Rust `f64` arrays will be needed
- `CompassCalibrator`'s thread model maps naturally to a Tokio task or `embassy` task; the semaphore pair maps to `Mutex<State>` + `Mutex<Sample>`
- The FIFO DMA buffer allocation (`MEM_DMA_SAFE`) maps to Rust `#[link_section = ".dma"]` or a pool allocator

---

## 5. Key Source Files (Absolute Paths)

```
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor.h
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor.cpp
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_Backend.h
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_Backend.cpp
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_Invensensev3.h
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_Invensensev3.cpp
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_BMI270.cpp
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_tempcal.h
D:\projects\ardupilot\libraries\AP_InertialSensor\AP_InertialSensor_tempcal.cpp
D:\projects\ardupilot\libraries\AP_Baro\AP_Baro.h
D:\projects\ardupilot\libraries\AP_Baro\AP_Baro.cpp
D:\projects\ardupilot\libraries\AP_Baro\AP_Baro_atmosphere.cpp
D:\projects\ardupilot\libraries\AP_Baro\AP_Baro_BMP280.cpp
D:\projects\ardupilot\libraries\AP_Baro\AP_Baro_BMP388.cpp
D:\projects\ardupilot\libraries\AP_Compass\AP_Compass.h
D:\projects\ardupilot\libraries\AP_Compass\AP_Compass.cpp
D:\projects\ardupilot\libraries\AP_Compass\AP_Compass_Calibration.cpp
D:\projects\ardupilot\libraries\AP_Compass\CompassCalibrator.h
D:\projects\ardupilot\libraries\AP_Compass\CompassCalibrator.cpp
D:\projects\ardupilot\libraries\AP_Compass\Compass_PerMotor.cpp
```
