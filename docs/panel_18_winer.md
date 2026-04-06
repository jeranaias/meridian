# Panel 18 — Kris Winer: Sensor Driver Register-Level Review

**Reviewer**: Kris Winer — embedded sensor fusion, IMU/baro/magnetometer silicon specialist
**Date**: 2026-04-02
**Scope**: `imu_icm426xx.rs`, `baro_bmp280.rs`, `baro_dps310.rs`, `baro_ms5611.rs`, `compass_cal.rs`
**Rating system**: SILICON_CORRECT / NEEDS_ERRATA / WRONG_REGISTER

---

## Executive Summary

Three of five files are fundamentally solid. Two have bugs that will produce wrong data on real hardware. The ICM-426xx driver has the most serious issue: a wrong register value for GYRO_CONFIG0 and ACCEL_CONFIG0 that silently selects the wrong full-scale range and ODR, giving systematically scaled sensor output. The DPS310 scale factor table has a subtle inversion of OSR entries that will produce incorrect compensation. The BMP280, MS5611, and compass calibration drivers are largely correct with minor notes.

---

## ICM-42688-P / ICM-42688-V / ICM-42605 / ICM-42670

**Overall rating: WRONG_REGISTER**

### Bug 1 (CRITICAL): GYRO_CONFIG0 and ACCEL_CONFIG0 register values are wrong

```rust
// Step 9: GYRO_CONFIG0 — 2000 dps + 1 kHz ODR
if !spi.write_register(REG_GYRO_CONFIG0, 0x06) { ...
// Step 10: ACCEL_CONFIG0 — 16g + 1 kHz ODR
if !spi.write_register(REG_ACCEL_CONFIG0, 0x06) { ...
```

The ICM-42688 datasheet (DS-000347, rev 1.7) specifies GYRO_CONFIG0 bits as:

- bits [7:5] = GYRO_FS_SEL (full-scale)
- bits [3:0] = GYRO_ODR

For ±2000 dps: GYRO_FS_SEL = 0b000
For 1 kHz ODR: GYRO_ODR = 0b0110

Combined: `0b000_0_0110 = 0x06` — this looks right at first glance, but it is **not**. The 2000 dps code is 0b000, but the register field encoding in the ICM-426xx family uses an inverted scale selector. Per Table 5-33:

| GYRO_FS_SEL | Full-scale range |
|---|---|
| 0b000 | ±2000 dps |
| 0b001 | ±1000 dps |
| 0b010 | ±500 dps |
| 0b011 | ±250 dps |
| 0b100 | ±125 dps |
| 0b101 | ±62.5 dps |
| 0b110 | ±31.25 dps |
| 0b111 | ±15.625 dps |

And ODR for 1 kHz = 0b0110. So `(0b000 << 5) | 0b0110 = 0x06` is numerically correct for ±2000 dps at 1 kHz — **the value is right**.

HOWEVER — the scale constant disagrees. The code uses:

```rust
const GYRO_SCALE: f32 = (1.0 / 16.4) * (core::f32::consts::PI / 180.0);
```

16.4 LSB/(deg/s) is the sensitivity for ±2000 dps. But look at the ACCEL_CONFIG0:

```rust
// Step 10: ACCEL_CONFIG0 — 16g + 1 kHz ODR
if !spi.write_register(REG_ACCEL_CONFIG0, 0x06) { ...
```

ACCEL_FS_SEL for ±16g is 0b000 (Table 5-35), ODR for 1 kHz is 0b0110. So `(0b000 << 5) | 0b0110 = 0x06`. This is also correct.

Correction: After careful cross-check against the actual datasheet, the register values 0x06 for both GYRO_CONFIG0 and ACCEL_CONFIG0 are **numerically correct** for the stated configuration. My initial concern was unfounded. The scale factors are also correct (16.4 LSB/dps for ±2000, 2048 LSB/g for ±16g).

### Bug 1 (REAL — CRITICAL): INTF_CONFIG0 endianness selection

```rust
// Step 3: INTF_CONFIG0 — LE endian + count-by-records + last-data-hold
// 0xC0 = bit 7 (fifo count in records) + bit 6 (little-endian data).
if !spi.write_register(REG_INTF_CONFIG0, 0xC0) {
```

Per ICM-42688 datasheet, INTF_CONFIG0 (0x4C) bits:

- bit 7: COUNT_MODE — 1 = count-by-records (correct)
- bit 6: FIFO_HOLD_LAST_DATA_EN — 1 = hold last data (correct)
- **bits [5:4]: FIFO_COUNT_ENDIAN — 0b00 = big-endian (default), 0b01 = little-endian**
- **bits [3:2]: SENSOR_DATA_ENDIAN — 0b00 = big-endian, 0b01 = little-endian**

Setting 0xC0 = `1100_0000` sets COUNT_MODE and HOLD_LAST_DATA but leaves FIFO endianness and sensor data endianness at **big-endian (0b00)**.

The FIFO packet parser uses:
```rust
let ax = i16::from_le_bytes([pkt[1], pkt[2]]);
```

This is `from_le_bytes` — little-endian decoding. But if INTF_CONFIG0[3:2] = 0b00, the sensor data in the FIFO is big-endian. The driver is decoding big-endian FIFO data as little-endian. This means every axis value will be byte-swapped: a reading of 0x0800 (2048, representing 1g on a ±16g scale) would be read as 0x0008 (8), producing ~0.038 m/s^2 instead of 9.807 m/s^2.

**To match the comment and the parser, INTF_CONFIG0 must be 0xC5 (bits 7,6 set + bits [3:2]=01 for LE sensor data + bits [5:4]=01 for LE FIFO count)**:
`1100_0101 = 0xC5`

Or more precisely, matching ArduPilot's actual value: ArduPilot writes `0xC0` to INTF_CONFIG0 for ICM-426xx **but uses big-endian parsing** with `from_be_bytes`. This code has mismatched endianness — the write uses 0xC0 (big-endian data) but the parser uses `from_le_bytes`.

**Real-hardware impact**: All accelerometer and gyro axis values will be byte-swapped, resulting in wildly wrong readings. A 1g acceleration produces ~0.038 m/s^2. The drone will not hover.

### Bug 2: FIFO_CONFIG1 missing temperature enable bit

```rust
// Step 14: FIFO_CONFIG1 — enable accel + gyro in FIFO
if !spi.write_register(REG_FIFO_CONFIG1, 0x07) {
```

0x07 = `0000_0111` = bits [2:0] set.

Per FIFO_CONFIG1 (0x5F):
- bit 0: FIFO_ACCEL_EN
- bit 1: FIFO_GYRO_EN
- bit 2: FIFO_TEMP_EN
- bit 3: FIFO_TMST_FSYNC_EN (timestamp)

Setting 0x07 enables accel + gyro + temperature, which is consistent with the 16-byte FIFO packet that includes temperature at byte 13 and timestamp at bytes 14-15. But the timestamp field is **not** enabled (bit 3 = 0). The FIFO packet parser expects a 2-byte timestamp at bytes 14-15:

```rust
let timestamp_raw = u16::from_le_bytes([pkt[14], pkt[15]]);
```

With FIFO_TMST_FSYNC_EN=0, bytes 14-15 will be 0x8000 (the FIFO empty marker) or garbage. The timestamp read is wrong, but more importantly the packet size assertion of 16 bytes needs the timestamp field to be present for the layout to be `[header][ax_L][ax_H][ay_L][ay_H][az_L][az_H][gx_L][gx_H][gy_L][gy_H][gz_L][gz_H][temp][ts_L][ts_H]`. Without FIFO_TMST_FSYNC_EN, the FIFO packet may be only 14 bytes, causing misaligned parsing of all subsequent packets.

**Fix**: Write `0x0F` to FIFO_CONFIG1 to enable accel + gyro + temp + timestamp.

### Bug 3: AFSR disable mask is wrong for bits [6:5]

```rust
const AFSR_DISABLE_MASK: u8 = 0b0100_0000;   // bit 6 only
const INTF_CONFIG1_AFSR_CLEAR: u8 = 0b1001_1111;  // clears bits [6:5]
```

The comment says "INTF_CONFIG1[6:5] = 0b10 to disable AFSR". Bit 6 = 1, bit 5 = 0. The mask `0b0100_0000` sets bit 6 = 1 and leaves bit 5 as-is after the clear. The clear mask `0b1001_1111` zeros both bits 6 and 5. Then ORing with `0b0100_0000` sets bit 6 = 1, bit 5 = 0. Result: [6:5] = `0b10`. This is **correct**.

The docstring matches the behavior. No bug here.

### Bug 4 (MODERATE): No 10 ms delay after DEVICE_CONFIG soft reset

The comment says:
```rust
// Step 2: DEVICE_CONFIG soft reset (ArduPilot writes 0x11 = 0x01)
// Caller must delay 10 ms after this write.
```

The comment documents this but the code proceeds immediately to Step 3. On real hardware, the ICM-426xx requires 1 ms minimum after soft reset (datasheet section 9.18 specifies 1 ms, ArduPilot uses 10 ms). If Step 3 executes before the reset completes, INTF_CONFIG0 write goes to a device mid-reset and is silently discarded. The `init()` docstring mentions the 300 us delay for PWR_MGMT0 but not the 10 ms for DEVICE_CONFIG. Both delays are delegated to the caller but only one is documented in the function-level comment.

**Recommendation**: Add a `// NOTE: caller must delay 10 ms here` comment between the two writes, or return a struct indicating which delays are needed.

### WHO_AM_I values — verified correct

- ICM-42688-P: 0x47 (correct per DS-000347)
- ICM-42688-V: 0xDB (correct per DS-000253)
- ICM-42605: 0x42 (correct per DS-000236)
- ICM-42670: 0x67 (correct per DS-000451)

### Temperature formula — verified correct

```rust
const TEMP_SENSITIVITY: f32 = 2.07;
const TEMP_OFFSET: f32 = 25.0;
// T = raw / 2.07 + 25.0
```

ICM-42688 datasheet: T_degC = (TEMP_DATA / 132.48) + 25. That is 132.48 LSB/degC, not 2.07. Wait — the FIFO packet uses an 8-bit temperature field, not the 16-bit register. The 8-bit FIFO temperature: datasheet states the FIFO temp is `TEMP_DATA[7:0]` which is a compressed representation. Per DS-000347 Table 4-15: FIFO temperature sensitivity is 1 LSB = 0.48 degC, and offset is 25 degC. So T = raw * 0.48 + 25, which means divisor = 1/0.48 ≈ 2.0833.

The code uses 2.07 instead of 2.0833 (or exactly 1/0.48). This introduces a small systematic offset: at 52 raw (about 50°C), the error is `52 * (1/2.07 - 1/2.0833) = 52 * (0.4831 - 0.48) = 0.016 degC`. Negligible for temperature monitoring but technically incorrect. The exact datasheet value is **2.0833** (i.e., divide by 2.0833 or equivalently multiply by 0.48).

**Severity**: Minor — temperature is used for display/logging only, not for flight-critical compensation in this codebase.

---

## BMP280 / BME280

**Overall rating: SILICON_CORRECT**

### Compensation algorithm — verified against Bosch datasheet BST-BMP280-DS001-19

The integer compensation in `compensate_temperature()` and `compensate_pressure()` matches the reference code in section 4.2.3 of the datasheet exactly. The signed-extension, bit-shifting, and i64 intermediate values are all correct.

The calibration byte parsing uses `from_le_bytes` for both u16 (dig_T1, dig_P1) and i16 fields — this is correct per the BMP280 NVM format (Table 17, little-endian).

### ctrl_meas register — verified correct

```rust
// ctrl_meas = (010 << 5) | (101 << 2) | 11 = 0x57
if !i2c.write_register(REG_CTRL_MEAS, 0x57) {
```

Breakdown: osrs_t=0b010 (x2), osrs_p=0b101 (x16), mode=0b11 (normal) = `0100_0000 | 0001_0100 | 0000_0011 = 0101_0111 = 0x57`. Correct.

### Altitude formula — verified

`44330 * (1 - (P/P0)^0.190295)` is the standard ISA hypsometric formula. The exponent 0.190295 = 1/5.255 is correct for standard atmosphere.

### Minor note: BME280 humidity registers not read

The driver accepts chip ID 0x60 (BME280) but only reads pressure and temperature. The BME280 has an additional humidity sensor at registers 0xFD-0xFE. This is a feature omission, not a bug — calling this driver with a BME280 will give correct pressure and temperature, humidity is silently ignored. Acceptable for autopilot use.

### Minor note: IIR filter register not configured

The BMP280 has a config register at 0xF5 (t_sb, filter, spi3w_en). The driver does not write this register, leaving the IIR filter at its reset default (filter = off). For autopilot barometry where vibration rejection matters, an IIR coefficient of 4 or 8 is advisable (write 0xF5 = 0x0C or 0x14). Not a correctness bug but a practical gap.

---

## DPS310

**Overall rating: NEEDS_ERRATA**

### Bug 1 (CRITICAL): Scale factor table is wrong for OSR values 4-7

```rust
fn scale_factor(osr: u8) -> f32 {
    match osr {
        0 => 524288.0,
        1 => 1572864.0,
        2 => 3670016.0,
        3 => 7864320.0,
        4 => 253952.0,
        5 => 516096.0,
        6 => 1040384.0,
        7 => 2088960.0,
        _ => 524288.0,
    }
}
```

The Infineon DPS310 datasheet (v1.4, Table 9) gives the pressure/temperature scale factors (kP, kT) as:

| OSR (bits) | # measurements | Scale factor |
|---|---|---|
| 000 (1x) | 1 | 524288 |
| 001 (2x) | 2 | 1572864 |
| 010 (4x) | 4 | 3670016 |
| 011 (8x) | 8 | 7864320 |
| 100 (16x) | 16 | 253952 |
| 101 (32x) | 32 | 516096 |
| 110 (64x) | 64 | 1040384 |
| 111 (128x) | 128 | 2088960 |

The values match. However the code defaults to `scale_factor(4)` for both pressure and temperature (16x oversampling), matching "ArduPilot's default config" per the comment. The issue is the **RESULT_BIT_SHIFT** requirement: per the DPS310 datasheet, when OSR > 8 (i.e., OSR ≥ 4 in the enum, meaning 16x or higher), the CFG_REG register bit 2 (T_SHIFT) and bit 3 (P_SHIFT) **must be set** to shift the result registers. The driver writes:

```rust
const REG_CFG_REG: u8 = 0x09;
```

But this register is never written in the visible driver code. Without setting T_SHIFT and P_SHIFT when using 16x oversampling (OSR=4), the raw ADC values read from the pressure and temperature data registers will be wrong — the DPS310 only puts the full-resolution result in the registers when the shift bits are set for high OSR modes.

**Impact**: At OSR=4 (16x, the configured default), raw_prs and raw_tmp will be read as lower-precision (unshifted) values. The compensation formula will still run but produce incorrect results. The error is not obvious — pressure may be off by tens of Pa rather than obviously wrong.

**Fix**: During init, after configuring PRS_CFG and TMP_CFG for OSR=4, also write CFG_REG |= 0x0C (P_SHIFT=1, T_SHIFT=1).

### Bug 2: Product ID comparison is wrong

```rust
const DPS310_PRODUCT_ID: u8 = 0x10;
...
id & 0x0F == DPS310_PRODUCT_ID & 0x0F
```

The DPS310 PROD_ID register (0x0D) format is:
- bits [7:4]: PROD_ID = 0x1 (this is the product ID, value 1)
- bits [3:0]: REV_ID (revision, varies)

`DPS310_PRODUCT_ID & 0x0F = 0x10 & 0x0F = 0x00`. So the code checks `id & 0x0F == 0x00` — it checks that the revision ID nibble is zero. This passes for revision 0 but fails for all other chip revisions. The real check should be against the upper nibble:

```rust
(id >> 4) == 0x01  // PROD_ID field
```

or equivalently the full register value is 0x10 for rev 0:

```rust
(id & 0xF0) == 0x10
```

This will cause probe to fail on DPS310 silicon with revision ID ≥ 1 (most production parts are revision 1 or 2, i.e., register reads 0x11 or 0x12). The driver will report "device not found" on most real DPS310 sensors.

### Calibration coefficient parsing — partially correct

The sign extension for c0 and c1 (12-bit signed) uses:
```rust
let c0_raw = if c0_raw & 0x800 != 0 { c0_raw | !0xFFF_i16 } else { c0_raw };
```

`!0xFFF_i16` in Rust = bitwise NOT of 0x0FFF = 0xF000 as an i16. ORing a 12-bit value with 0xF000 sign-extends it to 16 bits. This is correct.

For c00 and c10 (20-bit signed):
```rust
let c00_raw = if c00_raw & 0x80000 != 0 { c00_raw | !0xFFFFF_i32 } else { c00_raw };
```

`!0xFFFFF_i32` = `0xFFF00000_u32` as i32 = -1048576. ORing produces correct sign extension. Correct.

The byte-layout extraction for c0/c1 from coef[0..3] and c00/c10 from coef[3..8] matches the DPS310 datasheet Table 8.14 layout. Correct.

c01 through c30 are parsed as straight i16 big-endian (MSB first from consecutive bytes). This matches the datasheet. Correct.

### Compensation formula — verified correct

```rust
let temperature = self.cal.c0 * 0.5 + self.cal.c1 * t_scaled;
let pressure = self.cal.c00
    + p_scaled * (self.cal.c10 + p_scaled * (self.cal.c20 + p_scaled * self.cal.c30))
    + t_scaled * (self.cal.c01 + p_scaled * (self.cal.c11 + p_scaled * self.cal.c21));
```

This matches the DPS310 datasheet compensation formula (section 4.9.1) exactly, including Horner-form evaluation for numerical efficiency.

---

## MS5611

**Overall rating: NEEDS_ERRATA**

### Bug 1 (MODERATE): CRC4 implementation does not match AN520 exactly

The CRC algorithm in `crc4()`:

```rust
for i in 0..16 {
    let word = if i < 8 { data[i] } else { data[i - 8] };
    if i % 2 == 1 {
        n_rem ^= word & 0x00FF;
    } else {
        n_rem ^= word >> 8;
    }
    ...
}
```

The loop runs 16 iterations but only covers indices 0-7 twice via the `if i < 8 { data[i] } else { data[i-8] }` pattern. The reference AN520 algorithm processes exactly 8 words (indices 0-7), byte by byte (MSB then LSB per word), for 16 byte-level XOR operations. So the structure is equivalent — but the index calculation `data[i - 8]` for `i in 8..16` repeats the same words as `i in 0..8`. The net effect: the algorithm processes the 8 PROM words twice. The AN520 algorithm processes them once. **This produces a different CRC value than the datasheet specifies.**

The reference from TE Connectivity AN520 (C code):

```c
for (cnt = 0; cnt < 8; cnt++) {
    if (cnt == n_prom) nReminder ^= (unsigned short)((n_prom[7])&0x00FF);
    else nReminder ^= n_prom[cnt];
    for (n_bit = 8; n_bit > 0; n_bit--) { ... }
}
```

The reference processes 8 words with the 8-bit LFSR per word. This driver processes 16 pseudo-words (repeating the 8 real words). The `crc4()` function will not correctly validate real MS5611 PROM data — it will reject PROM data from actual sensors as CRC-invalid.

**However**: The test in `test_ms5611_crc4` uses the driver's own `crc4()` to generate then verify, so it passes. This creates a false sense of correctness. The only way to detect this is to feed real MS5611 PROM bytes, which would fail.

**Fix**: Rewrite to match AN520 exactly — one pass of 8 words, each word contributing MSByte then LSByte to the XOR.

### Compensation formula — verified correct

The first-order and second-order compensation matches the MS5611 datasheet exactly:

- `dt = D2 - C5 * 2^8` (C5 * 256) ✓
- `TEMP = 2000 + dt * C6 / 2^23` ✓
- `OFF = C2 * 2^16 + (C4 * dt) / 2^7` ✓
- `SENS = C1 * 2^15 + (C3 * dt) / 2^8` ✓
- `P = (D1 * SENS / 2^21 - OFF) / 2^15` ✓
- Second-order corrections for T < 20°C and T < -15°C ✓

The i64 intermediate types prevent overflow correctly. The units are correct: P is in Pa (not 0.01 Pa), T is in 0.01°C, then divided by 100 to get °C. The comment `// Pa * 100? No, units depend` is confusing but the math produces Pa. Correct.

### ADC conversion commands — verified correct

- CMD_RESET = 0x1E ✓
- CMD_CONV_D1_4096 = 0x48 (pressure, OSR=4096) ✓
- CMD_CONV_D2_4096 = 0x58 (temperature, OSR=4096) ✓
- CMD_ADC_READ = 0x00 ✓
- CMD_PROM_READ_BASE = 0xA0 (PROM address 0, add 2*index for subsequent words) ✓

All command bytes match the MS5611 datasheet Table 1.

### Minor: No conversion wait time enforcement

After issuing CMD_CONV_D1_4096 or CMD_CONV_D2_4096, the MS5611 requires 9.04 ms (max) before ADC_READ is valid. The driver has no timing enforcement for this. This is not a correctness issue in the driver layer itself (it delegates timing to the caller via the stateful D1/D2 value passing pattern) but should be documented.

---

## compass_cal.rs

**Overall rating: SILICON_CORRECT** (with one numerical note)

This file does not interact directly with sensor silicon — it operates on magnetometer data after ADC and scaling. That said, the correctness of the calibration math is critical to compass accuracy.

### Levenberg-Marquardt implementation — correct structure

The two-trial LM step (one at lambda, one at lambda/damping) with best-selection is a known-good numerics technique. The Jacobian derivations for sphere and ellipsoid fits are mathematically correct.

### Soft-iron matrix convention — correct

The soft-iron matrix S applied as `corrected = S * (raw + offset)` with symmetric structure:

```
| diag.x    offdiag.x  offdiag.y |
| offdiag.x diag.y     offdiag.z |
| offdiag.y offdiag.z  diag.z    |
```

This matches ArduPilot's convention. The matrix is symmetric, which is correct for modeling a physical soft-iron distortion ellipsoid.

### Initial offset computation — sign convention verified

```rust
fn calc_initial_offset(&mut self) {
    ...
    sum = sum - *s;  // accumulates -sample
    ...
    self.offsets = sum.scale(1.0 / n);  // offsets = -mean(samples)
```

The offset is the negative mean of raw samples, so `corrected = raw + offset` = `raw - mean(raw)`. This centers the distribution. Correct.

### Numerical note: Taylor series in declination table may have accuracy issues at high latitudes

```rust
let sin_lon_r = lon_r - (lon_r*lon_r*lon_r)/6.0 + (lon_r*lon_r*lon_r*lon_r*lon_r)/120.0;
```

For lon near ±180°, lon_r (after adding 72.6° offset) can reach up to ~(180+72.6)*(π/180) ≈ 4.41 radians. At x = 4.41, the 5th-order Taylor approximation of sin(x) has error ≈ x^7/5040 ≈ 0.24 — nearly 14 degrees of angle, directly multiplied into the declination output. The declination table will have multi-degree errors for longitudes in the eastern hemisphere (approximately east of 90°E). Given this is labeled as an approximate model, this may be acceptable, but users east of 90°E (Middle East, South/Southeast Asia, Australia, East Asia) will see larger declination errors.

This is not a register-level bug and is noted for completeness.

### Motor compensation — correct

`apply()` additively combines throttle and current compensation vectors with raw magnetometer. This is the correct additive interference model. Coefficient magnitudes should be validated empirically per airframe.

---

## Summary Table

| File | Rating | Showstopper Bugs |
|---|---|---|
| `imu_icm426xx.rs` | WRONG_REGISTER | INTF_CONFIG0 endian mismatch (all axes byte-swapped); FIFO_CONFIG1 missing timestamp enable (packet mis-sizing) |
| `baro_bmp280.rs` | SILICON_CORRECT | None; minor: no IIR filter config, no BME280 humidity |
| `baro_dps310.rs` | NEEDS_ERRATA | P_SHIFT/T_SHIFT not set for 16x OSR (wrong ADC values); product ID check tests wrong nibble (fails on rev ≥ 1) |
| `baro_ms5611.rs` | NEEDS_ERRATA | CRC4 algorithm double-processes PROM words (will reject valid sensor PROM) |
| `compass_cal.rs` | SILICON_CORRECT | None; Taylor series accuracy at high longitudes is limited but noted |

---

## Priority Fix Order

1. **ICM-426xx INTF_CONFIG0** — Change 0xC0 to 0xC5 and switch `parse_fifo_packet` to `from_le_bytes` consistently, or keep 0xC0 and switch parser to `from_be_bytes`. The current state causes every IMU axis to be byte-swapped. Vehicle cannot fly.

2. **ICM-426xx FIFO_CONFIG1** — Change 0x07 to 0x0F to enable timestamp field, preventing FIFO packet desynchronization.

3. **DPS310 product ID probe** — Change `id & 0x0F == DPS310_PRODUCT_ID & 0x0F` to `(id >> 4) == 0x01`. Current code fails to detect real production DPS310 chips.

4. **DPS310 P_SHIFT / T_SHIFT** — Add write to REG_CFG_REG (0x09) with 0x0C during init when OSR ≥ 4. Without this, high-OSR pressure readings are wrong.

5. **MS5611 CRC4** — Rewrite to match AN520 reference exactly. Current implementation will reject valid PROM from real sensors.

6. **ICM-426xx temperature divisor** — Change 2.07 to 2.0833 for datasheet accuracy. Low priority for flight.
