# Panel Round 2 — Wave 3 Review
**Date:** 2026-04-02
**Scope:** Three-persona follow-up targeting hardware-platform and math items from Round 1.

---

## Persona 8 — Jorge Aparicio (Embedded Rust / Unsound Items)

### Background
Round 1 flagged six UNSOUND items (A1–A6) in the STM32H7 platform crate. These are all hardware-platform-specific and were acknowledged as deferred to Phase 9. This review checks whether any were addressed and audits the IWDG implementation specifically requested.

---

### A1 — Semaphore guard drops immediately in SPI/I2C

**File:** `crates/meridian-platform-stm32/src/spi.rs` lines 88–103  
**File:** `crates/meridian-platform-stm32/src/i2c.rs` lines 62–74

The `Stm32SpiSemaphore::take_blocking()` and `Stm32I2cSemaphore::take_blocking()` implementations still bind the `RecursiveMutex` guard to a local `_guard` binding. The guard drops at the end of `take_blocking()`, releasing the lock before the caller has finished its critical section. The comment now reads:

```rust
// Guard is intentionally leaked here — the HAL semaphore model is
// take/give rather than RAII. Real implementation will store the guard.
```

That is accurate self-documentation, not a fix. The lock is still released on function return. The `give()` method is a no-op. The critical section provides zero mutual exclusion in the compiled binary.

**Rating: DEFERRED** — The issue is now explicitly acknowledged in comments, but no structural fix is present. The guard must be stored in the `Stm32SpiSemaphore` / `Stm32I2cSemaphore` structs (e.g., `Option<RecursiveGuard<'static, ()>>`), which requires resolving the self-referential lifetime, a Phase 9 problem.

---

### A2 — TIM2 double-use conflict (buzzer PWM vs. 1 kHz scheduler)

**Files:** `crates/meridian-platform-stm32/src/pwm.rs` line 16, `crates/meridian-platform-stm32/src/rtic_app.rs` lines 13, 133, 316

Both usages persist. `pwm.rs` header documents TIM2 channel 1 for the Buzzer (PA15). `rtic_app.rs` binds TIM2 at priority 4 as the 1 kHz timer interrupt (`timer_1khz`). These two roles are mutually exclusive on a single physical timer peripheral. No resolution code exists. No comment indicates a pin remap, timer swap, or conditional compilation guard.

**Rating: DEFERRED** — Conflict is unchanged. The correct fix (reassign buzzer to TIM15 or another free timer, or use a software-generated buzzer tone off the scheduler tick) has not been applied. This will produce a silent hardware conflict on the MatekH743 until addressed in Phase 9.

---

### A6 — D-cache coherency (SCB clean/invalidate)

**File:** `crates/meridian-platform-stm32/src/spi.rs` lines 272–276, 303–304  
**File:** `crates/meridian-platform-stm32/src/adc.rs` lines 180–181  
**File:** `crates/meridian-platform-stm32/src/uart.rs` lines 262–263, 314–315

All cache maintenance calls remain as commented-out TODO code:

```rust
// TODO: SCB::invalidate_dcache_by_address(...)
// SCB::clean_dcache_by_address(DMA_TX_BUF[bus].as_ptr() as usize, len);
// SCB::invalidate_dcache_by_address(DMA_RX_BUF[bus].as_ptr() as usize, len);
```

No `SCB::clean_dcache_by_address` or `SCB::invalidate_dcache_by_address` calls are present in any compiled code path. On the Cortex-M7 D-cache, DMA transfers to/from AXI SRAM without cache maintenance will silently corrupt data. The `memory.rs` `configure_mpu()` stub also defers MPU setup, so the SRAM4 non-cacheable region for DShot is not actually configured.

**Rating: DEFERRED** — All D-cache coherency points remain as TODO comments. No functional fix.

---

### IWDG — Hardware watchdog register writes

**File:** `crates/meridian-platform-stm32/src/watchdog.rs` lines 136–163

This is the most significant positive finding in this wave. The IWDG `start()` method now contains **real `#[cfg(target_arch = "arm")]` register writes** using `core::ptr::write_volatile`:

```rust
// Enable IWDG
core::ptr::write_volatile(kr, IWDG_KEY_ENABLE as u32);   // 0xCCCC

// Enable write access to PR and RLR
core::ptr::write_volatile(kr, IWDG_KEY_WRITE_ACCESS as u32); // 0x5555

// Set prescaler (64)
core::ptr::write_volatile(pr, IWDG_PRESCALER_REG as u32);   // 4 → /64

// Set reload value
core::ptr::write_volatile(rlr, IWDG_RELOAD as u32);         // 1000 → 2s

// Wait for PVU and RVU bits to clear
while core::ptr::read_volatile(sr) & 0x03 != 0 { asm!("nop"); }

// First pat
core::ptr::write_volatile(kr, IWDG_KEY_RELOAD as u32);      // 0xAAAA
```

Constants are correct: IWDG1_BASE = 0x5800_4800, reload = 2000 × (32000/64) / 1000 = 1000 counts = 2 s timeout. The `pat()` method writes `IWDG_KEY_RELOAD` (0xAAAA) to KR via `write_volatile`. The status-register poll before the first pat is correct per the STM32H7 reference manual (PR/RVU bits must clear before reload takes effect).

`check_reset_cause()`, `save_persistent()`, `load_persistent()`, and `clear_persistent()` are still TODO stubs, but those affect crash diagnostics, not the core watchdog function.

**Rating: FIXED** (core watchdog). Persistent data and reset-cause detection remain TODO.

---

### Summary — Persona 8

| Item | Rating |
|------|--------|
| A1 — SPI/I2C semaphore guard drops immediately | DEFERRED |
| A2 — TIM2 double-use conflict | DEFERRED |
| A6 — D-cache coherency TODO comments | DEFERRED |
| IWDG register writes | FIXED |

All three original UNSOUND items remain hardware-deferred. The watchdog is now functionally correct on ARM targets.

---

## Persona 18 — Kris Winer (Sensor Silicon)

### ICM-42688 — INTF_CONFIG0 and parser endianness consistency

**File:** `crates/meridian-drivers/src/imu_icm426xx.rs` lines 214–217, 389–403

`INTF_CONFIG0` is written as `0xC0` (bit 7 = FIFO count in records, bit 6 = big-endian FIFO count). All FIFO parser byte reads use `i16::from_be_bytes` and `u16::from_be_bytes`. The comment explicitly states: "Sensor data is big-endian (default). Parsers use from_be_bytes." Both sides are consistently big-endian.

**Rating: FIXED**

---

### ICM-42688 — FIFO_CONFIG1 includes timestamp bit (0x0F)

**File:** `crates/meridian-drivers/src/imu_icm426xx.rs` line 285

```rust
// 0x0F = accel(bit0) + gyro(bit1) + temp(bit2) + timestamp(bit3)
if !spi.write_register(REG_FIFO_CONFIG1, 0x0F) {
```

Value is 0x0F, which sets all four bits: accel, gyro, temp, and timestamp. Comment confirms all four fields.

**Rating: FIXED**

---

### DPS310 — probe checks upper nibble `(id >> 4) == 0x01`

**File:** `crates/meridian-drivers/src/baro_dps310.rs` lines 100–105

```rust
pub fn probe_i2c(i2c: &mut dyn I2cProbe, addr: u8) -> bool {
    if let Some(id) = i2c.read_register_at(addr, REG_PRODUCT_ID) {
        (id >> 4) == 0x01
    } else {
        false
    }
}
```

Checks `(id >> 4) == 0x01` exactly as required. The constant `DPS310_PRODUCT_ID: u8 = 0x10` is defined but not used in the probe path, which is correct — the raw nibble comparison is used directly.

**Rating: FIXED**

---

### DPS310 — CFG_REG has P_SHIFT/T_SHIFT set

**File:** `crates/meridian-drivers/src/baro_dps310.rs` lines 27–30, 117–123

```rust
/// CFG_REG value for 16x oversampling: P_SHIFT (bit 2) + T_SHIFT (bit 3) = 0x0C.
const DPS310_CFG_REG_SHIFT: u8 = 0x0C;
```

`init_register_writes()` returns `(REG_CFG_REG, DPS310_CFG_REG_SHIFT)` as the third entry in the sequence, written after PRS_CFG and TMP_CFG and before MEAS_CFG. 0x0C = bit 2 (P_SHIFT) | bit 3 (T_SHIFT). Sequence and value are both correct.

**Rating: FIXED**

---

### MS5611 — CRC4 processes each word once

**File:** `crates/meridian-drivers/src/baro_ms5611.rs` lines 79–103

```rust
for i in 0..16 {
    if i % 2 == 0 {
        n_rem ^= data[i >> 1] >> 8;   // high byte of word i/2
    } else {
        n_rem ^= data[i >> 1] & 0x00FF; // low byte of word i/2
    }
    for _ in 0..8 {
        // CRC bit shift
    }
}
```

Iterates i = 0..16. For each word index `i/2` (0..7, plus word 7's high byte for i=14), the high byte is processed on even i and the low byte on odd i. Each of the 8 words is processed exactly once via two half-word iterations. `data[7] &= 0xFF00` zeroes the CRC nibble before the loop. This matches TE Connectivity AN520 reference algorithm.

**Rating: FIXED**

---

### ICM-42688 — Soft-reset before signal_path_reset (correct order)

**File:** `crates/meridian-drivers/src/imu_icm426xx.rs` lines 195–209

The init comment explicitly documents the order fix:

```rust
// WH2 fix: Soft reset must happen BEFORE signal path reset.
// ArduPilot order: DEVICE_CONFIG reset → 10ms delay → SIGNAL_PATH_RESET

// Step 1: DEVICE_CONFIG soft reset (ArduPilot writes 0x11 = 0x01)
if !spi.write_register(REG_DEVICE_CONFIG, 0x01) { ... }

// Step 2: SIGNAL_PATH_RESET — flush FIFO after reset
if !spi.write_register(REG_SIGNAL_PATH_RESET, 0x02) { ... }
```

`REG_DEVICE_CONFIG` (0x11) is written first, then `REG_SIGNAL_PATH_RESET` (0x4B). Correct order.

**Rating: FIXED**

---

### Summary — Persona 18

| Item | Rating |
|------|--------|
| INTF_CONFIG0 and parser endianness consistent (both BE) | FIXED |
| FIFO_CONFIG1 includes timestamp bit (0x0F) | FIXED |
| DPS310 probe checks upper nibble (id >> 4) == 0x01 | FIXED |
| DPS310 CFG_REG has P_SHIFT/T_SHIFT set (0x0C) | FIXED |
| MS5611 CRC4 processes each word once | FIXED |
| ICM-42688 soft-reset before signal_path_reset | FIXED |

All six sensor register items are fully resolved. Clean sweep.

---

## Persona 15 — Jack Crenshaw (Math)

### Notch filter a2 coefficient: `1.0 - alpha/(A*A)` not `1.0 - alpha`

**File:** `crates/meridian-control/src/notch_filter.rs` lines 139–153

The coefficient calculation in `calculate_coefficients()`:

```rust
let a_sq = a * a;
self.b0 = 1.0 + alpha * a_sq;
self.b1 = -2.0 * cos_omega;
self.b2 = 1.0 - alpha * a_sq;
self.a0 = 1.0 + alpha / a_sq;
self.a1 = -2.0 * cos_omega;
self.a2 = 1.0 - alpha / a_sq;   // ← this is 1.0 - alpha / (A*A)
```

`a2 = 1.0 - alpha / a_sq` is mathematically `1.0 - alpha / (A^2)`, which is the correct ArduPilot notch formula. The test comment at line 697 also references "corrected a2 coefficient." This was the Round 1 bug: the original code had `1.0 - alpha` (dropping the `A^2` denominator), flattening the notch depth for large attenuation values. The fix is present.

**Rating: FIXED**

---

### LowPassFilter2p exists and is a 2nd-order Butterworth biquad

**File:** `crates/meridian-control/src/notch_filter.rs` lines 197–306

`LowPassFilter2p` is present in the same file as `NotchFilter`. It implements a Butterworth biquad via bilinear transform with `Q = 1/sqrt(2) = 0.7071` (the Butterworth maximally-flat condition). Coefficients follow the standard biquad low-pass form:

```rust
let q = core::f32::consts::FRAC_1_SQRT_2;  // Butterworth Q
let alpha = sin_w / (2.0 * q);
let a0 = 1.0 + alpha;
let inv_a0 = 1.0 / a0;

self.b0 = ((1.0 - cos_w) * 0.5) * inv_a0;
self.b1 = (1.0 - cos_w) * inv_a0;   // = 2 * b0 (correct)
self.b2 = self.b0;
self.a1 = (-2.0 * cos_w) * inv_a0;
self.a2 = (1.0 - alpha) * inv_a0;
```

Uses Direct Form II transposed in `apply()`. Four tests present and passing: DC passthrough, high-frequency attenuation (>20 dB), low-frequency pass (<3 dB loss at 10 Hz with 100 Hz cutoff), and state reset. This is a correct 2nd-order Butterworth biquad.

**Rating: FIXED**

---

### Quaternion from_dcm: Shepperd guard `s < 1e-6` present

**File:** `crates/meridian-math/src/quaternion.rs` lines 152–195

All four branches of Shepperd's method contain the guard:

```rust
// tr > 0 branch:
let s = libm::sqrtf(tr + 1.0) * 2.0;
if s < 1e-6 { return Self::identity(); }

// m[0][0] largest:
let s = libm::sqrtf(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
if s < 1e-6 { return Self::identity(); }

// m[1][1] largest:
let s = libm::sqrtf(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
if s < 1e-6 { return Self::identity(); }

// m[2][2] largest:
let s = libm::sqrtf(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
if s < 1e-6 { return Self::identity(); }
```

All four division-by-`s` paths are guarded. The `from_dcm` docstring explicitly mentions: "Includes a near-singularity guard: if the discriminant `s` is below 1e-6 (degenerate or near-zero-rotation matrix), returns identity instead of dividing by ~0."

Note: There is one unresolved issue flagged but not in scope for this review — the `#[ignore]`-annotated test `test_euler_convention_matches_ardupilot` at line 355 indicates a known Euler convention discrepancy against ArduPilot's ZYX expectation. This is not the `from_dcm` item but is visible in the file and should be tracked.

**Rating: FIXED**

---

### Summary — Persona 15

| Item | Rating |
|------|--------|
| Notch a2 coefficient: `1.0 - alpha/(A*A)` | FIXED |
| LowPassFilter2p exists as 2nd-order Butterworth biquad | FIXED |
| Quaternion from_dcm: Shepperd guard `s < 1e-6` present | FIXED |

All three math items fully resolved.

---

## Overall Round 2 Wave 3 Scorecard

| Persona | Item | Rating |
|---------|------|--------|
| 8 – Aparicio | A1 — SPI/I2C semaphore guard | DEFERRED |
| 8 – Aparicio | A2 — TIM2 double-use | DEFERRED |
| 8 – Aparicio | A6 — D-cache coherency TODOs | DEFERRED |
| 8 – Aparicio | IWDG register writes | **FIXED** |
| 18 – Winer | ICM-42688 INTF_CONFIG0 + parser endianness | **FIXED** |
| 18 – Winer | ICM-42688 FIFO_CONFIG1 timestamp bit 0x0F | **FIXED** |
| 18 – Winer | DPS310 probe upper nibble check | **FIXED** |
| 18 – Winer | DPS310 CFG_REG P_SHIFT/T_SHIFT | **FIXED** |
| 18 – Winer | MS5611 CRC4 one-pass per word | **FIXED** |
| 18 – Winer | ICM-42688 soft-reset order | **FIXED** |
| 15 – Crenshaw | Notch a2 = `1.0 - alpha/(A*A)` | **FIXED** |
| 15 – Crenshaw | LowPassFilter2p 2nd-order Butterworth | **FIXED** |
| 15 – Crenshaw | Quaternion from_dcm Shepperd guard | **FIXED** |

**10 FIXED / 3 DEFERRED (all hardware-platform, all correctly deferred to Phase 9)**

The sensor driver layer (Persona 18) and math layer (Persona 15) are fully resolved. The remaining open items are all in the STM32 platform HAL and require physical hardware integration work: semaphore storage lifetimes (A1), timer peripheral assignment (A2), and D-cache maintenance calls (A6). None of these affect SITL or Linux targets.

### Open Item Carry-Forward to Phase 9

1. **A1**: Store `RecursiveGuard` in `Stm32SpiSemaphore` / `Stm32I2cSemaphore` using `Option<RecursiveGuard<'static, ()>>` and impl `give()` to drop it.
2. **A2**: Reassign Buzzer from TIM2 to TIM15 channel 2 (available on MatekH743) or generate buzzer tones via software from the scheduler tick.
3. **A6**: Add `SCB::clean_dcache_by_address` before DMA TX and `SCB::invalidate_dcache_by_address` after DMA RX in `spi.rs`, `uart.rs`, and `adc.rs`. Also implement `configure_mpu()` to actually set SRAM4 as non-cacheable.
