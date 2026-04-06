# Panel Review #19 — Brandon Jones, AP_Motors Specialist
**Reviewer**: Brandon Jones  
**Domain**: Motor output, ESC communication, battery monitoring  
**Date**: 2026-04-02  
**Files reviewed**:
- `crates/meridian-mixing/src/lib.rs`
- `crates/meridian-mixing/src/spool.rs`
- `crates/meridian-hal/src/rc_output.rs`
- `crates/meridian-platform-stm32/src/pwm.rs`
- `crates/meridian-platform-stm32/src/adc.rs`
- `crates/meridian-failsafe/src/lib.rs` (battery consumer)
- `crates/meridian-arming/src/lib.rs` (battery consumer)
- `docs/identification_sensors_drivers.md` (AP_BattMonitor section)

---

## Section 1: 42-Frame Mixing Matrix

### Frame Count Verification

| Family | Count | Variants |
|--------|-------|----------|
| Quad | 14 | X, Plus, NYT-Plus, NYT-X, BF-X, BF-X-Rev, DJI-X, CW-X, V, H, V-Tail, A-Tail, Plus-Rev, Y4 |
| Hex | 5 | X, Plus, H, DJI-X, CW-X |
| Octa | 7 | Plus, X, V, H, I, DJI-X, CW-X |
| OctaQuad | 9 | Plus, X, V, H, CW-X, BF-X, BF-X-Rev, X-CoRot, CW-X-CoRot |
| Y6 | 3 | A, B, F |
| DodecaHexa | 2 | Plus, X |
| Deca | 2 | Plus, X |
| **Total** | **42** | |

Count matches claimed coverage. ArduPilot AP_MotorsMatrix covers the same 42 variants across the same 7 frame families. Motor count per family is correct (4, 6, 8, 8, 6, 12, 10).

### Normalization

`normalize()` scales RPY factors to ±0.5 and throttle to [0, 1.0] by dividing by per-axis maxima. This matches `AP_MotorsMatrix::normalise_rpy_factors()` exactly. Division-by-zero guards are present. Throttle is clamped to non-negative before taking the max, which correctly handles the co-rotating OctaQuad cases where throttle factors are non-uniform.

### Mixing Algorithm

`mix_compensated()` implements the ArduPilot `output_armed_stabilizing()` algorithm. The 11-step structure maps cleanly to ArduPilot lines 213-404:

1. Compensation gain applied to all axes — correct.
2. `throttle_thrust_best_rpy = min(throttle_avg_max, 0.5)` — correct.
3. Per-motor RP contribution computed before yaw — correct order.
4. Yaw headroom: iterates motors, computes available room per motor, takes the minimum — correct.
5. MOT_YAW_HEADROOM floor (0.200 default) — correct.
6. Thrust-boost motor-loss mode reduces headroom floor by 0.5 ratio — consistent with ArduPilot.
7. Yaw clamped to `[-yaw_allowed, +yaw_allowed]` — correct.
8. RPY combined; lost-motor zeroed when thrust_boost active — correct.
9. Proportional RPY scale computed from range, with secondary floor from throttle — correct.
10. Throttle windows `[thr_lo, thr_hi]`; bisects when infeasible — correct.
11. Final output `= throttle_out * throttle_factor + rpy[i]`, clamped to [0,1] — correct.

`spin_min` / `spin_max` from `MixerConfig` are referenced in the spool output but are NOT applied inside `mix_compensated()`. ArduPilot applies spin_min/spin_max scaling at the final output stage in `AP_MotorsMulticopter::output_min()` and `output_motor_mask()`. In Meridian this scaling must happen in the caller — if it is absent, motors will be commanded below spin_min during light throttle, which can cause motor dropouts. This is a latent integration defect, not a mixing-logic defect.

### CRITICAL BUG: octa_dji_x Duplicates octa_cw_x

`octa_dji_x()` and `octa_cw_x()` contain identical motor tables in the current code:

```
octa_dji_x (Meridian):
  (22.5, CCW), (67.5, CW), (112.5, CCW), (157.5, CW),
  (-157.5, CCW), (-112.5, CW), (-67.5, CCW), (-22.5, CW)

octa_cw_x (Meridian):
  (22.5, CCW), (67.5, CW), (112.5, CCW), (157.5, CW),
  (-157.5, CCW), (-112.5, CW), (-67.5, CCW), (-22.5, CW)
```

These are identical. The ArduPilot source at `AP_MotorsMatrix.cpp:935-948` defines DJI_X with motor indices ordered non-sequentially (1, 8, 7, 6, 5, 4, 3, 2 at angles 22.5, -22.5, -67.5, -112.5, -157.5, 157.5, 112.5, 67.5 respectively). When `add_motors()` inserts these by index, the resulting slot order differs from the CW-X definition. The Meridian `octa_dji_x()` currently produces the CW-X geometry for both frame types, which means any vehicle configured as `OCTAROTOR DJI_X` will receive incorrect motor mapping and will be uncontrollable.

**Correct Meridian `octa_dji_x()` should be:**
```rust
Self::from_angles(&[
    (22.5,   YAW_CCW),   // Motor 1 (index 1 -> slot 0)
    (-22.5,  YAW_CW),    // Motor 8 (index 8 -> slot 1)
    (-67.5,  YAW_CCW),   // Motor 7 (index 7 -> slot 2)
    (-112.5, YAW_CW),    // Motor 6 (index 6 -> slot 3)
    (-157.5, YAW_CCW),   // Motor 5 (index 5 -> slot 4)
    (157.5,  YAW_CW),    // Motor 4 (index 4 -> slot 5)
    (112.5,  YAW_CCW),   // Motor 3 (index 3 -> slot 6)
    (67.5,   YAW_CW),    // Motor 2 (index 2 -> slot 7)
])
```

The existing `test_all_frame_counts()` test only checks `motor_count() == 8` and would not catch this. A factor-comparison test against ArduPilot-derived expected values is required.

**Rating: NEEDS_WORK** (DJI-X octa frame uncontrollable; all 41 other frames are correct)

---

## Section 2: DShot Encoding

### Packet Structure

`fill_dshot_buffer()` in `crates/meridian-platform-stm32/src/pwm.rs` builds the DShot packet as follows:

```
throttle    = value & 0x07FF          (11 bits, correct)
telemetry   = 0u16                    (1 bit, hardcoded)
frame_no_crc = (throttle << 1) | telemetry   (12 bits)
crc = (frame_no_crc ^ (frame_no_crc >> 4) ^ (frame_no_crc >> 8)) & 0x0F  (4-bit XOR)
frame = (frame_no_crc << 4) | crc    (16 bits total)
```

This is the correct DShot frame structure: `[11-bit throttle][1-bit telemetry][4-bit CRC]`. The 16-bit encoding is correct.

The CRC computation: `(x ^ (x>>4) ^ (x>>8)) & 0x0F` where `x = frame_no_crc` — this XORs the bottom nibble, second nibble, and third nibble of the 12-bit pre-CRC word. This matches the DShot protocol specification and ArduPilot's CRC implementation in `RCOutput_bdshot.cpp`.

### Bit Ordering

Bits are serialized MSB-first: `(frame >> (DSHOT_FRAME_BITS - 1 - bit)) & 1`. DShot is MSB-first; this is correct.

### Bit Timing

Duty cycles are stored as percentage values (75 for '1', 37 for '0'). The DMA buffer stores these as `u32` percentage values rather than absolute timer-tick CCR values. The actual CCR values depend on the timer ARR (auto-reload register), which is set in `configure_timer()`. The `configure_timer()` body for DShot mode is marked `TODO` with commented-out register access. The timer period setup is entirely unimplemented; the percentage values in the DMA buffer will be written to CCR registers as raw counts, which is wrong. Without the ARR configured for the correct DShot bit period, the duty cycle values (37, 75) will produce random widths, not valid DShot signals.

The correct approach: store actual CCR ticks (`duty_percent * arr / 100`) computed once ARR is known. The current code cannot produce a valid DShot signal until `configure_timer()` is completed.

This is a platform implementation gap, not a protocol logic gap. The encoding logic itself is correct; the timer hardware setup is `TODO`.

### Bidirectional DShot

`OutputProtocol::BidirectionalDShot` is defined in the HAL trait but is not handled in `set_protocol()` match arm in `pwm.rs` — the match arm for `BidirectionalDShot` is absent, causing a compile-time non-exhaustive pattern warning or error (depending on `#[deny(non_exhaustive_patterns)]`). The `BDSHOT_RX_BUF` in SRAM4 is declared but the receive-path DMA switch-over (`start_dshot_dma()` comment references it) is not implemented.

### Telemetry Bit

`telemetry_bit` is hardcoded to 0. No mechanism exists to assert the telemetry bit on any channel. This means ESC telemetry cannot be requested even when using bidirectional DShot. The `get_esc_telemetry()` method returns `None` for all channels (rpm==0 and voltage_mv==0 initial state). ESC telemetry is effectively unavailable.

**Rating: NEEDS_WORK** (encoding logic correct; timer init TODO; bdshot match arm missing; telemetry bit stuck at 0)

---

## Section 3: Spool State Machine

### State Graph

```
ShutDown -> GroundIdle  (arm())
GroundIdle -> SpoolingUp  (request_spool_up(), after safe_time + idle_time + !spoolup_block)
SpoolingUp -> ThrottleUnlimited  (ramp reaches 1.0)
ThrottleUnlimited -> SpoolingDown  (request_spool_down())
SpoolingDown -> GroundIdle  (ramp reaches 0.0)
any -> ShutDown  (disarm(), immediate)
```

This matches ArduPilot `AP_MotorsMulticopter::output_logic()` exactly, including the `DESIRED_SPOOL_STATE` -> actual state mapping.

### Ramp Rate

`ramp += dt / spool_time` in SpoolingUp, `ramp -= dt / spool_time` in SpoolingDown. The `MINIMUM_SPOOL_TIME = 0.05s` prevents divide-by-zero. Fallback: spool-down with `spool_down_time = 0` uses the spool-up time, matching ArduPilot's `MOT_SPOOL_TIM_DN = 0` default behavior. Correct.

### Throttle Ceiling and Motor Floor

`throttle_ceiling()` returns:
- ShutDown: 0.0
- GroundIdle: `spin_arm` (0.10 default)
- SpoolingUp/Down: `spin_arm + ramp * (1 - spin_arm)` — linear interpolation from spin_arm to 1.0
- ThrottleUnlimited: 1.0

`motor_floor()` returns:
- ShutDown: 0.0
- GroundIdle: `spin_arm`
- SpoolingUp/SpoolingDown/ThrottleUnlimited: `spin_min`

This correctly models ArduPilot behavior: once spooling, motors hold at spin_min minimum, not spin_arm.

### Safe Time, Idle Time, Spoolup Block

All three gates in `request_spool_up()` are correctly ordered and tested. The spoolup block callback is a fn pointer (`SpoolupBlockFn = fn() -> bool`) rather than a trait object, which limits it to static functions. This is fine for embedded. Tests cover all three delay mechanisms and the block callback. Test for `idle_timer` reset on `SpoolState::SpoolingDown -> GroundIdle` transition is present.

### Missing: SpoolingDown Does Not Re-Check Spoolup Block

When in SpoolingDown, if `request_spool_up()` is called (i.e., the FC changes its mind mid-descent), the machine has no path to reverse. ArduPilot handles the desired-vs-actual state differently: the desired state can be changed at any time, and `output_logic()` evaluates transitions every tick. Meridian's explicit `request_*` model means a SpoolingDown vehicle cannot re-arm upward without going all the way to GroundIdle first. This is not a crash risk but it does mean a drop-throttle-then-re-throttle scenario (e.g., catch-spool in guided mode) will have an extra 0.5-second delay that ArduPilot does not have. Acceptable for first flight.

### Test Coverage

7 tests: lifecycle, safe_time, disarm_immediate, minimum_spool_time, spool_down_defaults_to_up, idle_time_delay, spoolup_block. All relevant edge cases covered. Tick arithmetic (`210 * 0.0025 = 0.525s > 0.5s`) is correct with proper float margin.

**Rating: FLIGHT_SAFE** (known minor behavioral difference in SpoolingDown re-spool; not safety-critical)

---

## Section 4: Battery Monitoring — Complete Absence

### Current State

AP_BattMonitor coverage: **0 of 25 backends. 0% implementation.**

The ADC hardware layer (`meridian-platform-stm32/src/adc.rs`) reads raw ADC counts, converts to pin voltage (0–3.3V), and applies IIR filtering. It stops there. The critical missing layer is the scale factor application. The MatekH743 battery voltage divider has a scale of 11.0x — a 12.6V pack appears as 1.145V on the ADC pin. Without the scale factor, the system reads ~1.1V instead of ~12.6V.

The failsafe system (`meridian-failsafe/src/lib.rs`) and arming system (`meridian-arming/src/lib.rs`) both accept `battery_voltage: f32` and `battery_remaining_pct: u8` as caller-supplied parameters. Nothing in the codebase connects the ADC output to these parameters. The battery voltage passed to failsafe is whatever the caller provides — with no BattMonitor, the caller has no source for it. In practice, the failsafe battery check would receive a stale 0.0 or a hardcoded value, making it non-functional.

There is also no current integration. Even if voltage were wired correctly, there is no mAh consumed tracking, no capacity-remaining estimate, and no low-mAh failsafe threshold enforcement.

### Is the Absence Acceptable?

**No. This is a GROUNDED condition for any real flight.**

The specific risks:

1. **Cell over-discharge**: LiPo cells damaged or fire risk below ~3.3V/cell. Without voltage monitoring, there is no low-voltage RTL or land trigger. A 4S pack at 14.8V nominal will reach 13.2V (3.3V/cell) with no automated response.

2. **Thrust compensation disabled by default**: `VoltageCompensation` and `apply_thrust_curve_with_comp()` exist in `lib.rs` and are correctly implemented. But `VoltageCompensation::update()` requires a real voltage reading. Without one, `compensation_gain` passed to `mix_compensated()` will be 1.0 (no compensation). On a pack sagging from 16.8V to 14.0V, thrust output drops ~30% without compensation — the vehicle will drift and lose altitude authority.

3. **Arming check non-functional**: `check_prearm()` compares `state.battery_voltage` against `config.min_battery_voltage` (default 10.0V). With no BattMonitor feeding this value, the field will be 0.0, which is below 10.0V, meaning the arming check either always fails (blocking arm entirely) or is disabled (if the check flag is cleared to bypass it). Either outcome is wrong.

### Minimum Battery Monitoring for First Flight

The minimum viable implementation requires four things, none of which currently exist as a complete unit:

**1. Scale-factor application** (BattMonitor_Analog equivalent):
```rust
let battery_voltage = adc_pin_voltage * scale_factor;  // e.g., 11.0 for MatekH743
```
The scale comes from board config (`BoardConfig::battery_voltage_scale`). The board config crate has the field but nothing reads it into the voltage path.

**2. Current integration** (or synthetic current):
```rust
let current_amps = adc_current_pin_voltage * current_scale;
mah_consumed += current_amps * dt * (1000.0 / 3600.0);
```
`AP_BattMonitor_Synthetic_Current` estimates current from throttle if no current sensor exists. Meridian has neither.

**3. Battery state struct** to hold filtered voltage, current, mAh, and cell count:
```rust
pub struct BatteryState {
    pub voltage_v: f32,
    pub current_a: f32,
    pub mah_consumed: f32,
    pub cell_count: u8,
}
```

**4. Integration into the arming/failsafe call sites**: both currently pass the battery voltage as a free parameter. A `BatteryState` reference needs to be routed there.

Total implementation estimate: ~200 lines for a minimal `BattMonitor_Analog` + `Synthetic_Current` equivalent. The ADC hardware plumbing is already present (DMA, IIR filter). The missing piece is purely the application layer.

### What Happens Without ESC Telemetry

ESC telemetry (BLHeli32/AM32 bidirectional DShot or UART telemetry) is the only way to get per-motor current, RPM, and ESC temperature without a dedicated power module. Since `get_esc_telemetry()` always returns `None` (telemetry bit hardcoded to 0, no receive path), the following capabilities are unavailable:

- **Per-motor current**: no way to detect a blocked/failed motor until it goes silent entirely.
- **Motor RPM**: cannot verify motors are spinning before takeoff. The spoolup block callback (`SpoolupBlockFn`) is designed to use RPM data to confirm ESC readiness — without RPM feedback, this check cannot be implemented and the callback is always false.
- **ESC temperature**: cannot detect thermal runaway before ESC shuts down.
- **Desync detection**: bidirectional DShot RPM feedback is the primary mechanism for detecting ESC desync (a common cause of crashes). Without it, a desynced motor produces incorrect thrust silently.

For first flight on a bench-tethered or low-stakes vehicle, the absence of ESC telemetry is survivable. For any autonomous or high-altitude operation, it is unacceptable.

---

## Summary Ratings

| Area | Rating | Primary Issue |
|------|--------|---------------|
| 42-frame mixing (41 of 42) | FLIGHT_SAFE | — |
| octa_dji_x frame | GROUNDED | Identical to octa_cw_x; wrong motor mapping |
| Mixer desaturation algorithm | FLIGHT_SAFE | Faithful to ArduPilot |
| Thrust linearization | FLIGHT_SAFE | Correct quadratic inversion and LP-filtered voltage comp |
| DShot packet encoding | NEEDS_WORK | Logic correct; timer init TODO; bdshot arm missing |
| Spool state machine | FLIGHT_SAFE | Minor re-spool delay vs ArduPilot; not safety-critical |
| Battery monitoring | GROUNDED | Complete absence; failsafe and thrust comp non-functional |
| ESC telemetry | NEEDS_WORK | Telemetry bit stuck at 0; receive path absent |

---

## Required Fixes Before Any Real Flight

### P0 — Blocking (must fix before powering motors)

1. **Fix `octa_dji_x()`** — reorder motor table to match ArduPilot index-based insertion order. Add a factor-validation test comparing against expected normalized values from ArduPilot.

2. **Implement minimal BattMonitor_Analog** — apply scale factor from board config to ADC pin voltage. Wire result to `ArmingState::battery_voltage` and `FailsafeManager::check_all()` call sites. Without this, battery voltage is always 0V, the arming check fires false, and low-voltage RTL never triggers.

### P1 — Required before autonomous operation

3. **Implement synthetic current** (`BattMonitor_Synthetic_Current`) — estimate current from throttle output, integrate to mAh. Required for mAh-based low-battery failsafe.

4. **Complete `configure_timer()` for DShot mode** — set ARR based on `dshot_timer_period()`, convert duty percentages to CCR ticks. Without this, DShot produces garbage on the wire.

5. **Add `OutputProtocol::BidirectionalDShot` match arm** in `set_protocol()` to fix the non-exhaustive pattern.

### P2 — Recommended before production

6. **Telemetry bit control** — expose a per-channel telemetry-request API. Add bidirectional DShot receive path in `start_dshot_dma()`.

7. **Wire ESC RPM to spoolup block** — implement `SpoolupBlockFn` using bidirectional DShot RPM to confirm each motor is spinning before advancing from GroundIdle.

8. **Apply spin_min/spin_max inside `mix_compensated()` or at call site** — prevent motor dropouts at low throttle.

---

*Reviewed by Brandon Jones. All ArduPilot source line references verified against `AP_MotorsMatrix.cpp` and `RCOutput.cpp` at time of review.*
