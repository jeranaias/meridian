# Final Control System & Motor Mixing Review
**Date:** 2026-04-02  
**Reviewer:** Automated audit (automated side-by-side diff)  
**Scope:** Pre-hardware deployment final review  
**Sources:** ArduPilot (master) vs Meridian crates `meridian-control` and `meridian-mixing`

---

## Summary Verdict

| Area | Status | Risk |
|---|---|---|
| pid.rs | MINOR (FF architecture) | Low |
| attitude_controller.rs | MINOR (angle boost, pd_scale) | Low |
| rate_controller.rs | **CONCERN** (landed multiplier defaults) | **Medium** |
| position_controller.rs | MINOR (corner accel, shape_pos_vel_accel API divergence) | Low |
| sqrt_controller.rs | MINOR (sqrt_controller_accel signature) | Low |
| tecs.rs | MATCH | None |
| notch_filter.rs | MINOR (Q formula) | Low |
| mixing/lib.rs | **CONCERN** (hex_plus ordering) | **Medium** |
| mixing/spool.rs | MATCH | None |
| heli/lib.rs | MATCH | None |

**Two items require resolution before first flight.** Nothing is an outright crash-level BUG in isolation, but the landed-multiplier default and hex_plus motor ordering are operationally incorrect.

---

## Item-by-Item Findings

---

### 1. pid.rs — MINOR

**What was checked:** 3 LP filters (filt_T_hz, filt_E_hz, filt_D_hz), SMAX adaptive slew, integrator with limit flag, D_FF, PDMX, get_pid_info.

**Findings:**

**1a. FF term included in `update_full()` return value — architectural divergence.**  
ArduPilot's `AC_PID::update_all()` returns `P + D + I` only. FF and D_FF are stored in `_pid_info` but deliberately excluded from the return. The caller (`AC_AttitudeControl_Multi::rate_controller_run_dt`) retrieves FF separately via `get_ff()` and passes it to `motors.set_roll_ff()`, which then adds it to `_roll_in_ff` before the mixing stage.

Meridian's `update_full()` returns `pd_output + integrator + ff_term + d_ff_term` — FF is baked in. This is internally consistent as long as Meridian's rate controller does not also add FF a second time upstream. Verified: Meridian's `RateController::update()` does not double-add FF. The end result is numerically identical for a single control path. However, any future code that calls `update_full()` and separately applies a feedforward must account for this difference.

**Verdict:** MINOR. Current integration is correct. Document the convention difference.

**1b. `integrator` zeroed when `ki == 0` — Meridian does not replicate this.**  
ArduPilot's `update_i()` sets `_integrator = 0.0f` when `_ki` is zero (`else { _integrator = 0.0f; }`). Meridian skips the integrator update when `ki == 0` (the `if ki > 0` condition is implicit via the multiply) but does not zero the existing stored value. If ki is dynamically set to 0.0 mid-flight, Meridian will retain a stale integrator that immediately resumes when ki is re-enabled. This is a minor behavioral difference; for typical flight where gains don't change it has no effect.

**Verdict:** MINOR. Add an explicit zero when `ki <= 0`.

**1c. All other elements — MATCH.**  
- 3 LP filters: identical IIR formula, identical ordering (T, E, D).  
- SMAX slew modifier: formula `smax / (smax + 1.5 * (peak - smax))`, 0.1 floor — MATCH.  
- PDMX: correct scaling — MATCH.  
- `get_pid_info()` struct fields — MATCH.  
- `relax_integrator()` formula `i += (target - i) * dt/(dt + tc)` — MATCH.  
- `reset_filter()` / `reset()` behavior — MATCH.

---

### 2. attitude_controller.rs — MINOR

**2a. Quaternion error convention — MATCH.**  
`q_err = target.inverse() * current` with axis-angle extraction using sign check on `q_err.w`. This matches ArduPilot exactly. The comment "CRITICAL FIX" in the source code is accurate.

**2b. Input shaping — MATCH.**  
Per-axis clamp `(raw_rate - shaped_rate).clamp(-accel * dt, +accel * dt)` is correct acceleration-limited trajectory shaping.

**2c. Angle boost — MINOR.**  
ArduPilot's `get_throttle_boosted()` is in `AP_MotorsMulticopter`, not `AC_AttitudeControl`. The `inverted_factor = (10 * cos_tilt).clamp(0,1)` formula is ArduPilot's. Meridian's implementation is correct.  

However, ArduPilot applies angle boost at the `set_throttle_out()` call site and the result is passed through `_throttle_in` before mixing. Meridian exposes `angle_boost()` as a standalone method that the caller must invoke and apply. Verify that every call site in Meridian's flight modes applies the boost before sending throttle to the mixer. If any mode skips this call, that mode will sink under tilt.

**2d. `pd_scale` parameter — not in Meridian.**  
ArduPilot's `update_all()` accepts a `pd_scale` parameter (used by throttle gain boost to multiply P and D outputs proportionally). Meridian's `update_full()` has no equivalent. The `update_throttle_gain_boost()` function exists in `rate_controller.rs` but returns a multiplier the caller must apply manually — unlike ArduPilot which passes it into the PID directly. Verify call sites use the returned boost factor.

**2e. Thrust vector heading, throttle-RPY mix — MATCH.**  
`THRUST_ERROR_ANGLE_RAD = 0.5236` (30 deg) matches `AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD`. FF attenuation formula is identical. ThrottleRpyMix slew rates (up=2.0, down=0.5) match ArduPilot.

**2f. `get_throttle_avg_max()` — MATCH.**  
`0.5 + 0.5 * mix` is correct.

---

### 3. rate_controller.rs — CONCERN

**3a. Default rate PID gains — MATCH.**  

| Parameter | ArduPilot | Meridian |
|---|---|---|
| roll/pitch P | 0.135 | 0.135 |
| roll/pitch I | 0.135 | 0.135 |
| roll/pitch D | 0.0036 | 0.0036 |
| roll/pitch IMAX | 0.5 | 0.5 |
| roll/pitch filt_T_hz | 20.0 | 20.0 |
| roll/pitch filt_D_hz | 20.0 | 20.0 |
| roll/pitch filt_E_hz | 0.0 | 0.0 |
| yaw P | 0.180 | 0.180 |
| yaw I | 0.018 | 0.018 |
| yaw D | 0.0 | 0.0 |
| yaw IMAX | 0.5 | 0.5 |
| yaw filt_E_hz | 2.5 | 2.5 |

All gains exact match.

**3b. Landed gain multiplier defaults — CONCERN.**  

ArduPilot default for `LAND_R_MULT`, `LAND_P_MULT`, `LAND_Y_MULT` is **1.0** (no reduction on ground).  
Meridian's `LandedGainMultipliers::default()` returns **0.5** for all three axes.

This means whenever Meridian's flight controller calls `apply_landed_gains(true)`, it will halve P and D gains on all axes while the vehicle is on the ground. ArduPilot with default parameters does nothing (multiplier = 1.0). This will cause sluggish attitude response on the ground and may prevent normal arming behavior if any ground attitude hold is active.

**This must be changed to 1.0 before hardware testing.** Set `roll: 1.0, pitch: 1.0, yaw: 1.0` as the default in `LandedGainMultipliers::default()`.

**3c. Throttle gain boost — MINOR.**  
`THR_G_BOOST_THRESH = 0.5` and `THR_G_BOOST_P_MAX = 2.0` are plausible but not verified against a specific ArduPilot parameter (ArduPilot stores this as `_throttle_gain_boost` AP_Float with runtime tuning). The implementation logic (slew rate check, linear boost scaling) is structurally correct.

---

### 4. position_controller.rs — MINOR

**4a. Sqrt-controller shaping pipeline — MATCH (with caveats).**  
The 4-step pipeline (forward integration, shape_pos_vel_accel, corner accel reduction, velocity PID) matches ArduPilot's AC_PosControl structure.

**4b. Corner acceleration — MINOR.**  
Meridian's `limit_accel_corner_xy()` uses a simple angle-threshold approach: if heading change > 45 deg, scale by `1/sqrt(2)`. ArduPilot's actual `limit_accel_corner_xy()` in `control.cpp` uses a velocity-direction decomposition that prioritizes cross-track braking over along-track acceleration (a more physically correct approach). Meridian's simplification is conservative and safe but will be less precise in sharp waypoint turns.

**4c. `shape_pos_vel_accel` API divergence — MINOR.**  
ArduPilot's current `shape_pos_vel_accel()` signature takes `(pos_desired, vel_desired, accel_desired, pos, vel, accel, vel_min, vel_max, accel_min, accel_max, jerk_max, dt, limit_total)` — note it takes **both** `pos_desired` and a separate `pos` (current position), and uses `pos_error = pos_desired - pos` internally. Meridian's version takes `pos_error` directly (pre-subtracted), which is correct but diverges from AP's actual API. This matters only if comparing directly for debugging.

Additionally, ArduPilot's `sqrt_controller_accel()` now takes 5 arguments: `(error, rate_cmd, rate_state, p, second_ord_lim)`, while Meridian's version takes 4: `(error, p, second_ord_lim, vel)` — the `rate_cmd` argument (the output of sqrt_controller for the same error) is missing. ArduPilot uses `rate_cmd` to detect direction (returns 0 if `rate_cmd * rate_state <= 0`, meaning moving away from target). Meridian's version does not have this guard. In practice this only affects the correction bias term and does not cause instability, but it will produce slightly different behavior when the vehicle is overshooting.

**4d. Hover throttle estimator — MATCH.**  
`tc = 10.0s`, `accel_threshold = 1.0 m/s^2`, LP filter formula, clamp to `[0.1, 0.8]` — all consistent with ArduPilot.

**4e. Vertical velocity limits — MATCH.**  
`[-5.0, 1.5] m/s` (5 m/s climb, 1.5 m/s descent) matches typical copter defaults.

---

### 5. sqrt_controller.rs — MINOR

**5a. `sqrt_controller()` — MATCH.**  
Hybrid linear/sqrt formula, linear region boundary `second_ord_lim / (p*p)`, overshoot clamp — exact match.

**5b. `sqrt_controller_accel()` — MINOR (signature divergence).**  
As noted above, the ArduPilot version takes `rate_cmd` as an explicit parameter (the pre-computed output of sqrt_controller) and uses it both for the directional guard and as the denominator in the sqrt-region branch. Meridian recomputes it internally. The directional guard is missing: ArduPilot returns 0 when `rate_cmd * rate_state <= 0` (vehicle moving away from target). Meridian does not check this, so the accel bias can be applied even when the vehicle is decelerating away from the target. This is a low-risk behavioral difference.

**5c. `inv_sqrt_controller()`, `stopping_distance()`, `shape_accel()`, `shape_vel_accel()`, `update_vel_accel()`, `update_pos_vel_accel()` — MATCH.**  
All formulas verified correct.

**5d. `shape_pos_vel_accel()` — see item 4c above.**

---

### 6. tecs.rs — MATCH

All key elements verified:

- **Energy computation:** `STE = g*h + 0.5*v^2`, `SEB = g*h - 0.5*v^2` — MATCH.
- **Underspeed protection:** flag triggered at `v < airspeed_min * 0.9`, forces `throttle = throttle_max` and `pitch <= 0` — MATCH.
- **Bad descent detection:** `height_rate < -max_sink_rate && height_estimate < height_demand` — MATCH.
- **Speed weighting (`seb_w`, `ste_w`):** both weights computed and both contribute to pitch demand — MATCH (the comment "FIX" in source is accurate; this was a prior bug that has been corrected).
- **Flare mode:** progressive sink rate ramp, reduced time constant, idle throttle — structurally correct.
- **Integrator limits:** throttle `[-0.3, 0.3]`, pitch `[-0.2, 0.2]` — consistent with ArduPilot defaults.
- **Default parameters:** all within normal ArduPilot ranges.

---

### 7. notch_filter.rs — MINOR

**7a. Biquad coefficient formula — MATCH.**  
`b0 = 1 + alpha*A^2`, `b1 = -2*cos(omega)`, `b2 = 1 - alpha*A^2`, `a0 = 1 + alpha`, `a2 = 1 - alpha` — exact match with ArduPilot's `NotchFilter.cpp`.

**7b. 5% slew limiter — MATCH.**  
`FREQ_SLEW_LIMIT = 0.05` matches `NOTCH_MAX_SLEW = 0.05f`. ArduPilot constrains to `[freq * 0.95, freq / 0.95]` while Meridian uses `delta.clamp(-freq * 0.05, +freq * 0.05)`. These are equivalent for small steps but differ slightly for large jumps (ArduPilot's upper bound is `freq * (1/0.95) = freq * 1.0526`, Meridian's is `freq * 1.05`). The difference is 0.26% and has no operational impact.

**7c. Q formula — MINOR.**  
ArduPilot computes Q using the octave formula:
```
octaves = log2(center / (center - bw/2)) * 2
Q = sqrt(2^octaves) / (2^octaves - 1)
```
Meridian uses the simpler approximation: `Q = center / bandwidth`.

For typical notch configs (center=100 Hz, bw=20 Hz): ArduPilot Q=4.74, Meridian Q=5.0 — a 5.6% difference. For wider notches (center=80, bw=40): ArduPilot Q=1.71, Meridian Q=2.0 — a 16.7% difference. The wider the notch, the more the Q underestimates, making the actual notch slightly narrower than requested. This means vibration rejection at the notch edges will be slightly weaker than expected. Not a safety issue but will require slight bandwidth adjustment for accurate notch width matching.

**7d. 16-harmonic bitmask, composite modes (Double/Triple/Quintuple) — MATCH.**  
All offset factors verified:
- Double: ±10% — MATCH.
- Triple: center ±15% (0.85, 1.0, 1.15) — MATCH.
- Quintuple: 0.83, 0.915, 1.0, 1.085, 1.17 — MATCH.

**7e. Filter application (series chain) — MATCH.**  
ArduPilot and Meridian both apply notch filters in series (each filter processes the output of the previous).

---

### 8. mixing/lib.rs — CONCERN

**8a. Frame angle sets — MATCH (all 42 frames).**  
All verified:  
- Quad: X, Plus, NYT-Plus, NYT-X, BF-X, BF-X-Rev, DJI-X, CW-X, V, H, VTail, ATail, Plus-Rev, Y4 — angles and yaw factors MATCH.
- Hex: X, H, DJI-X, CW-X — MATCH. **Hex-Plus — see 8b.**
- Octa: Plus, X, V, H, I, DJI-X, CW-X — MATCH.
- OctaQuad: Plus, X, V, H, CW-X, BF-X, BF-X-Rev, X-CoRotating, CW-X-CoRotating — MATCH.
- Y6: A, B, F — MATCH.
- DodecaHexa: Plus, X — MATCH.
- Deca: Plus, X — MATCH.

**V-tail and A-tail angles verified:**  
`add_motor(MOT_1, 60, 60, 0)` → roll=cos(150°)=-0.8660, pitch=cos(60°)=0.5 — matches Meridian exactly.  
`add_motor(MOT_2, 0, -160, CW)` → roll=cos(90°)=0.0, pitch=cos(-160°)=-0.9397 — matches Meridian exactly.

**8b. Hex-Plus motor index ordering — CONCERN.**  

ArduPilot motor index order for Hex-Plus (from `AP_MotorsMatrix.cpp`):
```
Index 0: angle=  0°, CW     (test order 1)
Index 1: angle=180°, CCW    (test order 4)
Index 2: angle=-120°, CW    (test order 5)
Index 3: angle= 60°, CCW    (test order 2)
Index 4: angle=-60°, CCW    (test order 6)
Index 5: angle=120°, CW     (test order 3)
```

Meridian's `hex_plus()`:
```
Index 0: angle=  0°, CW
Index 1: angle= 60°, CCW    ← AP index 3
Index 2: angle=120°, CW     ← AP index 5
Index 3: angle=180°, CCW    ← AP index 1
Index 4: angle=-120°, CW    ← AP index 2
Index 5: angle=-60°, CCW    ← AP index 4
```

Meridian uses the physically natural "clockwise around the frame" ordering. ArduPilot uses an irregular ordering driven by internal `testing_order` assignments. The motor angles and yaw factors are all correct; what differs is which physical ESC connector maps to which index.

**Impact:** If a Hex-Plus aircraft is wired for the ArduPilot index mapping (which is the standard wiring guide for ArduPilot hex frames) and flashed with Meridian's firmware, motors will receive the wrong mix signal. Motor 1 (front) would receive the back-motor command, motor 2 (front-right) would receive the front command, etc. This will cause immediate loss of control.

**Resolution required:** Either change `hex_plus()` to match AP ordering exactly, or ensure airframe documentation specifies Meridian's own ordering and all hex airframes are wired accordingly before first flight.

**8c. 12-step mix pipeline — MATCH.**  
Steps traced against `AP_MotorsMatrix::output_armed_stabilizing()`:
- Compensation gain applied to RPY and throttle — MATCH.
- `throttle_thrust_best_rpy = MIN(0.5, throttle_avg_max)` — MATCH.
- Per-motor yaw headroom calculation — MATCH.
- `yaw_allowed_min = yaw_headroom * 0.001` — MATCH.
- Thrust boost yaw floor at 50% for motor loss — MATCH.
- RPY proportional scale when `rpy_high - rpy_low > 1.0` — MATCH.
- Secondary scale floor `-throttle_avg_max / rpy_low` — MATCH.
- Throttle adjustment clamped to `[thr_lo, thr_hi]` — MATCH.
- Final output `throttle_out * throttle_factor + rpy[i]` — MATCH.

**8d. Limit flags — MATCH.**  
`rpy`, `throttle_upper`, `throttle_lower`, `yaw_limited` — all set in the correct conditions.

**8e. `compensation_gain` and `apply_thrust_curve()` — MATCH.**  
Quadratic inversion formula `(-b + sqrt(b^2 + 4*a*t)) / (2*a)` — MATCH. Voltage compensation LP filter at 0.5 Hz — MATCH.

---

### 9. mixing/spool.rs — MATCH

All 5 states present and transitions correct:
- `ShutDown → GroundIdle` on arm — MATCH.
- `GroundIdle → SpoolingUp` with `safe_time` gate, `idle_time` gate, and `spoolup_block` guard — MATCH.
- `SpoolingUp → ThrottleUnlimited` at ramp=1.0 — MATCH.
- `ThrottleUnlimited → SpoolingDown` — MATCH.
- `SpoolingDown → GroundIdle` at ramp=0.0 — MATCH.
- `MINIMUM_SPOOL_TIME = 0.05s` — MATCH.
- Default `spool_up_time = 0.5s`, `spin_arm = 0.10`, `spin_min = 0.15` — MATCH.
- `spool_down_time = 0` fallback to `spool_up_time` — MATCH.
- `throttle_ceiling()` linear interpolation during spool — MATCH.
- `motor_floor()` returns `spin_min` during flight, `spin_arm` at ground idle — MATCH.

---

### 10. heli/lib.rs — MATCH

**10a. Swashplate types — MATCH.**  
All 6 types present: H3Generic, H1, H3_140, H3_120, H4_90, H4_45. Servo counts correct (3 for H3, 4 for H1/H4). Phase angles:
- H3_120: 0°, 120°, 240° — MATCH.
- H3_140: 0°, 140°, 220° — MATCH.
- H4_90: 0°, 90°, 180°, 270° — MATCH.
- H4_45: 45°, 135°, 225°, 315° — MATCH.

**10b. CCPM mixing formula — MATCH.**  
`servo[i] = pitch * cos(angle[i]) + roll * sin(angle[i]) + collective_centered * weight[i]` — MATCH with AP_MotorsHeli_Swash.cpp.

**10c. RSC governor — MATCH.**  
PID on RPM error with collective feedforward, ramp state machine, autorotation interlock — all structurally correct. Default `ramp_time = 5s` matches `AP_MOTORS_HELI_RSC_RAMP_TIME = 1s` with note that the ArduPilot default is 1 second, Meridian uses 5 seconds. This is conservative (slower spool-up) and safe for initial testing, but should be tuned down before operational use.

**10d. Autorotation — MATCH.**  
`set_autorotation(true)` zeroes throttle immediately. RSC interlock behavior matches ArduPilot.

**10e. HeliDual/HeliQuad — not present in Meridian heli crate.**  
These are complex multi-rotor helicopter configurations. If Meridian does not support them, this is a known scope limitation, not a defect. Confirm not in scope for initial deployment.

---

## Action Items Before First Flight

### REQUIRED (block first flight)

1. **Fix landed gain multiplier defaults** (`rate_controller.rs` line 55-57).  
   Change `roll: 0.5, pitch: 0.5, yaw: 0.5` to `roll: 1.0, pitch: 1.0, yaw: 1.0` to match ArduPilot defaults. The 0.5 value will cause sluggish ground response and may interfere with armed idle.

2. **Fix Hex-Plus motor ordering** (`mixing/lib.rs` `hex_plus()` function).  
   Reorder the motor array to match ArduPilot's index assignment:
   ```
   (  0, YAW_CW)   // index 0
   (180, YAW_CCW)  // index 1
   (-120, YAW_CW)  // index 2
   ( 60, YAW_CCW)  // index 3
   (-60, YAW_CCW)  // index 4
   (120, YAW_CW)   // index 5
   ```
   Or document and enforce a different wiring standard consistently, and update all hex airframe build documents before any hex is flown.

### RECOMMENDED (fix before operational use)

3. **Zero integrator when `ki == 0`** (`pid.rs`).  
   Add `if self.gains.ki <= 0.0 { self.integrator = 0.0; }` to prevent stale integrator resumption on gain change.

4. **Add `sqrt_controller_accel` directional guard** (`sqrt_controller.rs` line ~82).  
   Add the ArduPilot check: return 0.0 when `rate_cmd * vel <= 0` (moving away from target). Prevents reverse bias in the correction term during overshoot.

5. **Document the FF architecture difference** (`pid.rs`, `rate_controller.rs`).  
   Meridian's `update_full()` includes FF in the return value. ArduPilot does not. Any future code integrating Meridian PIDs must not separately add FF again.

6. **Verify angle boost is applied at all throttle call sites.**  
   Search all flight mode implementations for `update_altitude()` and `update()` in `PositionController`. Each must call `angle_boost()` and apply the result before sending throttle to the mixer.

7. **Tune notch Q for wide notch applications.**  
   Meridian's `Q = center/bandwidth` underestimates Q by up to ~17% for wide notches (bandwidth > 30% of center). For narrow notches (< 10% bandwidth) the error is negligible. If flying with the default `base_freq=80, bandwidth=40` config, the actual notch will be slightly narrower than requested.

8. **RSC ramp time** (`heli/lib.rs`).  
   Default `ramp_time_s = 5.0s` vs ArduPilot's 1.0s. Conservative and safe, but note for heli pilots expecting AP behavior.

---

## Numerical Constants Summary

| Constant | ArduPilot | Meridian | Match? |
|---|---|---|---|
| THRUST_ERROR_ANGLE_RAD | 0.5236 (30 deg) | 0.5236 | YES |
| MINIMUM_SPOOL_TIME | 0.05 s | 0.05 s | YES |
| MOT_SPIN_MIN default | 0.15 | 0.15 | YES |
| MOT_SPIN_ARM default | 0.10 | 0.10 | YES |
| MOT_SPOOL_TIME default | 0.5 s | 0.5 s | YES |
| MOT_YAW_HEADROOM default | 200 | 200 | YES |
| ATC_ANG_RLL_P default | 4.5 | 4.5 | YES |
| ATC_INPUT_TC default | 0.15 s | 0.15 s | YES |
| ATC_ACCEL_RP_MAX default | 19.2 rad/s^2 | 19.2 | YES |
| ATC_ACCEL_Y_MAX default | 4.71 rad/s^2 | 4.71 | YES |
| ATC_RAT_RLL_P default | 0.135 | 0.135 | YES |
| ATC_RAT_RLL_D default | 0.0036 | 0.0036 | YES |
| ATC_RAT_YAW_FLTE default | 2.5 Hz | 2.5 Hz | YES |
| LAND_R/P/Y_MULT default | **1.0** | **0.5** | **NO** |
| NOTCH_MAX_SLEW | 0.05 (5%) | 0.05 (5%) | YES |
| Notch Q formula | octave-based | center/bw approx | APPROX |
| PSC_JERK_XY default | 17.0 m/s^3 | 17.0 | YES |
| Hover throttle default | 0.39 | 0.39 | YES |
| GRAVITY | 9.80665 | 9.80665 | YES |

---

*This document covers all 10 specified check items. Two items require resolution before first flight.*
