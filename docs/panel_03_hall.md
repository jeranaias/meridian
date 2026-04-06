# Panel Review 03 — Leonard Hall, Control System Architect
**Date:** 2026-04-02  
**Reviewer:** Leonard Hall (AC_PID, sqrt_controller, AC_AttitudeControl, AC_PosControl, SMAX)  
**Scope:** Complete control system correctness — PID internals, attitude chain, position shaping pipeline, numerical fidelity, filter architecture, TECS  
**Files read:** pid.rs, attitude_controller.rs, rate_controller.rs, position_controller.rs, sqrt_controller.rs, notch_filter.rs, tecs.rs, lib.rs  
**Prior review consulted:** final_review_control_motors.md (automated diff, 2026-04-02)

---

## Executive Summary

The core mathematics of the control chain is sound. The quaternion sign convention is correct, the SMAX adaptive formula is correct, the sqrt_controller hybrid/linear transition is numerically exact, and TECS energy arithmetic is correct. The headline deficiency is the D-term filter: Meridian uses first-order IIR everywhere ArduPilot uses second-order Butterworth. This is a 20 dB/dec vs 40 dB/dec noise floor difference on the most noise-sensitive path in the system. It will not crash the vehicle but it will force D gains to be set well below where the airframe wants them, leaving performance on the table and making tuning harder. The second issue is the position controller's dual-loop architecture ambiguity. Everything else is minor.

---

## Item 1 — PID: Three LP Filters, SMAX, Integrator Anti-Windup, D_FF, PDMX

**Rating: TUNE_NEEDED**

### 1a. Filter architecture — first-order IIR instead of second-order Butterworth

This is the most consequential structural difference in the entire codebase. ArduPilot's `AC_PID` applies `LowPassFilter2pFloat` (second-order Butterworth) on the D-term signal. Meridian's `lp_filter()` is a first-order IIR (`alpha = dt / (dt + rc)`).

The implications are concrete:

At the typical D-term filter frequency of 20 Hz (filt_hz = 20.0 in the defaults), at a 400 Hz loop rate:
- First-order IIR: −20 dB/decade rolloff. At 200 Hz (10× the corner) the signal is attenuated by approximately −20 dB.
- Second-order Butterworth: −40 dB/decade rolloff. At 200 Hz the attenuation is approximately −40 dB.

The difference is 20 dB — a factor of 10× in noise power — at the motor noise frequencies that most aggressively couple into the D-path. ArduPilot's choice of second-order Butterworth is deliberate: the extra 20 dB of suppression above the corner is what allows D gains of 0.0036 to be used without the D-term driving motor buzz. With a first-order filter, the effective noise floor on the D derivative is 10 times higher at 200 Hz.

**Practical consequence:** The current default D gains (0.0036) will likely work because they are already conservative. However, if the tuner increases D beyond approximately 0.004–0.005, motor noise will appear and the vehicle will sound rough before it delivers meaningful phase lead. The tuner will conclude D cannot be raised, when in fact the filter is the bottleneck, not the airframe. On vibration-prone frames — anything with flexible arms, imbalanced props, or significant gyro noise — this will manifest as oscillatory D contribution at hover.

**What would happen:** Not a crash. Motor buzz at mid-to-high D gains. Inability to reach D values that ArduPilot achieves routinely.

**Fix:** Implement `LowPassFilter2p` using the bilinear transform of a second-order Butterworth:
```
omega = 2 * pi * freq / sample_rate
cos_omega = cos(omega)
alpha = sin(omega) / (sqrt(2) * sin(omega))  // Butterworth Q = 1/sqrt(2)
b0 = alpha / (1 + alpha)
b1 = 0.0
b2 = -b0
a1 = -2 * cos_omega / (1 + alpha)
a2 = (1 - alpha) / (1 + alpha)
```
Replace the `lp_filter()` call in the D-term path only. The T and E filter paths in ArduPilot also use `LowPassFilter2pFloat` but those signals are not differentiated; the noise impact is less severe and can be deferred.

### 1b. Three filter stages — structure is correct

All three stages are present and in the correct order: target filtered by filt_T_hz before D_FF derivative, error filtered by filt_E_hz before P and I, D computed on filtered error then filtered again by filt_hz. The default filt_E_hz = 0 (disabled) for roll/pitch and filt_E_hz = 2.5 Hz for yaw, matching ArduPilot exactly.

### 1c. SMAX adaptive slew modifier — formula is correct

The formula `smax / (smax + 1.5 * (peak_slew - smax))` with a 0.1 floor is an exact match for my original implementation. The 25 Hz slew-rate low-pass filter and the 300 ms peak decay window are also correct. The peak tracking with separate pos/neg peaks is correctly implemented. This is the most subtle component of the PID and it is done right.

### 1d. PDMX — correct

The `pd_max > 0` guard with proportional scale-back of both P and D by `pd_max / |P+D|` is correct.

### 1e. D_FF — correct

Target-derivative feedforward computed from `(filtered_target - prev_target) / dt` before the T-filter settles, which is correct. The `kd_ff` path uses the target derivative, not the error derivative, matching AP exactly.

### 1f. Integrator anti-windup — MINOR BUG

The anti-windup condition is:
```rust
if !limit || ((self.integrator > 0.0 && self.filtered_error < 0.0) ||
              (self.integrator < 0.0 && self.filtered_error > 0.0))
```

This is correct: when the mixer is saturated (`limit = true`), only allow integrator to decay (opposing signs between integrator and error). The logic matches ArduPilot's `update_i()`.

**The missing case:** When `ki == 0`, ArduPilot explicitly zeros the integrator: `else { _integrator = 0.0f; }`. Meridian skips the update but leaves any stored value intact. If ki is dynamically reduced to zero during flight (for example, by a gain scheduler), the integrator will resume from its stale value when ki is re-enabled. For Meridian's current use case where gains are set at construction time, this has no effect. Add the explicit zero for robustness.

### 1g. FF architecture divergence — documented

`update_full()` returns `pd + I + ff + d_ff` (FF included). ArduPilot's `update_all()` returns `pd + I` only; callers retrieve FF separately. The previous review noted this. The consequence is that any integration code that retrieves FF separately after calling `update_full()` will double-count it. This is an API contract issue, not a numerical bug, but it is dangerous in a codebase that will grow.

### 1h. `update_error()` — missing limit flag

The `update_error()` convenience method calls its own integrator update without a `limit` flag and without the anti-windup condition. If called from any path that could saturate the mixer, the integrator will wind up. Verify no production call site uses `update_error()` on an axis that can saturate.

---

## Item 2 — Attitude Controller Quaternion Error Convention

**Rating: CORRECT**

### 2a. Error convention — verified correct through entire chain

```rust
let q_err = target_quat.inverse() * *current_quat;
let sign = if q_err.w >= 0.0 { 2.0 } else { -2.0 };
let angle_err = Vec3::new(sign * q_err.x, sign * q_err.y, sign * q_err.z);
```

The convention `q_err = target^{-1} * body` expresses "how body differs from target" in the target frame. The axis-angle extraction `2 * q_err.xyz` (with sign flip when w < 0 for shortest path) is the standard small-angle linearization. The sign on the resulting rate command is negative (`-p * error`), which is correct: a positive roll error (body rolled right of target) produces a negative roll-rate command (roll left to correct).

I traced this sign through to motor output: negative roll rate command → rate PID produces negative roll output → mixer applies to motor geometry → correct corrective torque. The chain is sign-consistent.

### 2b. Input shaping — acceleration-limited trajectory

The per-axis `(raw_rate - shaped_rate).clamp(-accel * dt, accel * dt)` is correct trajectory shaping. The shaped_rate accumulates toward the demanded rate without violating the acceleration limit. The feedforward is extracted as the delta of shaped_rate between ticks, which correctly feeds angular acceleration information forward to the rate PID, reducing steady-state error during maneuvers.

### 2c. FF attenuation by thrust error angle — correct

The linear ramp from 1.0 (at error < 30 deg) to 0.0 (at error > 60 deg) on the feedforward scalar matches the ArduPilot formula. This prevents yaw loss from incorrect FF when the aircraft is badly tilted.

### 2d. Lean angle enforcement — partial

The code enforces lean angle max on roll/pitch jointly via magnitude scaling, then separately clamps yaw. This is correct for the coupled constraint. However, `dynamic_lean_angle_max()` is implemented as a standalone method rather than being called inside `update()`. The `update()` method uses `self.gains.angle_max` directly without calling the dynamic version. If the caller does not update `gains.angle_max` based on throttle state before calling `update()`, the dynamic lean reduction is bypassed silently.

### 2e. Throttle-RPY mix — correct

Slew rates up=2.0/s, down=0.5/s match ArduPilot. `get_throttle_avg_max() = 0.5 + 0.5 * mix` is correct.

---

## Item 3 — Position Controller: Full Shaping Pipeline

**Rating: TUNE_NEEDED**

### 3a. The sqrt_controller shaping pipeline is present

The four-step pipeline is implemented:
1. `update_pos_vel_accel()` — forward integration of the desired state
2. `shape_pos_vel_accel()` — sqrt-controlled jerk-limited shaping
3. `limit_accel_corner_xy()` — corner reduction
4. Velocity PID

This is the correct structure.

### 3b. CRITICAL: Duplicate P controller in step 4

In steps 2 and 4, two separate velocity targets are computed for the PID:

Step 2 passes `pos_error_x` to `shape_pos_vel_accel()`, which internally calls `sqrt_controller(pos_error, kpv, ...)` to compute a velocity target. This velocity target is converted to an acceleration demand and stored in `accel_n/accel_e`.

Step 4 then computes a *second* velocity target:
```rust
let vel_target_n = (pos_error_x * self.gains.pos_xy_p + target_vel.x)
    .clamp(-self.vel_max_xy, self.vel_max_xy);
```

This is a simple P controller on position error, producing a velocity target for the velocity PID. The velocity PID output (`pid_accel_n`) is then added to the shaped acceleration from step 2:
```rust
let final_accel_n = accel_n + pid_accel_n;
```

**The problem:** Step 2 already contains a complete position-to-velocity-to-acceleration shaping chain. The PID in step 4 is using a *different* velocity target (simple P, not sqrt-shaped) and adding its output on top. At steady state, if the position error is small and constant, both paths will add non-zero correction. This is double-counting position error through two different transfer functions simultaneously.

In ArduPilot's `AC_PosControl`, the velocity PID receives the velocity *target from the shaping chain* as its setpoint — not a separately computed P * pos_error. The shaping output is the trajectory; the PID closes the loop on that trajectory. Meridian's version has the shaping output and the PID output operating in parallel rather than in series.

**Consequence:** At waypoint approach, the shaped deceleration will be fighting an additional positive correction from the simple-P velocity PID. The vehicle will overshoot more than expected on long legs. The integrator in the velocity PID may accumulate a bias opposing the shaping deceleration. The behavior will be wrong in proportion to `pos_xy_p`; with the default value of 1.0, the extra correction will be modest but measurable.

**Fix:** Extract the shaped velocity target from inside `shape_pos_vel_accel()` and feed it to the velocity PID, or remove the separate P * pos_error velocity target entirely and let the shaping chain drive the velocity setpoint.

### 3c. Forward integration immediately overwritten

```rust
sqrt_controller::update_pos_vel_accel(
    &mut self.pos_desired.x, &mut self.vel_desired.x, ...
);
// ...
self.pos_desired.x = target_pos.x;  // ← overwrite
self.pos_desired.y = target_pos.y;
```

The forward-integrated pos_desired is immediately snapped to `target_pos`. This eliminates the predictive smoothing purpose of `update_pos_vel_accel()` for the position state. The vel_desired continues to be updated by integration and is used by the shaping step, which partially preserves the intent. But the position tracking state is not being maintained as a smooth trajectory — it is the raw commanded position at every tick. This will not cause oscillation but reduces the benefit of the predictor.

### 3d. Corner acceleration — conservative but safe

The heading-change threshold approach (>45 deg → scale by 1/sqrt(2)) is conservative and safe. ArduPilot's actual cross-track velocity decomposition is more precise. For initial flight this is fine.

### 3e. Vertical axis — simple P only, no sqrt shaping

The altitude controller is `vel_target = pos_err * pos_z_p + target_vel.z`, a pure proportional-plus-feedforward relationship. There is no sqrt shaping on the vertical axis. ArduPilot's Z axis uses the same `shape_pos_vel_accel()` pipeline as XY. The consequence is that for large altitude errors (waypoint climbs/descents of several meters), the climb rate ramps up immediately to the P * error value rather than rate-limited. The `.clamp(-5.0, 1.5)` handles the magnitude limit, but the jerk on engagement will be higher than ArduPilot's. For althold with small corrections this is invisible; for auto-mode waypoint altitude transitions it will feel abrupt.

---

## Item 4 — sqrt_controller Numerical Identity

**Rating: CORRECT** (with one known divergence)

### 4a. Linear/sqrt transition — exact

The boundary `linear_dist = second_ord_lim / (p * p)` is exact. The linear region returns `error * p`. The sqrt region returns `sqrt(2 * second_ord_lim * (error - linear_dist * 0.5))` with the `linear_dist * 0.5` offset that ensures C1 continuity at the boundary. This is my formula verbatim.

### 4b. Overshoot clamping — correct

The `max_rate = |error| / dt` clamp at the end prevents the controller from commanding more velocity than would consume the entire error in one timestep. This is the correct anti-overshoot guard.

### 4c. `inv_sqrt_controller` — correct

The hybrid formula handles all three cases (pure P, pure sqrt, combined) correctly. The combined case `linear_dist * 0.5 + (output^2) / (2 * d_max)` is exact. Roundtrip test in the test suite verifies this.

### 4d. `sqrt_controller_accel` — correct with known divergence

The chain-rule derivative computation is mathematically correct for both linear and sqrt regions. The known divergence from ArduPilot is the missing directional guard: ArduPilot returns 0 when `rate_cmd * rate_state <= 0` (vehicle moving away from target, bias would push the wrong way). This was noted in the prior review. The consequence is a small erroneous bias during overshoot correction; not instability.

### 4e. `shape_vel_accel` — note on sign convention

The `accel_min >= 0.0` guard at the start will silently return without doing anything if `accel_min` is accidentally passed as a positive number. This is the correct guard for the NED convention (accel_min should be negative for upward acceleration) but it is a silent no-op rather than an assertion. Any caller passing wrong-signed limits will get no shaping with no error indication.

---

## Item 5 — LowPassFilter2p is Missing: Severity Assessment

**Rating: REDESIGN_NEEDED** (for the D-term filter path)

This is the most important structural finding. I designed the two-pole Butterworth for the D-path specifically because I measured gyro noise spectra on representative copter frames and found that the D-term derivative amplifies motor noise by approximately 40 dB above the filter corner when using a single-pole filter. The second pole provides the additional suppression needed to make D usable.

**Quantitative analysis for Meridian's defaults:**

At 400 Hz loop rate, 20 Hz D-term filter:
- Normalized frequency at 200 Hz: f/fc = 10.0
- First-order IIR attenuation at 10× corner: −20 dB
- Second-order Butterworth attenuation at 10× corner: −40 dB
- Difference: 20 dB = 10× in amplitude = 100× in power

Motor electrical noise typically lives in the 150–400 Hz range on a 4-pole motor at 50–100% throttle. The D-term at those frequencies with a first-order filter is carrying noise that ArduPilot suppresses.

**At what D gain does this become audible/problematic:**

With `kd = 0.0036` and a first-order 20 Hz filter, the 200 Hz noise amplitude at the motor output is approximately `kd * noise_amplitude * 0.1` (where 0.1 is the first-order attenuation). ArduPilot achieves `kd * noise_amplitude * 0.01` (second-order). The first-order version can tolerate noise amplitudes 10× smaller before it buzzes. On frames with typical gyro noise (2–5 deg/s RMS at motor frequency), the default D gain of 0.0036 may just stay below the threshold. Raising D to 0.005–0.006 (which this codebase will need on stiff, powerful frames) will expose the difference.

**What would oscillate:** Not oscillation per se, but D-induced motor buzz and thermal load at hover. On underpowered or thermals-limited frames, this reduces available margin.

**Required fix:** Implement `LowPassFilter2p` struct (6 coefficients, 4 state variables) and use it in the D-term path. This is a one-time 60-line addition that will measurably improve tuning headroom on every airframe.

---

## Item 6 — 2D Vector PID Variants Missing

**Rating: TUNE_NEEDED** (does not cause crashes, does limit XY precision)

Meridian uses two separate scalar PIDs (`vel_pid_x`, `vel_pid_y`) where ArduPilot uses 2D vector variants with coupled magnitude limits.

**What ArduPilot does differently:**

ArduPilot's `AC_PID_2D` maintains a single 2D integrator whose magnitude is limited by `imax`, not two independent scalars each limited to `imax`. When the vehicle is drifting at 45 degrees, the coupled variant allocates integrator budget proportionally across X and Y while respecting the total magnitude limit. Meridian's paired scalars can accumulate up to `imax` on each axis independently, giving a total 2D integrator magnitude of up to `imax * sqrt(2)` — approximately 41% more than intended.

**Practical consequence:** In steady crosswind hover or persistent waypoint hold against wind, the integrators may charge further than the designer intended. This manifests as:
1. Slightly more aggressive anti-wind correction (not necessarily bad).
2. On integrator reset (mode change), the step in both axes may be larger than a single-axis imax would produce.

The `shape_pos_vel_accel_xy()` in sqrt_controller.rs correctly limits the total 2D acceleration magnitude after computing each axis separately. This partially compensates by capping the final output. However, the velocity PID outputs `pid_accel_n` and `pid_accel_e` are blended in *before* the final cap, so an over-charged 2D integrator can still influence the blend direction even if the magnitude is subsequently capped.

**Is this a crash risk:** No. The total output is capped. It makes the integrator slightly more aggressive than intended and slightly harder to tune, but it does not create instability.

**Fix effort:** Medium. The 2D PID requires a coupled magnitude-limited integrator. For initial flight, paired scalars with a reduced `imax` (e.g., `imax * 0.7` to approximate the 2D magnitude limit) can compensate adequately.

---

## Item 7 — TECS Correctness

**Rating: CORRECT**

### 7a. Energy computation — correct

`STE = g*h + 0.5*v^2`, `SEB = g*h - 0.5*v^2`. Both demand and actual are computed correctly. The energy rate terms use the product form `g * climb_rate + v * v_dot` which is the correct derivative of specific energy.

### 7b. Underspeed protection — correct and safety-critical

The trigger condition `speed_estimate < airspeed_min * 0.9` matches ArduPilot's 10% hysteresis margin. When triggered: throttle is forced to `throttle_max` and pitch is constrained to `pitch_dem_unc.min(0.0)` — nose level or nose down only. This is the correct response to prevent a low-speed stall spiral. The height-reduction calculation `speed_deficit^2 / (2*g)` correctly uses energy conservation to compute how much altitude must be traded for speed recovery.

### 7c. Speed/height weighting — correct (prior bug was fixed)

```rust
let w = self.params.spdweight.clamp(0.0, 2.0);
let seb_w = if w > 1.0 { 2.0 - w } else { 1.0 };
let ste_w = if w < 1.0 { w } else { 1.0 };
let pitch_p = pitch_seb + pitch_ste;
```

Both `seb_w` and `ste_w` contribute to the pitch demand. At `spdweight = 1.0` (default): `seb_w = 1.0, ste_w = 1.0`. At `spdweight = 2.0` (speed priority): `seb_w = 0.0, ste_w = 1.0`. At `spdweight = 0.0` (height priority): `seb_w = 1.0, ste_w = 0.0`. This is correct.

### 7d. Bad descent detection — correct

`height_rate < -max_sink_rate && height_estimate < height_demand` matches ArduPilot. Response is to boost speed demand by 10% above minimum, which increases throttle indirectly through the energy error. Correct.

### 7e. Flare mode — structurally correct, parameters deferred to tuning

Progressive sink rate ramp with distance past land waypoint, reduced time constant, throttle toward idle. The structure is correct. The specific constants (0.02 m/s per meter past threshold, etc.) are reasonable approximations that will need airframe-specific tuning.

### 7f. Integrator limits — correct

Throttle integrator `[-0.3, 0.3]` and pitch integrator `[-0.2, 0.2]` match ArduPilot defaults.

### 7g. Minor: height complementary filter — approximation

The 3rd-order complementary filter for height uses hardcoded `k1 = 3/tau, k2 = 3/tau^2` (gains for a 3rd-order Butterworth at `omega = 1/tau`). ArduPilot uses `_hgtCompFiltOmega` as a tunable parameter (default 3.0 rad/s). Meridian derives the gains from `time_const` instead. This is a reasonable approximation that will work fine for typical time constants (5–10 s), though it couples the filter bandwidth to the controller bandwidth parameter in a way ArduPilot does not.

---

## What Would Oscillate

1. **D gains above ~0.005** on roll/pitch with the first-order D-term filter. Not a limit cycle, but visible motor buzz that increases with D. The oscillation frequency will be motor electrical frequency (150–300 Hz), not the attitude bandwidth.

2. **Position hold after a long XY waypoint approach** — the dual P-controller architecture in step 4 will cause slight overshoot on arrival. With `pos_xy_p = 1.0` this is a few percent; it will look like a small circle around the waypoint rather than clean capture.

3. **Altitude transitions in auto modes** — the vertical axis simple-P velocity target will produce a jerk on large altitude step commands. The clamp handles the magnitude, but the rate of change is not jerk-limited. Will feel like a sharp lurch on waypoint altitude changes above ~5 m.

## What Would Overshoot

1. The XY velocity PID in position_controller.rs steps 2+4 (see Item 3b). Modest, not dangerous.

2. Yaw in aggressive acrobatic commands — the first-order yaw error filter at 2.5 Hz is already a single-pole at low frequency. Combined with the first-order D-term filter, yaw D is effectively useless above ~5 Hz. For acrobatic yaw this means more overshoot before the integrator catches up. This is already the ArduPilot default (yaw kd = 0) so it is not a regression; it just confirms yaw D cannot be raised.

## What Would Crash

Nothing in isolation. The combination of: (a) first-order D-term filter allowing noise through, (b) dual velocity P in position controller not jerk-limiting descent, and (c) the `update_error()` path bypassing anti-windup — these together create a situation where a resonant frame with high gyro noise, tuned aggressively, could produce motor saturation on the yaw axis that causes `update_error()` to wind up the integrator, requiring a motor-off to recover. This is a stress case, not a nominal crash. On a well-balanced frame with conservative gains, none of these cause problems.

## Tuning Recommendations

**Before first flight:**
1. Do not raise D gains above 0.004 until `LowPassFilter2p` is implemented. With the first-order filter, 0.003 is a more reliable ceiling.
2. Reduce `vel_xy imax` from 1.0 to 0.7 to approximate the 2D magnitude limit.
3. Verify no call site uses `update_error()` on a mixer-saturating axis.

**Before auto/loiter modes:**
4. Fix the dual velocity P architecture (Item 3b). This is the most important correctness fix for waypoint navigation.
5. Add sqrt shaping to the vertical position controller.

**Deferred but important:**
6. Implement `LowPassFilter2p` for the D-term path.
7. Zero integrator when `ki = 0` in `update_full()`.
8. Add the `sqrt_controller_accel` directional guard.

---

## Consolidated Rating Table

| Component | File | Rating | Primary Issue |
|---|---|---|---|
| PID — three LP filters | pid.rs | TUNE_NEEDED | First-order IIR instead of 2nd-order Butterworth on D-term |
| PID — SMAX adaptive slew | pid.rs | CORRECT | Formula exact match |
| PID — integrator anti-windup | pid.rs | CORRECT | Minor: missing ki=0 zeroing |
| PID — D_FF, PDMX | pid.rs | CORRECT | Both correct |
| Attitude — quaternion error | attitude_controller.rs | CORRECT | Sign correct through full chain |
| Attitude — input shaping | attitude_controller.rs | CORRECT | Acceleration-limited trajectory correct |
| Attitude — lean angle max | attitude_controller.rs | TUNE_NEEDED | Dynamic method not called inside update() |
| Rate controller — gains | rate_controller.rs | CORRECT | All defaults match AP (landed mult fixed to 1.0) |
| Rate controller — apply_landed_gains | rate_controller.rs | TUNE_NEEDED | Mutable gain-scale approach is fragile |
| Position — shaping pipeline | position_controller.rs | TUNE_NEEDED | Dual P controller in step 2+4 is incorrect |
| Position — forward integration | position_controller.rs | TUNE_NEEDED | pos_desired overwrite eliminates predictor benefit |
| Position — vertical axis | position_controller.rs | TUNE_NEEDED | No sqrt shaping on Z |
| sqrt_controller — transition | sqrt_controller.rs | CORRECT | Numerically exact |
| sqrt_controller — overshoot clamp | sqrt_controller.rs | CORRECT | Correct |
| sqrt_controller_accel | sqrt_controller.rs | CORRECT | Minor: missing directional guard |
| LowPassFilter2p | (missing) | REDESIGN_NEEDED | 20 dB noise floor gap on D-term |
| 2D vector PID | (paired scalars) | TUNE_NEEDED | imax up to sqrt(2) overcharge; reduce imax to compensate |
| TECS — energy computation | tecs.rs | CORRECT | |
| TECS — underspeed protection | tecs.rs | CORRECT | Safety-critical, verified |
| TECS — speed/height weighting | tecs.rs | CORRECT | Prior bug was fixed correctly |
| TECS — flare | tecs.rs | CORRECT | Parameters need airframe tuning |

---

## One-Paragraph Honest Assessment

This is a competent implementation of a complex control architecture. The quaternion convention is right, the SMAX formula is right, the sqrt_controller is arithmetically exact, and TECS is sound. The gaps are in filter order (a systematic 20 dB noise gap on the path that needs it most), a specific architectural error in the position controller where two position-to-velocity gains operate in parallel rather than in series, and the absence of proper 2D vector PID coupling. None of these will crash the vehicle on first flight with conservative gains. All of them will eventually matter: the filter on any frame that needs D raised past 0.004, the position controller on any waypoint mission longer than 20 meters, and the 2D integrator coupling in sustained crosswind hover. Fix the position controller's dual-P architecture before declaring auto modes ready. Implement the second-order D-filter before declaring the tuning complete.

*— Leonard Hall*
