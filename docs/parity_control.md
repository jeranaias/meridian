# Control System Parity Audit — Meridian vs ArduPilot

**Date:** 2026-04-02
**ArduPilot source:** `libraries/AC_PID/`, `libraries/AC_AttitudeControl/`, `libraries/AP_TECS/`, `libraries/Filter/NotchFilter*`, `libraries/AP_Math/control.cpp`
**Meridian source:** `crates/meridian-control/src/`

---

## PID Controller

**Files compared:** `AC_PID.cpp` vs `pid.rs`

### 3 LP Filters (target / error / derivative)

**ArduPilot:** Three independent first-order IIR filters on target (`_filt_T_hz`), error (`_filt_E_hz`), and derivative (`_filt_D_hz`). Alpha computed via `calc_lowpass_alpha_dt(dt, hz)`.

**Meridian:** All three filters present (`filt_t_hz`, `filt_e_hz`, `filt_hz`). Alpha computed as `dt / (dt + rc)` where `rc = 1/(2π·hz)`. This is mathematically identical to ArduPilot's `calc_lowpass_alpha_dt`.

**VERDICT: MATCH.** On filter-reset (first sample), ArduPilot and Meridian both seed from the raw input rather than filtering.

---

### SMAX Slew Rate Limiter

**ArduPilot:** Uses `SlewLimiter`, a sophisticated adaptive gain-reduction system. It:
1. Low-pass filters `(sample - last_sample) / dt` at 25 Hz to get the running slew rate.
2. Tracks the peak positive and negative slew rates over a 300 ms window.
3. Decays peaks when outside the window (time-constant `slew_rate_tau`, default 1.0 s).
4. Stores up to `N_EVENTS=2` consecutive exceedance events per direction.
5. Computes a `mod` multiplier in range `[0.1, 1.0]` using:
   `mod = smax / (smax + 1.5 * (slew - smax))`
6. Applies `mod` to **P and D separately** before summing: `P *= mod; D *= mod`.
7. The modifier tracks the oscillation frequency to distinguish mode-change spikes from genuine oscillations.

**Meridian:** Uses a simple hard clamp:
```rust
let max_delta = self.gains.smax * dt;
let prev_pd = self.prev_output - self.integrator;
let delta = pd_raw - prev_pd;
let clamped = delta.clamp(-max_delta, max_delta);
```
This limits the *rate of change of the PD sum* — it is a direct delta clamp, not a gain-reduction multiplier.

**VERDICT: DIVERGE — CRITICAL.** Meridian's implementation is a naive delta clamp, not ArduPilot's adaptive SlewLimiter. ArduPilot's version:
- Reduces PD gain proportionally (does not hard-clip output)
- Operates on time-windowed peak detection
- Distinguishes sustained oscillation from transient spikes
- Has a tau parameter for gain relaxation rate
- Maintains P and D as separate scalable terms

The Meridian SMAX will overdamp during mode changes and underdamp during oscillation compared to ArduPilot.

---

### Integrator Anti-Windup

**ArduPilot:**
```cpp
void AC_PID::update_i(float dt, bool limit, float i_scale) {
    if (!limit || ((is_positive(_integrator) && is_negative(_error)) ||
                   (is_negative(_integrator) && is_positive(_error)))) {
        _integrator += ((float)_error * _ki) * i_scale * dt;
        _integrator = constrain_float(_integrator, -_kimax, _kimax);
    }
}
```
The `limit` flag (passed from the motor mixer when outputs are saturated) allows the integrator to **shrink but not grow** when the vehicle is against an actuator limit. `i_scale` allows partial integral scaling.

**Meridian:**
```rust
self.integrator += self.gains.ki * self.filtered_error * dt;
self.integrator = self.integrator.clamp(-self.gains.imax, self.gains.imax);
```

**VERDICT: DIVERGE.** Meridian has no `limit` flag and no `i_scale`. When the mixer saturates, ArduPilot's integrator stops growing (preventing integral wind-up); Meridian keeps integrating. This will cause overshoot after saturated states (heavy maneuvers, wind limits, takeoff from tight space).

---

### Feedforward Path

**ArduPilot:** Two feedforward terms:
- `FF = target * _kff` — proportional to the raw (unfiltered) target
- `DFF = _target_derivative * _kdff` — proportional to the rate of change of the *filtered* target

Both are returned separately in `get_pid_info()` but summed in the total output.

**Meridian:**
```rust
let ff_term = self.gains.ff * target;
```
Only the basic FF term. No derivative feedforward (`D_FF` / `kdff`).

**VERDICT: DIVERGE.** Meridian is missing `D_FF` (derivative feedforward on target). ArduPilot uses this to proactively command attitude rate changes when the target is moving, reducing lag.

---

### PD Sum Maximum (PDMX)

**ArduPilot:** Parameter `PDMX` (`_kpdmax`). When positive, limits `|P + D|` with a scaling clamp:
```cpp
if (PD_sum_abs > _kpdmax) {
    const float pd_limit_scale = _kpdmax / PD_sum_abs;
    P_out *= pd_limit_scale;
    D_out *= pd_limit_scale;
    _pid_info.PD_limit = true;
}
```

**Meridian:** No `PDMX` / PD sum maximum.

**VERDICT: DIVERGE.** Missing feature. PDMX prevents aggressive P+D from saturating the actuator during initial transients.

---

### `set_integrator()` for Mode Transitions

**ArduPilot:** `set_integrator(float integrator)` — clamps to `[-imax, +imax]` and sets `_flags._I_set` for logging. Also has `relax_integrator(float target, float dt, float tc)` which exponentially decays toward a target integrator value using a time constant.

**Meridian:** `set_integrator(val)` — present and clamped. No `relax_integrator()`.

**VERDICT: PARTIAL.** `set_integrator()` is present. `relax_integrator()` is missing.

---

### `get_pid_info()` for Logging

**ArduPilot:** `get_pid_info()` returns an `AP_PIDInfo` struct with: `target`, `actual`, `error`, `P`, `I`, `D`, `FF`, `DFF`, `Dmod`, `slew_rate`, `limit`, `PD_limit`, `reset`, `I_term_set`.

**Meridian:** No `get_pid_info()`. The `PidController` exposes only `get_integrator()` and `set_integrator()`. No logging struct.

**VERDICT: MISSING.** No PID telemetry logging support. Required for tuning via GCS and dataflash logs.

---

### Notch Filters on PID Signals

**ArduPilot:** Two optional notch filters per PID controller: `_notch_T_filter` (on target) and `_notch_E_filter` (on error). Configured via `set_notch_sample_rate()` and AP_Filter API. Applied *before* the LP filters to reduce shot noise.

**Meridian:** No per-PID notch filter support.

**VERDICT: MISSING.**

---

## Attitude Controller

**Files compared:** `AC_AttitudeControl.cpp` + `AC_AttitudeControl_Multi.cpp` vs `attitude_controller.rs`

### Quaternion Error Computation

**ArduPilot:** `thrust_heading_rotation_angles()` decomposes attitude error into a thrust-plane error (roll/pitch) and a heading error (yaw) separately. The full quaternion error path is:
```
q_err = _attitude_target.inverse() * attitude_body
```
Note the order: it is `target^-1 * current` (body expressed in target frame), **not** `target * current^-1`.

**Meridian:**
```rust
let q_err = *target_quat * current_quat.inverse();
```
This computes `target * current^-1`, expressing the target in the body frame.

**VERDICT: DIVERGE — CRITICAL.** The quaternion multiplication order is **reversed** relative to ArduPilot. ArduPilot uses `target.inverse() * body`; Meridian uses `target * body.inverse()`. Both representations encode the same attitude error, but the axis-angle extracted from `q_err` will be expressed in a different frame. The subsequent P-gain multiplication and feedforward rotation must be consistent with the chosen convention. Meridian's axis-angle extraction (`sign * q_err.{x,y,z}`) is the standard small-angle approximation and is self-consistent with `target * body^-1`, but it differs from ArduPilot's frame convention. This will produce correct behavior only if the rest of the pipeline uses the same frame convention. **This must be verified against the rate controller input frame.**

---

### Input Shaping (Acceleration-Limited Trajectory)

**ArduPilot:** Uses `attitude_command_model()` which implements a proper jerk-limited trajectory generator with independent per-axis acceleration limits (`_accel_roll_max_degss`, `_accel_pitch_max_degss`, `_accel_yaw_max_degss`), per-axis rate maximums (`_ang_vel_roll_max_degs`, etc.), and a time constant (`_input_tc`). Internally: if `_rate_bf_ff_enabled`, the trajectory is integrated in the body frame and a body-frame angular velocity feedforward is also generated.

**Meridian:**
```rust
let max_delta = self.gains.accel_max * dt;
let dx = (raw_rate.x - self.shaped_rate.x).clamp(-max_delta, max_delta);
```
Uses a single scalar `accel_max` for all three axes. No per-axis acceleration or rate limits.

**VERDICT: DIVERGE.** Meridian's acceleration limiting is correct in structure but uses a single shared `accel_max` for all three axes. ArduPilot has per-axis roll, pitch, and yaw acceleration limits — important because yaw authority is typically much lower. Meridian also does not propagate a shaped angular acceleration (only shaped angular rate) so the feedforward into the rate loop is first-derivative only.

---

### Angle Boost

**ArduPilot (`get_throttle_boosted`):**
```cpp
float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
float cos_tilt_target = cosf(_thrust_angle_rad);
float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);
float throttle_out = throttle_in * inverted_factor * boost_factor;
```
Uses `_thrust_angle_rad` (actual thrust vector angle from vertical) not the Euler pitch/roll angles. Applies an `inverted_factor` that smoothly reduces boost to zero as the vehicle approaches 90° tilt (preventing boost runaway past 60°). Cap is implicit via `constrain(cos_tilt_target, 0.1, 1.0)`.

**Meridian:**
```rust
let cos_tilt = libm::cosf(roll) * libm::cosf(pitch);
let cos_min = libm::cosf(self.gains.angle_max);
if cos_tilt > cos_min {
    1.0 / cos_tilt
} else {
    1.0 / cos_min
}
```
Uses Euler roll and pitch angles directly. Hard-caps at `angle_max`. Does not have the `inverted_factor` (10x cos_tilt smooth rolloff near 90°).

**VERDICT: DIVERGE.** Meridian's angle boost will behave incorrectly near 90° tilt — it will maintain maximum boost rather than smoothly rolling off to zero. Safe for vehicles limited to small angles, but wrong for aggressive aerobatics.

---

### Lean Angle Max Enforcement

**ArduPilot:** Lean angle limit is dynamic — `update_althold_lean_angle_max(throttle_in)` computes the maximum allowable lean based on available thrust headroom: `acos(throttle_in / (0.8 * thr_max))`. This decreases angle max at low throttle.

**Meridian:** Static `angle_max` clamp. No dynamic lean angle limit based on throttle.

**VERDICT: DIVERGE.** Static is safe but lacks the safety feature that reduces maximum lean when near throttle limits.

---

### Rate Feedforward from Shaped Trajectory

**ArduPilot:** Body-frame angular velocity feedforward from the shaped trajectory is explicitly computed and summed with the error-derived rate correction in `attitude_controller_run_quat()`. The feedforward is scaled by `_feedforward_scalar` which smoothly reduces to zero when thrust error exceeds `AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD` to prevent yaw authority loss.

**Meridian:** `update()` returns `rate_target` which includes shaping but the feedforward scalar (thrust error attenuation) is absent. No `_thrust_error_angle_rad` tracking.

**VERDICT: DIVERGE.** The thrust-error feedforward attenuation is missing.

---

### Thrust Vector Heading Decomposition (`input_thrust_vector_heading()`)

**ArduPilot:** Full `input_thrust_vector_heading()` family: accepts a 3D thrust vector and decomposes it via `attitude_from_thrust_vector()` to produce the roll/pitch quaternion, then appends yaw. Used by loiter, position hold, and auto modes.

**Meridian:** No `input_thrust_vector_heading()` or equivalent.

**VERDICT: MISSING.** The thrust-vector input path is absent. Meridian works from Euler angle targets only.

---

### Throttle-RPY Mix (`_throttle_rpy_mix`)

**ArduPilot:** `_throttle_rpy_mix` (range 0.1–0.9) blends pilot throttle vs attitude-control throttle priority. Slews at `+2.0/s` to increase and `-0.5/s` to decrease. `set_throttle_mix_max()` prioritizes attitude control, `set_throttle_mix_min()` prioritizes throttle (e.g., landing). Used by `get_throttle_avg_max()` to prevent attitude control from dropping the vehicle.

**Meridian:** No `_throttle_rpy_mix` state or mixing logic.

**VERDICT: MISSING.** This affects landing stability — without it, the mixer cannot de-prioritize attitude control at low throttle.

---

### Landed Gain Multipliers

**ArduPilot:** `LAND_R_MULT`, `LAND_P_MULT`, `LAND_Y_MULT` — reduce roll/pitch/yaw gains when landed to prevent ground oscillation.

**Meridian:** Not present.

**VERDICT: MISSING.**

---

## Rate Controller

**Files compared:** `AC_AttitudeControl_Multi.h/.cpp` (rate loop) vs `rate_controller.rs`

### Default Gains

| Axis | Parameter | ArduPilot Default | Meridian | Match? |
|------|-----------|-------------------|----------|--------|
| Roll P | `ATC_RAT_RLL_P` | 0.135 | 0.135 | YES |
| Roll I | `ATC_RAT_RLL_I` | 0.135 | 0.135 | YES |
| Roll D | `ATC_RAT_RLL_D` | 0.0036 | 0.0036 | YES |
| Roll IMAX | `ATC_RAT_RLL_IMAX` | 0.5 | 0.5 | YES |
| Roll filt_T_hz | `FLTT` | 20.0 | 0.0 | **NO** |
| Roll filt_D_hz | `FLTD` | 20.0 | 20.0 | YES |
| Roll filt_E_hz | `FLTE` | 0.0 | 0.0 | YES |
| Pitch P | `ATC_RAT_PIT_P` | 0.135 | 0.135 | YES |
| Pitch I | `ATC_RAT_PIT_I` | 0.135 | 0.135 | YES |
| Pitch D | `ATC_RAT_PIT_D` | 0.0036 | 0.0036 | YES |
| Yaw P | `ATC_RAT_YAW_P` | 0.180 | 0.180 | YES |
| Yaw I | `ATC_RAT_YAW_I` | 0.018 | 0.018 | YES |
| Yaw D | `ATC_RAT_YAW_D` | 0.0 | 0.0 | YES |
| Yaw IMAX | `ATC_RAT_YAW_IMAX` | 0.5 | 0.5 | YES |
| Yaw filt_E_hz | `ATC_RAT_YAW_FLTE` | **2.5** | **20.0** | **NO** |
| Yaw filt_T_hz | — | 20.0 | 0.0 | **NO** |

**VERDICT: TWO DIVERGENCES.**
1. Roll/Pitch `filt_T_hz` is set to 20 Hz in ArduPilot (`AC_ATC_MULTI_RATE_RPY_FILT_HZ`) but 0.0 in Meridian (disabled). This means Meridian does not low-pass filter the rate target input, which is minor since the attitude shaping already smooths it, but it is a deviation.
2. **Yaw `filt_E_hz` is 2.5 Hz in ArduPilot, but 20.0 Hz in Meridian.** This is a significant difference. ArduPilot's yaw error filter is much more aggressive (slow) to reduce gyro noise coupling into yaw output. Meridian's yaw D term will have ~8x more noise than ArduPilot's.

---

### Filter Reset on Mode Change

**ArduPilot:** `rate_controller_target_reset()` calls `reset_I()` and `reset_filter()` on all three rate PIDs.

**Meridian:** `RateController::reset()` calls `PidController::reset()` on all three. `PidController::reset()` zeroes all state including `initialized = false` which causes the filter to reseed on next call.

**VERDICT: MATCH.** Functionally equivalent.

---

### Throttle Gain Boost

**ArduPilot:** `update_throttle_gain_boost()` — during rapid throttle slew (`>AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH`), temporarily boosts P/D scale and Angle P scale by up to 2x and 4x respectively.

**Meridian:** Not present.

**VERDICT: MISSING.**

---

## Position Controller

**Files compared:** `AC_PosControl.cpp` vs `position_controller.rs`

### `update_xy_controller()` Path

**ArduPilot:** Full pipeline:
1. `update_pos_vel_accel_xy()` — integrates desired pos/vel/accel forward in time (model-predictive)
2. `shape_pos_vel_accel_xy()` — sqrt-controlled, jerk-limited position → velocity shaping with `sqrt_controller_accel()` bias
3. Velocity PID (`_pid_vel_ne_m`) with separate P, I, D and i_scale for saturation
4. `limit_accel_corner_xy()` — reduces acceleration at corners to prevent overshoot
5. Separate `_accel_target_ned_mss` tracking that feeds the lean angle converter

**Meridian:**
```rust
let vel_target_n = (pos_err_n * self.gains.pos_xy_p + target_vel.x).clamp(-5.0, 5.0);
let raw_accel_n = self.vel_pid_x.update(vel_target_n, current_vel.x, dt);
```
Simple P on position error + velocity PID. No `shape_pos_vel_accel_xy()`, no forward integration.

**VERDICT: DIVERGE — MAJOR.** ArduPilot's position controller uses a full jerk-limited shaping pipeline (`shape_pos_vel_accel_xy`) with `sqrt_controller_accel()` closing-rate bias. Meridian uses a direct P controller with no trajectory shaping. Waypoint following and loiter behavior will be different — specifically, ArduPilot decelerates smoothly before the target while Meridian will overshoot.

---

### `shape_pos_vel_accel()` Port

**ArduPilot:** `shape_pos_vel_accel()` includes `sqrt_controller_accel()` — a correction bias based on the closing rate (derivative of position error projected onto the correction direction). This is not a standard sqrt controller; it accounts for the rate-of-change of the correction command.

**Meridian `sqrt_controller.rs`:** Implements `shape_pos_vel_accel()` but without the `sqrt_controller_accel()` bias term. The ArduPilot signature:
```cpp
void shape_pos_vel_accel(postype_t pos_desired, float vel_desired, float accel_desired,
                         postype_t pos, float vel, float& accel,
                         float vel_min, float vel_max,
                         float accel_min, float accel_max,
                         float jerk_max, float dt, bool limit_total)
```
Meridian signature:
```rust
pub fn shape_pos_vel_accel(pos_error, vel_desired, accel_desired,
                            vel, accel: &mut f32,
                            vel_min, vel_max, accel_min, accel_max,
                            jerk_max, dt)
```

**VERDICT: DIVERGE.** Meridian's `shape_pos_vel_accel` omits the `sqrt_controller_accel` closing-rate bias. Also missing the `limit_total` flag. The function is structurally correct but behaviorally different for large position errors or fast-moving setpoints.

---

### Velocity Feedforward Terms

**ArduPilot:** `target_vel` is passed directly into `shape_pos_vel_accel_xy()` as `vel_desired` — it becomes the feedforward component of the velocity shaping and is also integrated into the desired velocity state. The desired velocity is propagated forward in time by `update_pos_vel_accel_xy()`.

**Meridian:**
```rust
let vel_target_n = (pos_err_n * self.gains.pos_xy_p + target_vel.x).clamp(-5.0, 5.0);
```
Velocity feedforward is summed directly into the velocity target. No state propagation.

**VERDICT: PARTIAL.** Feedforward is present but not properly integrated into a predictive state machine. For slowly moving targets (smooth trajectories) behavior is acceptable; for fast-changing velocity targets (mission segments, position offsets) there will be lag.

---

### Jerk Limiting

**ArduPilot:** Jerk limiting is applied via `shape_accel_xy()` as part of `shape_pos_vel_accel_xy()`. The jerk limit also feeds back into the dynamic gain `k_v = jerk_max / accel_max`.

**Meridian:**
```rust
let max_delta = self.jerk_max_xy * dt;
let delta_n = (raw_accel_n - self.prev_accel.x).clamp(-max_delta, max_delta);
```
Simple delta clamp on the acceleration output. Correct in concept, but applied *after* the PID output rather than within the trajectory shaper. This means the integrator can wind up against the jerk limit.

**VERDICT: DIVERGE.** Jerk limiting is present but applied at the wrong point in the pipeline (after PID vs inside trajectory shaper before PID).

---

### Lean Angle to Acceleration Conversion

**ArduPilot:** Uses `lean_angles_to_accel_xy()` and an intermediate thrust vector conversion. The horizontal acceleration target is computed from `_attitude_control.get_thrust_vector()` combined with `_motors.get_throttle()`. The conversion uses actual throttle-normalized thrust, not just gravity.

**Meridian:**
```rust
let pitch_target = libm::atan2f(-accel_fwd, GRAVITY);
let roll_target = libm::atan2f(accel_right, GRAVITY);
```
Uses gravity constant directly. Correct for hover but does not account for throttle-scaling of the thrust vector.

**VERDICT: DIVERGE.** At throttle settings far from hover, the lean angle for a given acceleration target will be off. Correct at hover; diverges as throttle increases.

---

### Hover Throttle Estimation

**ArduPilot:** `_motors.get_throttle_hover()` — dynamically estimated from a DC filter on actual throttle when not accelerating vertically. Used to initialize PID integrators and as the zero-acceleration reference point.

**Meridian:** Fixed `hover_throttle: 0.39` in gains.

**VERDICT: DIVERGE.** Static hover throttle. Will be wrong for any vehicle that differs from the default test configuration. No in-flight learning.

---

### Velocity/Acceleration Limit Enforcement

**ArduPilot:** Velocity limits are enforced through `constrain_float` on `_vel_desired` and via the `limit_vector_ned` which tracks which axes are saturated and feeds back into `update_pos_vel_accel_xy()` to prevent integrator wind-up.

**Meridian:** Simple clamp on velocity target (`clamp(-5.0, 5.0)` hardcoded). No `limit_vector` feedback.

**VERDICT: DIVERGE.** No saturation tracking feedback. When a velocity limit is hit, the integrator continues to grow, causing windup.

---

## sqrt_controller

**Files compared:** `AP_Math/control.cpp` vs `sqrt_controller.rs`

### Linear/Sqrt Transition Regions

**ArduPilot:**
```cpp
const float linear_dist = second_ord_lim / sq(p);
if (error > linear_dist)
    correction_rate = safe_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)));
```

**Meridian:**
```rust
let linear_dist = second_ord_lim / (p * p);
if error > linear_dist {
    correction_rate = libm::sqrtf(2.0 * second_ord_lim * (error - linear_dist * 0.5));
```

**VERDICT: MATCH.** Formula is identical.

---

### Overshoot Clamping

**ArduPilot:**
```cpp
return constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
```

**Meridian:**
```rust
let max_rate = libm::fabsf(error) / dt;
if correction_rate > max_rate { return max_rate; }
if correction_rate < -max_rate { return -max_rate; }
```

**VERDICT: MATCH.**

---

### Inverse Function

**ArduPilot and Meridian:** Both implement the same degenerate-case handling and piecewise linear/sqrt inverse. Formulas match exactly.

**VERDICT: MATCH.**

---

### `shape_vel_accel()` and `shape_pos_vel_accel()` Completeness

**ArduPilot extras not in Meridian:**

1. **`sqrt_controller_accel()`** — Estimates `d(rate_cmd)/dt` from the current closing rate using the chain rule. Used inside `shape_pos_vel_accel()` and `shape_pos_vel_accel_xy()` to add a velocity correction bias that pre-compensates for setpoint deceleration. **Not ported.**

2. **`shape_pos_vel_accel()` `limit_total` flag** — When true, constrains total velocity and acceleration after adding feedforward. Meridian omits this parameter.

3. **`limit_accel_corner_xy()`** — Reduces acceleration magnitude at waypoint corners by a factor of `1/√2`. Prevents cornering overshoot. **Not ported.**

4. **`shape_angular_vel()`** — Angular velocity shaper analogous to `shape_vel_accel` but for wrapped angles. Used for yaw rate shaping. **Not ported.**

5. **`stopping_distance()`** — Uses `inv_sqrt_controller` to compute deceleration distance. Used in mission planning for waypoint approach. **Not ported.**

**VERDICT: PARTIAL.** The core `sqrt_controller` and `inv_sqrt_controller` match exactly. `shape_vel_accel` and `shape_pos_vel_accel` are present but missing `sqrt_controller_accel` bias and `limit_total`. The 2D vector variants and angular forms are absent.

---

### 2D Vector Versions

**ArduPilot:** `sqrt_controller(Vector2f, p, lim, dt)`, `shape_vel_accel_xy()`, `shape_pos_vel_accel_xy()`, `update_vel_accel_xy()`, `update_pos_vel_accel_xy()` — all treat the XY plane as a unified 2D space with a total magnitude limit and proper corner handling.

**Meridian:** `sqrt_controller_2d()` — applies scalar sqrt_controller along the error direction. `shape_accel_2d()` — applies jerk limiting as a 2D vector. No `shape_vel_accel_xy()` or `shape_pos_vel_accel_xy()` counterparts.

**VERDICT: PARTIAL.** 2D magnitude-directed variants are present but the full shaping pipeline (vel_accel_xy, pos_vel_accel_xy with closing-rate bias) is not ported.

---

## TECS

**Files compared:** `AP_TECS.cpp` vs `tecs.rs`

### Energy Error Computation

**ArduPilot:** `_STE_error = _STE_desired - _STE_estimate` where STE = `g*h + 0.5*v²`. Balance: `_SEB_error = _SEB_desired - _SEB_estimate` where SEB = `g*h - 0.5*v²`.

**Meridian:**
```rust
self.ste_error = ste_demand - ste_actual;  // g*h + 0.5*v²
self.seb_error = seb_demand - seb_actual;  // g*h - 0.5*v²
```

**VERDICT: MATCH.**

---

### Speed/Height Weighting (SPDWEIGHT)

**ArduPilot:** `_SPDWeight` applied to blend STE and SEB contributions in the pitch demand.

**Meridian:**
```rust
let w = self.params.spdweight.clamp(0.0, 2.0);
let seb_w = if w > 1.0 { 2.0 - w } else { 1.0 };
let ste_w = if w < 1.0 { w } else { 1.0 };
```

**VERDICT: MATCH IN STRUCTURE.** The weighting formula is the same concept. Note: `ste_w` is computed but marked `#[allow(unused)]` in Meridian (`let _ = ste_w;`) — the STE weight is not actually applied in the pitch calculation. This is a latent bug.

---

### Complementary Filters for Height and Speed

**ArduPilot:** Uses `_hgtCompFiltOmega` (default `3.0 rad/s`) for a 3rd-order complementary filter on height, and `_spdCompFiltOmega` (default `2.0 rad/s`) for a 2nd-order filter on speed. These are configurable parameters. The filter blends baro altitude with vertical acceleration.

**Meridian:**
```rust
// 3rd-order complementary filter for height
let tau = 3.0;
let k1 = 3.0 / tau;
let k2 = 3.0 / (tau * tau);
```
Fixed `tau = 3.0`, not configurable.

**VERDICT: DIVERGE — MINOR.** Structure is correct (3rd-order height, 2nd-order speed). Gains are hardcoded rather than parameter-driven. For a configurable aircraft this needs to be exposed.

---

### Underspeed Protection

**ArduPilot:** `_flags.underspeed` — when airspeed drops below minimum, TECS overrides height demand with a speed-recovery demand, forcing a descent to regain airspeed. Triggered by `_badDescent` and `_underspeed` flags.

**Meridian:** No underspeed protection flag or logic.

**VERDICT: MISSING — SAFETY CRITICAL.** Without underspeed protection, TECS will continue to pitch up when slowing, which can cause a stall.

---

### Flare Mode

**ArduPilot:** On landing, TECS transitions to a flare mode with a separate time constant (`TECS_LAND_TCONST`), separate sink rate demand (`TECS_LAND_SINK`), a progressive sink rate ramp based on distance past the land waypoint, and a hold-off height (`TECS_FLARE_HGT`). The flare state is initialized once (`_flare_initialised`) and continuously updated until touchdown.

**Meridian:** No flare mode.

**VERDICT: MISSING.** Fixed-wing landing is incomplete without flare.

---

### Additional Missing TECS Features

- No `_maxSinkRate` (maximum sink rate for descents) configurable parameter — Meridian has `max_sink_rate` in params, so this is present.
- No `_flags.badDescent` — prevents excessively steep descent angles.
- No `_dthetadt` (flight path angle estimate from baro+accel fusion).
- No `_pitch_dem_unc` (unconstrained pitch demand before limiting) for logging.
- No `TECS_LAND_ARSPD` (reduced approach speed).
- No `time_const` parameter exposed (fixed at tau = 3.0 equivalent, i.e., `_timeConst` maps to the filter cutoff — ArduPilot default is `5.0 s`).

---

## Notch Filter

**Files compared:** `Filter/NotchFilter.h/.cpp` + `Filter/HarmonicNotchFilter.h` vs `notch_filter.rs`

### Biquad Coefficient Calculation

**ArduPilot:**
```cpp
A = powf(10, -attenuation_dB / 40.0f);
Q = sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1.0f);  // from bandwidth
alpha = sinf(omega) / (2 * Q);
b0 = 1.0 + alpha * sq(A);
b1 = -2.0 * cosf(omega);
b2 = 1.0 - alpha * sq(A);
a1 = b1;  // notch: a1 = b1
a2 = 1.0 - alpha;
```
Note: ArduPilot converts bandwidth to Q first via the octave formula, then computes coefficients.

**Meridian:**
```rust
let attenuation = libm::powf(10.0, -self.attenuation_db / 40.0);
let bandwidth_rad = 2.0 * PI * self.bandwidth_hz / self.sample_rate_hz;
let alpha = sin_omega * libm::sinhf(0.5 * bandwidth_rad);
self.b0 = 1.0 + alpha * attenuation;
self.b1 = -2.0 * cos_omega;
self.b2 = 1.0 - alpha * attenuation;
self.a0 = 1.0 + alpha / attenuation;
self.a1 = -2.0 * cos_omega;
self.a2 = 1.0 - alpha / attenuation;
```
Meridian uses `alpha = sin_omega * sinh(0.5 * bandwidth_rad)` (Audio EQ Cookbook formula for bandwidth in radians). ArduPilot uses Q-based alpha: `alpha = sin_omega / (2*Q)`.

**VERDICT: DIVERGE.** The coefficient formulas are numerically different. ArduPilot's Q is derived from the octave-bandwidth formula `Q = √(2^oct) / (2^oct - 1)`, which relates Q to the -3dB bandwidth. Meridian uses the `sinh` bandwidth formula from the Audio EQ Cookbook which relates directly to the bandwidth in radians. Both produce valid notch filters but they will have different frequency responses for the same `bandwidth_hz` input. The attenuation depth is also computed differently: ArduPilot uses `alpha * A²` / `alpha` (asymmetric numerator/denominator) while Meridian uses `alpha * A` / `alpha / A`. These produce slightly different notch depth characteristics.

---

### 5% Slew Limiter

**ArduPilot:**
```cpp
new_center_freq = constrain_float(new_center_freq,
    _center_freq_hz * NOTCH_MAX_SLEW_LOWER,  // 0.95 * freq
    _center_freq_hz * NOTCH_MAX_SLEW_UPPER); // freq / 0.95
```
Applied as a multiplicative range constraint on the new center frequency.

**Meridian:**
```rust
let delta = freq_hz - self.effective_freq_hz;
let max_delta = self.effective_freq_hz * FREQ_SLEW_LIMIT; // 0.05 * freq
let clamped = delta.clamp(-max_delta, max_delta);
self.effective_freq_hz += clamped;
```
Applied as an additive delta clamp of `±5%`.

**VERDICT: MATCH IN EFFECT.** Both limit the center frequency change to 5% of the current frequency per update. The additive delta clamp and multiplicative range constraint are equivalent for small steps.

---

### HarmonicNotchFilter — 16 Harmonics

**ArduPilot:** `HNF_MAX_HARMONICS = 16`. Uses a `uint32_t` harmonics bitmask (32 bits), supports up to 16 simultaneous harmonics. Also supports `DoubleNotch`, `TripleNotch`, and `QuintupleNotch` composite notch modes (multiple filters per harmonic). Also has `DynamicHarmonic` mode and `LoopRateUpdate`.

**Meridian:** `MAX_HARMONICS = 8`. Uses a `u8` bitmask (8 bits). No composite notch modes. No `DynamicHarmonic` or `LoopRateUpdate`.

**VERDICT: DIVERGE.**
- Max harmonics: 8 vs 16 (half of ArduPilot)
- No double/triple/quintuple composite notch modes
- No `DynamicHarmonic` (where harmonic ratios change dynamically based on FFT peaks)
- No `LoopRateUpdate` (update notch at main loop rate rather than sensor rate)

---

### Tracking Modes

**ArduPilot has 6 tracking modes:**
1. `Fixed` (no tracking)
2. `UpdateThrottle` (throttle → RPM model)
3. `UpdateRPM` (RPM sensor)
4. `UpdateBLHeli` (ESC telemetry via BLHeli protocol)
5. `UpdateGyroFFT` (in-flight FFT detection)
6. `UpdateRPM2` (secondary RPM sensor)

**Meridian has 5 tracking modes:**
1. `Fixed`
2. `Throttle`
3. `Rpm`
4. `EscRpm`
5. `Fft`

**VERDICT: PARTIAL.** All 5 Meridian modes map to 5 of the 6 ArduPilot modes. `UpdateRPM2` (secondary RPM source) is absent. The `EscRpm` mode is present but the BLHeli protocol integration to populate it is in `meridian-drivers`, not here.

---

## Summary of Missing and Diverging Features

### CRITICAL Divergences (will cause flight behavior differences)

| # | Feature | Meridian State | Impact |
|---|---------|---------------|--------|
| 1 | SMAX SlewLimiter | Naive delta clamp instead of adaptive gain reducer | PD tuning behavior differs; oscillation damping wrong |
| 2 | Quaternion error frame | `target * body^-1` vs ArduPilot `target^-1 * body` | Error vector in different frame; must verify entire pipeline is self-consistent |
| 3 | Integrator anti-windup `limit` flag | Not implemented | Integrator winds up against actuator limits; causes overshoot |
| 4 | Position controller shaping | Direct P+PID instead of sqrt-shaped trajectory | Waypoint overshoot; no smooth deceleration |
| 5 | Yaw filt_E_hz default | 20 Hz (Meridian) vs 2.5 Hz (ArduPilot) | ~8x more noise in yaw output; yaw oscillation risk |
| 6 | Notch biquad coefficients | sinh bandwidth formula vs Q-based | Different frequency response for same bandwidth_hz |
| 7 | TECS underspeed protection | Absent | Stall risk on approach |

### Significant Missing Features

| # | Feature | Impact |
|---|---------|--------|
| 8 | `D_FF` (derivative feedforward on target) | Increased tracking lag on rate commands |
| 9 | `get_pid_info()` telemetry struct | No PID logging; blind tuning |
| 10 | `relax_integrator()` | Harder mode transitions |
| 11 | `input_thrust_vector_heading()` | Position hold requires Euler angles; sub-optimal for precision modes |
| 12 | `_throttle_rpy_mix` | Landing instability; poor throttle-attitude priority blend |
| 13 | Dynamic lean angle max | Fixed angle limit regardless of throttle headroom |
| 14 | Thrust-error feedforward scalar | No yaw protection when tilted far from level |
| 15 | `sqrt_controller_accel()` closing-rate bias | Trajectory deceleration accuracy |
| 16 | `limit_accel_corner_xy()` | Waypoint cornering overshoot |
| 17 | Hover throttle adaptive estimation | Wrong at any non-default vehicle weight |
| 18 | TECS flare mode | Fixed-wing landing incomplete |
| 19 | TECS underspeed / bad descent flags | Safety-critical for fixed-wing |
| 20 | Per-PID notch filters on signal paths | Noise suppression for high-vibration airframes |
| 21 | Harmonic notch composite modes (double/triple/quintuple) | Less effective notch for multi-rotor blade harmonics |
| 22 | HNF max 16 harmonics (Meridian has 8) | High harmonic count motors unsupported |
| 23 | Landed gain multipliers (LAND_R/P/Y_MULT) | Ground oscillation on hard landings |
| 24 | PDMX (PD sum maximum) | No protection against P+D actuator saturation |
| 25 | Throttle gain boost during rapid throttle changes | Sluggish response to fast throttle inputs |
| 26 | `shape_pos_vel_accel` 2D (XY) versions | Horizontal position control uses simplified path |
| 27 | `shape_angular_vel()` | Yaw shaping not matched to ArduPilot |
| 28 | Position controller saturation limit tracking | Integrator windup at velocity limits |
| 29 | `i_scale` parameter on integrator update | Cannot scale I contribution from outer loops |
| 30 | `ste_w` applied in TECS pitch calc | Latent bug: STE weight computed but unused |

### Matched Features (PASS)

- sqrt_controller linear/sqrt formula
- sqrt_controller overshoot clamping
- inv_sqrt_controller roundtrip
- shape_accel jerk limiting (1D)
- shape_accel_2d (2D jerk limiting)
- shape_vel_accel structure (minus sqrt_controller_accel bias)
- update_vel_accel / update_pos_vel_accel
- 3 LP filter structure in PID
- PID `set_integrator()`
- Rate PID default gains (P/I/D/IMAX for R/P/Y)
- Rate controller filter reset on mode change
- Attitude controller P gains and defaults
- Angle boost formula (near-hover regime)
- Notch filter 5% slew limiter (functionally equivalent)
- Notch filter tracking modes (5 of 6)
- TECS energy error computation
- TECS speed/height weighting structure
- TECS complementary filter structure (fixed gains)
- NotchFilter biquad state machine (apply/reset)
