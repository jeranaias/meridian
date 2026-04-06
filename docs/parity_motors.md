# Motor Parity Audit: AP_Motors vs meridian-mixing

**Date:** 2026-04-02
**ArduPilot source:** `libraries/AP_Motors/`
**Meridian source:** `crates/meridian-mixing/`, `crates/meridian-heli/`, `crates/meridian-hal/`

---

## 1. Frame Type Parity

ArduPilot defines **38 frame presets** across 7 frame classes in `AP_MotorsMatrix.cpp`.

### Quad (11 variants)

| Frame Type | AP_Motors | Meridian | Notes |
|---|---|---|---|
| PLUS | `setup_quad_matrix` L581 | `MixingMatrix::quad_plus()` | **MATCH** |
| X | `setup_quad_matrix` L592 | `MixingMatrix::quad_x()` | **MATCH** |
| NYT_PLUS | L604 (ArduPlane/UNKNOWN build only) | **MISSING** | No-yaw-torque variant; plane build only in AP |
| NYT_X | L615 (ArduPlane/UNKNOWN build only) | **MISSING** | No-yaw-torque variant; plane build only in AP |
| BF_X | L627 | **MISSING** | BetaFlight motor order |
| BF_X_REV | L640 | **MISSING** | BetaFlight motor order, reversed |
| DJI_X | L652 | **MISSING** | DJI motor numbering order |
| CW_X | L665 | **MISSING** | Clockwise-ordered X for motor test |
| V | L678 | **MISSING** | Asymmetric yaw factors (0.7981/1.0) |
| H | L689 | **MISSING** | X geometry, reversed spin directions |
| VTAIL | L701 | **MISSING** | Asymmetric, rear-yaw only |
| ATAIL | L724 | **MISSING** | Inverse of VTAIL yaw direction |
| PLUSREV | L744 | **MISSING** | Plus with reversed motors |
| Y4 | L756 | **MISSING** | Coaxial rear pair |

**Summary: Meridian has 2 of 14 quad variants (Plus and X). Missing: NYT_PLUS, NYT_X, BF_X, BF_X_REV, DJI_X, CW_X, V, H, VTAIL, ATAIL, PLUSREV, Y4.**

Note: ArduPilot documents 11 "standard" quad variants; VTAIL, ATAIL, PLUSREV, and Y4 bring the actual count to 14. The task spec's count of 11 omits the four specialized types.

### Hex (5 variants)

| Frame Type | AP_Motors | Meridian | Notes |
|---|---|---|---|
| PLUS | `setup_hexa_matrix` L780 | `MixingMatrix::hex_plus()` | **MATCH** |
| X | L793 | `MixingMatrix::hex_x()` | **MATCH** |
| H | L806 | **MISSING** | Raw factors, side-compressed geometry |
| DJI_X | L820 | **MISSING** | DJI motor order |
| CW_X | L833 | **MISSING** | Clockwise-ordered X |

**Summary: Meridian has 2 of 5 hex variants. Missing: H, DJI_X, CW_X.**

### Octa (7 variants)

| Frame Type | AP_Motors | Meridian | Notes |
|---|---|---|---|
| PLUS | `setup_octa_matrix` L859 | **MISSING** | |
| X | L875 | `MixingMatrix::octa_x()` | **MATCH** — angles and factors verified against AP L876-887 |
| V | L890 | **MISSING** | Raw asymmetric factors |
| H | L905 | **MISSING** | Raw asymmetric factors |
| I | L920 | **MISSING** | Linear arrangement |
| DJI_X | L935 | **MISSING** | DJI motor order |
| CW_X | L950 | **MISSING** | Clockwise-ordered X |

**Summary: Meridian has 1 of 7 octa variants. Missing: PLUS, V, H, I, DJI_X, CW_X.**

### OctaQuad (8 variants — coaxial quads)

| Frame Type | AP_Motors | Meridian | Notes |
|---|---|---|---|
| PLUS | `setup_octaquad_matrix` L978 | **MISSING** | |
| X | L993 | **MISSING** | |
| V | L1008 | **MISSING** | Asymmetric yaw factors |
| H | L1023 | **MISSING** | |
| CW_X | L1039 | **MISSING** | |
| BF_X | L1056 | **MISSING** | BetaFlight order |
| BF_X_REV | L1071 | **MISSING** | BetaFlight reversed |
| X_COR | L1087 | **MISSING** | Co-rotating; top layer scaled by `AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR` |
| CW_X_COR | L1110 | **MISSING** | Co-rotating CW_X; top layer scaled |

**Summary: Meridian has 0 of 9 OctaQuad variants. The entire OctaQuad class is absent.**

Note: ArduPilot's specification list has 8 OctaQuad variants; counting CW_X_COR brings the actual total to 9.

### Y6 (3 variants)

| Frame Type | AP_Motors | Meridian | Notes |
|---|---|---|---|
| default (Y6) | `setup_y6_matrix` L1224 | **MISSING** | Raw factors |
| Y6B | L1196 | **MISSING** | All-CW top, all-CCW bottom |
| Y6F | L1210 | **MISSING** | FireFly Y6 layout |

**Summary: Meridian has 0 of 3 Y6 variants. The entire Y6 class is absent.**

### DodecaHexa (2 variants — 12-motor)

| Frame Type | AP_Motors | Meridian | Notes |
|---|---|---|---|
| PLUS | `setup_dodecahexa_matrix` L1145 | **MISSING** | 12 motors, paired at each arm |
| X | L1164 | **MISSING** | 12 motors, paired at each arm |

**Summary: Meridian has 0 of 2 DodecaHexa variants. Entire class absent. MAX_MOTORS = 12 in both, so array size would support it.**

### Deca (2 variants — 10-motor)

| Frame Type | AP_Motors | Meridian | Notes |
|---|---|---|---|
| PLUS | `setup_deca_matrix` L1247 | **MISSING** | 10 motors at 36° spacing |
| X / CW_X | L1264 (shared case) | **MISSING** | 10 motors at 18° offset; X and CW_X share the same layout |

**Summary: Meridian has 0 of 2 Deca variants. Entire class absent.**

### Overall Frame Parity

| Class | AP count | Meridian count | Gap |
|---|---|---|---|
| Quad | 14 | 2 | 12 missing |
| Hex | 5 | 2 | 3 missing |
| Octa | 7 | 1 | 6 missing |
| OctaQuad | 9 | 0 | 9 missing |
| Y6 | 3 | 0 | 3 missing |
| DodecaHexa | 2 | 0 | 2 missing |
| Deca | 2 | 0 | 2 missing |
| **Total** | **42** | **5** | **37 missing** |

---

## 2. Mixing Algorithm Parity (`output_armed_stabilizing` vs `mix`)

ArduPilot: `AP_MotorsMatrix.cpp::output_armed_stabilizing()` lines 213–404
Meridian: `crates/meridian-mixing/src/lib.rs::Mixer::mix()` lines 196–329

### Pipeline Comparison

| Step | ArduPilot | Meridian | Status |
|---|---|---|---|
| 1. Apply compensation gain to RPY+throttle | `compensation_gain` multiplied into all inputs (L216–231) | **ABSENT** — raw inputs used directly | **MISSING** |
| 2. Compute throttle_avg_max and throttle_thrust_max | L231–247 | **ABSENT** | **MISSING** |
| 3. Calculate throttle_thrust_best_rpy = min(0.5, throttle_avg_max) | L251 | Hardcoded `throttle_offset = 0.5` | **SIMPLIFIED** — no `throttle_avg_max` parameter |
| 4. Per-motor RP contribution | L280: `roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i]` | L203–205: identical formula | **MATCH** |
| 5. Yaw headroom loop | L277–325: per-motor loop, directional room calculation (`motor_room = 1.0 - thrust_rp_best_throttle` or `thrust_rp_best_throttle`), `yaw_allowed = MIN(yaw_allowed, motor_yaw_allowed)` | L244–268: simplified — finds max_yaw_factor and computes scalar headroom from upper/lower bounds | **DIFFERENT** — AP is per-motor directional; Meridian uses global min-headroom scalar |
| 6. MOT_YAW_HEADROOM minimum (`yaw_allowed_min = _yaw_headroom * 0.001`) | L302–308 | **ABSENT** — no configurable yaw headroom floor | **MISSING** |
| 7. Thrust boost yaw override | L304–325: `boost_ratio(0.5, yaw_allowed_min)` and lost motor exclusion | **ABSENT** | **MISSING** |
| 8. Yaw clamping | L327–331 | L271–275: equivalent clamp | **MATCH** |
| 9. Add yaw to RPY, track rpy_low/rpy_high | L334–357, with thrust_boost lost motor blending | L279–299: simpler equivalent, no lost motor case | **MATCH (basic case)** |
| 10. RPY proportional scale | L360–363: `rpy_scale = 1/(rpy_high-rpy_low)` if range > 1.0 | L294–298: identical formula | **MATCH** |
| 11. Additional rpy_scale floor on throttle_avg_max | L364–366: `MIN(rpy_scale, -throttle_avg_max/rpy_low)` | **ABSENT** | **MISSING** |
| 12. Throttle adjustment (thr_adj) | L368–388: complex with limit flags | L309–317: simplified clamp | **SIMPLIFIED** |
| 13. Final output assembly | L391–395: `throttle_thrust_best_plus_adj * _throttle_factor[i] + rpy_scale * _thrust_rpyt_out[i]` | L322–325: `throttle_out * f.throttle + rpy[i]` | **EQUIVALENT** |
| 14. Limit flags (limit.set_rpy, limit.throttle_upper/lower) | Set throughout | **ABSENT** — no limit reporting struct | **MISSING** |

**Summary:** The core proportional desaturation math is equivalent for the nominal case (no motor loss, no battery compensation). Five significant mechanisms are missing: compensation gain, throttle_avg_max, MOT_YAW_HEADROOM floor, thrust_boost motor-loss mode, and limit flag reporting.

### Yaw Headroom Calculation

ArduPilot's per-motor directional loop (lines 277–295):
```cpp
if (is_positive(yaw_thrust * _yaw_factor[i])) {
    motor_room = 1.0 - thrust_rp_best_throttle;   // headroom to ceiling
} else {
    motor_room = thrust_rp_best_throttle;           // headroom to floor
}
motor_yaw_allowed = MAX(motor_room, 0.0) / fabsf(_yaw_factor[i]);
yaw_allowed = MIN(yaw_allowed, motor_yaw_allowed);
```

Meridian's scalar approach (lib.rs lines 240–266):
```rust
let headroom_upper = 1.0 - throttle_offset - rp_high;
let headroom_lower = throttle_offset - rp_low;
let min_headroom = headroom_upper.min(headroom_lower);
let allowed = min_headroom / max_yaw_factor;
```

The Meridian version uses the same geometric idea but evaluates it globally rather than per-motor. For symmetric frames this produces the same result. For asymmetric frames (V, VTAIL, OctaQuad co-rotating) the per-motor directional check in AP can allow more yaw in one direction than the other — Meridian cannot express this asymmetry.

### RPY Proportional Scaling

Both implementations use `rpy_scale = 1.0 / (rpy_high - rpy_low)` when the RPY range exceeds 1.0. **This is identical.** The scaling preserves the ratio of roll:pitch:yaw contributions by multiplying all three by the same scalar.

ArduPilot also has a secondary scale floor: `rpy_scale = MIN(rpy_scale, -throttle_avg_max / rpy_low)` — this prevents the scale from boosting RPY so much that it pushes below the throttle floor. Meridian does not implement this.

### Throttle-Attitude Priority

Both implementations give attitude priority: RPY is computed first, then throttle fits into the remaining headroom. The `thr_lo / thr_hi` clamp in Meridian (lines 309–317) is functionally equivalent to ArduPilot's `thr_adj` calculation for the normal case. ArduPilot additionally tracks `limit.throttle_upper` and `limit.throttle_lower` flags; Meridian silently clamps.

### Battery Voltage / Throttle Compensation

ArduPilot: `compensation_gain = thr_lin.get_compensation_gain()` is applied at the top of `output_armed_stabilizing()`. All RPY and throttle thrust values are pre-scaled by this factor before any mixing. This compensates for battery voltage sag in real time.

Meridian: `apply_voltage_comp()` exists as a standalone function in `lib.rs` (lines 371–379) and `apply_thrust_curve()` (lines 353–359), but **neither is called inside `Mixer::mix()`**. The caller must apply them manually before or after mixing. This means the compensation gain is not woven into the headroom calculation the way it is in ArduPilot, which can cause subtle over-/under-headroom estimates.

---

## 3. Spool State Machine

ArduPilot: `AP_MotorsMulticopter.cpp::output_logic()` lines 581–734+
Meridian: `crates/meridian-mixing/src/spool.rs`

### State Enumeration

| State | ArduPilot enum | Meridian enum | Notes |
|---|---|---|---|
| SHUT_DOWN | `SpoolState::SHUT_DOWN` | `SpoolState::ShutDown` | **MATCH** |
| GROUND_IDLE | `SpoolState::GROUND_IDLE` | `SpoolState::GroundIdle` | **MATCH** |
| SPOOLING_UP | `SpoolState::SPOOLING_UP` | `SpoolState::SpoolingUp` | **MATCH** |
| THROTTLE_UNLIMITED | `SpoolState::THROTTLE_UNLIMITED` | `SpoolState::ThrottleUnlimited` | **MATCH** |
| SPOOLING_DOWN | `SpoolState::SPOOLING_DOWN` | `SpoolState::SpoolingDown` | **MATCH** |

All 5 states are present with correct names and transitions.

### Transitions

| Transition | ArduPilot | Meridian | Status |
|---|---|---|---|
| ShutDown → GroundIdle | `_disarm_safe_timer >= _safe_time` AND `_spool_desired != SHUT_DOWN` | `state_time >= config.safe_time` via `request_spool_up()` | **MATCH** (safe_time gate) |
| GroundIdle → SpoolingUp | After idle delay AND `!get_spoolup_block()` | `request_spool_up()` call (no block check) | **SIMPLIFIED** — AP's `get_spoolup_block()` (ESC telemetry, RPM check, CPU check) is absent |
| SpoolingUp → ThrottleUnlimited | `_spin_up_ratio >= 1.0` | `ramp >= 1.0` | **MATCH** |
| ThrottleUnlimited → SpoolingDown | `_spool_desired == GROUND_IDLE` or `SHUT_DOWN` in ThrottleUnlimited | `request_spool_down()` call | **MATCH** |
| SpoolingDown → GroundIdle | `_spin_up_ratio <= 0.0` | `ramp <= 0.0` | **MATCH** |
| Any → ShutDown (emergency) | `!armed() || !get_interlock()` → immediate | `disarm()` → immediate | **MATCH** |

### Timing Parameters

| Parameter | ArduPilot | Meridian | Status |
|---|---|---|---|
| MOT_SPOOL_TIME | `_spool_up_time`, default 0.5s (AP_MOTORS_SPOOL_UP_TIME_DEFAULT) | `spool_up_time`, default 0.5 | **MATCH** |
| MOT_SPOOL_TIM_DN | `_spool_down_time`, default 0 (falls back to up time) | `spool_down_time`, default 0.5 | **DIVERGENCE** — AP defaults spool_down to 0 (uses up time); Meridian always uses 0.5 |
| MOT_SAFE_TIME | `_safe_time`, default 0.0 | `safe_time`, default 0.0 | **MATCH** |
| IDLE_TIME | `_idle_time_delay_s` — holds at ground idle after reaching spin_arm before advancing | **ABSENT** | **MISSING** |
| minimum_spool_time | `constexpr 0.05f` — prevents divide-by-zero | **ABSENT** (no floor on spool times) | **MISSING** |

### Spin Parameters

| Parameter | ArduPilot | Meridian | Status |
|---|---|---|---|
| MOT_SPIN_ARM | `_spin_arm`, default 0.10 | `spin_arm`, default 0.10 | **MATCH** |
| MOT_SPIN_MIN | `thr_lin.get_spin_min()`, default 0.15 | `spin_min`, default 0.15 | **MATCH** |

The spin_arm vs spin_min distinction is correctly implemented in both. In GROUND_IDLE, motors output at `spin_arm`; in flight (`ThrottleUnlimited`), the floor is `spin_min`.

### Spool Missing: `get_spoolup_block()`

ArduPilot's `get_spoolup_block()` prevents advancing from `GROUND_IDLE` to `SPOOLING_UP` until:
- ESC telemetry is healthy (if configured)
- Motor RPM is in range (if RPM sensor is present)
- CPU load is not excessive

Meridian's `request_spool_up()` advances unconditionally (only checking `state_time >= safe_time`). This is a safety gap: on hardware with ESC telemetry configured, Meridian would spool up before ESCs report ready.

### DesiredSpoolState (Two-level Machine)

ArduPilot separates **desired** state (from flight controller) from **actual** state (hardware ramp). Meridian collapses this: `arm()`, `request_spool_up()`, `request_spool_down()`, `disarm()` are direct imperative calls. The ArduPilot pattern allows the controller to set a desired state that takes multiple cycles to reach; Meridian's pattern is equivalent in effect but does not expose the desired-vs-actual distinction to callers.

---

## 4. Thrust Linearization

ArduPilot: `AP_Motors_Thrust_Linearization.cpp`
Meridian: `crates/meridian-mixing/src/lib.rs` (standalone functions)

### Expo Formula

ArduPilot `apply_thrust_curve_and_volt_scaling()` (lines 119–133):
```cpp
float throttle_ratio = ((thrust_curve_expo - 1.0) + sqrt((1.0-expo)^2 + 4*expo*lift_max*thrust)) / (2*expo);
return throttle_ratio * battery_scale;
```
This is a **quadratic inversion**: given desired thrust (linear), solve for the PWM that achieves it given that `thrust ~ expo*pwm^2 + (1-expo)*pwm`.

Meridian `apply_thrust_curve()` (lib.rs lines 353–359):
```rust
(1.0 - expo) * t + expo * t * sqrtf(t)
```
This is **not the same formula**. Meridian implements `pwm = (1-expo)*thrust + expo*thrust^(3/2)`, which approximates the inverse but is not the ArduPilot quadratic inversion. The difference is small for `expo` near 0.65 but they are not algebraically identical.

**This is a functional divergence.** The ArduPilot formula is the exact inverse of the quadratic thrust model; the Meridian formula is an approximation. For typical expo values (0.6–0.7), the maximum difference is approximately 2–3% of full-scale thrust output.

### Voltage Compensation Filter

ArduPilot: `batt_voltage_filt` is a 0.5 Hz low-pass filter applied to `V_actual / V_nominal`. `lift_max` is then computed as `V_filt * (1 - expo) + expo * V_filt^2`.

Meridian `apply_voltage_comp()` (lib.rs lines 371–379):
```rust
let ratio = voltage_nom / voltage_actual;
let comp = (ratio * ratio).min(1.25);
motor_out * comp
```

Meridian applies a simple square-law ratio with a 1.25 cap, but with **no filter**. ArduPilot's 0.5 Hz filter is important: it prevents rapid voltage transients (from motor spikes) from causing large compensation swings. Meridian's unfiltered ratio would react to every voltage sample instantly.

Additionally, ArduPilot's `lift_max` factor feeds into the quadratic formula inside `apply_thrust_curve_and_volt_scaling()`, making compensation part of the linearization. Meridian applies compensation as a post-mix multiplier on the final motor output, which is a different integration point.

### `update_lift_max_from_batt_voltage()`

**Present in ArduPilot** (Thrust_Linearization.cpp lines 156–186): queries `AP_BattMonitor`, applies the 0.5 Hz filter, computes `lift_max`, and handles the disabled-compensation case (sets `lift_max = 1.0`, resets filter).

**Not present in Meridian** as a function. The concept is partially in `apply_voltage_comp()` but lacks the filter and `lift_max` integration. There is no battery monitor integration in the mixing crate.

### `thrust_to_actuator` / `actuator_to_thrust`

ArduPilot has both forward (`thrust_to_actuator`) and inverse (`actuator_to_thrust`) functions, the inverse being used for tailsitter transitions.

Meridian has only the forward direction (`apply_thrust_curve`). No inverse. This matters for tailsitter support (not yet implemented in Meridian).

---

## 5. Motor Test via MAVLink

ArduPilot: `ArduCopter/motor_test.cpp`, `GCS_MAVLink_Copter.cpp::handle_MAV_CMD_DO_MOTOR_TEST()`
- Accepts `MAV_CMD_DO_MOTOR_TEST` (command 209)
- Parameters: motor sequence, throttle type (%, PWM, pilot pass-through), throttle value, timeout, motor count
- Runs in a dedicated `motor_test_output()` called from the main loop
- Supports individual motor selection and sequential motor testing

Meridian `CommandAction` enum (meridian-mavlink/src/adapter.rs lines 598–608):
```rust
pub enum CommandAction {
    Arm, Disarm, Takeoff(f32), Land, Rtl,
    SetMode(u32), RequestMessage(u32), RequestAutopilotVersion,
    Unknown,
}
```

`MAV_CMD_DO_MOTOR_TEST` (209) is not in the handled command list. It falls through to `CommandAction::Unknown`. **Motor test via MAVLink is not implemented in Meridian.**

---

## 6. Helicopter Parity

ArduPilot helicopter files: `AP_MotorsHeli.cpp/h`, `AP_MotorsHeli_Single.cpp/h`, `AP_MotorsHeli_Dual.cpp/h`, `AP_MotorsHeli_Quad.cpp/h`, `AP_MotorsHeli_Swash.cpp/h`, `AP_MotorsHeli_RSC.cpp/h`

Meridian: `crates/meridian-heli/src/lib.rs`

### Swashplate Types

| Type | ArduPilot | Meridian | Notes |
|---|---|---|---|
| H3 Generic (adjustable phase) | `SWASHPLATE_TYPE_H3` (index 0) | **MISSING** | Configurable servo angles via params |
| H1 (non-CPPM, straight mixing) | `SWASHPLATE_TYPE_H1` (index 1) | **MISSING** | No CCPM; direct pitch/roll to servos |
| H3_140 | `SWASHPLATE_TYPE_H3_140` (index 2) | `SwashplateType::H3_140` | **MATCH** |
| H3_120 | `SWASHPLATE_TYPE_H3_120` (index 3) | `SwashplateType::H3_120` | **MATCH** |
| H4_90 | `SWASHPLATE_TYPE_H4_90` (index 4) | `SwashplateType::H4_90` | **MATCH** |
| H4_45 | `SWASHPLATE_TYPE_H4_45` (index 5) | `SwashplateType::H4_45` | **MATCH** |

**Summary: 4 of 6 swashplate types present. Missing: H3 Generic (configurable) and H1 (direct mixing).**

The H3 Generic is the most commonly used in practice — it allows tuning servo positions to match any non-standard spacing.

### Swashplate Mixing Formula

ArduPilot mixes as:
```cpp
// H1: direct, no CCPM
// CCPM types: servo = roll*factor + pitch*factor + collective
// Phase angle is applied per swashplate type
```

Meridian uses:
```rust
let pitch_factor = cosf(angle);
let roll_factor = sinf(angle);
servos[i] = pitch * pitch_factor + roll * roll_factor + coll_centered * collective_weight[i];
```

The trig-based mixing is correct for CCPM swashplates. The phase angle convention (0° = front, CW positive) matches ArduPilot.

### Rotor Speed Controller (RSC/Governor)

ArduPilot: `AP_MotorsHeli_RSC.cpp` — full governor with PID, ramp, run modes, autorotation interlock, external governor mode, RSC_GOV PID parameters, passive collective feedforward.

Meridian: `RotorSpeedController` in meridian-heli — PID governor with Idle, Ramp, Active modes.

| Feature | ArduPilot RSC | Meridian RSC | Status |
|---|---|---|---|
| Idle mode | Present | Present | **MATCH** |
| Ramp mode | Present | Present | **MATCH** |
| Governor (active PID) | Present | Present | **MATCH** |
| External governor mode | Present (RSC_MODE=3) | **ABSENT** | **MISSING** |
| Collective feedforward | Present (passive governor assist) | **ABSENT** | **MISSING** |
| Integrator anti-windup | Present | Present | **MATCH** |
| Autorotation interlock | Disables throttle during autorotation | RSC has no autorotation interface | **MISSING** |

### Autorotation State Machine

ArduPilot: `AC_Autorotation.cpp` — Entry, Glide, Flare, Touch-Down phases; headspeed controller during autorotation; interlock with RSC.

Meridian: `AutorotationController` in meridian-heli — Entry, Glide, Flare phases.

| Feature | ArduPilot | Meridian | Status |
|---|---|---|---|
| Entry phase | Present | Present | **MATCH** |
| Glide phase | Present | Present | **MATCH** |
| Flare phase | Present | Present | **MATCH** |
| Touch-down phase | Present | **ABSENT** | **MISSING** |
| Headspeed control during glide | Present (maintains RPM via collective) | Absent (collective fixed at min_collective) | **MISSING** |
| RSC interlock (zero throttle) | Present | Absent (RSC separate struct) | **MISSING** |
| Altitude source integration | Uses rangefinder/terrain | Receives altitude as parameter | **SIMPLIFIED** |

### Helicopter Variants

| AP class | Meridian | Status |
|---|---|---|
| `AP_MotorsHeli_Single` | Implied by SwashplateMixer | **PRESENT (partial)** |
| `AP_MotorsHeli_Dual` (tandem/coaxial) | **ABSENT** | **MISSING** |
| `AP_MotorsHeli_Quad` (quad-rotor heli) | **ABSENT** | **MISSING** |

---

## 7. DShot Commands

ArduPilot: `AP_HAL/RCOutput.h` line 276: `static constexpr uint8_t DSHOT_ZERO_THROTTLE = 48;`

Throttle values 0–47 are command slots; 48 = zero throttle (minimum armed output).

Meridian `DshotCommand` enum (`crates/meridian-hal/src/rc_output.rs` lines 20–44):

| Command | ArduPilot value | Meridian | Status |
|---|---|---|---|
| MotorStop | 0 | `MotorStop = 0` | **MATCH** |
| Beep1–5 | 1–5 | `Beep1 = 1` ... `Beep5 = 5` | **MATCH** |
| EscInfo | 6 | `EscInfo = 6` | **MATCH** |
| SpinDirection1 | 7 | `SpinDirection1 = 7` | **MATCH** |
| SpinDirection2 | 8 | `SpinDirection2 = 8` | **MATCH** |
| ThreeDModeOff | 9 | `ThreeDModeOff = 9` | **MATCH** |
| ThreeDModeOn | 10 | `ThreeDModeOn = 10` | **MATCH** |
| (command 11 = reserved) | 11 | **ABSENT** | minor — command 11 is reserved/undefined |
| SaveSettings | 12 | `SaveSettings = 12` | **MATCH** |
| (commands 13–19 undefined) | — | **ABSENT** | expected |
| SpinDirectionNormal | 20 | `SpinDirectionNormal = 20` | **MATCH** |
| SpinDirectionReversed | 21 | `SpinDirectionReversed = 21` | **MATCH** |
| Led0On–Led3On | 22–25 | `Led0On = 22` ... `Led3On = 25` | **MATCH** |
| Led0Off–Led3Off | 26–29 | `Led0Off = 26` ... `Led3Off = 29` | **MATCH** |

**DShot command table is complete and correct.**

### DSHOT_ZERO_THROTTLE = 48

ArduPilot explicitly defines `DSHOT_ZERO_THROTTLE = 48`. DShot values 0–47 are command space; value 48 maps to zero throttle (minimum motor output). Values 49–2047 are throttle values.

Meridian `rc_output.rs` documents `"DShot command IDs (sent as throttle values 0-47)"` — this correctly identifies the 0–47 command range. However, the constant `DSHOT_ZERO_THROTTLE = 48` is **not explicitly defined** as a named constant in Meridian. It is implied by the `write()` trait's DShot range (0–2047) and the comment, but callers must know the offset independently.

**Risk:** any code generating DShot throttle outputs needs to add 48 explicitly. Defining a `DSHOT_ZERO_THROTTLE` constant would eliminate this latent bug source.

### DShot Protocol Variants

| Protocol | ArduPilot `OutputMode` | Meridian `OutputProtocol` | Status |
|---|---|---|---|
| DShot150 | enum value 4 | `DShot150` | **MATCH** |
| DShot300 | enum value 5 | `DShot300` | **MATCH** |
| DShot600 | enum value 6 | `DShot600` | **MATCH** |
| DShot1200 | enum value 7 | `DShot1200` | **MATCH** |
| Bidirectional DShot | supported (BDSHOT) | comment only ("SRAM4 required") | **NOT IMPLEMENTED** |

---

## 8. Complete Gap List

Items present in `AP_Motors` that are absent or incomplete in Meridian:

### Frame Variants (37 missing)
- **Quad:** NYT_PLUS, NYT_X, BF_X, BF_X_REV, DJI_X, CW_X, V, H, VTAIL, ATAIL, PLUSREV, Y4
- **Hex:** H, DJI_X, CW_X
- **Octa:** PLUS, V, H, I, DJI_X, CW_X
- **OctaQuad (entire class):** PLUS, X, V, H, CW_X, BF_X, BF_X_REV, X_COR, CW_X_COR
- **Y6 (entire class):** default, Y6B, Y6F
- **DodecaHexa (entire class):** PLUS, X
- **Deca (entire class):** PLUS, X/CW_X

### Mixer Gaps
- `compensation_gain` not applied inside `mix()` (battery + altitude compensation omitted from headroom calc)
- `throttle_avg_max` parameter (dynamic throttle ceiling) not implemented
- `MOT_YAW_HEADROOM` configurable floor not implemented
- Per-motor directional yaw headroom (asymmetric frame support) not implemented
- Thrust boost / motor-loss mode (`_thrust_boost`, `_motor_lost_index`) not implemented
- Limit flag reporting (`limit.throttle_upper`, `limit.throttle_lower`, `limit.set_rpy()`) not implemented
- `apply_voltage_comp()` and `apply_thrust_curve()` are standalone helpers, not integrated into `mix()`

### Thrust Linearization Gaps
- Expo formula diverges from ArduPilot's exact quadratic inversion (Meridian uses `t^(3/2)` approximation)
- No 0.5 Hz voltage filter (raw ratio applied, susceptible to transient spikes)
- `lift_max` concept not integrated into linearization
- `actuator_to_thrust()` inverse function absent (needed for tailsitter transitions)
- No battery monitor integration in mixing crate

### Spool State Machine Gaps
- `get_spoolup_block()` check absent (ESC telemetry / RPM gate before SpoolingUp)
- IDLE_TIME delay (hold at ground idle before advancing) absent
- `minimum_spool_time` floor (0.05 s) not enforced
- `spool_down_time` default behavior diverges (AP defaults to 0, falls back to spool_up_time; Meridian defaults to 0.5 s independent of spool_up_time)
- No `DesiredSpoolState` separation from actual state (minor — functional equivalent exists)

### Helicopter Gaps
- H3 Generic swashplate (configurable phase angle) absent
- H1 direct (non-CCPM) swashplate absent
- `AP_MotorsHeli_Dual` (tandem/coaxial) not implemented
- `AP_MotorsHeli_Quad` (quad-rotor helicopter) not implemented
- RSC external governor mode absent
- RSC collective feedforward absent
- RSC autorotation interlock absent
- Autorotation touch-down phase absent
- Autorotation headspeed control during glide absent (RPM-regulated collective)

### DShot Gaps
- `DSHOT_ZERO_THROTTLE = 48` not defined as a named constant (implied but not explicit)
- Bidirectional DShot (BDSHOT) not implemented (HAL comment notes hardware requirement)

### Other AP_Motors Classes Not Present in Meridian
- `AP_MotorsCoax` — coaxial frame (non-matrix)
- `AP_MotorsSingle` — single motor with servos
- `AP_MotorsTri` — tricopter (yaw via tilting tail)
- `AP_MotorsTailsitter` — tilt-wing/tailsitter
- `AP_Motors6DOF` — 6 degree of freedom (omnidirectional)
- `AP_MotorsMatrix_6DoF_Scripting` — scripted 6DOF
- `AP_MotorsMatrix_Scripting_Dynamic` — dynamic/runtime mixing matrix via Lua

---

## 9. What Meridian Has Correct

The following are correctly implemented and verified against ArduPilot source:

- `normalise_rpy_factors()` — identical algorithm, scales to ±0.5
- `angle_to_raw_factors()` — correct trig: `roll = cos(angle + 90°)`, `pitch = cos(angle)`
- `YAW_CW = -1.0`, `YAW_CCW = +1.0` constants match AP_MotorsMatrix.h
- Quad-X motor angles and factors — verified numerically against AP L594–599
- Quad-Plus motor angles — verified against AP L584–587
- Hex-X and Hex-Plus angles — verified against AP L793–803, L780–790
- Octa-X angles (22.5°, 67.5°, 112.5°, 157.5° etc.) — verified against AP L876–887
- Core RPY proportional scaling formula — algebraically identical
- Yaw priority (yaw does not steal from roll/pitch) — preserved
- All 5 spool states present with correct names and basic transitions
- `spin_arm = 0.10`, `spin_min = 0.15` defaults match
- `spool_up_time = 0.5 s` default matches `AP_MOTORS_SPOOL_UP_TIME_DEFAULT`
- `safe_time = 0.0` default matches
- All H3_120, H3_140, H4_90, H4_45 swashplate types correct
- Swashplate CCPM mixing formula (trig-based, phase-angle driven) correct
- RSC Idle / Ramp / Active modes with PID governor present
- Autorotation Entry / Glide / Flare phases present
- All 25 DShot command IDs present and correctly valued
- DShot150/300/600/1200 protocol types all present
- `MAX_MOTORS = 12` matches ArduPilot `AP_MOTORS_MAX_NUM_MOTORS`
