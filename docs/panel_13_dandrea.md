# Panel Review 13 — Raffaello D'Andrea
## Motor Mixing, Thrust Modeling, Vehicle Dynamics
**Reviewer:** Raffaello D'Andrea, ETH Zurich / Institute for Dynamic Systems and Control  
**Domain:** Motor mixing correctness, desaturation algorithms, thrust curve linearization, SITL physics  
**Date:** 2026-04-02

---

## Overall Assessment

The implementation is serious work. The ArduPilot source tracing is meticulous, the normalization approach is sound, and the spool state machine is a faithful translation of AP_MotorsMulticopter. I found one confirmed defect (octa_dji_x / octa_cw_x duplication), one structural concern in the desaturation algorithm, two modeling issues in the SITL physics, and several minor points worth validating against hardware.

---

## 1. Frame Definitions and Motor Angles

### Quad Variants (14 frames)

**quad_x — CORRECT**  
Angles (45, -135, -45, 135) with YAW_CCW on diagonal pair and YAW_CW on the other diagonal match ArduPilot exactly. The test at line 1086 independently verifies: Motor 0 roll=-0.5, pitch=+0.5, yaw=+0.5 which is the correct front-right CCW motor in NED body convention. Symmetry test at line 1103 passes. No issues.

**quad_plus — CORRECT**  
Right motor (90°) and left motor (-90°) produce zero pitch contribution; front (0°) and back (180°) produce zero roll contribution. This is correct for a plus frame. The test at line 1115 confirms f[0].roll = -0.5 (right motor rolls left on starboard thrust increase — correct for right-hand roll convention).

**quad_nyt_plus / quad_nyt_x — CORRECT**  
Zero yaw factors are intentional for No-Yaw-Torque frames that use tilt for yaw. This is correct by design; however, it will produce zero yaw authority in simulation unless the tilt actuator path is also implemented. Note this as an integration risk, not a mixing defect.

**quad_bf_x — CORRECT**  
Motor order (back-right, front-right, back-left, front-left) matches BetaFlight 1-2-3-4 numbering. Spin directions are correct for standard BF layout.

**quad_bf_x_rev — CORRECT**  
Yaw factors are all inverted from quad_bf_x. This is the intended reverse-spin variant.

**quad_dji_x — CORRECT**  
Motor 1 front-right (45°, CCW), Motor 2 front-left (-45°, CW), Motor 3 back-left (-135°, CCW), Motor 4 back-right (135°, CW) matches DJI Phantom/Inspire motor assignment.

**quad_cw_x — CORRECT**  
Clockwise positional ordering with correct spin directions.

**quad_v — VERIFY**  
Asymmetric yaw factors: front motors use ±0.7981, rear use ±1.0. The intent is to compensate for the V geometry where rear motors have greater moment arm leverage for yaw. The value 0.7981 ≈ cos(37°), which is plausible but I cannot verify this without the physical geometry spec. This needs to be confirmed against an actual Quad-V frame's rear arm angle. If the frame is a pure X with motors physically at 45° but electronically remapped, the yaw factor asymmetry is wrong. If the rear arms are genuinely swept farther aft, it may be correct. **Flag for hardware verification.**

**quad_h — CORRECT**  
X geometry with all spin directions reversed from quad_x. This implements the H-frame inversion correctly.

**quad_vtail / quad_atail — VERIFY**  
These use from_raw() with precomputed trigonometric values rather than from_angles(). The values (0.8660, 0.9397) match cos(30°) and cos(20°) respectively. However, the V-tail and A-tail frames have physically tilted motor mounts; the roll/pitch factors should include the tilt projection. The code appears to encode the ArduPilot tabulated values directly which is acceptable, but without the original frame geometry drawing, I cannot confirm these are the correct pre-tilt projections. **Flag as VERIFY — needs cross-check against AP_MotorsMatrix.cpp:701-742 with the specific frame geometry.**

**quad_y4 — CORRECT**  
Left-front at (-1, 1, +1), right-front at (+1, 1, -1), coaxial rear pair at (0, -1, ±1). The rear motors share the same position hence identical roll=0 but produce pitch-down and counter-rotating yaw. This is the correct Y4 coaxial-rear configuration.

**quad_plus_rev — CORRECT**  
Spin directions inverted from quad_plus. Correct.

### Hex Variants (5 frames)

**hex_x — CORRECT**  
Six motors at 90°, -90°, -30°, 150°, 30°, -150°. Alternating CW/CCW in a standard hexarotor X layout. Yaw balance check: 3 CW + 3 CCW at symmetric positions — yaw torques sum to zero at equal throttle. Correct.

**hex_plus — CORRECT**  
The comment correctly warns about non-clockwise motor index ordering. ArduPilot's hex_plus uses a specific numbering that does not follow physical CW order around the frame. The code faithfully replicates this.

**hex_h — CORRECT**  
From_raw() with roll/pitch values structured as a 3×2 side layout. Left column (roll=-1) and right column (roll=+1) with three pitch levels each. CW/CCW alternation is correct for yaw balance.

**hex_dji_x — CORRECT**  
30° offset from standard hex_x. Motor 1 at 30° (CCW) is the DJI Spreading Wings / Matrice convention. Correct.

**hex_cw_x — CORRECT**  
Clockwise positional ordering. Spin directions alternate correctly.

### Octa Variants (7 frames)

**octa_plus — CORRECT**  
8 motors in plus/octagon. Front (0°) and back (180°) are CW, the four 45° diagonal positions are CCW, and left/right (±90°) are CW. Yaw balance: 4 CW + 4 CCW at equal angular spacing. Sum of yaw factors is zero. Correct.

**octa_x — CORRECT**  
22.5° offset. Alternating CW/CCW at 45° spacing. Correct and verified geometrically.

**octa_v / octa_h / octa_i — CORRECT**  
All use from_raw() with tabulated ArduPilot values. The V and H names refer to arm layout, not V-tail geometry. Values appear consistent with a swept-arm arrangement. Accepting as correct given source attribution.

**octa_dji_x — WRONG (duplicate of octa_cw_x)**  
This is a confirmed defect. `octa_dji_x()` (lines 421-432) and `octa_cw_x()` (lines 436-447) contain **byte-for-byte identical angle and yaw-factor tables**:

```
(22.5, YAW_CCW), (67.5, YAW_CW), (112.5, YAW_CCW), (157.5, YAW_CW),
(-157.5, YAW_CCW), (-112.5, YAW_CW), (-67.5, YAW_CCW), (-22.5, YAW_CW)
```

AP_MotorsMatrix.cpp lines 935-948 (DJI-X) and 950-963 (CW-X) define **different motor ordering**, not different angles. DJI-X uses DJI motor numbering (front-right = 1), CW-X uses clockwise positional ordering. The angle values may be the same, but the frame constructor is called with the same physical indices either way, so what differs between the two variants in ArduPilot is purely which physical motor connector maps to which software channel.

In Meridian, since the mixing matrix encodes physics (not wiring), these two frames should indeed produce identical mix matrices if the angles are the same. However, the function `octa_dji_x()` should at minimum document this explicitly, or alternatively, if the intent was to encode a different DJI-specific spin direction (some DJI octarotors use different spin patterns), the yaw factors need to be audited against actual DJI hardware docs. As written, users who call `octa_dji_x()` expecting DJI-specific behavior will get CW-X behavior without warning.

**Action required:** Either document that the two functions are physically equivalent and differ only in connector/wiring convention (add a comment), or fix the DJI-X yaw factors to match the actual DJI spin pattern if it differs. Do not leave two silently identical functions — this is a maintenance trap.

### OctaQuad Variants (9 frames)

**octaquad_plus / octaquad_x / octaquad_h / octaquad_cw_x / octaquad_bf_x / octaquad_bf_x_rev — CORRECT**  
Coaxial pairs at each arm position with opposing yaw factors per pair. At equal throttle, each pair's yaw torques cancel (CW + CCW). Yaw authority is available because the upper/lower motors are individually commanded. Correct.

**octaquad_v — VERIFY**  
Same 0.7981 asymmetric yaw factor as quad_v, applied to the coaxial variant. The same caveat applies: the physical basis for 0.7981 needs confirmation. **VERIFY against hardware.**

**octaquad_x_cor / octaquad_cw_x_cor (co-rotating) — CORRECT with note**  
The OCTAQUAD_COROTATING_SCALE = 0.7 factor scales down the top-layer motors' roll/pitch/yaw authority. This is consistent with ArduPilot's approach where co-rotating pairs use differential throttle between layers for yaw, and the scale factor accounts for the aerodynamic coupling difference between co-rotating and counter-rotating configurations.

The raw factors (-0.3536, +0.3536) ≈ ±1/√8 which are the correct position projections for a 45° arm. The 0.5 yaw factors are pre-normalization. The use of OCTAQUAD_COROTATING_SCALE on the top layer but not the bottom layer is intentional — it reflects that the top and bottom motors have different effective yaw authority due to rotor interaction effects. This is physically reasonable. **CORRECT, but the 0.7 value is empirical — it should be noted as a tuning parameter that may need adjustment for specific frame geometries.**

### Y6 Variants (3 frames)

**y6() — VERIFY**  
The Y-arms are at left (-1, 0.666), right (+1, 0.666), rear (0, -1.333). The pitch factor of 1.333 for the rear arm implies the rear has greater pitch leverage than the side arms, which is geometrically correct for a Y6 where the single rear arm is typically longer. However, the raw pitch factors before normalization are asymmetric: 0.666 × 2 sides + 1.333 × 1 rear = 2.665. After normalization the rear pitch factor will be ≈ ±0.5 and the front pair will be ≈ ±0.25, which seems like a large pitch authority imbalance. **VERIFY the arm length ratio against actual Y6 geometry.**

**y6b() — CORRECT**  
Symmetric factors: left/right at roll=±1, pitch=0.5; rear at roll=0, pitch=-1. This is a 2:1 pitch arm ratio (rear arm twice the lateral arm length), which is typical. CW/CCW pairs on each arm cancel yaw within pairs. Correct.

**y6f() — CORRECT**  
Same geometry as y6b but with inverted yaw convention (upper motors CCW, lower motors CW versus y6b's upper CW / lower CCW). This is the FireFly Y6 specific convention. Correct.

### DodecaHexa Variants (2 frames)

**dodecahexa_plus / dodecahexa_x — CORRECT**  
12 motors in 6 coaxial pairs at 60° spacing. Each pair has opposing yaw factors. DodecaHexa-X is DodecaHexa-Plus rotated 30°. Both are consistent with stacked hex geometry. Correct.

### Deca Variants (2 frames)

**deca_plus / deca_x — CORRECT**  
10 motors at 36° spacing, alternating CCW/CW. Deca-X is rotated 18° from Deca-Plus. This is the standard decacopter convention. Yaw balance: 5 CW + 5 CCW at equal spacing — sum is zero. Correct.

---

## 2. Angle-to-Factor Conversion

The `angle_to_raw_factors()` function at lines 43-51:

```rust
roll: libm::cosf(angle_rad + PI / 2.0),   // = -sin(angle_rad)
pitch: libm::cosf(angle_rad),
```

This means: a motor at angle θ from front (CW positive) contributes:
- pitch: cos(θ) — positive when θ=0 (front motor), negative when θ=180° (rear motor)
- roll: -sin(θ) — negative when θ=90° (right motor), positive when θ=-90° (left motor)

In NED body frame with x-forward, y-right, z-down: a front motor increasing thrust produces pitch-down (negative pitch moment in some conventions) or pitch-up depending on sign convention. The test at line 1115 confirms `f[2].pitch = 0.5` for the front motor (angle=0°, YAW_CW), meaning a positive pitch command increases the front motor — this is pitch-backward (nose up) which is correct for the ArduPilot pitch convention where positive pitch = nose up.

**Roll convention check:** For quad_plus, the right motor (90°) has roll = cos(90° + 90°) = cos(180°) = -1.0 (pre-normalization), normalized to -0.5. A positive roll command (roll right) decreases the right motor? That implies the right motor must decrease for roll-right, which would mean roll is commanded by decreasing the motor on the rolling side. This is **inverted from what I would expect.** In standard ArduPilot convention, roll right means the right motors decrease and left motors increase. With roll factor = -0.5 on the right motor, `output = throttle + roll * (-0.5)`, so positive roll increases right motor output. Wait — let me check the test.

The test at line 1115: `f[0].roll = -0.5` for Motor 0 which is the **right** motor (angle=90°, YAW_CCW). So positive roll command adds (-0.5) × roll to the right motor, reducing it. That means positive roll = left roll = right motor decreases. This matches the ArduPilot convention where positive roll is **roll left** (starboard motor decreases). Cross-checking with the roll_right test at line 1133: it calls `mixer.mix(0.3, 0.0, 0.0, 0.5)` and asserts motors with `roll > 0` increase and motors with `roll < 0` decrease. For quad_plus, the left motor has roll = +0.5 and the right motor has roll = -0.5. So mixing roll = +0.3 increases the left motor and decreases the right motor — that is roll-**left** in the physical world.

The test comment says "roll right" but the physics is roll left. **This is a documentation/test labeling error, not a physics error.** The math is consistent with ArduPilot where the positive roll axis is X (forward), and positive roll is right-hand rotation — which means the left side goes up and the right side goes down, i.e., the aircraft rolls right. With right motor decreasing, the left motor is higher thrust, left side goes up — that is indeed roll right (right-hand rule: positive X rotation lifts the left wing). 

**CORRECT** — the convention is internally consistent. The test label is accurate if you think in terms of right-hand-rule body axes.

---

## 3. Normalization Algorithm

The `normalize()` function (lines 96-115) scales roll, pitch, yaw independently to ±0.5 and throttle to 1.0. This matches ArduPilot's `normalise_rpy_factors()`.

**One concern: independent-axis normalization can distort the effective control coupling.** If a frame has naturally different max roll and pitch authority (e.g., an asymmetric hex or the Y6 variants), normalizing each axis independently removes that physical asymmetry from the matrix. The physical vehicle may have twice the pitch authority of roll authority, but the mixer will treat them as equal. The flight controller's PID rates would need to be calibrated to match. This is the ArduPilot approach, so it is correct by convention, but it means the mixing matrix does not represent physical authority ratios — it represents normalized demand fractions.

**CORRECT as an ArduPilot-compatible implementation. Note for users: PID tuning must compensate for any physical axis asymmetry.**

---

## 4. Desaturation Algorithm

The `mix_compensated()` function (lines 922-1059) implements ArduPilot's `output_armed_stabilizing()`.

**Step-by-step assessment:**

**Steps 1-3 (compensation gain, RP computation) — CORRECT.**

**Step 4 (yaw headroom calculation, lines 954-971) — VERIFY.**  
The code computes `motor_room` as either `(1.0 - thrust_rp_best_throttle)` or `thrust_rp_best_throttle` depending on whether yaw and the motor's yaw factor are aligned. This is the ArduPilot approach. However, `throttle_thrust_best_rpy` is clamped to `min(throttle_avg_max, 0.5)` at line 945. If `throttle_avg_max` is passed as 0.5 (the default in `mix()`), and if RP contributions push individual motors above 0.5, the motor_room calculation may underestimate available headroom. This is acceptable for safety (conservative yaw limiting) but can result in unnecessary yaw limiting at moderate throttle. **Flag as VERIFY — test with high-RP + moderate throttle scenarios.**

**Step 5-6 (yaw_headroom floor and thrust_boost) — CORRECT.**  
The `yaw_headroom` floor of 200 thousandths = 0.2 matches ArduPilot MOT_YAW_HEADROOM default. Thrust boost halves the floor when a motor is lost, which is the correct compensation.

**Step 9 (RPY proportional scale, lines 1009-1032) — CONCERN.**  
The secondary floor at lines 1018-1025:
```rust
if rpy_low < 0.0 {
    let scale_floor = -throttle_avg_max / rpy_low;
    if scale_floor < rpy_scale {
        rpy_scale = scale_floor;
    }
}
```
This prevents the RPY contribution from pulling any motor below `-throttle_avg_max`. With `throttle_avg_max = 0.5`, this means no motor will be pulled more than 0.5 below the throttle-adjusted value. This is correct in intent, but note that at very low throttle (e.g., 0.1), `rpy_low` could be large negative, and `scale_floor` could be very small, resulting in heavy RPY scaling even for modest attitude commands. This is a fundamental ArduPilot design choice (prioritize throttle floor), not a bug, but it means the vehicle becomes sluggish in attitude control at low throttle. **CORRECT by design. Note for flight controller tuning.**

**Step 10 (throttle adjustment) — CORRECT.**  
The thr_lo / thr_hi window correctly bounds the throttle to a range that keeps all motors within [0,1]. The midpoint fallback when `thr_lo > thr_hi` (a saturated-beyond-recovery condition) correctly sets both limit flags and outputs the midpoint.

**Step 11 (final output with per-motor throttle factor) — CORRECT.**  
The `f.throttle` factor here supports the OctaQuad co-rotating scale where top-layer motors have throttle = 0.7 and bottom = 1.0. This correctly modulates the throttle contribution per motor. **CORRECT.**

**Missing: differential scaling for RPY channels under compound saturation.** ArduPilot's full algorithm has an additional pass that proportionally reduces RPY when the RP+yaw combination cannot be simultaneously satisfied. The current implementation scales RPY as a single block (rpy_scale applies to all three axes equally). This means under compound roll+pitch+yaw saturation, all axes are reduced proportionally. A more optimal desaturation would reduce yaw first (lowest priority) and preserve RP. The `yaw_allowed` clamping in Step 7 partially addresses this, but once yaw is clamped, the remaining RP budget is not re-optimized. For most cases this will be adequate. **VERIFY — run compound saturation tests with yaw + full RP simultaneously.**

---

## 5. Thrust Curve Linearization

The `apply_thrust_curve()` function (lines 747-769) inverts the model:

```
thrust = expo * pwm² + (1 - expo) * pwm
```

using the quadratic formula:

```
pwm = (-(1-expo) + sqrt((1-expo)² + 4*expo*thrust)) / (2*expo)
```

**This is mathematically correct.** The discriminant is always non-negative for expo ∈ (0,1] and thrust ∈ [0,1] because `b² = (1-expo)² ≥ 0` and `4*a*t = 4*expo*thrust ≥ 0`. The fallback `thr_scaled` when discriminant < 0 is unreachable but harmless.

**The inverse `actuator_to_thrust()` at lines 773-777 is also correct** — it directly applies the forward model.

**Voltage compensation `lift_max` computation (lines 826-827):**
```rust
let lift_max = self.expo * f * f + (1.0 - self.expo) * f;
```
This computes what thrust is produced at `actuator = voltage_ratio`. This is correct: at a given voltage ratio f, the maximum achievable thrust (normalized) is the forward model evaluated at f. The resulting `lift_max` is then used to scale down the thrust demand via `apply_thrust_curve_with_comp`. **CORRECT.**

**The 0.5 Hz LP filter on voltage ratio (lines 819-823) — CORRECT.** The RC constant is properly computed as `1/(2π × 0.5)`. Alpha = dt/(RC + dt) is the standard first-order IIR formulation.

**One concern with `apply_voltage_comp()` (lines 845-850):**
```rust
let comp = (ratio * ratio).min(1.25);
```
This is the legacy (non-linearized) voltage compensation that applies a quadratic correction to motor output. Capping at 1.25 means maximum 25% over-compensation. This is a simplified approach that does not account for thrust curve expo. It is labeled "legacy" and should not be used in production flight paths. **CORRECT as a legacy fallback; document clearly that this should not be used with the linearized thrust curve.**

---

## 6. SITL Physics Model

### Thrust Model

**Lines 133-135:**
```rust
let linearized = (1.0 - motor.expo) * cmd + motor.expo * cmd * cmd;
let thrust = motor.max_thrust * linearized;
```

This applies the forward thrust model in the physics integrator. This is correct and consistent with `actuator_to_thrust()` in the mixing crate. **CORRECT.**

### Yaw Reaction Torque

**Line 147:**
```rust
let yaw_torque = motor.yaw_factor * cmd * motor.yaw_coupling * thrust;
```

**This has a dimensional problem.** `thrust` is in Newtons. `yaw_coupling = 0.05 * diagonal_m` is in meters. The product `yaw_coupling * thrust` has units of N·m (torque) — correct. But then multiplying by `cmd` (dimensionless) means the reaction torque scales as `cmd × thrust`, which is `cmd × Tmax × f(cmd)`. Since thrust already encodes `cmd` via the expo function, this double-counts the command influence on yaw torque.

The correct formulation should be: reaction torque = k_Q × Ω² ≈ k_Q_effective × thrust (since thrust ≈ k_T × Ω²). The torque-to-thrust ratio k_Q/k_T is a fixed constant for a given propeller, typically 0.01-0.1 of the arm length. The ArduPilot SIM_Motor.cpp computes yaw reaction torque as proportional to thrust alone, not to `cmd × thrust`.

**This is WRONG. The `* cmd` factor should be removed.** The torque should be:
```rust
let yaw_torque = motor.yaw_factor * motor.yaw_coupling * thrust;
```
As written, at hover (cmd ≈ 0.35-0.40), the yaw torque is ~35-40% of its correct value. At full throttle (cmd=1.0), it would be exactly correct. This means the yaw response in SITL will appear too weak at hover throttle and progressively stronger at high throttle. This will cause the yaw PID to be mistuned — gains tuned in SITL will be too aggressive when transferred to hardware.

### Rotational Damping

**Lines 96-100:**
```rust
let terminal_rad = vp.terminal_rotation_rate * core::f32::consts::PI / 180.0;
let rot_damping = if terminal_rad > 0.0 {
    (400.0 * core::f32::consts::PI / 180.0) / terminal_rad
} else { 0.0 };
```

Then applied as:
```rust
let damping = Vec3::new(-state.gyro.x * params.rotational_damping, ...);
```

The formula `damping_coeff = (400 deg/s in rad/s) / terminal_rate` gives a linear viscous drag coefficient such that at `terminal_rotation_rate`, the aerodynamic damping torque equals the maximum applied torque (400 deg/s · I being the approximate max angular acceleration × inertia). This is the ArduPilot SIM_Frame.cpp approach and is physically reasonable as a linearized aerodynamic drag. **CORRECT as a first-order approximation.** Note that real quadrotors have a nonlinear damping characteristic (scales with Ω²), but the linear model is sufficient for SITL at moderate rates.

### Ground Contact

**Lines 192-240:**  
The ground contact model clamps position[2] to 0.0 (ground level in NED positive-down, so position[2] ≥ 0 = on or below ground), zeros downward velocity, and kills all rotation. This is a hard-stop model with no bounce or friction. **CORRECT for basic SITL.** Missing: landing gear compliance, friction model, surface reaction forces. These omissions are acceptable for initial development but will cause unrealistic ground interactions (instantaneous rotational arrest, no lateral sliding).

**Bug: inconsistent ground detection logic (lines 194-202 vs 229-240).**  
The first check (lines 194-202) activates when `state.on_ground` is already true and `position[2] >= 0.0`. The second check (lines 229-240) sets `state.on_ground = true` when `position[2] > 0.0`. There is a sign inconsistency: NED convention has Z positive downward, so position[2] > 0 means below ground. But the first block checks `position[2] >= 0.0` (below or at ground level) while the second also checks `> 0.0`. The comment says "Below ground — clamp" at line 230 which is correct for NED. The altitude function at line 48 returns `-position[2]` as positive altitude. This is consistent, but the conditional on line 196 should use `> 0.0` not `>= 0.0` to match the landing check at line 229. At exactly position[2] = 0.0 the vehicle is at ground level and the takeoff/landing logic may oscillate. **VERIFY — test with slow descent to ground.**

### Integration Method

**Semi-implicit Euler at 1200 Hz (lines 205-226) — CORRECT.**  
The sequence: update gyro → integrate attitude → update velocity → integrate position is the standard semi-implicit Euler order. At 1200 Hz, the truncation error is small enough for SITL purposes. For stiff attitude dynamics (high-frequency oscillation modes), this will be numerically stable. **CORRECT.**

**Quaternion normalization at every step (line 219) — CORRECT.**  
Essential to prevent quaternion drift. Done correctly.

**f64 position, f32 velocity/attitude — CORRECT.**  
The position uses f64 for GPS-level precision over long ranges. Velocity and attitude remain f32. This matches ArduPilot SITL practice. The cast to f64 in position integration (lines 224-226) is correct.

### Helicopter SITL

The physics.rs step function assumes a multirotor model (upward thrust per motor, reaction torques from spin). The meridian-heli crate implements swashplate mixing and RSC but there is no helicopter-specific physics step. Helicopter aerodynamics are fundamentally different: rotor flapping, gyroscopic precession, translational lift, retreating blade stall, and tail rotor cross-coupling are all absent. **If helicopter SITL is in scope, a separate physics path is required. The current model will produce qualitatively wrong helicopter dynamics.**

---

## 7. Swashplate Mixer (meridian-heli)

**H3_120 / H3_140 / H4_90 / H4_45 — CORRECT.**  
Phase angles, collective weights, and CCPM mixing formula are standard. The mix formula:
```rust
servos[i] = pitch * cos(angle) + roll * sin(angle) + coll_centered * weight[i]
```
is the correct CCPM mixing equation. Collective is centered to [-1,1] via `2*collective - 1`. **CORRECT.**

**H1 — CORRECT.**  
Direct-mode mixing: pitch to servo 0, roll to servo 1, collective to servo 2. This is correct for H1 non-CCPM helis.

**H3_Generic — CORRECT.**  
Falls back to H3_120 default angles until custom parameters are set. Reasonable default.

**Dual helicopter (HeliDual) — VERIFY.**  
Tandem yaw via differential collective is correct for CH-47-style aircraft. Coaxial yaw via differential collective is mechanically correct but only works if the two rotors have different blade pitch — if both are collectively pitched the same, yaw control is via the torque imbalance. The implementation clamps coll_upper and coll_lower independently, which can cause a net collective change when yaw is applied (mean = collective + yaw_diff + collective - yaw_diff = 2*collective only if not clamped). When clamping is asymmetric, the mean changes. **VERIFY — test that yaw input does not cause altitude change in coaxial mode.**

---

## 8. RSC Governor

The PI(D) governor with collective feedforward is sound. The default gains (kp=0.002, ki=0.001, kd=0.0001) are very conservative — appropriate for a default that won't oscillate, but real helis typically need kp in the 0.005-0.02 range. The `ramp_target = 0.5` hardcoded in Ramp mode (line 336) means the ramp always goes to 50% throttle before handing off to the governor. If the hover throttle is not near 50%, the transition to Active mode will produce a step change in throttle output. **VERIFY — the Ramp mode ramp_target should ideally be the initial integrator seed, not a hardcoded 0.5.**

---

## 9. Frames That Risk Uncommanded Rotation

The following frame types carry specific risk of uncommanded rotation if misconfigured or misconnected:

1. **quad_vtail / quad_atail** — The asymmetric pitch factors for front vs rear motors mean that any collective error (e.g., one motor consistently lower output) will produce both a pitch moment and an uncommanded yaw moment, since the yaw authority is concentrated in the rear motors. If Motor 2 or 4 (rear, yaw-authority motors) are reversed, the vehicle will spin on arming.

2. **quad_v** — The 0.7981/1.0 asymmetric yaw factors mean the yaw mixing weight differs between front and rear. If rear motors are under-powered (aging battery, motor wear), the asymmetric yaw weighting will not cancel, producing a persistent yaw drift that the controller cannot fully correct.

3. **y6() (Y6-A)** — The pitch factor asymmetry (0.666 front / 1.333 rear, pre-normalization) means small throttle errors in the rear motor pair will produce disproportionately large pitch disturbances. Combined with co-rotating upper/lower pairs, a failed motor in the rear pair leaves pitch control severely compromised with no symmetric redundancy.

4. **octaquad_x_cor / octaquad_cw_x_cor (co-rotating)** — The OCTAQUAD_COROTATING_SCALE = 0.7 reduces top-layer mixing authority. If this scale factor does not match the actual aerodynamic torque ratio of the specific propeller/motor combination, residual yaw torque from the top layer will not be cancelled by the bottom layer. The 0.7 value is empirical and frame-specific. **Miscalibration will produce uncommanded slow yaw rotation at hover.** This is the highest risk for uncommanded rotation among all 42 frame types.

5. **Any NYT frame (quad_nyt_plus, quad_nyt_x)** — Zero yaw torque means any propeller imbalance that produces a net reaction torque will result in uncommanded yaw that the mixer literally cannot correct. These frames require tilt actuators for yaw, which are not present in the multirotor mixer path.

---

## Summary Ratings

| Item | Rating |
|------|--------|
| Quad frames (14 variants) | CORRECT (quad_v, quad_vtail, quad_atail: VERIFY) |
| Hex frames (5 variants) | CORRECT |
| Octa frames (7 variants) | WRONG — octa_dji_x is a duplicate of octa_cw_x |
| OctaQuad frames (9 variants) | CORRECT (octaquad_v: VERIFY; cor scale: VERIFY) |
| Y6 frames (3 variants) | CORRECT (y6() pitch asymmetry: VERIFY) |
| DodecaHexa frames (2 variants) | CORRECT |
| Deca frames (2 variants) | CORRECT |
| Normalization | CORRECT |
| Desaturation algorithm | CORRECT (compound saturation: VERIFY) |
| Thrust curve linearization | CORRECT |
| Voltage compensation (LP filter) | CORRECT |
| Voltage compensation (legacy) | CORRECT (label as deprecated) |
| Spool state machine | CORRECT |
| SITL physics — force/torque model | WRONG — yaw_torque double-counts cmd |
| SITL physics — integration | CORRECT |
| SITL physics — ground contact | VERIFY — sign edge case at position[2]=0 |
| Swashplate mixer | CORRECT (HeliDual coaxial: VERIFY) |
| RSC Governor | CORRECT (ramp_target hardcode: VERIFY) |
| Helicopter SITL physics | WRONG — no helicopter-specific physics path |

---

## Priority Actions

1. **[WRONG — fix now]** `octa_dji_x()` is a copy-paste duplicate of `octa_cw_x()`. Audit AP_MotorsMatrix.cpp lines 935-963 and either differentiate the functions or document explicitly that they are physically identical with different wiring conventions.

2. **[WRONG — fix now]** SITL yaw reaction torque at `physics.rs:147`: remove the `* cmd` factor. The torque should be proportional to thrust, not to `cmd × thrust`.

3. **[WRONG — not in scope yet but flag]** No helicopter physics step in SITL. Swashplate mixing is implemented but will connect to the multirotor physics path. This will produce qualitatively incorrect helicopter dynamics.

4. **[VERIFY]** `octaquad_x_cor` and `octaquad_cw_x_cor` OCTAQUAD_COROTATING_SCALE = 0.7 is empirical. Measure actual torque ratio on target hardware; miscalibration will produce sustained yaw drift.

5. **[VERIFY]** RSC Ramp mode `ramp_target = 0.5` should not be hardcoded. Use `throttle_output` at transition time as integrator seed for bumpless transfer.

6. **[VERIFY]** Ground contact edge case: position[2] == 0.0 boundary condition between on_ground checks.
