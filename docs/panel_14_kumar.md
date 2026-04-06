# Panel Review #14 — Vijay Kumar, GRASP Lab, University of Pennsylvania

**Review Domain:** Control theory correctness, trajectory optimization, estimation-control coupling
**Files Reviewed:**
- `crates/meridian-ekf/src/core.rs` + `output.rs`
- `crates/meridian-control/src/attitude_controller.rs`
- `crates/meridian-control/src/position_controller.rs`
- `crates/meridian-nav/src/scurve.rs`
- `crates/meridian-sitl/src/physics.rs`

**Rating: NEEDS_ANALYSIS**

---

## Overall Assessment

The fundamental architecture is recognizably correct — a cascaded position-to-attitude controller feeding into a delayed-state EKF with output prediction is exactly the right structure for a GPS-aided multirotor. The team has clearly studied ArduPilot deeply and reproduced its core logic faithfully in Rust. That said, I have identified several issues of varying severity that require analysis before I can elevate this to THEORETICALLY_SOUND.

---

## 1. EKF Output Predictor: Structurally Sound, One Concern

The delayed-state EKF architecture is correctly implemented. The ring buffer (100 samples at 400 Hz = 250 ms of IMU data) properly compensates for the 220 ms GPS latency by ensuring the EKF state at fusion time corresponds to the measurement epoch. The output predictor correctly integrates IMU deltas at full rate and applies PI-corrected EKF feedback to all buffered states when fusion occurs — this matches the canonical NavEKF3 `calcOutputStates()` approach.

**Concern: The output predictor's bias correction applies `del_ang_correction` as an additive delta to the raw angular rate rather than as a multiplicative scale on the bias estimate.** Specifically, in `output.rs` line 210:

```
let del_ang = imu.del_ang - *gyro_bias * imu.del_ang_dt + self.del_ang_correction;
```

The `del_ang_correction` is a body-frame angle vector (rad), and it is added without scaling by `del_ang_dt`. This means the correction magnitude is independent of the timestep — correct for a one-shot correction applied per EKF fusion event, but only if the caller zeroes it out after one step. If it persists across multiple IMU steps without decay, it will integrate spuriously and introduce a rate bias. The apply_ekf_correction method should be reviewed to confirm the correction is either zeroed after application or scaled by the EKF rate, not the IMU rate.

**Vibration gating is correctly implemented:** inflating GPS position and velocity noise by 10x during excessive vibration reduces filter trust in contaminated measurements without triggering a full reset. This is conservative and appropriate.

The health state machine is overly simplistic. A ratio of predict_count to fuse_count greater than 50 transitions to Unhealthy, but this counter is never reset after a healthy fusion event. In a 400 Hz EKF with 10 Hz GPS, the steady-state ratio is 40, which leaves only 20% margin before false Unhealthy transitions. This needs a sliding window or periodic counter reset.

---

## 2. Cascaded Controller Architecture: Mostly Correct, Bandwidth Issue

The four-loop hierarchy (position → velocity → attitude → angular rate) is structurally sound. The bandwidth separations in steady state are:

- Position P-loop: `pos_xy_p = 1.0 rad/s` effective closed-loop bandwidth ~1 Hz
- Velocity PID: `kp = 2.0`, `ki = 1.0`, `kd = 0.5` — effective bandwidth ~3-5 Hz
- Attitude P-loop: `roll_p = pitch_p = 4.5` — bandwidth ~4-5 Hz
- Rate loop (not reviewed here, assumed ArduPilot default ~10-20 Hz)

The standard rule of thumb for cascaded loops is a 3-5x bandwidth separation between adjacent loops. Position (1 Hz) to velocity (3-5 Hz) gives roughly 3-5x — marginal but acceptable. Velocity (3-5 Hz) to attitude (4-5 Hz) is **less than 2x separation**, which is the primary concern. This means the velocity PID can command attitude changes faster than the attitude loop can execute them, which will produce oscillatory or sluggish velocity tracking at the crossover, particularly during aggressive maneuvers.

**Recommendation:** Reduce velocity loop bandwidth to ~1-2 Hz effective (reduce kp to 1.0-1.5) or increase attitude loop bandwidth (roll_p/pitch_p to 6-7), ensuring the attitude loop runs at least 3x faster than the velocity loop.

The `input_tc = 0.15 s` input shaping on the attitude controller appropriately band-limits attitude commands to ~1 Hz, which partially mitigates this concern for gentle maneuvers but does not resolve it for trajectory tracking.

---

## 3. Quaternion Error Convention: Correct but Requires Vigilance

The quaternion error is computed as `q_err = target^-1 * current`, which expresses the body frame's deviation from the target in the target frame. This matches ArduPilot's convention. The sign flip on the P-gain output is correct: the error vector's components represent "how much the body has over-rotated relative to target," and negative feedback drives it to zero.

The small-angle approximation `angle_err ≈ 2 * [q_err.x, q_err.y, q_err.z]` (with the sign check on `q_err.w` for shortest-path) is correct up to about 30 degrees of error, which is exactly the angle_max enforced by the controller. This is a consistent design.

**One subtle issue:** the feedforward computation on line 244-252 computes rate feedforward as `(rate_target.x - prev_rate_target.x)` — this is a finite difference of the shaped rate, not divided by `dt`. This is a derivative of the shaped rate command, which is dimensionally a rate acceleration (rad/s²), not a rate (rad/s). Adding this directly to a rate target will produce physically wrong feedforward units unless the inner rate loop interprets it as an acceleration feedforward rather than a rate bias. This needs clarification at the rate loop interface.

---

## 4. Position Controller: Forward Integration Issue

In `position_controller.rs`, the update method calls `sqrt_controller::update_pos_vel_accel()` (lines 207-216) to perform forward integration of the desired state, then immediately **overwrites** `pos_desired` with the raw target on lines 219-220:

```rust
self.pos_desired.x = target_pos.x;
self.pos_desired.y = target_pos.y;
```

This defeats the purpose of the forward integration entirely. The ArduPilot source uses `update_pos_vel_accel()` to smoothly advance the desired position toward the target position, tracking commanded changes with saturation feedback. By snapping directly to target_pos, the controller discards the integrator-based limit feedback and relies solely on `shape_pos_vel_accel()` for rate limiting. This will cause the `limits` saturation tracking to be incorrect (since it tracks saturation of `vel_desired`, which is now inconsistently derived), potentially causing integrator wind-up during constrained maneuvers.

This is the most significant correctness issue I found. The forward integration should operate on a smoothly tracked desired position, not be immediately overwritten. Either remove the `update_pos_vel_accel` calls (if the design intent is snap-to-target with saturation purely from the sqrt_controller) or remove the overwrite and trust the forward integrator.

The throttle computation `(hover_thr * (GRAVITY - accel_d) / GRAVITY)` is a linearized approximation that is valid near hover. It correctly uses the adaptive `hover_estimator` to adjust for mass uncertainty. The 10-second time constant on the hover throttle DC filter is appropriate for slow trim adaptation.

The `thrust_ref` computation for lean angle uses `(throttle * GRAVITY / hover_thr).max(GRAVITY * 0.5)`, which is a reasonable approximation of the actual vertical thrust component. This is consistent with ArduPilot's lean-angle-to-acceleration conversion.

---

## 5. S-Curve Trajectory: Mathematically Correct, Feasibility Boundary Issue

The 7-phase jerk-limited profile implementation is mathematically correct. The kinematic recursion through the phases (each building on the terminal state of the previous) is properly computed. The binary search in `compute_reduced_profile()` for short segments correctly finds the achievable peak velocity.

**One concern:** the upper bound for the binary search in `compute_reduced_profile()` uses:

```rust
let mut v_hi: f32 = libm::sqrtf(distance * jerk_max).min(accel_max * accel_max / jerk_max);
```

The second term `accel_max^2 / jerk_max` is the velocity reachable if you ramp acceleration to max and immediately begin ramping it down — this is the "triangular accel" regime. However, `sqrt(distance * jerk_max)` is derived from `s = J*t^3/6` solved for `t`, then substituted into `v = J*t^2/2` — this approximation assumes the entire distance is consumed by a single jerk phase, which underestimates the achievable velocity for longer segments. In practice, the binary search will converge correctly because `distance_for_velocity()` provides an accurate feasibility check, but the initial bounds may require more iterations for edge cases. The 30-iteration cap is sufficient for float32 precision.

**The 3D trajectory does not account for vertical dynamics.** `SCurveSegment` treats the trajectory as a purely 1D scalar profile projected onto the 3D direction vector. For level flight this is correct, but for trajectories with significant vertical component (climb or descent), the effective horizontal and vertical accelerations are coupled through the thrust vector. A 5 m/s² horizontal acceleration command during a 45-degree climb requires significantly more total thrust than the same maneuver in level flight. The position controller's angle_max = 30 deg limit will be reached before the trajectory's accel_max of 5 m/s² is achievable during combined climb-and-accelerate maneuvers. This is a feasibility gap that the trajectory planner does not communicate to the controller.

---

## 6. SITL Physics: Adequate for Validation, Missing Rotor Dynamics

The 6-DOF rigid body model at 1200 Hz is appropriate for outer-loop validation. The per-motor thrust with expo curve, body-frame torque from moment arms, and yaw reaction torque are all correctly formulated. The drag model using velocity-squared in body frame is physically correct for aerodynamic drag.

**Missing elements for high-fidelity validation:**

1. **First-order motor dynamics.** The simulation applies motor commands instantaneously. Real ESC/motor systems have a first-order lag of approximately 30-100 ms (depending on motor size and ESC firmware). This lag is a primary driver of inner-loop bandwidth limits. Without it, the SITL will appear more stable than a real vehicle at high gains — the rate loop will look better in simulation than in hardware.

2. **Gyroscopic precession.** The angular momentum of spinning rotors creates gyroscopic torques when the vehicle rotates. For large-rotor vehicles these are non-negligible. The current model has no rotor angular momentum term.

3. **Ground effect.** The ground contact model simply clamps vertical velocity, which is sufficient for landing detection but not for hover-in-ground-effect validation.

These omissions do not invalidate the architecture but mean that gains tuned in SITL will require adjustment in hardware, particularly for the inner rate loop.

The semi-implicit Euler integration (update velocity before position) is correct and numerically stable at 1200 Hz. The f64 position accumulator prevents precision loss at long range.

---

## 7. EKF→Controller Interface: Data Flow Concerns

The EKF output predictor correctly provides IMU-rate estimates of position, velocity, and attitude to the flight controller. However, I did not find explicit documentation of which EKF state is consumed by the controllers: the delayed EKF state or the output predictor's current state. For the controller to be stable, it must use the **current** output predictor state (output_buf.newest()), not the delayed EKF state. If the implementation accidentally uses the delayed state (~220 ms stale), the control loop will exhibit a large effective time delay and require substantially lower gains for stability.

This should be explicitly enforced at the interface boundary (type-level or documentation-level) to prevent accidental regression.

---

## Summary Table

| Subsystem | Status | Severity |
|---|---|---|
| EKF delayed-state architecture | Correct | — |
| Output predictor del_ang_correction persistence | Needs analysis | Medium |
| EKF health counter reset | Bug | Low |
| Velocity-to-attitude bandwidth separation | Marginal (< 2x) | Medium |
| Rate feedforward units (rad/s² vs rad/s) | Bug | Medium |
| pos_desired overwrite defeating forward integration | Logic error | High |
| S-curve 3D feasibility (vertical coupling) | Gap | Medium |
| SITL missing motor lag | Fidelity gap | Low-Medium |
| EKF state vs output predictor at controller input | Needs explicit documentation | Medium |

---

## Conclusion

The cascaded control architecture is sound in its broad structure and follows established practice in quadrotor control (Mahony, Brescianini, our own GRASP work). The EKF architecture is mature and properly addresses GPS delay compensation. The S-curve trajectory planner is mathematically well-implemented.

The system is not yet THEORETICALLY_SOUND due to three issues that are correctness concerns rather than merely fidelity gaps: the pos_desired overwrite defeating the forward integration, the feedforward units mismatch, and the velocity-to-attitude bandwidth separation being below the 3x minimum for reliable decoupling. These are fixable issues, not architectural failures.

I would upgrade to THEORETICALLY_SOUND after: (1) resolving the pos_desired overwrite, (2) clarifying feedforward units at the rate loop boundary, (3) verifying that the controller consumes the output predictor's current state, and (4) widening the velocity-to-attitude bandwidth ratio.

**Rating: NEEDS_ANALYSIS**
