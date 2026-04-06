# Panel Round 2 — Wave 1
_Verification of Round 1 fixes. Date: 2026-04-02._

---

## Persona 1 — Andrew Tridgell (System Architecture)

**T1: gyro_corrected() returns raw_gyro - bias**
FIXED. `ahrs/src/lib.rs` line 131–138: `gyro_corrected()` explicitly computes
`last_raw_gyro.{x,y,z} - bias.{x,y,z}` component-wise.

**T2: DCM fallback wired and auto-switches on EKF unhealthy**
FIXED. `ahrs/src/lib.rs` lines 66–80: `dcm.update_imu()` is called every tick (stays warm),
and the source FSM switches to `AhrsSource::Dcm` when `ekf.health != EkfHealth::Healthy`,
back to `AhrsSource::Ekf` when healthy.

**T3: SITL uses sensor chain, not raw physics gyro**
FIXED. `bin/meridian-sitl/src/main.rs` lines 268–276: rate controller receives
`corrected_gyro = imu.gyro - ekf_state.gyro_bias` (from the sensor→EKF pipeline),
not `physics.gyro`. Comment at line 269 documents the intent.

**T4: Arming blocks without EKF healthy**
FIXED. `crates/meridian-arming/src/lib.rs` lines 234–240: `check_prearm` evaluates
`ArmingCheck::EkfHealth` and fails with "Not healthy" when `state.ekf_healthy == false`.
Test `test_ekf_unhealthy_fails` at line 511 confirms the block.

---

## Persona 2 — Paul Riseborough (EKF)

**R1: GSF wired into EkfCore**
FIXED. `ekf/src/core.rs` lines 303–307: `gsf.align_tilt()` called on first samples,
`gsf.predict()` called every IMU tick. `gsf: GsfYaw` field declared at line 209 and
initialized at line 264. `GsfYaw` imported from `gsf_yaw` module.

**R2: Output predictor gain = 2π × hrt_filt_freq × dt / time_delay**
FIXED. `ekf/src/output.rs` line 283:
`error_gain = 2.0 * PI * hrt_filt_freq * dt_ekf / time_delay.max(0.001)`
exactly matches the ArduPilot formula. Comment at lines 278–282 documents the previous bug.

**R3: Transition to AidingMode::Relative when optical flow available**
FIXED. `ekf/src/core.rs` lines 487–495: on GPS loss (`!fix_3d`) after 50 consecutive
missed updates, mode transitions to `AidingMode::Relative` if `has_optical_flow` is true,
otherwise to `AidingMode::None`.

**R4: Airspeed gate 3σ default**
STILL_BROKEN. `ekf/src/fusion.rs` line 658: `let gate = 10.0; // wide gate for airspeed`.
Default is 10.0 (σ²), not 3.0. The 3σ standard (gate = 9.0 = 3²) is not met; no
`EkfParams` field for airspeed gate exists.

**R5: Mag reset flags consumed in fuse_mag**
FIXED. `ekf/src/core.rs` lines 524–538: `fuse_mag()` checks
`alignment.need_mag_body_var_reset` before calling `fuse_magnetometer()` and clears
the flag (`= false`) after resetting body mag covariance states 19–21.

**R6: Health FSM tracks sustained innovation ratio failures**
FIXED. `ekf/src/core.rs` lines 375–379: `innov_ratio_fail_count >= innov_ratio_fail_limit`
(default 10 consecutive GPS-rate failures ≈ 1 s) transitions health to `Unhealthy`.
Lines 477–483 increment/reset the counter per GPS fusion. Both pass and fail counters present.

**Bonus — gyro_bias_process_noise = 1e-3**
FIXED. `ekf/src/params.rs` line 101: `gyro_bias_process_noise: 1.0e-3`.

**Bonus — strapdown pre-update**
FIXED. `ekf/src/predict.rs` file comment line 1: "strapdown navigation equations";
`predict_state()` at line 203 runs full strapdown (correct DCM rotate-then-integrate order).

---

## Persona 3 — Leonard Hall (Control System)

**LowPassFilter2p exists in notch_filter.rs and is wired into PID D-term**
STILL_BROKEN (partial). `LowPassFilter2p` struct is fully implemented in
`notch_filter.rs` lines 202–306 and `pid.rs` declares `d_lpf2p: Option<LowPassFilter2p>`
(line 104) with a path that uses it when `Some` (line 263–264). However, `set_d_lpf2p()`
is never called anywhere in the codebase — `d_lpf2p` is always `None` at runtime.
The infrastructure is present but not activated for roll/pitch/yaw rate PIDs.

**Position controller dual-P removed**
FIXED. `position_controller.rs` lines 274–282: comment "H2: Use the shaped velocity from
step 2 as the PID target, not an independent pos_err * pos_xy_p computation" confirms
only the sqrt-controller shaped velocity feeds the velocity PID, eliminating double-P.

**vel_xy imax = 0.7**
FIXED. `position_controller.rs` line 37: `imax: 0.7` in `PositionGains::default()`.

**Notch a2 coefficient uses alpha/(A*A)**
FIXED. `notch_filter.rs` line 153: `self.a2 = 1.0 - alpha / a_sq` where `a_sq = a * a`.
This matches `1 - alpha/A²` per the ArduPilot formula.

**Rate feedforward has /dt**
FIXED. `pid.rs` line 233: `target_deriv = (self.filtered_target - self.prev_target) / dt`
for the `kd_ff` term. Division by dt is correct.

---

## Summary Table

| ID | Persona | Status |
|----|---------|--------|
| T1 | Tridgell | FIXED |
| T2 | Tridgell | FIXED |
| T3 | Tridgell | FIXED |
| T4 | Tridgell | FIXED |
| R1 | Riseborough | FIXED |
| R2 | Riseborough | FIXED |
| R3 | Riseborough | FIXED |
| R4 | Riseborough | STILL_BROKEN |
| R5 | Riseborough | FIXED |
| R6 | Riseborough | FIXED |
| L1 (LPF2p wired) | Hall | STILL_BROKEN |
| L2 (dual-P removed) | Hall | FIXED |
| L3 (imax 0.7) | Hall | FIXED |
| L4 (notch a2) | Hall | FIXED |
| L5 (ff /dt) | Hall | FIXED |

**2 items remain open: R4 (airspeed gate hardcoded 10.0, not 3σ=9.0) and L1 (LPF2p never activated on rate PIDs).**
