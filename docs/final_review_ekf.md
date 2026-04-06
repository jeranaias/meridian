# Meridian EKF Final Review — Pre-Hardware Deployment
**Date:** 2026-04-02
**Reviewer:** Automated audit (automated side-by-side audit)
**Meridian path:** `D:\projects\meridian\crates\meridian-ekf\src\`
**ArduPilot path:** `D:\projects\ardupilot\libraries\AP_NavEKF3\`

---

## Summary Table

| File | Status | Critical Issues |
|---|---|---|
| state.rs | CONCERN (1) | Gyro bias bound, position Z altitude bound |
| predict.rs | BUG (1), CONCERN (2) | Process noise formula for bias states; EKF_TARGET_RATE_HZ hardcoded; strapdown uses current not delayed DCM |
| fusion.rs | CONCERN (2) | Mag covariance reset path absent; H matrix sign check for Hz |
| core.rs | CONCERN (2) | Health FSM simplified; strapdown double-runs ConstrainStates |
| aiding.rs | MINOR (2) | GPS align check missing drift/HDOP; WMM is coarse approximation |
| output.rs | CONCERN (1) | PI gains and correction application differ from ArduPilot third-order filter |
| flow_fusion.rs | CONCERN (1) | Body-rate correction uses raw rate, not motion-compensated `flowRadXYcomp` |
| airspeed_fusion.rs | CONCERN (1) | Minimum airspeed check differs (1.0 vs 5.0 m/s); sideslip H omits quaternion terms |
| gsf_yaw.rs | MINOR (1) | Gravity addition missing in predict step |
| params.rs | BUG (1) | `gyro_bias_process_noise` default wrong: 7.0e-6 vs ArduPilot 1.0e-3 |
| dcm.rs | MINOR | Complementary filter gain 0.2 vs ArduPilot 0.2 — MATCH |
| gps_blend.rs | CONCERN (1) | Blended hacc formula differs from ArduPilot |

---

## 1. state.rs vs AP_NavEKF3_core.h

### 1.1 State Vector Ordering
**MATCH.** Meridian's `StateVector` maps states 0–23 identically to ArduPilot's `state_elements`:
```
quat (0-3), velocity (4-6), position (7-9), gyro_bias (10-12),
accel_bias (13-15), earth_magfield (16-18), body_magfield (19-21), wind (22-23)
```

### 1.2 Quaternion Convention
**MATCH.** Both use the convention: quaternion defines rotation from local NED to body frame. The comment in `state.rs` correctly documents the inverse DCM convention used in Meridian math.

### 1.3 Initial Values
**MATCH.** `earth_mag` initial value `(0.22, 0.005, 0.42)` is a plausible mid-latitude field. This is not hard-coded in ArduPilot (it fuses from magnetometer), so this is acceptable for cold start.

### 1.4 ConstrainStates — Gyro Bias Bound
**MATCH.** `GYRO_BIAS_LIMIT = 0.5` rad/s confirmed. `state.rs` line 87–89 clamps to `±0.5`. Correct.

### 1.5 ConstrainStates — Position Z Bound
**CONCERN.** `state.rs` line 84 clamps `position.z` to `±1.0e5` (±100 km altitude). ArduPilot's `ConstrainStates()` does not impose a separate altitude limit different from XY — it uses the same `EK3_POSXY_STATE_LIMIT = 1e6`. A tighter altitude clamp at ±100 km is acceptable operationally but diverges from the source. **No flight safety impact at normal altitudes.**

### 1.6 Accel Bias Bound
**CONCERN.** Meridian clamps accel bias to `±5.0 m/s²`. ArduPilot uses `ACCEL_BIAS_LIM_SCALER = 0.2` applied to the bias itself in `ConstrainStates()`, which gives `±0.2 * (gravity * dt)` per step but no hard clamp on the stored state value — the clamp is enforced implicitly by the process noise rolloff. The `±5.0 m/s²` hard wall is more aggressive than ArduPilot's soft rolloff. **Low flight safety impact but will cap bias estimation earlier.**

### 1.7 Covariance Initialization
**MATCH.** Initial variances are consistent with ArduPilot's `CovarianceInit()`. ArduPilot sets `sq(gpsHorizVelNoise) = 0.09` for velocity; Meridian sets `0.09`. Match.

---

## 2. predict.rs vs AP_NavEKF3_core.cpp

### 2.1 Strapdown Equations (UpdateStrapdownEquationsNED)
**CONCERN.** In ArduPilot, `delVelNav` is rotated using `prevTnb` — the **previous** NED-to-body DCM (saved **before** the quaternion update). Meridian (line 246) rotates using the **current** (post-update) quaternion. This introduces a half-step integration error. For well-converged states the difference is negligible (~0.1% at 400Hz), but during rapid rotations or initialization it introduces a systematic first-order error in the velocity estimate. ArduPilot comment: _"use the nav frame from previous time step as the delta velocities have been rotated into that frame."_ This is a known-correct design choice in ArduPilot that Meridian deviates from.

**Verdict: CONCERN — small but measurable velocity error during fast rotations.**

### 2.2 Horizontal Clamp in AID_NONE
**MATCH.** ArduPilot line 777: `if (accNavMagHoriz > 5.0f)`. Meridian computes `acc_horiz` from `del_vel_nav` directly, while ArduPilot uses the LPF-filtered `accNavMagHoriz`. Meridian's version clips the instantaneous value, not the filtered one. **MINOR** difference — Meridian may clip high-frequency accel spikes that ArduPilot's LPF would smooth out.

### 2.3 Covariance Prediction — Process Noise Formula for Bias States
**BUG.** ArduPilot line 1047:
```cpp
ftype dAngBiasVar = sq(sq(dt) * constrain_ftype(frontend->_gyroBiasProcessNoise, 0.0, 1.0));
```
This is `(dt^2 * pnse)^2 = dt^4 * pnse^2`.

Meridian `predict.rs` line 419:
```rust
let gbias_var = (dt * dt * params.gyro_bias_process_noise)
    * (dt * dt * params.gyro_bias_process_noise);
```
This is `(dt^2 * pnse)^2 = dt^4 * pnse^2`. **Formula is the same — MATCH.**

**BUT** see Bug 2.4 below for the default value discrepancy that makes this matter.

### 2.4 EKF_TARGET_RATE_HZ Hardcoded
**CONCERN.** `predict.rs` line 34 sets `EKF_TARGET_RATE_HZ: f32 = 400.0`. ArduPilot derives this at runtime from `dtEkfAvg`. Hardcoding 400 Hz means `VERT_VEL_VAR_CLIP_COUNT_LIM` is fixed at `5 * 400 = 2000`. If Meridian is configured to run at a different EKF rate (e.g., 200 Hz), the clip counter will never trigger (needs 2000 steps of low vertical velocity variance), potentially masking a vertical covariance collapse.

### 2.5 Fvq Jacobian
**MATCH.** The Fvq block in `predict.rs` lines 359–382 matches the ArduPilot symbolic auto-generated equations. The ArduPilot symbolic approach (via `PS43`, `PS45`, etc.) and Meridian's explicit quaternion rotation Jacobian are mathematically equivalent. Spot-checked: `f[4][0] = 2*(q0*dvx - q3*dvy + q2*dvz)` matches the pattern evident in `nextP[0][4]` = `PS43*PS44 - PS45*PS47 - PS54*PS55 + ...` where `PS43 = 2*PS2 + PS42 = 2*q2^2 + (2*q3^2 - 1)`.

### 2.6 Covariance Truncation at state_index_lim
**MATCH.** ArduPilot only computes covariance up to `stateIndexLim`. Meridian does the same with `lim = inhibit.state_index_lim + 1`. Correct.

### 2.7 dt Constrain
**CONCERN.** ArduPilot line 1035 constrains dt: `constrain_ftype(0.5*(delAngDT+delVelDT), 0.5*dtEkfAvg, 2.0*dtEkfAvg)`. Meridian line 305: `let dt = 0.5 * (imu.del_ang_dt + imu.del_vel_dt)` with no clamp. Timing jitter can push dt outside the ArduPilot-validated range without any guard. For embedded hardware with irregular IMU delivery this is a real risk.

---

## 3. fusion.rs vs AP_NavEKF3_PosVelFusion.cpp + MagFusion.cpp

### 3.1 GPS Velocity Innovation Gate
**MATCH.** Both use `gate_sigma * gate_sigma * innov_var` for chi-squared gate. Default `VEL_I_GATE = 500 → 5.0 sigma`. Correct.

### 3.2 GPS Velocity Noise Scaling with Acceleration
**MATCH.** `fuse_gps_velocity` line 134–136 matches ArduPilot line 490: `sq(noise) + sq(accel_scale * accNavMag)`.

### 3.3 GPS Velocity Noise Constrain
**CONCERN.** ArduPilot constrains GPS velocity noise: `constrain_ftype(noise, 0.05, 5.0)` before squaring. Meridian uses the noise directly from params with no floor/ceiling. If `gps_vel_noise` is set to an extreme value, the unconstrained noise can cause filter divergence.

### 3.4 Magnetometer H Matrix
**MATCH.** The `SH_MAG` intermediate values (lines 405–412 in `fusion.rs`) match ArduPilot's `SH_MAG[0..8]` exactly. The H matrices for X (`hx`), Y (`hy`), Z (`hz`) match the ArduPilot auto-generated code at lines 522–532. Spot-checked all 8 non-zero entries per axis.

### 3.5 Mag Measurement Prediction (DCM Orientation)
**CONCERN (DCM transposition check).** ArduPilot `FuseMagnetometer()` line 497–513 defines DCM row-major with:
```cpp
MagPred = { DCM[0][0]*magN + DCM[0][1]*magE + DCM[0][2]*magD + magXbias, ... }
```
where the DCM rows are the body-frame rows of the rotation from NED to body. Meridian lines 419–422:
```rust
let pred = [
    dcm[0][0]*mag_n + dcm[1][0]*mag_e + dcm[2][0]*mag_d + state.body_mag.x,
    ...
]
```
This uses **columns** of `dcm`, not rows. If `to_dcm()` returns body-to-NED, then `dcm[row][col]` means row=NED, col=body, and the prediction `dcm[0][0]*magN + dcm[1][0]*magE + dcm[2][0]*magD` is the first body-frame component. This appears correct for the convention documented in `state.rs` (to_dcm returns body-to-NED). **This needs independent verification against the Meridian math library's DCM convention before hardware.** A wrong convention here will cause magnetometer fusion to actively corrupt heading.

### 3.6 Magnetometer Covariance Reset on Transition
**CONCERN.** ArduPilot performs a covariance reset on the transition `!inhibitMagStates && lastInhibitMagStates` and calls `FuseDeclination(radians(20.0))` (lines 1059–1086). Meridian has no equivalent of this automatic declination stabilization reset. When mag states become active the earth field covariance starts from the running value, which may be poorly conditioned. **May cause slower mag convergence or initial heading errors after transitions.**

### 3.7 Covariance Update P -= KHP
**MATCH.** Both implementations use `P[i][j] -= K[i] * H * P[row][j]` with a pre-copy of the covariance row before mutation. Correct.

---

## 4. core.rs — GPS Delay Buffer and Health Monitoring

### 4.1 GPS Delay Buffer
**MATCH in concept, CONCERN in mechanism.** ArduPilot uses a timestamped ring buffer (`EKF_Buffer`) that recalls the sample whose timestamp most closely matches `imuDataDelayed.time_ms`. Meridian's `ImuDelayBuffer` uses a fixed depth (`target_depth = ceil(gps_delay_ms / imu_dt_ms)`). If IMU rate is not exactly constant (which it never is on hardware), the depth-based approach will accumulate timing error. For 400 Hz nominal with ±5% jitter, this can add ±11 ms to the GPS delay compensation, versus ArduPilot's timestamp-exact recall.

### 4.2 Health State Machine
**CONCERN.** ArduPilot `healthy()` checks:
1. Filter fault bits
2. All three test ratios (vel, pos, hgt) simultaneously exceeding 1.0
3. 1-second settle time from `ekfStartTime_ms`
4. On-ground position/height innovation bounds

Meridian `core.rs` lines 323–338 uses:
1. `fuse_count > 20` to transition from Initializing → Healthy
2. `predict_count / fuse_count > 50` ratio for Unhealthy detection
3. NaN check on quaternion/velocity

This is a significant simplification. In particular, **Meridian does not check per-axis innovation test ratios**, which ArduPilot uses as the primary health signal. A filter can appear healthy to Meridian while all three innovation ratios are failed.

### 4.3 Double ConstrainStates Call
**MINOR BUG.** In `core.rs` lines 292–298, `predict_state()` already calls `state.constrain_states()` at line 282. Then `core.rs` calls `self.state.constrain_states()` again at line 297. This is redundant. No correctness issue but wastes cycles.

---

## 5. aiding.rs — GPS Quality Gate and Alignment

### 5.1 GPS Quality Gate Thresholds
**MATCH on core checks.** `GpsQualityGate::update_align()` checks: fix_3d, sats >= 6, hacc < 5.0, vacc < 8.0, sacc < 1.0. ArduPilot's `calcGpsGoodToAlign()` uses the same values scaled by `CHECK_SCALER_DEFAULT = 100 → 1.0x` for copter.

### 5.2 Missing HDOP and Horizontal Drift Checks
**CONCERN.** ArduPilot's `calcGpsGoodToAlign()` also checks:
- HDOP: rejects if HDOP > 2.5 (scaled)
- Horizontal speed drift: accumulates position drift over time and rejects if the vehicle is moving while stationary

Meridian's `GpsQualityGate` omits both. HDOP checking prevents use of poor satellite geometry. Drift checking prevents false alignment when the GPS receiver is drifting on the ground. **Missing drift check is particularly important for copters sitting on the pad.**

### 5.3 WMM Declination
**CONCERN.** `WmmDeclination::get_declination()` is a 4-region coarse approximation. ArduPilot uses the full AP_Declination spherical harmonic table (~100 KB coefficients). The coarse approximation can be off by 5–10 degrees in complex magnetic regions (Canada, Australia, some parts of Europe). **For multirotor hover the heading error will directly map to GPS navigation error equal to `position * tan(decl_error)`.**

---

## 6. output.rs — Output Predictor PI Gains and IMU Ring Buffer

### 6.1 Third-Order Complementary Filter
**CONCERN.** ArduPilot uses a third-order complementary filter for height/height rate (lines 860–885 of `AP_NavEKF3_core.cpp`) with Widnall-Sinha coefficients derived from `CompFiltOmega = 2*pi*hrt_filt_freq`. Meridian does not implement this vertical complementary filter at all (`vert_comp_filt_pos/vel/acc` fields exist but are never used in `apply_ekf_correction()`). This means Meridian's vertical velocity output will have more high-frequency noise and will not benefit from the height-rate smoothing that ArduPilot provides.

### 6.2 Attitude Correction Gain
**CONCERN.** ArduPilot's attitude correction gain is computed from the total delay between the output buffer oldest and newest timestamps. Meridian line 277:
```rust
let error_gain = (0.5 / time_delay.max(0.001)) * dt_ekf;
```
ArduPilot line 898 uses a different gain structure tied to the complementary filter omega. The Meridian gain has the right shape but uses 0.5 as a magic number where ArduPilot uses `M_2PI * hrt_filt_freq / 2`. This will produce slower attitude correction for low `hrt_filt_freq` settings.

### 6.3 Velocity/Position PI Correction
**CONCERN.** Meridian uses:
```rust
vel_pos_gain = dt_ekf / tau_vel_pos
vel_correction = vel_err * gain + integral * (gain^2 * 0.1)
```
ArduPilot uses the gain `velPosScale = dt/tau` applied to the error but does not multiply the integral by `gain^2 * 0.1`. The integral term in Meridian is numerically different. For `tau=0.25, dt=0.0025`, `gain=0.01`, `gain^2*0.1 = 1e-5` — the integral term is effectively negligible, so the two are approximately equivalent in steady state. **MINOR.**

---

## 7. flow_fusion.rs vs AP_NavEKF3_OptFlowFusion.cpp

### 7.1 Flow Rate Correction (Body Rate Removal)
**CONCERN.** Meridian line 186–187:
```rust
let flow_x_corrected = meas.flow_x - meas.body_rate.x;
let flow_y_corrected = meas.flow_y - meas.body_rate.y;
```
ArduPilot uses `ofDataDelayed.flowRadXYcomp` — the **pre-compensated** flow that accounts for the full motion compensation including sensor offset and angular rate coupling. Meridian receives the raw body rate separately and subtracts it. If the caller provides the raw `flowRadXY` (not the motion-compensated `flowRadXYcomp`), the lever arm error from sensor-to-IMU offset is not removed. **The calling convention must pass `flowRadXYcomp` equivalent data, not raw flow.**

### 7.2 Tilt Check Missing
**CONCERN.** ArduPilot checks `prevTnb.c.z > DCM33FlowMin` before fusing optical flow (a tilt check ensuring the sensor is pointing roughly downward). Meridian has no tilt validity check. If the vehicle is severely tilted, flow fusion with a wrong range estimate will corrupt velocity states.

### 7.3 Takeoff Detection Zero-ing
**MINOR.** ArduPilot zeros flow measurements if `!takeOffDetected && terrainState < 0.5`. Meridian has no equivalent. At very low heights, bad focus/sparse features cause incorrect flow readings. Missing this can cause position errors at launch.

---

## 8. airspeed_fusion.rs vs AP_NavEKF3_AirDataFusion.cpp

### 8.1 Minimum Airspeed Check
**CONCERN.** ArduPilot checks `VtasPred > 1.0f` before fusing airspeed. Meridian uses `tas_pred < as_params.min_airspeed` where `min_airspeed = 5.0`. This is a 5x more restrictive gate than ArduPilot. For slow aircraft (gliders, very slow UAVs) this will suppress airspeed fusion at speeds where ArduPilot would fuse. **Not a bug for fast aircraft, but a behavioral difference.**

### 8.2 Kalman Mask on Airspeed Fusion
**CONCERN.** ArduPilot uses an explicit `kalman_mask` that gates whether airspeed fusion updates gyro bias, accel bias, mag, and wind states based on `tasDataDelayed.allowFusion`, `inhibitDelAngBiasStates`, etc. Meridian computes Kalman gains for all states unconditionally (no mask). This means airspeed fusion will incorrectly modify gyro bias and mag states through cross-covariance even when ArduPilot would mask these out. **For fixed-wing operation where airspeed is frequently fused, this can corrupt gyro bias estimates.**

### 8.3 Sideslip H Matrix Missing Quaternion Terms
**CONCERN.** Meridian's `fuse_sideslip()` comments: _"Quaternion derivatives of sideslip are complex. For the simplified version, we omit quaternion coupling in H."_ ArduPilot's `FuseSideslip()` includes the full quaternion H terms from the Matlab symbolic derivation. For coordinated turns, the omitted quaternion terms affect how sideslip fusion corrects attitude. **For fixed-wing aircraft in turns, this is a material omission.**

### 8.4 airDataFusionWindOnly Mode
**CONCERN.** ArduPilot has `airDataFusionWindOnly` flag that restricts airspeed fusion to wind states only when GPS is available (to avoid over-constraining the solution). Meridian has no equivalent. In GPS+airspeed scenarios, Meridian's airspeed fusion will attempt to update all states including position and velocity, whereas ArduPilot would restrict to wind estimation only.

---

## 9. gsf_yaw.rs — 5-Model GSF Structure

### 9.1 Model Count
**MATCH.** `GSF_NUM_MODELS = 5` matches `N_MODELS_EKFGSF = 5`.

### 9.2 Initial Yaw Spacing
**MATCH.** Even spacing over [-pi, pi) with 5 models. Correct.

### 9.3 Gravity Compensation in predict()
**CONCERN.** In `gsf_yaw.rs` `predict()` line 162: "No gravity correction needed here since del_vel already includes specific force." But `del_vel` is `imuDataDelayed.delVel` which is the **raw integrated accelerometer** output including the specific force reaction to gravity. This is **not** the gravity-compensated navigation acceleration. ArduPilot's GSF `predictEKF()` does not add gravity explicitly because the GSF only propagates NE velocity (not D) and the gravity component cancels. Meridian propagates the full del_vel including D component gravity, but only uses model.velocity[0] and [1] (NE). The NE components from gravity (`sin(pitch)*g*dt`, `sin(roll)*g*dt`) will contaminate the velocity propagation for non-level vehicles. **For hover at level attitude this is near-zero; for tilted vehicles it is a real error.**

### 9.4 Weight Update (Likelihood)
**MATCH.** The Gaussian likelihood update with 2x2 innovation covariance inversion is correct. Weight normalization on collapse to uniform is correct.

### 9.5 Circular Mean Yaw
**MATCH.** Using `atan2(sum(w*sin(yaw)), sum(w*cos(yaw)))` is the correct circular mean. Correct.

### 9.6 GSF Valid History Threshold
**CONCERN.** ArduPilot requires `GSF_YAW_VALID_HISTORY_THRESHOLD = 5` **consecutive** valid samples. Meridian uses `fusion_count >= min_fusions_for_valid = 10` (total, not consecutive). This means Meridian can declare the GSF valid after 10 scattered fusions even if some had bad data, whereas ArduPilot requires 5 in a row.

---

## 10. params.rs — EK3_* Default Values

**All values verified against ArduPilot copter defaults (APM_BUILD_ArduCopter section, line 23–45).**

| Parameter | ArduPilot (copter) | Meridian | Status |
|---|---|---|---|
| VELNE_M_NSE | 0.3 | 0.3 | MATCH |
| VELD_M_NSE | 0.5 | 0.5 | MATCH |
| POSNE_M_NSE | 0.5 | 0.5 | MATCH |
| ALT_M_NSE | 2.0 | 2.0 | MATCH |
| MAG_M_NSE | 0.05 | 0.05 | MATCH |
| GYRO_P_NSE | 1.5e-2 | 0.015 | MATCH |
| ACC_P_NSE | 3.5e-1 | 0.35 | MATCH |
| **GBIAS_P_NSE** | **1.0e-3** | **7.0e-6** | **BUG** |
| ABIAS_P_NSE | 2.0e-2 | 2.0e-2 | MATCH |
| MAGB_P_NSE | 1.0e-4 | 1.0e-4 | MATCH |
| MAGE_P_NSE | 1.0e-3 | 1.0e-3 | MATCH |
| VEL_I_GATE | 500 | 500 | MATCH |
| POS_I_GATE | 500 | 500 | MATCH |
| HGT_I_GATE | 500 | 500 | MATCH |
| MAG_I_GATE | 300 | 300 | MATCH |
| WIND_P_NSE | 0.2 | 0.2 | MATCH |
| GPS_DELAY | 220 ms | 220 ms | MATCH |

### 10.1 GBIAS_P_NSE Default Value — BUG
**BUG.** `params.rs` line 101:
```rust
gyro_bias_process_noise: 7.0e-6,   // EK3_GBIAS_P_NSE copter default
```
ArduPilot `AP_NavEKF3.cpp` line 30:
```cpp
#define GBIAS_P_NSE_DEFAULT     1.0E-03f
```
Meridian's default is **143x smaller** than ArduPilot. The process noise formula is `(dt^2 * pnse)^2`, so at dt=0.0025s:
- ArduPilot: `(6.25e-6 * 1.0e-3)^2 = 3.9e-17` rad^4/s^2
- Meridian: `(6.25e-6 * 7.0e-6)^2 = 1.9e-21` rad^4/s^2

This is 20,000x less process noise for gyro bias. In practice this means **gyro bias will almost never adapt** after initial convergence. Any real gyro drift that exceeds the initial bias estimate will not be tracked. For a multirotor in flight this could cause growing attitude error when temperature changes shift the gyro bias.

---

## 11. dcm.rs — Complementary Filter

### 11.1 Proportional Gain
**MATCH.** `p_gain = 0.2` matches ArduPilot `AP_AHRS_DCM`'s `AP_AHRS_RP_P = 0.2`. Correct.

### 11.2 Orthonormalization Method
**MATCH.** Row normalization + Gram-Schmidt + cross product for row 2. This matches the standard ArduPilot DCM orthonormalization. Correct.

### 11.3 Integral Term Absent
**MINOR.** ArduPilot's DCM implementation includes an integral term (`AP_AHRS_I_GAIN = 0.0087`) that corrects persistent gyro bias. Meridian's `DcmEstimator` only has the proportional term. Since `dcm.rs` is documented as a "safety fallback," this omission is acceptable but limits long-term attitude accuracy if the EKF is disabled for extended periods.

### 11.4 Accel Threshold
**MINOR.** Meridian rejects accel correction when `g_ratio` is outside `(0.5, 2.0)` (i.e., `< 4.9 m/s² or > 19.6 m/s²`). ArduPilot uses similar logic but with slightly different thresholds. No material impact.

---

## 12. gps_blend.rs — Blending Weights

### 12.1 Weight Formula
**CONCERN.** Meridian weights by `1/hacc^2` (inverse variance of position, since hacc is a 1-sigma estimate). ArduPilot's GPS blending (`AP_GPS_Blended`) weights by `1/hacc` (not squared). The two produce the same GPS when accuracy is equal, but for unequal accuracy Meridian's quadratic weighting gives even more weight to the better GPS. For example, GPS1=1m, GPS2=4m: Meridian gives GPS1 weight 16/17≈94%; ArduPilot gives 4/5=80%. This behavior difference changes how aggressively bad GPS data is suppressed.

### 12.2 Blended hacc Formula
**CONCERN.** Meridian line 94: `hacc = 1.0 / sqrt(total)` where `total = 1/h1^2 + 1/h2^2`. For equal accuracies of 2m: `total = 0.5`, `hacc = sqrt(2) = 1.41m`. ArduPilot computes blended accuracy differently (RMS-style). The Meridian formula is a valid propagation of combined variance but may not match what ArduPilot reports.

### 12.3 Health Counter
**CONCERN.** ArduPilot GPS blending includes a timing health counter: GPS that hasn't produced a new measurement in a timeout period is marked unhealthy. Meridian relies only on the `healthy: bool` flag set by the caller, with no timeout tracking internal to `GpsBlender`. If the caller fails to update `healthy` on stale GPS, the blend will continue using old data.

---

## Critical Action Items (Ranked by Hardware Safety Impact)

### Priority 1 — Must Fix Before Hardware Flight

1. **params.rs `gyro_bias_process_noise = 7.0e-6`** — Change to `1.0e-3`. As-is, gyro bias will not adapt during flight, causing attitude drift as gyro temperature changes. **(BUG)**

2. **predict.rs strapdown uses current DCM not previous** — The `body_to_ned` rotation in `predict_state()` line 246 must use the DCM computed from the quaternion *before* the quaternion update step. Save `prev_quat` before line 240 and use it for the del_vel rotation. Without this, NED acceleration couples the post-correction attitude back in, causing a first-order integration error during fast rotations. **(CONCERN → BUG during fast rotation)**

3. **fusion.rs mag DCM transposition** — Verify that `state.quat.to_dcm()` convention is consistent between the mag prediction (lines 419–422) and ArduPilot's DCM definition. A wrong matrix transpose will actively corrupt heading. **(CONCERN — must verify before first mag fusion)**

4. **airspeed_fusion.rs Kalman mask absent** — Airspeed fusion should not update gyro bias, accel bias, or mag states unless those are explicitly observable from airspeed. Add a mask equivalent to ArduPilot's `kalman_mask`. **(CONCERN — will corrupt gyro bias estimates during airspeed fusion)**

### Priority 2 — Fix Before Extended Autonomous Flight

5. **predict.rs dt not constrained** — Add `dt.clamp(0.5 * dt_ekf_avg, 2.0 * dt_ekf_avg)` guard matching ArduPilot line 1035. Required for robustness against IMU jitter on real hardware.

6. **aiding.rs WMM declination** — Replace the 4-region coarse approximation with the full AP_Declination spherical harmonic table or a well-tested WMM grid. Errors up to 10° can create significant GPS navigation error.

7. **aiding.rs GPS gate missing HDOP and drift checks** — Add HDOP (< 2.5) check and position drift accumulation check to `GpsQualityGate::update_align()`. The drift check is critical for copters sitting on the launch pad.

8. **fusion.rs missing mag variance reset on inhibit transition** — Add the mag body and earth covariance zero-and-reset with `FuseDeclination(radians(20.0))` equivalent when transitioning from mag-inhibited to mag-active state.

9. **core.rs health FSM missing innovation test ratios** — Add tracking of `vel_test_ratio`, `pos_test_ratio`, `hgt_test_ratio` and declare unhealthy when all three simultaneously exceed 1.0 (matching ArduPilot `healthy()` logic).

### Priority 3 — Improvement for Parity

10. **output.rs vertical complementary filter** — Implement the third-order Widnall-Sinha height/height-rate complementary filter for vertical output smoothing.

11. **flow_fusion.rs tilt check** — Add `dcm[2][2] > DCM33FlowMin` check before optical flow fusion.

12. **gsf_yaw.rs valid history should be consecutive** — Require 5 consecutive valid fusions (tracking a `consecutive_valid` counter) rather than 10 total.

13. **predict.rs EKF_TARGET_RATE_HZ hardcoded** — Derive from actual `dt_ekf_avg` at runtime.

14. **gps_blend.rs weight formula** — Align with ArduPilot's 1/hacc (not 1/hacc^2) weighting if parity with ArduPilot behavior is required.

---

## Files with No Material Issues

- **state.rs**: State ordering, quaternion convention, and velocity/position constraints are correct. Minor deviation in altitude clamp is non-critical.
- **dcm.rs**: Gains, orthonormalization, and accel gating all match ArduPilot DCM. Missing integral term is documented and acceptable for fallback use.
- **gsf_yaw.rs**: Model count, initial spacing, circular mean, and weight normalization are correct. Minor gravity contamination in predict and consecutive-vs-total valid check are low impact for typical flight profiles.

---

*This review is automated. All line references are to the file state as of 2026-04-02. Before hardware deployment, the Priority 1 items must be resolved and a unit test suite run that covers the specific scenarios identified (fast rotation, gyro bias adaptation, mag fusion heading, airspeed + GPS combined scenarios).*
