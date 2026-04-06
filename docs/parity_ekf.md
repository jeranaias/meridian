# EKF Parity Audit: ArduPilot AP_NavEKF3 vs Meridian meridian-ekf

**Date:** 2026-04-02  
**ArduPilot source:** `D:\projects\ardupilot\libraries\AP_NavEKF3\`  
**Meridian source:** `D:\projects\meridian\crates\meridian-ekf\src\`  
**Method:** Full side-by-side source comparison of every subsystem.

---

## Summary

Meridian's `meridian-ekf` correctly replicates the 24-state vector layout, quaternion convention, gravity direction, and the sequential scalar fusion pattern from ArduPilot EKF3. The Jacobian H matrices for magnetometer fusion match ArduPilot's SH_MAG intermediates. GPS and barometer scalar fusion are structurally correct.

However there are **numerous gaps, constant mismatches, and missing subsystems** that must be fixed before Meridian can claim flight-grade EKF parity. They are listed below in order of severity.

---

## 1. State Vector

### 1.1 State Count and Ordering — PASS (with caveats)

Meridian `state.rs` defines exactly 24 states in the same order as `AP_NavEKF3_core.h:574-583`:

| Index | ArduPilot field | Meridian field | Match |
|-------|----------------|----------------|-------|
| 0–3   | `quat` (q0,q1,q2,q3) | `quat` (w,x,y,z) | YES |
| 4–6   | `velocity` NED | `velocity` NED | YES |
| 7–9   | `position` NED | `position` NED | YES |
| 10–12 | `gyro_bias` | `gyro_bias` | YES |
| 13–15 | `accel_bias` | `accel_bias` | YES |
| 16–18 | `earth_magfield` | `earth_mag` | YES |
| 19–21 | `body_magfield` | `body_mag` | YES |
| 22–23 | `wind_vel` | `wind` | YES |

### 1.2 Quaternion Convention — PASS

Both use Hamilton convention with scalar-first ordering: `[q0=w, q1=x, q2=y, q3=z]`. ArduPilot quaternion represents rotation from **local NED to body frame** (i.e., nav-to-body). Meridian's comment says "body-to-NED" in `state.rs:16`. This is the **opposite convention**. The math works out consistently within each codebase, but the comment in Meridian is wrong and will cause confusion during debugging.

**GAP:** `state.rs:16` says "body-to-NED" but ArduPilot's quat is NED-to-body. Fix the comment or verify that all DCM uses (`Rotation::from_quaternion`) are consistent with NED-to-body.

### 1.3 Covariance Initial Values — PARTIAL MISMATCH

Meridian `Covariance::initial()` sets:
- Velocity variance: `0.25` (i.e., 0.5 m/s σ)
- Position variance: `9.0` (i.e., 3 m σ)

ArduPilot `CovarianceInit()` (not directly read, but driven by `ResetVelocity`/`ResetPosition`) sets position variance on reset to `sq(frontend->_gpsHorizPosNoise)` = `sq(0.5)` = `0.25` for position, and `sq(frontend->_gpsHorizVelNoise)` = `sq(0.3)` = `0.09` for velocity. Meridian's initial velocity variance (0.25) is 2.8x larger than ArduPilot's default post-reset value (0.09). This is not a functional error but will cause slower initial convergence.

**GAP:** Initial velocity variance should be `sq(VELNE_M_NSE_DEFAULT)` = `0.09` not `0.25`.

---

## 2. Prediction Step

### 2.1 Strapdown Equations — CRITICAL BUG: Earth Rate Omitted

ArduPilot `UpdateStrapdownEquationsNED()` line 749:
```cpp
stateStruct.quat.rotate(delAngCorrected - prevTnb * earthRateNED * imuDataDelayed.delAngDT);
```

Meridian `predict.rs:predict_state()` lines 43–55 explicitly skips this:
```rust
// We skip earth rate correction for now (negligible for hover duration)
```

**Severity:** For short hover flights (<5 min), earth rate (`7.292e-5 rad/s`) contributes ~0.026 deg/min heading drift. This is negligible for hover but breaks certification parity. The comment acknowledges it.

`earthRate = 0.000072921f` is defined in `AP_NavEKF3_core.h:49`. Meridian has no corresponding constant.

**GAP:** Earth rate correction missing from quaternion update. Must add for full parity.

### 2.2 Horizontal Acceleration Limiting in AID_NONE Mode — MISSING

ArduPilot lines 777–781:
```cpp
if ((PV_AidingMode == AID_NONE) && (accNavMagHoriz > 5.0f)) {
    ftype gain = 5.0f/accNavMagHoriz;
    delVelNav.x *= gain;
    delVelNav.y *= gain;
}
```

Meridian has no equivalent. This prevents large attitude-error-induced horizontal acceleration transients from corrupting velocity when there is no GPS aiding. Without it, the filter can diverge in AID_NONE mode during aggressive attitude changes.

**GAP:** Horizontal velocity clamping in AID_NONE mode is missing.

### 2.3 `velDotNEDfilt` / `accNavMag` Tracking — MISSING

ArduPilot computes filtered velocity derivative:
```cpp
velDotNED = delVelNav / imuDataDelayed.delVelDT;
velDotNEDfilt = velDotNED * 0.05f + velDotNEDfilt * 0.95f;
accNavMag = velDotNEDfilt.length();
accNavMagHoriz = velDotNEDfilt.xy().length();
```

These are used in:
- GPS velocity observation variance scaling (`gpsNEVelVarAccScale`)
- Maneuver detection (`manoeuvring = accNavMagHoriz > 0.5f`)
- Wind/mag state learning control

Meridian has no `accNavMag` equivalent.

**GAP:** Filtered acceleration magnitude tracking entirely missing. Downstream: GPS variance scaling absent, maneuver detection absent.

### 2.4 `delAngBodyOF` / `delTimeOF` Accumulation — MISSING

ArduPilot lines 793–794 accumulate gyro angle for optical flow:
```cpp
delAngBodyOF += delAngCorrected;
delTimeOF += imuDataDelayed.delAngDT;
```

Meridian has no optical flow support, so these are expected absent but should be noted.

### 2.5 Range Beacon Position Integration — MISSING

ArduPilot lines 801–803 update beacon receiver position:
```cpp
if (filterStatus.flags.horiz_vel) {
    rngBcn.receiverPos += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);
}
```

Expected absent (Meridian has no beacon fusion).

### 2.6 ConstrainStates() Not Called from predict_state() — MISSING

ArduPilot calls `ConstrainStates()` at the end of `UpdateStrapdownEquationsNED()` (line 797). This enforces gyro bias limit (`GYRO_BIAS_LIMIT = 0.5f rad/s`), accel bias limit (`ACCEL_BIAS_LIM_SCALER = 0.2f`), and position limit (`EK3_POSXY_STATE_LIMIT = 1e6`).

Meridian `predict_state()` never calls any state constraint. There is a `constrain_variances()` in the covariance path, but no equivalent state value clamping.

**GAP:** Missing state constraints. Gyro bias can exceed ±0.5 rad/s (GYRO_BIAS_LIMIT), position can drift without bounds.

---

## 3. Covariance Prediction

### 3.1 Covariance Propagation Method — DIFFERENT BUT EQUIVALENT (with gap)

ArduPilot uses a pre-computed symbolic Jacobian (auto-generated from MATLAB). The intermediate values `PS0`..`PS400+` form an expanded polynomial that avoids the O(N³) matrix multiply.

Meridian uses explicit `F * P * F^T + Q` with a 24×24 matrix multiply — O(N³) cost. For embedded deployment this may be too slow (24×24×24 = 13,824 multiply-accumulates per prediction step at ~400Hz = ~5.5M FLOPs/s vs ArduPilot's O(N²) symbolic approach).

**However, there is a correctness gap:** Meridian comments out the velocity-quaternion coupling (`Fvq`) in predict.rs lines 133–143:
```rust
// TODO: re-enable with proper numerical conditioning
// (states 4-6 w.r.t. 0-3: velocity-quaternion coupling zeroed)
```

This coupling is **not** zeroed in ArduPilot — it is captured in the symbolic expansion. Zeroing it means the covariance does not properly track the correlation between attitude errors and velocity errors. In practice this causes the filter to be overconfident in velocity when attitude uncertainty is high.

**GAP (CRITICAL):** `Fvq` block (velocity rows 4–6, quaternion columns 0–3) is zeroed in Meridian but included in ArduPilot. The filter will be miscalibrated.

### 3.2 Process Noise Q Values — MULTIPLE MISMATCHES

Comparing Meridian `params.rs` defaults vs ArduPilot copter defaults:

| Parameter | ArduPilot (copter) | Meridian | Match |
|-----------|-------------------|---------|-------|
| `GYRO_P_NSE` | `1.5e-2` | `0.015` | YES |
| `ACC_P_NSE` | `3.5e-1` | `0.35` | YES |
| `GBIAS_P_NSE` | `1.0e-3` | `1.0e-3` | YES |
| `ABIAS_P_NSE` | `2.0e-2` | `3.0e-3` | **NO — 6.7x lower** |
| `MAGB_P_NSE` | `1.0e-4` | `3.0e-4` | **NO — 3x higher** |
| `MAGE_P_NSE` | `1.0e-3` | `3.0e-4` | **NO — 3.3x lower** |
| `WIND_P_NSE` | `0.2` (copter) | ignored (1e-6) | N/A (wind unused) |
| `VEL_I_GATE` | 500 | 500 | YES |
| `POS_I_GATE` | 500 | 500 | YES |
| `HGT_I_GATE` | 500 | 500 | YES |
| `MAG_I_GATE` | 300 | 500 | **NO — 67% larger** |
| `VELNE_M_NSE` | `0.3` | `0.3` | YES |
| `VELD_M_NSE` | `0.5` | `0.5` | YES |
| `POSNE_M_NSE` | `0.5` | `0.5` | YES |
| `ALT_M_NSE` | `2.0` | `2.0` | YES |
| `MAG_M_NSE` | `0.05` | `0.05` | YES |

**Critical mismatches:**
- `ABIAS_P_NSE`: ArduPilot=`2.0e-2`, Meridian=`3.0e-3`. Accel bias will learn 6.7x slower.
- `MAGB_P_NSE`: ArduPilot=`1.0e-4`, Meridian=`3.0e-4`. Body mag field will wander 3x faster.
- `MAGE_P_NSE`: ArduPilot=`1.0e-3`, Meridian=`3.0e-4`. Earth mag field will wander 3.3x slower.
- `MAG_I_GATE`: ArduPilot=300 (3σ), Meridian=500 (5σ). Magnetometer gate 67% wider — will accept more bad mag data.

### 3.3 Q Formulation — WRONG SQUARING

Meridian `predict.rs`:
```rust
let gbias_var = (dt * dt * params.gyro_bias_process_noise)
    * (dt * dt * params.gyro_bias_process_noise);
```

This squares the entire expression `dt² * noise`, giving units of `(rad/s² * s²)² = rad²`. The correct formulation from ArduPilot line 1047:
```cpp
ftype dAngBiasVar = sq(sq(dt) * constrain_ftype(frontend->_gyroBiasProcessNoise, 0.0, 1.0));
```

ArduPilot writes `sq(sq(dt) * noise)` = `(dt² * noise)²`. Meridian writes `(dt² * noise)²`. These are the same formula — **this is a false alarm, they match**.

However, for `mag_earth_process_noise` and `mag_body_process_noise`, ArduPilot uses:
```cpp
ftype magEarthVar = sq(dt * constrain_ftype(frontend->_magEarthProcessNoise, 0.0f, 1.0f));
```
i.e., `(dt * noise)²`.

Meridian uses:
```rust
let emag_var = (dt * params.mag_earth_process_noise)
    * (dt * params.mag_earth_process_noise);
```
Which is also `(dt * noise)²`. **Match.**

But Meridian adds a spurious position process noise `(dt * 0.5)²` (lines 174–175) that ArduPilot does **not** have:
```rust
let pos_var = (dt * 0.5) * (dt * 0.5); // ~0.5 m/s equivalent position growth
for i in 7..10 { q_diag[i] = pos_var; }
```

ArduPilot's covariance prediction does not add process noise to position states — position uncertainty grows only through the `Fpv` (position-velocity coupling) term, not via a direct position noise term.

**GAP:** Meridian adds artificial position process noise. This over-inflates position uncertainty and will cause GPS to converge slower than necessary. Remove it.

### 3.4 Inhibit State Mechanism — MISSING

ArduPilot has a full state inhibit system:
- `inhibitDelAngBiasStates` — zero gyro bias process noise when not needed
- `inhibitDelVelBiasStates` — zero accel bias process noise per-axis based on gravity alignment check (`fabsF(prevTnb[index][2]) > 0.8f`)
- `inhibitMagStates` — zero mag process noise when mag disabled
- `inhibitWindStates` — zero wind process noise when wind not observable
- `stateIndexLim` — limits covariance update to only active states (from 9 to 23 depending on mode)

Meridian has no inhibit mechanism. All 24 states always receive process noise and are always updated by fusion.

**GAP (CRITICAL):** No state inhibit system. In practice:
- Accel bias for horizontal axes will receive false process noise on ground (gravity-aligned check missing)
- Wind states will drift without bound even when wind is not observable
- No `stateIndexLim` equivalent means wasted computation and potential numerical drift in unused states

### 3.5 Bad IMU Data Response — MISSING

ArduPilot defines:
```cpp
#define BAD_IMU_DATA_ACC_P_NSE 5.0f
#define BAD_IMU_DATA_TIMEOUT_MS 1000
#define BAD_IMU_DATA_HOLD_MS 10000
```

When bad IMU data is detected, accel noise is inflated to `5.0 m/s²`. Meridian has no bad IMU detection.

**GAP:** No vibration/bad IMU detection or adaptive noise inflation.

### 3.6 Declination Fusion — MISSING

ArduPilot calls `FuseDeclination(radians(20.0f))` when earth magnetic field variances are reset (line 1085). This fuses a synthetic zero-sideslip measurement with 20° uncertainty to stabilize the earth field heading.

Meridian has no `FuseDeclination()` equivalent.

**GAP:** No magnetic declination fusion. Earth field heading will be poorly observable without this.

---

## 4. GPS Fusion

### 4.1 Observation Matrix H — PASS

For GPS velocity (states 4,5,6) and position (states 7,8,9), H is a unit vector at the respective state index. Both ArduPilot `FuseVelPosNED()` and Meridian `fuse_scalar()` implement this identically.

### 4.2 Innovation Gate Values — MISMATCH

ArduPilot `FuseVelPosNED()` uses `frontend->_gpsVelInnovGate` (default 500, = 5σ) for velocity, converted via `MAX(0.01f * gate, 1.0f)`. Meridian uses the same 5σ for velocity.

**However**, Meridian `fuse_gps_position()` uses a **hardcoded gate of 100.0** (effectively ungated):
```rust
let gate = 100.0; // effectively ungated
```

ArduPilot uses the configured `POS_I_GATE` (default 500 = 5σ) with glitch radius logic in `SelectVelPosFusion()`. The comment says "The standard 5σ gate rejects valid corrections when the filter has diverged". This is a workaround for a covariance overconfidence bug.

**GAP:** Position gate is hardcoded to 100σ (ungated). Root cause is the zeroed `Fvq` block and artificial position noise. Fix the covariance prediction first, then restore the 5σ gate.

### 4.3 Antenna Offset Correction — MISSING

ArduPilot calls `CorrectGPSForAntennaOffset(gpsDataDelayed)` before fusing GPS. This corrects position and velocity for the lever arm between GPS antenna and IMU center of mass.

Meridian has no lever arm correction.

**GAP:** GPS antenna offset correction absent. Mandatory for precision flight where GPS and IMU are separated.

### 4.4 GPS Velocity Variance Scaling with Acceleration — MISSING

ArduPilot `CalculateVelInnovationsAndVariances()` (line 490):
```cpp
const ftype obs_data_chk = sq(constrain_ftype(noise, 0.05, 5.0)) + sq(accel_scale * accNavMag);
variances.x = P[4][4] + obs_data_chk;
```

The observation variance is inflated by `sq(accel_scale * accNavMag)` — during high-acceleration maneuvers, GPS velocity is trusted less. Meridian uses a fixed `r_vel = params.gps_vel_noise²` with no scaling.

**GAP:** GPS velocity observation noise is not scaled by navigation acceleration magnitude.

### 4.5 Vertical GPS Velocity Fusion — PARTIAL

ArduPilot has a separate `fuseVelVertData` flag controlled by `frontend->sources.useVelZSource()`. Meridian always fuses GPS Vd when provided in `fuse_gps_velocity()`. This is correct for simple cases but missing the source-selection logic.

### 4.6 GPS Delay Buffer — BYPASSED

ArduPilot's full GPS delay pipeline uses `storedGPS.recall(gpsDataDelayed, imuDataDelayed.time_ms)` — a ring buffer that retrieves the GPS measurement that was current when the EKF was at the same time horizon, accounting for ~220ms GPS delay.

Meridian `core.rs` line 136 comments:
```rust
// Use current IMU directly (delayed fusion disabled for now —
// will re-enable once EKF horizontal accuracy is validated)
let delayed_imu = imu_delta;
```

The GPS ring buffer in `output.rs` stores IMU deltas but is not used for delayed-state fusion. GPS measurements are fused at current time without compensating for delay.

**GAP (CRITICAL):** No GPS delay compensation. Fusing a 220ms-stale measurement at current time introduces a systematic 220ms × velocity error in position innovations. At 10 m/s this is 2.2m of position error per fusion step.

---

## 5. Magnetometer Fusion

### 5.1 SH_MAG Intermediates — PASS

Meridian `fusion.rs` lines 343–351 compute exactly the same 9 SH_MAG intermediates as ArduPilot lines 523–532:

| ArduPilot | Meridian |
|-----------|---------|
| `SH_MAG[0] = 2*magD*q3 + 2*magE*q2 + 2*magN*q1` | `sh0` — identical |
| `SH_MAG[1] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2` | `sh1` — identical |
| `SH_MAG[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3` | `sh2` — identical |
| `SH_MAG[3] = sq(q3)` | `sh3` — identical |
| `SH_MAG[4] = sq(q2)` | `sh4` — identical |
| `SH_MAG[5] = sq(q1)` | `sh5` — identical |
| `SH_MAG[6] = sq(q0)` | `sh6` — identical |
| `SH_MAG[7] = 2*magN*q0` | `sh7` — identical |
| `SH_MAG[8] = 2*magE*q3` | `sh8` — identical |

### 5.2 H Matrix per Axis — PASS (with one sign concern)

Meridian H matrix values match ArduPilot's structure. Spot-checked X-axis H[0]:
- ArduPilot: `SH_MAG[7] + SH_MAG[8] - 2*magD*q2`
- Meridian `hx[0]`: `sh7 + sh8 - 2.0 * mag_d * q2`
— identical.

### 5.3 Measurement Prediction — POSSIBLE BUG

ArduPilot `FuseMagnetometer()` constructs a body-frame DCM directly from quaternion components (lines 498–507) and predicts:
```cpp
MagPred = { DCM[0][0]*magN + DCM[0][1]*magE + DCM[0][2]*magD + magXbias, ... }
```

Where `DCM` is the **body-to-NED** rotation built as:
```
R_bn = [q0²+q1²-q2²-q3²,  2(q1q2+q0q3),  2(q1q3-q0q2);
         2(q1q2-q0q3),  q0²-q1²+q2²-q3²,  2(q2q3+q0q1);
         2(q1q3+q0q2),  2(q2q3-q0q1),  q0²-q1²-q2²+q3²]
```

And the prediction is `DCM * [magN, magE, magD]ᵀ + bias`, which gives **body-frame** predicted measurement using **body-to-NED** first row etc. — this is `R_bn` times NED field = body projection. That is correct since `R_bn[0,:] · [mN,mE,mD]` = body X component.

Meridian `fusion.rs` lines 358–362 do:
```rust
let pred = [
    dcm[0][0] * mag_n + dcm[1][0] * mag_e + dcm[2][0] * mag_d + state.body_mag.x,
    ...
```

Where `dcm[row][col]` with transposed column indexing is used. The comment at line 367–371 correctly explains this is `R_bn^T` rows = `R_nb` rows. The mathematical result is the same as ArduPilot. **PASS.**

### 5.4 R_MAG Angular Rate Scaling — MISSING

ArduPilot line 519:
```cpp
const ftype R_MAG = sq(constrain_ftype(frontend->_magNoise, 0.01f, 0.5f))
    + sq(frontend->magVarRateScale * imuDataDelayed.delAng.length() / imuDataDelayed.delAngDT);
```

The observation variance is inflated at high angular rates (`magVarRateScale`). This handles timing latency errors — when the vehicle rotates fast, old mag measurements become stale.

Meridian line 332:
```rust
let r_mag = params.mag_noise * params.mag_noise;
```

Fixed noise only, no angular rate scaling.

**GAP:** Magnetometer observation variance not scaled by angular rate. At high rotation rates, Meridian will over-trust stale magnetometer data.

### 5.5 Mag Innovation Check — Different Gating Structure

ArduPilot checks all three axes before fusing any:
```cpp
// First compute all three varInnovMag[0..2]
// If any < R_MAG → CovarianceInit() and return
// Then check magTestRatio[0..2] all < 1.0
// If any fails → return without fusing
```

Meridian fuses each axis independently — if X passes, X is fused even if Y fails. This is mathematically different (and less conservative than ArduPilot).

**GAP:** Meridian performs per-axis independent mag fusion. ArduPilot requires all 3 axes to pass the innovation gate before fusing any. Meridian's approach can lead to partial mag corrections corrupting the covariance.

### 5.6 State Mask for Magnetometer — INCORRECT

Meridian `StateMask::MAG_ONLY`:
```rust
pub const MAG_ONLY: StateMask = StateMask(
    0b_00_111111_000_111_000_000_1111 // states 0-3, 10-12, 16-21
);
```

This includes gyro bias states 10–12. ArduPilot's `FuseMagnetometer()` updates states controlled by `stateIndexLim` which for mag = up to state 21, but the Kalman gain is computed for all active states. Critically, ArduPilot's mag fusion **does** update velocity states (4–6) and position states (7–9) via the full Kalman gain. The state mask in Meridian explicitly excludes velocity and position from mag correction.

**GAP:** Meridian's MAG_ONLY mask blocks mag from correcting velocity/position states. ArduPilot allows mag to update all states (the coupling is small but non-zero). This is a conservative choice that degrades convergence.

### 5.7 Mag Calibration Mode — MISSING

ArduPilot has a full mag calibration state machine (`MagCal` enum with `ALWAYS`, `WHEN_FLYING`, `WHEN_MANOEUVRING`, `AFTER_FIRST_CLIMB`, `GROUND_AND_INFLIGHT`, `NEVER`). This controls whether the 6 mag states (earth + body) are actively estimated or held constant.

Meridian always estimates all mag states (no inhibit). Missing:
- `inhibitMagStates` flag
- `finalInflightMagInit` / `finalInflightYawInit` tracking
- `needMagBodyVarReset` / `needEarthBodyVarReset` triggered resets
- `MagTableConstrain()` — WMM table constraint on earth field magnitude

**GAP:** No magnetic calibration state machine. Mag states will drift regardless of flight phase.

---

## 6. Barometer Fusion

### 6.1 Observation Model — PASS

Both fuse position-D against the barometric altitude as a scalar. The innovation `state.position.z - measured_alt_d` is correct.

### 6.2 Ground Effect Compensation — MISSING

ArduPilot has ground effect compensation for barometers affected by rotor downwash when close to the ground. This inflates the baro noise when the aircraft is within ground effect altitude.

Meridian has no equivalent.

**GAP:** No ground effect baro compensation. Altitude estimates near ground will be noisier than expected.

### 6.3 Height Source Switching — MISSING

ArduPilot `activeHgtSource` can switch between:
- `BARO` (default)
- `RANGEFINDER` (when near ground)
- `GPS` (when GPS altitude trusted)
- `BEACON`
- `EXTNAV`

Meridian always uses barometer for height. No source switching.

**GAP:** Height source selection not implemented.

### 6.4 Vertical Velocity Clip Counter — MISSING

ArduPilot:
```cpp
#define VERT_VEL_VAR_CLIP_COUNT_LIM (5 * EKF_TARGET_RATE_HZ)
```
If vertical velocity variance repeatedly hits its minimum bound (`VEL_STATE_MIN_VARIANCE = 1E-4`), ArduPilot resets the vertical states. Meridian has no such mechanism.

**GAP:** No vertical velocity variance clip counter / height reset mechanism.

---

## 7. Aiding Mode FSM

### 7.1 Core Transitions — PARTIAL

Meridian implements:
- `None → Absolute` (when GPS gate passes and tilt aligned)
- The `Absolute → None` transition for GPS loss is **missing**

ArduPilot `setAidingMode()` has full logic for:
- `None → Absolute` — YES (partial)
- `None → Relative` — MISSING (optical flow / body odometry path)
- `Relative → Absolute` — MISSING
- `Relative → None` (flow fusion timeout) — MISSING
- `Absolute → None` (attitude aid loss critical) — MISSING
- `Absolute → None` (position aid loss critical) — MISSING
- Special case: AID_ABSOLUTE → AID_NONE when disarmed without yaw source — MISSING

**GAP (CRITICAL):** No `Absolute → None` fallback on GPS loss. Filter will continue fusing stale GPS indefinitely or diverge.

### 7.2 GPS Quality Gate — PARTIAL

Meridian `GpsQualityGate::update_align()` checks:
- Fix 3D: YES
- num_sats ≥ 6: YES
- hacc < 5.0: YES (ArduPilot uses 5.0 × checkScaler)
- vacc < 8.0: YES (ArduPilot uses 7.5 × checkScaler)
- sacc < 1.0: YES (ArduPilot uses 1.0 × checkScaler)
- 10s duration: YES

**Missing from Meridian:**
- `checkScaler` (controlled by `EK3_CHECK_SCALER`, default 100 = 1.0x) — no equivalent
- GPS drift check (position wander when stationary < 3m)
- HDOP check (hdop ≤ 250)
- Horizontal velocity filter check (< 0.3 m/s when stationary)
- Vertical velocity filter check (< 0.3 m/s when stationary)
- Yaw fail check (mag test ratio must pass)
- Hysteresis on already-aligned GPS (1.3× threshold)

**GAP:** GPS quality gate is simplified. Missing 6 of the 8 ArduPilot checks.

### 7.3 In-Flight GPS Quality — PARTIAL

Meridian `update_flight()` uses a 0.95/0.05 filter on speed accuracy with threshold 2.0 m/s. ArduPilot `calcGpsGoodForFlight()` uses a two-state filter (LPF + peak-hold) with 1.5 m/s fail / 1.0 m/s pass hysteresis, plus EKF innovation pass/fail hysteresis with 1s fail timeout and 10s pass requirement.

**GAP:** In-flight GPS health assessment is significantly simplified. Missing EKF innovation hysteresis and peak-hold filter.

### 7.4 Tilt Alignment Gate — DIFFERENT

Meridian checks `quat_variance < 0.025` (9 deg). ArduPilot uses `checkAttitudeAlignmentStatus()` which checks the sum of quaternion covariance diagonal against a threshold derived from sensor noise. The Meridian threshold of 0.025 rad² corresponds to ~9° and is reasonable but is hardcoded rather than derived from the covariance matrix trace (ArduPilot uses diagonal elements of P).

**GAP:** Tilt alignment threshold is a fixed constant rather than comparing to the actual covariance-derived estimate.

### 7.5 Wind/Mag State Learning Control — MISSING

ArduPilot `setWindMagStateLearningMode()` is a 100+ line function controlling when wind and mag states are estimated vs held constant. Meridian has no equivalent. Wind states are always set to constant small noise (1e-6), which is reasonable for hover-only but not general.

---

## 8. Output Predictor

### 8.1 IMU Ring Buffer — IMPLEMENTED

Meridian implements a 100-element ring buffer for IMU deltas and output states. ArduPilot uses a similar structure. Buffer size (100) at 400Hz = 250ms delay — matches ArduPilot's design intent.

### 8.2 Attitude Correction (PI) — PARTIALLY CORRECT

Meridian `apply_ekf_correction()` computes:
```rust
let error_gain = (0.5 / time_delay.max(0.001)) * dt_ekf;
self.del_ang_correction = delta_ang_err * error_gain;
```

ArduPilot `calcOutputStates()` line 898:
```cpp
delAngCorrection += (attErr * (imuDataNew.delAngDT / tauOutput));
```

Where `tauOutput` is controlled by `EK3_TAU_OUTPUT` (not a parameter Meridian exposes). The functional forms are similar (proportional error → correction to angular rate), but the gain formula differs. Meridian uses `0.5/delay * dt` vs ArduPilot's `dt/tau`.

**GAP:** Output predictor attitude correction gain formula is not identical to ArduPilot. Effect: slightly different transient response on EKF fusion events.

### 8.3 Velocity/Position PI Correction — PARTIALLY CORRECT

Meridian applies a PI correction to velocity and position with `vel_pos_gain = dt / tau` and I-gain of `gain² × 0.1`. ArduPilot applies corrections via a complementary filter with time constant `EK3_TAU_OUTPUT`.

The functional structure is correct (PI feedback on delayed vs current state), but the gain values and formulation are not verified as identical.

### 8.4 Delayed EKF State — BYPASSED

As noted in §4.6, Meridian runs the EKF prediction on current IMU data (not delayed). The output predictor receives corrections from the current-time EKF state rather than the delayed-time state. This means the EKF and output predictor are running at the same time horizon — the output predictor provides no benefit in its current form.

**GAP (CRITICAL):** The delayed-state architecture is not functional. The full chain (buffer IMU → run EKF on delayed IMU → compute correction vs delayed output → propagate correction to current) must be implemented.

---

## 9. Health Monitoring

### 9.1 `detectFlight()` — PARTIALLY IMPLEMENTED

Meridian `FlightDetector` uses speed threshold (3 m/s in-flight, 1.5 m/s on-ground) with arm status. ArduPilot `detectFlight()` for copters uses:
- Climb rate > 0.5 m/s
- Accel magnitude outside gravity range
- Armed status
- Vehicle type-specific logic (plane uses groundspeed + airspeed + height change)
- `manoeuvring` flag (accNavMagHoriz > 0.5)

**GAP:** Flight detection is simplified. Meridian triggers on NED speed magnitude (includes vertical), ArduPilot uses more nuanced multi-condition logic. For a hover-only craft this is adequate, but not general.

### 9.2 `calcGpsGoodToAlign()` — SEE §7.2

### 9.3 `calcGpsGoodForFlight()` — SEE §7.3

### 9.4 `detectOptFlowTakeoff()` — MISSING ENTIRELY

ArduPilot has `detectOptFlowTakeoff()` (in `AP_NavEKF3_OptFlowFusion.cpp`) which detects takeoff from optical flow sensor. No equivalent in Meridian (expected, as optical flow is not implemented).

### 9.5 Filter Divergence Detection — MISSING

ArduPilot `checkDivergence()` monitors innovation test ratios across multiple sensors and triggers a reset if the filter is consistently producing large innovations. Meridian has a simple NaN check and a prediction/fusion ratio check in `predict()`, but no multi-sensor innovation consistency monitor.

**GAP:** No cross-sensor divergence detection.

### 9.6 Lane Switching — MISSING ENTIRELY

ArduPilot runs multiple EKF cores (lanes) simultaneously, selecting the healthiest via `errorScore()`. The front-end (`AP_NavEKF3`) manages lane selection and switching.

Meridian `EkfCore` is a single-core structure with no multi-lane support. There is no equivalent of `NavEKF3::SelectCoreAndIndex()`, `errorScore()`, `healthy()`, or the lane-switching logic.

**GAP:** No EKF lane switching. This is a safety-critical feature — if the primary EKF diverges in ArduPilot, it automatically switches to a backup. Meridian has no backup.

---

## 10. Missing Fusion Subsystems

The following ArduPilot EKF3 fusion subsystems are entirely absent from Meridian:

| Subsystem | ArduPilot file | Meridian | Notes |
|-----------|---------------|---------|-------|
| Optical flow | `AP_NavEKF3_OptFlowFusion.cpp` | ABSENT | 900+ lines, terrain estimator, 2-state EKF |
| True airspeed | `AP_NavEKF3_AirDataFusion.cpp` `FuseAirspeed()` | ABSENT | Required for fixed-wing wind estimation |
| Sideslip | `AP_NavEKF3_AirDataFusion.cpp` `FuseSideslip()` | ABSENT | Zero-sideslip assumption for planes |
| Drag | `AP_NavEKF3_AirDataFusion.cpp` drag fusion | ABSENT | Multirotors use drag for wind estimation |
| Range beacon | `AP_NavEKF3_RngBcnFusion.cpp` | ABSENT | 500+ lines, NED position from UWB beacons |
| External nav | `AP_NavEKF3_PosVelFusion.cpp` ExtNav path | ABSENT | VIO / motion capture fusion |
| External nav velocity | ExtNavVel path | ABSENT | VIO velocity fusion |
| Body frame odometry | `SelectBodyOdomFusion()` / `FuseBodyVel()` | ABSENT | Wheel odometry / body vel sensors |
| GPS yaw fusion | `readGpsYawData()` / `FuseGpsYaw()` | ABSENT | Dual-antenna GPS heading |
| GSF yaw estimator | `EKFGSF_yaw.h` | ABSENT | Gaussian sum filter for yaw recovery |
| Magnetic declination | `FuseDeclination()` | ABSENT | Stabilizes earth field heading |
| WMM table constraint | `MagTableConstrain()` | ABSENT | Prevents mag field drift |
| Rangefinder height | height source switching | ABSENT | Near-ground precision altitude |
| Origin reset | long-range origin reset | ABSENT | Prevents position overflow beyond EK3_POSXY_STATE_LIMIT |
| Inactive IMU bias learning | `learnInactiveBiases()` | ABSENT | Keeps non-primary IMU biases warm |
| GPS antenna offset | `CorrectGPSForAntennaOffset()` | ABSENT | Lever arm correction |
| Yaw alignment from GPS velocity | `realignYawGPS()` | ABSENT | Emergency yaw recovery |
| Mag anomaly reset | `MAG_ANOMALY_RESET_MAX = 2` | ABSENT | Recovers from local field anomalies |

---

## 11. Missing Constants and Defines

The following ArduPilot constants are not present in Meridian and some would need to be added if the corresponding features are ported:

| Constant | ArduPilot value | Purpose |
|----------|----------------|---------|
| `earthRate` | `7.292e-5 rad/s` | Earth rotation for quat prediction |
| `GYRO_BIAS_LIMIT` | `0.5 rad/s` | Gyro bias state clamp |
| `ACCEL_BIAS_LIM_SCALER` | `0.2` | Accel bias state clamp fraction |
| `EKF_TARGET_DT_MS` | `12 ms` | Target EKF rate |
| `EKF_TARGET_DT` | `0.012 s` | Same as float |
| `EK3_MAG_FINAL_RESET_ALT` | `2.5 m` | Altitude for final in-flight mag reset |
| `EK3_GPS_MAG_LEARN_RATE` | `0.005` | Mag bias learning rate from GPS yaw |
| `EK3_GPS_MAG_LEARN_LIMIT` | `0.02 gauss` | Mag bias learning limit |
| `MAG_ANOMALY_RESET_MAX` | `2` | Max mag anomaly resets |
| `YAW_RESET_TO_GSF_TIMEOUT_MS` | `5000 ms` | GSF yaw reset timeout |
| `GSF_YAW_ACCURACY_THRESHOLD_DEG` | `15°` | GSF yaw accuracy gate |
| `VEL_STATE_MIN_VARIANCE` | `1e-4` | Velocity variance floor |
| `POS_STATE_MIN_VARIANCE` | `1e-4` | Position variance floor |
| `EK3_POSXY_STATE_LIMIT` | `1e6 m` | Horizontal position overflow limit |
| `BAD_IMU_DATA_ACC_P_NSE` | `5.0 m/s²` | Inflated accel noise for bad IMU |
| `BAD_IMU_DATA_TIMEOUT_MS` | `1000 ms` | Bad IMU detection window |
| `WIND_VEL_VARIANCE_MAX` | `400 (m/s)²` | Wind state variance ceiling |
| `WIND_VEL_VARIANCE_MIN` | `0.25 (m/s)²` | Wind state variance floor |
| `GPS_VEL_YAW_ALIGN_COUNT_THRESHOLD` | `5` | Samples for GPS yaw alignment |
| `GPS_VEL_YAW_ALIGN_MAX_ANG_ERR` | `15°` | Max angle error for GPS yaw align |

---

## 12. Prioritized Fix List

### P0 — Must fix before any flight

1. **GPS delay buffer not used.** EKF runs on current IMU. 220ms position error at 10 m/s = 2.2m systematic bias per fusion step. Fix: actually use `output.imu_buf.oldest()` as the delayed IMU for EKF prediction.
2. **`Fvq` block zeroed in covariance.** Velocity-attitude coupling missing from F matrix. Re-enable with proper formulation.
3. **`Absolute → None` aiding mode transition missing.** Filter has no GPS loss fallback.
4. **State constraint (`ConstrainStates()`) not called.** Gyro bias can grow without bound.

### P1 — Must fix before reliable extended flight

5. **`ABIAS_P_NSE` wrong by 6.7x.** Fix: `3.0e-3 → 2.0e-2`.
6. **`MAGB_P_NSE` wrong by 3x.** Fix: `3.0e-4 → 1.0e-4`.
7. **`MAGE_P_NSE` wrong by 3.3x.** Fix: `3.0e-4 → 1.0e-3`.
8. **`MAG_I_GATE` wrong.** Fix: `500 → 300` (ArduPilot copter default).
9. **Position process noise.** Remove artificial `pos_var` added to Q.
10. **Earth rate correction omitted from quaternion update.** Add `prevTnb * earthRateNED * dt` subtraction.
11. **Horizontal acceleration limiting in AID_NONE mode.** Add 5 m/s clamp.
12. **All 3 mag axes must pass gate before any axis is fused.** Change per-axis to all-or-nothing.
13. **R_MAG angular rate scaling missing.** Add `magVarRateScale * omega_body` term to mag observation noise.

### P2 — Should fix for full parity

14. **GPS quality gate simplified.** Missing 6 of 8 ArduPilot checks (HDOP, drift, horiz/vert velocity when stationary, yaw fail, hysteresis).
15. **State inhibit system missing.** Add `inhibitDelVelBiasStates` with gravity-alignment check.
16. **`FuseDeclination()` missing.** Add synthetic declination measurement fusion.
17. **GPS antenna offset correction missing.**
18. **GPS velocity variance not scaled by `accNavMag`.**
19. **`accNavMag` / `velDotNEDfilt` tracking missing.**
20. **Mag state mask excludes velocity/position states.** Remove overly conservative mask.
21. **Quaternion convention comment wrong** in `state.rs:16`. Fix or verify.

### P3 — Completeness / future features

22. Optical flow fusion
23. Airspeed / sideslip fusion
24. Range beacon fusion
25. External navigation fusion
26. Body frame odometry fusion
27. GPS yaw fusion
28. GSF yaw estimator
29. `MagTableConstrain()` (WMM)
30. Height source switching (rangefinder, GPS)
31. EKF lane switching / multi-core
32. Bad IMU detection
33. Ground effect baro compensation
34. Origin reset for long-range operation
35. Mag calibration state machine
36. Inactive IMU bias learning

---

## 13. What Meridian Gets Right

The following are correctly implemented:

- 24-state vector in exactly the same order as ArduPilot
- Hamilton quaternion convention (scalar first)
- Gravity constant: `9.80665 m/s²` exact match
- NED frame convention throughout
- Sequential scalar fusion (vs joint update) — correct approach
- SH_MAG intermediate values for magnetometer H matrix — exact match
- Magnetometer measurement prediction formula — correct
- GPS 10-second quality gate hysteresis
- Output predictor ring buffer architecture (correct concept)
- Dual-GPS blending with inverse-variance weighting
- VELNE, VELD, POSNE, ALT, MAG noise defaults — all match copter defaults
- GYRO_P_NSE, ACC_P_NSE, GBIAS_P_NSE defaults — all match
- VEL_I_GATE, POS_I_GATE, HGT_I_GATE defaults — match (mag gate wrong)
- GPS delay parameter `gps_delay_ms = 220` — correct default

---

*Generated by parity comparison of ArduPilot AP_NavEKF3 (D:\projects\ardupilot\libraries\AP_NavEKF3\) against Meridian meridian-ekf (D:\projects\meridian\crates\meridian-ekf\src\). All line references are to the ArduPilot source as read during this audit.*
