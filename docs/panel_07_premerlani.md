# Panel Review — DCM Fallback, AHRS, Vertical CF, Attitude Fundamentals
## Reviewer: Bill Premerlani (DCM Algorithm, MatrixPilot, early ArduPilot)
## Date: 2026-04-02

---

## Preamble

I wrote the DCM paper. I spent years watching well-intentioned implementations
drift into numerical garbage and take aircraft with them. I am going to be
specific about what is right, what is wrong, and what will kill you.

Files reviewed: `crates/meridian-ekf/src/dcm.rs`, `vert_cf.rs`, `output.rs`,
`crates/meridian-ahrs/src/lib.rs`.

---

## 1. Orthonormalization

**Rating: ACCEPTABLE with reservations**

The method used is Gram-Schmidt on rows 0 and 1, then row 2 = cross(row0,
row1). This is the same approach I documented in the DCM paper and that
ArduPilot AP_AHRS_DCM uses. The implementation is numerically correct as
written.

However, there is a subtle preference issue. My original algorithm distributes
the orthonormality error symmetrically between row 0 and row 1 before
normalization, rather than anchoring row 0 and projecting row 1. The exact
form is:

```
error = dot(row0, row1)
row0_corrected = row0 - (error/2) * row1
row1_corrected = row1 - (error/2) * row0
normalize(row0_corrected), normalize(row1_corrected)
row2 = cross(row0_corrected, row1_corrected)
```

The implementation here anchors row 0 (normalize first, then project row 1
away from it). This is slightly less symmetric. Over long flight times the
error accumulates differently in roll versus pitch. For a safety fallback that
runs for seconds before the pilot recovers, this is acceptable. For a primary
estimator running indefinitely, I would fix it to the symmetric form.

There is a second issue. The guard `if len0 > 1e-6` and `if len1 > 1e-6`
protects against divide-by-zero but does not reset the row if it collapses.
If a degenerate state is reached (numerical catastrophe), the row is left
as-is and the estimator silently produces garbage rather than resetting to the
last good state. A production system should detect `len < 1e-6` and trigger a
full reset.

The cross product for row 2 is:

```
row2[0] = row0[1]*row1[2] - row0[2]*row1[1]   (correct)
row2[1] = row0[2]*row1[0] - row0[0]*row1[2]   (correct)
row2[2] = row0[0]*row1[1] - row0[1]*row1[0]   (correct)
```

That is the standard right-hand rule. It is correct for a NED, body-forward
frame where the third row should point down in level flight. No error here.

---

## 2. Complementary Filter Gain and Mahony-Style Correction

**Rating: ACCEPTABLE — gain choice is defensible but untested under dynamics**

### Gain value

`p_gain = 0.2` is a proportional-only complementary filter (no integral term).

In ArduPilot AP_AHRS_DCM the default is `AP_AHRS_RP_P = 0.2` with a
corresponding `AP_AHRS_RP_I = 0.02`. The integral term is critical — it
removes gyro bias over time. This implementation has no integral (I) term.

Without an integral term, a constant gyro bias will produce a constant tilt
error in steady state. For a fallback that runs for a few seconds while the
pilot switches to manual or the EKF recovers, this is tolerable. For a
fallback that runs for minutes (e.g. EKF never recovers, operator does not
notice), the bias error will cause the vehicle to be visibly off-level. A
multirotor with even 0.1 deg/s gyro bias will have ~0.5 deg of steady-state
tilt error at p_gain=0.2. That is flyable. At 1 deg/s bias the error reaches
~5 deg, which is marginal for tight position hold and unacceptable for
anything precision.

The recommendation is to add an integral term at `i_gain = 0.02` (the ArduPilot
default), applying it as a bias correction to the gyro input. This is the
Mahony formulation.

### Cross product direction

```rust
let error = accel_norm.cross(&accel_predicted);
```

The tilt error vector is `cross(measured_normalized, predicted)`. Let me check
the sign convention.

- `accel_predicted` is the third column of the DCM in body frame, which is the
  gravity direction as the DCM believes it. For a level vehicle pointing
  forward (DCM = identity), `accel_predicted = [0, 0, 1]` (gravity pointing
  body-Z down in NED convention).
- `accel_norm` is the normalized measured acceleration. For a level vehicle,
  this is also `[0, 0, 1]` (gravity in body frame).
- If the vehicle is tilted, `accel_norm != accel_predicted`, and `cross(norm,
  predicted)` produces a rotation vector that, when added to the gyro, rotates
  the DCM toward alignment.

The sign is correct for the convention used. The direction of the cross product
gives the rotation axis needed to align `predicted` with `measured`, and
applying it as a positive correction to omega is the standard Mahony approach.

### Acceleration gating

The gate `g_ratio > 0.5 && g_ratio < 2.0` is correct in principle but has one
edge case. The outer check is `if accel_len > 1.0` before entering the gate.
This means that if `accel_len` is between 0 and 1 m/s² (i.e. near-zero
acceleration, which could be free fall or a sensor failure), no correction is
applied and no branching into the inner gate occurs. The inner gate lower bound
of `0.5g` (~4.9 m/s²) would have caught this case anyway, but the outer check
makes the threshold documentation misleading — the effective lower bound is
`max(1.0 m/s², 0.5*9.81)` = 4.9 m/s², so the outer check is dead logic. It
should either be removed or changed to `accel_len > 0.5 * 9.80665`.

---

## 3. Small-Angle Rotation Application

**Rating: CORRECT**

```rust
new_dcm[i][0] = r[0] + r[1] * wz - r[2] * wy;
new_dcm[i][1] = r[1] - r[0] * wz + r[2] * wx;
new_dcm[i][2] = r[2] + r[0] * wy - r[1] * wx;
```

This is `R_new = R_old * (I + [omega_dt x])` where `[omega_dt x]` is the
right-side skew-symmetric rotation. The skew matrix for body-frame omega is:

```
[  0  -wz   wy ]
[ wz    0  -wx ]
[-wy   wx    0 ]
```

Applying on the right to each row `r` gives:
- `r_new[0] = r[0]           - r[1]*(-wz) - r[2]*(-(-wy))` ... let me expand fully.

`r_new = r * (I + S)` where `S` is the skew matrix above:

```
r_new[0] = r[0]*1 + r[1]*wz  + r[2]*(-wy)  = r[0] + r[1]*wz - r[2]*wy   ✓
r_new[1] = r[0]*(-wz) + r[1]*1 + r[2]*wx   = r[1] - r[0]*wz + r[2]*wx   ✓
r_new[2] = r[0]*wy + r[1]*(-wx) + r[2]*1   = r[2] + r[0]*wy - r[1]*wx   ✓
```

All three components are correct. The rotation is applied in body frame, which
is the correct formulation for a body-frame gyro input updating a body-to-NED
DCM.

---

## 4. Vertical Complementary Filter

**Rating: MARGINAL — missing third-order term, time constant marginally tuned**

### Filter order

The `VerticalCF` claims to be a 3rd-order complementary filter, and the comment
references ArduPilot `AP_InertialNav`. However, the implemented update is
second-order, not third.

A proper 3rd-order vertical complementary filter has three gains:

| Gain | Drives           | Form                    |
|------|------------------|-------------------------|
| k1   | altitude         | 3 / TC                  |
| k2   | climb rate       | 3 / TC²                 |
| k3   | accel bias       | 1 / TC³                 |

The update equations for a true 3rd-order filter are:

```
alt_err = baro_alt - estimated_alt
estimated_alt       += k1 * alt_err * dt  +  climb_rate * dt  +  0.5 * accel * dt²
estimated_vel       += k2 * alt_err * dt  +  accel * dt
estimated_accel_bias += k3 * alt_err * dt
```

This implementation applies `k1` and `k2` to the baro correction only and
integrates accel into climb rate and altitude independently. That is a 2nd-order
complementary filter with a separate accel integrator. The missing piece is the
`k3` term — an accel bias estimate driven by the altitude error. Without k3,
a DC accel bias (systematic error in the vertical accelerometer after
gravity removal) produces unbounded altitude drift. The baro correction pulls
altitude back but the accel bias keeps pushing it away, resulting in a
first-order bounded oscillation rather than true convergence.

For a fallback that blends baro at 50 Hz and accel at 400 Hz, omitting k3
means any accel bias (which is common — the tilt correction from DCM attitude
is never perfect) will cause altitude to oscillate around the baro truth with
an amplitude proportional to the bias and the time constant.

### Time constant

`time_constant = 3.0` seconds. The gains compute to:

```
k1 = 3/3 = 1.0   (altitude correction)
k2 = 3/9 = 0.333 (climb rate correction)
```

At 50 Hz baro updates, each step applies `k1 * alt_err * 0.02 = 0.02 * alt_err`
to altitude. The effective bandwidth for altitude is approximately `k1 / (2*pi) =
0.16 Hz`. This is aggressive enough that baro noise will bleed into altitude at
a noticeable rate. ArduPilot uses TC=3.0 as well, but paired with a baro
pre-filter. There is no baro pre-filter here. With an unfiltered baro input
(which can have 0.5–2m of noise at 50 Hz), the altitude estimate will have
measurable jitter that couples into climb rate via k2.

Recommendation: add a first-order low-pass on the baro input before the
complementary filter, or increase TC to 5.0–10.0 for less baro noise coupling,
or implement the k3 accel bias term to get true 3rd-order behavior.

### Calibration

Five samples at 50 Hz = 100ms of calibration. This is adequate for a stationary
pre-arm check but is dangerously short if the baro has not settled (sensor
warm-up transients can persist 500ms+). The `cal_target = 5` should be at
least 25 (500ms at 50Hz) for a production system. This is not a showstopper for
a fallback altitude estimator but it is a known gap.

---

## 5. Output Predictor (`output.rs`)

**Rating: SOUND — minor issues**

The output predictor architecture is correct: ring buffer of IMU deltas and
output states, correction applied to all buffered states when EKF fuses, PI
feedback for velocity/position error.

The attitude correction via `del_ang_correction` applied each IMU step is the
right approach (matching AP_NavEKF3_core.cpp lines 884-898). The quaternion
multiplication order `q_ekf * delayed.quat.inverse()` correctly gives the error
quaternion in the delayed frame.

One concern: the `apply_correction` method applies `vel_corr` and `pos_corr`
identically to every state in the buffer:

```rust
pub fn apply_correction(&mut self, vel_corr: Vec3<NED>, pos_corr: Vec3<NED>) {
    for i in 0..self.count {
        ...
        self.buf[idx].velocity = self.buf[idx].velocity + vel_corr;
        self.buf[idx].position = self.buf[idx].position + pos_corr;
    }
}
```

This applies the same step correction to both the oldest (delayed) state and
the newest (current) state. The intent from AP_NavEKF3_core.cpp is to apply
corrections in proportion to how far each buffered state is from the fusion
time, not uniformly. Applying the same delta to a state 250ms old and a state
1ms old overcorrects the recent states. In practice this means every EKF fusion
event applies a position/velocity jump to the current output. At typical EKF
rates (GPS at 5Hz, baro at 50Hz) this creates velocity noise spikes at fusion
boundaries. For GPS-rate fusion the jump is ~200ms * vel_error. At 1 m/s error
that is 0.2m of instantaneous position correction applied as a step — visible
in logs but probably not flight-critical. For high-frequency baro fusion it is
much smaller.

The PI gain structure `vel_pos_gain_sq * 0.1` for the integral is not matched
to the loop rate in an obvious way. The gains are dynamically computed from
`dt_ekf / tau_vel_pos`, which couples the integral gain to both step size and
tau. This will work but is harder to tune and reason about than fixed gains.

---

## 6. AHRS Manager (`meridian-ahrs/src/lib.rs`)

**Rating: INCOMPLETE — DCM fallback is a stub**

This is the most critical finding of this review.

The `AhrsSource` enum declares `Dcm` as an option. The `source` field is
initialized to `AhrsSource::Ekf` and never changed. There is no code path that:

1. Detects EKF divergence / unhealthy state
2. Switches `self.source` to `AhrsSource::Dcm`
3. Instantiates or updates a `DcmEstimator`
4. Returns DCM-sourced attitude from `attitude()`, `state()`, or any other
   accessor

The `DcmEstimator` in `dcm.rs` is a complete and functional module. The
`VerticalCF` in `vert_cf.rs` is functional (with caveats noted above). But
neither is wired into the AHRS manager. The fallback exists only in name.

The current behavior when `ekf.health != Healthy` is: `self.ekf.output_state()`
continues to return the last EKF state, whatever that is. If the EKF has
diverged, the AHRS manager returns diverged state to the flight controller
without any warning mode change visible to the caller other than `healthy()`
returning false.

A minimal correct implementation requires:

```rust
pub struct Ahrs {
    ekf: EkfCore,
    dcm: DcmEstimator,       // always running
    vert_cf: VerticalCF,     // always running
    source: AhrsSource,
}
```

With `dcm.update_imu()` called on every `predict()` call regardless of EKF
health, and `attitude()` returning `dcm.attitude()` when `source ==
AhrsSource::Dcm`. The switch to DCM should happen when
`ekf.health != EkfHealth::Healthy` and the switch back to EKF should require
the EKF to be healthy for a minimum time (hysteresis) to prevent thrashing.

Without this wiring, the DCM fallback provides zero safety margin. The EKF
can diverge and the autopilot will fly on garbage attitude with no fallback
engaged.

---

## 7. Summary of Specific Issues

| # | Location | Issue | Severity |
|---|----------|--------|----------|
| 1 | `dcm.rs` `orthonormalize()` | Asymmetric Gram-Schmidt (anchors row 0) vs symmetric preferred form | Low |
| 2 | `dcm.rs` `orthonormalize()` | No reset on degenerate row (len < 1e-6) | Medium |
| 3 | `dcm.rs` `update_imu()` | No integral (I) gain for gyro bias correction | Medium |
| 4 | `dcm.rs` `update_imu()` | Outer `accel_len > 1.0` guard is dead logic vs inner 0.5g gate | Low |
| 5 | `vert_cf.rs` | Filter is 2nd-order, not 3rd-order — missing k3 accel bias term | Medium |
| 6 | `vert_cf.rs` | No baro pre-filter; TC=3s with raw baro will produce visible altitude jitter | Medium |
| 7 | `vert_cf.rs` | Ground calibration uses only 5 samples (100ms) — too short | Low |
| 8 | `output.rs` | `apply_correction()` applies uniform step to all buffered states, not proportional | Low |
| 9 | `ahrs/src/lib.rs` | DCM fallback is completely unwired — never instantiated, never switched to | CRITICAL |

---

## 8. Overall Rating

**UNUSABLE as a safety fallback.**

The individual `DcmEstimator` module is sound and would be flyable in isolation
(with the bias correction gap noted). The vertical CF is functional at the 2nd
order level and would give usable altitude in degraded conditions. The output
predictor is architecturally correct.

But the AHRS manager never activates the fallback. When the EKF diverges, the
vehicle continues to fly on the last EKF output with no fallback engaged and
no attitude source change visible to the flight controller. This is worse than
having no fallback, because it creates a false sense of safety. The
`AhrsSource::Dcm` variant exists, the `DcmEstimator` compiles, but the wiring
between them is missing entirely.

The rating is **UNUSABLE** for the stated purpose of "DCM fallback when EKF
diverges." The fallback cannot engage because it has no trigger condition, no
instantiation in the AHRS struct, and no output path.

The fix is not large. The `DcmEstimator` needs to be added to `Ahrs`, updated
in lockstep with `predict()`, and selected in `attitude()` / `state()` when
`ekf.health != Healthy`. That is approximately 30-40 lines of wiring code. Once
wired, the system rating rises to **MARGINAL** (due to missing I-gain and
incomplete 3rd-order CF), and with the I-gain and k3 fixes it becomes
**FLYABLE** for a degraded-mode fallback.

Fix the wiring first. Everything else is secondary.

---

*Bill Premerlani — DCM Algorithm Author, MatrixPilot / early ArduPilot*
