# Wave 3 Audit: Advanced Navigation, Avoidance, and Signal Processing

**Audited from:** `D:\projects\ardupilot\libraries\`
**Target:** Meridian Rust autopilot at `D:\projects\meridian`
**Files read:** 30+ source files across 16 libraries

---

## AC_AutoTune

### Algorithm Overview

AC_AutoTune is a flight-time automated PID tuner organized as an abstract base class
(`AC_AutoTune`) with two concrete implementations: `AC_AutoTune_Multi` (multirotors) and
`AC_AutoTune_Heli` (helicopters). All source is in
`D:\projects\ardupilot\libraries\AC_AutoTune\`.

The system operates through a three-level state machine:

**Top-level TuneMode:**
- `UNINITIALISED` -> `TUNING` -> `FINISHED` / `FAILED` / `VALIDATING`

**Mid-level Step (inside TUNING):**
- `WAITING_FOR_LEVEL` -> `EXECUTING_TEST` -> `UPDATE_GAINS` -> (repeat or ABORT)

**Tune type sequence (multirotor, set in `set_tune_sequence()`):**
```
RATE_D_UP -> RATE_D_DOWN -> RATE_P_UP -> ANGLE_P_DOWN -> ANGLE_P_UP -> TUNE_COMPLETE
```

Success requires `AUTOTUNE_SUCCESS_COUNT = 4` consecutive passing results per tune type
before advancing. The sequence holds in `tune_seq[6]`, stepped via `tune_seq_index`.

### Multirotor Twitch Test (AC_AutoTune_Multi)

The "twitch" is a step input command that produces a measurable response. There are two
test variants:

**Rate tests (RATE_D_UP, RATE_D_DOWN, RATE_P_UP):**
1. Vehicle is brought level (`WAITING_FOR_LEVEL`)
2. A step rate command is injected: `attitude_control->input_rate_step_bf_roll_pitch_yaw_rads()`
   - Roll/pitch target: 180 deg/s (18000 cdeg/s), minimum 45 deg/s (4500 cdeg/s)
   - Yaw target: 90 deg/s (9000 cdeg/s), minimum 15 deg/s (1500 cdeg/s)
3. Gyro response is filtered with a LPF at `filt_D_hz * 2`
4. Peak rate (`test_rate_max`) and bounce-back rate (`test_rate_min`) are recorded
5. Test terminates when: angle reaches `angle_abort` (20 deg, configurable) OR 2-second timeout

**Angle tests (ANGLE_P_DOWN, ANGLE_P_UP):**
1. A step angle command is injected: `attitude_control->input_angle_step_bf_roll_pitch_yaw_rad()`
   - Target angle: 20 deg (configurable fraction of angle_max)
2. Peak angle and rates are recorded
3. Test completes when angle is reached or timeout

**Abort conditions:**
- Lean angle exceeds `angle_abort` (proportional to angle_max, default ~25 deg)
- Excessive negative bounce-back
- Pilot input override (temporarily pauses tuning, reverts to original gains)

### Rate D Tuning Logic

`updating_rate_d_up()` — increases D until bounce-back exceeds `aggressiveness * target_rate`:
- Default aggressiveness: 0.075 (7.5% of commanded rate as bounce criterion)
- Range 0.05 to 0.20, configurable via `AUTOTUNE_AGGR`
- Minimum D: 0.0005, configurable via `AUTOTUNE_MIN_D`
- D step increment: 5% per iteration (`AUTOTUNE_RD_STEP = 0.05`)

`updating_rate_d_down()` — decreases D until bounce-back is eliminated.

`updating_rate_p_up_d_down()` — raises P while reducing D if overshoot appears.

### Angle P Tuning Logic

`updating_angle_p_down()` — decreases angle P until vehicle cannot reach `target_angle * 1.0375`
within timeout. Stops decreasing when the vehicle barely reaches target.

`updating_angle_p_up()` — increases angle P until vehicle reliably reaches target.

### Gain Backoff

After each phase completes, `set_tuning_gains_with_backoff()` applies a safety margin.
Default backoff: 25% (`AUTOTUNE_GMBK = 0.25`). This means the final P and D gains used
are 75% of the maximum discovered values.

### Multi-Axis Handling

Axes tuned independently in order: ROLL -> PITCH -> YAW -> YAW_D (optional).
Each axis is tracked in an `axes_completed` bitmask:
- Bit 0: ROLL
- Bit 1: PITCH
- Bit 2: YAW (tuned with error filter LPF)
- Bit 3: YAW_D (yaw D gain, only on hardware that benefits)

Yaw uses 75% of calculated max step to avoid instability. Yaw I-to-P ratio is 0.1 (not
1.0 as for roll/pitch) to prevent wash-out.

Gain sets stored separately for each axis: `tune_roll_rp/rd/sp/accel_radss`, etc.

Between tests, `load_intra_test_gains()` restores original gains with I-term = 10% of P
(`AUTOTUNE_PI_RATIO_FOR_TESTING = 0.1`). On completion, I is set to 1x P for roll/pitch,
0.1x P for yaw.

### Helicopter (AC_AutoTune_Heli)

Uses a different approach: **frequency sweep (chirp)** rather than twitch.

Includes `AP_Math::chirp` for sinusoidal frequency sweep input. Additional tune types:
- `RATE_FF_UP` — feedforward gain tuning (dominant for heli rate control)
- `MAX_GAINS` — determines maximum stable P/D gains via frequency sweep
- `TUNE_CHECK` — verifies final gains with another sweep

The `AC_AutoTune_FreqResp` object tracks amplitude ratio and phase lag between commanded
and measured response for both DWELL (fixed frequency) and SWEEP (chirp) inputs. Peak
detection uses a ring buffer (`peak_info`, depth 12) to compare successive half-cycles.

Gain calculation from frequency response:
- `gain = measured_amplitude / command_amplitude`
- `phase = time_of_peak_meas - time_of_peak_tgt` (converted to degrees at test frequency)

### Completion Criteria

A tune type is "complete" when `success_counter >= AUTOTUNE_SUCCESS_COUNT (4)`. The
counter increments when the twitch result falls within acceptable bounds and decrements
when it does not. The full axis tune finishes when the sequence reaches `TUNE_COMPLETE`.
Gains are NOT saved until the pilot:
1. Engages testing mode (switch HIGH)
2. Verifies the new gains are acceptable
3. Disarms (triggers `save_tuning_gains()` if tuned gains were last active)

---

## AC_WPNav

Source: `D:\projects\ardupilot\libraries\AC_WPNav\`

### Spline Paths

AC_WPNav delegates to `AP_Math::SCurve` and `AP_Math::SplineCurve`. The header
`AC_WPNav.h` includes both `<AP_Math/SCurve.h>` and `<AP_Math/SplineCurve.h>`.

**SplineCurve** — Hermite cubic spline: each segment defined by position + tangent at
each waypoint. Tangents are computed from the line between the previous and next waypoints
(Catmull-Rom style), scaled to match the entry/exit speeds. This ensures C1 continuity
(continuous velocity direction) at corners.

**SCurve** — the primary path type for straight-line segments with jerk-limited
acceleration profiles. Uses 7-phase S-curve (acceleration, constant accel, deceleration
of accel, constant speed, deceleration, constant decel, final decel).

### Speed Profiling

Key parameters (configured in `AC_WPNav.cpp`):
- `WP_SPD_DEFAULT = 10.0 m/s` — cruise horizontal speed
- `WP_SPD_UP_DEFAULT = 2.5 m/s` — climb
- `WP_SPD_DOWN_DEFAULT = 1.5 m/s` — descent
- `WP_ACC_Z_DEFAULT = 1.0 m/s²` — vertical acceleration
- `WPNAV_ACCELERATION_MS = 2.5 m/s²` — horizontal acceleration
- `WP_JERK = 1.0 m/s³` (configurable, `WPNAV_JERK`) — rate of change of acceleration

SCurve uses jerk to shape the acceleration ramp. At each waypoint, entry and exit speeds
are negotiated using `get_corner_acceleration_mss()` (defaults to 2x horizontal accel) to
limit centripetal force in the turn.

"Fast waypoints" allow passing through a WP without stopping if geometry permits. The
position controller tracks the SCurve position/velocity/acceleration setpoint.

### Terrain Following

Terrain integration via `get_terrain_source()`:
1. **Rangefinder** — if `WPNAV_RFND_USE = 1` and rangefinder is healthy, use it
2. **Terrain database** — falls back to `AP_Terrain` SRTM database
3. **Unavailable** — fixed altitude mode

The terrain offset (`_rangefinder_terrain_u_m` or database lookup) is added to the
commanded altitude. A margin of `WP_TER_MARGIN = 10 m` (configurable) stops the vehicle
if it diverges further than this from the terrain-relative target altitude.

---

## AC_Avoidance (AC_Avoid)

Source: `D:\projects\ardupilot\libraries\AC_Avoidance\AC_Avoid.cpp`

### Role vs AP_Avoidance

**AC_Avoid** — reactive, position/velocity-level, for proximity sensors and fences.
Operates in body-relative velocity space. Used by copters/rovers during normal flight.

**AP_Avoidance** — strategic, ADSB-based, airplane-scale. Detects collision threats from
ADS-B transponder data, computes time-to-closest-approach, and initiates escape maneuvers.
Lives in `D:\projects\ardupilot\libraries\AP_Avoidance\`.

### Velocity Modification Algorithm (AC_Avoid)

The main entry point is `adjust_velocity(desired_vel_neu_cms, backing_up, kP, accel_cmss,
kP_z, accel_z_cmss, dt)`.

The approach:
1. For each active obstacle/fence, compute the required velocity limit using
   `limit_velocity_NE()` or `limit_velocity_NEU()`
2. `limit_velocity_NE()` projects the desired velocity onto the direction toward the
   obstacle, then caps that component so the vehicle can stop in the remaining distance

The stopping distance formula:
```
max_speed = sqrt(2 * accel * distance)
```
For non-linear (P controller) response (copters), the formula includes kP:
```
max_speed = distance * kP + sqrt(2 * accel * distance)  (approximately)
```
This is the `get_max_speed()` function.

Backup velocity: if the vehicle has already breached `AVOID_MARGIN`, a backup velocity
pushes it away at up to `AVOID_BACKUP_SPD` (default 0.75 m/s horizontal, 0.75 m/s
vertical). Four quadrant accumulation prevents backup velocities from cancelling across
orthogonal obstacle directions.

Acceleration limiting: `limit_accel_NEU_cm()` constrains the rate of velocity change from
avoidance to `AVOID_ACCEL_MAX` (default 3 m/s²), preventing sudden jerks.

### Fence Avoidance

`adjust_velocity_fence()` handles:
1. Circular fence — `adjust_velocity_circle_fence()`
2. Inclusion/exclusion polygons — `adjust_velocity_inclusion_and_exclusion_polygons()`
3. Inclusion/exclusion circles — `adjust_velocity_inclusion_circles()` /
   `adjust_velocity_exclusion_circles()`
4. Beacon fence — `adjust_velocity_beacon_fence()`
5. Altitude (ceiling/floor) — `adjust_velocity_z()`

All fence methods call `limit_velocity_NE()` with the distance to the fence boundary as
`limit_distance_cm`. The velocity component pointing toward the fence is reduced so that
stopping distance equals remaining fence clearance.

Behavior modes:
- `BEHAVIOR_SLIDE` (default for copters) — only the fence-ward component is limited; the
  vehicle slides along the fence boundary
- `BEHAVIOR_STOP` (default for rovers) — total velocity is reduced

### AP_Avoidance (ADSB-based)

Maintains an `Obstacle` array. Each entry has:
- `Location _location`, `Vector3f _velocity_ned_ms`
- `float closest_approach_ne_m` / `closest_approach_d_m`
- `float time_to_closest_approach_s` / `distance_to_closest_approach_ned_m`
- `MAV_COLLISION_THREAT_LEVEL threat_level`

Threat computation: predicts trajectories forward in time and computes minimum separation.
When threat level rises above threshold, triggers escape mode (plane-style avoidance
maneuver lasting `AP_AVOIDANCE_ESCAPE_TIME_SEC = 2` seconds). Recovery configurable:
remain in avoidance mode, resume previous mode, RTL, or resume-if-auto-else-loiter.

---

## AP_Proximity

Source: `D:\projects\ardupilot\libraries\AP_Proximity\`

### Supported Hardware

Backend drivers instantiated by type enum:
- `MAV (2)` — MAVLink `OBSTACLE_DISTANCE` messages
- `TRTOWER (3)` / `TRTOWEREVO (6)` — TeraRanger Tower (I2C)
- `RangeFinder (4)` — array of rangefinder instances treated as proximity
- `RPLidarA2 (5)` — Serial RPLidar A2
- `SF40C (7)` — LightWare SF40C (I2C/Serial)
- `SF45B (8)` — LightWare SF45B (Serial)
- `CYGBOT_D1 (13)` — Cygbot D1
- `DroneCAN (14)` — DroneCAN proximity sensor
- `Scripting (15)` — Lua scripting interface
- `LD06 (16)` — LD06 lidar
- `MR72 (17)` — MR72 radar
- Up to 5 instances (`AP_PROXIMITY_MAX_INSTANCES = 5`)

### 3D Boundary Storage (AP_Proximity_Boundary_3D)

The boundary represents obstacles in a 3D grid of faces:
- **8 sectors** horizontally at 45° each (`PROXIMITY_NUM_SECTORS = 8`)
- **5 layers** vertically at 30° pitch steps (`PROXIMITY_NUM_LAYERS = 5`)
- Middle layer (layer 2) is horizontal (0° pitch)
- Each face stores: distance, pitch angle, yaw angle, last-update timestamp

Constants:
```
PROXIMITY_SECTOR_WIDTH_DEG = 45°
PROXIMITY_PITCH_WIDTH_DEG = 30°
PROXIMITY_BOUNDARY_DIST_MIN = 0.6 m  (minimum plausible distance)
PROXIMITY_BOUNDARY_DIST_DEFAULT = 100 m  (default = "no obstacle")
PROXIMITY_FACE_RESET_MS = 1000  (face invalidated if not updated within 1s)
```

The `Face` struct is a (layer, sector) pair. `set_face_attributes(face, pitch, yaw,
distance, instance)` stores the measurement. `update_boundary()` propagates updated
sector distances to adjacent boundary points for conservative (shortest-distance) obstacle
envelope.

Filtering: each face has a `LowPassFilter` on distance to smooth sensor noise.
`PROXIMITY_FILT_RESET_TIME = 1000 ms` resets the filter if the face has gone stale.

### Integration with Avoidance

Proximity data flows into `AC_Avoid::adjust_velocity_proximity()`. The 3D boundary is
queried for the closest obstacle in each sector. For each sector where obstacle distance
is below `AVOID_MARGIN`, `limit_velocity_NEU()` is called with the obstacle direction
vector and remaining clearance distance. This bends the velocity vector away from that
sector.

The data format passed to AC_Avoid is a vector (direction + distance) per active sector.
All sectors are processed in a single loop; their velocity limit contributions accumulate
through the quadrant aggregation mechanism.

---

## AP_Terrain

Source: `D:\projects\ardupilot\libraries\AP_Terrain\`

### Terrain Database Architecture

ArduPilot stores SRTM-derived terrain data on microSD in a tile format:

**Grid structure:**
- GCS sends 4x4 grid blocks via MAVLink `TERRAIN_DATA`
- A disk `grid_block` is 2048 bytes containing an 8x7 arrangement of MAVLink 4x4 grids
- Total disk grid size: 32x28 points per block
- Block spacing: 24 east x 28 north (in grid units), overlapping by 1 on each side
- Altitude stored as `int16_t height[32][28]` in meters above MSL
- Default grid spacing: 100 m (configurable via `TERRAIN_SPACING`)
- File format version stored in `grid_block.version` and `version_minor`

**Memory cache:**
- 12 grid blocks in LRU cache (configurable `TERRAIN_CACHE_SZ`)
- Each block ~1800 bytes, 12 blocks = ~22 KB RAM
- Additional blocks fetched from SD card on demand
- Cache state machine: `INVALID -> DISKWAIT -> VALID / DIRTY`

**Disk I/O:** Union `grid_io_block` aligns `grid_block` to 2048-byte boundaries for
efficient block reads on SD cards.

### MAVLink Terrain Protocol

1. Vehicle sends `TERRAIN_REQUEST` to GCS listing which 4x4 tiles it needs (bitmask)
2. GCS responds with `TERRAIN_DATA` messages filling the tile
3. Vehicle stores tiles into the appropriate `grid_block`
4. Vehicle responds with `TERRAIN_REPORT` showing coverage status

### Height Lookup: `height_amsl()`

`bool AP_Terrain::height_amsl(const Location &loc, float &height, bool corrected)`

Procedure:
1. Convert `loc` lat/lon to `grid_info` (which 32x28 block and position within it)
2. Check LRU cache for the block; if miss, initiate SD card read
3. Bilinear interpolation from the four surrounding grid points
4. Optionally apply `corrected` adjustment (aligns terrain to GPS altitude at arm point,
   clamped by `TERRAIN_OFS_MAX = 30 m`)

Additional helpers:
- `height_above_terrain()` — height above terrain at current AHRS position
- `height_terrain_difference_home()` — terrain height delta vs home
- `lookahead(bearing, distance, climb_ratio)` — terrain rise ahead of vehicle on current
  track, for obstacle clearance in plane navigation

---

## AP_GyroFFT

Source: `D:\projects\ardupilot\libraries\AP_GyroFFT\AP_GyroFFT.cpp` and `.h`

### Algorithm and Architecture

AP_GyroFFT runs in a dedicated background thread (`apm_fft`, 1 KB stack, IO priority).
The main loop feeds gyro samples; the FFT thread consumes them.

**Window parameters (configurable):**
- Window size: power of 2, 32–512 (H7: up to 512, others: up to 256)
  Default: 64 (H7) or 32 (others)
- Overlap: 0–90%, default 75% (H7) or 50% (others)
- Samples per frame: `(1 - overlap) * window_size`, minimum 16
- Sampling rate: raw gyro rate (e.g., 1 kHz, 2 kHz) or fast-loop/N

**Frequency resolution:** `bin_resolution = sampling_rate / window_size`
Example: 1 kHz / 32 = 31.25 Hz/bin; 2 kHz / 64 = 31.25 Hz/bin.

**Bin layout:** bin 0 = DC+Nyquist combined; bin 1 onward = frequency content.
Bin N covers `(N-0.5)*res` to `(N+0.5)*res`.

### Sampling Modes

`FFT_SAMPLE_MODE`:
- 0: Raw gyro ring buffer (highest rate, e.g., 2 kHz on BMI088)
- 1–4: Fast-loop rate / N (downsampled with averaging, stored in `_downsampled_gyro_data`)

In mode 0, the INS provides a lock-free gyro ring buffer (`get_raw_gyro_window(axis)`).
In modes 1–4, `sample_gyros()` accumulates and averages N fast-loop samples, then pushes
to `_downsampled_gyro_data[axis]` (a `FloatBuffer`).

### FFT Execution

`run_cycle()` (FFT thread):
1. `start_analysis()` — checks sample count >= window_size; sets `_analysis_started`
2. Buffer overrun guard: if available > window_size + frame/2, drop oldest samples
3. `hal.dsp->fft_start(_state, gyro_buffer, _samples_per_frame)` — loads window
4. `hal.dsp->fft_analyse(_state, start_bin, end_bin, attenuation_cutoff)` — executes FFT
   and returns index of highest energy bin within `[FFT_MINHZ, FFT_MAXHZ]`
5. `update_ref_energy(bin_max)` — builds background noise reference during calibration
6. `calculate_noise()` — peak detection and tracking
7. Advance `_update_axis` (cycles through X, Y, Z — one axis per frame)

The HAL DSP layer (ChibiOS CMSIS-DSP on STM32) performs the actual FFT using
`arm_rfft_fast_f32`. Window function is Hann (applied before FFT in HAL layer). Optional
sliding-window averaging (`NUM_FRAMES`) averages multiple output frames.

### Peak Detection

`calculate_noise()` calls `calculate_tracking_peaks()` which:
1. Builds a `FrequencyData` object — for each of `MAX_TRACKED_PEAKS` peaks, computes
   weighted frequency and SNR from the DSP output
2. Applies a distance matrix (`find_distance_matrix`) between current peaks and history
   to prevent peak identity swaps between updates
3. `find_closest_peak()` matches each tracked peak to the nearest current peak
4. Updates filtered estimates via `MedianLowPassFilter3dFloat`:
   - Median of sliding window (3 samples) + low-pass filter
   - Separate filter instances for frequency, energy, and bandwidth per peak

**SNR threshold:** `FFT_SNR_DEFAULT = 25 dB` (pre-filter), `FFT_SNR_PFILT_DEFAULT = 10 dB`
(post-filter). Peaks below SNR are not reported as valid.

**Bandwidth:** computed as the width of the peak in Hz at `-FFT_ATT_REF dB` from peak
(default 15 dB attenuation). Stored and filtered separately, used for notch filter width.

**Health:** a peak is "healthy" if it was updated within `5 * frame_time_ms`. Health is
tracked per-axis per-peak.

### Multi-Peak Tracking

`MAX_TRACKED_PEAKS = 3` (CENTER, LOWER_SHOULDER, UPPER_SHOULDER). The three peaks
represent: the dominant motor noise peak, and the peaks just below and above it. This
allows tracking RPM harmonics simultaneously.

Up to 3 independent notch filter slots can be driven by these 3 peaks. The `_tracked_peaks`
count is set from the number of enabled harmonics across all harmonic notch filter
instances at init time.

### Harmonic Matching

`is_harmonic_of(harmonic, fundamental, mult, _fit)`:
```
fit = 100 * |harmonic - fundamental*mult| / harmonic
return (fit < _fit)  // default _fit = 10 (10%)
```

`calculate_notch_frequency(freqs, numpeaks, harmonic_fit, harmonics)`:
- Finds the lowest frequency in the peak list that is a harmonic of the highest-energy peak
- Returns that fundamental frequency and a bitmask of which harmonic multiples were matched

### Notch Filter Integration

`update_freq_hover(dt, throttle_out)` — runs at 100 Hz during stable hover:
- Tracks average throttle and FFT peak frequency using low-pass with `tau = 10 s`
- Stores `_freq_hover_hz` and `_throttle_ref` (both saved on disarm)
- These enable throttle-proportional notch: `f_notch = f_hover * sqrt(throttle / throttle_ref)`
  (because motor RPM ~ sqrt(thrust) ~ sqrt(throttle))

`start_notch_tune()` / `stop_notch_tune()` — manual notch tuning sequence:
1. Starts FFT averaging via `hal.dsp->fft_start_average()`
2. After hover, calls `fft_stop_average()` to get averaged spectrum
3. Calls `calculate_notch_frequency()` to find fundamental
4. Calls `_ins->setup_throttle_gyro_harmonic_notch()` to program the notch center and harmonics

Dynamic runtime notch: the `_global_state._center_freq_hz_filtered[]` values are polled
each update loop by the INS harmonic notch subsystem, which adjusts notch center
frequency in real time to track the current motor noise frequency.

**Post-filter mode** (`OPTIONS = 1`): FFT samples are taken after the filter bank instead
of before, allowing FFT to see residual noise that notch filters missed.

---

## AP_TECS

Source: `D:\projects\ardupilot\libraries\AP_TECS\`

### Total Energy Control System

TECS controls **throttle** and **pitch** simultaneously for fixed-wing aircraft, treating
altitude (potential energy, PE) and airspeed (kinetic energy, KE) as linked quantities
with a fixed total energy budget.

Called at 50 Hz (`update_50hz()`) for state estimation and 10–500 Hz
(`update_pitch_throttle()`) for control output.

### State Estimation

**Height and climb rate:** uses EKF vertical velocity if available; otherwise 3rd-order
complementary filter fusing baro altitude and vertical accelerometer:
```
omega = HGT_OMEGA (default 3.0 rad/s)
dd_height += (baro_err * omega^3) * dt
climb_rate += (dd_height + accel_z + baro_err * 3*omega^2) * dt
height += (climb_rate + baro_err * 3*omega) * dt
```

**Airspeed:** 2nd-order complementary filter fusing airspeed sensor and longitudinal
accelerometer:
```
aspdErr = (EAS * EAS2TAS) - TAS_state
integDTAS += aspdErr * spdCompFiltOmega^2 * dt
TAS_state += (integDTAS + vel_dot + aspdErr * spdCompFiltOmega * sqrt(2)) * dt
```

Vel_dot (longitudinal acceleration) computed as:
`(rotMat.c.x * gravity) + accel_x`, 5-point moving average filtered.

### Energy Error Calculation

`_update_energies()`:
```
_SPE_dem = hgt_dem * 9.81      // demanded potential energy (J/kg)
_SKE_dem = 0.5 * TAS_dem^2    // demanded kinetic energy (J/kg)
_SPE_est = height * 9.81      // estimated potential energy
_SKE_est = 0.5 * TAS_state^2  // estimated kinetic energy
_SPEdot  = climb_rate * 9.81  // PE rate
_SKEdot  = TAS_state * (vel_dot - vel_dot_lpf)  // KE rate (high-pass filtered)

_STE_error = (SPE_dem - SPE_est) + (SKE_dem - SKE_est)  // total energy error
```

### Throttle Demand

`_update_throttle_with_airspeed()`:
```
K_thr2STE = (STEdot_max - STEdot_min) / (THRmax - THRmin)
K_STE2Thr = 1 / (timeConst * K_thr2STE)

SPEdot_dem = (SPE_dem - SPE_est) / timeConst
STEdot_dem = SPEdot_dem + SKEdot_dem   // clamped to [STEdot_min, STEdot_max]
STEdot_error = STEdot_dem - SPEdot - SKEdot

// Feed-forward from demanded energy rate
ff_throttle = nomThr + STEdot_dem / K_thr2STE

// PD throttle
_throttle_dem = (STE_error + STEdot_error * THR_DAMP) * K_STE2Thr + ff_throttle

// Integrator
_integTHR_state += STE_error * i_gain * dt * K_STE2Thr

// Bank angle compensation (induced drag in turns)
STEdot_dem += RLL2THR * (1/cos^2(bank) - 1)

// Final
_throttle_dem = _throttle_dem + _integTHR_state
```

### Pitch Demand

`_update_pitch()` uses the **Specific Energy Balance (SEB)**:
```
SEB = SPE/SKE ratio — controls how pitch redistributes PE <-> KE
SEBdot_dem = SPEdot_dem - SKEdot_dem  (weighted by SPDWEIGHT)
```
`SPDWEIGHT = 0`: pitch controls altitude (SPE priority), throttle controls airspeed
`SPDWEIGHT = 2`: pitch controls airspeed (SKE priority), throttle controls altitude
`SPDWEIGHT = 1`: balanced (default)

The pitch integrator `_integSEBdot` trims out long-term SEB imbalance.

Underspeed protection: when `TAS_state < TASmin * 0.9` and throttle is at max, forces
`_TAS_dem = _TASmin` (airspeed priority override).

Landing mode: time constant and speed weighting change for approach. Flare detection
triggers sink-rate-based height demand instead of altitude tracking.

---

## AP_SmartRTL

Source: `D:\projects\ardupilot\libraries\AP_SmartRTL\`

### Path Storage

`_path[SMARTRTL_POINTS_MAX]` array (max 500 points, default 300, `Vector3f` each = 12
bytes, so max ~6 KB). Points added at 3+ Hz when moving more than `SMARTRTL_ACCURACY`
(default 2 m) from the last stored point.

### Dual Cleanup Algorithm

Background cleanup runs in the IO thread (`register_io_process`). Both algorithms are
**anytime algorithms** — they save state and yield after fixed time budgets:
- Simplification: 200 µs max per call (`SMARTRTL_SIMPLIFY_TIME_US`)
- Pruning: 200 µs max per call (`SMARTRTL_PRUNING_LOOP_TIME_US`)

**1. Simplification — Ramer-Douglas-Peucker:**

`detect_simplifications()` runs RDP recursively on the path. Uses a bitmask
(`_simplify.bitmask`, initially all-set) to mark which points to keep. Points are marked
for removal when their perpendicular distance to the line between outer endpoints is less
than `SMARTRTL_SIMPLIFY_EPSILON = accuracy * 0.5` (default 1.0 m). Stack-based
implementation avoids recursion depth issues.

Stack preallocated to `path_points_max * (2/3 + 1)` entries (overestimate of RDP's
maximum stack depth).

**2. Loop Pruning:**

`detect_loops()` — finds loops in the path: pairs of non-adjacent line segments that come
within `SMARTRTL_PRUNING_DELTA = accuracy * 0.99` (default ~1.98 m) of each other. When
found, marks everything between them for removal.

Detection: computes closest distance between all pairs of non-adjacent segments. Results
stored in `_prune.loops[]` buffer (up to `points_max * 0.25` loops).

`remove_points_by_loops(min_points)` — removes the detected loops starting from the
smallest loop (to maximize point savings).

**Cleanup triggers:**
- Routine: every 50 points added, or when fewer than 10 slots remain
- Thorough: called before entering SmartRTL flight mode; must complete before RTL begins
  (vehicle may pause for a few seconds)

---

## AP_Follow

Source: `D:\projects\ardupilot\libraries\AP_Follow\`

### Follow-Me Mechanism

Purely **MAVLink-based**. The vehicle listens for `GLOBAL_POSITION_INT` messages from the
target system ID (`FOLLOW_SYSID`, default 0 = any).

Each received message provides:
- Target position (lat/lon/alt)
- Target velocity (NED, cm/s)
- Target heading

`update_estimates()` uses dead-reckoning (kinematic integration) to project the target's
state forward between MAVLink update intervals. Input shaping smooths the velocity/accel
estimates.

Jitter correction (`AP_RTC::JitterCorrection`) compensates for variable-latency MAVLink
arrival times by estimating the transmission delay.

**Offset modes** (configured by `FOLLOW_OFS_TYPE`):
- NED offset: fixed displacement in meters from target in NED frame
- Relative offset: offset rotated to match target heading (follow behind, follow beside)

`get_ofs_pos_vel_accel_NED_m()` returns the desired follower position including offset,
the target velocity, and projected acceleration. This is fed directly into position
control for following.

Yaw behavior options: none, face lead vehicle, match lead vehicle heading, direction of
flight.

Auto-initialization of offsets: if `FOLLOW_OFS_X/Y/Z = 0` and
`FOLLOW_OPTIONS & MOUNT_FOLLOW_ON_ENTER`, the offset is set from current relative
position when entering follow mode.

---

## AP_Soaring

Source: `D:\projects\ardupilot\libraries\AP_Soaring\`

### Thermal Detection

The `Variometer` class estimates vertical air velocity (lift) by subtracting the
aircraft's expected sink rate from measured climb rate:

```
raw_lift = climb_rate - expected_sink_rate(airspeed, polar_params)
```

Polar model: `sink = K/airspeed + CD0*airspeed + B*airspeed^2`
where K, CD0, B are glider performance parameters.

Multiple time-constant filters separate fast measurement from slow thermal averaging:
- Audio vario output: 0.71 s
- Thermal trigger: 4.06 s LPF
- Thermal monitoring: 60 s LPF

Entry to thermal mode when `_trigger_filter > SOAR_VSPEED` (configurable threshold,
default ~0.7 m/s net lift).

### Extended Kalman Filter for Thermal Centering

`ExtendedKalmanFilter` (4-state): tracks thermal position and strength.

State vector: `[W (thermal strength m/s), R (radius m), x_offset, y_offset]`

Observation: measured lift rate.

Thermal model (Gaussian):
```
lift = W * exp(-r^2 / R^2)
```
where r is horizontal distance from aircraft to thermal center.

The EKF update drives the aircraft to circle in the direction that maximizes lift,
effectively centering on the thermal core. The `McCready(alt)` function computes
the optimal inter-thermal cruise airspeed given current altitude and expected thermal
strength.

Exit conditions (LoiterStatus enum):
- `ALT_TOO_HIGH / ALT_TOO_LOW` — altitude bounds
- `THERMAL_WEAK` — estimated lift below threshold
- `ALT_LOST` — barometric altitude loss > threshold
- `DRIFT_EXCEEDED` — thermal center has drifted too far from initial position
- `GOOD_TO_KEEP_LOITERING` — stay in thermal
- `EXIT_COMMANDED` — pilot/GCS command

---

## AP_Landing

Source: `D:\projects\ardupilot\libraries\AP_Landing\`

### Landing Types

- `TYPE_STANDARD_GLIDE_SLOPE = 0` — standard slope approach
- `TYPE_DEEPSTALL = 1` — stall descent for precision landing (separate `AP_Landing_Deepstall`)

### Glide Slope Approach

`setup_landing_glide_slope()` establishes a virtual slope from current position to the
runway threshold. The altitude target on approach is set proportionally along the slope
using `set_target_altitude_proportion_fn`.

Rangefinder integration: `adjust_landing_slope_for_rangefinder_bump()` corrects the
glide slope if the rangefinder detects terrain that differs from the planned approach
altitude, preventing premature touchdown or hold-short.

### Flare

Flare is triggered by altitude AGL crossing a threshold. In TECS flare mode:
- `_hgt_rate_dem` is set to `-land_sink_rate` (configurable sink rate, default ~0.5 m/s)
- Throttle goes to `THR_MIN` (or `THR_ZERO` depending on `ON_LANDING_FLARE_USE_THR_MIN`
  option bit)
- Height target is computed kinematically: `_hgt_dem += dt * _hgt_rate_dem`
- A "hold-off height" (`_flare_holdoff_hgt`) linearly blends between pre-flare tracking
  and pure sink-rate control

Flare sink rate increases with distance past the landing waypoint:
```
land_sink_rate_adj = land_sink + land_sink_rate_change * distance_beyond_land_wp
```

### Ground Detection

`verify_land()` checks:
1. Low altitude (rangefinder or barometric)
2. Low airspeed
3. Low sink rate
4. Vehicle has been armed for more than `last_flying_ms` threshold without detection of
   flight

When ground contact is confirmed, `disarm_if_autoland_complete_fn` is called.

---

## AC_Autorotation

Source: `D:\projects\ardupilot\libraries\AC_Autorotation\`

### Autorotation Phases

**Entry phase** (`init_entry()` / `run_entry()`):
- Duration: 2000 ms (`entry_time_ms`)
- Gradually reduces collective to autorotation target, with cutoff frequency configurable
  via `AROT_COL_ENTRY_FILT` parameter
- Collective commanded via LPF: `col_trim_lpf` at `_param_col_entry_cutoff_freq` Hz
- Forward speed controller is initialized

**Glide phase** (`init_glide()` / `run_glide()`):
- Head speed controller: P controller (`_p_hs`) drives collective to maintain normalized
  head speed `_target_head_speed`
- Head speed measured from RPM sensor (instance `_param_rpm_instance`), normalized by
  `_param_head_speed_set_point`
- Collective output: `_p_term_hs + _ff_term_hs` where ff tracks collective trim
- Forward speed controller: PID (`_fwd_speed_pid`) drives desired forward acceleration to
  reach `_param_target_speed_ms`; feedforward `_param_fwd_k_ff` added for wind

**Landed detection:**
- `_landed_reason.min_speed` — horizontal speed below threshold
- `_landed_reason.land_col` — collective at minimum
- `_landed_reason.is_still` — vehicle not accelerating

### Head Speed Control

The head speed controller normalizes RPM by the setpoint. A normalized value of 1.0 means
head speed equals the set point. The P gain drives collective up when head speed is low
(rotor slowing) and down when high (rotor overspeeding).

Entry phase uses a ramp: `_hs_accel` accelerates the target head speed from 0 toward 1.0
to avoid sudden collective changes.

---

## AP_WindVane

Source: `D:\projects\ardupilot\libraries\AP_WindVane\`

### Sensor Types

`WNDVN_TYPE`:
- 0: None
- 1: Heading when armed (apparent wind = vehicle heading at arm time)
- 2: RC input offset from heading
- 3: Analog voltage on pin — linear mapping `[DIR_V_MIN, DIR_V_MAX]` to `[0, 360°]`
- 4: NMEA serial sentence
- 10/11: SITL (true or apparent)

Speed sensors (separate from direction):
- ModernDevice rev P analog sensor (voltage pins for speed + temperature compensation)
- RPM-based wind speed
- NMEA
- Airspeed sensor (as apparent wind proxy)

### Apparent vs True Wind

Apparent wind = wind as seen by the moving vehicle (what sensors measure directly).
True wind = actual wind direction and speed relative to ground.

Conversion:
```
V_apparent = V_wind_true - V_vehicle
```
In vector form:
```
true_wind_vec = apparent_wind_vec + vehicle_velocity_vec
```

ArduPilot computes true wind from apparent wind + AHRS ground speed vector. Result stored
in `_direction_true` and `_speed_true`.

Filtering: `DIR_FILT` (default 0.5 Hz LPF) smooths direction output. Separate filters
for speed.

Calibration: rotating the vehicle 360° while recording min/max analog voltage maps the
full voltage range to compass directions.

---

## AP_Quicktune

Source: `D:\projects\ardupilot\libraries\AP_Quicktune\`

### Algorithm Summary

AP_Quicktune is a simplified, faster alternative to AC_AutoTune. Original Lua script
ported to C++. Activated via RC aux switch (3-position: LOW=off, MID=test with original
gains, HIGH=tune).

**Sequence:** Roll -> Pitch -> Yaw, per axis:
1. D gain: doubled every `DOUBLE_TIME` seconds (default 10 s) until oscillation detected
2. P gain: doubled every `DOUBLE_TIME` seconds until oscillation detected
3. I gain: set from P using ratio `RP_PI_RATIO` (roll/pitch, default 1.0) or `Y_PI_RATIO`
   (yaw, default 10.0)

**Oscillation detection:** SMAX (slew rate limit) of the rate PID output exceeds
`OSC_SMAX` threshold (default 4). When triggered:
- Gain is reduced by `GAIN_MARGIN` percent (default 60% = gain * 0.4)
- Axis marked done, advance to next

**Auto-filter:** when `AUTO_FILTER = 1`, sets FLTD and FLTT to `INS_GYRO_FILTER * 0.5`,
FLTE to 2.0 Hz (yaw max).

**Auto-save:** after `AUTO_SAVE` seconds (default 0 = disabled), saves gains automatically.

Parameters tuned: P, I, D, SMAX, FLTT, FLTD, FLTE, FF — for roll, pitch, yaw (24 total).

---

## AP_WheelEncoder

Source: `D:\projects\ardupilot\libraries\AP_WheelEncoder\`

### Architecture

Dual-encoder support (up to 2 instances, `WHEELENCODER_MAX_INSTANCES = 2`). Each encoder
maintains a `WheelEncoder_State` struct:
- `distance_count` — cumulative encoder count (forward positive, backward negative)
- `distance` — distance in meters (count / CPR * 2π * radius)
- `dist_count_change` + `dt_ms` — for rate calculation

Default encoder resolution: `WHEELENCODER_CPR_DEFAULT = 3200` counts/rev
Default wheel radius: `WHEELENCODER_RADIUS_DEFAULT = 0.05 m`

### Backend Drivers

**Quadrature encoder** (`WheelEncoder_Quadrature`): reads two digital input pins (A and B
channels). Phase relationship between A and B determines direction. Edge interrupts or
polling on both channels.

**SITL Quadrature**: simulation backend.

### AP_WheelRateControl

Separate `AP_WheelRateControl` class provides PID control of wheel rate (speed) using
encoder feedback. Used by Rover for precise speed control. Each encoder drives its own
rate PID controller. Output is motor throttle command.

---

## Meridian Port Priority

### Critical / Complex (implement first)

| Library | Complexity | Notes |
|---------|------------|-------|
| AP_GyroFFT | Very High | Dedicated thread, HAL DSP, multi-peak tracking, harmonic matching |
| AC_AutoTune | High | Complex state machine, two vehicle types, convergence criteria |
| AP_TECS | High | Core fixed-wing control, energy math, complementary filters |
| AC_WPNav | Medium-High | S-curve path planning, terrain following, jerk limits |
| AP_SmartRTL | Medium | Dual anytime cleanup algorithm, path storage |

### Medium Priority

| Library | Complexity | Notes |
|---------|------------|-------|
| AC_Avoidance | Medium | Velocity bending, fence avoidance, backup velocity |
| AP_Proximity | Medium | 3D boundary grid, multi-backend, aging/filtering |
| AP_Terrain | Medium | Tile-based SD cache, MAVLink protocol, bilinear interp |
| AP_Soaring | Medium | EKF, thermal model, variometer filter chain |
| AP_Landing | Medium | Flare logic, slope tracking, rangefinder correction |
| AC_Autorotation | Medium | Three-phase heli controller, head speed normalization |

### Lower Priority

| Library | Complexity | Notes |
|---------|------------|-------|
| AP_Avoidance | Low-Med | ADSB-only, airplane use case |
| AP_Follow | Low-Med | MAVLink dead-reckoning, mostly position control |
| AP_WindVane | Low | Sensor drivers + apparent/true conversion |
| AP_Quicktune | Low | Simple gain-doubling algorithm |
| AP_WheelEncoder | Low | Quadrature pulse counting |

---

## Key Rust Porting Notes

### AP_GyroFFT
- Requires a separate async task (tokio task or bare RTOS thread)
- HAL DSP layer wraps CMSIS-DSP; in Rust, use `microfft` or `realfft` crate
- Semaphore between main and FFT thread protects `_global_state`; use `Arc<Mutex<>>` or
  `Arc<RwLock<>>` with a try-lock pattern to avoid stalling the FFT thread
- The `MedianLowPassFilter3dFloat` is a 3-sample sliding median + single-pole IIR;
  straightforward to implement without external crates
- `FrequencyPeak` enum maps to a 3-element array (CENTER=0, LOWER_SHOULDER=1, UPPER_SHOULDER=2)

### AC_AutoTune
- State machine: best modeled as a Rust enum with data. `TuneMode`, `Step`, `TuneType` all
  map cleanly to Rust enums
- The multirotor and heli variants differ significantly; use a trait `AutoTuneBackend`
  mirroring the C++ pure-virtual interface
- Gain backup/restore requires storing 30+ floats per axis; a struct per-axis is cleaner
- `AUTOTUNE_SUCCESS_COUNT = 4` hysteresis counter prevents noisy convergence

### AP_TECS
- Energy calculations are pure floating-point math; no HAL dependencies
- Two complementary filters (height + airspeed) must run at 50 Hz independent of control
  loop; split `update_50hz()` from `update_pitch_throttle()`
- Watch for the `_flags` struct (reset, underspeed, badDescent, is_gliding, is_doing_auto_land) —
  each flag changes control behavior significantly
- `timeConstant()` has a landing-mode variant; a method rather than field

### AC_WPNav
- SCurve and SplineCurve math is in `AP_Math`; need Rust equivalents for both
- Terrain integration is optional; wrap in `Option<TerrainSource>` pattern
- Speed/accel limits change mid-flight; the position controller must accept updated setpoints

### AP_SmartRTL
- Path array can be `Vec<Vector3<f32>>` with capacity pre-allocated
- Background cleanup needs a Mutex-protected Vec; use `TryLock` to avoid blocking the IO
  thread on the flight controller path
- RDP simplification needs a stack (iterative); max stack depth = `path_len * 2/3`
- Loop pruning: O(N²) segment-pair distance check; for 500 points this is 125K checks at 200 µs budget

### AC_Avoidance
- `limit_velocity_NE()` is the core function; port this first
- Quadrant accumulation of backup velocities is the key insight to preserve
- `limit_accel_NEU_cm()` requires remembering previous avoidance velocity; stateful

### AP_Terrain
- Tile cache: `HashMap<TileKey, GridBlock>` with LRU eviction
- MAVLink terrain request/response protocol must be accurate or GCS will not fill tiles
- Bilinear interpolation from 4 corner heights is straightforward
- SD card I/O: async read via `tokio::fs` or block-device interface

### AP_Proximity
- `Boundary3D` maps to a `[[Option<FaceData>; 8]; 5]` array (layers x sectors)
- Aging: timestamp per face, reset if > 1 second old
- Multi-instance: `Vec<Box<dyn ProximityBackend>>` pattern
