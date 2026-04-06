# Wave 3 Audit: Core Utilities
## AP_Math, Filter, AP_Logger, AP_Param, AP_Scripting, AP_DAL, AP_Scheduler, AP_Common, AP_FlashStorage, AP_Filesystem, AP_AccelCal

**Audited:** 2026-04-02  
**Source:** `D:\projects\ardupilot\libraries\`  
**Files read:** 35+

---

## 1. AP_Math

### 1.1 Vector/Matrix Types

**Vector2\<T\>** (`vector2.h`) — templated 2D vector, instantiated as:
- `Vector2i` (int16_t), `Vector2ui` (uint16_t), `Vector2l` (int32_t), `Vector2ul` (uint32_t), `Vector2f` (float), `Vector2d` (double)

Fields: `x`, `y`

Methods:
- Arithmetic operators: `+`, `-`, `*`, `/`, `+=`, `-=`, `*=`, `/=` (uniform scaling and vector ops)
- `dot(v)` / `operator*(v)` — dot product
- `operator%(v)` — 2D cross product (scalar result)
- `angle(v2)` — angle between vectors (0 to PI)
- `angle()` — angle of this vector from (1,0), range -PI to PI
- `is_nan()`, `is_inf()`, `is_zero()`
- `length_squared()`, `length()`
- `limit_length(max_length)` — clamps length, returns bool if limited
- `normalize()`, `normalized()`
- `reflect(n)` — reflects about normal n
- `project(v)`, `projected(v)` — projection onto v
- `offset_bearing(bearing_deg, distance)` — adjusts position by bearing+distance
- `rotate(angle_rad)` — rotates in-place
- `tofloat()`, `todouble()` — precision conversion
- `zero()`
- Static: `perpendicular(pos_delta, v1)`
- Static: `closest_point(p, v, w)` — closest point on segment (v,w) to p
- Static: `closest_point(p, w)` — same but segment (0,w)
- Static: `closest_distance_between_line_and_point_squared(w1, w2, p)`
- Static: `closest_distance_between_line_and_point(w1, w2, p)`
- Static: `closest_distance_between_lines_squared(a1, a2, b1, b2)`
- Static: `closest_distance_between_radial_and_point_squared(w, p)`
- Static: `closest_distance_between_radial_and_point(w, p)`
- Static: `segment_intersection(seg1_start, seg1_end, seg2_start, seg2_end, intersection)` — bool
- Static: `circle_segment_intersection(seg_start, seg_end, center, radius, intersection)` — bool
- Static: `point_on_segment(point, seg_start, seg_end)` — bool

**Vector3\<T\>** (`vector3.h`) — templated 3D vector, instantiated as:
- `Vector3i`, `Vector3ui`, `Vector3l`, `Vector3ul`, `Vector3f`, `Vector3d`

Fields: `x`, `y`, `z`

Methods (all arithmetic operators analogous to Vector2, plus):
- `operator*(v)` / `dot(v)` — dot product
- `operator%(v)` / `cross(v)` — cross product
- `row_times_mat(m)` — multiply row vector by Matrix3
- `mul_rowcol(v)` — multiply column by row to get Matrix3
- `scale(v)` — uniform scale returning new vector
- `angle(v2)` — angle between vectors
- `is_nan()`, `is_inf()`, `is_zero()`
- `rotate(Rotation)`, `rotate_inverse(Rotation)` — apply standard rotation enum
- `rotate_xy(rotation_rad)` — rotates XY plane only, Z untouched
- `xy()` — returns reference to XY as Vector2 (zero-copy cast)
- `length_squared()`, `length()`
- `limit_length_xy(max_length)` — limit XY component only
- `normalize()`, `normalized()`
- `zero()`
- `reflect(n)`, `project(v)`, `projected(v)`
- `distance_squared(v)` — squared distance between tips
- `distance_to_segment(seg_start, seg_end)` — distance to a line segment
- `offset_bearing(bearing_deg, pitch_deg, distance)` — 3D offset
- `tofloat()`, `todouble()`
- `rfu_to_frd()` — converts Right-Front-Up to Front-Right-Down (ENU to NED)
- Static: `perpendicular(p1, v1)` — component of p1 perpendicular to v1
- Static: `closest_distance_between_line_and_point(w1, w2, p)`
- Static: `point_on_line_closest_to_other_point(w1, w2, p)`
- Static: `segment_to_segment_closest_point(seg1_start, seg1_end, seg2_start, seg2_end, closest_point)`
- Static: `segment_plane_intersect(seg_start, seg_end, plane_normal, plane_point)` — bool

**EKF type aliases** (in `AP_Math.h`):
- When `HAL_WITH_EKF_DOUBLE` is set: `Vector2F = Vector2<double>`, `Vector3F = Vector3<double>`, `Matrix3F = Matrix3<double>`, `QuaternionF = QuaternionD`
- Otherwise all are float variants

**Matrix3\<T\>** (`matrix3.h`) — 3x3 matrix in row-normal form, instantiated as `Matrix3i`, `Matrix3ui`, `Matrix3l`, `Matrix3ul`, `Matrix3f`, `Matrix3d`

Rows: `a`, `b`, `c` (each a Vector3\<T\>)

Methods:
- Arithmetic: `+`, `-`, `*` (scalar), `/` (scalar), `+=`, `-=`, `*=`, `/=`
- `operator*(Matrix3)` — matrix multiplication
- `operator*(Vector3)` — matrix-vector multiply
- `mul_transpose(Vector3)` — multiply transpose by vector
- `mulXY(Vector3)` — returns Vector2 (XY components of result)
- `operator[](i)` — row access
- `colx()`, `coly()`, `colz()` — extract column vectors
- `transposed()`, `transpose()` — in-place and copy variants
- `det()` — determinant
- `inverse(inv)` — returns bool, fills inv if invertible
- `invert()` — in-place inversion, returns bool
- `zero()`, `identity()`
- `is_nan()`
- `from_euler(roll, pitch, yaw)` — 321 Euler → rotation matrix
- `to_euler(roll, pitch, yaw)` — rotation matrix → 321 Euler angles
- `from_rotation(Rotation)` — populate from rotation enum
- `to_euler312()` — 312 convention (yaw-roll-pitch) → Vector3
- `from_euler312(roll, pitch, yaw)` — 312 convention
- `rotate(gyro_vec)` — apply incremental rotation from gyro vector
- `from_axis_angle(v, theta)` — Rodrigues formula
- `normalize()` — re-orthogonalize

**MatrixN\<T,N\>** (`matrixN.h`) — NxN matrix for EKF covariance:
- `mult(A, B)` — outer product of VectorN to give MatrixN
- `operator-=`, `operator+=`
- `force_symmetry()` — ensures P = (P + P^T) / 2

**VectorN\<T,N\>** (`vectorN.h`) — N-dimensional vector, used by EKF.

**QuaternionT\<T\>** (`quaternion.h`) — quaternion with fields `q1, q2, q3, q4` (w, x, y, z), instantiated as `Quaternion` (float) and `QuaternionD` (double).

Methods:
- `is_nan()`
- `rotation_matrix(Matrix3f&)`, `rotation_matrix(Matrix3d&)` — quaternion → rotation matrix
- `from_rotation_matrix(Matrix3<T>)` — rotation matrix → quaternion
- `from_rotation(Rotation)` — from enum
- `rotate(Rotation)` — apply enum rotation in-place
- `earth_to_body(Vector3)` — rotate vector from earth to body frame
- `from_euler(roll, pitch, yaw)`, `from_euler(Vector3)` — 321 convention
- `from_vector312(roll, pitch, yaw)` — 312 (yaw-roll-pitch) convention
- `to_axis_angle(Vector3)`, `from_axis_angle(Vector3)` — axis-angle representation
- `from_axis_angle(axis, theta)` — axis must be unit length
- `rotate(v)` — apply rotation vector
- `from_axis_angle_fast(v)`, `from_axis_angle_fast(axis, theta)` — small angles only (<0.17 rad)
- `from_angular_velocity(angular_velocity, time_delta)` — integrate angular velocity
- `rotate_fast(v)` — small angle version
- `get_euler_roll()`, `get_euler_pitch()`, `get_euler_yaw()` — individual angles
- `to_euler(float&, float&, float&)`, `to_euler(Vector3f&)` — 321 decomposition
- `to_vector312()` — 312 decomposition
- `length_squared()`, `length()`, `normalize()`
- `is_zero()`, `zero()`, `is_unit_length()`, `initialise()`
- `inverse()`, `invert()`
- `operator*(Quaternion)` — quaternion multiplication
- `operator*(Vector3)` — rotate vector by quaternion
- `operator*=(Quaternion)`, `operator/(Quaternion)`
- `angular_difference(v)` — quaternion representing difference
- `roll_pitch_difference(v)` — scalar earth-frame roll-pitch error (radians)
- `todouble()`, `tofloat()`

### 1.2 Location Math

Location math lives in two places: `AP_Math/location.h` (low-level primitives) and `AP_Common/Location.h` (the full Location class with altitude frames).

**`AP_Math/location.h` primitives:**
- `get_horizontal_distance(origin, destination)` — template, returns length of difference vector (no unit conversion)
- `get_bearing_rad(origin, destination)` — bearing in radians, 0..2PI, uses `atan2f`
- `get_bearing_cd(origin, destination)` — bearing in centi-degrees
- `wgsllh2ecef(llh, ecef)` — WGS84 lat/lon/height → ECEF XYZ (double precision)
- `wgsecef2llh(ecef, llh)` — ECEF XYZ → WGS84 lat/lon/height (double precision)
- `check_lat(float)`, `check_lat(int32_t)` — validates |lat| <= 90 or 90e7
- `check_lng(float)`, `check_lng(int32_t)` — validates |lng| <= 180 or 180e7
- `check_latlng(...)` — combined validation

**Conversion constants** (in `definitions.h`):
- `LATLON_TO_M = 0.011131884502145034` — converts 1e-7 degree to meters (at equator)
- `LATLON_TO_CM = 1.1131884502145034`
- `RADIUS_OF_EARTH = 6378100` meters (approximation)
- WGS84 full precision: `WGS84_A = 6378137.0`, `WGS84_IF = 298.257223563`, `WGS84_F = 1/WGS84_IF`, `WGS84_B = WGS84_A*(1-WGS84_F)`, `WGS84_E = sqrt(2*F - F^2)`

**Lua-exposed Location methods** (from `bindings.desc`):
- `get_distance(loc)` — horizontal distance in meters
- `offset(north_m, east_m)` — move by NE offset
- `offset_bearing(bearing_deg, distance_m)` — offset by bearing+distance
- `offset_bearing_and_pitch(bearing_deg, pitch_deg, distance_m)` — 3D offset
- `get_vector_from_origin_NEU_m(Vector3f)` — NED vector from EKF origin in meters (bool)
- `get_vector_from_origin_NEU_cm(Vector3f)` — same in cm (bool)
- `get_bearing(loc)` — bearing in radians
- `get_distance_NED(loc)` — 3D NED vector
- `get_distance_NE(loc)` — 2D NE vector
- `get_alt_frame()` — returns AltFrame enum
- `change_alt_frame(AltFrame)` — converts altitude frame (bool)
- `set_alt_m(alt, AltFrame)` — set altitude in meters

### 1.3 Rotation Enum

Defined in `rotations.h`, type `Rotation : uint8_t`. **44 standard entries** (0–43), plus custom entries at 100–102.

Complete list:
| Value | Name |
|-------|------|
| 0 | ROTATION_NONE |
| 1 | ROTATION_YAW_45 |
| 2 | ROTATION_YAW_90 |
| 3 | ROTATION_YAW_135 |
| 4 | ROTATION_YAW_180 |
| 5 | ROTATION_YAW_225 |
| 6 | ROTATION_YAW_270 |
| 7 | ROTATION_YAW_315 |
| 8 | ROTATION_ROLL_180 |
| 9 | ROTATION_ROLL_180_YAW_45 |
| 10 | ROTATION_ROLL_180_YAW_90 |
| 11 | ROTATION_ROLL_180_YAW_135 |
| 12 | ROTATION_PITCH_180 |
| 13 | ROTATION_ROLL_180_YAW_225 |
| 14 | ROTATION_ROLL_180_YAW_270 |
| 15 | ROTATION_ROLL_180_YAW_315 |
| 16 | ROTATION_ROLL_90 |
| 17 | ROTATION_ROLL_90_YAW_45 |
| 18 | ROTATION_ROLL_90_YAW_90 |
| 19 | ROTATION_ROLL_90_YAW_135 |
| 20 | ROTATION_ROLL_270 |
| 21 | ROTATION_ROLL_270_YAW_45 |
| 22 | ROTATION_ROLL_270_YAW_90 |
| 23 | ROTATION_ROLL_270_YAW_135 |
| 24 | ROTATION_PITCH_90 |
| 25 | ROTATION_PITCH_270 |
| 26 | ROTATION_PITCH_180_YAW_90 |
| 27 | ROTATION_PITCH_180_YAW_270 |
| 28 | ROTATION_ROLL_90_PITCH_90 |
| 29 | ROTATION_ROLL_180_PITCH_90 |
| 30 | ROTATION_ROLL_270_PITCH_90 |
| 31 | ROTATION_ROLL_90_PITCH_180 |
| 32 | ROTATION_ROLL_270_PITCH_180 |
| 33 | ROTATION_ROLL_90_PITCH_270 |
| 34 | ROTATION_ROLL_180_PITCH_270 |
| 35 | ROTATION_ROLL_270_PITCH_270 |
| 36 | ROTATION_ROLL_90_PITCH_180_YAW_90 |
| 37 | ROTATION_ROLL_90_YAW_270 |
| 38 | ROTATION_ROLL_90_PITCH_68_YAW_293 (non-orthogonal) |
| 39 | ROTATION_PITCH_315 |
| 40 | ROTATION_ROLL_90_PITCH_315 |
| 41 | ROTATION_PITCH_7 |
| 42 | ROTATION_ROLL_45 |
| 43 | ROTATION_ROLL_315 |
| 44 | ROTATION_MAX |
| 100 | ROTATION_CUSTOM_OLD |
| 101 | ROTATION_CUSTOM_1 |
| 102 | ROTATION_CUSTOM_2 |

**Important note:** values are stored to EEPROM — do not renumber. The Rotation enum maps directly to `MAV_SENSOR_ORIENTATION` in the MAVLink spec.

**How sensor rotations work:** External sensors (compass, IMU, rangefinder, etc.) each store a `Rotation` param. When reading sensor data, `Vector3::rotate(Rotation)` or `Vector3::rotate_inverse(Rotation)` is called to transform the measurement into body frame. The matrix for each enum value is computed at call time (not stored).

### 1.4 Control Math

All in `AP_Math.h` and `control.h`.

**Basic numeric utilities (`AP_Math.h`):**
- `safe_asin(v)` — clamps input to [-1,1], returns 0 for NaN
- `safe_sqrt(v)` — returns 0 for negative or NaN input (IEEE-754 `isgreaterequal` check)
- `constrain_float(amt, low, high)` — macro wrapping `constrain_value_line` with `__AP_LINE__` for NaN-reporting; same for `constrain_int8/16/32/64`, `constrain_uint8/16/32/64`, `constrain_double`
- `wrap_180(angle)` — wraps to -180..180 (degrees, deziDeg, or centiDeg depending on type)
- `wrap_180_cd(angle)` — centi-degree version
- `wrap_360(float/double/int)` — wraps to 0..360
- `wrap_360_cd(...)` — centi-degree version
- `wrap_2PI(float/double)` — wraps to 0..2PI
- `wrap_PI(T)` — wraps to -PI..PI
- `is_equal(v1, v2)` — epsilon comparison (template, float uses `fabsf`, double uses `fabs`)
- `is_zero(fVal)`, `is_positive(fVal)`, `is_negative(fVal)` — FLT_EPSILON threshold
- `sq(val)` — square (variadic template for dot product)
- `norm(first, second, ...)` — Euclidean norm (variadic)
- `linear_interpolate(out_low, out_high, in_val, in_low, in_high)` — linear remap
- `expo_curve(alpha, input)` — cubic expo in [-1,1], alpha in [0,1]
- `throttle_curve(thr_mid, alpha, thr_in)` — throttle shaping
- `calc_lowpass_alpha_dt(dt, cutoff_freq)` — compute IIR alpha from dt and cutoff
- `get_vel_correction_for_sensor_offset(sensor_offset_bf, rot_ef_to_bf, angular_rate)` — lever arm velocity correction
- `fixedwing_turn_rate(bank_angle_deg, airspeed)` — coordinated turn rate in deg/s
- `radians(deg)`, `degrees(rad)`, `cd_to_rad(cdeg)`, `rad_to_cd(rad)` — unit conversion
- `hz_to_nsec`, `nsec_to_hz`, `usec_to_nsec`, `nsec_to_usec`, `hz_to_usec`, `usec_to_hz` — time conversion
- `float_to_int16/uint16/int32/uint32`, `double_to_uint32/int32` — safe narrowing conversions
- `float_to_int32_le`, `int32_to_float_le`, `uint64_to_double_le` — type-punning-safe bit conversions
- `get_twos_complement(raw, length)` — sign-extend raw bits
- `fixed2float(input, fractional_bits=8)`, `float2fixed(input, fractional_bits=8)` — fixed-point conversion
- `degF_to_Kelvin(temp_f)` — temperature conversion
- `rotation_equal(r1, r2)` — handles non-orthogonal custom rotations
- `rand_float()`, `rand_vec3f()`, `get_random16()` — PRNG utilities

**Position/velocity/acceleration shaping (`control.h`):**
- `update_vel_accel(vel, accel, dt, limit, vel_error)` — 1D kinematic integrator with directional limiting
- `update_pos_vel_accel(pos, vel, accel, dt, limit, pos_error, vel_error)` — 1D position integrator
- `update_vel_accel_xy(vel, accel, dt, limit, vel_error)` — 2D version
- `update_pos_vel_accel_xy(pos, vel, accel, dt, limit, pos_error, vel_error)` — 2D position integrator
- `shape_accel(accel_desired, accel, jerk_max, dt)` — jerk-limited scalar accel shaping
- `shape_accel_xy(accel_desired, accel, jerk_max, dt)` — jerk-limited 2D/3D accel shaping
- `shape_vel_accel(vel_desired, accel_desired, vel, accel, accel_min, accel_max, jerk_max, dt, limit_total_accel)` — velocity-to-acceleration with sqrt controller
- `shape_vel_accel_xy(...)` — 2D version
- `shape_pos_vel_accel(pos_desired, vel_desired, accel_desired, pos, vel, accel, vel_min, vel_max, accel_min, accel_max, jerk_max, dt, limit_total)` — full 1D kinematic shaper
- `shape_pos_vel_accel_xy(...)` — 2D version
- `shape_angle_vel_accel(...)` — angular version for attitude control
- `limit_accel_xy(vel, accel, accel_max)` — lateral-priority acceleration limiter
- `limit_accel_corner_xy(vel, accel, accel_max)` — braking-priority variant
- `sqrt_controller(error, p, second_ord_lim, dt)` — piecewise P+sqrt controller
- `sqrt_controller(Vector2f, p, second_ord_lim, dt)` — 2D version
- `inv_sqrt_controller(output, p, D_max)` — invert sqrt_controller
- `sqrt_controller_accel(error, rate_cmd, rate_state, p, second_ord_lim)` — chain-rule derivative of sqrt_controller output
- `stopping_distance(velocity, p, accel_max)` — distance needed to stop
- `kinematic_limit(direction, max_xy, max_z_neg, max_z_pos)` — max speed in direction given independent axis limits
- `input_expo(input, expo)` — exponential curve for stick shaping [0..0.95]
- `angle_rad_to_accel_mss(angle_rad)`, `angle_deg_to_accel_mss(angle_deg)` — lean angle → accel conversion (g*tan(θ))
- `accel_mss_to_angle_rad/deg(accel_mss)` — accel → lean angle (atan(a/g))
- `rc_input_to_roll_pitch_rad(roll_in_norm, pitch_in_norm, angle_max_rad, angle_limit_rad, roll_out, pitch_out)` — stick → Euler angles

**Position type:** `postype_t` is `double` when `HAL_WITH_POSTYPE_DOUBLE`, else `float`; `Vector2p` and `Vector3p` alias accordingly. This matters for high-precision operations.

### 1.5 Polygon Math

In `polygon.h` / `polygon.cpp`.

Algorithm: Ray-casting (crossing-number test), O(n) per query. Handles both float and integer template types. Uses sign-based early-exit to avoid 64-bit multiply when possible.

- `Polygon_outside(P, V[], n)` — bool: returns true if P is outside polygon defined by n vertices V[]. Handles closed polygons (first==last). Template on T.
- `Polygon_complete(V[], n)` — bool: checks if first vertex == last vertex
- `Polygon_intersects(V, N, p1, p2, intersection)` — bool: does line p1-p2 intersect the polygon? Returns nearest intersection to p1
- `Polygon_closest_distance_line(V, N, p1, p2)` — float: closest distance between line segment p1-p2 and any polygon edge; negative means line crosses into polygon
- `Polygon_closest_distance_point(V, N, p, closest_segment)` — bool: closest distance from point p to any polygon edge, fills closest_segment with the closest point

**Note:** All polygon math is in 2D Cartesian coordinates. Does not account for Earth curvature. Suitable for fence boundaries at short range.

### 1.6 Spline Implementation

Two spline systems:

**`spline5.h` / `spline5.cpp`** — simple 5-point cubic interpolator:
```
void splinterp5(const float x[5], float out[4][4]);
```
Takes 5 control point values, outputs 4 cubic polynomial coefficients per segment (4x4 matrix). Low-level utility.

**`SplineCurve.h` / `SplineCurve.cpp`** — cubic Hermite spline for waypoint following (used by AC_WPNav):
- `set_speed_accel(speed_xy, speed_up, speed_down, accel_xy, accel_z)` — configure kinematic limits
- `set_origin_and_destination(origin, dest, origin_vel, dest_vel)` — define spline by endpoints and endpoint velocities; internally calls `update_solution()` which stores the 4-element `_hermite_solution` array
- `advance_target_along_track(dt, target_pos, target_vel)` — moves spline parameter forward; calls `calc_dt_speed_max()` and `calc_target_pos_vel()`
- `reached_destination()` — bool
- `get_destination_vel()` — Vector3f
- `get_origin_speed_max()`, `get_destination_speed_max()`, `set_destination_speed_max()`

Internal method `calc_target_pos_vel(time, position, velocity, acceleration, jerk)` — evaluates the Hermite polynomial at parameter `time` in [0,1] and returns position, velocity, acceleration, and jerk simultaneously.

Internal: `_hermite_solution[4]` — four Vector3p/Vector3f coefficients for the cubic parametric curve.

**`SCurve.h` / `SCurve.cpp`** — S-curve (7-segment jerk-limited) trajectory for smooth acceleration profiles. Separate from the Hermite spline. Used by more advanced waypoint navigation.

### 1.7 Geodesy

Constants in `definitions.h`:
- `GRAVITY_MSS = 9.80665f` (m/s²)
- `RADIUS_OF_EARTH = 6378100` (meters, approximate)
- `WGS84_A = 6378137.0` (semi-major axis, meters)
- `WGS84_IF = 298.257223563` (inverse flattening)
- `WGS84_F = 1/298.257223563`
- `WGS84_B = WGS84_A*(1-WGS84_F)` (semi-minor axis)
- `WGS84_E = sqrt(2*F - F^2)` (eccentricity, requires double)
- `LATLON_TO_M = 0.011131884502145034` — converts 1e-7 deg to meters (arc length per unit at equator, not accounting for longitude scaling)
- `LATLON_TO_M_INV = 89.83204953368922`

**ECEF conversions** in `location.cpp` / `location_double.cpp`:
- `wgsllh2ecef(llh, ecef)` — double precision WGS84 LLH → ECEF. Iterative Bowring method or Zhu method.
- `wgsecef2llh(ecef, llh)` — ECEF → WGS84 LLH. Both take/return `Vector3d` where x=lat (rad), y=lon (rad), z=height (m).

**GeodesicGrid** (`AP_GeodesicGrid.h`) — tessellated icosahedron with 80 triangular sections. Used for compass calibration to ensure even sphere coverage:
- `section(v, inclusive)` — returns which of 80 sections a unit vector falls in; -1 if not found

---

## 2. Filter

### 2.1 LowPassFilter — First-Order IIR

**`DigitalLPF<T>`** — base class with:
- `get()` — current output
- `reset(value)` — set to value
- `reset()` — flag for reset-on-next-sample
- `_apply(sample, alpha)` — `output = output + alpha*(sample - output)` if initialized, else `output = sample`

**`LowPassFilterConstDt<T>`** — constant time step variant:
- Constructor: `(sample_freq, cutoff_freq)`
- `set_cutoff_frequency(sample_freq, cutoff_freq)` — recalculates `alpha = calc_lowpass_alpha_dt(1/sample_freq, cutoff_freq)`
- `get_cutoff_freq()`, `apply(sample)`

**`LowPassFilter<T>`** — variable time step:
- Constructor: `(cutoff_freq)`
- `set_cutoff_frequency(cutoff_freq)`, `get_cutoff_freq()`
- `apply(sample, dt)` — recalculates alpha each call using `calc_lowpass_alpha_dt(dt, cutoff_freq)`

Alpha formula: `calc_lowpass_alpha_dt(dt, f_cutoff)` = `dt / (dt + 1/(2*pi*f_cutoff))`

Typedefs: `LowPassFilterFloat`, `LowPassFilterVector2f`, `LowPassFilterVector3f`, `LowPassFilterConstDtFloat`, `LowPassFilterConstDtVector2f`, `LowPassFilterConstDtVector3f`

### 2.2 LowPassFilter2p — Second-Order Biquad Butterworth

**`DigitalBiquadFilter<T>`** — Direct Form II transposed biquad:
```
delay_element_0 = sample - delay_element_1*a1 - delay_element_2*a2
output = delay_element_0*b0 + delay_element_1*b1 + delay_element_2*b2
```

State: `_delay_element_1`, `_delay_element_2`, `initialised` flag.

**`biquad_params` struct:** `cutoff_freq`, `sample_freq`, `a1`, `a2`, `b0`, `b1`, `b2`

**Parameter computation** (static, in `compute_params`):
```
cutoff is clamped to 0.4 * sample_freq (Nyquist margin)
fr = sample_freq / cutoff_freq
ohm = tan(PI / fr)
c = 1 + 2*cos(PI/4)*ohm + ohm^2
b0 = ohm^2/c, b1 = 2*ohm^2/c, b2 = ohm^2/c
a1 = 2*(ohm^2-1)/c, a2 = (1 - 2*cos(PI/4)*ohm + ohm^2)/c
```
This is a Butterworth 2nd-order lowpass with -3dB at cutoff_freq.

**Reset behavior:** On reset with value, sets delay elements to `value / (1 + a1 + a2)` (steady-state for DC input).

**`LowPassFilter2p<T>`** wraps the biquad:
- Constructor: `(sample_freq, cutoff_freq)` or default
- `set_cutoff_frequency(sample_freq, cutoff_freq)` — recomputes params
- `apply(sample)`, `reset()`, `reset(value)`
- `get_cutoff_freq()`, `get_sample_freq()`

Typedefs: `LowPassFilter2pInt`, `LowPassFilter2pLong`, `LowPassFilter2pFloat`, `LowPassFilter2pVector2f`, `LowPassFilter2pVector3f`

### 2.3 NotchFilter — Band-Reject

**`NotchFilter<T>`** — IIR notch (band-reject) filter:

Parameterization: `center_freq_hz`, `bandwidth_hz`, `attenuation_dB`

Internally uses:
- `A` — amplitude ratio (linear): `A = 10^(-attenuation_dB/40)`
- `Q` — quality factor: derived from octaves of bandwidth, `Q = sqrt(2^octaves) / (2^octaves - 1)`
- `omega = 2*PI*center_freq / sample_freq`
- `alpha = sin(omega) / (2*Q)`
- `b0 = 1 + alpha*A^2`, `b1 = -2*cos(omega)`, `b2 = 1 - alpha*A^2`
- `a1 = -2*cos(omega)` (note: same as b1), `a2 = 1 - alpha/A^2`

State: `ntchsig1`, `ntchsig2`, `signal1`, `signal2` — 4 delay elements (supports generalized biquad form)

**Frequency slew limiting:** Center frequency is limited to move at most 5% per update call (`NOTCH_MAX_SLEW = 0.05`). This prevents filter discontinuities when tracking dynamic frequencies.

Methods:
- `init(sample_freq_hz, center_freq_hz, bandwidth_hz, attenuation_dB)` — validates center < 0.5*sample and center > 0.5*bandwidth
- `init_with_A_and_Q(sample_freq_hz, center_freq_hz, A, Q)` — direct initialization
- `apply(sample)` — 2nd order IIR with slew limiting on center frequency
- `reset()` — clears initialized flag
- `calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, A, Q)` — static utility
- `center_freq_hz()`, `sample_freq_hz()`, `logging_frequency()` — accessors
- `disable()` — marks as uninitialized

**`NotchFilterParams`** — parameter storage (AP_Param integration):
- `_enable` (AP_Int8), `_center_freq_hz` (AP_Float), `_bandwidth_hz` (AP_Float), `_attenuation_dB` (AP_Float)

Typedefs: `NotchFilterFloat`, `NotchFilterVector2f`, `NotchFilterVector3f`

### 2.4 HarmonicNotchFilter — Dynamic Multi-Notch

**`HarmonicNotchFilter<T>`** — manages a bank of `NotchFilter<T>` instances targeting fundamental frequency and its harmonics.

Maximum harmonics: `HNF_MAX_HARMONICS = 16`

Composite notch options (from `HarmonicNotchFilterParams::Options`):
- `DoubleNotch` (bit 0) — two notches spread around center
- `DynamicHarmonic` (bit 1) — each harmonic gets independent frequency from FFT
- `LoopRateUpdate` (bit 2) — update every loop iteration
- `EnableOnAllIMUs` (bit 3)
- `TripleNotch` (bit 4) — three notches spread around center
- `TreatLowAsMin` (bit 5) — treat low RPM frequencies as minimum
- `QuintupleNotch` (bit 6)

Dynamic tracking modes (`HarmonicNotchDynamicMode`):
- `Fixed = 0` — static center frequency
- `UpdateThrottle = 1` — frequency scales with throttle (for constant-kv motors)
- `UpdateRPM = 2` — frequency from RPM telemetry
- `UpdateBLHeli = 3` — frequency from BLHeli ESC telemetry
- `UpdateGyroFFT = 4` — frequency from in-flight gyro FFT
- `UpdateRPM2 = 5` — secondary RPM source

Methods:
- `allocate_filters(num_notches, harmonics_bitmask, composite_notches)` — allocates filter bank; total count = num_notches * num_harmonics * composite_notches
- `expand_filter_count(total_notches)` — grow filter bank dynamically
- `init(sample_freq_hz, HarmonicNotchFilterParams)` — initialize all filters from params
- `update(center_freq_hz)` — update all notch centers from a single fundamental frequency; harmonics are multiples (2x, 4x, etc. based on harmonics bitmask)
- `update(num_centers, center_freq_hz[])` — update each notch independently (for DynamicHarmonic mode)
- `set_center_frequency(idx, center_freq_hz, spread_mul, harmonic_mul)` — set individual notch
- `apply(sample)` — apply all filters in series
- `reset()` — reset all filters
- `log_notch_centers(instance, now_us)` — logs current frequencies

Harmonic bitmask: bit N set means use harmonic (N+1). E.g., bitmask 0b00000111 = harmonics 1, 2, 3.

### 2.5 DerivativeFilter — Smooth Derivative

Based on Pavel Holoborodko's smooth low-noise differentiator algorithm.

**`DerivativeFilter<T, FILTER_SIZE>`** — extends `FilterWithBuffer<T,FILTER_SIZE>`:
- `update(sample, timestamp_us)` — adds sample with microsecond timestamp (handles non-uniform spacing)
- `slope()` — returns derivative estimate using Holoborodko coefficients over the buffer
- `reset()` — clears buffer

Typical sizes: 5, 7, 9 points. Pre-defined typedefs: `DerivativeFilterFloat_Size5/7/9`

Timestamps stored as `uint32_t _timestamps[FILTER_SIZE]` — accommodates 1.2 hour wraparound.

### 2.6 ModeFilter — Median Filter

**`ModeFilter<T, FILTER_SIZE>`** — insertion-sort based median:
- Constructor: `(return_element)` — specifies which sorted element to return (0=min, middle=median)
- `apply(sample)` — insertion sort + optionally drop highest or lowest (alternates to handle even window sizes)
- `get()` — last result

Uses `isort()` internally. Alternates between dropping highest and lowest sample on each call.

Pre-defined typedefs: `ModeFilterInt8_Size3..7`, `ModeFilterUInt8_Size3..7`, `ModeFilterInt16_Size3..7`, `ModeFilterUInt16_Size3..7`, `ModeFilterFloat_Size3..7`

### 2.7 AverageFilter — Moving Average

**`AverageFilter<T, U, FILTER_SIZE>`** — ring buffer averaging, where `U` is a wider type to prevent overflow:
- `apply(sample)` — adds to ring buffer via `FilterWithBuffer`, recomputes sum / count
- `reset()` — clears buffer and sample count

Ramps up: returns average of samples received so far until buffer fills.

**`AverageIntegralFilter<T, U, FILTER_SIZE>`** — optimized variant using running sum:
- `apply(sample)` — updates running `_sum` by subtracting the evicted element and adding new one; does not return value
- `getf()` — returns `(float)_sum / _num_samples`
- `getd()` — returns `(double)_sum / _num_samples`

Designed for integral types to avoid float rounding in the sum.

### 2.8 SlewLimiter

`SlewLimiter.h/cpp` — limits rate of change of a signal:
- Not detailed in audit scope but exists in Filter library
- Used for servo slew rate limiting

---

## 3. AP_Logger

### 3.1 Architecture

Three-layer architecture:

1. **Frontend (`AP_Logger`)** — singleton; exposes `Write()` API; handles rate limiting; routes to backends
2. **Backend abstract class (`AP_Logger_Backend`)** — defines the interface; manages format emission, message writing, rate limiting
3. **Concrete backends:**
   - `AP_Logger_File` — writes to SD card filesystem (FAT or LittleFS), most common
   - `AP_Logger_MAVLink` — streams log to GCS over MAVLink (useful without SD card)
   - `AP_Logger_Block` — abstract base for flash-memory block devices
     - `AP_Logger_Flash_JEDEC` — SPI JEDEC flash
     - `AP_Logger_W25NXX` — NAND flash (W25N series)

**Write-behind buffer:** Each backend has a `ByteBuffer writebuf`. On `WritePrioritisedBlock()`, data goes into `writebuf`. An IO thread (`io_timer()`) drains the buffer to the actual storage medium. This decouples the fast loop from slow I/O.

**Buffer sizes** (from `AP_Logger.cpp`):
- HAL_MEM_CLASS_1000+ : 200 KB (minus FAT IO size adjustment)
- HAL_MEM_CLASS_500+ : 80 KB
- HAL_MEM_CLASS_300+ : 50 KB
- Default : 16 KB

**Log file management:** Up to 500 log files (`MAX_LOG_FILES`), minimum 2. Files are numbered sequentially. On SD card, files are `APM/LOGS/00000001.BIN` etc.

**Logging thread:** Runs at `HAL_LOGGING_STACK_SIZE` (1580 bytes) stack. Separate from main loop.

### 3.2 FMT Format System

Binary log format is self-describing. Each message type is preceded (once per log) by a FMT record.

**FMT record structure** (`log_Format`):
```c
struct log_Format {
    LOG_PACKET_HEADER;  // head1=0xA3, head2=0x95, msgid=0x80
    uint8_t type;       // message type ID
    uint8_t length;     // total message length in bytes
    char name[4];       // 4-char message name
    char format[16];    // format string
    char labels[64];    // comma-separated field names
};
```

**Sync bytes:** `HEAD_BYTE1 = 0xA3` (163), `HEAD_BYTE2 = 0x95` (149). Every message starts with these two bytes plus a message ID byte.

**Format type codes** (in `LogStructure.h`):

| Code | Type | Description |
|------|------|-------------|
| `a` | int16_t[32] | Array of 16-bit integers |
| `b` | int8_t | Signed byte |
| `B` | uint8_t | Unsigned byte |
| `h` | int16_t | Signed 16-bit |
| `H` | uint16_t | Unsigned 16-bit |
| `i` | int32_t | Signed 32-bit |
| `I` | uint32_t | Unsigned 32-bit |
| `f` | float | Single precision float |
| `d` | double | Double precision float |
| `n` | char[4] | 4-char string |
| `N` | char[16] | 16-char string |
| `Z` | char[64] | 64-char string |
| `c` | int16_t*100 | Centiunits (e.g., centidegrees) |
| `C` | uint16_t*100 | Unsigned centiunits |
| `e` | int32_t*100 | Large centiunits |
| `E` | uint32_t*100 | Large unsigned centiunits |
| `L` | int32_t | Lat/lon in 1e-7 degrees |
| `M` | uint8_t | Flight mode |
| `q` | int64_t | Signed 64-bit |
| `Q` | uint64_t | Unsigned 64-bit |

**Units system:** Each field also has associated unit and multiplier codes stored in `log_Unit` and `log_Format_Units` records at the start of each log. Units are base SI units. Multipliers allow compact storage (e.g., an int16_t with multiplier 1e-3 stores milli-units).

### 3.3 Message Catalog

The `LogStructure` struct defines one entry per message type:
```c
struct LogStructure {
    uint8_t msg_type;
    uint8_t msg_len;
    const char *name;      // 4 chars max
    const char *format;    // format string
    const char *labels;    // comma-separated field names
    const char *units;     // per-field unit codes
    const char *multipliers; // per-field multiplier codes
    bool streaming;        // can be rate-limited
};
```

Message types are defined across many libraries, each contributing its own `LogStructure.h`. The master list is assembled in `AP_Logger/LogStructure.h` which includes:
- `AP_Beacon/LogStructure.h`
- `AP_DAL/LogStructure.h`
- `AP_NavEKF2/LogStructure.h`, `AP_NavEKF3/LogStructure.h`, `AP_NavEKF/LogStructure.h`
- `AP_GPS/LogStructure.h`
- `AP_BattMonitor/LogStructure.h`
- `AP_InertialSensor/LogStructure.h`
- `AP_AHRS/LogStructure.h`
- `AP_Camera/LogStructure.h`
- `AP_Mount/LogStructure.h`
- `AP_Baro/LogStructure.h`
- `AP_CANManager/LogStructure.h`
- `AP_VisualOdom/LogStructure.h`
- `AC_PrecLand/LogStructure.h`
- `AP_Proximity/LogStructure.h`
- `AC_Avoidance/LogStructure.h`
- `AP_ESC_Telem/LogStructure.h`
- `AP_AIS/LogStructure.h`
- `AP_HAL_ChibiOS/LogStructure.h`
- `AP_RPM/LogStructure.h`
- `AC_Fence/LogStructure.h`
- `AP_Landing/LogStructure.h`
- `AC_AttitudeControl/LogStructure.h`
- `AP_HAL/LogStructure.h`
- `AP_Mission/LogStructure.h`
- `AP_Servo_Telem/LogStructure.h`

Plus core structures defined inline:
- `log_Format` (FMT), `log_Unit` (UNIT), `log_Format_Multiplier` (MULT), `log_Format_Units` (FMTU)
- `log_Parameter` (PARM) — parameter name, value, default value
- `log_DSF` — data flash stats (dropped, blocks, bytes, buf_space)
- `log_Event` (EV) — event ID (see `LogEvent` enum, 100+ event types)
- `log_Error` (ERR) — subsystem + error code
- `log_MSG` (MSG) — text message with ID and chunk sequence
- `log_RCIN` — 14 RC input channels (uint16_t each)
- `log_RCI2` — channels 15-16 plus override mask and flags
- `log_RCOUT` — RC output channels

The total message type count across all subsystems is approximately 150-200 distinct message types.

### 3.4 Rate Limiting

**`AP_Logger_RateLimiter`** — per-backend rate limiter:

Parameters: `rate_limit_hz` (AP_Float) — max rate for streaming messages when armed; `disarm_rate_limit_hz` — rate when disarmed.

State: `last_send_ms[256]` (uint16_t) — last send time in ms for each message ID; `last_sched_count[256]` — scheduler counter at last send; `not_streaming` (Bitmask<256>); `last_return` (Bitmask<256>).

`should_log(msgid, writev_streaming)`:
- Non-streaming messages always pass through
- Streaming messages are checked against `last_send_ms[msgid]` and the rate limit
- Multi-instance messages (same type, multiple sensors) use `last_sched_count` to give all instances the same pass/fail result on a given tick

**Streaming flag:** `LogStructure.streaming = true` marks a message as rate-limitable. High-rate messages (IMU, attitude, etc.) are marked streaming. The scheduler also passes `is_streaming` flag to `WritePrioritisedBlock`.

**Critical messages:** `WriteCriticalBlock()` always writes, bypassing rate limiting. Used for events, errors, mode changes, arming/disarming.

### 3.5 Log Replay (DAL)

Documented in the AP_DAL section below.

### 3.6 Binary Log Format

**Block device format** (`AP_Logger_Block`):

Physical structure:
- Flash organized into pages (typically 256 bytes each)
- Pages grouped into sectors (typically 4KB) and blocks (typically 64KB)
- Parameters: `df_PageSize`, `df_PagePerBlock`, `df_PagePerSector`, `df_NumPages`

**PageHeader** (at start of each page):
```c
struct PageHeader {
    uint32_t FilePage;   // page index within the current log file
    uint16_t FileNumber; // log file number
    // optional: uint32_t crc (when BLOCK_LOG_VALIDATE=1)
};
```

**FileHeader** (at start of first page of each log):
```c
struct FileHeader {
    uint32_t utc_secs;   // UTC timestamp of log start
};
```

Ring buffer: New logs overwrite oldest logs when storage fills. `df_EraseFrom` marks the oldest page to erase.

**File backend format** (`AP_Logger_File`): Standard binary file. Each message is raw binary with the 3-byte header (0xA3, 0x95, msg_id) followed by the packed struct content. No page headers.

---

## 4. AP_Param

### 4.1 Storage Format

Parameters are stored in EEPROM (or EEPROM-emulated flash via StorageManager). The storage area is the `HAL_STORAGE_SIZE` region.

**Header:** A sentinel offset is tracked in `sentinel_offset`. Storage begins with parameters and ends with a sentinel value.

**Types (`ap_var_type`):**
- `AP_PARAM_NONE = 0` — end marker
- `AP_PARAM_INT8 = 1` — int8_t (1 byte)
- `AP_PARAM_INT16 = 2` — int16_t (2 bytes)
- `AP_PARAM_INT32 = 3` — int32_t (4 bytes)
- `AP_PARAM_FLOAT = 4` — float (4 bytes)
- `AP_PARAM_VECTOR3F = 5` — Vector3f (12 bytes, stored as 3 floats)
- `AP_PARAM_GROUP = 6` — group/subgroup (no storage, metadata only)

Each stored value is identified by a key encoding the parameter's path in the group hierarchy. The key is a 9-bit value.

**Token structure:**
```c
struct ParamToken {
    uint32_t key : 9;           // identifies top-level entry
    uint32_t idx : 4;           // array index for array types
    uint32_t group_element : 18; // 18-bit path through group tree
    uint32_t last_disabled : 1;
};
```
Group element uses 6 bits per nesting level (3 levels max, 6+6+6=18 bits), allowing up to 64 elements per group level.

### 4.2 Group Tables — GroupInfo System

Parameters are organized into a tree using `GroupInfo` arrays:

```c
struct GroupInfo {
    const char *name;           // parameter name suffix
    ptrdiff_t offset;           // byte offset from object start
    union {
        const struct GroupInfo *group_info;    // nested group
        const struct GroupInfo **group_info_ptr; // pointer to group (dynamic)
        const float def_value;                  // default for scalars
        ptrdiff_t def_value_offset;             // pointer-type default
    };
    uint16_t flags;             // AP_PARAM_FLAG_*
    uint8_t idx;                // index within group (0-63)
    uint8_t type;               // ap_var_type
};
```

**Definition macros:**
- `AP_GROUPINFO(name, idx, clazz, element, def)` — define a scalar in a group
- `AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, flags)` — with flags
- `AP_GROUPINFO_FRAME(name, idx, clazz, element, def, frame_flags)` — with frame type mask
- `AP_NESTEDGROUPINFO(clazz, idx)` — nest another class's var_info
- `AP_SUBGROUPINFO(element, name, idx, thisclazz, elclazz)` — nested object member
- `AP_SUBGROUPPTR(element, name, idx, thisclazz, elclazz)` — dynamically allocated member
- `AP_GROUPEND` — sentinel `{ idx:0xFF, type:AP_PARAM_NONE }`

**Flags:**
- `AP_PARAM_FLAG_NESTED_OFFSET` (1) — subgroup offset is from parent
- `AP_PARAM_FLAG_POINTER` (2) — dynamically allocated object
- `AP_PARAM_FLAG_ENABLE` (4) — controls visibility of subtree
- `AP_PARAM_FLAG_NO_SHIFT` (8) — don't shift idx 0 to 63
- `AP_PARAM_FLAG_INFO_POINTER` (16) — var_info is a pointer (dynamic)
- `AP_PARAM_FLAG_INTERNAL_USE_ONLY` (32) — visible to GCS but not user-settable
- `AP_PARAM_FLAG_HIDDEN` (64) — hidden from param download
- `AP_PARAM_FLAG_DEFAULT_POINTER` (128) — default is a relative offset

**Frame type flags** (bits 8+): `AP_PARAM_FRAME_COPTER`, `AP_PARAM_FRAME_ROVER`, `AP_PARAM_FRAME_PLANE`, `AP_PARAM_FRAME_SUB`, `AP_PARAM_FRAME_TRICOPTER`, `AP_PARAM_FRAME_HELI`, `AP_PARAM_FRAME_BLIMP`. Parameters can be hidden for irrelevant frame types.

### 4.3 Defaults

- Each `GroupInfo` entry stores a `def_value` float in the union.
- At startup, `setup_object_defaults()` iterates the var_info table and loads default values into each AP_Param-derived variable.
- Saved values in EEPROM override defaults.
- `set_default_by_name(name, value)` — override a default by parameter name at runtime
- `set_defaults_from_table(table, count)` — batch default override from `defaults_table_struct[]`
- **Embedded param defaults:** Boards can ship a param file embedded in firmware (APJ format, max 1-8KB). Loaded before EEPROM values on first boot.
- Dynamic defaults: `AP_PARAM_FLAG_DEFAULT_POINTER` stores a relative offset to another field in the same object as the default source.

### 4.4 Parameter Conversion

**`ConversionInfo` struct:**
```c
struct ConversionInfo {
    uint16_t old_key;           // k_param_* for the old parameter
    uint32_t old_group_element; // old group element path
    enum ap_var_type type;      // old type
    const char *new_name;       // new parameter name to convert to
};
```

Migration process: When firmware is upgraded, old parameters stored in EEPROM might have different keys. `AP_Param::convert_old_parameters()` is called at startup with a conversion table. It:
1. Looks up the old key/element in EEPROM
2. Reads the stored value
3. Finds the new parameter by name
4. Writes the old value to the new location
5. Saves the new parameter

**AP_Param_config.h:** Backup storage (`AP_PARAM_STORAGE_BAK_ENABLED`) mirrors EEPROM to a second storage region on boards with ≥32KB storage. Provides redundancy for parameter corruption.

**Dynamic parameters** (`AP_PARAM_DYNAMIC_ENABLED`): Up to 10 dynamically created parameter tables (from scripts), with keys starting at `AP_PARAM_DYNAMIC_KEY_BASE = 300`. Supports Lua scripting adding parameters at runtime.

**API:**
- `find(name, ptype, flags)` — look up by name
- `find_by_index(idx, ptype, token)` — enumerate parameters
- `save(force_save=false)` — async save to EEPROM
- `save_sync(force_save, send_to_gcs)` — synchronous save
- `load()` — read from EEPROM
- `set_by_name(name, value)`, `get(name, value)`, `set_and_save_by_name(name, value)` — by-name access

---

## 5. AP_FlashStorage

**Purpose:** Log-structured flash storage for parameters on embedded boards. Provides EEPROM-like byte-addressable API over flash that can only be erased in large blocks.

**Design principles:**
- Two flash sectors alternated via log-structure append
- Erase only on init (prevents main loop stalls)
- Read requires full sector scan (called rarely)
- Write: append new block with updated data; old versions abandoned
- Works with flash that erases to 0xFF, writing only clears bits (except on F1/F3 which require full-word write)

**Storage layout:**

Sector header varies by MCU:
- F4/F7: `state1:8 | signature1:24` (4 bytes)
- F1/F3: `state1:32 | signature1:16` (8 bytes)
- H7: 3 state slots × 32 bytes each = 96 bytes total
- G4: 3 state slots × 8 bytes each = 24 bytes total

Sector states: `AVAILABLE`, `IN_USE`, `FULL`, `INVALID`

Block header (for each data block):
```c
struct block_header {
    uint16_t state:2;               // AVAILABLE/WRITING/VALID
    uint16_t block_num:11;          // which block of storage (offset/block_size)
    uint16_t num_blocks_minus_one:3; // number of contiguous blocks minus 1
};
```

Block sizes:
- H7: 30 bytes data + 2 byte header (writes in 32-byte chunks)
- G4: 6 bytes data + 2 byte header (writes in 8-byte chunks)
- F1/F3/F4/F7: 8 bytes data + 2 byte header (F4/F7 can write up to 64 bytes at once)

**API:**
- `init()` — loads both sectors, finds latest valid version of each block
- `write(offset, length)` — appends updated blocks; calls `switch_sectors()` if current sector fills
- `re_initialise()` — reload from current mem_buffer
- `switch_full_sector()` — explicit sector switch (CPU offline momentarily)
- `erase()` — full erase and re-init
- All operations go through the caller-provided callbacks: `FlashWrite`, `FlashRead`, `FlashErase`, `FlashEraseOK`

---

## 6. AP_Filesystem

**Purpose:** Unified file system abstraction. Provides POSIX-like API across multiple storage backends.

**Backends (`AP_Filesystem.h` includes them all):**
- `AP_Filesystem_FATFS` — FatFS (SD cards, SDMMC)
- `AP_Filesystem_FlashMemory_LittleFS` — LittleFS on internal flash
- `AP_Filesystem_ROMFS` — read-only embedded filesystem in firmware flash
- `AP_Filesystem_posix` — POSIX OS (Linux, SITL)
- `AP_Filesystem_ESP32` — ESP32-specific
- `AP_Filesystem_Mission` — virtual filesystem for mission items (`@MISSION/`)
- `AP_Filesystem_Param` — virtual filesystem for parameters (`@PARAM/`)
- `AP_Filesystem_Sys` — virtual filesystem for system info (`@SYS/`)

**API:** Standard POSIX-like file operations: `open`, `close`, `read`, `write`, `lseek`, `stat`, `unlink`, `rename`, `opendir`, `readdir`, `closedir`, `mkdir`

**Routing:** The main `AP_Filesystem` class routes requests to the appropriate backend based on path prefix or mount point.

**`posix_compat.h`:** Provides POSIX stdio compatibility wrappers (`fopen`, `fclose`, `fread`, `fwrite`, `fprintf`, etc.) mapping to AP_Filesystem. Used by Lua runtime.

---

## 7. AP_Scripting

### 7.1 Lua Runtime

**Lua version:** Lua 5.4 (source in `modules/` directory, custom-patched for embedded use)

**Memory limits:**
- Heap: `SCRIPTING_HEAP_SIZE` — configurable
  - SITL/Linux/1000-class: 200 KB
  - 500-class: 100 KB
  - Default: 43 KB
- Stack: `SCRIPTING_STACK_SIZE = 17 KB`, max 64 KB, min 8 KB
- Stack is per-script execution context

**Memory allocation:** Custom `alloc()` function uses `MultiHeap` (`AP_MultiHeap`) — a pool allocator that can expand by allocating additional chunks from system heap if initial heap fills. Controlled by `DISABLE_HEAP_EXPANSION` debug option.

**Sandbox restrictions:**
- `create_sandbox()` — strips dangerous Lua standard library functions
- No `io`, `os`, limited `package` (can load from `/APM/scripts/` only)
- Scripts run in their own environment table (`env_ref`) — isolated globals
- `lua_abort()` — emergency stop from C++ side

**VM execution limit:** `_script_vm_exec_count` (AP_Int32) — maximum instructions per time slot. When exceeded, `overtime` flag is set and the hook function is called, which raises a Lua error to cleanly abort the overrunning script.

**Script scheduling:** Scripts are a linked list sorted by `next_run_ms`. The scheduler calls `run_next_script()` on each VM time slot. Scripts call `coroutine.yield()` to return control and reschedule themselves.

**Script loading:**
- Scripts loaded from `SCRIPTING_DIRECTORY` (`/APM/scripts` on ChibiOS, `./scripts` on SITL)
- Also from ROMFS (embedded in firmware) unless disabled by `_dir_disable` bitmask
- CRC32 checksum tracked for loaded and running scripts

**Parameters:**
- `SCR_ENABLE` (AP_Int8) — enable/disable
- `SCR_VM_I_COUNT` (AP_Int32) — VM instruction limit
- `SCR_HEAP_SIZE` (AP_Int32) — heap size in bytes
- `SCR_DEBUG_OPTS` (AP_Int8) — debug bitmask
- `SCR_DIR_DISABLE` (AP_Int16) — disable ROMFS/scripts directory
- `SCR_THD_PRI` (AP_Enum) — thread priority
- `SCR_USER1..6` (AP_Float) — user-accessible floats for script inputs

### 7.2 Binding Generator

**Tool location:** `generator/` directory with a single description file: `generator/description/bindings.desc`

**Process:** A Python script (`gen/`) reads `bindings.desc` and generates C++ binding code (placed in `lua_bindings.cpp/h`). The generated code is checked into the repository.

**Binding format in `bindings.desc`:**
```
include <header.h>                        # include a C++ header
singleton ClassName rename lua_name       # expose singleton as lua_name
singleton ClassName method func_name return_type arg_types...
userdata ClassName field field_name type read/write [min max]
userdata ClassName method method_name return_type arg_types...
userdata ClassName field field_name depends FEATURE_FLAG
```

**Type qualifiers:**
- `'Null` — parameter may be nil (nullable)
- `'Ref` — passed by reference
- `'skip_check` — skip range validation
- `'enum MIN MAX` — enum with min/max constraints
- `'literal` — use a literal value
- `depends FEATURE` — conditionally compiled

**Binding count:** ~922 `singleton`/`userdata`/`method` entries in `bindings.desc`.

### 7.3 Available APIs (Selected)

From `bindings.desc`, Lua scripts can access:

**Location (`Location` userdata):**
- Fields: `lat`, `lng`, `alt`, `relative_alt`, `terrain_alt`, `origin_alt`, `loiter_xtrack`
- Methods: `get_distance`, `offset`, `offset_bearing`, `offset_bearing_and_pitch`, `get_vector_from_origin_NEU_m/cm`, `get_bearing`, `get_distance_NED/NE`, `change_alt_frame`, `set_alt_m`, `copy`

**AHRS (`ahrs` singleton):**
- `get_roll_rad`, `get_pitch_rad`, `get_yaw_rad` (radians)
- `get_location`, `get_home`
- `get_gyro`, `get_accel` — Vector3f
- `get_hagl`, `wind_estimate`, `groundspeed_vector`
- `get_velocity_NED`, `get_relative_position_NED_home`
- `home_is_set`, `healthy`, `airspeed_EAS`, `get_vibration`
- `earth_to_body`, `body_to_earth` — vector rotation
- `get_EAS2TAS`, `get_variances`, `set_posvelyaw_source_set`
- `set_home`, `get_origin`, `set_origin`
- `get_quaternion`

**Arming (`arming` singleton):**
- `disarm`, `is_armed`, `pre_arm_checks`, `arm`, `arm_force`
- `get_aux_auth_id`, `set_aux_auth_passed/failed`

**Battery monitor (`battery` singleton):**
- voltage, current, consumed mAh, state of charge

**GPS (`gps` singleton):**
- location, velocity, fix type, HDOP, num_sats, last fix time
- per-instance access

**InertialSensor (`ins` singleton):**
- `get_accel(instance)`, `get_gyro(instance)` — Vector3f
- temperature, delta angle, delta velocity
- `get_accel_count`, `get_gyro_count`

**RangeFinder (`rangefinder` singleton):**
- `distance_cm`, `status`, `has_data`, `ground_clearance_cm`

**Notify (`notify` singleton):**
- `play_tune`, text notification

**Mission:**
- Read/write mission commands
- `set_current_cmd`, `get_current_do_cmd`

**Servo/RC:**
- `servo.set_output_pwm`, `servo.set_output_norm`
- `rc.get_pwm`, `rc.normalize`

**Parameter access:**
- `param.get(name)`, `param.set(name, value)`, `param.set_and_save(name, value)`

**Scripting utilities:**
- `millis()`, `micros()` — time
- `math.sqrt`, standard math library (restricted)
- `string` library (limited)

**MAVLink:**
- Receive/send MAVLink messages via `mavlink`

**CAN bus access via `ScriptingCANSensor`**

**Serial port access via `AP_Scripting_SerialDevice`**

**I2C device access** (up to 4 devices)

**Network sockets** (up to 50, when networking enabled)

**Vector2f, Vector3f userdata** with full arithmetic

**Quaternion userdata** with from_euler, to_euler, rotate, multiply

### 7.4 Use Cases

Typical scripting use cases (from applets directory, ~49 applets):

**Sensor integration:**
- `leds_on_a_switch.lua` / `Hexsoon LEDs.lua` — LED control via GPIO or CAN
- `runcam_on_arm.lua` — camera control on arm/disarm
- `ONVIF_Camera_Control.lua` — IP camera control

**Custom flight modes:**
- `Aerobatics/` — complex aerobatic sequences
- `copter_terrain_brake.lua` — terrain-following brake
- `QuadPlane_Low_Alt_FW_mode_prevention.lua`
- `forward_flight_motor_shutdown.lua`

**Navigation enhancements:**
- `advance-wp.lua` — automatically advance waypoints
- `copter-deadreckon-home.lua` — dead reckoning to home on GPS loss
- `copter-slung-payload.lua` — payload swing compensation
- `quadplane_terrain_avoid.lua`
- `plane_follow.lua`, `plane_ship_landing.lua`
- `UniversalAutoLand.lua`

**Telemetry and comms:**
- `RockBlock.lua` / `RockBlock-9704.lua` — Iridium satellite comms
- `SmartAudio.lua` — VTX control
- `net-ntrip.lua` — RTK corrections over network
- `net_webserver.lua` — web server on autopilot

**Calibration:**
- `crsf-calibrate.lua` — CRSF radio calibration
- `ahrs-source-extnav-optflow.lua` — AHRS source management

**Parameter management:**
- `Param_Controller.lua` — runtime parameter adjustment
- `param-lockdown.lua` — prevent param changes
- `revert_param.lua` — revert params on boot

**Tuning:**
- `VTOL-quicktune.lua` — automated VTOL PID tuning
- `rover-quicktune.lua`
- `Heli_IM_COL_Tune.lua` — helicopter collective tuning

**Diagnostics:**
- `BattEstimate.lua` — battery capacity estimation
- `motor_failure_test.lua`
- `repl.lua` — interactive Lua REPL

**Special hardware:**
- `pelco_d_antennatracker.lua` — Pelco-D PTZ camera control
- `winch-control.lua` — winch operation
- `CAN_playback.lua` — CAN bus message replay

### 7.5 Applets Count

99 files in the applets directory (49 .lua + 49 .md + 1 README.md + subdirectories like Aerobatics).

---

## 8. AP_DAL

### 8.1 Purpose

The Data Abstraction Layer (DAL) enables log replay for EKF debugging and tuning. All sensor data consumed by EKF2 and EKF3 must pass through DAL wrappers. When `LOG_REPLAY=1`, all DAL inputs are logged. The replay tool reads these logs and feeds the same data back to the EKF, reproducing estimator behavior deterministically.

Key use: After a flight incident, tune EKF parameters offline using actual flight data.

### 8.2 Architecture

**Frame structure:** Each EKF update cycle is a "frame." At `start_frame(FrameType)`, the DAL records the current state of all sensor inputs. At `end_frame()`, it finalizes.

**FrameType flags:**
- `InitialiseFilterEKF2/EKF3` — EKF initialization frames
- `UpdateFilterEKF2/EKF3` — regular update frames
- `LogWriteEKF2/EKF3` — logging frames

**Frame header records:**
- `log_RFRH` — frame header (time_us, time_flying_ms)
- `log_RFRF` — frame flags (which operations to perform)
- `log_RFRN` — frame non-changing state (armed, vehicle class, EKF type, AHRS trim, home location, feature flags)

**Sensor wrappers — each has `H` (header/config) and `I` (instance/data) log records:**

| Subsystem | DAL Wrapper | Log Records |
|-----------|-------------|-------------|
| IMU | `AP_DAL_InertialSensor` | RISH (header), RISI (instance data) |
| Barometer | `AP_DAL_Baro` | RBRH, RBRI |
| GPS | `AP_DAL_GPS` | RGPH, RGPI, RGPJ |
| Compass | `AP_DAL_Compass` | RMGH, RMGI |
| Airspeed | `AP_DAL_Airspeed` | RASH, RASI |
| RangeFinder | `AP_DAL_RangeFinder` | RRNH, RRNI |
| Beacon | `AP_DAL_Beacon` | RBCH, RBCI |
| VisualOdom | `AP_DAL_VisualOdom` | RVOH |

**Push-based sensor records** (called directly by subsystems):
- `log_ROFH` — optical flow
- `log_REPH` — external nav position
- `log_REVH` — external nav velocity
- `log_RWOH` — wheel odometry
- `log_RBOH` — body frame odometry
- `log_RSLL` — set lat/lng
- `log_RTER` — terrain data

**Event logging:** `log_event2/3()` records EKF events (resetGyroBias, resetHeightDatum, requestYawReset, etc.) so they can be replayed.

**Replay dispatch:** In replay mode, the replay tool reads log records and calls `handle_message(const log_XXX &)` on the AP_DAL singleton, which fills the sensor wrapper state. The EKF then reads from DAL wrappers instead of real hardware.

**`WRITE_REPLAY_BLOCK` macro:** Used at every point where sensor data enters the EKF path:
```c
WRITE_REPLAY_BLOCK(RISI, imu_data);  // logs the struct if changed
WRITE_REPLAY_BLOCK_IFCHANGED(RFRN, _RFRN, _RFRN_old);  // only if changed
```

**Core mapping:** `logging_core(c)` maps EKF core numbers for multi-core EKF3 replay.

---

## 9. AP_Scheduler

### 9.1 Task Table

Each vehicle defines its task table as an array of `AP_Scheduler::Task`:

```c
struct Task {
    task_fn_t function;      // FUNCTOR: void()
    const char *name;        // for debugging/logging
    float rate_hz;           // desired run rate (0 = loop rate)
    uint16_t max_time_micros; // time budget in microseconds
    uint8_t priority;        // lower number = higher priority
};
```

**Two task lists:**
- `_vehicle_tasks` — vehicle-specific tasks (passed to `init()`)
- `_common_tasks` — shared tasks from `AP_Vehicle::get_common_scheduler_tasks()`

Tasks within each list must be ordered by ascending priority.

**Loop rates:**
- Copter/Heli/Sub: `SCHEDULER_DEFAULT_LOOP_RATE = 400 Hz`
- All other vehicles: `SCHEDULER_DEFAULT_LOOP_RATE = 50 Hz`
- Range: 50–2000 Hz (enforced in init)

**Definition macros:**
```c
SCHED_TASK_CLASS(classname, classptr, func, rate_hz, max_time_micros, priority)
FAST_TASK_CLASS(classname, classptr, func)  // runs every loop iteration
```

Fast tasks have priorities 0, 1, 2 (`FAST_TASK_PRI0/1/2`); regular tasks have priority > 2.

### 9.2 Overrun Handling

In `run(time_available)`:

1. **Time budget check:** If `task.max_time_micros > time_available`, the task is skipped this tick (another task might fit in the remaining time).

2. **Slip detection:** If `dt >= interval_ticks * 2` (task is 2x or more overdue), `perf_info.task_slipped(i)` is recorded.

3. **Max slowdown:** If `dt >= interval_ticks * max_task_slowdown` (where `max_task_slowdown = 4`), `task_not_achieved++` is incremented. This drives adaptive loop extension.

4. **Adaptive extra time:** Counters `task_not_achieved` and `task_all_achieved` control `extra_loop_us`. When tasks are consistently not being achieved, the loop is given additional time budget. When tasks are consistently being achieved, extra time is reduced. This adapts to temporary CPU load spikes.

5. **Debug logging:** When `_debug >= 2`, "slip" messages are printed to console. When `_debug >= 3`, overrun messages are printed if a task exceeds its `max_time_micros`.

6. **Performance tracking:** `perf_info` records timing statistics per task when `Options::RECORD_TASK_INFO` is set.

### 9.3 Fast Loop Integration

Fast tasks (priority 0, 1, 2) are always given the full loop period as their time budget: `_task_time_allowed = get_loop_period_us()`. They are not rate-limited — they run every tick.

The main vehicle loop structure:
```
loop() {
    hal.scheduler->delay_microseconds_boost(loop_period_us);
    // Wait for IMU data (blocks on semaphore)
    tick();        // increment tick counter
    run(time_us);  // run tasks that fit in the remaining time
}
```

The scheduler semaphore `_rsem` is held during task execution and released while waiting for IMU samples.

**Parameters:**
- `SCHED_DEBUG` (AP_Int8) — debug level
- `SCHED_LOOP_RATE` (AP_Int16) — main loop rate in Hz
- `SCHED_OPTIONS` (AP_Int8) — options bitmask

---

## 10. AP_Common

Provides cross-cutting C++ utilities:

- `AP_Common.h` — `PACKED`, `WEAK`, `UNUSED_FUNCTION`, `OPTIMIZE(level)`, `NOINLINE`, `CLASS_NO_COPY`, `WARN_IF_UNUSED`, `FUNCTOR_TYPEDEF`, `FUNCTOR_BIND`
- `Bitmask.h` — template bitmask over fixed-size array
- `AP_ExpandingArray.h` — growable array
- `ExpandingString.h` — growable string (for task_info output)
- `TSIndex.h` — type-safe array index wrapper
- `float16.h` / `float16.cpp` — IEEE 754 half-precision float conversion
- `sorting.h` — `qsort_r` wrapper
- `time.h/cpp` — `AP_DateTime`, `AP_Clock` wrappers
- `NMEA.h/cpp` — NMEA sentence parser utility
- `Location.h/cpp` — the `Location` class (lat/lon/alt with altitude frame handling)
- `AP_FWVersion.h/cpp` — firmware version structure
- `c++.cpp` — `operator new` / `operator delete` with OOM handling; `NEW_NOTHROW` macro
- `missing/` — implementations of missing libc functions for embedded targets
- `AP_Test.h` — test framework macros

**`Location` class** (in `AP_Common/Location.h`) — the main geographic coordinate type used throughout ArduPilot:
- Fields: `int32_t lat` (1e-7 degrees), `int32_t lng` (1e-7 degrees), `int32_t alt` (cm), plus bitfield flags
- Altitude frames: `ABSOLUTE`, `RELATIVE`, `ABOVE_TERRAIN`, `ABOVE_ORIGIN`
- Methods: `get_distance(loc)`, `get_bearing_to(loc)`, `offset(north_m, east_m)`, `offset_bearing(bearing_cd, distance_m)`, `change_alt_frame(frame)`, `is_zero()`, `initialised()`, etc.

---

## 11. AP_AccelCal

Multi-position accelerometer calibration framework. Used during initial setup.

**`AP_AccelCal`** — orchestrates calibration:
- `start(gcs, sysid, compid)` — begin calibration sequence
- `update()` — called in main loop to advance calibration state
- `gcs_vehicle_position(position)` — receive vehicle orientation from GCS (6 positions: level, nose-up, nose-down, left-side-down, right-side-down, inverted)
- `get_status()` — returns `accel_cal_status_t`
- `cancel()` — abort calibration
- Supports up to 4 client subsystems via `register_client()`

**`AP_AccelCal_Client`** — interface for subsystems (INS, secondary IMU, etc.):
- `_acal_get_calibrator(instance)` — returns `AccelCalibrator*`
- `_acal_save_calibrations()` — save computed offsets/scale factors

**`AccelCalibrator`** — does the actual math:
- Collects samples at each orientation
- Fits an ellipsoid to the measurements
- Outputs offset vector and scale factors (3x3 diagonal or full matrix)
- Uses Levenberg-Marquardt optimization
- Status: `ACCEL_CAL_NOT_STARTED`, `ACCEL_CAL_WAITING_FOR_ORIENTATION`, `ACCEL_CAL_COLLECTING_SAMPLE`, `ACCEL_CAL_SUCCESS`, `ACCEL_CAL_FAILED`

---

## 12. Meridian Port Considerations

### High Priority Ports

**AP_Math:** All of this needs a Rust math library. Key design decisions:
- Use `f32` and `f64` generics matching ArduPilot's template approach
- `Vector2<T>`, `Vector3<T>`, `Matrix3<T>`, `QuaternionT<T>` as generic structs
- Rotation enum must match ArduPilot's numbering exactly for parameter/config compatibility
- WGS84 constants and ECEF conversion need double precision
- `constrain_float` / `safe_sqrt` / `safe_asin` — simple, critical for safety
- `wrap_PI` / `wrap_360` — used everywhere for angle normalization
- Control math (`sqrt_controller`, `shape_pos_vel_accel`, etc.) is core to all autopilot modes — complex but self-contained

**Filter:** All filters are needed for sensor processing:
- LowPassFilter: trivial IIR, implement both const-dt and variable-dt variants
- LowPassFilter2p: biquad, implement `compute_params` from sample/cutoff
- NotchFilter: need the A/Q computation, slew limiting on frequency update
- HarmonicNotchFilter: bank management, harmonics bitmask, dynamic mode enum
- DerivativeFilter: Holoborodko algorithm with non-uniform timestamps
- ModeFilter: insertion-sort median
- AverageFilter: ring buffer average with wider accumulator type

**AP_Logger:** For Meridian, the full logging infrastructure is needed:
- Binary format with sync bytes 0xA3/0x95 must be byte-for-byte compatible for GCS tools
- Implement the FMT/FMTU/UNIT/MULT startup records
- File backend (write to SD/filesystem) is the critical path
- Rate limiter is important for high-rate sensors
- MAVLink streaming backend is secondary

**AP_Param:** Essential for configuration persistence:
- EEPROM-compatible storage format needed for parameter persistence
- GroupInfo/var_info system is the most complex part — needs careful translation
- ConversionInfo for firmware upgrade migration
- Dynamic params (scripting) can be deferred

### Medium Priority Ports

**AP_Scheduler:** Task table + time-budget scheduler. Simpler to port than it looks — core is a loop over a sorted task list with `micros()` timing. The adaptive extra_loop_us feature is important for robustness.

**AP_DAL:** Only needed when implementing the EKF replay tool. Can defer until EKF is working and replay debugging is desired.

**AP_FlashStorage:** Only needed on embedded targets with internal flash for parameter storage. On SD-card targets, EEPROM-emulation through a file is simpler.

### Lower Priority Ports

**AP_Filesystem:** Rust's `std::fs` is sufficient for non-embedded. For embedded, use existing Rust embedded-hal storage traits. The virtual filesystems (Mission, Param, Sys) are nice-to-have.

**AP_AccelCal:** The calibration math (ellipsoid fitting, LM optimization) can be adapted from existing Rust crates. The protocol (6-position collection over MAVLink) needs to match ArduPilot's behavior for GCS compatibility.

**AP_Scripting:** Full Lua runtime is extremely complex to port. Consider:
1. Embedding the Lua 5.4 C library via FFI as a starting point
2. Or skipping scripting in Meridian's initial versions
3. The binding generator concept is valuable but the generated API surface (~922 bindings) is very large

**SplineCurve / SCurve:** Needed for smooth waypoint following. SplineCurve is straightforward Hermite polynomial evaluation. SCurve (7-segment jerk profile) is more complex.

---

## 13. Notable Implementation Details

1. **`constrain_float` is a macro**, not a function, because it injects `__AP_LINE__` for NaN tracking. In `constrain_value_line`, if the result is NaN, an internal error is triggered with the source line number. Meridian should have equivalent NaN detection.

2. **`ftype` / `postype_t` double-precision switch:** When `HAL_WITH_EKF_DOUBLE` is set, most filter and math types switch to double. EKF operates in double precision internally. Meridian should support this feature flag.

3. **Rotation enum stored in EEPROM:** The Rotation enum values are guaranteed stable. Adding new rotations requires MAVLink spec coordination. Meridian must use the same values.

4. **NotchFilter slew rate:** The 5% per-update slew limit on notch center frequency is critical — without it, rapid frequency changes cause filter ringing. The slew limit is applied in `init_with_A_and_Q()` before computing new coefficients.

5. **HarmonicNotchFilter dynamic allocation:** Filters are heap-allocated (`_filters = new NotchFilter<T>[count]`). On embedded, this happens at startup. If allocation fails, `_alloc_has_failed` is set and filtering is bypassed gracefully.

6. **AP_Logger write prioritization:** `WritePrioritisedBlock` reserves 1024 bytes for critical messages and another 1024 bytes for message-writer messages. Below these thresholds, non-critical writes are dropped. This ensures events/errors are never lost even when buffer is nearly full.

7. **Quaternion convention:** `q1=w, q2=x, q3=y, q4=z`. Note this is `w` first, not `x` first. Different from some libraries (e.g., nalgebra uses `[x,y,z,w]` or `w` first depending on API).

8. **Polygon algorithm performance:** Marked `#pragma GCC optimize("O2")` — indicates this is hot path. Meridian should ensure efficient implementation or inline.

9. **`LATLON_TO_M` constant:** This is the arc length per 1e-7 degree at the equator. For non-equatorial positions, longitude distances must be multiplied by `cos(lat)`. This scaling is **not** applied in `LATLON_TO_M` — callers must handle it. Critical for correct position estimates.

10. **Lua script isolation:** Each script has its own environment table (`env_ref`). Scripts cannot access each other's globals. They can only communicate via shared singleton APIs (params, mission, etc.).
