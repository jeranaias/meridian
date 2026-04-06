# Meridian Porting Audit — Wave 2: Motors, Servo, RC, Battery, ESC

**Source**: ArduPilot master, audited 2026-04-02  
**Libraries covered**: AP_Motors, AR_Motors (AP_MotorsUGV), SRV_Channel, RC_Channel, AP_BattMonitor, AP_ESC_Telem, AP_BLHeli  
**Auditor**: Automated audit via Automated tools  

---

## 1. AP_Motors — Class Hierarchy

### Three-level inheritance chain

```
AP_Motors                       (AP_Motors_Class.h/.cpp)
  └── AP_MotorsMulticopter      (AP_MotorsMulticopter.h/.cpp)
        └── AP_MotorsMatrix     (AP_MotorsMatrix.h/.cpp)
```

There is a parallel helicopter branch:
```
AP_Motors
  └── AP_MotorsHeli             (abstract)
        ├── AP_MotorsHeli_Single
        ├── AP_MotorsHeli_Dual
        └── AP_MotorsHeli_Quad
```

And several peer classes that also inherit AP_MotorsMulticopter:
- `AP_MotorsTri` — tricopter with yaw servo
- `AP_MotorsSingle` — single motor with fixed-pitch prop
- `AP_MotorsCoax` — coaxial frame
- `AP_MotorsTailsitter` — plane-copter hybrid tailsitter
- `AP_Motors6DOF` — full-6DOF frame
- `AP_MotorsMatrix_6DoF_Scripting` — scripted 6DOF variant

### Level 1: AP_Motors (base class)

Responsibilities:
- Defines the `DesiredSpoolState` enum (`SHUT_DOWN`, `GROUND_IDLE`, `THROTTLE_UNLIMITED`)
- Defines the `SpoolState` enum (`SHUT_DOWN`, `GROUND_IDLE`, `SPOOLING_UP`, `THROTTLE_UNLIMITED`, `SPOOLING_DOWN`)
- Holds pilot input scalars: `_roll_in`, `_pitch_in`, `_yaw_in`, `_throttle_in` (all from attitude controllers, range -1..+1 or 0..1)
- Also holds feed-forward variants: `_roll_in_ff`, `_pitch_in_ff`, `_yaw_in_ff`
- Holds `_forward_in`, `_lateral_in` for translation-capable frames
- Defines `PWMType` enum (NORMAL, ONESHOT, ONESHOT125, BRUSHED, DSHOT150/300/600/1200, PWM_RANGE, PWM_ANGLE)
- Holds motor limit flags struct (`limit.roll`, `limit.pitch`, `limit.yaw`, `limit.throttle_lower`, `limit.throttle_upper`)
- Declares pure-virtual interface: `init()`, `set_frame_class_and_type()`, `output()`, `output_min()`, `output_armed_stabilizing()`, `get_motor_mask()`, `update_throttle_filter()`, `get_throttle_hover()`
- Provides `output_test_seq()` for MAVLink motor test
- Provides `set_radio_passthrough()` for direct RC pass-through during setup
- Supports up to `AP_MOTORS_MAX_NUM_MOTORS` = 32 motors (MOT_1 through MOT_32)
- Default update rate: 490 Hz

Key internal state:
```cpp
float _roll_in, _roll_in_ff;         // -1..+1
float _pitch_in, _pitch_in_ff;       // -1..+1
float _yaw_in, _yaw_in_ff;           // -1..+1
float _throttle_in;                  // 0..1 from pilot
float _throttle_out;                 // 0..1 after mixing
float _throttle_avg_max;             // 0..1 ceiling hint
LowPassFilterFloat _throttle_filter; // smoothed throttle
DerivativeFilterFloat_Size7 _throttle_slew; // slew detector
bool _thrust_boost;                  // motor loss compensation active
float _thrust_boost_ratio;           // 0..1, smoothly transitions
```

### Level 2: AP_MotorsMulticopter

Responsibilities:
- Implements `output()` — orchestrates the full per-loop pipeline:
  1. `update_throttle_filter()` — LP-filter raw throttle input
  2. `thr_lin.update_lift_max_from_batt_voltage()` — compute voltage-based lift scaling
  3. `output_logic()` — spool state machine (see section 4)
  4. `output_armed_stabilizing()` — RPY+throttle mixing (implemented by child)
  5. `thrust_compensation()` — vehicle-specific callback (tiltrotor, tiltwing)
  6. `output_to_motors()` — write PWM
  7. `output_boost_throttle()` — optional booster motor
  8. `output_rpyt()` — expose scaled RPY+thrust as SRV_Channel outputs for logging
  9. `update_external_limits()` — merge scripting-supplied limit flags
- Implements `output_logic()` — the spool state machine
- Implements `get_current_limit_max_throttle()` — battery-current-based throttle ceiling
- Implements `output_to_pwm(float actuator)` — maps 0..1 normalized actuator to PWM microseconds
- Implements `set_actuator_with_slew()` — slew rate limiter
- Implements `update_throttle_hover()` — running estimate of hover throttle (TC = 10 s)
- Holds `Thrust_Linearization thr_lin` member object (see section 5)

Key parameters (MOT_ prefix for copter, Q_M_ for VTOL):
- `YAW_HEADROOM` (default 200 PWM units = 0.2 in 0..1 scale) — minimum yaw authority guaranteed
- `THST_EXPO` (default 0.65) — thrust curve exponent  
- `SPIN_MIN` (default 0.15) — normalized throttle at minimum thrust
- `SPIN_MAX` (default 0.95) — normalized throttle at maximum thrust
- `SPIN_ARM` (default 0.10) — armed idle spin
- `THST_HOVER` (default 0.35) — estimated hover throttle
- `HOVER_LEARN` (0=disabled, 1=learn, 2=learn+save)
- `BAT_VOLT_MAX`, `BAT_VOLT_MIN` — voltage compensation range
- `BAT_CURR_MAX` (default 0 = disabled) — current limit A
- `BAT_CURR_TC` (default 5 s) — current limit integrator time constant
- `PWM_TYPE` (0=Normal ... 7=DShot1200 ... 9=PWM_ANGLE)
- `PWM_MIN` (default 1000), `PWM_MAX` (default 2000)
- `SPOOL_TIME` (default 0.5 s) — ramp time for spin-up
- `SPOOL_TIM_DN` (0 = use SPOOL_TIME) — ramp time for spin-down
- `SLEW_UP_TIME` / `SLEW_DN_TIME` (default 0) — motor output slew limits, max 0.5 s
- `SAFE_TIME` (default 1.0 s) — post-arm delay when disarm-PWM was disabled
- `SAFE_DISARM` (0/1) — disable PWM while disarmed
- `IDLE_SEC` (default 0) — additional delay at ground-idle before spool-up
- `BOOST_SCALE` — booster motor scale factor
- `OPTIONS` bitmask (bit 0 = use raw voltage, not sag-compensated)

### Level 3: AP_MotorsMatrix

Responsibilities:
- Manages per-motor factor arrays: `_roll_factor[]`, `_pitch_factor[]`, `_yaw_factor[]`, `_throttle_factor[]`
- Stores `_test_order[]` — motor sequence for motor test
- Implements `setup_motors()` — dispatches to frame-specific `setup_*_matrix()` methods
- Calls `normalise_rpy_factors()` after all motors added
- Implements `output_armed_stabilizing()` — the core mixing algorithm (see section 2)
- Implements `output_to_motors()` — writes actuator values to PWM channels
- Tracks `_thrust_rpyt_out[]` — final per-motor output before linearization
- Tracks `_thrust_rpyt_out_filt[]` — 0.5 s EWM-filtered outputs for failure detection
- Tracks `_motor_lost_index` — index of detected failed motor

`add_motor()` helper computes roll/pitch factors from arm angle in degrees:
```cpp
roll_factor  = cosf(radians(angle_degrees + 90))
pitch_factor = cosf(radians(angle_degrees))
```
This means a motor at 45 deg gets roll_factor = cos(135) = -0.707, pitch_factor = cos(45) = +0.707.

---

## 2. AP_MotorsMatrix — output_armed_stabilizing() Algorithm

Source: `AP_MotorsMatrix.cpp`, lines 213–403

### Step-by-step breakdown

**Step 1 — Apply compensation gain**
```cpp
const float compensation_gain = thr_lin.get_compensation_gain();
// = 1/lift_max, where lift_max accounts for voltage sag and air density
// compensation_gain > 1 when battery is low (need more throttle for same thrust)
```

**Step 2 — Scale pilot inputs by compensation_gain**
```cpp
roll_thrust  = (_roll_in  + _roll_in_ff)  * compensation_gain;
pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
yaw_thrust   = (_yaw_in   + _yaw_in_ff)  * compensation_gain;
throttle_thrust     = get_throttle() * compensation_gain;
throttle_avg_max    = _throttle_avg_max * compensation_gain;
throttle_thrust_max = boost_ratio(1.0, _throttle_thrust_max * compensation_gain);
// throttle_thrust_max = 1.0 when thrust boost active, otherwise the spool ceiling
```

**Step 3 — Clamp throttle to valid range**
```cpp
throttle_thrust = constrain(throttle_thrust, 0, throttle_thrust_max);
throttle_avg_max = constrain(throttle_avg_max, throttle_thrust, throttle_thrust_max);
```

**Step 4 — Compute base throttle for yaw headroom calculation**
```cpp
throttle_thrust_best_rpy = MIN(0.5f, throttle_avg_max);
// This is the "center point" around which yaw headroom is computed.
// Choosing 0.5 means motors can swing ±0.5 for yaw without hitting bounds.
```

**Step 5 — Calculate maximum yaw that fits (per motor, take minimum)**

For each enabled motor:
```cpp
_thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
// (temporary: just roll+pitch, yaw not yet added)

thrust_rp_best_throttle = throttle_thrust_best_rpy + _thrust_rpyt_out[i];
if (yaw is positive on this motor):
    motor_room = 1.0 - thrust_rp_best_throttle   // room to ceiling
else:
    motor_room = thrust_rp_best_throttle           // room to floor
motor_yaw_allowed = MAX(motor_room, 0.0) / |_yaw_factor[i]|;
yaw_allowed = MIN(yaw_allowed, motor_yaw_allowed);
```

**Step 6 — Enforce yaw headroom minimum**
```cpp
yaw_allowed_min = _yaw_headroom * 0.001f;  // convert from PWM units to 0..1
// e.g., default 200 PWM headroom → yaw_allowed_min = 0.2
yaw_allowed_min = boost_ratio(0.5, yaw_allowed_min);
// → 0.5 when thrust boost active (motor lost)
yaw_allowed = MAX(yaw_allowed, yaw_allowed_min);
```

**Step 7 — Clamp yaw to what's allowed**
```cpp
if (|yaw_thrust| > yaw_allowed):
    yaw_thrust = constrain(yaw_thrust, -yaw_allowed, yaw_allowed)
    limit.yaw = true
```

**Step 8 — Add yaw to motor outputs, find min/max**
```cpp
for each motor:
    _thrust_rpyt_out[i] += yaw_thrust * _yaw_factor[i]
    rpy_low  = MIN(rpy_low,  _thrust_rpyt_out[i])
    rpy_high = MAX(rpy_high, _thrust_rpyt_out[i])  // excludes lost motor
```

**Step 9 — Compute rpy_scale (desaturation)**
```cpp
rpy_scale = 1.0f;
if (rpy_high - rpy_low > 1.0):
    rpy_scale = 1.0 / (rpy_high - rpy_low)   // scale so total swing = 1.0
if (throttle_avg_max + rpy_low < 0):
    rpy_scale = MIN(rpy_scale, -throttle_avg_max / rpy_low)
// rpy_scale < 1 means RPY was clipped — set limit flags
```

**Step 10 — Compute throttle adjustment**
```cpp
rpy_high *= rpy_scale;
rpy_low  *= rpy_scale;
throttle_thrust_best_rpy = -rpy_low;  // floor becomes the best base
thr_adj = throttle_thrust - throttle_thrust_best_rpy;
```
- If `rpy_scale < 1`: RPY is saturating full range; clamp `thr_adj = 0`
- If `thr_adj < 0`: throttle can't go lower, clamp to 0
- If `thr_adj > 1 - (throttle_thrust_best_rpy + rpy_high)`: would exceed ceiling, clamp

**Step 11 — Final per-motor output**
```cpp
throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
for each motor:
    _thrust_rpyt_out[i] = (throttle_thrust_best_plus_adj * _throttle_factor[i])
                          + (rpy_scale * _thrust_rpyt_out[i])
// throttle_factor is 1.0 for all standard frames (allows differential throttle)
```

**Step 12 — Record throttle out and check for failed motor**
```cpp
_throttle_out = throttle_thrust_best_plus_adj / compensation_gain;
check_for_failed_motor(throttle_thrust_best_plus_adj);
```

### Motor failure detection (check_for_failed_motor)

Uses a 0.5 s EWM filter on each motor's output. If a motor is consistently at the highest output level, it's flagged as the "lost" motor candidate. The criteria: if any one motor is carrying `≥ 1.5×` its fair share (`thrust_balance = rpy_high * num_motors / rpyt_sum`), `_thrust_balanced` is set false and thrust boost engages. Six or more motors required for this to activate; co-rotating frames are excluded.

---

## 3. Frame Types — setup_*_matrix() Functions

### Motor definition format
Each motor is defined by `{angle_degrees, yaw_factor, testing_order}` where:
- `angle_degrees`: arm direction measured CCW from front (0=front, 90=right, 180=rear, -90=left)
- `yaw_factor`: `AP_MOTORS_MATRIX_YAW_FACTOR_CW = -1`, `AP_MOTORS_MATRIX_YAW_FACTOR_CCW = +1`
- `testing_order`: 1-indexed sequence used by motor test command

Or raw form: `{roll_fac, pitch_fac, yaw_fac, testing_order}` for asymmetric arms.

After all motors added, `normalise_rpy_factors()` scales all factors so maximum magnitude = 0.5.

### QUAD (4 motors)

| Frame Type | Motor 1 | Motor 2 | Motor 3 | Motor 4 | Notes |
|---|---|---|---|---|---|
| PLUS | 90° CCW | -90° CCW | 0° CW | 180° CW | Standard + config |
| X | 45° CCW | -135° CCW | -45° CW | 135° CW | Standard X config |
| V | 45° 0.798 | -135° 1.0 | -45° -0.798 | 135° -1.0 | Reduced yaw authority |
| H | 45° CW | -135° CW | -45° CCW | 135° CCW | Same positions, reversed yaw |
| VTAIL | 60/60 none | 0/-160 CW | -60/-60 none | 0/160 CCW | Yaw only in rear, roll only in front |
| ATAIL | Same positions, reversed rear yaw | | | | V-shape vs A-shape props |
| BF_X | 135° CW | 45° CCW | -135° CCW | -45° CW | Betaflight ordering |
| DJI_X | 45° CCW | -45° CW | -135° CCW | 135° CW | DJI ordering |
| CW_X | 45° CCW | 135° CW | -135° CCW | -45° CW | Clockwise ordering |
| PLUSREV | All spins reversed vs PLUS | | | | |
| NYT_PLUS | Like PLUS but yaw_factor = 0 | | | | No differential torque for yaw |
| NYT_X | Like X but yaw_factor = 0 | | | | |
| Y4 | Raw: -1/+1 CCW, 0/-1 CW, 0/-1 CCW, +1/+1 CW | | | | Y4 shape |

### HEXA (6 motors)

| Frame Type | Notes |
|---|---|
| PLUS | 0°, ±60°, ±120°, 180° — alternating CW/CCW |
| X | 30°/90°/150°/-150°/-90°/-30° — alternating |
| H | Middle motors offset (raw factors) |
| DJI_X | DJI Matrice ordering |
| CW_X | Sequential clockwise from 30° |

### OCTA (8 motors)

| Frame Type | Notes |
|---|---|
| PLUS | 0°, ±45°, ±90°, ±135°, 180° |
| X | 22.5° offset from PLUS |
| V | Raw asymmetric factors |
| H | Horizontal layout (raw) |
| I | Sideways-H layout (raw) |
| DJI_X | DJI ordering, 22.5° steps |
| CW_X | Clockwise sequential |

### OCTAQUAD (8 motors, 4-arm coaxial)

Two motors per arm, one CW one CCW, at same position.

| Frame Type | Notes |
|---|---|
| PLUS | 4 arms at 0°/±90°/180°, pairs at each arm |
| X | 4 arms at ±45°/±135° |
| V | Reduced yaw on some motors |
| H | Reversed yaw direction vs X |
| CW_X | Sequential clockwise |
| BF_X | Betaflight 4-in-1 ESC ordering |
| BF_X_REV | Reversed motor direction |
| X_COR | Co-rotating pairs — top layer factors scaled by `AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR` |
| CW_X_COR | Co-rotating + clockwise ordering |

### Y6 (6 motors, 3-arm coaxial)

Three arms 120° apart, one motor on top one on bottom per arm.

| Variant | Notes |
|---|---|
| Y6B (default) | Top motors CW, bottom CCW |
| Y6F | FireFly Y6 layout — reversed top/bottom spin |
| default | Alternate arm weighting (0.666 vs 1.333) |

### DODECAHEXA (12 motors, 6-arm coaxial)

6 arms with paired motors (top/bottom). Available in PLUS and X configurations.

### DECA (10 motors)

10 arms at 36° spacing. Available in PLUS and X/CW_X configurations.

---

## 4. Spool State Machine

Source: `AP_MotorsMulticopter.cpp`, `output_logic()`, lines 581–864

### States

```
SHUT_DOWN → GROUND_IDLE → SPOOLING_UP → THROTTLE_UNLIMITED
                ↑              ↓              ↓
                └──────── SPOOLING_DOWN ←────┘
```

### State: SHUT_DOWN

- Entry condition: `!armed()` or `!get_interlock()`
- Sets: `_spin_up_ratio = 0`, `_throttle_thrust_max = 0`, `_thrust_boost = false`, `_thrust_boost_ratio = 0`
- All limit flags: `true` (no authority)
- Motor outputs: `_actuator[i] = 0` (in `output_to_motors()`)
- Exit to GROUND_IDLE: when `_spool_desired != SHUT_DOWN` AND `_disarm_safe_timer >= _safe_time`
  - `_disarm_safe_timer` accumulates only while armed, resets to 0 on disarm
  - Default `_safe_time = 1.0 s` — gives ESCs time to detect restored PWM after disarm

### State: GROUND_IDLE

- Servos move to correct attitude; motors at/near idle
- All limit flags: `true` (no authority)
- `_throttle_thrust_max = 0`
- `spin_up_ground_idle_ratio = _spin_arm / spin_min` (normalized armed idle as fraction of spin_min)
- `_idle_time` accumulates once `_spin_up_ratio >= spin_up_ground_idle_ratio`

Sub-transitions by `_spool_desired`:
- **→ SHUT_DOWN**: `spool_step = dt / spool_down_time`; decrement `_spin_up_ratio`; when hits 0 → SHUT_DOWN
- **→ GROUND_IDLE**: asymmetric slew; ramps `_spin_up_ratio` toward `spin_up_ground_idle_ratio` at different up/down rates
- **→ THROTTLE_UNLIMITED**: increment `_spin_up_ratio += dt / _spool_up_time`; hold at `spin_up_ground_idle_ratio` until `_idle_time >= _idle_time_delay_s`; once `_spin_up_ratio >= 1.0` and `!get_spoolup_block()` → SPOOLING_UP

`get_spoolup_block()` returns true when:
- ESC telemetry not yet active (if required)
- RPM not in expected range
- CPU overload
- Manual hold by vehicle code

### State: SPOOLING_UP

- Limit flags: all `false` (attitude control active during ramp)
- `_spin_up_ratio = 1.0` (held constant)
- `_throttle_thrust_max += dt / _spool_up_time`
- Exit to THROTTLE_UNLIMITED: when `_throttle_thrust_max >= MIN(get_throttle(), get_current_limit_max_throttle())`
- Reverse to SPOOLING_DOWN: if `_spool_desired != THROTTLE_UNLIMITED`
- Thrust boost fade: `_thrust_boost_ratio -= spool_step` toward 0

### State: THROTTLE_UNLIMITED

- Normal flight state
- `_spin_up_ratio = 1.0`
- `_throttle_thrust_max = get_current_limit_max_throttle()`
- If `_thrust_boost && !_thrust_balanced`: `_thrust_boost_ratio` ramps toward 1.0
- Else: `_thrust_boost_ratio` decays toward 0.0
- Exit to SPOOLING_DOWN: if `_spool_desired != THROTTLE_UNLIMITED`

### State: SPOOLING_DOWN

- Limit flags: all `false`
- `_spin_up_ratio = 1.0` (maintained during throttle ramp-down)
- `_throttle_thrust_max -= dt / spool_time` where `spool_time = SPOOL_TIM_DN if > 0.05 else SPOOL_TIME`
- Exit to GROUND_IDLE: when `_throttle_thrust_max <= 0`
- Reverse to SPOOLING_UP: if `_spool_desired == THROTTLE_UNLIMITED`
- `_thrust_boost_ratio` decays toward 0

### Motor output in output_to_motors()

```
SHUT_DOWN:          actuator[i] = 0 → output_to_pwm(0) = 0 or pwm_min
GROUND_IDLE:        actuator[i] = spin_up_ratio * spin_min  (via actuator_spin_up_to_ground_idle)
SPOOLING_UP/DOWN/THROTTLE_UNLIMITED:
                    actuator[i] = thr_lin.thrust_to_actuator(_thrust_rpyt_out[i])
```

---

## 5. Thrust Linearization — apply_thrust_curve_and_volt_scaling()

Source: `AP_Motors_Thrust_Linearization.cpp`

### Full pipeline: thrust → actuator

```cpp
float Thrust_Linearization::thrust_to_actuator(float thrust_in) const {
    thrust_in = constrain(thrust_in, 0.0, 1.0);
    return spin_min + (spin_max - spin_min) * apply_thrust_curve_and_volt_scaling(thrust_in);
}
```

So actuator = `spin_min + (spin_max - spin_min) * f(thrust)` where f is the compensation function.

### apply_thrust_curve_and_volt_scaling()

```cpp
float battery_scale = 1.0 / batt_voltage_filt.get();
// batt_voltage_filt is ratio V_actual / V_max, filtered at 0.5 Hz
// When battery is at 100% → battery_scale = 1.0
// When battery sags to 80% of max → battery_scale = 1.25

float e = constrain(curve_expo, -1.0, 1.0);  // THST_EXPO, default 0.65

if (e == 0):
    // Linear: directly scale thrust by voltage
    return lift_max * thrust * battery_scale;
else:
    // Quadratic-ish: solve second-order polynomial
    // Physical model: T = k * omega^2, PWM ~ omega
    // expo=0: linear T∝PWM; expo=1: T∝PWM^2
    float throttle_ratio = ((e - 1) + sqrt((1-e)^2 + 4*e*lift_max*thrust)) / (2*e);
    return constrain(throttle_ratio * battery_scale, 0.0, 1.0);
```

### update_lift_max_from_batt_voltage()

Called every loop. Uses `voltage_resting_estimate()` (sag-compensated) unless `BATT_RAW_VOLTAGE` option set.

```cpp
_batt_voltage = constrain(_batt_voltage, batt_voltage_min, batt_voltage_max);
batt_voltage_filt.apply(_batt_voltage / batt_voltage_max, dt);
// 0.5 Hz LP filter

lift_max = batt_voltage_filt.get() * (1 - e) + e * batt_voltage_filt.get()^2;
// For e=0.65: lift_max = 0.35*V_ratio + 0.65*V_ratio^2
// At V_ratio=1.0: lift_max=1.0
// At V_ratio=0.8: lift_max = 0.35*0.8 + 0.65*0.64 = 0.28 + 0.416 = 0.696
```

### get_compensation_gain()

```cpp
float ret = 1.0 / lift_max;
// Air density correction (AP_MOTORS_DENSITY_COMP=1 by default):
float air_density_ratio = AP::ahrs().get_air_density_ratio();
if (0.3 < air_density_ratio < 1.5):
    ret *= 1.0 / constrain(air_density_ratio, 0.5, 1.25);
// At sea level: density_ratio=1.0, no correction
// At altitude where air is 80% density: gain *= 1/0.8 = 1.25
```

### Inverse function (actuator_to_thrust)

Used in tailsitter transitions:
```cpp
float actuator_to_thrust(float actuator) {
    actuator = (actuator - spin_min) / (spin_max - spin_min);
    // Then: remove_thrust_curve_and_volt_scaling()
    // Solves reverse: if throttle_ratio * battery_scale = actuator, what is thrust?
    thrust = ((throttle/battery_scale)*(2e) - (e-1))^2 - (1-e)^2;
    thrust /= 4*e*lift_max;
}
```

---

## 6. Motor Test — MAVLink Interface

Source: `AP_Motors_Class.cpp`, `AP_MotorsMatrix.cpp`

`output_test_seq(motor_seq, pwm)` is the public entrypoint:
- Takes 1-based motor sequence number and literal PWM value (e.g., 1200)
- Calls vehicle-specific `_output_test_seq(motor_seq, pwm)`

In `AP_MotorsMatrix::_output_test_seq()`:
```cpp
for each motor i:
    if motor_enabled[i] && _test_order[i] == motor_seq:
        rc_write(i, pwm)
```

`_test_order[]` is set during frame setup. Testing order typically goes front-right=1, rear-right=2, rear-left=3, front-left=4 for Quad-X.

`output_test_num(channel, pwm)` also exists — directly by output channel (0-indexed), bypasses ordering.

MAVLink trigger: `MAV_CMD_DO_MOTOR_TEST` → vehicle code calls `output_test_seq()`. Requires `armed()` to be true or in test mode.

---

## 7. AP_MotorsHeli — Helicopter Specifics

Source: `AP_MotorsHeli.h`, `AP_MotorsHeli_RSC.h`

### Fundamental difference from multirotor

In AP_MotorsHeli, `_throttle_in` is reinterpreted as **collective pitch** (not motor speed). The main rotor spins at a fixed speed set by the RSC (Rotor Speed Controller). Yaw is controlled by the tail rotor.

### Key components

**Swashplate** (AP_MotorsHeli_Swash):
- Controls collective and cyclic pitch of main rotor blades
- For single-heli: 3 servo-actuated swashplate (H1, H3, CCPM)
- Mixing: `move_actuators(roll_out, pitch_out, coll_in, yaw_out)`

**RSC — Rotor Speed Controller** (AP_MotorsHeli_RSC):
- Controls main rotor RPM
- Modes: Throttle curve, governor, ESC governor, direct throttle
- Outputs to `k_heli_rsc` servo function
- Has `autorotation` state machine for engine-off glide recovery

**Collective parameters**:
```
H_COL_MIN (default 1250 PWM) — minimum collective
H_COL_MAX (default 1750 PWM) — maximum collective  
H_COL_MID — zero-thrust collective
H_COL_HOVER (default 0.5) — estimated hover collective
H_CYCSRV_MAX (default 2500 cdeg) — maximum cyclic servo throw
H_COL_MIN_DEG / H_COL_MAX_DEG — blade pitch angles in degrees
H_COL_LAND_MIN — minimum collective for altitude-controlled landed state
```

### Spool state differences

Heli uses the same `SpoolState` enum but transitions differ:
- SHUT_DOWN: RSC off, collective at minimum
- GROUND_IDLE: RSC at ground idle speed
- SPOOLING_UP: RSC ramping from idle to flight speed
- THROTTLE_UNLIMITED: RSC at flight speed, attitude control active
- SPOOLING_DOWN: RSC decelerating

`init_targets_on_arming()` returns false until rotors are tracking correctly — prevents control surface motion during engine startup.

`set_collective_for_landing(bool)` adjusts minimum collective to be higher than normal to prevent negative-G bounce on landing.

### Dual-heli and quad-heli

`AP_MotorsHeli_Dual`: tandem or side-by-side rotors, combined swashplate mixing.
`AP_MotorsHeli_Quad`: quad-rotor helicopter, uses matrix mixing.

---

## 8. DShot Commands

Source: `AP_HAL/RCOutput.h`, `AP_BLHeli/AP_BLHeli.cpp`

DShot uses values 0–47 as command codes (below minimum throttle of 48), sent repeatedly:

```cpp
enum BLHeliDshotCommand : uint8_t {
    DSHOT_RESET                      = 0,
    DSHOT_BEEP1                      = 1,   // low pitch beep
    DSHOT_BEEP2                      = 2,
    DSHOT_BEEP3                      = 3,
    DSHOT_BEEP4                      = 4,
    DSHOT_BEEP5                      = 5,   // high pitch beep
    DSHOT_ESC_INFO                   = 6,   // request ESC info
    DSHOT_ROTATE                     = 7,   // normal direction
    DSHOT_ROTATE_ALTERNATE           = 8,   // opposite direction
    DSHOT_3D_OFF                     = 9,   // disable 3D mode
    DSHOT_3D_ON                      = 10,  // enable 3D bidirectional mode
    DSHOT_SAVE                       = 12,  // save settings to flash
    DSHOT_EXTENDED_TELEMETRY_ENABLE  = 13,
    DSHOT_EXTENDED_TELEMETRY_DISABLE = 14,
    DSHOT_NORMAL                     = 20,  // set normal rotation (save via cmd 12)
    DSHOT_REVERSE                    = 21,  // set reverse rotation (save via cmd 12)
    // BLHeli32 only:
    DSHOT_LED0_ON  = 22, DSHOT_LED1_ON  = 23,
    DSHOT_LED2_ON  = 24, DSHOT_LED3_ON  = 25,
    DSHOT_LED0_OFF = 26, DSHOT_LED1_OFF = 27,
    DSHOT_LED2_OFF = 28, DSHOT_LED3_OFF = 29,
};
static constexpr uint8_t DSHOT_ZERO_THROTTLE = 48;  // min throttle value
```

Interface: `hal.rcout->send_dshot_command(command, chan, timeout_ms, repeat_count, priority)`

Direction reversal (`DSHOT_REVERSE` / `DSHOT_ROTATE_ALTERNATE`) requires `DSHOT_SAVE` afterward to persist across reboot.

3D mode (`DSHOT_3D_ON`): enables bidirectional operation where 48–1047 = reverse, 1049–2047 = forward, 1048 = stopped. Requires save + ESC reboot.

Bi-directional DShot (configured via `BLHELI_BDMASK`): ESC sends RPM data back on the same wire during the low phase. Requires compatible ESC firmware (BLHeli32 or BLHeli_S with BD patch). Used for harmonic notch filtering.

---

## 9. AR_Motors (AP_MotorsUGV)

Source: `AR_Motors/AP_MotorsUGV.h`, `AR_Motors/AP_MotorsUGV.cpp`

`AP_MotorsUGV` is a standalone class (does not inherit from AP_Motors). It manages rover/boat motors and servos through the SRV_Channel layer.

### Key interfaces

```cpp
void set_steering(float steering, bool apply_scaling = true);  // -4500 to +4500 centidegrees
void set_throttle(float throttle);                              // -100 to +100
void set_lateral(float lateral);                                // -100 to +100
void output(bool armed, float ground_speed, float dt);
```

### Output modes (detected by which SRV functions are assigned)

1. **Regular steering + throttle** — separate steering servo (`k_steering`) and throttle ESC (`k_throttle`)
2. **Skid steering** — `k_throttleLeft` + `k_throttleRight`, no separate steering servo
3. **Omni** — up to 4 motors with per-motor throttle/steering/lateral factors
4. **Sail** — `k_mainsail_sheet`, `k_wingsail_elevator`, `k_mast_rotation`

### Regular steering (output_regular)

- Vectored thrust mode (`VEC_ANGLEMAX > 0`, boat with outboard motor):
  - Steering angle = `atan(steering_norm / throttle_norm)`, limited to `VEC_ANGLEMAX`
  - Throttle boosted by `1/cos(steering_angle)` to compensate for rotation
- Speed-scaled steering (default, `SPD_SCA_BASE = 1 m/s`):
  - `steering *= SPD_SCA_BASE / |ground_speed|` when above base speed
  - Prevents excessive steering at high speed
  - When reversing: `steering *= -1` (reverse steering sense)

### Skid steering (output_skid_steering) — differential drive

```cpp
steering_scaled = steering / 4500.0
throttle_scaled = throttle * 0.01
lower_throttle_limit = -1.0 / thrust_asymmetry  // asymmetry default 1.0

best_steering_throttle = (1.0 + lower_throttle_limit) * 0.5  // = 0.0 when asymmetry=1

// Saturation mixing with priority parameter:
// str_thr_mix=0.5 (default): fair split
// str_thr_mix>0.5: steering priority
// str_thr_mix<0.5: throttle priority
motor_left  = throttle_scaled + steering_scaled
motor_right = throttle_scaled - steering_scaled

// Apply asymmetry correction (if motors stronger forward than backward):
if (motor < 0): motor *= thrust_asymmetry
```

### Boat mode differences

Boats typically use:
- Single throttle + vectored steering (outboard motor) → `output_regular()` with `VEC_ANGLEMAX` set
- Or differential thrust with two motors → `output_skid_steering()`
- Sail boats use additional `output_sail()` for mainsail, wingsail, mast rotation

Boat-specific: sail control outputs scaled -100..+100 or 0..100 to respective SRV functions. No special motor output mode; it's just regular/skid + sail overlay.

### Omni frames

```
OMNI3:        3 motors with factors [1,-1,-1], [0,-1,1], [1,1,1]
OMNIX:        4 motors ±throttle, ±steering, ±lateral
OMNIPLUS:     4 motors — 2 lateral-only, 2 throttle-only  
OMNI3MECANUM: 3 mecanum wheels with specific mixing constants
```

### Motor test

`output_test_pct(motor_seq, pct)`: accepts MOTOR_TEST_THROTTLE (1), MOTOR_TEST_STEERING (2), MOTOR_TEST_THROTTLE_LEFT (3), MOTOR_TEST_THROTTLE_RIGHT (4), MOTOR_TEST_MAINSAIL (5).

Parameters:
- `PWM_TYPE`: 0=Normal, 1=OneShot, 2=OneShot125, 3=BrushedWithRelay, 4=BrushedBiPolar, 5-8=DShot
- `THR_MIN` (0..20%), `THR_MAX` (5..100%)
- `SLEWRATE` (%/s, 0=disabled, default 100)
- `THST_EXPO` (-1..+1, default 0): thrust curve expo
- `SPD_SCA_BASE` (0..10 m/s, default 1): speed above which steering scales down
- `STR_THR_MIX` (0.2..1.0, default 0.5): steering vs throttle priority
- `VEC_ANGLEMAX` (0..90°, default 0): vectored thrust max angle
- `THST_ASYM` (1.0..10.0, default 1.0): forward/backward thrust ratio for skid steering
- `REV_DELAY` (0.1..1.0 s): delay before motor direction reversal

---

## 10. SRV_Channel — Servo Output Mapping

Source: `SRV_Channel/SRV_Channel.h`, `SRV_Channel/SRV_Channels.cpp`, `SRV_Channel/SRV_Channel_aux.cpp`

### Architecture

- `SRV_Channel`: represents one physical servo/ESC output (one per `NUM_SERVO_CHANNELS`, default 32)
- `SRV_Channels`: container class; manages all channels and provides static API
- Each physical channel has: `servo_min`, `servo_max`, `servo_trim` (all AP_Int16), `reversed` (AP_Int8), `function` (AP_Enum16)

### Function list (SRV_Channel::Function enum)

Total: 190 defined functions (k_nr_aux_servo_functions = 190). Selected key functions:

| Value | Name | Description |
|---|---|---|
| -1 | k_GPIO | GPIO pin |
| 0 | k_none | General PWM / scripting |
| 1 | k_manual | Direct RC pass-through (same channel) |
| 2-3 | k_flap, k_flap_auto | Flap control |
| 4 | k_aileron | Aileron |
| 6-9 | k_mount_pan/tilt/roll/open | Gimbal |
| 10 | k_cam_trigger | Camera trigger |
| 16-17 | k_dspoilerLeft1/Right1 | Differential spoilers |
| 19 | k_elevator | Elevator |
| 21 | k_rudder | Rudder |
| 26 | k_steering | Ground steering |
| 30 | k_engine_run_enable | Engine kill |
| 31 | k_heli_rsc | Helicopter main rotor RSC |
| 32 | k_heli_tail_rsc | Helicopter tail rotor RSC |
| 33-40 | k_motor1..k_motor8 | Copter motors 1-8 |
| 41 | k_motor_tilt | Tiltrotor motor tilt |
| 51-66 | k_rcin1..k_rcin16 | Raw RC pass-through by channel |
| 70 | k_throttle | Main throttle |
| 73-74 | k_throttleLeft/Right | Skid steering throttles |
| 75-76 | k_tiltMotorLeft/Right | Vectored thrust tilts |
| 77-78 | k_elevon_left/right | Elevon |
| 79-80 | k_vtail_left/right | V-tail |
| 81 | k_boost_throttle | Booster motor |
| 82-85 | k_motor9..k_motor12 | Copter motors 9-12 |
| 89 | k_mainsail_sheet | Sailboat mainsail |
| 94-109 | k_scripting1..k_scripting16 | Lua script outputs |
| 124-127 | k_roll_out/pitch_out/thrust_out/yaw_out | RPY+T for logging |
| 134-136 | k_min/k_trim/k_max | Fixed position outputs |
| 140-155 | k_rcin1_mapped..k_rcin16_mapped | Scaled RC pass-through |
| 160-179 | k_motor13..k_motor32 | Motors 13-32 |
| 181-183 | k_lights1/2/video_switch | Lights |
| 184-189 | k_actuator1..k_actuator6 | User peripherals |

### Servo output scaling modes

**Per-channel type setup** (called internally during function assignment):
- `set_angle(angle)`: output range is -angle to +angle (in centidegrees), mapped to servo_min..servo_max
- `set_range(high)`: output range is 0 to high, mapped to servo_min..servo_max

**Scaled value input** (from flight code):
```
SRV_Channels::set_output_scaled(function, value);
// Internally calls calc_pwm() which calls:
// - pwm_from_angle(scaled_value)  for angle type
// - pwm_from_range(scaled_value)  for range type
```

**Direct PWM** (for motor output):
```
SRV_Channels::set_output_pwm(function, pwm_us);
// Bypasses scaling; writes directly
```

### Reversing, trim, scaling per-channel

Each channel stores:
- `servo_min` (default 1000), `servo_max` (default 2000), `servo_trim` (default 1500)
- `reversed` flag: if set, maps are inverted
- Range-mode: `PWM = servo_min + (servo_max - servo_min) * (value / high_out)`
- Angle-mode when reversed: min/max swapped for direction, trim remains center

### Passthrough vs scaled output

**k_manual (function=1)**: reads `radio_in` from same-numbered RC input channel. Used to route RC input directly to servo output (e.g., aux pass-through).

**k_rcin1..k_rcin16 (51..66)**: reads `radio_in` from the specified RC input channel (1-indexed).

**k_rcin1_mapped..k_rcin16_mapped (140..155)**: reads RC input but converts through deadzone and normalization first (respects reversed, deadzone, trim). Outputs the properly scaled PWM.

Passthrough is disabled (`SRV_Channels::passthrough_disabled()`) when in certain states; channel outputs servo_trim in that case.

`ign_small_rcin_changes` flag: when set, ignores RC input changes smaller than deadzone. Used by `DO_SET_SERVO` MAVLink command to prevent jitter after direct servo override.

---

## 11. RC_Channel — Input Processing

Source: `RC_Channel/RC_Channel.h`, `RC_Channel/RC_Channel.cpp`

### Parameters per channel (RCn_)

- `MIN` (default 1100), `TRIM` (default 1500), `MAX` (default 1900) — calibration endpoints
- `REVERSED` (0/1) — reverse input
- `DZ` (default 0 PWM) — deadband around trim

### PWM → control_in conversion

`update()` is called once per RC input update (~50 Hz):
```cpp
if (type_in == ANGLE):
    control_in = pwm_to_angle()  // returns -4500 to +4500 centidegrees
else: // RANGE
    control_in = pwm_to_range()  // returns 0 to range_high
```

`pwm_to_angle()` → `pwm_to_angle_dz(dead_zone)` → `pwm_to_angle_dz_trim(dead_zone, radio_trim)`:
```cpp
if (radio_in < trim - dead_zone):
    return reverse_mul * (radio_in - trim) / (trim - radio_min) * high_in
    // -4500 to 0 when stick below center
elif (radio_in > trim + dead_zone):
    return reverse_mul * (radio_in - trim) / (radio_max - trim) * high_in
    // 0 to +4500 when stick above center
else:
    return 0  // deadzone region
```

`norm_input_dz()` returns same as above but normalized -1..+1 with deadzone applied.
`norm_input()` returns -1..+1 without deadzone (just reversal + linear mapping).
`norm_input_ignore_trim()` returns -1..+1 ignoring trim (maps min→-1, max→+1).

### Mode switching

`FLTMODE_CH` parameter (typically ch5 or ch6 for copter) selects the flight mode channel.

The vehicle code calls `rc().channel(FLTMODE_CH)->get_radio_in()` and maps PWM ranges to mode numbers. Not implemented in RC_Channel itself — vehicle-specific.

For copter: 6 mode slots, each assigned to a PWM sub-range (e.g., 1165-1360 for mode 2).

### Aux functions

`RCn_OPTION` parameter assigns an `AUX_FUNC` to the channel. Two-position or three-position behavior:

```cpp
bool RC_Channel::read_aux() {
    // Reads switch position: LOW (<1200), MIDDLE (1200-1800), HIGH (>1800)
    // Calls do_aux_function(AUX_FUNC, switch_pos) on state change
    // Has SWITCH_DEBOUNCE_TIME_MS = 200 ms debounce
}
```

The full `AUX_FUNC` list spans values 0..220+. Key groups:

**Flight modes** (can be triggered from aux channel):
RTL=4, AUTO=16, LAND=18, BRAKE=33, THROW=37, SMART_RTL=42, POSHOLD=69, ALTHOLD=70, FLOWHOLD=71, CIRCLE=72, STABILIZE=68, ACRO=52, MANUAL=51, GUIDED=55, LOITER=56, QSTABILIZE=170, QRTL=108, TURTLE=151, CRUISE=150, FBWA=92

**Arming/safety**:
ARMDISARM=153, ARMDISARM_AIRMODE=154, MOTOR_ESTOP=31, MOTOR_INTERLOCK=32, ARM_EMERGENCY_STOP=165

**Camera/mount**:
CAMERA_TRIGGER=9, CAMERA_REC_VIDEO=166, CAMERA_ZOOM=167, MOUNT_YAW_LOCK=163, RETRACT_MOUNT1=27

**Navigation/sensors**:
RANGEFINDER=10, FENCE=11, GPS_DISABLE=65, COMPASS_LEARN=62, EKF_SOURCE_SET=90

**Utility**:
LANDING_GEAR=29, GRIPPER=19, WINCH_ENABLE=44, WINCH_CONTROL=45, SPRAYER=15, PARACHUTE_RELEASE=22

**RC inputs** (map axes):
ROLL=201, PITCH=202, THROTTLE=203, YAW=204, MAINSAIL=207, FLAP=208, FWD_THR=209, AIRBRAKE=210

**Tuning/config**:
AUTOTUNE_MODE=17, AUTOTUNE_TEST_GAINS=180, TRANSMITTER_TUNING=219, FFT_NOTCH_TUNE=162, QUICKTUNE=181

Three-position switches (LOW/MID/HIGH) used for: ACRO_TRAINER=14, Q_ASSIST=82, SOARING=88, CROW_SELECT=87, SURFACE_TRACKING=75, SAILBOAT_MOTOR_3POS=74

---

## 12. AP_BattMonitor — Backend List

Source: `AP_BattMonitor/AP_BattMonitor_Params.h` (Type enum), `AP_BattMonitor.cpp`

### All backend types (Type enum)

| ID | Name | Description |
|---|---|---|
| 0 | NONE | Disabled |
| 3 | ANALOG_VOLTAGE_ONLY | Voltage divider ADC |
| 4 | ANALOG_VOLTAGE_AND_CURRENT | Voltage + current ADC (hall sensor) |
| 5 | SOLO | 3DR Solo specific |
| 6 | BEBOP | Parrot Bebop |
| 7 | SMBus_Generic | SMBus I2C battery |
| 8 | UAVCAN_BatteryInfo | DroneCAN BatteryInfo message |
| 9 | BLHeliESC | Battery data from BLHeli ESC telemetry |
| 10 | Sum | Virtual: sums multiple battery instances |
| 11 | FuelFlow | Fuel flow sensor |
| 12 | FuelLevel_PWM | Fuel level via PWM sender |
| 13 | SUI3 | SUI battery |
| 14 | SUI6 | SUI battery |
| 15 | NeoDesign | NeoDesign smart battery |
| 16 | MAXELL | Maxell smart battery |
| 17 | GENERATOR_ELEC | Generator electrical output |
| 18 | GENERATOR_FUEL | Generator fuel consumption |
| 19 | Rotoye | Rotoye smart battery |
| 21 | INA2XX | I2C INA219/INA220/INA226/INA228/INA231 |
| 22 | LTC2946 | I2C LTC2946 power monitor |
| 23 | Torqeedo | Torqeedo motor battery data |
| 24 | FuelLevel_Analog | Fuel level via analog sender |
| 25 | Analog_Volt_Synthetic_Current | Voltage ADC + synthetic current from throttle |
| 26 | INA239_SPI | SPI INA239 |
| 27 | EFI | Engine electronic fuel injection data |
| 28 | AD7091R5 | I2C AD7091R5 4-channel ADC |
| 29 | Scripting | Lua scripted battery |
| 30 | INA3221 | I2C INA3221 3-channel monitor |
| 31 | ANALOG_CURRENT_ONLY | Current-only ADC |
| 32 | TIBQ76952_I2C | Texas Instruments BQ76952 BMS |

### Analog backend (types 3, 4)

Reads ADC on two pins, applies voltage divider ratio:
- Voltage: `voltage = pin_reading * volt_multiplier`
- Current: `current = (pin_reading - offset) * amps_per_volt`
- Consumes integration: `consumed_mah += current * dt / 3600`

### SMBus backend (types 7, 13-16, 19)

I2C bus communication. Reads:
- Voltage from cell balancing registers (per-cell millivolts)
- Current (signed, for charge/discharge detection)
- State of charge, state of health, cycle count, serial number
- Temperature from thermistor register

### ESC telemetry backend (type 9)

Polls AP_ESC_Telem API to get per-ESC voltage, current, consumption. Aggregates:
```cpp
// From AP_BattMonitor_ESC.cpp:
voltage = max(per_esc_voltage)  // or average
current = sum(per_esc_current)
consumed_mah = sum(per_esc_consumed_mah)
```

---

## 13. AP_BattMonitor — Cell Monitoring

Source: `AP_BattMonitor.h` (`BattMonitor_State::cells`)

```cpp
struct cells {
    uint16_t cells[AP_BATT_MONITOR_CELLS_MAX];  // millivolts per cell
    // AP_BATT_MONITOR_CELLS_MAX = 14 (large firmware) or 12 (small)
    // Unused cells = 0 or UINT16_MAX
};
```

SMBus backends fill cell voltages from the CELL_VOLTAGE registers.
DroneCAN backend populates cells from BATTERY_STATUS MAVLink message cells array.

API:
- `has_cell_voltages(instance)` — returns true if cell data available
- `get_cell_voltages(instance)` — returns const ref to cells struct
- `get_cell_voltage(instance, cell, voltage)` — single cell access for scripting

---

## 14. AP_BattMonitor — Consumption Tracking

Source: `AP_BattMonitor_Backend.cpp`, `update_consumed()`

```cpp
void AP_BattMonitor_Backend::update_consumed(State &state, uint32_t dt_us) {
    if (state.last_time_micros != 0 && dt_us < 2000000) {
        float mah = calculate_mah(state.current_amps, dt_us);
        // calculate_mah = current_amps * dt_us / 3.6e9
        state.consumed_mah += mah;
        state.consumed_wh  += 0.001 * mah * state.voltage;
    }
}
```

`capacity_remaining_pct()`:
```cpp
float mah_remaining = pack_capacity - consumed_mah;
percentage = 100 * mah_remaining / pack_capacity;
```

`reset_remaining(percentage)`:
```cpp
consumed_mah = (1 - percentage/100) * pack_capacity;
consumed_wh  = consumed_mah * 0.001 * state.voltage;
```

---

## 15. AP_BattMonitor — Failsafe Thresholds

Source: `AP_BattMonitor_Backend.cpp`, `update_failsafes()` and `check_failsafe_types()`

### Failsafe hierarchy (severity, ascending)

```
Failsafe::None      → no action
Failsafe::Unhealthy → monitor has been silent > 5000 ms
Failsafe::Low       → vehicle-configured low action
Failsafe::Critical  → vehicle-configured critical action
```

### Trigger conditions

**Voltage sources** (configurable per monitor via `FAILSAFE_VOLTAGE_SRC`):
- Raw: `state.voltage`
- Sag-compensated: `voltage_resting_estimate = voltage + current * resistance` (see section 16)

**Low voltage** (`BATT_LOW_VOLT`, default 0 = disabled):
- If `voltage < low_voltage` for `LOW_VOLT_TIMEOUT` seconds → `Failsafe::Low`
- Timer resets when voltage recovers

**Critical voltage** (`BATT_CRT_VOLT`, default 0 = disabled):
- If `voltage < critical_voltage` for `LOW_VOLT_TIMEOUT` seconds → `Failsafe::Critical`
- Checked first — critical takes priority over low

**Capacity failsafe** (`BATT_LOW_MAH`, `BATT_CRT_MAH`, default 0 = disabled):
- Immediate trigger when `pack_capacity - consumed_mah < threshold_mah`
- No timer — instantaneous
- Critical capacity takes priority over low capacity

```cpp
// Priority order in update_failsafes():
if critical_voltage (with timer):   return Critical
if critical_capacity:               return Critical
if low_voltage (with timer):        return Low
if low_capacity:                    return Low
if monitor silent > 5s:             return Unhealthy
return None
```

### Arming minimum checks

- `BATT_ARM_VOLT` — must be above this to arm
- `BATT_ARM_MAH` — remaining capacity must be above this to arm
- Both checked in `AP_BattMonitor_Backend::arming_checks()`

### Failsafe actions

Set per-monitor via `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT`. Vehicle-specific numeric codes (varies by copter/plane/rover). Typical copter actions: 0=none, 1=land, 2=RTL, 3=SmartRTL, 4=SmartRTL/Land, 5=terminate.

---

## 16. AP_BattMonitor — Sag Compensation (Resistance Estimation)

Source: `AP_BattMonitor_Backend.cpp`, `update_resistance_estimate()`

### Algorithm

Runs on each battery read (nominally 10 Hz). Uses EWM filter with adaptive time constant:

```cpp
// Time since last update
float loop_interval = (now - _resistance_timer_ms) * 0.001f;

// Filter constant for voltage/current smoothing:
float filt_alpha = constrain(loop_interval / (loop_interval + TC1), 0, 0.5);
// TC1 = AP_BATT_MONITOR_RES_EST_TC_1 = 0.5 s

// Adaptive constant for resistance update — larger when current change is bigger:
float resistance_alpha = MIN(1, TC2 * |current_delta| / current_max);
// TC2 = AP_BATT_MONITOR_RES_EST_TC_2 = 0.1

// Instantaneous resistance estimate from Ohm's law:
float resistance_estimate = -(voltage - voltage_filt) / current_delta;
// Negative because voltage drops when current increases

// EWM update (only if resistance estimate is positive):
if (resistance_estimate > 0):
    state.resistance = state.resistance * (1 - resistance_alpha)
                     + resistance_estimate * resistance_alpha;

// Cap using max-current reference (prevents outlier contamination):
if (V_ref > V_now && I_now > I_ref):
    resistance_max = (V_ref - V_now) / (I_now - I_ref)
    state.resistance = MIN(state.resistance, resistance_max)

// Update filter state:
_voltage_filt = _voltage_filt * (1 - filt_alpha) + state.voltage * filt_alpha;
_current_filt = _current_filt * (1 - filt_alpha) + state.current_amps * filt_alpha;

// Sag-compensated estimate:
state.voltage_resting_estimate = state.voltage + state.current_amps * state.resistance;
```

### Usage in compensation

`voltage_resting_estimate()` returns `MAX(raw_voltage, voltage_resting_estimate)` — sag-compensated is always >= raw.

When `FAILSAFE_VOLTAGE_SRC = 1` (SagCompensated), failsafe thresholds compare against the resting estimate rather than the raw voltage. This prevents false failsafes when battery sags during high-current maneuvers.

When `Thrust_Linearization::update_lift_max_from_batt_voltage()` is called and `BATT_RAW_VOLTAGE` option is NOT set, it uses `voltage_resting_estimate()` from `AP::battery()`. This means voltage scaling uses the sag-compensated voltage by default.

---

## 17. AP_ESC_Telem — ESC Telemetry

Source: `AP_ESC_Telem/AP_ESC_Telem.h`, `AP_ESC_Telem/AP_ESC_Telem_Backend.h`

### Data per ESC (TelemetryData struct)

```cpp
struct TelemetryData {
    int16_t  temperature_cdeg;   // ESC temperature, centi-degrees C
    float    voltage;             // Bus voltage, Volts
    float    current;             // Phase current, Amps
    float    consumption_mah;     // Cumulative consumption, mAh
    uint32_t usage_s;             // Total run time, seconds
    int16_t  motor_temp_cdeg;    // Motor winding temperature, centi-degrees
    uint32_t last_update_ms;      // Timestamp; 0 = never received
    uint16_t types;               // Bitmask of which fields are valid
    uint16_t count;               // Number of updates received
    // Extended DShot telemetry v2:
    uint16_t edt2_status;        // Status flags from EDT v2
    uint16_t edt2_stress;        // Stress metric from EDT v2
    // Extended ESC telem (FETtec, etc.):
    uint8_t  input_duty;         // Input duty cycle
    uint8_t  output_duty;        // Output duty cycle  
    uint32_t flags;              // Status flags
    uint8_t  power_percentage;   // Power output %
};
```

### RPM data per ESC (RpmData struct)

```cpp
struct RpmData {
    float    rpm;              // Current RPM
    float    prev_rpm;         // Previous RPM (for filtering)
    float    error_rate;       // Packet error rate 0..1
    uint32_t last_update_us;   // Last update timestamp (us)
    float    update_rate_hz;   // How often RPM is updated
    bool     data_valid;       // Must be true before use
};
```

### Telemetry type bitmask

```cpp
enum TelemetryType {
    TEMPERATURE         = 1 << 0,
    MOTOR_TEMPERATURE   = 1 << 1,
    VOLTAGE             = 1 << 2,
    CURRENT             = 1 << 3,
    CONSUMPTION         = 1 << 4,
    USAGE               = 1 << 5,
    // Extended:
    EDT2_STATUS         = 1 << 8,
    EDT2_STRESS         = 1 << 9,
    INPUT_DUTY          = 1 << 10,
    OUTPUT_DUTY         = 1 << 11,
    FLAGS               = 1 << 12,
    POWER_PERCENTAGE    = 1 << 13,
};
```

### API (frontend AP_ESC_Telem)

- `get_rpm(esc_index, rpm)` — slewed RPM (data invalid after `ESC_RPM_DATA_TIMEOUT_US` = 1 s)
- `get_raw_rpm_and_error_rate(esc_index, rpm, error_rate)` — unslewed
- `get_temperature(esc_index, temp_cdeg)` — ESC temp
- `get_motor_temperature(esc_index, temp_cdeg)` — winding temp
- `get_current(esc_index, amps)` — per-ESC current
- `get_voltage(esc_index, volts)` — per-ESC bus voltage
- `get_consumption_mah(esc_index, mah)` — cumulative
- `get_average_motor_rpm(mask)` — average RPM across masked ESCs
- `are_motors_running(mask, min_rpm, max_rpm)` — health check
- `get_num_active_escs()` — count ESCs active within timeout
- `get_highest_temperature(temp)` — worst-case thermal

Timeout: `ESC_TELEM_DATA_TIMEOUT_MS = 5000` ms — data is "active" within this window.

### Backend push model

Drivers call into `AP_ESC_Telem_Backend`:
```cpp
void update_rpm(esc_index, new_rpm, error_rate);
void update_telem_data(esc_index, TelemetryData&, data_present_mask);
```

Backends include: AP_BLHeli (serial telemetry), HAL DShot bidirectional, FETtecOneWire, Torqeedo, UAVCAN/DroneCAN.

The frontend stores data in `volatile _rpm_data[]` and `volatile _telem_data[]` arrays (thread-safe via volatile access).

---

## 18. AP_BLHeli — BLHeli Passthrough

Source: `AP_BLHeli/AP_BLHeli.h`, `AP_BLHeli/AP_BLHeli.cpp`

### Purpose

BLHeli passthrough allows a ground station (Betaflight Configurator, BLHeli Suite) to directly communicate with ESCs for:
- Firmware flashing
- Settings configuration (motor direction, timing, demag compensation, etc.)
- ESC info readback

### Architecture

AP_BLHeli inherits `AP_ESC_Telem_Backend` and provides:
1. **Serial telemetry** — reads one-wire serial telemetry from BLHeli ESCs (RPM, current, temp, voltage)
2. **4-way passthrough** — transparent bridge between ground station UART and ESC programming interface

### Parameters

- `BLHELI_MASK` — bitmask of channels using BLHeli
- `BLHELI_OTYPE` — output type (0=auto, 1=SimonK, 2=BLHeli, etc.)
- `BLHELI_PORT` — serial port for passthrough GCS connection
- `BLHELI_POLES` — motor poles for RPM calculation (default 14)
- `BLHELI_TRATE` — telemetry request rate in Hz
- `BLHELI_BDMASK` — bi-directional DShot channel bitmask
- `BLHELI_RVMASK` — reversible motor channel bitmask
- `BLHELI_RMASK` — reversed motor direction bitmask

### Passthrough protocol

Uses the BLHeli 4-way interface protocol:
1. GCS sends MSP command `MSP_PASSTHROUGH_SERIAL_ESC` or `MSP_MOTOR_PASS` over the designated UART
2. AP_BLHeli detects the request, disables normal motor output
3. Opens the serial ESC interface to selected ESC via SIO (single-wire serial)
4. Byte-for-byte relay: GCS UART ↔ ESC SIO
5. Commands flow: read/write ESC flash, read EEPROM, reboot ESC

State machine states for 4-way protocol:
```
BLHELI_IDLE → BLHELI_HEADER_START → BLHELI_HEADER_CMD →
BLHELI_HEADER_ADDR_LOW → BLHELI_HEADER_ADDR_HIGH →
BLHELI_HEADER_LEN → BLHELI_CRC1 → BLHELI_CRC2 →
BLHELI_COMMAND_RECEIVED
```

### Serial telemetry collection

At `telem_rate` Hz, AP_BLHeli requests telemetry from one ESC per cycle (round-robin):
- 10-byte response: temperature, voltage (2B), current (2B), consumption (2B), RPM (2B), status
- Pushes data to `update_telem_data()` and `update_rpm()`
- RPM converted from eRPM: `actual_rpm = eRPM * 200 / motor_poles`

For bi-directional DShot: RPM comes back via `hal.rcout->get_erpm(motor_idx)` (eRPM) without needing separate UART. Same formula applies.

### ESC status address

BLHeli stores a status struct at flash address `0xEB00`:
```cpp
struct esc_status {
    uint8_t  unknown[3];
    enum esc_protocol protocol;  // NONE, NORMAL, ONESHOT125, DSHOT
    uint32_t good_frames;
    uint32_t bad_frames;
    uint32_t unknown2;
};
```

---

## 19. Summary: Key Porting Decisions for Meridian

### AP_Motors

1. **Trait-based hierarchy** — Meridian should model the 3-level hierarchy as Rust traits: `MotorController` → `MulticopterMotors` → `MotorMatrix`. Use composition for `ThrustLinearization`.

2. **Spool state machine** — implement as a Rust enum with explicit state transitions. The 5-state machine maps cleanly to a `match` expression. Critical: safety-forcing to SHUT_DOWN when not armed must be unconditional.

3. **Motor mixing** — `output_armed_stabilizing()` is entirely floating-point arithmetic. The rpy_scale desaturation and yaw headroom logic must be ported exactly. The `compensation_gain` must be applied before mixing.

4. **Frame definitions** — store as static arrays of `(angle_deg, yaw_factor, test_order)` structs. `normalise_rpy_factors()` must run after all motors added (scales to ±0.5 max magnitude).

5. **Thrust linearization** — the quadratic formula `throttle = ((e-1) + sqrt((1-e)^2 + 4e*lift_max*thrust)) / (2e)` must handle `e=0` as special case (linear). Apply `battery_scale = 1/batt_voltage_filt` afterward.

6. **Voltage compensation filter** — 0.5 Hz low-pass on `V_actual / V_max`. lift_max formula: `V_filt * (1-e) + e * V_filt^2`.

### AR_Motors (UGV)

7. **Separate from AP_Motors** — AP_MotorsUGV is completely independent. Meridian should have a `GroundVehicleMotors` struct separate from the `MotorMatrix` path.

8. **Skid steering mixer** — the saturation logic with `STR_THR_MIX` priority is non-obvious. The `saturation_value = MAX(max_output, min_output / lower_throttle_limit)` calculation is key.

### SRV_Channel

9. **Function registry** — 190 functions are a lot. Meridian should use an enum with the same numeric values for parameter compatibility, plus a mapping from function ID → physical output channel.

10. **Passthrough modes** — `k_manual` (same channel), `k_rcin1..16` (raw), `k_rcin1_mapped..16` (normalized) represent three distinct passthrough behaviors.

### RC_Channel

11. **Per-channel calibration** — each channel stores min/trim/max/reversed/deadzone independently. PWM→control conversion must apply deadzone correctly (return 0 within ±deadzone of trim).

12. **AUX function dispatch** — 200+ aux functions with debounced switch position detection. Implement as a trait/callback table indexed by AUX_FUNC enum. Three-position switch (LOW/MID/HIGH based on 1200/1800 thresholds) vs two-position.

### AP_BattMonitor

13. **Backend trait** — each backend implements a common trait with `read()`, `has_current()`, `has_cell_voltages()` etc. State is shared via a `BattMonitor_State` struct passed by reference.

14. **Resistance estimation** — the EWM resistance estimator with adaptive alpha is important for sag compensation. Both TC1 (0.5 s) and TC2 (0.1) are hardcoded constants.

15. **Failsafe priority** — critical voltage with timer takes priority, then critical capacity (immediate), then low voltage with timer, then low capacity (immediate), then unhealthy (5 s timeout). Order matters.

16. **Two failsafe actions** — each monitor has `_failsafe_low_action` and `_failsafe_critical_action`. The vehicle-level handler only escalates (never de-escalates) `_highest_failsafe_priority`.

### AP_ESC_Telem

17. **Volatile data** — `_rpm_data` and `_telem_data` arrays are `volatile` for thread safety between the update task and read accessors. Meridian should use `Mutex` or `AtomicCell`.

18. **Timeout tracking** — data is valid for 5000 ms (telem) or 1 s (RPM). Meridian should store timestamps and check recency on every access.

### AP_BLHeli / DShot

19. **DShot commands** — 30 defined commands (0-29), sent as repeat bursts at zero throttle. Commands 0-47 are reserved (below `DSHOT_ZERO_THROTTLE = 48`). Bi-directional DShot sends eRPM back; multiply by `200 / motor_poles` to get actual RPM.

20. **BLHeli passthrough** — this is an optional developer/maintenance feature. Meridian needs a dedicated serial bridge mode that disables motor output and relays bytes to/from ESCs via SIO.

---

## 20. Files Read (28 files)

1. `libraries/AP_Motors/AP_Motors_Class.h`
2. `libraries/AP_Motors/AP_Motors_Class.cpp`
3. `libraries/AP_Motors/AP_MotorsMulticopter.h`
4. `libraries/AP_Motors/AP_MotorsMulticopter.cpp`
5. `libraries/AP_Motors/AP_MotorsMatrix.h`
6. `libraries/AP_Motors/AP_MotorsMatrix.cpp`
7. `libraries/AP_Motors/AP_Motors_Thrust_Linearization.h`
8. `libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp`
9. `libraries/AP_Motors/AP_MotorsHeli.h`
10. `libraries/AP_Motors/AP_MotorsHeli_RSC.h`
11. `libraries/AR_Motors/AP_MotorsUGV.h`
12. `libraries/AR_Motors/AP_MotorsUGV.cpp`
13. `libraries/SRV_Channel/SRV_Channel.h`
14. `libraries/SRV_Channel/SRV_Channel_aux.cpp`
15. `libraries/SRV_Channel/SRV_Channels.cpp` (partial, via grep)
16. `libraries/RC_Channel/RC_Channel.h`
17. `libraries/RC_Channel/RC_Channel.cpp`
18. `libraries/AP_BattMonitor/AP_BattMonitor.h`
19. `libraries/AP_BattMonitor/AP_BattMonitor.cpp`
20. `libraries/AP_BattMonitor/AP_BattMonitor_Backend.h`
21. `libraries/AP_BattMonitor/AP_BattMonitor_Backend.cpp`
22. `libraries/AP_BattMonitor/AP_BattMonitor_Params.h`
23. `libraries/AP_ESC_Telem/AP_ESC_Telem.h`
24. `libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.h`
25. `libraries/AP_BLHeli/AP_BLHeli.h`
26. `libraries/AP_BLHeli/AP_BLHeli.cpp` (partial)
27. `libraries/AP_HAL/RCOutput.h` (DShot commands section)
28. `libraries/AP_Motors/AP_MotorsHeli.h` (helicopter specifics)
