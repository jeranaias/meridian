# Wave 3 Payload Audit: Camera, Gimbal, Mount, and Payload Systems

**Source**: ArduPilot `D:\projects\ardupilot\libraries\`
**Date**: 2026-04-02
**Scope**: AP_Camera, AP_Mount, AP_Gripper, AP_Winch, AP_Parachute, AP_LandingGear, AC_PrecLand, AP_IRLock

---

## Table of Contents

1. [AP_Camera](#1-ap_camera)
2. [AP_Mount](#2-ap_mount)
3. [AP_Gripper](#3-ap_gripper)
4. [AP_Winch](#4-ap_winch)
5. [AP_Parachute](#5-ap_parachute)
6. [AP_LandingGear](#6-ap_landinggear)
7. [AC_PrecLand + AP_IRLock](#7-ac_precland--ap_irlock)
8. [Meridian Port Notes](#8-meridian-port-notes)

---

## 1. AP_Camera

### 1.1 Backend List

Seven backend types plus RunCam, selected by the `CAM1_TYPE` / `CAM2_TYPE` parameter (enum `AP_Camera::CameraType`):

| Enum Value | Type | Description |
|---|---|---|
| 0 | NONE | Disabled |
| 1 | SERVO | Servo/PWM shutter trigger |
| 2 | RELAY | Relay-pin shutter trigger |
| 3 | SOLOGIMBAL | GoPro in 3DR Solo gimbal |
| 4 | MOUNT | Delegates to AP_Mount backend |
| 5 | MAVLINK | MAVLink camera (legacy, pre-v2) |
| 6 | MAVLINK_CAMV2 | MAVLink Camera Protocol v2 |
| 7 | SCRIPTING | Lua scripting backend |
| 8 | RUNCAM | RunCam serial protocol |

Maximum 2 simultaneous instances (`AP_CAMERA_MAX_INSTANCES = 2`).

All backends inherit from `AP_Camera_Backend` and must implement `trigger_pic()`. The frontend dispatches to all active instances for broadcast operations (e.g., `take_picture()`) or to a specific instance for targeted operations.

### 1.2 Trigger Types

**Shutter (still image)**:
- `take_picture()` — single image trigger
- `take_multiple_pictures(time_interval_ms, total_num)` — time-interval burst; `total_num = -1` means capture forever
- `stop_capture()` — halt the time-interval sequence
- Distance triggering via `trigg_dist` parameter (see section 1.4)

**Video**:
- `record_video(bool start_recording)` — start/stop recording

**Focus**:
- `set_focus(FocusType, float value)` — three FocusType values: `RATE` (value = -1 to +1), `PCT` (0–100%), `AUTO`
- Returns `SetFocusResult::{ACCEPTED, UNSUPPORTED, FAILED}`

**Zoom**:
- `set_zoom(ZoomType, float value)` — two ZoomType values: `RATE` (value = -1/0/+1), `PCT` (0–100%)

**Tracking**:
- `set_tracking(TrackingType, Vector2f p1, Vector2f p2)` — tracking types: `NONE`, `POINT`, `RECTANGLE`
- Coordinates normalized 0.0–1.0 with (0,0) at top-left

**Camera Source / Lens**:
- `set_lens(uint8_t lens)` — select physical lens 0–5
- `set_camera_source(CameraSource primary, CameraSource secondary)` — select by type: DEFAULT, RGB, IR, NDVI, RGB_WIDEANGLE

**Mode Toggle**:
- `cam_mode_toggle()` — momentary switch between image and video modes

### 1.3 Geo-Tagging (GPS Position Tagging)

Photo geo-tagging works through two paths:

**Immediate (software trigger)**:
When `trigger_pic()` succeeds, `log_picture()` is called which calls `prep_mavlink_msg_camera_feedback(AP::gps().time_epoch_usec())`. This captures:
- `camera_feedback.location` from `AP::ahrs().get_location()`
- `camera_feedback.roll_deg`, `pitch_deg`, `yaw_deg` from AHRS
- GPS epoch timestamp in microseconds

Then `Write_Camera()` writes a log entry, and `GCS_SEND_MESSAGE(MSG_CAMERA_FEEDBACK)` sends a `CAMERA_FEEDBACK` MAVLink message with lat/lon/alt MSL/alt-AGL/roll/pitch/yaw/image-index.

**Feedback pin (hardware-triggered)**:
When `feedback_pin` parameter > 0, an ISR or 1kHz timer monitors a GPIO pin. When the pin transitions, `feedback_pin_isr()` records `feedback_trigger_timestamp_us`. On next `check_feedback()` call:
- `prep_mavlink_msg_camera_feedback(feedback_trigger_timestamp_us)` captures location at the ISR timestamp
- Logged with the original hardware timestamp for accurate geo-tagging (not delayed by processing)

The `camera_feedback` struct stores: `timestamp_us`, `location`, `roll_deg`, `pitch_deg`, `yaw_deg`, `feedback_trigger_logged_count`.

### 1.4 Distance/Time Triggering

**Distance triggering** (survey/mapping mode):
- Controlled by `trigg_dist` parameter (meters)
- Evaluated in `AP_Camera_Backend::update()` at 50 Hz
- Logic:
  1. If `trigg_dist <= 0`, reset `last_location` and return
  2. Get current location from AHRS
  3. Guard: only trigger in AUTO mode if `_auto_mode_only` is set
  4. Guard: if `max_roll > 0` and current roll exceeds limit, skip
  5. Initialize `last_location` on first valid location
  6. If `current_loc.get_distance(last_location) >= trigg_dist`, call `take_picture()`

**Time-interval triggering**:
- Controlled by `time_interval_settings.{time_interval_ms, num_remaining}`
- Also evaluated in `update()` at 50 Hz
- If `num_remaining != 0`, time-interval triggering takes priority over distance triggering
- Counts down `num_remaining`; `num_remaining = -1` means infinite

**Minimum interval enforcement**:
- `interval_min` parameter: minimum seconds between shots
- If time since `last_picture_time_ms < interval_min * 1000`, sets `trigger_pending = true` and returns false
- Pending trigger is retried on next `update()` call

### 1.5 MAVLink Camera Protocol v2

Implemented in `AP_Camera_MAVLinkCamV2`. The backend:

**Discovery**: Scans MAVLink routing table via `find_camera()`, looking for a component with `MAV_COMP_ID_CAMERA*`. Throttled request of `CAMERA_INFORMATION` at startup.

**Messages sent by GCS to FC (then proxied to camera)**:
- `MAV_CMD_IMAGE_START_CAPTURE` → `trigger_pic()`
- `MAV_CMD_VIDEO_START_CAPTURE` / `MAV_CMD_VIDEO_STOP_CAPTURE` → `record_video()`
- `MAV_CMD_SET_CAMERA_ZOOM` → `set_zoom()`
- `MAV_CMD_SET_CAMERA_FOCUS` → `set_focus()`

**Messages sent by FC/backend to GCS** (via `send_mavlink_message()`):
- `CAMERA_INFORMATION` — vendor name, model name, focal length, sensor size, `lens_id` = instance number, `gimbal_device_id` = mount instance + 1, `CAMERA_CAP_FLAGS_*` bitmask
- `CAMERA_SETTINGS` — current camera mode (IMAGE/VIDEO), zoom %, focus % (both NaN if unknown)
- `CAMERA_CAPTURE_STATUS` — image_status (0:idle, 1:capturing, 2:interval-set/idle, 3:interval-set/capturing), interval seconds, image count
- `VIDEO_STREAM_INFORMATION` — conditionally compiled; populated via scripting or native camera
- `CAMERA_FOV_STATUS` — camera attitude quaternion (EF), camera location, POI location (uses AP_Mount's `get_poi()`)
- `CAMERA_THERMAL_RANGE` — optional, for thermal cameras

**From camera device to FC**:
- `CAMERA_INFORMATION` (received, stored in `_cam_info`)
- `CAMERA_CAPTURE_STATUS` (echoed to GCS)

---

## 2. AP_Mount

### 2.1 Backend List

Fourteen backend types selected by `MNT1_TYPE` / `MNT2_TYPE` parameter (enum `AP_Mount::Type`):

| Enum Value | Type | Protocol |
|---|---|---|
| 0 | None | — |
| 1 | Servo | PWM servo channels (direct HAL) |
| 2 | SoloGimbal | Custom MAVLink (3DR Solo) |
| 3 | Alexmos | SimpleBGC serial (UART, proprietary) |
| 4 | SToRM32 | MAVLink `DO_MOUNT_CONTROL` (via GCS routing) |
| 5 | SToRM32_serial | SToRM32 own binary serial protocol |
| 6 | MAVLink | MAVLink Gimbal Protocol v2 (`GIMBAL_DEVICE_*`) |
| 7 | BrushlessPWM | PWM servo channels (self-stabilizing, no FC stabilization) |
| 8 | Siyi | Siyi binary serial (custom, see section 2.7) |
| 9 | Scripting | Lua scripting |
| 10 | Xacti | DroneCAN-based |
| 11 | Viewpro | Viewpro binary serial (custom, see section 2.8) |
| 12 | Topotek | Topotek binary serial (custom) |
| 13 | CADDX | CADDX binary serial (custom) |
| 14 | XFRobot | XFRobot binary serial (custom) |

Maximum 2 simultaneous instances (`AP_MOUNT_MAX_INSTANCES = 2`).

All serial backends inherit from `AP_Mount_Backend_Serial` which handles UART lifecycle. All backends inherit from `AP_Mount_Backend` and must implement `has_pan_control()` and `get_attitude_quaternion()`.

### 2.2 Control Modes

MAVLink mount modes (enum `MAV_MOUNT_MODE`), stored as `_mode` in each backend:

| Mode | Description |
|---|---|
| `MAV_MOUNT_MODE_RETRACT` | Stow/retract mount into fuselage (servo driven by `k_mount_open` channel) |
| `MAV_MOUNT_MODE_NEUTRAL` | Move to neutral/level position, stop tracking |
| `MAV_MOUNT_MODE_MAVLINK_TARGETING` | Accept angle or rate targets from GCS/autopilot |
| `MAV_MOUNT_MODE_RC_TARGETING` | Pilot controls via RC sticks |
| `MAV_MOUNT_MODE_GPS_POINT` | Track a GPS coordinate (ROI) |
| `MAV_MOUNT_MODE_SYSID_TARGET` | Track another vehicle by MAVLink sysid |
| `MAV_MOUNT_MODE_HOME_LOCATION` | Point at home waypoint |

**Target types within `MAVLINK_TARGETING` mode** (enum `MountTargetType`):
- `ANGLE` — absolute angle command (roll/pitch/yaw in radians, with `yaw_is_ef` flag)
- `RATE` — angular rate command (rad/s, with `yaw_is_ef` flag)
- `RETRACTED` — command retract
- `NEUTRAL` — command neutral
- `LOCATION` — GPS location target (only supported by MAVLink-v2 backends with `GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL`)

**Frame-lock flags per axis** (stored in backend, used in RC_TARGETING mode):
- `_yaw_lock` — earth-frame yaw (gimbal maintains compass heading) vs. body-frame (follows vehicle heading)
- `_pitch_lock` — earth-frame pitch (horizon stays level) vs. body-frame
- `_roll_lock` — earth-frame roll (horizon stays level) vs. body-frame
- `FPV_LOCK` option — forces all axes to body-frame regardless of lock state

**RC rate vs angle**:
- If `rc_rate_max <= 0` parameter: RC sticks map to absolute angle targets within configured min/max range
- If `rc_rate_max > 0`: RC sticks map to rate commands (rate = stick_position * rc_rate_max_rads)

### 2.3 Gimbal Protocol per Backend

**Servo (Type 1, 7)**:
- Three SRV_Channel functions: `k_mount_roll`, `k_mount_tilt`, `k_mount_pan`
- Output computed via `move_servo()` which maps angle (deg*10) to PWM in [min_pwm, max_pwm]
- FC computes stabilization (adds AHRS angles to counteract vehicle lean) when `requires_stabilization = true` (Type 1)
- BrushlessPWM (Type 7) sets `requires_stabilization = false` — gimbal has internal IMU
- Native target type: ANGLE only

**SoloGimbal (Type 2)**:
- Custom MAVLink over companion-computer link; uses a dedicated small EKF (`SoloGimbalEKF`)
- Sends `GIMBAL_CONTROL` message with roll/pitch/yaw torque commands
- Receives `GIMBAL_REPORT` with actual angles
- Native target type: ANGLE only (after ROI computation)

**Alexmos / SimpleBGC (Type 3)**:
- UART serial at configurable baud rate
- Binary protocol: `0xXX CMD DATA_LEN DATA CHECKSUM`
- Commands: `CMD_BOARD_INFO`, `CMD_GET_ANGLES`, `CMD_CONTROL`, `CMD_READ_PARAMS`, `CMD_WRITE_PARAMS`
- `CMD_CONTROL` struct includes: mode byte, speed_roll (int16), angle_roll (int16), speed_pitch, angle_pitch, speed_yaw, angle_yaw — all in 0.02197-degree units
- `CMD_GET_ANGLES` returns struct with 9 int16 values (angle/rc_angle/rc_speed for each axis)
- Fixed-speed control at 30 deg/s
- Native target type: ANGLE only

**SToRM32 MAVLink (Type 4)**:
- Routes through MAVLink routing table using `find_gimbal()`
- Sends `MAV_CMD_DO_MOUNT_CONTROL` with roll/pitch/yaw in degrees
- No direct serial; uses MAVLink channel
- Native target type: ANGLE only
- FC adds vehicle roll/pitch to target if RP not locked (`apply_bf_roll_pitch_adjustments_in_rc_targeting = true`)

**SToRM32 serial (Type 5)**:
- UART serial at configurable baud rate
- Request-reply protocol: FC sends `cmd_set_angles_struct` containing roll/pitch/yaw as floats + flags + CRC16
- Reply is `SToRM32_reply_data_struct` (62 bytes) containing: state, status, IMU1 gyro/accel/AHRS, actual angles (imu1_pitch/roll/yaw), PID output, RC inputs, IMU2 angles, mag2 + CRC + magic byte
- Reply acknowledgment is 6-byte `SToRM32_reply_ack_struct`
- Native target type: ANGLE only
- Also adds vehicle lean angles when RP not locked

**MAVLink Gimbal Protocol v2 (Type 6)**:
- Implements the full MAVLink Gimbal Protocol v2 specification
- Discovery: scans routing table for `MAV_COMP_ID_GIMBAL*`
- Requests `GIMBAL_DEVICE_INFORMATION` (capability flags, vendor name, model, min/max angles)
- FC starts sending `ATTITUDE` and `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` so gimbal can use FC attitude
- Sends `GIMBAL_DEVICE_SET_ATTITUDE` with quaternion + angular velocity + flags
- Receives `GIMBAL_DEVICE_ATTITUDE_STATUS` with actual quaternion, angular velocity, failure flags
- Capability flags checked: `GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL` for GPS targeting
- For GPS targeting: sends `MAV_CMD_DO_SET_ROI` to gimbal
- FC publishes `GIMBAL_MANAGER_INFORMATION` and `GIMBAL_MANAGER_STATUS` to GCS
- Native target type: ANGLE + RATE + RETRACTED (+ LOCATION if capability flag set)

**Siyi (Type 8)**:
- See detailed protocol section 2.7 below

**Scripting (Type 9)**:
- Provides state to Lua script via `get_rate_target()`, `get_angle_target()`, `get_location_target()`
- Script provides actual attitude via `set_attitude_euler()`
- Native target type: whatever script implements

**Xacti (Type 10)**:
- DroneCAN-based — uses the DroneCAN bus, not a UART
- No native serial protocol; uses DroneCAN message types

**Viewpro (Type 11)**:
- See section 2.8 below

**Topotek (Type 12)**:
- Custom binary serial protocol
- Frame: 3-byte header + 2-byte address + 1-byte data_len + 1-byte control bit (r=query, w=set) + 3-byte identification + N-byte data + 2-byte ASCII-hex checksum
- Max packet: 36 bytes
- Native: ANGLE + RATE

**CADDX (Type 13)**:
- Custom binary serial protocol (not fully documented here)

**XFRobot (Type 14)**:
- Custom binary serial protocol (not fully documented here)

### 2.4 ROI (Region of Interest) — How Gimbal Points at GPS Coordinate

**Mode trigger**: When `set_roi_target(Location)` is called, `_roi_target` is stored and mode is set to `MAV_MOUNT_MODE_GPS_POINT`.

**Angle computation** (`get_angle_target_to_roi` → `get_angle_target_to_location`):
1. Get current vehicle position from AHRS
2. Compute bearing from vehicle to ROI: `atan2(lon_diff, lat_diff)` → target yaw (EF)
3. Compute horizontal distance: `current_loc.get_distance(roi_target)`
4. Compute vertical: target altitude minus vehicle altitude (positive = up)
5. Compute pitch: `atan2(height_diff, horizontal_distance)` → target pitch (EF)
6. Roll target: always 0 radians, earth-frame

Returned `MountAngleTarget` has `yaw_is_ef = true`, `pitch_is_ef = true`, `roll_is_ef = true`.

**Update loop**: In `_update_mnt_target()`, when mode is `MAV_MOUNT_MODE_GPS_POINT`, calls `get_angle_target_to_roi()` every cycle and updates `mnt_target.angle_rad`. The backend's `send_target_to_gimbal()` then converts as needed.

**POI Lock** (optional feature, `AP_MOUNT_POI_LOCK_ENABLED`):
- Aux function `MOUNT_POI_LOCK` — calculates where the gimbal is currently looking using terrain intersection
- Sets the intersection point as the new ROI target
- Requires AP_Terrain and runs in a dedicated background thread (`mount_calc_poi`)
- A "suspend" mid-position reverts to previous mode but keeps the target for later recall

**SysID tracking** (`MAV_MOUNT_MODE_SYSID_TARGET`):
- Receives `GLOBAL_POSITION_INT` from target sysid via `handle_global_position_int()`
- Stores target lat/lon/alt in `_target_sysid_location`
- Same bearing/pitch computation as ROI

### 2.5 Stabilization Role: FC vs Gimbal Internal

The division of labor depends on the backend:

**FC-provided stabilization** (Servo/SToRM32 types):
- FC reads roll/pitch/yaw from AHRS
- Adds AHRS lean angles to the desired earth-frame target to get body-frame output
- Called `adjust_mnt_target_if_RP_locked()` for locked axes
- `apply_bf_roll_pitch_adjustments_in_rc_targeting() = true` for these backends
- FC knows the actual vehicle attitude and compensates

**Gimbal-internal stabilization** (Alexmos, Siyi, Viewpro, MAVLink v2):
- The gimbal has its own IMU and runs its own stabilization loop
- FC only sends target angle/rate
- FC may also send its own attitude to the gimbal via `EXTERNAL_ATTITUDE` (Siyi) or `M_AHRS` (Viewpro) or `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` (MAVLink v2) so the gimbal can improve its estimate of vehicle attitude (reduces drift on long flights or during hard maneuvers)
- `apply_bf_roll_pitch_adjustments_in_rc_targeting() = false` for these backends

**Hybrid** (BrushlessPWM):
- PWM output like Servo, but no FC stabilization: `requires_stabilization = false`
- Assumes brushless gimbal with internal IMU

### 2.6 Rate Control vs Angle Control

The `MountTargetType` enum controls what the FC sends to the gimbal each cycle:

**Angle control**:
- Target stored as `MountAngleTarget {roll, pitch, yaw, yaw_is_ef, ...}` in radians
- Most serial backends (Alexmos, SToRM32, SoloGimbal) only support this natively
- `natively_supported_mount_target_types()` returns `NATIVE_ANGLES_ONLY`

**Rate control**:
- Target stored as `MountRateTarget {roll, pitch, yaw, yaw_is_ef}` in rad/s
- Siyi and Viewpro support both natively: `NATIVE_ANGLES_AND_RATES_ONLY`
- MAVLink v2 also supports both plus RETRACTED

**Automatic conversion** (`send_target_to_gimbal()` in `AP_Mount_Backend`):
- If backend only supports angles but a rate target is active, FC integrates the rate to produce an angle target via `update_angle_target_from_rate()`
- If backend only supports rates but an angle target is active, FC uses a P-controller to produce rate from angle error (e.g., Siyi's angle control is actually implemented this way at ~P=1.5)

**Rate-to-angle integration** (for angle-only backends):
- Assumes 50 Hz update rate (dt = 0.02s)
- New angle = old angle + rate * dt
- Clamped to configured min/max
- Yaw frame conversion handled explicitly

### 2.7 Siyi Protocol (Full Binary Protocol)

Used by: ZR10, ZR30, A2, A8, ZT6, ZT30

**Hardware identification**: Two-byte hardware ID in response to `HARDWARE_ID (0x02)`:
- `'7','5'` = A2; `'7','3'` = A8; `'6','B'` = ZR10; `'7','8'` = ZR30; `'8','3'` = ZT6; `'7','A'` = ZT30

**Frame format**:
```
Byte   Field           Size    Notes
0      STX low         1       0x55
1      STX high        1       0x66
2      CTRL            1       bit0=need_ack, bit1=ack_pack, bits2-7=reserved
3      Data_len low    1       little-endian
4      Data_len high   1       little-endian
5      SEQ low         1       sequence counter, little-endian, wraps at 65535
6      SEQ high        1
7      CMD_ID          1       command identifier
8..N   DATA            N       N = Data_len bytes
N+1    CRC16 low       1       CRC16-CCITT of full packet, little-endian
N+2    CRC16 high      1
```
Minimum packet size: 10 bytes (no data). Maximum: 42 bytes.
CRC algorithm: `crc16_ccitt()` (same as MAVLink CRC).

**Command IDs (SiyiCommandId enum)**:

| Cmd ID | Name | Direction | Notes |
|---|---|---|---|
| 0x01 | ACQUIRE_FIRMWARE_VERSION | Req/Reply | Reply: 8 bytes (A8) or 12 bytes (ZR10+) — cam.major/minor/patch, gimbal.major/minor/patch, zoom.major/minor/patch |
| 0x02 | HARDWARE_ID | Req/Reply | Reply: 2-byte hwid used for model lookup |
| 0x04 | AUTO_FOCUS | Cmd/Ack | Ack: 1-byte result |
| 0x05 | MANUAL_ZOOM_AND_AUTO_FOCUS | Cmd/Reply | Reply: uint16 * 0.1 = zoom multiple |
| 0x06 | MANUAL_FOCUS | Cmd/Ack | Ack: 1-byte result |
| 0x07 | GIMBAL_ROTATION | Cmd | 2 bytes: yaw_scalar (int8 -100..+100), pitch_scalar (int8 -100..+100) |
| 0x08 | CENTER | Cmd/Ack | Center/home gimbal |
| 0x0A | ACQUIRE_GIMBAL_CONFIG_INFO | Req/Reply | Reply: 7-byte `GimbalConfigInfo` struct |
| 0x0B | FUNCTION_FEEDBACK_INFO | Req/Reply | Reply: 1-byte `FunctionFeedbackInfo` |
| 0x0C | PHOTO | Cmd | 1 byte: `PhotoFunction` enum |
| 0x0D | ACQUIRE_GIMBAL_ATTITUDE | Req/Reply | Reply: 12 bytes — yaw,pitch,roll angles (int16 * 0.1 deg each) + yaw,pitch,roll rates (int16 * 0.1 deg/s) — all little-endian |
| 0x0F | ABSOLUTE_ZOOM | Cmd | Set zoom multiple directly |
| 0x11 | SET_CAMERA_IMAGE_TYPE | Cmd | Set dual-camera PIP configuration (ZT30) |
| 0x14 | GET_TEMP_FULL_IMAGE | Req/Reply | Reply: 12 bytes — max_C, min_C (int16 * 0.01), max_pos_x/y, min_pos_x/y (uint16 pixels) |
| 0x15 | READ_RANGEFINDER | Req/Reply | Reply: uint16 = distance in meters |
| 0x1B | SET_THERMAL_PALETTE | Cmd | 1 byte: palette index |
| 0x22 | EXTERNAL_ATTITUDE | Cmd | FC sends vehicle attitude to gimbal |
| 0x30 | SET_TIME | Cmd | 8 bytes: uint64 UTC microseconds (sent up to 5 times at startup) |
| 0x34 | SET_THERMAL_RAW_DATA | Cmd | 0:Disable (30fps), 1:Enable (25fps) |
| 0x38 | SET_THERMAL_GAIN | Cmd | 0:Low gain (50–550°C), 1:High gain (-20–150°C) |
| 0x3E | POSITION_DATA | Cmd | FC sends GPS position to gimbal |

**PhotoFunction enum** (used with PHOTO cmd 0x0C):
- 0: TAKE_PICTURE
- 1: HDR_TOGGLE
- 2: RECORD_VIDEO_TOGGLE
- 3: LOCK_MODE (yaw/pitch/roll all earth-frame)
- 4: FOLLOW_MODE (roll+pitch earth-frame, yaw body-frame)
- 5: FPV_MODE (all body-frame)

**GimbalConfigInfo struct** (7 bytes, reply to 0x0A):
- byte 0: reserved
- byte 1: HdrStatus (0=OFF, 1=ON)
- byte 2: reserved
- byte 3: RecordingStatus (0=OFF, 1=ON, 2=NO_CARD, 3=DATA_LOSS)
- byte 4: GimbalMotionMode (0=LOCK, 1=FOLLOW, 2=FPV)
- byte 5: GimbalMountingDirection (0=UNDEFINED, 1=NORMAL, 2=UPSIDE_DOWN)
- byte 6: VideoOutputStatus (0=HDMI, 1=CVBS)

**Attitude response** (reply to 0x0D, 12 bytes):
- bytes 0-1: yaw (int16, * 0.1 deg) — sign negated to match ArduPilot frame
- bytes 2-3: pitch (int16, * 0.1 deg)
- bytes 4-5: roll (int16, * 0.1 deg)
- bytes 6-7: yaw rate (int16, * 0.1 deg/s) — sign negated
- bytes 8-9: pitch rate (int16, * 0.1 deg/s)
- bytes 10-11: roll rate (int16, * 0.1 deg/s)
- Rotation order for quaternion: 312 (yaw, roll, pitch) via `from_vector312()`

**Angle control (FC-side P-controller)**:
- FC does NOT send an absolute angle command in one message
- FC requests attitude at 20 Hz (every 50 ms)
- FC computes angle error: `pitch_err = target_pitch - current_pitch`
- Converts to rate scalar: `rate_scalar = clamp(100 * err * P / MAX_RATE, -100, +100)` where P=1.5, MAX_RATE=90 deg/s
- Sends GIMBAL_ROTATION (0x07) with the resulting scalar
- For upside-down mounting, applies: `pitch_transformed = -(current_pitch + π)`, `yaw_transformed = -current_yaw`

**Communication timing**:
- Hardware ID request: 1 Hz until received
- Firmware version: 1 Hz until received
- Configuration poll: 1 Hz (ongoing)
- Attitude request: every 50 ms (20 Hz)
- Attitude/position send to gimbal: every 100 ms (10 Hz)
- Rangefinder (ZT30): every 100 ms (10 Hz)
- UTC time: sent up to 5 times at 1-second intervals after startup

### 2.8 STorM32 (Serial Variant) Protocol Detail

**Frame format** (based on SToRM32_serial.h struct analysis):

Request (`cmd_set_angles_struct`):
```
byte  0: byte1    (command byte)
byte  1: byte2
byte  2: byte3
bytes 3-6:  float pitch (little-endian)
bytes 7-10: float roll  (little-endian)
bytes 11-14: float yaw  (little-endian)
byte 15: flags
byte 16: type
bytes 17-18: CRC16
```
Total: 19 bytes.

Reply — data (`SToRM32_reply_data_struct`, 63 bytes + magic):
- uint16 state, status, status2
- uint16 i2c_errors, lipo_voltage, systicks, cycle_time
- int16 imu1_gx/gy/gz (raw gyro)
- int16 imu1_ax/ay/az (raw accel)
- int16 ahrs_x/y/z (AHRS attitude in 0.001 deg)
- int16 imu1_pitch/roll/yaw (actual angles)
- int16 cpid_pitch/roll/yaw (PID output)
- uint16 input_pitch/roll/yaw (RC inputs)
- int16 imu2_pitch/roll/yaw (second IMU)
- int16 mag2_yaw/pitch
- int16 ahrs_imu_confidence
- uint16 function_input_values
- uint16 CRC16
- uint8 magic byte

Reply — ACK (`SToRM32_reply_ack_struct`, 6 bytes):
- byte1, byte2, byte3, data, CRC16 (2 bytes)

The FC reads actual angles from the reply struct's `imu1_pitch/roll/yaw` fields.

### 2.9 Viewpro Protocol Summary

**Frame format**:
```
Byte 0-2: Header 0x55 0xAA 0xDC
Byte 3:   (frame_counter[2] | body_length[6]) — length covers byte3 to checksum inclusive
Byte 4:   Frame ID (command type)
Bytes 5..N: Data
Byte N+1: Checksum (XOR of bytes 3 to N inclusive)
```
Max packet: 63 bytes. CRC: single-byte XOR.

**Key Frame IDs**:

| Frame ID | Name | Direction |
|---|---|---|
| 0x00 | HANDSHAKE | FC→Gimbal |
| 0x01 | U (comm config control) | FC→Gimbal |
| 0x02 | V (comm config status) | Gimbal→FC |
| 0x10 | HEARTBEAT | Gimbal→FC |
| 0x1A | A1 (target angles/rates) | FC→Gimbal |
| 0x1C | C1 (camera controls) | FC→Gimbal |
| 0x1E | E1 (tracking controls) | FC→Gimbal |
| 0x2C | C2 (camera controls less common) | FC→Gimbal |
| 0x2E | E2 (tracking controls2) | FC→Gimbal |
| 0x40 | T1_F1_B1_D1 (status: angles/tracking/recording) | Gimbal→FC |
| 0xB1 | M_AHRS (vehicle attitude+position) | FC→Gimbal |

**A1 packet** (angle/rate control, 8 bytes after header):
- ServoStatus byte: MANUAL_SPEED_MODE (0x01), FOLLOW_YAW (0x03), MANUAL_ABSOLUTE_ANGLE_MODE (0x0B), FOLLOW_YAW_DISABLE (0x0A)
- yaw_be: big-endian int16 (angle or rate)
- pitch_be: big-endian int16

**M_AHRS packet** (vehicle state sent to gimbal):
- data_type byte (0x07 = attitude+GPS+gyro)
- roll, pitch, yaw as be16 (1 unit = 360°/65536)
- GPS date/time, position (lat/lon/height), ground speed N/E/D, vdop

---

## 3. AP_Gripper

### 3.1 Servo vs EPM

Two backend types, selected by `GRIP_TYPE` parameter:

**Servo (AP_Gripper_Servo)**:
- Uses a standard RC servo output channel
- Grab PWM → `GRIP_GRAB` parameter (default configurable)
- Release PWM → `GRIP_RELEASE` parameter
- Neutral PWM → `GRIP_NEUTRAL` parameter
- Actuation time: 500ms for servo to reach position
- Auto-close feature: `autoclose_time` parameter (seconds after release to auto-grab)

**EPM — ElectroMagnetic Permanent Magnet (AP_Gripper_EPM)**:
- Electromagnet controlled by servo channel PWM
- Grab PWM activates electromagnet to latch permanent magnet
- Release PWM reverses field to release
- `regrab_interval` parameter: re-energizes EPM every N seconds to compensate for field weakening from vibration

### 3.2 Gripper States

State machine in `AP_Gripper::gripper_state` enum:

```
STATE_NEUTRAL      → initial/idle state
STATE_GRABBING     → grab command issued, servo/EPM actuating
STATE_GRABBED      → grab complete (after actuation timeout)
STATE_RELEASING    → release command issued, servo/EPM actuating
STATE_RELEASED     → release complete (after actuation timeout)
```

State stored in `Backend_Config::state`. Transitions driven by:
- `grab()` call → sets STATE_GRABBING, moves servo to grab PWM
- `release()` call → sets STATE_RELEASING, moves servo to release PWM
- `update()` at ~10 Hz: checks actuation timeout (500ms), transitions GRABBING→GRABBED, RELEASING→RELEASED

### 3.3 MAVLink Control

Gripper is commanded via `MAV_CMD_DO_GRIPPER`:
- param1: gripper ID (0-indexed)
- param2: 0 = release, 1 = grab

Handled in vehicle/GCS command processing, then calls `AP_Gripper::release()` or `AP_Gripper::grab()`.

Status is reported via `SENSOR_OFFSETS` message or queried via `PARAM_REQUEST_READ`. There is no dedicated gripper status MAVLink message; state is observable through logging only.

---

## 4. AP_Winch

### 4.1 Control Modes

Four control modes (enum `AP_Winch::ControlMode`):

| Mode | Description |
|---|---|
| `RELAXED` | Winch neither resists nor drives — free spin |
| `POSITION` | Maintain or reach a target line length (meters from drum) |
| `RATE` | Extend/retract at a target rate (m/s, +ve = deploying, -ve = retracting) |
| `RATE_FROM_RC` | Rate controlled by RC input |

Transitions:
- `relax()` → RELAXED
- `release_length(float length)` → POSITION, sets `length_desired`
- `set_desired_rate(float rate)` → RATE, sets `rate_desired`

Two physical backends: `AP_Winch_PWM` (simple PWM rate control) and `AP_Winch_Daiwa` (Okaya Daiwa winch with serial telemetry).

**Daiwa winch specifics**:
- Two PWM channels: rate control (high=deploy, low=retract) + clutch (high=free/released, mid=soft-engage, low=hard-engage)
- Serial telemetry output: line length deployed, tension, clutch state
- Options bitmask: SpinFreelyOnStartup, VerboseOutput, RetryIfStuck

**Position PID**: When in POSITION mode, a P controller converts `position_error = length_desired - current_length` to a rate command. P gain = `pos_p` parameter. Rate then passed to underlying PWM rate control.

### 4.2 Integration with Altitude Hold

ArduPilot does not directly integrate winch state with altitude hold in the libraries. The vehicle-level code (ArduCopter) must handle this:
- When winch extends line, payload descends independently of vehicle altitude
- The vehicle may use downward-facing sonar/baro to maintain altitude while line deploys
- There is no automated "fly-down by extending winch" coupling in the library itself — that logic lives in mission/mode code

MAVLink interface: `WINCH_STATUS` message (sent by `send_status()`) carries:
- Time boot ms
- Line length deployed (m)
- Speed (m/s)
- Tension (kg)
- Voltage (V)
- Current (A)
- Temperature (°C)
- Status flags

Commanded via `MAV_CMD_DO_WINCH` with action (relax/rate/length) and param.

---

## 5. AP_Parachute

### 5.1 Activation Logic

The parachute has two activation paths:

**Manual release** (`release()` API):
- Called directly by vehicle code (e.g., mission command, RC aux function, GCS command)
- Can be called from `MAV_CMD_DO_PARACHUTE`

**Automatic — critical sink rate** (`check_sink_rate()` / `set_sink_rate(float sink_rate)`):
- Vehicle code calls `set_sink_rate()` at each EKF update with the current vertical velocity
- If `sink_rate > _critical_sink` parameter (m/s) and vehicle is flying, starts a 1-second timer
- If condition persists for > 1000ms, `release()` is called automatically
- Sink rate timer resets if: critical_sink <= 0 (disabled), vehicle not flying, or sink_rate drops below threshold

### 5.2 Motor Shutdown

On `release()` call:
1. Logs `PARACHUTE_RELEASED` event
2. Calls `AP::arming().disarm(AP_Arming::Method::PARACHUTE_RELEASE)` — this stops all motors immediately via the arming system
3. Optional: `SkipDisarmBeforeParachuteRelease` option bit skips step 2 (for fixed-wing where motors may need to keep running)
4. Sets `_release_time = AP_HAL::millis()`

### 5.3 Safety State Machine

Parachute uses a time-sequenced state machine (evaluated in `update()` at ~10 Hz):

**States** (flags, not an enum):
- `_release_initiated` — set on first `release()` call
- `_release_in_progress` — set when the servo/relay actually fires
- `_released` — set permanently after first firing

**Timing** (two phases):
1. **Delay phase** (default 500ms, `DELAY_MS` param): motors are stopped but chute not yet fired. Allows motors to fully stop before deployment.
2. **Release phase** (2000ms, `AP_PARACHUTE_RELEASE_DURATION_MS`): servo moves to `_servo_on_pwm` or relay is energized
3. **Reset**: after delay + release duration, servo returns to `_servo_off_pwm` and relay is de-energized

**Safety guards in arming checks** (`arming_checks()`):
- If servo type: verifies `SRV_Channel::k_parachute_release` is assigned
- If relay type: verifies relay function `PARACHUTE` is configured
- If `_release_initiated` is already true: blocks re-arm

**Minimum altitude**: Enforced by vehicle code, not the library. The `alt_min()` accessor returns the configured minimum altitude above home. The vehicle must check `alt_min()` before calling `release()`.

**Options bitmask**:
- bit 0: `HoldOpen` — servo/relay stays active forever after release (no reset)
- bit 1: `SkipDisarmBeforeParachuteRelease` — do not disarm motors

**Trigger mechanisms**: Relay (4 options, relay 0-3) or Servo. In 4.5+, relay is selected by `RELAYx_FUNCTION = PARACHUTE`.

---

## 6. AP_LandingGear

### 6.1 Servo-based Extend/Retract

Landing gear is controlled via two SRV_Channel functions:
- `k_landing_gear_control` — main gear control channel (deploy = high PWM, retract = low PWM)
- Actual PWM values set by servo trim/min/max in servo configuration

`deploy()` sets the servo to the deploy position; `retract()` sets it to the retract position.

Internal state tracking:
- `_deployed` bool (true = deployed)
- `_have_changed` bool (tracks whether servo has been commanded since boot)

### 6.2 Auto Deploy/Retract

Controlled by altitude triggers and OPTIONS parameter:

**HEIGHT-based logic** (`update(float height_above_ground_m)`):
- `_deploy_alt_m` parameter: deploy gear when descending below this altitude
- `_retract_alt_m` parameter: retract gear when climbing above this altitude
- Called by vehicle code with rangefinder/baro-based height estimate

**Options bitmask** (`AP_Int16 _options`):
- bit 0: `RETRACT_AFTER_TAKEOFF` — retract gear after takeoff climb
- bit 1: `DEPLOY_DURING_LANDING` — auto-deploy before landing

**Startup behavior** (`_startup_behaviour` parameter):
- 0: `WaitForPilotInput` — no action at boot
- 1: `Retract` — auto-retract at boot
- 2: `Deploy` — auto-deploy at boot

### 6.3 Weight-on-Wheels

**WoW sensor input**: Two debounced digital inputs controlled by parameters:
- `_pin_deployed` — GPIO pin indicating gear is deployed (with polarity param)
- `_pin_weight_on_wheels` — GPIO pin indicating weight is on wheels (with polarity param)

**WoW state enum**:
- `LG_WOW_UNKNOWN = -1`
- `LG_NO_WOW = 0`
- `LG_WOW = 1`

**Gear state enum**:
- `LG_UNKNOWN = -1`
- `LG_RETRACTED = 0`
- `LG_DEPLOYED = 1`
- `LG_RETRACTING = 2`
- `LG_DEPLOYING = 3`

Both states use debouncing to filter transient noise. Both state changes are logged via `log_wow_state()`.

`check_before_land()` verifies gear is deployed before landing (returns false if not deployed, vehicle code can abort landing).

---

## 7. AC_PrecLand + AP_IRLock

### 7.1 Architecture

`AC_PrecLand` is a singleton that provides a position estimate of a landing target relative to the vehicle. It consists of:

**Frontend** (`AC_PrecLand`):
- Manages a single active `AC_PrecLand_Backend`
- Runs a 2-axis Kalman filter (`PosVelEKF _ekf_x`, `PosVelEKF _ekf_y`) for smoothing
- Maintains target state machine
- Exposes target position relative to vehicle for use by position controller

**State machine** (`AC_PrecLand_StateMachine`):
- States: `TARGET_NEVER_SEEN`, `TARGET_OUT_OF_RANGE`, `TARGET_RECENTLY_LOST`, `TARGET_FOUND`
- Configurable retry strictness, max retries, timeout behavior

**Kalman filter** (`PosVelEKF`):
- 2-state (position, velocity) per axis
- Predict: called at 400 Hz with IMU delta-velocity
- Fuse: called when new sensor measurement arrives
- Covariance matrix: 3-element symmetric form `{P[0,0], P[0,1], P[1,1]}`
- Outlier rejection: normalized innovation squared (NIS); 3 consecutive outliers → force accept

### 7.2 Sensor Backends

Four backends selected by `PLND_TYPE` parameter:

| Type | Name | Interface |
|---|---|---|
| 0 | NONE | — |
| 1 | MAVLINK | `LANDING_TARGET` MAVLink message from companion computer or offboard |
| 2 | IRLOCK | I2C sensor (AP_IRLock) |
| 3 | SITL_GAZEBO | Gazebo simulation |
| 4 | SITL | ArduPilot SITL simulation |

**IRLock** (`AP_IRLock_I2C`):
- PixArt PAY2803 or Pixy2-compatible I2C device
- Outputs angular position of target: `pos_x` and `pos_y` in units of `tan(theta)` from image center
- `get_unit_vector_body()` converts to a 3D unit vector in body frame
- Default I2C bus selected by `PLND_BUS` parameter

**MAVLink** (`AC_PrecLand_MAVLink`):
- Receives `LANDING_TARGET` MAVLink messages
- Supports both frame types: `MAV_FRAME_BODY_FRD` (forward-right-down relative to vehicle) and `MAV_FRAME_LOCAL_FRD` (aligned to vehicle heading in horizontal plane)
- Message must contain: angle_x, angle_y (if using angular measurement) or position_valid + x/y/z

### 7.3 Landing Approach — Guiding Final Descent

The precision landing system feeds position offsets into the copter's position controller:

**Data flow**:
1. Backend measures target position (angle or position vector)
2. `construct_pos_meas_using_rangefinder()` converts angular measurement to NED position using rangefinder altitude
3. EKF predicts at 400 Hz using IMU, fuses measurements when available
4. `run_output_prediction()` compensates for sensor lag (`_lag_s` param, default 20ms) by propagating the estimate forward using stored inertial history
5. `get_target_position_relative_NE_m()` and `get_target_velocity_relative_NE_ms()` expose the result to vehicle position controller

**Descent control** (XY error limit):
- Vehicle only descends vertically if horizontal error < `XY_DIST_MAX` parameter (default 2.5m)
- This prevents overshooting the target at high altitude

**Retry logic** (on target loss):
- `PLND_STRICT` parameter: 0=land vertically (non-strict), 1=retry landing, 2=hover (very strict)
- `PLND_RET_MAX` parameter: max retries
- `PLND_RET_TIMEOUT` parameter: how long to continue descending after target loss before retry

**Target last-known position**:
- `_last_target_pos_rel_origin_ned_m` stores the last known target position in NED relative to EKF origin
- `_last_vehicle_pos_ned_m` stores vehicle position at last detection
- Used for retry logic and "target recently lost" state

### 7.4 EKF Integration

**PrecLand does NOT feed back into the main EKF** (EKF2/EKF3). The two systems are separate:
- The main EKF (EKF3) uses GPS, barometer, magnetometer, IMU for vehicle state estimation
- The PrecLand EKF (`PosVelEKF`) estimates only the landing *target's* relative position
- PrecLand uses the main EKF's output (vehicle velocity and attitude) as inputs for its prediction step

**Inertial history buffer** (`_inertial_history`):
- Ring buffer of `inertial_data_frame_s` structs sampled at 400 Hz
- Each entry: DCM rotation matrix Tbn, delta velocity (corrected), velocity, dt, timestamp
- Used to propagate the PrecLand estimate forward by `_lag_s` to compensate for sensor latency
- The "delayed" inertial data entry is selected by matching timestamp to sensor measurement time

**Camera offset compensation**:
- `CAM_POS` parameter (x, y, z in body frame) — position of camera relative to CG
- Applied when converting angular measurement to NED position vector

---

## 8. Meridian Port Notes

### 8.1 AP_Camera → Meridian

**Rust trait design**:
```rust
trait CameraBackend {
    fn trigger_pic(&mut self) -> bool;
    fn record_video(&mut self, start: bool) -> bool { false }
    fn set_zoom(&mut self, zoom_type: ZoomType, value: f32) -> bool { false }
    fn set_focus(&mut self, focus_type: FocusType, value: f32) -> SetFocusResult { SetFocusResult::Unsupported }
    fn set_tracking(&mut self, tracking_type: TrackingType, p1: Vector2, p2: Vector2) -> bool { false }
    fn update(&mut self);
}
```

**Distance triggering**: Needs `Location::get_distance()`, `AHRS::get_location()`, roll check. Keep as standalone update function called at 50 Hz.

**Geo-tagging**: On `trigger_pic()` success, capture GPS time + lat/lon/alt + attitude from AHRS. Send `CAMERA_FEEDBACK` MAVLink. Hardware feedback pin path requires GPIO ISR support.

**MAVLink Camera v2**: Implement `CAMERA_INFORMATION`, `CAMERA_SETTINGS`, `CAMERA_CAPTURE_STATUS`, `VIDEO_STREAM_INFORMATION` senders. The discovery loop (scanning routing table for camera component) maps to iterating MAVLink routing entries.

**Key parameters** (per instance): `type`, `trigg_dist`, `interval_min`, `servo_on_pwm`, `servo_off_pwm`, `feedback_pin`, `feedback_polarity`, `hfov`, `vfov`, `mount_instance`, `options`.

### 8.2 AP_Mount → Meridian

**Priority backend**: Implement Siyi first (most popular on commercial drones). Then MAVLink v2 (generic), then Servo (simplest).

**Trait hierarchy**:
```rust
trait MountBackend {
    fn has_pan_control(&self) -> bool;
    fn get_attitude_quaternion(&self) -> Option<Quaternion>;
    fn update(&mut self);
    fn send_target_angles(&mut self, target: &MountAngleTarget) {}
    fn send_target_rates(&mut self, target: &MountRateTarget) {}
    fn natively_supported_target_types(&self) -> u8;
}
```

**Rate/angle conversion**: The `send_target_to_gimbal()` → `update_angle_target_from_rate()` logic must be ported. It integrates rate to angle at 50 Hz update rate.

**ROI/GPS targeting**: `get_angle_target_to_location()` requires bearing + pitch from vehicle to target using `Location::get_distance()`, `atan2()`. All angles in radians, earth-frame.

**MAV_MOUNT_MODE enum**: Port all 7 modes. The mode state machine in `update_mnt_target()` (1200+ line function in `AP_Mount_Backend.cpp`) is the core of the system.

**Siyi implementation checklist**:
- [ ] Parser state machine (10 states, CRC16-CCITT on full packet)
- [ ] Command encoding: `send_packet(cmd_id, data, len)` with header 0x55 0x66, CTRL=1, little-endian seq/len/CRC
- [ ] Startup sequence: HARDWARE_ID → FIRMWARE_VERSION → CONFIGURATION
- [ ] Attitude poll at 20 Hz (0x0D)
- [ ] Attitude send at 10 Hz (0x22 EXTERNAL_ATTITUDE)
- [ ] Rate control: map to 0x07 GIMBAL_ROTATION (-100..+100 scalars)
- [ ] Angle control: P-controller with P=1.5, max_rate=90 deg/s → rate scalars
- [ ] Motion mode management: LOCK/FOLLOW/FPV via 0x0C PHOTO command
- [ ] Upside-down mounting compensation
- [ ] UTC time send at startup (0x30, up to 5 times)
- [ ] Model-specific capabilities (ZT30: rangefinder, thermal; ZR30: 30x zoom)

**SToRM32 serial struct**: The full 63-byte reply struct must be byte-exact. Use `#[repr(C, packed)]` in Rust.

**Viewpro M_AHRS packet**: The vehicle attitude/position packet is large (big-endian throughout) — use explicit `be16/be32` types.

### 8.3 AP_Gripper → Meridian

Simple state machine with servo PWM output. Two parameters: grab_pwm, release_pwm, neutral_pwm. 500ms actuation timer. EPM variant adds a regrab timer.

MAVLink command: `MAV_CMD_DO_GRIPPER` (param1=instance, param2=0/1).

### 8.4 AP_Winch → Meridian

Three modes: RELAXED, POSITION (P-controller), RATE. Daiwa variant requires UART telemetry parser. MAVLink `WINCH_STATUS` message.

Position controller: `rate = pos_p * (length_desired - current_length)`, clamped to `rate_max`.

### 8.5 AP_Parachute → Meridian

Critical path — safety system. Must implement:
- Sink-rate trigger (1-second sustained threshold)
- Motor disarm before deployment (call into arming system)
- Delay phase (configurable, default 500ms)
- Release hold (2000ms) then auto-reset
- Relay or servo output
- Altitude minimum guard (enforced by vehicle, not library)

### 8.6 AP_LandingGear → Meridian

Servo channel output + two GPIO inputs (deployed sense, WoW sense) with debouncing. Altitude-triggered auto-deploy/retract. State machine with 4 gear states + 3 WoW states.

### 8.7 AC_PrecLand → Meridian

Most complex payload component after AP_Mount. Key items:

**PosVelEKF**: 2-state linear Kalman filter (position + velocity). Simple enough to implement in ~50 lines of Rust. The covariance update is the symmetric 2x2 form.

**Inertial history buffer**: Ring buffer at 400 Hz for lag compensation. Size = lag / dt = 0.02s / 0.0025s = 8 entries minimum.

**IRLock I2C**: `pos_x`/`pos_y` in tan(theta) units → unit vector in body frame. Conversion: `Vector3f { x: 1.0, y: pos_x, z: pos_y }.normalize()` (approximately, with correct sign conventions).

**MAVLink LANDING_TARGET**: Parse frame type, extract angular or position measurement, fuse into EKF.

**Position controller interface**: Export `target_position_ne_m()` and `target_velocity_ne_ms()` for use by position controller. Vehicle code must handle XY descent inhibit (don't descend if error > XY_DIST_MAX).

**EKF isolation**: PrecLand EKF is completely separate from the main vehicle EKF. No data flows back into EKF3.

---

## Source Files Read

This audit covered the following source files directly:

1. `AP_Camera/AP_Camera.h`
2. `AP_Camera/AP_Camera.cpp` (partial)
3. `AP_Camera/AP_Camera_Backend.h`
4. `AP_Camera/AP_Camera_Backend.cpp`
5. `AP_Camera/AP_Camera_Params.h`
6. `AP_Camera/AP_Camera_MAVLinkCamV2.h`
7. `AP_Mount/AP_Mount.h`
8. `AP_Mount/AP_Mount_Backend.h`
9. `AP_Mount/AP_Mount_Backend.cpp` (multiple sections)
10. `AP_Mount/AP_Mount_Siyi.h`
11. `AP_Mount/AP_Mount_Siyi.cpp` (full)
12. `AP_Mount/AP_Mount_SToRM32.h`
13. `AP_Mount/AP_Mount_SToRM32_serial.h`
14. `AP_Mount/AP_Mount_Alexmos.h`
15. `AP_Mount/AP_Mount_MAVLink.h`
16. `AP_Mount/AP_Mount_Servo.h`
17. `AP_Mount/AP_Mount_Viewpro.h`
18. `AP_Mount/AP_Mount_Topotek.h` (partial)
19. `AP_Gripper/AP_Gripper.h`
20. `AP_Gripper/AP_Gripper_Servo.h`
21. `AP_Winch/AP_Winch.h`
22. `AP_Winch/AP_Winch_Daiwa.h` (partial)
23. `AP_Parachute/AP_Parachute.h`
24. `AP_Parachute/AP_Parachute.cpp`
25. `AP_LandingGear/AP_LandingGear.h`
26. `AC_PrecLand/AC_PrecLand.h`
27. `AC_PrecLand/AC_PrecLand.cpp` (partial)
28. `AC_PrecLand/PosVelEKF.h`
29. `AP_IRLock/AP_IRLock.h`
