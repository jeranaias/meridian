# Parity Verification: Payload, OSD, and Compliance Subsystems

**Generated**: 2026-04-02  
**Method**: Meridian source read (all crate .rs files) cross-referenced against ArduPilot source audit docs  
**ArduPilot reference**: `docs/audit_wave3_payload.md`, `docs/audit_wave3_osd_notify.md`  
**Meridian source**: `crates/meridian-{camera,mount,precland,osd,notify,adsb,opendroneid,autotune,fft,proximity}/`

---

## 1. Camera — AP_Camera vs meridian-camera

### 1.1 Backend count

| | ArduPilot | Meridian |
|---|---|---|
| **Backends** | 9 (NONE, SERVO, RELAY, SOLOGIMBAL, MOUNT, MAVLINK, MAVLINK_CAMV2, SCRIPTING, RUNCAM) | **2** (ServoCameraBackend, RelayCameraBackend) |
| Max simultaneous | 2 instances | 1 instance |

**Gap**: 7 of 9 ArduPilot backends are absent — SoloGimbal, Mount-delegate, MAVLink legacy, MAVLink Camera Protocol v2, Scripting (Lua), and RunCam serial are all missing.

### 1.2 Triggering

| Feature | ArduPilot | Meridian |
|---|---|---|
| Distance triggering | Yes — `trigg_dist` param, 50 Hz evaluation with minimum-interval guard and AUTO-mode-only option | **Yes** — `TriggerMode::Distance`, equirectangular distance, fires first shot immediately |
| Time triggering | Yes — `time_interval_settings`, supports finite count or infinite, minimum interval guard | **Yes** — `TriggerMode::Time`, millisecond interval |
| Burst / finite count | Yes — `take_multiple_pictures(interval_ms, total_num)`, `total_num = -1` = infinite | **No** — time mode runs indefinitely with no shot-count limit |
| Minimum interval guard | Yes — `interval_min` param, pending trigger mechanism | **No** |
| AUTO mode only flag | Yes — `_auto_mode_only` guard in distance mode | **No** |
| Roll-limit guard | Yes — skip trigger if roll exceeds `max_roll` | **No** |

### 1.3 Geotagging

| Feature | ArduPilot | Meridian |
|---|---|---|
| Software-trigger geotag | Yes — `prep_mavlink_msg_camera_feedback()`, logs lat/lon/alt-MSL/alt-AGL/roll/pitch/yaw/image-index | **Yes** — `GeotagEntry` captures lat/lon/alt/roll/pitch/yaw/timestamp |
| Hardware feedback-pin path | Yes — GPIO ISR timestamps trigger independently of software delay | **No** — no feedback pin / ISR path |
| Geotag log format | MAVLink `CAMERA_FEEDBACK` message + DataFlash log | Ring buffer (last 64 entries), no MAVLink emission |
| Geotag AGL altitude | Yes (separate alt-AGL field) | **No** — single `alt` field, no AGL distinction |

### 1.4 MAVLink Camera Protocol v2

**Present in Meridian**: No.  
ArduPilot's `AP_Camera_MAVLinkCamV2` handles `CAMERA_INFORMATION`, `CAMERA_SETTINGS`, `CAMERA_CAPTURE_STATUS`, `CAMERA_FOV_STATUS` messages and proxies zoom/focus/tracking commands. Meridian has no MAVLink camera protocol of any version.

### 1.5 Other missing camera features

- Focus control (RATE / PCT / AUTO)
- Zoom control (RATE / PCT)
- Object tracking (`TrackingType`: NONE/POINT/RECTANGLE)
- Lens selection (0–5)
- Camera source / dual-sensor selection (RGB / IR / NDVI / wideangle)
- Mode toggle (image ↔ video)
- Video stream information
- Camera thermal range

---

## 2. Mount/Gimbal — AP_Mount vs meridian-mount

### 2.1 Backend count

| | ArduPilot | Meridian |
|---|---|---|
| **Backends** | 14 (Servo, SoloGimbal, Alexmos, SToRM32-MAVLink, SToRM32-serial, MAVLink-v2, BrushlessPWM, Siyi, Scripting, Xacti, Viewpro, Topotek, CADDX, XFRobot) | **3** (ServoMount, MavlinkMount, SiyiMount) |

**Gap**: 11 of 14 ArduPilot backends are absent: SoloGimbal, Alexmos/SimpleBGC, SToRM32 MAVLink, SToRM32 serial, MAVLink Gimbal Protocol v2, BrushlessPWM, Scripting, Xacti (DroneCAN), Viewpro, Topotek, CADDX, XFRobot.

### 2.2 Siyi P-controller (angle → rate)

**Present in Meridian**: **Yes.**  
`siyi.rs` implements `angle_to_rate_scalar()` with `P = 1.5`, max rate `90 deg/s`, output clamped to `−100..+100`. This exactly matches ArduPilot's Siyi implementation.

**Gap**: ArduPilot requests attitude at 20 Hz (every 50ms) and sends position/AHRS data to the gimbal at 10 Hz. Meridian's SiyiMount has `build_attitude_request()` but no automatic 20 Hz polling loop or position data send (`0x22 EXTERNAL_ATTITUDE`, `0x3E POSITION_DATA` commands are absent).

ArduPilot also handles upside-down mounting transforms (`pitch_transformed = -(current_pitch + π)`, `yaw_transformed = -current_yaw`). Meridian does not.

### 2.3 STorM32 serial protocol

**Present in Meridian**: **No.**  
STorM32 binary serial (`cmd_set_angles_struct`, `SToRM32_reply_data_struct`) is absent. Neither the MAVLink-routed variant (Type 4) nor the serial-binary variant (Type 5) is implemented.

### 2.4 RC targeting

**Present in Meridian**: **Yes (partial).**  
`MountMode::RcTargeting` is defined and `update_rc_targeting()` converts normalized RC input (`−1..+1`) to rate commands. However, ArduPilot's RC targeting supports both rate mode (`rc_rate_max > 0`) and angle mode (`rc_rate_max <= 0`) via configurable parameters, plus earth-frame/body-frame yaw/pitch/roll locking per axis (`_yaw_lock`, `_pitch_lock`, `_roll_lock`, FPV_LOCK option). Meridian only implements rate mode with no frame-lock flags.

### 2.5 GPS ROI targeting

**Present in Meridian**: **Yes (partial).**  
`MountMode::GpsTargeting` is defined and `set_roi_target()` stores a `MountTarget::RoiLocation`. However, the bearing and pitch angle computation (atan2 of lat/lon difference → yaw, atan2 of height / horizontal distance → pitch) that converts the GPS coordinate to a mount angle target is **not implemented** in `MountManager`. The backend receives `RoiLocation` but comments say "ROI resolved to angle by MountManager" — this resolution is a stub (no math is present).

ArduPilot also supports:
- `MountMode::SysidTargeting` (track another MAVLink sysid) — **absent**
- `MountMode::HomeLocation` — **absent**
- POI Lock with terrain intersection — **absent**

### 2.6 Other missing mount features

- FC stabilization compensation (adding AHRS lean angles for servo-type gimbals)
- BrushlessPWM (no-FC-stabilization servo variant)
- MAVLink Gimbal Protocol v2 (GIMBAL_DEVICE_SET_ATTITUDE quaternion interface)
- AUTOPILOT_STATE_FOR_GIMBAL_DEVICE broadcast to gimbals
- Dual mount instances
- Siyi upside-down mounting transform

---

## 3. Precision Landing — AC_PrecLand vs meridian-precland

### 3.1 Per-axis 2-state Kalman filter

**Present in Meridian**: **Yes.**  
`kalman.rs` implements `PosVelKF` — a 2-state (position, velocity) filter with explicit predict/update steps and 2×2 covariance matrix. Two instances are used (one per axis). This matches ArduPilot's `PosVelEKF_x` / `PosVelEKF_y` structure.

**Gap**: ArduPilot's KF includes outlier rejection via Normalized Innovation Squared (NIS) — 3 consecutive outliers force an accept. Meridian's KF has no NIS outlier rejection.

ArduPilot runs predict at **400 Hz** using IMU delta-velocity. Meridian's KF has no integration with an IMU update loop — it must be driven externally.

### 3.2 Inertial history lag compensation

**Present in Meridian**: **Yes.**  
`inertial_history.rs` implements `InertialHistory<N>` — a ring buffer of `(vel_x, vel_y, timestamp_ms)` samples with `compute_displacement(from_ms, to_ms)` that integrates velocity over a time window to compensate for sensor lag. This matches ArduPilot's lag compensation architecture.

**Gap**: ArduPilot uses a configurable lag parameter (`PLND_LAG_S`, default 20ms). Meridian has the buffer but no `lag_s` parameter integration — the caller must determine what timestamps to pass.

### 3.3 Backend count

| | ArduPilot | Meridian |
|---|---|---|
| **Backends** | 4 (NONE, MAVLINK, IRLOCK, SITL_GAZEBO, SITL) | **3** (IRLockBackend, MavlinkLandingBackend, SitlLandingBackend) |

SITL_Gazebo backend is absent. IRLock and MAVLink backends are present.

### 3.4 Other missing precision landing features

- Target state machine (`TARGET_NEVER_FOUND → TARGET_OUT_OF_RANGE → TARGET_RECENTLY_LOST → TARGET_FOUND`)
- Retry logic (`PLND_STRICT`, `PLND_RET_MAX`, `PLND_RET_TIMEOUT`)
- XY distance gate (do not descend if horizontal error > `XY_DIST_MAX`)
- `construct_pos_meas_using_rangefinder()` — angle-to-NED conversion using rangefinder altitude
- `run_output_prediction()` — forward propagation of estimate to correct for `lag_s`
- `get_target_position_relative_NE_m()` / `get_target_velocity_relative_NE_ms()` output interface
- MAVLink `LANDING_TARGET` frame type support (BODY_FRD vs LOCAL_FRD)
- NIS outlier rejection in KF

---

## 4. Parachute — AP_Parachute vs meridian

### 4.1 Parachute subsystem present

**Present in Meridian**: **No.**  
There is no `meridian-parachute` crate and no parachute-related code in any crate.

### 4.2 Motor shutdown → 500ms delay → servo pulse sequence

**Present in Meridian**: **No** (subsystem absent).

ArduPilot's sequence:
1. `release()` → call `AP::arming().disarm(PARACHUTE_RELEASE)` — motors stop immediately
2. Log `PARACHUTE_RELEASED` event
3. Delay phase (default 500ms, configurable `DELAY_MS`) — chute not yet fired
4. Release phase (2000ms) — servo moves to `_servo_on_pwm` or relay energizes
5. Reset — servo returns to `_servo_off_pwm`

**This entire subsystem is a gap.**

---

## 5. Other Payload — Gripper, Winch, Landing Gear, Sprayer

| Subsystem | ArduPilot | Meridian | Gap |
|---|---|---|---|
| **Gripper** | AP_Gripper — Servo + EPM backends, grab/release state machine, MAVLink `DO_GRIPPER` | **Absent** | Full gap |
| **Winch** | AP_Winch — RELAXED/POSITION/RATE/RATE_FROM_RC modes, Daiwa serial telemetry backend, `WINCH_STATUS` MAVLink | **Absent** | Full gap |
| **Landing Gear** | AP_LandingGear — deploy/retract servo, auto altitude triggers, WoW sensor, gear state machine | **Absent** | Full gap |
| **Sprayer** | AP_Sprayer — pump + spinner PWM, ground speed-proportional flow, GPS coverage area | **Absent** | Full gap |

All four payload actuator subsystems are unimplemented in Meridian.

---

## 6. OSD — AP_OSD vs meridian-osd

### 6.1 OSD item count

| | ArduPilot | Meridian |
|---|---|---|
| **Standard items** | 56 (defined in `AP_OSD_Screen var_info`) | **50** (OsdItem enum) |
| Extended link stats | 5 (CRSF only) | 0 |
| MSP-only items | 7 | 0 |
| **Total** | ~68 | **50** |

Items present in ArduPilot but absent from Meridian's `OsdItem` enum:

- `xtrack_error` — cross-track error
- `pluscode` — Open Location Code (conditional)
- `aspd1` / `aspd2` — dual airspeed sensors
- `sidebars` — speed/altitude sidebars with tick marks
- `restvolt` — resting/idle battery voltage
- `avgcellrestvolt` — average cell resting voltage
- `bat2_vlt` / `bat2used` / `current2` — secondary battery
- `climbeff` — climb efficiency
- `esc_amps` — ESC current
- `rrpm` — AP_RPM rotor/motor RPM
- `vtx_power` — video transmitter power
- `rc_tx_power` / `rc_rssi_dbm` / `rc_snr` / `rc_active_antenna` / `rc_lq` — extended link stats
- `fence` — geo-fence status
- `stat` — post-flight statistics summary item (Meridian has stats screens but no `stat` summary item)
- `btemp` / `atemp` — second baro and airspeed sensor temperature
- `hgt_abvterr` — terrain-relative height

Meridian adds some items not enumerated in ArduPilot's standard set (`GpsSpeed3D`, `VerticalSpeed` as separate items), but these overlap with `gspeed`/`vspeed`.

### 6.2 MAX7456 register writes

**Present in Meridian**: **Yes, correct.**  
Register addresses match: `VM0=0x00`, `VM1=0x01`, `DMM=0x04`, `DMAH=0x05`, `DMAL=0x06`, `DMDI=0x07`, `OSDBL=0x6C`.  
Init sequence: soft reset → disable OSD → auto black-level → set video standard → set VM1. This matches the ArduPilot MAX7456 init sequence.

Auto-increment write with `END_STRING=0xFF` terminator is correctly implemented.

**Gap**: ArduPilot loads custom font data from ROMFS/SD card (`load_font_data()`) and uploads individual characters to MAX7456 NVM. Meridian has no font upload mechanism — it relies on the chip's ROM font only. The ArduPilot symbol table (107 symbols mapped to font positions) is also absent.

### 6.3 MSP DisplayPort protocol

**Present in Meridian**: **Yes, correct.**  
MSP v1 frame format (`$M<` + size + cmd + payload + XOR checksum) is correctly implemented. DisplayPort sub-commands match: HEARTBEAT(0), CLEAR(2), WRITE_STRING(3), DRAW(4).

**Gaps**:
- Meridian sends heartbeat only at `init()`. ArduPilot sends periodic heartbeats to maintain display control.
- Meridian has no INAV symbol mapping table (`ap_to_inav_symbols_map[256][2]`). ArduPilot's MSP backend translates ArduPilot internal character codes to INAV-compatible codes for goggle compatibility.
- Meridian uses fixed 30×16 grid. ArduPilot supports HD resolutions (50×18, 53×20) with `txt_resolution` and `font_index` per-screen parameters.
- Meridian has no blink state tracking (`_blink_on`).
- Meridian does not send MSP `RELEASE` sub-command (1) on cleanup.

### 6.4 Multi-screen switching

**Present in Meridian**: **Yes (partial).**  
`OsdManager` has 4 flight screens + 2 stats screens (6 total). Screen selection from RC PWM is implemented (`select_screen_from_pwm()`).

**Gaps**:
- ArduPilot supports 3 switching methods: TOGGLE (each transition), PWM_RANGE, and AUTO_SWITCH (armed/failsafe/disarmed each map to a screen). Meridian only supports PWM_RANGE mode.
- ArduPilot has per-screen `CHAN_MIN`/`CHAN_MAX` configurable PWM windows. Meridian uses fixed hardcoded ranges (0–1249/1250–1499/1500–1749/1750+).
- ArduPilot has a 300ms debounce timer for screen switching. Meridian has no debounce.
- ArduPilot has 2 parameter screens (`AP_OSD_ParamScreen`) for in-flight parameter adjustment via RC sticks. Meridian has no parameter screens.

### 6.5 Stats screens

**Present in Meridian**: **Yes (partial).**  
`OsdManager` has 2 stats screen slots (`stats_screen_mut()`). Stats items (`MaxAltitude`, `MaxSpeed`, `MaxDistance`) render `"--"` as a placeholder.

**Gap**: The max-value tracking state (actual max altitude, max speed, max distance accumulated during flight) is not implemented. Items render `"--"` unconditionally. ArduPilot tracks these across the flight and renders them on the post-flight stats screen.

### 6.6 Other missing OSD features

- SITL/SFML backend (simulation display)
- TXONLY backend (for CRSF external OSD)
- Dual simultaneous backends (ArduPilot supports `osd_type` + `osd_type2`)
- Unit system selection (METRIC / IMPERIAL / SI / AVIATION)
- Warning thresholds with visual alerts (`OSD_W_RSSI`, `OSD_W_BATVOLT`, etc.)
- Global position offsets (`OSD_V_OFFSET`, `OSD_H_OFFSET`)
- Decimal-pack compression for narrow screens
- Font loading/upload from SD card

---

## 7. Notification — AP_Notify vs meridian-notify

### 7.1 LED patterns

| | ArduPilot | Meridian |
|---|---|---|
| **Defined patterns** | 12 named sequences (initialising, trim/ESC, failsafe variants × 5, armed, armed-no-GPS, prearm-failing, disarmed-good-DGPS, disarmed-good-GPS, disarmed-bad-GPS) | **7** (DISARMED, ARMED, FAILSAFE, GPS_SEARCHING, GPS_LOCKED, LOW_BATTERY, CALIBRATING) |

**Missing patterns from ArduPilot**:
- Initializing (alternating RED/BLUE)
- ESC calibration / save trim pattern
- EKF failsafe variant (failsafe + RED)
- Leak failsafe (failsafe + WHITE)
- GPS glitching failsafe (failsafe + BLUE)
- Pre-arm check failing (double YELLOW blink)
- Armed with no GPS (solid BLUE variant)
- Disarmed + good DGPS (alternating GREEN/BLACK)

Meridian has LOW_BATTERY and CALIBRATING patterns not enumerated separately in ArduPilot's named sequences (though ArduPilot expresses these through the same waterfall with different colors).

### 7.2 Buzzer tunes

| | ArduPilot | Meridian |
|---|---|---|
| **Defined tunes** | 32 ToneAlarm MML tunes + simple buzzer patterns | **6** (arm, disarm, gps_fix, low_battery, failsafe, calibration_complete) |

ArduPilot's ToneAlarm uses MML (Music Macro Language) strings allowing full musical melodies. Meridian uses simple note arrays. 26 ArduPilot tunes have no Meridian equivalent including: startup melody, waypoint complete, EKF alert, battery continuous siren, vehicle lost, land warning, autotune complete/failed/next-axis, compass cal events, mission complete, no SD card, shutdown, and all 7 `LOUD_N` one-note tones.

ArduPilot supports custom tunes via MAVLink `PLAY_TUNE` message and Lua scripting. Meridian does not.

### 7.3 Priority system

**Present in Meridian**: **Yes.**  
`NotifyPriority` enum (Info=0 through Failsafe=5) correctly preempts lower-priority tunes and transitions LED patterns. ArduPilot uses a simpler waterfall (no numeric priority — pure if-else cascade). Both achieve the same result; Meridian's explicit priority enum is functionally equivalent or better.

### 7.4 NeoPixel/WS2812 output

**Present in Meridian**: **Yes (protocol-level).**  
The notify crate is documented as driving "WS2812 / NeoPixel LEDs" and the `NotifyManager::update()` returns RGB values per tick for the caller to apply to hardware. The HAL layer (`meridian-hal`) does not contain a WS2812 SPI/DMA bit-banging driver — there is no `gpio.rs` or `spi.rs` WS2812 protocol encoder. The RGB values are computed but the physical LED strip output path is not implemented.

ArduPilot has `NeoPixel` / `NeoPixelRGB` device classes with full WS2812B bit-timing SPI output, ProfiLED support, and DShot buzzer integration. Meridian has none of these physical driver implementations.

### 7.5 Other missing notify features

- Board LED GPIO drivers (ArduPilot has ~12 board-specific LED drivers)
- I2C LED controllers (ToshibaLED, NCP5623, LP5562, IS31FL3195, PCA9685, OreoLED)
- OLED display drivers (SSD1306, SH1106)
- DShotLED (ESC LED color via DShot protocol)
- DroneCAN RGB LED
- DroneCAN buzzer
- LED_OVERRIDE MAVLink bypass mode
- NTF_LED_BRIGHT brightness levels
- Separate continuous vs one-shot tune categories
- Multiple simultaneous notification devices (ArduPilot: up to 6 active)

---

## 8. ADSB — AP_ADSB vs meridian-adsb

### 8.1 Threat detection

**Present in Meridian**: **Yes.**  
`AircraftDatabase::threat_level()` classifies: NONE / ADVISORY (5 NM, TTCA < 120s) / WARNING (2 NM, TTCA < 60s) / ALERT (1 NM, TTCA < 30s). ArduPilot delegates classification to `AC_Avoidance`/`AP_Avoidance` rather than computing in ADSB itself, but the classification logic matches.

### 8.2 Time-to-closest-approach computation

**Present in Meridian**: **Yes.**  
TTCA computed via `t = -(r · v) / |v|²` using relative position (NE frame, haversine projected) and relative velocity from heading/speed. Negative TTCA (diverging traffic) → `f64::MAX` (no threat). This is correct.

**Gap**: ArduPilot filters by configurable radius (`ADSB_LIST_RADIUS`) and altitude (`ADSB_LIST_ALT`) windows before adding to the vehicle list. Meridian has no spatial pre-filter — all received aircraft are added up to `MAX_AIRCRAFT = 25`.

### 8.3 Receiver backends

| | ArduPilot | Meridian |
|---|---|---|
| **Backends** | 4 (uAvionix MAVLink, Sagetech legacy, uAvionix UCP, Sagetech MXS) | **0** |

Meridian's `AircraftDatabase` is populated purely by caller-fed `AircraftState` structs — there is no serial or MAVLink ADSB receiver parser. All four ArduPilot receiver backends (uAvionix, Sagetech) are absent.

### 8.4 ADS-B Out (transponder)

**Present in Meridian**: **No.**  
ArduPilot manages transponder output config (ICAO, callsign, emitter type, squawk, mode A/C/S selection, 7400/7600/7700 emergency squawk automation). Meridian has no ADS-B Out capability.

### 8.5 Other missing ADSB features

- `ADSB_ICAO_SPECL` — bypass filters for one specific ICAO
- Squawk 7400 automation on RC/GCS failsafe
- `ADSB_LIST_RADIUS` / `ADSB_LIST_ALT` spatial filters
- MAVLink `ADSB_VEHICLE` message parsing (receiver)
- `TRAFFIC_REPORT` MAVLink emission (to GCS)

---

## 9. OpenDroneID — AP_OpenDroneID vs meridian-opendroneid

### 9.1 All 5 message types

**Present in Meridian**: **Yes.**  
All five ASTM F3411-22a / MAVLink OpenDroneID message structs are defined:
- `BasicIdMessage` (IdType, UaType, UAS ID)
- `LocationMessage` (position, velocity, altitude, heading, accuracy, timestamp)
- `SystemMessage` (operator location, area info, classification)
- `SelfIdMessage` (description)
- `OperatorIdMessage` (operator ID)

### 9.2 Pre-arm transponder handshake

**Present in Meridian**: **Yes.**  
`OpenDroneId::prearm_ok()` requires `transponder_connected && transponder_arm_status_ok`. `set_transponder_status(connected, arm_ok)` allows the caller to feed handshake state. This matches ArduPilot's `arm_status` packet check.

**Gap**: ArduPilot has an `EnforcePreArmChecks` option bit that makes the transponder check a hard block on arming. The `AllowNonGPSPosition` and `LockUASIDOnFirstBasicIDRx` option bits are also absent. Meridian's `prearm_ok()` always enforces both conditions.

### 9.3 Broadcast timing

**Present in Meridian**: **Yes, correct.**  
- Location: `location_interval_ms = 1000` (1 Hz) — correct.
- Static messages: `static_interval_ms = 3000` (3 s) — correct.
- Static messages are round-robined: BasicID → SelfID → System → OperatorID → repeat — correct.
- First call (sentinel `u32::MAX`) always fires immediately — correct.

**Gap**: ArduPilot supports two broadcast transport paths: MAVLink serial relay (to a connected transponder module) and DroneCAN. Meridian's `BroadcastAction` enum tells the caller what to send but provides no transport abstraction — the caller must implement both transports. ArduPilot's persistent UAS ID (survives reboot via param system) is also absent.

---

## 10. AutoTune — AC_AutoTune vs meridian-autotune

### 10.1 5-step twitch method

**Present in Meridian**: **Yes.**  
`TuneStep` enum: `RateDUp → RateDDown → RatePUp → AnglePDown → AnglePUp` — 5 steps, correct order. The twitch-command / settle-measure / analyze loop is implemented in `AutoTuner::update()`.

### 10.2 25% gain backoff

**Present in Meridian**: **Yes.**  
`GAIN_REDUCTION = 0.75` (i.e., multiply by 0.75 = reduce by 25%) is applied when bounce-back exceeds threshold. This exactly matches ArduPilot.

### 10.3 Helicopter frequency sweep

**Present in Meridian**: **No.**  
ArduPilot's `AC_AutoTune_Heli` uses a frequency sweep method (injecting sinusoidal rate inputs at multiple frequencies and measuring the transfer function) rather than the twitch method. Meridian implements only the multirotor twitch method (`AC_AutoTune_Multi`). Helicopter autotuning is absent.

### 10.4 Other gaps

- Multi-axis sequential automation (ArduPilot auto-proceeds Roll → Pitch → Yaw; Meridian requires caller to call `start(axis)` for each)
- Save/load gains from param system
- GCS messaging during tune (progress, axis complete, failure)
- Autotune aux function / flight mode integration
- Bounce-back detection uses a simpler method in Meridian (tracking max abs rate during settle vs. signed bounce-back in ArduPilot)

---

## 11. GyroFFT — AP_GyroFFT vs meridian-fft

### 11.1 3-peak tracking with distance-matrix

**Present in Meridian**: **Yes.**  
`GyroFFT` tracks `NUM_PEAKS = 3` peaks. `match_peaks()` builds a `distance[3][3]` matrix of `|freq_old - freq_new|`, then uses greedy minimum-distance assignment to match peaks across frames, preventing identity swaps. This matches ArduPilot's peak-tracking approach.

### 11.2 Harmonic detection

**Present in Meridian**: **Yes.**  
`GyroFFT::is_harmonic(freq, fundamental)` checks if `freq ≈ N × fundamental` (N ≥ 2, tolerance 10% of fundamental). This matches ArduPilot's harmonic detection.

### 11.3 Other gaps

- ArduPilot runs FFT on 3 gyro axes simultaneously (X, Y, Z). Meridian's `GyroFFT` is a single-axis processor — the caller must instantiate 3 separate instances, and there is no multi-axis correlation.
- ArduPilot outputs peak frequency to a harmonic notch filter (`AP_InertialSensor_HarmonicNotch`). Meridian's `get_center_frequency()` provides the frequency, but no notch filter integration exists.
- ArduPilot's FFT runs on background thread with DMA-fed gyro samples. Meridian's `feed_sample()` is synchronous.
- Window size in ArduPilot is configurable via `FFT_WINDOW_SIZE` parameter with runtime validation. Meridian validates only via `debug_assert`.
- ArduPilot computes the noise floor and signal-to-noise ratio for each peak. Meridian tracks magnitude only.
- Throttle-frequency correlation for motor RPM estimation is absent.

---

## 12. Proximity — AP_Proximity vs meridian-proximity

### 12.1 8-sector × 5-layer boundary

**Present in Meridian**: **Yes, correct.**  
`ProximityBoundary` implements exactly `NUM_SECTORS = 8` (45° each) × `NUM_LAYERS = 5` = 40 faces. Layer elevation ranges match ArduPilot: below (−90 to −45°), low (−45 to −10°), level (−10 to 10°), high (10 to 45°), above (45 to 90°). Per-face 3-sample median + IIR filter + 1-second expiry is implemented correctly.

### 12.2 AC_Avoid velocity bending

**Present in Meridian**: **Yes.**  
`avoid_velocity()` checks each horizontal sector at the level layer, computes velocity component toward obstacle, and scales it down by `(dist / margin_m)`. This matches AC_Avoid's proportional velocity reduction behavior.

**Gaps**:

- **Backends**: ArduPilot has multiple proximity sensor backends (RPLidar, TeraRanger Tower, ST VL53L0X, Lightware SF45B, Cygbot, MAVLink `DISTANCE_SENSOR`, etc.) with a backend factory. Meridian has no sensor backends — the caller feeds distances directly via `boundary.update()`.
- **3D avoidance**: ArduPilot's AC_Avoid integrates vertical proximity (above/below layers). Meridian's `avoid_velocity()` only checks the level layer (layer index 2), ignoring vertical obstacles.
- **Fence integration**: ArduPilot's AC_Avoid also bends velocity for geo-fence boundaries. Meridian has no fence velocity bending.
- **Stop before impact**: ArduPilot has `get_closest_object()` and a hard stop mode when obstacle is within `AVOID_DIST_MAX`. Meridian has only proportional scaling.
- **Horizontal sectors only in avoidance**: Even the sector center offsets (22.5° center-of-sector approximation) introduce small cross-coupling errors that ArduPilot avoids by using the exact bearing from sensor data.

---

## Summary of All Gaps

### Complete subsystem absences (implement from scratch)

| # | Gap | Severity |
|---|---|---|
| 1 | `AP_Parachute` — motor shutdown → delay → chute deploy sequence | Critical safety |
| 2 | `AP_Gripper` — Servo + EPM, grab/release state machine | Payload |
| 3 | `AP_Winch` — position/rate/RC modes, Daiwa telemetry | Payload |
| 4 | `AP_LandingGear` — deploy/retract, altitude triggers, WoW sensor | Payload |
| 5 | `AP_Sprayer` — pump/spinner PWM, GPS coverage | Payload |
| 6 | ADS-B receiver backends (uAvionix, Sagetech) — no ADSB-In parser | Compliance |
| 7 | ADS-B Out — transponder configuration and emission | Compliance |
| 8 | WS2812/NeoPixel physical HAL driver | Hardware |

### Significant partial implementations (requires completion)

| # | Gap | Current state |
|---|---|---|
| 9 | Camera: 7 missing backends (SoloGimbal, MAVLink, MAVLink-CamV2, Scripting, RunCam, Mount-delegate, SoloGimbal) | 2 of 9 done |
| 10 | Camera: burst/finite-count mode, min-interval guard, roll-limit guard, hardware feedback pin | Not started |
| 11 | Camera: focus, zoom, tracking, lens, camera-source, cam-mode-toggle | Not started |
| 12 | Camera: MAVLink Camera Protocol v2 messages | Not started |
| 13 | Mount: 11 missing backends (Alexmos, SToRM32 × 2, MAVLink-v2, Viewpro, Topotek, CADDX, XFRobot, Xacti, Scripting, SoloGimbal) | 3 of 14 done |
| 14 | Mount GPS ROI — bearing/pitch math not computed in MountManager | Stub |
| 15 | Mount RC targeting — no frame-lock flags (yaw/pitch/roll earth-frame vs body-frame) | Partial |
| 16 | Mount Siyi — no 20 Hz attitude polling loop, no EXTERNAL_ATTITUDE / POSITION_DATA, no upside-down transform | Partial |
| 17 | Precision landing: NIS outlier rejection in KF, lag param integration, XY gate, state machine, retry logic | Partial |
| 18 | OSD: 18+ missing items (xtrack, aspd1/2, secondary battery, climbeff, VTX, link stats, fence, stat, hgt_abvterr, etc.) | 50 of ~68 done |
| 19 | OSD: MSP periodic heartbeat, INAV symbol map, HD resolution support, blink state | Partial |
| 20 | OSD: MAX7456 font upload from SD card, custom symbol table | Not started |
| 21 | OSD: Multi-screen TOGGLE and AUTO_SWITCH modes, per-screen PWM windows, debounce | Partial |
| 22 | OSD: Stats screen max-value tracking (MaxAltitude, MaxSpeed, MaxDistance render `--`) | Stub |
| 23 | OSD: SITL, TXONLY backends; dual backend support | Not started |
| 24 | Notify: 5 missing LED patterns (initializing, ESC-cal, EKF fail, GPS glitch, prearm fail, armed-no-GPS) | 7 of 12 done |
| 25 | Notify: 26 missing buzzer tunes (startup, waypoint, EKF alert, battery CTS, vehicle lost, autotune events, etc.) | 6 of 32 done |
| 26 | Notify: WS2812 HAL driver absent (RGB values computed but not output) | Stub |
| 27 | Notify: I2C LED controllers, OLED displays, DShot/DroneCAN buzzer, board LEDs | Not started |
| 28 | Notify: LED_OVERRIDE MAVLink bypass, brightness levels | Not started |
| 29 | ADSB: Spatial filters (ADSB_LIST_RADIUS, ADSB_LIST_ALT), ICAO special target | Not started |
| 30 | ADSB: TRAFFIC_REPORT MAVLink emission | Not started |
| 31 | OpenDroneID: Option bits (EnforcePreArmChecks, AllowNonGPS, LockUASID), dual transport (MAVLink relay + DroneCAN) | Partial |
| 32 | AutoTune: Helicopter frequency sweep method (AC_AutoTune_Heli) | Not started |
| 33 | AutoTune: Multi-axis automation, GCS messaging, param save/load | Partial |
| 34 | GyroFFT: 3-axis simultaneous processing and correlation, notch filter output, SNR/noise floor | Partial |
| 35 | Proximity: Sensor backends (RPLidar, TeraRanger, MAVLink DISTANCE_SENSOR, etc.) | Not started |
| 36 | Proximity: Vertical avoidance (above/below layers in avoid_velocity), fence velocity bending, hard stop | Partial |
