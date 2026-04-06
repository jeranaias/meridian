# ArduPilot Lua Scripting System — Full Audit
**Meridian Reference Document**
Date: 2026-04-02

---

## 1. Scripting Bindings — Complete Categorization

Source: `libraries/AP_Scripting/generator/description/bindings.desc` (1163 lines, ~450 unique binding entries)

The binding generator reads this file and emits C++ glue code (`lua_generated_bindings.cpp`). Each `singleton`, `userdata`, `ap_object`, and `global` line is a separately callable binding. Total distinct callable methods/fields: approximately 350–400 depending on build flags.

### 1.1 Bindings by Subsystem

#### Navigation / Attitude (AHRS)
- `ahrs.get_roll_rad/get_pitch_rad/get_yaw_rad` — Euler angles in radians
- `ahrs.get_location` — GPS-fused position
- `ahrs.get_gyro/get_accel` — raw IMU rates and accelerations
- `ahrs.get_velocity_NED` — NED velocity vector
- `ahrs.wind_estimate` — estimated wind vector
- `ahrs.get_quaternion` — full attitude quaternion
- `ahrs.get_variances` — EKF health variances
- `ahrs.set_posvelyaw_source_set` — switch EKF input source set
- `ahrs.handle_external_position_estimate` — inject external position fix
- `ahrs.set_home/set_origin` — move home and EKF origin

#### IMU / Inertial Sensor
- `ins.get_gyro/get_accel` — raw per-instance values
- `ins.get_temperature` — IMU temperature
- `ins.get_gyro_health/get_accel_health` — health per instance
- `ins.gyros_consistent/accels_consistent` — cross-sensor consistency check
- `ins.calibrating` — calibration in-progress flag

#### GPS
- `gps.status/location/velocity/ground_speed` — full GPS fix data
- `gps.num_sensors/primary_sensor` — multi-GPS instance
- `gps.speed_accuracy/horizontal_accuracy/vertical_accuracy` — accuracy metrics
- `gps.num_sats/time_week/time_week_ms/time_epoch_usec` — timing
- `gps.gps_yaw_deg` — dual-antenna GPS heading
- `gps.inject_data` (manual) — push raw RTCM/RTCA data to GPS

#### Barometer
- `baro.get_pressure/get_temperature/get_altitude/get_external_temperature`
- `baro.healthy` — per-instance health

#### Compass
- `compass.healthy` — per-instance health check

#### Battery Monitor
- `battery.voltage/current_amps/consumed_mah/consumed_wh`
- `battery.capacity_remaining_pct/pack_capacity_mah`
- `battery.get_cell_voltage` — individual cell voltages
- `battery.has_failsafed/overpower_detected`
- `battery.reset_remaining` — SOC reset
- `battery.handle_scripting` — write scripted battery state (virtual battery driver)
- `BattMonitorScript_State` — struct for virtual battery data injection

#### Rangefinder
- `rangefinder.distance_orient/min_distance_orient/max_distance_orient`
- `rangefinder.get_backend` — per-instance backend object
- `AP_RangeFinder_Backend.handle_script_msg` — inject range from script (virtual driver)
- `AP_RangeFinder_Backend.get_state` — full state snapshot

#### Proximity / Obstacle Avoidance
- `proximity.get_closest_object/get_object_angle_and_distance`
- `AP_Proximity_Backend.handle_script_distance_msg/handle_script_3d_msg` — virtual sensor injection
- `AP_Proximity_Backend.update_virtual_boundary`

#### Terrain
- `terrain.enabled/status`
- `terrain.height_amsl/height_terrain_difference_home/height_above_terrain`

#### RC Input
- `rc.get_pwm` — raw PWM per channel
- `rc.find_channel_for_option` — find channel by AUX function
- `rc.run_aux_function` — trigger AUX function from script
- `rc.get_aux_cached` — last known switch position
- `RC_Channel.norm_input/norm_input_dz/get_aux_switch_pos`
- `RC_Channel.set_override` — force RC override (WARNING: disables failsafes)

#### Servo Output
- `SRV_Channels.set_output_pwm/set_output_pwm_chan` — set servo PWM
- `SRV_Channels.set_output_pwm_chan_timeout` — timed servo override
- `SRV_Channels.set_output_scaled/set_output_norm` — normalized output
- `SRV_Channels.get_output_pwm/get_output_pwm_chan/get_output_scaled`
- `SRV_Channels.find_channel` — channel lookup by function
- `SRV_Channels.get_emergency_stop/get_safety_state`
- `SRV_Channels.set_angle/set_range` — configure servo travel

#### Motors (Copter/Plane multirotor)
- `motors.get_roll/get_pitch/get_yaw/get_throttle` — controller output demands
- `motors.get_spool_state/get_interlock` — motor arming state
- `motors.set_external_limits` — impose external saturation limits
- `MotorsMatrix.init/add_motor_raw/set_throttle_factor/get_lost_motor` — static mixer setup
- `Motors_dynamic.init/add_motor/load_factors` — runtime-configurable mixer
- `Motors_6DoF.init/add_motor` — 6-DoF multirotor mixer

#### Attitude Control
- `AC_AttitudeControl.get_rpy_srate` — actual roll/pitch/yaw rates
- `AC_AttitudeControl.get_att_error_angle_deg` — attitude error magnitude
- `AC_AttitudeControl_Multi_6DoF.set_lateral_enable/set_forward_enable/set_offset_roll_pitch`

#### Position Control (Copter)
- `poscontrol.set_posvelaccel_offset` — inject external position/velocity offset
- `poscontrol.get_posvelaccel_offset/get_vel_target/get_accel_target`

#### Vehicle (high-level)
- `vehicle.set_mode/get_mode` — flight mode switching
- `vehicle.start_takeoff` — scripted takeoff
- `vehicle.set_target_location/get_target_location/update_target_location` — guided target
- `vehicle.set_target_pos_NED/set_target_posvel_NED/set_target_posvelaccel_NED` — full NED navigation
- `vehicle.set_target_velaccel_NED/set_target_velocity_NED` — velocity commands
- `vehicle.set_target_angle_and_climbrate` — attitude + climb rate
- `vehicle.set_target_rate_and_throttle` — rate + throttle direct control
- `vehicle.set_target_angle_and_rate_and_throttle` — full aerobatic control
- `vehicle.nav_script_time/nav_script_time_done` — custom NAV_SCRIPT_TIME mission items
- `vehicle.nav_scripting_enable` — activate nav scripting mode
- `vehicle.set_steering_and_throttle` — rover direct control
- `vehicle.set_desired_speed/set_desired_turn_rate_and_speed` — rover nav
- `vehicle.reboot` — reboot from script
- `vehicle.has_ekf_failsafed` — EKF health gate
- `vehicle.register_custom_mode` — **define new flight modes at runtime**

#### Mission
- `mission.state/get_current_nav_index/set_current_cmd`
- `mission.get_item/set_item` — read/write individual mission items
- `mission.clear/num_commands`
- `mission.jump_to_tag/get_index_of_jump_tag/get_last_jump_tag`
- `mission.jump_to_landing_sequence/jump_to_abort_landing_sequence`
- `mavlink_mission_item_int_t` — full mission item struct with all fields

#### Parameters
- `param.get/set/set_and_save/set_default` — any param by name
- `param.add_table/add_param` — **define new parameters at runtime**
- `Parameter` userdata — cached parameter handle for fast repeated access

#### GCS / Telemetry
- `gcs.send_text` — MAVLink status text
- `gcs.send_named_float/send_named_string` — named telemetry channels
- `gcs.set_message_interval` — configure message rates
- `gcs.run_command_int` — send MAVLink commands
- `gcs.get_allow_param_set/set_allow_param_set` — gate parameter changes
- `gcs.enable_high_latency_connections`

#### MAVLink (raw)
- `mavlink.init/register_rx_msgid` — receive specific MAVLink message IDs
- `mavlink.send_chan/receive_chan` — send/receive raw MAVLink frames
- `mavlink.block_command` — intercept and suppress MAVLink commands

#### Serial
- `serial.find_serial/find_simulated_device` — find scripting serial port
- `AP_Scripting_SerialAccess.begin/read/write/readstring/writestring`
- `AP_Scripting_SerialAccess.available/configure_parity/set_stop_bits/set_flow_control`

#### CAN
- `CAN.get_device/get_device2` — get scripting CAN buffer
- `ScriptingCANBuffer.write_frame/read_frame/add_filter`
- `CANFrame` — CAN frame struct with id, data[], dlc
- `DroneCAN_Handle` — full DroneCAN message send/subscribe

#### I2C
- `i2c.get_device` — get I2C bus device handle
- `AP_HAL_I2CDevice.write_register/read_registers/transfer/set_retries/set_address`

#### GPIO / Analog
- `gpio.read/write/toggle/pinMode/get_mode/set_mode` — digital I/O
- `analog.channel` — get ADC channel handle
- `AP_HAL_AnalogSource.voltage_average/voltage_latest/voltage_average_ratiometric`
- `analog.mcu_temperature/mcu_voltage` — internal MCU sensors
- `PWMSource.set_pin/get_pwm_us/get_pwm_avg_us` — PWM input capture

#### Networking (TCP/UDP)
- `Socket` (global) — create TCP or UDP socket
- `SocketAPM.connect/bind/send/sendto/recv/accept/listen/close`
- `SocketAPM.pollin/pollout/is_connected/set_blocking/reuseaddress`
- `networking.get_ip_active/get_netmask_active/get_gateway_active/add_route`

#### LED / Notify
- `notify.play_tune` — play buzzer tune
- `notify.handle_rgb/handle_rgb_id` — set LED color by channel
- `notify.send_text/release_text` — scrolling text on OSD/LED
- `serialLED.set_num_neopixel/set_RGB/send` — NeoPixel/ProfiLED string control
- `LED.get_rgb` — read current ScriptingLED state

#### Camera / Gimbal / Mount
- `camera.take_picture/record_video/set_trigger_distance`
- `camera.get_state` — full camera state (recording, zoom, focus, tracking)
- `camera.change_setting` — thermal palette, gain, raw data
- `camera.set_camera_information/set_stream_information` — publish camera caps to GCS
- `mount.get_mode/set_mode/set_angle_target/set_rate_target/set_roi_target`
- `mount.get_attitude_euler/get_rate_target/get_angle_target`
- `mount.set_attitude_euler` — direct gimbal command

#### EFI (Engine / Generator)
- `efi.get_backend/get_state/get_last_update_ms`
- `AP_EFI_Backend.handle_scripting` — inject scripted EFI state (virtual EFI driver)
- `EFI_State` / `Cylinder_Status` — full engine telemetry structs

#### ESC Telemetry
- `esc_telem.get_rpm/get_temperature/get_motor_temperature/get_current/get_voltage`
- `esc_telem.update_rpm/update_telem_data` — inject scripted ESC data
- `esc_telem.get_last_telem_data_ms/set_rpm_scale`

#### Servo Telemetry
- `servo_telem.get_telem/update_telem_data`
- `ServoTelemetryData` — position, force, speed, voltage, current, temp

#### Optical Flow
- `optical_flow.enabled/healthy/quality`

#### Visual Odometry
- `visual_odom.healthy/quality`

#### Winch
- `winch.healthy/relax/release_length/set_desired_rate/get_rate_max`

#### Precision Landing
- `precland.healthy/target_acquired/get_last_valid_target_ms`
- `precland.get_target_velocity/get_target_location`

#### Follow Mode
- `follow.have_target/get_target_sysid/get_last_update_ms`
- `follow.get_target_location_and_velocity/get_target_dist_and_vel_NED_m`
- `follow.get_target_heading_deg`

#### Fencing
- `fence.get_breaches/get_breach_time/get_margin_breaches`
- `fence.get_breach_distance/get_breach_direction_NED`
- `fence.get_safe_alt_min/get_safe_alt_max/present/get_enabled_fences`

#### Rally Points
- `rally.get_rally_location`

#### RPM Sensor
- `RPM.get_rpm`

#### Temperature Sensor
- `temperature_sensor.get_temperature`

#### Relay
- `relay.on/off/toggle/enabled/get`

#### Arming
- `arming.arm/arm_force/disarm/is_armed/pre_arm_checks`
- `arming.get_aux_auth_id/set_aux_auth_passed/set_aux_auth_failed` — custom pre-arm checks

#### Button
- `button.get_button_state`

#### Filesystem
- `fs.stat/crc32/format/get_format_status`
- `dirlist/remove` (globals) — list directory, delete files
- `io` (standard Lua) — file open/read/write/close

#### Logging
- `logger.write` — custom DataFlash log entries
- `logger.log_file_content` — log a file to DataFlash

#### Clock / Time
- `millis/micros` (globals) — system time
- `rtc.clock_s_to_date_fields/date_fields_to_clock_s` — wall-clock conversion

#### CRSF
- `crsf.add_menu/get_menu_event/peek_menu_event/pop_menu_event/send_response`
- `CRSFMenu/CRSFParameter` — build custom CRSF transmitter menus

#### FrSky SPort
- `frsky_sport.sport_telemetry_push/prep_number`

#### Scripting Control
- `scripting.restart_all` — restart all scripts
- `FWVersion.string/type/major/minor/patch/hash` — firmware version

#### Vehicle-Type-Specific
- `quadplane.in_vtol_mode/in_assisted_flight/abort_landing/in_vtol_land_descent`
- `sub.get_and_clear_button_count/is_button_pressed/rangefinder_alt_ok`
- `AR_AttitudeControl.get_srate` — Rover attitude control rates

#### ONVIF (IP Camera Control)
- `onvif.start/set_absolutemove/get_pan_tilt_limit_min/get_pan_tilt_limit_max`

#### Simulator
- `sim.set_pose` — SITL only: force vehicle pose

#### Math Types
- `Vector3f`, `Vector2f`, `Quaternion`, `Location` — all with arithmetic operators
- `uint32_t`, `uint64_t` — 64-bit integer arithmetic
- `mavlink_mission_item_int_t` — mission item struct

### 1.2 Binding Entry Count
Total non-comment lines in bindings.desc: 1163  
Estimated distinct callable methods (counting `method`, `manual`, `field read/write`): approximately **380 callable entry points** across 40+ subsystems.

### 1.3 Most Complex Bindings

**Motor control** — scripts can define a completely custom motor mixer at runtime via `Motors_dynamic` and `Motors_6DoF`. Combined with `vehicle.set_target_rate_and_throttle` and `vehicle.set_target_angle_and_rate_and_throttle`, scripts can implement entire flight controllers in Lua, bypassing ArduPilot's standard rate loops.

**Vehicle control pipeline** — `vehicle.nav_scripting_enable` puts the autopilot in a mode where Lua is the *only* source of navigation commands. Scripts then supply position/velocity/acceleration setpoints or rate+throttle commands every update cycle. This is how the aerobatics applet works.

**Raw hardware access** — scripts can read/write GPIO pins, sample ADC channels, communicate on I2C buses, send/receive raw CAN frames (including DroneCAN), and open TCP/UDP sockets. This is genuine embedded-level hardware access.

**MAVLink interception** — scripts can register to receive specific MAVLink message IDs and can block specific commands (`mavlink.block_command`), effectively acting as a middleware shim between the GCS and the flight controller.

**Virtual driver injection** — scripts can inject data into: battery monitor, rangefinder backend, proximity backend, EFI backend, ESC telemetry, servo telemetry. This means a script IS a sensor driver.

---

## 2. Lua Runtime Configuration

Source: `libraries/AP_Scripting/AP_Scripting.cpp`, `lua_scripts.cpp`

### 2.1 Memory
| Parameter | Description | Default |
|-----------|-------------|---------|
| `SCR_HEAP_SIZE` | Total Lua heap in bytes | 43 KB (low mem), 100 KB (mid), 200 KB (SITL/Linux) |
| Stack | Per-coroutine stack | 17 KB default, range 8–64 KB |
| `SCR_VM_I_COUNT` | Max VM instructions per timeslice | 10,000 |

The heap supports dynamic expansion (up to 20 KB at a time on allocation failure) unless `SCR_DEBUG_OPTS` bit 6 is set. Complex applets like aerobatics require `SCR_HEAP_SIZE` of 100,000–200,000.

### 2.2 Sandbox
The sandbox (`create_sandbox`) exposes:
- Standard Lua: `math`, `table`, `string`, `io`, `package`
- ArduPilot bindings: all singletons and userdatas listed in section 1
- NOT exposed: `debug` module (conditional compile only), `os`, `coroutine`, `load` (restricted base sandbox)
- `io` is the full Lua io library — scripts have filesystem read/write access via the SD card

### 2.3 Script Loading
- Primary directory: `/APM/scripts` on SD card (ChibiOS/embedded)
- Simulator: `./scripts`
- Secondary: `@ROMFS/scripts` — scripts baked into firmware ROM
- Loading controlled by `SCR_DIR_DISABLE` bitmask (bit 0 = ROMFS, bit 1 = APM/scripts)
- All `.lua` files in the directory are loaded automatically at boot
- Modules (shared libraries): `require()` looks in `APM/scripts/modules/`
- File discovery: flat directory scan, no recursion (only top-level `.lua` files are auto-loaded)

### 2.4 Error Handling
- Each script runs in `lua_pcall` — a crashed script does not crash the engine
- On runtime error: script is **removed from the scheduler** and a `MAV_SEVERITY_CRITICAL` GCS message is sent
- On CPU overtime (`SCR_VM_I_COUNT` exceeded): script is removed, message sent
- The last error message is stored and checked at arming time (prevents arming if a script has crashed)
- Scripts can be restarted via `scripting.restart_all()` or via MAVLink command `SCRIPTING_CMD_STOP_AND_RESTART`
- CRC integrity check: `SCR_LD_CHECKSUM` and `SCR_RUN_CHECKSUM` — vehicle will not arm if script CRC mismatches

### 2.5 Scheduling Model
Scripts are cooperative coroutines, not preemptive threads. Each script is a function that returns `(next_function, delay_ms)`. The scheduler runs the earliest-due script each pass. Scripts share a single Lua state and heap. There is no per-script memory isolation. A script that corrupts global state affects all scripts.

### 2.6 What Scripts CANNOT Do
- No direct memory access (`ffi`, `debug` not exposed)
- No `os.execute` or shell access
- No direct register-level hardware access (must go through ArduPilot HAL bindings)
- No flash writes except via `param.set_and_save` and `logger.write`
- No network access on non-networking-capable hardware (SocketAPM requires `AP_NETWORKING_ENABLED`)
- No real-time guarantee — a busy system may delay script execution
- Cannot create threads or coroutines independent of the scheduler
- `RC_Channel.set_override` bypasses RC failsafes — this is a deliberate footgun documented in a comment

---

## 3. Complete Applets Catalog

Location: `libraries/AP_Scripting/applets/`

### 3.1 Flight Behavior / Navigation

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `Aerobatics/FixedWing/plane_aerobatics.lua` | Full aerobatic maneuver sequencer: loops, rolls, hammerheads, inverted flight, scripted mission profiles. Implements its own trajectory tracking and speed controller. | vehicle (full nav scripting), ahrs, param, gcs, mission, SRV_Channels | Yes — complete aerobatic flight mode not available in core firmware | Native Rust for Meridian fixed-wing. Not needed for Meridian copter MVP. |
| `copter-slung-payload.lua` | Reduces slung payload oscillation by receiving payload GPS (MAVLink) and computing vehicle position offsets | vehicle, mavlink, param, gcs, poscontrol | Yes — slung payload physics compensation | Script if needed; low priority for MVP |
| `copter_terrain_brake.lua` | Switches copter to BRAKE mode when altitude above terrain drops below threshold while in LOITER | rangefinder, vehicle, param, gcs, ahrs | Yes — terrain-responsive mode override | Consider native; important for terrain safety |
| `copter-deadreckon-home.lua` | Flies toward home in Guided_NoGPS when GPS quality drops or EKF fails | vehicle, gps, ahrs, param, gcs | Yes — GPS-loss dead reckoning behavior | Important safety feature; consider native |
| `quadplane_terrain_avoid.lua` | Terrain avoidance for QuadPlane using forward rangefinder; pitches up or switches to QLOITER if obstacle detected | rangefinder, vehicle, terrain, ahrs, param, gcs, mission | Yes — active rangefinder-based obstacle avoidance | Important for Meridian plane; consider native |
| `QuadPlane_Low_Alt_FW_mode_prevention.lua` | Prevents forward-flight mode near home at low altitude; forces QLAND | vehicle, param, ahrs, gps, quadplane | Yes — safety gate for hybrid transitions | Script works fine; low priority |
| `forward_flight_motor_shutdown.lua` | Stops specific VTOL motors during forward flight based on throttle thresholds | SRV_Channels, motors, param, rc | Yes — motor shutdown for tiltrotor/tailsitter efficiency | Native Rust for tiltrotors |
| `plane_ship_landing.lua` | VTOL takeoff and landing on moving platforms; tracks ship velocity and adjusts approach | vehicle, ahrs, gps, mission, param, gcs, quadplane | Yes — moving platform landing | Consider native for Meridian; highly novel |
| `plane_precland.lua` | Precision landing framework for QuadPlane using precland sensor | precland, vehicle, param, gcs | Extends precland API | Script works; low priority |
| `plane_follow.lua` | Implements follow mode for fixed-wing using GUIDED + AP_Follow library | follow, vehicle, mavlink, param, gcs | Yes — fixed-wing follow mode | Script; low priority |
| `plane_package_place.lua` | Package delivery landing for QuadPlane; monitors descent and triggers release servo | rangefinder, vehicle, SRV_Channels, param, mission | Yes — payload drop logic | Script; niche |
| `UniversalAutoLand.lua` | On arm, auto-generates a 4-waypoint mission (takeoff, approach, land) | mission, ahrs, gps, param, gcs | Yes — auto mission generation | Script; niche |
| `advance-wp.lua` | RC switch advances mission to next waypoint; optional audio announcements | mission, rc, gcs, param, notify | Yes — pilot-controlled waypoint skip | Script; works well in Lua |
| `ahrs-source-extnav-optflow.lua` | Auto-selects EKF source between external nav and optical flow based on quality | ahrs, optical_flow, rangefinder, param, gcs, rc | Yes — intelligent EKF source switching | Consider native; important for indoor nav |
| `ahrs-set-origin.lua` | Sets EKF/AHRS origin to a fixed configured GPS coordinate | ahrs, param, gcs | Yes — useful for GPS-denied environments | Simple; script is fine |
| `x-quad-cg-allocation.lua` | Adjusts motor allocation matrix for off-center CG on X-quads | Motors_dynamic, param, gcs | Yes — real-time mixer adjustment | Script; keep in Lua |

### 3.2 Auto-Tuning

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `VTOL-quicktune.lua` | Automated PID tuning for multirotor/QuadPlane in LOITER; adjusts ATC gains by exciting each axis | vehicle, ahrs, param, rc, gcs | Yes — in-flight auto-tune without dedicated mode | High priority — native Rust (or well-supported script with param binding) |
| `rover-quicktune.lua` | Automated steering and speed controller tuning in Circle mode | vehicle, param, rc, gcs | Yes — rover auto-tune | Script; medium priority |
| `Heli_idle_control.lua` | Closed-loop idle throttle control for traditional helicopter on ground | RPM, SRV_Channels, rc, param | Yes — trad-heli idle governor | Script; heli-specific |
| `Heli_IM_COL_Tune.lua` | Interactive collective management tuning for helicopters | param, rc, SRV_Channels | Yes — heli-specific tuning utility | Script; heli-specific |

### 3.3 Battery / Power

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `BattEstimate.lua` | Battery State-of-Charge estimator using resting voltage model (polynomial coefficients) | battery, param, gcs | Yes — voltage-based SOC estimation with curve fitting | Consider native; important for reliable remaining energy |
| `BatteryTag.lua` | Reads battery tag info (cycle count, capacity) from DroneCAN BatteryTag peripherals | DroneCAN, param, logger, gcs | Yes — DroneCAN battery metadata logging | Script with DroneCAN support; medium priority |

### 3.4 Communication / Telemetry

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `RockBlock.lua` | MAVLink HIGH_LATENCY2 over Iridium SBD satellite modem | serial, gcs, param | Yes — satellite telemetry link | Script; niche; serial binding sufficient |
| `RockBlock-9704.lua` | Updated RockBlock driver variant | serial, gcs, param | Same as above | Script |
| `follow-target-send.lua` | Broadcasts FOLLOW_TARGET MAVLink message at 10 Hz for swarm/follow use | mavlink, ahrs, gps, param, gcs | Yes — MAVLink follow target broadcast | Script |
| `SmartAudio.lua` | FPV VTX power control via SmartAudio 2.0 serial protocol | serial, rc, param | Yes — FPV VTX management | Script; serial binding sufficient |
| `net-ntrip.lua` | NTRIP RTK correction stream client over TCP | networking (SocketAPM), gps, gcs | Yes — direct LAN NTRIP client in flight controller | Script; requires networking hardware |
| `net_webserver.lua` | HTTP web server running on the flight controller | networking (SocketAPM), param | Yes — embedded HTTP server | Proof-of-concept; mostly for diagnostics |
| `CAN_playback.lua` | Replays CAN frames from a log file at original timing | CAN, io, gcs | Yes — CAN bus test/replay | Script; debug tool |
| `crsf-calibrate.lua` | Runs sensor calibration procedures via custom CRSF transmitter menu | crsf, gcs, mavlink | Yes — CRSF-controlled calibration UI | Script; CRSF-specific |

### 3.5 LED / Lighting

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `leds_on_a_switch.lua` | Controls LED brightness with a 3-position RC switch | rc, param | No — could use AUX function | Script; trivial |
| `Hexsoon LEDs.lua` | ProfiLED/NeoPixel control for Hexsoon EDU 450 drone, arm/disarm state-based strobing | serialLED, arming, param | Yes — pattern-based LED sequencing with arm state | Script; LED patterns belong in script layer |

### 3.6 Camera / Gimbal

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `Gimbal_Camera_Mode.lua` | Switches gimbal and camera between manual/auto modes via RC switch | SRV_Channels, rc, param | No — parameter manipulation | Script; simple |
| `ONVIF_Camera_Control.lua` | Controls IP camera gimbal via ONVIF protocol over network | onvif, vehicle, gcs | Yes — IP camera integration | Script; niche networking integration |
| `camera-change-setting.lua` | Sets thermal palette, gain, raw data mode via parameters | camera, param | No — parameter wrapping | Script; trivial |
| `mount-poi.lua` | Calculates and optionally locks gimbal to a terrain-intersecting POI from current attitude | mount, terrain, ahrs, gps, param, rc, gcs | Yes — terrain-based POI from gimbal pointing | Script; elegant use of terrain DB |
| `video-stream-information.lua` | Populates VIDEO_STREAM_INFORMATION MAVLink message from params | camera, param, gcs | Yes — GCS video stream registration | Script; works in Lua |
| `runcam_on_arm.lua` | Auto-starts/stops RunCam recording on arm/disarm via RC aux function | rc, arming, param, gcs | No — uses existing RC function | Script; simple |
| `pelco_d_antennatracker.lua` | Pelco-D PTZ camera control for antenna tracker | serial, SRV_Channels, vehicle | Yes — Pelco-D protocol implementation | Script; niche |

### 3.7 Parameter Management

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `Param_Controller.lua` | Loads parameter files from subdirectories based on AUX switch position | param, rc, io, gcs | Yes — context-sensitive parameter profiles | Script; useful for multi-mission configs |
| `Script_Controller.lua` | Loads different Lua scripts and missions from subdirectories based on AUX switch | rc, io, mission, gcs | Yes — runtime script switching | Script; useful but meta |
| `MissionSelector.lua` | Loads mission files (QGC WPL format) from SD card based on switch on arm | mission, rc, io, gcs | Yes — file-based mission loading | Script; useful for pre-loaded missions |
| `param-lockdown.lua` | Intercepts MAVLink PARAM_SET and only allows whitelisted parameters | mavlink, param, gcs | Yes — security/safety parameter lock | Script; important for fleet ops |
| `revert_param.lua` | Reverts modified parameters to their startup values via RC switch | param, rc, gcs | Yes — in-flight parameter undo | Script; useful for tuning |

### 3.8 Safety / Failsafe

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `arming-checks.lua` | Configurable arming checks with per-severity levels, extensible check framework | arming, vehicle, param, gcs | Yes — custom pre-arm validation framework | High priority — native Rust hook system needed |
| `throttle_kill.lua` | Engine/turbine kill switch via AUX function, sends specific PWM to throttle | SRV_Channels, rc, param | Yes — turbine kill | Script; straightforward |
| `motor_failure_test.lua` | Stops motors in flight for testing failure handling | SRV_Channels, motors, rc, param | Yes — motor failure simulation | Script; test/debug tool |

### 3.9 AHRS / Navigation Support

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `ahrs-source-extnav-optflow.lua` | (See 3.1 above) | | | |

### 3.10 Winch / Payload

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `winch-control.lua` | RC switch controls winch retract/deploy at fixed rates | winch, rc, param, gcs | Yes — manual winch control without dedicated flight mode | Script; payload-specific |

### 3.11 Development Tools

| File | Description | Subsystems | New Functionality | Meridian Priority |
|------|-------------|------------|-------------------|-------------------|
| `repl.lua` | Interactive Lua REPL over serial or MAVLink for live scripting | serial, mavlink, gcs | Yes — live Lua shell on flight controller | Script; critical debug tool |
| `WebExamples/` | Web server usage examples | networking | No | Examples only |

---

## 4. Driver Scripts — Complete Catalog

Location: `libraries/AP_Scripting/drivers/`

These scripts act as hardware device drivers — they sit in the standard scripting directory and populate ArduPilot subsystem backends from external hardware.

### 4.1 EFI / Engine

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `EFI_DLA.lua` | Gap-framed serial | DLA 232cc engine (AusStars) | serial, EFI backend injection, param |
| `EFI_DLA64.lua` | Serial variant | DLA 64cc engine | serial, EFI, param |
| `EFI_HFE.lua` | Serial | HFE International engine | serial, EFI, param |
| `EFI_Halo6000.lua` | Serial | Halo 6000 engine | serial, EFI, param |
| `EFI_NMEA2k.lua` | CAN (NMEA2000 marine) | Marine CAN EFI | CAN, EFI, param |
| `EFI_SkyPower.lua` | Serial | SkyPower SP engine | serial, EFI, param |
| `Generator_SVFFI.lua` | Serial | SVFFI generator system | serial, EFI, param |
| `INF_Inject.lua` | Serial | InnoFlight EFI system | serial, EFI, param, SRV_Channels |

All EFI drivers use `AP_EFI_Backend.handle_scripting(EFI_State)` to feed engine data into ArduPilot's EFI subsystem. This is the scripting EFI driver interface.

### 4.2 Battery Monitors

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `BattMon_ANX.lua` | CAN | ANX CAN battery monitor | CAN, battery scripting injection, param |

Uses `battery.handle_scripting(BattMonitorScript_State)` — injects complete battery state.

### 4.3 Servos / Actuators

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `UltraMotion.lua` | CAN | UltraMotion linear actuators | CAN, SRV_Channels, param |

Translates CAN servo feedback and commands to ArduPilot servo output layer.

### 4.4 ESC Telemetry

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `Hobbywing_DataLink.lua` | Serial | HobbyWing DataLink ESC telemetry | serial, esc_telem, param |

Uses `esc_telem.update_rpm/update_telem_data` to push data into ESC telemetry system.

### 4.5 Rangefinders / Proximity

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `TOFSense-M/TOFSense-M_CAN.lua` | CAN | MiraSense TOFSense-M time-of-flight | CAN, rangefinder backend injection |
| `TOFSense-M/TOFSense-M_Serial.lua` | Serial | Same sensor, serial variant | serial, rangefinder injection |

Uses `AP_RangeFinder_Backend.handle_script_msg` — injects range readings.

### 4.6 Gimbals / Camera Mounts

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `mount-djirs2-driver.lua` | CAN | DJI RS2/RS3 gimbal | CAN, mount, param |

Implements full DJI RS2 CAN protocol including CRC-16/CRC-32 framing, translates mount API commands to DJI proprietary format.

### 4.7 Propulsion / Marine

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `torqeedo-torqlink.lua` | CAN (TorqLink) | Torqeedo electric boat motors | CAN, SRV_Channels, param |

### 4.8 Flow Meters

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `VSPeak_flow_meter.lua` | Serial | VSPeak Modell fuel flow sensor | serial, logger, param |

Logs fuel consumption data; feeds into ArduPilot fuel tracking.

### 4.9 Connectivity

| File | Protocol | Device | Subsystems Used |
|------|----------|--------|-----------------|
| `LTE_modem.lua` | Serial AT commands | SIM7600/EC200/Air780 LTE modems | serial, networking, param |

Initializes LTE modem via AT commands and hands off the serial port to ArduPilot's PPP (protocol 48) networking stack.

### 4.10 Driver Architecture Summary

The driver pattern is consistent:
1. Find serial or CAN resource
2. Parse protocol from hardware
3. Call backend injection method (`handle_scripting`) OR directly update subsystem
4. Expose configuration parameters via `param:add_table/add_param`

This pattern lets ArduPilot support new hardware variants without firmware changes. The injection APIs (`handle_scripting` on EFI_Backend, RangeFinder_Backend, BattMonitor, ESC_Telem, Proximity_Backend) are the critical hooks.

---

## 5. Critical Applets — Deep Analysis

### 5.1 Aerobatics (plane_aerobatics.lua)
**What it does:** Full trajectory-tracking aerobatic flight controller for fixed-wing. Implements its own speed controller (PID), roll correction, path error correction (P+D), throttle lookahead, and a library of maneuvers (loops, rolls, hammerheads, Cuban eights, immelmanns, knife-edge, tailslides, etc.). Reads maneuver schedules from `.txt` files on SD card. Uses `vehicle.nav_scripting_enable` to take over navigation entirely, then issues `vehicle.set_target_angle_and_rate_and_throttle` every cycle.

**Why it matters for Meridian:** This is the benchmark for "what Lua scripting enables." ArduPilot needed to build a complete secondary flight controller in Lua to support aerobatics because the core firmware couldn't expose enough hooks. Meridian's Rust core should natively expose a `ScriptingFlightController` trait that allows external controllers to inject attitude+rate+throttle setpoints.

### 5.2 VTOL-quicktune.lua
**What it does:** Auto-tunes ATC_RAT_RLL/PIT/YAW P/I/D/FF gains while flying in LOITER. Excites each axis with a step input, measures the response, and adjusts gains. Runs entirely in Lua using `ahrs` for rate feedback and `param.set` for gain updates. No dedicated firmware mode needed.

**Why it matters:** Meridian needs equivalent capability. Either provide a Rust tuning subsystem or ensure the param API + ahrs bindings are solid enough to run this script directly.

### 5.3 arming-checks.lua
**What it does:** Replaces hard-coded arming checks with a Lua-configurable system. Each check has a MAV_SEVERITY level (warning allows arming, error blocks it). New checks are added as functions in a table. Uses `arming.get_aux_auth_id/set_aux_auth_passed/set_aux_auth_failed`.

**Why it matters:** Meridian's arming system must expose a plugin hook for custom checks. The ArduPilot `aux_auth` API is a clean pattern — scripts register a slot, then pass/fail it each cycle.

### 5.4 copter-deadreckon-home.lua
**What it does:** Monitors GPS quality (speed accuracy, satellite count) and EKF failsafe state. When quality drops below threshold, switches to Guided_NoGPS and computes a bearing toward last-known home position, then leans the vehicle in that direction.

**Why it matters:** This is safety-critical navigation behavior. ArduPilot shipped it as a script, not firmware. Meridian should consider whether this belongs natively or in a scripting layer that has access to GPS quality and EKF state.

### 5.5 plane_ship_landing.lua
**What it does:** Tracks a moving platform by monitoring its velocity (from MAVLink), adjusts the vehicle's approach path to account for deck motion, handles deck pitch/roll for touchdown assessment. Uses `vehicle.set_target_location` in a control loop.

**Why it matters:** Demonstrates that complex mission-critical functionality (carrier landing is life-safety) is implemented entirely in Lua. Meridian's scripting API must be performant enough to close a 10–20 Hz control loop.

### 5.6 param-lockdown.lua
**What it does:** Intercepts all `PARAM_SET` MAVLink messages, checks against a whitelist, and either passes them to ArduPilot or silently drops them. Uses `gcs.set_allow_param_set(false)` to disable normal parameter handling, then uses `mavlink.register_rx_msgid` to receive PARAM_SET.

**Why it matters:** Security/hardening script. Shows that Lua can completely take over MAVLink command handling. Meridian needs a MAVLink intercept hook in its scripting API.

---

## 6. Scripting as Extension Mechanism

### 6.1 Distribution
Scripts are distributed as individual `.lua` files, typically shared via:
- ArduPilot's official GitHub repo (`libraries/AP_Scripting/applets/`, `drivers/`)
- Mission Planner's MAVFtp upload tool
- Manual SD card copy
- ROMFS embedding in custom firmware builds

No package manager exists. Users copy `.lua` files to `/APM/scripts` on the SD card.

### 6.2 Typical Use Cases
1. **Hardware drivers** — A new EFI, ESC, gimbal, or sensor is supported without firmware update
2. **Vehicle-specific behavior** — Specialized landing sequences, custom failsafes, tuning automation
3. **Integration** — Connect to external systems (RockBlock, NTRIP, LTE modem, ONVIF camera)
4. **Prototyping** — Test new flight control concepts (aerobatics, slung payload, ship landing)
5. **Fleet hardening** — Param lockdown, arming check customization, mission profile selection

### 6.3 Custom Flight Modes
**Yes, scripts can define new flight modes.** The binding `vehicle.register_custom_mode(uint8_t mode_id, string name, string short_name)` returns a `custom_mode_state` object with `allow_entry` boolean. Combined with `vehicle.set_mode`, scripts can implement complete new flight modes by:
1. Registering the mode number and name
2. Detecting when the mode is active via `vehicle.get_mode`
3. Issuing navigation/control commands each cycle while in that mode

### 6.4 Failsafe Override
**Yes, scripts can modify failsafe behavior.** Mechanisms include:
- `mavlink.block_command` — block specific MAVLink commands
- `gcs.set_allow_param_set` — disable parameter changes
- `vehicle.set_mode` — force mode change in response to conditions
- Custom arming checks via `arming.set_aux_auth_failed` — prevent arming
- `RC_Channel.set_override` — override RC input (documented as disabling failsafes)
- Flight computer reboot via `vehicle.reboot`

Scripts cannot disable the hardware watchdog or prevent an EKF failsafe from triggering internally, but they can react to `vehicle.has_ekf_failsafed` and take compensating action.

### 6.5 Named Float Channels
**Yes.** `gcs.send_named_float(name, value)` sends a `NAMED_VALUE_FLOAT` MAVLink message to the GCS. Scripts use this extensively for telemetry. `gcs.send_named_string` sends string values. These appear in Mission Planner and Yaapu telemetry scripts.

---

## 7. Meridian Scripting Strategy

### 7.1 Functionality That Must Be Native Rust

The following AppletLua functionality is too safety-critical or performance-sensitive to leave in a scripting layer:

| Functionality | Reason |
|---------------|--------|
| EKF source switching (extnav/optflow) | <10ms latency required; affects navigation integrity |
| GPS-loss dead reckoning | Life-safety; must not fail due to script heap exhaustion |
| Motor mixer (basic frame types) | Real-time control; script latency unacceptable |
| Terrain brake / proximity avoidance | Hard safety; must run at control loop rate |
| Core arming check framework | Security; must not be bypassable by a crashed script |
| Battery monitor (core) | Safety data source |
| EFI/engine interface (core) | Engine safety |

### 7.2 Functionality Well-Suited to Scripting

These applets should remain in Lua (or Meridian's equivalent scripting layer):

| Functionality | Reason |
|---------------|--------|
| Auto-tune (quicktune) | Not time-critical; param reads/writes at 1–10 Hz |
| Aerobatics | Specialist use case; keep in script layer |
| LED patterns | No safety implications |
| Camera/gimbal modes | Mission/payload specific |
| Satellite modem telemetry | Async I/O |
| NTRIP client | Network I/O |
| Parameter lockdown | Security policy; belongs in configurable layer |
| Mission file loading | File I/O, not time-critical |
| Custom arming checks | Safety plugin, but via Rust hook interface |
| Hardware drivers (EFI, ESC telem, rangefinder scripting) | Rapid hardware support without Rust compile |
| Dead reckoning home | Safe to script IF scripting API has guaranteed heap |
| Ship landing | Specialist; 10 Hz control loop is within scripting capability |
| Winch control | Payload-specific |
| REPL | Debug tool; script layer is ideal |

### 7.3 Should Meridian Support Lua?

**Recommendation: Yes, with a smaller initial API surface.**

Arguments for Lua:
- The ArduPilot ecosystem has hundreds of existing scripts. Users migrating to Meridian will expect them to work.
- Lua is the established language for UAV scripting; pilots know it.
- The binding generator pattern is mature and generates safe glue code automatically.

Arguments against or for alternatives:
- Lua has no memory safety. A heap-exhausted script silently dies. In Meridian's safety model this is unacceptable for safety-critical hooks.
- WASM (WebAssembly) offers memory isolation, capability-based security, and near-native speed. WASM plugins could replace the Lua layer entirely for more complex controllers.
- Rust dynamic loading (`libloading`) is theoretically faster but impractical for cross-compilation on embedded targets.

**Recommendation:** Implement Lua scripting as a **non-safety-critical extension layer** following ArduPilot's model. Route all safety-critical data through Meridian's Rust core, exposing it to scripts as read-only telemetry. Allow scripts to *request* mode changes and position targets but always interpose a Rust safety validator. Consider WASM as a future upgrade path for better isolation.

### 7.4 Minimum Scripting API for Parity

To run the majority of ArduPilot applets, Meridian needs these binding categories at minimum:

**Tier 1 (critical for most applets):**
- `param.get/set/set_and_save/add_table/add_param` — all applets use dynamic params
- `gcs.send_text/send_named_float` — all applets send telemetry
- `ahrs.get_roll/pitch/yaw/get_location/get_velocity_NED` — most navigation applets
- `vehicle.set_mode/get_mode/nav_scripting_enable` — flight control scripts
- `vehicle.set_target_location/set_target_posvel_NED` — guided navigation
- `rc.get_pwm/get_aux_cached/find_channel_for_option` — RC switch reading
- `arming.is_armed/get_aux_auth_id/set_aux_auth_passed/set_aux_auth_failed`
- `millis/micros` globals
- `io` library (file access)
- `serial.find_serial` + serial read/write — driver scripts
- `mission.get_item/set_item/clear/num_commands` — mission manipulation

**Tier 2 (for specific applets):**
- `SRV_Channels.set_output_pwm_chan/set_output_pwm` — servo control
- `battery.voltage/current_amps/capacity_remaining_pct`
- `gps.status/location/num_sats`
- `rangefinder.distance_orient`
- `terrain.height_above_terrain`
- CAN bindings — for all hardware drivers
- `mavlink.register_rx_msgid/receive_chan/send_chan/block_command`
- `vehicle.register_custom_mode`
- `notify.play_tune/handle_rgb`
- `serialLED.set_RGB/send`

**Tier 3 (specialist/advanced):**
- `esc_telem`, `efi`, battery scripting injection — driver APIs
- `poscontrol`, `attitude_control` — aerobatics-class control
- `vehicle.set_target_angle_and_rate_and_throttle` — aerobatic control
- `Motors_dynamic.load_factors` — custom mixer
- `networking`, `SocketAPM` — TCP/UDP scripts
- `DroneCAN_Handle` — DroneCAN scripting drivers
- `crsf` — CRSF menu system
- `frsky_sport` — FrSky telemetry

### 7.5 API Design Principles from ArduPilot's Approach

1. **Fail gracefully** — crashed scripts are removed and logged; the vehicle continues flying
2. **No real-time guarantees to scripts** — scripts are best-effort; safety-critical paths must be in Rust
3. **CRC integrity** — ship safety-critical script bundles with expected CRCs; block arming on mismatch
4. **Memory budgets** — configure heap size per-deployment; 100 KB minimum for complex applets
5. **Dynamic parameters** — `add_table/add_param` is the key mechanism enabling clean per-applet parameter namespaces; Meridian must implement this
6. **Cooperative scheduling** — return `(function, delay_ms)` idiom works well; keep it
7. **Script checksum enforcement** — `SCR_LD_CHECKSUM/SCR_RUN_CHECKSUM` for fleet deployments

---

## Appendix: File Counts

- Applets: 47 `.lua` files (including Aerobatics subdirectory) across `applets/`
- Drivers: 17 `.lua` files (including TOFSense-M subdirectory variants) across `drivers/`
- Total scripting directory scripts: 64 `.lua` files
- Bindings file: 1163 lines, ~380 callable entry points
- Supported hardware via drivers: DLA engines (3 variants), HFE, Halo6000, SkyPower, SVFFI generator, INF Inject, NMEA2000 EFI, ANX CAN battery, HobbyWing ESC, UltraMotion servos, TOFSense-M (CAN+serial), DJI RS2/RS3 gimbal, Torqeedo marine motors, VSPeak fuel flow, LTE modems (SIM7600/EC200/Air780)
