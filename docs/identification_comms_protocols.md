# Meridian Parity Identification: Communications, Protocols, RC, Telemetry, CAN

**Date**: 2026-04-02
**Scope**: ArduPilot `libraries/GCS_MAVLink/`, `AP_RCProtocol/`, `AP_RCTelemetry/`, `AP_Frsky_Telem/`, `AP_CRSF/`, `AP_SerialManager/`, `AP_CANManager/`, `AP_DroneCAN/`, `AP_KDECAN/`, `AP_PiccoloCAN/`, `AP_MSP/`, `AP_DDS/`, `AP_Networking/`, `AP_Hott_Telem/`, `AP_LTM_Telem/`, `AP_IBus_Telem/`, `AP_Devo_Telem/`
**Meridian source tree**: `D:/projects/meridian/crates/`
**ArduPilot source tree**: `D:/projects/ardupilot/libraries/`

---

## Table of Contents

1. [GCS_MAVLink — Message Handlers](#1-gcs_mavlink--message-handlers)
2. [GCS_MAVLink — Outbound Messages (ap_message enum)](#2-gcs_mavlink--outbound-messages)
3. [GCS_MAVLink — MAV_CMD Handlers](#3-gcs_mavlink--mav_cmd-handlers)
4. [GCS_MAVLink — Support Systems](#4-gcs_mavlink--support-systems)
5. [AP_SerialManager — Serial Protocol Registry](#5-ap_serialmanager--serial-protocol-registry)
6. [AP_RCProtocol — Every Backend](#6-ap_rcprotocol--every-backend)
7. [AP_RCTelemetry — CRSF Telemetry](#7-ap_rctelemetry--crsf-telemetry)
8. [AP_RCTelemetry — GHST Telemetry](#8-ap_rctelemetry--ghst-telemetry)
9. [AP_RCTelemetry — Spektrum Telemetry](#9-ap_rctelemetry--spektrum-telemetry)
10. [AP_Frsky_Telem — FrSky D Protocol](#10-ap_frsky_telem--frsky-d-protocol)
11. [AP_Frsky_Telem — FrSky SPort](#11-ap_frsky_telem--frsky-sport)
12. [AP_Frsky_Telem — FrSky SPort Passthrough](#12-ap_frsky_telem--frsky-sport-passthrough)
13. [AP_Frsky_Telem — MAVlite Bidirectional](#13-ap_frsky_telem--mavlite-bidirectional)
14. [AP_CRSF — CRSF Protocol Library](#14-ap_crsf--crsf-protocol-library)
15. [AP_Hott_Telem — HoTT Telemetry](#15-ap_hott_telem--hott-telemetry)
16. [AP_LTM_Telem — Lightweight Telemetry](#16-ap_ltm_telem--lightweight-telemetry)
17. [AP_IBus_Telem — iBus Telemetry](#17-ap_ibus_telem--ibus-telemetry)
18. [AP_Devo_Telem — Walkera DEVO](#18-ap_devo_telem--walkera-devo)
19. [AP_CANManager — CAN Bus Management](#19-ap_canmanager--can-bus-management)
20. [AP_DroneCAN — DroneCAN (UAVCAN v0)](#20-ap_dronecan--dronecan-uavcan-v0)
21. [AP_KDECAN — KDE Direct ESC Protocol](#21-ap_kdecan--kde-direct-esc-protocol)
22. [AP_PiccoloCAN — Swift Navigation Piccolo](#22-ap_piccolocan--swift-navigation-piccolo)
23. [AP_MSP — MultiWii Serial Protocol](#23-ap_msp--multiwii-serial-protocol)
24. [AP_DDS — ROS 2 DDS/XRCE](#24-ap_dds--ros-2-ddsxrce)
25. [AP_Networking — TCP/UDP/PPP Stack](#25-ap_networking--tcpudpppp-stack)
26. [Gap Summary Table](#26-gap-summary-table)

---

## 1. GCS_MAVLink — Message Handlers

### PROTOCOL: MAVLink v2 — Inbound Message Dispatch

```
FILE:  libraries/GCS_MAVLink/GCS_Common.cpp
LINES: 7822
FUNCTION: Central receive dispatch (GCS_MAVLINK::handle_message).
          Every MAVLink message arriving on a serial port or network
          channel is routed through this function. Subclasses (ArduCopter,
          ArduPlane, etc.) override handle_message to add vehicle-specific
          handlers, then call Super::handle_message for common ones.
MERIDIAN HAS: STUB
```

**Every inbound message handler (GCS_Common.cpp `handle_message` switch):**

| Case ID | Message | Handler function | Meridian |
|---------|---------|------------------|----------|
| MAVLINK_MSG_ID_HEARTBEAT (0) | GCS heartbeat | `handle_heartbeat()` | YES (adapter.rs parses it as InboundCommand::GcsHeartbeat) |
| MAVLINK_MSG_ID_COMMAND_ACK (77) | ACK for commands we sent | `handle_command_ack()` | NO |
| MAVLINK_MSG_ID_SETUP_SIGNING (258) | Link signing setup | `handle_setup_signing()` | STUB (signing.rs exists, not wired) |
| MAVLINK_MSG_ID_PARAM_REQUEST_LIST (21) | Start param dump | param handler | YES |
| MAVLINK_MSG_ID_PARAM_SET (23) | Set a parameter | param handler | YES |
| MAVLINK_MSG_ID_PARAM_REQUEST_READ (20) | Read one param | param handler | YES |
| MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN (48) | Set map origin | `handle_set_gps_global_origin()` | NO |
| MAVLINK_MSG_ID_DEVICE_OP_READ (11000) | I2C/SPI read | `handle_device_op_read()` | NO |
| MAVLINK_MSG_ID_DEVICE_OP_WRITE (11001) | I2C/SPI write | `handle_device_op_write()` | NO |
| MAVLINK_MSG_ID_TIMESYNC (111) | Clock sync | `handle_timesync()` | NO |
| MAVLINK_MSG_ID_LOG_REQUEST_LIST (117) | Log enumerate | log download | STUB (log_download.rs) |
| MAVLINK_MSG_ID_LOG_REQUEST_DATA (119) | Log data fetch | log download | STUB |
| MAVLINK_MSG_ID_LOG_ERASE (121) | Erase logs | log download | NO |
| MAVLINK_MSG_ID_LOG_REQUEST_END (122) | End log transfer | log download | NO |
| MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS (184) | MAVLink logging ACK | log download | NO |
| MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL (110) | MAVFTP | `handle_file_transfer_protocol()` | STUB (mavftp.rs) |
| MAVLINK_MSG_ID_DIGICAM_CONTROL (155) | Camera trigger | camera handler | NO |
| MAVLINK_MSG_ID_GOPRO_HEARTBEAT (215) | GoPro status | camera handler | NO |
| MAVLINK_MSG_ID_CAMERA_INFORMATION (259) | Camera info | camera handler | NO |
| MAVLINK_MSG_ID_SET_MODE (11) | Legacy mode set | `handle_set_mode()` | NO |
| MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST (183) | Old version query | `handle_send_autopilot_version()` | YES (via MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) |
| MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST (38) | Mission partial | `handle_mission_write_partial_list()` | NO |
| MAVLINK_MSG_ID_MISSION_REQUEST_LIST (43) | Mission list | `handle_mission_request_list()` | YES |
| MAVLINK_MSG_ID_MISSION_COUNT (44) | Mission item count | `handle_mission_count()` | YES |
| MAVLINK_MSG_ID_MISSION_CLEAR_ALL (45) | Clear mission | `handle_mission_clear_all()` | YES |
| MAVLINK_MSG_ID_MISSION_ITEM (39) | Mission item (float) | `handle_mission_item()` | NO (only MISSION_ITEM_INT) |
| MAVLINK_MSG_ID_MISSION_ITEM_INT (73) | Mission item (int) | `handle_mission_item()` | YES |
| MAVLINK_MSG_ID_MISSION_REQUEST_INT (51) | Request item | mission protocol | YES |
| MAVLINK_MSG_ID_MISSION_REQUEST (40) | Request item (legacy) | mission protocol | NO |
| MAVLINK_MSG_ID_MISSION_ACK (47) | Mission upload ACK | mission protocol | YES |
| MAVLINK_MSG_ID_MISSION_SET_CURRENT (41) | Jump to waypoint | `handle_mission_set_current()` | NO |
| MAVLINK_MSG_ID_COMMAND_LONG (76) | Long command | `handle_command_long()` | YES (subset of MAV_CMDs) |
| MAVLINK_MSG_ID_COMMAND_INT (75) | Int command | `handle_command_int()` | YES (subset) |
| MAVLINK_MSG_ID_FENCE_POINT (160) | Geofence point | fence handler | NO |
| MAVLINK_MSG_ID_FENCE_FETCH_POINT (161) | Fetch fence | fence handler | NO |
| MAVLINK_MSG_ID_TERRAIN_DATA (134) | Terrain tile | terrain handler | NO |
| MAVLINK_MSG_ID_TERRAIN_CHECK (135) | Check terrain | terrain handler | NO |
| MAVLINK_MSG_ID_GIMBAL_REPORT (200) | Gimbal state | `handle_mount_message()` | NO |
| MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION (283) | Gimbal v3 info | `handle_mount_message()` | NO |
| MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS (285) | Gimbal v3 status | `handle_mount_message()` | NO |
| MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE (282) | Gimbal v3 cmd | `handle_mount_message()` | NO |
| MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW (287) | Gimbal v3 pitchyaw | `handle_mount_message()` | NO |
| MAVLINK_MSG_ID_PARAM_VALUE (22) | Param from downstream | `handle_param_value()` → mount | NO |
| MAVLINK_MSG_ID_RADIO (166) | SiK radio status | `handle_radio_status()` | NO |
| MAVLINK_MSG_ID_RADIO_STATUS (109) | Radio RSSI/noise | `handle_radio_status()` | NO |
| MAVLINK_MSG_ID_SERIAL_CONTROL (126) | Serial passthrough | `handle_serial_control()` | NO |
| MAVLINK_MSG_ID_GPS_RTCM_DATA (233) | RTCM corrections | AP_GPS handler | NO |
| MAVLINK_MSG_ID_GPS_INPUT (232) | GPS injection | AP_GPS handler | NO |
| MAVLINK_MSG_ID_GPS_INJECT_DATA (123) | Legacy GPS inject | AP_GPS handler | NO |
| MAVLINK_MSG_ID_STATUSTEXT (253) | Downstream statustext | `handle_statustext()` | NO (only sends, not receives) |
| MAVLINK_MSG_ID_LED_CONTROL (186) | LED command | `AP_Notify::handle_led_control()` | NO |
| MAVLINK_MSG_ID_MANUAL_CONTROL (69) | Joystick input | `handle_manual_control()` | NO |
| MAVLINK_MSG_ID_PLAY_TUNE (258) | Audio tune | `AP_Notify::handle_play_tune()` | NO |
| MAVLINK_MSG_ID_RALLY_POINT (175) | Rally point | `handle_common_rally_message()` | NO |
| MAVLINK_MSG_ID_RALLY_FETCH_POINT (176) | Fetch rally | `handle_common_rally_message()` | NO |
| MAVLINK_MSG_ID_REQUEST_DATA_STREAM (66) | Set stream rates (legacy) | `handle_request_data_stream()` | YES |
| MAVLINK_MSG_ID_DATA96 (172) | Opaque 96-byte blob | `handle_data_packet()` | NO |
| MAVLINK_MSG_ID_VISION_POSITION_DELTA (11011) | VisOdom delta | `handle_vision_position_delta()` | NO |
| MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE (102) | VisOdom abs | `handle_vision_position_estimate()` | NO |
| MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE (101) | Global VisOdom | `handle_global_vision_position_estimate()` | NO |
| MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE (104) | Vicon mocap | `handle_vicon_position_estimate()` | NO |
| MAVLINK_MSG_ID_ODOMETRY (331) | Full odometry | `handle_odometry()` | NO |
| MAVLINK_MSG_ID_ATT_POS_MOCAP (138) | MoCap pose | `handle_att_pos_mocap()` | NO |
| MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE (103) | VisOdom velocity | `handle_vision_speed_estimate()` | NO |
| MAVLINK_MSG_ID_SYSTEM_TIME (2) | GPS time sync | `handle_system_time_message()` | NO |
| MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE (70) | Override RC | `handle_rc_channels_override()` | NO |
| MAVLINK_MSG_ID_RADIO_RC_CHANNELS (11009) | Radio RC override | `handle_radio_rc_channels()` | NO |
| MAVLINK_MSG_ID_OPTICAL_FLOW (100) | Optical flow | `handle_optical_flow()` | NO |
| MAVLINK_MSG_ID_DISTANCE_SENSOR (132) | Range sensor | `handle_distance_sensor()` | NO |
| MAVLINK_MSG_ID_OBSTACLE_DISTANCE (330) | Obstacle scan | `handle_obstacle_distance()` | NO |
| MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D (11037) | 3D obstacle | `handle_obstacle_distance_3d()` | NO |
| MAVLINK_MSG_ID_OSD_PARAM_CONFIG (11033) | OSD param | `handle_osd_param_config()` | NO |
| MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG (11034) | OSD param show | `handle_osd_param_config()` | NO |
| MAVLINK_MSG_ID_ADSB_VEHICLE (246) | ADSB target | `handle_adsb_message()` | NO (meridian-adsb has stub) |
| MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG (10001) | ADSB config | `handle_adsb_message()` | NO |
| MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC (10002) | ADSB dynamic | `handle_adsb_message()` | NO |
| MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT (10003) | ADSB health | `handle_adsb_message()` | NO |
| MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL (10005) | ADSB control | `handle_adsb_message()` | NO |
| MAVLINK_MSG_ID_LANDING_TARGET (149) | Precision land | `handle_landing_target()` | NO |
| MAVLINK_MSG_ID_NAMED_VALUE_FLOAT (251) | Named telemetry | `handle_named_value()` | NO |
| MAVLINK_MSG_ID_CAN_FRAME (386) | CAN passthrough | `handle_can_frame()` | NO |
| MAVLINK_MSG_ID_CANFD_FRAME (387) | CANFD passthrough | `handle_can_frame()` | NO |
| MAVLINK_MSG_ID_CAN_FILTER_MODIFY (388) | CAN filter | `AP_MAVLinkCAN::handle_can_filter_modify()` | NO |
| MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS (12918) | OpenDroneID arm | `AP_OpenDroneID::handle_msg()` | NO |
| MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID (12905) | Operator ID | `AP_OpenDroneID::handle_msg()` | NO |
| MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID (12903) | Self ID | `AP_OpenDroneID::handle_msg()` | NO |
| MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID (12900) | Basic ID | `AP_OpenDroneID::handle_msg()` | NO |
| MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM (12904) | System info | `AP_OpenDroneID::handle_msg()` | NO |
| MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE (12919) | System update | `AP_OpenDroneID::handle_msg()` | NO |
| MAVLINK_MSG_ID_SECURE_COMMAND (11004) | Secure command | `AP_CheckFirmware::handle_msg()` | NO |
| MAVLINK_MSG_ID_SECURE_COMMAND_REPLY (11005) | Secure reply | `AP_CheckFirmware::handle_msg()` | NO |
| MAVLINK_MSG_ID_EFI_STATUS (225) | EFI data | `efi->handle_EFI_message()` | NO |
| MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI (10015) | Loweheiser EFI | `handle_generator_message()` | NO |

---

## 2. GCS_MAVLink — Outbound Messages

Every `ap_message` enum value in `ap_message.h` corresponds to one outbound MAVLink message.  
Source: `D:/projects/ardupilot/libraries/GCS_MAVLink/ap_message.h` (122 lines)

```
FILE:  libraries/GCS_MAVLink/ap_message.h + GCS_Common.cpp
LINES: 122 (enum) + ~7822 (send functions)
FUNCTION: Defines every scheduled and on-demand outbound message.
          Sent via the bucket/stream-rate system in GCS_MAVLINK::update_send().
MERIDIAN HAS: PARTIAL — meridian-mavlink sends ~12 of ~70+ distinct messages
```

**Every outbound message (ap_message values) vs Meridian:**

| ap_message ID | MAVLink Message | Meridian Sends? |
|---------------|----------------|-----------------|
| MSG_HEARTBEAT (0) | HEARTBEAT | YES |
| MSG_AHRS (1) | AHRS (ArduPilot custom) | NO |
| MSG_AHRS2 (2) | AHRS2 (ArduPilot custom) | NO |
| MSG_ATTITUDE (3) | ATTITUDE (#30) | YES |
| MSG_ATTITUDE_QUATERNION (4) | ATTITUDE_QUATERNION (#31) | NO |
| MSG_LOCATION (5) | GLOBAL_POSITION_INT (#33) | YES |
| MSG_VFR_HUD (6) | VFR_HUD (#74) | YES |
| MSG_SYS_STATUS (7) | SYS_STATUS (#1) | YES |
| MSG_POWER_STATUS (8) | POWER_STATUS (#125) | NO |
| MSG_MEMINFO (9) | MEMINFO (ArduPilot custom) | NO |
| MSG_NAV_CONTROLLER_OUTPUT (10) | NAV_CONTROLLER_OUTPUT (#62) | NO |
| MSG_CURRENT_WAYPOINT (11) | MISSION_CURRENT (#42) | NO |
| MSG_SERVO_OUTPUT_RAW (12) | SERVO_OUTPUT_RAW (#36) | NO |
| MSG_RC_CHANNELS (13) | RC_CHANNELS (#65) | YES (defined in v2.rs, server sends it) |
| MSG_RC_CHANNELS_RAW (14) | RC_CHANNELS_RAW (#35) | NO |
| MSG_RAW_IMU (15) | RAW_IMU (#27) | NO |
| MSG_SCALED_IMU (16) | SCALED_IMU (#26) | NO |
| MSG_SCALED_IMU2 (17) | SCALED_IMU2 (#116) | NO |
| MSG_SCALED_IMU3 (18) | SCALED_IMU3 (#129) | NO |
| MSG_SCALED_PRESSURE (19) | SCALED_PRESSURE (#29) | NO |
| MSG_SCALED_PRESSURE2 (20) | SCALED_PRESSURE2 (#137) | NO |
| MSG_SCALED_PRESSURE3 (21) | SCALED_PRESSURE3 (#143) | NO |
| MSG_GPS_RAW (22) | GPS_RAW_INT (#24) | YES |
| MSG_GPS_RTK (23) | GPS_RTK (#127) | NO |
| MSG_GPS2_RAW (24) | GPS2_RAW (#124) | NO |
| MSG_GPS2_RTK (25) | GPS2_RTK (#128) | NO |
| MSG_SYSTEM_TIME (26) | SYSTEM_TIME (#2) | NO |
| MSG_NEXT_MISSION_REQUEST_WAYPOINTS (28) | MISSION_REQUEST_INT | YES (mission protocol) |
| MSG_NEXT_MISSION_REQUEST_RALLY (29) | MISSION_REQUEST_INT for rally | NO |
| MSG_NEXT_MISSION_REQUEST_FENCE (30) | MISSION_REQUEST_INT for fence | NO |
| MSG_NEXT_PARAM (31) | PARAM_VALUE streaming | YES |
| MSG_FENCE_STATUS (32) | FENCE_STATUS (#162) | NO |
| MSG_SIMSTATE (33) | SIMSTATE (ArduPilot custom) | NO |
| MSG_SIM_STATE (34) | SIM_STATE (#108) | NO |
| MSG_HWSTATUS (35) | HWSTATUS (ArduPilot custom) | NO |
| MSG_WIND (36) | WIND (ArduPilot custom #168) | NO |
| MSG_RANGEFINDER (37) | RANGEFINDER (#173) | NO |
| MSG_DISTANCE_SENSOR (38) | DISTANCE_SENSOR (#132) | NO |
| MSG_TERRAIN_REQUEST (39) | TERRAIN_REQUEST (#133) | NO |
| MSG_TERRAIN_REPORT (40) | TERRAIN_REPORT (#136) | NO |
| MSG_CAMERA_FEEDBACK (42) | CAMERA_FEEDBACK (#180) | NO |
| MSG_CAMERA_INFORMATION (43) | CAMERA_INFORMATION (#259) | NO |
| MSG_CAMERA_SETTINGS (44) | CAMERA_SETTINGS (#260) | NO |
| MSG_CAMERA_FOV_STATUS (45) | CAMERA_FOV_STATUS (#271) | NO |
| MSG_CAMERA_CAPTURE_STATUS (46) | CAMERA_CAPTURE_STATUS (#262) | NO |
| MSG_CAMERA_THERMAL_RANGE (47) | CAMERA_THERMAL_RANGE (#278) | NO |
| MSG_GIMBAL_DEVICE_ATTITUDE_STATUS (48) | GIMBAL_DEVICE_ATTITUDE_STATUS (#285) | NO |
| MSG_GIMBAL_MANAGER_INFORMATION (49) | GIMBAL_MANAGER_INFORMATION (#280) | NO |
| MSG_GIMBAL_MANAGER_STATUS (50) | GIMBAL_MANAGER_STATUS (#281) | NO |
| MSG_VIDEO_STREAM_INFORMATION (51) | VIDEO_STREAM_INFORMATION (#269) | NO |
| MSG_OPTICAL_FLOW (52) | OPTICAL_FLOW (#100) | NO |
| MSG_MAG_CAL_PROGRESS (53) | MAG_CAL_PROGRESS (#191) | NO |
| MSG_MAG_CAL_REPORT (54) | MAG_CAL_REPORT (#192) | NO |
| MSG_EKF_STATUS_REPORT (55) | EKF_STATUS_REPORT (#193) | NO |
| MSG_LOCAL_POSITION (56) | LOCAL_POSITION_NED (#32) | NO |
| MSG_PID_TUNING (57) | PID_TUNING (#194) | NO |
| MSG_VIBRATION (58) | VIBRATION (#241) | NO |
| MSG_RPM (59) | RPM (#226) | NO |
| MSG_WHEEL_DISTANCE (60) | WHEEL_DISTANCE (#9000) | NO |
| MSG_MISSION_ITEM_REACHED (61) | MISSION_ITEM_REACHED (#46) | NO |
| MSG_POSITION_TARGET_GLOBAL_INT (62) | POSITION_TARGET_GLOBAL_INT (#87) | NO |
| MSG_POSITION_TARGET_LOCAL_NED (63) | POSITION_TARGET_LOCAL_NED (#85) | NO |
| MSG_ADSB_VEHICLE (64) | ADSB_VEHICLE (#246) | NO |
| MSG_BATTERY_STATUS (65) | BATTERY_STATUS (#147) | YES |
| MSG_AOA_SSA (66) | AOA_SSA (ArduPilot custom) | NO |
| MSG_LANDING (67) | DEEPSTALL (ArduPilot custom) | NO |
| MSG_ESC_TELEMETRY (68) | ESC_TELEMETRY_1_TO_4 (#11030) | NO |
| MSG_ORIGIN (69) | GPS_GLOBAL_ORIGIN (#49) | NO |
| MSG_HOME (70) | HOME_POSITION (#242) | YES |
| MSG_NAMED_FLOAT (71) | NAMED_VALUE_FLOAT (#251) | NO |
| MSG_EXTENDED_SYS_STATE (72) | EXTENDED_SYS_STATE (#245) | NO |
| MSG_AUTOPILOT_VERSION (73) | AUTOPILOT_VERSION (#148) | YES |
| MSG_EFI_STATUS (74) | EFI_STATUS (#225) | NO |
| MSG_GENERATOR_STATUS (75) | GENERATOR_STATUS (#373) | NO |
| MSG_WINCH_STATUS (76) | WINCH_STATUS (#9005) | NO |
| MSG_WATER_DEPTH (77) | WATER_DEPTH (ArduPilot custom) | NO |
| MSG_HIGH_LATENCY2 (78) | HIGH_LATENCY2 (#235) | NO |
| MSG_AIS_VESSEL (79) | AIS_VESSEL (#301) | NO |
| MSG_MCU_STATUS (90) | MCU_STATUS (#11039) | NO |
| MSG_UAVIONIX_ADSB_OUT_STATUS (91) | UAVIONIX_ADSB_OUT_STATUS (#10004) | NO |
| MSG_ATTITUDE_TARGET (92) | ATTITUDE_TARGET (#83) | NO |
| MSG_HYGROMETER (93) | HYGROMETER_SENSOR (ArduPilot custom) | NO |
| MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE (94) | AUTOPILOT_STATE_FOR_GIMBAL_DEVICE (#286) | NO |
| MSG_RELAY_STATUS (95) | RELAY_STATUS (#376) | NO |
| MSG_HIGHRES_IMU (96) | HIGHRES_IMU (#105) | NO |
| MSG_AIRSPEED (97) | AIRSPEED (#295) | NO |
| MSG_AVAILABLE_MODES (98) | AVAILABLE_MODES (#435) | NO |
| MSG_AVAILABLE_MODES_MONITOR (99) | AVAILABLE_MODES_MONITOR (#436) | NO |
| MSG_FLIGHT_INFORMATION (100) | FLIGHT_INFORMATION (#264) | NO |
| MSG_UTM_GLOBAL_POSITION (101) | UTM_GLOBAL_POSITION (#340) | NO |

**Meridian sends only:** HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT, SYS_STATUS, VFR_HUD, GPS_RAW_INT, BATTERY_STATUS, AUTOPILOT_VERSION, HOME_POSITION, PARAM_VALUE, STATUSTEXT, MISSION_COUNT, MISSION_ITEM_INT, MISSION_REQUEST_INT, MISSION_ACK, COMMAND_ACK, RC_CHANNELS (partial).

**Missing from Meridian (priority for QGC/MP full compat):** ATTITUDE_QUATERNION, LOCAL_POSITION_NED, SERVO_OUTPUT_RAW, SCALED_IMU/PRESSURE (sensor streams), EKF_STATUS_REPORT, PID_TUNING, VIBRATION, MISSION_CURRENT, EXTENDED_SYS_STATE, SYSTEM_TIME, NAV_CONTROLLER_OUTPUT, RAW_IMU, POWER_STATUS.

---

## 3. GCS_MAVLink — MAV_CMD Handlers

```
FILE:  libraries/GCS_MAVLink/GCS_Common.cpp  
       (handle_command_int_packet, lines ~5655-5893)
LINES: ~240
FUNCTION: Main MAV_CMD dispatch table — all COMMAND_INT / COMMAND_LONG 
          commands processed by the common GCS layer.
MERIDIAN HAS: PARTIAL — adapter.rs handles: ARM_DISARM (400), NAV_TAKEOFF (22),
              DO_SET_MODE (176), REQUEST_MESSAGE (520), REQUEST_AUTOPILOT_CAPABILITIES (519),
              DO_MOTOR_TEST (209).
```

**Every MAV_CMD handled (common layer only; vehicle subclasses add more):**

| MAV_CMD | ID | Meridian |
|---------|-----|---------|
| MAV_CMD_ACCELCAL_VEHICLE_POS | 42429 | NO |
| MAV_CMD_AIRFRAME_CONFIGURATION | 2520 | NO |
| MAV_CMD_BATTERY_RESET | 42001 | NO |
| MAV_CMD_CAN_FORWARD | 42000 | NO |
| MAV_CMD_CONTROL_HIGH_LATENCY | 2600 | NO |
| MAV_CMD_DEBUG_TRAP | 42700 | NO |
| MAV_CMD_DO_ADSB_OUT_IDENT | 10001 | NO |
| MAV_CMD_DO_SET_GLOBAL_ORIGIN | 48 | NO |
| MAV_CMD_DO_AUX_FUNCTION | 218 | NO |
| MAV_CMD_DO_FENCE_ENABLE | 207 | NO |
| MAV_CMD_DO_FLIGHTTERMINATION | 185 | NO |
| MAV_CMD_DO_GRIPPER | 211 | NO |
| MAV_CMD_DO_JUMP_TAG | 601 | NO |
| MAV_CMD_DO_SET_MISSION_CURRENT | 224 | NO |
| MAV_CMD_DO_SET_MODE | 176 | YES (stub — ACKs accepted) |
| MAV_CMD_DO_SPRAYER | 216 | NO |
| MAV_CMD_DO_DIGICAM_CONFIGURE | 202 | NO |
| MAV_CMD_DO_DIGICAM_CONTROL | 203 | NO |
| MAV_CMD_DO_SET_CAM_TRIGG_DIST | 206 | NO |
| MAV_CMD_SET_CAMERA_ZOOM | 531 | NO |
| MAV_CMD_SET_CAMERA_FOCUS | 532 | NO |
| MAV_CMD_SET_CAMERA_SOURCE | 534 | NO |
| MAV_CMD_IMAGE_START_CAPTURE | 2000 | NO |
| MAV_CMD_IMAGE_STOP_CAPTURE | 2001 | NO |
| MAV_CMD_CAMERA_TRACK_POINT | 2004 | NO |
| MAV_CMD_CAMERA_TRACK_RECTANGLE | 2005 | NO |
| MAV_CMD_CAMERA_STOP_TRACKING | 2010 | NO |
| MAV_CMD_VIDEO_START_CAPTURE | 2500 | NO |
| MAV_CMD_VIDEO_STOP_CAPTURE | 2501 | NO |
| MAV_CMD_DO_SET_ROI_NONE | 197 | NO |
| MAV_CMD_DO_SET_ROI | 201 | NO |
| MAV_CMD_DO_SET_ROI_LOCATION | 195 | NO |
| MAV_CMD_DO_SET_ROI_SYSID | 198 | NO |
| MAV_CMD_DO_MOUNT_CONFIGURE | 204 | NO |
| MAV_CMD_DO_MOUNT_CONTROL | 205 | NO |
| MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW | 1000 | NO |
| MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE | 1001 | NO |
| MAV_CMD_DO_SEND_BANNER | 42428 | NO |
| MAV_CMD_DO_SET_HOME | 179 | NO |
| MAV_CMD_EXTERNAL_POSITION_ESTIMATE | 43004 | NO |
| MAV_CMD_EXTERNAL_WIND_ESTIMATE | 43005 | NO |
| MAV_CMD_COMPONENT_ARM_DISARM | 400 | YES |
| MAV_CMD_FIXED_MAG_CAL_YAW | 42006 | NO |
| MAV_CMD_DO_START_MAG_CAL | 42424 | NO |
| MAV_CMD_DO_ACCEPT_MAG_CAL | 42425 | NO |
| MAV_CMD_DO_CANCEL_MAG_CAL | 42426 | NO |
| MAV_CMD_FLASH_BOOTLOADER | 42650 | NO |
| MAV_CMD_GET_HOME_POSITION | 410 | NO |
| MAV_CMD_PREFLIGHT_CALIBRATION | 241 | NO |
| MAV_CMD_PREFLIGHT_STORAGE | 245 | NO |
| MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN | 246 | NO |
| MAV_CMD_DO_SET_SAFETY_SWITCH_STATE | 43003 | NO |
| MAV_CMD_DO_SET_SERVO | 183 | NO |
| MAV_CMD_DO_REPEAT_SERVO | 184 | NO |
| MAV_CMD_DO_SET_RELAY | 181 | NO |
| MAV_CMD_DO_REPEAT_RELAY | 182 | NO |
| MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES | 520 | YES |
| MAV_CMD_RUN_PREARM_CHECKS | 401 | NO |
| MAV_CMD_SCRIPTING | 42701 | NO |
| MAV_CMD_SET_EKF_SOURCE_SET | 42004 | NO |
| MAV_CMD_START_RX_PAIR | 500 | NO |
| MAV_CMD_STORAGE_FORMAT | 42425 | NO |
| MAV_CMD_SET_MESSAGE_INTERVAL | 511 | NO |
| MAV_CMD_GET_MESSAGE_INTERVAL | 512 | NO |
| MAV_CMD_REQUEST_MESSAGE | 512 | YES (partial — adapter.rs case 520) |
| MAV_CMD_DO_FOLLOW | 32 | NO |
| MAV_CMD_NAV_TAKEOFF | 22 | YES |
| MAV_CMD_DO_MOTOR_TEST | 209 | YES (stub) |

**Vehicle-subclass-only MAV_CMDs** (not in common layer, need ArduCopter/ArduPlane override):
MAV_CMD_NAV_LAND (21), MAV_CMD_NAV_RETURN_TO_LAUNCH (20), MAV_CMD_NAV_LOITER_UNLIM (17), MAV_CMD_NAV_WAYPOINT (16), MAV_CMD_DO_CHANGE_SPEED (178), MAV_CMD_CONDITION_YAW (115), MAV_CMD_DO_PAUSE_CONTINUE (193), MAV_CMD_NAV_VTOL_TAKEOFF (84), MAV_CMD_NAV_VTOL_LAND (85).

---

## 4. GCS_MAVLink — Support Systems

### 4.1 MAVLink Routing

```
FILE:  libraries/GCS_MAVLink/MAVLink_routing.cpp
LINES: 468
FUNCTION: Multi-link packet routing. Learns routes from received messages,
          forwards targeted messages to correct channel, broadcasts to all
          non-private channels. 20-entry route table, sysid/compid/mavtype
          per route. Prevents feedback loops via no_route_mask.
MERIDIAN HAS: NO — single-link only, no routing table
GAPS: Route learning, multi-hop forwarding, private channel semantics,
      no_route_mask, broadcast vs unicast logic.
```

### 4.2 MAVLink Signing (HMAC-SHA256)

```
FILE:  libraries/GCS_MAVLink/GCS_Signing.cpp
LINES: 278
FUNCTION: MAVLINK_MSG_ID_SETUP_SIGNING handler, per-channel signing state
          (sign_outgoing flag, 48-bit timestamps, HMAC-SHA256 over header+payload).
          Secret key stored in EEPROM. Blocks unsigned messages when signing enabled.
MERIDIAN HAS: STUB (signing.rs exists, no implementation)
GAPS: Key storage, timestamp monotonicity check, per-message HMAC, 
      SETUP_SIGNING handler, accept_unsigned callback.
```

### 4.3 MAVFTP (MAVLink File Transfer Protocol)

```
FILE:  libraries/GCS_MAVLink/GCS_FTP.cpp / GCS_FTP.h
LINES: 840 / 118
FUNCTION: Full MAVFTP over FILE_TRANSFER_PROTOCOL (#110). Supports:
          OpenFileRO, OpenFileWO, CreateFile, CreateDirectory, RemoveDirectory,
          RemoveFile, TruncateFile, Rename, ReadFile, WriteFile,
          ListDirectory, CalcFileCRC32, BurstReadFile, TerminateSession.
          Async session state machine, CRC32 verification.
MERIDIAN HAS: STUB (mavftp.rs — no implementation)
GAPS: All operations. Session state machine. CRC32. Directory listing.
```

### 4.4 Log Download

```
FILE:  libraries/GCS_MAVLink/GCS_Common.cpp (log handling section)
LINES: ~200 lines spread through file
FUNCTION: LOG_REQUEST_LIST, LOG_REQUEST_DATA, LOG_ERASE, LOG_REQUEST_END,
          REMOTE_LOG_BLOCK_STATUS. Streams log entries in chunks.
MERIDIAN HAS: STUB (log_download.rs — no implementation)
GAPS: All log protocol operations.
```

### 4.5 MissionItemProtocol — Waypoints, Fence, Rally

```
FILE:  libraries/GCS_MAVLink/MissionItemProtocol*.cpp (4 files)
LINES: ~1100 total
FUNCTION: State machine for mission upload/download for each of three
          item types (waypoints, fence, rally). Handles partial writes,
          timeouts, NAK on bad items. MAVLink v2 MISSION_ITEM_INT only.
MERIDIAN HAS: PARTIAL — server.rs handles waypoints only (128 item max),
              no fence/rally protocol, no partial write, no timeout.
GAPS: Fence protocol, rally protocol, MISSION_ITEM (float, legacy),
      partial write handler, sequence gap recovery.
```

### 4.6 Device Operations (I2C/SPI passthrough)

```
FILE:  libraries/GCS_MAVLink/GCS_DeviceOp.cpp
LINES: 150
FUNCTION: DEVICE_OP_READ / DEVICE_OP_WRITE — lets GCS read/write raw I2C/SPI
          registers on the FC. Used by diagnostic tools.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

### 4.7 Serial Control (debug terminal)

```
FILE:  libraries/GCS_MAVLink/GCS_serial_control.cpp
LINES: 206
FUNCTION: MAVLINK_MSG_ID_SERIAL_CONTROL — raw byte tunnel to any serial port.
          Used by Mission Planner firmware update and SiK radio config.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

---

## 5. AP_SerialManager — Serial Protocol Registry

```
FILE:  libraries/AP_SerialManager/AP_SerialManager.cpp / .h
LINES: 920 / 242
FUNCTION: Central registry for all serial port protocol assignments.
          Each of up to 10 UART ports is assigned one SerialProtocol enum value.
          Provides find_serial(protocol, instance) for all consumers.
          Handles baud rate, flow control, inversion, FIFO sizing per protocol.
MERIDIAN HAS: PARTIAL — meridian-boardcfg has UartConfig with default_protocol u8,
              no runtime protocol assignment, no find_serial API.
```

**All 51 SerialProtocol values (ArduPilot) vs Meridian:**

| ID | Protocol | Meridian |
|----|---------|----------|
| 0 | None | YES (boardcfg recognizes 0) |
| 1 | MAVLink | YES |
| 2 | MAVLink2 (default) | YES |
| 3 | FrSky_D | NO |
| 4 | FrSky_SPort | NO |
| 5 | GPS | YES (boardcfg assigns to GPS port) |
| 6 | GPS2 | PARTIAL (same as 5) |
| 7 | AlexMos (gimbal) | NO |
| 8 | Gimbal (SToRM32/Siyi) | NO |
| 9 | Rangefinder | NO |
| 10 | FrSky_SPort_Passthrough | NO |
| 11 | Lidar360 | NO |
| 12 | Aerotenna_USD1 (deprecated) | NO |
| 13 | Beacon | NO |
| 14 | Volz servo | NO |
| 15 | SBUS1 output | NO |
| 16 | ESCTelemetry | NO |
| 17 | Devo_Telem | NO |
| 18 | OpticalFlow | NO |
| 19 | Robotis servo | NO |
| 20 | NMEAOutput | NO |
| 21 | WindVane | NO |
| 22 | SLCAN (CAN-over-serial) | NO |
| 23 | RCIN | YES (meridian-rc protocol parsers) |
| 24 | EFI | NO |
| 25 | LTM_Telem | NO |
| 26 | RunCam | NO |
| 27 | Hott | NO |
| 28 | Scripting | NO |
| 29 | CRSF | YES (meridian-rc has CrsfParser) |
| 30 | Generator | NO |
| 31 | Winch | NO |
| 32 | MSP | NO |
| 33 | DJI_FPV | NO |
| 34 | AirSpeed | NO |
| 35 | ADSB | NO |
| 36 | AHRS (external) | NO |
| 37 | SmartAudio (VTX) | NO |
| 38 | FETtecOneWire | NO |
| 39 | Torqeedo (electric motor) | NO |
| 40 | AIS (maritime) | NO |
| 41 | CoDevESC | NO |
| 42 | MSP_DisplayPort | NO |
| 43 | MAVLinkHL (high latency) | NO |
| 44 | Tramp (VTX) | NO |
| 45 | DDS_XRCE (ROS 2) | NO |
| 46 | IMUOUT | NO |
| 48 | PPP | NO |
| 49 | IBUS_Telem | NO |
| 50 | IOMCU | NO |

**Gaps**: No runtime `SerialManager` equivalent. Meridian has no protocol-dispatch-by-enum, no `find_serial()` API, no FIFO/baud parameter table, no protocol-match logic (MAVLink/MAVLink2/MAVLinkHL are interchangeable in ArduPilot).

---

## 6. AP_RCProtocol — Every Backend

```
FILE:  libraries/AP_RCProtocol/AP_RCProtocol.cpp / .h
LINES: 751 / 329
FUNCTION: Frontend singleton. Auto-detects RC protocol from serial or pulse input.
          Owns one backend pointer per protocol enum. Handles baud-rate rotation
          for UART scanning, protocol locking after 1-3 good frames, re-search on
          200ms timeout. MAX_RCIN_CHANNELS=18, MIN_RCIN_CHANNELS=5.
MERIDIAN HAS: NO frontend — individual parsers only (CrsfParser, SbusParser in lib.rs)
GAPS: Auto-detection state machine, baud rotation, protocol locking, re-search,
      pulse input path, UART scanning loop, multi-receiver support, IOMCU priority.
```

### Backend — PPMSum

```
FILE:  AP_RCProtocol_PPMSum.cpp / .h (69 / 37 lines)
FUNCTION: Pulse-position-modulation input. Timer-capture of rising/falling edges.
          Sync gap > 2.7ms separates frames. Up to 18 channels at 1000-2000us.
          Requires hardware pulse capture (no UART path).
MERIDIAN HAS: YES — ppm.rs implements PPM decoder
GAPS: None critical (pulse edge timing accuracy depends on HAL).
```

### Backend — SBUS

```
FILE:  AP_RCProtocol_SBUS.cpp / .h (216 / 50 lines)
FUNCTION: FrSky SBUS: 25-byte frame, 100000 baud inverted 8E2.
          11-bit channels (172-1811 range → 988-2012 PWM scaling).
          Supports both inverted (SBUS) and non-inverted (SBUS_NI).
          FastSBUS variant at 200000 baud.
          Frame16/17 digital channels + failsafe byte.
MERIDIAN HAS: YES — SbusParser in lib.rs
GAPS: FastSBUS variant (200k baud), SBUS_NI (non-inverted variant),
      digital channel bits (frame16/17), SBUS output (not just input).
```

### Backend — DSM

```
FILE:  AP_RCProtocol_DSM.cpp / .h (542 / 86 lines)
FUNCTION: Spektrum DSM/DSMX satellite receiver. 115200 baud, 16-byte frames.
          Resolution detection (DSM2 1024/2048, DSMX always 2048).
          RC bind via bind_request(). Auto-detects channel count (7 vs 11).
          Interleaved frames for >7 channels.
MERIDIAN HAS: YES — dsm.rs module
GAPS: Need to verify dsm.rs covers: bind_request, resolution auto-detect,
      11-channel interleaving, 1024-mode decoding.
```

### Backend — CRSF

```
FILE:  AP_RCProtocol_CRSF.cpp / .h (744 / 255 lines)
FUNCTION: ExpressLRS / TBS Crossfire. 420000 baud default, CRSFv3 up to 2Mbps.
          Frame types: RC_CHANNELS (0x16), LINK_STATISTICS (0x14),
          LINK_STATISTICS_TX (0x1C), LINK_STATISTICS_RX (0x1D),
          SUBSET_RC_CHANNELS (0x17, ELRS), GPS (0x02), VTX_STATUS (0x10).
          Bidirectional: TX commands (0x32): SET_VTX_CONFIG, PING_DEVICES,
          SET_ATTITUDE, SET_FLIGHT_MODE, SET_BATTERY, SET_GPS, DEVICE_INFO,
          DEVICE_PING (0x28), DEVICE_INFO (0x29), GET_EXTENDED_TYPE (0x31).
          Baud-rate negotiation for CRSFv3.
MERIDIAN HAS: YES — CrsfParser in lib.rs
GAPS: Bidirectional TX command handling (device ping, VTX config commands),
      SUBSET_RC_CHANNELS (0x17, ELRS packed format), CRSFv3 baud negotiation
      is present (CrsfV3Negotiation struct) but needs wiring to serial layer.
      TX output path (send telemetry frames back to TX module).
```

### Backend — FPort

```
FILE:  AP_RCProtocol_FPort.cpp / .h (322 / 70 lines)
FUNCTION: FrSky FPort v1: single-wire half-duplex UART at 115200 baud inverted.
          Combines RC input + telemetry on one wire. 25-byte RC frame identical
          to SBUS followed by control byte. Telemetry polls happen in 8ms gaps.
MERIDIAN HAS: YES — fport.rs module
GAPS: Telemetry response path (fport.rs appears to only parse RX, not TX
      side of the half-duplex exchange).
```

### Backend — FPort2

```
FILE:  AP_RCProtocol_FPort2.cpp / .h (301 / 65 lines)
FUNCTION: FrSky FPort v2: similar to FPort but with enhanced features,
          32-byte frames, different framing than FPort v1.
MERIDIAN HAS: NO — not in meridian-rc
GAPS: Full implementation.
```

### Backend — IBUS

```
FILE:  AP_RCProtocol_IBUS.cpp / .h (108 / 50 lines)
FUNCTION: FlySky iBus RC input: 32-byte frame, 115200 baud.
          14 channels at 1000-2000us. 2-byte checksum.
MERIDIAN HAS: YES — ibus.rs module
GAPS: None apparent from file size (108 lines matches ibus.rs).
```

### Backend — SUMD

```
FILE:  AP_RCProtocol_SUMD.cpp / .h (210 / 64 lines)
FUNCTION: Graupner/HoTT SUMD: 19+ byte frame, up to 16 channels.
          3000us range (3000-24000), CRC16-CCITT.
          Supports SUMD (regular) and SUMH (HoTT-specific variant).
MERIDIAN HAS: YES — sumd.rs module
GAPS: SUMH variant support.
```

### Backend — SRXL

```
FILE:  AP_RCProtocol_SRXL.cpp / .h (289 / 72 lines)
FUNCTION: Spektrum SRXL v1/v2 (older unidirectional). 115200 baud.
          Variable-length frames (24-80 bytes), 8-bit XOR checksum.
          Channels at 800-2200us scale.
MERIDIAN HAS: NO — srxl2.rs exists but SRXL v1 not present
GAPS: Full SRXL (v1/v2) implementation. Note: srxl2.rs is SRXL2, distinct.
```

### Backend — SRXL2

```
FILE:  AP_RCProtocol_SRXL2.cpp / .h (397 / 75 lines) + spm_srxl.cpp (1430 lines)
FUNCTION: Spektrum SRXL2 (bidirectional). 115200 baud. 
          Full Spektrum SRXL2 spec: handshake, device enumeration,
          telemetry exchange (SRXL_TELEM_* types), bind mode.
          Uses Spektrum SRXL library (spm_srxl.cpp, 1430 lines).
MERIDIAN HAS: YES — srxl2.rs module
GAPS: Need to verify srxl2.rs covers: device handshake, bidirectional
      telemetry frames (not just RC input), bind request.
```

### Backend — ST24

```
FILE:  AP_RCProtocol_ST24.cpp / .h (243 / 157 lines)
FUNCTION: Yuneec ST24 transmitter protocol. 115200 baud.
          Variable-length packets: StickPacket (RC channels),
          RcCodeChannelPacket, SetChannelPacket.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

### Backend — GHST (ImmersionRC Ghost)

```
FILE:  AP_RCProtocol_GHST.cpp / .h (472 / 198 lines)
FUNCTION: ImmersionRC Ghost 420000 baud. Similar to CRSF frame structure.
          Frame types: GHST_UL_RC_CHANS_HS4_12 (primary RC),
          GHST_UL_RC_CHANS_HS4_5TO8, _9TO12, _13TO16 (extra channels),
          GHST_UL_MENU_CTRL (OSD menu), GHST_DL_OPENTX_SYNC (timing).
          Telemetry downlink frames: battery, GPS, attitude, VTX.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

### Backend — DroneCAN RC

```
FILE:  AP_RCProtocol_DroneCAN.cpp / .h (138 / 74 lines)
FUNCTION: RC input received over DroneCAN bus from a CAN-connected RC receiver.
          Uses uavcan.equipment.rc.ChannelsPackedArray message.
MERIDIAN HAS: NO — meridian-can has DroneCAN node but not RC receiver subscription
GAPS: Subscribe to uavcan.equipment.rc.ChannelsPackedArray, decode channel values.
```

### Backend — MAVLinkRadio

```
FILE:  AP_RCProtocol_MAVLinkRadio.cpp / .h (25 / 21 lines)
FUNCTION: RC input from MAVLINK_MSG_ID_RADIO_RC_CHANNELS — lets a companion
          computer inject RC channels over MAVLink.
MERIDIAN HAS: NO
GAPS: handle_radio_rc_channels() → feed RC protocol backend.
```

### Backend — IOMCU

```
FILE:  AP_RCProtocol_IOMCU.cpp / .h (49 / 32 lines)
FUNCTION: RC input from IO MCU co-processor (Pixhawk-style boards with dual CPUs).
          IO MCU handles RC capture and sends to main MCU via IOMCU protocol.
MERIDIAN HAS: NO
GAPS: IOMCU IPC path. Only needed for Pixhawk-compatible boards.
```

### Backend — Emlid RCIO

```
FILE:  AP_RCProtocol_Emlid_RCIO.cpp / .h (68 / 28 lines)
FUNCTION: RC input from Emlid RCIO co-processor (Navio2 hat for Raspberry Pi).
          SPI-based RC input.
MERIDIAN HAS: NO
GAPS: Emlid-specific hardware backend. Low priority.
```

### Backend — FDM (Flight Dynamics Model, SITL)

```
FILE:  AP_RCProtocol_FDM.cpp / .h (50 / 24 lines)
FUNCTION: SITL-only: RC input injected directly from flight dynamics model,
          bypassing serial parsing. Used by ArduPilot SITL.
MERIDIAN HAS: NO (meridian-sitl exists but injects RC differently)
GAPS: SITL RC injection path.
```

### Backend — UDP

```
FILE:  AP_RCProtocol_UDP.cpp / .h (189 / 54 lines)
FUNCTION: RC input over UDP (sim/companion link). Receives raw channel
          data over UDP socket, useful for HIL/simulation.
MERIDIAN HAS: NO
GAPS: UDP RC input.
```

### Backend — Joystick_SFML (Linux desktop)

```
FILE:  AP_RCProtocol_Joystick_SFML.cpp / .h (48 / 24 lines)
FUNCTION: Linux/SFML game controller input. Only used in SITL on desktop.
MERIDIAN HAS: NO
GAPS: Low priority — SITL only.
```

### Backend — Radio (AP_Radio, SPI)

```
FILE:  AP_RCProtocol_Radio.cpp / .h (53 / 32 lines)
FUNCTION: SPI-connected radio module (AP_Radio: Cyrf6936, CC2500, BK2425).
          Used for bidirectional link with Skydroid H12 and similar radios.
MERIDIAN HAS: NO
GAPS: SPI radio driver path. Low priority (rare hardware).
```

---

## 7. AP_RCTelemetry — CRSF Telemetry

```
FILE:  libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp / .h
LINES: 2333 / 487
FUNCTION: Downlink telemetry from FC to TX module over CRSF serial.
          Scheduler with 20 time slots, sending sensor-specific frames.
MERIDIAN HAS: PARTIAL — CrsfTelemetry encoder in lib.rs (battery, GPS, attitude,
              flight_mode, VTX frames). No scheduler, no full frame set.
```

**Every CRSF telemetry frame type ArduPilot sends:**

| Frame Type | Byte ID | Content | Meridian |
|------------|---------|---------|----------|
| CRSF_FRAMETYPE_GPS | 0x02 | lat, lon, groundspeed, heading, altitude, sats | YES (encode_gps) |
| CRSF_FRAMETYPE_BATTERY_SENSOR | 0x08 | voltage, current, mAh, remaining% | YES (encode_battery) |
| CRSF_FRAMETYPE_LINK_STATISTICS | 0x14 | RSSI ant1/2, LQ, SNR, antenna, RF mode, power | NO encoder |
| CRSF_FRAMETYPE_RC_CHANNELS_PACKED | 0x16 | 16 channels 11-bit | NO (decode only) |
| CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED | 0x17 | ELRS subset | NO |
| CRSF_FRAMETYPE_LINK_RX_ID | 0x1C | RX link stats | NO |
| CRSF_FRAMETYPE_LINK_TX_ID | 0x1D | TX link stats | NO |
| CRSF_FRAMETYPE_ATTITUDE | 0x1E | pitch, roll, yaw (rad, x10000) | YES (encode_attitude) |
| CRSF_FRAMETYPE_FLIGHT_MODE | 0x21 | mode string (null-term) | YES (encode_flight_mode) |
| CRSF_FRAMETYPE_DEVICE_PING | 0x28 | ping for device discovery | NO |
| CRSF_FRAMETYPE_DEVICE_INFO | 0x29 | device name, serial, hw/sw version | NO |
| CRSF_FRAMETYPE_GET_EXTENDED_TYPE | 0x31 | query extended type | NO |
| CRSF_FRAMETYPE_COMMAND | 0x32 | sub-commands (VTX, FC, RX) | NO |
| CRSF_FRAMETYPE_VTX_TELEMETRY | 0x10 | VTX freq, power, pitmode | YES (encode_vtx) |
| CRSF_FRAMETYPE_PARAM_DEVICE_INFO | 0x3A | param device info | NO |
| CRSF_FRAMETYPE_PARAM_ENTRY | 0x3B | parameter definition | NO |
| CRSF_FRAMETYPE_PARAM_READ | 0x3C | read param | NO |
| CRSF_FRAMETYPE_PARAM_WRITE | 0x3D | write param | NO |
| CRSF_FRAMETYPE_ELRS_STATUS | 0x2E | ELRS status frame | NO |

**Gaps**: CRSF telemetry scheduler (20-slot round-robin), LINK_STATISTICS encoder, device ping/info exchange, CRSF parameter protocol (0x3A-0x3D), ELRS-specific frames.

---

## 8. AP_RCTelemetry — GHST Telemetry

```
FILE:  libraries/AP_RCTelemetry/AP_GHST_Telem.cpp / .h
LINES: 391 / 166
FUNCTION: ImmersionRC Ghost downlink telemetry.
          Scheduler with time slots, sending battery, GPS, VTX, attitude frames.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**GHST telemetry frames:**

| Frame | Content |
|-------|---------|
| GHST_DL_PACK_STAT | Battery voltage, current, capacity, RSSI |
| GHST_DL_GPS_PRIMARY | Lat, lon, altitude, ground speed, heading, sats |
| GHST_DL_GPS_SEC | Distance to home, vertical speed, heading, flags |
| GHST_DL_MAGBARO | Compass heading, baro altitude, vario |
| GHST_DL_VTX_STAT | VTX frequency, power, pitmode, spectral scan |

---

## 9. AP_RCTelemetry — Spektrum Telemetry

```
FILE:  libraries/AP_RCTelemetry/AP_Spektrum_Telem.cpp / .h
LINES: 645 / 140 (+ spektrumTelemetrySensors.h: 1354 lines)
FUNCTION: SRXL2 bidirectional telemetry. Sends 40+ sensor types as
          defined in spektrumTelemetrySensors.h. Scheduler-based.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**Spektrum sensor types (partial list from spektrumTelemetrySensors.h):**
SPKT_TELE_TYPE_AMPS, SPKT_TELE_TYPE_ALTITUDE, SPKT_TELE_TYPE_GPS_LOC, SPKT_TELE_TYPE_GPS_STAT, SPKT_TELE_TYPE_ATTITUDE, SPKT_TELE_TYPE_TEMP, SPKT_TELE_TYPE_BATTERY, SPKT_TELE_TYPE_BARO_ALT, SPKT_TELE_TYPE_VARIO, SPKT_TELE_TYPE_RPM, SPKT_TELE_TYPE_AIRSPEED, and ~30 more. Total sensor definitions: 1354 lines.

---

## 10. AP_Frsky_Telem — FrSky D Protocol

```
FILE:  libraries/AP_Frsky_Telem/AP_Frsky_D.cpp / .h
LINES: 88 / 35
FUNCTION: Legacy FrSky D-series protocol for older D-receivers (D4R-II, etc.).
          One-way 9600 baud. Sends 11 fixed sensor IDs at fixed rates.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**FrSky D data IDs sent:**
GPS_ALT_BP (0x01), TEMP1 (0x02), FUEL (0x04), TEMP2 (0x05), GPS_ALT_AP (0x09), BARO_ALT_BP (0x10), GPS_SPEED_BP (0x11), GPS_LONG_BP (0x12), GPS_LAT_BP (0x13), GPS_COURS_BP (0x14), GPS_SPEED_AP (0x19), GPS_LONG_AP (0x1A), GPS_LAT_AP (0x1B), BARO_ALT_AP (0x21), GPS_LONG_EW (0x22), GPS_LAT_NS (0x23), CURRENT (0x28), VFAS (0x39).

---

## 11. AP_Frsky_Telem — FrSky SPort

```
FILE:  libraries/AP_Frsky_Telem/AP_Frsky_SPort.cpp / .h
LINES: 482 / 70
FUNCTION: FrSky S.Port protocol for X-series receivers (X8R, X4R-SB, etc.).
          Half-duplex, poll-response. TX polls sensor IDs; FC responds.
          Each response is a 8-byte sport_packet_t. Scheduler with 
          round-robin across active sensors.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**S.Port sensor IDs and data IDs:**
- VARIO sensor (0x00): ALT_ID (0x010F), VARIO_ID (0x011F)
- FAS sensor (0x22): CURR_ID (0x020F), VFAS_ID (0x021F)
- GPS sensor (0x83): GPS_LONG_LATI (0x0800), GPS_ALT_ID (0x082F), GPS_SPEED_ID (0x083F), GPS_COURS_ID (0x084F)
- RPM sensor (0xE4): RPM1_ID (0x050E), RPM2_ID (0x050F)
- SP2UR sensor (0xC6): TEMP1_ID (0x040F), TEMP2_ID (0x041F), FUEL_ID (0x060F)

---

## 12. AP_Frsky_Telem — FrSky SPort Passthrough

```
FILE:  libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp / .h
LINES: 976 / 180
FUNCTION: OpenTX/EdgeTX passthrough mode. Uses DIY_FIRST_ID (0x5000) range
          to pack compressed telemetry into 4-byte payloads.
          15 distinct DIY data IDs, round-robin scheduler with priority.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**Passthrough data IDs (0x5000 range):**
| ID | Hex | Content |
|----|-----|---------|
| STATUS_TEXT | 0x5000 | Compressed status messages (5 chars/frame, 50-char queue) |
| AP_STATUS | 0x5001 | Armed, failsafe, flight mode, landed state, throttle% |
| GPS_STATUS | 0x5002 | GPS fix type, HDOP, VDOP, sats visible, sats used |
| HOME | 0x5004 | Distance to home, bearing, altitude vs home |
| VEL_YAW | 0x5005 | Velocity (N, E, D) + yaw |
| ATTITUDE | 0x5006 | Roll, pitch (0.2-deg resolution), range (cm) |
| PARAM | 0x5007 | Parameter values for LUA scripts |
| BATT_1 | 0x5008 | Cell voltage (cV), current, mAh, remaining% |
| BATT_2 | 0x5008 | Second battery |
| RPM | 0x500A | RPM sensors 1 and 2 |
| TERRAIN | 0x500B | Terrain height, terrain health |
| WIND | 0x500C | Wind speed, direction |
| WAYPOINT | 0x500D | Next waypoint bearing, distance |

---

## 13. AP_Frsky_Telem — MAVlite Bidirectional

```
FILE:  libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp / _MAVliteToSPort.cpp / _SPortToMAVlite.cpp
LINES: 295 / 99 / 105
FUNCTION: MAVlite protocol: a compact MAVLink-like protocol tunneled over FrSky
          SPort passthrough (data ID 0x5000 slot 12). Allows COMMAND_LONG,
          PARAM_SET, PARAM_REQUEST_READ over FrSky link.
          SPortToMAVlite: parses incoming SPort frames → MAVlite messages.
          MAVliteToSPort: encodes MAVlite replies → SPort packets.
          Handler: processes MAV_CMD_DO_SET_MODE, MAV_CMD_PREFLIGHT_REBOOT,
          MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_PREFLIGHT_CALIBRATION,
          PARAM_SET, PARAM_REQUEST_READ.
MERIDIAN HAS: NO
GAPS: Full MAVlite stack.
```

---

## 14. AP_CRSF — CRSF Protocol Library

```
FILE:  libraries/AP_CRSF/AP_CRSF_Protocol.cpp / .h
LINES: Not separately measured (small, included by AP_RCProtocol_CRSF)
       AP_CRSF_config.h: compile-time feature flags
FUNCTION: Shared CRSF frame type constants, build config for CRSF features.
          AP_CRSF_TELEM_ENABLED, AP_CRSF_RCPROTOCOL_ENABLED compile switches.
MERIDIAN HAS: YES (constants defined inline in meridian-rc/src/lib.rs)
GAPS: None for basic constants. Meridian lacks the compile-feature guard system.
```

---

## 15. AP_Hott_Telem — HoTT Telemetry

```
FILE:  libraries/AP_Hott_Telem/AP_Hott_Telem.cpp / .h
LINES: 454 / 56
FUNCTION: Graupner HoTT bidirectional wireless telemetry.
          Poll-response protocol. Two sensor modules: EAM (Electric Air Module)
          and GPS. Each sends 45-byte sensor packets on request.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**HoTT sensor packet types:**
- **EAM (Electric Air Module)**: battery cell voltages (cells 1-14), voltage, current, capacity, temperature1/2, altitude, climb rate, speed.
- **GPS module**: lat, lon, distance, altitude, speed, heading, sats, fix type, NN/EW/SE flags.

---

## 16. AP_LTM_Telem — Lightweight Telemetry

```
FILE:  libraries/AP_LTM_Telem/AP_LTM_Telem.cpp / .h
LINES: 238 / 54
FUNCTION: LTM (Lightweight Telemetry) — compact one-way serial protocol
          for FPV OSD systems and iNav. Three frame types at defined rates.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**LTM frame types:**
| Frame | Rate | Content |
|-------|------|---------|
| G (GPS) | 5 Hz | lat, lon, speed, alt, sats, fix type, heading |
| S (Status) | 2 Hz | voltage, current, RSSI, airspeed, arm/failsafe/GPS flags, mode |
| A (Attitude) | 10 Hz | pitch, roll, heading |

---

## 17. AP_IBus_Telem — iBus Telemetry

```
FILE:  libraries/AP_IBus_Telem/AP_IBus_Telem.cpp / .h
LINES: 792 / 126
FUNCTION: FlySky iBus sensor protocol. Bidirectional over UART.
          FC acts as a sensor hub; TX polls for sensor discovery
          (IBUSS_DISC command) then queries values (IBUSS_MEAS command).
          19 sensors configured in default sensor table.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**Default iBus sensor table (19 entries):**
ARMED (0x15), FLIGHT_MODE (0x16), GPS_STATUS (0x0B), FUEL (0x06), EXTERNAL_VOLTAGE (0x03), ALT (0x83 = 4-byte), GPS_DIST (0x14), CLIMB_RATE (0x09), GROUND_SPEED (0x13), ROLL (0x0F), PITCH (0x10), YAW (0x11), SPEED (0x7E), TEMPERATURE_PRESSURE (0x41 = 4-byte), RPM (0x07), BATTERY_CURRENT (0x05), AVERAGE_CELL_VOLTAGE (0x04), COMPASS_HEADING (0x08), and GPS_LAT/LNG/ALT as optional extras.

---

## 18. AP_Devo_Telem — Walkera DEVO

```
FILE:  libraries/AP_Devo_Telem/AP_Devo_Telem.cpp / .h
LINES: 139 / 49
FUNCTION: Walkera DEVO transmitter telemetry protocol over UART.
          8 fixed fields at ~10Hz: lon, lat, alt, speed, roll, pitch, yaw, voltage.
MERIDIAN HAS: NO
GAPS: Full implementation. Low priority (obscure protocol).
```

---

## 19. AP_CANManager — CAN Bus Management

```
FILE:  libraries/AP_CANManager/AP_CANManager.cpp / .h
LINES: 477 / 188
FUNCTION: Manages up to AP_CAN_MAX_PROTOCOL_DRIVERS (3) CAN protocol drivers.
          Each CAN bus can run one driver: DroneCAN, KDECAN, PiccoloCAN, or
          MAVLinkCAN passthrough. Singleton AP_CANManager. Handles driver
          instantiation, thread creation (one thread per driver), parameter tables.
MERIDIAN HAS: NO singleton manager — meridian-can has CanardNode (DroneCAN only)
GAPS: Multi-driver management, per-bus driver assignment, CAN thread management,
      KDECAN and PiccoloCAN driver slots.
```

### AP_SLCANIface — CAN-over-Serial

```
FILE:  libraries/AP_CANManager/AP_SLCANIface.cpp / .h
LINES: 745 / 146
FUNCTION: SLCAN protocol (CAN frames over ASCII serial, Lawicel format).
          Used for CAN sniffing/injection via USB serial. Supports SLCAN v1.1.
          Implements: open channel (O), close (C), set bitrate (Sn), transmit
          standard (t) and extended (T) frames, receive notification.
MERIDIAN HAS: NO
GAPS: Full SLCAN implementation.
```

### AP_MAVLinkCAN — CAN passthrough over MAVLink

```
FILE:  libraries/AP_CANManager/AP_MAVLinkCAN.cpp / .h
LINES: 360 / 87
FUNCTION: Bridges MAVLink CAN_FRAME/CANFD_FRAME messages to the CAN bus.
          Allows GCS to send/receive raw CAN frames via MAVLink.
          Handles CAN_FILTER_MODIFY to select which IDs to bridge.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

---

## 20. AP_DroneCAN — DroneCAN (UAVCAN v0)

```
FILE:  libraries/AP_DroneCAN/AP_DroneCAN.cpp / .h
LINES: 2029 / 449
FUNCTION: Full DroneCAN stack. Publishers and subscribers listed below.
          DNA server for dynamic node allocation. Separate thread per instance.
MERIDIAN HAS: PARTIAL — meridian-can has CanardNode, DNA server, CAN frame
              encode/decode, ESC command, GPS/Compass/Baro/Battery sensor adapters.
```

**AP_DroneCAN Outbound Publishers:**

| Message | DroneCAN Type | Meridian |
|---------|--------------|---------|
| Node Status | uavcan.protocol.NodeStatus | YES (maybe_send_heartbeat) |
| Safety State | ardupilot.indication.SafetyState | YES (send_safety_state) |
| Arming Status | uavcan.equipment.safety.ArmingStatus | YES (send_arming_status) |
| Notify State | ardupilot.indication.NotifyState | YES (send_lights) |
| ESC Raw Command | uavcan.equipment.esc.RawCommand | YES (send_esc_command) |
| Actuator Array Command | uavcan.equipment.actuator.ArrayCommand | NO |
| GNSS Fix2 (GPS TX) | uavcan.equipment.gnss.Fix2 | NO |
| GPS Auxiliary | uavcan.equipment.gnss.Auxiliary | NO |
| Himark servo command | com.himark.servo.ServoCmd | NO |
| Node restart request | uavcan.protocol.RestartNode | NO |
| Param get/set request | uavcan.protocol.param.GetSet | NO |

**AP_DroneCAN Inbound Subscribers:**

| Message | DroneCAN Type | Meridian |
|---------|--------------|---------|
| ESC Status | uavcan.equipment.esc.Status | YES (sensors.rs — CanBattery processes it) |
| ESC Status Extended | uavcan.equipment.esc.StatusExtended | NO |
| Actuator Status | uavcan.equipment.actuator.Status | NO |
| Button (safety switch) | ardupilot.indication.Button | NO |
| Traffic Report (ADSB) | ardupilot.equipment.trafficmonitor.TrafficReport | NO |
| Debug Log Message | uavcan.protocol.debug.LogMessage | NO |
| Himark servo info | com.himark.servo.ServoInfo | NO |
| Volz actuator status | com.volz.servo.ActuatorStatus | NO |
| Param GetSet response | uavcan.protocol.param.GetSetResponse | NO |
| Param save response | uavcan.protocol.param.ExecuteOpcodeResponse | NO |
| Restart node response | uavcan.protocol.RestartNodeResponse | NO |
| Node info request | uavcan.protocol.GetNodeInfoRequest | NO |
| FlexDebug | dronecan.protocol.FlexDebug | NO |
| Hobbywing ESC GetEscID | com.hobbywing.esc.GetEscID | NO |
| Hobbywing StatusMsg1 | com.hobbywing.esc.StatusMsg1 | NO |
| Hobbywing StatusMsg2 | com.hobbywing.esc.StatusMsg2 | NO |
| RC Channels | uavcan.equipment.rc.ChannelsPackedArray | NO (RC backend) |

**Sensor drivers that consume DroneCAN** (AP_DroneCAN integrates with):
AP_GPS_DroneCAN, AP_Compass_DroneCAN, AP_Baro_DroneCAN, AP_BattMonitor_DroneCAN, AP_Airspeed_DroneCAN, AP_RangeFinder_DroneCAN, AP_ADSB_DroneCAN.

**Gaps in meridian-can vs ArduPilot:**
- No actuator array command (PWM output via CAN)
- No GPS TX (broadcasting GPS over DroneCAN bus)
- No parameter get/set protocol
- No node restart
- No ESC Status Extended
- No debug log / FlexDebug
- No safety button subscriber
- No Hobbywing ESC vendor extension
- No Volz servo status
- No RC channels subscriber (needed for DroneCAN RC input)
- DNA server present but not tested against real hardware sequences

---

## 21. AP_KDECAN — KDE Direct ESC Protocol

```
FILE:  libraries/AP_KDECAN/AP_KDECAN.cpp / .h
LINES: 302 / 143
FUNCTION: Proprietary CAN protocol for KDE Direct UAS ESCs.
          Message objects: NODE_ID (0x00), TELEMETRY_OBJ (0x11),
          FLIGHT_CTRL_OBJ (0x20), MOTOR_CMD_OBJ (0x2F), ESC_INFO_OBJ (0x40).
          Sends: motor throttle commands, safety/arming state.
          Receives: telemetry (voltage, current, RPM, temperature per ESC).
          Separate thread, mutex-protected command queue.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

---

## 22. AP_PiccoloCAN — Swift Navigation Piccolo Protocol

```
FILE:  libraries/AP_PiccoloCAN/AP_PiccoloCAN.cpp + 4 sub-files
LINES: 797 + ~460 = ~1260 total
FUNCTION: Swift Navigation Piccolo CAN protocol for avionics-grade actuators.
          Sub-drivers: ESC (AP_PiccoloCAN_ESC), Servo (AP_PiccoloCAN_Servo),
          ECU (AP_PiccoloCAN_ECU), Cortex (AP_PiccoloCAN_Cortex).
          Sends: throttle commands (ESC), position commands (servo).
          Receives: ESC telemetry (voltage, current, RPM, temp), servo feedback.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

---

## 23. AP_MSP — MultiWii Serial Protocol

```
FILE:  libraries/AP_MSP/AP_MSP.cpp / AP_MSP_Telem_Backend.cpp / AP_MSP_Telem_DJI.cpp / AP_MSP_Telem_DisplayPort.cpp
LINES: 244 / 1328 / 252 / 56
FUNCTION: MSP v1/v2 protocol for Betaflight/iNav OSD compatibility.
          Three backends:
          - Generic: standard MSP OSD (all Betaflight-compatible OSDs)
          - DJI: DJI FPV goggles custom MSP (DJII_FPV, DJII_O3)
          - DisplayPort: canvas-mode OSD (HD zero, Avatar)
          Scheduler with 12 time slots.
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**MSP outbound packets sent by ArduPilot (12 scheduled slots):**

| MSP ID | Content | Rate |
|--------|---------|------|
| MSP_NAME (10) | Flight controller name | Scheduled |
| MSP_STATUS (150) | Arm state, flight mode, failsafe, sensor health | Scheduled |
| MSP_OSD_CONFIG (84) | OSD element positions and visibility | Scheduled |
| MSP_RAW_GPS (106) | GPS fix, sats, lat, lon, altitude, speed, heading | Scheduled |
| MSP_COMP_GPS (107) | Distance to home, bearing to home | Scheduled |
| MSP_ATTITUDE (108) | Pitch, roll, yaw | Scheduled |
| MSP_ALTITUDE (109) | Altitude, vertical speed | Scheduled |
| MSP_ANALOG (110) | Battery voltage, mAh, RSSI, current | Scheduled |
| MSP_BATTERY_STATE (130) | Battery cell count, capacity, voltage, current | Scheduled |
| MSP_ESC_SENSOR_DATA (134) | Per-ESC temperature and RPM | Scheduled |
| MSP_RTC (247) | Real-time clock | Scheduled |

**MSP inbound (sensor injection, MSPv2):**

| MSP2 ID | Content |
|---------|---------|
| MSP2_SENSOR_RANGEFINDER (0x1F01) | Rangefinder distance |
| MSP2_SENSOR_OPTIC_FLOW (0x1F02) | Optical flow x/y/quality |
| MSP2_SENSOR_GPS (0x1F03) | GPS fix/position injection |
| MSP2_SENSOR_COMPASS (0x1F04) | Compass heading injection |
| MSP2_SENSOR_BAROMETER (0x1F05) | Baro altitude injection |
| MSP2_SENSOR_AIRSPEED (0x1F06) | Airspeed injection |

**MSP inbound handlers in AP_MSP_Telem_Backend:**
- `msp_handle_opflow()` — MSP2_SENSOR_OPTIC_FLOW
- `msp_handle_rangefinder()` — MSP2_SENSOR_RANGEFINDER
- `msp_handle_gps()` — MSP2_SENSOR_GPS
- `msp_handle_compass()` — MSP2_SENSOR_COMPASS
- `msp_handle_baro()` — MSP2_SENSOR_BAROMETER
- `msp_handle_airspeed()` — MSP2_SENSOR_AIRSPEED

---

## 24. AP_DDS — ROS 2 DDS/XRCE

```
FILE:  libraries/AP_DDS/AP_DDS_Client.cpp / .h
LINES: 1935 / 422
FUNCTION: ROS 2 micro-XRCE-DDS client. Publishes ArduPilot state to ROS 2
          topics and subscribes to ROS 2 commands. Runs over serial (USB/UART)
          or UDP using the XRCE-DDS agent (micro-ros-agent).
MERIDIAN HAS: NO
GAPS: Full implementation.
```

**DDS Published Topics (DataWriter, FC → ROS 2):**

| Topic | ROS 2 Type | Meridian |
|-------|-----------|---------|
| time | rosgraph_msgs/Clock | NO |
| navsat | sensor_msgs/NavSatFix | NO |
| tf_static | tf2_msgs/TFMessage | NO |
| battery | sensor_msgs/BatteryState | NO |
| imu/experimental/data | sensor_msgs/Imu | NO |
| pose/filtered | geometry_msgs/PoseStamped | NO |
| twist/filtered | geometry_msgs/TwistStamped | NO |
| airspeed | geometry_msgs/Vector3Stamped | NO |
| rc | mavros_msgs/RCIn | NO |
| geopose/filtered | geographic_msgs/GeoPoseStamped | NO |
| goal_lla | geographic_msgs/GeoPointStamped | NO |
| clock | rosgraph_msgs/Clock | NO |
| gps_global_origin/filtered | geographic_msgs/GeoPointStamped | NO |
| status | ardupilot_msgs/Status | NO |

**DDS Subscribed Topics (DataReader, ROS 2 → FC):**

| Topic | ROS 2 Type | Function |
|-------|-----------|---------|
| joy | sensor_msgs/Joy | RC channel injection |
| tf | tf2_msgs/TFMessage | External transform (visual odom) |
| cmd_vel | geometry_msgs/TwistStamped | Velocity command |
| cmd_gps_pose | ardupilot_msgs/GlobalPosition | GPS waypoint command |

**DDS Services:**
`AP_DDS_Service_Table.h`: ARM/DISARM service, MODE_SWITCH service.

**Transport**: AP_DDS_Serial (UART XRCE) and AP_DDS_UDP (UDP XRCE).

---

## 25. AP_Networking — TCP/UDP/PPP Stack

```
FILE:  libraries/AP_Networking/AP_Networking.cpp / .h
LINES: 539 / 371
FUNCTION: Network stack for Ethernet-capable boards (ChibiOS with W5500 or
          built-in MAC). Provides TCP/UDP socket API, PPP (serial-to-IP),
          CAN-over-IP bridge, DHCP client.
MERIDIAN HAS: PARTIAL — meridian-platform-sitl has TCP for QGC (port 5760),
              meridian-comms Transport trait is transport-agnostic,
              no hardware networking stack.
```

**AP_Networking sub-components:**

| Component | File | LINES | Function | Meridian |
|-----------|------|-------|---------|---------|
| AP_Networking_ChibiOS | _ChibiOS.cpp | 475 | lwIP integration, W5500 driver, DHCP | NO |
| AP_Networking_PPP | _PPP.cpp | 580 | PPP over serial (UART → IP), used for cellular modems and RFD 900 IP mode | NO |
| AP_Networking_CAN | _CAN.cpp | 258 | CAN-over-IP bridge | NO |
| AP_Networking_port | _port.cpp | 524 | Socket API: UDPClient, UDPServer, TCPClient, TCPServer | PARTIAL (SITL only) |
| AP_Networking_address | _address.cpp | 79 | IP address parsing/formatting | NO |
| AP_Networking_macaddr | _macaddr.cpp | 91 | MAC address management | NO |

**Network configuration parameters:**
NETMASK, IPADDR, GWADDR, MACADDR, ENABLE, DHCP, OPTIONS (telnet debug, etc.).

**Gaps**: No hardware networking (Meridian SITL uses Tokio TCP, no ChibiOS/lwIP). No PPP for RFD900 IP mode. No CAN-over-IP. No DHCP. No telnet debug console.

---

## 26. Gap Summary Table

| Protocol Area | ArduPilot Scope | Meridian Status | Priority |
|---------------|----------------|-----------------|---------|
| MAVLink inbound (50+ message types) | Full GCS_Common handler | PARTIAL (12 handled) | HIGH |
| MAVLink outbound (70+ message types) | Full ap_message enum | PARTIAL (12 sent) | HIGH |
| MAV_CMD dispatch (55+ commands) | Common layer only | PARTIAL (6 commands) | HIGH |
| MAVLink routing (multi-link) | 20-route table, forwarding | NO | MEDIUM |
| MAVLink signing (HMAC-SHA256) | Full signing stack | STUB | MEDIUM |
| MAVFTP (file transfer) | 13 operations, async | STUB | MEDIUM |
| Log download protocol | 5 message types | STUB | LOW |
| MissionItemProtocol fence | Full fence protocol | NO | MEDIUM |
| MissionItemProtocol rally | Full rally protocol | NO | LOW |
| MISSION_ITEM (float, legacy) | Legacy compat | NO | LOW |
| MISSION_SET_CURRENT | Jump to waypoint | NO | MEDIUM |
| Serial port manager (51 protocols) | Runtime protocol assign | NO | HIGH |
| RC auto-detection frontend | Baud rotation, locking | NO | HIGH |
| RC — SBUS FastSBUS / SBUS_NI | 200k baud variant | NO | MEDIUM |
| RC — FPort2 | FPort v2 | NO | MEDIUM |
| RC — ST24 | Yuneec protocol | NO | LOW |
| RC — GHST | ImmersionRC Ghost | NO | LOW |
| RC — SRXL (v1) | Spektrum unidirectional | NO | LOW |
| RC — SRXL2 bidirectional | Telemetry exchange | PARTIAL | MEDIUM |
| RC — DroneCAN RC | CAN-based RC | NO | MEDIUM |
| RC — MAVLinkRadio | MAVLink RC injection | NO | LOW |
| RC — CRSF bidirectional TX | Device ping, VTX cmds | PARTIAL | MEDIUM |
| CRSF telemetry scheduler | 20-slot round-robin | NO | HIGH |
| CRSF LINK_STATISTICS encoder | Signal quality frame | NO | MEDIUM |
| CRSF device protocol (0x3A-0x3D) | Param protocol | NO | LOW |
| GHST telemetry | 5 frame types | NO | LOW |
| Spektrum/SRXL2 telemetry | 40+ sensor types | NO | LOW |
| FrSky D protocol | Legacy one-way | NO | LOW |
| FrSky SPort (sensor polling) | Poll-response sensors | NO | MEDIUM |
| FrSky SPort Passthrough | 13 DIY data IDs | NO | MEDIUM |
| FrSky MAVlite bidirectional | MAVLink-over-FrSky | NO | LOW |
| HoTT telemetry (EAM + GPS) | Graupner sensors | NO | LOW |
| LTM telemetry (G/S/A frames) | FPV OSD protocol | NO | LOW |
| iBus telemetry (19 sensors) | FlySky telemetry | NO | LOW |
| DEVO telemetry | Walkera protocol | NO | LOW |
| CAN manager (multi-driver) | 3-driver bus assign | NO | HIGH |
| SLCAN (CAN over serial ASCII) | Debugging / inject | NO | MEDIUM |
| MAVLinkCAN bridge | MAVLink ↔ CAN | NO | MEDIUM |
| DroneCAN actuator array cmd | PWM output via CAN | NO | HIGH |
| DroneCAN GPS TX | GPS broadcast on bus | NO | MEDIUM |
| DroneCAN param protocol | Node configuration | NO | MEDIUM |
| DroneCAN ESC status extended | Extra ESC telemetry | NO | LOW |
| DroneCAN safety button | Button subscriber | NO | MEDIUM |
| DroneCAN Hobbywing ESC | Vendor extension | NO | LOW |
| KDECAN (KDE Direct ESCs) | Full KDE protocol | NO | LOW |
| PiccoloCAN (4 sub-drivers) | Avionics actuators | NO | LOW |
| MSP OSD (Generic + DJI + DP) | Betaflight OSD compat | NO | MEDIUM |
| MSP sensor injection (6 types) | Sensor input via MSP | NO | LOW |
| DDS/XRCE ROS 2 (14 topics) | ROS 2 integration | NO | LOW |
| AP_Networking ChibiOS/lwIP | Hardware Ethernet | NO | LOW |
| AP_Networking PPP | Modem/RFD900 IP | NO | LOW |
| AP_Networking CAN-over-IP | CAN bridge | NO | LOW |

---

## Notes on Meridian's Current State

**What Meridian has well:**
- MAVLink v2 wire format (framing, CRC, parsing) — solid in v2.rs
- MAVLink server with stream-rate scheduler — server.rs covers the core loop
- Mission protocol (waypoints, upload/download) — server.rs handles it
- Parameter protocol (list, get, set, streaming) — complete in server.rs
- CRSF parser (RC input) — complete with link stats, failsafe, baud negotiation struct
- SBUS parser — complete
- DroneCAN frame encoding/decoding + DNA server + basic node management
- DroneCAN sensor adapters (GPS, Compass, Baro, Battery)

**Biggest gaps for real-hardware flight:**
1. Serial Manager — no runtime port assignment means can't dynamically configure protocols
2. RC auto-detection — can't run the baud-rotation scanner that ArduPilot uses
3. CAN manager — can't assign KDECAN/PiccoloCAN per-bus
4. MAVLink message coverage — missing ~58 of 70 outbound messages (QGC HUD works, full map/mission editor won't)
5. MAVLink command coverage — missing ~49 of 55 MAV_CMDs (arming works, calibration won't)
6. CRSF telemetry scheduler — can encode frames but doesn't schedule them
7. MSP OSD — zero implementation, needed for Betaflight-compatible OSDs
8. FrSky SPort Passthrough — widely used with OpenTX/EdgeTX radios
