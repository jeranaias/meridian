# ArduPilot Wave 2 Audit: GCS MAVLink + Serial Manager + MSP + DDS

**Source**: `D:\projects\ardupilot\libraries\`  
**Date**: 2026-04-02  
**Auditor**: Automated audit  
**Scope**: GCS_MAVLink (~15K lines), AP_SerialManager (~1K lines), AP_MSP (~4K lines), AP_DDS (~4K lines)

---

## 1. GCS_MAVLink

### 1.1 Architecture: GCS Singleton and GCS_MAVLINK Per-Link

There are two distinct class layers:

**`GCS`** — singleton, one per vehicle. Owns the array of per-link backends, the statustext queue, the MissionItemProtocol objects, and global parameters like `MAV_SYSID`, `MAV_GCS_SYSID`, and `MAV_OPTIONS`. Accessed globally via `gcs()`. Exposes `update_send()` / `update_receive()` which delegate to each backend.

**`GCS_MAVLINK`** — one instance per active MAVLink link (serial port or network connection). Owns a `mavlink_channel_t chan`, the outbound message bucket system, stream rates, signing state, and the parameter send queue. Vehicle subclasses (ArduCopter, ArduPlane, etc.) each create a subclass of `GCS_MAVLINK`.

**Simultaneous connection count**: Controlled by `MAVLINK_COMM_NUM_BUFFERS`:
- SITL: 16 simultaneous links
- Hardware with >1MB flash: 8 links
- Default hardware: 5 links

The `_chan[MAVLINK_COMM_NUM_BUFFERS]` array in `GCS` holds all backends.

**Message routing** (`MAVLink_routing`):
- A single static `MAVLink_routing routing` object is shared across all `GCS_MAVLINK` instances.
- It maintains a table of up to 20 routes. Each route records `{sysid, compid, channel, mavtype}`.
- On every received message, `routing.check_and_forward()` is called. It:
  1. Learns the sender's route (calls `learn_route()`).
  2. Extracts the message's `target_system` and `target_component`.
  3. If targeted to this vehicle: returns `true` so the message is processed locally.
  4. If targeted elsewhere: forwards to the appropriate channel and returns `false`.
- Broadcast packets (target sysid=0 or 0xFF) go to all active non-private channels.
- Private channels (set via `set_channel_private()`) do not receive forwarded packets or broadcasts — only heartbeats.
- A `no_route_mask` bitmask allows routing to be disabled per channel (e.g. to prevent feedback loops to CAN-bridged components).

**Channel-level bitmasks** (static on `GCS_MAVLINK`):
- `mavlink_active`: channels that have had traffic
- `chan_is_streaming`: channels currently sending stream-rate messages
- `mavlink_private`: private channels

### 1.2 Message Handlers (handle_* Methods)

All handlers are declared in `GCS.h` and implemented in `GCS_Common.cpp`. The main dispatch loop is `GCS_MAVLINK::handle_message()`.

#### Command Handling
| Handler | MAVLink Message |
|---------|----------------|
| `handle_command_int()` | `COMMAND_INT` |
| `handle_command_long()` | `COMMAND_LONG` (converts to COMMAND_INT internally via `try_command_long_as_command_int`) |
| `handle_command_ack()` | `COMMAND_ACK` |
| `handle_set_mode()` | `SET_MODE` |
| `handle_command_int_packet()` | dispatches all `MAV_CMD_*` within COMMAND_INT |

Specific MAV_CMD handlers (all return `MAV_RESULT`):
- `handle_command_do_set_home()` — MAV_CMD_DO_SET_HOME
- `handle_command_component_arm_disarm()` — MAV_CMD_COMPONENT_ARM_DISARM
- `handle_command_do_aux_function()` — MAV_CMD_DO_AUX_FUNCTION
- `handle_command_storage_format()` — MAV_CMD_STORAGE_FORMAT (triggers SD format, async via `GCS_MAVLINK_InProgress`)
- `handle_command_set_message_interval()` — MAV_CMD_SET_MESSAGE_INTERVAL
- `handle_command_get_message_interval()` — MAV_CMD_GET_MESSAGE_INTERVAL
- `handle_command_request_message()` — MAV_CMD_REQUEST_MESSAGE
- `handle_START_RX_PAIR()` — MAV_CMD_START_RX_PAIR
- `handle_flight_termination()` — MAV_CMD_DO_FLIGHTTERMINATION
- `handle_send_autopilot_version()` / `handle_command_request_autopilot_capabilities()` — MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
- `handle_preflight_reboot()` — MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
- `handle_command_preflight_calibration()` — MAV_CMD_PREFLIGHT_CALIBRATION (gyro, accel, baro, airspeed)
- `_handle_command_preflight_calibration_baro()` — baro-specific sub-handler
- `handle_command_do_set_mission_current()` — MAV_CMD_DO_SET_MISSION_CURRENT
- `handle_command_do_jump_tag()` — MAV_CMD_DO_JUMP_TAG
- `handle_command_battery_reset()` — MAV_CMD_BATTERY_RESET
- `handle_command_accelcal_vehicle_pos()` — MAV_CMD_ACCELCAL_VEHICLE_POS
- `handle_command_mount()` — MAV_CMD_DO_MOUNT_CONTROL and related mount commands
- `handle_command_mag_cal()` — MAV_CMD_FIXED_MAG_CAL, MAV_CMD_DO_START_MAG_CAL, MAV_CMD_DO_ACCEPT_MAG_CAL
- `handle_command_fixed_mag_cal_yaw()` — MAV_CMD_FIXED_MAG_CAL_YAW
- `handle_command_camera()` — MAV_CMD_DO_DIGICAM_CONTROL, MAV_CMD_IMAGE_START_CAPTURE, etc.
- `handle_command_do_set_roi()` — MAV_CMD_DO_SET_ROI
- `handle_command_do_gripper()` — MAV_CMD_DO_GRIPPER
- `handle_command_do_sprayer()` — MAV_CMD_DO_SPRAYER
- `handle_command_do_set_mode()` — MAV_CMD_DO_SET_MODE
- `handle_command_get_home_position()` — MAV_CMD_GET_HOME_POSITION
- `handle_command_do_fence_enable()` — MAV_CMD_DO_FENCE_ENABLE
- `handle_command_debug_trap()` — MAV_CMD_DEBUG_TRAP
- `handle_command_set_ekf_source_set()` — MAV_CMD_SET_EKF_SOURCE_SET
- `handle_command_airframe_configuration()` — MAV_CMD_AIRFRAME_CONFIGURATION
- `handle_can_forward()` — MAV_CMD_CAN_FORWARD
- `handle_do_set_safety_switch_state()` — MAV_CMD_DO_SET_SAFETY_SWITCH_STATE
- `handle_command_run_prearm_checks()` — MAV_CMD_RUN_PREARM_CHECKS
- `handle_command_flash_bootloader()` — MAV_CMD_FLASH_BOOTLOADER
- `handle_control_high_latency()` — MAV_CMD_CONTROL_HIGH_LATENCY
- `handle_command_int_external_position_estimate()` — MAV_CMD_EXTERNAL_POSITION_ESTIMATE
- `handle_command_int_external_wind_estimate()` — MAV_CMD_EXTERNAL_WIND_ESTIMATE
- `handle_command_do_follow()` — MAV_CMD_DO_FOLLOW
- `handle_command_do_set_global_origin()` — MAV_CMD_DO_SET_GLOBAL_ORIGIN
- `handle_servorelay_message()` — MAV_CMD_DO_SET_SERVO, MAV_CMD_DO_SET_RELAY, MAV_CMD_DO_REPEAT_SERVO, MAV_CMD_DO_REPEAT_RELAY

#### Parameter Protocol
| Handler | MAVLink Message |
|---------|----------------|
| `handle_param_request_list()` | `PARAM_REQUEST_LIST` |
| `handle_param_request_read()` | `PARAM_REQUEST_READ` |
| `handle_param_set()` | `PARAM_SET` |
| `handle_common_param_message()` | dispatches all three above |
| `handle_param_value()` | `PARAM_VALUE` (inbound, from a component) |

#### Mission Protocol
| Handler | MAVLink Message |
|---------|----------------|
| `handle_mission_request_list()` | `MISSION_REQUEST_LIST` |
| `handle_mission_request_int()` | `MISSION_REQUEST_INT` |
| `handle_mission_request()` | `MISSION_REQUEST` (deprecated, warns) |
| `handle_mission_count()` | `MISSION_COUNT` |
| `handle_mission_item()` | `MISSION_ITEM` or `MISSION_ITEM_INT` |
| `handle_mission_clear_all()` | `MISSION_CLEAR_ALL` |
| `handle_mission_write_partial_list()` | `MISSION_WRITE_PARTIAL_LIST` |
| `handle_mission_set_current()` | `MISSION_SET_CURRENT` |
| `handle_common_mission_message()` | dispatches `MISSION_ACK` too |

#### Fence Protocol
| Handler | MAVLink Message |
|---------|----------------|
| `handle_fence_message()` | `FENCE_POINT`, `FENCE_FETCH_POINT` (legacy), and mission-protocol fence items |

#### Rally Protocol
| Handler | MAVLink Message |
|---------|----------------|
| `handle_common_rally_message()` | dispatches rally point messages |
| `handle_rally_fetch_point()` | `RALLY_FETCH_POINT` (legacy) |
| `handle_rally_point()` | `RALLY_POINT` (legacy) |

#### Log Download
| Handler | MAVLink Message |
|---------|----------------|
| `AP::logger().handle_mavlink_msg()` | `LOG_REQUEST_LIST`, `LOG_REQUEST_DATA`, `LOG_ERASE`, `LOG_REQUEST_END`, `REMOTE_LOG_BLOCK_STATUS` |

Log download is entirely delegated to `AP_Logger`. ArduPilot does NOT directly arm while logging (AP_Logger blocks log sends when armed).

#### MAVFTP
| Handler | MAVLink Message |
|---------|----------------|
| `GCS_FTP::handle_file_transfer_protocol()` | `FILE_TRANSFER_PROTOCOL` |

#### Calibration
| Handler | MAVLink Message / Command |
|---------|--------------------------|
| `handle_command_preflight_calibration()` | MAV_CMD_PREFLIGHT_CALIBRATION (gyro/accel/baro/airspeed/compass) |
| `_handle_command_preflight_calibration_baro()` | baro sub-calibration |
| `handle_command_mag_cal()` | compass calibration start/accept/cancel |
| `handle_command_accelcal_vehicle_pos()` | accelerometer vehicle position during cal |
| `send_accelcal_vehicle_position()` | sends position request back to GCS |

#### Sensor Input (Inbound External Data)
| Handler | MAVLink Message |
|---------|----------------|
| `handle_distance_sensor()` | `DISTANCE_SENSOR` |
| `handle_obstacle_distance()` | `OBSTACLE_DISTANCE` |
| `handle_obstacle_distance_3d()` | `OBSTACLE_DISTANCE_3D` |
| `handle_optical_flow()` | `OPTICAL_FLOW` |
| `handle_vision_position_estimate()` | `VISION_POSITION_ESTIMATE` |
| `handle_global_vision_position_estimate()` | `GLOBAL_VISION_POSITION_ESTIMATE` |
| `handle_vicon_position_estimate()` | `VICON_POSITION_ESTIMATE` |
| `handle_vision_position_delta()` | `VISION_POSITION_DELTA` |
| `handle_vision_speed_estimate()` | `VISION_SPEED_ESTIMATE` |
| `handle_att_pos_mocap()` | `ATT_POS_MOCAP` |
| `handle_odometry()` | `ODOMETRY` |
| `handle_landing_target()` | `LANDING_TARGET` |
| `handle_adsb_message()` | `ADSB_VEHICLE`, `UAVIONIX_ADSB_*` |
| `handle_generator_message()` | `GENERATOR_STATUS` |
| `handle_data_packet()` | `DATA96` (AP_Radio firmware upload type=42/43) |

#### Control Input
| Handler | MAVLink Message |
|---------|----------------|
| `handle_rc_channels_override()` | `RC_CHANNELS_OVERRIDE` |
| `handle_manual_control()` | `MANUAL_CONTROL` |
| `handle_radio_rc_channels()` | `RADIO_RC_CHANNELS` |

#### Miscellaneous
| Handler | MAVLink Message |
|---------|----------------|
| `handle_heartbeat()` | `HEARTBEAT` |
| `handle_timesync()` | `TIMESYNC` |
| `handle_statustext()` | `STATUSTEXT` (inbound, chunked, logged) |
| `handle_named_value()` | `NAMED_VALUE_FLOAT` / `NAMED_VALUE_INT` |
| `handle_radio_status()` | `RADIO`, `RADIO_STATUS` |
| `handle_serial_control()` | `SERIAL_CONTROL` (port passthrough) |
| `handle_system_time_message()` | `SYSTEM_TIME` |
| `handle_setup_signing()` | `SETUP_SIGNING` |
| `handle_send_autopilot_version()` | `AUTOPILOT_VERSION_REQUEST` |
| `handle_set_gps_global_origin()` | `SET_GPS_GLOBAL_ORIGIN` |
| `handle_osd_param_config()` | `OSD_PARAM_CONFIG`, `OSD_PARAM_SHOW_CONFIG` |
| `handle_device_op_read()` | `DEVICE_OP_READ` |
| `handle_device_op_write()` | `DEVICE_OP_WRITE` |
| `handle_mount_message()` | `GIMBAL_REPORT`, `GIMBAL_DEVICE_INFORMATION`, `GIMBAL_DEVICE_ATTITUDE_STATUS`, `GIMBAL_MANAGER_SET_ATTITUDE`, `GIMBAL_MANAGER_SET_PITCHYAW` |
| `handle_can_frame()` | `CAN_FRAME` |
| `AP::gps().handle_msg()` | `GPS_RTCM_DATA`, `GPS_INPUT`, `GPS_INJECT_DATA` |
| `AP::terrain()->handle_message()` | `TERRAIN_DATA`, `TERRAIN_CHECK` |
| `camera->handle_message()` | `DIGICAM_CONTROL`, `GOPRO_HEARTBEAT`, `CAMERA_INFORMATION` |
| `AP_Notify::handle_led_control()` | `LED_CONTROL` |
| `AP_Notify::handle_play_tune()` | `PLAY_TUNE` |

### 1.3 Stream Rates — SRx_* Parameter System

The parameter naming scheme is `MAVx_<STREAM>` where x is the channel number (1–8). Each `GCS_MAVLINK` instance has an `AP_Int16 streamRates[NUM_STREAMS]` array. The parameter group name for each link's params is "1", "2", ... "8" (e.g., `MAV1_RAW_SENS` for link 1).

**Stream IDs and their parameter names:**

| Enum | Parameter Suffix | Default (Plane/Rover) | Default (Copter) | Messages |
|------|-----------------|----------------------|------------------|---------|
| `STREAM_RAW_SENSORS` | `_RAW_SENS` | 1 Hz | 0 Hz | RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3, AIRSPEED |
| `STREAM_EXTENDED_STATUS` | `_EXT_STAT` | 1 Hz | 0 Hz | SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK, GPS2_RAW, GPS2_RTK, NAV_CONTROLLER_OUTPUT, FENCE_STATUS, POSITION_TARGET_GLOBAL_INT |
| `STREAM_RC_CHANNELS` | `_RC_CHAN` | 1 Hz | 0 Hz | SERVO_OUTPUT_RAW, RC_CHANNELS, RC_CHANNELS_RAW (MAVLink1 only) |
| `STREAM_RAW_CONTROLLER` | `_RAW_CTRL` | 1 Hz | 0 Hz | (empty in base — vehicle subclasses may add) |
| `STREAM_POSITION` | `_POSITION` | 1 Hz (Rover 1, Sub 3) | 0 Hz | GLOBAL_POSITION_INT, LOCAL_POSITION_NED |
| `STREAM_EXTRA1` | `_EXTRA1` | 1 Hz (Sub 10) | 0 Hz | ATTITUDE, SIMSTATE, AHRS2, RPM, AOA_SSA (Plane), PID_TUNING, LANDING (Plane), ESC_TELEMETRY, WHEEL_DISTANCE (Rover), GENERATOR_STATUS, WINCH_STATUS, EFI_STATUS, HYGROMETER |
| `STREAM_EXTRA2` | `_EXTRA2` | 1 Hz (Sub 10) | 0 Hz | VFR_HUD |
| `STREAM_EXTRA3` | `_EXTRA3` | 1 Hz (Sub 3) | 0 Hz | AHRS, WIND, WATER_DEPTH (Rover), DISTANCE_SENSOR, SYSTEM_TIME, TERRAIN_REPORT, TERRAIN_REQUEST, BATTERY_STATUS, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION |
| `STREAM_PARAMS` | `_PARAMS` | 10 Hz | 0 Hz | NEXT_PARAM (queued param stream), AVAILABLE_MODES |
| `STREAM_ADSB` | `_ADSB` | 0 Hz (Plane 5) | 0 Hz | ADSB_VEHICLE, AIS_VESSEL |

**How REQUEST_DATA_STREAM maps to streams:**

`MAV_DATA_STREAM_ALL` → sets all streams (excluding PARAMS).  
`MAV_DATA_STREAM_RAW_SENSORS` → STREAM_RAW_SENSORS.  
`MAV_DATA_STREAM_EXTENDED_STATUS` → STREAM_EXTENDED_STATUS.  
`MAV_DATA_STREAM_RC_CHANNELS` → STREAM_RC_CHANNELS.  
`MAV_DATA_STREAM_RAW_CONTROLLER` → STREAM_RAW_CONTROLLER.  
`MAV_DATA_STREAM_POSITION` → STREAM_POSITION.  
`MAV_DATA_STREAM_EXTRA1` → STREAM_EXTRA1.  
`MAV_DATA_STREAM_EXTRA2` → STREAM_EXTRA2.  
`MAV_DATA_STREAM_EXTRA3` → STREAM_EXTRA3.  

If `Option::NOSTREAMOVERRIDE` is set on a channel, REQUEST_DATA_STREAM messages are silently ignored.

**Deferred message bucket system:**  
Messages are not sent immediately. They are placed into one of 10 `deferred_message_bucket_t` buckets, each with a rate interval. On each `update_send()` call, the scheduler finds the bucket whose `last_sent_ms + interval_ms <= now` and sends one message from it per call. Special messages (HEARTBEAT, NEXT_PARAM, HIGH_LATENCY2) have their own 3-slot deferred_message array and bypass the bucket slowdown.

**`stream_slowdown_ms`**: When the transmit buffer is backed up (reported via RADIO_STATUS `txbuf` field dropping below 33%), the system adds extra delay to bucket scheduling. Max slowdown accumulated via the `stream_slowdown_ms` field.

### 1.4 Mission Protocol — Full Upload/Download State Machine

Implemented in `MissionItemProtocol` (base class) with subclasses:
- `MissionItemProtocol_Waypoints` — MAV_MISSION_TYPE_MISSION
- `MissionItemProtocol_Fence` — MAV_MISSION_TYPE_FENCE  
- `MissionItemProtocol_Rally` — MAV_MISSION_TYPE_RALLY

Three instances held in `GCS::missionitemprotocols[3]` (indexed by MAV_MISSION_TYPE value).

**Download flow (GCS reads from vehicle):**

```
GCS sends MISSION_REQUEST_LIST (type=?)
  → vehicle replies MISSION_COUNT (count, type)
GCS sends MISSION_REQUEST_INT (seq=0, type=?)
  → vehicle replies MISSION_ITEM_INT (seq=0, data...)
GCS sends MISSION_REQUEST_INT (seq=1)
  → vehicle replies MISSION_ITEM_INT (seq=1, ...)
  ... repeat for all items ...
GCS sends MISSION_ACK (MAV_MISSION_ACCEPTED)
```

If `receiving` is true (upload in progress), download requests are rejected with `MAV_MISSION_DENIED`.

**Upload flow (GCS sends to vehicle):**

```
GCS sends MISSION_COUNT (count, type)
  → vehicle allocates resources, calls truncate()
  → vehicle sends MISSION_REQUEST_INT (seq=0) [or MISSION_REQUEST for legacy]
GCS sends MISSION_ITEM_INT (seq=0, frame, command, params...)
  → vehicle calls replace_item() or append_item()
  → vehicle sends MISSION_REQUEST_INT (seq=1)
GCS sends MISSION_ITEM_INT (seq=1, ...)
  ... repeat ...
Last item received:
  → vehicle calls complete() (e.g. rebuilds mission index)
  → vehicle sends MISSION_ACK (MAV_MISSION_ACCEPTED)
```

**Timeout / retry logic:**
- `upload_timeout_ms = 8000` ms. If no item is received within this window, the upload is cancelled with `MISSION_ACK (MAV_MISSION_OPERATION_CANCELLED)` and resources are freed.
- Re-request retry: If the vehicle sent `MISSION_REQUEST_INT` and has not received the item within `1000ms + stream_slowdown_ms`, it resends the request.
- `timelast_receive_ms` is updated on each valid item received.
- `timelast_request_ms` is updated on each request sent.

**Locks and exclusion:**
- Only one upload can be in progress at a time per protocol type.
- A different system cannot start an upload if one is in progress (gets `MAV_MISSION_DENIED`). The same system can restart by resending `MISSION_COUNT` (previous resources freed first).
- Downloads are blocked while an upload is in progress.

**MAVLink 2 requirement:** For fence and rally protocols (non-MISSION types), MAVLink 1 connections receive `MAV_MISSION_UNSUPPORTED` and a warning text.

**Partial list update:** `MISSION_WRITE_PARTIAL_LIST` with `start_index`/`end_index` allows updating a subset of items (calls `allocate_update_resources()` and `init_send_requests()` for that range).

### 1.5 Parameter Protocol

**PARAM_REQUEST_LIST streaming:**
- On receipt, sets `_queued_parameter` to the first AP_Param, `_queued_parameter_count` to total count, `_queued_parameter_index` to 0.
- Sending is driven by `queued_param_send()` called each scheduler cycle.
- Rate limiting: uses at most 30% of link bandwidth per call. Computes `bytes_allowed = link_bw * elapsed_ms / 3333` then divides by size of one PARAM_VALUE message to get a count.
- Without flow control: caps at 5 params per call.
- Stops sending if `last_txbuf_is_greater(33)` returns false (radio buffer < 33%).
- Max 1ms of CPU per call: exits if `AP_HAL::micros() - tstart > 1000`.
- Progress through params via `AP_Param::next_scalar()`.

**PARAM_REQUEST_READ:**
- Can specify by index or by name.
- Handled via the async IO thread: pushed into `param_requests` queue (capacity 20), processed by `param_io_timer()`, result placed in `param_replies` queue (capacity 5), sent by `send_parameter_async_replies()`.

**PARAM_SET with verification:**
- Decodes the message, finds the parameter, validates type, sets the value.
- If the parameter name or type doesn't match, returns a `MAV_PARAM_ERROR` via `send_param_error()`.
- After setting, immediately sends back a `PARAM_VALUE` with the new value (verification is implicit — GCS compares what it sent to what it received back).
- Saving to EEPROM happens inside `AP_Param::set_and_save()`.

**`reserve_param_space_start_ms`**: When a param read fails due to lack of tx space, this timestamp is set. For 2 seconds after, `packet_overhead_chan()` reserves 100 extra bytes, reducing the effective buffer for other messages and giving parameter reads priority.

### 1.6 Log Download

Entirely delegated to `AP_Logger::handle_mavlink_msg()`. ArduPilot handles:
- `LOG_REQUEST_LIST` — returns LOG_ENTRY for each log on the SD card
- `LOG_REQUEST_DATA` — streams LOG_DATA blocks with 90-byte payloads
- `LOG_ERASE` — erases all logs
- `LOG_REQUEST_END` — terminates a log download session
- `REMOTE_LOG_BLOCK_STATUS` — acknowledges blocks in remote logging mode

Log data is NOT sent while armed (AP_Logger blocks the send). The GCS_MAVLINK code calls `AP::logger().handle_log_send()` from `update_send()` when not in a delay callback.

### 1.7 MAVFTP

`GCS_FTP` class, activated by `FILE_TRANSFER_PROTOCOL` messages. Uses a 2560-byte IO thread and up to 5 simultaneous sessions (configurable via `AP_MAVLINK_FTP_MAX_SESSIONS`).

**Opcodes handled:**

| Opcode | Description |
|--------|-------------|
| `TerminateSession` | Close file handle |
| `ResetSessions` | Close all sessions |
| `ListDirectory` | Directory listing |
| `OpenFileRO` | Open file read-only |
| `ReadFile` | Read bytes from file |
| `CreateFile` | Create and open file for write |
| `WriteFile` | Write bytes to file |
| `RemoveFile` | Delete a file |
| `CreateDirectory` | Create directory |
| `RemoveDirectory` | Remove directory |
| `OpenFileWO` | Open existing file for write |
| `TruncateFile` | Truncate file at offset |
| `Rename` | Rename file or directory |
| `CalcFileCRC32` | Compute CRC32 of file |
| `BurstReadFile` | Burst read (sends multiple blocks) |

**Access scope:** MAVFTP has access to whatever AP_Filesystem exposes — primarily the SD card, and ROMFS (parameter files). This is how Mission Planner uploads parameter files and log downloads work beyond the basic log protocol.

**Session timeout:** Sessions that are inactive for 3000ms get killed if a new session needs the slot. Sessions inactive for 20000ms are killed unconditionally.

**Reply flow:** Replies go back via `FILE_TRANSFER_PROTOCOL` on the same channel. Rate-limited: won't send if `last_txbuf_is_greater(33)` is false.

**Param file upload security note:** MAVFTP writes to the filesystem, and AP reads `@ROMFS/defaults.parm` and SD card param files on boot. So MAVFTP provides a route to set parameters that bypasses PARAM_SET. `GCS::get_allow_param_set()` / `set_allow_param_set()` can be used to gate this.

### 1.8 Signing — MAVLink 2 Message Signing

Implemented in `GCS_Signing.cpp`.

**Key structure** stored in FRAM (`StorageManager::StorageKeys`):
```c
struct SigningKey {
    uint32_t magic;       // 0x3852fcd1
    uint64_t timestamp;   // in 10-microsecond units since 2015-01-01
    uint8_t secret_key[32]; // HMAC-SHA256 secret
};
```

**Key exchange:** The GCS sends a `SETUP_SIGNING` message containing `initial_timestamp` and `secret_key[32]`. The vehicle validates that it is not currently armed, saves the key to FRAM, and immediately activates signing on all links by calling `load_signing_key()` for each backend.

**Activation per link:** On `load_signing_key()`:
- Copies key bytes to `mavlink_signing_t signing`.
- Sets `signing.link_id = (uint8_t)chan`.
- Sets `signing.timestamp = saved_timestamp + 60_seconds_in_10us_units` (adds 60 seconds of buffer to prevent replay after reboot/save lag).
- Sets `signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING`.
- Sets `accept_unsigned_callback` (see below).
- Attaches `signing` and `signing_streams` to the channel's `mavlink_status_t`.

**Unsigned message acceptance:** The `accept_unsigned_callback` function:
- Always accepts on `MAVLINK_COMM_0` (assumed to be USB, trusted channel).
- Accepts `MAVLINK_MSG_ID_RADIO_STATUS` and `MAVLINK_MSG_ID_RADIO` unsigned on any channel (these radio modems send unsigned by design).
- Rejects everything else on non-channel-0 links.

**Timestamp management:**
- When GPS lock is acquired, `GCS_MAVLINK::update_signing_timestamp(timestamp_usec)` converts the GPS epoch to the signing epoch (offset from Unix 1970 to 2015-01-01 = 1420070400 seconds), multiplies to 10-microsecond units, and updates any link whose signing timestamp is behind.
- The timestamp is saved to FRAM every 30 seconds (or immediately on GPS update) to prevent replay attacks after reboot.
- On load, the saved timestamp is advanced by 60 seconds to cover any drift between the last save and the current boot.

**Option disable:** Setting `Option::MAVLINK2_SIGNING_DISABLED` on a link skips `load_signing_key()` for that link — signing is not used.

**Shared streams object:** `signing_streams` is a single static `mavlink_signing_streams_t` shared across all links to deduplicate seen timestamps/link-IDs and prevent cross-link replay.

**Packet overhead with signing:** `packet_overhead_chan()` returns `MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN` (=13 extra bytes) when signing is active, versus `MAVLINK_NUM_NON_PAYLOAD_BYTES` without signing.

### 1.9 Statustext Queuing

**Queue:** `GCS::StatusTextQueue _statustext_queue` — an `ObjectArray<statustext_t>` with capacity:
- 10 entries on small RAM boards (`HAL_MEM_CLASS <= HAL_MEM_CLASS_192`)
- 30 entries otherwise

**Each entry** (`statustext_t`):
```c
struct statustext_t {
    mavlink_statustext_t msg;       // includes severity + 50-char text + chunk_seq + id
    uint16_t entry_created_ms;      // 16-bit timestamp for pruning
    mavlink_channel_mask_t bitmask; // which channels still need to send this
};
```

**Push (send_text / send_textv):**
- Called from any thread; protected by `_statustext_queue.semaphore()`.
- Long messages (>50 chars) are split into chunks: each chunk gets the same `id` (auto-incrementing per-message ID), incremented `chunk_seq` field, and a `bitmask` of all active channels.
- If the queue is full, `push_force()` overwrites the oldest entry, keeping communications flowing at the cost of dropping old messages.

**Service (drain queue to wire):**
- `GCS::service_statustext()` is called from `GCS::update_send()` every scheduler cycle.
- Iterates all backends starting from `first_backend_to_send` (round-robin fairness).
- Each `GCS_MAVLINK::service_statustext()`:
  - Iterates the queue looking for entries with this channel's bit set in `bitmask`.
  - Checks `txspace()` before sending each entry.
  - Sends `STATUSTEXT` via `mavlink_msg_statustext_send()`.
  - Clears this channel's bit from the entry's bitmask.
  - If bitmask reaches 0, removes the entry from the queue.
  - Stops if `txspace()` is insufficient (backpressure).
- `StatusTextQueue::prune()` removes entries older than 60 seconds to prevent stale messages from blocking the queue forever.

**Inbound STATUSTEXT handling:**
- `handle_statustext()` receives STATUSTEXT messages from other components.
- Tracks chunking state per `(src_system, src_component, id)` tuple.
- Reassembles chunks and writes to AP_Logger as `Write_MessageChunk()`.

### 1.10 Heartbeat Handling and GCS Timeout

**Outbound heartbeat:** Sent via the `MSG_HEARTBEAT` deferred message (special slot, not in a bucket). Sent roughly every 1 second. Contains: vehicle type, autopilot type, base_mode (armed/stabilize bits), custom_mode (vehicle-specific flight mode number), system status, MAVLink version.

**Inbound heartbeat handling** (`handle_heartbeat()`):
- Checks if `msg.sysid` is a GCS system ID (via `gcs().sysid_is_gcs()`).
  - `sysid_is_gcs()` compares against `MAV_GCS_SYSID` and `MAV_GCS_SYSID_HI` range (or allows sysid 255 as default GCS).
- If it is a GCS heartbeat, calls `sysid_mygcs_seen(AP_HAL::millis())` to update the last-seen timestamp.
- Note: `MANUAL_CONTROL` and `RC_CHANNELS_OVERRIDE` also call `sysid_mygcs_seen()` — any GCS-control traffic resets the timeout.

**GCS timeout / failsafe trigger:**
- The `_sysid_gcs_last_seen_time_ms` timestamp is updated whenever a GCS heartbeat or manual control message is received.
- `GCS::sysid_mygcs_last_seen_time_ms()` exposes this to failsafe code.
- Vehicle-specific failsafe code (e.g., ArduCopter's `GCS_Failsafe`) polls this timestamp and triggers the GCS failsafe when `millis() - last_seen > gcs_failsafe_timeout_ms`.
- The timeout duration is set by the vehicle's failsafe parameter (typically `FS_GCS_ENABLE` + timeout configured separately).
- The GCS library itself does not trigger the failsafe — it only maintains the timestamp. The vehicle flight code triggers the action.

---

## 2. AP_SerialManager

### 2.1 Port Mapping

Up to `SERIALMANAGER_MAX_PORTS` (10 on typical hardware, minimum 4) ports are managed. Each has a `UARTState` with three `AP_Param` values:
- `SERIALn_BAUD` (`AP_Int32`) — baud rate (special: value 57 maps to 57600, etc. via `map_baudrate()`)
- `SERIALn_PROTOCOL` (`AP_Int8`) — selects the protocol
- `SERIALn_OPTIONS` (`AP_Int32`) — protocol-specific option bitmask

Ports are indexed 0–9. SERIAL0 is the console/USB port. The protocol assignment is static (requires reboot).

Finding a port for a protocol: `find_serial(protocol, instance)` searches `state[]` for the nth occurrence of the matching protocol and returns the `UARTDriver*`. `protocol_match()` handles equivalences (e.g. `SerialProtocol_MAVLink2` is treated as `SerialProtocol_MAVLink` for lookup purposes — it was deprecated).

**Extended ports**: `AP_SERIALMANAGER_NET_PORT_1=21` (networking), `AP_SERIALMANAGER_CAN_D1_PORT_1=41`/`D2=51` (DroneCAN serial tunnel), `AP_SERIALMANAGER_SCR_PORT_1=61` (scripting serial devices). These are registered dynamically via `register_port()`.

### 2.2 Complete Protocol ID List

| ID | Name | Description |
|----|------|-------------|
| -1 | `SerialProtocol_None` | Not used |
| 0 | `SerialProtocol_Console` | Unused (was console) |
| 1 | `SerialProtocol_MAVLink` | MAVLink 1 or 2 |
| 2 | `SerialProtocol_MAVLink2` | Deprecated (use MAVLink + instance) |
| 3 | `SerialProtocol_FrSky_D` | FrSky D protocol (D-series receivers) |
| 4 | `SerialProtocol_FrSky_SPort` | FrSky SPort (X-series receivers) |
| 5 | `SerialProtocol_GPS` | GPS (NMEA, UBlox, etc.) |
| 6 | `SerialProtocol_GPS2` | Deprecated (use GPS + instance=1) |
| 7 | `SerialProtocol_AlexMos` | AlexMos gimbal |
| 8 | `SerialProtocol_Gimbal` | SToRM32 / Siyi gimbal |
| 9 | `SerialProtocol_Rangefinder` | Rangefinder |
| 10 | `SerialProtocol_FrSky_SPort_Passthrough` | FrSky SPort Passthrough (OpenTX) |
| 11 | `SerialProtocol_Lidar360` | Lightware SF40C / TeraRanger Tower / RPLidar A2 |
| 12 | `SerialProtocol_Aerotenna_USD1` | Deprecated — use Rangefinder |
| 13 | `SerialProtocol_Beacon` | Indoor positioning beacons |
| 14 | `SerialProtocol_Volz` | Volz servo protocol |
| 15 | `SerialProtocol_Sbus1` | SBUS output |
| 16 | `SerialProtocol_ESCTelemetry` | ESC telemetry (BLHeli, etc.) |
| 17 | `SerialProtocol_Devo_Telem` | DEVO telemetry |
| 18 | `SerialProtocol_OpticalFlow` | Optical flow sensor |
| 19 | `SerialProtocol_Robotis` | Robotis servo bus |
| 20 | `SerialProtocol_NMEAOutput` | NMEA output stream |
| 21 | `SerialProtocol_WindVane` | Wind vane sensor |
| 22 | `SerialProtocol_SLCAN` | CAN-over-serial (SLCAN) |
| 23 | `SerialProtocol_RCIN` | RC input (PPM/SBUS from serial) |
| 24 | `SerialProtocol_EFI` | Electronic fuel injection |
| 25 | `SerialProtocol_LTM_Telem` | LTM telemetry |
| 26 | `SerialProtocol_RunCam` | RunCam camera control |
| 27 | `SerialProtocol_Hott` | HoTT telemetry |
| 28 | `SerialProtocol_Scripting` | Lua scripting serial port |
| 29 | `SerialProtocol_CRSF` | Crossfire RC/telemetry |
| 30 | `SerialProtocol_Generator` | Generator control |
| 31 | `SerialProtocol_Winch` | Winch controller |
| 32 | `SerialProtocol_MSP` | MSP (Betaflight/iNav OSD) |
| 33 | `SerialProtocol_DJI_FPV` | DJI FPV goggles (MSP variant) |
| 34 | `SerialProtocol_AirSpeed` | Airspeed sensor |
| 35 | `SerialProtocol_ADSB` | ADS-B receiver |
| 36 | `SerialProtocol_AHRS` | External AHRS |
| 37 | `SerialProtocol_SmartAudio` | VTX SmartAudio |
| 38 | `SerialProtocol_FETtecOneWire` | FETtec ESC protocol |
| 39 | `SerialProtocol_Torqeedo` | Torqeedo electric motor |
| 40 | `SerialProtocol_AIS` | Automatic Identification System (maritime) |
| 41 | `SerialProtocol_CoDevESC` | CoDev ESC protocol |
| 42 | `SerialProtocol_MSP_DisplayPort` | MSP DisplayPort (HD OSD) |
| 43 | `SerialProtocol_MAVLinkHL` | MAVLink High Latency (satellite link) |
| 44 | `SerialProtocol_Tramp` | VTX Tramp protocol |
| 45 | `SerialProtocol_DDS_XRCE` | DDS/XRCE (ROS 2 middleware) |
| 46 | `SerialProtocol_IMUOUT` | IMU data output stream |
| 48 | `SerialProtocol_PPP` | Point-to-Point Protocol (network) |
| 49 | `SerialProtocol_IBUS_Telem` | iBUS telemetry |
| 50 | `SerialProtocol_IOMCU` | IO co-processor |

**Default baud rates** (from config header):
- Console/USB: 115200
- MAVLink: 57600 (TX buf 256, RX buf 128)
- GPS: 230400 (TX buf 16, RX buf 256)
- MSP: 115200 (TX buf 256, RX buf 128)
- IMU OUT: 921600 (TX buf 2048, RX buf 128)
- FrSky D: 9600; FrSky SPort: 57600

---

## 3. AP_MSP

### 3.1 What Is MSP?

MultiWii Serial Protocol — originally developed for MultiWii flight controllers, adopted by Betaflight and iNav. Used as the native protocol for:
- **Betaflight OSD chips** (AT7456E and similar) — for rendering OSD overlays on FPV video
- **DJI FPV goggles** — for MSP-based OSD data
- **DisplayPort** — an extension for HD OSD systems (DJI O3, Walksnail, etc.)

ArduPilot implements MSP API version 1.42 (for DJI compatibility). It emits the `ARDUPILOT_IDENTIFIER = "ARDU"` flight controller identifier.

### 3.2 MSP Over MAVLink (Tunneling)

ArduPilot does NOT implement MSP-over-MAVLink tunneling via ENCAPSULATED_DATA. MSP is strictly a direct serial protocol. It is activated by setting `SERIALn_PROTOCOL=32` (MSP), `33` (DJI_FPV), or `42` (MSP_DisplayPort) on a serial port.

The GCS_MAVLINK code's `handle_data_packet()` handles `DATA96` messages but only for AP_Radio (type=42/43 for radio firmware upload) — completely separate from MSP.

### 3.3 Architecture

`AP_MSP` singleton owns up to 3 backend instances (`MSP_MAX_INSTANCES = 3`), one per configured MSP serial port. Each backend inherits from `AP_MSP_Telem_Backend` which inherits from `AP_RCTelemetry`.

**Backend types:**
- `AP_MSP_Telem_Generic` — standard MSP OSD (protocol=32)
- `AP_MSP_Telem_DJI` — DJI FPV MSP variant (protocol=33)
- `AP_MSP_Telem_DisplayPort` — HD OSD DisplayPort (protocol=42)

The main loop (`AP_MSP::loop()`) runs in a dedicated thread, calling `process_incoming_data()` and `process_outgoing_data()` on each backend.

### 3.4 MSP Packet Types Sent (OSD Data)

Data is sent in a 12-slot time-multiplexed schedule (`MSP_TIME_SLOT_MAX = 12`). Each slot sends one packet:

| Slot | MSP Packet | Content |
|------|-----------|---------|
| 0 | (empty) | — |
| 1 | `MSP_NAME` | Vehicle name |
| 2 | `MSP_STATUS` | Flight mode, arming status |
| 3 | `MSP_OSD_CONFIG` | OSD element positions and visibility |
| 4 | `MSP_RAW_GPS` | Fix type, satellites, lat/lon, altitude, speed, course |
| 5 | `MSP_COMP_GPS` | Distance to home, bearing to home |
| 6 | `MSP_ATTITUDE` | Roll, pitch, yaw |
| 7 | `MSP_ALTITUDE` | Altitude, vario |
| 8 | `MSP_ANALOG` | Battery voltage, mAh drawn, RSSI |
| 9 | `MSP_BATTERY_STATE` | Detailed battery state |
| 10 | `MSP_ESC_SENSOR_DATA` | ESC telemetry (if available) |
| 11 | `MSP_RTC_DATETIME` | Real-time clock for OSD timestamp |

**OSD item list** (58 items, `MSP::osd_items_e`): RSSI, battery voltage, crosshairs, artificial horizon, horizon sidebars, timers, fly mode, craft name, throttle, VTX channel, current draw, mAh drawn, GPS speed, GPS satellites, altitude, PID values, power, warnings, avg cell voltage, GPS lon/lat, pitch/roll angles, battery usage, disarmed indicator, home direction, home distance, heading, vario, compass bar, ESC temp, ESC RPM, remaining time, RTC datetime, G-force, motor diagnostics, log status, link quality, flight distance, RC channels, and more.

**Battery state struct** (passed to OSD):
- `batt_current_a` — current in amps
- `batt_consumed_mah` — consumed capacity
- `batt_voltage_v` — pack voltage
- `batt_capacity_mah` — configured capacity
- `batt_cellcount` — cell count
- `batt_state` — OK/WARNING/CRITICAL

**GPS state struct:**
- `fix_type`, `num_sats`, `lat`, `lon`, `alt_m`, `speed_cms`, `ground_course_dd`

**Options parameter (`MSP_OPTIONS`):**
- Bit 0: `TELEMETRY_MODE` — allows push-mode telemetry when only RX line connected
- Bit 1: unused
- Bit 2: `DISPLAYPORT_BTFL_SYMBOLS` — use Betaflight font table indices
- Bit 3: `DISPLAYPORT_INAV_SYMBOLS` — use iNav font table indices (overrides BTFL)

---

## 4. AP_DDS

### 4.1 What Is DDS?

Data Distribution Service — an OMG standard publish-subscribe middleware used as the transport layer for **ROS 2**. ArduPilot implements the **Micro XRCE-DDS** (formerly Micro-RTPS) client, which connects to an XRCE Agent running on a companion computer. The agent bridges to a full DDS network (and therefore to ROS 2).

This enables ArduPilot to appear as a native ROS 2 node, publishing sensor data and subscribing to control commands without any MAVLink translation layer.

**Library used:** `uxr/client/` (Micro XRCE-DDS client library). Serialization via `ucdr/microcdr`.

### 4.2 Integration Architecture

```
ArduPilot (AP_DDS_Client)
    |
    | Serial (SERIALn_PROTOCOL=45) or UDP
    |
XRCE-DDS Agent (companion computer, e.g., RPi)
    |
    | DDS (CDR-serialized)
    |
ROS 2 nodes / tools
```

The client runs in a dedicated thread (`main_loop()`). It:
1. Opens transport (serial or UDP).
2. Pings the XRCE agent with `ping_max_retry` attempts, `ping_timeout_ms` each.
3. Creates a session (`uxr_create_session()`), registers input/output reliable streams.
4. Creates entities: participant, publishers/subscribers, data writers/readers.
5. Runs the update loop: serializes current state, writes to output stream, runs `uxr_run_session_timeout()` to flush/receive.

**Transport options:**
- Serial: `SERIALn_PROTOCOL = 45` (`SerialProtocol_DDS_XRCE`). Uses custom transport callbacks.
- UDP: Requires networking support. Default IP `192.168.144.2` (ChibiOS) or `127.0.0.1` (SITL), port configured via param.

**Session parameters:**
- `DDS_MTU = 512` bytes
- `DDS_STREAM_HISTORY = 8` (8 MTU-sized buffers)
- `DDS_BUFFER_SIZE = 4096` bytes per stream
- Reliable streams only (no best-effort streams for the session itself)

**Namespace:** Topic names are prefixed with `rt/` (ROS topic convention). If `use_ns` is enabled, topics are further namespaced by `ap/SYS_ID/` (e.g., `rt/ap/1/navsat`).

### 4.3 Topics Published

| Topic name | ROS 2 type | Content | Default rate |
|-----------|-----------|---------|-------------|
| `time` | `builtin_interfaces/Time` | Boot time (sec + nsec) | 10ms (100 Hz) |
| `navsat` | `sensor_msgs/NavSatFix` | GPS lat/lon/alt, status | GPS update rate |
| `tf_static` | `tf2_msgs/TFMessage` | Static transforms (base_link → GPS frames) | On change (transient local) |
| `battery` | `sensor_msgs/BatteryState` | Voltage, current, capacity, cell count | 1000ms (1 Hz) |
| `imu/experimental/data` | `sensor_msgs/Imu` | Accel, gyro, orientation quat | 5ms (200 Hz) |
| `pose/filtered` | `geometry_msgs/PoseStamped` | Local NED position + orientation quat | 33ms (~30 Hz) |
| `twist/filtered` | `geometry_msgs/TwistStamped` | Local velocity (linear + angular) | 33ms (~30 Hz) |
| `airspeed` | `ardupilot_msgs/Airspeed` | True and indicated airspeed | 33ms (~30 Hz) |
| `rc` | `ardupilot_msgs/Rc` | RC channel values + RSSI | 100ms (10 Hz) |
| `geopose/filtered` | `geographic_msgs/GeoPoseStamped` | Global position + orientation | 33ms (~30 Hz) |
| `goal_lla` | `geographic_msgs/GeoPointStamped` | Current navigation goal (lat/lon/alt) | 200ms (5 Hz), only on change; transient local |
| `clock` | `rosgraph_msgs/Clock` | Simulation/wall clock (sim time) | 10ms (100 Hz) |
| `gps_global_origin/filtered` | `geographic_msgs/GeoPointStamped` | EKF origin in global coords | 1000ms (1 Hz) |
| `status` | `ardupilot_msgs/Status` | Armed state, mode, health flags | 100ms (10 Hz), only on change; transient local |

### 4.4 Topics Subscribed

| Topic name | ROS 2 type | Effect |
|-----------|-----------|--------|
| `joy` | `sensor_msgs/Joy` | Joystick input (axes → RC override equivalent) |
| `tf` | `tf2_msgs/TFMessage` | Dynamic transforms (external odometry/localization) |
| `cmd_vel` | `geometry_msgs/TwistStamped` | REP-147 velocity control commands |
| `cmd_gps_pose` | `ardupilot_msgs/GlobalPosition` | Global position/altitude setpoint commands |

### 4.5 Services (RPC, ArduPilot as Replier)

| Service name | Request type | Reply type | Effect |
|-------------|-------------|-----------|--------|
| `arm_motorsService` | `ardupilot_msgs/ArmMotors_Request` | `ArmMotors_Response` | Arm/disarm motors |
| `mode_switchService` | `ardupilot_msgs/ModeSwitch_Request` | `ModeSwitch_Response` | Change flight mode |
| `prearm_checkService` | `std_srvs/Trigger_Request` | `Trigger_Response` | Run pre-arm checks |
| `experimental/takeoffService` | `ardupilot_msgs/Takeoff_Request` | `Takeoff_Response` | VTOL takeoff |
| `set_parametersService` | `rcl_interfaces/SetParameters_Request` | `SetParameters_Response` | Set AP parameters via ROS 2 service |
| `get_parameterService` | `rcl_interfaces/GetParameters_Request` | `GetParameters_Response` | Get AP parameters via ROS 2 service |

**QoS note:** Most published topics use `VOLATILE` durability (no history for late-joiners) and `BEST_EFFORT` reliability. Exceptions: `tf_static`, `goal_lla`, `status`, and all services use `TRANSIENT_LOCAL` + `RELIABLE` so late-joining subscribers receive the last value.

---

## 5. Key Porting Notes for Meridian

### GCS_MAVLink Core

1. **The deferred bucket system is central**: Do not attempt to send messages directly. Every outbound message must go through the bucket/deferred system or it will interfere with flow control and rate limiting. Meridian needs an equivalent scheduler.

2. **Param protocol runs on an IO thread**: The async `param_requests`/`param_replies` queues decouple param lookup (which walks a linked list of all AP_Param) from the main loop. Meridian's param walk will need similar decoupling if param lookup is slow.

3. **Mission protocol has a mutex**: `MissionItemProtocol::receiving` plus the `link` pointer together form the critical section. Only one upload at a time, per protocol type. Meridian must enforce this.

4. **Signing storage is in FRAM, not flash**: The 32-byte key + 8-byte timestamp + magic are stored in a dedicated key-value storage area (`StorageManager::StorageKeys`). Meridian needs a persistent storage abstraction for this.

5. **Stream rate initialization comes from config files or param values**: On first boot with no params, all stream rates default to 0 (Copter/Blimp) or 1–10 Hz (Plane/Sub/Rover) based on vehicle type. Meridian must define defaults per vehicle configuration.

6. **`packet_overhead_chan()` must account for signing**: Any place that checks available buffer space must call this function, not hardcode the MAVLink overhead constant.

7. **MAVFTP runs in a separate thread with up to 5 sessions**: The IO thread with 2560-byte stack is significant. Meridian needs the AP_Filesystem abstraction or equivalent to support MAVFTP.

8. **Log download is fully delegated**: AP_Logger owns the entire log protocol. Meridian will need to implement this inside whatever logger is built.

### AP_SerialManager

9. **Port IDs above 20 are virtual**: Networking ports (21+), DroneCAN serial tunnel (41+, 51+), and scripting ports (61+) are dynamically registered and not physical UARTs. Meridian's serial abstraction must handle both physical and virtual ports through the same interface.

10. **Protocol 45 (DDS_XRCE) is the ROS 2 integration point**: This is how DDS gets a serial port. In Meridian, this needs to be plumbed to the DDS client's serial transport.

### AP_MSP

11. **MSP is direct serial, not tunneled over MAVLink**: There is no MAVLink encapsulation. Each MSP port is an independent serial connection. MSP and MAVLink coexist on different ports.

12. **DisplayPort is a separate sub-protocol** within MSP: It adds character rendering commands (`msp_displayport_write_string`, `msp_displayport_clear_screen`, `msp_displayport_draw_screen`). The font table selection (Betaflight vs iNav vs ArduPilot) matters for symbol rendering.

13. **MSP uses a fixed 12-slot time schedule**: Not rate-parameterizable. Each slot maps 1:1 to a packet type. Slot 0 is always empty (start/sync).

### AP_DDS

14. **DDS requires an XRCE agent on the companion computer**: ArduPilot only implements the client half. The agent (from `micro-xrce-dds-agent`) must be running on e.g. a Raspberry Pi, connected via serial or UDP.

15. **All DDS types are code-generated**: The `Idl/` directory contains `.idl` files from which the C type headers are generated by `gen_config_h.py`. Meridian will need Rust equivalents for each message type (or can use serde-derived structs).

16. **Services use a Replier pattern**: ArduPilot is the service server. ROS 2 nodes are clients. The request/reply is handled via separate request/reply topics with the `rq/` and `rr/` prefixes.

17. **DDS topic enables are compile-time flags**: Each topic is guarded by `#if AP_DDS_*_ENABLED`. In Meridian, these should be Cargo features.

18. **The IMU topic is marked experimental**: `AP_DDS_IMU_PUB_ENABLED = AP_DDS_EXPERIMENTAL_ENABLED`. At 200 Hz it generates significant traffic; treat as optional.

---

## Files Read

- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS.h` (entire, 1350+ lines)
- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS_MAVLink.h`
- `D:\projects\ardupilot\libraries\GCS_MAVLink\ap_message.h`
- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS_Common.cpp` (selected sections, 7600+ lines total)
- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS_MAVLink_Parameters.cpp`
- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS.cpp` (first 100 lines)
- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS_Param.cpp`
- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS_Signing.cpp`
- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS_FTP.h`
- `D:\projects\ardupilot\libraries\GCS_MAVLink\GCS_FTP.cpp` (first 120 lines)
- `D:\projects\ardupilot\libraries\GCS_MAVLink\MAVLink_routing.h`
- `D:\projects\ardupilot\libraries\GCS_MAVLink\MissionItemProtocol.h`
- `D:\projects\ardupilot\libraries\GCS_MAVLink\MissionItemProtocol.cpp`
- `D:\projects\ardupilot\libraries\AP_SerialManager\AP_SerialManager.h`
- `D:\projects\ardupilot\libraries\AP_SerialManager\AP_SerialManager_config.h`
- `D:\projects\ardupilot\libraries\AP_MSP\AP_MSP.h`
- `D:\projects\ardupilot\libraries\AP_MSP\AP_MSP.cpp`
- `D:\projects\ardupilot\libraries\AP_MSP\AP_MSP_Telem_Backend.h`
- `D:\projects\ardupilot\libraries\AP_MSP\msp_protocol.h`
- `D:\projects\ardupilot\libraries\AP_MSP\msp_osd.h`
- `D:\projects\ardupilot\libraries\AP_DDS\AP_DDS_Client.h`
- `D:\projects\ardupilot\libraries\AP_DDS\AP_DDS_Topic_Table.h`
- `D:\projects\ardupilot\libraries\AP_DDS\AP_DDS_Service_Table.h`
- `D:\projects\ardupilot\libraries\AP_DDS\AP_DDS_config.h`
