# ArduPilot Wave 3 Audit: OSD, Notification, Identity, and Compliance Systems

**Source libraries audited:** AP_OSD, AP_Notify, AP_ADSB, AP_OpenDroneID, AP_VideoTX, AP_AIS, AP_Radio
**Approximate source lines:** ~28K
**Audit date:** 2026-04-02

---

## 1. AP_OSD — On-Screen Display

### 1.1 Backend Types

Five OSD type values are defined in the `osd_types` enum (AP_OSD.h):

| Value | Name | Description |
|-------|------|-------------|
| 0 | `OSD_NONE` | Disabled |
| 1 | `OSD_MAX7456` | Analog video overlay, SPI-connected MAX7456 character generator IC |
| 2 | `OSD_SITL` | Software-in-the-loop simulation display via SFML window |
| 3 | `OSD_MSP` | MSP protocol OSD, used by Betaflight-compatible VTX units |
| 4 | `OSD_TXONLY` | No native display; OSD parameter block made available to external modules (e.g. CRSF) |
| 5 | `OSD_MSP_DISPLAYPORT` | MSP DisplayPort protocol, used for DJI O3/Walksnail digital goggles |

Two backends can run simultaneously — `osd_type` and `osd_type2` — and each is validated via `is_compatible_with_backend_type()` so they do not conflict (e.g. you cannot run two MAX7456 instances).

**MAX7456 specifics** (AP_OSD_MAX7456.h):
- SPI bus device
- Character frame buffer: 30 columns x 16 rows (PAL) or 13 rows (NTSC)
- Shadow frame buffer for dirty-rect optimization — only changed characters are re-sent
- `video_signal_reg` auto-detects PAL vs. NTSC
- SPI burst buffer is 512 bytes
- Font is loaded from ROMFS or SD card via `load_font_data()`

**MSP DisplayPort specifics** (AP_OSD_MSP_DisplayPort.h):
- Runs over a UART as an MSP protocol stream
- Supports a separate symbol set (INAV font mapping via `ap_to_inav_symbols_map[256][2]`)
- Per-screen HD resolution and font index parameters (`txt_resolution`, `font_index`)
- Supports WalkSnail and DJI WTFOS font directories under `fonts/HDFonts/`
- Blink state tracked per-instance (`_blink_on`)

**SITL specifics** (AP_OSD_SITL.h):
- Uses SFML for windowed rendering
- Character grid is 12x18 pixels per character with scale factor of 2
- Font loaded as 256-entry texture array (one per character code)
- Runs update loop in a separate pthread

### 1.2 Screen Layout — The OSDn_* Parameter System

Each of the four display screens is an `AP_OSD_Screen` object. Parameter names follow the pattern `OSD1_`, `OSD2_`, `OSD3_`, `OSD4_`.

Each item has three sub-parameters (AP_OSD_Setting):
- `*_EN` — enable/disable (0 or 1)
- `*_X` — horizontal position, range 0–59
- `*_Y` — vertical position, range 0–21

Example: `OSD1_ALTITUDE_EN`, `OSD1_ALTITUDE_X`, `OSD1_ALTITUDE_Y`

Screen-level parameters:
- `OSDn_ENABLE` — enable this screen (0/1)
- `OSDn_CHAN_MIN` / `OSDn_CHAN_MAX` — RC PWM range that activates this screen (900–2100)

Global OSD parameters:
- `OSD_TYPE` — backend type (1–5)
- `OSD_CHAN` — RC channel for screen switching
- `OSD_SW_METHOD` — switching method: TOGGLE (0), PWM_RANGE (1), AUTO_SWITCH (2)
- `OSD_FONT` — font selection index (matches font0.bin..font4.bin)
- `OSD_UNITS` — unit system: METRIC(0), IMPERIAL(1), SI(2), AVIATION(3)
- `OSD_V_OFFSET` / `OSD_H_OFFSET` — global position offsets for analog OSD
- `OSD_ARM_SCR` / `OSD_DISARM_SCR` / `OSD_FAILSAFE_SCR` — screen index for mode-based auto-switch

Warning thresholds (global):
- `OSD_W_RSSI`, `OSD_W_NSAT`, `OSD_W_TERR`, `OSD_W_AVGCELLV`, `OSD_W_RESTVOLT`, `OSD_W_BATVOLT`, `OSD_W_BAT2VLT`

### 1.3 OSD Item Types (Complete List)

Counting all items defined in `AP_OSD_Screen` (AP_OSD.h lines 178–256), there are **56 standard items** plus up to 5 extended link-stats items and 7 MSP-only items. The full list:

**Navigation & Position**
- `altitude` — barometric altitude AGL
- `gps_latitude` / `gps_longitude` — GPS coordinates
- `sats` — GPS satellite count
- `hdop` — GPS horizontal dilution of precision
- `home` — home direction arrow + distance
- `home_dist` (MSP only) — home distance
- `home_dir` (MSP only) — home direction
- `waypoint` — waypoint number and distance/bearing
- `xtrack_error` — cross-track error
- `dist` — total distance traveled
- `hgt_abvterr` — height above terrain (requires AP_Terrain)
- `pluscode` (conditional) — Open Location Code (Plus Code) of position

**Speed & Motion**
- `gspeed` — GPS ground speed
- `aspeed` — airspeed (primary sensor)
- `aspd1` / `aspd2` — airspeed sensors 1 and 2
- `vspeed` — vertical speed (climb/descent rate)
- `wind` — estimated wind speed and direction

**Attitude & Heading**
- `horizon` — artificial horizon (pitch ladder)
- `heading` — numeric magnetic heading
- `compass` — compass rose / heading tape
- `roll_angle` — roll angle
- `pitch_angle` — pitch angle
- `sidebars` (conditional) — speed/altitude sidebars with tick marks

**Power**
- `bat_volt` — primary battery voltage
- `restvolt` — resting/idle battery voltage
- `avgcellvolt` — average cell voltage
- `avgcellrestvolt` — average cell resting voltage
- `current` — primary battery current draw
- `batused` — primary battery mAh consumed
- `bat2_vlt` — secondary battery voltage
- `bat2used` — secondary battery mAh consumed
- `current2` — secondary battery current
- `eff` — flight efficiency (mAh/km or Wh/km)
- `climbeff` — climb efficiency
- `batt_bar` (MSP only) — battery bar indicator
- `cell_volt` (MSP only) — cell voltage
- `power` (MSP only) — power in watts

**ESC & RPM** (conditional compilation)
- `esc_temp` — ESC temperature
- `esc_rpm` — ESC RPM
- `esc_amps` — ESC current
- `rrpm` — rotor/motor RPM (AP_RPM sensor)

**RC & Link**
- `rssi` — RC signal strength (percentage)
- `link_quality` — link quality (alternative to RSSI)
- `throttle` — throttle position percentage
- `vtx_power` — VTX transmit power display

**Extended Link Stats** (CRSF only, `AP_OSD_EXTENDED_LNK_STATS`)
- `rc_tx_power` — TX transmit power
- `rc_rssi_dbm` — RSSI in dBm
- `rc_snr` — signal-to-noise ratio
- `rc_active_antenna` — active antenna index
- `rc_lq` — link quality

**Status & Info**
- `fltmode` — flight mode name string
- `message` — GCS status text (scrolling)
- `arming` (MSP only) — armed/disarmed indicator
- `crosshair` (MSP only) — center crosshair
- `stat` — post-flight statistics summary
- `flightime` — flight timer
- `clk` — real-time clock display
- `callsign` — operator callsign string (loaded from file)
- `fence` (conditional) — geo-fence status
- `rngf` (conditional) — rangefinder distance
- `temp` — barometer temperature
- `btemp` — second barometer temperature (when BARO_MAX_INSTANCES > 1)
- `atemp` — airspeed sensor temperature

**Total: 56 items in var_info, plus 5 extended link stats, plus 7 MSP-only = ~68 items defined in code.** All are individually positioned with (x, y, enabled) triplet parameters.

### 1.4 Rendering — Character-Based vs. Pixel-Based

All ArduPilot OSD backends are **character-cell based**, not pixel-based:
- The display is a grid of character positions
- Each cell maps to one character code (0x00–0xFF)
- Characters are rendered by the display hardware or font textures
- No pixel-level drawing is supported

The abstraction is the `write(uint8_t x, uint8_t y, const char* text)` method. All drawing functions format a string (with special character codes for symbols like arrows, battery icons, etc.) and call `write`.

**Font handling:**
- MAX7456: fonts are stored as `.mcm` files converted to binary via `mcm2bin.py`, then as `font0.bin`–`font4.bin`. Each file defines a 256-character set of 12x18 pixel glyphs (two bits per pixel = 4 gray levels). Fonts are uploaded to MAX7456 NVM over SPI via `update_font_char()`.
- MSP DisplayPort: fonts reside on the goggles. ArduPilot sends character codes; the goggle maps them. Separate symbol tables are maintained (see `AP_OSD_MSP_DisplayPort.h` symbol constants vs. `AP_OSD_Backend.h`). INAV font mapping is available via `ap_to_inav_symbols_map`.
- SITL: fonts loaded from file as SFML textures per character index.

**Symbol table:** 107 symbols are defined (`AP_OSD_NUM_SYMBOLS = 107`) in the backend base class. Each backend can override the default symbol codes via `init_symbol_set()`. The lookup table (`symbols_lookup_table[]`) is populated at init. Symbol categories include arrows (16 directional), artificial horizon chars, compass heading chars, battery, speed/altitude/voltage unit icons, sidebar elements, and status icons.

**Blink:** `blink_phase` cycles 0–3 at each `clear()` call; items drawn with `blink=true` are suppressed when `blink_phase != 0`.

**Decimal pack:** `OPTION_DECIMAL_PACK` compresses numeric strings by using single characters to represent decimal digits + decimal point, reducing character count for narrow screens.

### 1.5 Multi-Screen Configuration

**Screen count:**
- 4 display screens (`AP_OSD_NUM_DISPLAY_SCREENS = 4`)
- 2 parameter screens (`AP_OSD_NUM_PARAM_SCREENS = 2`, if `OSD_PARAM_ENABLED`)
- Total: up to 6 screen objects (`AP_OSD_NUM_SCREENS = 6`)

**Screen switching:** Controlled by `OSD_CHAN` (RC channel) and `OSD_SW_METHOD`:
- `TOGGLE (0)` — each transition above mid-stick toggles to next screen
- `PWM_RANGE (1)` — each screen has `CHAN_MIN`/`CHAN_MAX` defining the PWM range that selects it
- `AUTO_SWITCH (2)` — automatic: armed→`OSD_ARM_SCR`, failsafe→`OSD_FAILSAFE_SCR`, disarmed→`OSD_DISARM_SCR`

Debounce is implemented via `switch_debouncer` flag and 300ms timer (`last_switch_ms`).

**Parameter screens:** `AP_OSD_ParamScreen` (up to 9 parameters per screen) allows in-flight parameter adjustment via RC sticks. Supports a state machine: `PARAM_SELECT → PARAM_VALUE_MODIFY → PARAM_PARAM_MODIFY`. Parameter types include SERIAL_PROTOCOL, SERVO_FUNCTION, AUX_FUNCTION, FLIGHT_MODE, FAILSAFE_ACTION. Configuration via MAVLink `OSD_PARAM_CONFIG` / `OSD_PARAM_SHOW_CONFIG` messages.

---

## 2. AP_Notify — Notification System

### 2.1 Device Types

AP_Notify supports a large number of notification devices, selected via bitmask parameters. All devices inherit from `NotifyDevice`.

**RGB LED drivers** (via `RGBLed` base class):
- `AP_BoardLED` / `AP_BoardLED2` — built-in board LEDs (GPIO driven)
- `ToshibaLED_I2C` — Toshiba TLCXXXX I2C LED controllers (internal and external)
- `NCP5623` — NCP5623 I2C RGB LED (internal and external)
- `LP5562` — LP5562 I2C LED driver (internal and external)
- `IS31FL3195` — IS31FL3195 I2C LED controller (internal and external)
- `PCA9685LED_I2C` — PCA9685 16-channel I2C PWM controller
- `OreoLED_I2C` — 3DR Solo "Oreo" LED (I2C)
- `PixRacerLED` — PixRacer board LEDs
- `VRBoard_LED` — VRBrain board LEDs
- `NavigatorLED` — Navigator board LEDs
- `DiscreteRGBLed` — discrete GPIO-driven RGB channels
- `RCOutputRGBLed` — RGB via PWM/RC output channels
- `DiscoLED` — Parrot Disco LED

**Serial addressable LED strips:**
- `NeoPixel` / `NeoPixelRGB` — WS2812B / WS2811 addressable LED strip
- `ProfiLED` — ProfiLED addressable strip
- `ProfiLED_IOMCU` — ProfiLED driven from IO MCU
- `SerialLED` — base class for serial LED protocols
- `ScriptingLED` — virtual LED for Lua scripting access

**Buzzer:**
- `Buzzer` — simple GPIO buzzer (PWM or active buzzer), pin-selectable via `NTF_BUZZ_PIN`
- `AP_ToneAlarm` — musical tone alarm using MML (Music Macro Language) player
- DShot buzzer — ESC motor used as buzzer via DShot commands
- DroneCAN buzzer — buzzer via CAN bus

**Display:**
- `Display_SSD1306_I2C` — SSD1306 128x64 OLED (I2C)
- `Display_SH1106_I2C` — SH1106 128x64 OLED (I2C)
- `Display_SITL` — SITL text display
- `DroneCAN_RGB_LED` — RGB LED over DroneCAN

**DShot LED:**
- `DShotLED` — sets ESC LED color via DShot protocol commands

**SITL:**
- `SITL_SFML_LED` — SFML window showing LED state

Devices are selected at runtime via `NTF_LED_TYPES` (bitmask) and `NTF_BUZZ_TYPES` (bitmask). Maximum 6 simultaneously active devices (12 in SITL).

### 2.2 Notification Events

**Flags (persistent state, `notify_flags_and_values_type`):**
- `initialising` — system startup, do not move
- `gps_status` — GPS fix type enum
- `gps_num_sats` — satellite count
- `flight_mode` — current flight mode index
- `armed` — armed state
- `flying` — in-flight state
- `pre_arm_check` — pre-arm check pass
- `pre_arm_gps_check` — GPS pre-arm pass
- `save_trim` — trim save in progress
- `esc_calibration` — ESC calibration in progress
- `failsafe_radio` — RC failsafe active
- `failsafe_battery` — battery failsafe active
- `failsafe_gcs` — GCS (telemetry) failsafe active
- `failsafe_ekf` — EKF failsafe active
- `parachute_release` — parachute firing
- `ekf_bad` — EKF health failure
- `autopilot_mode` — in auto mode (used by OreoLED)
- `firmware_update` — firmware update in progress
- `compass_cal_running` — compass calibration active
- `leak_detected` — water leak detected (sub-specific)
- `gps_fusion` — GPS in use by EKF
- `gps_glitching` — GPS glitch detected
- `have_pos_abs` — absolute position valid
- `vehicle_lost` — lost vehicle tone requested
- `waiting_for_throw` — throw-to-launch mode waiting
- `powering_off` — smart battery shutdown
- `video_recording` — video recording active
- `temp_cal_running` — temperature calibration active
- `gyro_calibrated` — IMU calibration complete

**Events (transient, `notify_events_type`, auto-cleared after handling):**
- `arming_failed` — arm attempt rejected
- `user_mode_change` — pilot changed flight mode
- `user_mode_change_failed` — mode change rejected
- `failsafe_mode_change` — failsafe triggered mode change
- `autotune_complete` — autotune finished successfully
- `autotune_failed` — autotune failed
- `autotune_next_axis` — autotune moved to next axis
- `mission_complete` — autonomous mission finished
- `waypoint_complete` — waypoint reached
- `initiated_compass_cal` — compass calibration started
- `compass_cal_saved` — compass calibration saved
- `compass_cal_failed` — compass calibration failed
- `compass_cal_canceled` — compass calibration canceled
- `tune_started` — parameter tune started
- `tune_next` (3-bit) — tune advanced to next parameter
- `tune_save` — tune parameters saved
- `tune_error` — tune controller error
- `initiated_temp_cal` — temperature calibration started
- `temp_cal_saved` — temperature calibration saved
- `temp_cal_failed` — temperature calibration failed

### 2.3 LED Patterns

Patterns are defined as 30-bit packed color sequences (10 steps × 3 bits per step). Macros in `RGBLed.h`:

```
DEFINE_COLOUR_SEQUENCE(S0, S1, ..., S9)   — explicit 10-step sequence
DEFINE_COLOUR_SEQUENCE_SLOW(colour)        — 5 on, 5 off (slow blink)
DEFINE_COLOUR_SEQUENCE_SOLID(colour)       — all 10 steps same color
DEFINE_COLOUR_SEQUENCE_ALTERNATE(c1, c2)   — alternating between two colors
DEFINE_COLOUR_SEQUENCE_FAILSAFE(colour)    — 5 yellow, then 5 of color
```

Color values: BLACK=0, BLUE=1, GREEN=2, RED=4, YELLOW=5 (RED|GREEN), WHITE=7 (RED|GREEN|BLUE).

**Defined sequences (from RGBLed.h):**

| Sequence | Pattern |
|----------|---------|
| `sequence_initialising` | Alternating RED/BLUE |
| `sequence_trim_or_esc` | R/B/G repeating cycle |
| `sequence_failsafe_leak` | Failsafe + WHITE |
| `sequence_failsafe_ekf` | Failsafe + RED |
| `sequence_failsafe_gps_glitching` | Failsafe + BLUE |
| `sequence_failsafe_radio_or_battery` | Failsafe + BLACK (off) |
| `sequence_armed` | Solid GREEN |
| `sequence_armed_no_gps_or_no_location` | Solid BLUE |
| `sequence_prearm_failing` | Double YELLOW blink |
| `sequence_disarmed_good_dgps_and_location` | Alternating GREEN/BLACK |
| `sequence_disarmed_good_gps_and_location` | Slow GREEN blink |
| `sequence_disarmed_bad_gps_or_no_location` | Slow BLUE blink |

Sequences are sampled at 10Hz (one step per 100ms). LED update is called at 50Hz; step advance occurs every 5 calls.

**Priority:** The sequence selection follows a priority waterfall in `get_colour_sequence()`:
1. `initialising`
2. `esc_calibration` or `save_trim` (trim/ESC pattern)
3. `failsafe_ekf` (if ekf_bad)
4. `failsafe_leak`
5. `failsafe_gps_glitching`
6. `failsafe_radio` or `failsafe_battery`
7. `armed` (with GPS check for color variant)
8. `pre_arm_check` failing (yellow blink)
9. Disarmed with GPS quality variants

**Override sources** (`LED_OVERRIDE` parameter):
- 0: Standard (above logic)
- 1: MAVLink/Scripting/AP_Periph (external color command)
- 2: OutbackChallenge (GREEN=disarmed safe, RED=armed)
- 3: Traffic light (RED=armed, YELLOW=safety off, GREEN=safe)

Brightness is controlled by `NTF_LED_BRIGHT` (0=off, 1=low, 2=medium, 3=high). USB-connected boards cap at low brightness.

### 2.4 Buzzer Patterns

**Simple buzzer patterns** (Buzzer.cpp — bitmask, each bit = 100ms):
```
SINGLE_BUZZ  = 1 beep (100ms)
DOUBLE_BUZZ  = 2 beeps
ARMING_BUZZ  = 3 second continuous tone
BARO_BUZZ    = 5 short beeps
EKF_BAD      = morse-like pattern
INIT_GYRO    = triple-pulse repeating
PRE_ARM_GOOD = double + long sequence
```

**ToneAlarm MML tones** (ToneAlarm.cpp — 32 tones defined):
MML strings follow Microsoft QBasic PLAY syntax. The `MMLPlayer` interprets: `O` (octave), `L` (note length), `T` (tempo), `>/<` (octave up/down), note letters (A–G with sharps `#`), `P` (rest), `M` prefix for style (Normal/Legato/Staccato/Background).

| Index | Name | Tone String | Continuous |
|-------|------|-------------|------------|
| 0 | QUIET_NEG_FEEDBACK | `MFT200L4<<<B#A#2` | No |
| 1 | LOUD_NEG_FEEDBACK | `MFT100L4>B#A#2P8B#A#2` | No |
| 2 | QUIET_NEU_FEEDBACK | `MFT200L4<B#` | No |
| 3 | LOUD_NEU_FEEDBACK | `MFT100L4>B#` | No |
| 4 | QUIET_POS_FEEDBACK | `MFT200L4<A#B#` | No |
| 5 | LOUD_POS_FEEDBACK | `MFT100L4>A#B#` | No |
| 6 | LOUD_READY_OR_FINISHED | `MFT100L4>G#6A#6B#4` | No |
| 7 | QUIET_READY_OR_FINISHED | `MFT200L4<G#6A#6B#4` | No |
| 8 | LOUD_ATTENTION_NEEDED | `MFT100L4>A#A#A#A#` | No |
| 9 | QUIET_ARMING_WARNING | `MNT75L1O2G` | No |
| 10 | LOUD_WP_COMPLETE | `MFT200L8G>C3` | No |
| 11 | LOUD_LAND_WARNING_CTS | `MBT200L2A-G-A-G-A-G-` | **Yes** |
| 12 | LOUD_VEHICLE_LOST_CTS | `MBT200>A#1` | **Yes** |
| 13 | LOUD_BATTERY_ALERT_CTS | 16× rapid A# | **Yes** |
| 14 | QUIET_CALIBRATING_CTS | `MBNT255<C16P2` | **Yes** |
| 15 | WAITING_FOR_THROW | A#/D/F pattern | **Yes** |
| 16–22 | LOUD_1 through LOUD_7 | 1–7 B notes | No |
| 23 | TUNING_START | `MFT100L20>C#D#` | No |
| 24 | TUNING_SAVE | `MFT100L10DBDB>` | No |
| 25 | TUNING_ERROR | 8× B | No |
| 26 | LEAK_DETECTED | `MBT255L8>A+AA-` | **Yes** |
| 27 | QUIET_SHUTDOWN | Long melody | No |
| 28 | QUIET_NOT_READY | `MFT200L4<B#4A#6G#6` | No |
| 29 | STARTUP | Full melody | No |
| 30 | NO_SDCARD | `MNBGG` | No |
| 31 | EKF_ALERT | 4× groups of 4 rapid A# | **Yes** |

Custom tunes can be received via MAVLink `PLAY_TUNE` message or via Lua scripting `AP_Notify.play_tune()`.

### 2.5 Priority System

**LED priority:** State-machine waterfall (highest wins, described in 2.3). No numeric priority scores — pure if-else cascade in `get_colour_sequence()`.

**Buzzer priority (ToneAlarm):** Two categories:
- One-shot tones: queued and played once
- Continuous tones: stored as `_cont_tone_playing` index

When a continuous tone is active, one-shot tones interrupt it temporarily (max 2 second timeout via `AP_NOTIFY_TONEALARM_MAX_TONE_LENGTH_MS = 2000`), then the continuous tone resumes. Only one continuous tone plays at a time; a higher-priority event replaces the current one. Priority order (implicit, highest first):
1. EKF alert
2. Battery alert
3. Vehicle lost
4. Land warning
5. Leak detected
6. Waiting for throw
7. Calibrating

**LED_OVERRIDE flag:** When `LED_OVERRIDE=1`, MAVLink `LED_CONTROL` messages bypass the state machine entirely and directly set color/blink rate. This enables GCS-controlled lighting patterns.

---

## 3. AP_ADSB — Automatic Dependent Surveillance-Broadcast

### 3.1 What is ADSB?

ADS-B (Automatic Dependent Surveillance-Broadcast) is an aviation surveillance technology. Aircraft continuously broadcast their GPS position, altitude, velocity, ICAO identifier, and callsign on 1090 MHz (or 978 MHz UAT in North America). Ground stations and other aircraft with ADS-B receivers can track all nearby traffic. The "dependent" refers to dependency on onboard GPS rather than radar interrogation.

ArduPilot uses ADS-B for two purposes:
1. **ADS-B In:** receiving traffic from other aircraft to maintain a local vehicle list
2. **ADS-B Out:** broadcasting own position as a transponder (for regulatory compliance and collision avoidance reciprocity)

### 3.2 Receiver/Transponder Types

Four backend types are defined in `AP_ADSB::Type`:

| Type | Value | Description |
|------|-------|-------------|
| `None` | 0 | Disabled |
| `uAvionix_MAVLink` | 1 | uAvionix pingUSB/ping200X via MAVLink protocol |
| `Sagetech` | 2 | Sagetech XP (original/legacy) transponder |
| `uAvionix_UCP` | 3 | uAvionix UCP (newer serial protocol) |
| `Sagetech_MXS` | 4 | Sagetech MXS transponder (dedicated SDK) |

**uAvionix MAVLink** (AP_ADSB_uAvionix_MAVLink.h):
- Communicates via existing MAVLink channel
- Sends `UAVIONIX_ADSB_OUT_CFG` (static config) and dynamic position packets
- Uses `ADSB_VEHICLE` MAVLink messages for ADS-B In data

**Sagetech MXS** (AP_ADSB_Sagetech_MXS.h):
- Uses the official Sagetech SDK (`sagetech-sdk/sagetech_mxs.h`)
- Serial protocol with 0xAA start byte
- Message types: Installation, FlightID, Operating, GPS_Data, Target_Request, Mode
- Response types: ACK, Installation_Response, Status, Mode_Settings, Version, Serial_Number, Target_Summary, ADSB_StateVector, ADSB_ModeStatus, ADSB_TargetState, ADSB_AirRefVel
- Update schedule: Installation every 5s, FlightID every 8.2s, Operating every 1s, GPS at 5Hz (flying) or 1Hz (ground)

### 3.3 Threat Detection

Vehicle list management (AP_ADSB.h `in_state`):
- Configurable list size via `ADSB_LIST_MAX` (default 25 vehicles)
- Vehicles expire after 5000ms without update (`VEHICLE_TIMEOUT_MS`)
- List radius filter via `ADSB_LIST_RADIUS` — vehicles outside this radius are ignored entirely (default 10km for planes, 2km for others)
- Altitude filter via `ADSB_LIST_ALT`
- `determine_furthest_aircraft()` — tracks index and distance of the farthest vehicle to enable replacement when list is full

**ICAO target:** `ADSB_ICAO_SPECL` allows specifying one ICAO of interest that bypasses radius/altitude filters — used for monitoring specific aircraft.

Threat classification is delegated to the `AC_Avoidance` library (not in this library). AP_ADSB provides the vehicle list via `next_sample()` and `get_vehicle_by_ICAO()`. The avoidance subsystem polls for new samples and makes avoidance decisions.

### 3.4 Avoidance Actions

AP_ADSB itself does not take avoidance actions — it only maintains the sensor data. Avoidance action is handled by `AC_Avoidance` / `AP_Avoidance` which consumes ADSB data via the sample queue (`ObjectBuffer<adsb_vehicle_t> _samples{30}`). That subsystem is outside this library's scope.

### 3.5 Transponder Configuration (ADS-B Out)

`out_state.cfg` holds transponder configuration parameters:
- `ICAO_id` — 24-bit unique aircraft identifier (0 = auto-generate from GPS+time)
- `callsign` — 8-character aircraft identifier (A-Z, 0-9, space)
- `emitterType` — aircraft category (default 14 = UAV)
- `lengthWidth` — physical dimensions (for traffic display)
- `gpsOffsetLat` / `gpsOffsetLon` — GPS antenna physical offset
- `stall_speed_cm` — used for performance reporting
- `rfSelect` — RF capability bitmask (1090ES in/out, UAT in/out)
- `squawk_octal` — Mode 3/A squawk code (default 1200)

`out_state.ctrl` holds operational control:
- `modeAEnabled`, `modeCEnabled`, `modeSEnabled` — transponder mode selection
- `es1090TxEnabled` — 1090ES extended squitter transmit
- `emergencyState` — emergency squawk (7700, 7600, 7500)
- `identActive` — IDENT button activation

**Squawk automation via `AdsbOption` flags:**
- `Squawk_7400_FS_RC` — automatically squawk 7400 (lost link) on RC failsafe
- `Squawk_7400_FS_GCS` — automatically squawk 7400 on GCS failsafe
- `Ping200X_Send_GPS` — send GPS data to ping200X transponder

---

## 4. AP_OpenDroneID — Remote Identification

### 4.1 What is Remote ID?

Remote ID (RID) is an FAA (USA) and EU regulatory requirement for UAS (drones) to broadcast their identity and location so authorities can identify operators of UAVs in flight. The FAA rule (14 CFR Part 89) went into full effect September 2023. The European equivalent is EU 2019/945 and Commission Delegated Regulation 2019/945.

ArduPilot implements Remote ID via the OpenDroneID standard, which defines the message formats and broadcast methods.

### 4.2 Message Types

Five MAVLink OpenDroneID message types are used:

| Message | ArduPilot packet | Content |
|---------|-----------------|---------|
| `OPEN_DRONE_ID_BASIC_ID` | `pkt_basic_id` | UAS serial number, type (serial/CAA/etc), UA type |
| `OPEN_DRONE_ID_LOCATION` | `pkt_location` | Position, altitude, velocity, heading, status |
| `OPEN_DRONE_ID_SELF_ID` | `pkt_self_id` | Human-readable description string |
| `OPEN_DRONE_ID_SYSTEM` | `pkt_system` | Operator location, area count/radius/ceiling/floor |
| `OPEN_DRONE_ID_OPERATOR_ID` | `pkt_operator_id` | Operator ID string (CAA-assigned) |

**Transmission schedule:**
- Location message: 1 Hz (`_mavlink_dynamic_period_ms = 1000`)
- Static messages (BasicID, System, SelfID, OperatorID): every 3s (`_mavlink_static_period_ms = 3000`), round-robined

### 4.3 Broadcast Methods

Two transport paths are supported:

**MAVLink serial relay:**
- A serial port is configured via `DID_MAVPORT` parameter
- ArduPilot sends OpenDroneID MAVLink messages to a connected transponder module (e.g. BlueMark DroneBeacon, ArduRemoteID module)
- The transponder caches static messages and re-broadcasts at regulatory intervals
- The transponder handles the actual WiFi NAN and Bluetooth 4/5 radio transmission
- Reference implementation: https://github.com/ArduPilot/ArduRemoteID

**DroneCAN:**
- Via `DID_CANDRIVER` parameter
- Sends all 5 packet types over CAN to a DroneCAN-connected Remote ID node
- Tracks per-driver send masks (`need_send_location`, `need_send_basic_id`, etc.)

The physical broadcast methods (WiFi Neighbor Awareness Networking, Bluetooth 4 Legacy Advertising, Bluetooth 5 Long Range) are handled by the transponder hardware, not by ArduPilot directly.

### 4.4 Broadcast Data

Data field ranges enforced by constants:
- Direction: 0–360° (361 = invalid)
- Horizontal speed: 0–254.25 m/s (255 = invalid)
- Vertical speed: -62 to +62 m/s (63 = invalid)
- Altitude: -1000m to +31767.5m
- Timestamp: 0 to 3600 seconds (within the current hour)
- Area radius: 0–2550m
- Area count: 1–65000

**Location message data (sent at 1Hz):**
- Latitude/longitude (1e7 degrees)
- Geodetic altitude (WGS84) and barometric altitude
- Height above takeoff location
- Ground speed and climb rate
- Course over ground
- Horizontal/vertical/speed accuracy enums
- Timestamp within the current UTC hour
- Operational status (ground, airborne, emergency)

**Static message data (sent at 3s intervals):**
- Basic ID: UA type (helicopter/fixed-wing/etc.), ID type (serial number/CAA registration/UTM assignment), 20-char ID string
- System: operator lat/lon, area count/radius/floor/ceiling, timestamp
- Self ID: description type + 23-char description string
- Operator ID: operator ID type + 20-char operator ID

**Pre-arm checks:**
- ODID module must report arm status OK via `arm_status` packet from transponder
- Option `EnforcePreArmChecks` makes this a hard block
- Option `AllowNonGPSPosition` skips GPS requirement for position data
- Option `LockUASIDOnFirstBasicIDRx` locks UAS ID once received (prevents spoofing)

**Persistent UAS ID:** `load_UAS_ID_from_persistent_memory()` / `get_persistent_params()` ensure the UAS serial number survives reboots via the param system.

---

## 5. AP_VideoTX — Video Transmitter Control

### 5.1 VTX Control Protocols

Two serial protocols are implemented:

**SmartAudio** (AP_SmartAudio.h):
- Half-duplex UART at 4800 baud (auto-adjustable ±5% via smartbaud)
- Three protocol versions: v1, v2, v2.1
- Command/response packet structure: sync(0xAA) + header(0x55) + command + length + payload + CRC8
- Commands: GET_SETTINGS (0x03), SET_POWER (0x05), SET_CHANNEL (0x07), SET_FREQUENCY (0x09), SET_MODE (0x0B)
- v2.1 adds extended power level reporting (power in dBm + list of available levels)
- Ring buffer of 5 pending command packets

**TBS Tramp** (AP_Tramp.h):
- Full-duplex UART at 9600 baud (auto-adjustable ±5%)
- 16-byte fixed-length frames
- State machine: OFFLINE → INIT → ONLINE_MONITOR_FREQPWRPIT → ONLINE_MONITOR_TEMP → ONLINE_CONFIG
- Supports race lock detection (`TRAMP_CONTROL_RACE_LOCK`)
- Frequency range: 1000–5999 MHz
- Up to 20 retries for failed configuration changes

**CRSF VTX control** is also supported via the RC protocol layer (type `VTXType::CRSF = 1U<<0`).

### 5.2 Power, Frequency, and Band Management

**Bands** (AP_VideoTX.h, `VideoBand` enum):
- BAND_A, BAND_B, BAND_E, FATSHARK, RACEBAND, LOW_RACEBAND
- BAND_1G3_A, BAND_1G3_B (1.3 GHz)
- BAND_X, BAND_3G3_A, BAND_3G3_B (3.3 GHz)
- Total: 11 bands (`MAX_BANDS`)

Channels per band: up to 8 (`VTX_MAX_CHANNELS`). Frequency lookup via `VIDEO_CHANNELS[band][channel]` table.

**Power levels:** Up to 10 power levels (`VTX_MAX_POWER_LEVELS`). Each `PowerLevel` struct holds: level number, milliwatts, dBm, DAC value (v1 SmartAudio), active state. Power can be set by mW, dBm, level index, or DAC value.

**Parameters:**
- `VTX_FREQ` — frequency in MHz
- `VTX_POWER` — power in mW
- `VTX_MAX_POWER` — maximum allowed power cap
- `VTX_BAND` — band index
- `VTX_CHANNEL` — channel index within band
- `VTX_OPTIONS` — bitmask of options

**Configuration flow:**
1. At init, ArduPilot queries VTX for current settings
2. `set_defaults()` stores the VTX-reported settings as baseline
3. When params differ from current VTX state, `update_*()` methods return true
4. Backend (SmartAudio/Tramp) sends the appropriate command
5. VTX responds with confirmation; `_configuration_finished` is set

### 5.3 Pit Mode

Pit mode is zero-power transmit mode used before a race starts to avoid interfering with other pilots.

**Options bitmask (`VideoOptions`):**
- `VTX_PITMODE (1<<0)` — enable pit mode
- `VTX_PITMODE_UNTIL_ARM (1<<1)` — stay in pit mode until vehicle arms
- `VTX_PITMODE_ON_DISARM (1<<2)` — return to pit mode when disarmed
- `VTX_UNLOCKED (1<<3)` — unlock VTX from race lock
- `VTX_PULLDOWN (1<<4)` — pull SmartAudio wire low when idle
- `VTX_SA_ONE_STOP_BIT (1<<5)` — use 1 stop bit (some VTX quirk)
- `VTX_SA_IGNORE_CRC (1<<6)` — skip CRC validation
- `VTX_CRSF_IGNORE_STAT (1<<7)` — ignore CRSF VTX status packets

SmartAudio supports a separate pit-mode frequency (stored in VTX, retrieved via `request_pit_mode_frequency()`).
Tramp implements pit mode via `set_pit_mode(onoff)` which sends a dedicated command.

---

## 6. AP_AIS — Automatic Identification System

### 6.1 What is AIS?

AIS (Automatic Identification System) is the maritime equivalent of ADS-B. Ships broadcast their MMSI (Maritime Mobile Service Identity), name, position, speed, course, and voyage data on VHF radio (161.975 MHz and 162.025 MHz). All commercial vessels over 300 GT and passenger ships are legally required to carry AIS transponders under SOLAS.

ArduPilot implements AIS reception to provide maritime vessel awareness for USV (Unmanned Surface Vehicle) applications, primarily ArduRover in boat/rover mode.

### 6.2 Protocol — NMEA Sentence Parsing

**Message format:** NMEA 0183 `AIVDM` / `AIVDO` sentences (AP_AIS.h). AIVDM = received from other vessels; AIVDO = own vessel.

Sentence structure: `!AIVDM,<total>,<num>,<ID>,<channel>,<payload>,<pad>*<checksum>`

- `total` — number of sentences in this message (fragmentation)
- `num` — sentence number (for multi-part messages)
- `ID` — sequential message ID for matching fragments
- `payload` — ASCII armored 6-bit encoded data (up to 65 chars, `AIVDM_PAYLOAD_SIZE`)
- Buffer holds up to 10 fragmented messages simultaneously (`AIVDM_BUFFER_SIZE`)

**Payload encoding:** Each ASCII character maps to a 6-bit value via `payload_char_decode()`. Multi-character bit fields are extracted via `get_bits()` / `get_bits_signed()`.

**Message types decoded:**
- Position Report (types 1, 2, 3) — `decode_position_report()`: MMSI, nav status, SOG, COG, lat/lon, heading, timestamp
- Base Station Report (type 4) — `decode_base_station_report()`: fixed station position
- Static and Voyage Data (type 5) — `decode_static_and_voyage_data()`: vessel name, type, dimensions, destination, ETA

**Vessel list:**
- `ais_vehicle_t` wraps `mavlink_ais_vessel_t` with timestamps
- Max vessels tracked: `AIS_LIST_MAX` (default 25)
- Timeout: `AIS_TIME_OUT` seconds before removal (default: configurable)
- Expanding array (starts at 8 entries, grows as needed)

### 6.3 Integration — Data Reaching the GCS

Data flow:
1. AIS UART receives NMEA sentences character by character
2. `decode(char c)` parses the NMEA sentence incrementally
3. `decode_latest_term()` processes each comma-delimited field
4. On complete sentence: `payload_decode()` → message-type dispatch
5. Decoded vessels stored in `_list[]`
6. `send(mavlink_channel_t chan)` serializes `mavlink_ais_vessel_t` packets to GCS
7. Round-robin send: `_send_index` cycles through all tracked vessels

**Object avoidance integration:** `send_to_object_avoidance_database()` pushes valid-position vessels into `AP_OADatabase` so AC_Avoidance can treat ships as dynamic obstacles.

**Logging:** Three log options via `AIS_OPTIONS` bitmask:
- `AIS_OPTIONS_LOG_ALL_RAW` — log all raw AIVDM sentences
- `AIS_OPTIONS_LOG_UNSUPPORTED_RAW` — log unknown message types
- `AIS_OPTIONS_LOG_DECODED` — log decoded vessel data

Currently only one receiver type is implemented: `AISType::NMEA = 1` (direct UART parse).

---

## 7. AP_Radio — Integrated Radio Management

### 7.1 Note on Scope

This library is **not** SiK radio management. It is ArduPilot's driver for directly-attached 2.4 GHz RC radio chips (CYRF6936, CC2500, BK2425). SiK radio configuration (via AT commands, RADIO_STATUS) is handled by the GCS MAVLink layer and `AP_SerialManager`, not this library.

**Supported chips:**
- `CYRF6936` — used in original 3DR Spektrum-compatible receivers
- `CC2500` — used in FrSky-compatible embedded receivers (e.g. Pixhawk 4 Mini built-in)
- `BK2425` — used in NRF24L01-compatible systems

### 7.2 CC2500 / FrSky Protocol

The CC2500 backend (`AP_Radio_cc2500.cpp`) implements FrSky D16 and GFSK protocols. It operates at 2.4 GHz with frequency hopping. Telemetry is sent from RX (autopilot) to TX (transmitter) via the `telem_packet_cc2500` structure.

**Telemetry uplink to transmitter (`telem_structure.h`):**

```
telem_status_cc2500:
  pps          — packets per second (link quality indicator)
  rssi         — lowpass RSSI value
  flags        — GPS_OK, ARM_OK, BATT_OK, ARMED, POS_OK, VIDEO, HYBRID
  flight_mode  — current flight mode index
  wifi_chan    — companion computer WiFi channel (to avoid interference)
  tx_max       — max TX power setting
  note_adjust  — buzzer tone pitch adjustment for the transmitter
  rxid[2]      — receiver ID for anti-double-bind
```

**Packet types** (telem_structure.h):
- `TELEM_STATUS` — status packet (above struct)
- `TELEM_PLAY` — play a tune on the transmitter buzzer
- `TELEM_FW` — firmware update packet

**Autobind:** `autobind_packet_cc2500` contains TXID, TXID inverse (validation), and companion WiFi channel. Auto-bind can trigger if `RADIO_AUTO_BIND_TIME` seconds of no link.

### 7.3 BK2425 Protocol

`AP_Radio_bk2425.cpp` implements a similar protocol over NRF24L01-compatible BK2425 chips. Used in some FPV transmitters. Supports up to 16 PWM channels.

### 7.4 Radio Statistics

All backends expose `AP_Radio::stats`:
```
bad_packets   — CRC failures or malformed packets
recv_errors   — receive errors
recv_packets  — successfully received RC packets
lost_packets  — packets that timed out
timeouts      — total timeout events
```

RSSI is available via `radio.rssi_chan` — the channel number where the backend injects RSSI as a PWM value (0–100% scaled to 1000–2000 µs). Similarly `pps_chan` injects packets-per-second. These are consumed by the RC channel layer and can be displayed on OSD.

### 7.5 WiFi Interference Avoidance

`set_wifi_channel(uint8_t channel)` allows a companion computer to inform the radio driver which 2.4 GHz WiFi channel it is using. The radio driver can then avoid hopping to overlapping frequencies. The WiFi channel number is also relayed back to the transmitter in the telemetry status packet.

---

## Meridian Porting Notes

### AP_OSD
- The OSD system is highly display-specific. Meridian will need trait objects for `OsdBackend` with `write(x, y, text)`, `flush()`, `clear()`, `init()`.
- The parameter system (OSDn_ITEM_EN/X/Y) maps naturally to Rust structs with serde-compatible serialization.
- Character-based rendering means Meridian only needs to push character arrays to hardware — no pixel-level GPU API.
- MAX7456 SPI driver will need careful timing; uses 512-byte burst buffer.
- The symbol lookup table abstraction (107 symbols, different codes per backend) is important to preserve for font portability.
- 4 display screens + 2 param screens = 6 total screen objects.
- 56+ individual OSD item types, each with enabled/x/y — consider a `OsdItem` enum or trait.

### AP_Notify
- Priority system is implicitly encoded in a waterfall — Meridian should make it explicit (a priority enum).
- 10-step LED sequences are elegant; consider a `[Color; 10]` array with a step counter.
- 32 MML tone strings need a Rust MML player or equivalent.
- ~20 LED device types — many are I2C; most share the same RGB pattern logic via `RGBLed` base.
- The flags/events split (persistent vs. transient) is a clean design worth preserving.

### AP_ADSB
- ICAO 24-bit addressing, vehicle list management, timeout logic, radius filtering are all self-contained.
- Sagetech MXS SDK is in C — will need Rust FFI or reimplementation.
- ADS-B In and ADS-B Out are distinct subsystems sharing the same library.
- Threat detection is NOT in this library — it is in `AC_Avoidance`.

### AP_OpenDroneID
- The 5 message types map exactly to the OpenDroneID spec (ASTM F3411).
- Timing: location at 1Hz, static at 3s round-robin.
- Both MAVLink serial relay and DroneCAN paths are needed.
- UAS ID must persist across reboots.
- Pre-arm integration requires arm_status handshake from transponder.

### AP_VideoTX
- SmartAudio and Tramp are distinct UART protocols; both need half/full-duplex UART drivers.
- Band/channel/power abstraction is clean — `VideoBand` enum + lookup table.
- Pit mode logic (arm/disarm triggered) should be a state machine.
- Configuration "write-back confirmation" loop (query → configure → verify) is important for reliability.

### AP_AIS
- NMEA parsing is incremental (character by character) — suitable for DMA-driven UART in Rust.
- 6-bit ASCII armoring decode is simple but must be correct.
- Only one receiver type (NMEA UART) — straightforward to implement.
- Integration with obstacle avoidance database is the main output path.

### AP_Radio
- This library is for embedded RC radio chips (CC2500, BK2425, CYRF6936), not SiK/telemetry radios.
- Only relevant to Meridian if it flies with a directly-attached RF radio chip (unusual for fixed-wing/copter).
- WiFi channel interference avoidance and telemetry uplink (flight mode, RSSI, tune playback) are useful features.
- SiK radio MAVLink flow control (RADIO_STATUS) is in GCS_MAVLink layer, audited in Wave 2.
