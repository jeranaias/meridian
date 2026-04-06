# Mission Planner GCS — Exhaustive Feature Reference

*Research document for Meridian web GCS parity planning.*
*Sources: ArduPilot official docs, DeepWiki MissionPlanner repo analysis, GitHub source code (CurrentState.cs), ArduPilot Discourse.*

---

## 1. Main Screen Layout

### Overall Window Structure

Mission Planner is a Windows-native C# .NET application (WinForms). The main application form is `MainV2`. Navigation between major screens is managed by `MainSwitcher`, a tab/view control.

**Top bar (always visible):**
- Application title and version
- Main navigation tabs (see below)
- Upper-right connection cluster: COM port dropdown, baud rate dropdown, CONNECT/DISCONNECT button
- Right-click the CONNECT button → "Connection Options" (for multiple vehicle connections)
- "Stats…" hotlink below port selector — shows link statistics and MAVLink signing status

**Main navigation tabs across the top:**
1. CONNECT (or DATA / Flight Data)
2. PLAN (Flight Plan)
3. SETUP (Initial Setup)
4. CONFIG (Configuration and Tuning)
5. SIMULATION
6. HELP

Some versions label the first active tab "DATA" or "Flight Data" after connection.

**Default layout (Flight Data tab):**
- Upper-left: HUD (Heads-Up Display) — takes roughly the left third of the screen
- Lower-left: tabbed control panel (Actions, Quick, Gauges, Status, Servo/Relay, Telemetry Logs, DataFlash Logs, Scripts, Messages)
- Right (majority): Map view (GMapControl)

The HUD can be popped out to a second screen by double-clicking it. The map and HUD panels are separated by a splitter (`MainH`) that can be resized.

---

## 2. Flight Data Tab (DATA)

### 2.1 HUD (Heads-Up Display)

Located upper-left. The HUD renders via `ExtLibs/Controls/HUD.cs`.

**The 15 documented HUD indicators:**
1. **Airspeed** — airspeed in configured units; shows groundspeed if no airspeed sensor fitted
2. **Groundspeed** — speed over ground
3. **Altitude** — current altitude with home offset; blue vertical bar = rate of climb
4. **Rate of Climb** — visualized as bar next to altitude tape
5. **Artificial Horizon** — canvas animation of pitch/roll; green = ground, blue = sky
6. **Aircraft Attitude** — pitch angle and bank angle overlay on horizon
7. **Bank Angle** — wing-tilt indicator
8. **Heading Direction** — compass heading tape across the top of HUD
9. **Crosstrack Error and Turn Rate (T)** — deviation from planned track; turn rate indicator
10. **GPS Time** — GPS-derived UTC timestamp
11. **GPS Status** — lock quality indicator (No GPS / 2D / 3D / DGPS / RTK Float / RTK Fixed)
12. **Battery Status** — voltage and/or remaining percentage
13. **Distance to Waypoint / Waypoint Number** — "dist > WP#" display
14. **Current Flight Mode** — text label of active flight mode
15. **Telemetry Link Quality** — rolling average percentage of good packets

**Additional HUD elements (from hud.html web HUD):**
- Target heading (yaw) vs. actual ground course overlay
- Altitude above sea level (`altasl`)
- Terrain altitude (`ter_alt`)
- Home altitude (`HomeAlt`)
- EKF status indicator (icon or text, toggleable)
- VIBE status indicator (vibration level warning, toggleable)
- Armed / Disarmed state text

**HUD right-click context menu:**
- **Video Controls:**
  - Record as .avi (saves to logs folder)
  - Set MJPEG source (enter URL)
  - Set GStreamer source
  - Start camera feed
  - Display HereLink video
- **Display Settings:**
  - Toggle aspect ratio (4:1 or 16:9)
  - Swap HUD with map (puts map top-left, HUD in main area)
  - Change ground color
  - Toggle Russian-style HUD (alternate PFD layout)
- **User Items:** Add any telemetry parameter from CurrentState to the HUD overlay; scripting `NAMED_VALUE_FLOAT` values also appear here
- **HUD Items:** Toggle individual HUD elements on/off:
  - Heading
  - Speed
  - Altitude
  - Battery
  - EKF status
  - VIBE status
  - GPS status
- **Icon/Text Toggle:** Switch lower HUD status elements between icon and text mode

---

### 2.2 Map Area (Right Panel)

Rendered by `GMapControl1`. Multiple overlay layers:

**Map overlays:**
- Vehicle position marker (icon changes by vehicle type)
- Vehicle trail / track (purple line, cleared with "Clear Track" button)
- Home position marker
- Waypoints (numbered markers)
- Flight path / mission route line
- Geofence (polygon and circle overlays)
- Rally points
- ADSB/AIS traffic overlay
- Proximity sensor 360° overlay (if equipped)
- Camera footprint overlap visualization (for survey missions)

**Bottom of map controls:**
- `hdop` — GPS HDOP value
- `sats` — satellite count
- **Legend** — color-coded direction reference
- **Tuning** checkbox — opens real-time graph window; double-click the graph legend to select parameters to plot
- **Auto Pan** toggle — keeps vehicle centered on map
- Zoom scrollbar

**Map right-click context menu (Flight Data view):**
- **Fly to here** — sends Guided Mode target; vehicle navigates and loiters
- **Fly to here Alt** — Guided Mode with user-specified altitude dialog
- **Fly to Coords** — navigate using lat/lon coordinate input
- **Add POI** — create named Point of Interest at cursor
- **Delete POI** — remove POI
- **Save POI** — persist POI list to file
- **Load POI** — restore saved POI list
- **Point Camera Here** — sends DO_SET_ROI to point camera gimbal at cursor location
- **Point Camera Coords** — DO_SET_ROI with coordinate input
- **Trigger Camera Now** — fires camera shutter if configured
- **Flight Planner** — opens mission planning overlay within the current map
- **Set Home Here** — redefines the RTL home position at cursor
- **TakeOff** — command takeoff to specified altitude (Copter/QuadPlane)
- **Camera Overlap** — display survey mission camera overlap visualization
- **Altitude Angel Settings** — configure Altitude Angel airspace reporting integration
- **Stats** — developmental feature

---

### 2.3 Control and Status Panel (Lower-Left Tabs)

Eight tabs. Can be moved to separate windows.

#### Tab: Quick

Large-text display of user-selected telemetry values.
- Double-click to select which parameters to display
- Right-click to change the grid layout (rows × columns)
- Right-click to move to a separate floating window

#### Tab: Actions

**Dropdowns (top to bottom):**
1. MAVLink action selector — choose a MAVLink command to send
2. Waypoint selector — choose which WP to fly to
3. Flight mode selector — choose a mode to switch to
4. Camera mount state selector — choose gimbal/mount mode

Each dropdown has a corresponding button to send the selected value to the vehicle.

**Shortcut mode buttons:**
- **Auto** — switches to Auto mission mode
- **Loiter** — switches to Loiter mode
- **RTL** — switches to Return-To-Launch mode

**Other action buttons:**
- **Arm/Disarm** — toggles arm state (with safety confirmation)
- **Set Home Alt** — sets altitude reference zero to current altitude
- **Restart Mission** — resets mission to beginning
- **Resume Mission** — enters Auto mode resuming at last-reached waypoint
- **Change Speed** — sends DO_CHANGE_SPEED with m/s input
- **Change Alt** — sends altitude change command (in configured units)
- **Set Loiter Rad** — sets loiter circle radius in meters
- **Clear Track** — clears the purple vehicle trail on the map
- **Abort Landing** — executes platform-specific landing abort procedure
- **Joystick** — opens joystick configuration dialog
- **Raw Sensor View** — opens separate window showing:
  - Roll, pitch, yaw angles (degrees)
  - Gyroscope X/Y/Z readings
  - Accelerometer X/Y/Z readings
  - 8 RC input channel values (PWM)
  - 8 RC servo output values (PWM)

#### Tab: Gauges

- Displays four telemetry gauges in circular analog dial format
- Default gauges: typically airspeed, altitude, heading, vertical speed
- Double-click the speed gauge to adjust the maximum display value

#### Tab: Status

Displays all telemetry parameters from `CurrentState` as a scrollable key-value list. Every field is shown. Double-click any value to add it to the Tuning graph.

**Complete list of available Status fields (from CurrentState.cs):**

*Attitude:*
- roll, pitch, yaw
- SSA (Side Slip Angle), AOA (Angle of Attack), crit_AOA

*Position:*
- lat, lng, alt, altasl
- groundspeed, groundcourse
- airspeed, targetairspeed
- vx, vy, vz (velocity components), vlen
- distTraveled
- timeSinceArmInAir, timeInAir, timeInAirMinSec
- wind_dir, wind_vel
- QNH (sea-level barometric pressure)
- horizondist

*GPS (Primary):*
- gpsstatus, gpshdop, satcount
- gpsh_acc, gpsv_acc, gpsvel_acc, gpshdg_acc
- gpsyaw, gpstime

*GPS (Secondary):*
- lat2, lng2, altasl2
- gpsstatus2, gpshdop2, satcount2
- groundspeed2, groundcourse2
- gpsh_acc2, gpsv_acc2, gpsvel_acc2, gpshdg_acc2, gpsyaw2
- satcountB (combined count)

*IMU / Sensors (Primary):*
- ax, ay, az (accelerometer), accelsq
- gx, gy, gz (gyroscope), gyrosq
- mx, my, mz (magnetometer), magfield
- imu1_temp

*IMU / Sensors (Secondary):*
- ax2, ay2, az2, accelsq2
- gx2, gy2, gz2, gyrosq2
- mx2, my2, mz2, magfield2
- imu2_temp

*IMU / Sensors (Tertiary):*
- ax3, ay3, az3, accelsq3
- gx3, gy3, gz3, gyrosq3
- mx3, my3, mz3, magfield3
- imu3_temp

*Airspeed Sensors:*
- airspeed1_temp, airspeed2_temp
- asratio (airspeed ratio calibration factor)
- lowairspeed (flag)

*Hygrometer:*
- hygrotemp1, hygrohumi1
- hygrotemp2, hygrohumi2

*Radio Input (16 channels):*
- ch1in through ch16in (PWM values)

*Radio Output (32 channels):*
- ch1out through ch32out (PWM values)
- ch1percent, ch3percent

*ESC Telemetry (16 ESCs):*
- esc1_volt through esc16_volt
- esc1_curr through esc16_curr
- esc1_rpm through esc16_rpm
- esc1_temp through esc16_temp

*Navigation:*
- nav_roll, nav_pitch (targets)
- nav_bearing, target_bearing
- wp_dist (distance to waypoint)
- alt_error, ber_error (bearing error), aspd_error, xtrack_error
- wpno (current waypoint number)
- mode (flight mode name)
- climbrate, verticalspeed, verticalspeed_fpm
- tot (time over target), toh (time over home)
- targetalt, targetaltd100

*Battery (up to 9 batteries):*
- battery_voltage, battery_voltage2 … battery_voltage9
- battery_remaining, battery_remaining2 … battery_remaining9
- current, current2 … current9
- watts
- battery_usedmah, battery_usedmah2 … battery_usedmah9
- battery_cell1 through battery_cell14 (individual cell voltages)
- battery_temp through battery_temp9
- battery_remainmin through battery_remainmin9
- battery_mahperkm, battery_kmleft

*System:*
- failsafe (state)
- rxrssi (receiver RSSI)
- sensors_enabled, sensors_health, sensors_present
- prearmstatus
- messageHighSeverity

*Calculated:*
- glide_ratio, turnrate, turng, radius
- altd1000, altd100 (altitude digit extraction for OSD)
- lowgroundspeed (flag)
- customfield0 through customfield19 (scripting custom values)

*Location objects:*
- HomeLocation, PlannedHomeLocation, HomeAlt
- TrackerLocation, TargetLocation
- GeoFenceDist (distance to nearest fence boundary)

*RC Override:*
- rcoverridech1 through rcoverridech18

*Unit multipliers:*
- multiplierdist / DistanceUnit
- multiplierspeed / SpeedUnit
- multiplieralt / AltUnit

**Graphing (Tuning window):**
- Enable "Tuning" checkbox at bottom of map → floating graph window opens
- Double-click graph legend to select parameters
- Left-click a parameter → assigns to left Y-axis
- Right-click a parameter → assigns to right Y-axis
- Mouse wheel = zoom
- Drag = pan
- Color cycling per parameter

#### Tab: Servo/Relay

- Set relay pin 0-3 to LOW, HIGH, or momentary toggle
- RC channel override (channels 5–14): send PWM override values
- Caution warning: channel overrides can change flight modes
- Used for bench testing servo behavior and relay outputs

#### Tab: Telemetry Logs

Controls for `.tlog` files (MAVLink telemetry recordings):

- **Load Log** — open a .tlog file
- **Play** — begin playback
- **Speed buttons** — adjust playback rate (faster/slower)
- **Timeline slider** — jump to any point in the log
- During playback: HUD animates, map updates, Status tab updates in real time
- **Tlog > KML or Graph** button:
  - Graph mode: select data fields (organized by MAVLink message type: RC_CHANNELS, RAW_IMU, etc.)
  - KML/KMZ export: generates Google Earth file with color-coded flight mode segments
  - Parameter extraction: generates tab-separated .param file
  - Waypoint extraction: generates .txt mission file

#### Tab: DataFlash Logs

Controls for on-board `.bin` / `.log` dataflash files:

- **Download DataFlash Log Via MAVLink** — lists all logs on SD card; download selected to `MissionPlanner/logs/VEHICLETYPE/`
- Automatically generates `.kmz` file on download for Google Earth viewing
- **Review a Log** — opens the log analysis viewer:
  - Row-based log browser (message types as rows: PARM, GPS, IMU, ATT, MODE, etc.)
  - Click a row type → column headers update to show that message's fields
  - Click a column → **Graph this data** button plots the field
  - Scroll wheel = zoom in graph; drag-select = region zoom; right-click → "Set Scale to Default"
  - Filter dropdown — filter by message type (e.g., show only MODE messages)
  - Line numbers displayed on left side
  - Header shows firmware version, board type, log format version
  - Parameter values at beginning of log shown in PARM rows

#### Tab: Scripts

- Python scripting panel
- Run Python automation scripts using vehicle state variables from CurrentState
- Script access to vehicle data and command sending

#### Tab: Messages

- Scrolling text log of all messages from the vehicle
- Severity levels: EMERGENCY, ALERT, CRITICAL, ERROR, WARNING, NOTICE, INFO, DEBUG
- High-severity messages also appear overlaid on the HUD
- Color-coded by severity level

---

## 3. Flight Plan Tab (PLAN)

### 3.1 Layout

- **Left panel** (`panelWaypoints`): waypoint list grid + controls
- **Right panel** (`panelMap`): GMapControl with mission overlays
- **Top-left info bar**: distance from HOME, heading and distance from last waypoint, total trip distance
- **Mode selector dropdown**: MISSION / FENCES / RALLY

### 3.2 Waypoint List Grid (`Commands` DataGridView)

Columns present for each waypoint row:
- **#** — waypoint sequence number
- **Command** — MAVLink command dropdown selector
- **Param 1** — varies by command (e.g., delay seconds, turn count)
- **Param 2** — varies by command
- **Param 3** — varies by command
- **Param 4** — varies by command (often heading/yaw)
- **Lat** — latitude (decimal degrees)
- **Lon** — longitude (decimal degrees)
- **Alt** — altitude (in configured units)
- **Frame** — altitude reference (Relative / Absolute ASL / Above Terrain)
- **Delete** button per row

### 3.3 Waypoint Toolbar / Control Buttons

- **Read** — download current mission from autopilot
- **Write** — upload mission to autopilot
- **Load File** — import .waypoints / .txt / KML / SHP mission files
- **Save File** — export current mission to file
- **Clear Mission** (workspace only, does not affect autopilot)
- **WP Radius** input — sets acceptance radius for all waypoints
- **Loiter Radius** input — sets loiter circle radius
- **Default Alt** input — altitude applied to new waypoints
- **Altitude type** selector — Relative / Absolute / Above Terrain
- **Verify height** checkbox — adjusts waypoint altitudes using SRTM terrain data (Google Earth topology)
- **Home Location** button — manually set HOME coordinates
- **Prefetch** — caches map tiles for offline use
- **Modify Alt** — batch-change all mission waypoint altitudes

### 3.4 Supported MAVLink Mission Commands

**Navigation (NAV) Commands:**
- `NAV_WAYPOINT` — navigate to position (Param1: hold time seconds; Lat/Lon/Alt)
- `NAV_LOITER_UNLIM` — loiter indefinitely at position
- `NAV_LOITER_TURNS` — loiter N turns (Param1: turns; Param2: direction; Lat/Lon/Alt)
- `NAV_LOITER_TIME` — loiter for T seconds (Param1: seconds; Lat/Lon/Alt)
- `NAV_LOITER_TO_ALT` — loiter until reaching altitude
- `NAV_RETURN_TO_LAUNCH` — RTL to home or nearest rally point
- `NAV_LAND` — land at position
- `NAV_TAKEOFF` — takeoff to altitude
- `NAV_CONTINUE_AND_CHANGE_ALT` — continue mission and adjust altitude
- `NAV_DELAY` — delay mission for seconds or until a specific time
- `NAV_PAYLOAD_PLACE` — descend and place payload (Copter/Plane)
- `NAV_SPLINE_WAYPOINT` — smooth spline path waypoint (Copter)

**Condition Commands:**
- `CONDITION_DELAY` — delay DO commands by N seconds
- `CONDITION_DISTANCE` — delay DO commands until within N meters of next WP
- `CONDITION_YAW` — wait until vehicle reaches specified yaw

**DO Commands:**
- `DO_JUMP` — jump to WP# with optional repeat count
- `JUMP_TAG` — location marker for jump targets (tag number 1–65535)
- `DO_JUMP_TAG` — jump to tag number with optional repeat count
- `DO_CHANGE_SPEED` — set target speed (Param1: speed type; Param2: speed m/s; Param3: throttle %)
- `DO_SET_HOME` — set home to current position or specified coordinates
- `DO_SET_RELAY` — set relay pin voltage (Param1: relay number; Param2: 0=low/1=high)
- `DO_REPEAT_RELAY` — toggle relay N times with period (Param1: relay; Param2: count; Param3: delay)
- `DO_SET_SERVO` — set servo PWM (Param1: servo number; Param2: PWM microseconds)
- `DO_REPEAT_SERVO` — cycle servo between mid and target (Param1: servo; Param2: PWM; Param3: count; Param4: delay)
- `DO_SET_ROI` — point camera/vehicle toward location (Region of Interest)
- `DO_SET_ROI_LOCATION` — ROI at specific coordinates
- `DO_SET_ROI_NONE` — clear ROI
- `DO_DIGICAM_CONFIGURE` — configure camera (mode, shutter speed, aperture, ISO, exposure)
- `DO_DIGICAM_CONTROL` — trigger camera shutter
- `DO_MOUNT_CONTROL` — control gimbal (pitch, roll, yaw angles, mount mode)
- `DO_GIMBAL_MANAGER_PITCHYAW` — move gimbal to pitch/yaw angles with optional rates
- `DO_SET_CAM_TRIGG_DIST` — trigger camera every N meters
- `DO_FENCE_ENABLE` — enable/disable geofence (Param1: 0/1/2; Param2: fence type bitmask)
- `DO_AUX_FUNCTION` — trigger auxiliary function (like RC switch; Param1: function; Param2: switch position 0/1/2)
- `DO_SET_RESUME_REPEAT_DIST` — set rewind distance for mission resume
- `STORAGE_FORMAT` — format SD card (both params must be 1)

### 3.5 Right-Click Map Menu (Flight Plan view)

- **Delete WP** — removes clicked waypoint
- **Insert WP** — inserts new waypoint at cursor position
- **Insert Spline WP** — inserts spline waypoint (Copter only)
- **Loiter** — adds NAV_LOITER_UNLIM at cursor
- **Jump** — adds DO_JUMP command
- **RTL** — adds NAV_RETURN_TO_LAUNCH
- **Land** — adds NAV_LAND at cursor
- **Takeoff** — adds NAV_TAKEOFF
- **DO_SET_ROI** — adds region-of-interest command at cursor
- **Clear Mission** — clears workspace mission (not autopilot)
- **Set Home Here** — moves HOME marker to cursor
- **Modify Alt** — opens batch altitude edit dialog for all waypoints
- **Enter UTM Coord** — manual location entry for GPS-denied environments
- **Tracker Home** — sets Antenna Tracker HOME position
- **Measure Distance** — click start, right-click end → shows distance in meters

### 3.6 Polygon Tools (for Survey)

Polygon drawing tools (polygon icon upper-left of plan screen):
- **Draw a Polygon** — sequential left-click to place vertices; right-click to close
- **Clear Polygon**
- **Save Polygon** — save to file
- **Load Polygon** — load from file
- **From SHP** — load polygon from ESRI shapefile format
- **Fence Inclusion** — converts drawn polygon to a geofence inclusion zone (turns purple)
- **Fence Exclusion** — converts drawn polygon to exclusion zone

Polygon area displayed in status bar.

### 3.7 Auto WP / Survey Generation Tools

Right-click map → **Auto WP** submenu (requires polygon drawn first):
- **Create WP Circle** — adds waypoints in circular pattern around polygon
- **Create Spline WP Circle** — spline-interpolated circle
- **Create Circle Survey** — circular coverage survey
- **Survey (Grid)** — full lawnmower grid survey; opens configuration dialog:
  - *Grid Options:* line spacing, overshoot/lead-in distance, overlap %, cross-grid (double-pass) option, flight direction angle
  - *Camera Config:* camera make/model, sensor dimensions, focal length, image overlap percentage (typically 80% forward, 60% side)
  - *Altitude/Speed:* survey altitude, speed
  - Automatically inserts DO_SET_CAM_TRIGG_DIST commands
- **Survey (Gridv2)** — simplified rectangular survey interface (alternate UI)
- **Simple Grid** — basic grid without camera configuration
- **Face Map** — (in development)
- **Text** — (in development)

### 3.8 Map Tools

- **Measure Distance** — two-point distance measurement
- **Offline map cache** — download tiles for offline use (Prefetch button)
- **Altitude vs. terrain plot** — elevation profile along route

### 3.9 Geofence Mode (FENCES)

Switch mode selector from MISSION → FENCES.

**Fence types supported:**
- Circular inclusion zone (vehicle must stay inside)
- Circular exclusion zone (vehicle must stay outside)
- Polygon inclusion zone
- Polygon exclusion zone (Copter/Rover only)

**Plane-specific geofence right-click menu:**
- Upload to autopilot
- Download from autopilot
- Set Return Location
- Load from file
- Save to file
- Clear fence

**Copter/Rover:** Use polygon draw tool + "Fence Inclusion" / "Fence Exclusion" labels. Read/Write buttons for upload/download.

### 3.10 Rally Points Mode (RALLY)

Switch mode selector from MISSION → RALLY.

- Left-click on map → creates rally point at cursor; appears in mission list
- Rally points shown in list to the right
- Rally points serve as alternate land/loiter targets during failsafe

**Plane-specific rally right-click menu:**
- Set Rally Point at cursor
- Download from autopilot
- Upload to autopilot
- Clear Rally Points
- Save Rally to File
- Load Rally from File

**Copter/Rover:** Read/Write buttons for upload/download.

### 3.11 Bottom Status Bar (Flight Plan)

- Cursor lat/lon coordinates (live, updates as mouse moves)
- Terrain elevation at cursor (ASL)
- Map provider selector (Google, Bing, OpenStreetMap, custom)
- **Read** mission from autopilot button
- **Write** mission to autopilot button
- **Load File** button
- **Save File** button

---

## 4. Initial Setup Tab (SETUP)

### 4.1 Install Firmware

- Vehicle type selector grid (Copter, Plane, Rover, Boat, Sub, AntennaTracker, etc.)
- Frame type selector (for Copter: quad, hex, octo, Y6, X8, etc.)
- Firmware version: Stable / Beta / Developer (latest/master)
- Auto-detects connected board type
- Fetches firmware from `firmware.ardupilot.org`
- **Ctrl+C** on this screen: loads custom `.hex` file from local filesystem
- Available when **disconnected** only

### 4.2 Install Firmware Legacy

- Load older/specific firmware versions
- Available when **disconnected** only

### 4.3 Mandatory Hardware (requires connection)

#### Accel Calibration

Multi-position accelerometer wizard:
- Places vehicle in 6 orientations (level, nose down, nose up, left side, right side, back)
- Press key at each stable position
- "Calibrate Accel" button to start; "Simple Accel Cal" for level-only calibration

#### Compass Calibration

Onboard fitness calibration:
- "Start" button begins live calibration
- Rotate vehicle through all orientations (front/back/left/right/top/bottom facing ground)
- Green progress bars per compass extend as coverage improves
- Multiple compasses shown simultaneously (primary, secondary, tertiary)
- Cannot calibrate while armed
- "Fitness" score displayed
- Option to accept or reject each compass result
- Advanced settings: set offsets manually, set priority, enable/disable individual compasses

#### Radio Calibration

- "Calibrate Radio" green button starts calibration
- Move all sticks and switches to their full extents
- Green bars show min/max captured for each channel (CH1–CH8+)
- Channel mapping display (Roll, Pitch, Throttle, Yaw, etc.)
- "Click when done" finalization saves calibration

#### Servo Output

- Map autopilot servo output functions (Motor1, Motor2, Aileron, Elevator, etc.)
- Verify output assignments match physical wiring
- Test slider for each output channel (move to test position while disarmed)
- Reverse checkbox per channel

#### ESC Calibration

- Calibration method varies by platform:
  - Copter: all-at-once method (full throttle on power-up)
  - Plane: passthrough mode
  - Rover: different procedure
- Safety warnings shown

#### Flight Modes

- Six mode slots configurable (channels: RC channel 5 for Copter/Rover, channel 8 for Plane)
- PWM range displayed for each slot
- Mode dropdown per slot (all available modes for that vehicle type)
- Move RC switch → green highlight shows active slot
- Configurable via `FLTMODE_CH` / `MODE_CH` parameter
- Save button

#### Failsafe

- Throttle Failsafe: PWM threshold, action (RTL / Land / SmartRTL / etc.)
- Battery Failsafe: voltage/mAh thresholds, action
- GCS Failsafe: timeout for lost telemetry
- RC Failsafe: lost RC signal action
- Values and thresholds configurable per failsafe type

### 4.4 Optional Hardware

- **SiK Telemetry Radio** — local/remote radio programming (baud, net ID, TX power, duty cycle)
- **UAVCAN / DroneCAN** — CAN bus device management (SLCAN tool: view nodes, upload firmware, configure parameters, inspect CAN messages)
- **PX4 Optical Flow** sensor configuration
- **Antenna Tracker** — configure companion antenna tracker autopilot
- **Joystick** — axis mapping, deadzone, button assignment
- **Battery Monitors** — up to 9 monitors; voltage/current sensor type, divider values, capacity (mAh)
- **Integrated OSD** — layout editor for on-screen display (element position, enable/disable)
- **Airspeed Sensors** — sensor type, calibration, offset
- **Rangefinders** — sensor type, orientation, min/max distance
- **Motor Test** — spin individual motors at defined throttle % for direction verification

### 4.5 Advanced

- **Warning Manager** — create custom audio alerts based on MAVLink data comparisons
- **MAVLink Inspector** — real-time view of all incoming MAVLink messages; live graph selected values
- **Proximity** — 360° LIDAR visualization (requires proximity sensor)
- **MAVLink Signing** — configure secure MAVLink communication
- **MAVLink Mirror / Forwarding** — forward MAVLink stream via TCP/UDP to remote users; localhost forwarding for multiple GCS on same PC
- **NMEA Output** — GPS output at 2 Hz to serial port (also via Ctrl+G)
- **Follow Me** — sends Guided Mode waypoints based on GCS GPS position (NMEA GPS on PC COM port)
- **Parameter Regeneration** — regenerate default parameters for connected board
- **Moving Base** — shows PC's GPS position as moving on map
- **Anonymized Logging** — strips PII from logs
- **FFT Analysis** — IMU vibration frequency analysis for notch filter configuration

---

## 5. Config / Tuning Tab (CONFIG)

### 5.1 Planner Settings

Settings for the Mission Planner application itself (always accessible, even when disconnected):

- **Speech:** Enable/disable voice alerts; configure which events trigger speech (waypoint reached, low battery, mode change, etc.); Windows TTS voice selection
- **Log storage location** — set custom log directory path
- **Measurement units:**
  - Distance: meters / feet
  - Altitude: meters / feet
  - Speed: m/s / km/h / mph / knots
- **Video settings:** enable camera feed on HUD; set video source (capture card, USB device); overlay vs. replace HUD display
- **Telemetry rates** (requires connection): set MAVLink stream rates (Hz) for:
  - Attitude
  - Position
  - Mode/Status
  - RC channels
  - Sensors
- **Layout options:** Standard mode / Advanced mode / Custom mode (controls which menu items and tabs are visible)
- **Theme:** light / dark color schemes
- **Language** selector
- **Map provider** default selection
- **Console visibility** checkbox (show/hide startup console window)
- **Logging checkboxes** (what gets logged)

### 5.2 Flight Modes

Same as SETUP > Mandatory Hardware > Flight Modes (duplicate access point):
- 6-slot mode assignment
- Channel selector

### 5.3 GeoFence (Copter only in this panel)

- Enable/disable fence
- Fence type selector (altitude / circle / polygon)
- Max altitude
- Circle radius
- Fence action (RTL / Land / Smart RTL / Brake / Hold)
- Margin distance
- Note: fence coordinates set in PLAN view; other vehicles use Full Parameter List

### 5.4 Basic Tuning

Vehicle-specific simplified tuning sliders/inputs:
- Roll/Pitch rate P gain
- Yaw rate P gain
- Altitude Hold P gain
- Climb/Descent rate settings
- Content varies by vehicle type (Copter vs. Plane vs. Rover)

### 5.5 Extended Tuning (Copter only)

More detailed control loop tuning:
- Roll/Pitch Rate P, I, D gains
- Roll/Pitch Stabilize P gain
- Loiter PID (speed P, I; accel P, I, D)
- Throttle rate PID
- Throttle accel PID
- Climb rate / altitude P
- WP speed, radius, loiter speed, nav speed

### 5.6 Standard Params

Curated list of commonly-adjusted parameters with friendly names and descriptions (not the full raw list). Varies by vehicle type.

### 5.7 Advanced Params

Extended parameter set for advanced users — parameters not normally adjusted by average users.

### 5.8 OnBoard OSD

- Configure integrated OSD (for autopilots with onboard OSD hardware)
- Element placement on OSD grid
- Enable/disable individual OSD elements
- Multiple OSD screens configurable

### 5.9 MAVftp

- File system browser for autopilot SD card and internal flash (firmware 4.1+)
- Upload/download individual files
- Browse directory tree

### 5.10 User Params

- Streamlined access to Auxiliary Function assignments on RC channels
- Set which function each channel's switch triggers

### 5.11 Full Parameter List

The main power-user parameter screen:
- Tabular list of every parameter (name, value, description)
- **Search box** — filter by parameter name or description
- **Modified** filter — show only parameters changed from default
- **Load from File** — restore previously saved .param file
- **Save to File** — export all current parameters
- **Write Params** — commit screen changes to autopilot
- **Refresh Params** — reload from autopilot
- **Compare Params** — diff screen values against a saved .param file; shows differences; user selects which changes to apply
- **Load Presaved** dropdown — load community-submitted base parameter sets for specific hardware/frame combinations
- **Reset to Default** — factory reset all parameters (requires re-calibration)
- Double-click a value to edit it inline
- Color-coded rows for modified/unsaved values

### 5.12 Full Parameter Tree

- Tree-structured alternative view of all parameters
- Organized by subsystem (ATC, ARSPD, BATT, CAM, COMPASS, EK3, etc.)
- Same edit/save/load functionality as Full Parameter List

---

## 6. Simulation Tab (SIMULATION)

### 6.1 Vehicle Selection Grid

- Click vehicle icon to start simulation: Copter, Plane, Rover, Boat, Sub, AntennaTracker, Helicopter, Blimp
- Each vehicle icon corresponds to a SITL model

### 6.2 Frame / Model Dropdown

- Must select frame type BEFORE clicking vehicle
- Copter: quad, hexa, octo, y6, x8, tri, single, coax, dodecahexa, etc.
- Plane: standard, flying wing, quadplane, etc.
- Rover: rover, boat
- Firmware version prompt: Development (latest master) or Stable

### 6.3 Extra Command Line

Text input for SITL command line arguments:
- `--home={lat},{lon},{alt},{heading}` — set virtual start location
- `--defaults=<path>` — load custom parameter file (with Wipe checkbox)
- `--serialN=uart:COMX` — attach real serial device to simulated port

### 6.4 Simulation Behavior

- Runs a native SITL build of ArduPilot firmware
- Connects Mission Planner via simulated MAVLink as if a real vehicle
- All Flight Data, mission planning, parameter tuning features work normally
- SIM_x parameters viewable/editable in CONFIG > Full Parameter Tree
- Supports RealFlight integration (external flight dynamics)
- Uses same development code branch as Linux `sim_vehicle.py` SITL

---

## 7. Help Tab

- Version information display
- Update check
- Console visibility toggle
- Links to online documentation
- Bug report link
- Language translation settings
- About dialog

---

## 8. Top Toolbar / Connection Controls (Always Visible)

**Upper-right connection cluster:**
- **COM port dropdown** — lists all available COM ports + TCP + UDP options; "AUTO" for automatic detection
- **Baud rate dropdown** — common values: 9600, 57600, 115200; USB always uses 115200; radio typically 57600; TCP/UDP ignore this
- **CONNECT button** — on click: downloads parameters from autopilot, button changes to DISCONNECT
- **Right-click CONNECT button** → "Connection Options" for multi-vehicle simultaneous connections
- **Stats…** link below — shows link statistics and signing status

**Keyboard shortcuts:**
- `F2` — switch to Flight Data tab
- `F3` — switch to Flight Plan tab
- `F4` — switch to Configuration tab
- `F5` — refresh parameters
- `F12` or `Ctrl+T` — connect/disconnect toggle
- `Ctrl+G` — NMEA GPS output to serial port
- `Ctrl+W` — Wizard config interface (propagation settings)
- `Ctrl+P` — Plugin enable/disable manager
- `Ctrl+C` (on firmware screen) — load custom .hex file
- `Ctrl+F` — opens hidden "advanced tools" popup window (see below)

---

## 9. CTRL+F Hidden Tools Window

Accessed by pressing **Ctrl+F** at any screen. Contains advanced/developer tools with minimal documentation. Known buttons:

- **MAVLink Inspector** — real-time display of all incoming MAVLink messages by type and field; graph historical values
- **Warning Manager** — create custom audio alerts: select incoming data item → set comparison threshold → specify speech phrase to trigger
- **MAVLink Forwarding** — forward vehicle MAVLink stream via TCP/UDP; supports localhost for multiple GCS on same PC; limited to one destination
- **BootLoader Upgrade** — automatically update autopilot bootloader if outdated
- **Geo-ref Images** — geotag survey photos using flight log data; match image timestamps to GPS track; export geotagged images
- **Follow Me** — send Guided Mode waypoints from PC's GPS position (requires NMEA GPS on PC COM port)
- **Moving Base** — show PC's GPS location as moving marker on the map
- **Inject GPS** — RTK GPS correction injection
- **Map Logs** — overlay flight log data on map
- **DASHWARE** — export data for DashWare video overlay tool
- **Color-status Screen** — dedicated vehicle health monitoring interface

Many additional buttons exist with no public documentation.

---

## 10. Map Features (Both DATA and PLAN views)

### Map Providers

- Google Maps (satellite, hybrid, terrain)
- Bing Maps
- OpenStreetMap
- Custom tile server (configurable URL)
- Offline cached tiles (via Prefetch)

### Map Interaction

- Scroll wheel = zoom
- Left-click + drag = pan
- Double-click = center/zoom
- Vehicle icon follows GPS position in real time

### Map Overlays

- **Vehicle icon** — type-appropriate icon (quadrotor, plane, rover) with heading indicator
- **Track line** — historical path trail (purple); cleared with "Clear Track"
- **Home marker** — H icon at home/takeoff position
- **Waypoints** — numbered markers; click to select; drag to reposition (Plan view)
- **Mission route lines** — connects waypoints in sequence
- **Geofence** — red/purple polygon and circle overlays
- **Rally points** — R markers
- **ADSB traffic** — other aircraft from ADSB receiver or internet feed
- **AIS traffic** — marine vessel traffic (for boat/sub)
- **Proximity overlay** — 360° LIDAR distance returns visualized around vehicle
- **Camera overlap visualization** — footprint coverage overlay for survey missions
- **Altitude Angel airspace** — airspace restriction overlays (requires service configuration)
- **POI markers** — custom named points

---

## 11. Status Bar (Flight Data screen bottom)

- Connection type indicator (USB / Telemetry / UDP / TCP)
- Baud rate in use
- Link quality (packet success %)
- Armed / Disarmed state
- Battery voltage and percentage
- GPS lock status and satellite count
- Current flight mode
- Vehicle type indicator

---

## 12. Architecture Notes for Meridian Web GCS Reference

### Internal Component Map

| Mission Planner Component | Purpose |
|---|---|
| `MainV2.cs` | Top-level application form and lifecycle |
| `MAVLinkInterface.cs` | MAVLink protocol handler; all vehicle comms |
| `MAVState` | Per-vehicle state (sysid, compid, type, params, mission) |
| `CurrentState.cs` | Real-time telemetry field store (~200+ properties) |
| `FlightData.cs` | Flight Data tab logic |
| `FlightPlanner.cs` | Flight Plan tab logic |
| `HUD.cs` | Artificial horizon and instrument drawing |
| `GMapControl` | Embedded map widget (GMap.NET library) |
| `MAVLinkParamList` | Parameter dictionary with file I/O |
| `MAVFtp` | MAVLink FTP for file transfer |
| `ThemeManager.cs` | Light/dark theme application |
| `DisplayConfiguration.cs` | Standard/Advanced/Custom UI mode |

### Connection Types Supported

- USB / Serial (COM port)
- UDP (primary for SITL; responsive)
- TCP (network / HereLink / telemetry via internet)
- Bluetooth (via virtual COM port)
- SiK Telemetry Radio (serial bridge)
- Multi-vehicle: simultaneous connections via right-click → Connection Options

### Plugin System

- `Plugin/Plugin.cs` — extensibility via plugins
- Can add UI menu items, custom map layers, data processors, custom MAVLink handlers

### HTTP Server

- Built-in local HTTP server
- Exposes `hud.html` web interface (browser-accessible HUD with map)
- REST-style endpoints for external tool integration

---

## 13. Complete Summary: Meridian Parity Checklist

### Must-Have for Full Parity

**HUD instruments:**
- [ ] Artificial horizon (pitch + roll)
- [ ] Airspeed / groundspeed (toggle based on sensor availability)
- [ ] Altitude tape with rate-of-climb bar
- [ ] Heading tape
- [ ] Current flight mode label
- [ ] GPS status + satellite count + HDOP
- [ ] Battery voltage + percentage
- [ ] Armed/Disarmed state
- [ ] Distance to waypoint + WP number
- [ ] Telemetry link quality (%)
- [ ] EKF status
- [ ] VIBE status
- [ ] Crosstrack error + turn rate

**Flight Data panels:**
- [ ] Actions panel with all mode buttons and arm/disarm
- [ ] Quick panel (customizable large-text telemetry)
- [ ] Gauges (4 analog-style gauges)
- [ ] Status panel (all ~200 telemetry fields from CurrentState)
- [ ] Servo/Relay override panel
- [ ] Telemetry log playback with timeline scrubber
- [ ] DataFlash log download and graph
- [ ] Scripts panel
- [ ] Messages panel with severity levels

**Map:**
- [ ] Multiple tile providers
- [ ] Vehicle icon with heading
- [ ] Track trail
- [ ] Home marker
- [ ] Guided mode (fly-to-here)
- [ ] Set home on map
- [ ] Waypoints overlaid during flight
- [ ] Geofence overlay
- [ ] Real-time graph / tuning overlay

**Flight Plan:**
- [ ] Click-to-place waypoints
- [ ] Full command dropdown (all NAV/DO/CONDITION commands)
- [ ] Drag-to-reposition waypoints
- [ ] Read/write mission to/from autopilot
- [ ] Load/save mission files
- [ ] Geofence editor (polygon + circle)
- [ ] Rally point editor
- [ ] Survey/grid auto-generation
- [ ] Polygon draw tool
- [ ] Terrain verification (SRTM data)

**Setup/Config:**
- [ ] Compass calibration wizard
- [ ] Accelerometer calibration wizard
- [ ] Radio calibration with bar display
- [ ] Flight mode assignment (6 slots)
- [ ] Failsafe configuration
- [ ] Full parameter list (searchable, editable, save/load/compare)
- [ ] Basic and extended PID tuning panels
- [ ] Battery monitor configuration

---

*Sources consulted:*
- [Mission Planner Flight Data Screen](https://ardupilot.org/planner/docs/mission-planner-flight-data.html)
- [Mission Planner Flight PLAN](https://ardupilot.org/planner/docs/mission-planner-flight-plan.html)
- [Mission Planner Initial SETUP](https://ardupilot.org/planner/docs/mission-planner-initial-setup.html)
- [Mission Planner Configuration and Tuning](https://ardupilot.org/planner/docs/mission-planner-configuration-and-tuning.html)
- [Mission Planner Features/Screens](https://ardupilot.org/planner/docs/mission-planner-features.html)
- [Flight Data Screen Overview](https://ardupilot.org/planner/docs/mission-planner-ground-control-station.html)
- [Other Mission Planner Features](https://ardupilot.org/planner/docs/other-mission-planner-features.html)
- [MAVLink Mission Commands](https://ardupilot.org/planner/docs/common-mavlink-mission-command-messages-mav_cmd.html)
- [Planning a Mission with Waypoints](https://ardupilot.org/planner/docs/common-planning-a-mission-with-waypoints-and-events.html)
- [Downloading and Analyzing Data Logs](https://ardupilot.org/planner/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html)
- [Telemetry Logs](https://ardupilot.org/planner/docs/mission-planner-telemetry-logs.html)
- [Mission Planner Simulation](https://ardupilot.org/planner/docs/mission-planner-simulation.html)
- [Mission Planner Advanced Tools (CTRL+F)](https://ardupilot.org/planner/docs/common-mp-tools.html)
- [Connect Mission Planner to AutoPilot](https://ardupilot.org/planner/docs/common-connect-mission-planner-autopilot.html)
- [ArduPilot/MissionPlanner DeepWiki](https://deepwiki.com/ArduPilot/MissionPlanner)
- [CurrentState.cs source](https://github.com/ArduPilot/MissionPlanner/blob/master/ExtLibs/ArduPilot/CurrentState.cs)
- [hud.html web HUD](https://github.com/ArduPilot/MissionPlanner/blob/master/hud.html)
- [CTRL-F menu complexity GitHub issue](https://github.com/ArduPilot/MissionPlanner/issues/2027)
