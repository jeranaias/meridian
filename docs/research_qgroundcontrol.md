# QGroundControl (QGC) Feature Research
## Reference for Meridian Web GCS Parity

**Researched:** 2026-04-02  
**QGC Version Covered:** v4.3, v4.4, v5.0 (current stable), daily builds  
**Source:** Official QGC documentation at docs.qgroundcontrol.com, PX4 docs, ArduPilot docs

---

## 1. Main Views (Top-Level Navigation)

QGC presents four primary views, accessed via the "Q" icon in the top-left toolbar which opens a Select Tool popup.

### 1.1 Fly View
Real-time vehicle operation and monitoring. Default view when a vehicle is connected.

### 1.2 Plan Flight (Plan View)
Map-based mission design and waypoint/pattern planning.

### 1.3 Analyze Tools (Analyze View)
Post-flight log download, MAVLink inspection, GeoTagging.

### 1.4 Vehicle Configuration (Setup View)
Renamed from "Vehicle Setup" in v5.0. Firmware, sensors, radio, flight modes, safety, tuning.

### 1.5 Application Settings (Settings View)
Global QGC preferences: units, map, video, comm links, offline maps.

---

## 2. Fly View (Main Flying Screen)

### 2.1 Layout Overview
- **Toolbar** (top bar): View navigation, flight status, component status indicators
- **Vehicle Actions / Fly Tools** (left or center panel): ARM/DISARM, Takeoff, Land, RTL, Pause, Actions
- **Instrument Panel** (right side, bottom, or configurable): Telemetry widget
- **Attitude/Compass Widget**: Virtual horizon + heading
- **Camera Tools Widget**: Still/video capture, camera settings
- **Video Display** (lower left corner switcher): Toggle video/map foreground
- **Map** (main display): All connected vehicles + mission overlay
- **Altitude Slider** (right edge, visible when airborne): Vertical slider to adjust altitude

### 2.2 Map Display Elements
- Vehicle icon (position + heading arrow)
- Mission waypoints numbered on map with connecting flight-path lines and direction arrows
- Planned Home position (H marker), draggable in Plan view
- Mission path with direction-of-travel arrows
- ADSB traffic vehicles (smaller blue icons, with altitude and callsign label below)
- Multi-vehicle positions (active = opaque, inactive = semi-transparent)
- Map drag to pan, zoom via buttons / mouse wheel / trackpad / pinch
- Auto-recenter on vehicle after period of manual pan
- Click on map location → popup with contextual actions (Go to Location, Orbit at Location)

### 2.3 Fly View Toolbar (Top Bar) - Complete Indicator List

| Indicator | Description |
|-----------|-------------|
| Flight Readiness | "Not Ready" / "Ready to Fly" / "Armed" / "Flying" / "Landing" / "Communication Lost". Color: green (OK), amber (warning), red (critical). Dropdown → Arm / Disarm / Emergency Stop |
| Flight Mode | Current mode name. Dropdown to switch modes. Expanded page for land settings, geofence, mode customization |
| Vehicle Messages | Red indicator when important messages present. Dropdown shows full message list |
| GPS / RTK GPS | Satellite count + HDOP. Dual-mode: vehicle GPS or RTK base. RTK settings access |
| GPS Resilience | Authentication / spoofing / jamming state indicators per-GPS |
| Battery | Configurable colored icon. Shows percentage and/or voltage. Low battery failsafe config access. v5.0: dynamic bars with configurable thresholds (100%, Config 1, Config 2, Low, Critical) |
| Remote ID | Health status color. Status dropdown |
| ESC | Overall health indicator. Online motor count. Detailed status page |
| Joystick | Device detection and connection status |
| Telemetry RSSI | Local/remote signal strength, radio link quality |
| RC RSSI | RC link strength, detailed info page |
| Gimbal | Active gimbal status, controls and settings access |
| VTOL Transitions | VTOL mode/state indicator, transition status |
| Multi-Vehicle Selector | Quick vehicle switching |
| APM Support Forwarding | MAVLink forwarding status (ArduPilot) |
| Expand button (">") | Access detailed app and vehicle settings without leaving Fly View |

### 2.4 Fly Tools / Action Panel

All actions that modify vehicle state require a **confirmation slider** before executing.

| Action | Condition / Notes |
|--------|-------------------|
| **Pre-Flight Checklist** | Enabled via Settings. Can be made mandatory (enforce arming pre-condition) |
| **Arm** | Via flight readiness indicator dropdown |
| **Disarm** | Via flight readiness indicator dropdown |
| **Emergency Stop** | Same as disarm while flying — cuts motors immediately |
| **Takeoff** | Appears when landed. Shows altitude slider for target altitude. Toggles to Land after takeoff |
| **Land** | Land at current position. Confirmation slider |
| **RTL / Return** | Return to home or nearest rally point. Confirmation slider |
| **Pause** | Suspends operations: takeoff, landing, RTL, missions, orbit. Multirotors hover; fixed-wing circles. Cannot pause Goto operation |
| **Start Mission** | Visible when landed with a mission loaded |
| **Continue Mission** | Resume from next waypoint while airborne |
| **Resume Mission** | After RTL or landing mid-mission (battery swap). QGC rebuilds mission prepending needed commands |
| **Go To Location** | Click map → popup → "Go to Location". Max distance configurable in settings (default 1 km) |
| **Orbit at Location** | Click map → popup → "Orbit at Location". Adjustable radius |
| **Change Altitude** | Accessible via Actions button during flight. Vertical slider. Not available during missions |
| **Actions button** | Context-sensitive button showing other available actions based on current state |

Post-landing: QGC automatically prompts to remove mission to prevent stale mission re-execution.

### 2.5 Instrument Panel (Telemetry Widget)
Configurable widget; right-click (desktop) or press-and-hold (tablet) to enter edit mode.

**Default displayed values:**
- Altitude (relative to home)
- Horizontal speed
- Vertical speed
- Total flight time
- Distance between vehicle and ground station

**Customization:**
- Add/remove rows and columns via pencil/edit icon → "+" / "-" controls
- Per-value editor: change icon, text label, size, units
- Source selector (top-left): vehicle or specific sensor type
- Value selector (top-right): choose from all available telemetry

Configurable position: bottom-center or right-center (vertical panel mode via Settings).

### 2.6 Attitude / Compass Widget
- Virtual horizon (artificial horizon / attitude indicator)
- Heading compass rose
- Multiple instrument type variants selectable (right-click / long-press)
- Additional heading indicators configurable via Settings: Course-over-ground, Home direction, Waypoint direction
- Lock Compass Nose-Up option (keeps north fixed vs. heading-up)

### 2.7 Camera Tools Widget
- Toggle between Still Photo and Video modes
- Start/Stop capture button
- Configuration panel for simple autopilot-connected cameras
- Video Stream Page: enable/disable video streaming, grid overlay, image fit mode, local recording toggle
- Record video stream to GCS (red circle button, saves as Matroska .mkv)
- Record on camera (MAVLink-capable devices)

### 2.8 Video Display
- Corner switcher window (lower-left by default)
- Toggle video foreground / map foreground
- Resize the video window
- Hide/show toggle
- Detach video into separate window
- Video source: RTSP / UDP h.264 / UDP h.265 / TCP-MPEG2 / MPEG-TS / Integrated Camera

### 2.9 Altitude Slider
- Visible on right edge during flight
- Drag to set new target altitude
- Also appears during Pause with visible height context

### 2.10 Guided Mode (Click-to-Fly)
- Left-click on map anywhere → popup → "Go to Location" or "Orbit at Location"
- Confirmation step before command is sent
- Go To Location: guided waypoint navigation
- Orbit: vehicle circles selected point; adjustable orbit radius

### 2.11 Multi-Vehicle Support
- All vehicles displayed on map; active = opaque, inactive = semi-transparent
- Vehicle ID number shown below each vehicle icon
- Multi-vehicle selector in toolbar for quick switching
- Batch commands available from multi-vehicle list: Pause, Start Mission (applied to all)
- Configurable per-vehicle telemetry display

### 2.12 ADSB Traffic Display
- Connect to ADSB SBS server (e.g. dump1090 on TCP 30003)
- Detected aircraft shown on Fly View map as smaller blue icons
- Display: altitude and callsign below each icon
- Configured in Application Settings → ADSB Server

### 2.13 Follow Me Mode
- GCS streams its GPS position to vehicle
- Vehicle follows ground station location
- Stream GCS Position setting: Never / Always / When in Follow Me Flight Mode
- Only supported on PX4 Multicopters

### 2.14 Pre-Flight Checklist
- Automated standard pre-flight checks
- Enable in Settings → General → Fly View → "Use Preflight Checklist"
- Optionally enforce as arming pre-condition: "Enforce Preflight Checklist"
- Preflight check failures displayed with expandable detail + suggested solutions
- Available in 3D View as well as standard Fly View

### 2.15 3D View (v5.0+)
- Import and display 3D map from OpenStreetMap (.osm) file
- Mouse: left-click drag = move, right-click drag = rotate, scroll = zoom
- Touch: single-finger drag = move, two-finger rotate, pinch zoom
- Full flight controls available: arm, takeoff, land, RTL, mission start/pause/resume
- Switch between 3D map view and video view
- Settings: 3D Map File path, Average Building Level Height, Vehicle Altitude Bias

---

## 3. Plan View

### 3.1 Layout Overview
- **Map** (center): Numbered waypoint markers, home position (H), flight path lines with arrows
- **Plan Toolbar** (top): Open, Save, Upload, Clear buttons + hamburger menu
- **Plan Tools** (left vertical strip): Tool buttons for adding items
- **Plan Editor Panel** (right, collapsible): Tree view with all mission sections
- **Layer Switcher**: Toggle between Mission / Geo-Fence / Rally Points editing modes
- **Stats / Terrain Panel** (bottom): Toggle between terrain profile chart and mission statistics

### 3.2 Plan Toolbar (Top)
- **Open**: Load plan from file (.plan format)
- **Save**: Save plan to file
- **Save as KML**: Export mission as KML for Google Earth
- **Upload**: Upload plan to connected vehicle (highlighted when unsaved/un-uploaded changes exist)
- **Clear**: Remove all mission items
- **Download**: Download current plan from vehicle (hamburger menu)

### 3.3 Plan Tools (Left Strip)
| Tool | Function |
|------|----------|
| **Takeoff** | Add a takeoff command as first mission item |
| **Waypoint** | Toggle waypoint-on-click mode; clicking map adds waypoints. Stays active until deselected. Drag waypoints to reposition |
| **Pattern** | Opens pattern type selector (see §3.5) |
| **ROI** | Toggle ROI-on-click mode. Click sets Region of Interest for gimbal pointing. Only shown if firmware supports ROI |
| **Return / Land** | Add RTL or Land command at end of mission |
| **Stats Toggle** | Show/hide mission statistics / terrain profile at bottom |

Map navigation tools (do not create mission items): Center Map, Zoom In, Zoom Out.

### 3.4 Plan Editor Panel (Right Side)
Tree sections:
- **Plan Info**: Plan file name, vehicle type (Firmware/MAV_TYPE selectors when disconnected)
- **Defaults**: Altitude Frame, Waypoints Altitude, Flight Speed, Vehicle Speeds (cruise/hover for time estimates), Mission End (Return to Launch checkbox)
- **Mission Items**: Ordered list. Each item expandable. Hamburger per item: Show all values, Move to vehicle position, Move to previous position, Edit position, Delete
- **GeoFence**: Polygon and circular fence management (see §3.7)
- **Rally Points**: Rally point management (see §3.8)
- **Transform**: Offset Mission (East/North/Up), Reposition Mission (lat/lon/UTM/MGRS or vehicle position), Rotate Mission (degrees CW)

**Mission Item Editor per waypoint:**
- Command type dropdown (filterable by category: Basic Commands, All commands, etc.)
- All MAV_CMD parameters exposed with labels
- Altitude value + Altitude Mode selector
- "?" status indicator for incomplete values
- Initial Camera Settings section (pre-mission camera actions, gimbal pitch)

### 3.5 Pattern Types
All patterns generate as ComplexItems in the .plan file, expanding to individual SimpleItems on upload.

#### Survey (Grid Area Scan)
**Camera options:**
- Known Camera: select from preset list → auto-generates grid based on camera specs
  - Landscape/Portrait orientation
  - Overlap: frontal (along-track) and side (cross-track) percentage
  - Choose between Altitude (shows calculated GSD) or Ground Resolution (calculates altitude)
- Custom Camera: manually enter sensor dimensions, image resolution, focal length
- Manual: survey altitude + trigger distance + grid line spacing

**Grid / Transect settings:**
- Grid angle (relative to North)
- Turnaround distance (extra distance outside survey area for vehicle turns)
- Entry point rotation (button to swap start corner)
- Hover and Capture (multicopter: stop and capture at each trigger point)
- Refly 90 degrees (second pass at 90° to first for double coverage)
- Image capture during turnarounds toggle
- Relative vs. AMSL altitude reference

**Terrain Following:**
- Enable/disable
- Tolerance (accepted altitude deviation)
- Max Climb Rate
- Max Descent Rate

**Statistics output:** Survey area, photo interval, photo spacing, planned photo count

#### Corridor Scan
- Draw polyline path on map; click (+) on line segments to add vertices
- **Width**: corridor width around polyline
- All camera settings same as Survey (Known/Custom/Manual)
- Turnaround distance
- Take images in turnarounds toggle
- Relative altitude toggle
- Rotate entry point button
- Terrain Following (same options as Survey)
- Statistics: area, photo interval, photo spacing, photo count

#### Structure Scan
- Draw polygon or circle on map for structure footprint
- **Scan direction**: top-to-bottom or bottom-to-top
- **Structure Height**: vertical measurement of structure
- **Scan Distance**: distance from structure to flight path
- **Entrance/Exit Alt**: altitude to avoid obstacles between waypoints and structure
- **Scan Bottom Alt**: altitude at base of structure to avoid ground obstacles
- **Rotate Entry Point**: move start point to next vertex
- **Camera Mode**:
  - Manual: layer height, trigger distance, gimbal pitch
  - Predefined Camera: orientation, front lap %, side lap %, ground resolution
  - Custom Camera: manual specs with auto-calculated values
- Circular structures: configurable radius and center position

#### Fixed Wing Landing Pattern
Generates three mission items: `DO_LAND_START`, `NAV_LOITER_TO_ALT`, `NAV_LAND`
- **Loiter Point settings**: altitude, radius, clockwise/anti-clockwise direction
- **Landing Point settings**: heading (from loiter to land), altitude (nominally 0), landing distance OR glide slope (radio button selection)
- **Altitudes relative to home** checkbox (default: AMSL)
- In v5.0: multiple landing patterns can be added for different locations

#### Pattern Presets
Save and reuse custom pattern configurations.

### 3.6 Mission Statistics (Bottom Panel)
**Per-waypoint info** (relative to previous waypoint):
- Altitude difference
- Azimuth
- Distance
- Gradient
- Heading

**Total mission stats:**
- Horizontal distance
- Estimated flight time
- Max telemetry distance (from planned home to furthest waypoint)
- Batteries required + change points

**Terrain Profile Chart:**
- Height AMSL vs. distance from start across full mission
- Visualizes altitude changes relative to terrain

### 3.7 Altitude Modes (Plan Defaults)
- **Relative** (to home): MAV_FRAME_GLOBAL_RELATIVE_ALT
- **AMSL** (above mean sea level): MAV_FRAME_GLOBAL
- **Terrain** (above terrain, requires terrain data): MAV_FRAME_GLOBAL_TERRAIN_ALT

### 3.8 GeoFence Editing
Accessed via Layer Switcher → Geo-Fence tab.
- **Add Circular Fence**: Creates a circle. Drag center dot to move; drag edge dot to resize; or edit radius value in panel
- **Add Polygon Fence**: Creates a polygon. Drag filled vertices; click unfilled (midpoint) dots to add new vertices
- **Inclusion / Exclusion**: Each fence has an "Inclusion" checkbox. Unchecked = exclusion zone (no-fly area)
- Multiple fences allowed (complex compound geofences)
- Select a fence via its Edit radio button
- Delete fence via "Del" button
- Breach return point with altitude
- GeoFences upload via same file/upload mechanism as missions
- ArduPilot: requires Rover 3.6 / Copter 3.7+

### 3.9 Rally Points Editing
Accessed via Layer Switcher → Rally Points tab.
- Click map to add rally point (shown as "R" marker)
- Active marker shown in green
- Drag to reposition, or edit coordinates in panel
- Delete via panel menu
- Upload same as missions via File tool
- ArduPilot: Rover 3.6 / Copter 3.7+; PX4: planned for v1.10+

### 3.10 .plan File Format
JSON file with sections: `mission`, `geoFence`, `rallyPoints`.  
Mission items are either SimpleItem (single MAV_CMD) or ComplexItem (Survey, CorridorScan, StructureScan with camera calc data).  
Fields include: firmwareType, vehicleType, globalPlanAltitudeMode, cruiseSpeed, hoverSpeed, plannedHomePosition.

---

## 4. Mission Command Types (MAV_CMD)

Commands visible in QGC depend on firmware and vehicle type. Shown via "Select Mission Command" dialog with category filter (Basic / All).

### Navigation Commands (NAV)
| Command | Description |
|---------|-------------|
| MAV_CMD_NAV_WAYPOINT | Navigate to 3D position |
| MAV_CMD_NAV_TAKEOFF | Climb to altitude before next waypoint |
| MAV_CMD_NAV_VTOL_TAKEOFF | Vertical takeoff (QuadPlane) |
| MAV_CMD_NAV_LOITER_UNLIM | Circle indefinitely |
| MAV_CMD_NAV_LOITER_TURNS | Circle for N turns |
| MAV_CMD_NAV_LOITER_TIME | Circle for N seconds |
| MAV_CMD_NAV_LOITER_TO_ALT | Circle while climbing/descending to altitude |
| MAV_CMD_NAV_RETURN_TO_LAUNCH | Return to home / nearest rally point |
| MAV_CMD_NAV_LAND | Land at location |
| MAV_CMD_NAV_VTOL_LAND | VTOL land (QuadPlane) |
| MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT | Continue course while changing altitude |
| MAV_CMD_NAV_ALTITUDE_WAIT | Wait for altitude or downward speed threshold |
| MAV_CMD_NAV_DELAY | Delay before next command |
| MAV_CMD_NAV_PAYLOAD_PLACE | Descend to place payload |

### Conditional Commands
| Command | Description |
|---------|-------------|
| MAV_CMD_CONDITION_DELAY | Delay execution of next conditional DO |
| MAV_CMD_CONDITION_DISTANCE | Delay DO until within distance of next WP |

### DO Commands (Auxiliary)
| Command | Description |
|---------|-------------|
| MAV_CMD_DO_JUMP | Jump to mission item (loop counter) |
| MAV_CMD_JUMP_TAG | Location marker for jump target |
| MAV_CMD_DO_JUMP_TAG | Jump to tag marker |
| MAV_CMD_DO_CHANGE_SPEED | Change airspeed / groundspeed |
| MAV_CMD_DO_SET_HOME | Set home position |
| MAV_CMD_DO_SET_RELAY | Set relay pin high/low |
| MAV_CMD_DO_REPEAT_RELAY | Toggle relay N times |
| MAV_CMD_DO_SET_SERVO | Set servo PWM |
| MAV_CMD_DO_REPEAT_SERVO | Cycle servo |
| MAV_CMD_DO_LAND_START | Marker for landing sequence |
| MAV_CMD_DO_VTOL_TRANSITION | Switch VTOL ↔ fixed-wing |
| MAV_CMD_DO_SET_ROI | Point gimbal at ROI |
| MAV_CMD_DO_SET_ROI_LOCATION | Point gimbal at fixed location |
| MAV_CMD_DO_SET_ROI_NONE | Clear ROI targeting |
| MAV_CMD_DO_DIGICAM_CONFIGURE | Configure camera controller |
| MAV_CMD_DO_DIGICAM_CONTROL | Trigger camera shutter (param5=1) |
| MAV_CMD_DO_MOUNT_CONTROL | Control gimbal roll/pitch/yaw |
| MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW | Move gimbal to pitch+yaw |
| MAV_CMD_DO_SET_CAM_TRIGG_DIST | Trigger camera at distance intervals |
| MAV_CMD_DO_FENCE_ENABLE | Enable/disable geofence |
| MAV_CMD_DO_AUX_FUNCTION | Control aux function (like RC switch) |
| MAV_CMD_DO_INVERTED_FLIGHT | Normal/inverted flight toggle |
| MAV_CMD_DO_AUTOTUNE_ENABLE | Enable/disable AUTOTUNE |
| MAV_CMD_DO_ENGINE_CONTROL | Start/stop ICE |
| MAV_CMD_DO_SET_RESUME_REPEAT_DIST | Set rewind distance on resume |
| MAV_CMD_STORAGE_FORMAT | Format SD card |

In QGC the camera capture shortcut commands shown in Mission Defaults:
- Take photos (time-based)
- Take photos (distance-based)
- Stop taking photos
- Start recording video
- Stop recording video

---

## 5. Vehicle Setup View

### 5.1 Summary Page
Overview of all setup categories. Each block shows a red indicator when not configured. Categories shown:
- Firmware, Airframe, Radio, Sensors, Flight Modes, Power, Motors, Safety (and others depending on firmware)

### 5.2 Firmware
- Flash PX4 or ArduPilot firmware to Pixhawk-family flight controllers
- Select: PX4 Flight Stack (stable), ArduPilot Flight Stack (select vehicle type + variant), SiK Radio, PX4 Flow
- Advanced settings: Beta builds, Daily builds, Custom firmware file (local .px4/.apj)
- Progress bar during flash; auto-reboot on completion
- Desktop only (not available on tablet/phone builds)
- Requires direct USB (no hubs, no battery power during flash)

### 5.3 Airframe
**PX4:** Select broad vehicle group → dropdown for specific airframe (e.g., "Hexarotor X" → "Generic Hexarotor X").  
**ArduCopter / ArduSub:** Frame Class (broad type) → Frame Type (specific geometry).  
Not shown for ArduPilot Rover or Plane (frame type is defined by parameters instead).

Vehicle groups include: Quadrotor +/X/Wide, Hexarotor +/X/Coaxial, Octorotor, Tricopter, Fixed Wing variants, VTOL (Tailsitter, Tiltrotor, Standard), Airship, Helicopter, Rover, Boat, Submarine.

### 5.4 Sensors (Calibration)
Each sensor shows status indicator. Calibrate button + on-screen wizard.

| Sensor | Calibration Process |
|--------|---------------------|
| **Compass** | Place vehicle in multiple orientations (shown as 3D diagram). Rotate until all orientation images turn green |
| **Gyroscope** | Place on flat surface, don't move, click OK. Auto-restarts if vehicle moved |
| **Accelerometer** | Multiple set positions (6 orientations). Hold still at each position |
| **Level Horizon** | Place at level flight orientation. Click OK |
| **Airspeed** | Cover sensor → Click OK → Blow through sensor → Remove covering |

**FC Orientation:** Set YAW / PITCH / ROLL offsets (degrees) via "Set Orientations" button or within each sensor calibration screen.

### 5.5 Radio Setup
- Turn on RC transmitter first
- Select transmitter mode (Mode 1/2/3/4, affects stick diagram displayed)
- Move sticks through all positions as prompted; click Next at each step
- Move all other switches and dials (observed on Channel Monitor)
- Save

Additional options:
- **Spektrum Bind**: Select receiver type → press OK → power transmitter holding bind button
- **Copy Trims** (PX4): Center sticks / throttle down → OK → reset transmitter trims to zero
- **AUX Passthrough Channels** (PX4): Map up to 2 transmitter controls to AUX1/AUX2
- **Parameter Tuning Channels** (PX4): Map up to 3 transmitter dials to parameter tuning channels

### 5.6 Flight Modes
- **Mode Channel** selector (typically Ch 5 or Ch 6)
- Up to **6 flight modes** assignable to switch positions based on channel PWM ranges
- Visual Channel Monitor: real-time display confirming switch → channel → mode mapping
- Mode text turns yellow in QGC when switch activates that mode
- Additional action channels: Kill Switch, Landing Gear, others
- All values auto-save on change
- PX4 recommended modes: Position, Return, Mission
- v5.0: Configurable list to hide unused modes

### 5.7 Power Setup
**Battery Configuration:**
- Number of cells
- Full voltage per cell
- Empty voltage per cell
- Voltage Divider (measured via multimeter → Calculate button → enter measured voltage → generates value)
- Amps Per Volt (measured current → Calculate button → enter measured amps → generates value)

**Advanced Settings** (Show Advanced Settings checkbox):
- Voltage Drop on Full Load (per-cell difference idle vs. full throttle)
- Warning: if too high → battery may be deep-discharged

**ESC PWM Calibration:**
- Remove propellers first (mandatory safety warning)
- Connect via USB only (no battery)
- Click Calibrate button
- Note: some ESCs may spin at max speed if sequence not detected

**UAVCAN Settings** (Show UAVCAN Settings checkbox):
- UAVCAN Bus Configuration
- Motor index assignment
- Motor direction assignment

### 5.8 Motors
- Remove all propellers (safety requirement, prominent warning)
- PX4: Enable safety switch if present
- Enable slider: "Propellers are removed - Enable slider and motors"
- Power slider: select motor power level
- Individual motor test buttons (numbered by motor index)
- Motors auto-stop after 3 seconds; manual Stop button
- Verify direction of rotation against frame diagram
- If motor doesn't spin: increase throttle percentage

### 5.9 Safety
**ArduPilot Safety Settings:**
- Battery failsafe: Low action (None/Land/RTL/SmartRTL/SmartRTL or Land/Terminate), voltage threshold, critical voltage threshold, mAh thresholds
- GCS Failsafe: Disabled / Always RTL / Continue Mission / Always SmartRTL or RTL / Always SmartRTL or Land
- Throttle Failsafe: Disabled / Always RTL / Continue in Auto / Always Land. PWM threshold
- RTL: return at current altitude or specified altitude, loiter time, descent speed, final altitude
- Geofence (cylindrical): circle enable, altitude enable, action (report only / RTL+Land), max radius, max altitude
- Arming Checks bitmask: Barometer, Compass, GPS, INS, Parameters, RC Channels, Board voltage, Battery, Airspeed, Logging, Safety switch, GPS Config, System

**PX4 Safety Settings (via QGC Safety page):**
- Low battery: Warn Level %, Failsafe Level %, Emergency Level %, action per level (Warn/Return/Land)
- Flight time failsafe: max flight duration, safe return estimate
- RC Loss: timeout, reaction delay, action (Disabled/Loiter/Return/Land/Disarm/Terminate), mode exceptions
- Data Link Loss: timeout, action (Hold/Return/Land/Disarm/Terminate), mode exceptions
- Geofence: max horizontal distance, max altitude, action (Warning/Hold/Return/Terminate/Land), predictive mode
- Position estimation loss: dead-reckoning timeout, horizontal error threshold
- High wind: warning threshold, max threshold, action
- Offboard loss: timeout, action
- Traffic avoidance: action (Disabled/Warn/Return/Land)
- VTOL Quad-Chute: action, max height, altitude/transition loss thresholds, roll/pitch thresholds
- Failure Detector: attitude trigger (max pitch/roll, time thresholds), motor failure detection
- Emergency switches: Kill switch (5-second reversible), Arm/Disarm switch, Return switch
- Auto-disarm: post-landing timeout, slow takeoff timeout
- Arming pre-conditions: min battery %, GPS requirement, mission requirement, SD card requirement

### 5.10 Tuning

**PX4 Tuning:**
- Autotune: toggle enable → Autotune action button; 19–68 second duration; multicopter = land to apply
- Rate Controller tab (Roll/Pitch/Yaw axis sliders)
- Attitude Controller tab
- Velocity Controller tab
- Position Controller tab
- Real-time tracking chart (Save to Clipboard / Restore from Clipboard / Clear chart buttons)

**ArduCopter Tuning:**
- Basic tuning: sliders for flight characteristics (left = more stable, right = more agile)
- AutoTune: select axes to tune, assign to transmitter switch, fly in AltHold, land while switch engaged to save
- In-flight tuning: one transmitter dial channel → adjusts specific parameter with min/max range

### 5.11 Camera (Vehicle Setup)
- ArduPilot-specific camera configuration
- Camera trigger mode and settings
- Gimbal configuration
- Note: not available on FMUv2-based flight controllers by default

### 5.12 Joystick (Vehicle Setup)
- Activate joystick by selecting Gear icon → Joystick
- Select Active Joystick from dropdown
- Calibration Tab: Start button → follow instructions to move sticks
- General Tab: Axis/Button monitor for testing and button assignment
  - Assign flight modes or vehicle functions to each button
- Advanced Tab options:
  - Center stick is zero throttle (+ spring-loaded throttle smoothing sub-option)
  - Full down stick is zero throttle
  - Allow negative thrust (Rovers)
  - Expo slider (reduces center sensitivity)
  - Gimbal Control: enable 2 gimbal channels
  - Joystick Mode: Normal / Attitude / Position / Force / Velocity
  - Axis Frequency: 5 Hz idle, configurable active (default 25 Hz)
  - Button Frequency: repeated button action rate
  - Circle Correction: square vs. circle response
  - Deadbands: per-axis neutral zone filtering
- Tested hardware: PS3/PS4 DualShock 4, FrSky Taranis XD9+, TBS Tango 2, Logitech F310/F710/Extreme 3D Pro

### 5.13 Parameters Page
- Complete parameter database for connected vehicle (PX4 or ArduPilot)
- **Group navigation** (left side): clickable group buttons to filter by category
- **Search field**: filter by parameter name or description substring; Clear button to reset
- **Show modified only** checkbox: filter out unchanged parameters
- Click parameter row → side dialog to edit value with full description, units, valid range, reboot-required flag
- Auto-save to vehicle on change
- **Tools Menu** (top right): Refresh (re-request all), Reset all to defaults, Load from file, Save to file, Clear RC to Param, Reboot Vehicle

---

## 6. Analyze View

Accessed via Q icon → Analyze Tools.

### 6.1 Log Download
- List log files stored on vehicle (Refresh button)
- Download selected log files to ground station
- Erase All (delete all logs from vehicle)

### 6.2 MAVLink Inspector
- Real-time list of all MAVLink messages received from vehicle
- Per-message: source component ID, update rate/frequency
- Drill down into a message: message ID, component ID, all field values (live-updating)
- Chart fields: checkbox adjacent to field to add to chart
- Dual chart system: two separate charts available
- Multi-field plotting: multiple fields from multiple messages on one chart
- Chart customization: scale and range adjustment
- Messages with active chart fields marked with asterisks
- Desktop only (Windows, Linux, macOS)

### 6.3 GeoTag Images (PX4 only)
- Geotag survey mission photos using flight log
- Performed on computer after flight
- Inputs: image folder, log file
- Writes GPS EXIF tags to image files

### 6.4 MAVLink Console (PX4 only)
- Connect to PX4 nsh (Network Shell) on vehicle
- Send commands and see output
- Access PX4 System Console
- PX4 SITL and ArduPilot not supported; hardware only

---

## 7. Application Settings View

### 7.1 General Settings
**Units:**
- Distance: Meters | Feet
- Area: SquareMetres | SquareFeet | SquareKilometers | Hectares | Acres | SquareMiles
- Speed: Metres/second | Feet/second | Miles/hour | Kilometres/hour | Knots
- Temperature: Celsius | Fahrenheit

**Miscellaneous:**
- Language: System default + Bulgarian, Chinese, and others
- Color Scheme: Indoor (Dark) | Outdoor (Light)
- Map Provider: Google | Mapbox | Bing | Airmap | VWorld | Eniro | Statkart
- Map Type: Road | Hybrid | Satellite
- Stream GCS Position: Never | Always | When in Follow Me Flight Mode
- UI Scaling: percentage for fonts, icons, buttons
- Mute all audio output
- Check for Internet Connection (for map tile downloads)
- Autoload Missions (auto-upload plan on vehicle connection)
- Clear all settings on next start
- Announce battery lower than: threshold for low battery voice announcement
- Application Load/Save Path: default file storage location
- Disable all data persistence: prevents log saving and data caching

**Telemetry Logs:**
- Save log after each flight
- Save logs even if vehicle was not armed
- CSV Logging: telemetry subset export

**Fly View Settings:**
- Use Preflight Checklist
- Enforce Preflight Checklist
- Keep Map Centered on Vehicle (follow mode lock)
- Show Telemetry Log Replay Status Bar
- Virtual Joystick (PX4 only)
- Use Vertical Instrument Panel
- Show additional heading indicators on Compass (Course, Home, Waypoint)
- Lock Compass Nose-Up
- Guided Minimum Altitude (bottom of altitude slider)
- Guided Maximum Altitude (top of altitude slider)
- Go To Location Max Distance (max guided command range)

**Plan View Settings:**
- Default Mission Altitude

**AutoConnect Devices:**
- Pixhawk, SiK Radio, PX4 Flow, LibrePilot, UDP, RTK GPS, NMEA GPS Device (toggles)

**NMEA GPS Configuration:**
- NMEA GPS Device: Serial or UDP
- Baudrate
- UDP Port

**RTK GPS Settings:**
- Survey-in accuracy (U-blox only)
- Minimum observation duration
- Base Position: Latitude, Longitude, Alt (WGS84), Accuracy
- Save Current Base Position

**ADSB Server:**
- Connect to ADSB SBS server toggle
- Host address + Server port

**Brand Image:**
- Indoor Image (dark scheme logo)
- Outdoor Image (light scheme logo)
- Reset Default Brand Image

### 7.2 Video Settings
- Video Source: Disabled | RTSP | UDP h.264 | UDP h.265 | TCP-MPEG2 | MPEG-TS | Integrated Camera
- URL / Port for stream
- Aspect Ratio
- Disabled When Disarmed (cuts feed when motors off)
- Low Latency Mode (reduce delay vs. frame quality tradeoff)

**Video Recording:**
- Auto-Delete Files
- Max Storage Usage (GB limit)
- Video File Format: mkv | mov | mp4

### 7.3 Comm Links
- Manually create communication links (normally not needed; most connections auto-detected)
- Link types: Serial (select port, baud rate), UDP (port), TCP (host, port)
- Name each link, mark as Automatically Connect on startup
- Connect/Disconnect per link
- Edit and Delete links

### 7.4 Offline Maps
- Create named tile cache sets for offline use (no internet required)
- "Add new set": name, min zoom, max zoom
- Move map to area of interest, set zoom range, click Download
- Previews of min/max zoom levels shown during setup
- Multiple offline sets (different regions)
- Tiles cached from selected map provider

### 7.5 MAVLink Settings
- Ground Station MAVLink System ID (default 255)
- Heartbeat emission toggle (default: enabled)
- Protocol version filter (MAVLink 1 or MAVLink 2 only)
- MAVLink Message Forwarding: enable + UDP endpoint for one-way forwarding
- Link Status Monitor: message transfer stats, loss rate percentage
- MAVLink 2 Logging (PX4 only): real-time log streaming to QGC + upload to Flight Review
  - Email address, flight description, upload URL, video URL, wind speed, flight rating, public visibility, auto-upload, auto-delete after upload
- v5.0: MAVLink 2 signing support, configurable ArduPilot stream rates

### 7.6 Console Logging
- Capture internal application log for diagnostics
- Filter by log category

### 7.7 Virtual Joystick (PX4)
- Enable via Settings → General → "Virtual joystick" checkbox
- On-screen thumbsticks displayed in Fly View when enabled
- Less responsive than RC transmitter (MAVLink latency)
- v5.0: Left-handed mode support

---

## 8. Map Controls

### 8.1 Navigation
- **Drag**: pan map
- **Scroll wheel / trackpad**: zoom
- **Pinch** (tablet/touch): zoom
- **Zoom buttons**: on-screen +/- buttons
- **Center on Vehicle**: button to re-center map on active vehicle
- **Follow Mode**: "Keep Map Centered on Vehicle" in Settings → auto-follows vehicle; temporarily disabled when user pans (re-engages after timeout)

### 8.2 Map Provider / Type
Selected in Application Settings → General:
- **Providers**: Google, Mapbox, Bing, Airmap, VWorld, Eniro, Statkart
- **Types**: Road, Hybrid, Satellite
- API keys required for some providers (Mapbox, Google, Bing)
- Mapbox supports custom styles

### 8.3 Right-Click / Long-Press Context Menu (Fly View)
- Click on empty map area → popup
- Available actions: "Go to Location", "Orbit at Location"
- Confirmation required before command executes

### 8.4 Right-Click / Long-Press on Instruments
- Enter instrument edit/reposition mode
- Lock/unlock to finalize instrument layout changes

### 8.5 3D Map View (v5.0)
- Import .osm file for 3D terrain/building rendering
- Full navigation controls (mouse + touch)
- Accessible from Fly View as alternate display mode

---

## 9. Notification System

### 9.1 Flight Readiness Status Bar (Toolbar)
- **Green background**: All checks pass — "Ready to Fly"
- **Amber background**: Warning present — "Ready to Fly" with caution
- **Red background**: Critical issue — "Not Ready"
- **"Flying"**: Vehicle is airborne and armed
- **"Armed"**: Armed but not yet flying
- **"Landing"**: Landing in progress
- **"Communication Lost"**: Telemetry link lost

### 9.2 Vehicle Messages Indicator (Toolbar)
- Turns red when vehicle reports important messages
- Dropdown shows full chronological message list
- Messages sourced from vehicle MAVLink STATUSTEXT

### 9.3 Pre-Arm Failure Display
- Clicking flight readiness indicator when "Not Ready" shows pre-arm checks popup
- List of all failing checks with toggle to expand each item
- Expanded view: detailed description + possible solutions
- Also reported as PREFLIGHT FAIL STATUSTEXT messages
- Preflight checklist panel (if enabled) shows pass/fail status for each check item

### 9.4 Critical / Warning Audio
- Audio announcements for flight mode changes, battery warnings, arming state
- Mute all audio output option in Settings
- "Announce battery lower than X%" configurable threshold

### 9.5 Mission Upload / Download Status
- Plan toolbar buttons highlight when there are unsaved/un-uploaded changes
- Mission upload failure notification (common over noisy telemetry links)

---

## 10. Responsive / Mobile Behavior

### 10.1 Screen Size Adaptation Strategy
- Single codebase using Qt QML Layout for reflow — no separate mobile/desktop builds
- Design priority: Tablet (10" Samsung Galaxy Tab) → Laptop → Desktop → Phone
- Phone is lowest priority; "worst hit on usability" per QGC dev docs

### 10.2 Differences on Small Screens
- Setup and Plan views are "functionally usable but may be painful" on phones
- Panel visibility reduced on smaller screens
- Virtual Joystick available as touch alternative to RC transmitter
- Sidebar panels collapse / hide

### 10.3 Touch Optimization
- Press-and-hold instead of right-click for context menus and instrument editing
- Pinch-to-zoom on maps
- Slider controls for confirmations (arm, takeoff, land, RTL)
- v5.0: additional touch screen optimization, especially for Herelink-style integrated controllers
- New slider controls for value entry throughout v5.0

### 10.4 Firmware Loading
- Not available on tablet or phone builds — desktop only

### 10.5 Desktop-Only Features
- MAVLink Inspector (charting)
- Firmware flashing
- Full parameter editor is usable but cramped on mobile

---

## 11. Unique QGC Features (Beyond Basic GCS)

### 11.1 Virtual Joystick
- On-screen thumbsticks for PX4 vehicles
- Enable via Settings → General
- Left-handed mode in v5.0
- Less responsive than physical RC (MAVLink path)

### 11.2 Physical Joystick / Gamepad Support
- Full SDL2-based gamepad support
- Button → flight mode or function mapping
- Expo curve, deadbands, circle correction
- Gimbal control via joystick axes
- Multiple joystick modes: Normal, Attitude, Position, Force, Velocity

### 11.3 Photo / Video Controls
- In-flight: still photo capture, video start/stop
- Camera mode toggle (still/video)
- Local recording of video stream (Matroska .mkv)
- Camera recording via MAVLink
- Camera trigger settings (time/distance) configurable per mission or globally
- GeoTag images post-flight using .tlog + image folder

### 11.4 Custom MAVLink Commands
- Parameters page exposes all raw MAVLink parameters
- MAVLink Console (PX4): run nsh commands directly on vehicle
- MAVLink Message Forwarding: relay traffic to second UDP endpoint
- MAVLink Inspector: live inspection and charting of any MAVLink field
- DO command support in missions for custom servo/relay/camera/gimbal actions

### 11.5 Offline Maps
- Named tile cache sets with zoom range
- Multiple geographic areas cached
- Works with all map providers (that have been pre-cached)

### 11.6 Multi-Vehicle Switching
- All vehicles on map simultaneously
- Toolbar selector for active vehicle
- Batch commands to all vehicles: Pause, Start Mission
- Per-vehicle telemetry and status in multi-vehicle list

### 11.7 ADSB Traffic Display
- SBS format ADSB server (TCP, e.g. dump1090)
- Blue icons on Fly View map with altitude + callsign
- Configurable host/port in Settings
- PX4 Traffic Avoidance failsafe integration

### 11.8 Terrain Following (Mission)
- Survey, Corridor Scan support terrain-following mode
- Vehicle maintains constant AGL (above ground level) height
- Configurable: tolerance, max climb rate, max descent rate
- Uses external terrain data (SRTM)
- Terrain profile chart in Plan View bottom panel

### 11.9 Follow Me Mode
- GCS streams GPS position via MAVLink
- Vehicle follows GCS device location in real time
- PX4 Multicopter only

### 11.10 Telemetry Log Replay
- Load and replay .tlog files
- Status bar in Fly View (enable in Settings)
- Review past flights in QGC interface

### 11.11 Mission Transform Tools
- Offset (East/North/Up in meters)
- Reposition (to coordinates, UTM, MGRS, or vehicle position)
- Rotate (CW degrees)
- All transforms applied to full mission including or excluding takeoff/landing

### 11.12 MAVLink 2 Log Streaming (PX4)
- Real-time ulog streaming from PX4 to QGC
- Automatic upload to FlightReview (logs.px4.io)
- Configurable flight metadata (description, video URL, wind speed, rating)

### 11.13 RTK GPS Base Station
- Survey-in mode: configure accuracy threshold + observation duration
- Fixed base position: enter known coordinates
- RTK status in toolbar indicator
- Dual GPS resilience: spoofing/jamming/authentication states

### 11.14 Brand Customization
- Custom indoor/outdoor logo images
- Used by OEM vendors (e.g., Yuneec, Autel) who fork QGC

### 11.15 Remote ID
- Remote ID health status indicator in toolbar
- Status detail dropdown

---

## 12. Key Architecture Notes (Relevant to Meridian GCS)

- **Qt/QML**: QGC is built on Qt 6 (v5.0) with QML for UI. Not directly portable to web, but architecture patterns are instructive
- **FactSystem**: All vehicle parameters wrapped in a typed "Fact" system with metadata (units, range, description). Parameters editor auto-generated from this
- **MissionCommandTree**: JSON metadata files per firmware/vehicle type define available commands and their parameters — can be referenced for Meridian's mission editor
- **MAVLink integration**: QGC communicates via MAVLink over Serial, UDP, TCP, or WebRTC (some builds)
- **Multi-firmware**: Explicitly supports PX4 and ArduPilot with firmware-specific UI branches. Many features differ between the two
- **Plan file format**: JSON `.plan` files are fully documented and human-readable — Meridian should support same format for interoperability

---

## Sources

- [QGC Fly View (master)](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html)
- [QGC Fly View (v5.0)](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/fly_view/hud.html)
- [Fly View Toolbar](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view_toolbar.html)
- [Fly Tools](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_tools.html)
- [Instrument Panel](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/instrument_panel.html)
- [Plan View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html)
- [Survey Pattern](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/pattern_survey.html)
- [Corridor Scan](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/pattern_corridor_scan.html)
- [Structure Scan](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/pattern_structure_scan_v2.html)
- [Fixed Wing Landing Pattern](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/pattern_fixed_wing_landing.html)
- [GeoFence](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_geofence.html)
- [Rally Points](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_rally_points.html)
- [Plan File Format](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html)
- [Mission Command Tree](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/plan/mission_command_tree.html)
- [Setup View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/setup_view.html)
- [Firmware Loading](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html)
- [Sensors (PX4)](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html)
- [Radio Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/radio.html)
- [Flight Modes (PX4)](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/flight_modes_px4.html)
- [Power Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/power.html)
- [Motors Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/motors.html)
- [Safety (ArduPilot)](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/safety_ardupilot.html)
- [Safety (PX4)](https://docs.px4.io/main/en/config/safety)
- [Tuning (PX4)](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/tuning_px4.html)
- [Joystick Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html)
- [Parameters](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/parameters.html)
- [Analyze View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/index.html)
- [MAVLink Inspector](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_inspector.html)
- [MAVLink Console](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html)
- [General Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/general.html)
- [Comm Links](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/comm_links.html)
- [Offline Maps](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/offline_maps.html)
- [MAVLink Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/mavlink.html)
- [Virtual Joystick](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/virtual_joystick.html)
- [3D Viewer](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/viewer_3d/viewer_3d.html)
- [Multi-Device Design Pattern](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/user_interface_design/multi_device_pattern.html)
- [Daily Build New Features](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_build_new_features.html)
- [ArduPilot Mission Commands](https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html)
