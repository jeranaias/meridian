# Meridian GCS — Architecture Plan

## Design Philosophy
- **No monolithic files** — each view and component is its own file
- **No build step** — plain HTML/CSS/JS, loaded via `<script>` tags
- **No frameworks** — vanilla JS, Leaflet from CDN, fonts from CDN
- **Every Meridian capability exposed** — 43 modes, 120+ params, 17 drivers, mission planning, failsafe config, geofence, the works

## Research Base
- `docs/research_mission_planner.md` — 1,040 lines, every MP feature
- `docs/research_qgroundcontrol.md` — 970 lines, every QGC feature

---

## File Structure

```
gcs/
├── index.html              # Shell — nav bar + view container, loads everything
├── css/
│   ├── base.css            # Reset, variables, typography, grid system
│   ├── toolbar.css         # Top toolbar (connection, status, nav tabs)
│   ├── instruments.css     # Flight instruments (ADI, compass, tapes, battery)
│   ├── map.css             # Map overlays, context menu, waypoint markers
│   ├── panels.css          # Side panels, drawers, modals
│   ├── actions.css         # Bottom action bar, buttons, slide confirm
│   ├── setup.css           # Setup/config view styles
│   ├── params.css          # Parameter list/tree styles
│   └── logs.css            # Log viewer styles
├── js/
│   ├── state.js            # Global state (window.meridian), event bus
│   ├── connection.js       # WebSocket manager, auto-reconnect, protocol dispatch
│   ├── mnp.js              # MNP protocol: COBS codec, message encode/decode
│   ├── mavlink.js          # MAVLink v2: CRC, frame parser, message encode/decode
│   ├── router.js           # View router — switches between Fly/Plan/Setup/Params/Logs
│   ├── toolbar.js          # Toolbar logic — status updates, connection UI
│   ├── commands.js         # Vehicle commands — arm, mode, takeoff, guided, kill
│   ├── confirm.js          # Slide-to-confirm modal
│   ├── messages.js         # Message log with severity filtering
│   │
│   ├── fly/
│   │   ├── fly-view.js     # Fly view orchestrator — assembles map + instruments
│   │   ├── map.js          # Leaflet map — vehicle, trail, home, WPs, geofence, rally
│   │   ├── adi.js          # Artificial horizon — sky/ground, pitch ladder, bank marks
│   │   ├── compass.js      # Compass strip — scrolling heading tape
│   │   ├── tapes.js        # Altitude + speed tapes — vertical scrolling strips
│   │   ├── battery.js      # Battery gauge — bar + voltage + current + mAh + cells
│   │   ├── status.js       # System health grid — GPS/EKF/RC/IMU/BAR/MAG
│   │   └── context-menu.js # Map right-click: Fly Here, Set Home, Add WP, Measure
│   │
│   ├── plan/
│   │   ├── plan-view.js    # Plan view orchestrator
│   │   ├── mission.js      # Mission item list — add/edit/delete/reorder WPs
│   │   ├── wp-editor.js    # Waypoint editor — command type, lat/lon/alt, params
│   │   ├── survey.js       # Survey tool — polygon area scan with camera settings
│   │   ├── geofence.js     # Geofence editor — polygon/circle inclusion/exclusion
│   │   ├── rally.js        # Rally point editor
│   │   └── terrain.js      # Terrain elevation profile along mission
│   │
│   ├── setup/
│   │   ├── setup-view.js   # Setup view orchestrator
│   │   ├── frame.js        # Frame type selection (quad/hex/octo/plane/rover/sub)
│   │   ├── accel-cal.js    # Accelerometer 6-point calibration wizard
│   │   ├── compass-cal.js  # Compass calibration wizard (rotate vehicle)
│   │   ├── radio-cal.js    # RC radio calibration (move all sticks)
│   │   ├── flight-modes.js # 6-mode channel configuration
│   │   ├── failsafe.js     # Failsafe configuration (RC/GPS/Battery/EKF thresholds)
│   │   ├── battery.js      # Battery monitor setup (voltage divider, current sensor)
│   │   └── motor-test.js   # Individual motor test (spin one at a time)
│   │
│   ├── params/
│   │   ├── params-view.js  # Parameter view orchestrator
│   │   ├── param-list.js   # Full parameter list — searchable, filterable
│   │   ├── param-tree.js   # Parameter tree view — grouped by subsystem
│   │   ├── param-file.js   # Save/load/compare parameter files
│   │   └── tuning.js       # PID tuning panel — sliders for key gains
│   │
│   └── logs/
│       ├── logs-view.js    # Logs view orchestrator
│       ├── log-list.js     # List of onboard logs — download, delete
│       ├── log-graph.js    # Graph telemetry from downloaded logs
│       └── mavlink-inspector.js # Live MAVLink message inspector with rates
│
└── assets/
    └── icons/              # SVG icons for toolbar, modes, status (inline SVG preferred)
```

---

## Views (5 main views, matching QGC)

### 1. FLY VIEW (default)
The primary flight display. What the pilot sees during flight.

**Layout:** Map (65%) + Instruments (35%) + Action bar (bottom)

**Map features (matching MP + QGC):**
- Vehicle icon with heading rotation + glow
- Position trail (polyline, configurable length)
- Home position marker
- Waypoint markers (numbered, command-colored)
- Waypoint connecting lines with direction arrows
- Geofence polygon overlay (red/translucent)
- Rally point markers
- ADSB traffic markers (if available)
- Click-to-fly-here (guided mode)
- Right-click context menu: Fly Here, Set Home, Add WP, Measure Distance, Set ROI
- Map type selector: Dark / Satellite / Street / Terrain
- Auto-center toggle, Follow vehicle toggle
- Zoom controls

**Instruments (matching MP HUD + QGC instrument panel):**
- Artificial horizon (pitch ladder every 5°, bank marks at 10/20/30/45/60°, fixed aircraft symbol)
- Compass heading strip
- Altitude tape (vertical, scrolling numbers)
- Speed tape (vertical, scrolling numbers)
- Vertical speed indicator
- Battery: voltage, current, remaining %, consumed mAh, per-cell voltage
- GPS: fix type, satellite count, HDOP
- EKF health with variance display
- RC: RSSI bar, link quality %, failsafe indicator
- System health grid: IMU, Baro, Mag, GPS (green/yellow/red)
- Distance to waypoint + WP number
- Flight time (since arm)

**Action bar:**
- ARM/DISARM with slide-to-confirm
- Mode selector (grouped: Copter/Plane/Rover/Sub with all 43 modes)
- TAKEOFF (altitude input popup)
- RTL (slide confirm)
- LAND (slide confirm)
- PAUSE/BRAKE
- KILL SWITCH (double confirm)
- Speed adjust slider
- Altitude adjust slider
- Message toggle (expand message log drawer)

**Message drawer (collapsible from bottom):**
- System messages with severity: ERROR (red), WARNING (amber), INFO (cyan), DEBUG (dim)
- Timestamp + source + text
- Filter by severity
- Auto-scroll with pause on hover
- Pre-arm failure messages highlighted

### 2. PLAN VIEW
Mission planning interface matching MP Flight Plan + QGC Plan View.

**Layout:** Map (full width) + Mission sidebar (left, collapsible)

**Mission sidebar:**
- Mission item list (scrollable, numbered)
- Each item: command type dropdown, lat/lon/alt fields, param fields
- Drag to reorder
- Add/Insert/Delete buttons
- Default altitude setting
- Total distance + estimated time display

**Map features (plan-specific):**
- Click map to add waypoint at that location
- Drag waypoint markers to reposition
- Waypoint connection lines with distance labels
- Numbered waypoint markers (different colors by command type)
- NAV = blue, TAKEOFF = green, LAND = orange, RTL = magenta, LOITER = yellow

**Command types (all we support in meridian-mission):**
- NAV_WAYPOINT, NAV_TAKEOFF, NAV_LAND, NAV_RETURN_TO_LAUNCH
- NAV_LOITER_UNLIM, NAV_LOITER_TURNS, NAV_LOITER_TIME
- NAV_SPLINE_WAYPOINT, NAV_DELAY
- DO_SET_SPEED, DO_CHANGE_SPEED, DO_JUMP
- DO_SET_ROI, DO_MOUNT_CONTROL
- DO_SET_SERVO, DO_SET_RELAY
- DO_LAND_START, DO_PARACHUTE, DO_GRIPPER
- DO_CAMERA_TRIGGER, DO_SET_CAM_TRIGG_DIST

**Survey tool:**
- Draw polygon on map
- Configure: camera overlap %, altitude, speed, angle
- Auto-generate survey waypoints within polygon

**Geofence editor:**
- Draw inclusion polygon(s)
- Draw exclusion polygon(s)
- Set circular fence (center + radius)
- Set altitude fence (max alt)
- Breach action selector

**Rally point editor:**
- Click map to add rally points
- Drag to reposition
- Set altitude per rally point

**Terrain profile:**
- Elevation chart along mission path
- AGL vs AMSL altitude display
- Terrain clearance warnings

**File operations:**
- Upload mission to vehicle
- Download mission from vehicle
- Save mission to file (.plan JSON format)
- Load mission from file
- Clear mission

### 3. SETUP VIEW
Vehicle configuration matching MP Initial Setup + QGC Vehicle Setup.

**Layout:** Sidebar navigation (left) + Content area (right)

**Sidebar sections:**
1. **Summary** — Vehicle type, firmware version, board type, serial number
2. **Frame Type** — Select: Quad-X/+/V/H, Hex, Octo, Plane, Rover, Sub (with diagrams)
3. **Accelerometer Calibration** — Step-by-step 6-position wizard (level, nose-up, nose-down, left, right, back)
4. **Compass Calibration** — Instruction to rotate vehicle in all orientations, progress bar, fitness display
5. **Radio Calibration** — Move all sticks/switches, show min/center/max per channel, calibrate button
6. **Flight Modes** — 6 mode slots mapped to RC switch positions, dropdown per slot
7. **Failsafe** — RC loss action, GCS loss action, Battery low/critical voltage+mAh, EKF action, Fence action
8. **Battery Monitor** — Sensor type, voltage divider, amp-per-volt, capacity
9. **Motor Test** — Test individual motors 1-8, throttle slider, safety warnings
10. **ESC Calibration** — All-at-once ESC calibration sequence
11. **OSD** — OSD item enable/disable and position (if OSD present)
12. **Servo Output** — Per-channel function assignment, min/max/trim/reverse

### 4. PARAMS VIEW
Full parameter management matching MP Config/Tuning + QGC Parameters.

**Layout:** Toolbar (search + filter) + Parameter list/tree (main area)

**Features:**
- **Search** — Type-ahead search across all parameter names and descriptions
- **List view** — Flat list, sortable by name/value/default
- **Tree view** — Grouped by prefix (ATC_, PSC_, EK3_, MOT_, FS_, etc.)
- **Per-parameter:**
  - Name, current value (editable), default value, min/max range
  - Description tooltip
  - Modified indicator (yellow if changed from default)
  - Reset-to-default button
- **Bulk operations:**
  - Save all to file (.param format)
  - Load from file
  - Compare two files
  - Write changed params to vehicle
  - Refresh from vehicle
- **Tuning panel (sidebar):**
  - PID gain sliders: Rate Roll/Pitch/Yaw P/I/D
  - Attitude P gains
  - Position controller gains
  - Speed limits
  - Live-updating (write on slider change)

### 5. LOGS VIEW
Flight log management matching MP Telemetry Logs/DataFlash + QGC Analyze.

**Layout:** Toolbar + Content area

**Features:**
- **Log list** — Available logs on vehicle (number, date, size)
- **Download** — Download log from vehicle to browser
- **Graph** — Plot telemetry values from downloaded log (multi-axis, zoomable)
- **MAVLink Inspector** — Live message view with message names, rates, field values
- **Message Console** — Live system message feed (like MP Messages tab)

---

## Shared Components

### Toolbar (always visible, all views)
- Brand + nav tabs + status chips + connection controls
- Status chips update at 10Hz from state.js

### Slide-to-Confirm Modal
- Used for: ARM, DISARM, TAKEOFF, RTL, LAND, KILL
- Draggable thumb, must reach 85% of track width
- Cancel button + overlay dim

### Notification System
- Toast notifications for quick feedback
- Persistent message log in drawer
- Pre-arm failure display (blocks ARM, shows all failed checks)

### Connection Manager
- Auto-reconnect on disconnect (3s delay)
- Protocol selector: MNP (native) / MAVLink (adapter)
- 1Hz GCS heartbeat for MAVLink connections
- Byte-level frame extraction for both protocols

---

## State Management

All state lives in `window.meridian` — a single flat object:

```javascript
window.meridian = {
  // Connection
  connected: false,
  protocol: 'mavlink', // or 'mnp'
  
  // Vehicle state
  armed: false,
  modeNum: 0,
  modeName: 'STABILIZE',
  
  // Attitude
  roll: 0, pitch: 0, yaw: 0,
  
  // Position
  lat: 0, lon: 0,
  alt: 0, altAgl: 0, altAmsl: 0,
  
  // Velocity
  groundspeed: 0, airspeed: 0, climb: 0,
  heading: 0, groundCourse: 0,
  
  // Battery
  voltage: 0, current: 0, remaining: 0, mah: 0,
  cells: [],
  
  // GPS
  gpsFix: 0, gpsSats: 0, hdop: 99,
  
  // Sensors
  ekfOk: false,
  imuOk: false, baroOk: false, magOk: false,
  rcRssi: 0, linkQuality: 0,
  vibration: [0, 0, 0],
  
  // RC
  rcChannels: new Array(18).fill(1500),
  rcFailsafe: false,
  
  // Navigation
  wpDist: 0, wpNum: 0, wpBearing: 0,
  homeDistance: 0, homeBearing: 0,
  
  // Mission
  missionItems: [],
  missionCurrentSeq: 0,
  
  // Parameters
  params: {},
  paramCount: 0,
  
  // Messages
  messages: [],
  
  // Trail
  trail: [],
  
  // Timing
  armTime: 0,
  flightTime: 0,
  
  // WebSocket
  ws: null,
};
```

## Event Bus

Simple pub/sub for decoupling:

```javascript
const Events = {
  _listeners: {},
  on(event, fn) { (this._listeners[event] ||= []).push(fn) },
  emit(event, data) { (this._listeners[event] || []).forEach(fn => fn(data)) },
};
```

Events: `connected`, `disconnected`, `armed`, `disarmed`, `mode-changed`, `position-updated`, `attitude-updated`, `battery-updated`, `gps-updated`, `message-received`, `param-received`, `mission-received`, `heartbeat`

---

## Build Order

Phase 1: Core infrastructure
1. `css/base.css` — variables, reset, grid
2. `js/state.js` — global state + event bus
3. `js/router.js` — view switching
4. `index.html` — shell with nav + view containers

Phase 2: Fly View (most used)
5. `css/toolbar.css` + `js/toolbar.js`
6. `css/instruments.css` + instrument components (ADI, compass, tapes, battery, status)
7. `css/map.css` + `js/fly/map.js`
8. `css/actions.css` + `js/commands.js` + `js/confirm.js`
9. `js/fly/fly-view.js` — wires it all together
10. `js/messages.js` — message log drawer

Phase 3: Connection
11. `js/connection.js` — WebSocket manager
12. `js/mnp.js` — MNP protocol
13. `js/mavlink.js` — MAVLink protocol

Phase 4: Plan View
14. `js/plan/plan-view.js` + mission sidebar + WP editor
15. `js/plan/survey.js` + geofence + rally + terrain

Phase 5: Setup View
16. `js/setup/setup-view.js` + all config wizards

Phase 6: Params View
17. `js/params/params-view.js` + list/tree/tuning

Phase 7: Logs View
18. `js/logs/logs-view.js` + graph + inspector

---

## Color Palette

```
Deep Void:      #06080d  (body background)
Panel:          #0b0f18  (sidebar, toolbar)
Surface:        #101624  (cards, raised areas)
Raised:         #161d2e  (hover states, active)
Well:           #080b12  (inputs, inset areas)

Edge:           #1a2338  (borders, dividers)
Edge Bright:    #253050  (focus borders, active)

Text Primary:   #e2e8f0  (main text)
Text Secondary: #94a3b8  (labels)
Text Dim:       #475569  (placeholders, inactive)
Text Muted:     #1e293b  (very dim)

Cyan:           #00e5ff  (primary accent, MNP)
Green:          #00ff7f  (safe, armed-safe, GPS OK)
Amber:          #ffa726  (warnings, mode display)
Red:            #ff1744  (danger, armed, kill)
Magenta:        #e040fb  (home marker, special)
Blue:           #448aff  (waypoints, info)
```

## Typography

- **Display**: Syne (800 weight for brand, 600-700 for headings)
- **Data**: DM Mono (300-500, all telemetry values, parameters, coordinates)
- **Labels**: DM Mono 500, 8-9px, uppercase, letter-spacing 1-2px

---

## What This Covers From Research

### From Mission Planner (1,040 lines of research):
- [x] HUD with all 15 indicators → Fly View instruments
- [x] Map with all overlays → Fly View map
- [x] Actions tab → Action bar
- [x] Status tab → System health grid
- [x] Messages tab → Message drawer
- [x] Flight Plan tab → Plan View
- [x] Initial Setup tab → Setup View
- [x] Config/Tuning tab → Params View
- [x] Full Parameter List → Params View list/tree
- [x] Telemetry Logs → Logs View
- [x] DataFlash Logs → Logs View
- [x] Right-click menus → Context menus
- [x] Connection controls → Toolbar
- [x] Servo/Relay → Setup View
- [x] Motor test → Setup View

### From QGroundControl (970 lines of research):
- [x] Fly View → Fly View
- [x] Plan View → Plan View
- [x] Analyze View → Logs View
- [x] Vehicle Setup → Setup View
- [x] Application Settings → integrated into Params
- [x] Toolbar indicators → Toolbar status chips
- [x] Fly Tools / Action Panel → Action bar
- [x] Instrument Panel → Instruments panel
- [x] Slide-to-confirm → Confirm modal
- [x] Survey/Corridor/Structure Scan → Plan View survey
- [x] GeoFence editor → Plan View geofence
- [x] Rally points → Plan View rally
- [x] Terrain profile → Plan View terrain
- [x] MAVLink Inspector → Logs View inspector
- [x] Parameter management → Params View
- [x] Sensor calibration wizards → Setup View
- [x] Motor test → Setup View
- [x] Offline maps → future (not Phase 1)
- [x] Multi-vehicle → toolbar vehicle selector (future)
- [x] Virtual joystick → future
