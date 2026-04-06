# Meridian GCS — Architecture v2
## Revised after 5-expert panel review (Oborne, Meier, Tufte, Krug, Victor)

---

## Changes from v1 (every expert finding addressed)

### From Michael Oborne (Mission Planner creator):
- [x] Added automatic tlog recording (on connect → record every byte)
- [x] Added tlog playback with speed controls (0.5x/1x/2x/5x)
- [x] Added Status/raw telemetry view (200+ fields, searchable, 4Hz update)
- [x] Added camera/gimbal operations to Fly View context menu
- [x] Added Resume Mission and Restart Mission to action bar
- [x] Added "Quick" widget — user-selectable large-text values

### From Lorenz Meier (QGC creator):
- [x] Fly View is now the primary canvas — other views open as overlays/drawers, not hard tab switches
- [x] State refactored: `meridian.vehicles[sysid]` not flat namespace
- [x] Added Application Settings view (units, map provider, guided limits, ADSB, reconnect)
- [x] Expanded Survey tool (camera model library, terrain following, hover-and-capture, corridor scan)
- [x] Mission stats panel always-visible at bottom of Plan view
- [x] Added keyboard shortcuts (F=Fly, P=Plan, S=Setup, Ctrl+P=Params, L=Logs)

### From Edward Tufte (data visualization):
- [x] Artificial horizon uses flat colors (no gradients)
- [x] Battery widget: % + bar always visible, voltage/current/mAh on hover, per-cell only on variance warning
- [x] GPS shows numbers always (sats + HDOP), not just colored dot
- [x] EKF variance value visible as number, not just dot
- [x] Red reserved exclusively for emergency — armed state uses ORANGE
- [x] Altitude/speed tape: tick intervals, visible range, center position all specified
- [x] Data-ink audit checklist added to implementation guidelines

### From Steve Krug (usability):
- [x] KILL SWITCH moved out of action bar — isolated position, different shape, long-press required
- [x] Primary Flight State badge: ARM STATE + MODE in one prominent area (top center)
- [x] Mode selector: 5 common modes as one-tap buttons + "More..." for full list
- [x] Plan View: Upload button has dirty-state indicator (amber pulsing when unsent)
- [x] Setup View: Summary page is a checklist with completion status per step

### From Bret Victor (direct manipulation):
- [x] Map shows live commanded trajectory (controller setpoint path, next 5-10s)
- [x] Map shows position uncertainty ellipse (from GPS HDOP + EKF variance)
- [x] Instruments show target-vs-actual: ghost markers on altitude/speed tapes at commanded values
- [x] Terrain profile is interactive: drag altitude handles to edit waypoint altitude
- [x] PID tuning: live response chart per axis (commanded vs actual rate, last 10s)
- [x] Mission validator runs on every change (turn radius, geofence, terrain clearance)

---

## Navigation Model (Meier revision)

**Fly View is ALWAYS the base layer.** Other views open as overlays:

```
┌─────────────────────────────────────────────────┐
│  TOOLBAR (always visible)                        │
├─────────────────────────────────────────────────┤
│                                                  │
│  FLY VIEW (always active, never unmounts)        │
│  ┌──────────┐                                    │
│  │ Map +    │  ┌─────────────────────────────┐  │
│  │ Vehicle  │  │ OVERLAY PANEL (right side)   │  │
│  │ Trail +  │  │                              │  │
│  │ WPs      │  │ Can be: Instruments (default)│  │
│  │          │  │         Plan sidebar          │  │
│  │          │  │         Setup wizard          │  │
│  │          │  │         Params list           │  │
│  │          │  │         Logs viewer           │  │
│  │          │  │         Status telemetry      │  │
│  │          │  │         Settings              │  │
│  │          │  │                              │  │
│  └──────────┘  └─────────────────────────────┘  │
│                                                  │
├─────────────────────────────────────────────────┤
│  ACTION BAR (always visible)                     │
└─────────────────────────────────────────────────┘
```

**Keyboard shortcuts:**
- `F` or `Escape` → Instruments panel (default Fly view)
- `P` → Plan overlay (mission editor replaces instruments panel)
- `S` → Setup overlay (config wizards)
- `Ctrl+P` → Params overlay (parameter list)
- `L` → Logs overlay (log viewer)
- `T` → Status/telemetry overlay (raw values)
- `Ctrl+,` → Settings overlay

The map + action bar + toolbar are ALWAYS visible. Only the right panel changes.

---

## State Architecture (Meier revision)

```javascript
window.meridian = {
  // Multi-vehicle ready from day 1
  activeVehicleId: 1,
  vehicles: {
    1: {
      sysid: 1,
      connected: false,
      armed: false,
      modeNum: 0,
      // ... all vehicle state
    }
  },
  
  // Active vehicle shortcut (updated on vehicle switch)
  get v() { return this.vehicles[this.activeVehicleId] || {} },
  
  // App-level state
  settings: {
    units: 'metric',         // 'metric' | 'imperial'
    mapProvider: 'carto-dark',
    guidedAltMax: 120,       // meters
    guidedDistMax: 1000,     // meters
    reconnectDelay: 3000,    // ms
    adsbServer: '',
    showUncertainty: true,   // position uncertainty ellipse
    showTrajectory: true,    // commanded trajectory line
    commonModes: [0, 5, 6, 9, 3], // Stabilize, Loiter, RTL, Land, Auto
  },
  
  // Recording
  tlog: {
    recording: false,
    bytes: [],
    startTime: null,
    filename: '',
  },
  
  // Events
  events: { /* pub/sub */ },
  
  // WebSocket
  ws: null,
};
```

---

## Color Palette (Tufte revision)

```
CRITICAL CHANGE: Red is EMERGENCY ONLY. Armed state = Orange.

Emergency:    #ff1744  (ONLY for: kill switch, crash, motor failure, critical battery)
Warning:      #ffa726  (amber — warnings, cautions, attention needed)
Armed:        #ff6d00  (ORANGE — distinct from emergency red, distinct from warning amber)
Safe/OK:      #00ff7f  (green — healthy, disarmed, good)
Primary:      #00e5ff  (cyan — MNP, interactive elements, primary accent)
Info:         #448aff  (blue — waypoints, informational)
Special:      #e040fb  (magenta — home marker, ROI)
Neutral:      #94a3b8  (text secondary)

Rule: Emergency red (#ff1744) is NEVER used for routine states.
When armed with no emergency: orange badge, not red.
When armed AND emergency: red badge replaces orange.
```

---

## Fly View Instruments Panel (Tufte + Victor revisions)

### Artificial Horizon
- **Flat colors** (no gradient): solid #1a3a6a sky, solid #5a3a1a ground (Tufte)
- Pitch ladder every 5°, bank marks at 10/20/30/45/60°
- Fixed aircraft symbol (amber wings)
- **Target pitch/roll ghost overlay** (Victor): thin dashed line showing controller's attitude target

### Altitude Tape
- Vertical strip, 120px wide
- **Specified tick intervals**: major marks every 10m (below 100m) or 50m (above 100m)
- **Visible range**: ±50m from current (adjusts with climb rate)
- **Current value**: bright green box at center
- **Target altitude ghost**: small cyan marker at commanded altitude (Victor)
- **Geofence ceiling**: red line if altitude fence is set (Tufte)
- Numbers in DM Mono 500

### Speed Tape
- Same layout as altitude tape, mirrored
- Major marks every 1 m/s (below 10) or 5 m/s (above 10)
- Current value: cyan box
- **Target speed ghost**: small green marker at commanded speed
- **Stall speed warning**: red zone below ARSPD_FBW_MIN (plane only)

### Battery Widget (Tufte revision)
- **Always visible**: colored bar + percentage number (large)
- **On hover/tap**: expand to show voltage, current, mAh consumed
- **On cell variance warning**: auto-expand to show per-cell voltages with problem cell highlighted red
- **Visual change on warning**: bar turns amber at 30%, pulses red at 15%
- The widget should look DIFFERENT when something is wrong vs normal

### GPS Display (Tufte revision)
- Always show: satellite count + HDOP as numbers
- Format: "14 SAT · 0.8 HDOP" in DM Mono
- Fix type as text label: "3D FIX" / "RTK FIXED" / "NO FIX"
- Colored indicator dot only as redundant channel, not primary

### EKF Display (Tufte revision)
- Show variance numbers (vel/pos/hgt) as small bars, not just a dot
- "EKF OK" / "EKF WARN" / "EKF BAD" text label
- Variance values visible without hover

### System Health Grid
- Each sensor: dot + name + key metric number
- GPS: dot + "GPS 14/0.8" (sats/hdop)
- EKF: dot + "EKF 0.12" (max variance)
- RC: dot + "RC 98%" (LQ)
- IMU: dot + "IMU OK"
- BAR: dot + "BAR OK"
- MAG: dot + "MAG OK"

### Quick Widget (Oborne addition)
- 4-6 user-configurable large-text values
- Right-click to select which telemetry field to show
- Default: WP dist, Home dist, Flight time, Throttle %

---

## Map Features (Victor revisions)

### Standard (from v1):
- Vehicle icon with heading rotation
- Position trail (configurable length)
- Home marker, waypoint markers, geofence overlay, rally points
- Click-to-fly, right-click context menu
- Map type selector, center/follow toggles

### New (Victor additions):
- **Commanded trajectory line**: short projected path (5-10s) from controller setpoints (nav_roll, nav_pitch, target_bearing). Distinct from planned path — shows what the vehicle is TRYING to do right now.
- **Position uncertainty ellipse**: circle around vehicle scaled by HDOP * EKF position variance. Toggle-able via settings.
- **Deviation indicator**: when in Auto mode, show crosstrack error as a perpendicular line from planned path to actual position.

### Context Menu (Oborne addition):
- Fly Here (guided)
- Set Home Here
- Add Waypoint
- Measure Distance
- **Point Camera Here** (DO_SET_ROI)
- **Trigger Camera Now**
- Set ROI

---

## Action Bar (Krug revision)

### Layout (left to right):

```
┌────────────────────────────────────────────────────────────────────┐
│ [ARM/DISARM]  │  [STAB] [LOIT] [RTL] [LAND] [AUTO] [More▾]      │
│               │                                                   │
│  slide-to-    │  Common modes as one-tap buttons (configurable)   │
│  confirm      │                                                   │
├───────────────┼───────────────────────────────────────────────────┤
│ [TAKEOFF↑]    │ [PAUSE] [RESUME] [RESTART]  │  Speed▸  Alt▸     │
│  with alt     │  mission controls            │  sliders          │
│  input        │                              │                    │
├───────────────┴──────────────────────────────┴───────────────────┤
│                                              │ [MSGS] │ [⚠ KILL]│
│                     (spacer)                 │        │ isolated │
│                                              │        │ long-press│
└──────────────────────────────────────────────┴────────┴──────────┘
```

**Key changes from v1:**
- **Common modes are one-tap buttons**, not a dropdown (Krug)
- **"More..."** opens full 43-mode dropdown (filtered by vehicle type)
- **Resume Mission / Restart Mission** buttons visible when mission loaded (Oborne)
- **KILL SWITCH isolated** at far right with different shape + long-press required (Krug)
- **Speed/Alt sliders** in their own group, separated from discrete actions (Krug)

### Primary Flight State Badge (Krug addition)
Top-center of toolbar, largest visual element:
```
┌──────────────────────────┐
│  ● ARMED · LOITER · WP 7/14  │
│    12.4V · 14 SAT · 2:47     │
└──────────────────────────┘
```
- Line 1: Armed state (colored dot) + Mode + Mission progress
- Line 2: Battery + GPS + Flight time
- This is the FIRST thing the pilot reads

---

## Plan View — Mission Stats Panel (Meier revision)

Always-visible bottom panel in Plan View:

```
┌──────────────────────────────────────────────────────────┐
│ WPs: 14  │  Dist: 2.3 km  │  Est. Time: 8:42  │  Batt: ~1.2  │
│ Max Alt: 80m  │  Max Dist: 450m  │  ⚠ WP4 turn radius too tight │
└──────────────────────────────────────────────────────────┘
```

- Total waypoints, total distance, estimated flight time
- Battery estimate (how many batteries needed)
- Max altitude and max distance from home
- **Mission validator warnings inline** (Victor): turn radius, geofence containment, terrain clearance

---

## Plan View — Interactive Terrain Profile (Victor revision)

The terrain profile chart at the bottom of Plan View is **bidirectional**:
- Shows elevation + planned altitude + terrain clearance
- **Drag altitude handles** on the chart to edit waypoint altitude
- Waypoint sidebar fields update when handles are dragged
- Terrain clearance zones colored: green (>20m), amber (10-20m), red (<10m)
- Geofence ceiling shown as horizontal red line

---

## Setup View — Commissioning Checklist (Krug revision)

Summary page shows completion status:

```
┌─────────────────────────────────────────┐
│  Vehicle Setup Checklist                 │
│                                          │
│  ✅ Frame Type         Quad-X            │
│  ✅ Accelerometer      Calibrated        │
│  ⬜ Compass            Not calibrated    │
│  ⬜ Radio              Not calibrated    │
│  ✅ Flight Modes       6 modes set       │
│  ⬜ Failsafe           Not configured    │
│  ✅ Battery            Configured        │
│  ── Motor Test         Optional          │
│  ── ESC Calibration    Optional          │
│  ── OSD               Optional           │
│  ── Servo Output       Optional          │
│                                          │
│  ⚠ 3 required items not complete        │
│  Vehicle cannot arm until completed      │
└─────────────────────────────────────────┘
```

Required items block arming. Optional items shown but not blocking.

---

## PID Tuning Panel (Victor revision)

```
┌─────────────────────────────────────────────────┐
│  Rate Roll PID                                   │
│                                                  │
│  P: [====●========] 0.135                        │
│  I: [====●========] 0.135                        │
│  D: [==●==========] 0.0036                       │
│                                                  │
│  ┌──────────────────────────────────────────┐    │
│  │  Response Chart (last 10s)                │    │
│  │  ─── commanded rate (cyan)                │    │
│  │  ─── actual rate (white)                  │    │
│  │  Gap between them = tracking error        │    │
│  │                                           │    │
│  │  [oscillating wave pattern]               │    │
│  └──────────────────────────────────────────┘    │
│                                                  │
│  Move slider → see response change immediately   │
└─────────────────────────────────────────────────┘
```

---

## Tlog Recording + Playback (Oborne addition)

### Recording:
- Starts automatically on WebSocket connect
- Records every incoming byte to an in-memory ArrayBuffer
- On disconnect or manual save: download as `.tlog` file (timestamped MAVLink) or `.mnplog` (timestamped MNP)
- Status indicator in toolbar: "● REC 2:47 · 1.2 MB"

### Playback:
- Load a `.tlog` or `.mnplog` file
- Parse frames, feed through event bus at original timing
- Speed controls: 0.5x, 1x, 2x, 5x, 10x
- Scrub bar (timeline slider)
- All instruments animate from log data
- Map replays trail

---

## Application Settings (Meier addition)

Accessible via `Ctrl+,` or gear icon in toolbar:

- **Units**: Metric / Imperial / Aviation (ft + knots)
- **Map Provider**: CartoDB Dark / Satellite / Street / Terrain / Custom tile URL
- **Guided Limits**: Max altitude (m), Max distance from home (m)
- **Connection**: Default URL, auto-reconnect delay, GCS heartbeat interval
- **ADSB**: Server URL, display range
- **Display**: Show uncertainty ellipse, show trajectory, show crosstrack
- **Common Modes**: Configure which 5 modes appear as one-tap buttons
- **Recording**: Auto-record on connect (on/off)

---

## File Structure (updated)

```
gcs/
├── index.html              # Shell — toolbar + map + overlay container + action bar
├── css/
│   ├── base.css            # Variables (v2 palette), reset, typography, grid
│   ├── toolbar.css         # Toolbar + primary flight state badge
│   ├── instruments.css     # ADI (flat), compass, tapes (with ghost markers), battery (tiered)
│   ├── map.css             # Map overlays, context menu, trajectory, uncertainty ellipse
│   ├── panels.css          # Overlay panel transitions, side panels
│   ├── actions.css         # Action bar (separated groups), kill switch isolation
│   ├── plan.css            # Plan overlay, mission list, terrain profile (interactive)
│   ├── setup.css           # Setup checklist, calibration wizards
│   ├── params.css          # Parameter list/tree, PID tuning with response chart
│   ├── logs.css            # Log viewer, tlog playback controls, MAVLink inspector
│   ├── status.css          # Raw telemetry status view
│   └── settings.css        # Settings modal
├── js/
│   ├── state.js            # Multi-vehicle state + event bus + active vehicle shortcut
│   ├── connection.js       # WebSocket + tlog recording
│   ├── mnp.js              # MNP COBS codec
│   ├── mavlink.js          # MAVLink v2 codec
│   ├── router.js           # Overlay panel router (not tab router)
│   ├── toolbar.js          # Toolbar + primary flight state badge
│   ├── commands.js         # Vehicle commands
│   ├── confirm.js          # Slide-to-confirm + long-press-to-kill
│   ├── messages.js         # Message log
│   ├── settings.js         # Application settings
│   ├── tlog.js             # Recording + playback engine
│   │
│   ├── fly/
│   │   ├── fly-view.js     # Fly view orchestrator
│   │   ├── map.js          # Leaflet + trajectory + uncertainty + crosstrack
│   │   ├── adi.js          # Flat ADI + target ghost overlay
│   │   ├── compass.js      # Compass strip
│   │   ├── tapes.js        # Alt/speed tapes with ghost targets + geofence ceiling
│   │   ├── battery.js      # Tiered battery (% always, details on hover, cells on warning)
│   │   ├── status.js       # System health with numbers (GPS 14/0.8, EKF 0.12)
│   │   ├── quick.js        # User-configurable quick values
│   │   └── context-menu.js # Right-click: fly here + camera trigger + gimbal point
│   │
│   ├── plan/
│   │   ├── plan-view.js    # Plan overlay orchestrator
│   │   ├── mission.js      # Mission list + dirty-state indicator
│   │   ├── wp-editor.js    # Waypoint editor
│   │   ├── survey.js       # Survey + camera models + terrain follow + corridor
│   │   ├── geofence.js     # Geofence editor
│   │   ├── rally.js        # Rally points
│   │   ├── terrain.js      # Interactive terrain profile (drag to edit altitude)
│   │   ├── stats.js        # Always-visible mission stats panel
│   │   └── validator.js    # Real-time mission validator (turn radius, geofence, terrain)
│   │
│   ├── setup/
│   │   ├── setup-view.js   # Setup overlay with checklist
│   │   ├── checklist.js    # Commissioning checklist logic
│   │   ├── frame.js        # Frame type
│   │   ├── accel-cal.js    # Accel calibration wizard
│   │   ├── compass-cal.js  # Compass calibration wizard
│   │   ├── radio-cal.js    # Radio calibration
│   │   ├── flight-modes.js # Flight mode configuration
│   │   ├── failsafe.js     # Failsafe configuration
│   │   ├── battery.js      # Battery monitor setup
│   │   └── motor-test.js   # Motor test
│   │
│   ├── params/
│   │   ├── params-view.js  # Parameter overlay
│   │   ├── param-list.js   # Full list (searchable)
│   │   ├── param-tree.js   # Tree view
│   │   ├── param-file.js   # Save/load/compare
│   │   └── tuning.js       # PID tuning with live response chart
│   │
│   ├── logs/
│   │   ├── logs-view.js    # Logs overlay
│   │   ├── log-list.js     # Log list + download
│   │   ├── log-graph.js    # Graph from log data
│   │   ├── playback.js     # Tlog playback UI (scrub bar, speed controls)
│   │   └── mavlink-inspector.js # Live MAVLink inspector
│   │
│   └── status/
│       └── status-view.js  # Raw telemetry (200+ fields, searchable, 4Hz)
│
└── assets/
    └── icons/
```

**Total: 48 JS files, 12 CSS files, 1 HTML shell**

---

## Implementation Priority

### Phase 1: Core + Fly View (ship this first)
- index.html shell, base.css, state.js, router.js, connection.js
- toolbar.js + primary flight state badge
- map.js (basic: vehicle, trail, home, center/follow)
- adi.js (flat colors, pitch ladder, bank marks)
- compass.js, tapes.js (with ghost targets), battery.js (tiered)
- status.js (with numbers), quick.js
- commands.js, confirm.js (slide + long-press kill)
- messages.js, context-menu.js (with camera trigger)
- mnp.js, mavlink.js
- Demo mode (simulated vehicle)

### Phase 2: Plan View
- plan-view.js, mission.js (with dirty state), wp-editor.js
- Map waypoint editing (drag, number, color-coded)
- stats.js (always-visible bottom panel)
- validator.js (real-time checks)
- terrain.js (interactive profile)
- geofence.js, rally.js
- survey.js (basic polygon + camera)

### Phase 3: Setup + Params
- setup-view.js, checklist.js
- accel-cal.js, compass-cal.js, radio-cal.js
- flight-modes.js, failsafe.js, battery.js, motor-test.js
- params-view.js, param-list.js, param-tree.js, param-file.js
- tuning.js (with live response chart)

### Phase 4: Logs + Status + Settings + Recording
- logs-view.js, log-list.js, log-graph.js
- tlog.js (recording + playback), playback.js
- mavlink-inspector.js
- status-view.js (200+ fields)
- settings.js

### Phase 5: Advanced
- Map trajectory line + uncertainty ellipse + crosstrack
- Survey camera model library + terrain following
- Corridor scan + structure scan
- Multi-vehicle support
- Offline maps

---

## Expert Sign-off Checklist

Before building, verify this v2 addresses every concern:

| Expert | #1 Concern | Addressed? |
|--------|-----------|-----------|
| Oborne | No tlog recording | ✅ tlog.js + auto-record |
| Meier | Flat state breaks multi-vehicle | ✅ vehicles[sysid] |
| Tufte | Red = armed AND danger | ✅ Orange for armed, Red for emergency only |
| Krug | KILL next to LAND | ✅ KILL isolated, long-press required |
| Victor | Map is passive | ✅ Trajectory line + uncertainty + interactive terrain |
