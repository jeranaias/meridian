<p align="center">
  <img src="../assets/meridian-logo.svg" width="64" alt="Meridian">
</p>

<h1 align="center">Meridian GCS</h1>

<p align="center">
  <strong>A modern, browser-based Ground Control Station.</strong><br>
  No install. No build step. No framework. Just open and fly.
</p>

<p align="center">
  <a href="#features">Features</a> &middot;
  <a href="#quick-start">Quick Start</a> &middot;
  <a href="#architecture">Architecture</a> &middot;
  <a href="#why-meridian">Why Meridian</a> &middot;
  <a href="#contributing">Contributing</a>
</p>

---

## Why Meridian

Mission Planner and QGroundControl are remarkable software that have enabled millions of flights. They set the standard for what a ground control station should do. Meridian GCS builds on their legacy with a browser-native approach — modern browsers can now render real-time flight instruments at 30fps, handle WebSocket telemetry, and run on any device with a screen.

**Meridian GCS is built from scratch for the way people fly today:**

- Open a URL. You're flying. No download, no installer, no framework, no build step.
- Runs on anything with a browser — laptop, tablet, phone, Raspberry Pi kiosk.
- Dark and light themes that actually work, including canvas instruments.
- Touch-first controls with keyboard shortcuts for power users.
- Speaks **Meridian Native Protocol** over WebSocket, with MAVLink v2 compatibility for legacy autopilots.

This is the GCS for everyone who's been waiting for something better.

---

## Features

### Fly View
- **Canvas flight instruments** — Artificial horizon (ADI) with pitch ladder, compass strip, speed and altitude tapes, all GPU-accelerated
- **Leaflet map** — Vehicle tracking, heading-rotated icon, velocity trail, trajectory projection, home guide line, mission path overlay, ADSB traffic, drag-to-fly
- **8-field quick readout** — Color-coded telemetry values (altitude, speed, distance, climb, heading, time, throttle) with right-click to customize
- **Always-visible telemetry strip** — GPS fix, battery %, RC signal, EKF health, flight time in the toolbar across all views
- **Wind estimation** — Real-time wind speed and direction from ground/airspeed vector difference, arrow overlay on map
- **Battery intelligence** — Time remaining estimate, consumption rate, voltage/current/mAh, color-coded warnings
- **Video feed** — MJPEG/RTSP PiP overlay, draggable, fullscreen swap
- **Slide-to-arm** with pre-flight checklist gate, long-press emergency KILL

### Plan View
- **Waypoint editor** — Click-to-add, drag-to-reorder, inline parameter editing
- **Survey tools** — Polygon grid scan, corridor scan, orbit missions, cinematic quickshots (Dronie/Helix/Reveal)
- **Terrain profile** — Altitude chart with ground elevation and clearance warnings
- **Mission validator** — Altitude limits, distance limits, battery endurance vs flight time, duplicate waypoint detection
- **Statistics** — Distance, estimated time, max altitude, battery endurance margin
- **Geofence** — Polygon drawing with FENCE_POINT upload
- **Import/Export** — QGC WPL 110 waypoint files

### Setup
- **Pre-flight regulatory checklist** — 7-item FAA/EU compliance check (GPS, battery, weather, airspace, visual observer)
- **Calibration wizards** — Accelerometer (6-position), compass, radio (live channel bars)
- **Frame selection** — Visual grid with motor layouts
- **Flight modes** — 6-slot configuration with PWM ranges
- **Failsafe** — RC loss, battery, GCS timeout configuration
- **Motor test** — Individual motor spin with throttle/duration control

### Parameters
- **Grouped by prefix** — 17 categories (Attitude Control, Battery, Failsafe, Geofence, etc.)
- **Descriptions** — 50+ common parameters with human-readable explanations
- **Search, load, save** — Filter by name, import/export .param files, Betaflight CLI dump import
- **PID tuning** — Per-axis sliders with step response chart
- **Modified highlighting** — Changed params are visually marked

### Logs
- **Tlog recording** — IndexedDB-backed, 64KB chunks, auto-start on connect
- **Flight replay** — Play back tlog through HUD instruments and map at variable speed
- **Graph viewer** — Time-series plot of any telemetry field
- **MAVLink inspector** — Live message stream with field-level decode
- **Battery lifecycle** — Per-pack cycle tracking with health scoring
- **Auto-analysis** — 6 anomaly checks on recorded data
- **Scripting console** — Sandboxed JavaScript with vehicle state access

### Status
- **166+ telemetry fields** — Organized by category (Attitude, Position, GPS, EKF, RC, Battery)
- **Units on every value** — Degrees, m/s, V, A, %, mAh
- **4Hz live update** — Change-flash animation on value updates
- **Collapsible sections** — Focus on what matters

### Settings
- **22 configuration sections** — Theme, units, map provider, connection, ADSB, operator identity, EU compliance, offline maps, recording, ROS2 bridge, STANAG 4586
- **Offline tile caching** — Download map regions for field use without internet
- **Multi-vehicle** — Connection pool, vehicle selector, fleet registry
- **i18n framework** — Locale-based string translation system

---

## Quick Start

```bash
# Clone the repo
git clone https://github.com/jeranaias/meridian-gcs.git
cd meridian-gcs

# Serve it (any static server works)
python -m http.server 8080
# or: npx serve .
# or just open index.html directly
```

Open [http://localhost:8080](http://localhost:8080). That's it.

### Connect to a Vehicle

Click the **DISCONNECTED** indicator in the toolbar and enter a WebSocket URL:

```
ws://localhost:5760
```

The GCS defaults to **Meridian Native Protocol (MNP)** over WebSocket. For MAVLink v2 vehicles, use a MAVLink-to-WebSocket bridge.

### Demo Mode

Go to **Settings > Connection > Start Demo Mode** to explore the interface with simulated telemetry.

---

## Architecture

Single `index.html` with no build step, no npm, no framework. All JavaScript uses the IIFE module pattern with strict mode. CSS uses custom properties.

```
meridian-gcs/
  index.html          # Shell — toolbar, map, HUD, action bar, panels
  css/
    base.css          # Design tokens, grid layout, component standards
    theme-dark.css    # Dark theme overrides
    instruments.css   # HUD instruments (ADI, compass, tapes, quick)
    toolbar.css       # Toolbar, telemetry strip
    actions.css       # Action bar, arm slider, mode buttons, KILL
    map.css           # Leaflet overrides, vehicle icon, markers
    a11y.css          # Focus rings, reduced motion, touch targets, print
    [9 more panel CSS files]
  js/
    state.js          # Multi-vehicle state, event bus, message dispatch
    connection.js     # WebSocket manager, dual protocol (MNP/MAVLink)
    mnp.js            # Meridian Native Protocol: COBS + postcard codec
    mavlink.js        # MAVLink v2: CRC-X.25, frame parser, 15+ decoders
    router.js         # Panel switching, keyboard shortcuts
    fly/              # ADI, compass, tapes, map, quick, battery, wind, video
    plan/             # Mission, waypoint editor, survey, terrain profile
    setup/            # Calibration, frame, radio, motor test
    params/           # Parameter list, PID tuning, Betaflight import
    logs/             # Recording, playback, graph, inspector
    status/           # Raw telemetry view
  locales/            # i18n string files
  tests/              # Unit tests (MAVLink codec, MNP codec, validator)
```

### Design Principles

1. **No build step.** Open `index.html` and it works. Anywhere. Forever.
2. **Pilot-first.** Every pixel serves the operator. Information density over whitespace.
3. **Protocol-native.** Meridian Native Protocol is the primary transport. MAVLink is supported for legacy compatibility.
4. **Touch-ready.** All controls meet 44px minimum height. Coarse pointer media queries bump to 48px.
5. **Theme-aware.** Dark and light themes cascade to every element including canvas instruments.
6. **Accessible.** Focus-visible rings, skip-to-content link, ARIA roles, reduced-motion support, screen reader landmarks.

---

## Browser Support

Any modern browser with ES2017+ support:

| Browser | Minimum Version |
|---------|----------------|
| Chrome / Edge | 60+ |
| Firefox | 55+ |
| Safari | 11+ |

No polyfills. No transpilation. No vendor lock-in.

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

Meridian GCS welcomes contributions of all kinds — bug fixes, new instrument widgets, protocol adapters, accessibility improvements, translations, documentation.

---

## License

[MIT](LICENSE) — Use it, fork it, fly with it.
