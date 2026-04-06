# Meridian GCS Fly View — Expert Panel Review

**Date:** 2026-04-04  
**Version:** Phase 1 Fly View  
**Panel:** Oborne, Meier, Tufte, Krug, Victor

---

## Reviewer 1: Michael Oborne — REWORK

### Top 3 Issues

**1. No tlog recording despite UI scaffolding for it.**
The rec-indicator element exists in the toolbar HTML. The `meridian.tlog` state object exists. But nothing actually records bytes. When a pilot connects and flies for 30 minutes, then loses the vehicle, they have zero data to investigate. This was my #1 concern in Round 1 and it's still not functional. The IndexedDB streaming approach was agreed upon — implement it before shipping.

**2. Health monitoring is superficial — IMU is hardcoded to "OK."**
`status.js`: `setItem('imu', 'OK', 'ok')` — this is always green regardless of vibration, temperature, or clipping. A real IMU health check needs accelerometer clipping count, gyro noise level, and temperature delta. Same issue for BAR ("OK" if altitude is nonzero, which is a meaningless check). These give false confidence. Either show real data or show "--" with a gray dot, never fake green.

**3. No parameter awareness for context.**
The GCS shows no fence status, no failsafe configuration, no RTL altitude. A pilot looking at this Fly View has no idea if a geofence is set, what happens on RC loss, or where RTL will take the vehicle. Mission Planner shows FS_THR_ENABLE, FENCE_ENABLE, RTL_ALT prominently. Add a "Safety" line to the flight state badge or health grid.

### What's Working
The map-dominant layout with floating HUD is the right call. The instrument cluster (ADI + tapes + compass) as overlays is exactly how QGC does it and it's correct. The dual-protocol support (MNP + MAVLink) in connection.js is well-structured.

---

## Reviewer 2: Lorenz Meier — SHIP (conditional)

### Top 3 Issues

**1. HUD dims to 40% opacity when any panel opens — kills situational awareness.**
`router.js`: `if (hud) hud.style.opacity = '0.4'` — when a pilot opens the Plan view to check a waypoint, their attitude/altitude/speed instruments become nearly invisible. This is dangerous. QGC keeps instruments fully visible always. Either keep the HUD at full opacity, or dim it to 0.85 at most, or only dim the parts behind the panel.

**2. Vehicle marker transform accumulates.**
`map.js`: `iconEl.style.transform += ' rotate(...)'` — this string concatenates on every position update (10Hz). After 10 minutes that's 6,000 appended rotation strings. The browser may or may not optimize this, but it's a DOM performance hazard. Set the SVG's transform attribute directly or use a CSS class.

**3. Keyboard shortcut `Ctrl+P` collides with browser print.**
Pressing `Ctrl+P` is supposed to open Params but will trigger the browser's print dialog. Remap Params to something else.

### What's Working
The state architecture (`meridian.vehicles[sysid]`) is correct from day one. The overlay navigation model is right — Fly View as persistent base with panels sliding in is how QGC evolved. The event bus is clean.

---

## Reviewer 3: Edward Tufte — REWORK

### Top 3 Issues

**1. Seven instances of 7-8px labels are below the threshold of legibility.**
`.tape-label` (7px), `.quick-label` (7px), `.kill-label` (7px), `.arm-label` (7px), `.takeoff-alt-unit` (7px), `.battery-detail-label` (7px), `.health-label` (8px). At standard viewing distances, 7px text on 96 DPI renders at ~5.25 physical points — below WCAG minimum. Raise all labels to minimum 9px or remove them entirely.

**2. Speed tape renders negative values (-1.0, -2.0 m/s).**
Speed is a scalar magnitude — it cannot be negative. Showing -2.0 m/s is physically meaningless and erodes trust in all data. Clamp at 0.

**3. Color as sole channel for health status.**
Health grid encodes ok/warn/bad exclusively through color (green/amber/red). 8% of males are colorblind. Add a second channel: text labels or shape differences.

### What's Working
GPS display follows "numbers always" correctly — "14/0.8" is immediately parseable. Battery tiered disclosure is correct. ADI data-ink ratio is clean.

---

## Reviewer 4: Steve Krug — SHIP (conditional)

### Top 3 Issues

**1. KILL button sends `sendDisarm()` — this is NOT emergency kill.**
`confirm.js`: the KILL long-press calls `Connection.sendDisarm()`. Emergency kill should send MAV_CMD_DO_FLIGHTTERMINATION (cmd 185) which immediately stops all motors regardless of flight state. A pilot who long-presses KILL during a flyaway expects instant motor shutoff, not a polite disarm request the FC might reject because it's in flight. **Safety-critical bug.**

**2. TAKEOFF has zero confirmation or prerequisite checks.**
Pressing TAKEOFF immediately sends the command. No check for: is the vehicle armed? Is GPS adequate? Is altitude reasonable (could type 999)? QGC shows a confirmation slider. A one-tap takeoff with freeform altitude input is an accident waiting to happen.

**3. "Fly Here" uses a setTimeout race condition.**
`context-menu.js`: `sendSetMode('GUIDED'); setTimeout(() => sendGoto(...), 200)`. Assumes mode change completes in 200ms. On laggy connection, GOTO arrives before GUIDED is active and gets silently rejected. Wait for `command_ack`.

### What's Working
Slide-to-arm is intuitive and prevents accidental arming. KILL switch isolation (far right, different shape, long-press) is correct pattern. Flight state badge placement correctly prioritizes "what state is my vehicle in?"

---

## Reviewer 5: Bret Victor — RETHINK (partial)

### Top 3 Issues

**1. Ghost target system exists but is never populated.**
ADI supports `targetRoll`/`targetPitch`, tapes support target altitude/speed. But demo never sets them — they're all `null`. The entire target-vs-actual system is invisible. Set demo targets: `targetAlt = 25`, `targetSpeed = 3.5`, oscillating `targetRoll`/`targetPitch`.

**2. No commanded trajectory on the map.**
Architecture specifies "commanded trajectory line: short projected path (5-10s)." Not implemented. Even in demo, project a 5-second velocity vector. This transforms the map from "where is it" to "where is it going."

**3. No direct manipulation anywhere.**
Every interaction is indirect: type number, press button, wait. No drag-to-reposition, no drag altitude handles. At minimum: let pilot drag vehicle icon on map to set GUIDED target.

### What's Working
ADI vignette provides genuine depth. Tape architecture with ghost markers is correctly designed. Event-driven architecture makes trajectory rendering trivially addable.

---

## Priority Fix List

| # | Severity | Issue |
|---|----------|-------|
| 1 | CRITICAL | KILL sends disarm, not flight termination |
| 2 | CRITICAL | TAKEOFF has no confirmation or checks |
| 3 | HIGH | HUD dims to 40% when panels open |
| 4 | HIGH | IMU/BAR health hardcoded to OK |
| 5 | HIGH | 7px labels throughout (legibility) |
| 6 | HIGH | Speed tape shows negative values |
| 7 | MEDIUM | Fly Here race condition (setTimeout) |
| 8 | MEDIUM | Vehicle transform string accumulates |
| 9 | MEDIUM | Demo doesn't set ghost targets |
| 10 | MEDIUM | No commanded trajectory on map |
| 11 | MEDIUM | Color-only health encoding |
| 12 | LOW | Ctrl+P collides with browser print |
| 13 | LOW | No tlog recording implementation |
| 14 | LOW | No safety parameter context |
| 15 | LOW | Dark theme CSS doesn't cascade |
