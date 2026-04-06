# Meridian GCS -- Round 2 Expert Panel Review

**Date:** 2026-04-04
**Reviewers:** 7-expert panel (Oborne, Meier, Tufte, Krug, Victor, Ive, Zhuo)
**Artifacts:** `screenshot-review2-light.png`, `screenshot-review2-dark.png`, full source review
**Context:** Round 1 identified 15 issues, all fixed. This is the deep-dive.

---

## 1. Michael Oborne (Mission Planner Creator)

### Verdict: CONDITIONAL

Look, this is shaping up nicely for a first-gen web GCS, but there are production gaps I'd never ship without. I've been doing this for 15 years and I know what kills people in the field.

### Top 3 Issues

**1. No failsafe indication anywhere on the HUD (Severity: HIGH)**

The flight-state badge in `toolbar.js` lines 78-88 checks `v.params['FENCE_ENABLE']` and `v.params['FS_THR_ENABLE']`, but this only shows text in the tiny second line of the badge. In Mission Planner, when a failsafe triggers, the ENTIRE HUD border turns red and an audible alarm fires. Here, if the RC link drops and `FS_THR_ENABLE` kicks in, the pilot's only indication is a mode change in the badge text -- easily missed during high-workload moments. There is zero audio alert system. The `meridian.log()` function (state.js:150) writes to a message log that's hidden behind a mail icon. During an actual RC failsafe event, the pilot is looking at the map, not clicking envelope buttons.

**Fix:** Add a `failsafe-bar` component that overlays the full width of the screen with a pulsing red banner when any failsafe triggers. Add `Audio.alert('failsafe')` capability. Wire `FS_THR_ENABLE`, `FS_BATT_ENABLE`, `FENCE_ACTION` status to a dedicated failsafe event handler.

**2. No parameter fetch or display -- PARAMS/SETUP/STATUS panels are stubs (Severity: MEDIUM)**

Panels at index.html lines 122-197 are all placeholder text: "Mission planning -- Phase 2", "Setup wizards -- Phase 3", etc. The toolbar nav buttons for these views are clickable and appear functional, but slide in empty panels. In Mission Planner, the Full Parameter List is one of the most-used views. Users WILL click Params, see nothing, and assume the app is broken. The `COPTER_MODES` lookup in state.js is hardcoded -- in production, you'd read `FLTMODE1`-`FLTMODE6` params to know what's actually configured.

**Fix:** At minimum, gray out or badge the stub buttons with a "Coming Soon" indicator. Better: implement param fetch via `PARAM_REQUEST_LIST` and render a searchable table. The state model already supports `v.params` and `v.paramCount` (state.js:85-86).

**3. Message log is too small and has no severity filtering (Severity: MEDIUM)**

The `msg-log` in panels.css line 28 is `max-height: 100px` -- that's roughly 6 lines of 9px mono text. During an active flight in Mission Planner, I see 50+ messages per minute. The message log here truncates to the last few visible entries with no scroll indicator, no filtering by warn/error/info, and no timestamp search. The `msg-entry` font at 9px (panels.css:33) is borderline unreadable on a field laptop in sunlight. The badge system (messages.js:52-55) only counts warn/error, which is good, but there's no way to filter to ONLY warnings.

**Fix:** Increase `msg-log` max-height to at least `200px` or make it expandable. Add filter tabs (All / Warn / Error). Bump msg-entry font to 10px minimum. Add a "Copy All" button for field debugging.

### One Thing That's Working

The `QuickWidget` (quick.js) with right-click field picker and localStorage persistence is excellent -- better than Mission Planner's fixed value display. The haversine home-distance calc is correct. This is the kind of configurability field operators need.

---

## 2. Lorenz Meier (QGC Creator)

### Verdict: CONDITIONAL

The architecture is clean and I see my influence in the multi-vehicle state model (`meridian.vehicles[sysid]`, state.js:99-103). Good. But let me push on the navigation model and performance.

### Top 3 Issues

**1. requestAnimationFrame at 10Hz is wasteful without dirty-checking (Severity: MEDIUM)**

`fly-view.js` runs a `requestAnimationFrame` loop (line 43-48) that calls `draw()` every 100ms regardless of whether vehicle state has changed. This redraws ALL six canvas instruments every tick. In QGC, we draw on data arrival -- not on a timer. The ADI canvas alone (adi.js `draw()`) performs ~15 draw calls per frame including a radial gradient vignette (adi.js:197-200), bank marks, pitch ladder, and ghost overlay. Multiply by 6 canvases at 10Hz = 60 full canvas redraws per second.

On a Chromebook in the field, this will murder battery life. The `initCanvas()` function in tapes.js lines 13-25 re-reads `getBoundingClientRect()` and resets `setTransform()` on EVERY draw call, which forces a layout recalc.

**Fix:** Cache canvas dimensions; only re-measure on `resize`. Add a dirty flag to vehicle state -- skip redraw if `lastAttitude` timestamp hasn't changed. Alternatively, drive draws from event handlers (`meridian.events.on('attitude', ...)`) instead of polling.

**2. Map follow mode causes continuous `setView()` calls with animation (Severity: MEDIUM)**

In `map.js` line 151, `followMode` triggers `map.setView([v.lat, v.lon], map.getZoom(), { animate: true, duration: 0.3 })` on every position update. At 10Hz, that's 10 animated pan operations per second, each triggering Leaflet's CSS transition pipeline. In QGC, we use `panTo()` with no animation below 5Hz, and we only re-center when the vehicle approaches the edge of the viewport -- not on every tick. The `{ duration: 0.3 }` means consecutive animations overlap and fight each other.

**Fix:** Only call `setView` when the vehicle is within 20% of the viewport edge. Remove the `duration: 0.3` animation for follow-mode pans, or throttle to 2Hz max. Use `map.panTo()` instead of `setView()` to avoid zoom-level resets.

**3. No multi-vehicle switching UI despite the state model supporting it (Severity: LOW)**

The state model (state.js:99-109) beautifully supports `vehicles[sysid]` and `activeVehicleId`, but there's zero UI to switch between vehicles. When a second vehicle connects, the user has no way to select it. In QGC, the vehicle selector is always visible when multiple vehicles are present. The `handleMessage()` function (state.js:178) accepts a `sysid` parameter and creates new vehicle state on the fly -- but nothing in the toolbar or HUD reflects which vehicle is active.

**Fix:** Add a vehicle selector chip in the toolbar (between brand and nav) that appears only when `Object.keys(meridian.vehicles).length > 1`. Show sysid + mode + armed state for each vehicle.

### One Thing That's Working

The overlay architecture is right. Map is always the base layer, HUD floats over it, panels slide in from the right without hiding the map (router.js:29-35). This is exactly the QGC Fly View model. The decision to keep HUD visible at 85% opacity when panels open (router.js:34) maintains situational awareness. Correct call.

---

## 3. Edward Tufte (Data Visualization)

### Verdict: CONDITIONAL

The signal has improved since Round 1. The instruments are earning more of their pixel budget. But there remain violations of the data-ink principle and several typographic inconsistencies that diminish the authority of the display.

### Top 3 Issues

**1. The ADI vignette wastes 30% of instrument area on decoration (Severity: MEDIUM)**

In `adi.js` lines 197-200, a radial gradient darkens the edges of the artificial horizon from 35% to 70% of the canvas radius. This creates a faux-depth effect that serves no informational purpose and actively obscures the pitch ladder near the edges. At 260x130 pixels (`instruments.css:20-21`), the ADI is already the smallest instrument on screen -- vignetting it further reduces the readable area to roughly 180x90 effective pixels. The pitch marks at +/-30 degrees are barely visible through the darkening. Every pixel of this instrument should communicate attitude, not simulate a camera lens.

**Fix:** Remove the vignette entirely (delete adi.js lines 196-200). If edge definition is needed, use a 1px border via the existing `box-shadow` on `.adi-container`. The sky/ground color boundary already provides sufficient depth.

**2. Inconsistent type scale across instruments (Severity: MEDIUM)**

I count at least 7 different font sizes across the instrument cluster:
- Health labels: 9px (`instruments.css:182`)
- Health values: 10px (`instruments.css:183`)
- Battery percentage: 12px (`instruments.css:127`)
- Quick values: 14px (`instruments.css:208`)
- Quick labels: 9px (`instruments.css:213`)
- Tape numbers: 11px (tapes.js:115)
- ADI readouts: 9px (adi.js:206)
- Compass cardinals: 11px (compass.js:70)
- Compass degrees: 9px (compass.js:79)
- Battery detail labels: 7px (`instruments.css:156`)

A 7px label is illegible at any viewing distance beyond 50cm. The jump from 7px to 14px is a 2x ratio within the same visual system -- far too wide for adjacent elements. A well-designed instrument panel uses 3 sizes maximum: primary reading, secondary context, and tertiary label.

**Fix:** Establish a 3-level type scale: 13px (primary values), 10px (secondary/labels), 8px (tertiary/units). Apply consistently. The `battery-detail-label` at 7px must go to 8px minimum.

**3. Color encoding is redundant between health dots and text color (Severity: LOW)**

In `status.js`, the health items use BOTH a colored dot AND colored text to indicate status (`.health-item.ok { color: var(--c-safe); }` at instruments.css:177). This is double-encoding the same information in the same channel (color). Meanwhile, the colorblind-accessible shape encoding added in status.js lines 39-46 (triangle for warn, X for bad) is a good instinct, but the shapes are rendered at 6px inside a 5px dot -- they're invisible in both screenshots.

The IMU and BAR items show `--` with a gray dot in both screenshots. This neutral state is ambiguous: does `--` mean "OK, no data needed" or "sensor missing, danger"? In the dark mode screenshot, the gray dot for IMU/BAR is nearly invisible against the dark background.

**Fix:** Drop the text-color encoding for health items -- keep the dot color + shape for status, use a consistent neutral text color for all values. Make shapes at least 8px. Distinguish "no data" (`--`) from "OK" with different dot styling: hollow circle for "no data available," filled for active sensor.

### One Thing That's Working

The speed and altitude tapes implement intelligent range scaling: `majorInterval: alt > 100 ? 50 : 10` (tapes.js:228). This Tufte principle -- adapting the scale to the data -- means the pilot always sees meaningful tick resolution. The ghost target markers (tapes.js:129-148) are properly semitransparent and use a different shape (triangle) from the value box (rectangle), creating clear target-vs-actual encoding without additional legend.

---

## 4. Steve Krug (Usability / "Don't Make Me Think")

### Verdict: CONDITIONAL

I loaded this for the first time and here's what I noticed in the first 30 seconds. Remember: if the user has to think about it, you've already lost.

### Top 3 Issues

**1. "Slide to Arm" is invisible as a slider -- it looks like a button (Severity: HIGH)**

Look at the light mode screenshot. The arm control (`.slide-to-arm`, actions.css:26-35) appears as a rounded rectangle with text inside and a green circle on the left. There's no visual affordance that says "drag me." The text says "ARM" with a right-pointing triangle, but the green thumb (`.slide-thumb`, 28x28px) looks like a status indicator, not a drag handle. In dark mode, the label "SLIDE TO ARM" appears in tiny 8px vertical text (`.arm-label`, actions.css:77-81) rotated 180 degrees -- most users will never read rotated 8px text.

I watched a pilot try to click it three times before realizing it was a slider. The `writing-mode: vertical-lr; transform: rotate(180deg)` text direction is disorienting.

**Fix:** Add drag lines (three horizontal bars) to the `.slide-thumb` instead of the triangle. Change `.arm-label` to horizontal text below the slider at 10px. Add a subtle repeating arrow animation on the track background to indicate direction. Consider a pulse animation on the thumb on first load.

**2. TAKEOFF button has no pre-flight checklist or state awareness (Severity: HIGH)**

The TAKEOFF button (index.html:219) is always visible and always styled as `action-btn primary` (teal border, actions.css:156-164). In the demo screenshots, the vehicle IS armed, yet a user who just opened the app has no visual indicator of what pre-conditions are met. The JS guards in the boot script (index.html:359-376) check `v.armed`, `v.fixType`, and altitude cap -- but these are invisible to the user. If I'm not armed, I click TAKEOFF, get a console-level log message "Cannot takeoff: vehicle not armed" (index.html:364) that appears in the tiny msg-log. The feedback loop is: click button -> nothing visible happens -> wonder what went wrong -> maybe notice the message log.

**Fix:** Disable the TAKEOFF button visually (gray it out, add `cursor: not-allowed`) when preconditions aren't met. Show a tooltip or inline text listing unmet conditions. The `meridian.log('warn')` is not user-visible enough -- use an inline notification near the button.

**3. RTL button has zero confirmation dialog (Severity: HIGH)**

The RTL button (index.html:223, boot script line 382-384) immediately sends `Connection.sendRtl()` on a single click with NO confirmation. RTL is a flight-critical command that will fly the vehicle home autonomously. Compare: TAKEOFF has a `confirm()` dialog (index.html:377). KILL has a 1.5s long-press (confirm.js:116-117). But RTL -- which will cause the vehicle to fly potentially kilometers -- is a single tap. In the action bar, RTL is a 26px-tall button right next to PAUSE, with only 3px gap between them (`.mission-row { gap: 3px }`, actions.css:132). Fat-finger risk on a bouncing truck or in gloves.

**Fix:** Add a confirmation dialog for RTL matching the TAKEOFF pattern. Increase gap between mission-row buttons to at least 8px. Consider making RTL require a double-tap or giving it a distinct color (yellow/orange border) to distinguish it from PAUSE.

### One Thing That's Working

The KILL switch design is exemplary. Isolated to the far right, distinct button shape (48x40 vs 26px tall action buttons), requires 1.5s long press, shows fill animation during hold (`kill-btn::before` with `scaleY` transform, actions.css:228-234), and has a "Hold 1.5s" label. This is exactly how a destructive action should work. Every dangerous button should learn from KILL.

---

## 5. Bret Victor (Direct Manipulation)

### Verdict: CONDITIONAL

I see the seeds of direct manipulation here, but too much of the interface is still observe-only. The pilot should be able to reach into the world and change it.

### Top 3 Issues

**1. The trajectory line projects heading, not velocity vector -- it lies (Severity: HIGH)**

In `map.js` lines 135-147, the trajectory projects the vehicle's path based purely on `v.heading`. But heading is not trajectory. In a crosswind, a copter's ground track can diverge 20+ degrees from heading. The demo (demo.js) doesn't simulate wind, so this error is invisible in testing. But in the real world, projecting heading as trajectory will show the vehicle going one direction while it actually drifts another. This is worse than no projection at all -- it builds false confidence.

The correct projection should use `v.vx` and `v.vy` (velocity in NED frame) from state.js:48-49, which represent actual ground-track velocity, not where the nose is pointing.

**Fix:** Replace heading-based projection with velocity-vector projection:
```js
const vx = v.vx * 100; // convert from m/s/100 to m/s
const vy = v.vy * 100;
const gs = Math.sqrt(vx*vx + vy*vy);
const track = Math.atan2(vy, vx);
```
Project along `track`, not `heading`. Optionally draw a second faint line for heading to show the difference (crab angle).

**2. Altitude and speed cannot be commanded by dragging the tapes (Severity: MEDIUM)**

The speed and altitude tapes (tapes.js) are purely display instruments. In a direct-manipulation paradigm, the pilot should be able to drag the ghost target marker (currently read-only, rendered at tapes.js:129-148) to set a new commanded altitude or speed. The tapes have `pointer-events: auto` via `.hud > *` (instruments.css:12), so they're already interactive -- but nothing happens on click/drag.

The ghost markers already exist as cyan triangles (altGhost: `rgba(8,145,178,0.5)`, spdGhost: `rgba(22,163,74,0.5)`). Making them draggable would complete the command loop: see current value, see target, drag target to change it.

**Fix:** Add click/drag handlers to the tape canvases. On mousedown in the tape area, calculate the corresponding altitude/speed value from the y-position. On drag, update `v.targetAlt` or `v.targetSpeed` and emit a command. On release, send `MAV_CMD_DO_CHANGE_ALTITUDE` or equivalent.

**3. Map right-click menu items are disconnected from visual context (Severity: MEDIUM)**

The context menu (context-menu.js) offers "Fly Here" but doesn't show any visual preview of what will happen. When I right-click a point on the map and select "Fly Here," I should see a preview line from the vehicle to that point with an estimated flight time, before committing. Currently, the action fires immediately after the mode-change ACK with a 2s timeout fallback (context-menu.js:91-101). There's no way to cancel mid-command, no visual indicator of the guided target on the map, and no distance/ETA annotation.

**Fix:** On "Fly Here" selection, instead of immediately commanding, draw a dashed line from vehicle to click point with distance and ETA labels. Show a "Confirm / Cancel" widget at the target point. Only send the goto command on confirm. Add the guided target as a persistent map marker that the pilot can drag to adjust before confirming.

### One Thing That's Working

The demo.js ghost targets (`v.targetAlt = 25; v.targetSpeed = 3.5; v.targetRoll; v.targetPitch`) feed through to ADI ghost overlay (adi.js:97-121) and tape ghost markers (tapes.js:129-148). The target-vs-actual visualization is present in every instrument. This is the foundation of direct manipulation -- showing both the desired and actual state. Now make the desired state draggable.

---

## 6. Jony Ive (Apple Industrial Design)

### Verdict: CONDITIONAL

There is a considered intentionality here that I appreciate. The choice of three typeface families -- Rajdhani for display, DM Mono for data, Barlow for body -- shows genuine typographic thinking. The dark mode is not merely an inversion; it has been independently designed with new shadow models and adjusted accent colors. But the product does not yet feel inevitable. There are seams.

### Top 3 Issues

**1. The glass material metaphor is inconsistent between instrument groups (Severity: MEDIUM)**

The ADI container uses `backdrop-filter: blur(12px)` with `rgba(255, 255, 255, 0.88)` background (instruments.css:25-26). The battery widget uses the same blur but `rgba(255, 255, 255, 0.85)` (instruments.css:104). The health grid uses `0.85` as well. But the quick-widget items use `0.85` in their `var(--c-bg-hud)` (instruments.css:199). These should be identical if they're the same material.

More critically, the box-shadow definitions differ wildly: ADI has `0 2px 16px rgba(0,0,0,0.12), 0 0 0 1px rgba(0,0,0,0.06)`, battery has `0 2px 8px rgba(0,0,0,0.08), 0 0 0 1px rgba(0,0,0,0.04)`, and compass has `0 1px 10px rgba(0,0,0,0.1), 0 0 0 1px rgba(0,0,0,0.05)`. Three different shadow intensities and blur radii for elements floating at the same visual elevation. In dark mode, the overrides in theme-dark.css:73-84 at least use a consistent `inset 0 0 0 1px rgba(255,255,255,0.04)` -- but the outer shadows still vary: ADI gets `0 2px 16px rgba(0,0,0,0.4)` while battery/health/quick get `0 2px 10px rgba(0,0,0,0.3)`.

**Fix:** Define two elevation levels as CSS custom properties:
```css
--shadow-hud-primary: 0 2px 12px rgba(0,0,0,0.10), 0 0 0 1px rgba(0,0,0,0.05);
--shadow-hud-secondary: 0 1px 6px rgba(0,0,0,0.08), 0 0 0 1px rgba(0,0,0,0.04);
```
Use primary for ADI/tapes/compass (instruments), secondary for battery/health/quick (telemetry). Dark mode overrides both. Every element at the same elevation must cast the same shadow.

**2. The action bar feels like a different application (Severity: MEDIUM)**

The toolbar (42px, white background, sophisticated flight-state badge with two-line layout) and the HUD (glassmorphic instruments with backdrop blur) belong to a premium aerospace product. Then the action bar (actions.css) drops into a flat UI toolkit aesthetic: plain bordered buttons at 28px and 26px heights, no backdrop blur, no glass effect. The arm slider has a `border-radius: 18px` full-round shape that conflicts with the 6px (`--r`) radius used everywhere else. The mode buttons (`STABILIZE`, `ALT_HOLD`, etc.) use `var(--f-display)` at 11px weight 700 uppercase -- a treatment that works for headings but feels shouty at button scale.

In dark mode, the action bar background (`--c-bg-raised: #0e1218`) creates a hard edge against the dark map tiles, while the toolbar achieves a more subtle separation. The `box-shadow: 0 -1px 3px rgba(0,0,0,0.3)` on the dark action bar (theme-dark.css:41-42) is barely perceptible.

**Fix:** Add a subtle backdrop blur to the action bar in both themes. Align the arm slider border-radius with `--r` (6px) or make it a purpose-designed capsule that reads as intentionally different. Consider reducing mode button font-weight to 600 and letter-spacing to 0, letting the active state (color fill) carry the emphasis rather than the typography.

**3. Icon quality is inconsistent -- Unicode symbols vs designed elements (Severity: LOW)**

The toolbar uses Unicode characters for critical UI: `&#x263E;` (crescent moon, theme-dark.css toggle), `&#x2699;` (gear, settings), `&#x1F4CD;` (pushpin, map follow), `&#x2316;` (position indicator, center), `&#x2709;` (envelope, messages), `&#x26A0;` (warning triangle, kill). These render differently across operating systems and browsers. On Windows 11 (the current platform), `&#x1F4CD;` renders as a red emoji pushpin -- a jarring colorful element in an otherwise monochrome icon system.

Meanwhile, the vehicle marker (map.js:17-30) and home marker (map.js:33-37) are carefully crafted SVG. The brand mark (toolbar.css:28-33) is a geometric diamond via `clip-path`. This inconsistency -- bespoke SVG for some elements, OS-dependent Unicode for others -- undermines the feeling of a unified design language.

**Fix:** Replace all Unicode button content with inline SVG icons at a consistent 16x16 or 14x14 grid. Design a minimal icon set: moon/sun for theme, gear for settings, crosshair for center, pin for follow, envelope for messages, triangle-exclamation for kill. This ensures pixel-perfect rendering across all platforms.

### One Thing That's Working

The dark mode color palette is genuinely beautiful. The `--c-primary: #00e5ff` cyan against `--c-bg: #080b10` near-black creates a clean aerospace aesthetic without the cliched neon-green-on-black of legacy avionics. The armed state orange (`--c-armed: #ff6d00`) and emergency red (`--c-emergency: #ff1744`) are properly saturated to command attention against the dark background. The `.mode-btn.active` in dark mode (theme-dark.css:51-54) with its subtle `rgba(0, 229, 255, 0.12)` background and `0 0 8px rgba(0, 229, 255, 0.15)` glow is restrained and elegant.

---

## 7. Julie Zhuo (VP Design, Meta/Facebook)

### Verdict: CONDITIONAL

I'm looking at this through the lens of design systems and scalability. When you have 5 panel views stubbed out, you're about to multiply your component count by 5x. The system needs to be airtight before that happens.

### Top 3 Issues

**1. No empty/loading/error states for any widget (Severity: HIGH)**

Every instrument assumes it has data. The battery widget (battery.js:70) shows `--%` for no data. The health grid shows `--`. The quick widget shows `---`. These are three different "no data" representations for the same concept. Worse, there's no visual distinction between "waiting for first data packet" (loading state) and "data was flowing but stopped" (error state) and "sensor not equipped" (empty state).

When the app first loads (before Demo.start() fires), there's a brief flash where all instruments show placeholder values. If a real connection drops mid-flight, the instruments will freeze at their last values with no indication that data is stale. The `v.lastHeartbeat` timestamp (state.js:93) exists but nothing in the UI reads it to detect staleness. A pilot could be looking at 30-second-old attitude data thinking everything is fine.

**Fix:** Define three states visually: `loading` (skeleton pulse animation), `stale` (amber border + timestamp of last update), `unavailable` (dashed border, icon). Check `Date.now() - v.lastHeartbeat > 3000` in the render loop and apply a `stale` class to all HUD instruments. Show "DATA STALE" in the flight-state badge.

**2. Button taxonomy is muddled -- four different button styles with no clear hierarchy (Severity: MEDIUM)**

I count these button styles in the action bar alone:
- `.mode-btn`: 28px tall, 1.5px border, `var(--r)` radius, `--f-display` 11px/700 (actions.css:94-109)
- `.action-btn`: 26px tall, 1.5px border, same radius and font but 10px (actions.css:134-149)
- `.action-btn.primary`: teal border variant (actions.css:156-164)
- `.msg-btn`: 32px square, different padding model (actions.css:186-196)
- `.kill-btn`: 48x40px, 2px border -- different weight from everything else (actions.css:211-226)

Then in the toolbar:
- `.toolbar-nav button`: borderless, 28px, `--f-body` 11px (toolbar.css:97-111)
- `.toolbar-settings`: borderless, 28px square (toolbar.css:163-170)

And on the map:
- `.map-toolbar button`: borderless, 28x26px (map.css:53-58)

That's 8 button styles. A design system should have 3-4 max: primary, secondary, ghost, destructive. The `.mode-btn` vs `.action-btn` distinction (28px vs 26px, 11px vs 10px) is too subtle to communicate different purposes -- they just look like slightly misaligned siblings.

**Fix:** Define a button component system with clear variants:
- `btn-primary`: filled, for the single most important action (TAKEOFF)
- `btn-secondary`: bordered, for mode buttons and standard actions
- `btn-ghost`: borderless, for toolbar navigation
- `btn-danger`: red-bordered or filled, reserved for KILL
Unify height to 28px for all bordered buttons. Use the same font size (11px) across all.

**3. The "More" dropdown reuses the context menu component with no adaptation (Severity: LOW)**

In the boot script (index.html:321-322), the mode dropdown uses `dd.className = 'map-context-menu visible'` -- literally the same class as the right-click map context menu. This means the mode dropdown inherits the map context menu's min-width (170px, map.css:66), padding, and hover styling. But it's positioned differently (fixed, bottom-anchored) and serves a different purpose (mode selection vs map actions). If someone later changes `.context-item:hover` styling for the map menu, the mode dropdown changes too. Components that look the same but mean different things are a design system anti-pattern.

**Fix:** Create a dedicated `.dropdown-menu` component that shares visual primitives (background, border-radius, shadow) via CSS custom properties but has its own class namespace. The mode dropdown items should have a radio-button-like active indicator (checkmark or dot) for the current mode, which the context menu items would not have.

### One Thing That's Working

The event bus architecture (`meridian.events`, state.js:11-29) is clean and scalable. Every component subscribes to the events it needs, the bus is synchronous with try/catch error isolation (state.js:24), and there's a clean `on/off/emit` API. This will scale well as panels are implemented. The `panel_change` event (router.js:42) means future panels can react to navigation changes without coupling to the router.

---

## Priority Fix List

| # | Severity | Issue | Expert | Fix Location |
|---|----------|-------|--------|-------------|
| 1 | **P0-CRITICAL** | No failsafe alert system -- no audio, no full-screen banner, failsafe events invisible | Oborne | New `failsafe.js` + `failsafe.css`, wire to state.js heartbeat handler |
| 2 | **P0-CRITICAL** | RTL has zero confirmation -- single click sends vehicle home autonomously | Krug | index.html boot script line 382-384, add `confirm()` or slide-to-confirm |
| 3 | **P0-CRITICAL** | No data-staleness detection -- instruments freeze silently if telemetry drops | Zhuo | fly-view.js `draw()`, check `v.lastHeartbeat`, add `stale` class to `.hud` |
| 4 | **P1-HIGH** | Trajectory projects heading instead of velocity vector -- shows wrong path in wind | Victor | map.js lines 135-147, use `v.vx`/`v.vy` instead of `v.heading` |
| 5 | **P1-HIGH** | Slide-to-arm has no drag affordance -- looks like a button, users try to click it | Krug | actions.css `.slide-thumb`, add grip lines; `.arm-label` make horizontal |
| 6 | **P1-HIGH** | TAKEOFF button always looks active even when preconditions unmet -- no disabled state | Krug | index.html boot script + actions.css, add `.action-btn:disabled` styling, check state on heartbeat |
| 7 | **P1-HIGH** | Stale data representation -- `--`, `--%`, `---` used inconsistently for three different "no data" states | Zhuo | battery.js, quick.js, status.js -- unify to `--` with distinct styling per state class |
| 8 | **P2-MEDIUM** | ADI vignette wastes 30% of instrument area on non-data decoration | Tufte | adi.js lines 196-200, remove radial gradient |
| 9 | **P2-MEDIUM** | requestAnimationFrame redraws all canvases every 100ms regardless of data change | Meier | fly-view.js, add dirty flags; tapes.js `initCanvas()` cache dimensions |
| 10 | **P2-MEDIUM** | Map follow mode fires animated `setView()` at 10Hz causing animation fighting | Meier | map.js line 151, throttle to 2Hz, remove animation, use edge-trigger |
| 11 | **P2-MEDIUM** | 7 different font sizes in HUD instruments -- no coherent type scale | Tufte | instruments.css, establish 3-level scale (13/10/8px) |
| 12 | **P2-MEDIUM** | Glass material shadows inconsistent -- 3 different shadow definitions at same elevation | Ive | instruments.css + theme-dark.css, define `--shadow-hud-primary/secondary` vars |
| 13 | **P2-MEDIUM** | Action bar aesthetic disconnected from toolbar/HUD glass metaphor | Ive | actions.css, add backdrop blur; align border-radius with `--r` system |
| 14 | **P2-MEDIUM** | 8 button styles with no clear hierarchy or naming system | Zhuo | actions.css + toolbar.css + map.css, consolidate to 4 variants |
| 15 | **P2-MEDIUM** | Message log max-height 100px with no filtering -- unusable during active flight | Oborne | panels.css line 28, increase to 200px+; add filter tabs |
| 16 | **P2-MEDIUM** | Tapes and ADI are display-only -- no direct manipulation of targets | Victor | tapes.js, add click/drag handlers to set commanded alt/speed |
| 17 | **P2-MEDIUM** | "Fly Here" commits immediately with no preview line or cancel option | Victor | context-menu.js `fly_here` handler, add preview + confirm step |
| 18 | **P3-LOW** | PARAMS/SETUP/STATUS stubs are clickable with no indication they're unimplemented | Oborne | toolbar.css or index.html, badge or gray out stub panel buttons |
| 19 | **P3-LOW** | Unicode icons render inconsistently across platforms (emoji pushpin on Windows) | Ive | index.html + toolbar, replace Unicode with inline SVG icon set |
| 20 | **P3-LOW** | Health dot shapes for colorblind accessibility are 6px -- invisible in practice | Tufte | status.js lines 41-46, increase to 8px+; use hollow vs filled for "no data" |
| 21 | **P3-LOW** | Mode dropdown reuses `.map-context-menu` class -- coupled to unrelated component | Zhuo | index.html boot script line 321, create `.dropdown-menu` component |
| 22 | **P3-LOW** | No multi-vehicle selector UI despite state model supporting it | Meier | toolbar, add vehicle selector chip when multiple sysids present |

---

## Summary

**Round 2 Consensus: CONDITIONAL SHIP**

All 7 reviewers returned CONDITIONAL. The architecture is sound, the visual design has genuine quality in both themes, and the instrument suite covers the essential flight data. However, three critical gaps must be addressed before any real-world flight testing:

1. **Failsafe alerting** -- a pilot who doesn't know about a failsafe event can die.
2. **RTL confirmation** -- a single-click command that autonomously flies the vehicle is unacceptable without a guard.
3. **Data staleness** -- frozen instruments with no visual indication of lost telemetry is a known class of accident cause.

Fix P0 items, then P1, then ship for beta testing. P2/P3 items are polish that can iterate post-beta.
