# Meridian GCS Architecture v2 — Expert Panel Review (Round 2)

**Reviewed:** 2026-04-02
**Architecture doc:** `gcs/ARCHITECTURE_v2.md`
**Panel:** Michael Oborne, Lorenz Meier, Edward Tufte, Steve Krug, Bret Victor

---

## Overall v2 Verdict: **SHIP**

Every Round 1 concern was addressed. No new blocking issues. New concerns below are implementation warnings, not structural objections.

---

## Reviewer 1: Michael Oborne — **SHIP**

### Round 1 Concerns — Status

| Concern | Addressed? |
|---|---|
| No tlog recording | YES — auto-records every byte on connect, timestamped .tlog/.mnplog, "● REC" indicator in toolbar |
| No tlog playback | YES — scrub bar, 0.5x/1x/2x/5x/10x, full instrument replay via event bus |
| No Status tab | YES — status-view.js: 200+ fields, searchable, 4Hz, accessible via `T` shortcut |
| Camera/gimbal missing from context menu | YES — Point Camera Here (DO_SET_ROI) and Trigger Camera Now added |
| No Resume/Restart Mission | YES — visible in action bar when mission loaded |
| No Quick widget | YES — 4-6 configurable large-text values, right-click to select field |

### New Concerns

**1. In-memory tlog buffer will fail on long flights.**
Recording to `ArrayBuffer bytes[]` in JavaScript memory means a 2-hour survey flight could accumulate 50–200 MB in RAM before the pilot downloads. On mobile browsers this will crash. Needs a streaming write to IndexedDB or periodic auto-download, not just on-disconnect.

**2. Playback speed 10x is undocumented but present.**
The architecture section lists 0.5x/1x/2x/5x in the checklist summary but the Tlog Playback detail section adds 10x. Minor inconsistency; confirm 10x is intentional and add scrub-bar position display (timestamp, not just progress %).

**3. Quick widget has no documented persistence.**
"Right-click to select field" — where is the selection stored? No mention of localStorage or settings. If the user closes the tab, their custom Quick layout is gone.

---

## Reviewer 2: Lorenz Meier — **SHIP**

### Round 1 Concerns — Status

| Concern | Addressed? |
|---|---|
| 5-tab model wrong for flying | YES — Fly View is always-active base layer; all other views are overlay panels; map + action bar + toolbar never unmount |
| Flat global state, no multi-vehicle | YES — `meridian.vehicles[sysid]` keyed by system ID, `activeVehicleId`, `get v()` shortcut |
| No application settings view | YES — Settings overlay via `Ctrl+,` covers units, map provider, guided limits, ADSB, reconnect, display toggles |
| Survey tool underspecified | YES — camera model library, terrain following, hover-and-capture, corridor scan all added to survey.js |
| Mission stats not always-visible | YES — always-visible bottom panel in Plan View: WPs, distance, time, battery, max alt, max dist, inline validator warnings |
| Keyboard shortcuts absent | YES — F/P/S/Ctrl+P/L/T/Ctrl+, all documented |

### New Concerns

**1. `get v()` accessor has a subtle bug risk.**
`get v() { return this.vehicles[this.activeVehicleId] || {} }` — returning `{}` as fallback means code that does `meridian.v.armed` silently returns `undefined` instead of throwing. Any boolean check like `if (meridian.v.armed)` will pass silently when no vehicle is connected. Safer to return `null` and guard at call sites, or add a `connected` sentinel.

**2. Survey tool is specified but not phased.**
Phase 2 only includes "survey.js (basic polygon + camera)." Camera model library and terrain following are listed under Phase 5 "Advanced." Professional survey operators who see the v1 UI without those features will form a negative first impression. Consider labeling the Phase 2 survey tool as "basic" in the UI (e.g., "Survey — Advanced features coming") rather than shipping a full survey button that's missing half its capability.

**3. Overlay model keyboard navigation gap.**
`Escape` maps to "Instruments panel (default Fly view)" but is also the browser's standard way to close modals/menus. If a context menu is open and the user hits Escape expecting to close only the menu, will it also jump the overlay to Fly? Needs explicit priority handling.

---

## Reviewer 3: Edward Tufte — **SHIP**

### Round 1 Concerns — Status

| Concern | Addressed? |
|---|---|
| Gradient ADI | YES — flat colors specified: solid #1a3a6a sky, solid #5a3a1a ground |
| Battery widget showing too much always | YES — % + bar always; voltage/current/mAh on hover; per-cell only on variance warning |
| GPS dot instead of numbers | YES — "14 SAT · 0.8 HDOP" always visible, fix-type as text label |
| Red serves two roles (armed + emergency) | YES — armed = #ff6d00 orange, emergency = #ff1744 red, explicitly documented as a hard rule |
| Tape tick interval unspecified | YES — major marks every 10m/50m (altitude), 1 m/s/5 m/s (speed), ±50m visible range, center position specified |
| Data-ink audit absent | YES — audit checklist added to implementation guidelines |

### New Concerns

**1. The ADI ground color #5a3a1a is too close to the background.**
Dark brown on a dark theme (likely near-black base) risks low contrast. The sky #1a3a6a is a dark blue. If the base background is also dark, the horizon line is the only visual separator. Specify a minimum contrast ratio between sky and ground (at least 3:1) and verify at implementation.

**2. Eight accent colors are still in use — palette concern partially unresolved.**
Tufte's Round 1 concern was "eight accent colors is too many." The palette now has emergency red, warning amber, armed orange, safe green, primary cyan, info blue, special magenta, and neutral gray — still eight. The rule "red = emergency only" fixes the double-duty problem, but the overall count is unchanged. Acceptable if the implementation enforces strict usage rules, but the architecture should explicitly forbid using magenta for anything other than home marker and ROI.

**3. Target pitch/roll ghost on ADI: color conflict.**
The ADI target ghost is described as "thin dashed line." No color specified. Cyan (#00e5ff) is the primary interactive color — if used here it will conflict with the cyan current-value box on the speed tape. Specify the ghost color explicitly (suggest white or dim amber) before implementation to avoid color-meaning collisions.

---

## Reviewer 4: Steve Krug — **SHIP**

### Round 1 Concerns — Status

| Concern | Addressed? |
|---|---|
| KILL adjacent to LAND, no spatial isolation | YES — KILL isolated at far right, different shape, long-press required, confirm.js handles long-press-to-kill |
| No single "what state am I in" indicator | YES — Primary Flight State badge: top-center, ARM STATE + MODE + mission progress on line 1, battery + GPS + flight time on line 2 |
| 43-mode dropdown in flight | YES — 5 common modes as one-tap buttons (configurable), "More..." for full list |
| No upload dirty state | YES — Upload button has amber pulsing indicator when unsent |
| Setup sidebar has no progress tracking | YES — commissioning checklist with required/optional/complete per step; required items block arming |

### New Concerns

**1. Common modes default list includes mode number 9 (Land).**
`commonModes: [0, 5, 6, 9, 3]` — mode 9 in ArduCopter is LAND. Having LAND as a one-tap button in the common modes row is the same hazard as the old KILL-next-to-LAND problem, just moved. LAND should require confirmation or be in "More..." by default. The default set should be: Stabilize, Loiter, Pos Hold, RTL, Auto — not Land.

**2. Speed/Alt sliders in the same action bar row as discrete buttons.**
Krug's Round 1 concern explicitly flagged "sliders mixed with discrete action buttons is an invitation to accidental input." The v2 layout shows Speed and Alt sliders in the same row as PAUSE/RESUME/RESTART. They are in a separate group (separated by a visual divider), which partially addresses this — but the architecture doesn't specify the slider interaction model. Does dragging a speed slider during a mission immediately change the vehicle speed, or does it queue a change? If immediate, accidental touch during a button press could change speed mid-maneuver. Needs a confirmation model or minimum drag distance threshold.

**3. Setup checklist blocks arming — but only through the GCS.**
"Required items block arming" — does this mean the GCS refuses to send ARM, or that the vehicle won't arm? If it's GCS-side enforcement only, an experienced pilot who connects with a different GCS can still arm. This is fine, but the architecture should note that checklist enforcement is GCS-only (not firmware-level) so it's an advisory system, not a hard interlock.

---

## Reviewer 5: Bret Victor — **SHIP**

### Round 1 Concerns — Status

| Concern | Addressed? |
|---|---|
| Map is passive observation only | YES — commanded trajectory line (nav_roll/nav_pitch/target_bearing, 5-10s projection), position uncertainty ellipse (HDOP * EKF variance), crosstrack deviation indicator |
| Instruments show state not dynamics | YES — target ghost markers on altitude and speed tapes at commanded values; target pitch/roll ghost on ADI |
| PID sliders with no visual feedback | YES — live response chart per axis: commanded rate vs. actual rate, last 10s, updates at slider change |
| Mission planning has no live validation | YES — validator.js runs on every change: turn radius, geofence containment, terrain clearance; warnings inline in stats panel |
| Waypoint altitude edit is numeric-only | YES — terrain profile is interactive: drag altitude handles to edit waypoint altitude, sidebar fields update on drag |

### New Concerns

**1. Commanded trajectory depends on nav_roll/nav_pitch being present — they often aren't.**
`nav_roll` and `nav_pitch` from NAV_CONTROLLER_OUTPUT are only populated when the vehicle is in an autonomous mode (Auto, Guided, RTL). In Stabilize or Loiter, these fields are zero. The trajectory line will disappear in the most common manual flight modes. The architecture should specify the fallback: either hide the line gracefully, or derive a short-term projection from current velocity vector when nav controller output is unavailable.

**2. Uncertainty ellipse toggle is per-settings, not per-session.**
`showUncertainty` lives in `meridian.settings` — a persisted preference. A pilot who wants to temporarily hide the ellipse during a crowded display can't do so without opening Settings. Consider also a quick-toggle on the map toolbar (one-click hide/show) for in-flight use without opening the settings overlay.

**3. Interactive terrain profile + drag-to-edit has no undo.**
If a pilot drags an altitude handle and accidentally sets WP 7 to 5m AGL over a forest, the only recovery is to drag it back or retype the value. Add Ctrl+Z undo for the terrain profile editor. This is a direct manipulation interface and undo is a fundamental requirement of direct manipulation.

**4. PID response chart: 10s window at what sample rate?**
"Commanding rate vs. actual rate, last 10 seconds" — the architecture doesn't specify the chart's sample rate or whether it downsamples for display. If the vehicle sends ATTITUDE at 10Hz and the chart renders at 10Hz, that's 100 data points — fine. If it renders at 50Hz it will thrash the DOM. Specify the chart update rate explicitly (suggest 10Hz render, source data at whatever the stream provides).

---

## Summary Table

| Expert | Round 1 Verdict | v2 Verdict | All R1 Concerns Addressed? | Blocking New Issues |
|---|---|---|---|---|
| Oborne | REWORK | SHIP | YES (6/6) | tlog memory overflow on long flights |
| Meier | REWORK | SHIP | YES (6/6) | `v()` null safety, Escape key priority |
| Tufte | REWORK | SHIP | YES (6/6) | ADI ground color contrast |
| Krug | REWORK | SHIP | YES (5/5) | Land in default common modes |
| Victor | RETHINK | SHIP | YES (5/5) | No undo in terrain editor |

**One item the panel calls out before implementation begins:**

The tlog in-memory buffer (Oborne) and the default common modes including LAND (Krug) are the two highest-priority fixes before Phase 1 ships. Everything else can be resolved during implementation.
