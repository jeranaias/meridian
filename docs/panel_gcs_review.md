# Meridian GCS Architecture — Expert Panel Review

**Reviewed:** 2026-04-02  
**Architecture doc:** `gcs/ARCHITECTURE.md`  
**Panel:** Michael Oborne, Lorenz Meier, Edward Tufte, Steve Krug, Bret Victor

---

## Reviewer 1: Michael Oborne
*Creator and lead developer, Mission Planner. Maintaining it since 2010.*

### Verdict: REWORK

You've read my source code. I can tell because you got the 15 HUD indicators right and you have the right tab count. That's flattering. But reading code and shipping a GCS that pilots will trust with a $3,000 aircraft at 400 feet are two different things, and this architecture document reveals several places where the gap will hurt you.

---

### Top 5 Concerns

**1. No tlog (telemetry log) recording.**

This is not a logs view gap — this is a fundamental missing feature. When Mission Planner users come screaming to the forum after a crash, the first question is always "do you have the tlog?" The .tlog is the continuous MAVLink recording that runs from connect to disconnect, regardless of arming state. It is the black box. Your architecture shows a Logs view that lets users download DataFlash logs from the vehicle — fine — but there is no mention anywhere of the GCS automatically recording every byte of incoming MAVLink to a timestamped .tlog file on disk. Without this, every crash investigation starts blind. This is not optional. Mission Planner logs everything by default. Pilots expect it.

**2. No telemetry log playback.**

Closely related to the above: MP's tlog playback is one of its most-used features. A pilot lands, opens MP, loads the tlog, and watches the HUD replay exactly what happened. The instruments animate, the map replays, the Status tab updates in real time. It's how instructors debrief students. It's how pilots figure out what went wrong on a bad flight. Your architecture has no playback capability at all. The log-graph.js will let you look at a downloaded DataFlash log, but that's not the same as watching the full telemetry replay. This omission will frustrate every pilot who tries to debug anything.

**3. The Status tab is completely absent.**

This is the most-used tab in Mission Planner for any pilot doing setup or debugging. It is a scrollable key-value list of every parameter in CurrentState — roll, pitch, yaw, all 16 RC inputs, all 32 servo outputs, 16 ESC telemetry channels per motor, battery_cell1 through battery_cell14, wind_dir, wind_vel, glide_ratio, xtrack_error — over 200 fields. Your architecture replaces this with a "system health grid" showing six binary green/red indicators. That's fine for a first-year pilot flying in a park. It is useless for anyone doing a bench test, commissioning a new vehicle, debugging a compass calibration that keeps failing, or chasing an intermittent sensor fault. You need a raw telemetry dump somewhere. The MAVLink Inspector in the Logs view is close but it's MAVLink-frame oriented (show me ATTITUDE_QUATERNION), not pilot-parameter oriented (show me roll/pitch/yaw). They serve different purposes.

**4. The context menu in Fly View is missing camera operations.**

Your right-click menu has: Fly Here, Set Home, Add WP, Measure Distance, Set ROI. Mission Planner's fly-view context menu has all of that plus: Point Camera Here (DO_SET_ROI), Trigger Camera Now, and access to the Camera Overlap visualization. For anyone flying survey or precision agriculture work, the ability to right-click and immediately trigger the camera or point the gimbal is heavily used. You have DO_SET_ROI in plan view — good — but the real-time gimbal pointing during flight is a different workflow and belongs on the fly-view context menu. This gap will annoy the exact professional users who need a capable GCS.

**5. The action bar has no Resume Mission or Restart Mission.**

Your action bar has ARM, DISARM, TAKEOFF, RTL, LAND, PAUSE, KILL, speed slider, altitude slider. What it doesn't have is Resume Mission and Restart Mission. These are two of the most-used buttons in any real field operation. You arm, take off, run a survey in Auto mode, you hit a waypoint that's in a tree line and you pause, you go back 5 waypoints, you hit Resume. This is daily workflow. Your architecture supports Auto mode in the mode selector, but there's no UI pathway to restart from waypoint 1 or resume from current position. The mode selector puts the vehicle in Auto, but the vehicle needs to know *which* waypoint to execute from. That logic has to live somewhere in the action bar.

---

### Top 3 "This Is Good" Items

**1. Slide-to-confirm on every dangerous action.**

MP uses a simple dialog box for arm/disarm. Users have been accidentally arming vehicles on the bench for 15 years because they misclicked. Your slide-to-confirm with an 85% threshold is the right call. The KILL SWITCH getting a double confirm is exactly correct — I've seen two people kill flying aircraft by accidentally clicking that in MP.

**2. Pre-arm failure messages highlighted in the message drawer.**

In MP, pre-arm failures appear in the Messages tab and also briefly on the HUD, but they get mixed in with everything else. You've made pre-arm failures a separate highlighted category. Good. That's a real improvement over MP's approach where new pilots often miss the "PreArm: Compass variance" message buried in a wall of INFO messages.

**3. The battery widget showing cells individually.**

Voltage, current, mAh consumed, per-cell voltage — all in one widget. MP shows the individual cell data in the Status tab which most pilots never find. Having per-cell voltage visible by default will catch a weak cell before it causes a mid-flight shutdown. Every pilot who has lost a vehicle to a failing cell on a 4S battery will appreciate this.

---

### Specific Recommendations

- Add automatic .tlog recording: on WebSocket connect, open a file (timestamp + protocol), append every incoming byte, close on disconnect. Offer a download link in the Logs view.
- Add tlog playback: load a file, parse MAVLink frames, feed them through the existing event bus with speed controls (0.5x, 1x, 2x, 5x). The existing HUD instruments will animate for free.
- Add a Status/Telemetry tab: a scrollable key-value table from `window.meridian` state, updated at 4Hz, searchable. This is 40 lines of JS. There is no reason not to have it.
- Add Resume Mission and Restart Mission to the action bar. These should appear when the vehicle is in Auto mode or when a mission is loaded.
- Add camera trigger and gimbal point to the Fly View context menu.
- Add a "Quick" widget: 4–6 user-selectable large-text telemetry values for pilots who want to monitor non-default data at a glance. This is extremely popular in MP.

---
---

## Reviewer 2: Lorenz Meier
*Creator of QGroundControl and PX4 autopilot. Designed QGC's UX architecture from the beginning.*

### Verdict: REWORK

I appreciate that you studied QGC carefully. The .plan file format, the geofence architecture, the slide-to-confirm pattern — these are good choices and I'm glad they're being adopted. But I have significant concerns about the view architecture and some structural decisions that will create problems at scale.

---

### Top 5 Concerns

**1. The 5-tab navigation model is the wrong mental model for a flying tool.**

QGC started with a tabbed layout — I know because I built it that way. We moved away from it. The problem is that tabs create a binary choice: you are either flying or you are planning. But real operations aren't binary. A pilot in the field wants to check a parameter while on the Fly view. A new user wants to compare their mission against the live vehicle position. With tabs, they have to leave the Fly view to do anything else, losing situational awareness while the vehicle is in the air.

QGC's current architecture puts *everything accessible from the Fly view*: the parameter page is reachable via the ">" expand button on the toolbar without leaving Fly view. The plan view map is overlaid on the fly view map. Consider carefully whether five hard tabs is the right answer or whether Fly view should be the primary canvas with other views as overlays and drawers.

That said — I acknowledge that for a web GCS running in a browser, overlaid views are harder to implement than in a native Qt application. The five-tab structure may be a pragmatic compromise. If you keep it, the tabs must be reachable with a single key press (F1–F5 or similar) so a pilot can instantly jump back to Fly view during any setup operation.

**2. The state object is a global flat namespace and it will rot.**

`window.meridian` as a single flat object with 40+ fields is the simplest possible state architecture. It will work for Phase 1. It will become unmanageable by Phase 4. QGC uses a strongly-typed vehicle model with per-vehicle state objects (`Vehicle` class), and even that required significant refactoring over time. Your state has no concept of multiple vehicles. It has no concept of state history. When you add the second vehicle (and you will), you will either add `lat2`, `lon2`, `armed2` etc. to the flat object — which is how MP's CurrentState.cs ended up with `battery_voltage` through `battery_voltage9` — or you will have to refactor everything. Design the vehicle state as an array or map from vehicle ID to state from day one, even if you only support one vehicle now. The event bus should carry the vehicle ID on every event.

**3. No application settings view.**

QGC's Application Settings handle: telemetry stream rates, unit preferences (metric vs. imperial), map provider API keys, offline map cache, ADSB server address, guided flight altitude limits, preflight checklist enforcement. None of this exists in your architecture. Where does the user configure the units they want? Where do they set their preferred map provider? Where do they configure the auto-reconnect timeout? Where do they set the guided maximum altitude (a critical safety parameter)? These aren't advanced features — they're baseline expectations for any GCS. Without a settings view, all of these either get hardcoded or they live in the params view, which is the wrong place.

**4. The Survey tool is incomplete.**

Your survey.js covers the basic lawnmower grid: polygon, overlap %, altitude, speed, angle. QGC's survey implementation includes: camera-aware grid generation (enter camera make/model, sensor dimensions, focal length, desired ground resolution → GCS calculates grid spacing), hover-and-capture mode (multicopter stops at each trigger point), terrain following (AGL altitude derived from SRTM data per waypoint), and corridor scan for linear features (roads, pipelines). Your architecture says "Survey tool — polygon area scan with camera settings" but the camera settings section is critical and underspecified. Survey work is one of the primary commercial use cases. If the camera model library and terrain following are missing, professional survey operators will not use this tool.

**5. The Plan view sidebar is left-side only with no stats panel.**

QGC's Plan view has a right-side editor panel and a bottom stats/terrain panel. Your architecture puts everything in a left sidebar. The bottom panel for terrain profile and mission statistics (total distance, estimated flight time, batteries required) is documented as a feature in `terrain.js` but it's not clear how it surfaces. Mission statistics — total horizontal distance, estimated flight time, max telemetry distance — are planning critical. Pilots need to know if their mission exceeds battery capacity before they upload it to the vehicle. If this information is buried in the sidebar rather than always-visible at the bottom, pilots will miss it and land short of home.

---

### Top 3 "This Is Good" Items

**1. The file structure — one file per component — is correct.**

QGC's early architecture had monolithic source files. `MainWindow.cc` was over 5,000 lines at one point. Your insistence on one JS file per component with clear orchestrator files (fly-view.js, plan-view.js) is the right discipline. The build order in the architecture document is also correct: core infrastructure first, then Fly view, then connection, then everything else.

**2. The event bus design matches QGC's signal/slot architecture in spirit.**

Your pub/sub Events object is the vanilla JS equivalent of Qt's signal/slot system. Decoupling components via events is correct and will allow independent testing and replacement of individual components. The event names are well-chosen and cover the right lifecycle moments.

**3. MNP + MAVLink dual protocol support from the beginning.**

QGC's decision to be MAVLink-only has occasionally limited what Meridian's own protocol could do. Supporting both MNP natively and MAVLink as an adapter is architecturally correct for a GCS that is tightly coupled to a specific autopilot. The COBS codec work in mnp.js is nontrivial and getting it right at the start avoids protocol debt later.

---

### Specific Recommendations

- Add keyboard shortcuts to all tabs, documented in the toolbar tooltips. At minimum: `F` for Fly, `P` for Plan, `S` for Setup, `Ctrl+P` for Params, `L` for Logs.
- Refactor state to `window.meridian.vehicles = {}` keyed by system ID. Add `window.meridian.activeVehicleId`. Even with one vehicle this costs nothing and avoids the refactor wall later.
- Add a Settings view or at minimum a settings modal: unit system, map provider, guided altitude limits, reconnect behavior, ADSB server config.
- Expand the survey tool specification to include camera model library, terrain-following waypoint altitude adjustment, and hover-and-capture mode. These are not phase 2 features — they're required for commercial use.
- Make the mission stats panel (total distance, flight time, battery count) always visible at the bottom of the Plan view, not buried in a sidebar.

---
---

## Reviewer 3: Edward Tufte
*Professor Emeritus, Yale University. Author of* The Visual Display of Quantitative Information, *1983.*

### Verdict: REWORK

I will not discuss aesthetics. Dark backgrounds, cyan accents, monospace fonts — these are stylistic choices and not my concern. What I will discuss is whether this design will accurately communicate quantitative information to the person who must act on it.

It will not. Not in its current form.

---

### Top 5 Concerns

**1. The instrument layout violates every principle of small multiples.**

The Fly view allocates 35% of screen width to instruments, showing: an artificial horizon, a compass strip, an altitude tape, a speed tape, a vertical speed indicator, a battery gauge, GPS status, EKF health, RC RSSI, a system health grid, distance to waypoint, and flight time. That is twelve distinct information sources crammed into a 35% panel. The natural temptation when building this will be to make each instrument large and self-contained with its own frame, its own label, its own decorative tick marks. Resist this entirely.

Aviation instruments fail the data-ink ratio test badly. A traditional artificial horizon has a horizon line, a sky gradient, a ground gradient, a pitch ladder, bank angle marks, a fixed aircraft symbol, and often a decorative bezel ring. Of these, the information content is: "what is roll and what is pitch." Two numbers. Everything else — the gradients, the bezel, the chevrons — is chartjunk. Framing chrome, decorative borders, 3D effects on gauges, shadows on the speed tape are all noise. Every pixel spent on non-data ink is a pixel stolen from data.

The architecture lists "pitch ladder every 5°, bank marks at 10/20/30/45/60°, fixed aircraft symbol." This is acceptable. But I note that the architecture document does not specify whether the sky/ground will be rendered as a gradient fill or as flat color. Use flat color. Aviation authority documents prove that flat color ADIs have equal or better recognition performance compared to gradient ADIs, and they consume less visual processing bandwidth at glance rates.

**2. The battery widget is doing too much in one place.**

The architecture specifies the battery widget shows: "bar + voltage + current + remaining % + consumed mAh + per-cell voltage." That is six numbers plus a bar. For a status-check view (pre-flight, bench test), this density is appropriate. During active flight the operator needs exactly two battery values at a glance: remaining percentage and whether it is falling faster than expected. Voltage, current, and consumed mAh are fine as secondary information accessible on hover or tap. Per-cell voltage should appear only when a cell variance warning is active. A battery widget that shows six numbers at all times will train pilots to ignore it, because it always looks the same until it doesn't.

Consider: what does the battery widget look like normally versus what does it look like when a cell is failing? If they look similar, the design has failed.

**3. The system health grid gives binary indicators where gradients are needed.**

"IMU / Baro / Mag / GPS — green/yellow/red." This is better than nothing. But GPS quality is not a ternary state. The architecture already stores `gpsSats` and `hdop` in state — show them as numbers alongside the GPS indicator, not as a colored dot. "GPS: 14 sats, HDOP 0.8" contains orders of magnitude more information than a green dot. Similarly, EKF health is displayed as "EKF health with variance display" — good. But where in the layout does the variance number actually appear? If it is tooltip-only, it will never be seen. EKF variance is one of the most important pre-flight indicators for ArduPilot operators and should be readable at arm's length.

**4. The color palette encodes too many categorical distinctions.**

The architecture defines eight accent colors: Cyan, Green, Amber, Red, Magenta, Blue, and implicitly White and Dim. These are used for: connection type (Cyan = MNP), arming state (Red = armed), GPS OK (Green), warnings (Amber), danger (Red), home marker (Magenta), waypoints (Blue), mode display (Amber). 

Red serves double duty as "armed" and "danger." When the vehicle is armed and there is also a danger condition, how does the operator distinguish routine armed-state red from emergency-state red? When Magenta is used only for the home marker, it wastes a distinct color channel on a low-priority element. The color encoding needs a formal hierarchy: one color reserved exclusively for emergency conditions, one for warning, one for nominal-safe states, and position-coding for everything else. Eight accent colors in a single interface is typically too many.

**5. The altitude and speed tapes will have tick mark density problems.**

Vertical scrolling tapes are correct for altitude and speed — aviation has converged on this for good reason, as tapes show both current value and trend simultaneously. But "scrolling numbers" as the architecture describes is underspecified. At what interval do numbers appear? At what interval do tick marks appear? What font size? What is the tape's visible range? If the altitude tape shows 10m increments and the vehicle is at 120m AGL, the tape should not be showing 80m–160m with equal density ticks throughout. The current value should be at a visually distinct central position, major marks should be at round numbers, and the tape should widen its range when climb rate is high (anticipatory display). None of this specification exists in the architecture document.

---

### Top 3 "This Is Good" Items

**1. Message severity color coding is correctly hierarchized.**

"ERROR (red), WARNING (amber), INFO (cyan), DEBUG (dim)" — this is a correct severity encoding. Red for errors, amber for warnings, information in a visually lighter color, debug almost invisible. Many GCS implementations use the same color for INFO and WARNING. You have not made this mistake.

**2. DM Mono for all data values is correct.**

Proportional fonts are wrong for telemetry displays. When "1234" and "9876" occupy different widths, the pilot's eye has to re-learn the number's position on every update. Monospace fonts allow the eye to mask individual digit positions and detect changes by movement rather than reading. The choice of DM Mono is sound. The explicit specification that *all telemetry values* use DM Mono is the kind of disciplined rule that prevents the interface from accumulating exceptions over time.

**3. Waypoint color coding by command type in the Plan view is good data encoding.**

"NAV = blue, TAKEOFF = green, LAND = orange, RTL = magenta, LOITER = yellow" — this is categorical encoding by type, not by arbitrary assignment. A pilot scanning a complex mission can immediately identify the command type distribution without reading labels. This is the correct use of color as a data channel.

---

### Specific Recommendations

- Remove gradient fills from the artificial horizon. Use flat contrasting colors: sky solid blue, ground solid brown. Period.
- Battery widget: show percentage and a colored bar always. Show voltage/current/mAh as a secondary layer visible on hover. Show per-cell voltage only when cell variance exceeds threshold.
- GPS indicator: always show satellite count and HDOP as numbers, not just a colored dot. "GPS 14 / 0.8" in DM Mono takes 14 characters and provides vastly more information than a colored circle.
- Conduct a data-ink audit before implementation: for every SVG element in the instrument panel, ask "does removing this pixel reduce the data shown?" If no, remove it.
- Define and document the altitude tape's tick interval, visible range, and center position specification before writing `tapes.js`. Implement tape antibugging (emergency color change when altitude exceeds geofence ceiling) during the same pass.
- Reserve Red exclusively for emergency states. Use a distinct color (Orange, not Amber) for the armed-but-normal state, so that true emergency red stands out from routine red.

---
---

## Reviewer 4: Steve Krug
*Author of* Don't Make Me Think. *UX testing consultant.*

### Verdict: REWORK

I'm going to apply my "trunk test" first — the test where you imagine landing on a page with a bag over your head and asking three questions: Where am I? What can I do here? How do I get back?

For the Fly view: **Where am I?** In the GCS, apparently flying. **What can I do here?** There's an action bar at the bottom — I can see ARM and some other things. **How do I get back?** Back to what? Back to a safe state? The action bar has RTL and LAND but I'd have to understand the difference between them before I could confidently pick one.

That last question is the one that kills me. The Fly view's action bar has ARM/DISARM, TAKEOFF, RTL, LAND, PAUSE/BRAKE, and KILL SWITCH — all in a row. Each of these, if activated at the wrong moment, has a different consequence ranging from "vehicle hovers" to "vehicle falls out of the sky." There is nothing in the architecture that explains how these actions are visually distinguished from each other by severity. In a high-stress moment — unexpected behavior, degraded link, unexpected wind — a pilot's fine motor control decreases and their tunnel vision increases. They will misread or misclick. The interface needs to be designed for that moment.

---

### Top 5 Concerns

**1. The action bar has seven high-stakes actions in a flat list.**

ARM, TAKEOFF, RTL, LAND, PAUSE, KILL — these all sit in the same "action bar." What prevents a pilot from accidentally hitting KILL when they meant to hit LAND? The slide-to-confirm is good for ARM/DISARM/RTL/LAND/KILL — but the architecture also lists "PAUSE/BRAKE" which presumably doesn't require slide confirmation, and "Speed adjust slider" and "Altitude adjust slider" which are continuous inputs. Sliders mixed with discrete action buttons in the same bar is an invitation to accidental input.

The actions need to be visually grouped and spatially separated by danger level. KILL SWITCH should not be adjacent to LAND. It should be isolated — different position, different color, different interaction pattern. QGC puts the emergency stop in the flight readiness dropdown, not inline with other actions. That's the right call: make the nuclear option harder to reach accidentally, not just harder to confirm.

**2. No clear "what state am I in right now" indicator.**

When I land on the Fly view, I see a map, instruments, and an action bar. Where does it tell me, clearly and unambiguously: "Armed, in AUTO mode, executing waypoint 7 of 14"? The architecture mentions a "mode selector (grouped: Copter/Plane/Rover/Sub with all 43 modes)" in the action bar — that's the current mode display. But the armed/disarmed state, the current mission execution progress, and the connection status are scattered across the toolbar (status chips) and the instruments panel (various indicators). 

A pilot picking up this GCS for the first time should be able to read the current vehicle state — armed/disarmed, mode, mission progress, link quality — at a glance from one area of the screen. QGC solved this with the flight readiness indicator in the toolbar that says "Armed / Flying / Not Ready" in a single colored chip. Your architecture has similar status chips, but the architecture document doesn't make clear what they show, in what priority order, and whether they're the first thing the eye goes to.

**3. The mode selector with "all 43 modes" is a usability disaster in flight.**

"Mode selector (grouped: Copter/Plane/Rover/Sub with all 43 modes)" — this is a dropdown containing 43 items during flight? I understand why you want comprehensive coverage. But a pilot in the air should not be scrolling through a dropdown to find LOITER. In QGC, the most common modes (Stabilize, Loiter, RTL, Land, Auto) are reachable in two taps from dedicated buttons. The full 43-mode list should be available, but the common modes should be surfaced as one-tap shortcuts. The architecture should specify: what are the 5 most-used modes, and can they be reached in one action?

**4. The Plan view has no visual confirmation of mission upload status.**

The architecture specifies upload/download/save/load buttons in the Plan view. But what does the interface look like when a mission has been planned but not yet uploaded? What does it look like when upload is in progress? What does it look like when upload is complete? QGC highlights the Upload button when there are unsynced changes. This is critical for flight safety — a pilot who plans a mission, closes the sidebar, and then forgets to upload will execute whatever mission was previously on the vehicle. The "dirty" state of the mission (planned but not uploaded) needs to be visually obvious, not just a normal button.

**5. The Setup view's sidebar navigation has no progress indicators.**

The Setup view lists 12 sections: Summary, Frame Type, Accel Cal, Compass Cal, Radio Cal, Flight Modes, Failsafe, Battery Monitor, Motor Test, ESC Cal, OSD, Servo Output. For a new vehicle that needs to be commissioned, the user needs to work through all 12 of these in order. But the sidebar just lists them as navigation items with no indication of which are complete, which are required, which are optional, or what order makes sense. MP has the same problem — it's just a list of tabs. QGC's Summary page shows a red indicator on each block that hasn't been completed. Your architecture says "Summary — Vehicle type, firmware version, board type, serial number." That's not a progress tracker; it's just info. A pilot commissioning a new vehicle needs a checklist, not a navigation menu.

---

### Top 3 "This Is Good" Items

**1. Slide-to-confirm at 85% threshold is well-calibrated.**

This is not too hard (frustrating for legitimate operations) and not too easy (triggering on misclick). The 85% threshold means you must clearly intend to complete the drag. Good. The cancel button and overlay dim add a second bail-out path. This pattern is as close to correct as you can get for high-stakes touch interactions.

**2. Pre-arm failures highlighted and blocking ARM.**

The architecture explicitly blocks ARM when pre-arm checks are failing, and highlights the failures prominently. This is the right UX pattern: don't let the user do the dangerous thing, and tell them exactly why and what to fix. Many pilots have launched with compass variance or EKF not healthy because the failure message wasn't prominent enough. This design makes it impossible to ignore.

**3. Message drawer with auto-scroll-pause on hover.**

Auto-scroll that pauses when the user is reading is a thoughtful micro-interaction. It means that a critical error message that appears during a fast-scrolling message stream won't scroll away while the pilot is trying to read it. Small detail, correct behavior.

---

### Specific Recommendations

- Move KILL SWITCH out of the main action bar. Put it under a guarded location — a long-press on the ARM/DISARM button, or behind a dedicated isolated button at the far right of the toolbar with a different shape and color. It should require deliberate effort to reach during normal operation.
- Add a primary flight state badge: a large, prominent area (top-center or top-left) that shows ARM STATE + MODE in one glance. This is the single most important piece of information during flight and should have the most visual weight.
- Reduce the mode dropdown to a two-tier UX: "Common modes" (5 most-used, configurable) shown as one-tap buttons, with "More modes..." opening the full 43-mode list. Default common modes: Stabilize/Manual, Loiter/Position, RTL, Auto, Land.
- Add visual dirty-state indicator to Plan view: when the current plan differs from what's uploaded to the vehicle, the Upload button should be highlighted (amber background, pulsing, "Unsent" label). After successful upload, it should show "Uploaded" confirmation for 3 seconds.
- Redesign the Setup view Summary as a checklist with explicit required/optional/complete status per step. Required steps not completed should prevent the pilot from arming (or at minimum show a persistent pre-arm warning).

---
---

## Reviewer 5: Bret Victor
*Former Apple HID researcher. Creator of "Inventing on Principle" (2012) and "Learnable Programming" (2012).*

### Verdict: RETHINK

I want to be clear about what I mean by RETHINK. I don't mean the architecture is wrong — the feature coverage is thoughtful and the component decomposition is sound. I mean the architecture is solving the wrong problem. It is building a better dashboard for a vehicle that the operator cannot directly touch. A great dashboard is not the same as direct manipulation.

The fundamental question is: can a pilot understand what the vehicle is doing, and *why*, by looking at this interface? Not "can they read the telemetry values" — that's just observation. I mean: can they see the relationship between the inputs they're giving and the outputs the vehicle is producing? Can they feel the vehicle responding?

The answer, based on this architecture, is no.

---

### Top 5 Concerns

**1. The map is mostly passive observation.**

Your map.js supports click-to-fly-here and right-click context menu. That's the extent of direct manipulation on the primary display. The vehicle is a dot on a map, and the pilot watches it move. There is no feedback on the map showing: where is the vehicle *trying* to go? If the vehicle is in auto mode executing a waypoint, what path is it actually computing in real-time vs. the planned path? Is it fighting crosswind and compensating? Is EKF drifting it off-course? None of this is visible.

A direct manipulation map would show, in real time: the vehicle's current estimated position with uncertainty ellipse, the vehicle's *intended* flight path (the controller's commanded trajectory, not just the planned waypoints), and the deviation between the two. When the pilot sees the vehicle drifting from its intended path, they should be able to drag a *live target* on the map to redirect it, not just click a point and watch the vehicle figure it out.

This is not a luxury feature. This is the difference between a pilot who understands what their vehicle is doing and a pilot who is watching a video of their vehicle.

**2. The instruments show state, not dynamics.**

The artificial horizon shows current roll and pitch. The altitude tape shows current altitude. The speed tape shows current speed. None of these shows *why* these values are what they are, or *where they are going*. 

Consider altitude: the tape shows current AGL. But is the vehicle climbing toward a target altitude? If so, where is the target altitude marked on the tape? Is the vehicle decelerating as it approaches the target (showing it will stop near the target), or is it oscillating? The vertical speed indicator is a separate instrument that shows climb rate, but it doesn't show whether that climb rate is commanded or incidental. A pilot reading 2 m/s climb on the VSI doesn't know if that's the controller working correctly or the vehicle fighting a thermal.

Target state vs. current state separation is the core principle here. Show the target alongside the actual. Show the error as a gap. Make the controller's intention visible, not just its current output.

**3. PID tuning is sliders disconnected from immediate visual feedback.**

The architecture includes a tuning.js with "PID gain sliders: Rate Roll/Pitch/Yaw P/I/D, live-updating (write on slider change)." Sliders that write parameters on change — good. But what does the pilot see when they change P gain? The parameter changes. The vehicle... does something different. But the connection between "I moved this slider" and "this is what happened to the vehicle's response" is invisible unless the pilot happens to be watching the right instrument at the right moment.

A learnable PID tuning interface would show a live graph of the axis being tuned, with the commanded input and the actual response overlaid, updating as the pilot moves the slider. The pilot would see: "my P gain is too low — the response is sluggish and undershooting." They would increase P and immediately see the response become crisper on the same graph. The causal connection would be visible. Without this, PID tuning is guesswork.

**4. Mission planning has no "what will this actually look like" preview.**

The plan view lets pilots build missions by clicking waypoints, configuring commands, and drawing geofences. The terrain profile chart is a good step. But there is no simulation of what the vehicle will actually do when it executes this mission.

What if the vehicle is in Stabilize, not Auto, when the mission starts? What if the geofence is smaller than the mission extent? What if the vehicle can't make the turn between waypoint 3 and waypoint 4 at the configured speed? None of these failure modes are visible before upload. A planning environment with direct feedback would validate the mission in real time: "warning — at 15 m/s, the vehicle cannot make this turn radius at waypoint 4. Reduce speed to 8 m/s or increase turn radius." The data is available. The geometry is calculable. The feedback is currently absent.

**5. Waypoint editing is numeric-first instead of spatial-first.**

The plan view waypoint editor shows "command type dropdown, lat/lon/alt fields, param fields." The operator types numbers into boxes to configure a waypoint. But the waypoint represents a spatial location — a place in 3D space that the vehicle will fly to. The most natural way to edit a spatial thing is to manipulate it spatially.

Can the pilot drag a waypoint vertically on the terrain profile chart to change its altitude? The architecture shows `terrain.js` generates a profile but there's no mention of the profile being interactive. Can the pilot grab an altitude and move it? Can they see the safety margin above terrain as they do? Currently, editing the altitude means typing a number in a field, and then checking the terrain profile separately. These should be the same action.

---

### Top 3 "This Is Good" Items

**1. Click-to-fly-here in Fly view is the right starting point.**

This is direct manipulation: the pilot touches a location on the map and the vehicle goes there. It is immediate, spatial, and causal. The right-click context menu extending this with Set ROI and Measure Distance follows the same principle. This pattern should be expanded, not treated as a complete implementation.

**2. Drag waypoints to reposition in Plan view.**

Typing coordinates to position a waypoint would be a crime. The plan view correctly makes waypoints draggable on the map. This is the spatial editing that waypoints demand. Extend this principle: can the waypoint's altitude be edited by dragging it on the terrain profile chart? Same concept, different axis.

**3. MAVLink inspector with live rates.**

This is one of the few places in the architecture where the operator can see the *dynamics* of the system, not just its current state. Message rates, field values updating in real time — this is a window into what the vehicle is actually communicating. A pilot debugging a sensor issue can watch the ATTITUDE message rate drop to zero and know the IMU is disconnected. This is learnable because it makes the invisible (MAVLink traffic) visible.

---

### Specific Recommendations

- Add a live commanded trajectory to the fly view map: a short projected path line showing where the current controller setpoints will take the vehicle in the next 5-10 seconds. This is distinct from the planned waypoints and requires reading `nav_roll`, `nav_pitch`, and `target_bearing` from state.
- Show target vs. actual on all tape instruments: a ghost marker on the altitude tape at the commanded altitude, a ghost on the speed tape at the commanded speed. When the vehicle reaches target, the ghost and the value coincide. The gap IS the error. Make the error visible spatially, not numerically.
- Make the terrain profile in the Plan view interactive: the pilot should be able to drag altitude handles on the profile chart, and the waypoint editor fields should update to match. The profile chart and the sidebar should be two views of the same data, not two separate things.
- On the PID tuning panel: add a live response chart per axis. Commanded rate vs. actual rate, updating at 10Hz, showing the last 10 seconds. The pilot changes the slider and immediately sees the effect on the chart. This is "Inventing on Principle" applied to drone tuning.
- Show uncertainty: the vehicle position dot on the map should optionally display an accuracy circle based on GPS HDOP and EKF position variance. A large circle means "the vehicle could be anywhere in here." A small circle means "we know exactly where it is." This is the most honest thing a map can show.
- In the Plan view, add a mission validator that runs on every change: checks turn radius vs. speed vs. waypoint spacing, checks geofence containment, checks terrain clearance, shows warnings inline before upload. Make planning safe by default, not just possible.

---

*End of panel review.*
*All five reviewers reviewed the same document: `gcs/ARCHITECTURE.md` as of 2026-04-02.*
