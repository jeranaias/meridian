# Meridian GCS Master Review Report

**45 Expert Persona Reviews | Compiled 4 April 2026**

---

## PART 1: EXECUTIVE SUMMARY

### Overall Scores by Category

| Category | Reviewers | Avg Score | Range |
|---|---|---|---|
| **Pilots (Commercial/AG/SAR)** | Maria Chen, Jake Morrison, Tommy Nguyen | 5.3 | 4-7 |
| **Consumer/Prosumer Pilots** | Sarah Blackwood, Lisa Chang, Nina Volkov | 3.7 | 2-5 |
| **Regulators** | Dr. Hayward (FAA), Cmdr Wu (NTSB), Dr. Schreiber (EASA) | 4.0 | 3-6 |
| **Military/Defense** | SSgt O'Brien, Col. Frost | 5.6 | 5-6.2 |
| **Engineers (Frontend/RT/Perf)** | Amir Patel, Dr. Tanaka, Olga Petrov | 7.0 | 6.5-7.5 |
| **Security** | Lena Kovacs | 4.0 | 4 |
| **Designers (HMI/UX/Data)** | Karim Al-Rashid, Dr. Park, Alex Navarro, Mei Lin, Jordan Reeves | 6.5 | 6.5-6.5 |
| **General Public** | Jayden Park, Harold Jennings, Marcus Rodriguez | 5.2 | 3-7.5 |
| **Competitive (QGC/MP/DJI)** | Dmitri Volkov, Brad Thompson, Lisa Chang | 5.5 | 5-6.5 |
| **Business/Enterprise** | Michael Brennan (VC), Priya Sharma, Wei Zhang | 6.3* | 3-8 |
| **Technical Integration** | Tridge (MAVLink), Vladimir (Leaflet), Lea Verou (Canvas), Randy Mackay (ArduPilot) | 6.3 | 5-8 |
| **Edge Cases** | Greg Holloway (colorblind), Ana Delgado (tablet), Chris Dawson (lost vehicle), Takeshi Yamamoto (i18n), Maya Chen (motor impairment) | 3.5 | 2-5 |
| **Domain Specialists** | Dr. Osei (wildlife), Erik Lindqvist (offshore), Sam Nguyen (delivery), Capt. Torres (training) | 5.3 | 4-6 |
| **Open Source** | Kris Kowal, Prof. Russo | 6.0 | 5-7 |
| **Accessibility** | Devon Williams | 2.5 | 2.5 |
| **Insurance** | Rachel Torres | 8.0 | 8 |

*Brennan scored as Watch (no numeric), average uses Sharma + Zhang only.

**Grand Mean: 5.3 / 10** (across all 44 numeric scores)

---

### Top 10 Most Critical Findings

1. **KILL command silently sends soft disarm instead of flight termination** (Tridge #31) -- `MAVLink.encodeCommandLong` is not exported; the kill path always falls through to `encodeDisarm()`. A pilot pressing KILL during an emergency gets a soft disarm, not motor cut. **Severity: life-safety.**

2. **No Remote ID implementation whatsoever** (Hayward #6, Schreiber #9) -- Zero code for FAA Part 89 broadcast RID or EU OPEN_DRONE_ID. Flying with this GCS violates federal law in the US and EU as of September 2023. **Severity: legal showstopper.**

3. **STATUSTEXT (msg 253) completely absent** (Tridge #31, Mackay #38) -- ArduPilot's primary operator communication channel (pre-arm failures, EKF warnings, failsafe reasons) is never decoded or displayed. Pilots fly blind on FC-internal reporting. **Severity: safety-critical.**

4. **MISSION_ITEM_INT field offset bug** (Tridge #31) -- `command` is read at byte 26 instead of byte 30. Missions downloaded from the FC will have corrupted command fields. **Severity: mission-corrupting.**

5. **Failsafe alert auto-dismisses after 15 seconds** (Wu #7, Al-Rashid #16, Park #17, Dawson #36) -- Critical safety alerts (DISARMED IN FLIGHT, FAILSAFE: RTL) disappear without requiring pilot acknowledgment. Violates IEC 60601 and ISO 15005 alarm hierarchy requirements. **Severity: safety-critical.**

6. **No geofence drawing or enforcement** (Chen #1, Morrison #2, Hayward #6, Torres #8, Schreiber #9, Frost #10, and 6 others) -- The most requested missing feature across all reviewers. No polygon geofence tool in Plan view, no in-flight boundary enforcement at GCS level. **Severity: safety/regulatory.**

7. **Zero authentication on WebSocket** (O'Brien #5, Kovacs #12, Frost #10) -- Any device on the local network can inject MAVLink commands including ARM, TAKEOFF, and KILL. No tokens, no TLS enforcement, no session management. **Severity: security-critical.**

8. **Camera/gimbal commands are stubs** (Chen #1, Blackwood #4, Volkov #25, Osei #24, Lindqvist #42, and 8 others) -- Context menu items "Point Camera Here" and "Trigger Camera Now" log messages and send nothing. No DO_MOUNT_CONTROL encoder exists. **Severity: feature-blocking for all camera operations.**

9. **No terrain awareness or terrain-following** (Chen #1, Morrison #2, Nguyen #3, Osei #24) -- No terrain elevation data, no AGL vs MSL disambiguation, no terrain profile in Plan view. "Terrain follow" without actual terrain data is hazardous near hills. **Severity: safety-critical in terrain.**

10. **ARM and KILL have zero keyboard/AT accessibility** (Williams #13, Chen Maya #43) -- The two most safety-critical controls require drag gesture (ARM) and sustained mouse hold (KILL). No keyboard shortcut, no screen reader path, no alternative input. **Severity: accessibility showstopper, Section 508 fail.**

---

### Consensus Strengths (What Everyone Agrees Is Good)

These items were praised by 10+ reviewers and should NOT be changed:

- **Flight state badge** -- Top-center persistent display of armed state, mode, voltage, GPS count, flight time. Called "the single best design decision" (Morrison #2), "strongest single safety feature" (Park #17), "exactly right" (Frost #10)
- **Failsafe detection logic** -- `_userModeChange` flag pattern distinguishing pilot vs autopilot mode changes. Called "the best I've seen in any GCS" (Chen #1), "genuinely good" (Torres Rachel #8)
- **Tlog recording to IndexedDB** -- 64KB chunked writes, auto-start, session management, binary preservation. Praised by Wu #7, Torres Rachel #8, Frost #10, Osei #24, Volkov #26, Thompson #27, and 8 others
- **Demo mode auto-start** -- Simulated dual-vehicle flight on page load. Called "genuinely smart" (Blackwood #4, Mei Lin #19), "the right hook" (Mei Lin #19)
- **Kill switch design** -- 1.5s hold, fill animation, physical isolation. Called "the best-designed element in the entire interface" (Al-Rashid #16), "correct human factors engineering" (Thompson #27)
- **Dark theme palette** -- `#080b10` background with cyan/green/red semantic colors. Called "looks like a NASA control room" (Park Jayden #21), "serious and precise" (Navarro #18)
- **Health grid dual-channel encoding** -- Color + shape (triangle/X) for colorblind accessibility. Praised by Holloway #34, Frost #10, Torres Capt #40, Park #17
- **Multi-vehicle architecture from day one** -- `vehicles[sysid]` map with clean state isolation. Praised by Chen #1, Volkov #26, Sharma #30, Nguyen Sam #41, Brennan #29
- **Offline tile caching** -- Cache API with batched downloads. Praised by Morrison #2, Osei #24, Frost #10, Zhang #39
- **Drag-to-fly** -- Dragging vehicle icon for GUIDED mode repositioning. Called "clever and fast" (Morrison #2), "something I've wanted for years" (Chen #1)

---

### "Would You Use This?" Tally

| Response | Count | Reviewers |
|---|---|---|
| **Yes, today** | 3 | Torres Rachel (insurance context), Zhang (OEM), Park Jayden (demo/hobby) |
| **Yes, with caveats** | 8 | Morrison, Volkov Dmitri, Patel, Petrov, Verou, Vladimir, Russo, Nguyen Sam |
| **Maybe, after fixes** | 12 | Chen, O'Brien, Wu, Frost, Tanaka, Al-Rashid, Park Helen, Navarro, Reeves, Osei, Dawson, Torres Capt |
| **No, wrong tool for me** | 14 | Nguyen Tommy, Blackwood, Hayward, Schreiber, Kovacs, Rodriguez, Volkov Nina, Chang Lisa, Sharma, Holloway, Delgado, Yamamoto, Lindqvist, Chen Maya |
| **No, not safe enough** | 5 | Williams, Thompson Brad, Mackay, Tridge, Dawson |
| **Watch (investment)** | 1 | Brennan |
| **No, need enterprise** | 2 | Sharma, Rodriguez |

---

## PART 2: EVERY FINDING, CATEGORIZED

### A. SAFETY-CRITICAL -- Things That Could Cause Injury, Vehicle Loss, or Regulatory Violation

**A1. Kill command broken -- sends disarm instead of flight termination**
- Tridge (#31): `MAVLink.encodeCommandLong` is not in the public API export. The kill path in `commands.js` always falls through to `encodeDisarm()`, which sends a soft disarm instead of `MAV_CMD_DO_FLIGHTTERMINATION (185)`. The kill button UI works; the actual MAVLink command sent is wrong.
- File: `js/commands.js`, `js/mavlink.js`

**A2. Failsafe alert auto-dismisses without acknowledgment**
- Wu (#7): 15-second auto-dismiss on active failsafe alert is "operationally inappropriate." Banner should persist until condition clears or pilot explicitly acknowledges.
- Al-Rashid (#16): "Emergency alerts in safety-critical systems do not auto-dismiss. ISO 15005:2002 section 7.2 requires operator acknowledgment."
- Park (#17): "A high-priority alarm must require active acknowledgment -- it cannot self-clear. IEC 62366 Annex C."
- Dawson (#36): Pilot looking at aircraft misses the banner; no persistent history indicator remains.
- File: `index.html` (boot IIFE alert-banner handler), `css/map.css`

**A3. No geofence drawing or polygon enforcement**
- Chen (#1): "The single biggest gap. No geofence drawing tool in plan view."
- Morrison (#2): "I can't see the actual fence boundary on the map."
- Hayward (#6): Only soft advisory limits via `guidedAltMax`/`guidedDistMax` in Settings; user can dial these to any value.
- Torres Rachel (#8): "Geofencing in the traditional sense -- a polygon the vehicle cannot cross -- is not present."
- Schreiber (#9): "Geofence is entirely operator-drawn and operator-controlled."
- Frost (#10): Not present in FMEA context.
- Dawson (#36): "Nothing prevents uploading a mission that flies the bird into the hillside."
- File: `js/plan/validator.js`, `js/settings.js`

**A4. No terrain awareness**
- Chen (#1): "Terrain follow without actual terrain elevation service is a checkbox that could kill the aircraft on a hillside approach."
- Morrison (#2): "No terrain data layer, no AGL readout that distinguishes from MSL, no elevation profile in Plan view."
- Nguyen (#3): "No terrain follow. The altitude field is one flat number per mission."
- Osei (#24): `terrain.js` listed in architecture but not in filesystem. "Without terrain profile visualization, I cannot verify clearance over ridgelines."
- File: Missing `js/plan/terrain.js`

**A5. "Set Home Here" modifies client-side only**
- Hayward (#6): "Modifies `v.homeLat`/`v.homeLon` client-side without sending a MAVLink command. RTL will return to the flight controller's stored home, not the one displayed on the map. Incident waiting to happen."
- File: `js/map.js` (context menu handler)

**A6. No heartbeat timeout watchdog**
- Wu (#7): "If heartbeats stop arriving, the UI stays green."
- Tanaka (#14): "No staleness detection on UI side for individual instruments."
- Dawson (#36): "The GCS will show the last known state indefinitely. No VEHICLE TIMEOUT banner."
- File: `js/state.js`, `index.html` (staleness detector only flashes badge opacity)

**A7. Pre-flight checklist does not gate arming**
- Hayward (#6): "Pilot can arm via slide-to-arm without the Setup checklist completing."
- Torres Rachel (#8): "No enforcement preventing arming when checklist shows incomplete required items."
- Mei Lin (#19): "DJI won't let you arm without walking through safety items. Meridian's checklist exists inside a setup panel."
- File: `js/setup/checklist.js`, `js/confirm.js`

**A8. No audio alerts of any kind**
- Wu (#7): "Battery critical produces no audio."
- Park (#17): "No audio for any alarm state. An unattended GCS with a silent critical alert is a meaningful gap."
- Al-Rashid (#16): No audio mentioned in emergency alert chain.
- File: Entire codebase -- no Web Audio API or `<audio>` usage anywhere

**A9. Battery critical has no banner/failsafe event**
- Wu (#7): "The only visual signal is a slightly redder bar in the top-right HUD corner."
- Dawson (#36): "No battery failsafe countdown. Voltage and percentage are shown but no configurable low-battery alert at GCS layer."
- File: `js/fly/hud.js` (battery widget), `js/state.js`

**A10. RC failsafe detection is passive only**
- Wu (#7): "No failsafe event fired from GCS when RC RSSI drops below threshold. GCS waits to see if vehicle announces a mode change."
- Dawson (#36): "No GPS fix degradation warning, no EKF variance threshold alarm, no geofence breach detection at GCS level."
- File: `js/state.js`

**A11. Guided-mode fly-to does not validate against distance limit**
- Hayward (#6): "`guidedDistMax` is only checked in mission plan validator, not in real-time guided operations."
- File: `js/map.js`, `js/fly/context-menu.js`

**A12. Fly Here sends goto on 2-second timeout regardless of ACK**
- Hayward (#6): "In degraded link conditions this could result in duplicate commands."
- Dawson (#36): "Timeout fallback sends goto after 2 seconds regardless, and does not deregister the onAck listener. Both may fire."
- File: `js/fly/context-menu.js`

**A13. `toggleArm()` references nonexistent `meridian.telemetry`**
- Dawson (#36): "`commands.js` references `window.meridian.telemetry.armed` but global is `meridian.v.armed`. TypeError in emergency disarm path."
- File: `js/commands.js`

**A14. Demo mode confusion hazard**
- Park (#17): "No persistent 'THIS IS SIMULATED DATA' banner. Simulation/live mode confusion hazard per IEC 62366."
- File: `index.html` (connection indicator only shows "DEMO" in small text)

**A15. No Remote ID**
- Hayward (#6): "Zero implementation. The word 'Remote ID' does not appear anywhere in the codebase. Flying with this GCS violates federal law."
- Schreiber (#9): "Zero matches for RemoteID, RID, ASTM, EN4709, UAS_ID. MAVLink decoder does not include OPEN_DRONE_ID messages (12900-12919)."
- File: Entire codebase -- complete absence

**A16. No LAANC / airspace awareness**
- Hayward (#6): "No mechanism to query FAA UAS Facility Map, submit LAANC request, or display approved altitudes."
- Schreiber (#9): "No connection to any authoritative airspace data source. No NOTAM APIs, no U-space USSP."
- Torres Rachel (#8): "No airspace API integration. No NOTAM check, no TFR awareness."
- File: Entire codebase

**A17. No operator identity / PIC attribution**
- Hayward (#6): "No pilot certificate number field, no DroneZone registration number."
- Schreiber (#9): "No operator license/certificate field, no registration number entry."
- Sharma (#30): "No pilot name attached to session, no aircraft tail number."
- File: `js/logs/tlog.js`, `js/settings.js`

**A18. Moving home point for maritime operations**
- Lindqvist (#42): "Home marker is static. On a vessel doing 4-6 knots, home drifts 15-20m per minute. RTL with stale home is a hazard."
- File: `js/fly/map.js`

---

### B. PROTOCOL & INTEGRATION -- MAVLink Bugs, Missing Messages, Connection Issues

**B1. MISSION_ITEM_INT decoder field offset bug**
- Tridge (#31): `command` read at byte 26 instead of byte 30. Missions from FC will have corrupted command fields.
- File: `js/mavlink.js`

**B2. Missing STATUSTEXT (msg 253)**
- Tridge (#31): "Completely absent. Not decoded, not listed in CRC_EXTRA."
- Mackay (#38): "The GCS will never show ArduPilot's own log messages."
- File: `js/mavlink.js`

**B3. No COMMAND_ACK retry logic**
- Tridge (#31): "`confirmation` hardcoded to 0. No retry on timeout, no pending-command queue."
- Mackay (#38): "COMMAND_ACK result=4 (TEMPORARILY_REJECTED) happens all the time. GCS logs nothing."
- File: `js/mavlink.js`, `js/connection.js`

**B4. Mission upload/download protocol state machine missing**
- Mackay (#38): "Download path calls sendMissionRequestList and then nothing. No listener responds to mission_count. The handshake engine is not implemented."
- File: `js/plan/mission.js`, `js/connection.js`

**B5. Mode encoder silent failure**
- Mackay (#38): "mavlink.js carries only 8 modes; state.js has 27. Encoder gets undefined for unlisted modes, falls through to 0 (STABILIZE)."
- File: `js/mavlink.js`, `js/state.js`

**B6. COBS trailing zero removal bug**
- Tridge (#31): "Decoder removes trailing zero, which will silently corrupt any message whose last byte is legitimately 0x00."
- File: `js/mnp.js`

**B7. MNP mission upload is fire-and-forget**
- Tridge (#31): "Blind dump of all mission items without FC acknowledgment. Will corrupt uploads over lossy links."
- File: `js/mnp.js`

**B8. No COMMAND_INT (msg 75) encoder**
- Tridge (#31): "Preferred message for geographically-targeted commands in MAVLink 2."
- File: `js/mavlink.js`

**B9. CRC failure does not advance by one byte**
- Tridge (#31): "Bad CRC consumes frameLen bytes instead of advancing by 1 and retrying. Could silently drop valid frames."
- File: `js/mavlink.js`

**B10. Missing critical MAVLink messages**
- Tridge (#31): LOCAL_POSITION_NED (32), HOME_POSITION (242), AUTOPILOT_VERSION (148), BATTERY_STATUS (147), FENCE_STATUS (162), RALLY_POINT (175), VIBRATION (241), POWER_STATUS (125)
- Mackay (#38): TIMESYNC (111), SYSTEM_TIME (2), PARAM_REQUEST_READ encoder
- File: `js/mavlink.js`

**B11. No MAVLink message signing**
- O'Brien (#5): "`incompat_flags = 0` hardcoded. Signing is spec-supported but not implemented."
- File: `js/mavlink.js`

**B12. No MAVFTP**
- Thompson (#27): "No SD card browsing, no DataFlash log pull, no file push to board."
- File: Entire codebase

**B13. GCS heartbeat at fixed 1Hz**
- Lindqvist (#42): "No adaptive heartbeat rate, no link quality metric, no jitter buffer."
- File: `js/connection.js`

---

### C. MISSING FEATURES -- Features That Don't Exist But Should

**C1. No video feed / RTSP / WebRTC**
- Volkov Nina (#25): "No video feed anywhere. No RTSP player, no WebRTC, no `<video>` element."
- Volkov Dmitri (#26): "QGC handles RTSP, UDP video, H.264 decode, gimbal control with video."
- Osei (#24): "No live video or thermal feed integration."
- Park Jayden (#21): "FPV video feed is the big one."
- Chang (#28): "The camera doesn't exist in the UI at all."
- File: Entire GCS -- no video infrastructure

**C2. No ADSB traffic display**
- Chen (#1): "State has AVOID_ADSB mode and settings reference adsbServer, but no ADSB rendering on map."
- Morrison (#2): "No ADSB display."
- O'Brien (#5): "No ADSB_VEHICLE message handler, no airspace picture."
- File: `js/state.js`, `js/settings.js` (adsbServer field exists but unwired)

**C3. No joystick/gamepad support**
- Volkov Dmitri (#26): "QGC maps USB joystick axes directly. Browser Gamepad API supported but not implemented."
- File: Entire codebase

**C4. No .waypoints file import/export**
- Chen (#1): "No file-based mission import. I have years of missions saved as MP .waypoints files."
- File: `js/plan/mission.js`

**C5. No saved mission library**
- Nguyen Tommy (#3): "No saved mission library. For a field I've flown before, redraw from scratch every time."
- Sharma (#30): "No mission template library, no way to publish standard survey profiles."
- File: `js/plan/mission.js`

**C6. No spline waypoints in mission editor**
- Volkov Nina (#25): "NAV_SPLINE_WAYPOINT listed in architecture but not in mission.js COMMANDS array."
- File: `js/plan/mission.js`

**C7. No orbit/circle mission type**
- Blackwood (#4): "No orbit/circle mission type. No POI/ROI tracking that actually works."
- Chang (#28): "No one-tap cinematic automation. No Dronie, no Helix, no Boomerang."
- File: `js/plan/` -- no orbit tool

**C8. No parameter metadata/descriptions**
- Volkov Dmitri (#26): "QGC loads rich JSON manifests. Meridian has names and values only."
- Thompson (#27): "MP ships with full ArduPilot parameter metadata -- descriptions, units, min/max, bitmask decoding."
- File: `js/params/`

**C9. No structure scan**
- Volkov Dmitri (#26): "QGC has structure scan. Meridian has polygon and corridor only."
- Lindqvist (#42): "No vertical scan pattern, no structure scan mode for towers."
- File: `js/plan/survey.js`

**C10. No auto-analysis for post-flight logs**
- Thompson (#27): "After a crash in MP: DataFlash > Auto Analysis reads the log and gives plain-English verdict. Entirely absent."
- File: `js/logs/`

**C11. No scripting console**
- Thompson (#27): "MP has built-in Lua/Python console. Meridian has nothing equivalent."
- File: Entire codebase

**C12. No OTA firmware update**
- Zhang (#39): "No OTA firmware update module. Would need FILE_TRANSFER_PROTOCOL or MAVFTP."
- File: `js/setup/`

**C13. No fleet management / drone inventory**
- Sharma (#30): "No tail numbers, no airframe records, no assignment of aircraft to pilots. No concept of a fleet."
- File: Entire codebase

**C14. No user accounts / role-based access**
- O'Brien (#5): "No login, no role separation between crew. One trust level: whoever opened the browser."
- Sharma (#30): "No user authentication, no role-based permissions, no organization hierarchy."
- Frost (#10): "Absent for ATO."
- File: Entire codebase

**C15. No multi-field log graphing**
- Thompson (#27): "Live graph plots one field at 4Hz. MP can plot 10+ fields simultaneously with synchronized cursors."
- File: `js/logs/log-graph.js`

**C16. No i18n / localization**
- Yamamoto (#37): "Zero internationalization. Every string hardcoded in English. No translation file, no i18n object. Canvas labels cannot be translated."
- File: Entire codebase

**C17. No tooltips on flight instruments or mode buttons**
- Jennings (#22): "No tooltips on mode buttons anywhere."
- Torres Capt (#40): "Not a single tooltip on any instrument."
- Yamamoto (#37): "No mode button has a two-word explanation."
- Blackwood (#4): "POSHOLD vs LOITER is not self-explanatory."
- File: `index.html`, `js/` (mode button construction in boot IIFE)

**C18. No wind data overlay**
- Lindqvist (#42): "No environmental data layer, no NMEA or sensor bus integration."
- File: `js/fly/map.js`

**C19. No inspection workflow / defect annotation**
- Lindqvist (#42): "No image annotation, no defect marking, no turbine component library, no severity tagging."
- File: Entire codebase

**C20. No pre-flight regulatory checklist**
- Schreiber (#9): "No pre-flight regulatory checklist distinct from the hardware checklist."
- Hayward (#6): "No operator identity layer for accountability."
- File: `js/setup/checklist.js`

---

### D. FAKE/SKELETON FEATURES -- Features That Have UI But Don't Work

**D1. Camera ROI / Trigger Camera Now -- stubs**
- Chen (#1): "context-menu.js has camera_roi, camera_trigger entries that log messages and do nothing."
- Volkov Nina (#25): "Neither sends a MAVLink command."
- Blackwood (#4): "Point Camera Here and Set ROI just log a message."
- Lindqvist (#42): "Camera controls are unimplemented stubs."
- File: `js/fly/context-menu.js`

**D2. DO_SET_ROI in mission -- no encoder**
- Volkov Nina (#25): "mission.js stores it as a numeric command with no actual send logic."
- File: `js/plan/mission.js`, `js/mavlink.js`

**D3. WP_dist in quick widget returns '---'**
- Chen (#1): "WP_dist is hardcoded to return '---'. Distance to next waypoint is probably the most important number during autonomous flight."
- File: `js/fly/quick.js`

**D4. ADSB server setting exists but is unwired**
- O'Brien (#5): "adsbServer field exists in settings, empty string, no code wired to it."
- File: `js/settings.js`

**D5. Offline map download UI incomplete**
- Volkov Dmitri (#26): "downloadRegion() function works via Cache API but no UI for bounds selection."
- File: `js/offline-tiles.js`, `js/settings.js`

**D6. Motor test sends simulated command in demo**
- Volkov Dmitri (#26): "Motor test UI exists but sends simulated command in demo mode, unclear if real MAVLink DO_MOTOR_TEST is encoded."
- File: `js/setup/motor-test.js`

**D7. Message badge not rendered**
- Wu (#7): "msg-badge class referenced in messages.js has no corresponding DOM element in index.html."
- Williams (#13): Confirmed.
- File: `js/messages.js`, `index.html`

---

### E. UI/UX DESIGN -- Layout, Typography, Touch Targets, Visual Hierarchy

**E1. Fragmented scan zones -- instruments too spread out**
- Al-Rashid (#16): "Badge at top, ADI at bottom, telemetry strip top-right, tapes bottom-left/right. Four dwell zones for primary info."
- Navarro (#18): "Cluster sits at bottom:8px, stapled to footer chrome. No breathing room."
- Reeves (#20): "Quick-widget and telemetry strip have identical visual weight with no priority ordering."
- File: `css/instruments.css`, `css/base.css`

**E2. Minimum font sizes too small (9-10px)**
- Morrison (#2): "Health grid items are 10px. Secondary readouts are 9px. Too small for field conditions."
- Jennings (#22): "9px and 10px text in health grid and message log is unusable for anyone over 50."
- Al-Rashid (#16): "14px tape value boxes are below MIL-STD-1472G 20-arcminute minimum."
- Delgado (#35): "flight-state-line2 uses 9px. Outdoors on tablet this is a blur."
- Reeves (#20): "flight-state-line2 is 9px. Below their own 10px floor."
- File: `css/base.css` (violates its own `--ts-xs: 10px` floor), `css/instruments.css`, `css/toolbar.css`

**E3. Touch targets systematically below 44px minimum**
- Delgado (#35): Toolbar nav 32px, settings gear 28px, panel close 24px, map toolbar 30px, params action 28px, settings mode-remove 20px. "Systematic and pervasive."
- Mei Lin (#19): "Action bar 34-36px buttons. DJI minimum is 56-64px."
- Chen Maya (#43): "Map zoom toolbar buttons are 30x30px."
- File: `css/toolbar.css`, `css/actions.css`, `css/map.css`, `css/params.css`, `css/settings.css`

**E4. No responsive breakpoints / no portrait mode**
- Delgado (#35): "No @media breakpoints in base.css, toolbar.css, actions.css, instruments.css, or map.css. Portrait rotation: nothing adapts."
- File: `css/base.css` and all CSS files

**E5. Light mode glass HUD washes out in sunlight**
- Delgado (#35): "rgba(255,255,255,0.85) over white tile map is near-zero contrast."
- Navarro (#18): "Light mode instruments and map tiles are almost identical luminance."
- Al-Rashid (#16): "Glass morphism is a hazard in bright sunlight."
- File: `css/instruments.css`, `css/base.css`

**E6. Action bar too dense**
- Williams (#13): "Eleven interactive elements in one 48px horizontal strip."
- Blackwood (#4): "ARM slider sitting right next to KILL with no visual breathing room."
- File: `css/actions.css`

**E7. No onboarding / empty state / first-time guidance**
- Mei Lin (#19): "No empty state. No 'here's what you do next.'"
- Jennings (#22): "No onboarding. No 'first time here?' prompt."
- Blackwood (#4): "No 'skip if you don't know what this means' guidance."
- Torres Capt (#40): "No guided tour or callout overlays."
- File: Entire GCS

**E8. Mode buttons lack context**
- Blackwood (#4): "I use Sport mode and QuickShot. I don't know what half of these do."
- Jennings (#22): "No tooltips. I hover over STABILIZE and nothing happens."
- Chang (#28): "20+ modes. DJI has three: Normal, Cine, Sport."
- File: `index.html` (mode tray), `css/actions.css`

**E9. No confirmation on mode switching**
- Jennings (#22): "No confirmation dialog on mode switches. Takeoff has one, but modes don't."
- Dawson (#36): "Rapid mode switching calls sendSetMode directly with no debounce."
- File: `js/commands.js`, `index.html`

**E10. ADI placed too low**
- Al-Rashid (#16): "Bottom-center, competing with map. Primary instrument should be center-eye-level."
- Navarro (#18): "Instruments feel stapled to the footer chrome."
- File: `css/instruments.css`

**E11. Altitude input for takeoff is tiny**
- Park Jayden (#21): "40-pixel wide number box. Easy to mistype 100 when you meant 10."
- File: `index.html` (takeoff-alt input)

**E12. Hover-only battery detail inaccessible on touch**
- Mei Lin (#19): "Battery detail information is on hover. Hover does not exist on a touchscreen."
- File: `css/instruments.css`

**E13. Params tab visible at top level during flight**
- Mei Lin (#19): "Params should not be tab-level visible on the fly screen. One accidental tap and you leave fly view."
- File: `index.html` (toolbar nav)

**E14. Keyboard shortcuts displayed on toolbar waste space on tablets**
- Mei Lin (#19): "Nobody flying from a tablet uses keyboard shortcuts. The F, P, S badges waste space."
- File: `css/toolbar.css`

**E15. Quick-widget numeric values not fixed-width**
- Reeves (#20): "Numbers shift laterally as they update. No min-width character budget."
- File: `css/instruments.css`, `js/fly/quick.js`

**E16. Stale-data handler is hand-rolled opacity toggle**
- Reeves (#20): "badge.style.opacity toggle in setInterval fights CSS transitions. Define a CSS class instead."
- Patel (#11): "Reading .style.opacity after writing is classic thrash pattern."
- File: `index.html` (staleness detector)

**E17. Staleness opacity flicker too subtle for critical warning**
- Al-Rashid (#16): "1Hz opacity oscillation won't capture attention under stress. Should be amber background fill."
- Williams (#13): "Will not be perceivable by users with low vision. Borderline for WCAG 2.3.1."
- File: `index.html`

**E18. One-handed reach zones wrong for tablet**
- Delgado (#35): "ARM slider at far left of action bar requires grip shift. Map toolbar buttons top-left are unreachable with right thumb."
- File: `css/actions.css`, `css/map.css`

---

### F. ACCESSIBILITY -- Colorblind, Motor Impairment, Screen Reader, Keyboard, i18n

**F1. Zero ARIA attributes in entire codebase**
- Williams (#13): "Zero results for aria-, role=, tabindex, aria-live, aria-label. Not a single ARIA attribute exists."
- File: Entire codebase

**F2. ARM and KILL inaccessible to keyboard / AT users**
- Williams (#13): "ARM is drag-only. KILL is hold-only. No keyboard equivalent whatsoever."
- Chen Maya (#43): "Two most critical safety actions are physically inaccessible to me."
- File: `js/confirm.js`, `js/commands.js`

**F3. No focus-visible indicators anywhere**
- Williams (#13): "Twelve outline:none declarations across CSS files. No :focus-visible replacement."
- File: `css/actions.css`, `css/params.css`, `css/plan.css`, `css/settings.css`, `css/setup.css`, `css/survey.css`, `css/status.css`

**F4. Connection indicator color-only**
- Holloway (#34): "Three states encoded entirely through text color and a 5px dot. No shape change, no label change."
- File: `css/toolbar.css`

**F5. Status dots color-only**
- Holloway (#34): ".status-dot.green, .yellow, .red -- same size, same shape, differentiated only by color."
- File: `css/gcs.css`

**F6. Battery bar color-only alarm**
- Holloway (#34): "Transitions between green/yellow/red purely by background color."
- File: `css/instruments.css`

**F7. ARM dot color-only**
- Holloway (#34): "Changes between safe (green) and armed (orange) with no shape or label change."
- File: `css/toolbar.css`

**F8. Red/green hex values indistinguishable for deuteranopia**
- Holloway (#34): "Light mode --c-safe #16a34a and --c-emergency #dc2626 both look like dull brownish-amber to me."
- File: `css/base.css`, `css/theme-dark.css`

**F9. Canvas instruments inherently inaccessible with no text alternatives**
- Williams (#13): "Canvas ADI, compass, speed tape, altitude tape have no aria-label, no offscreen equivalent."
- File: `js/fly/adi.js`, `js/fly/compass.js`, `js/fly/tapes.js`

**F10. No dynamic live regions for state changes**
- Williams (#13): "Flight state badge updates on every heartbeat but no aria-live region. Failsafe banner invisible to AT."
- File: `index.html`, `css/map.css`

**F11. Right-click only for map context menu**
- Chen Maya (#43): "No long-press alternative, no keyboard shortcut. Right-click not available to left-hand-only mouse users."
- File: `js/fly/context-menu.js`

**F12. No tabindex anywhere**
- Chen Maya (#43): "tabindex appears nowhere in the codebase."
- File: Entire codebase

**F13. Global user-select:none interferes with screen readers**
- Chen Maya (#43): "Interferes with screen-reader text selection."
- Williams (#13): "Sets a tone that prioritizes pointer interaction."
- File: `css/base.css`

**F14. Light theme contrast failures at small sizes**
- Williams (#13): "--c-text-dim (#64748b) on white is 4.5:1 but fails for 9-10px text. --c-neutral-dim (#94a3b8) is 2.4:1 -- fails outright."
- Delgado (#35): "c-neutral-dim at #94a3b8 on white is around 2.4:1. Fails."
- File: `css/base.css`

**F15. Zero internationalization infrastructure**
- Yamamoto (#37): "No language setting, no locale option, no i18n object, no data-i18n attributes. Canvas labels always English."
- File: Entire codebase

**F16. No coordinate format options**
- Yamamoto (#37): "Only decimal degrees. No degree-minute-second option."
- File: `js/settings.js`

**F17. Panel close buttons announced as "times" by screen readers**
- Williams (#13): "Render as &times; with no aria-label."
- File: `index.html`

---

### G. PERFORMANCE -- Memory Leaks, Render Bottlenecks, Bundle Loading

**G1. getBoundingClientRect called every frame in tapes.js**
- Patel (#11): "drawTape calls initCanvas which calls getBoundingClientRect. 20 forced layouts per second."
- Tanaka (#14): Confirmed. "Layout query at 10Hz from render loop."
- Petrov (#15): Confirmed.
- Verou (#33): Confirmed. "Canvas dimensions re-queried from DOM every frame."
- File: `js/fly/tapes.js`

**G2. ADI vignette gradient recreated every frame**
- Tanaka (#14): "createRadialGradient on every draw creates new CanvasGradient 10x/second. GC pressure."
- Petrov (#15): Confirmed.
- Verou (#33): "Overdraw that bites at 60fps on embedded hardware."
- File: `js/fly/adi.js` (~line 197)

**G3. Event listener accumulation -- no events.off() cleanup**
- Patel (#11): "off() is almost never called. FlyView, Toolbar, PlanView subscribe on init and never unsubscribe."
- Petrov (#15): "MavlinkInspector calls attachListeners at module-load with no teardown."
- File: `js/state.js`, all view modules

**G4. LogGraph resize listener leak**
- Patel (#11): "resize listener accumulates across tab switches. destroy() only called explicitly."
- File: `js/logs/log-graph.js`

**G5. StatusView setTimeout proliferation**
- Petrov (#15): "For every field that changes, schedules setTimeout(fn, 300). At 4Hz with 200+ fields, 50-100 pending timers alive."
- File: `js/status-view.js`

**G6. Trail polyline setLatLngs called every tick with full array**
- Petrov (#15): "trailLine.setLatLngs(v.trail) passes entire 1000-point trail every tick even when trail has not grown."
- Vladimir (#32): "setLatLngs on long polyline forces Leaflet to re-project every coordinate."
- File: `js/fly/map.js`

**G7. 68 blocking script tags with no bundling**
- Petrov (#15): "68 scripts as individual <script> tags. Estimated 300-500ms parse/compile on mid-range device."
- Zhang (#39): "No bundling means 48+ individual network requests."
- File: `index.html`

**G8. Uncertainty circle setStyle called every frame even when unchanged**
- Vladimir (#32): "setStyle triggers full SVG style recalculation. Skip when state has not changed."
- File: `js/fly/map.js`

**G9. No backpressure on event bus**
- Tanaka (#14): "No queue depth limit, no message rate limit, no frame-drop policy."
- File: `js/state.js`

**G10. Pitch ladder bounds check inverted**
- Verou (#33): "`if (y < -h && y > h) continue` is always false. Should be ||. Draws all ticks regardless of visibility."
- File: `js/fly/adi.js`

**G11. Tuning chart has no DPR handling**
- Verou (#33): "Hardcoded width/height with no DPR scaling. PID chart blurry on 2x displays."
- File: `js/params/tuning.js`

**G12. MNP buffer grows without bound**
- Kovacs (#12): "mnpBuffer doubles indefinitely. Malformed COBS frames that never emit 0x00 grow buffer until tab crash."
- Dawson (#36): "FrameParser buffer also grows without limit on garbage data."
- File: `js/mnp.js`, `js/mavlink.js`

**G13. Tlog flushChunk has no backpressure or error handling**
- Dawson (#36): "No error callback on IndexedDB put. At 100Hz telemetry, dozens of in-flight transactions queue up."
- File: `js/logs/tlog.js`

**G14. Tlog download revokeObjectURL race condition**
- Petrov (#15): "URL.revokeObjectURL called synchronously before browser starts download."
- File: `js/logs/tlog.js`

**G15. HeartBeat listener in MultiVehicle rebuilds innerHTML at 1Hz**
- Patel (#11): "Full innerHTML rebuild of vehicle selector on every heartbeat."
- File: `js/multi-vehicle.js`

**G16. CSS: 12 separate link tags block rendering**
- Petrov (#15): "No critical/non-critical CSS split."
- File: `index.html`

---

### H. SECURITY -- WebSocket Auth, XSS, Data Exposure, CDN Supply Chain

**H1. Zero WebSocket authentication**
- Kovacs (#12): "No token, no API key, no handshake challenge."
- O'Brien (#5): "Any user on the local network can send ARM, TAKEOFF, KILL commands."
- Frost (#10): "ATO will not be granted without WSS minimum and preferably mTLS."
- File: `js/connection.js`

**H2. No SRI on Leaflet CDN dependency**
- Kovacs (#12): "Leaflet loaded from unpkg.com with no Subresource Integrity hash. Compromise gives full FC access."
- Frost (#10): "CDN dependencies are non-starter for JCTD or fielded use."
- O'Brien (#5): "Fonts and Leaflet from public CDNs. Hard no for SIPR."
- File: `index.html`

**H3. XSS via MAVLink inspector**
- Kovacs (#12): "mavlink-inspector.js builds innerHTML with m.data from parsed state. Spoofed FC with malicious param name triggers stored XSS."
- File: `js/logs/mavlink-inspector.js`

**H4. Modal.js innerHTML injection vector**
- Kovacs (#12): "confirm() and prompt() interpolate title/message without escaping. Currently hardcoded, but one refactor away from live XSS."
- File: `js/modal.js`

**H5. Param name length unbounded in MNP**
- Kovacs (#12): "No maximum length check on string. 100 crafted 65KB param names = memory exhaustion DoS."
- File: `js/mnp.js`

**H6. IndexedDB telemetry accessible to any same-origin script**
- Kovacs (#12): "Full tlog including GPS trails and home position readable via indexedDB.open."
- File: `js/logs/tlog.js`

**H7. localStorage settings in plaintext**
- O'Brien (#5): "Settings including WebSocket URL, failsafe config stored as plaintext."
- Frost (#10): "On shared GCS laptop, confidentiality and integrity concern."
- File: `js/settings.js`

**H8. Auto-reconnect compounds unauthorized access**
- Kovacs (#12): "If attacker briefly hijacks endpoint, GCS reconnects in 3s automatically."
- File: `js/connection.js`

---

### I. DOCUMENTATION & COMMUNITY -- README, Tests, Contributing Guide, Licensing

**I1. No README in gcs/**
- Kowal (#45): "No README, no CONTRIBUTING, no LICENSE file, no package.json."
- File: `gcs/` directory

**I2. Zero test coverage for GCS**
- Kowal (#45): "No test runner, no test directory, no snapshot tests, no headless browser tests."
- Frost (#10): "No unit tests visible."
- File: `gcs/` directory

**I3. No CONTRIBUTING guide**
- Kowal (#45): "Cannot contribute in a day without significant reverse-engineering."
- File: `gcs/` directory

**I4. No LICENSE file**
- Kowal (#45): "No LICENSE file anywhere under gcs/."
- File: `gcs/` directory

**I5. var/let split across files**
- Kowal (#45): "Core files use let/const. Plan/setup files use var throughout. Same directory, same patterns."
- Patel (#11): "Mixed let/const vs var style across files."
- File: `js/plan/validator.js`, `js/plan/stats.js`, `js/setup/checklist.js`, `js/plan/plan-view.js`

**I6. Boot IIFE should be extracted to app.js**
- Patel (#11): "260 lines at bottom of index.html contain 4+ distinct concerns. Untestable, unsearchable."
- File: `index.html`

**I7. Two architecture docs suggest drift**
- Frost (#10): "ARCHITECTURE.md and ARCHITECTURE_v2.md -- documentation drift."
- File: `gcs/ARCHITECTURE.md`, `gcs/ARCHITECTURE_v2.md`

**I8. No JSDoc or API contracts**
- Kowal (#45): "No JSDoc annotations. No documentation for event bus contract -- which events fire, what payload shape."
- File: All JS files

**I9. No package.json**
- Kowal (#45): "No declared entry point, no scripts, no dependency locking."
- File: `gcs/` directory

---

### J. DOMAIN-SPECIFIC GAPS -- Features Needed By Specific Industries

**J1. Agriculture: No spray rate telemetry, no swath width, no battery cycle tracking**
- Nguyen Tommy (#3): "Missing spray system telemetry integration, per-battery cycle tracking, and saved mission library."
- File: `js/state.js`, `js/plan/survey.js`

**J2. Agriculture: Flight time estimator assumes wrong speed**
- Nguyen Tommy (#3): "PlanStats assumes 5 m/s. My T40 does 7-10 m/s. Estimate off by 30-40%."
- File: `js/plan/stats.js`

**J3. Agriculture: No expert mode to skip confirmation dialogs**
- Nguyen Tommy (#3): "After flight 30, the confirm dialog on takeoff gets old."
- File: `js/commands.js`

**J4. SAR: No AGL vs ASL disambiguation**
- Morrison (#2): "Need to know whether 'hold altitude' means AGL or ASL."
- File: `js/fly/tapes.js`, `js/state.js`

**J5. SAR: RTL altitude not visible in fly view**
- Morrison (#2): "RTL_ALT buried in params. Need persistent readout in action bar."
- Chen (#1): "Can't change RTL altitude from fly view during flight."
- File: `js/fly/` -- no RTL altitude widget

**J6. Real Estate / Cinematography: No cinematic orbit, POI tracking, quickshots**
- Blackwood (#4): "No orbit/circle mission type. No shutter speed, ISO, white balance controls."
- Volkov Nina (#25): "No gimbal pitch control, no roll control, no speed ramping."
- Chang (#28): "No one-tap cinematic automation."
- File: `js/plan/`, `js/fly/`

**J7. Construction: No photogrammetry, volume calc, GCP, reports**
- Rodriguez (#23): "Everything downstream of flight is missing. No orthomosaic, no volume calculation, no GCP, no reports."
- File: Entire codebase -- post-flight processing out of scope but noted

**J8. Wildlife: No thermal camera integration**
- Osei (#24): "No FLIR/Boson camera protocol, no thermal overlay on map, no GStreamer RTSP support."
- File: Entire codebase

**J9. Offshore/Maritime: No moving home, no AIS, no wind overlay, no vessel-relative display**
- Lindqvist (#42): All four missing. "Every gap is a maritime/offshore gap."
- File: `js/fly/map.js`, `js/state.js`

**J10. Delivery: No automated dispatch, no fleet dashboard, no connection pool**
- Nguyen Sam (#41): "No REST/WS server interface, no bulk-command surface, no automated dispatch."
- File: `js/connection.js`, entire UI

**J11. Training: No fault injection in demo, no guided tour, no actionable error messages**
- Torres Capt (#40): "Demo starts armed and airborne -- student never sees pre-flight sequence. No simulated GPS loss or battery failsafe."
- File: `js/demo.js`

**J12. Military: No STANAG 4586, no classification marking, no audit log**
- O'Brien (#5): "STANAG 4586 DLI Level 3 and above requires defined VSM interfaces."
- Frost (#10): "No formal hazard log, no FMEA documentation."
- File: Entire codebase

**J13. Military: No payload/ISR integration**
- O'Brien (#5): "No gimbal control, no camera trigger, no EO/IR feed, no slant range calculation."
- File: `js/fly/`, `js/mavlink.js`

**J14. Military: No encrypted C2 or operator handoff**
- O'Brien (#5): "No crew coordination layer, no PIC designation, no handoff authority transfer."
- File: `js/connection.js`

**J15. Commercial survey: No waypoint altitude bulk-set**
- Chen (#1): "No 'Set All Altitudes to X' in mission toolbar. Need to set 200 waypoints individually."
- File: `js/plan/wp-editor.js`

**J16. Commercial survey: Limited camera presets**
- Chen (#1): "Only DJI Mini 3 Pro and Sony A7R IV. No save-your-own-camera feature."
- File: `js/plan/survey.js`

**J17. Research: No live time-series chart, no ROS2 bridge**
- Russo (#44): "No built-in time-series chart widget for live streaming. Binary-only WebSocket assumption."
- File: `js/logs/log-graph.js`, `js/connection.js`

---

## PART 3: PRIORITIZED FIX PLAN

### TIER 0 -- SHIP BLOCKERS (Must Fix Before Any Real Flight)

| # | Fix | Files | Size | Reviewers |
|---|---|---|---|---|
| T0-1 | **Fix kill command: export encodeCommandLong in MAVLink public API** | `js/mavlink.js`, `js/commands.js` | S | Tridge #31 |
| T0-2 | **Fix MISSION_ITEM_INT field offset: command at byte 30, not 26** | `js/mavlink.js` | S | Tridge #31 |
| T0-3 | **Add STATUSTEXT (msg 253) decoder and display** | `js/mavlink.js`, `js/state.js`, `js/messages.js` | M | Tridge #31, Mackay #38 |
| T0-4 | **Fix mode encoder: unify mode tables between mavlink.js and state.js** | `js/mavlink.js`, `js/state.js` | S | Mackay #38 |
| T0-5 | **Fix toggleArm() reference: `meridian.telemetry` -> `meridian.v`** | `js/commands.js` | S | Dawson #36 |
| T0-6 | **Make failsafe alert require explicit acknowledgment (remove auto-dismiss)** | `index.html` (boot IIFE), `css/map.css` | S | Wu #7, Al-Rashid #16, Park #17, Dawson #36 |
| T0-7 | **Add heartbeat timeout watchdog with LINK STALE / VEHICLE TIMEOUT banner** | `js/state.js`, `index.html` | M | Wu #7, Tanaka #14, Dawson #36 |
| T0-8 | **Fix Set Home Here to send MAVLink SET_HOME_POSITION** | `js/fly/context-menu.js`, `js/mavlink.js` | S | Hayward #6 |
| T0-9 | **Fix COBS trailing zero removal bug** | `js/mnp.js` | S | Tridge #31 |
| T0-10 | **Fix CRC failure to advance by one byte** | `js/mavlink.js` | S | Tridge #31 |
| T0-11 | **Add keyboard shortcut for ARM (with 2-key confirmation) and KILL** | `js/confirm.js`, `js/commands.js` | M | Williams #13, Chen Maya #43 |
| T0-12 | **Fix Fly Here dual-fire: deregister onAck before timeout fallback sends** | `js/fly/context-menu.js` | S | Hayward #6, Dawson #36 |
| T0-13 | **Cap MNP buffer and MAVLink FrameParser buffer at max size** | `js/mnp.js`, `js/mavlink.js` | S | Kovacs #12, Dawson #36 |
| T0-14 | **Add battery critical banner and failsafe event** | `js/state.js`, `index.html` | S | Wu #7, Dawson #36 |
| T0-15 | **Add pitch ladder bounds check fix (|| not &&)** | `js/fly/adi.js` | S | Verou #33 |

### TIER 1 -- COMPETITIVE PARITY (Must Fix to Match QGC/MP)

| # | Fix | Files | Size | Reviewers |
|---|---|---|---|---|
| T1-1 | **Implement mission upload/download protocol state machine** | `js/plan/mission.js`, `js/connection.js` | L | Mackay #38 |
| T1-2 | **Add geofence polygon drawing tool in Plan view** | `js/plan/` (new file), `js/fly/map.js` | L | Chen #1, Morrison #2, Hayward #6, Torres #8, Schreiber #9 + 6 others |
| T1-3 | **Add COMMAND_ACK handling with retry and user-visible feedback** | `js/connection.js`, `js/commands.js` | M | Tridge #31, Mackay #38 |
| T1-4 | **Wire camera commands: DO_MOUNT_CONTROL, DO_SET_ROI, camera trigger** | `js/mavlink.js`, `js/fly/context-menu.js` | M | Chen #1, Blackwood #4, Volkov Nina #25 + 8 others |
| T1-5 | **Add ADSB traffic display on map** | `js/mavlink.js`, `js/fly/map.js`, `js/state.js` | L | Chen #1, Morrison #2, O'Brien #5 |
| T1-6 | **Add video feed panel (RTSP/WebRTC)** | New `js/fly/video.js`, `index.html` | XL | Volkov Nina #25, Dmitri #26, Osei #24, Park Jayden #21, Chang #28 |
| T1-7 | **Add parameter metadata loader (descriptions, units, ranges)** | `js/params/` | L | Dmitri #26, Thompson #27 |
| T1-8 | **Add .waypoints file import/export** | `js/plan/mission.js` | M | Chen #1 |
| T1-9 | **Add spline waypoints (NAV_SPLINE_WAYPOINT)** | `js/plan/mission.js`, `js/mavlink.js` | M | Volkov Nina #25 |
| T1-10 | **Add pre-flight checklist as arming gate** | `js/confirm.js`, `js/setup/checklist.js` | M | Hayward #6, Torres Rachel #8, Mei Lin #19 |
| T1-11 | **Add tooltips to all mode buttons and instruments** | `index.html`, `css/actions.css` | M | Jennings #22, Torres Capt #40, Yamamoto #37, Blackwood #4 |
| T1-12 | **Implement Remote ID (OPEN_DRONE_ID MAVLink messages)** | `js/mavlink.js`, `js/state.js`, `js/settings.js`, `js/fly/map.js` | L | Hayward #6, Schreiber #9 |
| T1-13 | **Add SRI hashes on all CDN dependencies** | `index.html` | S | Kovacs #12, Frost #10, O'Brien #5 |
| T1-14 | **Fix tapes.js: cache canvas dimensions, call getBoundingClientRect only on resize** | `js/fly/tapes.js` | S | Patel #11, Tanaka #14, Petrov #15, Verou #33 |
| T1-15 | **Cache ADI vignette gradient, recreate only on resize** | `js/fly/adi.js` | S | Tanaka #14, Petrov #15, Verou #33 |
| T1-16 | **Add saved mission library** | `js/plan/mission.js` | M | Nguyen Tommy #3, Sharma #30 |
| T1-17 | **Add terrain elevation integration and terrain profile in Plan view** | New `js/plan/terrain.js`, `js/fly/tapes.js` | XL | Chen #1, Morrison #2, Nguyen Tommy #3, Osei #24 |
| T1-18 | **Wire WP_dist in quick widget** | `js/fly/quick.js`, `js/state.js` | S | Chen #1 |
| T1-19 | **Add ARIA attributes: aria-live regions, aria-labels, role attributes** | `index.html`, all JS files | M | Williams #13 |
| T1-20 | **Add :focus-visible indicators on all interactive elements** | All CSS files | M | Williams #13 |
| T1-21 | **Fix light theme contrast: bump --c-text-dim and --c-neutral-dim** | `css/base.css` | S | Williams #13, Delgado #35 |
| T1-22 | **Add colorblind-safe shape encoding to connection indicator, status dots, battery bar, arm dot** | `css/toolbar.css`, `css/gcs.css`, `css/instruments.css` | M | Holloway #34 |
| T1-23 | **Increase minimum font to 11-12px; fix 9px violations** | `css/base.css`, `css/toolbar.css`, `css/instruments.css`, `css/actions.css` | S | Morrison #2, Jennings #22, Delgado #35, Reeves #20 |
| T1-24 | **Add audio alerts for critical events** | New `js/audio.js` | M | Wu #7, Park #17 |
| T1-25 | **Increase all touch targets to 44px minimum** | All CSS files | M | Delgado #35, Mei Lin #19, Chen Maya #43 |
| T1-26 | **Add link-loss alarm (audio + visual)** | `js/connection.js`, `js/state.js` | M | Dawson #36 |
| T1-27 | **Add missing MAVLink messages: HOME_POSITION, BATTERY_STATUS, FENCE_STATUS, VIBRATION, AUTOPILOT_VERSION** | `js/mavlink.js` | L | Tridge #31, Mackay #38 |
| T1-28 | **Extract boot IIFE to js/app.js** | `index.html` -> new `js/app.js` | M | Patel #11 |
| T1-29 | **Add README, LICENSE, CONTRIBUTING to gcs/** | `gcs/README.md`, `gcs/LICENSE`, `gcs/CONTRIBUTING.md` | S | Kowal #45 |
| T1-30 | **Add unit tests for mavlink.js, validator.js, mnp.js** | New `gcs/tests/` | L | Kowal #45, Frost #10 |

### TIER 2 -- DIFFERENTIATION (Features That Would Make Meridian Better Than Alternatives)

| # | Fix | Files | Size | Reviewers |
|---|---|---|---|---|
| T2-1 | **LAANC / airspace awareness integration** | New module + API integration | XL | Hayward #6, Schreiber #9, Torres Rachel #8 |
| T2-2 | **Operator identity: PIC name, certificate, registration in tlog metadata** | `js/settings.js`, `js/logs/tlog.js` | M | Hayward #6, Schreiber #9, Sharma #30 |
| T2-3 | **WebSocket authentication (token auth, WSS enforcement)** | `js/connection.js` | L | Kovacs #12, O'Brien #5, Frost #10 |
| T2-4 | **Per-instrument staleness indicator (gray out stale values)** | `js/fly/`, `js/state.js` | M | Tanaka #14, Reeves #20 |
| T2-5 | **Event bus backpressure / coalescing** | `js/state.js` | M | Tanaka #14 |
| T2-6 | **Guided-mode distance limit enforcement in real-time** | `js/fly/context-menu.js`, `js/map.js` | S | Hayward #6 |
| T2-7 | **Demo mode fault injection (GPS loss, battery fail, RC fail)** | `js/demo.js` | M | Torres Capt #40 |
| T2-8 | **Bundle build step (concat + minify)** | New build script | M | Petrov #15, Zhang #39 |
| T2-9 | **Offline map download UI with bounds selection** | `js/offline-tiles.js`, `js/settings.js` | M | Dmitri #26 |
| T2-10 | **Orbit/circle mission tool** | `js/plan/` | L | Blackwood #4, Chang #28 |
| T2-11 | **Joystick/gamepad support via Gamepad API** | New `js/gamepad.js` | L | Dmitri #26 |
| T2-12 | **Structure scan for towers/vertical surfaces** | `js/plan/survey.js` | L | Dmitri #26, Lindqvist #42 |
| T2-13 | **Onboarding / first-time guidance flow** | New module | M | Mei Lin #19, Jennings #22, Blackwood #4 |
| T2-14 | **i18n framework with JSON translation files** | New `js/i18n.js`, translation JSONs | L | Yamamoto #37 |
| T2-15 | **Fixed-width numeric columns on quick-widget** | `css/instruments.css`, `js/fly/quick.js` | S | Reeves #20 |
| T2-16 | **Replace stale-data setInterval with CSS class toggle** | `index.html` | S | Reeves #20, Patel #11 |
| T2-17 | **Configurable cruise speed for plan stats** | `js/plan/stats.js` | S | Nguyen Tommy #3 |
| T2-18 | **RTL altitude widget in fly view** | `js/fly/`, action bar | S | Morrison #2, Chen #1 |
| T2-19 | **Responsive CSS breakpoints for tablet/portrait** | All CSS files | L | Delgado #35 |
| T2-20 | **Save custom camera presets in survey tool** | `js/plan/survey.js` | S | Chen #1 |
| T2-21 | **Escape HTML in MAVLink inspector** | `js/logs/mavlink-inspector.js` | S | Kovacs #12 |
| T2-22 | **Add MAVFTP for log/file transfer** | New `js/mavftp.js` | XL | Thompson #27 |
| T2-23 | **Multi-field overlay in log graph** | `js/logs/log-graph.js` | L | Thompson #27 |
| T2-24 | **Voice command layer** | New `js/voice.js` | L | Chen Maya #43 |
| T2-25 | **Large target mode / coarse pointer CSS** | CSS override sheet | M | Chen Maya #43 |
| T2-26 | **Render loop at 30Hz for ADI/compass (currently 10Hz)** | `js/fly/fly-view.js` | S | Navarro #18 |

### TIER 3 -- MARKET EXPANSION (Features for Specific Verticals)

| # | Fix | Files | Size | Reviewers |
|---|---|---|---|---|
| T3-1 | **Spray system telemetry integration (custom MAVLink messages)** | `js/mavlink.js`, `js/state.js`, new widget | L | Nguyen Tommy #3 |
| T3-2 | **Per-battery cycle tracking and lifecycle view** | New `js/battery-lifecycle.js` | L | Nguyen Tommy #3 |
| T3-3 | **Thermal camera / FLIR integration** | New `js/fly/thermal.js` | XL | Osei #24 |
| T3-4 | **Moving home point with AIS vessel feed** | `js/fly/map.js`, `js/connection.js` | L | Lindqvist #42 |
| T3-5 | **Wind/met data overlay** | `js/fly/map.js` | L | Lindqvist #42 |
| T3-6 | **Inspection defect annotation and reporting** | New module | XL | Lindqvist #42 |
| T3-7 | **Fleet management: drone inventory, pilot accounts, central config** | New module | XL | Sharma #30 |
| T3-8 | **Automated dispatch engine for delivery** | New module | XL | Nguyen Sam #41 |
| T3-9 | **Connection pool for multi-vehicle at scale** | `js/connection.js` | L | Nguyen Sam #41 |
| T3-10 | **Post-flight auto-analysis (log anomaly detection)** | New `js/logs/auto-analysis.js` | XL | Thompson #27 |
| T3-11 | **Scripting console (Lua/Python)** | New module | XL | Thompson #27 |
| T3-12 | **Split-screen multi-vehicle view** | `js/fly/fly-view.js`, `js/multi-vehicle.js` | L | Nguyen Tommy #3 |
| T3-13 | **EU C-class operational enforcement** | `js/state.js`, `js/settings.js` | L | Schreiber #9 |
| T3-14 | **STANAG 4586 compatibility layer** | New module | XL | O'Brien #5 |
| T3-15 | **ROS2 bridge / rosbridge adapter** | `js/connection.js` | L | Russo #44 |
| T3-16 | **Photogrammetry/GCP/volume calculation** | Out of GCS scope | XL | Rodriguez #23 |
| T3-17 | **Quickshot cinematic modes (Dronie, Helix, Orbit)** | New `js/plan/quickshots.js` | L | Chang #28, Blackwood #4 |
| T3-18 | **Betaflight PID import** | New integration | L | Park Jayden #21 |
| T3-19 | **OTA firmware update module** | New `js/setup/firmware.js` | XL | Zhang #39 |
| T3-20 | **Pre-flight regulatory checklist (Part 107 / EU 2019/947)** | `js/setup/checklist.js` | M | Schreiber #9, Hayward #6 |

---

## PART 4: WHAT'S ACTUALLY GOOD

Every positive finding from all 45 reviewers, organized by component. These items should be preserved and built upon.

### Flight State Badge (Toolbar Center)
- "The single best design decision in this tool" -- Morrison (#2)
- "Everything I need in a 200ms glance" -- Morrison (#2)
- "Exactly the first fixation principle" -- Frost (#10)
- "Strongest single safety feature" -- Park (#17)
- "A student reads DISARMED/UNKNOWN and understands immediately" -- Torres Capt (#40)
- "I've wanted this in MP for years" -- Thompson (#27)
- "Hierarchy through affect, not just color" -- Navarro (#18)
- "Best piece of emotional design in the layout" -- Navarro (#18)

### Failsafe Detection Logic (state.js)
- "The best I've seen in any GCS" -- Chen (#1)
- "Smart. Catches unexpected mode changes rather than just logging them" -- Morrison (#2)
- "Clean solution to the basic problem" -- Wu (#7)
- "Genuinely safety-enhancing" -- Schreiber (#9)
- "Shows real operational thinking" -- O'Brien (#5)
- "Shows someone actually understands ArduPilot behavior" -- Mackay (#38)
- "Proactive at the state layer rather than reactive" -- Chen (#1)

### Kill Switch Design
- "The best-designed element in the entire interface" -- Al-Rashid (#16)
- "Correct human factors engineering" -- Thompson (#27)
- "Textbook hazardous-control isolation per MIL-STD-1472" -- Frost (#10)
- "Real-world safety control that most civilian tools get wrong" -- O'Brien (#5)
- "Correct safety affordance" -- Park (#17)
- "Fills up red while you hold it. Sick." -- Park Jayden (#21)

### Tlog Recording (IndexedDB)
- "Solid data recording foundation" -- Wu (#7)
- "This is where Meridian earns its keep" -- Torres Rachel (#8)
- "Correct approach for flight data preservation" -- Frost (#10)
- "The right implementation" -- Volkov Dmitri (#26)
- "Best-in-class design decisions" -- Osei (#24)
- "Better than QGC, which still makes you manually start logging" -- Thompson (#27)
- "Essential when recording 4-hour inspection sorties" -- Lindqvist (#42)

### Demo Mode Auto-Start
- "Genuinely smart" -- Blackwood (#4)
- "The right hook" -- Mei Lin (#19)
- "I did not expect that. Genuinely cool" -- Park Jayden (#21)
- "Removes a significant hardware barrier" -- Torres Capt (#40)
- "Means GCS can be demoed and tested without hardware. QGC requires SITL." -- Volkov Dmitri (#26)

### Dark Theme
- "Looks like a NASA control room" -- Park Jayden (#21)
- "Serious and precise" -- Navarro (#18)
- "Well-executed. Near-black backgrounds, high-contrast text, emergency red that punches through sunlight" -- Morrison (#2)
- "Professional, not hobbyist" -- Mei Lin (#19)
- "A serious palette. Not consumer-grade, not gratuitous" -- Reeves (#20)
- "Mostly solid contrast" -- Williams (#13)

### Health Grid Dual-Channel Encoding
- "The best-designed element in the whole codebase" -- Holloway (#34)
- "Real dual-channel encoding. Someone was thinking about this" -- Holloway (#34)
- "Directly addressing colorblindness under MIL-STD-1472" -- Frost (#10)
- "A detail most GCS designers miss entirely" -- Al-Rashid (#16)
- "The right instinct for accessibility" -- Williams (#13)

### Multi-Vehicle Architecture
- "Ahead of where QGC was at comparable maturity" -- Chen (#1)
- "Multi-vehicle from day one" -- O'Brien (#5), Frost (#10)
- "QGC had to retrofit multi-vehicle and it shows. Meridian's state design is cleaner" -- Volkov Dmitri (#26)
- "First-class multi-vehicle data architecture" -- Nguyen Sam (#41)
- "Real building block" -- Sharma (#30)

### Drag-to-Fly
- "Something I've wanted in a GCS for years" -- Chen (#1)
- "Clever and fast" -- Morrison (#2)
- "More direct than MP's right-click menu" -- Chen (#1)

### Offline Tile Caching
- "Real and usable" -- Chen (#1)
- "Essential and implemented correctly" -- Morrison (#2)
- "Properly thought through" -- Osei (#24)
- "Tactically relevant" -- O'Brien (#5)
- "Already built" -- Zhang (#39)

### Commissioning Checklist
- "The strongest single safety feature in this GCS" -- Torres Rachel (#8)
- "Something QGC does not have" -- Volkov Dmitri (#26)
- "Genuinely the best educational feature" -- Torres Capt (#40)
- "Shows exactly what's done, what's blocking arming" -- Thompson (#27)

### Commanded Trajectory Line
- "I want this in QGC. I have filed a feature request" -- Volkov Dmitri (#26)
- 5-second velocity-vector projection implemented correctly from GLOBAL_POSITION_INT

### Position Uncertainty Ellipse
- "A clean Bret Victor idea. Pilots understand GPS is not a point" -- Volkov Dmitri (#26)
- "Genuinely useful under degraded GPS" -- Lindqvist (#42)

### Architecture Quality
- "Genuinely good code for a no-build application of this complexity" -- Patel (#11)
- "The overlay panel architecture is genuinely better than QGC's hard tab switch" -- Volkov Dmitri (#26)
- "Best-architected open GCS codebase I've reviewed" -- Morrison (#2)
- "Not a prototype, it is a real tool with a few gaps" -- Volkov Dmitri (#26)
- "Clean enough that I would start a pilot integration project" -- Zhang (#39)
- "The code quality here tells me the team has at least one engineer who thinks in systems" -- Brennan (#29)

### Event Bus Design
- "The cleanest thing in this codebase" -- Nguyen Sam (#41)
- "Architecturally solid backbone" -- Patel (#11)
- "Clean pub/sub with errors caught per-handler" -- Patel (#11)

### CSS Variable System / White-Labeling
- "The branding work is genuinely minimal" -- Zhang (#39)
- "Change --c-primary in one place. Typography, spacing, border radii, everything is tokenized" -- Zhang (#39)
- "Disciplined three-font stack" -- Al-Rashid (#16)

### Canvas Instruments
- "Structurally solid. DPR handling right, theme integration works" -- Verou (#33)
- "The ghost overlays, trend arrows, adaptive tick intervals show someone who read Tufte and implemented it" -- Navarro (#18)
- "Genuinely avionics-appropriate rather than generic dashboard widget" -- Verou (#33)
- "Look legitimately good. Sky is blue, ground is brown" -- Park Jayden (#21)

### Leaflet Map Implementation
- "8/10. Genuinely competent Leaflet work" -- Vladimir (#32)
- "API usage is mostly correct and current" -- Vladimir (#32)
- "Rotation implementation avoids the common pitfalls" -- Vladimir (#32)
- "Offline tile layer is a proper subclass rather than monkey-patch" -- Vladimir (#32)

### Resource Efficiency
- "The architecture choices alone would extend battery life 30-45 minutes vs Mission Planner" -- Osei (#24)
- "IndexedDB keeps browser heap flat" -- Osei (#24)
- "Plain HTML/CSS/JS with no framework overhead" -- Osei (#24)

### Slide-to-Arm Design
- "Clever design" -- Blackwood (#4)
- "Position is an excellent non-color cue" -- Holloway (#34)
- "Good touch target. The gesture helps cover precision" -- Delgado (#35)
- "No idle animation (Rams: no idle animation) -- exactly right" -- Navarro (#18)

### Insurance/Compliance Value
- "Insurability score: 8/10. I would offer 10-15% premium discount" -- Torres Rachel (#8)
- "Replay-capable, timestamped, IndexedDB-persistent logs make claims investigation tractable" -- Torres Rachel (#8)

### OEM / White-Label Value
- "8/10 for OEM adoption. Extension architecture is sound" -- Zhang (#39)
- "Event bus means I can add proprietary sensor panels without forking the core" -- Zhang (#39)

### Research Extensibility
- "7/10. Well-structured, genuinely hackable GCS" -- Russo (#44)
- "Leaflet map exposure, open event bus, direct vehicle state mutation give real extensibility" -- Russo (#44)
- "StatusView Other section renders any unknown keys automatically" -- Russo (#44)

---

*End of Master GCS Review Report. 45 reviewers. 285+ individual findings. 65+ prioritized fixes across 4 tiers.*
