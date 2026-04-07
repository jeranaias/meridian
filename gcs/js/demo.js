/* ============================================================
   demo.js — Simulated vehicle for testing without FC
   Generates realistic copter telemetry: orbit pattern,
   attitude oscillation, GPS, battery drain.
   ============================================================ */

'use strict';

window.Demo = (function () {

    let timer = null;
    let t = 0;
    const DT = 0.1; // 10Hz

    // Base position (Camp Roberts, CA)
    const BASE_LAT = 35.7516;
    const BASE_LON = -120.7710;
    const ORBIT_R = 0.0015; // ~150m radius — bigger orbit for visual interest

    function start() {
        meridian.demo = true;
        Toolbar.updateConnection(1); // Show DEMO state

        const v = meridian.v;
        v.connected = true;
        v.armed = true;
        v.lastHeartbeat = Date.now();
        v.modeNum = 5;
        v.modeName = 'LOITER';
        v.homeLat = BASE_LAT;
        v.homeLon = BASE_LON;
        v.homeAlt = 350;
        v.flightStartTime = Date.now() - (2 * 60 + 37) * 1000; // 2:37 simulated flight time
        v.satellites = 14;
        v.fixType = 3;
        v.hdop = 0.8;
        v.vdop = 1.2;
        v.voltage = 12.4;
        v.current = 18.2;
        v.batteryPct = 72;
        v.ekfVelVar = 0.08;
        v.ekfPosVar = 0.12;
        v.ekfHgtVar = 0.05;
        v.ekfMagVar = 0.04;
        v.ekfTerrVar = 0.01;
        v.ekfFlags = 0x1FF;
        v.rcRssi = 230;
        v.rcChannels = [1500, 1500, 1500, 1500, 1000, 1500, 1500, 1500];

        // Second vehicle (sysid 2) — AUTO mode, offset orbit
        meridian.vehicles[2] = meridian.createVehicleState(2);
        var v2 = meridian.vehicles[2];
        v2.connected = true;
        v2.armed = true;
        v2.lastHeartbeat = Date.now();
        v2.modeNum = 3;
        v2.modeName = 'AUTO';
        v2.homeLat = BASE_LAT + 0.003;
        v2.homeLon = BASE_LON + 0.003;
        v2.homeAlt = 350;
        v2.flightStartTime = Date.now() - 90000;
        v2.satellites = 12;
        v2.fixType = 3;
        v2.hdop = 1.1;
        v2.vdop = 1.5;
        v2.voltage = 11.8;
        v2.current = 20.5;
        v2.batteryPct = 58;
        v2.ekfVelVar = 0.10;
        v2.ekfPosVar = 0.15;
        v2.ekfHgtVar = 0.07;
        v2.ekfFlags = 0x1FF;
        v2.rcRssi = 200;

        // Demo params for Setup/Params views
        v.params = {
            FRAME_CLASS: 1, FRAME_TYPE: 1,
            ARMING_CHECK: 1,
            INS_ACCOFFS_X: 0.12, INS_ACCOFFS_Y: -0.08, INS_ACCOFFS_Z: 0.34,
            COMPASS_OFS_X: 42.5, COMPASS_OFS_Y: -18.3, COMPASS_OFS_Z: 105.7,
            RC1_MIN: 1015, RC1_MAX: 1985,
            RC2_MIN: 1020, RC2_MAX: 1980,
            RC3_MIN: 1010, RC3_MAX: 1990,
            RC4_MIN: 1025, RC4_MAX: 1975,
            RC5_MIN: 1000, RC5_MAX: 2000,
            RC6_MIN: 1000, RC6_MAX: 2000,
            RC7_MIN: 1000, RC7_MAX: 2000,
            RC8_MIN: 1000, RC8_MAX: 2000,
            FLTMODE1: 0, FLTMODE2: 2, FLTMODE3: 5, FLTMODE4: 6, FLTMODE5: 3, FLTMODE6: 9,
            FLTMODE_CH: 5,
            FS_THR_ENABLE: 1, FS_THR_VALUE: 975,
            BATT_FS_LOW_ACT: 2, BATT_LOW_VOLT: 10.5, BATT_LOW_MAH: 0,
            BATT_FS_CRT_ACT: 1, BATT_CRT_VOLT: 10.0,
            FS_GCS_ENABLE: 1, FS_GCS_TIMEOUT: 5,
            BATT_MONITOR: 4, BATT_CAPACITY: 5200, BATT_CELL_COUNT: 4,
            BATT_AMP_PERVLT: 17, BATT_VOLT_MULT: 10.1,
            ATC_RAT_RLL_P: 0.135, ATC_RAT_RLL_I: 0.135, ATC_RAT_RLL_D: 0.0036,
            ATC_RAT_PIT_P: 0.135, ATC_RAT_PIT_I: 0.135, ATC_RAT_PIT_D: 0.0036,
            ATC_RAT_YAW_P: 0.18, ATC_RAT_YAW_I: 0.018, ATC_RAT_YAW_D: 0.0,
            ATC_ACCEL_R_MAX: 110000, ATC_ACCEL_P_MAX: 110000, ATC_ACCEL_Y_MAX: 27000,
            MOT_SPIN_ARM: 0.1, MOT_SPIN_MIN: 0.15,
            PILOT_SPEED_UP: 250, PILOT_SPEED_DN: 150,
            WPNAV_SPEED: 500, WPNAV_SPEED_DN: 150, WPNAV_SPEED_UP: 250,
            WPNAV_RADIUS: 200, WPNAV_ACCEL: 100,
            RTL_ALT: 1500, RTL_SPEED: 0,
            LAND_SPEED: 50,
            FENCE_ENABLE: 1, FENCE_TYPE: 7, FENCE_ALT_MAX: 100,
            SERIAL0_BAUD: 115, SERIAL0_PROTOCOL: 2,
            LOG_BITMASK: 176126, LOG_DISARMED: 0,
        };

        meridian.events.emit('heartbeat', v);
        meridian.events.emit('home_changed', v);
        meridian.log('Demo mode active — simulated copter in LOITER', 'info');

        timer = setInterval(tick, DT * 1000);
    }

    function tick() {
        t += DT;
        const v = meridian.v;
        if (!v) return;

        // Orbit pattern
        const angle = t * 0.15; // slow orbit
        const lat = BASE_LAT + ORBIT_R * Math.cos(angle);
        const lon = BASE_LON + ORBIT_R * Math.sin(angle) / Math.cos(BASE_LAT * Math.PI / 180);

        // Altitude: gentle oscillation around 25m
        const alt = 25 + 2 * Math.sin(t * 0.3);

        // Heading: tangent to orbit
        const hdg = ((Math.atan2(Math.cos(angle), -Math.sin(angle)) * 180 / Math.PI) + 360) % 360;

        // Attitude: gentle oscillation
        const roll = 0.08 * Math.sin(t * 0.7) + 0.02 * Math.sin(t * 2.3);
        const pitch = -0.05 + 0.04 * Math.cos(t * 0.9);
        const yaw = hdg * Math.PI / 180;

        // Speed
        const gs = 3.5 + 0.5 * Math.sin(t * 0.4);
        const climb = 0.3 * Math.cos(t * 0.3);

        // Update vehicle state
        v.lat = lat;
        v.lon = lon;
        v.alt = 350 + alt;
        v.relativeAlt = alt;
        v.heading = hdg;
        v.hdg = hdg;
        v.roll = roll;
        v.pitch = pitch;
        v.yaw = yaw;
        v.rollspeed = 0.02 * Math.cos(t * 0.7);
        v.pitchspeed = -0.01 * Math.sin(t * 0.9);
        v.yawspeed = 0.005;
        // Simulated wind: 3.5 m/s from 240° (southwest wind)
        const windSpeed = 3.5;
        const windFromDeg = 240;
        const windToRad = ((windFromDeg + 180) % 360) * Math.PI / 180;
        const windE = windSpeed * Math.sin(windToRad); // east component
        const windN = windSpeed * Math.cos(windToRad); // north component

        v.airspeed = gs;
        // Ground velocity = airspeed vector + wind vector
        v.vx = gs * Math.sin(yaw) + windE; // east (MAVLink NED: vx=north but display convention varies)
        v.vy = gs * Math.cos(yaw) + windN; // north
        v.groundspeed = Math.sqrt(v.vx * v.vx + v.vy * v.vy);
        v.climb = climb;
        v.throttle = 52 + Math.round(5 * Math.sin(t * 0.5));
        v.vz = climb / 100;

        // Battery drain
        v.batteryPct = Math.max(0, 72 - t * 0.01);
        v.voltage = 12.4 - t * 0.0005;
        v.current = 18.2 + 2 * Math.sin(t * 0.3);
        v.mah = t * 3;

        // Ghost targets (Victor: target-vs-actual visible in instruments)
        v.targetAlt = 25;
        v.targetSpeed = 3.5;
        v.targetRoll = 0.03 * Math.sin(t * 0.5);
        v.targetPitch = -0.04;

        // Slight EKF variance changes
        v.ekfVelVar = 0.08 + 0.03 * Math.sin(t * 0.2);
        v.ekfPosVar = 0.12 + 0.04 * Math.sin(t * 0.15);

        // Trail
        v.trail.push([lat, lon]);
        if (v.trail.length > 1000) v.trail.shift();

        // Heartbeat every ~1s
        if (Math.round(t * 10) % 10 === 0) {
            v.lastHeartbeat = Date.now();
            meridian.events.emit('heartbeat', v);
        }

        // Emit events
        meridian.events.emit('attitude', v);
        meridian.events.emit('position', v);
        meridian.events.emit('vfr_hud', v);
        meridian.events.emit('battery', v);
        meridian.events.emit('gps', v);
        meridian.events.emit('telemetry', v);

        // Vehicle 2 update
        var v2 = meridian.vehicles[2];
        if (v2) {
            var angle2 = -t * 0.08;
            var lat2 = (BASE_LAT + 0.003) + 0.0008 * Math.cos(angle2);
            var lon2 = (BASE_LON + 0.003) + 0.0008 * Math.sin(angle2) / Math.cos(BASE_LAT * Math.PI / 180);
            var alt2 = 35 + 3 * Math.sin(t * 0.2);
            var hdg2 = ((Math.atan2(Math.cos(angle2), Math.sin(angle2)) * 180 / Math.PI) + 360) % 360;

            v2.lat = lat2;
            v2.lon = lon2;
            v2.alt = 350 + alt2;
            v2.relativeAlt = alt2;
            v2.heading = hdg2;
            v2.hdg = hdg2;
            v2.roll = 0.05 * Math.sin(t * 0.5);
            v2.pitch = -0.03;
            v2.yaw = hdg2 * Math.PI / 180;
            v2.groundspeed = 2.5;
            v2.airspeed = 2.8;
            v2.throttle = 48;
            v2.batteryPct = Math.max(0, 58 - t * 0.012);
            v2.voltage = 11.8 - t * 0.0006;
            v2.lastHeartbeat = Date.now();
            v2.trail.push([lat2, lon2]);
            if (v2.trail.length > 300) v2.trail.shift();
        }
    }

    function stop() {
        if (timer) clearInterval(timer);
        timer = null;
        meridian.demo = false;
    }

    function isActive() { return timer !== null; }

    // T2-7: Demo fault injection — keyboard shortcuts for simulating failures
    // Ctrl+Shift+G = GPS loss, Ctrl+Shift+B = battery critical,
    // Ctrl+Shift+R = RC failsafe, Ctrl+Shift+N = reset all faults
    let _faultActive = {};

    function injectFault(name) {
        const v = meridian.v;
        if (!v) return;

        switch (name) {
            case 'gps_loss':
                _faultActive.gps = true;
                v.fixType = 0;
                v.satellites = 0;
                v.hdop = 99.9;
                meridian.log('[DEMO FAULT] GPS loss injected — fixType=0, satellites=0', 'warn');
                meridian.events.emit('gps', v);
                break;

            case 'battery_critical':
                _faultActive.battery = true;
                v.batteryPct = 10;
                v.voltage = 9.5;
                meridian.log('[DEMO FAULT] Battery critical injected — 10%, 9.5V', 'warn');
                meridian.events.emit('battery', v);
                break;

            case 'rc_failsafe':
                _faultActive.rc = true;
                v.rcRssi = 0;
                v._userModeChange = false; // ensure failsafe detection fires
                v.modeName = 'RTL';
                v.modeNum = 6;
                meridian.log('[DEMO FAULT] RC failsafe injected — RSSI=0, mode→RTL', 'warn');
                meridian.events.emit('heartbeat', v);
                meridian.events.emit('rc', v);
                break;

            case 'reset':
                _faultActive = {};
                v.fixType = 3;
                v.satellites = 14;
                v.hdop = 0.8;
                v.batteryPct = 72;
                v.voltage = 12.4;
                v.rcRssi = 230;
                v.modeName = 'LOITER';
                v.modeNum = 5;
                meridian.log('[DEMO FAULT] All faults reset to normal', 'info');
                meridian.events.emit('gps', v);
                meridian.events.emit('battery', v);
                meridian.events.emit('heartbeat', v);
                meridian.events.emit('rc', v);
                break;
        }
    }

    function _onFaultKey(e) {
        if (!meridian.demo) return;
        if (!e.ctrlKey || !e.shiftKey) return;

        switch (e.key) {
            case 'G': case 'g':
                e.preventDefault();
                injectFault('gps_loss');
                break;
            case 'B': case 'b':
                e.preventDefault();
                injectFault('battery_critical');
                break;
            case 'R': case 'r':
                e.preventDefault();
                injectFault('rc_failsafe');
                break;
            case 'N': case 'n':
                e.preventDefault();
                injectFault('reset');
                break;
        }
    }

    document.addEventListener('keydown', _onFaultKey);

    return { start, stop, isActive, injectFault };

})();
