/* ============================================================
   toolbar.js — Toolbar + Primary Flight State Badge
   Krug: ARM STATE + MODE is the first thing the pilot reads.
   ============================================================ */

'use strict';

window.Toolbar = (function () {

    let stripEl, telemModeEl, telemStateEl;
    let tvGps, tvBatt, tvRc, tvEkf, tvTime;
    let tgGps, tgBatt, tgRc, tgEkf;
    let connEl, connDotEl, connTextEl;
    let recEl, recTimeEl;
    let recStartTime = null;
    let recTimer = null;

    function init() {
        stripEl = document.getElementById('telem-strip');
        telemModeEl = document.getElementById('telem-mode');
        telemStateEl = document.getElementById('telem-state');
        tvGps = document.getElementById('tv-gps');
        tvBatt = document.getElementById('tv-batt');
        tvRc = document.getElementById('tv-rc');
        tvEkf = document.getElementById('tv-ekf');
        tvTime = document.getElementById('tv-time');
        tgGps = document.getElementById('telem-gps');
        tgBatt = document.getElementById('telem-batt');
        tgRc = document.getElementById('telem-rc');
        tgEkf = document.getElementById('telem-ekf');
        connEl = document.querySelector('.conn-indicator');
        connDotEl = connEl ? connEl.querySelector('.conn-dot') : null;
        connTextEl = connEl ? connEl.querySelector('.conn-text') : null;
        recEl = document.querySelector('.rec-indicator');
        recTimeEl = recEl ? recEl.querySelector('.rec-time') : null;

        // Connection click
        if (connEl) {
            connEl.addEventListener('click', async () => {
                if (meridian.connectionState === 2) {
                    Connection.disconnect();
                } else {
                    const url = await Modal.prompt('Connect', 'Enter WebSocket URL:', 'ws://localhost:5760');
                    if (url) Connection.connect(url, 'mavlink');
                }
            });
        }

        // Listen for state changes
        meridian.events.on('heartbeat', updateBadge);
        meridian.events.on('battery', updateBadge);
        meridian.events.on('gps', updateBadge);

        Connection.onStateChange = updateConnection;

        updateBadge();
        updateConnection(0);
    }

    function updateBadge() {
        const v = meridian.v;
        if (!v || !stripEl) return;

        // Strip arm state
        stripEl.className = 'telem-strip';
        if (v.armed) stripEl.classList.add('armed');

        // Mode + arm state — compact
        const modeText = v.modeName || '???';
        if (telemModeEl) {
            telemModeEl.textContent = (v.armed ? 'ARM' : 'DSRM') + ' \u00b7 ' + modeText;
        }

        // GPS: fix type + sats
        const fixNames = ['No GPS', 'No Fix', '2D', '3D', 'DGPS', 'RTK\u2009F', 'RTK'];
        if (tvGps) tvGps.textContent = (fixNames[v.fixType] || '--') + ' ' + v.satellites + 'S';
        if (tgGps) {
            tgGps.className = 'telem-group' + (v.fixType >= 3 ? ' ok' : (v.fixType >= 2 ? ' warn' : ' bad'));
            tgGps.title = 'GPS: ' + (fixNames[v.fixType] || 'Unknown') + ', ' + v.satellites + ' sats, HDOP ' + v.hdop.toFixed(1);
        }

        // Battery
        const bPct = v.batteryPct >= 0 ? v.batteryPct : 0;
        if (tvBatt) tvBatt.textContent = Math.round(bPct) + '% ' + v.voltage.toFixed(1) + 'V';
        if (tgBatt) {
            tgBatt.className = 'telem-group' + (bPct > 40 ? ' ok' : (bPct > 20 ? ' warn' : ' bad'));
            tgBatt.title = 'Battery: ' + bPct + '%, ' + v.voltage.toFixed(2) + 'V, ' + v.current.toFixed(1) + 'A';
        }

        // RC RSSI
        const rcPct = Math.round(v.rcRssi / 255 * 100);
        if (tvRc) tvRc.textContent = rcPct + '%';
        if (tgRc) {
            tgRc.className = 'telem-group' + (rcPct > 50 ? ' ok' : (rcPct > 20 ? ' warn' : ' bad'));
            tgRc.title = 'RC signal: ' + rcPct + '% (raw ' + v.rcRssi + '/255)';
        }

        // EKF
        const ekfMax = Math.max(v.ekfVelVar || 0, v.ekfPosVar || 0, v.ekfHgtVar || 0);
        if (tvEkf) tvEkf.textContent = ekfMax.toFixed(2);
        if (tgEkf) {
            tgEkf.className = 'telem-group' + (ekfMax < 0.5 ? ' ok' : (ekfMax < 0.8 ? ' warn' : ' bad'));
            tgEkf.title = 'EKF max variance: ' + ekfMax.toFixed(3);
        }

        // Flight time
        let timeStr = '--:--';
        if (v.flightStartTime) {
            const s = Math.floor((Date.now() - v.flightStartTime) / 1000);
            timeStr = Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0');
        }
        if (tvTime) tvTime.textContent = timeStr;
    }

    function updateConnection(state) {
        if (!connEl) return;
        connEl.className = 'conn-indicator';
        if (meridian.demo) {
            connEl.classList.add('connecting');
            if (connTextEl) connTextEl.textContent = 'DEMO — click to connect';
            return;
        }
        if (state === 2) {
            connEl.classList.add('connected');
            if (connTextEl) connTextEl.textContent = 'CONNECTED';
        } else if (state === 1) {
            connEl.classList.add('connecting');
            if (connTextEl) connTextEl.textContent = 'CONNECTING';
        } else {
            connEl.classList.add('disconnected');
            if (connTextEl) connTextEl.textContent = 'DISCONNECTED';
        }
    }

    function startRecording() {
        if (recEl) recEl.classList.add('active');
        recStartTime = Date.now();
        recTimer = setInterval(updateRecTime, 1000);
    }

    function stopRecording() {
        if (recEl) recEl.classList.remove('active');
        clearInterval(recTimer);
        recStartTime = null;
    }

    function updateRecTime() {
        if (!recTimeEl || !recStartTime) return;
        const s = Math.floor((Date.now() - recStartTime) / 1000);
        recTimeEl.textContent = Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0');
    }

    return { init, updateBadge, updateConnection, startRecording, stopRecording };

})();
