/* ============================================================
   status.js — System health grid
   Tufte: show numbers always, not just colored dots.
   Format: "GPS 14/0.8", "EKF 0.12", "RC 98%"
   ============================================================ */

'use strict';

window.HealthStatus = (function () {

    let el;

    const items = [
        { id: 'gps', label: 'GPS' },
        { id: 'ekf', label: 'EKF' },
        { id: 'rc',  label: 'RC' },
        { id: 'imu', label: 'IMU' },
        { id: 'bar', label: 'BAR' },
        { id: 'mag', label: 'MAG' },
        { id: 'rid', label: 'RID' },
    ];

    function init(container) {
        el = container;
        el.innerHTML = items.map(item =>
            `<div class="health-item" id="health-${item.id}">
                <span class="health-dot"></span>
                <span class="health-label">${item.label}</span>
                <span class="health-value" id="hval-${item.id}">--</span>
            </div>`
        ).join('');
    }

    function setItem(id, value, status, tooltip) {
        const item = document.getElementById('health-' + id);
        const val = document.getElementById('hval-' + id);
        if (!item || !val) return;
        item.className = 'health-item ' + status;
        item.title = tooltip || '';
        // Tufte: dual-channel encoding (color + shape) for colorblind accessibility
        const dot = item.querySelector('.health-dot');
        if (dot) {
            if (status === 'warn') dot.textContent = '\u25B2'; // triangle
            else if (status === 'bad') dot.textContent = '\u2716'; // X
            else dot.textContent = '';
            dot.style.fontSize = status === '' ? '' : '6px';
            dot.style.width = status === 'warn' || status === 'bad' ? 'auto' : '';
            dot.style.height = status === 'warn' || status === 'bad' ? 'auto' : '';
        }
        val.textContent = value;
    }

    function update(v) {
        if (!el) return;

        // GPS: sats/hdop
        const fixOk = v.fixType >= 3;
        const gpsStr = v.satellites + '/' + v.hdop.toFixed(1);
        setItem('gps', gpsStr, fixOk ? 'ok' : (v.fixType >= 2 ? 'warn' : 'bad'),
            'Satellites/HDOP \u2014 Fix: ' + ['None','No Fix','2D','3D','DGPS','RTK Float','RTK'][v.fixType || 0]);

        // EKF: max variance
        const ekfMax = Math.max(v.ekfVelVar, v.ekfPosVar, v.ekfHgtVar);
        const ekfStr = ekfMax.toFixed(2);
        setItem('ekf', ekfStr, ekfMax < 0.5 ? 'ok' : (ekfMax < 0.8 ? 'warn' : 'bad'),
            'EKF variance (vel:' + v.ekfVelVar.toFixed(2) + ' pos:' + v.ekfPosVar.toFixed(2) + ' hgt:' + v.ekfHgtVar.toFixed(2) + ')');

        // RC: RSSI %
        const rcPct = Math.round(v.rcRssi / 255 * 100);
        setItem('rc', rcPct + '%', rcPct > 50 ? 'ok' : (rcPct > 20 ? 'warn' : 'bad'),
            'RC signal strength: ' + rcPct + '% (raw: ' + v.rcRssi + '/255)');

        // IMU: show "--" unless we have real vibration data
        if (v.imuVibe !== undefined && v.imuVibe > 0) {
            setItem('imu', v.imuVibe.toFixed(1), v.imuVibe < 30 ? 'ok' : (v.imuVibe < 60 ? 'warn' : 'bad'), 'IMU vibration (m/s\u00B2)');
        } else {
            setItem('imu', 'N/A', '', 'No vibration data received');
        }

        // Baro: requires SYS_STATUS bitmask — not yet decoded
        setItem('bar', 'N/A', '', 'No barometer health data');

        // Mag: OK if compass variance is low
        const magOk = v.ekfMagVar < 0.3;
        setItem('mag', v.ekfMagVar > 0 ? v.ekfMagVar.toFixed(2) : 'OK',
            magOk ? 'ok' : (v.ekfMagVar < 0.6 ? 'warn' : 'bad'), 'Compass variance');

        // T1-12: Remote ID status
        if (v.remoteId && v.remoteId.uasId) {
            setItem('rid', 'OK', 'ok', 'Remote ID broadcasting');
        } else {
            setItem('rid', 'N/A', '', 'No Remote ID module detected');
        }
    }

    return { init, update };

})();
