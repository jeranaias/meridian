/* ============================================================
   mavlink-inspector.js — Live MAVLink message inspector
   Shows last N messages with timestamp, type, key fields.
   Filterable by message type. Updates at 4Hz max.
   ============================================================ */

'use strict';

window.MavlinkInspector = (function () {

    let container = null;
    let messageLog = [];
    let maxMessages = 200;
    let filterType = '';
    let paused = false;
    let updateTimer = null;
    let listEl = null;
    let dirty = false;
    let countEl = null;

    // Message type stats
    let typeCounts = {};

    function render(cont) {
        container = cont;
        container.innerHTML = '';
        messageLog = [];
        typeCounts = {};
        paused = false;
        filterType = '';

        const wrapper = document.createElement('div');
        wrapper.className = 'inspector-panel';

        // Controls
        const controls = document.createElement('div');
        controls.className = 'inspector-controls';

        const filterInput = document.createElement('input');
        filterInput.type = 'text';
        filterInput.className = 'inspector-filter';
        filterInput.placeholder = 'Filter by msg type...';
        filterInput.addEventListener('input', function () {
            filterType = filterInput.value.toLowerCase().trim();
            dirty = true;
        });
        controls.appendChild(filterInput);

        const pauseBtn = document.createElement('button');
        pauseBtn.className = 'log-action-btn';
        pauseBtn.textContent = 'Pause';
        pauseBtn.addEventListener('click', function () {
            paused = !paused;
            pauseBtn.textContent = paused ? 'Resume' : 'Pause';
            if (paused) pauseBtn.classList.add('active');
            else pauseBtn.classList.remove('active');
        });
        controls.appendChild(pauseBtn);

        const clearBtn = document.createElement('button');
        clearBtn.className = 'log-action-btn';
        clearBtn.textContent = 'Clear';
        clearBtn.addEventListener('click', function () {
            messageLog = [];
            typeCounts = {};
            dirty = true;
        });
        controls.appendChild(clearBtn);

        countEl = document.createElement('span');
        countEl.className = 'inspector-count';
        countEl.textContent = '0 msgs';
        controls.appendChild(countEl);

        wrapper.appendChild(controls);

        // Message list
        listEl = document.createElement('div');
        listEl.className = 'inspector-list';
        wrapper.appendChild(listEl);

        container.appendChild(wrapper);

        // Listen for telemetry events
        meridian.events.on('telemetry', onTelemetry);

        // Render at 4Hz
        if (updateTimer) clearInterval(updateTimer);
        updateTimer = setInterval(renderList, 250);
    }

    function onTelemetry(v) {
        if (paused) return;

        // We intercept the last processed message type from the event
        // Since the event bus fires after handleMessage, we capture the vehicle state
        // We need to capture from the raw message flow instead
        // Use a different approach: hook into all known event types
    }

    // Attach to specific events to capture message details
    function attachListeners() {
        const types = ['heartbeat', 'attitude', 'position', 'vfr_hud', 'battery',
                       'gps', 'ekf', 'rc', 'param', 'command_ack'];

        types.forEach(function (type) {
            meridian.events.on(type, function (data) {
                if (paused) return;
                const entry = {
                    time: Date.now(),
                    type: type,
                    data: summarize(type, data),
                };
                messageLog.push(entry);
                if (messageLog.length > maxMessages) messageLog.shift();

                typeCounts[type] = (typeCounts[type] || 0) + 1;
                dirty = true;
            });
        });
    }

    function summarize(type, v) {
        switch (type) {
            case 'heartbeat':
                return (v.armed ? 'ARMED' : 'DISARMED') + ' ' + v.modeName;
            case 'attitude':
                return 'R:' + deg(v.roll) + ' P:' + deg(v.pitch) + ' Y:' + deg(v.yaw);
            case 'position':
                return v.lat.toFixed(6) + ',' + v.lon.toFixed(6) + ' alt:' + v.relativeAlt.toFixed(1);
            case 'vfr_hud':
                return 'gs:' + v.groundspeed.toFixed(1) + ' alt:' + v.alt.toFixed(1) + ' thr:' + v.throttle + '%';
            case 'battery':
                return v.voltage.toFixed(1) + 'V ' + v.current.toFixed(1) + 'A ' + v.batteryPct + '%';
            case 'gps':
                return 'fix:' + v.fixType + ' sat:' + v.satellites + ' hdop:' + v.hdop.toFixed(1);
            case 'ekf':
                return 'vel:' + v.ekfVelVar.toFixed(3) + ' pos:' + v.ekfPosVar.toFixed(3) + ' hgt:' + v.ekfHgtVar.toFixed(3);
            case 'rc':
                return v.rcChannels.length + 'ch rssi:' + v.rcRssi;
            case 'param':
                return (v.name || '?') + '=' + (v.value !== undefined ? v.value : '?');
            case 'command_ack':
                return 'cmd:' + (v.command || '?') + ' result:' + (v.result || '?');
            default:
                return JSON.stringify(v).substring(0, 60);
        }
    }

    function deg(rad) {
        return (rad * 180 / Math.PI).toFixed(1) + '\u00b0';
    }

    // T2-21: Escape HTML to prevent XSS from crafted FC messages
    function escapeHtml(str) {
        if (str === null || str === undefined) return '';
        return String(str)
            .replace(/&/g, '&amp;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;')
            .replace(/'/g, '&#39;');
    }

    function renderList() {
        if (!dirty || !listEl) return;
        dirty = false;

        const filtered = filterType
            ? messageLog.filter(function (m) { return m.type.toLowerCase().indexOf(filterType) >= 0; })
            : messageLog;

        // Show most recent at top, limit display to 50
        const display = filtered.slice(-50).reverse();

        let html = '';
        for (let i = 0; i < display.length; i++) {
            const m = display[i];
            const t = new Date(m.time);
            const timeStr = t.getMinutes() + ':' + String(t.getSeconds()).padStart(2, '0') + '.' +
                            String(Math.floor(t.getMilliseconds() / 100));
            // T2-21: Escape all dynamic content to prevent XSS from crafted FC messages
            html += '<div class="inspector-row">' +
                '<span class="inspector-time">' + escapeHtml(timeStr) + '</span>' +
                '<span class="inspector-type">' + escapeHtml(m.type) + '</span>' +
                '<span class="inspector-data">' + escapeHtml(m.data) + '</span>' +
                '</div>';
        }

        listEl.innerHTML = html;

        if (countEl) {
            const totalTypes = Object.keys(typeCounts).length;
            countEl.textContent = messageLog.length + ' msgs \u00b7 ' + totalTypes + ' types';
        }
    }

    // Init: attach listeners immediately so they're ready when render() is called
    attachListeners();

    return { render };

})();
