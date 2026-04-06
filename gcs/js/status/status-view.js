/* ============================================================
   status-view.js — Raw telemetry view
   200+ field display organized by category, 4Hz update,
   searchable, compact monospace layout.
   ============================================================ */

'use strict';

window.StatusView = (function () {

    let container = null;
    let filterText = '';
    let updateTimer = null;
    let fieldEls = {};    // key -> value element for direct DOM updates
    let initialized = false;

    // Categories and their fields from meridian.v
    const CATEGORIES = {
        'System': [
            'sysid', 'connected', 'armed', 'modeNum', 'modeName', 'systemStatus',
            'lastHeartbeat', 'flightStartTime',
        ],
        'Attitude': [
            'roll', 'pitch', 'yaw',
            'rollspeed', 'pitchspeed', 'yawspeed',
            'targetRoll', 'targetPitch',
        ],
        'Position': [
            'lat', 'lon', 'alt', 'relativeAlt',
            'vx', 'vy', 'vz', 'hdg',
            'homeLat', 'homeLon', 'homeAlt',
        ],
        'VFR HUD': [
            'airspeed', 'groundspeed', 'heading', 'throttle', 'climb',
            'targetAlt', 'targetSpeed',
        ],
        'Battery': [
            'voltage', 'current', 'batteryPct', 'mah',
        ],
        'GPS': [
            'fixType', 'satellites', 'hdop', 'vdop',
            'gpsLat', 'gpsLon',
        ],
        'EKF': [
            'ekfVelVar', 'ekfPosVar', 'ekfHgtVar',
            'ekfMagVar', 'ekfTerrVar', 'ekfFlags',
        ],
        'RC': [
            'rcRssi',
        ],
        'Mission': [
            'missionCount', 'missionSeq',
        ],
    };

    function render() {
        container = document.querySelector('#panel-status .panel-body');
        if (!container) return;
        container.innerHTML = '';
        fieldEls = {};

        // Search bar
        const searchBar = document.createElement('div');
        searchBar.className = 'status-search';
        const searchInput = document.createElement('input');
        searchInput.type = 'text';
        searchInput.className = 'status-search-input';
        searchInput.placeholder = 'Filter fields...';
        searchInput.addEventListener('input', function () {
            filterText = searchInput.value.toLowerCase().trim();
            applyFilter();
        });
        searchBar.appendChild(searchInput);

        const countEl = document.createElement('span');
        countEl.className = 'status-field-count';
        countEl.id = 'status-field-count';
        searchBar.appendChild(countEl);

        container.appendChild(searchBar);

        // Build category sections
        const v = meridian.v;
        const body = document.createElement('div');
        body.className = 'status-body';

        // Known categories
        for (const cat in CATEGORIES) {
            const section = createSection(cat, CATEGORIES[cat]);
            body.appendChild(section);
        }

        // RC channels (dynamic)
        if (v && v.rcChannels && v.rcChannels.length > 0) {
            const rcFields = [];
            for (let i = 0; i < v.rcChannels.length; i++) {
                rcFields.push('rcCh' + (i + 1));
            }
            const section = createSection('RC Channels', rcFields);
            body.appendChild(section);
        }

        // Parameters (dynamic, can be 200+)
        if (v && v.params && Object.keys(v.params).length > 0) {
            const paramKeys = Object.keys(v.params).sort();
            const section = createSection('Parameters (' + paramKeys.length + ')', paramKeys.map(function (k) { return 'param:' + k; }));
            body.appendChild(section);
        }

        // Any unknown fields on v
        if (v) {
            const known = new Set();
            for (const cat in CATEGORIES) {
                CATEGORIES[cat].forEach(function (f) { known.add(f); });
            }
            known.add('trail'); known.add('rcChannels'); known.add('params');
            known.add('missionItems'); known.add('_userModeChange'); known.add('_staleLogged');
            known.add('paramCount');

            const extra = Object.keys(v).filter(function (k) { return !known.has(k) && typeof v[k] !== 'function'; });
            if (extra.length > 0) {
                const section = createSection('Other', extra);
                body.appendChild(section);
            }
        }

        container.appendChild(body);
        initialized = true;

        updateValues();
        updateFieldCount();

        // Start 4Hz update
        if (updateTimer) clearInterval(updateTimer);
        updateTimer = setInterval(updateValues, 250);
    }

    function createSection(title, fields) {
        const section = document.createElement('div');
        section.className = 'status-section';

        const header = document.createElement('div');
        header.className = 'status-section-header';
        header.textContent = title;
        header.addEventListener('click', function () {
            section.classList.toggle('collapsed');
        });
        section.appendChild(header);

        const grid = document.createElement('div');
        grid.className = 'status-grid';

        fields.forEach(function (field) {
            const row = document.createElement('div');
            row.className = 'status-row';
            row.dataset.field = field.toLowerCase();

            const nameEl = document.createElement('span');
            nameEl.className = 'status-field-name';
            nameEl.textContent = field.replace('param:', '');

            const valEl = document.createElement('span');
            valEl.className = 'status-field-value';
            valEl.textContent = '--';

            row.appendChild(nameEl);
            row.appendChild(valEl);
            grid.appendChild(row);

            fieldEls[field] = valEl;
        });

        section.appendChild(grid);
        return section;
    }

    function updateValues() {
        const v = meridian.v;
        if (!v) return;

        for (const field in fieldEls) {
            let val;
            if (field.startsWith('param:')) {
                val = v.params[field.substring(6)];
            } else if (field.startsWith('rcCh')) {
                const idx = parseInt(field.substring(4)) - 1;
                val = v.rcChannels ? v.rcChannels[idx] : undefined;
            } else {
                val = v[field];
            }

            const el = fieldEls[field];
            const text = formatValue(field, val);
            if (el.textContent !== text) {
                el.textContent = text;
                // Flash on change
                el.classList.add('changed');
                setTimeout(function () { el.classList.remove('changed'); }, 300);
            }
        }
    }

    function formatValue(field, val) {
        if (val === null || val === undefined) return '--';
        if (typeof val === 'boolean') return val ? 'TRUE' : 'FALSE';
        if (typeof val === 'number') {
            // Angles in radians: convert to degrees
            if (field === 'roll' || field === 'pitch' || field === 'yaw' ||
                field === 'rollspeed' || field === 'pitchspeed' || field === 'yawspeed' ||
                field === 'targetRoll' || field === 'targetPitch') {
                return (val * 180 / Math.PI).toFixed(2) + '\u00b0';
            }
            // Altitude check (before lat/lon since 'relativeAlt' contains 'lat')
            if (field === 'alt' || field === 'relativeAlt' || field === 'homeAlt' || field === 'targetAlt') return val.toFixed(1) + ' m';
            // Lat/lon: 7 decimal places
            if ((field === 'lat' || field === 'lon' || field === 'homeLat' || field === 'homeLon' || field === 'gpsLat' || field === 'gpsLon')) {
                return val.toFixed(7);
            }
            // Variances: 4 decimals
            if (field.indexOf('Var') >= 0) return val.toFixed(4);
            // Voltage: 2 decimals
            if (field === 'voltage') return val.toFixed(2) + ' V';
            if (field === 'current') return val.toFixed(1) + ' A';
            if (field === 'batteryPct') return Math.round(val) + ' %';
            if (field === 'mah') return Math.round(val) + ' mAh';
            if (field === 'airspeed' || field === 'groundspeed' || field === 'targetSpeed') return val.toFixed(1) + ' m/s';
            if (field === 'climb') return val.toFixed(2) + ' m/s';
            if (field === 'throttle') return Math.round(val) + ' %';
            if (field === 'heading' || field === 'hdg') return Math.round(val) + '\u00b0';
            if (field === 'rcRssi') return val + '/255 (' + Math.round(val / 255 * 100) + '%)';
            // Timestamps
            if (field === 'lastHeartbeat' || field === 'lastAttitude' || field === 'lastPosition' || field === 'flightStartTime') {
                if (val === 0) return '--';
                return new Date(val).toLocaleTimeString();
            }
            // HDOP/VDOP
            if (field === 'hdop' || field === 'vdop') return val.toFixed(1);
            // General
            if (Number.isInteger(val)) return val.toString();
            return val.toFixed(2);
        }
        if (typeof val === 'string') return val;
        if (Array.isArray(val)) return '[' + val.length + ' items]';
        if (typeof val === 'object') return '{...}';
        return String(val);
    }

    function applyFilter() {
        const rows = container.querySelectorAll('.status-row');
        let visible = 0;
        rows.forEach(function (row) {
            const fieldName = row.dataset.field || '';
            const show = !filterText || fieldName.indexOf(filterText) >= 0;
            row.style.display = show ? '' : 'none';
            if (show) visible++;
        });

        // Hide empty sections
        container.querySelectorAll('.status-section').forEach(function (sec) {
            const visibleRows = sec.querySelectorAll('.status-row:not([style*="display: none"])');
            sec.style.display = visibleRows.length > 0 ? '' : 'none';
        });

        updateFieldCount();
    }

    function updateFieldCount() {
        const el = document.getElementById('status-field-count');
        if (!el) return;
        const total = Object.keys(fieldEls).length;
        const visible = container ? container.querySelectorAll('.status-row:not([style*="display: none"])').length : total;
        el.textContent = visible + ' / ' + total + ' fields';
    }

    function destroy() {
        if (updateTimer) clearInterval(updateTimer);
        updateTimer = null;
        initialized = false;
    }

    // Re-render when panel opens
    meridian.events.on('panel_change', function (panel) {
        if (panel === 'status') {
            render();
        } else if (initialized) {
            destroy();
        }
    });

    return { render, destroy };

})();
