/* ============================================================
   quick.js — User-configurable quick-values widget
   Oborne addition: 4 large-text telemetry values.
   Right-click to change field. Persists to localStorage.
   ============================================================ */

'use strict';

window.QuickWidget = (function () {

    let el;

    const FIELDS = {
        'wp_dist':    { label: 'WP Dist',    unit: 'm',  get: v => {
            if (!v.lat || !v.missionItems || !v.missionItems.length) return '---';
            var seq = v.missionSeq || 0;
            var wp = v.missionItems[seq];
            if (!wp || !wp.lat || !wp.lon) return '---';
            return Math.round(haversine(v.lat, v.lon, wp.lat, wp.lon)).toString();
        }},
        'home_dist':  { label: 'Home Dist',  unit: 'm',  get: v => {
            if (!v.homeLat || v.lat === 0) return '---';
            return Math.round(haversine(v.lat, v.lon, v.homeLat, v.homeLon)).toString();
        }},
        'flight_time':{ label: 'Flt Time',   unit: '',   get: v => {
            if (!v.flightStartTime) return '0:00';
            const s = Math.floor((Date.now() - v.flightStartTime) / 1000);
            return Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0');
        }},
        'throttle':   { label: 'Throttle',   unit: '%',  get: v => v.throttle.toString() },
        'alt':        { label: 'Alt',         unit: 'm',  get: v => v.relativeAlt.toFixed(1) },
        'gndspd':     { label: 'Gnd Spd',    unit: 'm/s',get: v => v.groundspeed.toFixed(1) },
        'climb':      { label: 'Climb',       unit: 'm/s',get: v => v.climb.toFixed(1) },
        'heading':    { label: 'Heading',     unit: '°',  get: v => Math.round(v.heading).toString() },
        'voltage':    { label: 'Voltage',     unit: 'V',  get: v => v.voltage.toFixed(1) },
        'sats':       { label: 'Sats',        unit: '',   get: v => v.satellites.toString() },
        // T2-18: RTL altitude from params (RTL_ALT is in cm, display in m)
        'rtl_alt':    { label: 'RTL Alt',     unit: 'm',  get: v => {
            if (v.params && v.params.RTL_ALT !== undefined) {
                return (v.params.RTL_ALT / 100).toFixed(0);
            }
            return '---';
        }},
        // T3-5: Wind speed and direction from WindOverlay estimate
        'wind_spd':   { label: 'Wind Spd',    unit: 'm/s', get: function () {
            if (window.WindOverlay) {
                const est = WindOverlay.getCurrentEstimate();
                if (est) return est.speed.toFixed(1);
            }
            return '---';
        }},
        'wind_dir':   { label: 'Wind From',   unit: '°',  get: function () {
            if (window.WindOverlay) {
                const est = WindOverlay.getCurrentEstimate();
                if (est) return Math.round(est.fromDeg).toString();
            }
            return '---';
        }},
    };

    // Color assignments per field (MP-style bright on dark)
    const FIELD_COLORS = {
        'alt':         'var(--c-safe)',
        'gndspd':      'var(--c-primary)',
        'climb':       'var(--c-warning)',
        'heading':     'var(--c-special)',
        'home_dist':   'var(--c-info)',
        'wp_dist':     'var(--c-emergency)',
        'flight_time': 'var(--c-text)',
        'throttle':    'var(--c-warning)',
        'voltage':     'var(--c-safe)',
        'sats':        'var(--c-primary)',
        'rtl_alt':     'var(--c-neutral)',
        'wind_spd':    'var(--c-primary)',
        'wind_dir':    'var(--c-primary)',
    };

    let slots = ['alt', 'gndspd', 'home_dist', 'wp_dist', 'climb', 'heading', 'flight_time', 'throttle'];

    function haversine(lat1, lon1, lat2, lon2) {
        const R = 6371000;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(dLat / 2) ** 2 +
                  Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                  Math.sin(dLon / 2) ** 2;
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    function init(container) {
        el = container;
        // Load saved config
        try {
            const saved = localStorage.getItem('meridian_quick_slots');
            if (saved) slots = JSON.parse(saved);
        } catch (e) { /* ignore */ }
        render();
    }

    function render() {
        if (!el) return;
        el.innerHTML = slots.map((fieldId, i) => {
            const f = FIELDS[fieldId] || FIELDS['alt'];
            const color = FIELD_COLORS[fieldId] || 'var(--c-text)';
            return `<div class="quick-item" data-slot="${i}" data-field="${fieldId}">
                <span class="quick-value" id="qval-${i}" style="color:${color}">---</span>
                <span class="quick-label">${f.label}</span>
            </div>`;
        }).join('');

        // Right-click to change
        el.querySelectorAll('.quick-item').forEach(item => {
            item.addEventListener('contextmenu', e => {
                e.preventDefault();
                const slotIdx = parseInt(item.dataset.slot);
                showFieldPicker(slotIdx, e.clientX, e.clientY);
            });
        });
    }

    function showFieldPicker(slotIdx, x, y) {
        // Remove existing picker
        const old = document.getElementById('quick-picker');
        if (old) old.remove();

        const picker = document.createElement('div');
        picker.id = 'quick-picker';
        picker.className = 'map-context-menu visible';
        picker.style.left = x + 'px';
        picker.style.top = y + 'px';
        picker.style.position = 'fixed';

        for (const [id, f] of Object.entries(FIELDS)) {
            const item = document.createElement('div');
            item.className = 'context-item';
            item.textContent = f.label;
            if (id === slots[slotIdx]) item.style.color = 'var(--c-primary)';
            item.addEventListener('click', () => {
                slots[slotIdx] = id;
                try { localStorage.setItem('meridian_quick_slots', JSON.stringify(slots)); } catch (e) {}
                render();
                picker.remove();
            });
            picker.appendChild(item);
        }

        document.body.appendChild(picker);

        // Close on click outside
        setTimeout(() => {
            const close = (e) => {
                if (!picker.contains(e.target)) {
                    picker.remove();
                    document.removeEventListener('click', close);
                }
            };
            document.addEventListener('click', close);
        }, 0);
    }

    function update(v) {
        if (!el) return;
        slots.forEach((fieldId, i) => {
            const f = FIELDS[fieldId];
            const valEl = document.getElementById('qval-' + i);
            if (valEl && f) {
                const val = f.get(v);
                valEl.textContent = val + (f.unit && val !== '---' ? f.unit : '');
            }
        });
    }

    return { init, update };

})();
