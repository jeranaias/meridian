/* ============================================================
   regulatory-checklist.js — Pre-flight regulatory checklist (T3-20)
   Auto-checked items pull from live telemetry.
   Manual items require pilot confirmation.
   Resets on each new connection or arm cycle.
   Injected as "Pre-Flight" section in the Setup panel.
   ============================================================ */

'use strict';

window.RegulatoryChecklist = (function () {

    // Item definitions
    // type: 'auto' | 'manual'
    // auto items derive their state from a function; manual require explicit check
    var ITEMS = [
        {
            id:       'gps_fix',
            label:    'GPS Fix Quality',
            type:     'auto',
            hint:     'Requires 3D fix (fixType \u2265 3) and HDOP < 2.0',
            check:    function (v) {
                if (!v) return { ok: false, detail: 'No vehicle data' };
                var ok = v.fixType >= 3 && v.hdop < 2.0;
                return {
                    ok: ok,
                    detail: 'fixType=' + v.fixType + ', HDOP=' + (v.hdop || 99).toFixed(1),
                };
            },
        },
        {
            id:       'battery_level',
            label:    'Battery Above 80%',
            type:     'auto',
            hint:     'Battery must be at or above 80% to fly',
            check:    function (v) {
                if (!v) return { ok: false, detail: 'No vehicle data' };
                var pct = v.batteryPct;
                if (pct < 0) return { ok: false, detail: 'Battery % unknown' };
                return {
                    ok: pct >= 80,
                    detail: pct.toFixed(0) + '%',
                };
            },
        },
        {
            id:       'remote_id',
            label:    'Remote ID Broadcasting',
            type:     'auto',
            hint:     'UAS ID must be present in telemetry (FAA Part 89 / EU UAS Reg)',
            check:    function (v) {
                if (!v) return { ok: false, detail: 'No vehicle data' };
                var hasId = v.remoteId && v.remoteId.uasId && v.remoteId.uasId.length > 0;
                return {
                    ok: !!hasId,
                    detail: hasId ? 'UAS ID: ' + v.remoteId.uasId : 'No UAS ID received',
                };
            },
        },
        {
            id:       'weather',
            label:    'Weather Conditions Acceptable',
            type:     'manual',
            hint:     'Check wind, precipitation, and visibility before flight',
        },
        {
            id:       'airspace',
            label:    'Airspace Authorization Obtained',
            type:     'manual',
            hint:     'Verify LAANC approval or waiver for controlled airspace (B/C/D/E). Not required for Class G uncontrolled.',
            laanc:    true,
        },
        {
            id:       'visual_observer',
            label:    'Visual Observer Present',
            type:     'manual',
            hint:     'Required when VLOS cannot be maintained by PIC alone',
        },
        {
            id:       'emergency_procedures',
            label:    'Emergency Procedures Reviewed',
            type:     'manual',
            hint:     'RC failsafe, RTL altitude, LOS recovery plan reviewed',
        },
    ];

    // Runtime state — manual checkbox values; reset on connect/arm
    var _manualState = {};
    var _container   = null;
    var _initialized = false;

    // ─── Init ─────────────────────────────────────────────────

    function init() {
        if (_initialized) return;
        _initialized = true;

        _resetManual();

        // Reset on new connection or arm
        meridian.events.on('heartbeat', function (v) {
            if (!v) return;
            if (v.armed && !v._prevArmed) {
                _resetManual();
                _refresh();
            }
            v._prevArmed = v.armed;
        });

        meridian.events.on('connection_change', function (state) {
            if (state === 0) {
                _resetManual();
                _refresh();
            }
        });

        // Auto-item refresh on telemetry
        meridian.events.on('gps',     function () { _refresh(); });
        meridian.events.on('battery', function () { _refresh(); });
        meridian.events.on('remote_id', function () { _refresh(); });
    }

    function _resetManual() {
        ITEMS.forEach(function (item) {
            if (item.type === 'manual') _manualState[item.id] = false;
        });
    }

    // ─── Render ───────────────────────────────────────────────

    function render(container) {
        _container = container;
        _draw();
    }

    function _refresh() {
        if (!_container) return;
        _draw();
    }

    function _draw() {
        if (!_container) return;

        var v = meridian.v;

        // Evaluate all items
        var evaluated = ITEMS.map(function (item) {
            if (item.type === 'auto') {
                var result = item.check(v);
                return { item: item, ok: result.ok, detail: result.detail };
            } else {
                return { item: item, ok: !!_manualState[item.id], detail: null };
            }
        });

        var checkedCount = evaluated.filter(function (e) { return e.ok; }).length;
        var totalCount   = evaluated.length;
        var allReady     = checkedCount === totalCount;

        _container.innerHTML = '';

        // Status banner
        var banner = document.createElement('div');
        banner.className = 'reg-banner ' + (allReady ? 'ready' : 'not-ready');
        banner.innerHTML =
            '<div class="reg-banner-status">' +
                (allReady
                    ? '<svg width="16" height="16" viewBox="0 0 16 16" fill="none"><circle cx="8" cy="8" r="7" stroke="var(--c-safe)" stroke-width="1.5"/><path d="M5 8l2 2 4-4" stroke="var(--c-safe)" stroke-width="1.5" fill="none"/></svg>'
                    : '<svg width="16" height="16" viewBox="0 0 16 16" fill="none"><path d="M8 2L14 13H2L8 2z" stroke="var(--c-warning)" stroke-width="1.5"/><line x1="8" y1="7" x2="8" y2="10" stroke="var(--c-warning)" stroke-width="1.5"/><circle cx="8" cy="12" r="0.8" fill="var(--c-warning)"/></svg>') +
                '<span class="reg-banner-label">' + (allReady ? 'READY TO FLY' : 'NOT READY') + '</span>' +
            '</div>' +
            '<div class="reg-banner-count">' + checkedCount + ' / ' + totalCount + ' items checked</div>';
        _container.appendChild(banner);

        // Checklist items
        var list = document.createElement('div');
        list.className = 'reg-checklist';

        evaluated.forEach(function (ev) {
            var item = ev.item;
            var row  = document.createElement('div');
            row.className = 'reg-item ' + (ev.ok ? 'ok' : 'pending') + ' ' + item.type;

            var icon = ev.ok
                ? '<svg class="reg-icon ok" width="16" height="16" viewBox="0 0 16 16" fill="none"><circle cx="8" cy="8" r="7" stroke="var(--c-safe)" stroke-width="1.5"/><path d="M5 8l2 2 4-4" stroke="var(--c-safe)" stroke-width="1.5" fill="none"/></svg>'
                : '<svg class="reg-icon pending" width="16" height="16" viewBox="0 0 16 16" fill="none"><circle cx="8" cy="8" r="7" stroke="var(--c-neutral-dim)" stroke-width="1.5" stroke-dasharray="3 2"/></svg>';

            var autoTag  = item.type === 'auto'
                ? '<span class="reg-tag auto">AUTO</span>'
                : '';
            var laancTag = item.laanc
                ? ' <span class="reg-tag laanc">LAANC</span>'
                : '';

            row.innerHTML =
                icon +
                '<div class="reg-item-body">' +
                    '<div class="reg-item-header">' +
                        '<span class="reg-item-label">' + item.label + '</span>' +
                        autoTag + laancTag +
                    '</div>' +
                    (ev.detail ? '<div class="reg-item-detail">' + _esc(ev.detail) + '</div>' : '') +
                    '<div class="reg-item-hint">' + item.hint + '</div>' +
                '</div>';

            if (item.type === 'manual') {
                var checkbox = document.createElement('input');
                checkbox.type    = 'checkbox';
                checkbox.className = 'reg-checkbox';
                checkbox.checked = !!_manualState[item.id];
                checkbox.setAttribute('aria-label', item.label);
                checkbox.addEventListener('change', (function (id) {
                    return function () {
                        _manualState[id] = checkbox.checked;
                        _draw();
                    };
                })(item.id));
                row.appendChild(checkbox);
            }

            list.appendChild(row);
        });

        _container.appendChild(list);

        // Reset link
        var resetRow = document.createElement('div');
        resetRow.className = 'reg-reset-row';
        var resetBtn = document.createElement('button');
        resetBtn.className = 'reg-reset-btn';
        resetBtn.textContent = 'Reset Manual Items';
        resetBtn.addEventListener('click', function () {
            _resetManual();
            _draw();
        });
        resetRow.appendChild(resetBtn);
        _container.appendChild(resetRow);
    }

    function _esc(str) {
        return String(str)
            .replace(/&/g, '&amp;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;');
    }

    return { init, render };

})();
