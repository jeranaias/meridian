/* ============================================================
   wp-editor.js — Waypoint editor fields
   Shows command dropdown, lat/lon, altitude, params 1-4
   when a waypoint is selected. Edits update mission + mark dirty.
   ============================================================ */

'use strict';

window.WpEditor = (function () {

    var container = null;
    var currentIdx = -1;

    // Command options for the dropdown
    var COMMANDS = [
        { value: 16,  label: 'NAV_WAYPOINT' },
        { value: 17,  label: 'NAV_LOITER_UNLIM' },
        { value: 18,  label: 'NAV_LOITER_TURNS' },
        { value: 19,  label: 'NAV_LOITER_TIME' },
        { value: 21,  label: 'NAV_LAND' },
        { value: 22,  label: 'NAV_TAKEOFF' },
        { value: 20,  label: 'NAV_RETURN_TO_LAUNCH' },
        { value: 201, label: 'DO_SET_ROI' },
    ];

    // Param labels vary by command type
    var PARAM_LABELS = {};
    PARAM_LABELS[16]  = ['Hold (s)',     'Accept Radius', 'Pass By (m)',  'Yaw (deg)'];
    PARAM_LABELS[17]  = ['---',          '---',           'Radius (m)',   'Yaw (deg)'];
    PARAM_LABELS[18]  = ['Turns',        '---',           'Radius (m)',   'Yaw (deg)'];
    PARAM_LABELS[19]  = ['Time (s)',     '---',           'Radius (m)',   'Yaw (deg)'];
    PARAM_LABELS[20]  = ['---',          '---',           '---',          '---'];
    PARAM_LABELS[21]  = ['Abort Alt',    'Precision',     '---',          'Yaw (deg)'];
    PARAM_LABELS[22]  = ['Pitch (deg)',  '---',           '---',          'Yaw (deg)'];
    PARAM_LABELS[201] = ['ROI Mode',     'WP Index',      '---',          '---'];

    function init(el) {
        container = el;
        meridian.events.on('mission_select', onSelect);
        meridian.events.on('mission_change', onMissionChange);
        render();
    }

    function onSelect(idx) {
        currentIdx = idx;
        render();
    }

    function onMissionChange() {
        // Re-render if current selection is still valid
        if (currentIdx >= Mission.count()) {
            currentIdx = -1;
        }
        render();
    }

    function render() {
        if (!container) return;

        var item = Mission.getItem(currentIdx);
        if (!item) {
            container.innerHTML = '';
            container.style.display = 'none';
            return;
        }

        container.style.display = '';

        var labels = PARAM_LABELS[item.command] || ['Param 1', 'Param 2', 'Param 3', 'Param 4'];

        var html = '<div class="wp-editor-title">WP ' + (item.seq + 1) + ' Editor</div>';
        html += '<div class="wp-editor-grid">';

        // Command dropdown (full width)
        html += '<div class="wp-field full">';
        html += '<label>Command</label>';
        html += '<select id="wp-cmd">';
        for (var i = 0; i < COMMANDS.length; i++) {
            var sel = COMMANDS[i].value === item.command ? ' selected' : '';
            html += '<option value="' + COMMANDS[i].value + '"' + sel + '>' + COMMANDS[i].label + '</option>';
        }
        html += '</select></div>';

        // Lat / Lon
        html += '<div class="wp-field"><label>Latitude</label>';
        html += '<input type="number" id="wp-lat" step="0.000001" value="' + item.lat.toFixed(7) + '"></div>';

        html += '<div class="wp-field"><label>Longitude</label>';
        html += '<input type="number" id="wp-lon" step="0.000001" value="' + item.lon.toFixed(7) + '"></div>';

        // Altitude
        html += '<div class="wp-field"><label>Altitude (m)</label>';
        html += '<input type="number" id="wp-alt" step="0.5" value="' + item.alt.toFixed(1) + '"></div>';

        // Frame (hidden for now, always relative alt)
        html += '<div class="wp-field"><label>Frame</label>';
        html += '<select id="wp-frame">';
        html += '<option value="3"' + (item.frame === 3 ? ' selected' : '') + '>Relative Alt</option>';
        html += '<option value="0"' + (item.frame === 0 ? ' selected' : '') + '>Absolute</option>';
        html += '<option value="10"' + (item.frame === 10 ? ' selected' : '') + '>Terrain</option>';
        html += '</select></div>';

        // Params 1-4
        for (var p = 0; p < 4; p++) {
            var paramKey = 'param' + (p + 1);
            var lbl = labels[p];
            if (lbl === '---') continue; // Skip unused params
            html += '<div class="wp-field"><label>' + lbl + '</label>';
            html += '<input type="number" id="wp-p' + (p + 1) + '" step="0.1" value="' + (item[paramKey] || 0) + '"></div>';
        }

        html += '</div>';
        container.innerHTML = html;

        // Wire change handlers
        wire('wp-cmd', 'command', true);
        wire('wp-lat', 'lat', false);
        wire('wp-lon', 'lon', false);
        wire('wp-alt', 'alt', false);
        wire('wp-frame', 'frame', true);
        wire('wp-p1', 'param1', false);
        wire('wp-p2', 'param2', false);
        wire('wp-p3', 'param3', false);
        wire('wp-p4', 'param4', false);
    }

    function wire(elId, field, isInt) {
        var el = document.getElementById(elId);
        if (!el) return;

        var handler = function () {
            if (currentIdx < 0) return;
            var val = isInt ? parseInt(el.value, 10) : parseFloat(el.value);
            if (isNaN(val)) return;
            var changes = {};
            changes[field] = val;
            Mission.updateItem(currentIdx, changes);

            // If command changed, re-render to update param labels
            if (field === 'command') render();
        };

        el.addEventListener('change', handler);
        el.addEventListener('input', handler);
    }

    return { init: init };

})();
