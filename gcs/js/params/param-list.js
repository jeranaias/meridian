/* ============================================================
   param-list.js — Full parameter list component
   Searchable list, click to edit inline, modified highlighting.
   Load/Save buttons for param files.
   ============================================================ */

'use strict';

window.ParamList = (function () {

    var editingParam = null;
    var modifiedParams = {};  // tracks params changed this session
    var sortKey = 'name';     // 'name' or 'modified'
    var groupView = true;     // group by prefix

    // Common parameter descriptions (ArduPilot)
    var DESCRIPTIONS = {
        ARMING_CHECK: 'Bitmask of pre-arm checks to perform',
        FRAME_CLASS: 'Frame class (1=Quad, 2=Hexa, 3=Octa, etc)',
        FRAME_TYPE: 'Frame geometry (0=Plus, 1=X, etc)',
        ATC_RAT_RLL_P: 'Roll rate controller P gain',
        ATC_RAT_RLL_I: 'Roll rate controller I gain',
        ATC_RAT_RLL_D: 'Roll rate controller D gain',
        ATC_RAT_PIT_P: 'Pitch rate controller P gain',
        ATC_RAT_PIT_I: 'Pitch rate controller I gain',
        ATC_RAT_PIT_D: 'Pitch rate controller D gain',
        ATC_RAT_YAW_P: 'Yaw rate controller P gain',
        ATC_RAT_YAW_I: 'Yaw rate controller I gain',
        ATC_RAT_YAW_D: 'Yaw rate controller D gain',
        ATC_ACCEL_P_MAX: 'Max pitch acceleration (cdeg/s/s)',
        ATC_ACCEL_R_MAX: 'Max roll acceleration (cdeg/s/s)',
        ATC_ACCEL_Y_MAX: 'Max yaw acceleration (cdeg/s/s)',
        BATT_MONITOR: 'Battery monitoring type (0=Off, 3=Analog V, 4=Analog V+I)',
        BATT_CAPACITY: 'Battery capacity (mAh)',
        BATT_LOW_VOLT: 'Low battery voltage threshold',
        BATT_FS_LOW_ACT: 'Low battery failsafe action (0=None, 1=Land, 2=RTL)',
        FS_THR_ENABLE: 'Throttle failsafe (0=Off, 1=Enabled always RTL)',
        FS_THR_VALUE: 'Throttle PWM value for failsafe trigger',
        FS_GCS_ENABLE: 'GCS failsafe (0=Off, 1=RTL, 2=Continue)',
        FS_GCS_TIMEOUT: 'GCS heartbeat timeout (seconds)',
        FENCE_ENABLE: 'Geofence enable (0=Off, 1=On)',
        FENCE_TYPE: 'Fence type bitmask (1=Alt, 2=Circle, 4=Polygon)',
        FENCE_ALT_MAX: 'Maximum altitude fence (meters)',
        FENCE_RADIUS: 'Circular fence radius (meters)',
        WPNAV_SPEED: 'Waypoint navigation speed (cm/s)',
        WPNAV_SPEED_UP: 'Max climb speed in auto (cm/s)',
        WPNAV_SPEED_DN: 'Max descent speed in auto (cm/s)',
        WPNAV_ACCEL: 'Waypoint acceleration (cm/s/s)',
        RTL_ALT: 'RTL altitude (cm, 0=current alt)',
        RTL_SPEED: 'RTL speed (cm/s, 0=WPNAV_SPEED)',
        LAND_SPEED: 'Final stage landing speed (cm/s)',
        INS_ACCOFFS_X: 'Accelerometer X offset',
        INS_ACCOFFS_Y: 'Accelerometer Y offset',
        INS_ACCOFFS_Z: 'Accelerometer Z offset',
        COMPASS_OFS_X: 'Compass X offset',
        COMPASS_OFS_Y: 'Compass Y offset',
        COMPASS_OFS_Z: 'Compass Z offset',
        RC1_MIN: 'RC channel 1 minimum PWM', RC1_MAX: 'RC channel 1 maximum PWM',
        RC1_TRIM: 'RC channel 1 trim PWM',
        FLTMODE1: 'Flight mode 1 (switch position 1)',
        FLTMODE2: 'Flight mode 2', FLTMODE3: 'Flight mode 3',
        FLTMODE4: 'Flight mode 4', FLTMODE5: 'Flight mode 5', FLTMODE6: 'Flight mode 6',
        PILOT_SPEED_UP: 'Pilot max climb rate (cm/s)',
        PILOT_SPEED_DN: 'Pilot max descent rate (cm/s)',
        PILOT_ACCEL_Z: 'Pilot vertical acceleration (cm/s/s)',
    };

    // Group labels by prefix
    var GROUP_LABELS = {
        ATC: 'Attitude Control', BATT: 'Battery', FS: 'Failsafe', FENCE: 'Geofence',
        WPNAV: 'Waypoint Navigation', RTL: 'Return to Launch', LAND: 'Landing',
        INS: 'Inertial Sensors', COMPASS: 'Compass', RC: 'RC Channels',
        FLTMODE: 'Flight Modes', PILOT: 'Pilot Input', ARMING: 'Arming',
        FRAME: 'Frame', EK2: 'EKF2', EK3: 'EKF3', GPS: 'GPS',
        SR0: 'Stream Rate', MOT: 'Motors', LOG: 'Logging',
    };

    // Default values for common params (for showing diff)
    var DEFAULTS = {
        FRAME_CLASS: 1, FRAME_TYPE: 1, ARMING_CHECK: 1,
        ATC_RAT_RLL_P: 0.135, ATC_RAT_RLL_I: 0.135, ATC_RAT_RLL_D: 0.0036,
        ATC_RAT_PIT_P: 0.135, ATC_RAT_PIT_I: 0.135, ATC_RAT_PIT_D: 0.0036,
        ATC_RAT_YAW_P: 0.18, ATC_RAT_YAW_I: 0.018, ATC_RAT_YAW_D: 0.0,
        BATT_MONITOR: 4, BATT_CAPACITY: 5200,
        FS_THR_ENABLE: 1, FS_THR_VALUE: 975,
        FS_GCS_ENABLE: 1, FS_GCS_TIMEOUT: 5,
        BATT_FS_LOW_ACT: 2, BATT_LOW_VOLT: 10.5,
        INS_ACCOFFS_X: 0, INS_ACCOFFS_Y: 0, INS_ACCOFFS_Z: 0,
    };

    function getParams() {
        var v = meridian.v;
        return v ? v.params : {};
    }

    function render(container, filter) {
        filter = (filter || '').toUpperCase();
        var params = getParams();
        var keys = Object.keys(params).sort();

        if (sortKey === 'modified') {
            keys.sort(function (a, b) {
                var am = modifiedParams[a] ? 1 : 0;
                var bm = modifiedParams[b] ? 1 : 0;
                if (am !== bm) return bm - am;
                return a.localeCompare(b);
            });
        }

        if (filter) {
            keys = keys.filter(function (k) { return k.indexOf(filter) !== -1; });
        }

        var html = '';

        if (keys.length === 0) {
            html += '<div class="param-empty">';
            if (Object.keys(params).length === 0) {
                html += 'No parameters loaded. Connect to a vehicle or load a param file.';
            } else {
                html += 'No parameters match "' + filter + '"';
            }
            html += '</div>';
        } else {
            html += '<div class="param-count">' + keys.length + ' parameter' + (keys.length !== 1 ? 's' : '') + '</div>';
            html += '<div class="param-table">';
            html += '<div class="param-row header">';
            html += '<span class="param-cell name">Parameter</span>';
            html += '<span class="param-cell value">Value</span>';
            html += '<span class="param-cell default">Default</span>';
            html += '</div>';

            // Group params by prefix if groupView and not filtering
            var grouped = groupView && !filter;
            var currentGroup = '';

            for (var i = 0; i < keys.length; i++) {
                var name = keys[i];
                var val = params[name];
                var def = DEFAULTS[name];
                var desc = DESCRIPTIONS[name] || '';
                var isModified = modifiedParams[name] !== undefined;
                var isDifferentFromDefault = def !== undefined && val !== def;

                // Group header
                if (grouped) {
                    var prefix = name.replace(/_.*/, '').replace(/[0-9]+$/, '');
                    if (prefix !== currentGroup) {
                        currentGroup = prefix;
                        var groupLabel = GROUP_LABELS[prefix] || prefix;
                        html += '<div class="param-group-header" data-group="' + prefix + '">' + groupLabel + '</div>';
                    }
                }

                var tooltip = desc ? desc : 'Click to edit';

                if (editingParam === name) {
                    html += '<div class="param-row editing" data-param="' + name + '">';
                    html += '<span class="param-cell name">' + name;
                    if (desc) html += '<span class="param-desc">' + desc + '</span>';
                    html += '</span>';
                    html += '<span class="param-cell value">';
                    html += '<input type="text" class="param-edit-input" id="param-edit-input" value="' + val + '" data-param="' + name + '">';
                    html += '</span>';
                    html += '<span class="param-cell default">';
                    html += '<button class="param-save-btn" id="param-save-btn">Save</button>';
                    html += '<button class="param-cancel-btn" id="param-cancel-btn">Cancel</button>';
                    html += '</span>';
                    html += '</div>';
                } else {
                    var rowClass = 'param-row';
                    if (isModified) rowClass += ' modified';
                    else if (isDifferentFromDefault) rowClass += ' non-default';
                    html += '<div class="' + rowClass + '" data-param="' + name + '" title="' + tooltip + '">';
                    html += '<span class="param-cell name">' + name + '</span>';
                    html += '<span class="param-cell value">' + formatValue(val) + '</span>';
                    html += '<span class="param-cell default">' + (def !== undefined ? formatValue(def) : '--') + '</span>';
                    html += '</div>';
                }
            }
            html += '</div>';
        }

        container.innerHTML = html;

        // Wire row clicks
        container.querySelectorAll('.param-row:not(.header):not(.editing)').forEach(function (row) {
            row.addEventListener('click', function () {
                editingParam = row.dataset.param;
                render(container, filter);
                var input = document.getElementById('param-edit-input');
                if (input) { input.focus(); input.select(); }
            });
        });

        // Wire editing controls
        var saveBtn = document.getElementById('param-save-btn');
        var cancelBtn = document.getElementById('param-cancel-btn');
        var editInput = document.getElementById('param-edit-input');

        if (saveBtn) {
            saveBtn.addEventListener('click', function () {
                saveParam(editInput);
                render(container, filter);
            });
        }
        if (cancelBtn) {
            cancelBtn.addEventListener('click', function () {
                editingParam = null;
                render(container, filter);
            });
        }
        if (editInput) {
            editInput.addEventListener('keydown', function (e) {
                if (e.key === 'Enter') {
                    saveParam(editInput);
                    render(container, filter);
                } else if (e.key === 'Escape') {
                    editingParam = null;
                    render(container, filter);
                }
            });
        }
    }

    function saveParam(input) {
        if (!input) return;
        var name = input.dataset.param;
        var rawVal = input.value.trim();
        var val = parseFloat(rawVal);
        if (isNaN(val)) val = rawVal;

        var v = meridian.v;
        if (v) {
            v.params[name] = val;
            modifiedParams[name] = true;
            meridian.events.emit('param', { name: name, value: val });
            meridian.log('Param ' + name + ' = ' + val, 'info');
        }
        editingParam = null;
    }

    function formatValue(val) {
        if (typeof val === 'number') {
            if (Number.isInteger(val)) return val.toString();
            return val.toFixed(6).replace(/0+$/, '').replace(/\.$/, '.0');
        }
        return String(val);
    }

    function getModifiedParams() { return Object.assign({}, modifiedParams); }
    function clearModified() { modifiedParams = {}; }

    function exportParams() {
        var params = getParams();
        var keys = Object.keys(params).sort();
        var lines = [];
        for (var i = 0; i < keys.length; i++) {
            lines.push(keys[i] + ',' + params[keys[i]]);
        }
        var text = lines.join('\n');
        var blob = new Blob([text], { type: 'text/plain' });
        var url = URL.createObjectURL(blob);
        var a = document.createElement('a');
        a.href = url;
        a.download = 'parameters.param';
        a.click();
        URL.revokeObjectURL(url);
        meridian.log('Parameters exported', 'info');
    }

    function importParams(file) {
        var reader = new FileReader();
        reader.onload = function (e) {
            var text = e.target.result;
            var lines = text.split(/\r?\n/);
            var v = meridian.v;
            if (!v) return;
            var count = 0;
            for (var i = 0; i < lines.length; i++) {
                var line = lines[i].trim();
                if (!line || line.startsWith('#')) continue;
                var parts = line.split(/[,\t ]+/);
                if (parts.length >= 2) {
                    var name = parts[0];
                    var val = parseFloat(parts[1]);
                    if (!isNaN(val)) {
                        v.params[name] = val;
                        modifiedParams[name] = true;
                        count++;
                    }
                }
            }
            meridian.log('Loaded ' + count + ' parameters from file', 'info');
            meridian.events.emit('param', { name: '_bulk', value: count });
        };
        reader.readAsText(file);
    }

    return { render, exportParams, importParams, getModifiedParams, clearModified, DEFAULTS };

})();
