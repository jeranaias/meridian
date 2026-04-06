/* ============================================================
   battery.js — Battery monitor setup
   Configures BATT_MONITOR, BATT_CAPACITY, cell count.
   ============================================================ */

'use strict';

window.BatterySetup = (function () {

    var MONITOR_TYPES = [
        { val: 0, name: 'Disabled' },
        { val: 3, name: 'Analog Voltage Only' },
        { val: 4, name: 'Analog Voltage + Current' },
        { val: 5, name: 'Solo (Smart Battery)' },
        { val: 6, name: 'Maxell' },
        { val: 7, name: 'UAVCAN' },
        { val: 8, name: 'BLHeli ESC Telemetry' },
    ];

    var CELL_COUNTS = [1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14];

    function render(container, params, onParamChange) {
        params = params || {};
        var monitor = params.BATT_MONITOR !== undefined ? params.BATT_MONITOR : 0;
        var capacity = params.BATT_CAPACITY !== undefined ? params.BATT_CAPACITY : 0;
        var cellCount = params.BATT_CELL_COUNT || 0;
        var ampPerVolt = params.BATT_AMP_PERVLT !== undefined ? params.BATT_AMP_PERVLT : 17;
        var voltMult = params.BATT_VOLT_MULT !== undefined ? params.BATT_VOLT_MULT : 10.1;

        var html = '<div class="setup-form-section">';
        html += '<div class="setup-form-title">Battery Monitor</div>';
        html += '</div>';

        html += '<div class="fs-section">';
        html += '<div class="fs-row">';
        html += '<label>Monitor Type:</label>';
        html += '<select class="setup-select" id="batt-monitor">';
        for (var i = 0; i < MONITOR_TYPES.length; i++) {
            var t = MONITOR_TYPES[i];
            html += '<option value="' + t.val + '"' + (t.val === monitor ? ' selected' : '') + '>' + t.name + '</option>';
        }
        html += '</select>';
        html += '</div>';

        html += '<div class="fs-row">';
        html += '<label>Capacity (mAh):</label>';
        html += '<input type="number" class="setup-input" id="batt-cap" value="' + capacity + '" min="0" max="50000" step="100">';
        html += '</div>';

        html += '<div class="fs-row">';
        html += '<label>Cell Count:</label>';
        html += '<select class="setup-select" id="batt-cells">';
        html += '<option value="0"' + (cellCount === 0 ? ' selected' : '') + '>Auto Detect</option>';
        for (var j = 0; j < CELL_COUNTS.length; j++) {
            var c = CELL_COUNTS[j];
            html += '<option value="' + c + '"' + (c === cellCount ? ' selected' : '') + '>' + c + 'S (' + (c * 3.7).toFixed(1) + 'V nom / ' + (c * 4.2).toFixed(1) + 'V full)</option>';
        }
        html += '</select>';
        html += '</div>';

        if (monitor >= 3) {
            html += '<div class="fs-row">';
            html += '<label>Voltage Multiplier:</label>';
            html += '<input type="number" class="setup-input" id="batt-voltmult" value="' + voltMult + '" min="0" max="100" step="0.01">';
            html += '</div>';

            if (monitor >= 4) {
                html += '<div class="fs-row">';
                html += '<label>Amps per Volt:</label>';
                html += '<input type="number" class="setup-input" id="batt-ampvolt" value="' + ampPerVolt + '" min="0" max="100" step="0.1">';
                html += '</div>';
            }
        }
        html += '</div>';

        // Live readings
        var v = meridian.v;
        if (v && monitor > 0) {
            html += '<div class="fs-section">';
            html += '<div class="fs-title">Current Readings</div>';
            html += '<div class="batt-live-grid">';
            html += '<div class="batt-live-item"><span class="batt-live-label">Voltage</span><span class="batt-live-val">' + (v.voltage || 0).toFixed(2) + ' V</span></div>';
            html += '<div class="batt-live-item"><span class="batt-live-label">Current</span><span class="batt-live-val">' + (v.current || 0).toFixed(1) + ' A</span></div>';
            html += '<div class="batt-live-item"><span class="batt-live-label">Remaining</span><span class="batt-live-val">' + (v.batteryPct >= 0 ? v.batteryPct.toFixed(0) + '%' : '--') + '</span></div>';
            html += '<div class="batt-live-item"><span class="batt-live-label">Consumed</span><span class="batt-live-val">' + (v.mah || 0).toFixed(0) + ' mAh</span></div>';
            html += '</div>';
            html += '</div>';
        }

        html += '<div class="setup-form-actions">';
        html += '<button class="setup-btn primary" id="batt-save-btn">Save Battery Config</button>';
        html += '</div>';

        container.innerHTML = html;

        var saveBtn = document.getElementById('batt-save-btn');
        if (saveBtn) {
            saveBtn.addEventListener('click', function () {
                if (onParamChange) {
                    onParamChange('BATT_MONITOR', parseInt(document.getElementById('batt-monitor').value));
                    onParamChange('BATT_CAPACITY', parseInt(document.getElementById('batt-cap').value));
                    onParamChange('BATT_CELL_COUNT', parseInt(document.getElementById('batt-cells').value));
                    var vm = document.getElementById('batt-voltmult');
                    if (vm) onParamChange('BATT_VOLT_MULT', parseFloat(vm.value));
                    var av = document.getElementById('batt-ampvolt');
                    if (av) onParamChange('BATT_AMP_PERVLT', parseFloat(av.value));
                }
                meridian.log('Battery configuration saved', 'info');
            });
        }
    }

    return { render };

})();
