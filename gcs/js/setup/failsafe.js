/* ============================================================
   failsafe.js — Failsafe configuration
   RC, Battery, and GCS failsafe settings.
   ============================================================ */

'use strict';

window.Failsafe = (function () {

    var RC_ACTIONS = [
        { val: 0, name: 'Disabled' },
        { val: 1, name: 'Land' },
        { val: 2, name: 'RTL' },
        { val: 3, name: 'SmartRTL or RTL' },
        { val: 4, name: 'SmartRTL or Land' },
        { val: 5, name: 'Terminate' },
    ];

    var BATT_ACTIONS = [
        { val: 0, name: 'Disabled' },
        { val: 1, name: 'Land' },
        { val: 2, name: 'RTL' },
        { val: 3, name: 'SmartRTL or RTL' },
        { val: 4, name: 'SmartRTL or Land' },
        { val: 5, name: 'Terminate' },
    ];

    var GCS_ACTIONS = [
        { val: 0, name: 'Disabled' },
        { val: 1, name: 'RTL' },
        { val: 2, name: 'Continue Mission' },
        { val: 3, name: 'SmartRTL or RTL' },
        { val: 4, name: 'SmartRTL or Land' },
    ];

    function makeSelect(id, options, currentVal) {
        var html = '<select class="setup-select" id="' + id + '">';
        for (var i = 0; i < options.length; i++) {
            var opt = options[i];
            html += '<option value="' + opt.val + '"' + (opt.val === currentVal ? ' selected' : '') + '>' + opt.name + '</option>';
        }
        html += '</select>';
        return html;
    }

    function render(container, params, onParamChange) {
        params = params || {};

        var rcEnable = params.FS_THR_ENABLE !== undefined ? params.FS_THR_ENABLE : 0;
        var rcThreshold = params.FS_THR_VALUE !== undefined ? params.FS_THR_VALUE : 975;
        var battLowAct = params.BATT_FS_LOW_ACT !== undefined ? params.BATT_FS_LOW_ACT : 0;
        var battLowVolt = params.BATT_LOW_VOLT !== undefined ? params.BATT_LOW_VOLT : 10.5;
        var battLowMah = params.BATT_LOW_MAH !== undefined ? params.BATT_LOW_MAH : 0;
        var battCritAct = params.BATT_FS_CRT_ACT !== undefined ? params.BATT_FS_CRT_ACT : 0;
        var battCritVolt = params.BATT_CRT_VOLT !== undefined ? params.BATT_CRT_VOLT : 10.0;
        var gcsEnable = params.FS_GCS_ENABLE !== undefined ? params.FS_GCS_ENABLE : 0;
        var gcsTimeout = params.FS_GCS_TIMEOUT !== undefined ? params.FS_GCS_TIMEOUT : 5;

        var html = '';

        // RC Failsafe
        html += '<div class="fs-section">';
        html += '<div class="fs-header">';
        html += '<div class="fs-title">RC Failsafe</div>';
        html += '<div class="fs-status ' + (rcEnable > 0 ? 'enabled' : 'disabled') + '">' + (rcEnable > 0 ? 'Enabled' : 'Disabled') + '</div>';
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Action:</label>';
        html += makeSelect('fs-rc-action', RC_ACTIONS, rcEnable);
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Throttle PWM Threshold:</label>';
        html += '<input type="number" class="setup-input" id="fs-rc-thr" value="' + rcThreshold + '" min="900" max="1100" step="1">';
        html += '</div>';
        html += '</div>';

        // Battery Failsafe
        html += '<div class="fs-section">';
        html += '<div class="fs-header">';
        html += '<div class="fs-title">Battery Failsafe (Low)</div>';
        html += '<div class="fs-status ' + (battLowAct > 0 ? 'enabled' : 'disabled') + '">' + (battLowAct > 0 ? 'Enabled' : 'Disabled') + '</div>';
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Action:</label>';
        html += makeSelect('fs-batt-action', BATT_ACTIONS, battLowAct);
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Voltage Threshold (V):</label>';
        html += '<input type="number" class="setup-input" id="fs-batt-volt" value="' + battLowVolt + '" min="6" max="60" step="0.1">';
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>mAh Remaining:</label>';
        html += '<input type="number" class="setup-input" id="fs-batt-mah" value="' + battLowMah + '" min="0" max="50000" step="50">';
        html += '</div>';
        html += '</div>';

        // Battery Critical
        html += '<div class="fs-section">';
        html += '<div class="fs-header">';
        html += '<div class="fs-title">Battery Failsafe (Critical)</div>';
        html += '<div class="fs-status ' + (battCritAct > 0 ? 'enabled' : 'disabled') + '">' + (battCritAct > 0 ? 'Enabled' : 'Disabled') + '</div>';
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Action:</label>';
        html += makeSelect('fs-batt-crit-action', BATT_ACTIONS, battCritAct);
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Critical Voltage (V):</label>';
        html += '<input type="number" class="setup-input" id="fs-batt-crit-volt" value="' + battCritVolt + '" min="6" max="60" step="0.1">';
        html += '</div>';
        html += '</div>';

        // GCS Failsafe
        html += '<div class="fs-section">';
        html += '<div class="fs-header">';
        html += '<div class="fs-title">GCS Failsafe</div>';
        html += '<div class="fs-status ' + (gcsEnable > 0 ? 'enabled' : 'disabled') + '">' + (gcsEnable > 0 ? 'Enabled' : 'Disabled') + '</div>';
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Action:</label>';
        html += makeSelect('fs-gcs-action', GCS_ACTIONS, gcsEnable);
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Timeout (seconds):</label>';
        html += '<input type="number" class="setup-input" id="fs-gcs-timeout" value="' + gcsTimeout + '" min="1" max="300" step="1">';
        html += '</div>';
        html += '</div>';

        // Save
        html += '<div class="setup-form-actions">';
        html += '<button class="setup-btn primary" id="fs-save-btn">Save Failsafe Config</button>';
        html += '</div>';

        container.innerHTML = html;

        // Wire save
        var saveBtn = document.getElementById('fs-save-btn');
        if (saveBtn) {
            saveBtn.addEventListener('click', function () {
                if (onParamChange) {
                    onParamChange('FS_THR_ENABLE', parseInt(document.getElementById('fs-rc-action').value));
                    onParamChange('FS_THR_VALUE', parseInt(document.getElementById('fs-rc-thr').value));
                    onParamChange('BATT_FS_LOW_ACT', parseInt(document.getElementById('fs-batt-action').value));
                    onParamChange('BATT_LOW_VOLT', parseFloat(document.getElementById('fs-batt-volt').value));
                    onParamChange('BATT_LOW_MAH', parseInt(document.getElementById('fs-batt-mah').value));
                    onParamChange('BATT_FS_CRT_ACT', parseInt(document.getElementById('fs-batt-crit-action').value));
                    onParamChange('BATT_CRT_VOLT', parseFloat(document.getElementById('fs-batt-crit-volt').value));
                    onParamChange('FS_GCS_ENABLE', parseInt(document.getElementById('fs-gcs-action').value));
                    onParamChange('FS_GCS_TIMEOUT', parseInt(document.getElementById('fs-gcs-timeout').value));
                }
                meridian.log('Failsafe configuration saved', 'info');
            });
        }
    }

    return { render };

})();
