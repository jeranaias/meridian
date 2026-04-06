/* ============================================================
   flight-modes.js — 6 flight mode slot configuration
   Each slot is a dropdown; shows which RC channel triggers which.
   ============================================================ */

'use strict';

window.FlightModes = (function () {

    var AVAILABLE_MODES = [
        { num: 0,  name: 'Stabilize' },
        { num: 1,  name: 'Acro' },
        { num: 2,  name: 'Alt Hold' },
        { num: 3,  name: 'Auto' },
        { num: 4,  name: 'Guided' },
        { num: 5,  name: 'Loiter' },
        { num: 6,  name: 'RTL' },
        { num: 7,  name: 'Circle' },
        { num: 9,  name: 'Land' },
        { num: 11, name: 'Drift' },
        { num: 13, name: 'Sport' },
        { num: 14, name: 'Flip' },
        { num: 15, name: 'Autotune' },
        { num: 16, name: 'PosHold' },
        { num: 17, name: 'Brake' },
        { num: 21, name: 'Smart RTL' },
    ];

    // PWM ranges for each mode slot (from ArduCopter docs)
    var SLOT_RANGES = [
        { slot: 1, min: 0,    max: 1230 },
        { slot: 2, min: 1231, max: 1360 },
        { slot: 3, min: 1361, max: 1490 },
        { slot: 4, min: 1491, max: 1620 },
        { slot: 5, min: 1621, max: 1749 },
        { slot: 6, min: 1750, max: 2500 },
    ];

    function render(container, params, onParamChange) {
        params = params || {};
        var modeChannel = params.FLTMODE_CH || 5;

        var html = '<div class="setup-form-section">';
        html += '<div class="setup-form-title">Flight Modes</div>';
        html += '<div class="cal-description">Configure which flight mode is activated for each position of your mode switch (Channel ' + modeChannel + ').</div>';
        html += '</div>';

        html += '<div class="fltmode-channel">';
        html += '<label class="fltmode-ch-label">Mode Channel:</label>';
        html += '<select class="setup-select" id="fltmode-ch-select">';
        for (var ch = 5; ch <= 8; ch++) {
            html += '<option value="' + ch + '"' + (ch === modeChannel ? ' selected' : '') + '>Channel ' + ch + '</option>';
        }
        html += '</select>';
        html += '</div>';

        html += '<div class="fltmode-slots">';
        for (var i = 0; i < 6; i++) {
            var slot = i + 1;
            var paramName = 'FLTMODE' + slot;
            var currentMode = params[paramName] !== undefined ? params[paramName] : 0;
            var range = SLOT_RANGES[i];

            // Determine if this slot is active (demo: highlight slot based on RC channel)
            var v = meridian.v;
            var chVal = 0;
            if (v && v.rcChannels && v.rcChannels.length >= modeChannel) {
                chVal = v.rcChannels[modeChannel - 1] || 0;
            }
            var isActive = chVal >= range.min && chVal <= range.max;

            html += '<div class="fltmode-slot' + (isActive ? ' active' : '') + '">';
            html += '<div class="fltmode-slot-header">';
            html += '<span class="fltmode-slot-num">Mode ' + slot + '</span>';
            html += '<span class="fltmode-slot-range">' + range.min + ' - ' + range.max + ' \u00b5s</span>';
            if (isActive) {
                html += '<span class="fltmode-active-dot"></span>';
            }
            html += '</div>';
            html += '<select class="setup-select fltmode-select" data-slot="' + slot + '">';
            for (var j = 0; j < AVAILABLE_MODES.length; j++) {
                var m = AVAILABLE_MODES[j];
                html += '<option value="' + m.num + '"' + (m.num === currentMode ? ' selected' : '') + '>' + m.name + '</option>';
            }
            html += '</select>';
            html += '</div>';
        }
        html += '</div>';

        container.innerHTML = html;

        // Wire selects
        container.querySelectorAll('.fltmode-select').forEach(function (sel) {
            sel.addEventListener('change', function () {
                var slot = parseInt(sel.dataset.slot);
                var val = parseInt(sel.value);
                if (onParamChange) onParamChange('FLTMODE' + slot, val);
            });
        });

        var chSelect = document.getElementById('fltmode-ch-select');
        if (chSelect) {
            chSelect.addEventListener('change', function () {
                if (onParamChange) onParamChange('FLTMODE_CH', parseInt(chSelect.value));
            });
        }
    }

    return { render };

})();
