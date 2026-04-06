/* ============================================================
   radio-cal.js — Radio calibration
   Shows 8 channel bars with current values + min/max capture.
   ============================================================ */

'use strict';

window.RadioCal = (function () {

    var calState = {
        running: false,
        mins: [],
        maxs: [],
        listener: null,
    };

    var CH_NAMES = ['Roll', 'Pitch', 'Throttle', 'Yaw', 'Ch 5', 'Ch 6', 'Ch 7', 'Ch 8'];

    function render(container) {
        var v = meridian.v;
        var channels = (v && v.rcChannels && v.rcChannels.length > 0) ? v.rcChannels : [1500,1500,1000,1500,1000,1500,1500,1500];
        var params = v ? v.params : {};

        var html = '<div class="setup-form-section">';
        html += '<div class="setup-form-title">Radio Calibration</div>';
        html += '<div class="cal-description">';
        if (calState.running) {
            html += 'Move all sticks and switches to their full extent, then click Save.';
        } else {
            html += 'Verify your RC transmitter is bound and channels are responding. Click Calibrate to capture min/max ranges.';
        }
        html += '</div>';
        html += '</div>';

        html += '<div class="radio-channels">';
        for (var i = 0; i < 8; i++) {
            var val = channels[i] || 1500;
            var pMin = params['RC' + (i + 1) + '_MIN'] || 1000;
            var pMax = params['RC' + (i + 1) + '_MAX'] || 2000;
            var range = pMax - pMin;
            var pct = range > 0 ? Math.max(0, Math.min(100, ((val - pMin) / range) * 100)) : 50;

            var calMin = calState.running && calState.mins[i] !== undefined ? calState.mins[i] : null;
            var calMax = calState.running && calState.maxs[i] !== undefined ? calState.maxs[i] : null;

            html += '<div class="radio-ch">';
            html += '<div class="radio-ch-header">';
            html += '<span class="radio-ch-name">' + CH_NAMES[i] + '</span>';
            html += '<span class="radio-ch-val">' + val + '</span>';
            html += '</div>';
            html += '<div class="radio-bar-track">';
            html += '<div class="radio-bar-fill" style="width:' + pct + '%"></div>';
            html += '<div class="radio-bar-marker" style="left:' + pct + '%"></div>';
            html += '</div>';
            html += '<div class="radio-ch-range">';
            if (calState.running && calMin !== null) {
                html += '<span class="radio-cal-val">' + calMin + '</span>';
                html += '<span class="radio-cal-val">' + calMax + '</span>';
            } else {
                html += '<span>' + pMin + '</span>';
                html += '<span>' + pMax + '</span>';
            }
            html += '</div>';
            html += '</div>';
        }
        html += '</div>';

        html += '<div class="setup-form-actions">';
        if (calState.running) {
            html += '<button class="setup-btn danger" id="radio-cancel-btn">Cancel</button>';
            html += '<button class="setup-btn primary" id="radio-save-btn">Save Calibration</button>';
        } else {
            html += '<button class="setup-btn primary" id="radio-cal-btn">Calibrate</button>';
        }
        html += '</div>';

        container.innerHTML = html;

        var calBtn = document.getElementById('radio-cal-btn');
        if (calBtn) calBtn.addEventListener('click', function () { startCal(container); });

        var cancelBtn = document.getElementById('radio-cancel-btn');
        if (cancelBtn) cancelBtn.addEventListener('click', function () { cancelCal(container); });

        var saveBtn = document.getElementById('radio-save-btn');
        if (saveBtn) saveBtn.addEventListener('click', function () { saveCal(container); });
    }

    function startCal(container) {
        var channels = meridian.v && meridian.v.rcChannels ? meridian.v.rcChannels : [];
        calState.running = true;
        calState.mins = [];
        calState.maxs = [];
        for (var i = 0; i < 8; i++) {
            var val = channels[i] || 1500;
            calState.mins[i] = val;
            calState.maxs[i] = val;
        }

        // Listen for RC updates
        calState.listener = function () {
            var v = meridian.v;
            if (!v || !calState.running) return;
            var ch = v.rcChannels || [];
            for (var j = 0; j < 8; j++) {
                var c = ch[j];
                if (c === undefined) continue;
                if (c < calState.mins[j]) calState.mins[j] = c;
                if (c > calState.maxs[j]) calState.maxs[j] = c;
            }
            render(container);
        };
        meridian.events.on('rc', calState.listener);

        render(container);

        // In demo mode, simulate some stick movement
        if (meridian.demo) {
            simulateStickMovement(container);
        }
    }

    function simulateStickMovement(container) {
        var steps = 0;
        var simTimer = setInterval(function () {
            if (!calState.running) { clearInterval(simTimer); return; }
            steps++;
            var v = meridian.v;
            if (!v) return;
            // Simulate moving channels to min/max
            for (var i = 0; i < 8; i++) {
                if (steps < 10) {
                    calState.mins[i] = Math.min(calState.mins[i], 1000 + Math.random() * 100);
                } else {
                    calState.maxs[i] = Math.max(calState.maxs[i], 1900 + Math.random() * 100);
                }
            }
            render(container);
            if (steps >= 20) clearInterval(simTimer);
        }, 300);
    }

    function saveCal(container) {
        if (calState.listener) {
            meridian.events.off('rc', calState.listener);
            calState.listener = null;
        }
        // Write params
        var v = meridian.v;
        if (v) {
            for (var i = 0; i < 8; i++) {
                var chNum = i + 1;
                v.params['RC' + chNum + '_MIN'] = calState.mins[i] || 1000;
                v.params['RC' + chNum + '_MAX'] = calState.maxs[i] || 2000;
            }
            meridian.events.emit('param', { name: 'RC1_MIN', value: calState.mins[0] });
        }
        calState.running = false;
        meridian.log('Radio calibration saved', 'info');
        render(container);
    }

    function cancelCal(container) {
        if (calState.listener) {
            meridian.events.off('rc', calState.listener);
            calState.listener = null;
        }
        calState.running = false;
        render(container);
    }

    return { render };

})();
