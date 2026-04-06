/* ============================================================
   compass-cal.js — Compass calibration wizard
   Progress bar + rotate vehicle instruction.
   ============================================================ */

'use strict';

window.CompassCal = (function () {

    var calState = {
        running: false,
        progress: 0,
        timer: null,
    };

    function render(container) {
        var params = meridian.v ? meridian.v.params : {};
        var isCalibrated = params.COMPASS_OFS_X !== undefined &&
            !(params.COMPASS_OFS_X === 0 && params.COMPASS_OFS_Y === 0 && params.COMPASS_OFS_Z === 0);

        var html = '<div class="setup-form-section">';
        html += '<div class="setup-form-title">Compass Calibration</div>';
        html += '<div class="cal-description">Slowly rotate the vehicle in all 3 axes until the progress bar reaches 100%. Try to cover all orientations evenly.</div>';
        html += '</div>';

        if (calState.running) {
            html += '<div class="compass-cal-visual">';
            html += '<div class="compass-cal-circle">';
            html += '<svg viewBox="0 0 120 120" width="120" height="120">';
            var r = 50, cx = 60, cy = 60;
            var circumference = 2 * Math.PI * r;
            var offset = circumference - (calState.progress / 100) * circumference;
            html += '<circle cx="' + cx + '" cy="' + cy + '" r="' + r + '" fill="none" stroke="var(--c-border)" stroke-width="6"/>';
            html += '<circle cx="' + cx + '" cy="' + cy + '" r="' + r + '" fill="none" stroke="var(--c-primary)" stroke-width="6" ';
            html += 'stroke-dasharray="' + circumference + '" stroke-dashoffset="' + offset + '" ';
            html += 'transform="rotate(-90 ' + cx + ' ' + cy + ')" stroke-linecap="round"/>';
            html += '<text x="' + cx + '" y="' + (cy + 6) + '" text-anchor="middle" fill="var(--c-text)" ';
            html += 'font-family="var(--f-display)" font-size="22" font-weight="700">' + Math.round(calState.progress) + '%</text>';
            html += '</svg>';
            html += '</div>';
            html += '<div class="compass-cal-instruction">Rotate the vehicle slowly...</div>';
            html += '</div>';

            html += '<div class="compass-cal-axes">';
            html += '<div class="compass-axis"><span class="axis-label">Roll</span><div class="axis-bar"><div class="axis-fill" style="width:' + Math.min(100, calState.progress * 1.1) + '%"></div></div></div>';
            html += '<div class="compass-axis"><span class="axis-label">Pitch</span><div class="axis-bar"><div class="axis-fill" style="width:' + Math.min(100, calState.progress * 0.95) + '%"></div></div></div>';
            html += '<div class="compass-axis"><span class="axis-label">Yaw</span><div class="axis-bar"><div class="axis-fill" style="width:' + Math.min(100, calState.progress * 1.05) + '%"></div></div></div>';
            html += '</div>';
        } else if (isCalibrated) {
            html += '<div class="cal-success">';
            html += 'Compass calibrated. Offsets: ';
            html += 'X=' + (params.COMPASS_OFS_X || 0).toFixed(1) + ', ';
            html += 'Y=' + (params.COMPASS_OFS_Y || 0).toFixed(1) + ', ';
            html += 'Z=' + (params.COMPASS_OFS_Z || 0).toFixed(1);
            html += '</div>';
        }

        html += '<div class="setup-form-actions">';
        if (calState.running) {
            html += '<button class="setup-btn danger" id="compass-cancel-btn">Cancel</button>';
        } else {
            html += '<button class="setup-btn primary" id="compass-start-btn">' + (isCalibrated ? 'Recalibrate' : 'Start Calibration') + '</button>';
        }
        html += '</div>';

        container.innerHTML = html;

        var startBtn = document.getElementById('compass-start-btn');
        if (startBtn) {
            startBtn.addEventListener('click', function () { startCal(container); });
        }
        var cancelBtn = document.getElementById('compass-cancel-btn');
        if (cancelBtn) {
            cancelBtn.addEventListener('click', function () { cancelCal(container); });
        }
    }

    function startCal(container) {
        calState.running = true;
        calState.progress = 0;
        render(container);

        calState.timer = setInterval(function () {
            calState.progress += 1.5 + Math.random() * 1.5;
            if (calState.progress >= 100) {
                calState.progress = 100;
                clearInterval(calState.timer);
                calState.timer = null;
                calState.running = false;
                // Write demo offsets
                var v = meridian.v;
                if (v) {
                    v.params.COMPASS_OFS_X = 42.5;
                    v.params.COMPASS_OFS_Y = -18.3;
                    v.params.COMPASS_OFS_Z = 105.7;
                    meridian.events.emit('param', { name: 'COMPASS_OFS_X', value: 42.5 });
                }
                meridian.log('Compass calibration complete', 'info');
                render(container);
                return;
            }
            render(container);
        }, 200);
    }

    function cancelCal(container) {
        if (calState.timer) clearInterval(calState.timer);
        calState.timer = null;
        calState.running = false;
        calState.progress = 0;
        render(container);
    }

    return { render };

})();
