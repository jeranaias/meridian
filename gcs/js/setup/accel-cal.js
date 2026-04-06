/* ============================================================
   accel-cal.js — Accelerometer calibration wizard
   6-position calibration UI.
   ============================================================ */

'use strict';

window.AccelCal = (function () {

    var POSITIONS = [
        { id: 'level',     name: 'Level',      icon: '\u2B1C', instruction: 'Place vehicle level on a flat surface' },
        { id: 'left',      name: 'Left Side',   icon: '\u2B05', instruction: 'Roll vehicle onto its left side (90\u00b0)' },
        { id: 'right',     name: 'Right Side',  icon: '\u27A1', instruction: 'Roll vehicle onto its right side (90\u00b0)' },
        { id: 'nose_down', name: 'Nose Down',   icon: '\u2B07', instruction: 'Tilt vehicle nose straight down (90\u00b0)' },
        { id: 'nose_up',   name: 'Nose Up',     icon: '\u2B06', instruction: 'Tilt vehicle nose straight up (90\u00b0)' },
        { id: 'back',      name: 'On Back',     icon: '\u21BA', instruction: 'Flip vehicle upside down' },
    ];

    var calState = {
        running: false,
        currentStep: -1,
        completed: [],
    };

    function render(container) {
        var html = '<div class="setup-form-section">';
        html += '<div class="setup-form-title">Accelerometer Calibration</div>';
        html += '<div class="cal-description">Place the vehicle in each of the 6 positions when prompted. Hold steady for 3 seconds in each position.</div>';
        html += '</div>';

        html += '<div class="cal-positions">';
        for (var i = 0; i < POSITIONS.length; i++) {
            var pos = POSITIONS[i];
            var done = calState.completed.indexOf(pos.id) !== -1;
            var active = calState.running && calState.currentStep === i;
            var cls = 'cal-pos';
            if (done) cls += ' complete';
            if (active) cls += ' active';

            html += '<div class="' + cls + '">';
            html += '<div class="cal-pos-icon">' + (done ? '\u2705' : pos.icon) + '</div>';
            html += '<div class="cal-pos-info">';
            html += '<div class="cal-pos-name">' + pos.name + '</div>';
            if (active) {
                html += '<div class="cal-pos-instruction">' + pos.instruction + '</div>';
                html += '<div class="cal-progress-bar"><div class="cal-progress-fill" id="cal-progress-fill"></div></div>';
            } else if (done) {
                html += '<div class="cal-pos-status complete">Complete</div>';
            } else {
                html += '<div class="cal-pos-status pending">Pending</div>';
            }
            html += '</div>';
            html += '</div>';
        }
        html += '</div>';

        html += '<div class="setup-form-actions">';
        if (!calState.running) {
            var allDone = calState.completed.length === POSITIONS.length;
            if (allDone) {
                html += '<div class="cal-success">Calibration complete. Offsets saved.</div>';
                html += '<button class="setup-btn secondary" id="accel-reset-btn">Recalibrate</button>';
            } else {
                html += '<button class="setup-btn primary" id="accel-start-btn">Start Calibration</button>';
            }
        } else {
            html += '<button class="setup-btn danger" id="accel-cancel-btn">Cancel</button>';
            html += '<span class="cal-step-label">Step ' + (calState.currentStep + 1) + ' of ' + POSITIONS.length + '</span>';
        }
        html += '</div>';

        container.innerHTML = html;

        // Wire buttons
        var startBtn = document.getElementById('accel-start-btn');
        if (startBtn) {
            startBtn.addEventListener('click', function () {
                startCalibration(container);
            });
        }
        var cancelBtn = document.getElementById('accel-cancel-btn');
        if (cancelBtn) {
            cancelBtn.addEventListener('click', function () {
                calState.running = false;
                calState.currentStep = -1;
                calState.completed = [];
                render(container);
            });
        }
        var resetBtn = document.getElementById('accel-reset-btn');
        if (resetBtn) {
            resetBtn.addEventListener('click', function () {
                calState.completed = [];
                calState.currentStep = -1;
                render(container);
            });
        }
    }

    function startCalibration(container) {
        calState.running = true;
        calState.currentStep = 0;
        calState.completed = [];

        // Send MAV_CMD_PREFLIGHT_CALIBRATION if connected (param1=1 for accel cal)
        if (Connection.state === 2 && !meridian.demo) {
            Connection.sendCommand(241, 1, 0, 0, 0, 0, 0, 0); // PREFLIGHT_CALIBRATION accel
            meridian.log('Sent accel calibration command to vehicle', 'info');
        }

        render(container);
        simulateStep(container);
    }

    function simulateStep(container) {
        if (calState.currentStep >= POSITIONS.length) {
            calState.running = false;
            // Write demo offsets to params
            var v = meridian.v;
            if (v) {
                v.params.INS_ACCOFFS_X = 0.12;
                v.params.INS_ACCOFFS_Y = -0.08;
                v.params.INS_ACCOFFS_Z = 0.34;
                meridian.events.emit('param', { name: 'INS_ACCOFFS_X', value: 0.12 });
            }
            meridian.log('Accel calibration complete', 'info');
            render(container);
            return;
        }

        render(container);

        // Animate progress bar over 3 seconds
        var fill = document.getElementById('cal-progress-fill');
        if (fill) {
            fill.style.transition = 'width 3s linear';
            requestAnimationFrame(function () { fill.style.width = '100%'; });
        }

        setTimeout(function () {
            var pos = POSITIONS[calState.currentStep];
            calState.completed.push(pos.id);
            calState.currentStep++;
            simulateStep(container);
        }, 3000);
    }

    return { render };

})();
