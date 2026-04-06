/* ============================================================
   motor-test.js — Motor test interface
   Test individual motors at configurable throttle.
   ============================================================ */

'use strict';

window.MotorTest = (function () {

    var testState = {
        activeMotor: -1,
        throttle: 5,
        duration: 2,
    };

    function render(container) {
        var v = meridian.v;
        var params = v ? v.params : {};
        var frameClass = params.FRAME_CLASS || 1;

        // Determine motor count from frame class
        var motorCount = 4;
        if (frameClass === 2 || frameClass === 5) motorCount = 6;
        else if (frameClass === 3 || frameClass === 4) motorCount = 8;
        else if (frameClass === 7) motorCount = 3;

        var html = '<div class="setup-form-section">';
        html += '<div class="setup-form-title">Motor Test</div>';
        html += '<div class="cal-description">Test individual motors. Ensure propellers are removed for safety. Vehicle must be disarmed.</div>';
        html += '</div>';

        html += '<div class="motor-test-controls">';
        html += '<div class="fs-row">';
        html += '<label>Throttle (%):</label>';
        html += '<input type="range" class="setup-range" id="motor-throttle" min="1" max="30" value="' + testState.throttle + '">';
        html += '<span class="range-val" id="motor-throttle-val">' + testState.throttle + '%</span>';
        html += '</div>';
        html += '<div class="fs-row">';
        html += '<label>Duration (s):</label>';
        html += '<input type="range" class="setup-range" id="motor-duration" min="1" max="10" value="' + testState.duration + '">';
        html += '<span class="range-val" id="motor-duration-val">' + testState.duration + 's</span>';
        html += '</div>';
        html += '</div>';

        html += '<div class="motor-grid">';
        for (var i = 0; i < motorCount; i++) {
            var isActive = testState.activeMotor === i;
            html += '<button class="motor-btn' + (isActive ? ' active' : '') + '" data-motor="' + i + '">';
            html += '<div class="motor-icon">';
            html += '<svg viewBox="0 0 32 32" width="32" height="32">';
            html += '<circle cx="16" cy="16" r="10" fill="none" stroke="' + (isActive ? 'var(--c-primary)' : 'currentColor') + '" stroke-width="1.5"/>';
            if (isActive) {
                html += '<circle cx="16" cy="16" r="6" fill="none" stroke="var(--c-primary)" stroke-width="1" opacity="0.5"><animate attributeName="r" from="6" to="12" dur="0.8s" repeatCount="indefinite"/><animate attributeName="opacity" from="0.5" to="0" dur="0.8s" repeatCount="indefinite"/></circle>';
            }
            html += '<text x="16" y="20" text-anchor="middle" fill="currentColor" font-size="10" font-family="var(--f-display)">' + (i + 1) + '</text>';
            html += '</svg>';
            html += '</div>';
            html += '<span class="motor-label">Motor ' + (i + 1) + '</span>';
            html += '</button>';
        }
        html += '</div>';

        html += '<div class="setup-form-actions">';
        html += '<button class="setup-btn primary" id="motor-test-all-btn">Test All Sequence</button>';
        html += '<button class="setup-btn danger" id="motor-stop-btn">Stop All</button>';
        html += '</div>';

        container.innerHTML = html;

        // Wire throttle slider
        var thrSlider = document.getElementById('motor-throttle');
        var thrVal = document.getElementById('motor-throttle-val');
        if (thrSlider) {
            thrSlider.addEventListener('input', function () {
                testState.throttle = parseInt(thrSlider.value);
                if (thrVal) thrVal.textContent = testState.throttle + '%';
            });
        }

        // Wire duration slider
        var durSlider = document.getElementById('motor-duration');
        var durVal = document.getElementById('motor-duration-val');
        if (durSlider) {
            durSlider.addEventListener('input', function () {
                testState.duration = parseInt(durSlider.value);
                if (durVal) durVal.textContent = testState.duration + 's';
            });
        }

        // Wire motor buttons
        container.querySelectorAll('.motor-btn').forEach(function (btn) {
            btn.addEventListener('click', function () {
                var motor = parseInt(btn.dataset.motor);
                testMotor(motor, container);
            });
        });

        // Wire test all
        var testAllBtn = document.getElementById('motor-test-all-btn');
        if (testAllBtn) {
            testAllBtn.addEventListener('click', function () {
                testAllSequence(motorCount, container);
            });
        }

        // Wire stop
        var stopBtn = document.getElementById('motor-stop-btn');
        if (stopBtn) {
            stopBtn.addEventListener('click', function () {
                testState.activeMotor = -1;
                meridian.log('Motors stopped', 'info');
                render(container);
            });
        }
    }

    function testMotor(motorNum, container) {
        testState.activeMotor = motorNum;
        meridian.log('Testing motor ' + (motorNum + 1) + ' at ' + testState.throttle + '% for ' + testState.duration + 's', 'info');

        // Send MAV_CMD_DO_MOTOR_TEST (cmd 209) if connected
        // param1=motor instance (1-based), param2=throttle type (0=%), param3=throttle%, param4=timeout, param5=motor count, param6=test order
        if (Connection.state === 2 && !meridian.demo) {
            Connection.sendCommand(209, motorNum + 1, 0, testState.throttle, testState.duration, 0, 0, 0);
        }

        render(container);

        setTimeout(function () {
            if (testState.activeMotor === motorNum) {
                testState.activeMotor = -1;
                render(container);
            }
        }, testState.duration * 1000);
    }

    function testAllSequence(count, container) {
        var idx = 0;
        function next() {
            if (idx >= count) {
                testState.activeMotor = -1;
                render(container);
                meridian.log('Motor test sequence complete', 'info');
                return;
            }
            testState.activeMotor = idx;
            render(container);
            meridian.log('Testing motor ' + (idx + 1), 'info');
            idx++;
            setTimeout(next, testState.duration * 1000);
        }
        next();
    }

    return { render };

})();
