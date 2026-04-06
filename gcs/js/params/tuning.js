/* ============================================================
   tuning.js — PID tuning panel
   Rate Roll/Pitch/Yaw P, I, D sliders.
   Live response chart (canvas: commanded vs actual rate).
   ============================================================ */

'use strict';

window.Tuning = (function () {

    var activeAxis = 'roll'; // 'roll' | 'pitch' | 'yaw'
    var chartCanvas = null;
    var chartCtx = null;
    var chartData = { commanded: [], actual: [] };
    var animFrame = null;

    var AXES = {
        roll:  { label: 'Rate Roll',  prefix: 'ATC_RAT_RLL_', pRange: [0, 0.5], iRange: [0, 0.5], dRange: [0, 0.02] },
        pitch: { label: 'Rate Pitch', prefix: 'ATC_RAT_PIT_', pRange: [0, 0.5], iRange: [0, 0.5], dRange: [0, 0.02] },
        yaw:   { label: 'Rate Yaw',   prefix: 'ATC_RAT_YAW_', pRange: [0, 0.5], iRange: [0, 0.05], dRange: [0, 0.02] },
    };

    function getAxisParams(axis) {
        var v = meridian.v;
        var params = v ? v.params : {};
        var cfg = AXES[axis];
        return {
            P: params[cfg.prefix + 'P'] !== undefined ? params[cfg.prefix + 'P'] : 0.135,
            I: params[cfg.prefix + 'I'] !== undefined ? params[cfg.prefix + 'I'] : 0.135,
            D: params[cfg.prefix + 'D'] !== undefined ? params[cfg.prefix + 'D'] : 0.0036,
        };
    }

    function render(container) {
        var cfg = AXES[activeAxis];
        var vals = getAxisParams(activeAxis);

        var html = '<div class="tuning-panel">';

        // Axis tabs
        html += '<div class="tuning-tabs">';
        var axisKeys = ['roll', 'pitch', 'yaw'];
        for (var i = 0; i < axisKeys.length; i++) {
            var ak = axisKeys[i];
            html += '<button class="tuning-tab' + (ak === activeAxis ? ' active' : '') + '" data-axis="' + ak + '">' + AXES[ak].label + '</button>';
        }
        html += '</div>';

        // Sliders
        html += '<div class="tuning-sliders">';
        html += renderSlider('P', vals.P, cfg.pRange[0], cfg.pRange[1], 0.001);
        html += renderSlider('I', vals.I, cfg.iRange[0], cfg.iRange[1], 0.001);
        html += renderSlider('D', vals.D, cfg.dRange[0], cfg.dRange[1], 0.0001);
        html += '</div>';

        // Response chart
        html += '<div class="tuning-chart-section">';
        html += '<div class="tuning-chart-label">Response (last 10s)</div>';
        html += '<div class="tuning-chart-legend">';
        html += '<span class="legend-cmd">Commanded</span>';
        html += '<span class="legend-act">Actual</span>';
        html += '</div>';
        html += '<canvas id="tuning-chart" width="300" height="120"></canvas>';
        html += '</div>';

        // Write button
        html += '<div class="setup-form-actions">';
        html += '<button class="setup-btn secondary" id="tuning-reset-btn">Reset to Defaults</button>';
        html += '<button class="setup-btn primary" id="tuning-write-btn">Write to Vehicle</button>';
        html += '</div>';

        html += '</div>';

        container.innerHTML = html;

        // Wire tabs
        container.querySelectorAll('.tuning-tab').forEach(function (tab) {
            tab.addEventListener('click', function () {
                activeAxis = tab.dataset.axis;
                render(container);
            });
        });

        // Wire sliders
        ['P', 'I', 'D'].forEach(function (gain) {
            var slider = document.getElementById('tuning-slider-' + gain);
            var valSpan = document.getElementById('tuning-val-' + gain);
            if (slider) {
                slider.addEventListener('input', function () {
                    var newVal = parseFloat(slider.value);
                    if (valSpan) valSpan.textContent = newVal.toFixed(gain === 'D' ? 4 : 3);
                });
            }
        });

        // Wire write
        var writeBtn = document.getElementById('tuning-write-btn');
        if (writeBtn) {
            writeBtn.addEventListener('click', function () {
                writePID(container);
            });
        }

        // Wire reset
        var resetBtn = document.getElementById('tuning-reset-btn');
        if (resetBtn) {
            resetBtn.addEventListener('click', function () {
                var defaults = ParamList.DEFAULTS;
                var prefix = AXES[activeAxis].prefix;
                var v = meridian.v;
                if (v) {
                    v.params[prefix + 'P'] = defaults[prefix + 'P'] || 0.135;
                    v.params[prefix + 'I'] = defaults[prefix + 'I'] || 0.135;
                    v.params[prefix + 'D'] = defaults[prefix + 'D'] || 0.0036;
                }
                meridian.log('PID reset to defaults for ' + AXES[activeAxis].label, 'info');
                render(container);
            });
        }

        // Init chart
        chartCanvas = document.getElementById('tuning-chart');
        if (chartCanvas) {
            chartCtx = chartCanvas.getContext('2d');
            chartCanvas.width = chartCanvas.parentElement.clientWidth - 4;
            startChartAnimation();
        }
    }

    function renderSlider(gain, value, min, max, step) {
        var html = '<div class="tuning-slider-row">';
        html += '<span class="tuning-gain-label">' + gain + ':</span>';
        html += '<input type="range" class="tuning-slider" id="tuning-slider-' + gain + '" ';
        html += 'min="' + min + '" max="' + max + '" step="' + step + '" value="' + value + '">';
        html += '<span class="tuning-val" id="tuning-val-' + gain + '">' + value.toFixed(gain === 'D' ? 4 : 3) + '</span>';
        html += '</div>';
        return html;
    }

    function writePID(container) {
        var prefix = AXES[activeAxis].prefix;
        var v = meridian.v;
        if (!v) return;

        ['P', 'I', 'D'].forEach(function (gain) {
            var slider = document.getElementById('tuning-slider-' + gain);
            if (slider) {
                var val = parseFloat(slider.value);
                v.params[prefix + gain] = val;
                meridian.events.emit('param', { name: prefix + gain, value: val });
            }
        });
        meridian.log('PID values written for ' + AXES[activeAxis].label, 'info');
    }

    function startChartAnimation() {
        // Generate simulated response data
        chartData.commanded = [];
        chartData.actual = [];

        var t = 0;
        function animate() {
            if (!chartCanvas || !chartCtx) return;
            if (!document.getElementById('tuning-chart')) { animFrame = null; return; }

            t += 0.05;
            var w = chartCanvas.width;
            var h = chartCanvas.height;

            // Get current PID values for response simulation
            var pSlider = document.getElementById('tuning-slider-P');
            var dSlider = document.getElementById('tuning-slider-D');
            var pVal = pSlider ? parseFloat(pSlider.value) : 0.135;
            var dVal = dSlider ? parseFloat(dSlider.value) : 0.0036;

            // Simulated commanded rate (step + sine)
            var cmd = 30 * Math.sin(t * 0.8) + 10 * Math.sin(t * 2.1);

            // Simulated actual rate (delayed + damped based on PID)
            var responsiveness = Math.min(1, pVal * 5);
            var damping = 1 - Math.min(0.8, dVal * 100);
            var overshoot = Math.max(0, (pVal - 0.1) * 3);
            var act = cmd * responsiveness + overshoot * 5 * Math.sin(t * 4) * damping;

            chartData.commanded.push(cmd);
            chartData.actual.push(act);
            if (chartData.commanded.length > 200) {
                chartData.commanded.shift();
                chartData.actual.shift();
            }

            // Draw
            chartCtx.fillStyle = 'var(--c-bg-input)';
            chartCtx.fillRect(0, 0, w, h);

            // Resolve CSS variable
            var cs = getComputedStyle(document.documentElement);
            var bgColor = cs.getPropertyValue('--c-bg-input').trim() || '#1a1f2e';
            var cmdColor = cs.getPropertyValue('--c-primary').trim() || '#0891b2';
            var actColor = cs.getPropertyValue('--c-text').trim() || '#e2e8f0';
            var gridColor = cs.getPropertyValue('--c-border').trim() || '#2a3041';

            chartCtx.fillStyle = bgColor;
            chartCtx.fillRect(0, 0, w, h);

            // Grid lines
            chartCtx.strokeStyle = gridColor;
            chartCtx.lineWidth = 0.5;
            for (var g = 0; g < 5; g++) {
                var gy = (g / 4) * h;
                chartCtx.beginPath();
                chartCtx.moveTo(0, gy);
                chartCtx.lineTo(w, gy);
                chartCtx.stroke();
            }

            // Zero line
            chartCtx.strokeStyle = gridColor;
            chartCtx.lineWidth = 1;
            chartCtx.beginPath();
            chartCtx.moveTo(0, h / 2);
            chartCtx.lineTo(w, h / 2);
            chartCtx.stroke();

            // Draw commanded
            drawLine(chartData.commanded, cmdColor, 1.5, w, h);
            // Draw actual
            drawLine(chartData.actual, actColor, 1.5, w, h);

            animFrame = requestAnimationFrame(animate);
        }

        animate();
    }

    function drawLine(data, color, width, canvasW, canvasH) {
        if (data.length < 2) return;
        chartCtx.strokeStyle = color;
        chartCtx.lineWidth = width;
        chartCtx.beginPath();
        var scale = canvasH / 120;
        var mid = canvasH / 2;
        for (var i = 0; i < data.length; i++) {
            var x = (i / 200) * canvasW;
            var y = mid - data[i] * scale;
            if (i === 0) chartCtx.moveTo(x, y);
            else chartCtx.lineTo(x, y);
        }
        chartCtx.stroke();
    }

    function destroy() {
        if (animFrame) {
            cancelAnimationFrame(animFrame);
            animFrame = null;
        }
        chartCanvas = null;
        chartCtx = null;
    }

    return { render, destroy };

})();
