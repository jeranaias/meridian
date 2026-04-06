/* ============================================================
   log-graph.js — Canvas-based line chart for telemetry fields
   Plots any numeric field from meridian.v over time.
   ============================================================ */

'use strict';

window.LogGraph = (function () {

    let canvas = null;
    let ctx = null;
    let container = null;
    let selectedField = 'relativeAlt';
    let dataBuffer = [];
    let maxPoints = 600; // ~2.5 min at 4Hz
    let updateTimer = null;
    let scrollOffset = 0;
    let paused = false;

    // Fields available for graphing
    const FIELDS = [
        { key: 'relativeAlt',  label: 'Altitude (rel)' },
        { key: 'alt',          label: 'Altitude (abs)' },
        { key: 'groundspeed',  label: 'Ground Speed' },
        { key: 'airspeed',     label: 'Air Speed' },
        { key: 'climb',        label: 'Climb Rate' },
        { key: 'throttle',     label: 'Throttle %' },
        { key: 'voltage',      label: 'Battery Voltage' },
        { key: 'current',      label: 'Battery Current' },
        { key: 'batteryPct',   label: 'Battery %' },
        { key: 'heading',      label: 'Heading' },
        { key: 'satellites',   label: 'GPS Sats' },
        { key: 'hdop',         label: 'HDOP' },
        { key: 'roll',         label: 'Roll (rad)' },
        { key: 'pitch',        label: 'Pitch (rad)' },
        { key: 'yaw',          label: 'Yaw (rad)' },
        { key: 'ekfVelVar',    label: 'EKF Vel Var' },
        { key: 'ekfPosVar',    label: 'EKF Pos Var' },
        { key: 'ekfHgtVar',    label: 'EKF Hgt Var' },
        { key: 'rcRssi',       label: 'RC RSSI' },
    ];

    function render(cont) {
        container = cont;
        container.innerHTML = '';
        dataBuffer = [];
        scrollOffset = 0;
        paused = false;

        // Controls
        const controls = document.createElement('div');
        controls.className = 'graph-controls';

        const select = document.createElement('select');
        select.className = 'graph-field-select';
        FIELDS.forEach(function (f) {
            const opt = document.createElement('option');
            opt.value = f.key;
            opt.textContent = f.label;
            if (f.key === selectedField) opt.selected = true;
            select.appendChild(opt);
        });
        select.addEventListener('change', function () {
            selectedField = select.value;
            dataBuffer = [];
            scrollOffset = 0;
        });
        controls.appendChild(select);

        const pauseBtn = document.createElement('button');
        pauseBtn.className = 'log-action-btn';
        pauseBtn.textContent = 'Pause';
        pauseBtn.addEventListener('click', function () {
            paused = !paused;
            pauseBtn.textContent = paused ? 'Resume' : 'Pause';
        });
        controls.appendChild(pauseBtn);

        const clearBtn = document.createElement('button');
        clearBtn.className = 'log-action-btn';
        clearBtn.textContent = 'Clear';
        clearBtn.addEventListener('click', function () {
            dataBuffer = [];
            scrollOffset = 0;
        });
        controls.appendChild(clearBtn);

        container.appendChild(controls);

        // Canvas
        const canvasWrap = document.createElement('div');
        canvasWrap.className = 'graph-canvas-wrap';

        canvas = document.createElement('canvas');
        canvas.className = 'graph-canvas';
        canvasWrap.appendChild(canvas);
        container.appendChild(canvasWrap);

        // Scroll bar
        const scrollBar = document.createElement('input');
        scrollBar.type = 'range';
        scrollBar.className = 'graph-scroll';
        scrollBar.min = 0;
        scrollBar.max = 0;
        scrollBar.value = 0;
        scrollBar.addEventListener('input', function () {
            scrollOffset = parseInt(scrollBar.value);
            drawChart();
        });
        container.appendChild(scrollBar);

        // Value readout
        const readout = document.createElement('div');
        readout.className = 'graph-readout';
        readout.id = 'graph-readout';
        container.appendChild(readout);

        resizeCanvas();

        // Start sampling
        if (updateTimer) clearInterval(updateTimer);
        updateTimer = setInterval(function () {
            if (paused) return;
            const v = meridian.v;
            if (!v) return;
            const val = v[selectedField];
            if (typeof val === 'number') {
                dataBuffer.push({ t: Date.now(), v: val });
                if (dataBuffer.length > maxPoints) dataBuffer.shift();

                // Update scroll range
                const visiblePoints = Math.floor((canvas.width - 60) / 3);
                if (dataBuffer.length > visiblePoints) {
                    scrollBar.max = dataBuffer.length - visiblePoints;
                    scrollBar.value = scrollBar.max;
                    scrollOffset = parseInt(scrollBar.max);
                } else {
                    scrollBar.max = 0;
                    scrollBar.value = 0;
                    scrollOffset = 0;
                }
            }
            drawChart();
        }, 250); // 4Hz

        window.addEventListener('resize', resizeCanvas);
    }

    function resizeCanvas() {
        if (!canvas || !canvas.parentElement) return;
        const rect = canvas.parentElement.getBoundingClientRect();
        canvas.width = Math.floor(rect.width);
        canvas.height = Math.floor(rect.height);
        ctx = canvas.getContext('2d');
        drawChart();
    }

    function drawChart() {
        if (!ctx || !canvas) return;

        const W = canvas.width;
        const H = canvas.height;
        const pad = { top: 10, right: 10, bottom: 24, left: 50 };

        const isDark = document.documentElement.getAttribute('data-theme') === 'dark';
        const bgColor = isDark ? '#0e1218' : '#f8f9fb';
        const gridColor = isDark ? 'rgba(255,255,255,0.06)' : 'rgba(0,0,0,0.06)';
        const textColor = isDark ? '#7b8da3' : '#64748b';
        const lineColor = isDark ? '#00e5ff' : '#0891b2';
        const fillColor = isDark ? 'rgba(0,229,255,0.08)' : 'rgba(8,145,178,0.08)';

        ctx.fillStyle = bgColor;
        ctx.fillRect(0, 0, W, H);

        if (dataBuffer.length < 2) {
            ctx.fillStyle = textColor;
            ctx.font = '12px "DM Mono", monospace';
            ctx.textAlign = 'center';
            ctx.fillText('Waiting for data...', W / 2, H / 2);
            return;
        }

        const chartW = W - pad.left - pad.right;
        const chartH = H - pad.top - pad.bottom;

        // Determine visible slice
        const pixPerPoint = 3;
        const visibleCount = Math.floor(chartW / pixPerPoint);
        const start = scrollOffset;
        const end = Math.min(start + visibleCount, dataBuffer.length);
        const visible = dataBuffer.slice(start, end);

        if (visible.length < 2) return;

        // Value range
        let min = Infinity, max = -Infinity;
        for (let i = 0; i < visible.length; i++) {
            if (visible[i].v < min) min = visible[i].v;
            if (visible[i].v > max) max = visible[i].v;
        }
        if (min === max) { min -= 1; max += 1; }
        const range = max - min;
        const margin = range * 0.1;
        min -= margin;
        max += margin;

        // Grid lines (5 horizontal)
        ctx.strokeStyle = gridColor;
        ctx.lineWidth = 1;
        ctx.font = '10px "DM Mono", monospace';
        ctx.fillStyle = textColor;
        ctx.textAlign = 'right';
        for (let i = 0; i <= 4; i++) {
            const y = pad.top + (chartH * i / 4);
            const val = max - (max - min) * (i / 4);
            ctx.beginPath();
            ctx.moveTo(pad.left, y);
            ctx.lineTo(W - pad.right, y);
            ctx.stroke();
            ctx.fillText(val.toFixed(1), pad.left - 4, y + 3);
        }

        // Time axis labels
        ctx.textAlign = 'center';
        const timeSpan = visible[visible.length - 1].t - visible[0].t;
        for (let i = 0; i <= 4; i++) {
            const x = pad.left + (chartW * i / 4);
            const t = visible[0].t + timeSpan * (i / 4);
            const d = new Date(t);
            ctx.fillText(d.getMinutes() + ':' + String(d.getSeconds()).padStart(2, '0'), x, H - 4);
        }

        // Data line
        ctx.beginPath();
        for (let i = 0; i < visible.length; i++) {
            const x = pad.left + (i / (visible.length - 1)) * chartW;
            const y = pad.top + chartH - ((visible[i].v - min) / (max - min)) * chartH;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.strokeStyle = lineColor;
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Fill under line
        ctx.lineTo(pad.left + chartW, pad.top + chartH);
        ctx.lineTo(pad.left, pad.top + chartH);
        ctx.closePath();
        ctx.fillStyle = fillColor;
        ctx.fill();

        // Current value readout
        const last = visible[visible.length - 1];
        const readout = document.getElementById('graph-readout');
        if (readout) {
            const fieldLabel = FIELDS.find(function (f) { return f.key === selectedField; });
            readout.textContent = (fieldLabel ? fieldLabel.label : selectedField) + ': ' + last.v.toFixed(2);
        }
    }

    function destroy() {
        if (updateTimer) clearInterval(updateTimer);
        updateTimer = null;
        window.removeEventListener('resize', resizeCanvas);
    }

    return { render, destroy };

})();
