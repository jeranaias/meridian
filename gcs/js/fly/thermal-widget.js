/* ============================================================
   thermal-widget.js — Thermal Camera Placeholder Widget
   T3-3: HUD widget (bottom-left) for thermal camera data.
   Reads v.thermalMinTemp / v.thermalMaxTemp / v.thermalSpotTemp.
   Toggle via meridian.settings.showThermalWidget.
   ============================================================ */

'use strict';

window.ThermalWidget = (function () {

    let widgetEl = null;
    let canvasEl = null;
    let ctx = null;
    let updateTimer = null;

    const W = 160;
    const BAR_H = 12;

    // --- Create DOM ---
    function createWidget() {
        if (widgetEl) return;

        widgetEl = document.createElement('div');
        widgetEl.id = 'thermal-widget';
        widgetEl.style.cssText = [
            'position:absolute',
            'bottom:100px',
            'left:12px',
            'z-index:1000',
            'background:rgba(8,11,16,0.85)',
            'border:1px solid rgba(0,229,255,0.2)',
            'border-radius:4px',
            'padding:7px 10px',
            'font-family:var(--font-mono,"DM Mono",monospace)',
            'font-size:11px',
            'color:#94a3b8',
            'min-width:' + W + 'px',
            'pointer-events:none',
            'display:none',
        ].join(';');

        widgetEl.innerHTML =
            '<div style="color:var(--c-primary);font-size:10px;letter-spacing:.08em;margin-bottom:5px">THERMAL</div>' +
            '<div id="thermal-status" style="color:var(--c-neutral)">Not Connected</div>' +
            '<div id="thermal-temps" style="display:none">' +
                '<div id="thermal-temp-row" style="display:flex;justify-content:space-between;margin-bottom:4px">' +
                    '<span>MIN: <span id="th-min" style="color:var(--c-text)">--</span>\u00B0C</span>' +
                    '<span>MAX: <span id="th-max" style="color:var(--c-text)">--</span>\u00B0C</span>' +
                '</div>' +
                '<div style="margin-bottom:4px">SPOT: <span id="th-spot" style="color:var(--c-warning)">--</span>\u00B0C</div>' +
                '<canvas id="thermal-bar" width="' + W + '" height="' + BAR_H + '" style="display:block;border-radius:2px"></canvas>' +
                '<div id="thermal-bar-labels" style="display:flex;justify-content:space-between;font-size:10px;color:var(--c-neutral-dim);margin-top:2px">' +
                    '<span id="th-label-min">--</span><span id="th-label-max">--</span>' +
                '</div>' +
            '</div>';

        const mapArea = document.getElementById('map-area');
        if (mapArea) mapArea.appendChild(widgetEl);

        canvasEl = document.getElementById('thermal-bar');
        if (canvasEl) ctx = canvasEl.getContext('2d');
    }

    // --- Draw gradient bar min -> max ---
    function drawBar(minT, maxT, spotT) {
        if (!ctx) return;
        ctx.clearRect(0, 0, W, BAR_H);

        // Gradient: deep blue (cold) -> cyan -> green -> yellow -> red (hot)
        const grad = ctx.createLinearGradient(0, 0, W, 0);
        grad.addColorStop(0.0, '#0c4a6e');   // cold: dark blue
        grad.addColorStop(0.25, '#0891b2');  // cyan
        grad.addColorStop(0.5, '#22c55e');   // green (mid)
        grad.addColorStop(0.75, '#f59e0b');  // amber
        grad.addColorStop(1.0, '#ef4444');   // red (hot)

        ctx.fillStyle = grad;
        ctx.beginPath();
        ctx.roundRect(0, 0, W, BAR_H, 2);
        ctx.fill();

        // Spot temperature marker
        if (spotT !== undefined && maxT !== minT) {
            const range = maxT - minT;
            const spotX = Math.max(2, Math.min(W - 2, ((spotT - minT) / range) * W));
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(spotX, 0);
            ctx.lineTo(spotX, BAR_H);
            ctx.stroke();
        }
    }

    // --- Update widget from vehicle state ---
    function update() {
        if (!widgetEl) return;

        const show = meridian.settings.showThermalWidget !== false;
        if (!show) {
            widgetEl.style.display = 'none';
            return;
        }

        widgetEl.style.display = 'block';

        const v = meridian.v;
        const hasData = v &&
            v.thermalMinTemp !== undefined &&
            v.thermalMaxTemp !== undefined;

        const statusEl = document.getElementById('thermal-status');
        const tempsEl = document.getElementById('thermal-temps');

        if (!hasData) {
            if (statusEl) statusEl.style.display = 'block';
            if (tempsEl) tempsEl.style.display = 'none';
            if (statusEl) statusEl.textContent = 'Not Connected';
            return;
        }

        if (statusEl) statusEl.style.display = 'none';
        if (tempsEl) tempsEl.style.display = 'block';

        const minT = v.thermalMinTemp;
        const maxT = v.thermalMaxTemp;
        const spotT = v.thermalSpotTemp;

        const minEl = document.getElementById('th-min');
        const maxEl = document.getElementById('th-max');
        const spotEl = document.getElementById('th-spot');
        const labelMin = document.getElementById('th-label-min');
        const labelMax = document.getElementById('th-label-max');

        if (minEl) minEl.textContent = minT.toFixed(1);
        if (maxEl) maxEl.textContent = maxT.toFixed(1);
        if (spotEl) spotEl.textContent = spotT !== undefined ? spotT.toFixed(1) : '--';

        if (labelMin) labelMin.textContent = minT.toFixed(0) + '°';
        if (labelMax) labelMax.textContent = maxT.toFixed(0) + '°';

        drawBar(minT, maxT, spotT);
    }

    function init() {
        createWidget();
        update();
        updateTimer = setInterval(update, 1000);

        // Respond to settings changes
        meridian.events.on('settings_change', function (data) {
            if (data.key === 'showThermalWidget') update();
        });
    }

    function destroy() {
        if (updateTimer) { clearInterval(updateTimer); updateTimer = null; }
    }

    return { init, destroy, update };

})();
