/* ============================================================
   split-view.js — Split-screen multi-vehicle view (T3-12)
   Toggle splits map area: Vehicle 1 instruments left,
   Vehicle 2 instruments right, shared map in center.
   ============================================================ */

'use strict';

window.SplitView = (function () {

    var active = false;
    var containerEl = null;
    var leftPanelEl = null;
    var rightPanelEl = null;
    var overlayEl = null;
    var toggleBtn = null;
    var drawTimer = null;

    // ---- Init ----

    function init() {
        injectToggleButton();
        buildOverlay();

        meridian.events.on('heartbeat', function () {
            if (active) updatePanels();
        });

        meridian.events.on('battery', function () {
            if (active) updatePanels();
        });
    }

    // ---- Toolbar Button ----

    function injectToggleButton() {
        var mapToolbar = document.querySelector('.map-toolbar');
        if (!mapToolbar || document.getElementById('btn-split-view')) return;

        var sep = document.createElement('div');
        sep.className = 'map-toolbar-sep';

        toggleBtn = document.createElement('button');
        toggleBtn.id = 'btn-split-view';
        toggleBtn.title = 'Toggle split-screen multi-vehicle view';
        toggleBtn.setAttribute('aria-label', 'Toggle split view');
        toggleBtn.setAttribute('aria-pressed', 'false');
        toggleBtn.innerHTML =
            '<svg width="14" height="14" viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.5">' +
            '<rect x="1" y="1" width="5" height="12" rx="1"/>' +
            '<rect x="8" y="1" width="5" height="12" rx="1"/>' +
            '</svg>';
        toggleBtn.addEventListener('click', toggle);

        mapToolbar.appendChild(sep);
        mapToolbar.appendChild(toggleBtn);
    }

    // ---- Build Overlay ----

    function buildOverlay() {
        var mapArea = document.getElementById('map-area');
        if (!mapArea || document.getElementById('split-overlay')) return;

        overlayEl = document.createElement('div');
        overlayEl.id = 'split-overlay';
        overlayEl.className = 'split-overlay';
        overlayEl.style.display = 'none';

        // Left panel
        leftPanelEl = document.createElement('div');
        leftPanelEl.className = 'split-panel split-panel-left';
        leftPanelEl.innerHTML = buildPanelHTML('left');

        // Right panel
        rightPanelEl = document.createElement('div');
        rightPanelEl.className = 'split-panel split-panel-right';
        rightPanelEl.innerHTML = buildPanelHTML('right');

        overlayEl.appendChild(leftPanelEl);
        overlayEl.appendChild(rightPanelEl);
        mapArea.appendChild(overlayEl);
    }

    function buildPanelHTML(side) {
        var id = side;
        return [
            '<div class="split-vehicle-label" id="split-label-' + id + '">---</div>',
            '<div class="split-mode-row" id="split-mode-' + id + '">',
                '<span class="split-arm-dot" id="split-arm-dot-' + id + '"></span>',
                '<span class="split-mode-text" id="split-mode-text-' + id + '">UNKNOWN</span>',
            '</div>',
            '<canvas class="split-adi-canvas" id="split-adi-' + id + '" width="120" height="120"></canvas>',
            '<div class="split-tape-row">',
                '<div class="split-tape-cell">',
                    '<div class="split-tape-label">ALT</div>',
                    '<div class="split-tape-value" id="split-alt-' + id + '">--</div>',
                '</div>',
                '<div class="split-tape-cell">',
                    '<div class="split-tape-label">SPD</div>',
                    '<div class="split-tape-value" id="split-spd-' + id + '">--</div>',
                '</div>',
            '</div>',
            '<div class="split-battery-row" id="split-batt-' + id + '">',
                '<div class="split-batt-bar"><div class="split-batt-fill" id="split-batt-fill-' + id + '"></div></div>',
                '<span class="split-batt-text" id="split-batt-text-' + id + '">--V</span>',
            '</div>',
        ].join('');
    }

    // ---- Toggle ----

    function toggle() {
        active = !active;

        if (active) {
            enable();
        } else {
            disable();
        }
    }

    function enable() {
        active = true;
        if (overlayEl) overlayEl.style.display = 'flex';
        if (toggleBtn) {
            toggleBtn.classList.add('active');
            toggleBtn.setAttribute('aria-pressed', 'true');
        }
        document.getElementById('map-area').classList.add('split-active');
        updatePanels();
        startMiniADILoop();
        meridian.log('Split view enabled', 'info');
    }

    function disable() {
        active = false;
        if (overlayEl) overlayEl.style.display = 'none';
        if (toggleBtn) {
            toggleBtn.classList.remove('active');
            toggleBtn.setAttribute('aria-pressed', 'false');
        }
        document.getElementById('map-area').classList.remove('split-active');
        if (drawTimer) { cancelAnimationFrame(drawTimer); drawTimer = null; }
        meridian.log('Split view disabled', 'info');
    }

    // ---- Update Panel Data ----

    function updatePanels() {
        var ids = Object.keys(meridian.vehicles).map(Number).sort();
        var v1 = meridian.vehicles[ids[0]] || null;
        var v2 = meridian.vehicles[ids[1]] || null;

        updatePanel('left', v1, ids[0]);
        updatePanel('right', v2, ids[1]);
    }

    function updatePanel(side, v, sysid) {
        if (!v) {
            setEl('split-label-' + side, 'No Vehicle');
            setEl('split-mode-text-' + side, '---');
            setEl('split-alt-' + side, '--');
            setEl('split-spd-' + side, '--');
            setEl('split-batt-text-' + side, '--');
            var dot = document.getElementById('split-arm-dot-' + side);
            if (dot) dot.className = 'split-arm-dot disconnected';
            return;
        }

        // Label: fleet name if available, else V<sysid>
        var label = (v._fleetName) ? v._fleetName : ('V' + sysid);
        setEl('split-label-' + side, label);

        // Mode + arm state
        setEl('split-mode-text-' + side, v.modeName || 'UNKNOWN');
        var armDot = document.getElementById('split-arm-dot-' + side);
        if (armDot) {
            armDot.className = 'split-arm-dot ' + (v.armed ? 'armed' : (v.connected ? 'connected' : 'disconnected'));
        }

        // Alt / speed
        setEl('split-alt-' + side, v.altRel != null ? v.altRel.toFixed(0) + 'm' : '--');
        setEl('split-spd-' + side, v.groundspeed != null ? v.groundspeed.toFixed(1) + ' m/s' : '--');

        // Battery
        var pct = v.batteryPct >= 0 ? v.batteryPct : 0;
        var voltStr = v.voltage > 0 ? v.voltage.toFixed(1) + 'V' : '--V';
        setEl('split-batt-text-' + side, voltStr);
        var fill = document.getElementById('split-batt-fill-' + side);
        if (fill) {
            fill.style.width = Math.max(0, Math.min(100, pct)) + '%';
            fill.className = 'split-batt-fill' +
                (pct <= 20 ? ' critical' : pct <= 40 ? ' low' : '');
        }
    }

    function setEl(id, text) {
        var el = document.getElementById(id);
        if (el) el.textContent = text;
    }

    // ---- Mini ADI Canvas ----

    function startMiniADILoop() {
        if (drawTimer) cancelAnimationFrame(drawTimer);
        function loop() {
            if (!active) return;
            drawMiniADIs();
            drawTimer = requestAnimationFrame(loop);
        }
        drawTimer = requestAnimationFrame(loop);
    }

    function drawMiniADIs() {
        var ids = Object.keys(meridian.vehicles).map(Number).sort();
        drawMiniADI('left', meridian.vehicles[ids[0]] || null);
        drawMiniADI('right', meridian.vehicles[ids[1]] || null);
    }

    function drawMiniADI(side, v) {
        var canvas = document.getElementById('split-adi-' + side);
        if (!canvas) return;
        var ctx = canvas.getContext('2d');
        var w = canvas.width;
        var h = canvas.height;
        var cx = w / 2;
        var cy = h / 2;
        var r = Math.min(w, h) / 2 - 2;

        ctx.clearRect(0, 0, w, h);

        var roll  = v ? (v.roll  || 0) : 0;   // radians
        var pitch = v ? (v.pitch || 0) : 0;   // radians

        // Clipping circle
        ctx.save();
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.clip();

        ctx.save();
        ctx.translate(cx, cy);
        ctx.rotate(-roll);

        // Sky
        var skyGrad = ctx.createLinearGradient(0, -r, 0, 0);
        skyGrad.addColorStop(0,   '#1a3a5c');
        skyGrad.addColorStop(1,   '#1e5f8a');
        ctx.fillStyle = skyGrad;
        ctx.fillRect(-r, -r - pitch * r * 0.8, r * 2, r + pitch * r * 0.8);

        // Ground
        var gndGrad = ctx.createLinearGradient(0, 0, 0, r);
        gndGrad.addColorStop(0,   '#5c3a1a');
        gndGrad.addColorStop(1,   '#3a2008');
        ctx.fillStyle = gndGrad;
        ctx.fillRect(-r, -pitch * r * 0.8, r * 2, r * 2);

        // Horizon line
        ctx.strokeStyle = 'rgba(255,255,255,0.5)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(-r, -pitch * r * 0.8);
        ctx.lineTo( r, -pitch * r * 0.8);
        ctx.stroke();

        ctx.restore();

        // Fixed aircraft crosshair
        ctx.strokeStyle = '#00e5ff';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(cx - 18, cy);
        ctx.lineTo(cx - 6, cy);
        ctx.moveTo(cx + 6, cy);
        ctx.lineTo(cx + 18, cy);
        ctx.moveTo(cx, cy - 4);
        ctx.lineTo(cx, cy + 4);
        ctx.stroke();

        ctx.restore();

        // Bezel ring
        ctx.strokeStyle = 'rgba(0,229,255,0.25)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.stroke();

        // No-vehicle overlay
        if (!v || !v.connected) {
            ctx.fillStyle = 'rgba(8,11,16,0.7)';
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.fill();
            ctx.fillStyle = 'rgba(255,255,255,0.4)';
            ctx.font = '9px monospace';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText('NO LINK', cx, cy);
        }
    }

    return {
        init: init,
        toggle: toggle,
        get active() { return active; },
    };

})();
