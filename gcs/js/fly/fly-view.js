/* ============================================================
   fly-view.js — Fly View orchestrator
   Initializes all instruments, wires to event bus,
   runs 10Hz UI refresh loop.
   ============================================================ */

'use strict';

window.FlyView = (function () {

    let animFrame;
    let lastDraw = 0;
    let lastStateHash = '';
    const DRAW_INTERVAL = 33; // 30Hz

    function init() {
        // Initialize canvas instruments
        const adiCanvas = document.getElementById('adi-canvas');
        const compassCanvas = document.getElementById('compass-canvas');
        const altCanvas = document.getElementById('alt-canvas');
        const spdCanvas = document.getElementById('spd-canvas');

        if (adiCanvas) ADI.init(adiCanvas);
        if (compassCanvas) Compass.init(compassCanvas);
        if (altCanvas) Tapes.initAlt(altCanvas);
        if (spdCanvas) Tapes.initSpd(spdCanvas);

        // Initialize battery
        const battEl = document.getElementById('battery-widget');
        if (battEl) Battery.init(battEl);

        // Initialize health grid
        const healthEl = document.getElementById('health-grid');
        if (healthEl) HealthStatus.init(healthEl);

        // Initialize quick widget
        const quickEl = document.getElementById('quick-widget');
        if (quickEl) QuickWidget.init(quickEl);

        // Force full redraw on theme change (canvas colors need refresh)
        meridian.events.on('theme_change', function () {
            lastStateHash = ''; // invalidate hash to force redraw
        });

        // Start render loop
        startRenderLoop();
    }

    function startRenderLoop() {
        function loop(ts) {
            animFrame = requestAnimationFrame(loop);

            if (ts - lastDraw < DRAW_INTERVAL) return;
            lastDraw = ts;

            draw();
        }
        animFrame = requestAnimationFrame(loop);
    }

    // T2-4: Toggle .stale on instrument containers based on telemetry timestamps
    function updateStaleness(v) {
        const now = Date.now();
        const STALE_MS = 2000;

        function setStale(selector, isStale) {
            const el = document.querySelector(selector);
            if (el) el.classList.toggle('stale', isStale);
        }

        // ADI + compass use attitude timestamp
        const attStale = (v.lastAttitude > 0) && (now - v.lastAttitude > STALE_MS);
        setStale('.adi-container', attStale);
        setStale('.compass-container', attStale);

        // Tapes (speed/alt) use position timestamp
        const posStale = (v.lastPosition > 0) && (now - v.lastPosition > STALE_MS);
        setStale('.tape-speed', posStale);
        setStale('.tape-alt', posStale);

        // Battery widget uses battery event (approximate via v.lastHeartbeat since
        // battery events don't have their own timestamp; use a dedicated lastBattery
        // if available, else fall back gracefully)
        const battStale = (v.lastBattery > 0) && (now - v.lastBattery > STALE_MS);
        setStale('#battery-widget', battStale);
    }

    function draw() {
        const v = meridian.v;
        if (!v) return;

        // Dirty check — skip if nothing changed
        const hash = '' + v.roll.toFixed(3) + v.pitch.toFixed(3) + v.heading.toFixed(1) +
                     v.relativeAlt.toFixed(1) + v.groundspeed.toFixed(1) + v.batteryPct;
        if (hash === lastStateHash) return;
        lastStateHash = hash;

        // ADI
        ADI.draw(v.roll, v.pitch, v.targetRoll, v.targetPitch);

        // Compass
        Compass.draw(v.heading);

        // Tapes
        Tapes.drawAlt(v.relativeAlt, v.targetAlt, v.climb, null);
        Tapes.drawSpd(v.groundspeed, v.targetSpeed, null);

        // Battery
        Battery.update(v);

        // Health
        HealthStatus.update(v);

        // Quick
        QuickWidget.update(v);

        // T2-4: Per-instrument staleness indicators
        updateStaleness(v);

        // Multi-vehicle secondary markers (throttle to ~2Hz)
        if (!draw._mvTick) draw._mvTick = 0;
        draw._mvTick++;
        if (draw._mvTick % 5 === 0 && FlyMap.updateSecondaryVehicles) {
            FlyMap.updateSecondaryVehicles();
        }
    }

    function destroy() {
        if (animFrame) cancelAnimationFrame(animFrame);
    }

    return { init, destroy };

})();
