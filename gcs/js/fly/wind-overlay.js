/* ============================================================
   wind-overlay.js — Wind Estimation and Map Overlay
   T3-5: Estimates wind from telemetry (groundspeed vector vs
   airspeed*heading vector), displays wind arrow on map,
   shows readout widget.
   ============================================================ */

'use strict';

window.WindOverlay = (function () {

    // --- State ---
    let windArrowMarker = null;
    let readoutEl = null;
    let lastEstimate = null; // { speed, fromDeg }
    let updateTimer = null;

    // --- Wind speed color thresholds ---
    function windColor(speed) {
        if (speed < 5) return '#22c55e';   // green
        if (speed < 10) return '#f59e0b';  // amber
        return '#ef4444';                   // red
    }

    // --- Build the wind arrow SVG ---
    // Arrow points FROM the wind source direction (i.e. points into the wind)
    function makeArrowSvg(fromDeg, speed) {
        const color = windColor(speed);
        // Arrow is drawn pointing up (north). Rotate so it points FROM fromDeg
        // (i.e. wind coming from 270° means arrow points left/west)
        const rot = fromDeg; // rotate by from-direction so arrow tip points into wind source
        return `<svg viewBox="0 0 40 60" xmlns="http://www.w3.org/2000/svg"
                     style="transform:rotate(${rot}deg);display:block;overflow:visible">
            <!-- Shaft -->
            <line x1="20" y1="54" x2="20" y2="10" stroke="${color}" stroke-width="3" stroke-linecap="round"/>
            <!-- Arrowhead pointing up (toward source) -->
            <polygon points="20,4 13,18 27,18" fill="${color}"/>
            <!-- Speed barbs (each barb = ~2 m/s) -->
            ${makeBarbs(speed, color)}
        </svg>`;
    }

    function makeBarbs(speed, color) {
        // Simple tick marks on the shaft indicating speed
        const barbs = [];
        const fullBarbs = Math.floor(speed / 2.5);
        for (let i = 0; i < Math.min(fullBarbs, 6); i++) {
            const y = 44 - i * 6;
            barbs.push(`<line x1="20" y1="${y}" x2="30" y2="${y - 4}" stroke="${color}" stroke-width="1.8" stroke-linecap="round"/>`);
        }
        return barbs.join('');
    }

    function makeWindIcon(fromDeg, speed) {
        return L.divIcon({
            className: 'wind-arrow-icon',
            html: makeArrowSvg(fromDeg, speed),
            iconSize: [40, 60],
            iconAnchor: [20, 30],
        });
    }

    // --- Wind estimation from telemetry ---
    // Wind = groundspeed_vector - airspeed_vector
    // groundspeed vector: (vx, vy) from VFR_HUD or GPS_RAW_INT
    // airspeed vector: airspeed * (sin(heading), cos(heading))
    function estimateWind(v) {
        // Wind = groundspeed_vector - airspeed_vector
        const as = v.airspeed || 0;
        const hdgRad = (v.heading || v.hdg || 0) * Math.PI / 180;

        // Ground velocity vector: use vx/vy directly from telemetry
        // (vx=east, vy=north in our convention)
        let gvx = v.vx || 0;
        let gvy = v.vy || 0;

        // Only reconstruct from gs+heading if no velocity components AND no airspeed
        // (when vx/vy are set, they already include wind drift — that's the signal)
        if (gvx === 0 && gvy === 0 && (v.groundspeed || 0) > 0.5) {
            // Fallback: can't estimate wind without separate airspeed vs ground vectors
            return { speed: 0, fromDeg: 0 };
        }

        // Airspeed vector aligned to heading (the direction the vehicle is pointed)
        const avx = as * Math.sin(hdgRad);
        const avy = as * Math.cos(hdgRad);

        // Wind vector (wind speed + direction)
        const wx = gvx - avx;
        const wy = gvy - avy;

        const windSpeed = Math.sqrt(wx * wx + wy * wy);

        // Wind FROM direction (meteorological convention: where wind comes from)
        // Wind vector points in the direction the air is moving TO.
        // "From" direction = opposite of "to" direction.
        let windTo = Math.atan2(wx, wy) * 180 / Math.PI; // degrees from north
        windTo = ((windTo % 360) + 360) % 360;
        const windFrom = ((windTo + 180) % 360);

        return { speed: windSpeed, fromDeg: windFrom };
    }

    // --- Check if vehicle is flying (armed + alt > 2m) ---
    function isFlying(v) {
        return v && v.armed && v.relativeAlt > 2;
    }

    // --- Get map from FlyMap ---
    function getMap() {
        return window.FlyMap ? FlyMap.getMap() : null;
    }

    // --- Update wind overlay ---
    function update() {
        const v = meridian.v;
        const map = getMap();
        if (!map) return;

        if (!isFlying(v)) {
            // Remove overlay when not flying
            if (windArrowMarker) {
                map.removeLayer(windArrowMarker);
                windArrowMarker = null;
            }
            if (readoutEl) readoutEl.style.display = 'none';
            return;
        }

        const est = estimateWind(v);
        lastEstimate = est;

        // Wind arrow on map removed — too cluttered.
        // Wind info shown in the readout widget only.
        if (windArrowMarker) {
            map.removeLayer(windArrowMarker);
            windArrowMarker = null;
        }

        updateReadout(est);
    }

    // --- Wind readout widget ---
    function createReadout() {
        if (readoutEl) return;
        readoutEl = document.createElement('div');
        readoutEl.id = 'wind-readout';
        readoutEl.style.cssText = [
            'position:absolute',
            'bottom:12px',
            'right:8px',
            'z-index:1000',
            'background:var(--c-bg-hud, rgba(8,11,16,0.82))',
            'border:1px solid var(--c-border)',
            'border-radius:var(--r-sm, 4px)',
            'padding:6px 10px',
            'font-family:var(--f-mono, "DM Mono", Consolas, monospace)',
            'font-size:11px',
            'color:var(--c-neutral)',
            'pointer-events:none',
            'display:none',
            'min-width:130px',
        ].join(';');
        const mapArea = document.getElementById('map-area');
        if (mapArea) mapArea.appendChild(readoutEl);
    }

    function updateReadout(est) {
        if (!readoutEl) return;
        const color = windColor(est.speed);
        readoutEl.style.display = 'block';
        readoutEl.innerHTML =
            `<span style="color:${color};font-weight:600">WIND</span> ` +
            `<span style="color:var(--c-text)">${est.speed.toFixed(1)} m/s</span> ` +
            `FROM <span style="color:var(--c-text)">${Math.round(est.fromDeg)}\u00B0</span>`;
    }

    // --- Public: get current estimate (for quick widget) ---
    function getCurrentEstimate() {
        return lastEstimate;
    }

    function init() {
        createReadout();
        // Update on telemetry events (position carries vx/vy)
        meridian.events.on('position', function () {
            update();
        });
        // Also poll at 2Hz as fallback
        updateTimer = setInterval(update, 500);
    }

    function destroy() {
        if (updateTimer) { clearInterval(updateTimer); updateTimer = null; }
        const map = getMap();
        if (map && windArrowMarker) { map.removeLayer(windArrowMarker); windArrowMarker = null; }
        if (readoutEl) readoutEl.style.display = 'none';
    }

    return { init, destroy, getCurrentEstimate };

})();
