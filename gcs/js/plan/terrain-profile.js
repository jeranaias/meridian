/* ============================================================
   terrain-profile.js — Mission altitude profile chart
   Canvas-drawn elevation profile showing:
   - Mission altitude (blue line)
   - Ground elevation (brown fill) — uses demo data or SRTM
   - Clearance warnings (red zones where clearance < threshold)
   ============================================================ */

'use strict';

window.TerrainProfile = (function () {

    let canvas, ctx, container;
    const MIN_CLEARANCE = 15; // meters — warn if below this

    function init(parentEl) {
        container = document.createElement('div');
        container.className = 'terrain-profile';

        const header = document.createElement('div');
        header.className = 'terrain-profile-header';
        header.textContent = 'Altitude Profile';
        container.appendChild(header);

        canvas = document.createElement('canvas');
        canvas.className = 'terrain-canvas';
        canvas.width = 600;
        canvas.height = 100;
        canvas.style.cssText = 'width:100%;height:80px;display:block;border-radius:var(--r-sm)';
        container.appendChild(canvas);
        ctx = canvas.getContext('2d');

        parentEl.appendChild(container);

        meridian.events.on('mission_change', render);
        render();
    }

    function render() {
        if (!canvas || !ctx) return;
        const items = Mission.getItems ? Mission.getItems() : [];
        const dpr = window.devicePixelRatio || 1;
        const w = canvas.clientWidth * dpr;
        const h = canvas.clientHeight * dpr;
        canvas.width = w;
        canvas.height = h;
        ctx.clearRect(0, 0, w, h);

        if (items.length < 2) {
            ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--c-neutral-dim').trim() || '#666';
            ctx.font = (11 * dpr) + 'px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('Add waypoints to see altitude profile', w / 2, h / 2);
            return;
        }

        // Collect altitudes and compute cumulative distances
        const points = [];
        let totalDist = 0;
        for (let i = 0; i < items.length; i++) {
            const item = items[i];
            const alt = item.alt || item.p7 || 0;
            if (i > 0 && items[i-1].lat && item.lat) {
                totalDist += haversine(items[i-1].lat, items[i-1].lon, item.lat, item.lon);
            }
            // Simulated ground elevation (rolling hills for demo)
            const groundAlt = 20 + 10 * Math.sin(totalDist / 200) + 5 * Math.cos(totalDist / 80);
            points.push({ dist: totalDist, alt: alt, ground: groundAlt, seq: i });
        }

        if (totalDist === 0) return;

        // Scale
        const maxAlt = Math.max(...points.map(p => Math.max(p.alt, p.ground))) * 1.15;
        const minAlt = 0;
        const range = maxAlt - minAlt || 1;
        const pad = 4 * dpr;

        const xScale = (w - pad * 2) / totalDist;
        const yScale = (h - pad * 2) / range;
        const toX = d => pad + d * xScale;
        const toY = a => h - pad - (a - minAlt) * yScale;

        // Draw ground fill
        ctx.beginPath();
        ctx.moveTo(toX(0), h);
        points.forEach(p => ctx.lineTo(toX(p.dist), toY(p.ground)));
        ctx.lineTo(toX(totalDist), h);
        ctx.closePath();
        ctx.fillStyle = 'rgba(146, 64, 14, 0.3)';
        ctx.fill();

        // Draw ground line
        ctx.beginPath();
        points.forEach((p, i) => i === 0 ? ctx.moveTo(toX(p.dist), toY(p.ground)) : ctx.lineTo(toX(p.dist), toY(p.ground)));
        ctx.strokeStyle = 'rgba(146, 64, 14, 0.6)';
        ctx.lineWidth = 1.5 * dpr;
        ctx.stroke();

        // Draw clearance warning zones
        for (let i = 0; i < points.length - 1; i++) {
            const p1 = points[i], p2 = points[i + 1];
            const clearance1 = p1.alt - p1.ground;
            const clearance2 = p2.alt - p2.ground;
            if (clearance1 < MIN_CLEARANCE || clearance2 < MIN_CLEARANCE) {
                ctx.fillStyle = 'rgba(220, 38, 38, 0.15)';
                ctx.fillRect(toX(p1.dist), 0, toX(p2.dist) - toX(p1.dist), h);
            }
        }

        // Draw mission altitude line
        ctx.beginPath();
        points.forEach((p, i) => i === 0 ? ctx.moveTo(toX(p.dist), toY(p.alt)) : ctx.lineTo(toX(p.dist), toY(p.alt)));
        const primaryColor = getComputedStyle(document.documentElement).getPropertyValue('--c-primary').trim() || '#0891b2';
        ctx.strokeStyle = primaryColor;
        ctx.lineWidth = 2 * dpr;
        ctx.stroke();

        // Draw waypoint dots
        points.forEach(p => {
            ctx.beginPath();
            ctx.arc(toX(p.dist), toY(p.alt), 3 * dpr, 0, Math.PI * 2);
            ctx.fillStyle = primaryColor;
            ctx.fill();
        });

        // Draw altitude labels at start/end
        ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--c-text').trim() || '#333';
        ctx.font = 'bold ' + (10 * dpr) + 'px sans-serif';
        ctx.textAlign = 'left';
        if (points[0]) ctx.fillText(Math.round(points[0].alt) + 'm', toX(0) + 4 * dpr, toY(points[0].alt) - 4 * dpr);
        ctx.textAlign = 'right';
        const last = points[points.length - 1];
        if (last) ctx.fillText(Math.round(last.alt) + 'm', toX(last.dist) - 4 * dpr, toY(last.alt) - 4 * dpr);
    }

    function haversine(lat1, lon1, lat2, lon2) {
        const R = 6371000;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(dLat / 2) ** 2 +
                  Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                  Math.sin(dLon / 2) ** 2;
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    return { init, render };

})();
