/* ============================================================
   compass.js — Horizontal compass strip
   ============================================================ */

'use strict';

window.Compass = (function () {

    let canvas, ctx;
    let w, h;

    function TICK_COLOR()  { return (window._themeColors || {}).compassTick  || 'rgba(30,41,59,0.2)'; }
    function MAJOR_COLOR() { return (window._themeColors || {}).compassMajor || 'rgba(30,41,59,0.4)'; }
    function TEXT_COLOR()  { return (window._themeColors || {}).compassText  || '#1e293b'; }
    function POINTER_COLOR() { return (window._themeColors || {}).compassPtr || '#0891b2'; }
    const CARDINAL = { 0: 'N', 45: 'NE', 90: 'E', 135: 'SE', 180: 'S', 225: 'SW', 270: 'W', 315: 'NW' };

    function init(canvasEl) {
        canvas = canvasEl;
        ctx = canvas.getContext('2d');
        resize();
        window.addEventListener('resize', resize);
    }

    function resize() {
        const rect = canvas.parentElement.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        w = rect.width;
        h = rect.height;
        canvas.width = w * dpr;
        canvas.height = h * dpr;
        canvas.style.width = w + 'px';
        canvas.style.height = h + 'px';
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    function draw(heading) {
        if (!ctx) return;

        ctx.clearRect(0, 0, w, h);
        const cx = w / 2;
        const ppd = w / 120; // pixels per degree (show ±60°)

        // Background
        ctx.fillStyle = (window._themeColors || {}).compassBg || '#f4f5f7';
        ctx.fillRect(0, 0, w, h);

        // Draw tick marks
        for (let deg = -180; deg <= 540; deg += 5) {
            const offset = ((deg - heading + 540) % 360 - 180) * ppd;
            const x = cx + offset;

            if (x < -10 || x > w + 10) continue;

            const normDeg = ((deg % 360) + 360) % 360;
            const isMajor = normDeg % 30 === 0;
            const isCardinal = CARDINAL[normDeg] !== undefined;

            if (isMajor || normDeg % 10 === 0) {
                const tickH = isMajor ? 12 : 7;
                ctx.strokeStyle = isMajor ? MAJOR_COLOR() : TICK_COLOR();
                ctx.lineWidth = isMajor ? 1.2 : 0.7;
                ctx.beginPath();
                ctx.moveTo(x, h);
                ctx.lineTo(x, h - tickH);
                ctx.stroke();
            }

            if (isCardinal) {
                ctx.font = '600 11px "Rajdhani", sans-serif';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'top';
                ctx.fillStyle = normDeg === 0 ? ((window._themeColors || {}).compassN || '#e65100') : TEXT_COLOR();
                ctx.fillText(CARDINAL[normDeg], x, 3);
            } else if (isMajor) {
                ctx.font = '500 10px "DM Mono", monospace';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'top';
                ctx.fillStyle = (window._themeColors || {}).compassDeg || 'rgba(30,41,59,0.4)';
                ctx.fillText(normDeg.toString(), x, 5);
            }
        }

        // Center pointer
        ctx.fillStyle = POINTER_COLOR();
        ctx.beginPath();
        ctx.moveTo(cx, h);
        ctx.lineTo(cx - 4, h - 6);
        ctx.lineTo(cx + 4, h - 6);
        ctx.closePath();
        ctx.fill();

        // Heading readout — bold and prominent
        ctx.font = '700 12px "DM Mono", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';
        ctx.fillStyle = POINTER_COLOR();
        ctx.fillText(Math.round(heading) + '\u00b0', cx, h - 5);
    }

    return { init, draw, resize };

})();
