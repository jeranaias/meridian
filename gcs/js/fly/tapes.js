/* ============================================================
   tapes.js — Altitude and Speed tape instruments
   Tufte: specified tick intervals, visible range, center pos.
   Victor: ghost markers at target/commanded values.
   ============================================================ */

'use strict';

window.Tapes = (function () {

    // ---- Cached canvas dimensions (resize only, not every draw) ----

    // Per-canvas dimension cache: keyed by canvas element
    var _canvasCache = new Map();

    function resizeCanvas(canvasEl) {
        const ctx = canvasEl.getContext('2d');
        const rect = canvasEl.parentElement.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        const w = rect.width;
        const h = rect.height;
        canvasEl.width = w * dpr;
        canvasEl.height = h * dpr;
        canvasEl.style.width = w + 'px';
        canvasEl.style.height = h + 'px';
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        var dims = { ctx: ctx, w: w, h: h };
        _canvasCache.set(canvasEl, dims);
        return dims;
    }

    function getCanvasDims(canvasEl) {
        return _canvasCache.get(canvasEl) || resizeCanvas(canvasEl);
    }

    function drawTape(canvasEl, value, target, opts) {
        var dims = getCanvasDims(canvasEl);
        var ctx = dims.ctx, w = dims.w, h = dims.h;
        if (!ctx) return;

        const {
            majorInterval,
            minorInterval,
            range,
            boxColor,
            ghostColor,
            unit,
            warnMin,
            warnMax,
            geofence,
            isAlt,
        } = opts;

        const cy = h / 2;
        const ppu = (h * 0.85) / (range * 2); // pixels per unit
        const tapeW = w;

        ctx.clearRect(0, 0, w, h);

        // Background
        ctx.fillStyle = (window._themeColors || {}).tapeBg || '#f4f5f7';
        ctx.fillRect(0, 0, w, h);

        // Stall/warning zone
        if (warnMin !== undefined && warnMin !== null) {
            const yWarn = cy + (value - warnMin) * ppu;
            if (yWarn < h) {
                ctx.fillStyle = 'rgba(255, 23, 68, 0.08)';
                ctx.fillRect(0, yWarn, tapeW, h - yWarn);
                ctx.strokeStyle = 'rgba(255, 23, 68, 0.3)';
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(0, yWarn);
                ctx.lineTo(tapeW, yWarn);
                ctx.stroke();
            }
        }

        // Geofence ceiling (red line)
        if (geofence !== undefined && geofence !== null) {
            const yFence = cy + (value - geofence) * ppu;
            if (yFence > 0 && yFence < h) {
                ctx.strokeStyle = '#ff1744';
                ctx.lineWidth = 1.5;
                ctx.setLineDash([4, 3]);
                ctx.beginPath();
                ctx.moveTo(0, yFence);
                ctx.lineTo(tapeW, yFence);
                ctx.stroke();
                ctx.setLineDash([]);
                // Label
                ctx.font = '500 10px "DM Mono", monospace';
                ctx.fillStyle = '#ff1744';
                ctx.textAlign = isAlt ? 'left' : 'right';
                ctx.fillText('FENCE', isAlt ? 4 : tapeW - 4, yFence - 3);
            }
        }

        // Tick marks and numbers
        let start = Math.floor((value - range) / minorInterval) * minorInterval;
        const end = Math.ceil((value + range) / minorInterval) * minorInterval;
        // Speed tape: never show below 0 (Tufte: speed is scalar)
        if (!isAlt && start < 0) start = 0;

        for (let v = start; v <= end; v += minorInterval) {
            const y = cy + (value - v) * ppu;
            if (y < -5 || y > h + 5) continue;

            const isMajor = Math.abs(v % majorInterval) < 0.01;

            // Tick
            const tickW = isMajor ? 16 : 8;
            ctx.strokeStyle = isMajor ? ((window._themeColors || {}).tapeTick || 'rgba(30,41,59,0.4)') : ((window._themeColors || {}).tapeTickMin || 'rgba(30,41,59,0.15)');
            ctx.lineWidth = isMajor ? 1 : 0.5;
            ctx.beginPath();
            if (isAlt) {
                ctx.moveTo(0, y);
                ctx.lineTo(tickW, y);
            } else {
                ctx.moveTo(tapeW, y);
                ctx.lineTo(tapeW - tickW, y);
            }
            ctx.stroke();

            // Number
            if (isMajor) {
                ctx.font = '500 11px "DM Mono", monospace';
                ctx.fillStyle = (window._themeColors || {}).tapeNum || 'rgba(30,41,59,0.6)';
                ctx.textBaseline = 'middle';
                if (isAlt) {
                    ctx.textAlign = 'left';
                    ctx.fillText(v.toFixed(0), tickW + 4, y);
                } else {
                    ctx.textAlign = 'right';
                    ctx.fillText(v.toFixed(v < 10 ? 1 : 0), tapeW - tickW - 4, y);
                }
            }
        }

        // Ghost target marker (Victor)
        if (target !== null && target !== undefined) {
            const yTarget = cy + (value - target) * ppu;
            if (yTarget > 0 && yTarget < h) {
                ctx.fillStyle = ghostColor || 'rgba(0, 229, 255, 0.5)';
                ctx.beginPath();
                if (isAlt) {
                    // Small triangle on left edge
                    ctx.moveTo(0, yTarget - 5);
                    ctx.lineTo(8, yTarget);
                    ctx.lineTo(0, yTarget + 5);
                } else {
                    // Small triangle on right edge
                    ctx.moveTo(tapeW, yTarget - 5);
                    ctx.lineTo(tapeW - 8, yTarget);
                    ctx.lineTo(tapeW, yTarget + 5);
                }
                ctx.closePath();
                ctx.fill();
            }
        }

        // Current value box
        const boxH = 22;
        const boxW = 56;
        ctx.fillStyle = boxColor;
        ctx.strokeStyle = boxColor;
        ctx.lineWidth = 1.5;

        const boxX = isAlt ? tapeW - boxW - 2 : 2;

        // Rounded rect
        const r = 3;
        ctx.beginPath();
        ctx.moveTo(boxX + r, cy - boxH / 2);
        ctx.lineTo(boxX + boxW - r, cy - boxH / 2);
        ctx.quadraticCurveTo(boxX + boxW, cy - boxH / 2, boxX + boxW, cy - boxH / 2 + r);
        ctx.lineTo(boxX + boxW, cy + boxH / 2 - r);
        ctx.quadraticCurveTo(boxX + boxW, cy + boxH / 2, boxX + boxW - r, cy + boxH / 2);
        ctx.lineTo(boxX + r, cy + boxH / 2);
        ctx.quadraticCurveTo(boxX, cy + boxH / 2, boxX, cy + boxH / 2 - r);
        ctx.lineTo(boxX, cy - boxH / 2 + r);
        ctx.quadraticCurveTo(boxX, cy - boxH / 2, boxX + r, cy - boxH / 2);
        ctx.closePath();

        ctx.fillStyle = (window._themeColors || {}).tapeBoxBg || '#ffffff';
        ctx.fill();
        ctx.stroke();

        // Value text
        ctx.font = '600 14px "DM Mono", monospace';
        ctx.fillStyle = boxColor;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        const fmt = value < 10 ? value.toFixed(1) : value.toFixed(0);
        ctx.fillText(fmt, boxX + boxW / 2, cy + 1);

        // Climb/speed trend arrow
        if (opts.trend) {
            const arrowLen = Math.min(Math.abs(opts.trend) * ppu * 2, 30);
            if (arrowLen > 3) {
                const arrowY = opts.trend > 0 ? cy - boxH / 2 - arrowLen : cy + boxH / 2 + arrowLen;
                ctx.strokeStyle = boxColor;
                ctx.lineWidth = 1.5;
                ctx.globalAlpha = 0.6;
                ctx.beginPath();
                ctx.moveTo(boxX + boxW / 2, opts.trend > 0 ? cy - boxH / 2 : cy + boxH / 2);
                ctx.lineTo(boxX + boxW / 2, arrowY);
                ctx.stroke();
                // Arrow head
                const dir = opts.trend > 0 ? -1 : 1;
                ctx.beginPath();
                ctx.moveTo(boxX + boxW / 2, arrowY);
                ctx.lineTo(boxX + boxW / 2 - 3, arrowY + dir * 5);
                ctx.moveTo(boxX + boxW / 2, arrowY);
                ctx.lineTo(boxX + boxW / 2 + 3, arrowY + dir * 5);
                ctx.stroke();
                ctx.globalAlpha = 1;
            }
        }
    }

    // ---- Altitude Tape ----

    let altCanvas;

    function initAlt(canvasEl) {
        altCanvas = canvasEl;
        resizeCanvas(canvasEl);
        window.addEventListener('resize', () => {
            resizeCanvas(canvasEl);
            drawAlt(lastAlt, lastAltTarget);
        });
    }

    let lastAlt = 0, lastAltTarget = null;

    function drawAlt(alt, target, climb, geofence) {
        lastAlt = alt;
        lastAltTarget = target;
        if (!altCanvas) return;
        drawTape(altCanvas, alt, target, {
            // Tufte: major every 10m below 100m, 50m above
            majorInterval: alt > 100 ? 50 : 10,
            minorInterval: alt > 100 ? 10 : 5,
            range: 50,
            boxColor: (window._themeColors || {}).altBox || '#16a34a',
            ghostColor: (window._themeColors || {}).altGhost || 'rgba(8,145,178,0.5)',
            unit: 'm',
            geofence: geofence,
            isAlt: true,
            trend: climb,
        });
    }

    // ---- Speed Tape ----

    let spdCanvas;

    function initSpd(canvasEl) {
        spdCanvas = canvasEl;
        resizeCanvas(canvasEl);
        window.addEventListener('resize', () => {
            resizeCanvas(canvasEl);
            drawSpd(lastSpd, lastSpdTarget);
        });
    }

    let lastSpd = 0, lastSpdTarget = null;

    function drawSpd(speed, target, stallSpeed) {
        speed = Math.max(0, speed); // Speed is scalar — never negative (Tufte)
        lastSpd = speed;
        lastSpdTarget = target;
        if (!spdCanvas) return;
        drawTape(spdCanvas, speed, target, {
            majorInterval: speed > 10 ? 5 : 1,
            minorInterval: speed > 10 ? 1 : 0.5,
            range: speed > 10 ? 25 : 5,
            boxColor: (window._themeColors || {}).spdBox || '#0891b2',
            ghostColor: (window._themeColors || {}).spdGhost || 'rgba(22,163,74,0.5)',
            unit: 'm/s',
            warnMin: stallSpeed,
            isAlt: false,
        });
    }

    return {
        initAlt, drawAlt,
        initSpd, drawSpd,
    };

})();
