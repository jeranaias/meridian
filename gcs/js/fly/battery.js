/* ============================================================
   battery.js — Tiered battery widget
   Tufte: % + bar always visible, details on hover,
   per-cell only on variance warning.
   ============================================================ */

'use strict';

window.Battery = (function () {

    let el;

    function init(container) {
        el = container;
        render();
    }

    function render() {
        if (!el) return;
        el.innerHTML = `
            <div class="battery-icon"></div>
            <div class="battery-bar-track">
                <div class="battery-bar-fill" id="batt-fill"></div>
            </div>
            <div class="battery-pct" id="batt-pct">--%</div>
            <div class="battery-detail" id="batt-detail">
                <div class="battery-detail-item">
                    <div class="battery-detail-value" id="batt-volt">--V</div>
                    <div class="battery-detail-label">Voltage</div>
                </div>
                <div class="battery-detail-item">
                    <div class="battery-detail-value" id="batt-curr">--A</div>
                    <div class="battery-detail-label">Current</div>
                </div>
                <div class="battery-detail-item">
                    <div class="battery-detail-value" id="batt-mah">--mAh</div>
                    <div class="battery-detail-label">Used</div>
                </div>
                <div class="battery-detail-item">
                    <div class="battery-detail-value" id="batt-eta" style="color:var(--c-primary)">--</div>
                    <div class="battery-detail-label">Time Left</div>
                </div>
                <div class="battery-detail-item">
                    <div class="battery-detail-value" id="batt-rate">--</div>
                    <div class="battery-detail-label">mAh/min</div>
                </div>
            </div>
        `;
    }

    function update(v) {
        if (!el) return;
        const pct = v.batteryPct >= 0 ? v.batteryPct : 0;
        const fill = el.querySelector('#batt-fill');
        const pctEl = el.querySelector('#batt-pct');
        const voltEl = el.querySelector('#batt-volt');
        const currEl = el.querySelector('#batt-curr');
        const mahEl = el.querySelector('#batt-mah');

        if (!fill) return;

        // Bar width
        fill.style.width = Math.max(0, Math.min(100, pct)) + '%';

        // Color classes
        fill.className = 'battery-bar-fill';
        el.className = 'battery-widget';

        if (pct <= 15) {
            fill.classList.add('critical');
            el.classList.add('critical');
        } else if (pct <= 30) {
            fill.classList.add('warning');
            el.classList.add('warning');
        }

        // Percentage text
        pctEl.textContent = v.batteryPct >= 0 ? Math.round(pct) + '%' : '--%';

        // Detail values
        voltEl.textContent = v.voltage > 0 ? v.voltage.toFixed(1) + 'V' : '--V';
        currEl.textContent = v.current > 0 ? v.current.toFixed(1) + 'A' : '--A';
        mahEl.textContent = v.mah > 0 ? v.mah.toFixed(0) + 'mAh' : '--mAh';

        // Time remaining estimate
        const etaEl = el.querySelector('#batt-eta');
        const rateEl = el.querySelector('#batt-rate');
        if (etaEl && v.current > 0.5 && v.batteryPct > 0) {
            // Compute mAh/min from current draw (current is in Amps)
            const mahPerMin = v.current * 1000 / 60; // mA * 60s = mAh/min
            if (rateEl) rateEl.textContent = mahPerMin.toFixed(0);

            // Remaining capacity estimate
            const capacity = (v.params && v.params.BATT_CAPACITY) || 5200;
            const remaining = capacity - (v.mah || 0);
            if (remaining > 0 && mahPerMin > 0) {
                const minsLeft = remaining / mahPerMin;
                const m = Math.floor(minsLeft);
                const s = Math.round((minsLeft - m) * 60);
                etaEl.textContent = m + ':' + String(s).padStart(2, '0');
                if (minsLeft < 3) etaEl.style.color = 'var(--c-emergency)';
                else if (minsLeft < 5) etaEl.style.color = 'var(--c-warning)';
                else etaEl.style.color = 'var(--c-primary)';
            }
        }
    }

    return { init, update };

})();
