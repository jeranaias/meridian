/* ============================================================
   validator.js — Real-time mission validator
   Runs on every mission change. Checks:
   - Consecutive WPs too close (<1m)
   - Altitude below 0
   - Altitude above guidedAltMax
   - Distance from home exceeds guidedDistMax
   Shows warnings inline in the stats panel.
   ============================================================ */

'use strict';

window.PlanValidator = (function () {

    function validate() {
        var el = PlanStats.getWarningsEl();
        if (!el) return;

        var items = Mission.getItems();
        var warnings = [];

        if (items.length === 0) {
            el.innerHTML = '';
            return;
        }

        var v = meridian.v;
        var homeLat = (v && v.homeLat) ? v.homeLat : (items[0] ? items[0].lat : 0);
        var homeLon = (v && v.homeLon) ? v.homeLon : (items[0] ? items[0].lon : 0);
        var altMax = meridian.settings.guidedAltMax || 120;
        var distMax = meridian.settings.guidedDistMax || 1000;

        for (var i = 0; i < items.length; i++) {
            var it = items[i];

            // Altitude below 0
            if (it.alt < 0) {
                warnings.push({ type: 'error', text: 'WP ' + (i + 1) + ': altitude below 0 (' + it.alt.toFixed(1) + 'm)' });
            }

            // Altitude above max
            if (it.alt > altMax) {
                warnings.push({ type: 'warn', text: 'WP ' + (i + 1) + ': altitude ' + it.alt.toFixed(0) + 'm exceeds limit (' + altMax + 'm)' });
            }

            // Distance from home exceeds max
            if (homeLat && it.lat) {
                var dHome = PlanStats.haversine(homeLat, homeLon, it.lat, it.lon);
                if (dHome > distMax) {
                    warnings.push({ type: 'warn', text: 'WP ' + (i + 1) + ': ' + Math.round(dHome) + 'm from home (limit ' + distMax + 'm)' });
                }
            }

            // Consecutive waypoints too close
            if (i > 0) {
                var prev = items[i - 1];
                var d = PlanStats.haversine(prev.lat, prev.lon, it.lat, it.lon);
                if (d < 1 && it.command !== Mission.CMD.DO_SET_ROI) {
                    warnings.push({ type: 'warn', text: 'WP ' + i + ' and ' + (i + 1) + ' are < 1m apart' });
                }
                // Duplicate position
                if (d < 0.1 && Math.abs(it.alt - prev.alt) < 0.1) {
                    warnings.push({ type: 'warn', text: 'WP ' + i + ' and ' + (i + 1) + ' are duplicates' });
                }
            }
        }

        // Battery endurance check
        if (items.length >= 2) {
            var totalDist = 0;
            for (var j = 1; j < items.length; j++) {
                totalDist += PlanStats.haversine(items[j-1].lat, items[j-1].lon, items[j].lat, items[j].lon);
            }
            var speed = 5.0;
            if (v && v.params && v.params.WPNAV_SPEED > 0) speed = v.params.WPNAV_SPEED / 100;
            var flightMinutes = (totalDist / speed) / 60;
            var battCap = (v && v.params && v.params.BATT_CAPACITY) || 5200;
            var avgCurr = (v && v.current > 0) ? v.current : 15;
            var endurance = (battCap / 1000) / avgCurr * 60;
            if (flightMinutes > endurance * 0.85) {
                warnings.push({ type: 'error', text: 'Mission time (' + flightMinutes.toFixed(1) + 'min) may exceed battery (' + endurance.toFixed(0) + 'min)' });
            } else if (flightMinutes > endurance * 0.7) {
                warnings.push({ type: 'warn', text: 'Mission uses ' + Math.round(flightMinutes / endurance * 100) + '% of estimated battery' });
            }
        }

        // First WP should be takeoff
        if (items.length > 0 && items[0].command !== 22 && items[0].alt > 0) {
            warnings.push({ type: 'warn', text: 'First waypoint is not a TAKEOFF command' });
        }

        if (warnings.length === 0) {
            el.innerHTML = '';
            return;
        }

        var html = '';
        for (var w = 0; w < warnings.length; w++) {
            var cls = warnings[w].type === 'error' ? ' error' : '';
            html += '<div class="plan-warn-item' + cls + '">';
            html += '<span class="plan-warn-icon">\u26A0</span> ';
            html += warnings[w].text;
            html += '</div>';
        }
        el.innerHTML = html;
    }

    return { validate: validate };

})();
