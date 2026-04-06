/* ============================================================
   stats.js — Plan stats bar (bottom of plan panel)
   Shows: WP count, total distance, est flight time,
   max altitude, max distance from home.
   Updates on every mission change.
   ============================================================ */

'use strict';

window.PlanStats = (function () {

    var container = null;
    var warningsEl = null;

    // Haversine distance between two lat/lon points (meters)
    function haversine(lat1, lon1, lat2, lon2) {
        var R = 6371000;
        var dLat = (lat2 - lat1) * Math.PI / 180;
        var dLon = (lon2 - lon1) * Math.PI / 180;
        var a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                Math.sin(dLon / 2) * Math.sin(dLon / 2);
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    function init(el) {
        container = el;
        meridian.events.on('mission_change', compute);
        compute();
    }

    function compute() {
        if (!container) return;

        var items = Mission.getItems();
        var wpCount = items.length;

        if (wpCount === 0) {
            container.innerHTML = '<div class="plan-stats-row">' +
                '<div class="plan-stat"><span class="plan-stat-label">WPs</span><span class="plan-stat-value">0</span></div>' +
                '</div>';
            return;
        }

        // Calculate stats
        var totalDist = 0;
        var maxAlt = 0;
        var maxDistHome = 0;
        var v = meridian.v;
        var homeLat = (v && v.homeLat) ? v.homeLat : (items[0] ? items[0].lat : 0);
        var homeLon = (v && v.homeLon) ? v.homeLon : (items[0] ? items[0].lon : 0);

        for (var i = 0; i < items.length; i++) {
            var it = items[i];

            // Distance between consecutive waypoints
            if (i > 0) {
                var prev = items[i - 1];
                totalDist += haversine(prev.lat, prev.lon, it.lat, it.lon);
            }

            // Max altitude
            if (it.alt > maxAlt) maxAlt = it.alt;

            // Max distance from home
            if (homeLat && it.lat) {
                var dHome = haversine(homeLat, homeLon, it.lat, it.lon);
                if (dHome > maxDistHome) maxDistHome = dHome;
            }
        }

        // T2-17: Use WPNAV_SPEED param (cm/s → m/s) if available, else fall back to 5.0 m/s
        var avgSpeed = 5.0;
        if (v && v.params && v.params.WPNAV_SPEED && v.params.WPNAV_SPEED > 0) {
            avgSpeed = v.params.WPNAV_SPEED / 100;
        }
        var estSeconds = totalDist / avgSpeed;
        var estMin = Math.floor(estSeconds / 60);
        var estSec = Math.floor(estSeconds % 60);
        var estTime = estMin + ':' + (estSec < 10 ? '0' : '') + estSec;

        // Battery endurance estimate
        var battCapacity = (v && v.params && v.params.BATT_CAPACITY) || 5200; // mAh
        var avgCurrent = (v && v.current > 0) ? v.current : 15; // Amps, fallback 15A for copter
        var enduranceMin = (battCapacity / 1000) / avgCurrent * 60; // minutes
        var battMargin = enduranceMin - (estSeconds / 60);
        var battWarning = battMargin < 3; // less than 3 min margin

        var html = '<div class="plan-stats-row">';
        html += stat('WPs', '' + wpCount);
        html += stat('Dist', (totalDist / 1000).toFixed(2) + ' km');
        html += stat('Est Time', estTime);
        html += stat('Max Alt', maxAlt.toFixed(0) + ' m');
        html += stat('Max Dist', maxDistHome.toFixed(0) + ' m');
        html += stat('Batt Est', Math.floor(enduranceMin) + ' min');
        if (battWarning) {
            html += stat('\u26A0 Margin', battMargin.toFixed(1) + ' min');
        }
        html += '</div>';

        // Warnings container
        html += '<div class="plan-warnings" id="plan-warnings"></div>';

        container.innerHTML = html;
        warningsEl = document.getElementById('plan-warnings');

        // Run validator
        PlanValidator.validate();
    }

    function stat(label, value) {
        return '<div class="plan-stat">' +
            '<span class="plan-stat-label">' + label + '</span>' +
            '<span class="plan-stat-value">' + value + '</span>' +
            '</div>';
    }

    function getWarningsEl() {
        return warningsEl;
    }

    // Expose haversine for use by other modules
    return { init: init, compute: compute, haversine: haversine, getWarningsEl: getWarningsEl };

})();
