/* ============================================================
   orbit.js — Orbit / circle mission tool (T2-10)
   Click map to set center, configure radius/altitude/laps/
   direction, generate waypoints in a circle pattern.
   ============================================================ */

'use strict';

window.OrbitTool = (function () {

    var _active = false;
    var _previewCircle = null;
    var _centerMarker = null;
    var _map = null;

    // Default parameters — overridden by the modal
    var DEFAULTS = {
        radius:     50,   // meters
        altitude:   30,   // meters AGL
        orbits:     1,    // number of full loops
        direction:  'CW', // 'CW' or 'CCW'
        roiCenter:  true, // prepend DO_SET_ROI at center
    };

    // ─── Public API ──────────────────────────────────────────

    function init() {
        meridian.events.on('panel_change', function (panel) {
            if (panel !== 'plan') deactivate();
        });
    }

    function activate() {
        if (_active) { deactivate(); return; }
        _active = true;
        _map = FlyMap.getMap();
        if (!_map) return;

        _map.getContainer().style.cursor = 'crosshair';
        _map.on('click', _onMapClick);

        var btn = document.getElementById('btn-orbit');
        if (btn) btn.classList.add('active');
    }

    function deactivate() {
        if (!_active) return;
        _active = false;

        if (_map) {
            _map.getContainer().style.cursor = '';
            _map.off('click', _onMapClick);
        }

        _removePreview();
        var btn = document.getElementById('btn-orbit');
        if (btn) btn.classList.remove('active');
    }

    // ─── Map click ───────────────────────────────────────────

    function _onMapClick(e) {
        var center = e.latlng;
        deactivate(); // exit draw mode, keep preview
        _showOrbitModal(center);
    }

    // ─── Modal ───────────────────────────────────────────────

    function _showOrbitModal(center) {
        // Remove previous preview
        _removePreview();

        // Build modal content
        var form = document.createElement('div');
        form.className = 'orbit-form';
        form.innerHTML =
            '<div class="orbit-field"><label>Radius (m)</label>' +
            '<input type="number" id="orbit-radius" value="' + DEFAULTS.radius + '" min="5" max="5000" step="1"></div>' +

            '<div class="orbit-field"><label>Altitude (m AGL)</label>' +
            '<input type="number" id="orbit-alt" value="' + DEFAULTS.altitude + '" min="5" max="500" step="1"></div>' +

            '<div class="orbit-field"><label>Number of orbits</label>' +
            '<input type="number" id="orbit-laps" value="' + DEFAULTS.orbits + '" min="1" max="20" step="1"></div>' +

            '<div class="orbit-field"><label>Direction</label>' +
            '<select id="orbit-dir">' +
            '<option value="CW">Clockwise (CW)</option>' +
            '<option value="CCW">Counter-clockwise (CCW)</option>' +
            '</select></div>' +

            '<div class="orbit-field orbit-toggle-row"><label>Set ROI at center</label>' +
            '<label class="settings-toggle">' +
            '<input type="checkbox" id="orbit-roi" ' + (DEFAULTS.roiCenter ? 'checked' : '') + '>' +
            '<span class="settings-toggle-slider"></span></label></div>' +

            '<div class="orbit-preview-info" id="orbit-preview-info"></div>';

        // Live preview on input change
        var updatePreview = function () {
            var r   = parseFloat(document.getElementById('orbit-radius').value) || DEFAULTS.radius;
            var alt = parseFloat(document.getElementById('orbit-alt').value)    || DEFAULTS.altitude;
            var laps = parseInt(document.getElementById('orbit-laps').value)    || DEFAULTS.orbits;
            var pts  = _wpCount(laps);
            var info = document.getElementById('orbit-preview-info');
            if (info) {
                info.textContent = pts + ' waypoints \u2022 r=' + r + 'm \u2022 alt=' + alt + 'm';
            }
            _drawPreview(center, r);
        };

        // Show via meridian modal helper
        var modal = _makeModal('Orbit Tool', form, function () {
            // OK
            var radius   = parseFloat(document.getElementById('orbit-radius').value) || DEFAULTS.radius;
            var altitude = parseFloat(document.getElementById('orbit-alt').value)    || DEFAULTS.altitude;
            var orbits   = parseInt(document.getElementById('orbit-laps').value)     || DEFAULTS.orbits;
            var dir      = document.getElementById('orbit-dir').value;
            var roi      = document.getElementById('orbit-roi').checked;

            // Persist defaults for next use
            DEFAULTS.radius   = radius;
            DEFAULTS.altitude = altitude;
            DEFAULTS.orbits   = orbits;
            DEFAULTS.direction = dir;
            DEFAULTS.roiCenter = roi;

            _generateOrbit(center, radius, altitude, orbits, dir, roi);
            _removePreview();
        }, function () {
            // Cancel
            _removePreview();
        });

        document.body.appendChild(modal);

        // Wire live preview after modal is in DOM
        setTimeout(function () {
            ['orbit-radius', 'orbit-alt', 'orbit-laps', 'orbit-dir'].forEach(function (id) {
                var el = document.getElementById(id);
                if (el) el.addEventListener('input', updatePreview);
            });
            updatePreview();
        }, 0);
    }

    // ─── Waypoint generation ─────────────────────────────────

    function _wpCount(orbits) {
        // 16 points per orbit gives smooth circle for typical radii
        return orbits * 16;
    }

    function _generateOrbit(center, radius, altitude, orbits, direction, addRoi) {
        if (!window.Mission) return;

        var waypoints = [];
        var totalPoints = _wpCount(orbits);
        var sign = (direction === 'CCW') ? -1 : 1;

        // Optional ROI at center (MAV_CMD_ROI = 201)
        if (addRoi) {
            waypoints.push({
                type: 'DO_SET_ROI',
                lat: center.lat,
                lng: center.lng,
                alt: 0,
                command: 201,
                frame: 3,   // MAV_FRAME_GLOBAL_RELATIVE_ALT
                param1: 0, param2: 0, param3: 0, param4: 0,
            });
        }

        // Circle waypoints — evenly spaced around circumference
        var earthR = 6378137; // meters
        for (var i = 0; i < totalPoints; i++) {
            var angle = (2 * Math.PI / totalPoints) * i * sign;
            var dLat = (radius * Math.cos(angle)) / earthR;
            var dLng = (radius * Math.sin(angle)) / (earthR * Math.cos(center.lat * Math.PI / 180));

            waypoints.push({
                type: 'WAYPOINT',
                lat:  center.lat + (dLat * 180 / Math.PI),
                lng:  center.lng + (dLng * 180 / Math.PI),
                alt:  altitude,
                command: 16,  // MAV_CMD_NAV_WAYPOINT
                frame: 3,
                param1: 0, param2: 0, param3: 0, param4: NaN,
            });
        }

        // Close the loop — return to first orbit wp (after optional ROI)
        var firstOrbitIdx = addRoi ? 1 : 0;
        if (waypoints[firstOrbitIdx]) {
            waypoints.push({
                type: 'WAYPOINT',
                lat:  waypoints[firstOrbitIdx].lat,
                lng:  waypoints[firstOrbitIdx].lng,
                alt:  altitude,
                command: 16,
                frame: 3,
                param1: 0, param2: 0, param3: 0, param4: NaN,
            });
        }

        // Append to existing mission
        var existing = Mission.getWaypoints ? Mission.getWaypoints() : [];
        var combined = existing.concat(waypoints);
        Mission.setWaypoints(combined);
        meridian.events.emit('mission_change');
        meridian.log('Orbit: ' + waypoints.length + ' waypoints added (r=' + radius + 'm)', 'info');
    }

    // ─── Map preview circle ───────────────────────────────────

    function _drawPreview(center, radius) {
        if (!_map) return;
        if (_previewCircle) _map.removeLayer(_previewCircle);
        _previewCircle = L.circle(center, {
            radius: radius,
            color: '#9333ea',
            weight: 2,
            dashArray: '6,4',
            fillOpacity: 0.07,
        }).addTo(_map);
        if (!_centerMarker) {
            _centerMarker = L.circleMarker(center, {
                radius: 5,
                color: '#9333ea',
                fillColor: '#9333ea',
                fillOpacity: 0.8,
                weight: 2,
            }).addTo(_map);
        } else {
            _centerMarker.setLatLng(center);
        }
    }

    function _removePreview() {
        if (!_map) return;
        if (_previewCircle) { _map.removeLayer(_previewCircle); _previewCircle = null; }
        if (_centerMarker) { _map.removeLayer(_centerMarker); _centerMarker = null; }
    }

    // ─── Minimal modal builder ────────────────────────────────

    function _makeModal(title, body, onOk, onCancel) {
        var overlay = document.createElement('div');
        overlay.className = 'orbit-modal-overlay';

        var dialog = document.createElement('div');
        dialog.className = 'orbit-modal';

        var header = document.createElement('div');
        header.className = 'orbit-modal-header';
        header.innerHTML = '<span>' + title + '</span><button class="orbit-modal-close">&times;</button>';

        var content = document.createElement('div');
        content.className = 'orbit-modal-body';
        content.appendChild(body);

        var footer = document.createElement('div');
        footer.className = 'orbit-modal-footer';
        footer.innerHTML =
            '<button class="orbit-modal-cancel">Cancel</button>' +
            '<button class="orbit-modal-ok">Generate Orbit</button>';

        dialog.appendChild(header);
        dialog.appendChild(content);
        dialog.appendChild(footer);
        overlay.appendChild(dialog);

        function close() { if (overlay.parentNode) overlay.parentNode.removeChild(overlay); }

        overlay.querySelector('.orbit-modal-close').addEventListener('click', function () { close(); if (onCancel) onCancel(); });
        overlay.querySelector('.orbit-modal-cancel').addEventListener('click', function () { close(); if (onCancel) onCancel(); });
        overlay.querySelector('.orbit-modal-ok').addEventListener('click', function () { close(); if (onOk) onOk(); });

        return overlay;
    }

    // ─── Init ────────────────────────────────────────────────

    return { init, activate, deactivate };

})();
