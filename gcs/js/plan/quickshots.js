/* ============================================================
   quickshots.js — Cinematic Quickshot mission generator (T3-17)
   Modes: Dronie, Helix, Orbit (link), Reveal
   Each: click map to set subject, configure params, generate waypoints.
   Injects "Quickshots" button into Plan view toolbar.
   ============================================================ */

'use strict';

window.Quickshots = (function () {

    var _active    = false;      // waiting for map click
    var _mode      = null;       // 'dronie' | 'helix' | 'reveal' | 'orbit'
    var _map       = null;
    var _previewLayer = null;    // Leaflet layer group for preview markers
    var _subjectMarker = null;

    var MAV_CMD_NAV_WAYPOINT  = 16;
    var MAV_CMD_DO_SET_ROI    = 201;

    // ─── Toolbar injection ────────────────────────────────────

    function init() {
        meridian.events.on('panel_change', function (panel) {
            if (panel === 'plan') {
                _injectButton();
            } else {
                _deactivate();
            }
        });
    }

    function _injectButton() {
        // Wait for PlanView to render the toolbar
        setTimeout(function () {
            var toolbar = document.querySelector('.mission-toolbar');
            if (!toolbar || document.getElementById('btn-quickshots')) return;

            var btn = document.createElement('button');
            btn.id = 'btn-quickshots';
            btn.title = 'Cinematic quickshot modes';
            btn.textContent = '\u25B6 Quickshots';

            btn.addEventListener('click', function () {
                _showQuickshotModal();
            });

            // Insert after orbit button
            var orbitBtn = document.getElementById('btn-orbit');
            if (orbitBtn && orbitBtn.nextSibling) {
                toolbar.insertBefore(btn, orbitBtn.nextSibling);
            } else {
                toolbar.appendChild(btn);
            }
        }, 50);
    }

    // ─── Modal: mode picker ───────────────────────────────────

    function _showQuickshotModal() {
        var overlay = document.createElement('div');
        overlay.className = 'modal-overlay quickshot-overlay';
        overlay.id = 'quickshot-modal-overlay';

        overlay.innerHTML =
            '<div class="modal-box quickshot-modal">' +
                '<div class="modal-title">Quickshot Modes</div>' +
                '<div class="quickshot-grid">' +
                    _modeCard('dronie',  '\u2197', 'Dronie',  'Fly backward & up from subject') +
                    _modeCard('helix',   '\u29B8', 'Helix',   'Spiral up around subject') +
                    _modeCard('orbit',   '\u25CB', 'Orbit',   'Circle around subject (use Orbit tool)') +
                    _modeCard('reveal',  '\u2198', 'Reveal',  'Descending approach toward subject') +
                '</div>' +
                '<div class="modal-actions">' +
                    '<button class="modal-cancel-btn" id="qs-cancel-btn">Cancel</button>' +
                '</div>' +
            '</div>';

        document.body.appendChild(overlay);

        overlay.querySelectorAll('.qs-mode-card').forEach(function (card) {
            card.addEventListener('click', function () {
                var mode = card.dataset.mode;
                document.body.removeChild(overlay);
                if (mode === 'orbit') {
                    if (window.OrbitTool) OrbitTool.activate();
                } else {
                    _pickMode(mode);
                }
            });
        });

        document.getElementById('qs-cancel-btn').addEventListener('click', function () {
            document.body.removeChild(overlay);
        });
    }

    function _modeCard(mode, icon, label, desc) {
        return '<div class="qs-mode-card" data-mode="' + mode + '">' +
            '<div class="qs-mode-icon">' + icon + '</div>' +
            '<div class="qs-mode-label">' + label + '</div>' +
            '<div class="qs-mode-desc">' + desc + '</div>' +
            '</div>';
    }

    // ─── Map click: pick subject ──────────────────────────────

    function _pickMode(mode) {
        _mode = mode;
        _active = true;
        _map = FlyMap.getMap();
        if (!_map) return;

        _map.getContainer().style.cursor = 'crosshair';
        _map.on('click', _onMapClick);

        var btn = document.getElementById('btn-quickshots');
        if (btn) btn.classList.add('active');

        meridian.log('Quickshots: click map to set subject point for ' + mode, 'info');
    }

    function _onMapClick(e) {
        if (!_active) return;
        _deactivate();

        var subject = e.latlng;

        // Show marker for subject
        _previewLayer = L.layerGroup().addTo(_map);
        _subjectMarker = L.circleMarker(subject, {
            radius: 7, color: '#f59e0b', weight: 2.5,
            fillColor: '#f59e0b', fillOpacity: 0.3,
        }).bindTooltip(_mode.toUpperCase() + ' subject', { permanent: false }).addTo(_previewLayer);

        _showConfigModal(subject);
    }

    function _deactivate() {
        _active = false;
        if (_map) {
            _map.getContainer().style.cursor = '';
            _map.off('click', _onMapClick);
        }
        var btn = document.getElementById('btn-quickshots');
        if (btn) btn.classList.remove('active');
    }

    // ─── Config modal ─────────────────────────────────────────

    function _showConfigModal(subject) {
        var defaults = _modeDefaults(_mode);

        var overlay = document.createElement('div');
        overlay.className = 'modal-overlay quickshot-overlay';
        overlay.id = 'qs-config-overlay';

        overlay.innerHTML =
            '<div class="modal-box quickshot-config-modal">' +
                '<div class="modal-title">' + _modeLabel(_mode) + ' Configuration</div>' +
                '<div class="qs-config-fields">' +
                    _configField('qs-distance', 'Distance (m)', defaults.distance, 5, 500) +
                    _configField('qs-altitude', 'Altitude (m AGL)', defaults.altitude, 5, 200) +
                    _configField('qs-speed',    'Speed (m/s)',    defaults.speed,    0.5, 15) +
                    (_mode === 'helix' ? _configField('qs-turns', 'Turns', defaults.turns, 1, 5) : '') +
                '</div>' +
                '<div class="modal-actions">' +
                    '<button class="modal-cancel-btn" id="qs-cfg-cancel">Cancel</button>' +
                    '<button class="modal-ok-btn" id="qs-cfg-ok">Generate Waypoints</button>' +
                '</div>' +
            '</div>';

        document.body.appendChild(overlay);

        document.getElementById('qs-cfg-cancel').addEventListener('click', function () {
            document.body.removeChild(overlay);
            _clearPreview();
        });

        document.getElementById('qs-cfg-ok').addEventListener('click', function () {
            var params = {
                distance: parseFloat(document.getElementById('qs-distance').value) || defaults.distance,
                altitude: parseFloat(document.getElementById('qs-altitude').value) || defaults.altitude,
                speed:    parseFloat(document.getElementById('qs-speed').value)    || defaults.speed,
                turns:    parseFloat((document.getElementById('qs-turns') || {}).value || defaults.turns),
            };
            document.body.removeChild(overlay);
            _clearPreview();
            _generateWaypoints(subject, params);
        });
    }

    function _configField(id, label, val, min, max) {
        return '<div class="qs-field">' +
            '<label class="qs-field-label" for="' + id + '">' + label + '</label>' +
            '<input class="qs-field-input" type="number" id="' + id + '" value="' + val + '" min="' + min + '" max="' + max + '" step="0.5">' +
            '</div>';
    }

    function _modeDefaults(mode) {
        switch (mode) {
            case 'dronie':  return { distance: 30, altitude: 20, speed: 3, turns: 1 };
            case 'helix':   return { distance: 20, altitude: 40, speed: 2, turns: 2 };
            case 'reveal':  return { distance: 30, altitude: 20, speed: 3, turns: 1 };
            default:        return { distance: 30, altitude: 20, speed: 3, turns: 1 };
        }
    }

    function _modeLabel(mode) {
        return { dronie: 'Dronie', helix: 'Helix', reveal: 'Reveal', orbit: 'Orbit' }[mode] || mode;
    }

    // ─── Waypoint generators ──────────────────────────────────

    function _generateWaypoints(subject, params) {
        var wps = [];

        // Always prepend DO_SET_ROI at the subject point
        wps.push({
            command: MAV_CMD_DO_SET_ROI,
            lat:  subject.lat,
            lon:  subject.lng,
            alt:  0,
            param1: 0, param2: 0, param3: 0, param4: 0,
            frame: 3, // MAV_FRAME_GLOBAL_RELATIVE_ALT
            autocontinue: 1,
            _qs: true,
        });

        switch (_mode) {
            case 'dronie':
                wps = wps.concat(_dronieWaypoints(subject, params));
                break;
            case 'helix':
                wps = wps.concat(_helixWaypoints(subject, params));
                break;
            case 'reveal':
                wps = wps.concat(_revealWaypoints(subject, params));
                break;
        }

        // Insert into mission (append)
        if (window.Mission) {
            var existing = Mission.getItems();
            var all = existing.concat(wps);
            Mission.setItems(all);
            meridian.log('Quickshots: added ' + wps.length + ' waypoints for ' + _modeLabel(_mode), 'info');
        }
    }

    // Dronie: 5 waypoints flying backward + upward from subject
    // Bearing based on vehicle heading or 0° if unavailable
    function _dronieWaypoints(subject, params) {
        var bearing = _vehicleBearing(subject);
        var wps = [];
        var steps = 5;

        for (var i = 1; i <= steps; i++) {
            var frac   = i / steps;
            var dist   = params.distance * frac;
            var alt    = params.altitude * frac;
            var pos    = _offsetLatLon(subject.lat, subject.lng, bearing, dist);
            wps.push(_navWp(pos.lat, pos.lng, alt, params.speed));
        }
        return wps;
    }

    // Reveal: reverse of dronie — descending approach from far to subject
    function _revealWaypoints(subject, params) {
        var bearing = (_vehicleBearing(subject) + 180) % 360;
        var wps = [];
        var steps = 5;

        for (var i = steps; i >= 1; i--) {
            var frac = i / steps;
            var dist = params.distance * frac;
            var alt  = params.altitude * frac;
            var pos  = _offsetLatLon(subject.lat, subject.lng, bearing, dist);
            wps.push(_navWp(pos.lat, pos.lng, alt, params.speed));
        }
        return wps;
    }

    // Helix: 16 waypoints per turn, spiraling outward+upward
    function _helixWaypoints(subject, params) {
        var wps = [];
        var turns    = Math.round(params.turns) || 2;
        var total    = turns * 16;
        var maxDist  = params.distance;
        var maxAlt   = params.altitude;

        for (var i = 0; i < total; i++) {
            var frac    = i / total;
            var angle   = frac * 360 * turns;
            var radius  = maxDist * (0.15 + 0.85 * frac);  // start close, expand outward
            var alt     = maxAlt * frac;
            var pos     = _polarOffset(subject.lat, subject.lng, angle, radius);
            wps.push(_navWp(pos.lat, pos.lng, alt, params.speed));
        }

        // Final point above subject at max altitude
        wps.push(_navWp(subject.lat, subject.lng, maxAlt + 5, params.speed));
        return wps;
    }

    // ─── Geometry helpers ─────────────────────────────────────

    var DEG2RAD = Math.PI / 180;
    var RAD2DEG = 180 / Math.PI;
    var EARTH_R = 6378137; // metres

    function _offsetLatLon(lat, lon, bearingDeg, distM) {
        var d    = distM / EARTH_R;
        var b    = bearingDeg * DEG2RAD;
        var lat1 = lat * DEG2RAD;
        var lon1 = lon * DEG2RAD;
        var lat2 = Math.asin(Math.sin(lat1) * Math.cos(d) + Math.cos(lat1) * Math.sin(d) * Math.cos(b));
        var lon2 = lon1 + Math.atan2(Math.sin(b) * Math.sin(d) * Math.cos(lat1), Math.cos(d) - Math.sin(lat1) * Math.sin(lat2));
        return { lat: lat2 * RAD2DEG, lng: lon2 * RAD2DEG };
    }

    // Polar offset: angle in degrees from north, going around subject
    function _polarOffset(lat, lon, angleDeg, radiusM) {
        return _offsetLatLon(lat, lon, angleDeg, radiusM);
    }

    function _vehicleBearing(subject) {
        var v = meridian.v;
        if (v && v.lat && v.lon) {
            var dLon = (subject.lng - v.lon) * DEG2RAD;
            var lat1 = v.lat * DEG2RAD;
            var lat2 = subject.lat * DEG2RAD;
            var y = Math.sin(dLon) * Math.cos(lat2);
            var x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
            var bear = Math.atan2(y, x) * RAD2DEG;
            // Return bearing AWAY from subject (vehicle is at the subject, so fly away from it)
            return (bear + 180) % 360;
        }
        return 0;
    }

    function _navWp(lat, lng, alt, speed) {
        return {
            command: MAV_CMD_NAV_WAYPOINT,
            lat: lat,
            lon: lng,
            alt: alt,
            param1: 0,
            param2: speed * 2,  // acceptance radius ~= 2x speed
            param3: 0,
            param4: NaN,
            frame: 3, // MAV_FRAME_GLOBAL_RELATIVE_ALT
            autocontinue: 1,
            _qs: true,
        };
    }

    function _clearPreview() {
        if (_previewLayer && _map) {
            _map.removeLayer(_previewLayer);
            _previewLayer = null;
            _subjectMarker = null;
        }
    }

    return { init };

})();
