/* ============================================================
   survey.js — Survey & Corridor scan tool
   Polygon survey: click 3+ points to define area, auto-generate
   parallel waypoint lines inside polygon.
   Corridor scan: draw polyline path, generate parallel waypoints
   along the corridor.
   Camera model presets for ground coverage calculation.
   ============================================================ */

'use strict';

window.Survey = (function () {

    // --- Camera Presets ---
    var CAMERAS = {
        'dji-mini3-pro': {
            name: 'DJI Mini 3 Pro',
            sensorWidth: 9.7,    // mm
            focalLength: 6.72,   // mm
            imageWidth: 4032,    // px
            imageHeight: 3024,
        },
        'sony-a7r': {
            name: 'Sony A7R IV',
            sensorWidth: 35.7,   // mm
            focalLength: 35,     // mm
            imageWidth: 9504,
            imageHeight: 6336,
        },
        'custom': {
            name: 'Custom',
            sensorWidth: 13.2,
            focalLength: 8.8,
            imageWidth: 5472,
            imageHeight: 3648,
        },
    };

    // T2-20: Load saved custom camera presets from localStorage on startup
    (function loadCustomCameras() {
        try {
            var saved = localStorage.getItem('meridian_custom_cameras');
            if (saved) {
                var customs = JSON.parse(saved);
                for (var key in customs) {
                    CAMERAS[key] = customs[key];
                }
            }
        } catch (e) { /* ignore */ }
    })();

    // --- State ---
    var mode = null;            // null, 'polygon', 'corridor'
    var polygonPoints = [];     // [{lat, lon}]
    var corridorPoints = [];    // [{lat, lon}]
    var params = {
        spacing: 30,            // meters between lines
        altitude: 50,           // meters AGL
        angle: 0,               // degrees (0 = north-south lines)
        camera: 'dji-mini3-pro',
        corridorWidth: 40,      // meters
        overlap: 70,            // % front overlap
    };

    // Map layers
    var polygonLayer = null;
    var corridorLayer = null;
    var previewMarkers = [];
    var previewLines = [];
    var clickHandler = null;

    // Panel elements
    var panelEl = null;

    // --- Polygon Survey ---

    function startPolygonSurvey() {
        cancelSurvey();
        mode = 'polygon';
        polygonPoints = [];

        var map = FlyMap.getMap();
        if (!map) return;

        map.getContainer().style.cursor = 'crosshair';
        clickHandler = function (e) {
            addPolygonPoint(e.latlng.lat, e.latlng.lng);
        };
        map.on('click', clickHandler);

        meridian.log('Survey: click map to define polygon (3+ points)', 'info');
        renderPanel();
    }

    function addPolygonPoint(lat, lon) {
        polygonPoints.push({ lat: lat, lon: lon });
        drawPolygonPreview();
        renderPanel();
    }

    function drawPolygonPreview() {
        var map = FlyMap.getMap();
        if (!map) return;

        // Remove old polygon
        if (polygonLayer) { map.removeLayer(polygonLayer); polygonLayer = null; }

        if (polygonPoints.length < 2) return;

        var latlngs = polygonPoints.map(function (p) { return [p.lat, p.lon]; });

        if (polygonPoints.length >= 3) {
            polygonLayer = L.polygon(latlngs, {
                color: '#00e5ff',
                weight: 2,
                fillColor: '#00e5ff',
                fillOpacity: 0.1,
                dashArray: '6,4',
            }).addTo(map);
        } else {
            polygonLayer = L.polyline(latlngs, {
                color: '#00e5ff',
                weight: 2,
                dashArray: '6,4',
            }).addTo(map);
        }
    }

    // --- Corridor Scan ---

    function startCorridorScan() {
        cancelSurvey();
        mode = 'corridor';
        corridorPoints = [];

        var map = FlyMap.getMap();
        if (!map) return;

        map.getContainer().style.cursor = 'crosshair';
        clickHandler = function (e) {
            addCorridorPoint(e.latlng.lat, e.latlng.lng);
        };
        map.on('click', clickHandler);

        meridian.log('Corridor: click map to define path (2+ points)', 'info');
        renderPanel();
    }

    function addCorridorPoint(lat, lon) {
        corridorPoints.push({ lat: lat, lon: lon });
        drawCorridorPreview();
        renderPanel();
    }

    function drawCorridorPreview() {
        var map = FlyMap.getMap();
        if (!map) return;

        if (corridorLayer) { map.removeLayer(corridorLayer); corridorLayer = null; }

        if (corridorPoints.length < 2) return;

        var latlngs = corridorPoints.map(function (p) { return [p.lat, p.lon]; });

        // Draw center line
        corridorLayer = L.layerGroup();

        var centerLine = L.polyline(latlngs, {
            color: '#e040fb',
            weight: 2,
            dashArray: '6,4',
        });
        corridorLayer.addLayer(centerLine);

        // Draw corridor width band
        var halfW = params.corridorWidth / 2;
        var leftPts = [];
        var rightPts = [];

        for (var i = 0; i < latlngs.length; i++) {
            var bearing;
            if (i < latlngs.length - 1) {
                bearing = calcBearing(latlngs[i][0], latlngs[i][1], latlngs[i + 1][0], latlngs[i + 1][1]);
            } else {
                bearing = calcBearing(latlngs[i - 1][0], latlngs[i - 1][1], latlngs[i][0], latlngs[i][1]);
            }

            var leftBearing = (bearing - 90 + 360) % 360;
            var rightBearing = (bearing + 90) % 360;

            leftPts.push(offsetPoint(latlngs[i][0], latlngs[i][1], leftBearing, halfW));
            rightPts.push(offsetPoint(latlngs[i][0], latlngs[i][1], rightBearing, halfW));
        }

        var bandPts = leftPts.concat(rightPts.reverse());
        var band = L.polygon(bandPts, {
            color: '#e040fb',
            weight: 1,
            fillColor: '#e040fb',
            fillOpacity: 0.08,
        });
        corridorLayer.addLayer(band);
        corridorLayer.addTo(map);
    }

    // --- Generate Survey Waypoints ---

    function generatePolygonWaypoints() {
        if (polygonPoints.length < 3) {
            meridian.log('Need at least 3 points for survey polygon', 'warn');
            return;
        }

        var latlngs = polygonPoints.map(function (p) { return [p.lat, p.lon]; });

        // Compute bounding box
        var minLat = Infinity, maxLat = -Infinity;
        var minLon = Infinity, maxLon = -Infinity;
        latlngs.forEach(function (ll) {
            if (ll[0] < minLat) minLat = ll[0];
            if (ll[0] > maxLat) maxLat = ll[0];
            if (ll[1] < minLon) minLon = ll[1];
            if (ll[1] > maxLon) maxLon = ll[1];
        });

        var centerLat = (minLat + maxLat) / 2;
        var centerLon = (minLon + maxLon) / 2;

        // Convert angle to radians
        var angleRad = params.angle * Math.PI / 180;

        // Generate parallel lines across bounding box
        // Spacing in degrees (approximate)
        var spacingLat = params.spacing / 111320;
        var spacingLon = params.spacing / (111320 * Math.cos(centerLat * Math.PI / 180));

        // Determine scan direction
        var cosA = Math.cos(angleRad);
        var sinA = Math.sin(angleRad);

        // Compute bounding box diagonal
        var diagLat = maxLat - minLat;
        var diagLon = maxLon - minLon;
        var diag = Math.sqrt(diagLat * diagLat + diagLon * diagLon);

        // Number of passes
        var spacingDeg = Math.max(spacingLat, spacingLon);
        if (spacingDeg === 0) spacingDeg = 0.0001;
        var nPasses = Math.ceil(diag / spacingDeg) + 2;

        var waypoints = [];
        var reverse = false;

        for (var i = -Math.floor(nPasses / 2); i <= Math.ceil(nPasses / 2); i++) {
            // Offset perpendicular to scan angle
            var offsetLat = centerLat + i * spacingDeg * (-sinA);
            var offsetLon = centerLon + i * spacingDeg * cosA;

            // Line endpoints (extend beyond bbox)
            var ext = diag * 1.5;
            var p1Lat = offsetLat - ext * cosA;
            var p1Lon = offsetLon - ext * sinA;
            var p2Lat = offsetLat + ext * cosA;
            var p2Lon = offsetLon + ext * sinA;

            // Clip line to polygon
            var intersections = clipLineToPolygon(p1Lat, p1Lon, p2Lat, p2Lon, latlngs);
            if (intersections.length >= 2) {
                // Sort by distance from p1
                intersections.sort(function (a, b) {
                    var da = (a[0] - p1Lat) * (a[0] - p1Lat) + (a[1] - p1Lon) * (a[1] - p1Lon);
                    var db = (b[0] - p1Lat) * (b[0] - p1Lat) + (b[1] - p1Lon) * (b[1] - p1Lon);
                    return da - db;
                });

                var start = intersections[0];
                var end = intersections[intersections.length - 1];

                if (reverse) {
                    waypoints.push({ lat: end[0], lon: end[1] });
                    waypoints.push({ lat: start[0], lon: start[1] });
                } else {
                    waypoints.push({ lat: start[0], lon: start[1] });
                    waypoints.push({ lat: end[0], lon: end[1] });
                }
                reverse = !reverse;
            }
        }

        return waypoints;
    }

    function generateCorridorWaypoints() {
        if (corridorPoints.length < 2) {
            meridian.log('Need at least 2 points for corridor', 'warn');
            return;
        }

        var latlngs = corridorPoints.map(function (p) { return [p.lat, p.lon]; });
        var halfW = params.corridorWidth / 2;
        var waypoints = [];
        var reverse = false;

        // For each segment, generate perpendicular passes
        for (var i = 0; i < latlngs.length - 1; i++) {
            var segBearing = calcBearing(latlngs[i][0], latlngs[i][1], latlngs[i + 1][0], latlngs[i + 1][1]);
            var segDist = haversine(latlngs[i][0], latlngs[i][1], latlngs[i + 1][0], latlngs[i + 1][1]);
            var nSteps = Math.ceil(segDist / params.spacing);

            for (var j = 0; j <= nSteps; j++) {
                var frac = nSteps === 0 ? 0 : j / nSteps;
                var midLat = latlngs[i][0] + frac * (latlngs[i + 1][0] - latlngs[i][0]);
                var midLon = latlngs[i][1] + frac * (latlngs[i + 1][1] - latlngs[i][1]);

                var leftBearing = (segBearing - 90 + 360) % 360;
                var rightBearing = (segBearing + 90) % 360;

                var pLeft = offsetPoint(midLat, midLon, leftBearing, halfW);
                var pRight = offsetPoint(midLat, midLon, rightBearing, halfW);

                if (reverse) {
                    waypoints.push({ lat: pRight[0], lon: pRight[1] });
                    waypoints.push({ lat: pLeft[0], lon: pLeft[1] });
                } else {
                    waypoints.push({ lat: pLeft[0], lon: pLeft[1] });
                    waypoints.push({ lat: pRight[0], lon: pRight[1] });
                }
                reverse = !reverse;
            }
        }

        return waypoints;
    }

    function applyToMission(waypoints) {
        if (!waypoints || waypoints.length === 0) return;

        // Add each as NAV_WAYPOINT
        for (var i = 0; i < waypoints.length; i++) {
            Mission.addWaypoint(waypoints[i].lat, waypoints[i].lon, params.altitude);
        }

        meridian.log('Survey: added ' + waypoints.length + ' waypoints', 'info');
        cancelSurvey();
    }

    // --- Camera Ground Coverage ---

    function calcGroundCoverage(altitude, cameraId) {
        var cam = CAMERAS[cameraId] || CAMERAS['custom'];
        // Ground Sample Distance (m/px)
        var gsd = (altitude * cam.sensorWidth) / (cam.focalLength * cam.imageWidth);
        // Footprint
        var footprintW = gsd * cam.imageWidth;
        var footprintH = gsd * cam.imageHeight;
        return {
            gsd: gsd,
            width: footprintW,
            height: footprintH,
            camera: cam.name,
        };
    }

    // --- Cancel / Cleanup ---

    function cancelSurvey() {
        var map = FlyMap.getMap();
        if (map) {
            if (clickHandler) { map.off('click', clickHandler); clickHandler = null; }
            map.getContainer().style.cursor = '';
        }

        clearPreview();
        mode = null;
        polygonPoints = [];
        corridorPoints = [];
        renderPanel();
    }

    function clearPreview() {
        var map = FlyMap.getMap();
        if (!map) return;

        if (polygonLayer) { map.removeLayer(polygonLayer); polygonLayer = null; }
        if (corridorLayer) { map.removeLayer(corridorLayer); corridorLayer = null; }

        previewMarkers.forEach(function (m) { map.removeLayer(m); });
        previewMarkers = [];
        previewLines.forEach(function (l) { map.removeLayer(l); });
        previewLines = [];
    }

    // --- Panel Rendering ---

    function renderPanel() {
        if (!panelEl) return;

        if (!mode) {
            panelEl.style.display = 'none';
            return;
        }

        panelEl.style.display = '';
        var html = '<div class="survey-panel">';

        if (mode === 'polygon') {
            html += '<div class="survey-header">Polygon Survey</div>';
            html += '<div class="survey-points">Points: ' + polygonPoints.length + ' / 3+ needed</div>';
        } else {
            html += '<div class="survey-header">Corridor Scan</div>';
            html += '<div class="survey-points">Path points: ' + corridorPoints.length + ' / 2+ needed</div>';
        }

        // Parameters
        html += '<div class="survey-params">';

        html += '<div class="survey-field">';
        html += '<label>Altitude (m)</label>';
        html += '<input type="number" id="survey-alt" value="' + params.altitude + '" min="5" max="500" step="5">';
        html += '</div>';

        if (mode === 'polygon') {
            html += '<div class="survey-field">';
            html += '<label>Line Spacing (m)</label>';
            html += '<input type="number" id="survey-spacing" value="' + params.spacing + '" min="5" max="500" step="5">';
            html += '</div>';

            html += '<div class="survey-field">';
            html += '<label>Scan Angle (deg)</label>';
            html += '<input type="number" id="survey-angle" value="' + params.angle + '" min="0" max="359" step="5">';
            html += '</div>';
        } else {
            html += '<div class="survey-field">';
            html += '<label>Corridor Width (m)</label>';
            html += '<input type="number" id="survey-width" value="' + params.corridorWidth + '" min="10" max="500" step="5">';
            html += '</div>';

            html += '<div class="survey-field">';
            html += '<label>Pass Spacing (m)</label>';
            html += '<input type="number" id="survey-spacing" value="' + params.spacing + '" min="5" max="500" step="5">';
            html += '</div>';
        }

        // Camera selector
        html += '<div class="survey-field full">';
        html += '<label>Camera</label>';
        html += '<div class="survey-camera-row">';
        html += '<select id="survey-camera">';
        for (var key in CAMERAS) {
            var sel = (key === params.camera) ? ' selected' : '';
            html += '<option value="' + key + '"' + sel + '>' + CAMERAS[key].name + '</option>';
        }
        html += '</select>';
        // T2-20: Show "Save Preset" button when Custom is selected
        if (params.camera === 'custom') {
            html += '<button class="survey-btn save-preset" id="btn-survey-save-preset" title="Save current custom values as a named preset">Save Preset</button>';
        }
        html += '</div>';
        html += '</div>';

        // Ground coverage info
        var cov = calcGroundCoverage(params.altitude, params.camera);
        html += '<div class="survey-coverage">';
        html += '<span>GSD: ' + (cov.gsd * 100).toFixed(1) + ' cm/px</span>';
        html += '<span>Footprint: ' + cov.width.toFixed(0) + ' x ' + cov.height.toFixed(0) + ' m</span>';
        html += '</div>';

        html += '</div>'; // survey-params

        // Action buttons
        var canGenerate = (mode === 'polygon' && polygonPoints.length >= 3) ||
                          (mode === 'corridor' && corridorPoints.length >= 2);

        html += '<div class="survey-actions">';
        html += '<button class="survey-btn generate" id="btn-survey-generate"' +
                (canGenerate ? '' : ' disabled') + '>Generate Waypoints</button>';
        html += '<button class="survey-btn undo" id="btn-survey-undo"' +
                ((mode === 'polygon' ? polygonPoints.length : corridorPoints.length) > 0 ? '' : ' disabled') +
                '>Undo Last Point</button>';
        html += '<button class="survey-btn cancel" id="btn-survey-cancel">Cancel</button>';
        html += '</div>';

        html += '</div>';

        panelEl.innerHTML = html;

        // Wire events
        wireInputs();
    }

    function wireInputs() {
        var altInput = document.getElementById('survey-alt');
        var spacingInput = document.getElementById('survey-spacing');
        var angleInput = document.getElementById('survey-angle');
        var widthInput = document.getElementById('survey-width');
        var cameraSelect = document.getElementById('survey-camera');
        var generateBtn = document.getElementById('btn-survey-generate');
        var undoBtn = document.getElementById('btn-survey-undo');
        var cancelBtn = document.getElementById('btn-survey-cancel');

        if (altInput) altInput.addEventListener('change', function () {
            params.altitude = parseFloat(altInput.value) || 50;
            renderPanel();
        });
        if (spacingInput) spacingInput.addEventListener('change', function () {
            params.spacing = parseFloat(spacingInput.value) || 30;
        });
        if (angleInput) angleInput.addEventListener('change', function () {
            params.angle = parseFloat(angleInput.value) || 0;
        });
        if (widthInput) widthInput.addEventListener('change', function () {
            params.corridorWidth = parseFloat(widthInput.value) || 40;
            drawCorridorPreview();
        });
        if (cameraSelect) cameraSelect.addEventListener('change', function () {
            params.camera = cameraSelect.value;
            renderPanel();
        });

        if (generateBtn) generateBtn.addEventListener('click', function () {
            var wps;
            if (mode === 'polygon') {
                wps = generatePolygonWaypoints();
            } else {
                wps = generateCorridorWaypoints();
            }
            applyToMission(wps);
        });

        if (undoBtn) undoBtn.addEventListener('click', function () {
            if (mode === 'polygon' && polygonPoints.length > 0) {
                polygonPoints.pop();
                drawPolygonPreview();
            } else if (mode === 'corridor' && corridorPoints.length > 0) {
                corridorPoints.pop();
                drawCorridorPreview();
            }
            renderPanel();
        });

        if (cancelBtn) cancelBtn.addEventListener('click', cancelSurvey);

        // T2-20: Save Preset button — only shown when camera === 'custom'
        var savePresetBtn = document.getElementById('btn-survey-save-preset');
        if (savePresetBtn) {
            savePresetBtn.addEventListener('click', function () {
                var presetName = prompt('Enter a name for this camera preset:');
                if (!presetName || !presetName.trim()) return;
                var key = 'custom_' + presetName.trim().toLowerCase().replace(/\s+/g, '_');
                var cam = CAMERAS['custom'];
                var newPreset = {
                    name: presetName.trim(),
                    sensorWidth: cam.sensorWidth,
                    focalLength: cam.focalLength,
                    imageWidth: cam.imageWidth,
                    imageHeight: cam.imageHeight,
                };
                CAMERAS[key] = newPreset;
                // Persist only the user-saved presets (keys starting with 'custom_')
                try {
                    var toSave = {};
                    for (var k in CAMERAS) {
                        if (k.indexOf('custom_') === 0) toSave[k] = CAMERAS[k];
                    }
                    localStorage.setItem('meridian_custom_cameras', JSON.stringify(toSave));
                } catch (e) { /* ignore */ }
                params.camera = key;
                meridian.log('Camera preset saved: ' + presetName.trim(), 'info');
                renderPanel();
            });
        }
    }

    // --- Geometry Helpers ---

    function calcBearing(lat1, lon1, lat2, lon2) {
        var dLon = (lon2 - lon1) * Math.PI / 180;
        var y = Math.sin(dLon) * Math.cos(lat2 * Math.PI / 180);
        var x = Math.cos(lat1 * Math.PI / 180) * Math.sin(lat2 * Math.PI / 180) -
                Math.sin(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) * Math.cos(dLon);
        return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
    }

    function haversine(lat1, lon1, lat2, lon2) {
        var R = 6371000;
        var dLat = (lat2 - lat1) * Math.PI / 180;
        var dLon = (lon2 - lon1) * Math.PI / 180;
        var a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                Math.sin(dLon / 2) * Math.sin(dLon / 2);
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    function offsetPoint(lat, lon, bearingDeg, distM) {
        var R = 6371000;
        var brng = bearingDeg * Math.PI / 180;
        var lat1 = lat * Math.PI / 180;
        var lon1 = lon * Math.PI / 180;
        var lat2 = Math.asin(Math.sin(lat1) * Math.cos(distM / R) +
                   Math.cos(lat1) * Math.sin(distM / R) * Math.cos(brng));
        var lon2 = lon1 + Math.atan2(
            Math.sin(brng) * Math.sin(distM / R) * Math.cos(lat1),
            Math.cos(distM / R) - Math.sin(lat1) * Math.sin(lat2));
        return [lat2 * 180 / Math.PI, lon2 * 180 / Math.PI];
    }

    // Line-polygon intersection (2D lat/lon)
    function clipLineToPolygon(lat1, lon1, lat2, lon2, polygon) {
        var intersections = [];
        var n = polygon.length;
        for (var i = 0; i < n; i++) {
            var j = (i + 1) % n;
            var pt = lineIntersect(
                lat1, lon1, lat2, lon2,
                polygon[i][0], polygon[i][1],
                polygon[j][0], polygon[j][1]
            );
            if (pt) intersections.push(pt);
        }
        return intersections;
    }

    function lineIntersect(x1, y1, x2, y2, x3, y3, x4, y4) {
        var denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(denom) < 1e-12) return null;
        var t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        var u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            return [x1 + t * (x2 - x1), y1 + t * (y2 - y1)];
        }
        return null;
    }

    // --- Public API ---

    function setPanelEl(el) {
        panelEl = el;
    }

    function isActive() {
        return mode !== null;
    }

    function getMode() {
        return mode;
    }

    return {
        startPolygonSurvey: startPolygonSurvey,
        startCorridorScan: startCorridorScan,
        cancelSurvey: cancelSurvey,
        setPanelEl: setPanelEl,
        isActive: isActive,
        getMode: getMode,
        CAMERAS: CAMERAS,
        calcGroundCoverage: calcGroundCoverage,
    };

})();
