/* ============================================================
   geofence.js — T1-2: Geofence polygon drawing tool
   Draw polygon fence on map, store as {lat, lon} points,
   upload as FENCE_POINT MAVLink messages, show in Fly view.
   ============================================================ */

'use strict';

window.Geofence = (function () {

    let points = [];       // Array of {lat, lon}
    let drawing = false;
    let polygon = null;    // Leaflet polygon on plan/fly map
    let tempMarkers = [];  // Markers during drawing
    let tempLine = null;   // Polyline during drawing

    // --- Drawing Mode ---

    function startDrawing() {
        if (drawing) return;
        drawing = true;
        points = [];
        clearMapDisplay();

        var map = FlyMap.getMap();
        if (!map) return;

        map.on('click', onMapClick);
        map.getContainer().style.cursor = 'crosshair';
        meridian.log('Geofence: click map points, then press Finish', 'info');
        meridian.events.emit('geofence_drawing', true);
    }

    function onMapClick(e) {
        if (!drawing) return;
        // Don't capture clicks on existing markers
        if (e.originalEvent && e.originalEvent._wpMarkerClick) return;

        var pt = { lat: e.latlng.lat, lon: e.latlng.lng };
        points.push(pt);

        var map = FlyMap.getMap();
        if (!map) return;

        // Add vertex marker
        var marker = L.circleMarker([pt.lat, pt.lon], {
            radius: 6,
            color: '#f44336',
            fillColor: '#f44336',
            fillOpacity: 0.7,
            weight: 2,
        }).addTo(map);
        tempMarkers.push(marker);

        // Update temp polyline
        updateTempLine();
    }

    function updateTempLine() {
        var map = FlyMap.getMap();
        if (!map) return;

        var latlngs = points.map(function (p) { return [p.lat, p.lon]; });

        if (tempLine) {
            map.removeLayer(tempLine);
        }
        if (latlngs.length >= 2) {
            // Close the polygon visually
            var closed = latlngs.concat([latlngs[0]]);
            tempLine = L.polyline(closed, {
                color: '#f44336',
                weight: 2,
                opacity: 0.7,
                dashArray: '6,4',
            }).addTo(map);
        }
    }

    function finishDrawing() {
        if (!drawing) return;
        drawing = false;

        var map = FlyMap.getMap();
        if (map) {
            map.off('click', onMapClick);
            map.getContainer().style.cursor = '';
        }

        // Clean up temp markers and line
        clearTempDrawing();

        if (points.length < 3) {
            meridian.log('Geofence needs at least 3 points', 'warn');
            points = [];
            meridian.events.emit('geofence_drawing', false);
            return;
        }

        // Show the final polygon
        showPolygon();
        meridian.log('Geofence set with ' + points.length + ' vertices', 'info');
        meridian.events.emit('geofence_drawing', false);
        meridian.events.emit('geofence_change', points);
    }

    function cancelDrawing() {
        if (!drawing) return;
        drawing = false;

        var map = FlyMap.getMap();
        if (map) {
            map.off('click', onMapClick);
            map.getContainer().style.cursor = '';
        }

        clearTempDrawing();
        points = [];
        meridian.events.emit('geofence_drawing', false);
        meridian.log('Geofence drawing cancelled', 'info');
    }

    function clearTempDrawing() {
        var map = FlyMap.getMap();
        if (!map) return;
        for (var i = 0; i < tempMarkers.length; i++) {
            map.removeLayer(tempMarkers[i]);
        }
        tempMarkers = [];
        if (tempLine) {
            map.removeLayer(tempLine);
            tempLine = null;
        }
    }

    // --- Display ---

    function showPolygon() {
        clearMapDisplay();
        if (points.length < 3) return;

        var map = FlyMap.getMap();
        if (!map) return;

        var latlngs = points.map(function (p) { return [p.lat, p.lon]; });
        polygon = L.polygon(latlngs, {
            color: '#f44336',
            weight: 2,
            opacity: 0.7,
            fillColor: '#f44336',
            fillOpacity: 0.1,
            dashArray: '8,4',
        }).addTo(map);
    }

    function clearMapDisplay() {
        var map = FlyMap.getMap();
        if (map && polygon) {
            map.removeLayer(polygon);
            polygon = null;
        }
    }

    // --- Upload to vehicle ---

    function upload() {
        if (points.length < 3) {
            meridian.log('No geofence to upload', 'warn');
            return;
        }
        for (var i = 0; i < points.length; i++) {
            Connection.send(MAVLink.encodeFencePoint(i, points.length, points[i].lat, points[i].lon));
        }
        meridian.log('Geofence uploaded (' + points.length + ' points)', 'info');
    }

    // --- Clear ---

    function clearFence() {
        points = [];
        clearMapDisplay();
        meridian.log('Geofence cleared', 'info');
        meridian.events.emit('geofence_change', points);
    }

    // --- FlyMap integration: updateGeofence for external use ---

    function updateGeofence(newPoints) {
        points = newPoints || [];
        if (points.length >= 3) {
            showPolygon();
        } else {
            clearMapDisplay();
        }
    }

    // --- Circle Geofence ---

    let circleGeofence = null; // { lat, lon, radius }
    let circleLayer = null;

    function setCircle(lat, lon, radius) {
        circleGeofence = { lat: lat, lon: lon, radius: radius };
        var map = FlyMap.getMap();
        if (!map) return;
        if (circleLayer) map.removeLayer(circleLayer);
        circleLayer = L.circle([lat, lon], {
            radius: radius,
            color: '#f44336',
            weight: 2,
            fillColor: '#f44336',
            fillOpacity: 0.08,
            dashArray: '8,4',
        }).addTo(map);
    }

    function getCircle() { return circleGeofence; }

    // --- Getters ---

    function getPoints() { return points; }
    function isDrawing() { return drawing; }
    function hasGeofence() { return points.length >= 3 || circleGeofence !== null; }

    return {
        startDrawing: startDrawing,
        finishDrawing: finishDrawing,
        cancelDrawing: cancelDrawing,
        clearFence: clearFence,
        setCircle: setCircle,
        getCircle: getCircle,
        upload: upload,
        updateGeofence: updateGeofence,
        showPolygon: showPolygon,
        getPoints: getPoints,
        isDrawing: isDrawing,
        hasGeofence: hasGeofence,
    };

})();
