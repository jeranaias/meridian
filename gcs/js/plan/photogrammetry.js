/* ============================================================
   photogrammetry.js — Post-flight Photogrammetry Integration
   T3-16: Export helpers for Pix4D / ODM / Metashape.
   - Export mission as GeoJSON + photo timestamps from tlog
   - GCP editor: click map to place Ground Control Points
   - GCP export as CSV
   ============================================================ */

'use strict';

window.Photogrammetry = (function () {

    var _gcpMarkers = [];  // Array of { marker, lat, lon, alt, name, measuredLat, measuredLon, measuredAlt }
    var _active = false;   // GCP placement mode active

    // -------------------------------------------------------------------------
    // Export for Processing (called from Logs panel)
    // -------------------------------------------------------------------------

    function exportForProcessing() {
        var v = meridian.v;
        var items = window.Mission ? Mission.getItems() : [];

        if (items.length === 0) {
            meridian.log('No mission waypoints to export', 'warn');
        }

        // --- GeoJSON mission export ---
        var geojson = _missionToGeoJSON(items);
        var geojsonStr = JSON.stringify(geojson, null, 2);
        var ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        _downloadText('mission_' + ts + '.geojson', geojsonStr, 'application/geo+json');
        meridian.log('Exported mission GeoJSON (' + items.length + ' waypoints)', 'info');

        // --- Photo trigger timestamps (from tlog) ---
        var triggerTimes = _extractPhotoTriggers();
        if (triggerTimes.length > 0) {
            var csv = 'timestamp_ms,lat,lon,alt\n' +
                triggerTimes.map(function (t) {
                    return t.ts + ',' + t.lat + ',' + t.lon + ',' + t.alt;
                }).join('\n');
            _downloadText('photo_triggers_' + ts + '.csv', csv, 'text/csv');
            meridian.log('Exported ' + triggerTimes.length + ' photo trigger timestamps', 'info');
        } else {
            meridian.log('No photo trigger events found in tlog', 'info');
        }

        // --- Camera metadata from survey settings ---
        var meta = _buildCameraMetadata();
        if (meta) {
            _downloadText('camera_metadata_' + ts + '.json', JSON.stringify(meta, null, 2), 'application/json');
        }

        _showIntegrationInstructions();
    }

    // -------------------------------------------------------------------------
    // GeoJSON conversion
    // -------------------------------------------------------------------------

    function _missionToGeoJSON(items) {
        var features = items.map(function (wp, i) {
            return {
                type: 'Feature',
                geometry: {
                    type: 'Point',
                    coordinates: [wp.lon, wp.lat, wp.alt],
                },
                properties: {
                    seq:     i + 1,
                    command: wp.command,
                    name:    (window.Mission ? Mission.cmdName(wp.command) : ('CMD_' + wp.command)),
                    alt:     wp.alt,
                },
            };
        });

        // Also include the path as a LineString if > 1 waypoint
        var lineFeature = null;
        if (items.length > 1) {
            lineFeature = {
                type: 'Feature',
                geometry: {
                    type: 'LineString',
                    coordinates: items.map(function (wp) {
                        return [wp.lon, wp.lat, wp.alt];
                    }),
                },
                properties: { name: 'Mission Path' },
            };
        }

        return {
            type: 'FeatureCollection',
            features: lineFeature ? [lineFeature].concat(features) : features,
            properties: {
                exported:  new Date().toISOString(),
                source:    'Meridian GCS',
                waypointCount: items.length,
            },
        };
    }

    // -------------------------------------------------------------------------
    // Photo trigger extraction from tlog
    // -------------------------------------------------------------------------

    function _extractPhotoTriggers() {
        // We look for CAMERA_FEEDBACK (msg 180) or DO_DIGICAM_CONTROL events
        // in the in-memory tlog ring buffer if available.
        var triggers = [];

        if (!meridian.tlogBuffer) return triggers;  // no tlog in memory

        meridian.tlogBuffer.forEach(function (entry) {
            if (entry.msgId === 180) {  // CAMERA_FEEDBACK
                triggers.push({
                    ts:  entry.ts || 0,
                    lat: (entry.lat || 0) / 1e7,
                    lon: (entry.lng || 0) / 1e7,
                    alt: (entry.alt_gps || entry.alt || 0),
                });
            }
        });

        return triggers;
    }

    // -------------------------------------------------------------------------
    // Camera metadata
    // -------------------------------------------------------------------------

    function _buildCameraMetadata() {
        var s = meridian.settings;
        var survey = s.lastSurveySettings || null;

        return {
            source:        'Meridian GCS',
            exportedAt:    new Date().toISOString(),
            survey:        survey || {},
            vehicle: {
                lat:       meridian.v ? meridian.v.lat : null,
                lon:       meridian.v ? meridian.v.lon : null,
                alt:       meridian.v ? meridian.v.alt : null,
            },
        };
    }

    // -------------------------------------------------------------------------
    // Integration instructions panel
    // -------------------------------------------------------------------------

    function _showIntegrationInstructions() {
        var panel = document.getElementById('photogrammetry-instructions');
        if (panel) {
            panel.style.display = 'block';
            return;
        }

        var overlay = document.createElement('div');
        overlay.id = 'photogrammetry-instructions';
        overlay.className = 'photogrammetry-overlay';
        overlay.innerHTML =
            '<div class="photogrammetry-header">' +
            '<span class="photogrammetry-title">Post-Flight Processing Integration</span>' +
            '<button class="panel-close" onclick="this.closest(\'#photogrammetry-instructions\').style.display=\'none\'">&times;</button>' +
            '</div>' +
            '<div class="photogrammetry-body">' +
            _buildInstructionsHTML() +
            '</div>';

        document.body.appendChild(overlay);
    }

    function _buildInstructionsHTML() {
        return '' +
            '<div class="phg-section">' +
            '<div class="phg-app-title">Pix4Dmapper / Pix4Dmatic</div>' +
            '<ol class="phg-steps">' +
            '<li>New Project &rarr; <em>New Flights</em> &rarr; import photos from SD card.</li>' +
            '<li>Add <code>.geojson</code> mission file as reference for area extent.</li>' +
            '<li>Import GCP file: <em>GCP/MTP Manager</em> &rarr; Import CSV &rarr; select <code>gcps_*.csv</code>.</li>' +
            '<li>Mark GCPs on at least 3 images each, then run initial processing.</li>' +
            '</ol>' +
            '</div>' +
            '<div class="phg-section">' +
            '<div class="phg-app-title">OpenDroneMap (ODM / WebODM)</div>' +
            '<ol class="phg-steps">' +
            '<li>Upload photos + optional <code>gcp_list.txt</code> (rename exported CSV, add OPK header row).</li>' +
            '<li>Set <em>GPS CRS</em> to WGS84 (EPSG:4326).</li>' +
            '<li>Enable <em>Use Existing GPS Exif</em> if photos have embedded GPS.</li>' +
            '<li>Run task; orthophoto and point cloud output to <code>/output</code> directory.</li>' +
            '</ol>' +
            '</div>' +
            '<div class="phg-section">' +
            '<div class="phg-app-title">Agisoft Metashape</div>' +
            '<ol class="phg-steps">' +
            '<li>Add Photos &rarr; Align Photos (High accuracy).</li>' +
            '<li>Reference pane &rarr; Import GCPs &rarr; select CSV, column order: Label, Lon, Lat, Alt.</li>' +
            '<li>Mark markers on images, then Optimize Cameras.</li>' +
            '<li>Build Dense Cloud &rarr; Build Orthomosaic.</li>' +
            '</ol>' +
            '</div>' +
            '<div class="phg-gcp-format">' +
            '<strong>GCP CSV format (exported by Meridian):</strong><br>' +
            '<code>name,lat,lon,alt_m,measured_lat,measured_lon,measured_alt_m</code>' +
            '</div>';
    }

    // -------------------------------------------------------------------------
    // Render in Logs panel (Export button row)
    // -------------------------------------------------------------------------

    function renderLogsSection(container) {
        var section = document.createElement('div');
        section.className = 'photogrammetry-logs-section';

        var title = document.createElement('div');
        title.className = 'logs-section-title';
        title.textContent = 'Photogrammetry Export';
        section.appendChild(title);

        var desc = document.createElement('div');
        desc.className = 'photogrammetry-desc';
        desc.textContent = 'Export mission data for post-flight processing with Pix4D, ODM, or Metashape.';
        section.appendChild(desc);

        var btnRow = document.createElement('div');
        btnRow.className = 'photogrammetry-btn-row';

        var exportBtn = document.createElement('button');
        exportBtn.className = 'offline-btn download';
        exportBtn.textContent = '\u21E9 Export for Processing';
        exportBtn.addEventListener('click', exportForProcessing);

        var gcpBtn = document.createElement('button');
        gcpBtn.className = 'offline-btn draw';
        gcpBtn.id = 'btn-gcp-editor';
        gcpBtn.textContent = '\u25CE Place GCPs on Map';
        gcpBtn.addEventListener('click', function () {
            if (_active) {
                stopGcpPlacement();
                gcpBtn.textContent = '\u25CE Place GCPs on Map';
            } else {
                startGcpPlacement();
                gcpBtn.textContent = '\u2714 Done Placing GCPs';
            }
        });

        var gcpExportBtn = document.createElement('button');
        gcpExportBtn.className = 'offline-btn draw';
        gcpExportBtn.textContent = '\u21E9 Export GCPs CSV';
        gcpExportBtn.addEventListener('click', exportGcpCsv);

        btnRow.appendChild(exportBtn);
        btnRow.appendChild(gcpBtn);
        btnRow.appendChild(gcpExportBtn);
        section.appendChild(btnRow);

        // GCP list
        var gcpList = document.createElement('div');
        gcpList.id = 'photogrammetry-gcp-list';
        gcpList.className = 'photogrammetry-gcp-list';
        section.appendChild(gcpList);

        _renderGcpList(gcpList);

        container.appendChild(section);
    }

    // -------------------------------------------------------------------------
    // GCP placement on map
    // -------------------------------------------------------------------------

    function startGcpPlacement() {
        _active = true;
        var map = window.FlyMap ? FlyMap.getMap() : null;
        if (!map) { meridian.log('Map not available for GCP placement', 'warn'); return; }

        map.getContainer().style.cursor = 'crosshair';
        map._gcpClickHandler = function (e) {
            _addGcp(e.latlng.lat, e.latlng.lng, 0);
        };
        map.on('click', map._gcpClickHandler);
        meridian.log('GCP placement mode active — click map to place points', 'info');
    }

    function stopGcpPlacement() {
        _active = false;
        var map = window.FlyMap ? FlyMap.getMap() : null;
        if (!map) return;
        map.getContainer().style.cursor = '';
        if (map._gcpClickHandler) {
            map.off('click', map._gcpClickHandler);
            map._gcpClickHandler = null;
        }
    }

    function _addGcp(lat, lon, alt) {
        var idx = _gcpMarkers.length + 1;
        var name = 'GCP' + idx;

        var map = window.FlyMap ? FlyMap.getMap() : null;
        var marker = null;
        if (map && window.L) {
            var icon = L.divIcon({
                className: 'gcp-marker',
                html: '<div class="gcp-dot">' + idx + '</div>',
                iconSize: [24, 24],
                iconAnchor: [12, 12],
            });
            marker = L.marker([lat, lon], { icon: icon }).addTo(map);
            marker.bindPopup('<b>' + name + '</b><br>' + lat.toFixed(6) + ', ' + lon.toFixed(6));
        }

        var gcp = { name: name, lat: lat, lon: lon, alt: alt,
                    measuredLat: lat, measuredLon: lon, measuredAlt: alt,
                    marker: marker };
        _gcpMarkers.push(gcp);

        _renderGcpList(document.getElementById('photogrammetry-gcp-list'));
        meridian.log('GCP placed: ' + name + ' at ' + lat.toFixed(6) + ', ' + lon.toFixed(6), 'info');
    }

    // -------------------------------------------------------------------------
    // GCP list rendering
    // -------------------------------------------------------------------------

    function _renderGcpList(container) {
        if (!container) return;
        container.innerHTML = '';

        if (_gcpMarkers.length === 0) {
            container.innerHTML = '<div class="photogrammetry-gcp-empty">No GCPs placed. Click map in GCP mode to add.</div>';
            return;
        }

        var table = document.createElement('table');
        table.className = 'photogrammetry-gcp-table';
        table.innerHTML =
            '<thead><tr>' +
            '<th>Name</th><th>Map Lat</th><th>Map Lon</th>' +
            '<th>Meas. Lat</th><th>Meas. Lon</th><th>Meas. Alt</th><th></th>' +
            '</tr></thead>';

        var tbody = document.createElement('tbody');
        _gcpMarkers.forEach(function (gcp, i) {
            var tr = document.createElement('tr');
            tr.innerHTML =
                '<td><input class="gcp-name-input" value="' + _esc(gcp.name) + '" data-gcpidx="' + i + '" data-field="name"></td>' +
                '<td>' + gcp.lat.toFixed(6) + '</td>' +
                '<td>' + gcp.lon.toFixed(6) + '</td>' +
                '<td><input class="gcp-coord-input" value="' + gcp.measuredLat.toFixed(6) + '" data-gcpidx="' + i + '" data-field="measuredLat"></td>' +
                '<td><input class="gcp-coord-input" value="' + gcp.measuredLon.toFixed(6) + '" data-gcpidx="' + i + '" data-field="measuredLon"></td>' +
                '<td><input class="gcp-coord-input" value="' + gcp.measuredAlt.toFixed(2) + '" data-gcpidx="' + i + '" data-field="measuredAlt"></td>' +
                '<td><button class="gcp-delete-btn" data-gcpidx="' + i + '">\u2715</button></td>';
            tbody.appendChild(tr);
        });

        table.appendChild(tbody);
        container.appendChild(table);

        // Wire input changes
        container.querySelectorAll('[data-gcpidx]').forEach(function (el) {
            el.addEventListener('change', function () {
                var idx = parseInt(el.dataset.gcpidx, 10);
                var field = el.dataset.field;
                var gcp = _gcpMarkers[idx];
                if (!gcp) return;
                if (field === 'name') {
                    gcp.name = el.value;
                } else {
                    gcp[field] = parseFloat(el.value) || 0;
                }
            });
        });

        // Wire delete buttons
        container.querySelectorAll('.gcp-delete-btn').forEach(function (btn) {
            btn.addEventListener('click', function () {
                var idx = parseInt(btn.dataset.gcpidx, 10);
                _removeGcp(idx, container);
            });
        });
    }

    function _removeGcp(idx, container) {
        var gcp = _gcpMarkers[idx];
        if (!gcp) return;
        if (gcp.marker && window.FlyMap) {
            var map = FlyMap.getMap();
            if (map) map.removeLayer(gcp.marker);
        }
        _gcpMarkers.splice(idx, 1);
        _renderGcpList(container || document.getElementById('photogrammetry-gcp-list'));
    }

    // -------------------------------------------------------------------------
    // GCP CSV export
    // -------------------------------------------------------------------------

    function exportGcpCsv() {
        if (_gcpMarkers.length === 0) {
            meridian.log('No GCPs to export', 'warn');
            return;
        }

        var lines = ['name,lat,lon,alt_m,measured_lat,measured_lon,measured_alt_m'];
        _gcpMarkers.forEach(function (gcp) {
            lines.push([
                gcp.name,
                gcp.lat.toFixed(8),
                gcp.lon.toFixed(8),
                gcp.alt.toFixed(3),
                gcp.measuredLat.toFixed(8),
                gcp.measuredLon.toFixed(8),
                gcp.measuredAlt.toFixed(3),
            ].join(','));
        });

        var ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        _downloadText('gcps_' + ts + '.csv', lines.join('\n'), 'text/csv');
        meridian.log('Exported ' + _gcpMarkers.length + ' GCPs to CSV', 'info');
    }

    // -------------------------------------------------------------------------
    // Plan view toolbar button (GCP placement shortcut)
    // -------------------------------------------------------------------------

    function addPlanToolbarButton(toolbar) {
        var gcpBtn = document.createElement('button');
        gcpBtn.id = 'btn-plan-gcp';
        gcpBtn.title = 'Ground Control Points — place and export GCPs';
        gcpBtn.innerHTML = '\u25CE GCPs';
        gcpBtn.addEventListener('click', function () {
            if (_active) {
                stopGcpPlacement();
                gcpBtn.textContent = '\u25CE GCPs';
                gcpBtn.classList.remove('active');
            } else {
                startGcpPlacement();
                gcpBtn.textContent = '\u2714 Done';
                gcpBtn.classList.add('active');
            }
        });
        toolbar.appendChild(gcpBtn);
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    function _downloadText(filename, text, mime) {
        var blob = new Blob([text], { type: mime || 'text/plain' });
        var url = URL.createObjectURL(blob);
        var a = document.createElement('a');
        a.href = url;
        a.download = filename;
        a.click();
        URL.revokeObjectURL(url);
    }

    function _esc(str) {
        return String(str).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
    }

    return {
        exportForProcessing,
        exportGcpCsv,
        startGcpPlacement,
        stopGcpPlacement,
        renderLogsSection,
        addPlanToolbarButton,
    };

})();
