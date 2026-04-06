/* ============================================================
   plan-view.js — Plan overlay orchestrator
   When Plan view activates: render mission sidebar, enable
   map click-to-add-waypoint. Deactivates: disable editing.
   Renders mission list with drag-to-reorder in panel body.
   ============================================================ */

'use strict';

window.PlanView = (function () {

    var active = false;
    var panelBody = null;
    var listEl = null;
    var editorEl = null;
    var statsEl = null;

    // ─── UI Helpers ─────────────────────────────────────────
    function sectionHeader(text) {
        var el = document.createElement('div');
        el.className = 'plan-section-header';
        el.textContent = text;
        return el;
    }

    function buttonRow(buttons) {
        var row = document.createElement('div');
        row.className = 'plan-btn-row';
        buttons.forEach(function (b) {
            var btn = document.createElement('button');
            btn.type = 'button';
            btn.id = b.id;
            btn.className = 'plan-btn' + (b.cls ? ' ' + b.cls : '');
            btn.title = b.title || '';
            btn.innerHTML = '<span class="plan-btn-icon">' + (b.icon || '') + '</span>' +
                            '<span class="plan-btn-label">' + b.label + '</span>';
            row.appendChild(btn);
        });
        return row;
    }

    function updateFenceStatus() {
        var el = document.getElementById('fence-status');
        if (!el) return;
        if (window.Geofence && Geofence.getPoints && Geofence.getPoints().length > 0) {
            el.textContent = 'Polygon: ' + Geofence.getPoints().length + ' vertices';
        } else if (window.Geofence && Geofence.getCircle && Geofence.getCircle()) {
            var c = Geofence.getCircle();
            el.textContent = 'Circle: ' + c.radius.toFixed(0) + 'm radius';
        } else {
            el.textContent = 'No geofence defined';
        }
    }

    // Drag state
    var dragIdx = -1;
    var dragOverIdx = -1;

    function init() {
        // Listen for panel changes
        meridian.events.on('panel_change', onPanelChange);

        // Listen for mission changes to re-render list
        meridian.events.on('mission_change', renderList);
        meridian.events.on('mission_select', renderList);
        meridian.events.on('mission_dirty', updateUploadBtn);

        // Listen for mission download responses
        meridian.events.on('mission_count', onMissionCount);
        meridian.events.on('mission_item', onMissionItem);
        meridian.events.on('mission_ack', onMissionAck);
    }

    function onPanelChange(panelId) {
        var wasActive = active;
        active = (panelId === 'plan');

        if (active && !wasActive) {
            activate();
        } else if (!active && wasActive) {
            deactivate();
        }
    }

    function activate() {
        panelBody = document.querySelector('#panel-plan .panel-body');
        if (!panelBody) return;

        panelBody.innerHTML = '';

        // ─── MISSION ─────────────────────────────────────────
        panelBody.appendChild(sectionHeader('Mission'));

        // Sync row: Upload + Download
        panelBody.appendChild(buttonRow([
            { id: 'btn-mission-upload', label: 'Upload', icon: '\u2B06', cls: 'upload-btn', title: 'Upload mission to vehicle' },
            { id: 'btn-mission-download', label: 'Download', icon: '\u2B07', title: 'Download mission from vehicle' },
        ]));

        // Tools row: Survey + Corridor + Orbit
        panelBody.appendChild(buttonRow([
            { id: 'btn-survey', label: 'Survey', icon: '\u25A8', title: 'Polygon survey tool' },
            { id: 'btn-corridor', label: 'Corridor', icon: '\u2550', title: 'Corridor scan tool' },
            { id: 'btn-orbit', label: 'Orbit', icon: '\u25CB', title: 'Orbit / circle mission tool' },
        ]));

        // File row: Export + Import + Clear
        panelBody.appendChild(buttonRow([
            { id: 'btn-wp-export', label: 'Export', icon: '\u21E9', title: 'Export .waypoints file' },
            { id: 'btn-wp-import', label: 'Import', icon: '\u21E7', title: 'Import .waypoints file' },
            { id: 'btn-mission-clear', label: 'Clear', icon: '\u2715', cls: 'clear-btn', title: 'Clear all waypoints' },
        ]));

        // Waypoint list
        listEl = document.createElement('div');
        listEl.className = 'mission-list';
        panelBody.appendChild(listEl);

        // Waypoint editor (shown when a WP is selected)
        editorEl = document.createElement('div');
        editorEl.className = 'wp-editor';
        editorEl.style.display = 'none';
        panelBody.appendChild(editorEl);

        // Survey panel (expands inline when survey tool active)
        var surveyEl = document.createElement('div');
        surveyEl.className = 'survey-container';
        surveyEl.style.display = 'none';
        panelBody.appendChild(surveyEl);
        if (window.Survey) Survey.setPanelEl(surveyEl);

        // Stats + terrain profile
        statsEl = document.createElement('div');
        statsEl.className = 'plan-stats-bar';
        panelBody.appendChild(statsEl);

        WpEditor.init(editorEl);
        PlanStats.init(statsEl);
        if (window.TerrainProfile) TerrainProfile.init(statsEl);

        // ─── GEOFENCE ────────────────────────────────────────
        panelBody.appendChild(sectionHeader('Geofence'));

        var fenceDesc = document.createElement('div');
        fenceDesc.className = 'plan-section-desc';
        fenceDesc.textContent = 'Define a boundary the vehicle must stay within. Draw a polygon or set a circular radius.';
        panelBody.appendChild(fenceDesc);

        panelBody.appendChild(buttonRow([
            { id: 'btn-geofence', label: 'Draw Polygon', icon: '\u2B1F', title: 'Click map points to draw a geofence polygon' },
            { id: 'btn-fence-circle', label: 'Circle', icon: '\u25EF', title: 'Set a circular geofence radius' },
            { id: 'btn-clear-fence', label: 'Clear', icon: '\u2715', cls: 'clear-btn', title: 'Remove all geofence boundaries' },
        ]));

        // Fence status display
        var fenceStatus = document.createElement('div');
        fenceStatus.id = 'fence-status';
        fenceStatus.className = 'plan-section-desc';
        fenceStatus.style.fontFamily = 'var(--f-mono)';
        fenceStatus.style.fontSize = '11px';
        fenceStatus.textContent = 'No geofence defined';
        panelBody.appendChild(fenceStatus);

        // ─── INSPECTION ──────────────────────────────────────
        if (window.InspectionTool) {
            panelBody.appendChild(sectionHeader('Inspection'));
            InspectionTool.initForPlanPanel(panelBody);
        }

        // Wire toolbar buttons
        document.getElementById('btn-mission-upload').addEventListener('click', onUpload);
        document.getElementById('btn-mission-download').addEventListener('click', onDownload);
        document.getElementById('btn-mission-clear').addEventListener('click', onClear);

        // Geofence buttons
        var geofenceBtn = document.getElementById('btn-geofence');
        if (geofenceBtn) {
            geofenceBtn.addEventListener('click', function () {
                if (window.Geofence) {
                    var label = geofenceBtn.querySelector('.plan-btn-label');
                    if (Geofence.isDrawing()) {
                        Geofence.finishDrawing();
                        if (label) label.textContent = 'Draw Polygon';
                        updateFenceStatus();
                    } else {
                        Geofence.startDrawing();
                        if (label) label.textContent = 'Finish';
                    }
                }
            });
        }
        var clearFenceBtn = document.getElementById('btn-clear-fence');
        if (clearFenceBtn) {
            clearFenceBtn.addEventListener('click', function () {
                if (window.Geofence) Geofence.clearFence();
                updateFenceStatus();
            });
        }

        // Circle geofence
        var circleFenceBtn = document.getElementById('btn-fence-circle');
        if (circleFenceBtn) {
            circleFenceBtn.addEventListener('click', async function () {
                var radius = await Modal.prompt('Circle Geofence', 'Enter radius in meters:', '200');
                if (radius && parseFloat(radius) > 0) {
                    var v = meridian.v;
                    if (v && v.homeLat) {
                        if (window.Geofence) {
                            Geofence.setCircle(v.homeLat, v.homeLon, parseFloat(radius));
                            meridian.log('Circle geofence set: ' + radius + 'm radius', 'info');
                            updateFenceStatus();
                        }
                    } else {
                        meridian.log('Set home position first', 'warn');
                    }
                }
            });
        }

        // T1-8: Wire waypoints import/export
        var exportBtn = document.getElementById('btn-wp-export');
        if (exportBtn) {
            exportBtn.addEventListener('click', function () {
                if (Mission.count() === 0) {
                    meridian.log('No waypoints to export', 'warn');
                    return;
                }
                var text = Mission.exportWaypoints();
                var filename = 'mission_' + new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19) + '.waypoints';
                Mission.downloadFile(filename, text);
                meridian.log('Exported ' + Mission.count() + ' waypoints to ' + filename, 'info');
            });
        }

        var importBtn = document.getElementById('btn-wp-import');
        if (importBtn) {
            importBtn.addEventListener('click', function () {
                var input = document.createElement('input');
                input.type = 'file';
                input.accept = '.waypoints,.txt';
                input.addEventListener('change', function () {
                    if (!input.files || !input.files[0]) return;
                    var reader = new FileReader();
                    reader.onload = function (ev) {
                        Mission.importWaypoints(ev.target.result);
                    };
                    reader.readAsText(input.files[0]);
                });
                input.click();
            });
        }

        var surveyBtn = document.getElementById('btn-survey');
        if (surveyBtn) {
            surveyBtn.addEventListener('click', function () {
                if (window.Survey) {
                    if (Survey.isActive()) { Survey.cancelSurvey(); }
                    else { Survey.startPolygonSurvey(); }
                }
            });
        }

        var corridorBtn = document.getElementById('btn-corridor');
        if (corridorBtn) {
            corridorBtn.addEventListener('click', function () {
                if (window.Survey) {
                    if (Survey.isActive()) { Survey.cancelSurvey(); }
                    else { Survey.startCorridorScan(); }
                }
            });
        }

        // T2-10: Orbit tool
        var orbitBtn = document.getElementById('btn-orbit');
        if (orbitBtn) {
            orbitBtn.addEventListener('click', function () {
                if (window.OrbitTool) OrbitTool.activate();
            });
        }

        // T3-16: Photogrammetry GCP placement (handled via inspection section)

        // Enable map editing
        FlyMap.enablePlanMode(true);

        // Update dirty state on upload button
        updateUploadBtn();

        // Render current mission
        renderList();
    }

    function deactivate() {
        // Disable map editing
        FlyMap.enablePlanMode(false);
        active = false;
    }

    // --- Toolbar Actions ---

    function onUpload() {
        if (Mission.count() === 0) {
            meridian.log('No waypoints to upload', 'warn');
            return;
        }
        Mission.upload();
    }

    function onDownload() {
        Mission.download();
    }

    async function onClear() {
        if (Mission.count() === 0) return;
        var ok = await Modal.confirm('Clear Mission',
            'Remove all ' + Mission.count() + ' waypoints?', 'Clear All', true);
        if (ok) {
            Mission.clearAll();
            FlyMap.clearMissionMarkers();
        }
    }

    function updateUploadBtn() {
        var btn = document.getElementById('btn-mission-upload');
        if (!btn) return;
        btn.classList.toggle('dirty', Mission.isDirty());
    }

    // --- Mission List Rendering ---

    function renderList() {
        if (!listEl || !active) return;

        var items = Mission.getItems();
        var selIdx = Mission.getSelectedIdx();

        if (items.length === 0) {
            listEl.innerHTML =
                '<div class="mission-list-empty">' +
                '<div class="empty-icon">\u2691</div>' +
                '<div class="empty-text">No Waypoints</div>' +
                '<div class="empty-hint">Click on the map to add waypoints</div>' +
                '</div>';

            // Update map
            FlyMap.updateMissionMarkers([], -1);
            return;
        }

        var html = '';
        for (var i = 0; i < items.length; i++) {
            var it = items[i];
            var sel = (i === selIdx) ? ' selected' : '';
            var seqClass = Mission.cmdClass(it.command);

            html += '<div class="mission-item' + sel + '" data-idx="' + i + '" draggable="true">';
            html += '<div class="mi-drag" title="Drag to reorder">\u2630</div>';
            html += '<div class="mi-seq ' + seqClass + '">' + (i + 1) + '</div>';
            html += '<div class="mi-body">';
            html += '<div class="mi-cmd">' + Mission.cmdName(it.command) + '</div>';
            html += '<div class="mi-coords">' + it.lat.toFixed(6) + ', ' + it.lon.toFixed(6) + '</div>';
            html += '</div>';
            html += '<div class="mi-alt">' + it.alt.toFixed(0) + 'm</div>';
            html += '<button class="mi-delete" data-delidx="' + i + '" title="Delete waypoint">\u2715</button>';
            html += '</div>';
        }

        listEl.innerHTML = html;

        // Wire events
        var rows = listEl.querySelectorAll('.mission-item');
        for (var r = 0; r < rows.length; r++) {
            wireRow(rows[r]);
        }

        // Wire delete buttons
        var delBtns = listEl.querySelectorAll('.mi-delete');
        for (var d = 0; d < delBtns.length; d++) {
            wireDelete(delBtns[d]);
        }

        // Update map markers
        FlyMap.updateMissionMarkers(items, selIdx);
    }

    function wireRow(row) {
        var idx = parseInt(row.dataset.idx, 10);

        // Click to select
        row.addEventListener('click', function (e) {
            if (e.target.classList.contains('mi-delete') || e.target.classList.contains('mi-drag')) return;
            Mission.select(idx);
        });

        // Drag start
        row.addEventListener('dragstart', function (e) {
            dragIdx = idx;
            row.classList.add('dragging');
            e.dataTransfer.effectAllowed = 'move';
            e.dataTransfer.setData('text/plain', '' + idx);
        });

        row.addEventListener('dragend', function () {
            row.classList.remove('dragging');
            clearDragOver();
            dragIdx = -1;
        });

        // Drop target
        row.addEventListener('dragover', function (e) {
            e.preventDefault();
            e.dataTransfer.dropEffect = 'move';
            if (dragOverIdx !== idx) {
                clearDragOver();
                row.classList.add('drag-over');
                dragOverIdx = idx;
            }
        });

        row.addEventListener('dragleave', function () {
            row.classList.remove('drag-over');
        });

        row.addEventListener('drop', function (e) {
            e.preventDefault();
            clearDragOver();
            var fromIdx = parseInt(e.dataTransfer.getData('text/plain'), 10);
            if (!isNaN(fromIdx) && fromIdx !== idx) {
                Mission.moveItem(fromIdx, idx);
            }
        });
    }

    function wireDelete(btn) {
        btn.addEventListener('click', function (e) {
            e.stopPropagation();
            var idx = parseInt(btn.dataset.delidx, 10);
            Mission.removeAt(idx);
        });
    }

    function clearDragOver() {
        var overs = listEl ? listEl.querySelectorAll('.drag-over') : [];
        for (var i = 0; i < overs.length; i++) {
            overs[i].classList.remove('drag-over');
        }
        dragOverIdx = -1;
    }

    // --- Mission Download/Upload Handling ---
    // T1-1: Connection.js now drives the state machine for upload/download.
    // These handlers update the UI in response to events.

    function onMissionCount(msg) {
        // Connection.js handles the download protocol
    }

    function onMissionItem(msg) {
        // Connection.js handles item collection and calls Mission.setItems()
    }

    function onMissionAck(msg) {
        if (msg.result === 0) {
            meridian.log('Mission accepted by vehicle', 'info');
        } else {
            meridian.log('Mission rejected (error ' + msg.result + ')', 'error');
        }
    }

    return { init: init };

})();
