/* ============================================================
   inspection.js — Inspection Defect Annotation
   T3-6: Click map to place numbered defect markers with
   severity, description, photo reference.
   Defect list in plan panel sidebar.
   Export as JSON or CSV.
   ============================================================ */

'use strict';

window.InspectionTool = (function () {

    // --- State ---
    let defects = [];       // Array of { id, lat, lon, severity, description, photoRef }
    let nextId = 1;
    let active = false;
    let markerLayer = [];   // Leaflet marker objects, index = defect array index
    let panelEl = null;     // The panel container
    let inspectionMode = false; // Click-to-place mode

    const SEVERITY_COLORS = {
        low:      '#22c55e',  // green
        medium:   '#f59e0b',  // amber
        high:     '#f97316',  // orange
        critical: '#ef4444',  // red
    };

    const SEVERITY_LABELS = {
        low:      'LOW',
        medium:   'MED',
        high:     'HIGH',
        critical: 'CRIT',
    };

    // --- Map helpers ---
    function getMap() {
        return window.FlyMap ? FlyMap.getMap() : null;
    }

    // --- Create Leaflet marker for a defect ---
    function makeDefectIcon(defect) {
        const color = SEVERITY_COLORS[defect.severity] || SEVERITY_COLORS.medium;
        const sev = SEVERITY_LABELS[defect.severity] || defect.severity.toUpperCase();
        return L.divIcon({
            className: 'defect-marker-icon',
            html: `<div style="
                width:28px;height:28px;border-radius:50%;
                background:${color};
                border:2px solid rgba(255,255,255,0.7);
                display:flex;align-items:center;justify-content:center;
                font-family:'DM Mono',monospace;font-size:10px;font-weight:700;
                color:#000;box-shadow:0 1px 4px rgba(0,0,0,0.5);
                cursor:pointer;
            ">${defect.id}</div>`,
            iconSize: [28, 28],
            iconAnchor: [14, 14],
        });
    }

    function addMarkerToMap(defect) {
        const map = getMap();
        if (!map) return null;

        const marker = L.marker([defect.lat, defect.lon], {
            icon: makeDefectIcon(defect),
            zIndexOffset: 700,
            draggable: false,
        });

        marker.bindPopup(buildPopupHtml(defect), { maxWidth: 260 });
        marker.addTo(map);
        return marker;
    }

    function buildPopupHtml(defect) {
        const color = SEVERITY_COLORS[defect.severity] || SEVERITY_COLORS.medium;
        return `
            <div style="font-family:'DM Mono',monospace;font-size:12px;min-width:180px">
                <div style="font-weight:700;font-size:13px;margin-bottom:4px">
                    Defect #${defect.id}
                    <span style="background:${color};color:#000;padding:1px 5px;border-radius:3px;font-size:10px;margin-left:6px">${SEVERITY_LABELS[defect.severity]}</span>
                </div>
                <div style="color:var(--c-neutral);margin-bottom:3px">${defect.lat.toFixed(6)}, ${defect.lon.toFixed(6)}</div>
                <div style="margin-bottom:3px"><strong>Desc:</strong> ${defect.description || '<em>none</em>'}</div>
                <div><strong>Photo:</strong> ${defect.photoRef || '<em>none</em>'}</div>
                <button onclick="InspectionTool.removeDefect(${defect.id})" style="margin-top:8px;padding:4px 10px;min-height:28px;background:var(--c-emergency);color:#fff;border:none;border-radius:3px;cursor:pointer;font-size:11px">Remove</button>
            </div>`;
    }

    // --- Add a defect ---
    function addDefect(lat, lon) {
        const defect = {
            id: nextId++,
            lat,
            lon,
            severity: 'medium',
            description: '',
            photoRef: '',
        };
        defects.push(defect);
        const marker = addMarkerToMap(defect);
        markerLayer.push(marker);

        meridian.log('Defect #' + defect.id + ' placed at ' + lat.toFixed(5) + ', ' + lon.toFixed(5), 'info');
        renderList();
        return defect;
    }

    // --- Remove a defect ---
    function removeDefect(id) {
        const idx = defects.findIndex(d => d.id === id);
        if (idx < 0) return;

        const map = getMap();
        if (map && markerLayer[idx]) map.removeLayer(markerLayer[idx]);
        markerLayer.splice(idx, 1);
        defects.splice(idx, 1);

        renderList();
        meridian.log('Defect #' + id + ' removed', 'info');
    }

    // --- Update defect field ---
    function updateDefect(id, field, value) {
        const defect = defects.find(d => d.id === id);
        if (!defect) return;
        defect[field] = value;

        // Refresh marker icon (severity may have changed)
        if (field === 'severity') {
            const idx = defects.indexOf(defect);
            const map = getMap();
            if (map && markerLayer[idx]) {
                markerLayer[idx].setIcon(makeDefectIcon(defect));
                markerLayer[idx].setPopupContent(buildPopupHtml(defect));
            }
        }
    }

    // --- Render the defect list in the plan panel ---
    function renderList() {
        if (!panelEl) return;

        let listContainer = document.getElementById('inspection-list');
        if (!listContainer) return;

        if (defects.length === 0) {
            listContainer.innerHTML = '<div style="color:var(--c-text-dim);font-size:11px;padding:8px 0">No defects placed. ' +
                'Enable Inspection Mode and click the map to add defects.</div>';
            return;
        }

        listContainer.innerHTML = defects.map(d => {
            const color = SEVERITY_COLORS[d.severity] || SEVERITY_COLORS.medium;
            return `<div class="defect-list-item" style="
                border:1px solid var(--c-border);
                border-radius:4px;margin-bottom:6px;padding:7px 8px;
                background:var(--c-bg-input)">
                <div style="display:flex;align-items:center;margin-bottom:5px">
                    <span style="background:${color};color:#000;width:20px;height:20px;border-radius:50%;
                        display:flex;align-items:center;justify-content:center;
                        font-size:10px;font-weight:700;margin-right:8px;flex-shrink:0">${d.id}</span>
                    <select data-id="${d.id}" data-field="severity"
                        style="flex:1;background:var(--c-bg);color:var(--c-text);border:1px solid var(--c-border);
                        border-radius:3px;padding:2px 4px;font-size:11px;font-family:inherit"
                        onchange="InspectionTool._updateFromSelect(this)">
                        ${Object.entries(SEVERITY_LABELS).map(([val, lbl]) =>
                            `<option value="${val}" ${d.severity === val ? 'selected' : ''}>${lbl}</option>`
                        ).join('')}
                    </select>
                    <button onclick="InspectionTool.removeDefect(${d.id})"
                        style="margin-left:6px;background:transparent;border:none;color:var(--c-emergency);cursor:pointer;font-size:14px;line-height:1">&#x2715;</button>
                </div>
                <input type="text" placeholder="Description..." value="${escapeAttr(d.description)}"
                    data-id="${d.id}" data-field="description"
                    oninput="InspectionTool._updateFromInput(this)"
                    style="width:100%;background:#0f172a;color:#e2e8f0;border:1px solid rgba(255,255,255,0.12);
                    border-radius:3px;padding:3px 5px;font-size:10px;font-family:inherit;margin-bottom:3px;box-sizing:border-box">
                <input type="text" placeholder="Photo filename..." value="${escapeAttr(d.photoRef)}"
                    data-id="${d.id}" data-field="photoRef"
                    oninput="InspectionTool._updateFromInput(this)"
                    style="width:100%;background:#0f172a;color:#e2e8f0;border:1px solid rgba(255,255,255,0.12);
                    border-radius:3px;padding:3px 5px;font-size:10px;font-family:inherit;box-sizing:border-box">
                <div style="font-size:9px;color:#475569;margin-top:3px">${d.lat.toFixed(5)}, ${d.lon.toFixed(5)}</div>
            </div>`;
        }).join('');
    }

    function escapeAttr(s) {
        return (s || '').replace(/"/g, '&quot;').replace(/'/g, '&#39;');
    }

    // Called from inline oninput/onchange
    function _updateFromInput(el) {
        const id = parseInt(el.dataset.id);
        const field = el.dataset.field;
        updateDefect(id, field, el.value);
    }

    function _updateFromSelect(el) {
        const id = parseInt(el.dataset.id);
        const field = el.dataset.field;
        updateDefect(id, field, el.value);
        renderList(); // Re-render to update color dot
    }

    // --- Export ---
    function exportJSON() {
        const data = JSON.stringify(defects, null, 2);
        downloadBlob(data, 'defects_' + dateTag() + '.json', 'application/json');
    }

    function exportCSV() {
        const header = 'id,lat,lon,severity,description,photoRef';
        const rows = defects.map(d =>
            [d.id, d.lat, d.lon, d.severity,
                '"' + (d.description || '').replace(/"/g, '""') + '"',
                '"' + (d.photoRef || '').replace(/"/g, '""') + '"'
            ].join(',')
        );
        downloadBlob([header, ...rows].join('\n'), 'defects_' + dateTag() + '.csv', 'text/csv');
    }

    function downloadBlob(content, filename, mime) {
        const blob = new Blob([content], { type: mime });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url; a.download = filename;
        document.body.appendChild(a);
        a.click();
        setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 1000);
    }

    function dateTag() {
        return new Date().toISOString().slice(0, 16).replace('T', '_').replace(':', '');
    }

    // --- Map click handler ---
    function onMapClick(e) {
        if (!inspectionMode) return;
        addDefect(e.latlng.lat, e.latlng.lng);
    }

    // --- Toggle inspection click mode ---
    function toggleMode(on) {
        inspectionMode = on;
        const map = getMap();
        if (!map) return;
        if (on) {
            map.on('click', onMapClick);
            map.getContainer().style.cursor = 'crosshair';
        } else {
            map.off('click', onMapClick);
            map.getContainer().style.cursor = '';
        }
    }

    // --- Build the inspection panel section ---
    function buildPanel(container) {
        panelEl = container;

        const wrapper = document.createElement('div');
        wrapper.id = 'inspection-panel';
        wrapper.style.cssText = 'padding:8px;font-family:var(--font-mono,"DM Mono",monospace);font-size:12px';

        // Mode toggle + export toolbar
        wrapper.innerHTML =
            '<div style="display:flex;gap:6px;margin-bottom:10px;align-items:center;flex-wrap:wrap">' +
                '<button id="btn-inspection-mode" style="flex:1;min-width:80px;min-height:36px" class="upload-btn" ' +
                    'onclick="InspectionTool._toggleModeBtn(this)">' +
                    '&#x271A; Add Defects' +
                '</button>' +
                '<button style="flex:1;min-width:60px;min-height:36px" class="upload-btn" onclick="InspectionTool.exportJSON()">JSON</button>' +
                '<button style="flex:1;min-width:60px;min-height:36px" class="upload-btn" onclick="InspectionTool.exportCSV()">CSV</button>' +
            '</div>' +
            '<div id="inspection-list"></div>';

        container.appendChild(wrapper);
        renderList();
    }

    function _toggleModeBtn(btn) {
        inspectionMode = !inspectionMode;
        toggleMode(inspectionMode);
        if (inspectionMode) {
            btn.textContent = '\u2715 Stop Adding';
            btn.style.background = 'rgba(239,68,68,0.2)';
            btn.style.borderColor = '#ef4444';
        } else {
            btn.textContent = '\u271A Add Defects';
            btn.style.background = '';
            btn.style.borderColor = '';
        }
    }

    // --- Plan view activation hook ---
    function onPlanActivate(container) {
        // Append inspection section to plan panel
        const section = document.createElement('div');
        section.id = 'inspection-section';
        section.innerHTML = '<div style="padding:6px 8px 3px;font-size:10px;letter-spacing:.08em;color:#64748b;border-top:1px solid rgba(255,255,255,0.07);margin-top:10px">INSPECTION DEFECTS</div>';
        const inner = document.createElement('div');
        buildPanel(inner);
        section.appendChild(inner);
        container.appendChild(section);
    }

    // Expose for use when plan view renders
    function initForPlanPanel(container) {
        onPlanActivate(container);
    }

    function init() {
        // Nothing to do until plan panel is activated
    }

    return {
        init,
        initForPlanPanel,
        addDefect,
        removeDefect,
        updateDefect,
        exportJSON,
        exportCSV,
        renderList,
        _updateFromInput,
        _updateFromSelect,
        _toggleModeBtn,
    };

})();
