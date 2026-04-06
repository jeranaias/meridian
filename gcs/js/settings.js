/* ============================================================
   settings.js — Application settings panel
   Units, map provider, guided limits, connection, display,
   common modes, recording, theme.
   All changes persist via meridian.saveSettings().
   ============================================================ */

'use strict';

window.Settings = (function () {

    let container = null;

    const MAP_PROVIDERS = [
        { id: 'carto-voyager', label: 'CartoDB Voyager', url: 'https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png' },
        { id: 'carto-dark',    label: 'CartoDB Dark',    url: 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png' },
        { id: 'osm',           label: 'OpenStreetMap',   url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png' },
        { id: 'satellite',     label: 'Satellite (ESRI)', url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' },
    ];

    const COPTER_MODES = meridian.COPTER_MODES;

    function render() {
        container = document.querySelector('#panel-settings .panel-body');
        if (!container) return;
        container.innerHTML = '';

        const s = meridian.settings;
        const wrapper = document.createElement('div');
        wrapper.className = 'settings-panel';

        // --- Theme ---
        wrapper.appendChild(createSectionHeader('Theme'));
        wrapper.appendChild(createRadioGroup('theme', [
            { value: 'light', label: 'Light' },
            { value: 'dark',  label: 'Dark' },
            { value: 'system', label: 'System' },
        ], getThemeValue(), function (val) {
            if (val === 'system') {
                const prefer = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
                Theme.set(prefer);
                localStorage.setItem('meridian_theme_mode', 'system');
            } else {
                Theme.set(val);
                localStorage.setItem('meridian_theme_mode', val);
            }
        }));

        // --- Units ---
        wrapper.appendChild(createSectionHeader('Units'));
        wrapper.appendChild(createRadioGroup('units', [
            { value: 'metric',   label: 'Metric (m, m/s)' },
            { value: 'imperial', label: 'Imperial (ft, mph)' },
            { value: 'aviation', label: 'Aviation (ft, kts)' },
        ], s.units, function (val) {
            s.units = val;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'units', value: val });
        }));

        // --- Map Provider ---
        wrapper.appendChild(createSectionHeader('Map Provider'));
        const mapSelect = document.createElement('select');
        mapSelect.className = 'settings-select';
        MAP_PROVIDERS.forEach(function (mp) {
            const opt = document.createElement('option');
            opt.value = mp.id;
            opt.textContent = mp.label;
            if (mp.id === s.mapProvider) opt.selected = true;
            mapSelect.appendChild(opt);
        });
        mapSelect.addEventListener('change', function () {
            s.mapProvider = mapSelect.value;
            meridian.saveSettings();
            applyMapProvider(mapSelect.value);
        });
        wrapper.appendChild(wrapField(mapSelect));

        // --- Guided Limits ---
        wrapper.appendChild(createSectionHeader('Guided Limits'));
        wrapper.appendChild(createNumberField('Max Altitude (m)', s.guidedAltMax, 1, 1000, function (val) {
            s.guidedAltMax = val;
            meridian.saveSettings();
        }));
        wrapper.appendChild(createNumberField('Max Distance (m)', s.guidedDistMax, 10, 10000, function (val) {
            s.guidedDistMax = val;
            meridian.saveSettings();
        }));

        // --- Connection ---
        wrapper.appendChild(createSectionHeader('Connection'));
        wrapper.appendChild(createTextField('Default URL', s.defaultUrl || 'ws://localhost:5760', function (val) {
            s.defaultUrl = val;
            meridian.saveSettings();
        }));
        wrapper.appendChild(createToggle('Auto-reconnect', s.reconnectDelay > 0, function (on) {
            s.reconnectDelay = on ? 3000 : 0;
            meridian.saveSettings();
        }));

        // Demo mode button
        var demoBtn = document.createElement('button');
        demoBtn.type = 'button';
        demoBtn.style.cssText = 'width:100%;margin-top:6px';
        demoBtn.className = 'setup-btn secondary';
        demoBtn.textContent = Demo.isActive() ? 'Stop Demo Mode' : 'Start Demo Mode';
        demoBtn.addEventListener('click', function () {
            if (Demo.isActive()) {
                Demo.stop();
                var v = meridian.v;
                if (v) { v.connected = false; v.armed = false; v.trail = []; }
                Toolbar.updateConnection(0);
                demoBtn.textContent = 'Start Demo Mode';
                meridian.log('Demo stopped', 'info');
            } else {
                Demo.start();
                demoBtn.textContent = 'Stop Demo Mode';
            }
        });
        wrapper.appendChild(demoBtn);

        // --- ADSB ---
        wrapper.appendChild(createSectionHeader('ADS-B Traffic'));
        wrapper.appendChild(createToggle('Show ADSB Traffic', s.showAdsb, function (on) {
            s.showAdsb = on;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'showAdsb', value: on });
        }));
        wrapper.appendChild(createTextField('ADSB Server (ws://)', s.adsbServer || '', function (val) {
            s.adsbServer = val;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'adsbServer', value: val });
        }));

        // --- Operator Identity (T2-2) ---
        wrapper.appendChild(createSectionHeader('Operator'));
        wrapper.appendChild(createTextField('PIC Name', s.operatorName || '', function (val) {
            s.operatorName = val;
            meridian.saveSettings();
        }));
        wrapper.appendChild(createTextField('Certificate #', s.operatorCert || '', function (val) {
            s.operatorCert = val;
            meridian.saveSettings();
        }));
        wrapper.appendChild(createTextField('Registration #', s.operatorRegistration || '', function (val) {
            s.operatorRegistration = val;
            meridian.saveSettings();
        }));

        // --- Remote ID ---
        wrapper.appendChild(createSectionHeader('Remote ID'));
        var ridNote = document.createElement('div');
        ridNote.style.cssText = 'font-size:11px;color:var(--c-neutral-dim);padding:0 0 8px;line-height:1.5';
        ridNote.textContent = 'Remote ID status is read automatically from the vehicle when connected. Configure RID module via vehicle parameters (DID_* params).';
        wrapper.appendChild(ridNote);
        wrapper.appendChild(createToggle('Show RID in Health Grid', s.showRidStatus !== false, function (on) {
            s.showRidStatus = on;
            meridian.saveSettings();
        }));

        // --- Agricultural / Spray ---
        wrapper.appendChild(createSectionHeader('Agricultural'));
        wrapper.appendChild(createToggle('Show Spray Widget', s.showSprayWidget, function (on) {
            s.showSprayWidget = on;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'showSprayWidget', value: on });
        }));

        // --- T3-3: Thermal Camera ---
        wrapper.appendChild(createSectionHeader('Thermal Camera'));
        wrapper.appendChild(createToggle('Show Thermal Widget', s.showThermalWidget !== false, function (on) {
            s.showThermalWidget = on;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'showThermalWidget', value: on });
        }));

        // --- T3-4: AIS Vessel Tracker ---
        wrapper.appendChild(createSectionHeader('AIS Vessel Tracker'));
        wrapper.appendChild(createTextField('AIS Server (ws://)', s.aisServer || '', function (val) {
            s.aisServer = val;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'aisServer', value: val });
        }));
        wrapper.appendChild(createTextField('Own Vessel MMSI', s.ownMmsi || '', function (val) {
            s.ownMmsi = val;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'ownMmsi', value: val });
        }));

        // --- T3-13: EU Compliance ---
        if (window.EUCompliance) {
            EUCompliance.buildSettingsSection(wrapper);
        }

        // --- Display ---
        wrapper.appendChild(createSectionHeader('Display'));
        wrapper.appendChild(createToggle('Show Trajectory', s.showTrajectory, function (on) {
            s.showTrajectory = on;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'showTrajectory', value: on });
        }));
        wrapper.appendChild(createToggle('Show Uncertainty', s.showUncertainty, function (on) {
            s.showUncertainty = on;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'showUncertainty', value: on });
        }));

        // --- Common Modes ---
        wrapper.appendChild(createSectionHeader('Common Modes'));
        wrapper.appendChild(createModeList(s.commonModes));

        // --- Offline Maps ---
        wrapper.appendChild(createSectionHeader('Offline Maps'));
        wrapper.appendChild(createOfflineMapsSection());

        // --- Recording ---
        wrapper.appendChild(createSectionHeader('Recording'));
        wrapper.appendChild(createToggle('Auto-record on connect', s.autoRecord, function (on) {
            s.autoRecord = on;
            meridian.saveSettings();
        }));

        // --- Language / i18n (T2-14) ---
        wrapper.appendChild(createSectionHeader('Language'));
        var localeRow = document.createElement('div');
        localeRow.className = 'settings-field';
        var localeLbl = document.createElement('label');
        localeLbl.className = 'settings-field-label';
        localeLbl.textContent = 'Interface language';
        var localeSelect = document.createElement('select');
        localeSelect.className = 'settings-select';
        var locales = [{ code: 'en', label: 'English' }];
        locales.forEach(function (loc) {
            var opt = document.createElement('option');
            opt.value = loc.code;
            opt.textContent = loc.label;
            if (loc.code === (s.locale || 'en')) opt.selected = true;
            localeSelect.appendChild(opt);
        });
        localeSelect.addEventListener('change', function () {
            s.locale = localeSelect.value;
            meridian.saveSettings();
            if (window.i18n) i18n.setLocale(localeSelect.value);
        });
        localeRow.appendChild(localeLbl);
        localeRow.appendChild(localeSelect);
        wrapper.appendChild(localeRow);

        // --- ROS2 Bridge (T3-15) ---
        if (window.RosBridge) {
            RosBridge.renderSettingsSection(
                wrapper,
                createSectionHeader,
                createTextField,
                createToggle,
                createNumberField
            );
        }

        // --- STANAG 4586 (T3-14) ---
        if (window.StanagNotes) {
            StanagNotes.renderSettingsSection(wrapper, createSectionHeader);
        }

        // --- Firmware Upload Endpoint (T3-19) ---
        wrapper.appendChild(createSectionHeader('Firmware'));
        wrapper.appendChild(createTextField('HTTP upload endpoint', meridian.settings.firmwareUploadUrl || '', function (val) {
            meridian.settings.firmwareUploadUrl = val;
            meridian.saveSettings();
        }));

        // --- Onboarding (T2-13) ---
        wrapper.appendChild(createSectionHeader('Help'));
        var tutRow = document.createElement('div');
        tutRow.className = 'settings-field';
        var tutBtn = document.createElement('button');
        tutBtn.className = 'offline-btn draw';
        tutBtn.textContent = 'Show Tutorial';
        tutBtn.style.marginTop = '4px';
        tutBtn.addEventListener('click', function () {
            if (window.Onboarding) Onboarding.start(true);
        });
        tutRow.appendChild(tutBtn);
        wrapper.appendChild(tutRow);

        container.appendChild(wrapper);
    }

    // --- Offline Maps Section (T2-9: area draw + zoom range) ---

    var _offlineDrawing = false;        // rectangle draw mode active
    var _offlineCorner1 = null;         // first click corner {lat,lng}
    var _offlineRect = null;            // Leaflet.Rectangle preview
    var _offlineBounds = null;          // confirmed L.LatLngBounds for download

    function createOfflineMapsSection() {
        var section = document.createElement('div');
        section.className = 'offline-maps-section';

        // Cache size display
        var statusEl = document.createElement('div');
        statusEl.className = 'offline-status';
        statusEl.innerHTML =
            '<span class="offline-status-label">Cache size:</span>' +
            '<span id="offline-cache-info">calculating...</span>';
        section.appendChild(statusEl);

        // Zoom range row
        var zoomRow = document.createElement('div');
        zoomRow.className = 'offline-zoom-row';
        zoomRow.innerHTML =
            '<span class="offline-zoom-label">Zoom range:</span>' +
            '<select id="offline-zoom-min" class="settings-select offline-zoom-sel"></select>' +
            '<span class="offline-zoom-sep">to</span>' +
            '<select id="offline-zoom-max" class="settings-select offline-zoom-sel"></select>';
        section.appendChild(zoomRow);

        // Tile estimate row
        var estimateEl = document.createElement('div');
        estimateEl.className = 'offline-estimate';
        estimateEl.id = 'offline-estimate';
        estimateEl.textContent = 'Draw an area to see estimate';
        section.appendChild(estimateEl);

        // Progress bar (hidden by default)
        var progressEl = document.createElement('div');
        progressEl.className = 'offline-progress';
        progressEl.id = 'offline-progress';
        progressEl.innerHTML =
            '<div class="offline-progress-bar"><div class="offline-progress-fill" id="offline-progress-fill"></div></div>' +
            '<div class="offline-progress-text" id="offline-progress-text">0 / 0</div>';
        section.appendChild(progressEl);

        // Buttons
        var btnsEl = document.createElement('div');
        btnsEl.className = 'offline-btns';
        btnsEl.innerHTML =
            '<button class="offline-btn draw" id="btn-offline-draw">Draw Area</button>' +
            '<button class="offline-btn download" id="btn-offline-download" disabled>Download Area</button>' +
            '<button class="offline-btn clear" id="btn-offline-clear">Clear Cache</button>';
        section.appendChild(btnsEl);

        // Wire events after DOM paint
        requestAnimationFrame(function () {
            updateCacheInfo();
            _initZoomSelectors();
            _wireOfflineButtons();
        });

        return section;
    }

    function _initZoomSelectors() {
        var map = FlyMap.getMap();
        var currentZoom = map ? Math.round(map.getZoom()) : 15;
        var minSel = document.getElementById('offline-zoom-min');
        var maxSel = document.getElementById('offline-zoom-max');
        if (!minSel || !maxSel) return;

        var lo = Math.max(1, currentZoom - 2);
        var hi = Math.min(18, currentZoom + 2);

        for (var z = 1; z <= 18; z++) {
            var o1 = document.createElement('option');
            o1.value = z; o1.textContent = z;
            if (z === lo) o1.selected = true;
            minSel.appendChild(o1);

            var o2 = document.createElement('option');
            o2.value = z; o2.textContent = z;
            if (z === hi) o2.selected = true;
            maxSel.appendChild(o2);
        }

        // Clamp min <= max on change
        minSel.addEventListener('change', function () {
            if (parseInt(minSel.value) > parseInt(maxSel.value)) {
                maxSel.value = minSel.value;
            }
            _updateTileEstimate();
        });
        maxSel.addEventListener('change', function () {
            if (parseInt(maxSel.value) < parseInt(minSel.value)) {
                minSel.value = maxSel.value;
            }
            _updateTileEstimate();
        });
    }

    function _wireOfflineButtons() {
        var drawBtn  = document.getElementById('btn-offline-draw');
        var dlBtn    = document.getElementById('btn-offline-download');
        var clrBtn   = document.getElementById('btn-offline-clear');

        if (drawBtn) {
            drawBtn.addEventListener('click', function () {
                if (_offlineDrawing) {
                    _cancelAreaDraw();
                    drawBtn.textContent = 'Draw Area';
                    drawBtn.classList.remove('active');
                } else {
                    _startAreaDraw();
                    drawBtn.textContent = 'Cancel Draw';
                    drawBtn.classList.add('active');
                }
            });
        }

        if (dlBtn) {
            dlBtn.addEventListener('click', function () {
                if (!window.OfflineTiles || !_offlineBounds) return;

                var minZoom = parseInt(document.getElementById('offline-zoom-min').value) || 13;
                var maxZoom = parseInt(document.getElementById('offline-zoom-max').value) || 17;
                var tileUrl = _currentTileUrl();

                var progressDiv  = document.getElementById('offline-progress');
                var progressFill = document.getElementById('offline-progress-fill');
                var progressText = document.getElementById('offline-progress-text');

                if (progressDiv) progressDiv.classList.add('active');
                dlBtn.disabled = true;

                OfflineTiles.downloadRegion(tileUrl, _offlineBounds, minZoom, maxZoom, function (done, total) {
                    var pct = total > 0 ? (done / total * 100).toFixed(0) : 0;
                    if (progressFill) progressFill.style.width = pct + '%';
                    if (progressText) progressText.textContent = done + ' / ' + total + ' tiles';
                }).then(function (result) {
                    if (progressDiv) progressDiv.classList.remove('active');
                    dlBtn.disabled = false;
                    meridian.log('Downloaded ' + result.downloaded + ' tiles (' + result.failed + ' failed)', 'info');
                    updateCacheInfo();
                }).catch(function (err) {
                    if (progressDiv) progressDiv.classList.remove('active');
                    dlBtn.disabled = false;
                    meridian.log('Tile download failed: ' + err.message, 'error');
                });
            });
        }

        if (clrBtn) {
            clrBtn.addEventListener('click', function () {
                if (!window.OfflineTiles) return;
                OfflineTiles.clearCache().then(function () {
                    meridian.log('Tile cache cleared', 'info');
                    updateCacheInfo();
                });
            });
        }
    }

    function _startAreaDraw() {
        _offlineDrawing = true;
        _offlineCorner1 = null;
        if (_offlineRect) { FlyMap.getMap().removeLayer(_offlineRect); _offlineRect = null; }

        var map = FlyMap.getMap();
        if (!map) return;

        map.getContainer().style.cursor = 'crosshair';

        map._offlineClickHandler = function (e) {
            if (!_offlineCorner1) {
                _offlineCorner1 = e.latlng;
            } else {
                var bounds = L.latLngBounds(_offlineCorner1, e.latlng);
                _offlineBounds = bounds;
                if (_offlineRect) map.removeLayer(_offlineRect);
                _offlineRect = L.rectangle(bounds, {
                    color: '#0891b2', weight: 2,
                    fillOpacity: 0.15, dashArray: '5,5'
                }).addTo(map);
                _cancelAreaDraw();
                var drawBtn = document.getElementById('btn-offline-draw');
                if (drawBtn) { drawBtn.textContent = 'Redraw Area'; drawBtn.classList.remove('active'); }
                var dlBtn = document.getElementById('btn-offline-download');
                if (dlBtn) dlBtn.disabled = false;
                _updateTileEstimate();
            }
        };

        map.on('click', map._offlineClickHandler);
    }

    function _cancelAreaDraw() {
        _offlineDrawing = false;
        _offlineCorner1 = null;
        var map = FlyMap.getMap();
        if (!map) return;
        map.getContainer().style.cursor = '';
        if (map._offlineClickHandler) {
            map.off('click', map._offlineClickHandler);
            map._offlineClickHandler = null;
        }
    }

    function _updateTileEstimate() {
        var el = document.getElementById('offline-estimate');
        if (!el || !_offlineBounds) return;

        var minZoom = parseInt((document.getElementById('offline-zoom-min') || {}).value) || 13;
        var maxZoom = parseInt((document.getElementById('offline-zoom-max') || {}).value) || 17;
        var total = 0;

        for (var z = minZoom; z <= maxZoom; z++) {
            var n = Math.pow(2, z);
            var sw = _offlineBounds.getSouthWest();
            var ne = _offlineBounds.getNorthEast();

            var xMin = Math.floor((sw.lng + 180) / 360 * n);
            var xMax = Math.floor((ne.lng + 180) / 360 * n);
            var yMin = Math.floor((1 - Math.log(Math.tan(ne.lat * Math.PI / 180) + 1 / Math.cos(ne.lat * Math.PI / 180)) / Math.PI) / 2 * n);
            var yMax = Math.floor((1 - Math.log(Math.tan(sw.lat * Math.PI / 180) + 1 / Math.cos(sw.lat * Math.PI / 180)) / Math.PI) / 2 * n);

            total += (Math.abs(xMax - xMin) + 1) * (Math.abs(yMax - yMin) + 1);
        }

        var bytes = total * 15 * 1024; // ~15KB avg tile
        el.textContent = '~' + total.toLocaleString() + ' tiles (' + OfflineTiles.formatBytes(bytes) + ') zoom ' + minZoom + '-' + maxZoom;
    }

    function _currentTileUrl() {
        var tileUrl = 'https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png';
        var provider = MAP_PROVIDERS.find(function (mp) { return mp.id === meridian.settings.mapProvider; });
        if (provider) tileUrl = provider.url.replace('{s}', 'a').replace('{r}', '');
        return tileUrl;
    }

    function updateCacheInfo() {
        var infoEl = document.getElementById('offline-cache-info');
        if (!infoEl) return;
        if (!window.OfflineTiles) {
            infoEl.textContent = 'not available';
            return;
        }
        OfflineTiles.getCacheSize().then(function (info) {
            if (infoEl) {
                infoEl.textContent = info.count + ' tiles (~' + OfflineTiles.formatBytes(info.bytes) + ')';
            }
        });
    }

    // --- Helpers ---

    function createSectionHeader(text) {
        const h = document.createElement('div');
        h.className = 'settings-section-header';
        h.textContent = text;
        return h;
    }

    function createRadioGroup(name, options, current, onChange) {
        const group = document.createElement('div');
        group.className = 'settings-radio-group';
        options.forEach(function (opt) {
            const label = document.createElement('label');
            label.className = 'settings-radio-label';

            const radio = document.createElement('input');
            radio.type = 'radio';
            radio.name = name;
            radio.value = opt.value;
            radio.checked = opt.value === current;
            radio.addEventListener('change', function () {
                if (radio.checked) onChange(opt.value);
            });

            const span = document.createElement('span');
            span.textContent = opt.label;

            label.appendChild(radio);
            label.appendChild(span);
            group.appendChild(label);
        });
        return group;
    }

    function createNumberField(label, value, min, max, onChange) {
        const row = document.createElement('div');
        row.className = 'settings-field';

        const lbl = document.createElement('label');
        lbl.className = 'settings-field-label';
        lbl.textContent = label;

        const input = document.createElement('input');
        input.type = 'number';
        input.className = 'settings-input';
        input.value = value;
        input.min = min;
        input.max = max;
        input.addEventListener('change', function () {
            const v = parseFloat(input.value);
            if (!isNaN(v) && v >= min && v <= max) onChange(v);
        });

        row.appendChild(lbl);
        row.appendChild(input);
        return row;
    }

    function createTextField(label, value, onChange) {
        const row = document.createElement('div');
        row.className = 'settings-field';

        const lbl = document.createElement('label');
        lbl.className = 'settings-field-label';
        lbl.textContent = label;

        const input = document.createElement('input');
        input.type = 'text';
        input.className = 'settings-input wide';
        input.value = value;
        input.addEventListener('change', function () {
            onChange(input.value);
        });

        row.appendChild(lbl);
        row.appendChild(input);
        return row;
    }

    function createToggle(label, checked, onChange) {
        const row = document.createElement('div');
        row.className = 'settings-toggle-row';

        const lbl = document.createElement('span');
        lbl.className = 'settings-toggle-label';
        lbl.textContent = label;

        const toggle = document.createElement('label');
        toggle.className = 'settings-toggle';

        const input = document.createElement('input');
        input.type = 'checkbox';
        input.checked = checked;
        input.addEventListener('change', function () {
            onChange(input.checked);
        });

        const slider = document.createElement('span');
        slider.className = 'settings-toggle-slider';

        toggle.appendChild(input);
        toggle.appendChild(slider);

        row.appendChild(lbl);
        row.appendChild(toggle);
        return row;
    }

    function wrapField(el) {
        const row = document.createElement('div');
        row.className = 'settings-field';
        row.appendChild(el);
        return row;
    }

    // --- Common Modes drag-to-reorder ---

    function createModeList(modes) {
        const wrapper = document.createElement('div');
        wrapper.className = 'settings-mode-list';

        // Current modes
        const list = document.createElement('div');
        list.className = 'settings-mode-sortable';
        list.id = 'mode-sortable';

        modes.forEach(function (num) {
            const item = createModeItem(num);
            list.appendChild(item);
        });

        wrapper.appendChild(list);

        // Add mode dropdown
        const addRow = document.createElement('div');
        addRow.className = 'settings-mode-add';

        const addSelect = document.createElement('select');
        addSelect.className = 'settings-select';
        addSelect.innerHTML = '<option value="">+ Add mode...</option>';
        for (const num in COPTER_MODES) {
            if (!modes.includes(parseInt(num))) {
                const opt = document.createElement('option');
                opt.value = num;
                opt.textContent = COPTER_MODES[num];
                addSelect.appendChild(opt);
            }
        }
        addSelect.addEventListener('change', function () {
            if (!addSelect.value) return;
            const num = parseInt(addSelect.value);
            meridian.settings.commonModes.push(num);
            meridian.saveSettings();
            render(); // Re-render to rebuild
            meridian.events.emit('settings_change', { key: 'commonModes' });
        });

        addRow.appendChild(addSelect);
        wrapper.appendChild(addRow);

        // Enable drag sorting
        enableDragSort(list);

        return wrapper;
    }

    function createModeItem(num) {
        const item = document.createElement('div');
        item.className = 'settings-mode-item';
        item.draggable = true;
        item.dataset.mode = num;

        const handle = document.createElement('span');
        handle.className = 'settings-mode-handle';
        handle.textContent = '\u2261';

        const name = document.createElement('span');
        name.className = 'settings-mode-name';
        name.textContent = COPTER_MODES[num] || ('MODE_' + num);

        const removeBtn = document.createElement('button');
        removeBtn.className = 'settings-mode-remove';
        removeBtn.textContent = '\u2715';
        removeBtn.addEventListener('click', function () {
            const idx = meridian.settings.commonModes.indexOf(num);
            if (idx >= 0) meridian.settings.commonModes.splice(idx, 1);
            meridian.saveSettings();
            render();
            meridian.events.emit('settings_change', { key: 'commonModes' });
        });

        item.appendChild(handle);
        item.appendChild(name);
        item.appendChild(removeBtn);
        return item;
    }

    function enableDragSort(list) {
        let dragEl = null;

        list.addEventListener('dragstart', function (e) {
            dragEl = e.target.closest('.settings-mode-item');
            if (!dragEl) return;
            dragEl.classList.add('dragging');
            e.dataTransfer.effectAllowed = 'move';
        });

        list.addEventListener('dragend', function () {
            if (dragEl) dragEl.classList.remove('dragging');
            dragEl = null;
            // Read new order from DOM
            const newOrder = [];
            list.querySelectorAll('.settings-mode-item').forEach(function (el) {
                newOrder.push(parseInt(el.dataset.mode));
            });
            meridian.settings.commonModes = newOrder;
            meridian.saveSettings();
            meridian.events.emit('settings_change', { key: 'commonModes' });
        });

        list.addEventListener('dragover', function (e) {
            e.preventDefault();
            e.dataTransfer.dropEffect = 'move';
            const afterEl = getDragAfterElement(list, e.clientY);
            if (afterEl) {
                list.insertBefore(dragEl, afterEl);
            } else {
                list.appendChild(dragEl);
            }
        });
    }

    function getDragAfterElement(container, y) {
        const els = Array.from(container.querySelectorAll('.settings-mode-item:not(.dragging)'));
        let closest = null;
        let closestOffset = Number.NEGATIVE_INFINITY;

        els.forEach(function (el) {
            const box = el.getBoundingClientRect();
            const offset = y - box.top - box.height / 2;
            if (offset < 0 && offset > closestOffset) {
                closestOffset = offset;
                closest = el;
            }
        });

        return closest;
    }

    // --- Map provider swap ---

    function applyMapProvider(id) {
        const provider = MAP_PROVIDERS.find(function (mp) { return mp.id === id; });
        if (!provider) return;

        const map = FlyMap.getMap();
        if (!map) return;

        // Remove existing tile layers
        map.eachLayer(function (layer) {
            if (layer instanceof L.TileLayer) map.removeLayer(layer);
        });

        L.tileLayer(provider.url, {
            attribution: '&copy; OSM',
            subdomains: provider.url.indexOf('{s}') >= 0 ? 'abcd' : undefined,
            maxZoom: 19,
        }).addTo(map);
    }

    function getThemeValue() {
        const mode = localStorage.getItem('meridian_theme_mode');
        if (mode === 'system') return 'system';
        return Theme.get();
    }

    // Re-render when panel opens
    meridian.events.on('panel_change', function (panel) {
        if (panel === 'settings') render();
    });

    return { render };

})();
