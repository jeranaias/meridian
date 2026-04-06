/* ============================================================
   firmware.js — OTA Firmware Update Module
   T3-19: Firmware checklist item for Setup panel.
   - Shows current firmware version from AUTOPILOT_VERSION
   - .apj file picker with JSON envelope parsing
   - Companion-computer HTTP upload endpoint
   - Progress bar
   ============================================================ */

'use strict';

window.FirmwareSetup = (function () {

    // -------------------------------------------------------------------------
    // Render — called by setup-view.js renderExpanded('firmware')
    // -------------------------------------------------------------------------

    function render(container) {
        container.innerHTML = '';

        var v = meridian.v;
        var params = v ? v.params : {};

        // ---- Current firmware version block ----
        var versionSection = document.createElement('div');
        versionSection.className = 'setup-form-section';

        var versionTitle = document.createElement('div');
        versionTitle.className = 'setup-form-title';
        versionTitle.textContent = 'Current Firmware';
        versionSection.appendChild(versionTitle);

        var versionGrid = document.createElement('div');
        versionGrid.className = 'firmware-version-grid';
        versionGrid.id = 'firmware-version-grid';
        versionSection.appendChild(versionGrid);

        _renderVersionGrid(versionGrid);

        container.appendChild(versionSection);

        // ---- Upload section ----
        var uploadSection = document.createElement('div');
        uploadSection.className = 'setup-form-section';

        var uploadTitle = document.createElement('div');
        uploadTitle.className = 'setup-form-title';
        uploadTitle.textContent = 'Upload Firmware';
        uploadSection.appendChild(uploadTitle);

        // Info notice about USB vs HTTP
        var notice = document.createElement('div');
        notice.className = 'firmware-notice';
        notice.innerHTML =
            '<svg width="14" height="14" viewBox="0 0 14 14" fill="none">' +
            '<circle cx="7" cy="7" r="6" stroke="var(--c-info,#0891b2)" stroke-width="1.5"/>' +
            '<line x1="7" y1="5" x2="7" y2="10" stroke="var(--c-info,#0891b2)" stroke-width="1.5"/>' +
            '<circle cx="7" cy="3.5" r="0.7" fill="var(--c-info,#0891b2)"/></svg>' +
            '<span>For direct USB flashing, connect via USB and use <strong>Mission Planner</strong> or ' +
            '<strong>QGC</strong>. Browser WebSocket cannot access bootloader protocol.<br>' +
            'HTTP upload requires a companion computer running a firmware-relay service.</span>';
        uploadSection.appendChild(notice);

        // .apj file picker
        var fileRow = document.createElement('div');
        fileRow.className = 'firmware-file-row';

        var fileInput = document.createElement('input');
        fileInput.type = 'file';
        fileInput.id = 'firmware-file-input';
        fileInput.accept = '.apj,.px4,.bin';
        fileInput.style.display = 'none';

        var fileBtn = document.createElement('button');
        fileBtn.className = 'offline-btn draw';
        fileBtn.textContent = '\u21E7 Choose .apj File';
        fileBtn.addEventListener('click', function () { fileInput.click(); });

        var fileLabel = document.createElement('span');
        fileLabel.id = 'firmware-file-label';
        fileLabel.className = 'firmware-file-label';
        fileLabel.textContent = 'No file selected';

        fileInput.addEventListener('change', function () {
            if (!fileInput.files || !fileInput.files[0]) return;
            var file = fileInput.files[0];
            fileLabel.textContent = file.name + ' (' + _formatBytes(file.size) + ')';
            _parseApjFile(file, container);
        });

        fileRow.appendChild(fileBtn);
        fileRow.appendChild(fileInput);
        fileRow.appendChild(fileLabel);
        uploadSection.appendChild(fileRow);

        // Parsed APJ info placeholder
        var apjInfo = document.createElement('div');
        apjInfo.id = 'firmware-apj-info';
        apjInfo.className = 'firmware-apj-info';
        uploadSection.appendChild(apjInfo);

        // HTTP endpoint for companion-computer upload
        var endpointRow = document.createElement('div');
        endpointRow.className = 'firmware-endpoint-row';
        var endpointLabel = document.createElement('label');
        endpointLabel.className = 'firmware-endpoint-label';
        endpointLabel.textContent = 'Upload endpoint (companion):';
        var endpointInput = document.createElement('input');
        endpointInput.type = 'text';
        endpointInput.className = 'settings-input wide';
        endpointInput.id = 'firmware-endpoint';
        endpointInput.placeholder = 'http://192.168.1.1:5000/upload';
        endpointInput.value = meridian.settings.firmwareUploadUrl || '';
        endpointInput.addEventListener('change', function () {
            meridian.settings.firmwareUploadUrl = endpointInput.value;
            meridian.saveSettings();
        });
        endpointRow.appendChild(endpointLabel);
        endpointRow.appendChild(endpointInput);
        uploadSection.appendChild(endpointRow);

        // Progress bar (hidden by default)
        var progressWrap = document.createElement('div');
        progressWrap.id = 'firmware-progress-wrap';
        progressWrap.className = 'firmware-progress-wrap';
        progressWrap.style.display = 'none';
        progressWrap.innerHTML =
            '<div class="offline-progress">' +
            '<div class="offline-progress-bar">' +
            '<div class="offline-progress-fill" id="firmware-progress-fill"></div>' +
            '</div>' +
            '<div class="offline-progress-text" id="firmware-progress-text">0%</div>' +
            '</div>';
        uploadSection.appendChild(progressWrap);

        // Upload button
        var uploadBtn = document.createElement('button');
        uploadBtn.id = 'firmware-upload-btn';
        uploadBtn.className = 'offline-btn download';
        uploadBtn.textContent = '\u2B06 Upload to Companion';
        uploadBtn.style.marginTop = '8px';
        uploadBtn.disabled = true;
        uploadBtn.addEventListener('click', function () {
            _doUpload(fileInput, endpointInput.value, uploadBtn, progressWrap);
        });
        uploadSection.appendChild(uploadBtn);

        container.appendChild(uploadSection);

        // Enable upload button once file is chosen
        fileInput.addEventListener('change', function () {
            uploadBtn.disabled = !fileInput.files || !fileInput.files[0];
        });

        // Listen for AUTOPILOT_VERSION event to refresh version display
        var _listener = function (msg) {
            _renderVersionGrid(document.getElementById('firmware-version-grid'));
        };
        meridian.events.on('autopilot_version', _listener);
        // Cleanup when element is removed (best effort)
        container.addEventListener('remove', function () {
            meridian.events.off('autopilot_version', _listener);
        });
    }

    // -------------------------------------------------------------------------
    // Version grid
    // -------------------------------------------------------------------------

    function _renderVersionGrid(grid) {
        if (!grid) return;
        var v = meridian.v;
        var fwv = v && v.firmwareVersion;

        var fields = [
            { label: 'Version',  value: fwv ? (fwv.major + '.' + fwv.minor + '.' + fwv.patch) : 'Unknown' },
            { label: 'Type',     value: fwv ? _fwTypeName(fwv.type) : '---' },
            { label: 'Git hash', value: fwv ? (fwv.gitHash || '---') : '---' },
            { label: 'Board',    value: fwv ? (fwv.board || '---') : '---' },
        ];

        grid.innerHTML = '';
        fields.forEach(function (f) {
            var row = document.createElement('div');
            row.className = 'firmware-version-row';
            row.innerHTML =
                '<span class="firmware-version-label">' + f.label + '</span>' +
                '<span class="firmware-version-value">' + f.value + '</span>';
            grid.appendChild(row);
        });

        if (!fwv) {
            var hint = document.createElement('div');
            hint.className = 'cal-description';
            hint.textContent = 'Connect to vehicle to retrieve firmware version (AUTOPILOT_VERSION message).';
            grid.appendChild(hint);
        }
    }

    function _fwTypeName(type) {
        var names = {
            0: 'Custom', 1: 'Official', 2: 'Alpha', 3: 'Beta',
            4: 'RC', 255: 'Unknown',
        };
        return names[type] || ('Type ' + type);
    }

    // -------------------------------------------------------------------------
    // .apj file parsing
    // -------------------------------------------------------------------------

    function _parseApjFile(file, container) {
        var infoEl = document.getElementById('firmware-apj-info');
        if (!infoEl) return;

        if (!file.name.endsWith('.apj')) {
            infoEl.innerHTML = '<div class="firmware-apj-warning">Note: not an .apj file — metadata parsing skipped.</div>';
            return;
        }

        var reader = new FileReader();
        reader.onload = function (ev) {
            var text = ev.target.result;
            var json;
            try {
                json = JSON.parse(text);
            } catch (e) {
                infoEl.innerHTML = '<div class="firmware-apj-warning">Could not parse .apj JSON envelope.</div>';
                return;
            }

            // Typical .apj fields: board_id, board_revision, description, version, git_identity, firmware_size
            var fields = [
                { label: 'Board',       value: json.board_id || json.board || '---' },
                { label: 'Version',     value: json.version || '---' },
                { label: 'Git hash',    value: json.git_identity || '---' },
                { label: 'Description', value: json.description || '---' },
                { label: 'Size',        value: json.firmware_size ? _formatBytes(json.firmware_size) : _formatBytes(file.size) },
            ];

            infoEl.innerHTML = '<div class="firmware-apj-title">APJ Envelope</div>';
            var grid = document.createElement('div');
            grid.className = 'firmware-version-grid';
            fields.forEach(function (f) {
                var row = document.createElement('div');
                row.className = 'firmware-version-row';
                row.innerHTML =
                    '<span class="firmware-version-label">' + f.label + '</span>' +
                    '<span class="firmware-version-value">' + f.value + '</span>';
                grid.appendChild(row);
            });
            infoEl.appendChild(grid);
        };
        reader.readAsText(file);
    }

    // -------------------------------------------------------------------------
    // HTTP upload to companion computer
    // -------------------------------------------------------------------------

    function _doUpload(fileInput, endpoint, uploadBtn, progressWrap) {
        if (!fileInput.files || !fileInput.files[0]) {
            meridian.log('Firmware: no file selected', 'warn');
            return;
        }
        if (!endpoint) {
            meridian.log('Firmware: no upload endpoint configured', 'warn');
            return;
        }

        var file = fileInput.files[0];
        var formData = new FormData();
        formData.append('firmware', file);

        var fill = document.getElementById('firmware-progress-fill');
        var text = document.getElementById('firmware-progress-text');

        progressWrap.style.display = 'block';
        uploadBtn.disabled = true;

        var xhr = new XMLHttpRequest();
        xhr.open('POST', endpoint, true);

        xhr.upload.addEventListener('progress', function (e) {
            if (e.lengthComputable) {
                var pct = Math.round(e.loaded / e.total * 100);
                if (fill) fill.style.width = pct + '%';
                if (text) text.textContent = pct + '%';
            }
        });

        xhr.addEventListener('load', function () {
            progressWrap.style.display = 'none';
            uploadBtn.disabled = false;
            if (xhr.status >= 200 && xhr.status < 300) {
                meridian.log('Firmware upload complete', 'info');
                if (fill) fill.style.width = '100%';
                if (text) text.textContent = '100% — Upload complete';
                progressWrap.style.display = 'block';
            } else {
                meridian.log('Firmware upload failed: HTTP ' + xhr.status, 'error');
            }
        });

        xhr.addEventListener('error', function () {
            progressWrap.style.display = 'none';
            uploadBtn.disabled = false;
            meridian.log('Firmware upload error — check endpoint URL and companion connection', 'error');
        });

        meridian.log('Uploading firmware to ' + endpoint + '...', 'info');
        xhr.send(formData);
    }

    // -------------------------------------------------------------------------
    // Checklist item definition (registered in checklist.js OPTIONAL_ITEMS)
    // -------------------------------------------------------------------------

    var CHECKLIST_ITEM = {
        id:        'firmware',
        name:      'Firmware',
        required:  false,
        component: 'FirmwareSetup',
        check: function () {
            var v = meridian.v;
            var fwv = v && v.firmwareVersion;
            if (!fwv) return { status: 'optional', text: 'Version unknown' };
            var ver = fwv.major + '.' + fwv.minor + '.' + fwv.patch;
            return { status: 'complete', text: ver };
        },
    };

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    function _formatBytes(bytes) {
        if (bytes === 0) return '0 B';
        var k = 1024;
        var sizes = ['B', 'KB', 'MB', 'GB'];
        var i = Math.floor(Math.log(bytes) / Math.log(k));
        return (bytes / Math.pow(k, i)).toFixed(1) + ' ' + sizes[i];
    }

    return {
        render,
        CHECKLIST_ITEM,
    };

})();
