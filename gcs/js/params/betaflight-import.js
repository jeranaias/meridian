/* ============================================================
   betaflight-import.js — Import Betaflight CLI dump into ArduPilot params (T3-18)
   Parses Betaflight `set` commands, maps PID names to ArduPilot params,
   shows preview table, writes to v.params + Connection.sendParamSet.
   ============================================================ */

'use strict';

window.BetaflightImport = (function () {

    // Betaflight PID name → { ardupilotParam, scale }
    // Scale: ArduPilot value = Betaflight value / scale
    var PARAM_MAP = [
        // Roll
        { bf: 'p_roll',    ap: 'ATC_RAT_RLL_P', scale: 40   },
        { bf: 'i_roll',    ap: 'ATC_RAT_RLL_I', scale: 400  },
        { bf: 'd_roll',    ap: 'ATC_RAT_RLL_D', scale: 4000 },
        { bf: 'f_roll',    ap: 'ATC_RAT_RLL_FF',scale: 400  },
        // Pitch
        { bf: 'p_pitch',   ap: 'ATC_RAT_PIT_P', scale: 40   },
        { bf: 'i_pitch',   ap: 'ATC_RAT_PIT_I', scale: 400  },
        { bf: 'd_pitch',   ap: 'ATC_RAT_PIT_D', scale: 4000 },
        { bf: 'f_pitch',   ap: 'ATC_RAT_PIT_FF',scale: 400  },
        // Yaw
        { bf: 'p_yaw',     ap: 'ATC_RAT_YAW_P', scale: 40   },
        { bf: 'i_yaw',     ap: 'ATC_RAT_YAW_I', scale: 400  },
        { bf: 'd_yaw',     ap: 'ATC_RAT_YAW_D', scale: 4000 },
        { bf: 'f_yaw',     ap: 'ATC_RAT_YAW_FF',scale: 400  },
    ];

    // ─── Public: inject button into Params view ───────────────

    function injectButton() {
        // Called by ParamsView after rendering the panel
        var actionsEl = document.querySelector('.params-actions');
        if (!actionsEl || document.getElementById('btn-bf-import')) return;

        var btn = document.createElement('button');
        btn.className = 'params-action-btn';
        btn.id = 'btn-bf-import';
        btn.title = 'Import from Betaflight CLI dump';
        btn.setAttribute('aria-label', 'Import PIDs from Betaflight');
        btn.innerHTML =
            '<svg width="12" height="12" viewBox="0 0 12 12" fill="none" stroke="currentColor" stroke-width="1.5">' +
                '<path d="M1 6h5M3 4l3 2-3 2"/>' +
                '<rect x="7" y="1" width="4" height="10" rx="1"/>' +
            '</svg>';

        btn.addEventListener('click', _showImportModal);
        actionsEl.appendChild(btn);
    }

    // ─── Modal: paste or upload ───────────────────────────────

    function _showImportModal() {
        if (document.getElementById('bf-import-overlay')) return;

        var overlay = document.createElement('div');
        overlay.className = 'modal-overlay bf-import-overlay';
        overlay.id = 'bf-import-overlay';

        overlay.innerHTML =
            '<div class="modal-box bf-import-modal">' +
                '<div class="modal-title">Import from Betaflight</div>' +
                '<div class="bf-import-hint">Paste a Betaflight CLI dump (from the <code>diff all</code> or <code>dump</code> command), or upload a file.</div>' +
                '<textarea class="bf-import-text" id="bf-dump-text" placeholder="Paste Betaflight CLI output here&#x0A;e.g.: set p_roll = 45&#x0A;      set i_roll = 90&#x0A;      set d_roll = 26" rows="8"></textarea>' +
                '<div class="bf-import-or">— or —</div>' +
                '<div class="bf-import-file-row">' +
                    '<button class="modal-cancel-btn" id="bf-browse-btn">Browse File...</button>' +
                '</div>' +
                '<div class="modal-actions">' +
                    '<button class="modal-cancel-btn" id="bf-cancel-btn">Cancel</button>' +
                    '<button class="modal-ok-btn" id="bf-parse-btn">Parse</button>' +
                '</div>' +
            '</div>';

        document.body.appendChild(overlay);

        document.getElementById('bf-cancel-btn').addEventListener('click', function () {
            document.body.removeChild(overlay);
        });

        document.getElementById('bf-browse-btn').addEventListener('click', function () {
            var input = document.createElement('input');
            input.type = 'file';
            input.accept = '.txt,.cli,.bf';
            input.addEventListener('change', function () {
                if (!input.files || !input.files[0]) return;
                var reader = new FileReader();
                reader.onload = function (e) {
                    var textArea = document.getElementById('bf-dump-text');
                    if (textArea) textArea.value = e.target.result;
                };
                reader.readAsText(input.files[0]);
            });
            input.click();
        });

        document.getElementById('bf-parse-btn').addEventListener('click', function () {
            var text = document.getElementById('bf-dump-text').value;
            var mapped = _parseDump(text);
            if (mapped.length === 0) {
                alert('No recognized Betaflight PID values found.\nLook for lines like: set p_roll = 45');
                return;
            }
            document.body.removeChild(overlay);
            _showPreviewModal(mapped);
        });
    }

    // ─── Parser ───────────────────────────────────────────────

    function _parseDump(text) {
        var mapped = [];
        var lines = text.split(/\r?\n/);

        // Build lookup: lowercase bf name → map entry
        var lookup = {};
        PARAM_MAP.forEach(function (m) { lookup[m.bf] = m; });

        lines.forEach(function (line) {
            // Match: set <name> = <value>
            var m = line.trim().match(/^set\s+(\w+)\s*=\s*([\d.]+)/i);
            if (!m) return;
            var bfName = m[1].toLowerCase();
            var bfVal  = parseFloat(m[2]);
            var entry  = lookup[bfName];
            if (!entry || isNaN(bfVal)) return;

            var apVal = Math.round((bfVal / entry.scale) * 10000) / 10000;

            mapped.push({
                bfName:  entry.bf,
                bfValue: bfVal,
                apName:  entry.ap,
                apValue: apVal,
                scale:   entry.scale,
            });
        });

        return mapped;
    }

    // ─── Preview modal ────────────────────────────────────────

    function _showPreviewModal(mapped) {
        var overlay = document.createElement('div');
        overlay.className = 'modal-overlay bf-preview-overlay';
        overlay.id = 'bf-preview-overlay';

        var rows = mapped.map(function (m) {
            return '<tr class="bf-table-row">' +
                '<td class="bf-td bf-td-bf">' + m.bfName + '</td>' +
                '<td class="bf-td bf-td-bfval">' + m.bfValue + '</td>' +
                '<td class="bf-td bf-td-arrow">&#x2192;</td>' +
                '<td class="bf-td bf-td-ap">' + m.apName + '</td>' +
                '<td class="bf-td bf-td-apval">' + m.apValue + '</td>' +
                '</tr>';
        }).join('');

        overlay.innerHTML =
            '<div class="modal-box bf-preview-modal">' +
                '<div class="modal-title">Betaflight PID Import — Preview</div>' +
                '<div class="bf-preview-note">' + mapped.length + ' parameter' + (mapped.length !== 1 ? 's' : '') + ' will be written.</div>' +
                '<div class="bf-table-wrapper">' +
                    '<table class="bf-table">' +
                        '<thead>' +
                            '<tr>' +
                                '<th class="bf-th">BF Param</th>' +
                                '<th class="bf-th">BF Value</th>' +
                                '<th class="bf-th"></th>' +
                                '<th class="bf-th">ArduPilot Param</th>' +
                                '<th class="bf-th">Value</th>' +
                            '</tr>' +
                        '</thead>' +
                        '<tbody>' + rows + '</tbody>' +
                    '</table>' +
                '</div>' +
                '<div class="bf-preview-warning">&#x26A0; Verify these values before arming. PID scales differ significantly between flight controllers.</div>' +
                '<div class="modal-actions">' +
                    '<button class="modal-cancel-btn" id="bf-prev-cancel">Cancel</button>' +
                    '<button class="modal-ok-btn" id="bf-prev-apply">Apply to Vehicle</button>' +
                '</div>' +
            '</div>';

        document.body.appendChild(overlay);

        document.getElementById('bf-prev-cancel').addEventListener('click', function () {
            document.body.removeChild(overlay);
        });

        document.getElementById('bf-prev-apply').addEventListener('click', function () {
            document.body.removeChild(overlay);
            _applyParams(mapped);
        });
    }

    // ─── Apply ────────────────────────────────────────────────

    function _applyParams(mapped) {
        var v = meridian.v;
        if (!v) {
            meridian.log('[BF Import] No active vehicle', 'warn');
            return;
        }

        var written = 0;
        mapped.forEach(function (m) {
            v.params[m.apName] = m.apValue;

            if (window.Connection && Connection.sendParamSet) {
                try {
                    Connection.sendParamSet(m.apName, m.apValue);
                    written++;
                } catch (e) {
                    meridian.log('[BF Import] Failed to send ' + m.apName + ': ' + e.message, 'warn');
                }
            } else {
                // Local-only update when disconnected
                written++;
            }
        });

        meridian.log('[BF Import] Applied ' + written + ' PID parameters from Betaflight', 'info');
        meridian.events.emit('param', {});

        // Re-render params list
        if (window.ParamsView) ParamsView.renderPanel();
    }

    // ─── Public API ───────────────────────────────────────────

    return { injectButton };

})();
