/* ============================================================
   stanag-notes.js — STANAG 4586 Compatibility Notes
   T3-14: Informational panel showing LOI mapping and current
   compliance status. Not a full STANAG implementation.
   Accessible from Settings panel.
   ============================================================ */

'use strict';

window.StanagNotes = (function () {

    // Level of Interoperability definitions
    const LOI_LEVELS = [
        {
            level: 1,
            name:  'Indirect Receipt of ISR Data',
            description: 'UAV payload data received indirectly via a separate data link or relay. GCS does not directly communicate with the UAV.',
            meridianStatus: 'supported',
            statusNote: 'Meridian can receive telemetry via a MAVLink proxy or relay. Indirect receipt works with any intermediary that forwards MAVLink.',
        },
        {
            level: 2,
            name:  'Direct Receipt of ISR Data (DLI)',
            description: 'Direct UAV payload data link interface (DLI) between the UAV and GCS for sensor/payload data.',
            meridianStatus: 'not-implemented',
            statusNote: 'Requires a STANAG 4586-compliant DLI adapter. Not implemented — Meridian uses MAVLink only.',
        },
        {
            level: 3,
            name:  'Direct Control of UAV Payload (DLI + VSM)',
            description: 'Full payload control including direct UAV control via Vehicle Specific Module (VSM) interface.',
            meridianStatus: 'not-implemented',
            statusNote: 'Requires a STANAG VSM interface layer. Not implemented.',
        },
        {
            level: 4,
            name:  'Direct Control of UAV (Flight-Critical)',
            description: 'Direct flight control of the UAV including navigation commands, beyond visual line of sight operations.',
            meridianStatus: 'not-implemented',
            statusNote: 'Requires full weapon/vehicle command interface per STANAG 4586 DLI. Not implemented.',
        },
        {
            level: 5,
            name:  'Direct Control of Weapons/Payloads',
            description: 'Full control including weapons release and tactical payload management.',
            meridianStatus: 'not-implemented',
            statusNote: 'Requires mil-spec weapon interface. Not implemented. Out of scope for Meridian.',
        },
    ];

    // What Meridian currently complies with and what it doesn't
    const COMPLIANCE_SUMMARY = [
        { item: 'Heartbeat (armed state, mode)',         compliant: true  },
        { item: 'Position telemetry (lat/lon/alt)',      compliant: true  },
        { item: 'MAVLink message framing and CRC',       compliant: true  },
        { item: 'Multi-vehicle (sysid segregation)',     compliant: true  },
        { item: 'STANAG DLI data link interface',        compliant: false },
        { item: 'VSM Vehicle Specific Module',           compliant: false },
        { item: 'STANAG security / message signing',     compliant: false },
        { item: 'Platform encryption (AES-256 datalink)',compliant: false },
        { item: 'NATO data format (NSILI / GMTI)',       compliant: false },
    ];

    // -------------------------------------------------------------------------
    // Render — creates the STANAG info panel in the given container
    // -------------------------------------------------------------------------

    function render(container) {
        container.innerHTML = '';
        container.className = 'stanag-panel';

        // Header
        var header = document.createElement('div');
        header.className = 'stanag-header';
        header.innerHTML =
            '<div class="stanag-title">STANAG 4586 Compatibility Notes</div>' +
            '<div class="stanag-subtitle">Informational only — not a conformance certification</div>';
        container.appendChild(header);

        // LOI table
        var loiSection = document.createElement('div');
        loiSection.className = 'stanag-section';
        var loiTitle = document.createElement('div');
        loiTitle.className = 'stanag-section-title';
        loiTitle.textContent = 'Level of Interoperability (LOI)';
        loiSection.appendChild(loiTitle);

        LOI_LEVELS.forEach(function (loi) {
            var row = document.createElement('div');
            row.className = 'stanag-loi-row';

            var badge = document.createElement('div');
            badge.className = 'stanag-loi-badge loi-' + loi.meridianStatus;
            badge.textContent = 'LOI ' + loi.level;

            var body = document.createElement('div');
            body.className = 'stanag-loi-body';

            var nameEl = document.createElement('div');
            nameEl.className = 'stanag-loi-name';
            nameEl.textContent = loi.name;

            var descEl = document.createElement('div');
            descEl.className = 'stanag-loi-desc';
            descEl.textContent = loi.description;

            var statusEl = document.createElement('div');
            statusEl.className = 'stanag-loi-status stanag-status-' + loi.meridianStatus;

            var icon = loi.meridianStatus === 'supported'
                ? '\u2713 '   // checkmark
                : '\u2715 ';  // x
            statusEl.textContent = icon + loi.statusNote;

            body.appendChild(nameEl);
            body.appendChild(descEl);
            body.appendChild(statusEl);
            row.appendChild(badge);
            row.appendChild(body);
            loiSection.appendChild(row);
        });

        container.appendChild(loiSection);

        // Current compliance checklist
        var compSection = document.createElement('div');
        compSection.className = 'stanag-section';
        var compTitle = document.createElement('div');
        compTitle.className = 'stanag-section-title';
        compTitle.textContent = 'Current Meridian Compliance Status';
        compSection.appendChild(compTitle);

        var compList = document.createElement('div');
        compList.className = 'stanag-compliance-list';
        COMPLIANCE_SUMMARY.forEach(function (item) {
            var row = document.createElement('div');
            row.className = 'stanag-compliance-row';

            var mark = document.createElement('span');
            mark.className = item.compliant ? 'stanag-check' : 'stanag-cross';
            mark.textContent = item.compliant ? '\u2713' : '\u2715';

            var label = document.createElement('span');
            label.className = 'stanag-compliance-label';
            label.textContent = item.item;

            row.appendChild(mark);
            row.appendChild(label);
            compList.appendChild(row);
        });
        compSection.appendChild(compList);
        container.appendChild(compSection);

        // Disclaimer
        var disclaimer = document.createElement('div');
        disclaimer.className = 'stanag-disclaimer';
        disclaimer.textContent =
            'STANAG 4586 full compliance requires mil-spec development, ' +
            'formal verification, and NATO certification. These notes are ' +
            'provided for integration planning only.';
        container.appendChild(disclaimer);

        // Export button
        var exportRow = document.createElement('div');
        exportRow.className = 'stanag-export-row';
        var exportBtn = document.createElement('button');
        exportBtn.className = 'offline-btn draw';
        exportBtn.textContent = '\u21E9 Export Compliance Report';
        exportBtn.addEventListener('click', _exportReport);
        exportRow.appendChild(exportBtn);
        container.appendChild(exportRow);
    }

    // -------------------------------------------------------------------------
    // Export compliance report as plain text
    // -------------------------------------------------------------------------

    function _exportReport() {
        var lines = [];
        lines.push('MERIDIAN GCS — STANAG 4586 COMPATIBILITY REPORT');
        lines.push('Generated: ' + new Date().toISOString());
        lines.push('Version: ' + (meridian.VERSION || 'unknown'));
        lines.push('');
        lines.push('=== LEVEL OF INTEROPERABILITY ===');
        lines.push('');
        LOI_LEVELS.forEach(function (loi) {
            var supported = loi.meridianStatus === 'supported';
            lines.push('LOI ' + loi.level + ': ' + loi.name);
            lines.push('  Status: ' + (supported ? 'SUPPORTED' : 'NOT IMPLEMENTED'));
            lines.push('  Note: ' + loi.statusNote);
            lines.push('');
        });
        lines.push('=== COMPLIANCE CHECKLIST ===');
        lines.push('');
        COMPLIANCE_SUMMARY.forEach(function (item) {
            lines.push((item.compliant ? '[PASS] ' : '[FAIL] ') + item.item);
        });
        lines.push('');
        lines.push('DISCLAIMER: This report is informational only and does not');
        lines.push('constitute a formal STANAG conformance certification.');

        var text = lines.join('\n');
        var blob = new Blob([text], { type: 'text/plain' });
        var url = URL.createObjectURL(blob);
        var a = document.createElement('a');
        a.href = url;
        a.download = 'meridian_stanag_4586_notes_' +
            new Date().toISOString().slice(0, 10) + '.txt';
        a.click();
        URL.revokeObjectURL(url);
        meridian.log('STANAG 4586 compliance report exported', 'info');
    }

    // -------------------------------------------------------------------------
    // Settings panel section (called from settings.js)
    // -------------------------------------------------------------------------

    function renderSettingsSection(wrapper, createSectionHeader) {
        wrapper.appendChild(createSectionHeader('STANAG 4586'));

        var row = document.createElement('div');
        row.className = 'settings-field';
        var btn = document.createElement('button');
        btn.className = 'offline-btn draw';
        btn.textContent = 'View STANAG 4586 Compatibility Notes';
        btn.style.marginTop = '4px';
        btn.addEventListener('click', function () {
            _showModal();
        });
        row.appendChild(btn);
        wrapper.appendChild(row);
    }

    function _showModal() {
        // Use the existing Modal if available, else create a floating div
        var content = document.createElement('div');
        render(content);

        // Always use fallback overlay (Modal doesn't have a .show method)
        {
            // Fallback: inject into a full-panel overlay
            var overlay = document.getElementById('stanag-overlay');
            if (!overlay) {
                overlay = document.createElement('div');
                overlay.id = 'stanag-overlay';
                overlay.style.cssText =
                    'position:fixed;inset:0;background:var(--c-bg,#080b10);z-index:9999;' +
                    'overflow:auto;padding:24px;';
                var closeBtn = document.createElement('button');
                closeBtn.textContent = '\u2715 Close';
                closeBtn.className = 'offline-btn draw';
                closeBtn.style.marginBottom = '16px';
                closeBtn.addEventListener('click', function () {
                    overlay.remove();
                });
                overlay.appendChild(closeBtn);
                overlay.appendChild(content);
                document.body.appendChild(overlay);
            } else {
                // Refresh content
                overlay.innerHTML = '';
                var closeBtn2 = document.createElement('button');
                closeBtn2.textContent = '\u2715 Close';
                closeBtn2.className = 'offline-btn draw';
                closeBtn2.style.marginBottom = '16px';
                closeBtn2.addEventListener('click', function () { overlay.remove(); });
                overlay.appendChild(closeBtn2);
                overlay.appendChild(content);
            }
        }
    }

    return {
        render,
        renderSettingsSection,
    };

})();
