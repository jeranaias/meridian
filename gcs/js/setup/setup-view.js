/* ============================================================
   setup-view.js — Setup overlay orchestrator
   Renders commissioning checklist in panel-setup .panel-body.
   Click item to expand inline config form.
   ============================================================ */

'use strict';

window.SetupView = (function () {

    var expandedItem = null;
    var initialized = false;

    function init() {
        if (initialized) return;
        initialized = true;

        meridian.events.on('panel_change', function (panel) {
            if (panel === 'setup') renderChecklist();
        });

        // Re-render when params change
        meridian.events.on('param', function () {
            if (Router.active === 'setup' && !expandedItem) {
                renderChecklist();
            }
        });
    }

    function getBody() {
        var panel = document.querySelector('#panel-setup .panel-body');
        return panel;
    }

    function renderChecklist() {
        var body = getBody();
        if (!body) return;

        var v = meridian.v;
        var params = v ? v.params : {};
        var results = Checklist.evaluate(params);
        var incompleteCount = Checklist.incompleteRequiredCount(params);

        // T3-20: Pre-flight regulatory checklist rendered above commissioning list
        var pfSection = document.getElementById('preflight-reg-section');
        if (!pfSection) {
            pfSection = document.createElement('div');
            pfSection.id = 'preflight-reg-section';
            pfSection.className = 'setup-preflight-section';

            var pfHeader = document.createElement('div');
            pfHeader.className = 'setup-section-label preflight-toggle';
            pfHeader.innerHTML =
                '<span>Pre-Flight Checklist</span>' +
                '<svg class="preflight-chevron" width="10" height="10" viewBox="0 0 10 10" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M2 4l3 3 3-3"/></svg>';
            pfSection.appendChild(pfHeader);

            var pfContent = document.createElement('div');
            pfContent.id = 'preflight-reg-content';
            pfSection.appendChild(pfContent);

            pfHeader.addEventListener('click', function () {
                pfContent.style.display = pfContent.style.display === 'none' ? '' : 'none';
                pfHeader.querySelector('.preflight-chevron').style.transform =
                    pfContent.style.display === 'none' ? 'rotate(-90deg)' : '';
            });

            body.innerHTML = '';
            body.appendChild(pfSection);

            if (window.RegulatoryChecklist) {
                RegulatoryChecklist.init();
                RegulatoryChecklist.render(pfContent);
            }
        }

        var html = '<div class="setup-checklist">';

        // Required items
        html += '<div class="setup-section-label">Required</div>';
        for (var i = 0; i < results.length; i++) {
            var r = results[i];
            if (!r.required) continue;
            html += renderItem(r);
        }

        // Optional items
        html += '<div class="setup-section-label optional">Optional</div>';
        for (var j = 0; j < results.length; j++) {
            var o = results[j];
            if (o.required) continue;
            html += renderItem(o);
        }

        html += '</div>';

        // Warning bar
        if (incompleteCount > 0) {
            html += '<div class="setup-warning-bar">';
            html += '<svg width="14" height="14" viewBox="0 0 14 14" fill="none"><path d="M7 1L13 12H1L7 1z" stroke="var(--c-warning)" stroke-width="1.5"/><line x1="7" y1="5" x2="7" y2="8" stroke="var(--c-warning)" stroke-width="1.5"/><circle cx="7" cy="10" r="0.7" fill="var(--c-warning)"/></svg>';
            html += '<span>' + incompleteCount + ' required item' + (incompleteCount > 1 ? 's' : '') + ' not complete</span>';
            html += '</div>';
        } else {
            html += '<div class="setup-ready-bar">';
            html += '<svg width="14" height="14" viewBox="0 0 14 14" fill="none"><circle cx="7" cy="7" r="6" stroke="var(--c-safe)" stroke-width="1.5"/><path d="M4 7l2 2 4-4" stroke="var(--c-safe)" stroke-width="1.5" fill="none"/></svg>';
            html += '<span>All required items complete</span>';
            html += '</div>';
        }

        // Preserve preflight section; replace or append commissioning checklist
        var existingChecklist = body.querySelector('#commissioning-checklist-wrapper');
        if (existingChecklist) {
            existingChecklist.innerHTML = html;
        } else {
            var wrapper = document.createElement('div');
            wrapper.id = 'commissioning-checklist-wrapper';
            wrapper.innerHTML = html;
            body.appendChild(wrapper);
        }
        // Wire click handlers — scope to the wrapper
        var wrapperEl = body.querySelector('#commissioning-checklist-wrapper');

        wrapperEl.querySelectorAll('.setup-item').forEach(function (el) {
            el.addEventListener('click', function () {
                var id = el.dataset.itemId;
                if (expandedItem === id) {
                    expandedItem = null;
                    renderChecklist();
                } else {
                    expandedItem = id;
                    renderExpanded(id);
                }
            });
        });
    }

    function renderItem(r) {
        var statusClass = 'status-' + r.status;
        var icon = '';
        if (r.status === 'complete') icon = '<svg width="16" height="16" viewBox="0 0 16 16" fill="none"><circle cx="8" cy="8" r="7" stroke="var(--c-safe)" stroke-width="1.5"/><path d="M5 8l2 2 4-4" stroke="var(--c-safe)" stroke-width="1.5" fill="none"/></svg>';
        else if (r.status === 'warning') icon = '<svg width="16" height="16" viewBox="0 0 16 16" fill="none"><circle cx="8" cy="8" r="7" stroke="var(--c-warning)" stroke-width="1.5"/><line x1="8" y1="4" x2="8" y2="9" stroke="var(--c-warning)" stroke-width="1.5"/><circle cx="8" cy="11" r="0.8" fill="var(--c-warning)"/></svg>';
        else if (r.status === 'incomplete') icon = '<svg width="16" height="16" viewBox="0 0 16 16" fill="none"><circle cx="8" cy="8" r="7" stroke="' + (r.required ? 'var(--c-emergency)' : 'var(--c-neutral-dim)') + '" stroke-width="1.5" stroke-dasharray="3 2"/></svg>';
        else icon = '<svg width="16" height="16" viewBox="0 0 16 16" fill="none"><line x1="3" y1="8" x2="13" y2="8" stroke="var(--c-neutral-dim)" stroke-width="1.5"/></svg>';

        var html = '<div class="setup-item ' + statusClass + (expandedItem === r.id ? ' expanded' : '') + '" data-item-id="' + r.id + '">';
        html += '<div class="setup-item-icon">' + icon + '</div>';
        html += '<div class="setup-item-info">';
        html += '<div class="setup-item-name">' + r.name + '</div>';
        html += '<div class="setup-item-text">' + r.text + '</div>';
        html += '</div>';
        html += '<div class="setup-item-arrow">';
        html += '<svg width="10" height="10" viewBox="0 0 10 10" fill="none" stroke="var(--c-neutral-dim)" stroke-width="1.5"><path d="M3 2l4 3-4 3"/></svg>';
        html += '</div>';
        html += '</div>';
        return html;
    }

    function renderExpanded(id) {
        var body = getBody();
        if (!body) return;

        var v = meridian.v;
        var params = v ? v.params : {};

        function onParamChange(name, value) {
            if (v) {
                v.params[name] = value;
                meridian.events.emit('param', { name: name, value: value });
            }
        }

        // Header with back button
        var html = '<div class="setup-expanded">';
        html += '<button class="setup-back-btn" id="setup-back-btn">';
        html += '<svg width="12" height="12" viewBox="0 0 12 12" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M8 2L4 6l4 4"/></svg>';
        html += ' Back to Checklist';
        html += '</button>';
        html += '<div class="setup-expanded-body" id="setup-expanded-body"></div>';
        html += '</div>';

        body.innerHTML = html;

        var expBody = document.getElementById('setup-expanded-body');

        // Render the appropriate component
        switch (id) {
            case 'frame':
                FrameSetup.render(expBody, params, onParamChange);
                break;
            case 'accel':
                AccelCal.render(expBody);
                break;
            case 'compass':
                CompassCal.render(expBody);
                break;
            case 'radio':
                RadioCal.render(expBody);
                break;
            case 'modes':
                FlightModes.render(expBody, params, onParamChange);
                break;
            case 'failsafe':
                Failsafe.render(expBody, params, onParamChange);
                break;
            case 'battery':
                BatterySetup.render(expBody, params, onParamChange);
                break;
            case 'motor_test':
                MotorTest.render(expBody);
                break;
            case 'firmware':    // T3-19
                if (window.FirmwareSetup) FirmwareSetup.render(expBody);
                else expBody.innerHTML = '<div class="setup-form-section"><div class="cal-description">Firmware module not loaded.</div></div>';
                break;
            default:
                expBody.innerHTML = '<div class="setup-form-section"><div class="setup-form-title">' + id + '</div><div class="cal-description">Configuration not available in demo mode.</div></div>';
                break;
        }

        // Wire back button
        var backBtn = document.getElementById('setup-back-btn');
        if (backBtn) {
            backBtn.addEventListener('click', function () {
                expandedItem = null;
                renderChecklist();
            });
        }
    }

    return { init, renderChecklist };

})();
