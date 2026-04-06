/* ============================================================
   spray-widget.js — Agricultural spray system telemetry widget
   T3-1: Displays flow rate, tank level, spray status, area sprayed.
   Reads: v.sprayFlowRate, v.sprayTankLevel, v.sprayActive, v.sprayArea
   Renders in HUD top-left area, toggled by meridian.settings.showSprayWidget
   ============================================================ */

'use strict';

window.SprayWidget = (function () {

    var _container = null;
    var _initialized = false;

    // ─── Init ─────────────────────────────────────────────────

    function init() {
        if (_initialized) return;
        _initialized = true;

        _buildContainer();

        meridian.events.on('telemetry', _onTelemetry);
        meridian.events.on('settings_change', _onSettingsChange);

        _updateVisibility();
    }

    // ─── DOM ──────────────────────────────────────────────────

    function _buildContainer() {
        var hud = document.getElementById('hud');
        if (!hud) return;

        _container = document.createElement('div');
        _container.className = 'spray-widget glass';
        _container.id = 'spray-widget';
        _container.setAttribute('title', 'Agricultural spray telemetry');

        _container.innerHTML =
            '<div class="spray-header">' +
                '<svg class="spray-icon" width="13" height="13" viewBox="0 0 13 13" fill="none">' +
                    '<path d="M2 8 Q4 3 8 3" stroke="currentColor" stroke-width="1.5" fill="none"/>' +
                    '<path d="M8 1v4M6.5 2.5h3" stroke="currentColor" stroke-width="1.4"/>' +
                    '<circle cx="3" cy="10" r="1" fill="var(--c-spray-on)"/>' +
                    '<circle cx="6" cy="11" r="1" fill="var(--c-spray-on)"/>' +
                    '<circle cx="9" cy="10" r="1" fill="var(--c-spray-on)"/>' +
                '</svg>' +
                '<span class="spray-title">SPRAY</span>' +
                '<span class="spray-status-badge" id="spray-status-badge">OFF</span>' +
            '</div>' +
            '<div class="spray-metrics">' +
                '<div class="spray-metric">' +
                    '<div class="spray-metric-value" id="spray-flow">--</div>' +
                    '<div class="spray-metric-label">L/min</div>' +
                '</div>' +
                '<div class="spray-metric">' +
                    '<div class="spray-tank-track">' +
                        '<div class="spray-tank-fill" id="spray-tank-fill"></div>' +
                    '</div>' +
                    '<div class="spray-metric-value" id="spray-tank">--%</div>' +
                    '<div class="spray-metric-label">TANK</div>' +
                '</div>' +
                '<div class="spray-metric">' +
                    '<div class="spray-metric-value" id="spray-area">--</div>' +
                    '<div class="spray-metric-label">ha</div>' +
                '</div>' +
            '</div>';

        hud.appendChild(_container);
    }

    // ─── Telemetry Update ─────────────────────────────────────

    function _onTelemetry(v) {
        if (!_container || !v) return;
        if (!meridian.settings.showSprayWidget) return;

        var flowEl   = document.getElementById('spray-flow');
        var tankEl   = document.getElementById('spray-tank');
        var fillEl   = document.getElementById('spray-tank-fill');
        var areaEl   = document.getElementById('spray-area');
        var badgeEl  = document.getElementById('spray-status-badge');
        if (!flowEl) return;

        // Flow rate
        var flow = v.sprayFlowRate;
        flowEl.textContent = (typeof flow === 'number') ? flow.toFixed(1) : '--';

        // Tank level
        var tank = v.sprayTankLevel;
        if (typeof tank === 'number') {
            tankEl.textContent = tank.toFixed(0) + '%';
            fillEl.style.width = Math.max(0, Math.min(100, tank)) + '%';
            fillEl.className = 'spray-tank-fill' +
                (tank < 10 ? ' critical' : tank < 25 ? ' warning' : '');
        } else {
            tankEl.textContent = '--%';
            fillEl.style.width = '0%';
            fillEl.className = 'spray-tank-fill';
        }

        // Spray active status
        var active = v.sprayActive;
        if (badgeEl) {
            if (active === true || active === 1) {
                badgeEl.textContent = 'ON';
                badgeEl.className = 'spray-status-badge on';
                _container.classList.add('spraying');
            } else {
                badgeEl.textContent = 'OFF';
                badgeEl.className = 'spray-status-badge';
                _container.classList.remove('spraying');
            }
        }

        // Area sprayed
        var area = v.sprayArea;
        areaEl.textContent = (typeof area === 'number') ? area.toFixed(2) : '--';
    }

    // ─── Visibility ───────────────────────────────────────────

    function _onSettingsChange(change) {
        if (change.key === 'showSprayWidget') _updateVisibility();
    }

    function _updateVisibility() {
        if (!_container) return;
        var show = meridian.settings.showSprayWidget;
        _container.style.display = show ? '' : 'none';
    }

    // Public
    return { init };

})();
