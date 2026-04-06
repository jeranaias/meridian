/* ============================================================
   logs-view.js — Logs overlay orchestrator
   Manages tabs for log list, playback, graph, inspector.
   ============================================================ */

'use strict';

window.LogsView = (function () {

    let container = null;
    let activeTab = 'list';

    const TABS = [
        { id: 'list',           label: 'Sessions' },
        { id: 'playback',       label: 'Playback' },
        { id: 'graph',          label: 'Graph' },
        { id: 'inspector',      label: 'Inspector' },
        { id: 'batteries',      label: 'Battery Log' },   // T3-2
        { id: 'script',         label: 'Script' },        // T3-11
        { id: 'photogrammetry', label: 'Photogrammetry' }, // T3-16
    ];

    function init() {
        container = document.querySelector('#panel-logs .panel-body');
        if (!container) return;
        render();
    }

    function render() {
        container.innerHTML = '';

        // Tab bar
        const tabBar = document.createElement('div');
        tabBar.className = 'logs-tabs';
        TABS.forEach(function (tab) {
            const btn = document.createElement('button');
            btn.type = 'button';
            btn.className = 'logs-tab' + (tab.id === activeTab ? ' active' : '');
            btn.textContent = tab.label;
            btn.addEventListener('click', function () {
                activeTab = tab.id;
                render();
            });
            tabBar.appendChild(btn);
        });
        container.appendChild(tabBar);

        // Tab content
        const content = document.createElement('div');
        content.className = 'logs-content';
        container.appendChild(content);

        switch (activeTab) {
            case 'list':
                LogList.render(content);
                // T3-10: inject Analyze buttons after sessions load (async, slight delay)
                if (window.AutoAnalysis) {
                    setTimeout(function () { AutoAnalysis.injectAnalyzeButtons(); }, 400);
                }
                break;
            case 'playback':
                Playback.render(content);
                break;
            case 'graph':
                LogGraph.render(content);
                break;
            case 'inspector':
                MavlinkInspector.render(content);
                break;
            case 'batteries':   // T3-2
                if (window.BatteryLifecycle) BatteryLifecycle.render(content);
                break;
            case 'script':      // T3-11
                if (window.Scripting) Scripting.render(content);
                break;
            case 'photogrammetry': // T3-16
                if (window.Photogrammetry) Photogrammetry.renderLogsSection(content);
                break;
        }
    }

    function refresh() {
        if (container) render();
    }

    // Re-render when panel opens
    meridian.events.on('panel_change', function (panel) {
        if (panel === 'logs') {
            init();
        }
    });

    return { init, refresh };

})();
