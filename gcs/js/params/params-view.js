/* ============================================================
   params-view.js — Params overlay orchestrator
   Renders in panel-params .panel-body.
   Search bar at top, tabbed between list and tuning.
   ============================================================ */

'use strict';

window.ParamsView = (function () {

    var initialized = false;
    var activeTab = 'list'; // 'list' | 'tuning'
    var searchFilter = '';

    function init() {
        if (initialized) return;
        initialized = true;

        meridian.events.on('panel_change', function (panel) {
            if (panel === 'params') {
                renderPanel();
            } else {
                Tuning.destroy();
            }
        });

        // Re-render list when params change (debounced)
        var renderTimeout = null;
        meridian.events.on('param', function () {
            if (Router.active !== 'params') return;
            if (renderTimeout) clearTimeout(renderTimeout);
            renderTimeout = setTimeout(function () {
                if (activeTab === 'list') renderParamContent();
            }, 100);
        });
    }

    function getBody() {
        return document.querySelector('#panel-params .panel-body');
    }

    function renderPanel() {
        var body = getBody();
        if (!body) return;

        var html = '';

        // Tabs
        html += '<div class="params-tabs">';
        html += '<button class="params-tab' + (activeTab === 'list' ? ' active' : '') + '" data-tab="list">All Parameters</button>';
        html += '<button class="params-tab' + (activeTab === 'tuning' ? ' active' : '') + '" data-tab="tuning">PID Tuning</button>';
        html += '</div>';

        if (activeTab === 'list') {
            // Search bar
            html += '<div class="params-search-bar">';
            html += '<input type="text" class="params-search" id="params-search" placeholder="Search parameters..." value="' + searchFilter + '">';
            html += '<div class="params-actions">';
            html += '<button class="params-action-btn" id="params-load-btn" title="Load .param file">';
            html += '<svg width="12" height="12" viewBox="0 0 12 12" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M2 8v2h8V8M6 2v6M3 5l3 3 3-3"/></svg>';
            html += '</button>';
            html += '<button class="params-action-btn" id="params-save-btn" title="Save .param file">';
            html += '<svg width="12" height="12" viewBox="0 0 12 12" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M2 4V2h8v2M6 10V4M3 7l3-3 3 3"/></svg>';
            html += '</button>';
            html += '</div>';
            html += '</div>';

            // List container
            html += '<div id="param-list-container"></div>';
        } else {
            html += '<div id="tuning-container"></div>';
        }

        body.innerHTML = html;

        // Wire tabs
        body.querySelectorAll('.params-tab').forEach(function (tab) {
            tab.addEventListener('click', function () {
                activeTab = tab.dataset.tab;
                Tuning.destroy();
                renderPanel();
            });
        });

        if (activeTab === 'list') {
            renderParamContent();

            // Wire search
            var searchInput = document.getElementById('params-search');
            if (searchInput) {
                searchInput.addEventListener('input', function () {
                    searchFilter = searchInput.value;
                    renderParamContent();
                });
                searchInput.focus();
            }

            // Wire load/save
            var loadBtn = document.getElementById('params-load-btn');
            if (loadBtn) {
                loadBtn.addEventListener('click', function () {
                    var input = document.createElement('input');
                    input.type = 'file';
                    input.accept = '.param,.parm,.txt';
                    input.addEventListener('change', function () {
                        if (input.files.length > 0) {
                            ParamList.importParams(input.files[0]);
                            setTimeout(renderParamContent, 500);
                        }
                    });
                    input.click();
                });
            }

            var saveBtn = document.getElementById('params-save-btn');
            if (saveBtn) {
                saveBtn.addEventListener('click', function () {
                    ParamList.exportParams();
                });
            }

            // T3-18: Betaflight import button
            if (window.BetaflightImport) BetaflightImport.injectButton();
        } else {
            var tuningContainer = document.getElementById('tuning-container');
            if (tuningContainer) Tuning.render(tuningContainer);
        }
    }

    function renderParamContent() {
        var listContainer = document.getElementById('param-list-container');
        if (listContainer) {
            ParamList.render(listContainer, searchFilter);
        }
    }

    return { init, renderPanel };

})();
