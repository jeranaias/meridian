/* ============================================================
   multi-vehicle.js — Multi-vehicle selector
   Shows active vehicle sysid with dropdown, status dots,
   switches activeVehicleId and emits vehicle_switch event.
   ============================================================ */

'use strict';

window.MultiVehicle = (function () {

    var selectorEl = null;
    var _closeHandler = null;

    function init() {
        selectorEl = document.getElementById('vehicle-selector');
        if (!selectorEl) return;

        // Render initial state
        render();

        // Update on heartbeat (may discover new vehicles)
        meridian.events.on('heartbeat', function () {
            render();
        });

        // Update on vehicle switch
        meridian.events.on('vehicle_switch', function () {
            render();
        });
    }

    function render() {
        if (!selectorEl) return;

        var vehicles = meridian.vehicles;
        var activeId = meridian.activeVehicleId;
        var ids = Object.keys(vehicles).map(Number).sort();

        // Single vehicle — compact display
        if (ids.length <= 1) {
            var v = vehicles[activeId];
            var statusClass = getStatusClass(v);
            selectorEl.innerHTML =
                '<div class="mv-single">' +
                '<span class="mv-dot ' + statusClass + '"></span>' +
                '<span class="mv-id">V' + activeId + '</span>' +
                '</div>';
            selectorEl.className = 'vehicle-selector';
            return;
        }

        // Multiple vehicles — dropdown
        var html = '<div class="mv-active" id="mv-active-btn">';
        var activeV = vehicles[activeId];
        var activeStatus = getStatusClass(activeV);
        html += '<span class="mv-dot ' + activeStatus + '"></span>';
        html += '<span class="mv-id">V' + activeId + '</span>';
        html += '<span class="mv-caret">&#x25BE;</span>';
        html += '</div>';

        // Dropdown
        html += '<div class="mv-dropdown" id="mv-dropdown">';
        for (var i = 0; i < ids.length; i++) {
            var id = ids[i];
            var veh = vehicles[id];
            var cls = getStatusClass(veh);
            var sel = (id === activeId) ? ' mv-item-active' : '';
            html += '<div class="mv-item' + sel + '" data-sysid="' + id + '">';
            html += '<span class="mv-dot ' + cls + '"></span>';
            html += '<span class="mv-item-label">Vehicle ' + id + '</span>';
            html += '<span class="mv-item-mode">' + (veh ? veh.modeName : 'N/A') + '</span>';
            html += '</div>';
        }
        html += '</div>';

        selectorEl.innerHTML = html;
        selectorEl.className = 'vehicle-selector multi';

        // Wire events
        var activeBtn = document.getElementById('mv-active-btn');
        var dropdown = document.getElementById('mv-dropdown');

        if (activeBtn && dropdown) {
            activeBtn.addEventListener('click', function (e) {
                e.stopPropagation();
                dropdown.classList.toggle('open');
            });

            var items = dropdown.querySelectorAll('.mv-item');
            for (var j = 0; j < items.length; j++) {
                items[j].addEventListener('click', function () {
                    var newId = parseInt(this.dataset.sysid, 10);
                    switchVehicle(newId);
                    dropdown.classList.remove('open');
                });
            }

            // Close on outside click — remove previous listener to prevent leak
            if (_closeHandler) document.removeEventListener('click', _closeHandler);
            _closeHandler = function () {
                if (dropdown) dropdown.classList.remove('open');
            };
            document.addEventListener('click', _closeHandler);
        }
    }

    function switchVehicle(sysid) {
        if (!meridian.vehicles[sysid]) return;
        if (sysid === meridian.activeVehicleId) return;

        meridian.activeVehicleId = sysid;
        meridian.events.emit('vehicle_switch', sysid);
        meridian.log('Switched to Vehicle ' + sysid, 'info');
    }

    function getStatusClass(v) {
        if (!v || !v.connected) return 'disconnected';
        if (v.armed) return 'armed';
        return 'connected';
    }

    return { init: init, render: render, switchVehicle: switchVehicle };

})();
