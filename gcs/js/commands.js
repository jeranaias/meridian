/* ============================================================
   commands.js — Vehicle Command Interface
   ARM/Disarm, mode change, takeoff, guided goto, mission.
   ============================================================ */

'use strict';

window.Commands = (function () {

    // ---- Confirmation Modal ----

    function showConfirm(title, message, onConfirm) {
        const overlay = document.getElementById('modal-overlay');
        const titleEl = document.getElementById('modal-title');
        const msgEl = document.getElementById('modal-msg');
        const confirmBtn = document.getElementById('modal-confirm');
        const cancelBtn = document.getElementById('modal-cancel');

        titleEl.textContent = title;
        msgEl.textContent = message;
        overlay.classList.add('show');

        function cleanup() {
            overlay.classList.remove('show');
            confirmBtn.removeEventListener('click', onYes);
            cancelBtn.removeEventListener('click', onNo);
        }

        function onYes() { cleanup(); onConfirm(); }
        function onNo() { cleanup(); }

        confirmBtn.addEventListener('click', onYes);
        cancelBtn.addEventListener('click', onNo);
    }

    // ---- ARM / DISARM ----

    function arm() {
        showConfirm('ARM VEHICLE', 'Confirm arming motors. Ensure area is clear.', function () {
            Connection.sendArm();
            window.meridian.log('ARM command sent', 'warn');
        });
    }

    function disarm() {
        showConfirm('DISARM VEHICLE', 'Confirm disarming motors.', function () {
            Connection.sendDisarm();
            window.meridian.log('DISARM command sent', 'info');
        });
    }

    function toggleArm() {
        if (window.meridian.v && window.meridian.v.armed) {
            disarm();
        } else {
            arm();
        }
    }

    // ---- MODE CHANGE ----

    function setMode(mode) {
        Connection.sendSetMode(mode);
        window.meridian.log('Mode change: ' + mode, 'info');
    }

    // ---- TAKEOFF ----

    function takeoff() {
        const altInput = document.getElementById('takeoff-alt');
        const alt = parseFloat(altInput.value) || 10;
        if (alt < 1 || alt > 500) {
            window.meridian.log('Invalid takeoff altitude: ' + alt, 'error');
            return;
        }
        showConfirm('TAKEOFF', 'Takeoff to ' + alt + 'm altitude?', function () {
            // Set mode to GUIDED first, then takeoff
            Connection.sendSetMode('GUIDED');
            setTimeout(function () {
                Connection.sendTakeoff(alt);
                window.meridian.log('Takeoff to ' + alt + 'm', 'warn');
            }, 500);
        });
    }

    // ---- RTL ----

    function rtl() {
        showConfirm('RETURN TO LAUNCH', 'Vehicle will return to home position.', function () {
            Connection.sendRtl();
            window.meridian.log('RTL command sent', 'warn');
        });
    }

    // ---- LAND ----

    function land() {
        showConfirm('LAND', 'Vehicle will land at current position.', function () {
            Connection.sendLand();
            window.meridian.log('LAND command sent', 'warn');
        });
    }

    // ---- MISSION ----

    function uploadMission() {
        const M = window.meridian;
        if (M.waypoints.length === 0) {
            M.log('No waypoints to upload', 'warn');
            return;
        }
        Connection.sendMissionUpload(M.waypoints);
        M.log('Uploading ' + M.waypoints.length + ' waypoints...', 'info');
    }

    function downloadMission() {
        Connection.sendMissionRequestList();
        window.meridian.log('Requesting mission download...', 'info');
    }

    function clearMission() {
        MapManager.clearWaypoints();
        window.meridian.log('Mission cleared', 'info');
    }

    // ---- PARAMETERS ----

    function requestParams() {
        Connection.sendParamRequestList();
        window.meridian.log('Requesting parameters...', 'info');
    }

    function setParam(name, value) {
        Connection.sendParamSet(name, value);
        window.meridian.log('Set ' + name + ' = ' + value, 'info');
    }

    // ---- GUIDED CLICK-TO-FLY ----

    function enableGuidedClick() {
        const M = window.meridian;
        M.guidedClickMode = !M.guidedClickMode;
        if (M.guidedClickMode) {
            // Switch to GUIDED mode first
            Connection.sendSetMode('GUIDED');
            M.log('Guided click-to-fly ENABLED. Click map to send position.', 'info');
        } else {
            M.log('Guided click-to-fly disabled.', 'info');
        }
        return M.guidedClickMode;
    }

    // ---- BIND UI ----

    function bindUI() {
        const btnArm = document.getElementById('btn-arm');
        const modeSelect = document.getElementById('mode-select');
        const btnTakeoff = document.getElementById('btn-takeoff');
        const btnRtl = document.getElementById('btn-rtl');
        const btnLand = document.getElementById('btn-land');
        const btnMission = document.getElementById('btn-mission-panel');
        const btnParams = document.getElementById('btn-param-panel');
        const btnGuided = document.getElementById('btn-guided');

        if (btnArm) btnArm.addEventListener('click', toggleArm);
        if (modeSelect) modeSelect.addEventListener('change', function () {
            setMode(this.value);
        });
        if (btnTakeoff) btnTakeoff.addEventListener('click', takeoff);
        if (btnRtl) btnRtl.addEventListener('click', rtl);
        if (btnLand) btnLand.addEventListener('click', land);

        // Mission panel toggle
        if (btnMission) btnMission.addEventListener('click', function () {
            const panel = document.getElementById('mission-panel');
            panel.classList.toggle('open');
            this.classList.toggle('active');
        });

        // Param panel toggle
        if (btnParams) btnParams.addEventListener('click', function () {
            const panel = document.getElementById('param-panel');
            panel.classList.toggle('open');
            this.classList.toggle('active');
            if (panel.classList.contains('open') && window.meridian.params.length === 0) {
                requestParams();
            }
        });

        // Guided click-to-fly
        if (btnGuided) btnGuided.addEventListener('click', function () {
            const active = enableGuidedClick();
            this.classList.toggle('active', active);
        });

        // Mission panel buttons
        const btnUpload = document.getElementById('btn-mission-upload');
        const btnDownload = document.getElementById('btn-mission-download');
        const btnClear = document.getElementById('btn-mission-clear');
        const btnAddWp = document.getElementById('btn-add-wp');

        if (btnUpload) btnUpload.addEventListener('click', uploadMission);
        if (btnDownload) btnDownload.addEventListener('click', downloadMission);
        if (btnClear) btnClear.addEventListener('click', clearMission);
        if (btnAddWp) btnAddWp.addEventListener('click', function () {
            const active = !MapManager.addWaypointMode;
            MapManager.setAddWaypointMode(active);
            this.classList.toggle('active', active);
            this.textContent = active ? 'Click Map...' : '+ Add WP';
        });

        // Mission panel close
        const missionClose = document.querySelector('#mission-panel .close-btn');
        if (missionClose) missionClose.addEventListener('click', function () {
            document.getElementById('mission-panel').classList.remove('open');
            document.getElementById('btn-mission-panel').classList.remove('active');
        });

        // Param panel close
        const paramClose = document.querySelector('#param-panel .close-btn');
        if (paramClose) paramClose.addEventListener('click', function () {
            document.getElementById('param-panel').classList.remove('open');
            document.getElementById('btn-param-panel').classList.remove('active');
        });

        // Param search
        const paramSearch = document.getElementById('param-search');
        if (paramSearch) paramSearch.addEventListener('input', function () {
            filterParams(this.value);
        });

        // Map overlay buttons
        const btnCenter = document.getElementById('btn-center-vehicle');
        const btnTrail = document.getElementById('btn-toggle-trail');
        if (btnCenter) btnCenter.addEventListener('click', function () {
            const active = MapManager.toggleAutoCenter();
            this.classList.toggle('active', active);
        });
        if (btnTrail) btnTrail.addEventListener('click', function () {
            const active = MapManager.toggleTrail();
            this.classList.toggle('active', active);
        });

        // Connection
        const btnConnect = document.getElementById('btn-connect');
        const connInput = document.getElementById('conn-url');
        const connProto = document.getElementById('conn-proto');

        if (btnConnect) btnConnect.addEventListener('click', function () {
            if (Connection.state === Connection.STATE.CONNECTED) {
                Connection.disconnect();
            } else {
                const url = connInput.value.trim();
                const proto = connProto.value;
                if (url) {
                    Connection.connect(url, proto);
                }
            }
        });
    }

    // ---- Param List Rendering ----

    function renderParams() {
        const list = document.getElementById('param-list');
        if (!list) return;
        const M = window.meridian;
        list.innerHTML = '';

        M.params.forEach(function (p) {
            const li = document.createElement('li');
            li.className = 'param-item';
            li.dataset.name = p.name.toLowerCase();
            li.innerHTML = `
                <span class="p-name" title="${p.name}">${p.name}</span>
                <input class="p-value" type="number" step="any" value="${p.value}" data-param="${p.name}">
                <button class="p-save" data-param="${p.name}">Set</button>
            `;
            list.appendChild(li);
        });

        // Bind save buttons
        list.querySelectorAll('.p-save').forEach(function (btn) {
            btn.addEventListener('click', function () {
                const name = this.dataset.param;
                const input = list.querySelector('input[data-param="' + name + '"]');
                if (input) {
                    setParam(name, parseFloat(input.value));
                }
            });
        });
    }

    function filterParams(query) {
        const list = document.getElementById('param-list');
        if (!list) return;
        const q = query.toLowerCase();
        list.querySelectorAll('.param-item').forEach(function (item) {
            item.style.display = item.dataset.name.includes(q) ? '' : 'none';
        });
    }

    // ---- Waypoint List Rendering ----

    function renderWaypoints() {
        const list = document.getElementById('wp-list');
        if (!list) return;
        const M = window.meridian;
        list.innerHTML = '';

        M.waypoints.forEach(function (wp, i) {
            const li = document.createElement('li');
            li.className = 'wp-item';
            li.draggable = true;
            li.dataset.index = i;
            li.innerHTML = `
                <span class="wp-num">${i + 1}</span>
                <span class="wp-coords">${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)}</span>
                <span class="wp-alt">${wp.alt}m</span>
                <button class="wp-remove" data-index="${i}">&times;</button>
            `;
            list.appendChild(li);
        });

        // Remove buttons
        list.querySelectorAll('.wp-remove').forEach(function (btn) {
            btn.addEventListener('click', function () {
                MapManager.removeWaypoint(parseInt(this.dataset.index));
            });
        });

        // Drag-to-reorder
        setupDragReorder(list);
    }

    function setupDragReorder(list) {
        let dragIdx = null;

        list.addEventListener('dragstart', function (e) {
            dragIdx = parseInt(e.target.dataset.index);
            e.target.style.opacity = '0.4';
        });

        list.addEventListener('dragover', function (e) {
            e.preventDefault();
        });

        list.addEventListener('drop', function (e) {
            e.preventDefault();
            const target = e.target.closest('.wp-item');
            if (!target || dragIdx === null) return;
            const dropIdx = parseInt(target.dataset.index);
            if (dragIdx === dropIdx) return;

            const M = window.meridian;
            const item = M.waypoints.splice(dragIdx, 1)[0];
            M.waypoints.splice(dropIdx, 0, item);
            M.waypoints.forEach(function (wp, i) { wp.seq = i; });
            MapManager.refreshWaypoints();
            renderWaypoints();
        });

        list.addEventListener('dragend', function (e) {
            e.target.style.opacity = '';
            dragIdx = null;
        });
    }

    // ---- Public API ----

    return {
        bindUI,
        renderParams,
        renderWaypoints,
        showConfirm,
    };

})();
