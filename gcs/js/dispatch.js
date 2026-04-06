/* ============================================================
   dispatch.js — Automated mission dispatch engine (T3-8)
   Simple mission queue. Queues current plan for a vehicle,
   auto-uploads when vehicle is armed and ready.
   ============================================================ */

'use strict';

window.Dispatch = (function () {

    // Queue: array of { id, vehicleId, mission (waypoint array), status, createdAt }
    var queue = [];
    var nextId = 1;
    var statusBarEl = null;
    var pollTimer = null;

    // Status values
    var STATUS = {
        PENDING:  'pending',
        ACTIVE:   'active',
        COMPLETE: 'complete',
        FAILED:   'failed',
    };

    // ---- Init ----

    function init() {
        buildStatusBar();
        startPoller();

        // Wire the "Dispatch" button injected by plan-view integration
        meridian.events.on('panel_change', function (panel) {
            if (panel === 'plan') {
                setTimeout(injectDispatchButton, 80);
            }
        });

        // Listen for mission uploads completing successfully
        meridian.events.on('mission_ack', function (msg) {
            if (msg.result === 0) {
                onMissionAckSuccess();
            } else {
                onMissionAckFail(msg.result);
            }
        });

        // Listen for heartbeat to auto-dispatch
        meridian.events.on('heartbeat', function (v) {
            if (!v) return;
            maybeTriggerDispatch(v.sysid, v);
        });
    }

    // ---- Status Bar (action-bar injection) ----

    function buildStatusBar() {
        var actionBar = document.getElementById('action-bar');
        if (!actionBar || document.getElementById('dispatch-status')) return;

        statusBarEl = document.createElement('div');
        statusBarEl.id = 'dispatch-status';
        statusBarEl.className = 'dispatch-status';
        statusBarEl.style.display = 'none';
        statusBarEl.title = 'Dispatch queue status';

        // Insert before the spacer (flex:1 div)
        var spacer = actionBar.querySelector('[style*="flex:1"]') ||
                     actionBar.querySelector('[style*="flex: 1"]');
        if (spacer) {
            actionBar.insertBefore(statusBarEl, spacer);
        } else {
            actionBar.appendChild(statusBarEl);
        }
    }

    function updateStatusBar() {
        if (!statusBarEl) return;

        var pending  = queue.filter(function (q) { return q.status === STATUS.PENDING; }).length;
        var active   = queue.filter(function (q) { return q.status === STATUS.ACTIVE; }).length;
        var complete = queue.filter(function (q) { return q.status === STATUS.COMPLETE; }).length;
        var failed   = queue.filter(function (q) { return q.status === STATUS.FAILED; }).length;

        if (pending === 0 && active === 0 && complete === 0 && failed === 0) {
            statusBarEl.style.display = 'none';
            return;
        }

        statusBarEl.style.display = 'flex';

        var parts = [];
        if (pending > 0) parts.push(pending + ' pending');
        if (active > 0)  parts.push(active + ' active');
        if (complete > 0) parts.push(complete + ' done');
        if (failed > 0)  parts.push('<span class="dispatch-failed">' + failed + ' failed</span>');

        statusBarEl.innerHTML =
            '<span class="dispatch-icon">&#x2699;</span> Queue: ' + parts.join(', ');
    }

    // ---- Dispatch Button in Plan View ----

    function injectDispatchButton() {
        var toolbar = document.querySelector('.mission-toolbar');
        if (!toolbar || document.getElementById('btn-dispatch')) return;

        var btn = document.createElement('button');
        btn.id = 'btn-dispatch';
        btn.title = 'Queue current mission for dispatch';
        btn.textContent = '\u21E5 Dispatch';
        btn.addEventListener('click', onDispatchClick);
        toolbar.appendChild(btn);
    }

    function onDispatchClick() {
        if (typeof Mission === 'undefined') {
            meridian.log('Dispatch: Mission module not available', 'warn');
            return;
        }
        var items = Mission.getItems();
        if (!items || items.length === 0) {
            meridian.log('Dispatch: No waypoints to queue', 'warn');
            return;
        }

        var vehicleId = meridian.activeVehicleId;
        if (!vehicleId) {
            meridian.log('Dispatch: No active vehicle', 'warn');
            return;
        }

        var entry = {
            id: nextId++,
            vehicleId: vehicleId,
            mission: items.slice(),   // copy of current mission
            status: STATUS.PENDING,
            createdAt: Date.now(),
        };

        queue.push(entry);
        meridian.log('Dispatch: queued mission #' + entry.id + ' (' + items.length + ' waypoints) for vehicle ' + vehicleId, 'info');
        updateStatusBar();
    }

    // ---- Auto-dispatch Poller ----

    function startPoller() {
        if (pollTimer) clearInterval(pollTimer);
        pollTimer = setInterval(function () {
            var ids = Object.keys(meridian.vehicles);
            ids.forEach(function (id) {
                var v = meridian.vehicles[id];
                if (v) maybeTriggerDispatch(parseInt(id, 10), v);
            });
        }, 2000);
    }

    function maybeTriggerDispatch(sysid, v) {
        // Only auto-upload if vehicle is armed and connected
        if (!v || !v.connected) return;

        // Find the first pending entry for this vehicle
        var entry = queue.find(function (q) {
            return q.vehicleId === sysid && q.status === STATUS.PENDING;
        });
        if (!entry) return;

        // Check: no other active entry for this vehicle
        var alreadyActive = queue.some(function (q) {
            return q.vehicleId === sysid && q.status === STATUS.ACTIVE;
        });
        if (alreadyActive) return;

        // Vehicle must be armed and have GPS
        if (!v.armed || v.fixType < 3) return;

        // Mark active and upload
        entry.status = STATUS.ACTIVE;
        entry._uploadedAt = Date.now();
        updateStatusBar();

        meridian.log('Dispatch: uploading mission #' + entry.id + ' to vehicle ' + sysid, 'info');

        // If this is the active vehicle, upload directly
        if (sysid === meridian.activeVehicleId) {
            if (window.Mission) {
                Mission.setItems(entry.mission);
                Mission.upload();
            }
        } else {
            // For non-active vehicle we emit an event that connection pool handles
            meridian.events.emit('dispatch_upload', { entry: entry, sysid: sysid });
        }
    }

    function onMissionAckSuccess() {
        // Mark the oldest active entry as complete
        var active = queue.find(function (q) { return q.status === STATUS.ACTIVE; });
        if (active) {
            active.status = STATUS.COMPLETE;
            active._completedAt = Date.now();
            meridian.log('Dispatch: mission #' + active.id + ' upload confirmed', 'info');
            updateStatusBar();
        }
    }

    function onMissionAckFail(result) {
        var active = queue.find(function (q) { return q.status === STATUS.ACTIVE; });
        if (active) {
            active.status = STATUS.FAILED;
            meridian.log('Dispatch: mission #' + active.id + ' upload failed (error ' + result + ')', 'error');
            updateStatusBar();
        }
    }

    // ---- Public API ----

    function getQueue() { return queue.slice(); }

    function clearCompleted() {
        queue = queue.filter(function (q) {
            return q.status !== STATUS.COMPLETE && q.status !== STATUS.FAILED;
        });
        updateStatusBar();
    }

    function cancelEntry(id) {
        var entry = queue.find(function (q) { return q.id === id; });
        if (!entry || entry.status === STATUS.ACTIVE) return false;
        queue = queue.filter(function (q) { return q.id !== id; });
        updateStatusBar();
        return true;
    }

    return {
        init: init,
        getQueue: getQueue,
        clearCompleted: clearCompleted,
        cancelEntry: cancelEntry,
        STATUS: STATUS,
    };

})();
