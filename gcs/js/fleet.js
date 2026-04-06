/* ============================================================
   fleet.js — Fleet management (T3-7)
   Vehicle registry in IndexedDB 'meridian_fleet'.
   Fleet panel with vehicle list, detail card, and edit.
   Prompts to register or match when a new sysid connects.
   ============================================================ */

'use strict';

window.Fleet = (function () {

    const DB_NAME = 'meridian_fleet';
    const DB_VERSION = 1;
    const STORE = 'vehicles';

    let db = null;
    let panelEl = null;
    let panelVisible = false;

    // In-memory cache of all fleet records keyed by tailNumber
    let records = {};

    // ---- IndexedDB ----

    function openDB() {
        return new Promise(function (resolve, reject) {
            var req = indexedDB.open(DB_NAME, DB_VERSION);
            req.onupgradeneeded = function (e) {
                var d = e.target.result;
                if (!d.objectStoreNames.contains(STORE)) {
                    var store = d.createObjectStore(STORE, { keyPath: 'tailNumber' });
                    store.createIndex('sysid', 'sysid', { unique: false });
                    store.createIndex('name', 'name', { unique: false });
                }
            };
            req.onsuccess = function () { resolve(req.result); };
            req.onerror = function () { reject(req.error); };
        });
    }

    function loadAll() {
        return new Promise(function (resolve) {
            if (!db) { resolve({}); return; }
            var tx = db.transaction(STORE, 'readonly');
            var store = tx.objectStore(STORE);
            var req = store.getAll();
            req.onsuccess = function () {
                var all = {};
                (req.result || []).forEach(function (r) { all[r.tailNumber] = r; });
                resolve(all);
            };
            req.onerror = function () { resolve({}); };
        });
    }

    function saveRecord(rec) {
        return new Promise(function (resolve, reject) {
            if (!db) { reject(new Error('DB not open')); return; }
            var tx = db.transaction(STORE, 'readwrite');
            var req = tx.objectStore(STORE).put(rec);
            req.onsuccess = function () {
                records[rec.tailNumber] = rec;
                resolve(rec);
            };
            req.onerror = function () { reject(req.error); };
        });
    }

    function deleteRecord(tailNumber) {
        return new Promise(function (resolve) {
            if (!db) { resolve(); return; }
            var tx = db.transaction(STORE, 'readwrite');
            tx.objectStore(STORE).delete(tailNumber);
            tx.oncomplete = function () {
                delete records[tailNumber];
                resolve();
            };
        });
    }

    function findBySysid(sysid) {
        for (var key in records) {
            if (records[key].sysid === sysid) return records[key];
        }
        return null;
    }

    // ---- Init ----

    async function init() {
        try {
            db = await openDB();
            records = await loadAll();
        } catch (e) {
            console.warn('Fleet: IndexedDB unavailable', e);
        }

        injectToolbarButton();
        buildPanel();

        // When a new sysid appears, prompt to register or match
        meridian.events.on('heartbeat', function (v) {
            if (!v) return;
            var sysid = v.sysid;
            if (sysid == null) return;
            if (v._fleetChecked) return;
            v._fleetChecked = true;

            var existing = findBySysid(sysid);
            if (existing) {
                // Update last seen in multi-vehicle selector display
                updateSelectorLabel(sysid, existing.name || existing.tailNumber);
            } else {
                promptRegister(sysid);
            }
        });

        // Re-render panel body when vehicles switch
        meridian.events.on('vehicle_switch', function () {
            if (panelVisible) renderList();
        });
    }

    // ---- Toolbar Button ----

    function injectToolbarButton() {
        var right = document.querySelector('.toolbar-right');
        if (!right) return;
        if (document.getElementById('btn-fleet')) return;

        var btn = document.createElement('button');
        btn.type = 'button';
        btn.id = 'btn-fleet';
        btn.className = 'toolbar-settings';
        btn.title = 'Fleet registry';
        btn.setAttribute('aria-label', 'Open fleet panel');
        btn.innerHTML = '&#x2708;';  // airplane symbol
        btn.addEventListener('click', togglePanel);

        // Insert before theme toggle
        var themeToggle = document.getElementById('theme-toggle');
        if (themeToggle) {
            right.insertBefore(btn, themeToggle);
        } else {
            right.prepend(btn);
        }
    }

    // ---- Fleet Panel ----

    function buildPanel() {
        panelEl = document.createElement('div');
        panelEl.id = 'panel-fleet';
        panelEl.className = 'fleet-panel';
        panelEl.setAttribute('role', 'dialog');
        panelEl.setAttribute('aria-label', 'Fleet registry');
        panelEl.style.display = 'none';

        panelEl.innerHTML =
            '<div class="fleet-panel-header">' +
                '<span class="fleet-panel-title">Fleet Registry</span>' +
                '<button class="fleet-close" aria-label="Close fleet panel" id="fleet-close-btn">&times;</button>' +
            '</div>' +
            '<div class="fleet-panel-body" id="fleet-body"></div>';

        document.getElementById('app').appendChild(panelEl);

        document.getElementById('fleet-close-btn').addEventListener('click', togglePanel);
    }

    function togglePanel() {
        panelVisible = !panelVisible;
        panelEl.style.display = panelVisible ? 'flex' : 'none';
        var btn = document.getElementById('btn-fleet');
        if (btn) btn.classList.toggle('active', panelVisible);
        if (panelVisible) renderList();
    }

    function renderList() {
        var body = document.getElementById('fleet-body');
        if (!body) return;
        body.innerHTML = '';

        // Add vehicle button
        var addBtn = document.createElement('button');
        addBtn.type = 'button';
        addBtn.className = 'fleet-add-btn';
        addBtn.textContent = '+ Add Vehicle';
        addBtn.addEventListener('click', function () { showEditCard(null); });
        body.appendChild(addBtn);

        var keys = Object.keys(records);
        if (keys.length === 0) {
            var empty = document.createElement('div');
            empty.className = 'fleet-empty';
            empty.textContent = 'No vehicles registered. Connect a vehicle or click + Add Vehicle.';
            body.appendChild(empty);
            return;
        }

        // Sort by last flown (newest first)
        keys.sort(function (a, b) {
            return (records[b].lastFlown || 0) - (records[a].lastFlown || 0);
        });

        keys.forEach(function (tail) {
            body.appendChild(createVehicleRow(records[tail]));
        });
    }

    function createVehicleRow(rec) {
        var row = document.createElement('div');
        row.className = 'fleet-row';

        var lastFlownStr = rec.lastFlown
            ? new Date(rec.lastFlown).toLocaleDateString()
            : 'Never';

        var hoursStr = rec.totalHours != null
            ? rec.totalHours.toFixed(1) + 'h'
            : '0.0h';

        row.innerHTML =
            '<div class="fleet-row-main">' +
                '<div class="fleet-row-name">' + escHtml(rec.name || rec.tailNumber) + '</div>' +
                '<div class="fleet-row-tail">' + escHtml(rec.tailNumber) + '</div>' +
            '</div>' +
            '<div class="fleet-row-stats">' +
                '<span title="Total flights">' + (rec.totalFlights || 0) + ' flights</span>' +
                '<span title="Total hours">' + hoursStr + '</span>' +
                '<span title="Last flown">' + lastFlownStr + '</span>' +
            '</div>';

        row.addEventListener('click', function () { showDetailCard(rec); });
        return row;
    }

    // ---- Detail Card ----

    function showDetailCard(rec) {
        var body = document.getElementById('fleet-body');
        if (!body) return;
        body.innerHTML = '';

        var lastFlownStr = rec.lastFlown
            ? new Date(rec.lastFlown).toLocaleString()
            : 'Never';

        var card = document.createElement('div');
        card.className = 'fleet-detail-card';
        card.innerHTML =
            '<button class="fleet-back-btn" id="fleet-back">&#x2190; Back</button>' +
            '<div class="fleet-detail-name">' + escHtml(rec.name || rec.tailNumber) + '</div>' +
            '<div class="fleet-detail-row"><span>Tail Number</span><strong>' + escHtml(rec.tailNumber) + '</strong></div>' +
            '<div class="fleet-detail-row"><span>Registration</span><strong>' + escHtml(rec.registrationNumber || '--') + '</strong></div>' +
            '<div class="fleet-detail-row"><span>Frame Type</span><strong>' + escHtml(rec.frameType || '--') + '</strong></div>' +
            '<div class="fleet-detail-row"><span>SysID</span><strong>' + (rec.sysid != null ? rec.sysid : '--') + '</strong></div>' +
            '<div class="fleet-detail-row"><span>Total Flights</span><strong>' + (rec.totalFlights || 0) + '</strong></div>' +
            '<div class="fleet-detail-row"><span>Total Hours</span><strong>' + (rec.totalHours || 0).toFixed(1) + 'h</strong></div>' +
            '<div class="fleet-detail-row"><span>Last Flown</span><strong>' + lastFlownStr + '</strong></div>' +
            '<div class="fleet-detail-notes"><span>Notes</span><p>' + escHtml(rec.notes || '') + '</p></div>' +
            '<div class="fleet-detail-actions">' +
                '<button class="fleet-edit-btn" id="fleet-edit-btn">Edit</button>' +
                '<button class="fleet-del-btn" id="fleet-del-btn">Delete</button>' +
            '</div>';

        body.appendChild(card);

        document.getElementById('fleet-back').addEventListener('click', renderList);
        document.getElementById('fleet-edit-btn').addEventListener('click', function () { showEditCard(rec); });
        document.getElementById('fleet-del-btn').addEventListener('click', async function () {
            var ok = await Modal.confirm('Delete Vehicle',
                'Remove ' + (rec.name || rec.tailNumber) + ' from fleet registry?',
                'Delete', true);
            if (ok) {
                await deleteRecord(rec.tailNumber);
                meridian.log('Fleet: removed ' + rec.tailNumber, 'info');
                renderList();
            }
        });
    }

    // ---- Edit Card ----

    function showEditCard(existing) {
        var body = document.getElementById('fleet-body');
        if (!body) return;
        body.innerHTML = '';

        var rec = existing ? Object.assign({}, existing) : {
            tailNumber: '',
            name: '',
            sysid: null,
            frameType: '',
            registrationNumber: '',
            lastFlown: null,
            totalFlights: 0,
            totalHours: 0,
            notes: '',
        };

        var isNew = !existing;
        var title = isNew ? 'Add Vehicle' : 'Edit Vehicle';

        var form = document.createElement('div');
        form.className = 'fleet-edit-form';
        form.innerHTML =
            '<button class="fleet-back-btn" id="fleet-edit-back">&#x2190; Back</button>' +
            '<div class="fleet-edit-title">' + title + '</div>' +
            makeField('Tail Number *', 'fleet-f-tail', rec.tailNumber, 'text', isNew) +
            makeField('Name', 'fleet-f-name', rec.name, 'text', true) +
            makeField('Frame Type', 'fleet-f-frame', rec.frameType, 'text', true) +
            makeField('Registration No.', 'fleet-f-reg', rec.registrationNumber, 'text', true) +
            makeField('SysID', 'fleet-f-sysid', rec.sysid != null ? rec.sysid : '', 'number', true) +
            makeField('Total Flights', 'fleet-f-flights', rec.totalFlights || 0, 'number', true) +
            makeField('Total Hours', 'fleet-f-hours', rec.totalHours || 0, 'number', true) +
            '<div class="fleet-field">' +
                '<label for="fleet-f-notes">Notes</label>' +
                '<textarea id="fleet-f-notes" rows="3">' + escHtml(rec.notes || '') + '</textarea>' +
            '</div>' +
            '<div class="fleet-edit-actions">' +
                '<button class="fleet-save-btn" id="fleet-save-btn">Save</button>' +
                '<button class="fleet-cancel-btn" id="fleet-cancel-btn">Cancel</button>' +
            '</div>';

        body.appendChild(form);

        document.getElementById('fleet-edit-back').addEventListener('click', function () {
            existing ? showDetailCard(existing) : renderList();
        });
        document.getElementById('fleet-cancel-btn').addEventListener('click', function () {
            existing ? showDetailCard(existing) : renderList();
        });
        document.getElementById('fleet-save-btn').addEventListener('click', async function () {
            var tail = document.getElementById('fleet-f-tail').value.trim();
            if (!tail) { meridian.log('Fleet: tail number required', 'warn'); return; }

            if (isNew && records[tail]) {
                meridian.log('Fleet: tail number already exists', 'warn'); return;
            }

            var sysidVal = document.getElementById('fleet-f-sysid').value.trim();
            var updated = {
                tailNumber: tail,
                name: document.getElementById('fleet-f-name').value.trim(),
                sysid: sysidVal !== '' ? parseInt(sysidVal, 10) : null,
                frameType: document.getElementById('fleet-f-frame').value.trim(),
                registrationNumber: document.getElementById('fleet-f-reg').value.trim(),
                lastFlown: rec.lastFlown,
                totalFlights: parseInt(document.getElementById('fleet-f-flights').value, 10) || 0,
                totalHours: parseFloat(document.getElementById('fleet-f-hours').value) || 0,
                notes: document.getElementById('fleet-f-notes').value.trim(),
            };

            // If tail number changed during edit, remove old record
            if (!isNew && existing.tailNumber !== tail) {
                await deleteRecord(existing.tailNumber);
            }

            await saveRecord(updated);
            meridian.log('Fleet: saved ' + tail, 'info');
            showDetailCard(updated);
        });
    }

    function makeField(label, id, value, type, editable) {
        return '<div class="fleet-field">' +
            '<label for="' + id + '">' + label + '</label>' +
            '<input type="' + type + '" id="' + id + '" value="' + escHtml(String(value)) + '"' +
            (editable ? '' : ' readonly') + '>' +
            '</div>';
    }

    // ---- Register Prompt ----

    async function promptRegister(sysid) {
        // Debounce — only show once per sysid per page load
        if (!window._fleetPrompted) window._fleetPrompted = {};
        if (window._fleetPrompted[sysid]) return;
        window._fleetPrompted[sysid] = true;

        // Don't show during demo or onboarding tutorial
        if (meridian.demo) return;
        if (document.querySelector('.onboard-overlay')) return;

        var keys = Object.keys(records);
        if (keys.length > 0) {
            // Offer to match to existing or create new
            var choice = await Modal.prompt(
                'Register Vehicle',
                'Vehicle ' + sysid + ' connected. Enter tail number to link (leave blank to skip):',
                ''
            );
            if (choice && choice.trim()) {
                var tail = choice.trim();
                var existing = records[tail];
                if (existing) {
                    // Link sysid
                    existing.sysid = sysid;
                    await saveRecord(existing);
                    updateSelectorLabel(sysid, existing.name || existing.tailNumber);
                    meridian.log('Fleet: linked sysid ' + sysid + ' to ' + tail, 'info');
                } else {
                    // Create new with this tail
                    await saveRecord(makeNewRecord(sysid, tail));
                    meridian.log('Fleet: registered ' + tail + ' (sysid ' + sysid + ')', 'info');
                }
            }
        } else {
            // First vehicle — simpler prompt
            var tail2 = await Modal.prompt(
                'Register Vehicle',
                'New vehicle (sysid ' + sysid + ') connected.\nEnter a tail number to register (leave blank to skip):',
                'UAV-' + sysid
            );
            if (tail2 && tail2.trim()) {
                await saveRecord(makeNewRecord(sysid, tail2.trim()));
                meridian.log('Fleet: registered ' + tail2.trim(), 'info');
            }
        }
    }

    function makeNewRecord(sysid, tailNumber) {
        return {
            tailNumber: tailNumber,
            name: tailNumber,
            sysid: sysid,
            frameType: '',
            registrationNumber: '',
            lastFlown: null,
            totalFlights: 0,
            totalHours: 0,
            notes: '',
        };
    }

    // ---- Multi-vehicle selector integration ----

    function updateSelectorLabel(sysid, label) {
        // Update any rendered mv-item-label or mv-id for this sysid
        document.querySelectorAll('[data-sysid="' + sysid + '"] .mv-item-label').forEach(function (el) {
            el.textContent = label;
        });
        // Store on vehicle object for Dispatch / other modules
        var v = meridian.vehicles[sysid];
        if (v) v._fleetName = label;
    }

    // ---- Public: update flight stats after a flight ends ----

    async function recordFlight(sysid, durationHours) {
        var rec = findBySysid(sysid);
        if (!rec) return;
        rec.totalFlights = (rec.totalFlights || 0) + 1;
        rec.totalHours = (rec.totalHours || 0) + durationHours;
        rec.lastFlown = Date.now();
        await saveRecord(rec);
    }

    // ---- Helpers ----

    function escHtml(s) {
        var d = document.createElement('div');
        d.textContent = s;
        return d.innerHTML;
    }

    return {
        init: init,
        findBySysid: findBySysid,
        saveRecord: saveRecord,
        recordFlight: recordFlight,
        renderList: renderList,
        get records() { return records; },
    };

})();
