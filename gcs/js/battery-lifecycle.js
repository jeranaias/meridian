/* ============================================================
   battery-lifecycle.js — Per-battery cycle tracking (T3-2)
   IndexedDB store 'meridian_batteries'
   Records: { id, name, cellCount, capacity, cycles, totalMah,
              events: [{date, startVoltage, endVoltage, mah, duration}] }
   Logs one cycle entry per arm→disarm flight.
   UI: "Battery Log" tab injected into Logs panel.
   ============================================================ */

'use strict';

window.BatteryLifecycle = (function () {

    var DB_NAME    = 'meridian_batteries';
    var DB_VERSION = 1;
    var STORE_NAME = 'batteries';
    var _db = null;

    // Per-flight tracking state
    var _flightStart      = null;   // ms timestamp at arm
    var _flightStartVolt  = null;   // voltage at arm
    var _currentMah       = 0;
    var _currentVolt      = 0;
    var _currentPct       = -1;

    // ─── IndexedDB helpers ────────────────────────────────────

    function _openDb() {
        return new Promise(function (resolve, reject) {
            if (_db) { resolve(_db); return; }
            var req = indexedDB.open(DB_NAME, DB_VERSION);
            req.onupgradeneeded = function (e) {
                var db = e.target.result;
                if (!db.objectStoreNames.contains(STORE_NAME)) {
                    db.createObjectStore(STORE_NAME, { keyPath: 'id' });
                }
            };
            req.onsuccess = function (e) {
                _db = e.target.result;
                resolve(_db);
            };
            req.onerror = function () { reject(req.error); };
        });
    }

    function _tx(mode) {
        return _db.transaction([STORE_NAME], mode).objectStore(STORE_NAME);
    }

    function _getAll() {
        return _openDb().then(function (db) {
            return new Promise(function (resolve, reject) {
                var req = _tx('readonly').getAll();
                req.onsuccess = function () { resolve(req.result || []); };
                req.onerror   = function () { reject(req.error); };
            });
        });
    }

    function _get(id) {
        return _openDb().then(function () {
            return new Promise(function (resolve, reject) {
                var req = _tx('readonly').get(id);
                req.onsuccess = function () { resolve(req.result || null); };
                req.onerror   = function () { reject(req.error); };
            });
        });
    }

    function _put(record) {
        return _openDb().then(function () {
            return new Promise(function (resolve, reject) {
                var req = _tx('readwrite').put(record);
                req.onsuccess = function () { resolve(); };
                req.onerror   = function () { reject(req.error); };
            });
        });
    }

    function _delete(id) {
        return _openDb().then(function () {
            return new Promise(function (resolve, reject) {
                var req = _tx('readwrite').delete(id);
                req.onsuccess = function () { resolve(); };
                req.onerror   = function () { reject(req.error); };
            });
        });
    }

    // ─── Battery fingerprint ─────────────────────────────────

    function _fingerprintId(cellCount, capacity) {
        return 'bat_' + (cellCount || 0) + 'S_' + (capacity || 0);
    }

    function _getOrCreate(cellCount, capacity) {
        var id = _fingerprintId(cellCount, capacity);
        return _get(id).then(function (rec) {
            if (rec) return rec;
            var newRec = {
                id: id,
                name: cellCount + 'S ' + capacity + 'mAh',
                cellCount: cellCount || 0,
                capacity: capacity || 0,
                cycles: 0,
                totalMah: 0,
                events: [],
                createdAt: Date.now(),
            };
            return _put(newRec).then(function () { return newRec; });
        });
    }

    // ─── Flight event telemetry wiring ────────────────────────

    function _init() {
        meridian.events.on('heartbeat', function (v) {
            if (!v) return;

            // Arm event — start tracking
            if (v.armed && !_flightStart) {
                _flightStart     = Date.now();
                _flightStartVolt = _currentVolt;
                meridian.log('[Battery] Flight started — ' +
                    (_flightStartVolt > 0 ? _flightStartVolt.toFixed(2) + 'V' : 'voltage unknown'), 'info');
            }

            // Disarm event — log cycle
            if (!v.armed && _flightStart !== null) {
                var duration = Date.now() - _flightStart;
                _logCycle(duration);
                _flightStart = null;
                _flightStartVolt = null;
            }
        });

        meridian.events.on('battery', function (v) {
            if (!v) return;
            _currentVolt = v.voltage || 0;
            _currentPct  = v.batteryPct;
            _currentMah  = v.mah || 0;
        });

        // Reset on new connection
        meridian.events.on('connection_change', function (state) {
            if (state === 0) {
                _flightStart = null;
                _flightStartVolt = null;
            }
        });
    }

    function _logCycle(durationMs) {
        var v = meridian.v;
        var cellCount = v ? (v.cellVoltages ? v.cellVoltages.length : 0) : 0;
        var capacity  = v ? (v.params && v.params['BATT_CAPACITY'] ? v.params['BATT_CAPACITY'] : 0) : 0;

        _getOrCreate(cellCount, capacity).then(function (rec) {
            var entry = {
                date:         Date.now(),
                startVoltage: _flightStartVolt || 0,
                endVoltage:   _currentVolt,
                mah:          _currentMah,
                duration:     Math.round(durationMs / 1000),
                endPct:       _currentPct,
            };
            rec.events.push(entry);
            rec.cycles += 1;
            rec.totalMah += entry.mah;
            rec.lastUsed = entry.date;
            return _put(rec);
        }).then(function () {
            meridian.log('[Battery] Cycle logged — ' + Math.round(durationMs / 1000) + 's, ' +
                _currentMah.toFixed(0) + ' mAh', 'info');
        }).catch(function (err) {
            meridian.log('[Battery] Failed to log cycle: ' + err.message, 'warn');
        });
    }

    // ─── Health estimate ──────────────────────────────────────

    function _healthPercent(rec) {
        if (!rec || rec.cycles === 0 || rec.capacity === 0) return null;
        var avgMah = rec.totalMah / rec.cycles;
        return Math.min(100, Math.round((avgMah / rec.capacity) * 100));
    }

    // ─── UI: Battery Log tab ──────────────────────────────────

    function render(container) {
        container.innerHTML = '<div class="bat-log-loading">Loading battery records...</div>';

        _getAll().then(function (records) {
            container.innerHTML = '';

            // Toolbar
            var toolbar = document.createElement('div');
            toolbar.className = 'bat-log-toolbar';
            toolbar.innerHTML =
                '<span class="bat-log-count">' + records.length + ' pack' + (records.length !== 1 ? 's' : '') + ' tracked</span>';
            container.appendChild(toolbar);

            if (records.length === 0) {
                var empty = document.createElement('div');
                empty.className = 'bat-log-empty';
                empty.innerHTML =
                    '<div class="bat-log-empty-icon">&#x1F50B;</div>' +
                    '<div>No battery records yet.</div>' +
                    '<div class="bat-log-empty-hint">Records are created automatically on arm/disarm cycles.</div>';
                container.appendChild(empty);
                return;
            }

            // Sort by last used (most recent first)
            records.sort(function (a, b) { return (b.lastUsed || 0) - (a.lastUsed || 0); });

            var list = document.createElement('div');
            list.className = 'bat-log-list';

            records.forEach(function (rec) {
                list.appendChild(_renderBatteryCard(rec));
            });

            container.appendChild(list);
        }).catch(function () {
            container.innerHTML = '<div class="bat-log-error">Could not load battery records.</div>';
        });
    }

    function _renderBatteryCard(rec) {
        var health = _healthPercent(rec);
        var healthStr = health !== null ? health + '%' : 'N/A';
        var healthClass = health === null ? '' : health >= 80 ? 'good' : health >= 60 ? 'warn' : 'bad';

        var lastUsed = rec.lastUsed ? new Date(rec.lastUsed).toLocaleDateString() : 'Never';

        var card = document.createElement('div');
        card.className = 'bat-card';

        card.innerHTML =
            '<div class="bat-card-header">' +
                '<div class="bat-card-name" id="bat-name-' + rec.id + '">' +
                    '<span class="bat-name-text">' + _esc(rec.name) + '</span>' +
                    '<button class="bat-rename-btn" title="Rename" data-id="' + rec.id + '">&#x270E;</button>' +
                '</div>' +
                '<div class="bat-card-meta">' +
                    (rec.cellCount > 0 ? rec.cellCount + 'S' : '') +
                    (rec.capacity > 0 ? ' &middot; ' + rec.capacity + ' mAh' : '') +
                '</div>' +
            '</div>' +
            '<div class="bat-card-stats">' +
                '<div class="bat-stat">' +
                    '<div class="bat-stat-value">' + rec.cycles + '</div>' +
                    '<div class="bat-stat-label">Cycles</div>' +
                '</div>' +
                '<div class="bat-stat">' +
                    '<div class="bat-stat-value health ' + healthClass + '">' + healthStr + '</div>' +
                    '<div class="bat-stat-label">Health</div>' +
                '</div>' +
                '<div class="bat-stat">' +
                    '<div class="bat-stat-value">' + lastUsed + '</div>' +
                    '<div class="bat-stat-label">Last Used</div>' +
                '</div>' +
            '</div>' +
            '<div class="bat-events-toggle" data-id="' + rec.id + '">' +
                'Show ' + rec.events.length + ' flight record' + (rec.events.length !== 1 ? 's' : '') +
            '</div>' +
            '<div class="bat-events-list" id="bat-events-' + rec.id + '" style="display:none"></div>' +
            '<button class="bat-delete-btn" data-id="' + rec.id + '" title="Delete battery record">Delete Pack</button>';

        // Wire rename
        card.querySelector('.bat-rename-btn').addEventListener('click', function () {
            _renameInteractive(rec, card);
        });

        // Wire events toggle
        card.querySelector('.bat-events-toggle').addEventListener('click', function () {
            var evEl = document.getElementById('bat-events-' + rec.id);
            if (!evEl) return;
            if (evEl.style.display === 'none') {
                evEl.style.display = '';
                evEl.innerHTML = _renderEventRows(rec.events);
                this.textContent = 'Hide flight records';
            } else {
                evEl.style.display = 'none';
                this.textContent = 'Show ' + rec.events.length + ' flight record' + (rec.events.length !== 1 ? 's' : '');
            }
        });

        // Wire delete
        card.querySelector('.bat-delete-btn').addEventListener('click', function () {
            if (!confirm('Delete battery record "' + rec.name + '"?')) return;
            _delete(rec.id).then(function () {
                card.remove();
            });
        });

        return card;
    }

    function _renderEventRows(events) {
        if (!events || events.length === 0) return '<div class="bat-events-empty">No flight records.</div>';

        var rows = events.slice().reverse().map(function (e) {
            var d    = new Date(e.date);
            var dt   = d.toLocaleDateString() + ' ' + d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
            var dur  = _formatDuration(e.duration);
            var mah  = e.mah ? e.mah.toFixed(0) + ' mAh' : '--';
            var volt = e.endVoltage ? e.endVoltage.toFixed(2) + 'V' : '--';
            return '<div class="bat-event-row">' +
                '<span class="bat-ev-date">' + dt + '</span>' +
                '<span class="bat-ev-dur">' + dur + '</span>' +
                '<span class="bat-ev-mah">' + mah + '</span>' +
                '<span class="bat-ev-volt">' + volt + '</span>' +
                '</div>';
        }).join('');

        return '<div class="bat-events-header">' +
            '<span>Date</span><span>Duration</span><span>Consumed</span><span>End V</span>' +
            '</div>' + rows;
    }

    function _renameInteractive(rec, card) {
        var nameEl = card.querySelector('.bat-name-text');
        if (!nameEl) return;
        var input = document.createElement('input');
        input.type = 'text';
        input.className = 'bat-name-input';
        input.value = rec.name;
        nameEl.replaceWith(input);
        input.focus();
        input.select();

        function save() {
            var newName = input.value.trim() || rec.name;
            rec.name = newName;
            _put(rec);
            var span = document.createElement('span');
            span.className = 'bat-name-text';
            span.textContent = newName;
            input.replaceWith(span);
        }

        input.addEventListener('blur', save);
        input.addEventListener('keydown', function (e) {
            if (e.key === 'Enter') input.blur();
            if (e.key === 'Escape') { input.value = rec.name; input.blur(); }
        });
    }

    function _formatDuration(seconds) {
        if (!seconds) return '--';
        var m = Math.floor(seconds / 60);
        var s = seconds % 60;
        return m + ':' + (s < 10 ? '0' : '') + s;
    }

    function _esc(str) {
        return String(str)
            .replace(/&/g, '&amp;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;');
    }

    // ─── Public API ───────────────────────────────────────────

    function init() {
        _init();
    }

    return { init, render };

})();
