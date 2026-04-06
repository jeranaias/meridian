/* ============================================================
   mission.js — Mission list component
   Manages ordered list of waypoints with drag-to-reorder,
   dirty-state tracking, upload/download/clear.
   ============================================================ */

'use strict';

window.Mission = (function () {

    // --- Mission Data Model ---
    let items = [];       // Array of mission item objects
    let dirty = false;    // Modified since last upload?
    let selectedIdx = -1; // Currently selected index

    // Command type constants (MAVLink)
    var CMD = {
        NAV_WAYPOINT:         16,
        NAV_LOITER_UNLIM:     17,
        NAV_LOITER_TURNS:     18,
        NAV_LOITER_TIME:      19,
        NAV_RETURN_TO_LAUNCH: 20,
        NAV_LAND:             21,
        NAV_TAKEOFF:          22,
        DO_SET_ROI:           201,
    };

    // Reverse lookup: number -> short name
    var CMD_NAMES = {};
    CMD_NAMES[CMD.NAV_WAYPOINT]         = 'WAYPOINT';
    CMD_NAMES[CMD.NAV_LOITER_UNLIM]     = 'LOITER';
    CMD_NAMES[CMD.NAV_LOITER_TURNS]     = 'LOITER TURNS';
    CMD_NAMES[CMD.NAV_LOITER_TIME]      = 'LOITER TIME';
    CMD_NAMES[CMD.NAV_RETURN_TO_LAUNCH] = 'RTL';
    CMD_NAMES[CMD.NAV_LAND]             = 'LAND';
    CMD_NAMES[CMD.NAV_TAKEOFF]          = 'TAKEOFF';
    CMD_NAMES[CMD.DO_SET_ROI]           = 'SET ROI';

    // Friendly name for display
    function cmdName(cmd) {
        return CMD_NAMES[cmd] || ('CMD_' + cmd);
    }

    // CSS class for sequence badge color
    function cmdClass(cmd) {
        switch (cmd) {
            case CMD.NAV_TAKEOFF:          return 'takeoff';
            case CMD.NAV_LAND:             return 'land';
            case CMD.NAV_LOITER_UNLIM:
            case CMD.NAV_LOITER_TURNS:
            case CMD.NAV_LOITER_TIME:      return 'loiter';
            case CMD.NAV_RETURN_TO_LAUNCH: return 'rtl';
            case CMD.DO_SET_ROI:           return 'roi';
            default:                        return '';
        }
    }

    // --- Create a new mission item ---
    function createItem(lat, lon, alt, cmd) {
        return {
            seq: 0,                     // assigned on render
            command: cmd || CMD.NAV_WAYPOINT,
            lat: lat || 0,
            lon: lon || 0,
            alt: alt || 10,
            param1: 0,                  // hold time / turns / radius etc.
            param2: 0,
            param3: 0,
            param4: 0,                  // yaw
            frame: 3,                   // MAV_FRAME_GLOBAL_RELATIVE_ALT
        };
    }

    // --- Mutators ---
    function addWaypoint(lat, lon, alt, cmd) {
        var item = createItem(lat, lon, alt, cmd);
        items.push(item);
        markDirty();
        emitChange();
        return item;
    }

    function insertAt(index, lat, lon, alt, cmd) {
        var item = createItem(lat, lon, alt, cmd);
        items.splice(index, 0, item);
        markDirty();
        emitChange();
        return item;
    }

    function removeAt(index) {
        if (index < 0 || index >= items.length) return;
        items.splice(index, 1);
        if (selectedIdx >= items.length) selectedIdx = items.length - 1;
        if (selectedIdx < 0) selectedIdx = -1;
        markDirty();
        emitChange();
    }

    function moveItem(fromIdx, toIdx) {
        if (fromIdx === toIdx) return;
        if (fromIdx < 0 || fromIdx >= items.length) return;
        if (toIdx < 0 || toIdx >= items.length) return;
        var item = items.splice(fromIdx, 1)[0];
        items.splice(toIdx, 0, item);
        if (selectedIdx === fromIdx) selectedIdx = toIdx;
        markDirty();
        emitChange();
    }

    function updateItem(index, changes) {
        if (index < 0 || index >= items.length) return;
        Object.assign(items[index], changes);
        markDirty();
        emitChange();
    }

    function clearAll() {
        items = [];
        selectedIdx = -1;
        markDirty();
        emitChange();
    }

    function setItems(newItems) {
        items = newItems.map(function (src, i) {
            return {
                seq: i,
                command: src.command || CMD.NAV_WAYPOINT,
                lat: src.lat || src.x || 0,
                lon: src.lon || src.y || 0,
                alt: src.alt || src.z || 0,
                param1: src.param1 || 0,
                param2: src.param2 || 0,
                param3: src.param3 || 0,
                param4: src.param4 || 0,
                frame: src.frame || 3,
            };
        });
        dirty = false;
        selectedIdx = -1;
        emitChange();
    }

    function select(index) {
        selectedIdx = (index >= 0 && index < items.length) ? index : -1;
        meridian.events.emit('mission_select', selectedIdx);
    }

    function markDirty() {
        dirty = true;
        meridian.events.emit('mission_dirty', true);
    }

    function markClean() {
        dirty = false;
        meridian.events.emit('mission_dirty', false);
    }

    function emitChange() {
        // Reassign sequence numbers
        for (var i = 0; i < items.length; i++) {
            items[i].seq = i;
        }
        meridian.events.emit('mission_change', items);
    }

    // --- Upload / Download ---
    function upload() {
        if (items.length === 0) {
            meridian.log('No mission items to upload', 'warn');
            return;
        }
        var wps = items.map(function (it) {
            return {
                seq: it.seq,
                command: it.command,
                lat: it.lat,
                lon: it.lon,
                alt: it.alt,
                param1: it.param1,
                param2: it.param2,
                param3: it.param3,
                param4: it.param4,
                frame: it.frame,
            };
        });
        Connection.sendMissionUpload(wps);
        // T1-1: Listen for mission_ack to mark clean (state machine handles retries)
        var ackHandler = function (msg) {
            if (msg.result === 0) {
                markClean();
            }
            meridian.events.off('mission_ack', ackHandler);
        };
        meridian.events.on('mission_ack', ackHandler);
        meridian.log('Uploading mission (' + wps.length + ' items)...', 'info');
    }

    function download() {
        Connection.sendMissionRequestList();
        meridian.log('Requesting mission from vehicle...', 'info');
    }

    // --- Getters ---
    function getItems() { return items; }
    function getItem(i) { return items[i] || null; }
    function count() { return items.length; }
    function isDirty() { return dirty; }
    function getSelectedIdx() { return selectedIdx; }
    function getSelected() { return items[selectedIdx] || null; }

    // --- T1-8: .waypoints file import/export ---

    function exportWaypoints() {
        var lines = ['QGC WPL 110'];
        for (var i = 0; i < items.length; i++) {
            var it = items[i];
            var current = (i === 0) ? 1 : 0;
            var autocontinue = 1;
            // seq  current  frame  command  param1  param2  param3  param4  lat  lon  alt  autocontinue
            lines.push([
                i, current, it.frame || 3, it.command,
                it.param1, it.param2, it.param3, it.param4,
                it.lat.toFixed(8), it.lon.toFixed(8), it.alt.toFixed(6),
                autocontinue,
            ].join('\t'));
        }
        return lines.join('\n');
    }

    function importWaypoints(text) {
        var lines = text.trim().split(/\r?\n/);
        if (lines.length === 0) return false;

        // Validate header
        var header = lines[0].trim();
        if (header !== 'QGC WPL 110') {
            meridian.log('Invalid waypoints file — expected QGC WPL 110 header', 'error');
            return false;
        }

        var newItems = [];
        for (var i = 1; i < lines.length; i++) {
            var line = lines[i].trim();
            if (line === '' || line.startsWith('#')) continue;
            var parts = line.split(/\s+/);
            if (parts.length < 12) continue;

            newItems.push({
                seq:     parseInt(parts[0], 10),
                command: parseInt(parts[3], 10),
                param1:  parseFloat(parts[4]) || 0,
                param2:  parseFloat(parts[5]) || 0,
                param3:  parseFloat(parts[6]) || 0,
                param4:  parseFloat(parts[7]) || 0,
                lat:     parseFloat(parts[8]) || 0,
                lon:     parseFloat(parts[9]) || 0,
                alt:     parseFloat(parts[10]) || 0,
                frame:   parseInt(parts[2], 10) || 3,
            });
        }

        if (newItems.length === 0) {
            meridian.log('No valid waypoints found in file', 'warn');
            return false;
        }

        setItems(newItems);
        meridian.log('Imported ' + newItems.length + ' waypoints from file', 'info');
        return true;
    }

    function downloadFile(filename, content) {
        var blob = new Blob([content], { type: 'text/plain' });
        var a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = filename;
        a.click();
        URL.revokeObjectURL(a.href);
    }

    return {
        CMD: CMD,
        CMD_NAMES: CMD_NAMES,
        cmdName: cmdName,
        cmdClass: cmdClass,
        createItem: createItem,

        addWaypoint: addWaypoint,
        insertAt: insertAt,
        removeAt: removeAt,
        moveItem: moveItem,
        updateItem: updateItem,
        clearAll: clearAll,
        setItems: setItems,
        select: select,
        markDirty: markDirty,
        markClean: markClean,

        upload: upload,
        download: download,

        exportWaypoints: exportWaypoints,
        importWaypoints: importWaypoints,
        downloadFile: downloadFile,

        getItems: getItems,
        getItem: getItem,
        count: count,
        isDirty: isDirty,
        getSelectedIdx: getSelectedIdx,
        getSelected: getSelected,
    };

})();
