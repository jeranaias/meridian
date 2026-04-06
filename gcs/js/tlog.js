/* ============================================================
   tlog.js — Tlog recording to IndexedDB
   Oborne: stream every incoming byte, auto-start on connect,
   never accumulate in RAM (IndexedDB chunks instead).
   ============================================================ */

'use strict';

window.Tlog = (function () {

    let db = null;
    let recording = false;
    let sessionId = null;
    let chunkBuffer = [];
    let chunkSize = 0;
    const CHUNK_LIMIT = 64 * 1024; // flush every 64KB
    let byteCount = 0;
    let startTime = null;

    // --- IndexedDB Setup ---
    function openDB() {
        return new Promise((resolve, reject) => {
            const req = indexedDB.open('meridian_tlog', 1);
            req.onupgradeneeded = (e) => {
                const db = e.target.result;
                if (!db.objectStoreNames.contains('sessions')) {
                    db.createObjectStore('sessions', { keyPath: 'id' });
                }
                if (!db.objectStoreNames.contains('chunks')) {
                    const store = db.createObjectStore('chunks', { autoIncrement: true });
                    store.createIndex('session', 'sessionId', { unique: false });
                }
            };
            req.onsuccess = () => resolve(req.result);
            req.onerror = () => reject(req.error);
        });
    }

    async function init() {
        try {
            db = await openDB();
        } catch (e) {
            console.warn('Tlog: IndexedDB unavailable', e);
        }

        // Auto-start on connect
        meridian.events.on('heartbeat', () => {
            if (!recording && meridian.connectionState === 2 && meridian.settings.autoRecord) {
                startRecording();
            }
        });

        // Stop on disconnect
        Connection.onStateChange = function (state) {
            Toolbar.updateConnection(state);
            if (state === 0 && recording) {
                stopRecording();
            }
        };
    }

    function startRecording() {
        if (recording || !db) return;
        recording = true;
        byteCount = 0;
        startTime = Date.now();
        sessionId = 'tlog_' + new Date().toISOString().replace(/[:.]/g, '-');
        chunkBuffer = [];
        chunkSize = 0;

        // Create session record — include operator identity (T2-2)
        const s = meridian.settings || {};
        const tx = db.transaction('sessions', 'readwrite');
        tx.objectStore('sessions').put({
            id: sessionId,
            start: startTime,
            protocol: Connection.protocol,
            bytes: 0,
            operatorName: s.operatorName || '',
            operatorCert: s.operatorCert || '',
            operatorRegistration: s.operatorRegistration || '',
        });

        meridian.tlog.recording = true;
        meridian.tlog.startTime = startTime;
        meridian.tlog.filename = sessionId;
        meridian.tlog.byteCount = 0;

        Toolbar.startRecording();
        meridian.log('Recording started: ' + sessionId, 'info');
    }

    function stopRecording() {
        if (!recording) return;
        flushChunk(); // write remaining
        recording = false;

        // Update session record with final byte count (preserve operator fields)
        if (db) {
            const s = meridian.settings || {};
            const tx = db.transaction('sessions', 'readwrite');
            tx.objectStore('sessions').put({
                id: sessionId,
                start: startTime,
                end: Date.now(),
                protocol: Connection.protocol,
                bytes: byteCount,
                operatorName: s.operatorName || '',
                operatorCert: s.operatorCert || '',
                operatorRegistration: s.operatorRegistration || '',
            });
        }

        meridian.tlog.recording = false;
        Toolbar.stopRecording();
        meridian.log('Recording stopped: ' + byteCount + ' bytes', 'info');
    }

    // Called from connection.js on every incoming message
    function recordBytes(data) {
        if (!recording || !db) return;

        // Timestamp + raw bytes
        const timestamp = Date.now() - startTime;
        const entry = new Uint8Array(8 + data.length);
        const dv = new DataView(entry.buffer);
        dv.setFloat64(0, timestamp, true);
        entry.set(data, 8);

        chunkBuffer.push(entry);
        chunkSize += entry.length;
        byteCount += data.length;
        meridian.tlog.byteCount = byteCount;

        if (chunkSize >= CHUNK_LIMIT) {
            flushChunk();
        }
    }

    function flushChunk() {
        if (!db || chunkBuffer.length === 0) return;
        // Merge buffer entries into single blob
        const totalLen = chunkBuffer.reduce((s, e) => s + e.length, 0);
        const merged = new Uint8Array(totalLen);
        let offset = 0;
        for (const entry of chunkBuffer) {
            merged.set(entry, offset);
            offset += entry.length;
        }
        chunkBuffer = [];
        chunkSize = 0;

        const tx = db.transaction('chunks', 'readwrite');
        tx.objectStore('chunks').put({
            sessionId: sessionId,
            data: merged.buffer,
            timestamp: Date.now(),
        });
    }

    // Download session as .tlog file
    async function downloadSession(sid) {
        if (!db) return;
        sid = sid || sessionId;

        const tx = db.transaction('chunks', 'readonly');
        const store = tx.objectStore('chunks');
        const index = store.index('session');
        const req = index.getAll(sid);

        req.onsuccess = () => {
            const chunks = req.result;
            if (!chunks.length) {
                meridian.log('No data for session ' + sid, 'warn');
                return;
            }
            const totalLen = chunks.reduce((s, c) => s + c.data.byteLength, 0);
            const merged = new Uint8Array(totalLen);
            let off = 0;
            for (const chunk of chunks) {
                merged.set(new Uint8Array(chunk.data), off);
                off += chunk.data.byteLength;
            }
            const blob = new Blob([merged], { type: 'application/octet-stream' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = sid + '.tlog';
            a.click();
            URL.revokeObjectURL(url);
            meridian.log('Downloaded ' + sid + ' (' + totalLen + ' bytes)', 'info');
        };
    }

    return {
        init,
        startRecording,
        stopRecording,
        recordBytes,
        downloadSession,
        get recording() { return recording; },
    };

})();
