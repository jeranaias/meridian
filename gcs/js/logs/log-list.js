/* ============================================================
   log-list.js — Lists recorded tlog sessions from IndexedDB
   Shows session ID, date, duration, byte count.
   Download + delete per session.
   ============================================================ */

'use strict';

window.LogList = (function () {

    function render(container) {
        container.innerHTML = '';

        const wrapper = document.createElement('div');
        wrapper.className = 'log-list';

        const header = document.createElement('div');
        header.className = 'log-list-header';
        header.innerHTML = '<span class="log-list-title">Recorded Sessions</span>';

        // Manual record toggle
        const recBtn = document.createElement('button');
        recBtn.className = 'log-action-btn';
        recBtn.textContent = Tlog.recording ? 'Stop Recording' : 'Start Recording';
        if (Tlog.recording) recBtn.classList.add('recording');
        recBtn.addEventListener('click', function () {
            if (Tlog.recording) {
                Tlog.stopRecording();
            } else {
                Tlog.startRecording();
            }
            recBtn.textContent = Tlog.recording ? 'Stop Recording' : 'Start Recording';
            recBtn.classList.toggle('recording', Tlog.recording);
        });
        header.appendChild(recBtn);
        wrapper.appendChild(header);

        // Load sessions from IndexedDB
        const listEl = document.createElement('div');
        listEl.className = 'log-list-body';
        listEl.innerHTML = '<div class="log-list-loading">Loading sessions...</div>';
        wrapper.appendChild(listEl);
        container.appendChild(wrapper);

        loadSessions(listEl);
    }

    function loadSessions(listEl) {
        let db;
        const req = indexedDB.open('meridian_tlog', 1);
        req.onerror = function () {
            listEl.innerHTML = '<div class="log-list-empty">IndexedDB unavailable</div>';
        };
        req.onsuccess = function () {
            db = req.result;
            const tx = db.transaction('sessions', 'readonly');
            const store = tx.objectStore('sessions');
            const getAll = store.getAll();

            getAll.onsuccess = function () {
                const sessions = getAll.result;
                if (!sessions || sessions.length === 0) {
                    listEl.innerHTML = '<div class="log-list-empty">No recorded sessions yet.<br>Connect to a vehicle to start recording.</div>';
                    return;
                }

                // Sort newest first
                sessions.sort(function (a, b) { return (b.start || 0) - (a.start || 0); });

                listEl.innerHTML = '';
                sessions.forEach(function (sess) {
                    listEl.appendChild(createSessionRow(sess, db, listEl));
                });
            };

            getAll.onerror = function () {
                listEl.innerHTML = '<div class="log-list-empty">Error reading sessions</div>';
            };
        };
    }

    function createSessionRow(sess, db, listEl) {
        const row = document.createElement('div');
        row.className = 'log-session-row';

        const startDate = sess.start ? new Date(sess.start) : null;
        const dateStr = startDate ? startDate.toLocaleDateString() + ' ' + startDate.toLocaleTimeString() : 'Unknown';
        let durationStr = '--';
        if (sess.start && sess.end) {
            const dur = Math.floor((sess.end - sess.start) / 1000);
            const m = Math.floor(dur / 60);
            const s = dur % 60;
            durationStr = m + ':' + String(s).padStart(2, '0');
        }
        const bytesStr = formatBytes(sess.bytes || 0);

        const info = document.createElement('div');
        info.className = 'log-session-info';
        info.innerHTML =
            '<div class="log-session-id">' + escHtml(sess.id) + '</div>' +
            '<div class="log-session-meta">' +
                '<span>' + dateStr + '</span>' +
                '<span>' + durationStr + '</span>' +
                '<span>' + bytesStr + '</span>' +
            '</div>';

        const actions = document.createElement('div');
        actions.className = 'log-session-actions';

        const dlBtn = document.createElement('button');
        dlBtn.className = 'log-icon-btn';
        dlBtn.title = 'Download';
        dlBtn.innerHTML = '&#x2B73;';
        dlBtn.addEventListener('click', function () {
            Tlog.downloadSession(sess.id);
        });

        const delBtn = document.createElement('button');
        delBtn.className = 'log-icon-btn danger';
        delBtn.title = 'Delete';
        delBtn.innerHTML = '&#x2715;';
        delBtn.addEventListener('click', function () {
            deleteSession(sess.id, db, row, listEl);
        });

        actions.appendChild(dlBtn);
        actions.appendChild(delBtn);

        row.appendChild(info);
        row.appendChild(actions);
        return row;
    }

    function deleteSession(sid, db, rowEl, listEl) {
        // Delete session record
        const tx1 = db.transaction('sessions', 'readwrite');
        tx1.objectStore('sessions').delete(sid);

        // Delete associated chunks
        const tx2 = db.transaction('chunks', 'readwrite');
        const store = tx2.objectStore('chunks');
        const index = store.index('session');
        const range = IDBKeyRange.only(sid);
        const cursor = index.openCursor(range);
        cursor.onsuccess = function () {
            const c = cursor.result;
            if (c) {
                c.delete();
                c.continue();
            }
        };

        rowEl.style.opacity = '0';
        rowEl.style.height = rowEl.offsetHeight + 'px';
        setTimeout(function () {
            rowEl.style.height = '0';
            rowEl.style.padding = '0';
            rowEl.style.margin = '0';
            setTimeout(function () {
                rowEl.remove();
                if (listEl.children.length === 0) {
                    listEl.innerHTML = '<div class="log-list-empty">No recorded sessions.</div>';
                }
            }, 200);
        }, 100);

        meridian.log('Deleted session: ' + sid, 'info');
    }

    function formatBytes(b) {
        if (b < 1024) return b + ' B';
        if (b < 1024 * 1024) return (b / 1024).toFixed(1) + ' KB';
        return (b / (1024 * 1024)).toFixed(1) + ' MB';
    }

    function escHtml(s) {
        const d = document.createElement('div');
        d.textContent = s;
        return d.innerHTML;
    }

    return { render };

})();
