/* ============================================================
   auto-analysis.js — Post-flight auto-analysis (T3-10)
   Reads tlog binary from IndexedDB, scans for anomalies,
   presents pass/warn/fail report in a modal.
   ============================================================ */

'use strict';

window.AutoAnalysis = (function () {

    // ---- Thresholds ----
    var THRESH = {
        EKF_VARIANCE:    0.8,    // flag if any EKF variance field > this
        VOLT_DROP_1S:    0.5,    // V — battery drop in 1 second
        GPS_MIN_FIX:     3,      // minimum fix type during flight
        RSSI_MIN:        0.5,    // 0–1 normalised (50%)
        VIBE_MAX:        60,     // m/s² clip-point
    };

    // Severity levels
    var SEV = { PASS: 'pass', WARN: 'warn', FAIL: 'fail' };

    // ---- Public: attach Analyze button to log-list rows ----

    function injectAnalyzeButtons() {
        var listEl = document.querySelector('.log-list-body');
        if (!listEl) return;

        var rows = listEl.querySelectorAll('.log-session-row');
        rows.forEach(function (row) {
            if (row.querySelector('.analyze-btn')) return;

            var actions = row.querySelector('.log-session-actions');
            if (!actions) return;

            var btn = document.createElement('button');
            btn.className = 'log-icon-btn analyze-btn';
            btn.title = 'Analyze flight log';
            btn.textContent = '\u2315'; // search/analyze symbol
            btn.addEventListener('click', function (e) {
                e.stopPropagation();
                var idEl = row.querySelector('.log-session-id');
                var sessId = idEl ? idEl.textContent.trim() : null;
                if (sessId) analyzeSession(sessId);
            });

            // Insert before download button
            var dlBtn = actions.querySelector('.log-icon-btn:first-child');
            if (dlBtn) {
                actions.insertBefore(btn, dlBtn);
            } else {
                actions.appendChild(btn);
            }
        });
    }

    // ---- Entry Point ----

    function analyzeSession(sessionId) {
        showProgressModal(sessionId);

        loadSessionData(sessionId)
            .then(function (rawData) {
                var report = runAnalysis(rawData, sessionId);
                showReportModal(report, sessionId);
            })
            .catch(function (err) {
                showErrorModal(sessionId, err.message || String(err));
            });
    }

    // ---- IndexedDB Loader ----

    function loadSessionData(sessionId) {
        return new Promise(function (resolve, reject) {
            var req = indexedDB.open('meridian_tlog', 1);
            req.onerror = function () { reject(new Error('IndexedDB unavailable')); };
            req.onsuccess = function () {
                var db = req.result;
                var tx = db.transaction('chunks', 'readonly');
                var store = tx.objectStore('chunks');
                var index = store.index('session');
                var range = IDBKeyRange.only(sessionId);
                var chunks = [];
                var cursor = index.openCursor(range);
                cursor.onsuccess = function () {
                    var c = cursor.result;
                    if (c) {
                        chunks.push(c.value.data);
                        c.continue();
                    } else {
                        // Merge all chunks
                        var totalLen = chunks.reduce(function (s, b) { return s + b.byteLength; }, 0);
                        var merged = new Uint8Array(totalLen);
                        var offset = 0;
                        chunks.forEach(function (b) {
                            merged.set(new Uint8Array(b), offset);
                            offset += b.byteLength;
                        });
                        resolve(merged);
                    }
                };
                cursor.onerror = function () { reject(new Error('Failed to read chunks')); };
            };
        });
    }

    // ---- Analysis Engine ----

    function runAnalysis(raw, sessionId) {
        // Parse tlog entries: each entry is [8-byte float64 timestamp | N bytes mavlink frame]
        var frames = parseTlogFrames(raw);

        var checks = {
            ekf:         { sev: SEV.PASS, label: 'EKF Variance', findings: [] },
            battery:     { sev: SEV.PASS, label: 'Battery Voltage', findings: [] },
            gps:         { sev: SEV.PASS, label: 'GPS Fix', findings: [] },
            rssi:        { sev: SEV.PASS, label: 'RC Link Quality', findings: [] },
            modeChange:  { sev: SEV.PASS, label: 'Flight Mode Changes', findings: [] },
            vibration:   { sev: SEV.PASS, label: 'Vibration', findings: [] },
        };

        var stats = {
            totalFrames: frames.length,
            duration: 0,
            flightStart: null,
            flightEnd: null,
            armed: false,
        };

        if (frames.length === 0) {
            return { sessionId: sessionId, stats: stats, checks: checks, empty: true };
        }

        stats.duration = frames[frames.length - 1].ts - frames[0].ts;

        // State tracking
        var lastVoltage = null;
        var lastVoltageTs = null;
        var lastMode = null;
        var inFlight = false;
        var prevEkfTs = null;

        // Parse MAVLink frames
        var parser = new MAVLink.FrameParser();

        frames.forEach(function (frame) {
            var ts = frame.ts;

            parser.push(frame.data);
            var msgs = parser.extract();

            msgs.forEach(function (msg) {

                switch (msg.type) {

                    // ---- ARM/DISARM tracking via HEARTBEAT ----
                    case 'heartbeat': {
                        // base_mode bit 7 = armed
                        var armed = !!(msg.base_mode & 0x80);
                        if (armed && !inFlight) {
                            inFlight = true;
                            stats.flightStart = ts;
                        } else if (!armed && inFlight) {
                            inFlight = false;
                            stats.flightEnd = ts;
                        }
                        stats.armed = armed;

                        // Mode change detection
                        var modeNum = msg.custom_mode;
                        if (lastMode !== null && modeNum !== lastMode) {
                            var note = 'Mode changed from ' + lastMode + ' to ' + modeNum +
                                       ' at ' + fmtTs(ts);
                            checks.modeChange.findings.push({ ts: ts, sev: SEV.WARN, text: note });
                            upgradeSev(checks.modeChange, SEV.WARN);
                        }
                        lastMode = modeNum;
                        break;
                    }

                    // ---- EKF variance ----
                    case 'ekf_status_report': {
                        var fields = [
                            { name: 'velocity_variance',  val: msg.velocity_variance },
                            { name: 'pos_horiz_variance',  val: msg.pos_horiz_variance },
                            { name: 'pos_vert_variance',   val: msg.pos_vert_variance },
                            { name: 'compass_variance',    val: msg.compass_variance },
                            { name: 'terrain_alt_variance',val: msg.terrain_alt_variance },
                        ];
                        fields.forEach(function (f) {
                            if (f.val != null && f.val > THRESH.EKF_VARIANCE) {
                                // Debounce — 2s gap
                                if (!prevEkfTs || (ts - prevEkfTs) > 2000) {
                                    prevEkfTs = ts;
                                    var sev = f.val > THRESH.EKF_VARIANCE * 2 ? SEV.FAIL : SEV.WARN;
                                    var note = f.name + ' = ' + f.val.toFixed(3) +
                                               ' (threshold ' + THRESH.EKF_VARIANCE + ') at ' + fmtTs(ts);
                                    checks.ekf.findings.push({ ts: ts, sev: sev, text: note });
                                    upgradeSev(checks.ekf, sev);
                                }
                            }
                        });
                        break;
                    }

                    // ---- Battery voltage drop ----
                    case 'sys_status': {
                        var volt = msg.voltage_battery != null ? msg.voltage_battery / 1000 : null;
                        if (volt !== null && volt > 0) {
                            if (lastVoltage !== null && lastVoltageTs !== null) {
                                var dtSec = (ts - lastVoltageTs) / 1000;
                                if (dtSec > 0 && dtSec <= 2) {
                                    var drop = lastVoltage - volt;
                                    if (drop > THRESH.VOLT_DROP_1S) {
                                        var notev = 'Voltage drop ' + drop.toFixed(2) + 'V in ' +
                                                    dtSec.toFixed(1) + 's at ' + fmtTs(ts) +
                                                    ' (' + lastVoltage.toFixed(2) + 'V → ' + volt.toFixed(2) + 'V)';
                                        checks.battery.findings.push({ ts: ts, sev: SEV.WARN, text: notev });
                                        upgradeSev(checks.battery, SEV.WARN);
                                    }
                                }
                            }
                            lastVoltage = volt;
                            lastVoltageTs = ts;
                        }
                        break;
                    }

                    // ---- GPS fix drop ----
                    case 'gps_raw_int': {
                        var fix = msg.fix_type;
                        if (fix != null && fix < THRESH.GPS_MIN_FIX && inFlight) {
                            var noteg = 'GPS fix dropped to type ' + fix +
                                        ' (below ' + THRESH.GPS_MIN_FIX + ') at ' + fmtTs(ts);
                            // Debounce
                            var last = checks.gps.findings[checks.gps.findings.length - 1];
                            if (!last || (ts - last.ts) > 3000) {
                                checks.gps.findings.push({ ts: ts, sev: SEV.FAIL, text: noteg });
                                upgradeSev(checks.gps, SEV.FAIL);
                            }
                        }
                        break;
                    }

                    // ---- RC RSSI ----
                    case 'rc_channels': {
                        var rssi = msg.rssi;   // 0–255 in MAVLink
                        if (rssi != null) {
                            var pct = rssi / 255;
                            if (pct < THRESH.RSSI_MIN && inFlight) {
                                var noter = 'RC RSSI dropped to ' + Math.round(pct * 100) + '% at ' + fmtTs(ts);
                                var lastr = checks.rssi.findings[checks.rssi.findings.length - 1];
                                if (!lastr || (ts - lastr.ts) > 3000) {
                                    checks.rssi.findings.push({ ts: ts, sev: SEV.WARN, text: noter });
                                    upgradeSev(checks.rssi, SEV.WARN);
                                }
                            }
                        }
                        break;
                    }

                    // ---- Vibration ----
                    case 'vibration': {
                        var vx = msg.vibration_x;
                        var vy = msg.vibration_y;
                        var vz = msg.vibration_z;
                        [['X', vx], ['Y', vy], ['Z', vz]].forEach(function (pair) {
                            if (pair[1] != null && pair[1] > THRESH.VIBE_MAX) {
                                var notevib = 'Vibration ' + pair[0] + ' = ' + pair[1].toFixed(1) +
                                              ' m/s² (threshold ' + THRESH.VIBE_MAX + ') at ' + fmtTs(ts);
                                var lastv = checks.vibration.findings[checks.vibration.findings.length - 1];
                                if (!lastv || (ts - lastv.ts) > 3000) {
                                    var sevv = pair[1] > THRESH.VIBE_MAX * 1.5 ? SEV.FAIL : SEV.WARN;
                                    checks.vibration.findings.push({ ts: ts, sev: sevv, text: notevib });
                                    upgradeSev(checks.vibration, sevv);
                                }
                            }
                        });
                        break;
                    }
                }
            });
        });

        return { sessionId: sessionId, stats: stats, checks: checks, empty: false };
    }

    // ---- Tlog Frame Parser ----
    // Tlog format: each entry is 8-byte little-endian float64 timestamp + raw MAVLink bytes

    function parseTlogFrames(raw) {
        var frames = [];
        var offset = 0;
        var dv = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);

        while (offset + 9 <= raw.length) {
            var ts = dv.getFloat64(offset, true);
            offset += 8;

            // Find end of this MAVLink frame by checking for next timestamp marker
            // Heuristic: look for MAVLink start byte (0xFE or 0xFD)
            var frameStart = offset;
            var frameLen = 0;

            var startByte = raw[offset];
            if (startByte === 0xFE) {
                // MAVLink v1: [start(1) + len(1) + seq(1) + sys(1) + comp(1) + msgid(1) + payload(len) + crc(2)]
                if (offset + 2 <= raw.length) {
                    var payloadLen = raw[offset + 1];
                    frameLen = 6 + payloadLen + 2;
                }
            } else if (startByte === 0xFD) {
                // MAVLink v2: [start(1) + len(1) + incomp(1) + comp_flags(1) + seq(1) + sys(1) + comp(1) + msgid(3) + payload(len) + crc(2)]
                if (offset + 2 <= raw.length) {
                    var payloadLen2 = raw[offset + 1];
                    frameLen = 10 + payloadLen2 + 2;
                }
            }

            if (frameLen > 0 && offset + frameLen <= raw.length) {
                frames.push({ ts: ts, data: raw.slice(offset, offset + frameLen) });
                offset += frameLen;
            } else {
                // Skip one byte to re-sync
                offset++;
            }
        }

        return frames;
    }

    // ---- Helpers ----

    function upgradeSev(check, newSev) {
        var order = [SEV.PASS, SEV.WARN, SEV.FAIL];
        if (order.indexOf(newSev) > order.indexOf(check.sev)) {
            check.sev = newSev;
        }
    }

    function fmtTs(ms) {
        var s = Math.round(ms / 1000);
        var m = Math.floor(s / 60);
        var sec = s % 60;
        return m + ':' + String(sec).padStart(2, '0');
    }

    function fmtDuration(ms) {
        if (ms <= 0 || ms == null) return '--';
        var s = Math.round(ms / 1000);
        var m = Math.floor(s / 60);
        var sec = s % 60;
        return m + 'm ' + sec + 's';
    }

    function sevIcon(sev) {
        if (sev === SEV.PASS) return '<span class="aa-sev-pass">&#x2714; PASS</span>';
        if (sev === SEV.WARN) return '<span class="aa-sev-warn">&#x26A0; WARN</span>';
        return '<span class="aa-sev-fail">&#x2716; FAIL</span>';
    }

    function sevClass(sev) {
        return 'aa-check-' + sev;
    }

    // ---- Progress Modal ----

    function showProgressModal(sessionId) {
        var body = document.createElement('div');
        body.className = 'aa-progress';
        body.innerHTML = '<div class="aa-spinner"></div><div>Analyzing ' + escHtml(sessionId) + '...</div>';
        showModal('Post-Flight Analysis', body, null);
    }

    // ---- Error Modal ----

    function showErrorModal(sessionId, errMsg) {
        var body = document.createElement('div');
        body.className = 'aa-error';
        body.innerHTML = '<div class="aa-sev-fail">Analysis failed</div><div>' + escHtml(errMsg) + '</div>';
        showModal('Analysis Error', body, null);
    }

    // ---- Report Modal ----

    function showReportModal(report, sessionId) {
        var body = document.createElement('div');
        body.className = 'aa-report';

        // Header summary
        var overallSev = SEV.PASS;
        Object.values(report.checks).forEach(function (c) {
            if (c.sev === SEV.FAIL) overallSev = SEV.FAIL;
            else if (c.sev === SEV.WARN && overallSev !== SEV.FAIL) overallSev = SEV.WARN;
        });

        var duration = report.stats.duration > 0
            ? fmtDuration(report.stats.duration)
            : '--';

        body.innerHTML =
            '<div class="aa-header ' + sevClass(overallSev) + '">' +
                '<div class="aa-overall-sev">' + sevIcon(overallSev) + '</div>' +
                '<div class="aa-session-id">' + escHtml(sessionId) + '</div>' +
                '<div class="aa-stats">' +
                    '<span>Frames: ' + report.stats.totalFrames + '</span>' +
                    '<span>Duration: ' + duration + '</span>' +
                '</div>' +
            '</div>';

        if (report.empty) {
            var emptyDiv = document.createElement('div');
            emptyDiv.className = 'aa-empty';
            emptyDiv.textContent = 'No telemetry data found in this session.';
            body.appendChild(emptyDiv);
        } else {
            var checksEl = document.createElement('div');
            checksEl.className = 'aa-checks';

            Object.values(report.checks).forEach(function (check) {
                var section = document.createElement('div');
                section.className = 'aa-check-section ' + sevClass(check.sev);

                var header = document.createElement('div');
                header.className = 'aa-check-header';
                header.innerHTML =
                    '<span class="aa-check-label">' + escHtml(check.label) + '</span>' +
                    sevIcon(check.sev);
                section.appendChild(header);

                if (check.findings.length > 0) {
                    var list = document.createElement('ul');
                    list.className = 'aa-findings';
                    check.findings.forEach(function (f) {
                        var li = document.createElement('li');
                        li.className = 'aa-finding-item aa-finding-' + f.sev;
                        li.textContent = f.text;
                        list.appendChild(li);
                    });
                    section.appendChild(list);
                }

                checksEl.appendChild(section);
            });

            body.appendChild(checksEl);
        }

        // Export button
        var exportBtn = document.createElement('button');
        exportBtn.className = 'aa-export-btn';
        exportBtn.textContent = 'Export Report (.txt)';
        exportBtn.addEventListener('click', function () {
            exportReport(report, sessionId);
        });
        body.appendChild(exportBtn);

        showModal('Post-Flight Analysis', body, null);
    }

    // ---- Report Export ----

    function exportReport(report, sessionId) {
        var lines = [];
        lines.push('MERIDIAN GCS — POST-FLIGHT ANALYSIS REPORT');
        lines.push('=========================================');
        lines.push('Session: ' + sessionId);
        lines.push('Generated: ' + new Date().toISOString());
        lines.push('Frames analyzed: ' + report.stats.totalFrames);
        if (report.stats.duration > 0) {
            lines.push('Log duration: ' + fmtDuration(report.stats.duration));
        }
        lines.push('');

        // Overall
        var overallSev = SEV.PASS;
        Object.values(report.checks).forEach(function (c) {
            if (c.sev === SEV.FAIL) overallSev = SEV.FAIL;
            else if (c.sev === SEV.WARN && overallSev !== SEV.FAIL) overallSev = SEV.WARN;
        });
        lines.push('OVERALL: ' + overallSev.toUpperCase());
        lines.push('');

        Object.values(report.checks).forEach(function (check) {
            lines.push('--- ' + check.label + ' [' + check.sev.toUpperCase() + '] ---');
            if (check.findings.length === 0) {
                lines.push('  No anomalies detected.');
            } else {
                check.findings.forEach(function (f) {
                    lines.push('  [' + f.sev.toUpperCase() + '] ' + f.text);
                });
            }
            lines.push('');
        });

        var text = lines.join('\n');
        var blob = new Blob([text], { type: 'text/plain' });
        var url = URL.createObjectURL(blob);
        var a = document.createElement('a');
        a.href = url;
        a.download = 'analysis_' + sessionId + '.txt';
        a.click();
        URL.revokeObjectURL(url);
    }

    // ---- Modal Helper ----

    function showModal(title, bodyEl, onClose) {
        // Re-use or replace existing analysis modal
        var existing = document.getElementById('aa-modal-overlay');
        if (existing) existing.remove();

        var overlay = document.createElement('div');
        overlay.id = 'aa-modal-overlay';
        overlay.className = 'aa-modal-overlay';

        var modal = document.createElement('div');
        modal.className = 'aa-modal';
        modal.setAttribute('role', 'dialog');
        modal.setAttribute('aria-label', title);

        var header = document.createElement('div');
        header.className = 'aa-modal-header';
        header.innerHTML = '<span class="aa-modal-title">' + escHtml(title) + '</span>';

        var closeBtn = document.createElement('button');
        closeBtn.className = 'aa-modal-close';
        closeBtn.innerHTML = '&times;';
        closeBtn.setAttribute('aria-label', 'Close');
        closeBtn.addEventListener('click', function () {
            overlay.remove();
            if (onClose) onClose();
        });
        header.appendChild(closeBtn);

        var content = document.createElement('div');
        content.className = 'aa-modal-body';
        content.appendChild(bodyEl);

        modal.appendChild(header);
        modal.appendChild(content);
        overlay.appendChild(modal);
        document.body.appendChild(overlay);

        // Close on overlay click outside modal
        overlay.addEventListener('click', function (e) {
            if (e.target === overlay) { overlay.remove(); if (onClose) onClose(); }
        });
    }

    function escHtml(s) {
        var d = document.createElement('div');
        d.textContent = s;
        return d.innerHTML;
    }

    return {
        analyzeSession: analyzeSession,
        injectAnalyzeButtons: injectAnalyzeButtons,
    };

})();
