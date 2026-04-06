/* ============================================================
   playback.js — Tlog playback UI
   Load .tlog file, play back with speed controls and scrub bar.
   Feeds parsed frames through meridian.handleMessage.
   ============================================================ */

'use strict';

window.Playback = (function () {

    let container = null;
    let frames = [];
    let playing = false;
    let playTimer = null;
    let currentIndex = 0;
    let speed = 1;
    let scrubEl = null;
    let statusEl = null;
    let playBtn = null;

    const SPEEDS = [0.5, 1, 2, 5, 10];

    function render(cont) {
        container = cont;
        container.innerHTML = '';
        stop();

        const wrapper = document.createElement('div');
        wrapper.className = 'playback-panel';

        // File input
        const fileGroup = document.createElement('div');
        fileGroup.className = 'playback-file-group';

        const fileLabel = document.createElement('label');
        fileLabel.className = 'playback-file-label';
        fileLabel.textContent = 'Load .tlog file:';

        const fileInput = document.createElement('input');
        fileInput.type = 'file';
        fileInput.accept = '.tlog,.mnplog';
        fileInput.className = 'playback-file-input';
        fileInput.addEventListener('change', function (e) {
            const file = e.target.files[0];
            if (file) loadFile(file);
        });

        fileGroup.appendChild(fileLabel);
        fileGroup.appendChild(fileInput);
        wrapper.appendChild(fileGroup);

        // Transport controls
        const transport = document.createElement('div');
        transport.className = 'playback-transport';

        playBtn = document.createElement('button');
        playBtn.className = 'playback-btn play';
        playBtn.textContent = '\u25B6';
        playBtn.title = 'Play';
        playBtn.disabled = true;
        playBtn.addEventListener('click', togglePlay);
        transport.appendChild(playBtn);

        const stopBtn = document.createElement('button');
        stopBtn.className = 'playback-btn';
        stopBtn.textContent = '\u25A0';
        stopBtn.title = 'Stop';
        stopBtn.addEventListener('click', function () {
            stop();
            currentIndex = 0;
            updateScrub();
            updateStatus();
        });
        transport.appendChild(stopBtn);

        // Speed selector
        const speedGroup = document.createElement('div');
        speedGroup.className = 'playback-speed-group';
        speedGroup.innerHTML = '<span class="playback-label">Speed:</span>';

        SPEEDS.forEach(function (s) {
            const btn = document.createElement('button');
            btn.className = 'playback-speed-btn' + (s === speed ? ' active' : '');
            btn.textContent = s + 'x';
            btn.addEventListener('click', function () {
                speed = s;
                container.querySelectorAll('.playback-speed-btn').forEach(function (b) {
                    b.classList.toggle('active', parseFloat(b.textContent) === s);
                });
                if (playing) { stop(); play(); }
            });
            speedGroup.appendChild(btn);
        });
        transport.appendChild(speedGroup);

        wrapper.appendChild(transport);

        // Scrub bar
        scrubEl = document.createElement('input');
        scrubEl.type = 'range';
        scrubEl.className = 'playback-scrub';
        scrubEl.min = 0;
        scrubEl.max = 0;
        scrubEl.value = 0;
        scrubEl.disabled = true;
        scrubEl.addEventListener('input', function () {
            const wasPlaying = playing;
            if (playing) stop();
            currentIndex = parseInt(scrubEl.value);
            updateStatus();
            // Replay current frame
            if (frames[currentIndex]) {
                meridian.handleMessage(frames[currentIndex].msg);
            }
            if (wasPlaying) play();
        });
        wrapper.appendChild(scrubEl);

        // Status
        statusEl = document.createElement('div');
        statusEl.className = 'playback-status';
        statusEl.textContent = 'No file loaded';
        wrapper.appendChild(statusEl);

        container.appendChild(wrapper);
    }

    function loadFile(file) {
        const reader = new FileReader();
        reader.onload = function () {
            const data = new Uint8Array(reader.result);
            frames = parseFrames(data);

            if (frames.length === 0) {
                if (statusEl) statusEl.textContent = 'No frames found in file';
                return;
            }

            currentIndex = 0;
            if (scrubEl) {
                scrubEl.max = frames.length - 1;
                scrubEl.value = 0;
                scrubEl.disabled = false;
            }
            if (playBtn) playBtn.disabled = false;
            updateStatus();

            meridian.log('Loaded ' + frames.length + ' frames from ' + file.name, 'info');
        };
        reader.readAsArrayBuffer(file);
    }

    function parseFrames(data) {
        // Tlog format: 8 bytes timestamp (float64 LE, ms offset) + raw MAVLink bytes
        // We parse each entry: read timestamp, then attempt MAVLink decode
        const result = [];
        let offset = 0;
        const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);

        while (offset + 8 < data.length) {
            const timestamp = dv.getFloat64(offset, true);
            offset += 8;

            // Find next MAVLink v2 header (0xFD) or end of data
            if (offset >= data.length) break;

            // Look for MAVLink v2 frame
            if (data[offset] === 0xFD && offset + 12 <= data.length) {
                const payloadLen = data[offset + 1];
                const frameLen = 12 + payloadLen + 2; // header(10) + compat(1) + incompat(1) already in 12... simplified
                // Simple: just grab the msg type from bytes
                const msgId = data[offset + 7] | (data[offset + 8] << 8) | (data[offset + 9] << 16);

                // Create a simplified message for playback
                const frameEnd = Math.min(offset + 12 + payloadLen + 2, data.length);
                const frameBytes = data.slice(offset, frameEnd);
                offset = frameEnd;

                // Try to parse via MAVLink codec if available
                if (window.MAVLink && MAVLink.FrameParser) {
                    const parser = new MAVLink.FrameParser();
                    parser.push(frameBytes);
                    const msgs = parser.extract();
                    for (let i = 0; i < msgs.length; i++) {
                        result.push({ t: timestamp, msg: msgs[i] });
                    }
                } else {
                    result.push({ t: timestamp, msg: { type: 'raw', msgId: msgId, data: frameBytes } });
                }
            } else {
                // Skip unknown byte
                offset++;
            }
        }
        return result;
    }

    function play() {
        if (playing || frames.length === 0) return;
        playing = true;
        if (playBtn) playBtn.textContent = '\u23F8';

        scheduleNext();
    }

    function scheduleNext() {
        if (!playing || currentIndex >= frames.length - 1) {
            stop();
            return;
        }

        const current = frames[currentIndex];
        const next = frames[currentIndex + 1];
        const delay = Math.max(1, (next.t - current.t) / speed);

        playTimer = setTimeout(function () {
            currentIndex++;
            meridian.handleMessage(frames[currentIndex].msg);
            updateScrub();
            updateStatus();
            scheduleNext();
        }, delay);
    }

    function stop() {
        playing = false;
        if (playTimer) { clearTimeout(playTimer); playTimer = null; }
        if (playBtn) playBtn.textContent = '\u25B6';
    }

    function togglePlay() {
        if (playing) stop();
        else play();
    }

    function updateScrub() {
        if (scrubEl) scrubEl.value = currentIndex;
    }

    function updateStatus() {
        if (!statusEl) return;
        if (frames.length === 0) {
            statusEl.textContent = 'No file loaded';
            return;
        }
        const pct = ((currentIndex / (frames.length - 1)) * 100).toFixed(0);
        const timeMs = frames[currentIndex] ? frames[currentIndex].t : 0;
        const totalMs = frames[frames.length - 1].t;
        statusEl.textContent = 'Frame ' + (currentIndex + 1) + ' / ' + frames.length +
            ' (' + pct + '%) \u00b7 ' + formatTime(timeMs) + ' / ' + formatTime(totalMs) +
            ' \u00b7 ' + speed + 'x';
    }

    function formatTime(ms) {
        const s = Math.floor(ms / 1000);
        const m = Math.floor(s / 60);
        return m + ':' + String(s % 60).padStart(2, '0');
    }

    return { render };

})();
