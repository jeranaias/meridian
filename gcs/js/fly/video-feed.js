/* ============================================================
   video-feed.js — Video feed PiP overlay
   Supports MJPEG URLs and HTML5 video sources.
   Floating PiP window over map, click header to drag,
   click swap button to toggle fullscreen.
   ============================================================ */

'use strict';

window.VideoFeed = (function () {

    let pipEl, videoEl, imgEl, headerEl;
    let source = null;
    let isFullscreen = false;
    let dragOffset = { x: 0, y: 0 };
    let dragging = false;

    function init() {
        // Create PiP container
        pipEl = document.createElement('div');
        pipEl.className = 'video-pip';
        pipEl.id = 'video-pip';
        pipEl.innerHTML =
            '<div class="video-pip-header">' +
                '<span class="video-pip-title">VIDEO</span>' +
                '<button type="button" class="video-pip-btn" id="video-swap" title="Swap with map">&#x21F1;</button>' +
                '<button type="button" class="video-pip-btn" id="video-close" title="Close video">&times;</button>' +
            '</div>' +
            '<div class="video-pip-body" id="video-body">' +
                '<div class="video-placeholder">No Video Source<br><span style="font-size:10px;opacity:0.5">Configure in Settings &gt; Connection</span></div>' +
            '</div>';

        const mapArea = document.getElementById('map-area');
        if (mapArea) mapArea.appendChild(pipEl);

        // Drag
        headerEl = pipEl.querySelector('.video-pip-header');
        headerEl.addEventListener('mousedown', onDragStart);

        // Close
        document.getElementById('video-close').addEventListener('click', () => {
            pipEl.style.display = 'none';
        });

        // Swap
        document.getElementById('video-swap').addEventListener('click', toggleFullscreen);

        // Listen for settings changes
        meridian.events.on('video_source_change', setSource);

        // Check saved source
        const saved = meridian.settings && meridian.settings.videoUrl;
        if (saved) setSource(saved);
    }

    function setSource(url) {
        if (!url) return;
        source = url;
        const body = document.getElementById('video-body');
        if (!body) return;
        body.innerHTML = '';
        pipEl.style.display = 'flex';

        if (url.match(/\.(mjpg|mjpeg|jpg|jpeg)/i) || url.includes('mjpeg')) {
            // MJPEG stream via img tag
            imgEl = document.createElement('img');
            imgEl.className = 'video-feed-img';
            imgEl.src = url;
            imgEl.alt = 'Video feed';
            body.appendChild(imgEl);
        } else {
            // HTML5 video (RTSP via WebRTC proxy, HLS, etc)
            videoEl = document.createElement('video');
            videoEl.className = 'video-feed-el';
            videoEl.autoplay = true;
            videoEl.muted = true;
            videoEl.playsInline = true;
            videoEl.src = url;
            body.appendChild(videoEl);
        }
    }

    function toggleFullscreen() {
        isFullscreen = !isFullscreen;
        pipEl.classList.toggle('video-fullscreen', isFullscreen);
    }

    function onDragStart(e) {
        if (e.target.tagName === 'BUTTON') return;
        dragging = true;
        const r = pipEl.getBoundingClientRect();
        dragOffset.x = e.clientX - r.left;
        dragOffset.y = e.clientY - r.top;
        document.addEventListener('mousemove', onDragMove);
        document.addEventListener('mouseup', onDragEnd);
    }

    function onDragMove(e) {
        if (!dragging) return;
        pipEl.style.left = (e.clientX - dragOffset.x) + 'px';
        pipEl.style.top = (e.clientY - dragOffset.y) + 'px';
        pipEl.style.right = 'auto';
        pipEl.style.bottom = 'auto';
    }

    function onDragEnd() {
        dragging = false;
        document.removeEventListener('mousemove', onDragMove);
        document.removeEventListener('mouseup', onDragEnd);
    }

    function show() { if (pipEl) pipEl.style.display = 'flex'; }
    function hide() { if (pipEl) pipEl.style.display = 'none'; }

    return { init, setSource, show, hide };

})();
