/* ============================================================
   confirm.js — Slide-to-confirm ARM/DISARM + Long-press KILL
   Krug: KILL isolated with different shape, long-press required.
   ============================================================ */

'use strict';

window.Confirm = (function () {

    let slideEl, thumbEl, labelEl;
    let killBtn;
    let killTimer = null;
    const KILL_HOLD_MS = 1500;

    function init() {
        slideEl = document.querySelector('.slide-to-arm');
        thumbEl = slideEl ? slideEl.querySelector('.slide-thumb') : null;
        labelEl = slideEl ? slideEl.querySelector('.slide-track-label') : null;
        killBtn = document.querySelector('.kill-btn');

        if (slideEl) initSlideToArm();
        if (killBtn) initKillSwitch();

        meridian.events.on('heartbeat', updateArmState);
    }

    // ---- Slide-to-Arm ----

    function initSlideToArm() {
        let dragging = false;
        let startX = 0;
        const maxTravel = 100; // px

        function onStart(e) {
            e.preventDefault();
            dragging = true;
            startX = (e.touches ? e.touches[0].clientX : e.clientX);
            thumbEl.style.transition = 'none';
        }

        function onMove(e) {
            if (!dragging) return;
            const x = (e.touches ? e.touches[0].clientX : e.clientX);
            const dx = x - startX;
            const v = meridian.v;
            const armed = v && v.armed;

            if (armed) {
                // Slide LEFT to disarm
                const clamped = Math.max(-maxTravel, Math.min(0, dx));
                const pct = Math.abs(clamped) / maxTravel;
                thumbEl.style.left = `calc(100% - 35px + ${clamped}px)`;
                if (labelEl) labelEl.style.opacity = pct > 0.3 ? pct.toString() : '0';
            } else {
                // Slide RIGHT to arm
                const clamped = Math.max(0, Math.min(maxTravel, dx));
                const pct = clamped / maxTravel;
                thumbEl.style.left = (3 + clamped) + 'px';
                if (labelEl) labelEl.style.opacity = pct > 0.3 ? pct.toString() : '0';
            }
        }

        function onEnd(e) {
            if (!dragging) return;
            dragging = false;
            thumbEl.style.transition = '';

            const x = (e.changedTouches ? e.changedTouches[0].clientX : e.clientX);
            const dx = x - startX;
            const v = meridian.v;
            const armed = v && v.armed;
            const threshold = maxTravel * 0.7;

            if (armed && dx < -threshold) {
                Connection.sendDisarm();
                meridian.log('Disarm command sent', 'info');
            } else if (!armed && dx > threshold) {
                // T1-10: Pre-flight checklist gate before arming
                checklistGateAndArm();
            }

            // Reset position
            updateArmState();
        }

        thumbEl.addEventListener('mousedown', onStart);
        thumbEl.addEventListener('touchstart', onStart, { passive: false });
        document.addEventListener('mousemove', onMove);
        document.addEventListener('touchmove', onMove, { passive: false });
        document.addEventListener('mouseup', onEnd);
        document.addEventListener('touchend', onEnd);
    }

    function updateArmState() {
        if (!slideEl || !thumbEl || !labelEl) return;
        const v = meridian.v;
        const armed = v && v.armed;

        slideEl.classList.toggle('armed', armed);
        thumbEl.style.left = '';

        if (armed) {
            labelEl.textContent = '\u25C0 DISARM';
        } else {
            labelEl.textContent = 'ARM \u25B6';
        }
        labelEl.style.opacity = '1';
    }

    // ---- Long-press Kill Switch ----

    function initKillSwitch() {
        function onDown(e) {
            e.preventDefault();
            killBtn.classList.add('pressing');
            killTimer = setTimeout(() => {
                // KILL = flight termination (MAV_CMD_DO_FLIGHTTERMINATION)
                // NOT disarm — this forcibly stops all motors mid-flight
                if (Connection.protocol === 'mavlink') {
                    const frame = MAVLink.encodeCommandLong
                        ? MAVLink.encodeCommandLong(1, 1, 185, 1, 0, 0, 0, 0, 0, 0) // cmd 185, param1=1
                        : MAVLink.encodeDisarm(); // fallback
                    Connection.send(frame);
                } else {
                    Connection.sendDisarm(); // MNP fallback
                }
                meridian.log('EMERGENCY KILL — FLIGHT TERMINATION SENT', 'error');
                killBtn.classList.remove('pressing');
                killBtn.classList.add('killed');
                setTimeout(() => killBtn.classList.remove('killed'), 2000);
            }, KILL_HOLD_MS);
        }

        function onUp() {
            killBtn.classList.remove('pressing');
            if (killTimer) {
                clearTimeout(killTimer);
                killTimer = null;
            }
        }

        killBtn.addEventListener('mousedown', onDown);
        killBtn.addEventListener('touchstart', onDown, { passive: false });
        killBtn.addEventListener('mouseup', onUp);
        killBtn.addEventListener('touchend', onUp);
        killBtn.addEventListener('mouseleave', onUp);
    }

    // T1-10: Check pre-flight checklist before arming
    async function checklistGateAndArm() {
        if (window.Checklist) {
            const v = meridian.v;
            const params = v ? v.params : {};
            const incomplete = Checklist.incompleteRequiredCount(params);
            if (incomplete > 0) {
                // Build list of incomplete items
                const results = Checklist.evaluate(params);
                const incompleteItems = results
                    .filter(function (r) { return r.required && r.status !== 'complete'; })
                    .map(function (r) { return r.name + ' (' + r.text + ')'; });
                const listHtml = incompleteItems.map(function (s) { return '&bull; ' + s; }).join('<br>');
                const ok = await Modal.confirm(
                    'Pre-Flight Checklist Incomplete',
                    incomplete + ' required setup item' + (incomplete > 1 ? 's' : '') + ' incomplete:<br><br>' +
                    '<div style="font-size:12px;color:var(--c-warning);margin-bottom:8px;line-height:1.6;">' + listHtml + '</div>' +
                    '<strong style="color:var(--c-emergency);">ARM anyway?</strong>',
                    'ARM Override',
                    true
                );
                if (!ok) {
                    meridian.log('Arm cancelled — checklist incomplete', 'warn');
                    return;
                }
                meridian.log('Arm override — ' + incomplete + ' checklist items incomplete', 'warn');
            }
        }
        Connection.sendArm();
        meridian.log('Arm command sent', 'info');
    }

    // T0-11: Keyboard shortcuts for ARM (Ctrl+Shift+A) and KILL (Ctrl+Shift+K)
    function initKeyboardShortcuts() {
        document.addEventListener('keydown', function (e) {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

            // Ctrl+Shift+A = toggle arm/disarm
            if (e.ctrlKey && e.shiftKey && e.key === 'A') {
                e.preventDefault();
                const v = meridian.v;
                if (v && v.armed) {
                    Connection.sendDisarm();
                    meridian.log('Keyboard DISARM sent', 'info');
                } else {
                    Connection.sendArm();
                    meridian.log('Keyboard ARM sent', 'info');
                }
            }

            // Ctrl+Shift+K = emergency kill (immediate, no hold required for keyboard)
            if (e.ctrlKey && e.shiftKey && e.key === 'K') {
                e.preventDefault();
                if (Connection.protocol === 'mavlink') {
                    Connection.send(MAVLink.encodeCommandLong(1, 1, 185, 1, 0, 0, 0, 0, 0, 0));
                } else {
                    Connection.sendDisarm();
                }
                meridian.log('KEYBOARD KILL — FLIGHT TERMINATION', 'error');
            }
        });
    }

    return { init, updateArmState, initKeyboardShortcuts };

})();
