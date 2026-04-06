/* ============================================================
   onboarding.js — First-time guided tour (T2-13)
   Step-by-step spotlight overlay highlighting key UI elements.
   Runs on first visit; can be re-triggered from Settings.
   ============================================================ */

'use strict';

window.Onboarding = (function () {

    var STORAGE_KEY = 'meridian_onboarded';

    // Ordered steps: each step highlights an element and shows text
    var STEPS = [
        {
            selector: '.flight-state-badge',
            title: 'Flight State Badge',
            text: 'This is the flight state badge — always shows your vehicle status: armed/disarmed, flight mode, voltage, GPS satellite count, and flight time.',
            position: 'below',
        },
        {
            selector: '.mode-row',
            title: 'Flight Modes',
            text: 'These are flight modes — tap any button to switch the vehicle mode. Common modes like STABILIZE, LOITER, and RTL are shown here.',
            position: 'above',
        },
        {
            selector: '.slide-to-arm',
            title: 'ARM Slider',
            text: 'Slide to ARM your vehicle. Drag the thumb all the way to the right to arm. Drag left to disarm. The vehicle must be connected and ready.',
            position: 'above',
        },
        {
            selector: '#btn-takeoff',
            title: 'Takeoff Button',
            text: 'Click TAKEOFF to launch. Set the desired altitude in the number field next to it. The vehicle must be armed first.',
            position: 'above',
        },
        {
            selector: '.kill-btn',
            title: 'Kill Switch',
            text: 'KILL button — hold 1.5 seconds for emergency motor stop. Use only in emergencies: this cuts motor power immediately.',
            position: 'above',
        },
    ];

    var _overlay   = null;
    var _spotlight = null;
    var _card      = null;
    var _step      = 0;

    // ─── Public API ───────────────────────────────────────────

    function init() {
        // Don't auto-start tutorial — user can trigger via Settings > Help > Show Tutorial
        // First-run hint in the message log instead
        if (!localStorage.getItem(STORAGE_KEY)) {
            setTimeout(function () {
                meridian.log('Welcome to Meridian GCS. Press ? for keyboard shortcuts, or visit Settings to start the tutorial.', 'info');
                localStorage.setItem(STORAGE_KEY, '1');
            }, 800);
        }
    }

    function start(forced) {
        if (_overlay) return; // already running
        _step = 0;
        _build();
        _showStep(0);
    }

    // ─── Build overlay elements ───────────────────────────────

    function _build() {
        // Dark backdrop
        _overlay = document.createElement('div');
        _overlay.className = 'onboard-overlay';
        _overlay.setAttribute('aria-modal', 'true');
        _overlay.setAttribute('role', 'dialog');
        _overlay.setAttribute('aria-label', 'Meridian tutorial');

        // Spotlight cut-out (SVG mask approach via clip-path on a hole div)
        _spotlight = document.createElement('div');
        _spotlight.className = 'onboard-spotlight';

        // Info card
        _card = document.createElement('div');
        _card.className = 'onboard-card';
        _card.innerHTML =
            '<div class="onboard-step-indicator" id="onboard-step-indicator"></div>' +
            '<div class="onboard-title" id="onboard-title"></div>' +
            '<div class="onboard-text" id="onboard-text"></div>' +
            '<div class="onboard-actions">' +
            '<button class="onboard-btn-skip" id="onboard-skip">Skip Tutorial</button>' +
            '<button class="onboard-btn-next" id="onboard-next">Next</button>' +
            '</div>';

        _overlay.appendChild(_spotlight);
        _overlay.appendChild(_card);
        document.body.appendChild(_overlay);

        document.getElementById('onboard-skip').addEventListener('click', _finish);
        document.getElementById('onboard-next').addEventListener('click', _advance);

        // Keyboard support
        _overlay.addEventListener('keydown', function (e) {
            if (e.key === 'Escape') { _finish(); }
            else if (e.key === 'ArrowRight' || e.key === 'Enter') { _advance(); }
        });
        _overlay.setAttribute('tabindex', '-1');
        _overlay.focus();
    }

    function _showStep(idx) {
        var step = STEPS[idx];
        var target = document.querySelector(step.selector);

        // Update card content
        var indicator = document.getElementById('onboard-step-indicator');
        var title     = document.getElementById('onboard-title');
        var text      = document.getElementById('onboard-text');
        var nextBtn   = document.getElementById('onboard-next');

        if (indicator) {
            indicator.innerHTML = '';
            STEPS.forEach(function (_, i) {
                var dot = document.createElement('span');
                dot.className = 'onboard-dot' + (i === idx ? ' active' : '');
                indicator.appendChild(dot);
            });
        }
        if (title) title.textContent = step.title;
        if (text)  text.textContent  = step.text;
        if (nextBtn) nextBtn.textContent = (idx === STEPS.length - 1) ? 'Done' : 'Next';

        // Spotlight the target element
        if (target) {
            var rect = target.getBoundingClientRect();
            var pad  = 8;
            _spotlight.style.cssText =
                'left:'   + (rect.left   - pad) + 'px;' +
                'top:'    + (rect.top    - pad) + 'px;' +
                'width:'  + (rect.width  + pad * 2) + 'px;' +
                'height:' + (rect.height + pad * 2) + 'px;';
            _spotlight.style.display = 'block';
            target.classList.add('onboard-highlighted');

            // Position card
            _positionCard(rect, step.position);
        } else {
            _spotlight.style.display = 'none';
            // Center card if element not found
            _card.style.cssText = 'top:50%;left:50%;transform:translate(-50%,-50%);';
        }
    }

    function _positionCard(targetRect, position) {
        var cw  = 320; // card width
        var ch  = 160; // estimated card height
        var pad = 16;
        var vw  = window.innerWidth;
        var vh  = window.innerHeight;

        var left = targetRect.left + targetRect.width / 2 - cw / 2;
        var top;

        if (position === 'below') {
            top = targetRect.bottom + pad;
        } else {
            top = targetRect.top - ch - pad;
            if (top < pad) top = targetRect.bottom + pad; // flip if no room above
        }

        // Clamp to viewport
        left = Math.max(pad, Math.min(left, vw - cw - pad));
        top  = Math.max(pad, Math.min(top,  vh - ch - pad));

        _card.style.cssText = 'left:' + left + 'px;top:' + top + 'px;transform:none;';
    }

    function _clearHighlight() {
        if (_step >= 0 && _step < STEPS.length) {
            var prev = document.querySelector(STEPS[_step].selector);
            if (prev) prev.classList.remove('onboard-highlighted');
        }
    }

    function _advance() {
        _clearHighlight();
        _step++;
        if (_step >= STEPS.length) {
            _finish();
        } else {
            _showStep(_step);
        }
    }

    function _finish() {
        _clearHighlight();
        if (_overlay && _overlay.parentNode) {
            _overlay.parentNode.removeChild(_overlay);
        }
        _overlay   = null;
        _spotlight = null;
        _card      = null;
        localStorage.setItem(STORAGE_KEY, '1');
    }

    return { init, start };

})();
