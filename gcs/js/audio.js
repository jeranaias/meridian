/* ============================================================
   audio.js — T1-24: Audio alerts for critical events
   Web Audio API tone generation, no external sound files.
   ============================================================ */

'use strict';

window.AudioAlerts = (function () {

    let ctx = null;
    let muted = false;

    function getContext() {
        if (!ctx) {
            try {
                ctx = new (window.AudioContext || window.webkitAudioContext)();
            } catch (e) {
                meridian.log('Web Audio API not available', 'warn');
                return null;
            }
        }
        return ctx;
    }

    // Play a tone: frequency (Hz), duration (ms), startDelay (ms)
    function playTone(freq, durationMs, startDelayMs) {
        var ac = getContext();
        if (!ac || muted) return;

        // Resume context if suspended (browser autoplay policy)
        if (ac.state === 'suspended') ac.resume();

        var osc = ac.createOscillator();
        var gain = ac.createGain();
        osc.connect(gain);
        gain.connect(ac.destination);

        osc.frequency.value = freq;
        osc.type = 'square';
        gain.gain.value = 0.15; // Keep volume low

        var start = ac.currentTime + (startDelayMs || 0) / 1000;
        var end = start + durationMs / 1000;

        osc.start(start);
        osc.stop(end);
    }

    // Failsafe: rapid beep pattern (440Hz, 100ms on/100ms off, 5 times)
    function playFailsafe() {
        for (var i = 0; i < 5; i++) {
            playTone(440, 100, i * 200);
        }
    }

    // Battery critical: low tone (220Hz, 500ms)
    function playBatteryCritical() {
        playTone(220, 500, 0);
    }

    // Link stale: double beep (880Hz, 50ms, gap 100ms, 50ms)
    function playLinkStale() {
        playTone(880, 50, 0);
        playTone(880, 50, 150);
    }

    function setMuted(val) {
        muted = !!val;
        // Persist preference
        try {
            localStorage.setItem('meridian_audio_muted', muted ? '1' : '0');
        } catch (e) { /* ignore */ }
    }

    function isMuted() {
        return muted;
    }

    function init() {
        // Load mute preference
        try {
            muted = localStorage.getItem('meridian_audio_muted') === '1';
        } catch (e) { /* ignore */ }

        // Wire to meridian events
        meridian.events.on('failsafe', function (data) {
            if (data.type === 'battery_critical') {
                playBatteryCritical();
            } else {
                playFailsafe();
            }
        });

        // Link stale — fired from heartbeat timeout watchdog
        meridian.events.on('link_stale', function () {
            playLinkStale();
        });
    }

    return {
        init: init,
        playFailsafe: playFailsafe,
        playBatteryCritical: playBatteryCritical,
        playLinkStale: playLinkStale,
        setMuted: setMuted,
        isMuted: isMuted,
    };

})();
