/* ============================================================
   router.js — Overlay panel router
   Fly View = HUD on map (no panel). Other views = panel slides in.
   ============================================================ */

'use strict';

window.Router = (function () {

    let activePanel = 'fly';

    function switchTo(panelId) {
        const panelArea = document.getElementById('panel-area');
        const hud = document.getElementById('hud');

        // Deactivate all panel views
        document.querySelectorAll('.panel-view').forEach(v => v.classList.remove('active'));

        // Update toolbar nav
        document.querySelectorAll('.toolbar-nav button').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.panel === panelId);
        });

        if (panelId === 'fly') {
            // Fly view: hide panel, show HUD
            if (panelArea) panelArea.classList.remove('open');
            if (hud) hud.style.display = '';
        } else {
            // Other views: show panel, keep HUD visible (map is still base)
            const target = document.getElementById('panel-' + panelId);
            if (target) target.classList.add('active');
            if (panelArea) panelArea.classList.add('open');
            // Keep HUD visible for SA (Meier: never dim below 0.85)
            if (hud) hud.style.opacity = '0.85';
        }

        if (panelId === 'fly' && hud) {
            hud.style.opacity = '';
        }

        activePanel = panelId;
        meridian.events.emit('panel_change', panelId);
    }

    function showShortcutHelp() {
        var existing = document.getElementById('shortcut-overlay');
        if (existing) { existing.remove(); return; }
        var overlay = document.createElement('div');
        overlay.id = 'shortcut-overlay';
        overlay.style.cssText = 'position:fixed;inset:0;z-index:var(--z-modal,10000);background:rgba(0,0,0,0.6);display:flex;align-items:center;justify-content:center';
        overlay.innerHTML =
            '<div style="background:var(--c-bg-raised);border:1px solid var(--c-border);border-radius:var(--r-lg,10px);padding:20px 28px;max-width:380px;box-shadow:0 8px 40px rgba(0,0,0,0.3)">' +
            '<div style="font-family:var(--f-display);font-size:16px;font-weight:700;color:var(--c-text);margin-bottom:12px">Keyboard Shortcuts</div>' +
            '<div style="display:grid;grid-template-columns:60px 1fr;gap:6px 12px;font-size:12px;color:var(--c-text-dim)">' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">F</kbd><span>Fly view</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">P</kbd><span>Plan view</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">S</kbd><span>Setup view</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">R</kbd><span>Parameters</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">L</kbd><span>Logs view</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">T</kbd><span>Status view</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">Ctrl+,</kbd><span>Settings</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">Esc</kbd><span>Close panel / menu</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">Ctrl+Shift+A</kbd><span>Arm / Disarm</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">Ctrl+Shift+K</kbd><span>Emergency KILL</span>' +
            '<kbd style="background:var(--c-bg-input);padding:2px 8px;border-radius:3px;font-family:var(--f-mono);text-align:center;color:var(--c-text)">?</kbd><span>This help</span>' +
            '</div>' +
            '<div style="margin-top:14px;font-size:11px;color:var(--c-neutral-dim);text-align:center">Press Esc or ? to close</div>' +
            '</div>';
        overlay.addEventListener('click', function (e) { if (e.target === overlay) overlay.remove(); });
        document.body.appendChild(overlay);
    }

    function handleKeyboard(e) {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

        // ? key: toggle keyboard shortcut help
        if (e.key === '?') {
            e.preventDefault(); showShortcutHelp(); return;
        }

        // Escape: close shortcut overlay, then context menu, then panel
        var shortcutOverlay = document.getElementById('shortcut-overlay');
        if (e.key === 'Escape' && shortcutOverlay) {
            shortcutOverlay.remove(); return;
        }

        var ctxMenu = document.querySelector('.map-context-menu.visible');
        if (e.key === 'Escape' && ctxMenu) {
            ctxMenu.classList.remove('visible');
            return;
        }

        if (e.key === 'Escape' || e.key === 'f' || e.key === 'F') {
            e.preventDefault(); switchTo('fly');
        } else if (e.key === 'p' && !e.ctrlKey) {
            e.preventDefault(); switchTo('plan');
        } else if (e.key === 'r' && !e.ctrlKey) {
            e.preventDefault(); switchTo('params');
        } else if (e.key === 's' || e.key === 'S') {
            e.preventDefault(); switchTo('setup');
        } else if (e.key === 'l' || e.key === 'L') {
            e.preventDefault(); switchTo('logs');
        } else if (e.key === 't' && !e.ctrlKey) {
            e.preventDefault(); switchTo('status');
        } else if (e.key === ',' && e.ctrlKey) {
            e.preventDefault(); switchTo('settings');
        }
    }

    function init() {
        document.addEventListener('keydown', handleKeyboard);
        document.querySelectorAll('.toolbar-nav button').forEach(btn => {
            btn.addEventListener('click', () => switchTo(btn.dataset.panel));
        });
        switchTo('fly');
    }

    return { init, switchTo, get active() { return activePanel; } };

})();
