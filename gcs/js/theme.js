/* ============================================================
   theme.js — Light/dark toggle with localStorage persistence
   Swaps map tiles and canvas colors on the fly.
   ============================================================ */

'use strict';

window.Theme = (function () {

    const TILES = {
        light: 'https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png',
        dark:  'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
    };

    // Canvas color sets
    const CANVAS = {
        light: {
            tapeBg:       '#f4f5f7',
            tapeBoxBg:    '#ffffff',
            tapeTick:     'rgba(30, 41, 59, 0.4)',
            tapeTickMin:  'rgba(30, 41, 59, 0.15)',
            tapeNum:      'rgba(30, 41, 59, 0.6)',
            compassBg:    '#f4f5f7',
            compassTick:  'rgba(30, 41, 59, 0.2)',
            compassMajor: 'rgba(30, 41, 59, 0.4)',
            compassText:  '#1e293b',
            compassDeg:   'rgba(30, 41, 59, 0.4)',
            compassPtr:   '#0891b2',
            compassN:     '#e65100',
            sky:          '#3b82f6',
            ground:       '#92400e',
            horizon:      '#ffffff',
            aircraft:     '#f59e0b',
            pitch:        'rgba(255, 255, 255, 0.45)',
            pitchText:    'rgba(255, 255, 255, 0.5)',
            bank:         'rgba(255, 255, 255, 0.55)',
            ghost:        'rgba(6, 182, 212, 0.7)',
            readout:      'rgba(255, 255, 255, 0.6)',
            altBox:       '#16a34a',
            altGhost:     'rgba(8, 145, 178, 0.5)',
            spdBox:       '#0891b2',
            spdGhost:     'rgba(22, 163, 74, 0.5)',
            vehicleColor: '#0891b2',
            trailColor:   '#0891b2',
        },
        dark: {
            tapeBg:       '#0a0e14',
            tapeBoxBg:    '#080c12',
            tapeTick:     'rgba(200, 210, 220, 0.5)',
            tapeTickMin:  'rgba(200, 210, 220, 0.2)',
            tapeNum:      'rgba(200, 210, 220, 0.6)',
            compassBg:    '#0a0e14',
            compassTick:  'rgba(200, 210, 220, 0.3)',
            compassMajor: 'rgba(200, 210, 220, 0.55)',
            compassText:  '#d0d8e2',
            compassDeg:   'rgba(200, 210, 220, 0.5)',
            compassPtr:   '#00e5ff',
            compassN:     '#ff6d00',
            sky:          '#1a3a6a',
            ground:       '#7a4e2e',
            horizon:      '#d0d8e0',
            aircraft:     '#ffa726',
            pitch:        'rgba(200, 215, 230, 0.35)',
            pitchText:    'rgba(200, 215, 230, 0.3)',
            bank:         'rgba(200, 215, 230, 0.45)',
            ghost:        'rgba(0, 229, 255, 0.7)',
            readout:      'rgba(180, 195, 210, 0.5)',
            altBox:       '#00ff7f',
            altGhost:     'rgba(0, 229, 255, 0.6)',
            spdBox:       '#00e5ff',
            spdGhost:     'rgba(0, 255, 127, 0.6)',
            vehicleColor: '#00e5ff',
            trailColor:   '#00e5ff',
        }
    };

    let current = 'light';
    let tileLayer = null;

    function get() { return current; }

    function set(theme) {
        current = theme;
        document.documentElement.setAttribute('data-theme', theme);
        localStorage.setItem('meridian_theme', theme);

        // Expose colors for canvas renderers
        window._themeColors = CANVAS[theme];

        // Swap map tiles
        swapTiles(theme);

        // Update toggle button glyph and label
        const btn = document.getElementById('theme-toggle');
        if (btn) {
            btn.textContent = theme === 'dark' ? '\u2600' : '\u263E';
            btn.title = theme === 'dark' ? 'Switch to light theme' : 'Switch to dark theme';
            btn.setAttribute('aria-label', btn.title);
        }

        meridian.events.emit('theme_change', theme);
    }

    function toggle() {
        set(current === 'light' ? 'dark' : 'light');
    }

    function swapTiles(theme) {
        const map = FlyMap.getMap();
        if (!map) return;

        if (tileLayer) map.removeLayer(tileLayer);

        tileLayer = L.tileLayer(TILES[theme], {
            attribution: '&copy; OSM &copy; CARTO',
            subdomains: 'abcd',
            maxZoom: 19,
        }).addTo(map);
    }

    function init() {
        const saved = localStorage.getItem('meridian_theme');
        const prefer = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
        set(saved || prefer);
    }

    return { get, set, toggle, init, CANVAS };

})();
