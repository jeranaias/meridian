/* ============================================================
   frame.js — Frame type selector
   Grid of frame icons, sets FRAME_CLASS and FRAME_TYPE.
   ============================================================ */

'use strict';

window.FrameSetup = (function () {

    // FRAME_CLASS values (ArduCopter)
    var FRAME_CLASSES = [
        { cls: 1, name: 'Quad',   motors: 4 },
        { cls: 2, name: 'Hexa',   motors: 6 },
        { cls: 3, name: 'Octa',   motors: 8 },
        { cls: 4, name: 'OctaQuad', motors: 8 },
        { cls: 5, name: 'Y6',     motors: 6 },
        { cls: 7, name: 'Tri',    motors: 3 },
        { cls: 6, name: 'Heli',   motors: 1 },
        { cls: 11, name: 'Single', motors: 1 },
    ];

    // FRAME_TYPE values (layout)
    var FRAME_TYPES = [
        { type: 0, name: 'Plus (+)',  symbol: '+' },
        { type: 1, name: 'X',         symbol: 'X' },
        { type: 2, name: 'V',         symbol: 'V' },
        { type: 3, name: 'H',         symbol: 'H' },
        { type: 10, name: 'Y',        symbol: 'Y' },
    ];

    function getFrameName(cls, type) {
        var c = null;
        for (var i = 0; i < FRAME_CLASSES.length; i++) {
            if (FRAME_CLASSES[i].cls === cls) { c = FRAME_CLASSES[i]; break; }
        }
        var t = null;
        for (var j = 0; j < FRAME_TYPES.length; j++) {
            if (FRAME_TYPES[j].type === type) { t = FRAME_TYPES[j]; break; }
        }
        if (!c) return 'Unknown';
        if (!t) return c.name;
        return c.name + '-' + t.symbol;
    }

    function buildFrameIcon(cls, isSelected) {
        var entry = null;
        for (var i = 0; i < FRAME_CLASSES.length; i++) {
            if (FRAME_CLASSES[i].cls === cls) { entry = FRAME_CLASSES[i]; break; }
        }
        if (!entry) return '';
        var n = entry.motors;
        var svg = '<svg viewBox="0 0 48 48" width="48" height="48">';
        svg += '<circle cx="24" cy="24" r="5" fill="none" stroke="currentColor" stroke-width="1.5"/>';
        var positions = [];
        if (n === 3) { positions = [[24,8],[8,36],[40,36]]; }
        else if (n === 4) { positions = [[10,10],[38,10],[10,38],[38,38]]; }
        else if (n === 6) { positions = [[24,6],[8,16],[40,16],[8,32],[40,32],[24,42]]; }
        else if (n === 8) { positions = [[24,6],[38,10],[42,24],[38,38],[24,42],[10,38],[6,24],[10,10]]; }
        else { positions = [[24,10]]; }
        for (var j = 0; j < positions.length; j++) {
            var px = positions[j][0], py = positions[j][1];
            svg += '<line x1="24" y1="24" x2="' + px + '" y2="' + py + '" stroke="currentColor" stroke-width="1" opacity="0.5"/>';
            svg += '<circle cx="' + px + '" cy="' + py + '" r="4" fill="' + (isSelected ? 'var(--c-primary)' : 'currentColor') + '" opacity="' + (isSelected ? '1' : '0.6') + '"/>';
        }
        svg += '</svg>';
        return svg;
    }

    function render(container, params, onParamChange) {
        params = params || {};
        var currentClass = params.FRAME_CLASS || 0;
        var currentType = params.FRAME_TYPE || 0;

        var html = '<div class="setup-form-section">';
        html += '<div class="setup-form-title">Frame Class</div>';
        html += '<div class="frame-grid">';
        for (var i = 0; i < FRAME_CLASSES.length; i++) {
            var fc = FRAME_CLASSES[i];
            var sel = fc.cls === currentClass;
            html += '<button class="frame-card' + (sel ? ' selected' : '') + '" data-frame-class="' + fc.cls + '">';
            html += buildFrameIcon(fc.cls, sel);
            html += '<span class="frame-card-name">' + fc.name + '</span>';
            html += '</button>';
        }
        html += '</div>';
        html += '</div>';

        html += '<div class="setup-form-section">';
        html += '<div class="setup-form-title">Frame Type (Layout)</div>';
        html += '<div class="frame-type-row">';
        for (var j = 0; j < FRAME_TYPES.length; j++) {
            var ft = FRAME_TYPES[j];
            var ssel = ft.type === currentType;
            html += '<button class="frame-type-btn' + (ssel ? ' selected' : '') + '" data-frame-type="' + ft.type + '">';
            html += '<span class="frame-type-symbol">' + ft.symbol + '</span>';
            html += '<span class="frame-type-name">' + ft.name + '</span>';
            html += '</button>';
        }
        html += '</div>';
        html += '</div>';

        html += '<div class="setup-form-section">';
        html += '<div class="setup-current">Current: <strong>' + getFrameName(currentClass, currentType) + '</strong></div>';
        html += '</div>';

        container.innerHTML = html;

        // Wire class buttons
        container.querySelectorAll('[data-frame-class]').forEach(function (btn) {
            btn.addEventListener('click', function () {
                var cls = parseInt(btn.dataset.frameClass);
                if (onParamChange) {
                    onParamChange('FRAME_CLASS', cls);
                }
                render(container, Object.assign({}, params, { FRAME_CLASS: cls }), onParamChange);
            });
        });

        // Wire type buttons
        container.querySelectorAll('[data-frame-type]').forEach(function (btn) {
            btn.addEventListener('click', function () {
                var type = parseInt(btn.dataset.frameType);
                if (onParamChange) {
                    onParamChange('FRAME_TYPE', type);
                }
                render(container, Object.assign({}, params, { FRAME_TYPE: type }), onParamChange);
            });
        });
    }

    return { render, getFrameName, FRAME_CLASSES, FRAME_TYPES };

})();
