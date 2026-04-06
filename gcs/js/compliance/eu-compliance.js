/* ============================================================
   eu-compliance.js — EU UAS C-class Operational Enforcement
   T3-13: C0-C6 class selector + Open/Specific/Certified category.
   Enforces mass and altitude limits. Integrates with PlanValidator.
   Displays class/category on flight-state-badge line 2.
   ============================================================ */

'use strict';

window.EUCompliance = (function () {

    // --- Class Definitions ---
    const C_CLASSES = {
        C0: {
            label: 'C0',
            maxMassKg: 0.250,
            maxAltM: 120,
            requiresGeoAwareness: false,
            requiresCert: null,
            notes: 'Max 250g, max 120m AGL, no geo-awareness required',
        },
        C1: {
            label: 'C1',
            maxMassKg: 0.900,
            maxAltM: 120,
            requiresGeoAwareness: true,
            requiresCert: null,
            notes: 'Max 900g, max 120m, geo-awareness required',
        },
        C2: {
            label: 'C2',
            maxMassKg: 4,
            maxAltM: 120,
            requiresGeoAwareness: true,
            requiresCert: 'A2',
            notes: 'Max 4kg, max 120m, A2 certificate required',
        },
        C3: {
            label: 'C3',
            maxMassKg: 25,
            maxAltM: 120,
            requiresGeoAwareness: true,
            requiresCert: 'STS',
            notes: 'Max 25kg, max 120m, STS required',
        },
        C4: {
            label: 'C4',
            maxMassKg: 25,
            maxAltM: 120,
            requiresGeoAwareness: false,
            requiresCert: null,
            notes: 'Max 25kg, max 120m, no geo-awareness, legacy class',
        },
        C5: {
            label: 'C5',
            maxMassKg: 25,
            maxAltM: 120,
            requiresGeoAwareness: true,
            requiresCert: 'STS',
            notes: 'Max 25kg, max 120m, STS or NTS required',
        },
        C6: {
            label: 'C6',
            maxMassKg: 25,
            maxAltM: 120,
            requiresGeoAwareness: true,
            requiresCert: 'STS',
            notes: 'Max 25kg, max 120m, tethered or restricted',
        },
    };

    const OP_CATEGORIES = {
        'open_a1':   { label: 'Open A1', description: 'Over uninvolved persons, VLOS' },
        'open_a2':   { label: 'Open A2', description: 'Near uninvolved persons, A2 cert' },
        'open_a3':   { label: 'Open A3', description: 'Away from people, VLOS' },
        'specific':  { label: 'Specific', description: 'Requires operational authorization' },
        'certified': { label: 'Certified', description: 'Certified operations, like manned aviation' },
    };

    // --- Current selection ---
    let currentClass = meridian.settings.euUasClass || 'C0';
    let currentCategory = meridian.settings.euOpCategory || 'open_a1';
    let _violations = []; // Cache of current violations for other modules

    // --- Validate current mission against class limits ---
    function validate() {
        _violations = [];
        const cls = C_CLASSES[currentClass];
        if (!cls) return _violations;

        if (!window.Mission) return _violations;
        const items = Mission.getItems();

        for (let i = 0; i < items.length; i++) {
            const it = items[i];
            if (it.alt > cls.maxAltM) {
                _violations.push({
                    type: 'error',
                    text: 'WP ' + (i + 1) + ': altitude ' + it.alt.toFixed(0) +
                          'm exceeds EU C' + currentClass + ' limit (' + cls.maxAltM + 'm AGL)',
                });
            }
        }

        if (cls.requiresGeoAwareness) {
            const geoOk = meridian.v && meridian.v.params && meridian.v.params['FENCE_ENABLE'] === 1;
            if (!geoOk) {
                _violations.push({
                    type: 'warn',
                    text: cls.label + ' requires geo-awareness (geofence). FENCE_ENABLE not set.',
                });
            }
        }

        if (cls.requiresCert) {
            _violations.push({
                type: 'warn',
                text: cls.label + ' class requires ' + cls.requiresCert + ' operational certification.',
            });
        }

        return _violations;
    }

    function getViolations() { return _violations; }

    // --- Inject into PlanValidator ---
    // Called from validator.js — hook into its validate() cycle
    function injectIntoValidator() {
        if (!window.PlanValidator) return;

        const origValidate = PlanValidator.validate;
        PlanValidator.validate = function () {
            origValidate.call(PlanValidator);

            // Append EU violations to the existing warnings el
            const el = window.PlanStats && PlanStats.getWarningsEl ? PlanStats.getWarningsEl() : null;
            if (!el) return;

            const euViols = validate();
            if (euViols.length === 0) return;

            let extra = '';
            for (const v of euViols) {
                const cls = v.type === 'error' ? ' error' : '';
                extra += '<div class="plan-warn-item' + cls + '">' +
                    '<span class="plan-warn-icon">&#x1F1EA;&#x1F1FA;</span> ' +
                    v.text + '</div>';
            }
            el.innerHTML = el.innerHTML + extra;
        };
    }

    // --- Badge line 2 supplement ---
    // Called by toolbar.js updateBadge — append class/category info
    function getBadgeSupplement() {
        const cat = OP_CATEGORIES[currentCategory];
        return 'EU ' + currentClass + ' \u00b7 ' + (cat ? cat.label : currentCategory);
    }

    // --- Inject badge supplement ---
    function injectBadge() {
        if (!window.Toolbar) return;

        const origUpdateBadge = Toolbar._updateBadgeHook || null;

        // Patch meridian.events 'heartbeat' listener to append EU info to line 2
        meridian.events.on('heartbeat', function () {
            const line2 = document.querySelector('.flight-state-line2');
            if (!line2) return;
            // Remove any existing EU badge fragment and re-append
            let text = line2.textContent || '';
            // Strip previous EU tag if present
            text = text.replace(/\s*\|\s*EU\s+C\d.*/i, '').replace(/\s*\|\s*EU\s+open.*/i, '');
            line2.textContent = text + ' | ' + getBadgeSupplement();
        });
    }

    // --- Persist settings ---
    function applyClass(cls) {
        if (!C_CLASSES[cls]) return;
        currentClass = cls;
        meridian.settings.euUasClass = cls;
        meridian.saveSettings();
        meridian.events.emit('eu_class_change', { cls, cat: currentCategory });
        if (window.PlanValidator) PlanValidator.validate();
    }

    function applyCategory(cat) {
        if (!OP_CATEGORIES[cat]) return;
        currentCategory = cat;
        meridian.settings.euOpCategory = cat;
        meridian.saveSettings();
        meridian.events.emit('eu_class_change', { cls: currentClass, cat });
    }

    // --- Settings panel section builder ---
    function buildSettingsSection(wrapper, helpers) {
        // Expect helpers = { createSectionHeader, createSelectField }
        // Build a standalone block if helpers not available

        const section = document.createElement('div');
        section.innerHTML = '<div class="settings-section-header" style="' +
            'font-family:var(--font-display,Rajdhani,sans-serif);font-weight:600;font-size:11px;' +
            'letter-spacing:.12em;color:#64748b;padding:10px 0 4px;border-top:1px solid rgba(255,255,255,0.06);' +
            'text-transform:uppercase">EU Compliance</div>';

        // Class selector
        const classRow = document.createElement('div');
        classRow.className = 'settings-field';
        classRow.style.cssText = 'display:flex;align-items:center;padding:5px 0;gap:8px';

        const classLabel = document.createElement('label');
        classLabel.className = 'settings-field-label';
        classLabel.textContent = 'UAS C-Class';
        classLabel.style.cssText = 'flex:1;color:var(--c-neutral);font-size:12px';

        const classSelect = document.createElement('select');
        classSelect.className = 'settings-select';
        Object.entries(C_CLASSES).forEach(function ([key, cls]) {
            const opt = document.createElement('option');
            opt.value = key;
            opt.textContent = cls.label + ' (' + cls.notes.split(',')[0] + ')';
            if (key === currentClass) opt.selected = true;
            classSelect.appendChild(opt);
        });
        classSelect.addEventListener('change', function () { applyClass(classSelect.value); });

        classRow.appendChild(classLabel);
        classRow.appendChild(classSelect);
        section.appendChild(classRow);

        // Category selector
        const catRow = document.createElement('div');
        catRow.className = 'settings-field';
        catRow.style.cssText = 'display:flex;align-items:center;padding:5px 0;gap:8px';

        const catLabel = document.createElement('label');
        catLabel.className = 'settings-field-label';
        catLabel.textContent = 'Op. Category';
        catLabel.style.cssText = 'flex:1;color:var(--c-neutral);font-size:12px';

        const catSelect = document.createElement('select');
        catSelect.className = 'settings-select';
        Object.entries(OP_CATEGORIES).forEach(function ([key, cat]) {
            const opt = document.createElement('option');
            opt.value = key;
            opt.textContent = cat.label + ' — ' + cat.description;
            if (key === currentCategory) opt.selected = true;
            catSelect.appendChild(opt);
        });
        catSelect.addEventListener('change', function () { applyCategory(catSelect.value); });

        catRow.appendChild(catLabel);
        catRow.appendChild(catSelect);
        section.appendChild(catRow);

        // Class info note
        const note = document.createElement('div');
        note.id = 'eu-class-note';
        note.style.cssText = 'font-size:10px;color:var(--c-text-dim);padding:3px 0 6px';
        note.textContent = C_CLASSES[currentClass] ? C_CLASSES[currentClass].notes : '';
        section.appendChild(note);

        classSelect.addEventListener('change', function () {
            note.textContent = C_CLASSES[classSelect.value] ? C_CLASSES[classSelect.value].notes : '';
        });

        wrapper.appendChild(section);
    }

    function init() {
        // Restore settings
        currentClass = meridian.settings.euUasClass || 'C0';
        currentCategory = meridian.settings.euOpCategory || 'open_a1';

        // Inject validator hook after a tick (so PlanValidator is initialized)
        setTimeout(injectIntoValidator, 100);

        // Inject badge supplement
        injectBadge();

        // Re-validate on mission change
        meridian.events.on('mission_change', function () { validate(); });
    }

    return {
        init,
        validate,
        getViolations,
        applyClass,
        applyCategory,
        buildSettingsSection,
        getBadgeSupplement,
        C_CLASSES,
        OP_CATEGORIES,
    };

})();
