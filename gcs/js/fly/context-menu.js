/* ============================================================
   context-menu.js — Map right-click context menu
   Oborne addition: fly here, camera, gimbal.
   ============================================================ */

'use strict';

window.ContextMenu = (function () {

    let menuEl;
    let clickLat = 0, clickLon = 0;

    const items = [
        { label: 'Fly Here', icon: '\u2708', action: 'fly_here' },
        { label: 'Set Home Here', icon: '\u2302', action: 'set_home' },
        { sep: true },
        { label: 'Add Waypoint', icon: '+', action: 'add_wp' },
        { label: 'Measure Distance', icon: '\u21D4', action: 'measure' },
        { sep: true },
        { label: 'Point Camera Here', icon: '\u25CE', action: 'camera_roi' },
        { label: 'Trigger Camera Now', icon: '\u25C9', action: 'camera_trigger' },
        { label: 'Set ROI', icon: '\u29BF', action: 'set_roi' },
    ];

    function init() {
        menuEl = document.getElementById('context-menu');
        if (!menuEl) return;

        menuEl.innerHTML = items.map(item => {
            if (item.sep) return '<div class="context-sep"></div>';
            return `<div class="context-item" data-action="${item.action}">
                <span class="ctx-icon">${item.icon}</span>
                <span>${item.label}</span>
            </div>`;
        }).join('');

        // Handle clicks
        menuEl.querySelectorAll('.context-item').forEach(el => {
            el.addEventListener('click', () => {
                const action = el.dataset.action;
                handleAction(action);
                hide();
            });
        });

        // Close on click outside
        document.addEventListener('click', e => {
            if (!menuEl.contains(e.target)) hide();
        });

        // Close on Escape
        document.addEventListener('keydown', e => {
            if (e.key === 'Escape') hide();
        });
    }

    function show(x, y, lat, lon) {
        if (!menuEl) return;
        clickLat = lat;
        clickLon = lon;

        // Position menu
        menuEl.style.left = x + 'px';
        menuEl.style.top = y + 'px';
        menuEl.classList.add('visible');

        // Ensure menu stays within viewport
        const rect = menuEl.getBoundingClientRect();
        if (rect.right > window.innerWidth) {
            menuEl.style.left = (x - rect.width) + 'px';
        }
        if (rect.bottom > window.innerHeight) {
            menuEl.style.top = (y - rect.height) + 'px';
        }
    }

    function hide() {
        if (menuEl) menuEl.classList.remove('visible');
    }

    // T2-6: Check guided distance limit. Returns true if the goto should proceed.
    function checkGuidedDistLimit(targetLat, targetLon) {
        const v = meridian.v;
        if (!v) return false;

        // Block if no home set — we can't validate distance without a reference
        if (v.homeLat === null || v.homeLon === null) {
            meridian.log('Cannot fly: no home position set', 'warn');
            Modal.confirm('No Home Position',
                'Home position is not set. Cannot validate distance limit. Set home first.',
                'OK');
            return false;
        }

        const maxDist = (meridian.settings && meridian.settings.guidedDistMax) || 1000;

        // Compute distance from home to target (haversine)
        function haversine(lat1, lon1, lat2, lon2) {
            const R = 6371000;
            const dLat = (lat2 - lat1) * Math.PI / 180;
            const dLon = (lon2 - lon1) * Math.PI / 180;
            const a = Math.sin(dLat / 2) ** 2 +
                      Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                      Math.sin(dLon / 2) ** 2;
            return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        }

        const dist = Math.round(haversine(v.homeLat, v.homeLon, targetLat, targetLon));

        if (dist > maxDist) {
            meridian.log('Guided target exceeds distance limit: ' + dist + 'm (max ' + maxDist + 'm)', 'warn');
            Modal.confirm('Distance Limit Exceeded',
                'Target is ' + dist + 'm from home. Maximum guided distance is ' + maxDist + 'm.\n' +
                'Adjust the limit in Settings → Guided Dist Max to proceed.',
                'OK');
            return false;
        }

        return true;
    }

    function handleAction(action) {
        const v = meridian.v;
        switch (action) {
            case 'fly_here':
                if (!v || !v.armed) {
                    meridian.log('Cannot fly here: not armed', 'warn');
                    return;
                }
                // T2-6: Enforce guided distance limit
                if (!checkGuidedDistLimit(clickLat, clickLon)) return;
                const alt = v.relativeAlt > 0 ? v.relativeAlt : 10;
                Connection.sendSetMode('GUIDED');
                // Wait for mode ACK before sending goto — T0-12: prevent dual-fire
                let gotoSent = false;
                const sendGoto = () => {
                    if (gotoSent) return;
                    gotoSent = true;
                    meridian.events.off('command_ack', onAck);
                    Connection.sendGoto(clickLat, clickLon, alt);
                };
                const onAck = () => sendGoto();
                meridian.events.on('command_ack', onAck);
                setTimeout(sendGoto, 2000); // fallback if ACK not received within 2s
                meridian.log('Flying to ' + clickLat.toFixed(6) + ', ' + clickLon.toFixed(6), 'info');
                break;

            case 'set_home':
                if (v) {
                    v.homeLat = clickLat;
                    v.homeLon = clickLon;
                    // T0-8: Also send MAVLink SET_HOME_POSITION to FC
                    if (Connection.protocol === 'mavlink' && MAVLink.encodeCommandLong) {
                        // MAV_CMD_DO_SET_HOME (179), param1=0 (use specified location)
                        // param5=lat, param6=lon, param7=alt
                        Connection.send(MAVLink.encodeCommandLong(1, 1, 179, 0, 0, 0, 0,
                            clickLat, clickLon, v.homeAlt || 0));
                    }
                    meridian.events.emit('home_changed', v);
                    meridian.log('Home set to ' + clickLat.toFixed(6) + ', ' + clickLon.toFixed(6), 'info');
                }
                break;

            case 'add_wp':
                meridian.events.emit('add_waypoint', { lat: clickLat, lon: clickLon });
                break;

            case 'measure':
                meridian.events.emit('measure_start', { lat: clickLat, lon: clickLon });
                break;

            case 'camera_roi': {
                // T1-4: Send DO_SET_ROI to point camera/gimbal at this location
                const roiAlt = v ? v.relativeAlt || 0 : 0;
                if (Connection.protocol === 'mavlink') {
                    Connection.send(MAVLink.encodeDoSetRoi(clickLat, clickLon, roiAlt));
                    Connection.trackCommand(201, 'DO_SET_ROI');
                }
                meridian.log('Camera ROI: ' + clickLat.toFixed(6) + ', ' + clickLon.toFixed(6), 'info');
                break;
            }

            case 'camera_trigger':
                // T1-4: Send IMAGE_START_CAPTURE to trigger camera shutter
                if (Connection.protocol === 'mavlink') {
                    Connection.send(MAVLink.encodeImageStartCapture());
                    Connection.trackCommand(2000, 'IMAGE_START_CAPTURE');
                }
                meridian.log('Camera triggered', 'info');
                break;

            case 'set_roi': {
                // T1-4: Same as camera_roi — send DO_SET_ROI
                const sroiAlt = v ? v.relativeAlt || 0 : 0;
                if (Connection.protocol === 'mavlink') {
                    Connection.send(MAVLink.encodeDoSetRoi(clickLat, clickLon, sroiAlt));
                    Connection.trackCommand(201, 'DO_SET_ROI');
                }
                meridian.log('ROI set: ' + clickLat.toFixed(6) + ', ' + clickLon.toFixed(6), 'info');
                break;
            }
        }
    }

    return { init, show, hide };

})();
