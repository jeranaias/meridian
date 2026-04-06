/* ============================================================
   vessel-tracker.js — AIS Vessel Tracker with Moving Home Point
   T3-4: Connects to AIS WebSocket, parses NMEA AIS messages,
   displays vessels on Leaflet map, tracks own vessel MMSI,
   updates home position when own vessel moves >5m.
   ============================================================ */

'use strict';

window.VesselTracker = (function () {

    // --- State ---
    let ws = null;
    let wsUrl = '';
    let ownMmsi = '';
    let vessels = {};         // mmsi -> { mmsi, lat, lon, heading, speed, lastSeen, marker }
    let ownVesselPos = null;  // { lat, lon }
    let hudEl = null;
    let reconnectTimer = null;
    const HOME_UPDATE_THRESHOLD_M = 5;
    let homeUpdateTimer = null;

    // --- Vessel SVG icon (ship shape) ---
    function makeVesselIcon(heading, isOwn) {
        const color = isOwn ? '#00e5ff' : '#fbbf24';
        const rot = (heading || 0);
        return L.divIcon({
            className: 'vessel-icon',
            html: `<svg viewBox="0 0 24 32" xmlns="http://www.w3.org/2000/svg"
                        style="transform:rotate(${rot}deg);display:block">
                <polygon points="12,2 4,28 12,22 20,28"
                    fill="${color}" fill-opacity="0.85"
                    stroke="${isOwn ? '#0891b2' : '#d97706'}" stroke-width="1.2"/>
            </svg>`,
            iconSize: [24, 32],
            iconAnchor: [12, 16],
        });
    }

    // --- AIS NMEA Parser (simplified VDM/VDO payload) ---
    // Decodes !AIVDM / !AIVDO Type 1/2/3 (Class A position report)
    function decodeBitfield(payload, start, len) {
        let val = 0;
        for (let i = 0; i < len; i++) {
            val = (val << 1) | getBit(payload, start + i);
        }
        return val;
    }

    function getBit(bits, pos) {
        const byteIdx = Math.floor(pos / 6);
        const bitIdx = 5 - (pos % 6);
        return (bits[byteIdx] >> bitIdx) & 1;
    }

    function armourToPayload(armour) {
        const bits = [];
        for (let i = 0; i < armour.length; i++) {
            let c = armour.charCodeAt(i) - 48;
            if (c > 40) c -= 8;
            bits.push(c);
        }
        return bits;
    }

    function parseAIS(line) {
        // Accept !AIVDM and !AIVDO lines
        if (!line || (!line.startsWith('!AIVDM') && !line.startsWith('!AIVDO'))) return null;

        const parts = line.split(',');
        if (parts.length < 6) return null;

        // Only handle single-fragment messages for simplicity
        const fragCount = parseInt(parts[1]);
        if (fragCount !== 1) return null;

        const payloadStr = parts[5];
        if (!payloadStr) return null;

        const payload = armourToPayload(payloadStr);
        const msgType = decodeBitfield(payload, 0, 6);

        // Type 1, 2, 3: Class A Position Report
        if (msgType >= 1 && msgType <= 3) {
            const mmsi = decodeBitfield(payload, 8, 30).toString();

            // Longitude: I28, degrees * 1/10000 min
            let lon = decodeBitfield(payload, 61, 28);
            if (lon >= (1 << 27)) lon -= (1 << 28);
            lon = lon / 600000.0;

            // Latitude: I27, degrees * 1/10000 min
            let lat = decodeBitfield(payload, 89, 27);
            if (lat >= (1 << 26)) lat -= (1 << 27);
            lat = lat / 600000.0;

            // Speed over ground: U10, 1/10 knot
            const sogRaw = decodeBitfield(payload, 50, 10);
            const sog = sogRaw / 10.0; // knots

            // Course over ground: U12, 1/10 degree
            const cogRaw = decodeBitfield(payload, 116, 12);
            const cog = cogRaw / 10.0;

            // True heading: U9, degrees
            const heading = decodeBitfield(payload, 128, 9);

            // Sanity check: skip invalid positions
            if (Math.abs(lat) < 0.001 && Math.abs(lon) < 0.001) return null;
            if (Math.abs(lat) > 90 || Math.abs(lon) > 180) return null;

            return { mmsi, lat, lon, heading: heading < 360 ? heading : cog, speed: sog };
        }

        // Type 18: Class B Position Report
        if (msgType === 18) {
            const mmsi = decodeBitfield(payload, 8, 30).toString();

            let lon = decodeBitfield(payload, 57, 28);
            if (lon >= (1 << 27)) lon -= (1 << 28);
            lon = lon / 600000.0;

            let lat = decodeBitfield(payload, 85, 27);
            if (lat >= (1 << 26)) lat -= (1 << 27);
            lat = lat / 600000.0;

            const sogRaw = decodeBitfield(payload, 46, 10);
            const sog = sogRaw / 10.0;
            const cogRaw = decodeBitfield(payload, 112, 12);
            const cog = cogRaw / 10.0;

            if (Math.abs(lat) < 0.001 && Math.abs(lon) < 0.001) return null;
            if (Math.abs(lat) > 90 || Math.abs(lon) > 180) return null;

            return { mmsi, lat, lon, heading: cog, speed: sog };
        }

        return null;
    }

    // --- Haversine distance ---
    function haversine(lat1, lon1, lat2, lon2) {
        const R = 6371000;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(dLat / 2) ** 2 +
                  Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                  Math.sin(dLon / 2) ** 2;
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    // --- Bearing from p1 to p2 ---
    function bearing(lat1, lon1, lat2, lon2) {
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const lat1r = lat1 * Math.PI / 180;
        const lat2r = lat2 * Math.PI / 180;
        const y = Math.sin(dLon) * Math.cos(lat2r);
        const x = Math.cos(lat1r) * Math.sin(lat2r) - Math.sin(lat1r) * Math.cos(lat2r) * Math.cos(dLon);
        return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
    }

    // --- Map access ---
    function getMap() {
        return window.FlyMap ? FlyMap.getMap() : null;
    }

    // --- Update or create vessel marker ---
    function updateVesselMarker(mmsi, data) {
        const map = getMap();
        if (!map) return;

        const isOwn = (ownMmsi && mmsi === ownMmsi);

        if (vessels[mmsi] && vessels[mmsi].marker) {
            vessels[mmsi].marker.setLatLng([data.lat, data.lon]);
            vessels[mmsi].marker.setIcon(makeVesselIcon(data.heading, isOwn));
        } else {
            const marker = L.marker([data.lat, data.lon], {
                icon: makeVesselIcon(data.heading, isOwn),
                zIndexOffset: isOwn ? 900 : 500,
                title: 'MMSI: ' + mmsi + (isOwn ? ' (own vessel)' : ''),
            });

            marker.bindTooltip(
                '<strong>' + (isOwn ? 'OWN VESSEL' : 'VESSEL') + '</strong><br>' +
                'MMSI: ' + mmsi + '<br>' +
                'SOG: ' + data.speed.toFixed(1) + ' kts',
                { permanent: false, direction: 'top', className: 'vessel-tooltip' }
            );

            marker.addTo(map);
            if (!vessels[mmsi]) vessels[mmsi] = {};
            vessels[mmsi].marker = marker;
        }

        // Update tooltip content dynamically
        if (vessels[mmsi].marker) {
            vessels[mmsi].marker.setTooltipContent(
                '<strong>' + (isOwn ? 'OWN VESSEL' : 'VESSEL') + '</strong><br>' +
                'MMSI: ' + mmsi + '<br>' +
                'HDG: ' + Math.round(data.heading) + '° &nbsp; SOG: ' + data.speed.toFixed(1) + ' kts'
            );
        }

        Object.assign(vessels[mmsi], { lat: data.lat, lon: data.lon, heading: data.heading, speed: data.speed, lastSeen: Date.now() });
    }

    // --- Handle incoming own-vessel movement, update home ---
    function handleOwnVesselUpdate(mmsi, data) {
        if (!ownMmsi || mmsi !== ownMmsi) return;

        const newPos = { lat: data.lat, lon: data.lon };

        if (ownVesselPos) {
            const moved = haversine(ownVesselPos.lat, ownVesselPos.lon, newPos.lat, newPos.lon);
            if (moved < HOME_UPDATE_THRESHOLD_M) return;
        }

        ownVesselPos = newPos;

        // Send MAV_CMD_DO_SET_HOME (179) via MAVLink
        sendSetHome(newPos.lat, newPos.lon);
    }

    // --- Send MAV_CMD_DO_SET_HOME ---
    function sendSetHome(lat, lon) {
        if (!window.MAVLink || !window.Connection) return;
        // MAV_CMD_DO_SET_HOME = 179
        // param1=1 means use current location, param1=0 means use provided lat/lon
        const frame = MAVLink.encodeCommandLong(
            179,       // MAV_CMD_DO_SET_HOME
            0,         // param1: 0 = use provided lat/lon
            0, 0, 0, 0, // param 2-5 unused
            lat,       // param6: lat
            lon        // param7: lon (alt=0 keeps existing home alt)
        );
        if (frame) {
            Connection.send(frame);
            const v = meridian.v;
            if (v) {
                v.homeLat = lat;
                v.homeLon = lon;
                if (window.FlyMap && FlyMap.updateHome) FlyMap.updateHome(lat, lon);
            }
            meridian.log('Home updated to vessel position (' + lat.toFixed(5) + ', ' + lon.toFixed(5) + ')', 'info');
        }
    }

    // --- HUD widget ---
    function createHud() {
        if (hudEl) return;
        hudEl = document.createElement('div');
        hudEl.id = 'vessel-hud';
        hudEl.className = 'vessel-hud';
        hudEl.style.cssText = [
            'position:absolute',
            'bottom:140px',
            'left:12px',
            'z-index:1000',
            'background:rgba(8,11,16,0.82)',
            'border:1px solid rgba(0,229,255,0.25)',
            'border-radius:4px',
            'padding:6px 10px',
            'font-family:var(--font-mono,"DM Mono",monospace)',
            'font-size:11px',
            'color:#94a3b8',
            'min-width:160px',
            'pointer-events:none',
            'display:none',
        ].join(';');
        hudEl.innerHTML = '<div class="vessel-hud-title" style="color:var(--c-primary);margin-bottom:4px;font-size:10px;letter-spacing:.08em">VESSEL</div>' +
                          '<div id="vessel-hud-body">--</div>';
        const mapArea = document.getElementById('map-area');
        if (mapArea) mapArea.appendChild(hudEl);
    }

    function updateHud() {
        if (!hudEl) return;
        const v = meridian.v;
        if (!ownVesselPos || !v || !v.lat) {
            hudEl.style.display = 'none';
            return;
        }

        const dist = haversine(v.lat, v.lon, ownVesselPos.lat, ownVesselPos.lon);
        const brg = bearing(v.lat, v.lon, ownVesselPos.lat, ownVesselPos.lon);
        const ownData = vessels[ownMmsi];

        hudEl.style.display = 'block';
        const body = document.getElementById('vessel-hud-body');
        if (body) {
            body.innerHTML =
                `<div>BRG <span style="color:var(--c-text)">${Math.round(brg)}\u00B0</span></div>` +
                `<div>DIST <span style="color:var(--c-text)">${dist < 1000 ? Math.round(dist) + 'm' : (dist / 1000).toFixed(2) + 'km'}</span></div>` +
                (ownData ? `<div>SOG <span style="color:var(--c-text)">${ownData.speed.toFixed(1)} kts</span></div>` : '');
        }
    }

    // --- WebSocket connection ---
    function connect(url) {
        if (ws) {
            ws.onclose = null;
            ws.close();
            ws = null;
        }
        if (!url) return;

        wsUrl = url;
        meridian.log('AIS: connecting to ' + url, 'info');

        try {
            ws = new WebSocket(url);
        } catch (e) {
            meridian.log('AIS: connection failed — ' + e.message, 'error');
            scheduleReconnect();
            return;
        }

        ws.onopen = function () {
            meridian.log('AIS: connected', 'info');
        };

        ws.onmessage = function (evt) {
            const lines = (evt.data || '').split('\n');
            for (const line of lines) {
                const trimmed = line.trim();
                if (!trimmed) continue;
                const parsed = parseAIS(trimmed);
                if (parsed) {
                    updateVesselMarker(parsed.mmsi, parsed);
                    handleOwnVesselUpdate(parsed.mmsi, parsed);
                }
            }
        };

        ws.onerror = function () {
            meridian.log('AIS: connection error', 'warn');
        };

        ws.onclose = function () {
            meridian.log('AIS: disconnected', 'warn');
            ws = null;
            scheduleReconnect();
        };
    }

    function scheduleReconnect() {
        if (reconnectTimer) return;
        reconnectTimer = setTimeout(function () {
            reconnectTimer = null;
            if (wsUrl && meridian.settings.aisServer) connect(wsUrl);
        }, 5000);
    }

    function disconnect() {
        if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
        if (ws) { ws.onclose = null; ws.close(); ws = null; }
        wsUrl = '';
    }

    // --- Stale vessel cleanup (remove vessels not seen for 120s) ---
    setInterval(function () {
        const now = Date.now();
        for (const mmsi of Object.keys(vessels)) {
            const v = vessels[mmsi];
            if (now - v.lastSeen > 120000 && v.marker) {
                const map = getMap();
                if (map) map.removeLayer(v.marker);
                delete vessels[mmsi];
            }
        }
    }, 30000);

    // --- HUD update every 5s ---
    setInterval(updateHud, 5000);

    // --- Settings change handler ---
    meridian.events.on('settings_change', function (data) {
        if (data.key === 'aisServer') {
            disconnect();
            if (data.value) connect(data.value);
        }
        if (data.key === 'ownMmsi') {
            ownMmsi = data.value || '';
            ownVesselPos = null;
            // Re-render all markers to update color
            const map = getMap();
            if (map) {
                for (const mmsi of Object.keys(vessels)) {
                    const v = vessels[mmsi];
                    if (v.marker) {
                        v.marker.setIcon(makeVesselIcon(v.heading, mmsi === ownMmsi));
                    }
                }
            }
        }
    });

    function init() {
        createHud();
        ownMmsi = (meridian.settings.ownMmsi || '').toString();

        const url = meridian.settings.aisServer || '';
        if (url) connect(url);
    }

    return {
        init,
        connect,
        disconnect,
        getVessels: function () { return vessels; },
    };

})();
