/* ============================================================
   ros-bridge.js — ROS2 / rosbridge_server WebSocket adapter
   T3-15: Connects to a rosbridge_server endpoint, subscribes
   to configurable ROS2 topics, and maps them to Meridian
   vehicle state. Also publishes to ROS topics on commands.
   ============================================================ */

'use strict';

window.RosBridge = (function () {

    const DEFAULT_URL = 'ws://localhost:9090';

    // Default topic mappings: ROS topic → Meridian state field
    const DEFAULT_TOPIC_MAP = {
        '/mavros/state':                  'heartbeat',
        '/mavros/global_position/global': 'position',
        '/mavros/imu/data':               'attitude',
        '/mavros/battery':                'battery',
    };

    // ROS message type for each topic
    const TOPIC_TYPES = {
        '/mavros/state':                  'mavros_msgs/State',
        '/mavros/global_position/global': 'sensor_msgs/NavSatFix',
        '/mavros/imu/data':               'sensor_msgs/Imu',
        '/mavros/battery':                'sensor_msgs/BatteryState',
    };

    // Publish topics for outbound commands
    const PUBLISH_TOPICS = {
        setpoint_position: '/mavros/setpoint_position/global',
        set_mode:          '/mavros/set_mode',
        arming:            '/mavros/cmd/arming',
    };

    let ws = null;
    let connected = false;
    let rosUrl = DEFAULT_URL;
    let topicMap = Object.assign({}, DEFAULT_TOPIC_MAP);
    let _reconnectTimer = null;
    let _advertised = {};

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    function connect(url) {
        if (ws) disconnect();
        rosUrl = url || meridian.settings.rosBridgeUrl || DEFAULT_URL;
        _openSocket();
    }

    function disconnect() {
        if (_reconnectTimer) { clearTimeout(_reconnectTimer); _reconnectTimer = null; }
        if (ws) {
            try { ws.close(); } catch (e) { /* ignore */ }
            ws = null;
        }
        _setConnected(false);
    }

    function isConnected() { return connected; }

    function getUrl() { return rosUrl; }

    function getTopicMap() { return Object.assign({}, topicMap); }

    function setTopicMap(map) {
        // Unsubscribe old topics, subscribe new
        if (connected) {
            Object.keys(topicMap).forEach(function (topic) { _unsubscribe(topic); });
        }
        topicMap = Object.assign({}, map);
        if (connected) {
            Object.keys(topicMap).forEach(function (topic) { _subscribe(topic); });
        }
    }

    // Publish a command to a ROS topic
    function publish(topicKey, msg) {
        var topic = PUBLISH_TOPICS[topicKey];
        if (!topic) { meridian.log('ROS: unknown publish key: ' + topicKey, 'warn'); return; }
        _send({ op: 'publish', topic: topic, msg: msg });
    }

    // -------------------------------------------------------------------------
    // Socket management
    // -------------------------------------------------------------------------

    function _openSocket() {
        try {
            ws = new WebSocket(rosUrl);
        } catch (e) {
            meridian.log('ROS bridge: failed to open socket — ' + e.message, 'error');
            return;
        }

        ws.binaryType = 'arraybuffer';

        ws.onopen = function () {
            meridian.log('ROS bridge connected to ' + rosUrl, 'info');
            _setConnected(true);
            _subscribeAll();
        };

        ws.onmessage = function (event) {
            var data;
            try { data = JSON.parse(event.data); } catch (e) { return; }
            _handleMessage(data);
        };

        ws.onerror = function (e) {
            meridian.log('ROS bridge error', 'warn');
        };

        ws.onclose = function () {
            _setConnected(false);
            ws = null;
            var delay = meridian.settings.reconnectDelay || 0;
            if (delay > 0) {
                _reconnectTimer = setTimeout(function () { _openSocket(); }, delay);
            }
        };
    }

    function _setConnected(val) {
        connected = val;
        _updateIndicator();
        meridian.events.emit('ros_connection', { connected: val, url: rosUrl });
    }

    // -------------------------------------------------------------------------
    // rosbridge protocol helpers
    // -------------------------------------------------------------------------

    function _send(obj) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify(obj));
        }
    }

    function _subscribe(topic) {
        var type = TOPIC_TYPES[topic] || 'std_msgs/String';
        _send({
            op: 'subscribe',
            topic: topic,
            type: type,
        });
    }

    function _unsubscribe(topic) {
        _send({ op: 'unsubscribe', topic: topic });
    }

    function _subscribeAll() {
        Object.keys(topicMap).forEach(function (topic) { _subscribe(topic); });
    }

    // Advertise an outbound topic (called lazily on first publish)
    function _advertise(topic, type) {
        if (_advertised[topic]) return;
        _advertised[topic] = true;
        _send({ op: 'advertise', topic: topic, type: type });
    }

    // -------------------------------------------------------------------------
    // Message router — maps ROS messages to Meridian vehicle state
    // -------------------------------------------------------------------------

    function _handleMessage(data) {
        if (data.op !== 'publish') return;

        var topic = data.topic;
        var msg = data.msg;
        if (!msg) return;

        var stateField = topicMap[topic];
        if (!stateField) return;

        var v = meridian.v;
        if (!v) return;

        switch (stateField) {
            case 'heartbeat':
                _applyHeartbeat(v, msg);
                break;
            case 'position':
                _applyPosition(v, msg);
                break;
            case 'attitude':
                _applyAttitude(v, msg);
                break;
            case 'battery':
                _applyBattery(v, msg);
                break;
            default:
                meridian.log('ROS bridge: unhandled state field ' + stateField, 'warn');
        }
    }

    // /mavros/state → mavros_msgs/State
    //   { connected, armed, guided, manual_input, mode, system_status }
    function _applyHeartbeat(v, msg) {
        if (msg.armed !== undefined) v.armed = !!msg.armed;
        if (msg.mode !== undefined) {
            v.mode = msg.mode;
            // Map MAVROS mode string to Meridian mode number if possible
            var modeNum = _modeStringToNumber(msg.mode);
            if (modeNum >= 0) v.modeNum = modeNum;
        }
        meridian.events.emit('heartbeat', { sysid: v.sysid });
    }

    // /mavros/global_position/global → sensor_msgs/NavSatFix
    //   { latitude, longitude, altitude, status: { status, service } }
    function _applyPosition(v, msg) {
        if (msg.latitude !== undefined)  v.lat = msg.latitude;
        if (msg.longitude !== undefined) v.lon = msg.longitude;
        if (msg.altitude !== undefined)  v.alt = msg.altitude;
        // GPS fix: status 0 = no fix, ≥0 = fix
        if (msg.status && msg.status.status !== undefined) {
            v.gpsFix = msg.status.status >= 0 ? 3 : 0;
        }
        meridian.events.emit('position', { sysid: v.sysid, lat: v.lat, lon: v.lon, alt: v.alt });
    }

    // /mavros/imu/data → sensor_msgs/Imu
    //   orientation: { x, y, z, w } (quaternion)
    function _applyAttitude(v, msg) {
        if (msg.orientation) {
            var euler = _quatToEuler(msg.orientation.x, msg.orientation.y,
                                     msg.orientation.z, msg.orientation.w);
            v.roll    = euler.roll  * (180 / Math.PI);
            v.pitch   = euler.pitch * (180 / Math.PI);
            v.yaw     = euler.yaw   * (180 / Math.PI);
        }
        meridian.events.emit('attitude', { sysid: v.sysid });
    }

    // /mavros/battery → sensor_msgs/BatteryState
    //   { voltage, current, percentage }
    function _applyBattery(v, msg) {
        if (msg.voltage !== undefined)    v.voltage    = msg.voltage;
        if (msg.current !== undefined)    v.current    = msg.current;
        if (msg.percentage !== undefined) v.battPct    = Math.round(msg.percentage * 100);
        meridian.events.emit('battery', { sysid: v.sysid });
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    // Quaternion → Euler (roll, pitch, yaw) in radians
    function _quatToEuler(x, y, z, w) {
        var sinr = 2 * (w * x + y * z);
        var cosr = 1 - 2 * (x * x + y * y);
        var roll = Math.atan2(sinr, cosr);

        var sinp = 2 * (w * y - z * x);
        var pitch;
        if (Math.abs(sinp) >= 1) {
            pitch = (Math.PI / 2) * (sinp < 0 ? -1 : 1);
        } else {
            pitch = Math.asin(sinp);
        }

        var siny = 2 * (w * z + x * y);
        var cosy = 1 - 2 * (y * y + z * z);
        var yaw = Math.atan2(siny, cosy);

        return { roll: roll, pitch: pitch, yaw: yaw };
    }

    // Map MAVROS mode string to ArduPilot mode number (best effort)
    function _modeStringToNumber(modeStr) {
        var modeMap = {
            'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3,
            'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'CIRCLE': 7,
            'LAND': 9, 'DRIFT': 11, 'SPORT': 13, 'FLIP': 14,
            'AUTOTUNE': 15, 'POSHOLD': 16, 'BRAKE': 17,
        };
        var upper = (modeStr || '').toUpperCase().replace('COPTER.', '');
        return modeMap[upper] !== undefined ? modeMap[upper] : -1;
    }

    // -------------------------------------------------------------------------
    // Connection indicator
    // -------------------------------------------------------------------------

    function _updateIndicator() {
        var indicator = document.querySelector('.conn-indicator');
        var text = indicator ? indicator.querySelector('.conn-text') : null;
        var dot  = indicator ? indicator.querySelector('.conn-dot')  : null;

        if (!indicator) return;

        if (connected) {
            indicator.className = 'conn-indicator ros-connected';
            if (text) text.textContent = 'ROS';
            if (dot)  dot.style.background = 'var(--c-info, #0891b2)';
        }
        // If not connected and MAVLink is also down, the normal connection logic
        // will handle the indicator state; we only override when ROS is active.
    }

    // -------------------------------------------------------------------------
    // Settings panel section (called from settings.js)
    // -------------------------------------------------------------------------

    function renderSettingsSection(wrapper, createSectionHeader, createTextField, createToggle, createNumberField) {
        wrapper.appendChild(createSectionHeader('ROS2 Bridge'));

        // ROS bridge URL
        wrapper.appendChild(createTextField(
            'rosbridge URL',
            meridian.settings.rosBridgeUrl || DEFAULT_URL,
            function (val) {
                meridian.settings.rosBridgeUrl = val;
                meridian.saveSettings();
            }
        ));

        // Connect / Disconnect toggle
        var connectRow = document.createElement('div');
        connectRow.className = 'settings-field';
        var connectBtn = document.createElement('button');
        connectBtn.className = 'offline-btn draw';
        connectBtn.textContent = connected ? 'Disconnect ROS' : 'Connect ROS';
        connectBtn.style.marginTop = '4px';
        connectBtn.addEventListener('click', function () {
            if (connected) {
                disconnect();
                connectBtn.textContent = 'Connect ROS';
            } else {
                connect(meridian.settings.rosBridgeUrl || DEFAULT_URL);
                connectBtn.textContent = 'Disconnect ROS';
            }
        });
        connectRow.appendChild(connectBtn);
        wrapper.appendChild(connectRow);

        // Topic mapping editor
        wrapper.appendChild(_buildTopicMappingEditor());
    }

    function _buildTopicMappingEditor() {
        var section = document.createElement('div');
        section.className = 'settings-subsection';
        var header = document.createElement('div');
        header.className = 'settings-field-label';
        header.style.marginTop = '8px';
        header.textContent = 'Topic Mappings';
        section.appendChild(header);

        var defaultTopics = Object.keys(DEFAULT_TOPIC_MAP);
        defaultTopics.forEach(function (topic) {
            var row = document.createElement('div');
            row.className = 'settings-field';
            row.style.flexDirection = 'column';
            row.style.gap = '2px';

            var topicLabel = document.createElement('div');
            topicLabel.style.fontSize = 'var(--ts-xs, 10px)';
            topicLabel.style.color = 'var(--c-text-dim)';
            topicLabel.textContent = topic;

            var input = document.createElement('input');
            input.type = 'text';
            input.className = 'settings-input wide';
            input.value = topicMap[topic] || DEFAULT_TOPIC_MAP[topic];
            input.placeholder = DEFAULT_TOPIC_MAP[topic];
            input.addEventListener('change', function () {
                var newMap = Object.assign({}, topicMap);
                newMap[topic] = input.value || DEFAULT_TOPIC_MAP[topic];
                setTopicMap(newMap);
                // Persist custom map
                var saved = meridian.settings.rosTopicMap || {};
                saved[topic] = newMap[topic];
                meridian.settings.rosTopicMap = saved;
                meridian.saveSettings();
            });

            row.appendChild(topicLabel);
            row.appendChild(input);
            section.appendChild(row);
        });

        return section;
    }

    // -------------------------------------------------------------------------
    // Init — restore persisted settings
    // -------------------------------------------------------------------------

    (function _init() {
        meridian.events.on('settings_change', function (evt) {
            if (evt.key === 'rosTopicMap' && evt.value) {
                setTopicMap(Object.assign({}, DEFAULT_TOPIC_MAP, evt.value));
            }
        });

        // Restore topic map from settings if persisted
        var savedMap = meridian.settings && meridian.settings.rosTopicMap;
        if (savedMap) {
            topicMap = Object.assign({}, DEFAULT_TOPIC_MAP, savedMap);
        }
    }());

    return {
        connect,
        disconnect,
        isConnected,
        getUrl,
        getTopicMap,
        setTopicMap,
        publish,
        renderSettingsSection,
        DEFAULT_TOPIC_MAP,
        DEFAULT_URL,
    };

})();
