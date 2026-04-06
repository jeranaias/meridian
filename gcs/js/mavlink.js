/* ============================================================
   mavlink.js — MAVLink v2 Decoder / Encoder for SITL
   Supports the core telemetry + command messages needed
   to fly ArduPilot SITL from the Meridian GCS.
   ============================================================ */

'use strict';

window.MAVLink = (function () {

    const MAVLINK_STX = 0xFD; // MAVLink v2 start byte

    // Message IDs
    const MSG_ID = {
        HEARTBEAT:          0,
        SYS_STATUS:         1,
        GPS_RAW_INT:        24,
        ATTITUDE:           30,
        GLOBAL_POSITION_INT: 33,
        RC_CHANNELS:        65,
        VFR_HUD:            74,
        COMMAND_LONG:        76,
        COMMAND_ACK:         77,
        PARAM_REQUEST_LIST: 21,
        PARAM_REQUEST_READ: 20,
        PARAM_VALUE:        22,
        PARAM_SET:          23,
        MISSION_REQUEST_LIST: 43,
        MISSION_COUNT:      44,
        MISSION_ITEM_INT:   73,
        MISSION_REQUEST_INT: 51,
        MISSION_ACK:        47,
        BATTERY_STATUS:     147,
        AUTOPILOT_VERSION:  148,
        FENCE_STATUS:       162,
        VIBRATION:          241,
        HOME_POSITION:      242,
        ADSB_VEHICLE:       246,
        STATUSTEXT:         253,
        OPEN_DRONE_ID_BASIC_ID:  12900,
        OPEN_DRONE_ID_LOCATION:  12901,
        OPEN_DRONE_ID_SYSTEM:    12904,
    };

    // MAV_CMD values
    const MAV_CMD = {
        COMPONENT_ARM_DISARM: 400,
        NAV_TAKEOFF:          22,
        NAV_LAND:             21,
        NAV_RETURN_TO_LAUNCH: 20,
        DO_SET_MODE:          176,
        NAV_WAYPOINT:         16,
        DO_SET_ROI:           201,  // T1-4: Camera ROI
        DO_MOUNT_CONTROL:     205,  // T1-4: Gimbal control
        IMAGE_START_CAPTURE:  2000, // T1-4: Trigger camera
    };

    // ArduCopter flight mode numbers — full table (matches state.js)
    const COPTER_MODES = {
        'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3,
        'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'CIRCLE': 7,
        'LAND': 9, 'DRIFT': 11, 'SPORT': 13, 'FLIP': 14,
        'AUTOTUNE': 15, 'POSHOLD': 16, 'BRAKE': 17, 'THROW': 18,
        'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21,
        'FLOWHOLD': 22, 'FOLLOW': 23, 'ZIGZAG': 24, 'SYSTEMID': 25,
        'AUTOROTATE': 26, 'AUTO_RTL': 27,
    };

    const COPTER_MODE_NAMES = {};
    for (const [k, v] of Object.entries(COPTER_MODES)) COPTER_MODE_NAMES[v] = k;

    // CRC-X.25 seed extras per message ID
    const CRC_EXTRA = {
        0:  50,  // HEARTBEAT
        1:  124, // SYS_STATUS
        20: 214, // PARAM_REQUEST_READ
        21: 159, // PARAM_REQUEST_LIST
        22: 220, // PARAM_VALUE
        23: 168, // PARAM_SET
        24: 24,  // GPS_RAW_INT
        30: 39,  // ATTITUDE
        33: 104, // GLOBAL_POSITION_INT
        43: 132, // MISSION_REQUEST_LIST
        44: 221, // MISSION_COUNT
        47: 153, // MISSION_ACK
        51: 196, // MISSION_REQUEST_INT
        65: 118, // RC_CHANNELS
        73: 38,  // MISSION_ITEM_INT
        74: 20,  // VFR_HUD
        76: 152, // COMMAND_LONG
        77: 143, // COMMAND_ACK
        147: 154, // BATTERY_STATUS
        148: 178, // AUTOPILOT_VERSION
        162: 189, // FENCE_STATUS
        241: 90,  // VIBRATION
        242: 104, // HOME_POSITION
        246: 184, // ADSB_VEHICLE
        160: 78,  // FENCE_POINT
        253: 83,  // STATUSTEXT
        12900: 114, // OPEN_DRONE_ID_BASIC_ID
        12901: 254, // OPEN_DRONE_ID_LOCATION
        12904: 77,  // OPEN_DRONE_ID_SYSTEM
    };

    // ---- CRC-X.25 ----

    function crc16(data, crcExtra) {
        let crc = 0xFFFF;
        for (let i = 0; i < data.length; i++) {
            let tmp = data[i] ^ (crc & 0xFF);
            tmp = (tmp ^ (tmp << 4)) & 0xFF;
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
            crc &= 0xFFFF;
        }
        // Include CRC_EXTRA
        if (crcExtra !== undefined) {
            let tmp = crcExtra ^ (crc & 0xFF);
            tmp = (tmp ^ (tmp << 4)) & 0xFF;
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
            crc &= 0xFFFF;
        }
        return crc;
    }

    // ---- Frame Parser ----
    // Accumulates bytes and extracts complete MAVLink v2 frames

    class FrameParser {
        constructor() {
            this.buf = new Uint8Array(2048);
            this.len = 0;
        }

        push(data) {
            const u8 = data instanceof Uint8Array ? data : new Uint8Array(data);
            // T0-13: Cap buffer at 64KB to prevent unbounded growth from corrupt streams
            if (this.len + u8.length > 65536) {
                this.len = 0; // Reset on overflow
                return;
            }
            if (this.len + u8.length > this.buf.length) {
                const newBuf = new Uint8Array(Math.min(Math.max(this.buf.length * 2, this.len + u8.length), 65536));
                newBuf.set(this.buf.subarray(0, this.len));
                this.buf = newBuf;
            }
            this.buf.set(u8, this.len);
            this.len += u8.length;
        }

        extract() {
            const messages = [];
            while (true) {
                // Find STX
                let stxIdx = -1;
                for (let i = 0; i < this.len; i++) {
                    if (this.buf[i] === MAVLINK_STX) { stxIdx = i; break; }
                }
                if (stxIdx < 0) { this.len = 0; break; }
                if (stxIdx > 0) {
                    // Discard bytes before STX
                    this.buf.copyWithin(0, stxIdx, this.len);
                    this.len -= stxIdx;
                }

                // MAVLink v2 header: 10 bytes + payload + 2 CRC
                if (this.len < 12) break;

                const payloadLen = this.buf[1];
                const frameLen = 10 + payloadLen + 2;
                if (this.len < frameLen) break;

                // Validate CRC before consuming frame
                const msgId = this.buf[7] | (this.buf[8] << 8) | (this.buf[9] << 16);
                const extra = CRC_EXTRA[msgId];
                if (extra === undefined) {
                    // Unknown message — skip past STX, try next
                    this.buf.copyWithin(0, 1, this.len);
                    this.len -= 1;
                    continue;
                }
                const crcData = this.buf.subarray(1, 10 + payloadLen);
                const expectedCrc = crc16(crcData, extra);
                const actualCrc = this.buf[10 + payloadLen] | (this.buf[10 + payloadLen + 1] << 8);
                if (expectedCrc !== actualCrc) {
                    // Bad CRC — advance by 1 byte to find next STX (T0-10 fix)
                    this.buf.copyWithin(0, 1, this.len);
                    this.len -= 1;
                    continue;
                }

                // Valid frame — extract and consume
                const frame = this.buf.slice(0, frameLen);
                this.buf.copyWithin(0, frameLen, this.len);
                this.len -= frameLen;

                const payload = frame.subarray(10, 10 + payloadLen);
                const msg = decodePayload(msgId, payload);
                if (msg) messages.push(msg);
            }
            return messages;
        }
    }

    // ---- Payload Decoders ----

    function decodePayload(msgId, payload) {
        const dv = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
        try {
            switch (msgId) {
                case MSG_ID.HEARTBEAT:
                    return {
                        type: 'heartbeat',
                        custom_mode: dv.getUint32(0, true),
                        mav_type:    dv.getUint8(4),
                        autopilot:   dv.getUint8(5),
                        base_mode:   dv.getUint8(6),
                        system_status: dv.getUint8(7),
                        armed: (dv.getUint8(6) & 0x80) !== 0,
                        mode: COPTER_MODE_NAMES[dv.getUint32(0, true)] || 'MODE_' + dv.getUint32(0, true),
                    };

                case MSG_ID.ATTITUDE:
                    return {
                        type: 'attitude',
                        time_boot_ms: dv.getUint32(0, true),
                        roll:  dv.getFloat32(4, true),
                        pitch: dv.getFloat32(8, true),
                        yaw:   dv.getFloat32(12, true),
                        rollspeed:  dv.getFloat32(16, true),
                        pitchspeed: dv.getFloat32(20, true),
                        yawspeed:   dv.getFloat32(24, true),
                    };

                case MSG_ID.GLOBAL_POSITION_INT:
                    return {
                        type: 'position',
                        time_boot_ms: dv.getUint32(0, true),
                        lat: dv.getInt32(4, true) / 1e7,
                        lon: dv.getInt32(8, true) / 1e7,
                        alt: dv.getInt32(12, true) / 1000,
                        relative_alt: dv.getInt32(16, true) / 1000,
                        vx: dv.getInt16(20, true) / 100,
                        vy: dv.getInt16(22, true) / 100,
                        vz: dv.getInt16(24, true) / 100,
                        hdg: dv.getUint16(26, true) / 100,
                    };

                case MSG_ID.SYS_STATUS:
                    return {
                        type: 'sys_status',
                        voltage_battery: dv.getUint16(14, true) / 1000,
                        current_battery: dv.getInt16(16, true) / 100,
                        battery_remaining: dv.getInt8(30),
                    };

                case MSG_ID.VFR_HUD:
                    return {
                        type: 'vfr_hud',
                        airspeed:    dv.getFloat32(0, true),
                        groundspeed: dv.getFloat32(4, true),
                        alt:         dv.getFloat32(8, true),
                        climb:       dv.getFloat32(12, true),
                        heading:     dv.getInt16(16, true),
                        throttle:    dv.getUint16(18, true),
                    };

                case MSG_ID.GPS_RAW_INT:
                    return {
                        type: 'gps_raw',
                        fix_type:   dv.getUint8(28),
                        lat:        dv.getInt32(8, true) / 1e7,
                        lon:        dv.getInt32(12, true) / 1e7,
                        alt:        dv.getInt32(16, true) / 1000,
                        eph:        dv.getUint16(20, true),
                        epv:        dv.getUint16(22, true),
                        satellites: dv.getUint8(29),
                    };

                case MSG_ID.RC_CHANNELS: {
                    const channels = [];
                    // 18 channels starting at offset 4
                    for (let i = 0; i < 18 && (4 + i*2 + 1) < payload.length; i++) {
                        channels.push(dv.getUint16(4 + i*2, true));
                    }
                    return {
                        type: 'rc_channels',
                        rssi: payload.length > 41 ? dv.getUint8(41) : 255,
                        channels,
                    };
                }

                case MSG_ID.PARAM_VALUE: {
                    const paramValue = dv.getFloat32(0, true);
                    const paramCount = dv.getUint16(4, true);
                    const paramIndex = dv.getUint16(6, true);
                    // param_id is 16 bytes starting at offset 8 (null-terminated)
                    let name = '';
                    for (let i = 0; i < 16; i++) {
                        const c = payload[8 + i];
                        if (c === 0) break;
                        name += String.fromCharCode(c);
                    }
                    return {
                        type: 'param_value',
                        name,
                        value: paramValue,
                        param_count: paramCount,
                        param_index: paramIndex,
                        param_type: payload.length > 24 ? dv.getUint8(24) : 0,
                    };
                }

                case MSG_ID.COMMAND_ACK:
                    return {
                        type: 'command_ack',
                        command: dv.getUint16(0, true),
                        result:  dv.getUint8(2),
                    };

                case MSG_ID.MISSION_COUNT:
                    return {
                        type: 'mission_count',
                        count: dv.getUint16(0, true),
                    };

                case MSG_ID.MISSION_ITEM_INT:
                    // Wire layout: param1-4(0-15), x(16), y(20), z(24), seq(28), command(30), ...
                    return {
                        type: 'mission_item',
                        param1:  dv.getFloat32(0, true),
                        param2:  dv.getFloat32(4, true),
                        param3:  dv.getFloat32(8, true),
                        param4:  dv.getFloat32(12, true),
                        lat:     dv.getInt32(16, true) / 1e7,
                        lon:     dv.getInt32(20, true) / 1e7,
                        alt:     dv.getFloat32(24, true),
                        seq:     dv.getUint16(28, true),
                        command: dv.getUint16(30, true),
                        frame:   dv.getUint8(32),
                    };

                case MSG_ID.MISSION_ACK:
                    return { type: 'mission_ack', result: dv.getUint8(2) };

                // T1-27: HOME_POSITION (242)
                case MSG_ID.HOME_POSITION:
                    return {
                        type: 'home_position',
                        lat: dv.getInt32(0, true) / 1e7,
                        lon: dv.getInt32(4, true) / 1e7,
                        alt: dv.getInt32(8, true) / 1000,
                    };

                // T1-27: BATTERY_STATUS (147)
                case MSG_ID.BATTERY_STATUS: {
                    const voltages = [];
                    // 10 cells, 2 bytes each, starting at offset 10
                    for (let i = 0; i < 10 && (10 + i * 2 + 1) < payload.length; i++) {
                        const mv = dv.getUint16(10 + i * 2, true);
                        if (mv !== 0xFFFF) voltages.push(mv / 1000);
                    }
                    return {
                        type: 'battery_status',
                        current_consumed: dv.getInt32(0, true),  // mAh
                        energy_consumed:  dv.getInt32(4, true),  // hJ
                        temperature:      dv.getInt16(8, true),  // cdegC
                        voltages:         voltages,
                        current_battery:  payload.length > 30 ? dv.getInt16(30, true) / 100 : 0,
                        battery_remaining: payload.length > 35 ? dv.getInt8(35) : -1,
                    };
                }

                // T1-27: FENCE_STATUS (162)
                case MSG_ID.FENCE_STATUS:
                    return {
                        type: 'fence_status',
                        breach_status: dv.getUint8(4),
                        breach_count:  dv.getUint16(2, true),
                        breach_type:   dv.getUint8(5),
                        breach_time:   dv.getUint32(0, true),
                    };

                // T1-27: VIBRATION (241)
                case MSG_ID.VIBRATION:
                    return {
                        type: 'vibration',
                        time_usec:    dv.getFloat64 ? 0 : 0, // skip 8-byte timestamp
                        vibration_x:  dv.getFloat32(8, true),
                        vibration_y:  dv.getFloat32(12, true),
                        vibration_z:  dv.getFloat32(16, true),
                        clipping_0:   dv.getUint32(20, true),
                        clipping_1:   dv.getUint32(24, true),
                        clipping_2:   dv.getUint32(28, true),
                    };

                // T1-27: AUTOPILOT_VERSION (148)
                case MSG_ID.AUTOPILOT_VERSION: {
                    const caps = dv.getUint32(0, true) | (dv.getUint32(4, true) << 32);
                    return {
                        type: 'autopilot_version',
                        capabilities:   dv.getUint32(0, true),
                        flight_sw_version: dv.getUint32(8, true),
                        middleware_sw_version: dv.getUint32(12, true),
                        os_sw_version:  dv.getUint32(16, true),
                        board_version:  dv.getUint32(20, true),
                        vendor_id:      payload.length > 36 ? dv.getUint16(36, true) : 0,
                        product_id:     payload.length > 38 ? dv.getUint16(38, true) : 0,
                    };
                }

                // T1-5: ADSB_VEHICLE (246)
                case MSG_ID.ADSB_VEHICLE: {
                    let callsign = '';
                    // callsign: 9 bytes starting at offset 24
                    for (let i = 0; i < 9 && (24 + i) < payload.length; i++) {
                        const c = payload[24 + i];
                        if (c === 0) break;
                        callsign += String.fromCharCode(c);
                    }
                    return {
                        type: 'adsb_vehicle',
                        ICAO_address:  dv.getUint32(0, true),
                        lat:           dv.getInt32(4, true) / 1e7,
                        lon:           dv.getInt32(8, true) / 1e7,
                        altitude:      dv.getInt32(12, true) / 1000,
                        heading:       dv.getUint16(16, true) / 100,
                        hor_velocity:  dv.getUint16(18, true) / 100,
                        ver_velocity:  dv.getInt16(20, true) / 100,
                        callsign:      callsign.trim(),
                        squawk:        payload.length > 34 ? dv.getUint16(34, true) : 0,
                    };
                }

                // T1-12: OPEN_DRONE_ID_BASIC_ID (12900)
                case MSG_ID.OPEN_DRONE_ID_BASIC_ID: {
                    let uasId = '';
                    // id_or_mac: 20 bytes at offset 0, then id_type(1), ua_type(1), uas_id(20)
                    // Simplified: target_system(1), target_component(1), id_or_mac(20), id_type(1), ua_type(1), uas_id(20)
                    const idTypeOff = 22;
                    const uaTypeOff = 23;
                    const uasIdOff = 24;
                    for (let i = 0; i < 20 && (uasIdOff + i) < payload.length; i++) {
                        const c = payload[uasIdOff + i];
                        if (c === 0) break;
                        uasId += String.fromCharCode(c);
                    }
                    return {
                        type: 'open_drone_id_basic_id',
                        id_type: payload.length > idTypeOff ? payload[idTypeOff] : 0,
                        ua_type: payload.length > uaTypeOff ? payload[uaTypeOff] : 0,
                        uas_id:  uasId.trim(),
                    };
                }

                // T1-12: OPEN_DRONE_ID_LOCATION (12901)
                case MSG_ID.OPEN_DRONE_ID_LOCATION:
                    return {
                        type: 'open_drone_id_location',
                        lat:           payload.length > 7 ? dv.getInt32(4, true) / 1e7 : 0,
                        lon:           payload.length > 11 ? dv.getInt32(8, true) / 1e7 : 0,
                        altitude_baro: payload.length > 15 ? dv.getFloat32(12, true) : 0,
                        altitude_geo:  payload.length > 19 ? dv.getFloat32(16, true) : 0,
                        height:        payload.length > 23 ? dv.getFloat32(20, true) : 0,
                        status:        payload.length > 0 ? dv.getUint8(0) : 0,
                    };

                // T1-12: OPEN_DRONE_ID_SYSTEM (12904)
                case MSG_ID.OPEN_DRONE_ID_SYSTEM: {
                    let operatorId = '';
                    // operator_id is 20 bytes at offset 24
                    for (let i = 0; i < 20 && (24 + i) < payload.length; i++) {
                        const c = payload[24 + i];
                        if (c === 0) break;
                        operatorId += String.fromCharCode(c);
                    }
                    return {
                        type: 'open_drone_id_system',
                        operator_lat: payload.length > 7 ? dv.getInt32(4, true) / 1e7 : 0,
                        operator_lon: payload.length > 11 ? dv.getInt32(8, true) / 1e7 : 0,
                        operator_id:  operatorId.trim(),
                    };
                }

                // T1-27: MISSION_REQUEST_INT (51) — decode for upload state machine
                case MSG_ID.MISSION_REQUEST_INT:
                    return {
                        type: 'mission_request_int',
                        seq: dv.getUint16(0, true),
                    };

                case MSG_ID.STATUSTEXT: {
                    const severity = dv.getUint8(0);
                    let text = '';
                    for (let i = 0; i < 50 && (1 + i) < payload.length; i++) {
                        const c = payload[1 + i];
                        if (c === 0) break;
                        text += String.fromCharCode(c);
                    }
                    return { type: 'statustext', severity, text };
                }

                default:
                    return null;
            }
        } catch (e) {
            console.warn('MAVLink decode error msgId=' + msgId, e);
            return null;
        }
    }

    // ---- Encoder Helpers ----

    let seq = 0;

    function buildFrame(msgId, payloadBytes) {
        const payloadLen = payloadBytes.length;
        const frame = new Uint8Array(10 + payloadLen + 2);
        frame[0] = MAVLINK_STX;
        frame[1] = payloadLen;
        frame[2] = 0; // incompat_flags
        frame[3] = 0; // compat_flags
        frame[4] = (seq++) & 0xFF;
        frame[5] = 255; // system_id (GCS)
        frame[6] = 0;   // component_id
        frame[7] = msgId & 0xFF;
        frame[8] = (msgId >> 8) & 0xFF;
        frame[9] = (msgId >> 16) & 0xFF;

        frame.set(payloadBytes, 10);

        const crcData = frame.subarray(1, 10 + payloadLen);
        const crcVal = crc16(crcData, CRC_EXTRA[msgId] || 0);
        frame[10 + payloadLen] = crcVal & 0xFF;
        frame[10 + payloadLen + 1] = (crcVal >> 8) & 0xFF;

        return frame;
    }

    // ---- Command Encoders ----

    function encodeCommandLong(targetSys, targetComp, command, param1, param2, param3, param4, param5, param6, param7) {
        const buf = new ArrayBuffer(33);
        const dv = new DataView(buf);
        dv.setFloat32(0, param1 || 0, true);
        dv.setFloat32(4, param2 || 0, true);
        dv.setFloat32(8, param3 || 0, true);
        dv.setFloat32(12, param4 || 0, true);
        dv.setFloat32(16, param5 || 0, true);
        dv.setFloat32(20, param6 || 0, true);
        dv.setFloat32(24, param7 || 0, true);
        dv.setUint16(28, command, true);
        dv.setUint8(30, targetSys);
        dv.setUint8(31, targetComp);
        dv.setUint8(32, 0); // confirmation
        return buildFrame(MSG_ID.COMMAND_LONG, new Uint8Array(buf));
    }

    function encodeArm(targetSys = 1) {
        return encodeCommandLong(targetSys, 1, MAV_CMD.COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0);
    }

    function encodeDisarm(targetSys = 1) {
        return encodeCommandLong(targetSys, 1, MAV_CMD.COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0);
    }

    function encodeTakeoff(alt, targetSys = 1) {
        return encodeCommandLong(targetSys, 1, MAV_CMD.NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt);
    }

    function encodeLand(targetSys = 1) {
        return encodeCommandLong(targetSys, 1, MAV_CMD.NAV_LAND, 0, 0, 0, 0, 0, 0, 0);
    }

    function encodeRtl(targetSys = 1) {
        return encodeCommandLong(targetSys, 1, MAV_CMD.NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0);
    }

    function encodeSetMode(mode, targetSys = 1) {
        const modeNum = typeof mode === 'string' ? (COPTER_MODES[mode] ?? 0) : mode;
        return encodeCommandLong(targetSys, 1, MAV_CMD.DO_SET_MODE, 1, modeNum, 0, 0, 0, 0, 0);
    }

    // T1-4: Camera ROI — sets region of interest for gimbal to point at
    function encodeDoSetRoi(lat, lon, alt, targetSys = 1) {
        // MAV_CMD_DO_SET_ROI (201): param5=lat, param6=lon, param7=alt
        return encodeCommandLong(targetSys, 1, MAV_CMD.DO_SET_ROI, 0, 0, 0, 0, lat, lon, alt);
    }

    // T1-4: Gimbal control — direct pitch/roll/yaw
    function encodeDoMountControl(pitch, roll, yaw, targetSys = 1) {
        // MAV_CMD_DO_MOUNT_CONTROL (205): param1=pitch, param2=roll, param3=yaw
        return encodeCommandLong(targetSys, 1, MAV_CMD.DO_MOUNT_CONTROL, pitch, roll, yaw, 0, 0, 0, 0);
    }

    // T1-4: Trigger camera capture
    function encodeImageStartCapture(targetSys = 1) {
        // MAV_CMD_IMAGE_START_CAPTURE (2000): param1=0 (reserved), param2=interval(0=single), param3=1 (count)
        return encodeCommandLong(targetSys, 1, MAV_CMD.IMAGE_START_CAPTURE, 0, 0, 1, 0, 0, 0, 0);
    }

    function encodeSetPositionTargetGlobalInt(lat, lon, alt, targetSys = 1) {
        // SET_POSITION_TARGET_GLOBAL_INT (msgId 86)
        const buf = new ArrayBuffer(53);
        const dv = new DataView(buf);
        dv.setUint32(0, 0, true); // time_boot_ms
        dv.setInt32(4, Math.round(lat * 1e7), true);
        dv.setInt32(8, Math.round(lon * 1e7), true);
        dv.setFloat32(12, alt, true);
        // vx, vy, vz = 0
        // afx, afy, afz = 0
        // yaw, yaw_rate = 0
        dv.setUint16(48, 0x0DF8, true); // type_mask: position only
        dv.setUint8(50, targetSys);
        dv.setUint8(51, 1); // target component
        dv.setUint8(52, 6); // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        return buildFrame(86, new Uint8Array(buf));
    }

    function encodeGcsHeartbeat() {
        const buf = new ArrayBuffer(9);
        const dv = new DataView(buf);
        dv.setUint32(0, 0, true);  // custom_mode
        dv.setUint8(4, 6);  // MAV_TYPE_GCS
        dv.setUint8(5, 8);  // MAV_AUTOPILOT_INVALID
        dv.setUint8(6, 0);  // base_mode
        dv.setUint8(7, 0);  // system_status
        dv.setUint8(8, 3);  // mavlink_version
        return buildFrame(MSG_ID.HEARTBEAT, new Uint8Array(buf));
    }

    function encodeParamRequestList(targetSys = 1) {
        const buf = new ArrayBuffer(2);
        const dv = new DataView(buf);
        dv.setUint8(0, targetSys);
        dv.setUint8(1, 1);
        return buildFrame(MSG_ID.PARAM_REQUEST_LIST, new Uint8Array(buf));
    }

    function encodeParamSet(name, value, paramType = 9, targetSys = 1) {
        // PARAM_SET: param_value(4) + target_system(1) + target_comp(1) + param_id(16) + param_type(1) = 23
        const buf = new ArrayBuffer(23);
        const dv = new DataView(buf);
        dv.setFloat32(0, value, true);
        dv.setUint8(4, targetSys);
        dv.setUint8(5, 1);
        const nameBytes = new TextEncoder().encode(name);
        const arr = new Uint8Array(buf);
        for (let i = 0; i < 16 && i < nameBytes.length; i++) {
            arr[6 + i] = nameBytes[i];
        }
        dv.setUint8(22, paramType); // MAV_PARAM_TYPE_REAL32
        return buildFrame(MSG_ID.PARAM_SET, new Uint8Array(buf));
    }

    function encodeMissionRequestList(targetSys = 1) {
        const buf = new ArrayBuffer(4);
        const dv = new DataView(buf);
        dv.setUint8(0, targetSys);
        dv.setUint8(1, 1); // target component
        dv.setUint8(2, 0); // mission_type (MAV_MISSION_TYPE_MISSION)
        return buildFrame(MSG_ID.MISSION_REQUEST_LIST, new Uint8Array(buf, 0, 3));
    }

    function encodeMissionCount(count, targetSys = 1) {
        const buf = new ArrayBuffer(5);
        const dv = new DataView(buf);
        dv.setUint16(0, count, true);
        dv.setUint8(2, targetSys);
        dv.setUint8(3, 1);
        dv.setUint8(4, 0); // mission_type
        return buildFrame(MSG_ID.MISSION_COUNT, new Uint8Array(buf));
    }

    function encodeMissionItemInt(seq, lat, lon, alt, command = MAV_CMD.NAV_WAYPOINT, targetSys = 1) {
        const buf = new ArrayBuffer(37);
        const dv = new DataView(buf);
        dv.setFloat32(0, 0, true); // param1
        dv.setFloat32(4, 0, true); // param2
        dv.setFloat32(8, 0, true); // param3
        dv.setInt32(12, Math.round(lat * 1e7), true);
        dv.setInt32(16, Math.round(lon * 1e7), true);
        dv.setFloat32(20, alt, true);
        dv.setUint16(24, seq, true);
        dv.setUint16(26, command, true);
        dv.setUint8(28, targetSys);
        dv.setUint8(29, 1); // target component
        dv.setUint8(30, 6); // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        dv.setUint8(31, 0); // current
        dv.setUint8(32, 1); // autocontinue
        dv.setFloat32(33, 0, true); // param4
        return buildFrame(MSG_ID.MISSION_ITEM_INT, new Uint8Array(buf));
    }

    // T1-1: Encode full mission item with all params for upload state machine
    function encodeMissionItemIntFull(wp, targetSys = 1) {
        const buf = new ArrayBuffer(37);
        const dv = new DataView(buf);
        dv.setFloat32(0, wp.param1 || 0, true);
        dv.setFloat32(4, wp.param2 || 0, true);
        dv.setFloat32(8, wp.param3 || 0, true);
        dv.setInt32(12, Math.round((wp.lat || 0) * 1e7), true);
        dv.setInt32(16, Math.round((wp.lon || 0) * 1e7), true);
        dv.setFloat32(20, wp.alt || 0, true);
        dv.setUint16(24, wp.seq || 0, true);
        dv.setUint16(26, wp.command || MAV_CMD.NAV_WAYPOINT, true);
        dv.setUint8(28, targetSys);
        dv.setUint8(29, 1);
        dv.setUint8(30, wp.frame || 6);
        dv.setUint8(31, wp.seq === 0 ? 1 : 0); // current
        dv.setUint8(32, 1); // autocontinue
        dv.setFloat32(33, wp.param4 || 0, true);
        return buildFrame(MSG_ID.MISSION_ITEM_INT, new Uint8Array(buf));
    }

    // T1-1: Encode MISSION_REQUEST_INT for download state machine
    function encodeMissionRequestInt(seq, targetSys = 1) {
        const buf = new ArrayBuffer(4);
        const dv = new DataView(buf);
        dv.setUint16(0, seq, true);
        dv.setUint8(2, targetSys);
        dv.setUint8(3, 1); // target component
        return buildFrame(MSG_ID.MISSION_REQUEST_INT, new Uint8Array(buf));
    }

    // T1-2: Encode FENCE_POINT for geofence upload
    function encodeFencePoint(idx, count, lat, lon, targetSys = 1) {
        // FENCE_POINT (msg 160, CRC_EXTRA 78)
        const buf = new ArrayBuffer(12);
        const dv = new DataView(buf);
        dv.setFloat32(0, lat, true);
        dv.setFloat32(4, lon, true);
        dv.setUint8(8, targetSys);
        dv.setUint8(9, 1);
        dv.setUint8(10, idx);
        dv.setUint8(11, count);
        return buildFrame(160, new Uint8Array(buf));
    }

    // ---- Public API ----

    return {
        MSG_ID, MAV_CMD, COPTER_MODES, COPTER_MODE_NAMES,
        FrameParser, crc16, CRC_EXTRA, buildFrame,
        encodeCommandLong,
        encodeArm, encodeDisarm,
        encodeTakeoff, encodeLand, encodeRtl,
        encodeSetMode,
        encodeDoSetRoi, encodeDoMountControl, encodeImageStartCapture,
        encodeSetPositionTargetGlobalInt,
        encodeGcsHeartbeat,
        encodeParamRequestList, encodeParamSet,
        encodeMissionRequestList, encodeMissionCount, encodeMissionItemInt,
        encodeMissionItemIntFull, encodeMissionRequestInt, encodeFencePoint,
    };

})();
