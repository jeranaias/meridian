/* ============================================================
   checklist.js — Commissioning checklist data model
   Checks vehicle params to determine completion status.
   ============================================================ */

'use strict';

window.Checklist = (function () {

    // Status: 'complete' | 'incomplete' | 'warning'
    // check() receives params object, returns status string + detail text

    const REQUIRED_ITEMS = [
        {
            id: 'frame',
            name: 'Frame Type',
            required: true,
            component: 'FrameSetup',
            check: function (params) {
                var fc = params.FRAME_CLASS;
                var ft = params.FRAME_TYPE;
                if (fc === undefined || fc === 0) return { status: 'incomplete', text: 'Not configured' };
                var name = FrameSetup ? FrameSetup.getFrameName(fc, ft) : ('Class ' + fc);
                return { status: 'complete', text: name };
            },
        },
        {
            id: 'accel',
            name: 'Accelerometer Cal',
            required: true,
            component: 'AccelCal',
            check: function (params) {
                // INS_ACCOFFS_X/Y/Z != 0 means calibrated
                var x = params.INS_ACCOFFS_X;
                var y = params.INS_ACCOFFS_Y;
                var z = params.INS_ACCOFFS_Z;
                if (x === undefined) return { status: 'incomplete', text: 'Not calibrated' };
                if (x === 0 && y === 0 && z === 0) return { status: 'incomplete', text: 'Not calibrated' };
                return { status: 'complete', text: 'Calibrated' };
            },
        },
        {
            id: 'compass',
            name: 'Compass Cal',
            required: true,
            component: 'CompassCal',
            check: function (params) {
                var ox = params.COMPASS_OFS_X;
                var oy = params.COMPASS_OFS_Y;
                var oz = params.COMPASS_OFS_Z;
                if (ox === undefined) return { status: 'incomplete', text: 'Not calibrated' };
                if (ox === 0 && oy === 0 && oz === 0) return { status: 'warning', text: 'Offsets all zero' };
                return { status: 'complete', text: 'Calibrated' };
            },
        },
        {
            id: 'radio',
            name: 'Radio Cal',
            required: true,
            component: 'RadioCal',
            check: function (params) {
                var min = params.RC1_MIN;
                var max = params.RC1_MAX;
                if (min === undefined || max === undefined) return { status: 'incomplete', text: 'Not calibrated' };
                if (min === 1100 && max === 1900) return { status: 'warning', text: 'Using defaults' };
                if (max - min < 400) return { status: 'warning', text: 'Low range (' + (max - min) + ')' };
                return { status: 'complete', text: 'Calibrated (' + min + '-' + max + ')' };
            },
        },
        {
            id: 'modes',
            name: 'Flight Modes',
            required: true,
            component: 'FlightModes',
            check: function (params) {
                var count = 0;
                for (var i = 1; i <= 6; i++) {
                    if (params['FLTMODE' + i] !== undefined) count++;
                }
                if (count === 0) return { status: 'incomplete', text: 'Not configured' };
                if (count < 6) return { status: 'warning', text: count + '/6 modes set' };
                return { status: 'complete', text: '6 modes set' };
            },
        },
        {
            id: 'failsafe',
            name: 'Failsafe',
            required: true,
            component: 'Failsafe',
            check: function (params) {
                var rc = params.FS_THR_ENABLE;
                var batt = params.BATT_FS_LOW_ACT;
                var gcs = params.FS_GCS_ENABLE;
                if (rc === undefined && batt === undefined && gcs === undefined) {
                    return { status: 'incomplete', text: 'Not configured' };
                }
                var parts = [];
                if (rc && rc > 0) parts.push('RC');
                if (batt && batt > 0) parts.push('Batt');
                if (gcs && gcs > 0) parts.push('GCS');
                if (parts.length === 0) return { status: 'warning', text: 'All disabled' };
                return { status: 'complete', text: parts.join(', ') + ' enabled' };
            },
        },
        {
            id: 'battery',
            name: 'Battery Monitor',
            required: true,
            component: 'BatterySetup',
            check: function (params) {
                var monitor = params.BATT_MONITOR;
                if (monitor === undefined || monitor === 0) return { status: 'incomplete', text: 'Disabled' };
                var cap = params.BATT_CAPACITY || 0;
                if (cap === 0) return { status: 'warning', text: 'Monitor on, no capacity set' };
                return { status: 'complete', text: cap + ' mAh' };
            },
        },
    ];

    var OPTIONAL_ITEMS = [
        {
            id: 'motor_test',
            name: 'Motor Test',
            required: false,
            component: 'MotorTest',
            check: function () { return { status: 'optional', text: 'Optional' }; },
        },
        {
            id: 'esc_cal',
            name: 'ESC Calibration',
            required: false,
            component: null,
            check: function () { return { status: 'optional', text: 'Optional' }; },
        },
        {
            id: 'osd',
            name: 'OSD',
            required: false,
            component: null,
            check: function (params) {
                var type = params.OSD_TYPE;
                if (type === undefined || type === 0) return { status: 'optional', text: 'Not configured' };
                return { status: 'complete', text: 'Configured' };
            },
        },
        {
            id: 'servo_out',
            name: 'Servo Output',
            required: false,
            component: null,
            check: function () { return { status: 'optional', text: 'Optional' }; },
        },
        {
            // T3-19: Firmware update module
            id: 'firmware',
            name: 'Firmware',
            required: false,
            component: 'FirmwareSetup',
            check: function () {
                var v = meridian.v;
                var fwv = v && v.firmwareVersion;
                if (!fwv) return { status: 'optional', text: 'Version unknown' };
                var ver = fwv.major + '.' + fwv.minor + '.' + fwv.patch;
                return { status: 'complete', text: ver };
            },
        },
    ];

    function getAllItems() {
        return REQUIRED_ITEMS.concat(OPTIONAL_ITEMS);
    }

    function getRequiredItems() {
        return REQUIRED_ITEMS;
    }

    function getOptionalItems() {
        return OPTIONAL_ITEMS;
    }

    function evaluate(params) {
        var results = [];
        var items = getAllItems();
        for (var i = 0; i < items.length; i++) {
            var item = items[i];
            var result = item.check(params || {});
            results.push({
                id: item.id,
                name: item.name,
                required: item.required,
                component: item.component,
                status: result.status,
                text: result.text,
            });
        }
        return results;
    }

    function incompleteRequiredCount(params) {
        var count = 0;
        var items = REQUIRED_ITEMS;
        for (var i = 0; i < items.length; i++) {
            var result = items[i].check(params || {});
            if (result.status !== 'complete') count++;
        }
        return count;
    }

    return { getAllItems, getRequiredItems, getOptionalItems, evaluate, incompleteRequiredCount };

})();
