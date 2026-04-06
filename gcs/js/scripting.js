/* ============================================================
   scripting.js — Live JavaScript Console for GCS Scripting
   T3-11: "Script" tab in the Logs panel.
   Sandboxed eval with access to vehicle state, Connection,
   Mission, FlyMap. Predefined helpers: arm, disarm, takeoff,
   rtl, setMode, goto, getParam, setParam.
   ============================================================ */

'use strict';

window.Scripting = (function () {

    const MAX_HISTORY = 50;
    const MAX_OUTPUT_LINES = 200;

    var _history = [];      // command strings
    var _historyIdx = -1;   // current history navigation index
    var _outputLines = [];  // rendered output line strings

    // -------------------------------------------------------------------------
    // Render — called by LogsView when "Script" tab is active
    // -------------------------------------------------------------------------

    function render(container) {
        container.innerHTML = '';
        container.className = 'scripting-container';

        // Output area
        var output = document.createElement('div');
        output.className = 'scripting-output';
        output.id = 'scripting-output';
        output.setAttribute('aria-live', 'polite');
        output.setAttribute('aria-label', 'Script output');

        // Render any existing output
        _outputLines.forEach(function (line) {
            output.appendChild(_makeOutputLine(line));
        });
        output.scrollTop = output.scrollHeight;
        container.appendChild(output);

        // Divider
        var divider = document.createElement('div');
        divider.className = 'scripting-divider';
        container.appendChild(divider);

        // Input row
        var inputRow = document.createElement('div');
        inputRow.className = 'scripting-input-row';

        var prompt = document.createElement('span');
        prompt.className = 'scripting-prompt';
        prompt.textContent = '>';
        inputRow.appendChild(prompt);

        var input = document.createElement('textarea');
        input.className = 'scripting-input';
        input.id = 'scripting-input';
        input.rows = 3;
        input.placeholder = 'arm()  //  takeoff(10)  //  goto(37.7749, -122.4194, 50)';
        input.setAttribute('aria-label', 'Script input');
        inputRow.appendChild(input);
        container.appendChild(inputRow);

        // Button row
        var btnRow = document.createElement('div');
        btnRow.className = 'scripting-btn-row';

        var runBtn = document.createElement('button');
        runBtn.className = 'offline-btn download';
        runBtn.textContent = '\u25B6 Run';
        runBtn.title = 'Run script (Ctrl+Enter)';
        runBtn.addEventListener('click', function () { _execute(input, output); });

        var clearBtn = document.createElement('button');
        clearBtn.className = 'offline-btn draw';
        clearBtn.textContent = '\u2715 Clear Output';
        clearBtn.addEventListener('click', function () { _clearOutput(output); });

        var helpBtn = document.createElement('button');
        helpBtn.className = 'offline-btn draw';
        helpBtn.textContent = '? Help';
        helpBtn.addEventListener('click', function () { _showHelp(output); });

        btnRow.appendChild(runBtn);
        btnRow.appendChild(clearBtn);
        btnRow.appendChild(helpBtn);
        container.appendChild(btnRow);

        // Keyboard handling
        input.addEventListener('keydown', function (e) {
            // Ctrl+Enter to run
            if (e.ctrlKey && e.key === 'Enter') {
                e.preventDefault();
                _execute(input, output);
                return;
            }
            // Up/Down arrow for history
            if (e.key === 'ArrowUp') {
                e.preventDefault();
                if (_history.length === 0) return;
                if (_historyIdx < 0) _historyIdx = _history.length - 1;
                else if (_historyIdx > 0) _historyIdx--;
                input.value = _history[_historyIdx] || '';
                return;
            }
            if (e.key === 'ArrowDown') {
                e.preventDefault();
                if (_historyIdx < 0) return;
                _historyIdx++;
                if (_historyIdx >= _history.length) {
                    _historyIdx = -1;
                    input.value = '';
                } else {
                    input.value = _history[_historyIdx] || '';
                }
            }
        });

        // Focus input
        setTimeout(function () { input.focus(); }, 0);
    }

    // -------------------------------------------------------------------------
    // Script execution — sandboxed eval
    // -------------------------------------------------------------------------

    function _execute(inputEl, outputEl) {
        var code = inputEl.value.trim();
        if (!code) return;

        // Add to history
        if (_history[_history.length - 1] !== code) {
            _history.push(code);
            if (_history.length > MAX_HISTORY) _history.shift();
        }
        _historyIdx = -1;

        _appendOutput(outputEl, '> ' + code, 'scripting-line-input');

        // Capture console.log
        var logs = [];
        var fakeConsole = {
            log: function () {
                var args = Array.prototype.slice.call(arguments);
                logs.push(args.map(function (a) {
                    try { return typeof a === 'object' ? JSON.stringify(a, null, 2) : String(a); }
                    catch (e) { return String(a); }
                }).join(' '));
            },
            warn: function () {
                var args = Array.prototype.slice.call(arguments);
                logs.push('[warn] ' + args.join(' '));
            },
            error: function () {
                var args = Array.prototype.slice.call(arguments);
                logs.push('[error] ' + args.join(' '));
            },
        };

        var result;
        var errMsg = null;

        try {
            result = _sandboxedEval(code, fakeConsole);
        } catch (e) {
            errMsg = e.message || String(e);
        }

        // Show captured console output first
        logs.forEach(function (line) {
            _appendOutput(outputEl, line, 'scripting-line-log');
        });

        // Show result or error
        if (errMsg !== null) {
            _appendOutput(outputEl, 'Error: ' + errMsg, 'scripting-line-error');
        } else if (result !== undefined) {
            var display;
            try { display = JSON.stringify(result, null, 2); }
            catch (e) { display = String(result); }
            _appendOutput(outputEl, '\u2190 ' + display, 'scripting-line-result');
        } else {
            _appendOutput(outputEl, '\u2190 undefined', 'scripting-line-result scripting-dim');
        }

        inputEl.value = '';
        outputEl.scrollTop = outputEl.scrollHeight;
    }

    // Sandboxed eval: access to vehicle state, connections, mission, map helpers.
    // No direct DOM access. Cannot modify meridian.events.
    function _sandboxedEval(code, fakeConsole) {
        // Build the sandbox scope object
        var scope = _buildScope(fakeConsole);

        // Construct a function with the scope variables as named parameters
        // This prevents access to globals not explicitly exposed.
        var paramNames = Object.keys(scope);
        var paramValues = paramNames.map(function (k) { return scope[k]; });

        // eslint-disable-next-line no-new-func
        var fn = new Function(
            paramNames.concat(['__code__']).join(','),
            '"use strict";\nreturn eval(__code__);'
        );

        return fn.apply(null, paramValues.concat([code]));
    }

    function _buildScope(fakeConsole) {
        // Vehicle state (read-only proxy if Proxy available)
        var vState = meridian.v || {};

        // Expose immutable snapshot of vehicle state
        var vSnapshot = {};
        try { vSnapshot = JSON.parse(JSON.stringify(vState)); } catch (e) { vSnapshot = vState; }

        var scope = {
            // Vehicle state (read-only copy for inspection)
            'meridian': { v: vSnapshot },

            // Console output capture
            'console': fakeConsole,

            // Mission data
            'Mission': window.Mission || null,

            // Map access (Leaflet)
            'FlyMap': window.FlyMap || null,

            // Helper functions
            'arm':       _helperArm,
            'disarm':    _helperDisarm,
            'takeoff':   _helperTakeoff,
            'rtl':       _helperRtl,
            'setMode':   _helperSetMode,
            'goto':      _helperGoto,
            'getParam':  _helperGetParam,
            'setParam':  _helperSetParam,

            // Utilities
            'JSON':     JSON,
            'Math':     Math,
            'Date':     Date,
            'parseFloat': parseFloat,
            'parseInt':   parseInt,
            'isNaN':      isNaN,
            'isFinite':   isFinite,
            'String':     String,
            'Number':     Number,
            'Boolean':    Boolean,
            'Array':      Array,
            'Object':     Object,

            // Block dangerous globals explicitly
            'window':    undefined,
            'document':  undefined,
            'location':  undefined,
            'navigator': undefined,
            'fetch':     undefined,
            'XMLHttpRequest': undefined,
        };

        return scope;
    }

    // -------------------------------------------------------------------------
    // Predefined helper functions
    // -------------------------------------------------------------------------

    function _helperArm() {
        if (!window.Connection) return 'Connection not available';
        Connection.sendArm(true);
        return 'ARM command sent';
    }

    function _helperDisarm() {
        if (!window.Connection) return 'Connection not available';
        Connection.sendArm(false);
        return 'DISARM command sent';
    }

    function _helperTakeoff(alt) {
        alt = parseFloat(alt) || 10;
        if (!window.Connection) return 'Connection not available';
        Connection.sendTakeoff(alt);
        return 'TAKEOFF command sent (alt=' + alt + 'm)';
    }

    function _helperRtl() {
        if (!window.Connection) return 'Connection not available';
        Connection.sendSetMode(6); // RTL = 6
        return 'RTL mode set';
    }

    function _helperSetMode(name) {
        var modeMap = {
            'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3,
            'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'CIRCLE': 7,
            'LAND': 9, 'SPORT': 13, 'FLIP': 14, 'AUTOTUNE': 15,
            'POSHOLD': 16, 'BRAKE': 17,
        };
        var upper = (name || '').toUpperCase();
        var num = modeMap[upper];
        if (num === undefined) {
            return 'Unknown mode: ' + name + '. Known: ' + Object.keys(modeMap).join(', ');
        }
        if (!window.Connection) return 'Connection not available';
        Connection.sendSetMode(num);
        return 'Mode set: ' + upper + ' (' + num + ')';
    }

    function _helperGoto(lat, lon, alt) {
        lat = parseFloat(lat);
        lon = parseFloat(lon);
        alt = parseFloat(alt) || 10;
        if (isNaN(lat) || isNaN(lon)) return 'Invalid coordinates';
        if (!window.Connection) return 'Connection not available';
        Connection.sendGoto(lat, lon, alt);
        return 'GOTO sent: lat=' + lat + ' lon=' + lon + ' alt=' + alt;
    }

    function _helperGetParam(name) {
        var v = meridian.v;
        if (!v || !v.params) return null;
        var val = v.params[name];
        return val !== undefined ? val : null;
    }

    function _helperSetParam(name, val) {
        if (!window.Connection) return 'Connection not available';
        Connection.sendParamSet(name, parseFloat(val));
        return 'PARAM_SET sent: ' + name + ' = ' + val;
    }

    // -------------------------------------------------------------------------
    // Output helpers
    // -------------------------------------------------------------------------

    function _appendOutput(outputEl, text, cssClass) {
        var line = { text: text, cssClass: cssClass || '' };
        _outputLines.push(line);
        if (_outputLines.length > MAX_OUTPUT_LINES) _outputLines.shift();

        if (outputEl) {
            var el = _makeOutputLine(line);
            outputEl.appendChild(el);
            // Trim excess lines from DOM
            while (outputEl.children.length > MAX_OUTPUT_LINES) {
                outputEl.removeChild(outputEl.firstChild);
            }
        }
    }

    function _makeOutputLine(line) {
        var el = document.createElement('div');
        el.className = 'scripting-line ' + (line.cssClass || '');
        el.textContent = line.text;
        return el;
    }

    function _clearOutput(outputEl) {
        _outputLines = [];
        if (outputEl) outputEl.innerHTML = '';
    }

    function _showHelp(outputEl) {
        var help = [
            '--- Meridian Scripting Console ---',
            'Ctrl+Enter  Run script',
            'Up/Down     Command history',
            '',
            'Available helpers:',
            '  arm()                  Send arm command',
            '  disarm()               Send disarm command',
            '  takeoff(alt)           Takeoff to altitude (meters)',
            '  rtl()                  Return to launch',
            '  setMode(name)          Set flight mode by name',
            '  goto(lat, lon, alt)    Fly to coordinates',
            '  getParam(name)         Read a parameter value',
            '  setParam(name, val)    Set a parameter',
            '',
            'Available objects:',
            '  meridian.v             Vehicle state snapshot',
            '  Mission                Mission data and API',
            '  FlyMap.getMap()        Leaflet map instance',
            '  console.log(...)       Output to this console',
            '',
            'Examples:',
            '  getParam("ATC_RAT_RLL_P")   // 0.135',
            '  setMode("LOITER")',
            '  goto(37.7749, -122.4194, 30)',
            '  meridian.v.lat + ", " + meridian.v.lon',
        ];
        help.forEach(function (line) {
            _appendOutput(outputEl, line, 'scripting-line-help');
        });
        if (outputEl) outputEl.scrollTop = outputEl.scrollHeight;
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    return {
        render,
    };

})();
