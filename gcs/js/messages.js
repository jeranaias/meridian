/* ============================================================
   messages.js — Message log display (expandable)
   Compact: shows latest 1-2 messages in action bar.
   Expanded: click to open full scrollable log overlay.
   ============================================================ */

'use strict';

window.Messages = (function () {

    let logEl;
    let expandedEl;
    let expanded = false;
    let entries = [];
    const MAX_ENTRIES = 200;

    function init() {
        logEl = document.querySelector('.msg-log-bar');
        if (!logEl) return;

        // Make compact log clickable to expand
        logEl.style.cursor = 'pointer';
        logEl.title = 'Click to expand message log';
        logEl.addEventListener('click', toggleExpand);

        meridian.events.on('message', onMessage);
    }

    function onMessage(entry) {
        const time = new Date(entry.time);
        const timeStr = time.getHours().toString().padStart(2, '0') + ':' +
                        time.getMinutes().toString().padStart(2, '0') + ':' +
                        time.getSeconds().toString().padStart(2, '0');

        entries.push({ timeStr: timeStr, text: entry.text, level: entry.level });
        if (entries.length > MAX_ENTRIES) entries.shift();

        // Update compact view — show latest 2 entries
        renderCompact();

        // Update expanded view if open
        if (expanded && expandedEl) renderExpanded();
    }

    function renderCompact() {
        if (!logEl) return;
        logEl.innerHTML = '';
        const recent = entries.slice(-2);
        recent.forEach(function (e) {
            const div = document.createElement('div');
            div.className = 'msg-entry';
            div.innerHTML = '<span class="msg-time">' + e.timeStr + '</span>' +
                '<span class="msg-text ' + e.level + '">' + escapeHtml(e.text) + '</span>';
            logEl.appendChild(div);
        });
    }

    function toggleExpand() {
        if (expanded) {
            collapse();
        } else {
            expand();
        }
    }

    function expand() {
        expanded = true;
        logEl.classList.add('msg-log-expanded-active');

        expandedEl = document.createElement('div');
        expandedEl.className = 'msg-log-expanded';
        expandedEl.innerHTML =
            '<div class="msg-log-expanded-header">' +
                '<span class="msg-log-expanded-title">Message Log (' + entries.length + ')</span>' +
                '<button type="button" class="msg-log-expanded-close" title="Close">&times;</button>' +
            '</div>' +
            '<div class="msg-log-expanded-body" id="msg-log-expanded-body"></div>';

        // Position above the action bar
        const ab = document.getElementById('action-bar');
        if (ab) ab.parentElement.insertBefore(expandedEl, ab);
        else document.body.appendChild(expandedEl);

        expandedEl.querySelector('.msg-log-expanded-close').addEventListener('click', function (e) {
            e.stopPropagation();
            collapse();
        });

        renderExpanded();

        // Close on Escape
        document.addEventListener('keydown', onEscape);
    }

    function collapse() {
        expanded = false;
        logEl.classList.remove('msg-log-expanded-active');
        if (expandedEl) { expandedEl.remove(); expandedEl = null; }
        document.removeEventListener('keydown', onEscape);
    }

    function onEscape(e) {
        if (e.key === 'Escape') collapse();
    }

    function renderExpanded() {
        const body = document.getElementById('msg-log-expanded-body');
        if (!body) return;
        body.innerHTML = '';
        entries.forEach(function (e) {
            const div = document.createElement('div');
            div.className = 'msg-entry';
            div.innerHTML = '<span class="msg-time">' + e.timeStr + '</span>' +
                '<span class="msg-text ' + e.level + '">' + escapeHtml(e.text) + '</span>';
            body.appendChild(div);
        });
        body.scrollTop = body.scrollHeight;

        // Update count
        var title = expandedEl.querySelector('.msg-log-expanded-title');
        if (title) title.textContent = 'Message Log (' + entries.length + ')';
    }

    function escapeHtml(s) {
        const div = document.createElement('div');
        div.textContent = s;
        return div.innerHTML;
    }

    return { init };

})();
