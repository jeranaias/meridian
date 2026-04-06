/* ============================================================
   modal.js — Custom modal dialogs (no browser confirm/prompt)
   ============================================================ */

'use strict';

window.Modal = (function () {

    let overlay;

    function init() {
        overlay = document.createElement('div');
        overlay.id = 'modal-overlay';
        overlay.style.cssText = `
            position: fixed; inset: 0; z-index: 10000;
            background: rgba(0,0,0,0.35);
            display: none; align-items: center; justify-content: center;
            backdrop-filter: blur(2px); -webkit-backdrop-filter: blur(2px);
        `;
        document.body.appendChild(overlay);
    }

    function show(html) {
        return new Promise(resolve => {
            const card = document.createElement('div');
            card.style.cssText = `
                background: var(--c-bg-raised); border: 1px solid var(--c-border);
                border-radius: 8px; padding: 20px 24px; min-width: 300px; max-width: 420px;
                box-shadow: 0 8px 32px rgba(0,0,0,0.2);
                font-family: var(--f-body); color: var(--c-text);
                animation: fadeIn 0.15s ease-out;
            `;
            card.innerHTML = html;
            overlay.innerHTML = '';
            overlay.appendChild(card);
            overlay.style.display = 'flex';

            // Wire buttons
            card.querySelectorAll('[data-modal]').forEach(btn => {
                btn.addEventListener('click', () => {
                    const val = btn.dataset.modal;
                    overlay.style.display = 'none';
                    resolve(val === 'true' ? true : val === 'false' ? false : val);
                });
            });

            // Wire inputs
            const input = card.querySelector('input[data-modal-input]');
            if (input) {
                input.focus();
                input.addEventListener('keydown', e => {
                    if (e.key === 'Enter') {
                        overlay.style.display = 'none';
                        resolve(input.value);
                    }
                });
            }

            // Escape to cancel
            const esc = (e) => {
                if (e.key === 'Escape') {
                    overlay.style.display = 'none';
                    resolve(false);
                    document.removeEventListener('keydown', esc);
                }
            };
            document.addEventListener('keydown', esc);
        });
    }

    async function confirm(title, message, confirmText, danger) {
        const color = danger ? 'var(--c-emergency)' : 'var(--c-primary)';
        const result = await show(`
            <div style="font-family:var(--f-display);font-size:16px;font-weight:700;margin-bottom:8px;">${title}</div>
            <div style="font-size:13px;color:var(--c-text-dim);margin-bottom:16px;line-height:1.5;">${message}</div>
            <div style="display:flex;gap:8px;justify-content:flex-end;">
                <button data-modal="false" style="
                    padding:6px 16px;border:1px solid var(--c-border);background:transparent;
                    color:var(--c-neutral);border-radius:6px;cursor:pointer;font-family:var(--f-body);
                    font-size:12px;font-weight:600;
                ">Cancel</button>
                <button data-modal="true" style="
                    padding:6px 16px;border:none;background:${color};
                    color:white;border-radius:6px;cursor:pointer;font-family:var(--f-body);
                    font-size:12px;font-weight:600;
                ">${confirmText || 'Confirm'}</button>
            </div>
        `);
        return result === true;
    }

    async function prompt(title, message, defaultVal) {
        const result = await show(`
            <div style="font-family:var(--f-display);font-size:16px;font-weight:700;margin-bottom:8px;">${title}</div>
            <div style="font-size:13px;color:var(--c-text-dim);margin-bottom:12px;">${message}</div>
            <input data-modal-input type="text" value="${defaultVal || ''}" style="
                width:100%;padding:8px 10px;border:1.5px solid var(--c-border);
                border-radius:6px;background:var(--c-bg);color:var(--c-text);
                font-family:var(--f-mono);font-size:13px;outline:none;
                margin-bottom:16px;
            ">
            <div style="display:flex;gap:8px;justify-content:flex-end;">
                <button data-modal="false" style="
                    padding:6px 16px;border:1px solid var(--c-border);background:transparent;
                    color:var(--c-neutral);border-radius:6px;cursor:pointer;font-family:var(--f-body);
                    font-size:12px;font-weight:600;
                ">Cancel</button>
                <button data-modal="true" style="
                    padding:6px 16px;border:none;background:var(--c-primary);
                    color:white;border-radius:6px;cursor:pointer;font-family:var(--f-body);
                    font-size:12px;font-weight:600;
                ">Connect</button>
            </div>
        `);
        if (result === false) return null;
        const input = overlay.querySelector('input[data-modal-input]');
        return input ? input.value : result;
    }

    return { init, show, confirm, prompt };

})();
