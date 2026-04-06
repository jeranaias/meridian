/* ============================================================
   i18n.js — Lightweight internationalisation framework (T2-14)
   Usage: i18n.t('key') returns translated string.
   Strings loaded from locales/<code>.json.
   Default locale: 'en', configurable via settings.
   Canvas-rendered text (ADI, compass, tapes) is NOT translated —
   only DOM text uses i18n.
   ============================================================ */

'use strict';

window.i18n = (function () {

    var _locale  = 'en';
    var _strings = {};          // loaded strings for current locale
    var _fallback = {};         // always-loaded English fallback

    // ─── Public API ───────────────────────────────────────────

    /**
     * Translate a key. Supports {{placeholder}} interpolation.
     * @param {string} key
     * @param {Object} [vars]  e.g. { count: 5 }
     * @returns {string}
     */
    function t(key, vars) {
        var str = _strings[key] || _fallback[key] || key;
        if (vars) {
            str = str.replace(/\{\{(\w+)\}\}/g, function (_, k) {
                return (vars[k] !== undefined) ? vars[k] : '{{' + k + '}}';
            });
        }
        return str;
    }

    /**
     * Load a locale. Returns a Promise that resolves when done.
     * @param {string} locale  e.g. 'en', 'ar'
     */
    function setLocale(locale) {
        _locale = locale;
        return _load(locale).then(function (data) {
            _strings = data;
            meridian.events.emit('locale_change', { locale: locale });
        }).catch(function (err) {
            console.warn('i18n: failed to load locale "' + locale + '"', err);
            _strings = _fallback;
        });
    }

    function getLocale() {
        return _locale;
    }

    // ─── Internal ─────────────────────────────────────────────

    function _load(locale) {
        return fetch('locales/' + locale + '.json')
            .then(function (r) {
                if (!r.ok) throw new Error('HTTP ' + r.status);
                return r.json();
            });
    }

    function init() {
        var locale = (meridian.settings && meridian.settings.locale) || 'en';
        _locale = locale;

        // Always load English as fallback first
        _load('en').then(function (data) {
            _fallback = data;
            _strings  = data;

            if (locale !== 'en') {
                return setLocale(locale);
            }
        }).catch(function (err) {
            console.warn('i18n: could not load en.json — i18n disabled', err);
        });
    }

    return { t, setLocale, getLocale, init };

})();
