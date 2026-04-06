# Contributing to Meridian GCS

Thank you for your interest in contributing. This document covers the conventions and philosophy that keep the codebase consistent.

## Philosophy

Meridian GCS is a **zero-dependency, zero-build-step** web application. There is no npm, no bundler, no transpiler. You open `index.html` in a browser and it works. This is intentional and non-negotiable.

- **No npm.** No `package.json`. No `node_modules`.
- **No build step.** No Webpack, Vite, Rollup, or esbuild.
- **No framework.** No React, Vue, Svelte, or Angular.
- **One CDN dependency:** Leaflet for mapping. That's it.

## Code Style

### JavaScript

- Every module uses the **IIFE pattern** (Immediately Invoked Function Expression) and exposes a single global:
  ```js
  'use strict';

  window.ModuleName = (function () {
      // Private state and functions here

      return { publicMethod1, publicMethod2 };
  })();
  ```
- `'use strict';` at the top of every file.
- Use `var` for module-level state when you need hoisting clarity; `const`/`let` inside functions.
- Prefer `function` declarations over arrow functions for named public methods.
- No classes unless the data structure genuinely warrants it (e.g. `FrameParser`).
- All DOM queries should be cached where possible. Never query inside a render loop.

### CSS

- All colors, spacing, and typography use **CSS custom properties** defined in `css/base.css`.
- Never use hardcoded colors outside of theme files (`base.css` for light, `theme-dark.css` for dark).
- **Minimum font size is 10px.** No exceptions. This is the van Schneider floor defined in base.css.
- Use the type scale variables: `--ts-xs` (10px), `--ts-sm` (12px), `--ts-md` (14px), `--ts-lg` (17px), `--ts-xl` (20px).
- Fonts: `--f-display` (Rajdhani) for headings, `--f-body` (Barlow) for text, `--f-mono` (DM Mono) for data.

### Canvas

- Cache dimensions on resize, not on every draw call.
- Cache gradient objects; only recreate them on resize.
- Use `window._themeColors` for dynamic theme-aware canvas colors.
- Target 10 Hz update rate for instruments (attitude, tapes, compass).

## File Organization

```
gcs/
  index.html          <- Single HTML entry point
  css/
    base.css          <- Light theme + variables + reset
    theme-dark.css    <- Dark theme overrides
    toolbar.css       <- Toolbar styles
    instruments.css   <- HUD instrument styles
    ...
  js/
    state.js          <- Multi-vehicle state + event bus (load first)
    mavlink.js        <- MAVLink v2 codec
    connection.js     <- WebSocket transport
    router.js         <- View routing
    fly/              <- Fly view modules (ADI, tapes, compass, map, etc.)
    plan/             <- Plan view modules (mission, waypoints, survey)
    setup/            <- Setup view modules (calibration, failsafe, etc.)
    params/           <- Parameter list and tuning
    logs/             <- Tlog recording, playback, inspector
```

Script load order matters. Core modules (`state.js`, `mavlink.js`, `connection.js`, `router.js`) must load before view modules.

## Making Changes

1. Fork the repository and create a feature branch.
2. Make your changes following the conventions above.
3. Test in both dark and light themes.
4. Test with the demo mode (default on page load) and, if possible, with a SITL instance.
5. Open a pull request with a clear description of what changed and why.

## Reporting Issues

Open a GitHub issue with:
- What you expected to happen
- What actually happened
- Browser and OS version
- Screenshots if it's a visual issue
