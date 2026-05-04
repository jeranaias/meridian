# Meridian GCS — moved to its own repository

The browser-based ground control station for Meridian now lives at:

**https://github.com/jeranaias/meridian-gcs**

It was split out of this repository on 2026-05-04 (issue [#1](https://github.com/jeranaias/meridian/issues/1)) so the GCS and the firmware can release independently. Full git history was preserved via `git subtree split`.

If you are looking for `index.html`, `mission.html`, the MNP / MAVLink browser client, the styling, the i18n locales — clone the new repo:

```bash
git clone https://github.com/jeranaias/meridian-gcs.git
```

This stub remains so that anyone arriving via an old link or an old `meridian/gcs/` reference finds the new home.
