# Build Meridian With Us

Meridian is not one person's project. It's an answer to a question the entire drone community has been asking for years: **what comes after ArduPilot?**

Not a replacement. Not a fork. A fresh start — built on modern foundations, designed for the next decade of autonomous flight, and open to everyone.

---

## How to Get Involved

### 1. Try It

```bash
git clone https://github.com/jeranaias/meridian.git
cd meridian/gcs
python -m http.server 8080
```

Open http://localhost:8080. Go to Settings > Connection > Start Demo Mode. Click around. Break things. Tell us what's wrong.

### 2. File Issues

Found a bug? UI feel off? Missing a feature you need? **Open an issue.** We have templates for:

- **Bug reports** — something doesn't work
- **Feature requests** — something should exist
- **Board support requests** — your flight controller isn't listed

Every issue gets read. Every issue gets a response.

### 3. Fork and Build

This is where the real magic happens. Meridian is MIT-licensed. Fork it. Build what you need. Put in a PR.

**Things we need right now:**

#### Flight Stack (Rust)
- [ ] SITL integration testing — connect GCS to SITL, fly a mission, verify telemetry
- [ ] Board TOML validation — test the 378 auto-generated board configs against real hardware
- [ ] Sensor driver testing — if you have an ICM-42688, BMI270, BMP388, or uBlox GPS on a dev board, test the drivers
- [ ] EKF tuning — the filter works but needs real-world validation data
- [ ] RC protocol testing — CRSF/ELRS, SBUS, DSM with real receivers

#### Ground Control (Browser)
- [ ] UI/UX feedback — what feels clunky, what's missing, what's in the wrong place
- [ ] New instrument widgets — vertical speed indicator, turn coordinator, engine gauges
- [ ] i18n translations — the framework exists (`locales/en.json`), we need other languages
- [ ] Accessibility testing — screen reader compatibility, keyboard-only navigation
- [ ] Mobile testing — tablet and phone layouts on real devices
- [ ] Video stream testing — MJPEG, RTSP via WebRTC proxy, HLS

#### Board Support
- [ ] Test auto-generated TOML configs against real boards
- [ ] Write new board TOMLs for controllers not in ArduPilot
- [ ] Validate pin mappings, timer assignments, DMA channels
- [ ] Test DShot, I2C, SPI on popular F4/F7/H7 boards

#### Documentation
- [ ] Getting started guides for specific boards
- [ ] SITL setup tutorials for Windows/Mac/Linux
- [ ] GCS feature walkthroughs
- [ ] Architecture deep-dives for contributors
- [ ] Video tutorials

#### Testing & CI
- [ ] Expand the test suite (every crate should have comprehensive tests)
- [ ] SITL automated test scenarios (takeoff, mission, RTL, failsafe)
- [ ] GCS unit tests (MAVLink codec, MNP codec, validator)
- [ ] Performance benchmarks (EKF update time, control loop latency)

### 4. Spread the Word

If Meridian resonates with you — if you're excited about Rust-native flight control and a browser-based GCS — share it. Blog about it. Demo it at your local drone meetup. The more people who know about it, the faster it grows.

---

## What Makes a Good PR

1. **Solves a real problem.** Not speculative features — things people actually need.
2. **Tested.** `cargo test` passes. GCS loads. Hardware was used if applicable.
3. **Minimal.** One change per PR. Easy to review, easy to revert.
4. **Documented.** If you added a feature, update the README or add a doc.
5. **No build step.** The GCS must remain zero-dependency. No npm. No bundler. No transpiler. Open `index.html` and it works.

We review every PR. We give constructive feedback. We merge good work fast.

---

## Architecture for Contributors

### Rust Crates

Each crate is independent. You can work on `meridian-drivers` without understanding `meridian-ekf`. Each has its own `Cargo.toml`, its own tests, its own README (coming soon).

```bash
# Work on just one crate
cd crates/meridian-drivers
cargo test
cargo check --no-default-features  # verify no_std compatibility
```

### GCS

The GCS is vanilla JavaScript with zero dependencies. Every file is an IIFE (Immediately Invoked Function Expression). State flows through `meridian.events` pub/sub. Views are panel-based via `router.js`.

```bash
# Serve and develop
cd gcs
python -m http.server 8080
# Edit any file, refresh browser
```

### Board Definitions

Board support is TOML-based. Adding a new board is adding a file — no Rust code changes needed.

```toml
# boards/MyBoard.toml
[board]
name = "My Custom Board"
board_id = 999
mcu = "STM32H743"

[imu.primary]
driver = "icm42688"
bus = "spi1"
cs_pin = "PE4"
drdy_pin = "PB0"

# ... pins, UARTs, timers, DMA channels
```

---

## Current Board Support

| Tier | Count | Status | What it means |
|------|-------|--------|---------------|
| **Tier 1** | 5 | Flight-tested | Full CI, sensor validation, recommended for first flights |
| **Tier 2** | 19 | Build-tested | Popular boards, TOML validated, awaiting hardware test |
| **Tier 3** | 359 | Auto-generated | Converted from ArduPilot hwdef.dat, need community validation |

**Total: 383 boards.**

If your board is in Tier 3, you can help promote it to Tier 2 by testing the TOML config on real hardware and filing a PR with corrections.

---

## Code of Conduct

Be excellent to each other. We're all here because we love drones and want better tools. Constructive criticism is welcome. Personal attacks are not. We don't gatekeep — if you can write code, solder boards, design UIs, write docs, or test in the field, you belong here.

---

## Contact

- **Issues:** https://github.com/jeranaias/meridian/issues
- **Discussions:** https://github.com/jeranaias/meridian/discussions (coming soon)
- **PRs:** https://github.com/jeranaias/meridian/pulls

---

*Meridian is built by the community, for the community. Every contribution matters. Let's build the future of autonomous flight together.*
