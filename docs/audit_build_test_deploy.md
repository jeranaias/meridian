# ArduPilot Build System, Testing, CI/CD, and Deployment Audit

**Source**: D:\projects\ardupilot  
**Date**: 2026-04-02  
**Auditor**: Automated audit

---

## 1. Build System (WAF)

### How WAF Works

ArduPilot uses WAF (waf-based build system), a Python build tool driven by a root `wscript` and a suite of tools under `Tools/ardupilotwaf/`. The core build flow for a real board is:

```
./waf configure --board MatekH743
./waf build --target bin/arducopter
```

1. **Configure phase** (`wscript:options` + `boards.py:Board.configure()`): WAF detects the board name, resolves the `hwdef.dat` from `libraries/AP_HAL_ChibiOS/hwdef/<BOARD>/hwdef.dat`, runs `chibios_hwdef.py` to parse the hardware definition, sets compiler flags, defines, toolchain, and writes a persistent config cache.

2. **Build phase** (`chibios.py:chibios_firmware()`): Compiles all C++, links the ELF, then runs three artifact generation tasks in sequence:
   - `generate_bin` — `objcopy -O binary` → `.bin` (raw flash image, gap-filled with 0xFF)
   - `generate_apj` — wraps the binary into the `.apj` JSON container
   - `build_intel_hex` — generates `.hex` for DFU upload

### Board Discovery

Boards are **not** statically registered. `boards.py:add_dynamic_boards_chibios()` walks `libraries/AP_HAL_ChibiOS/hwdef/` at configure time: any subdirectory containing a `hwdef.dat` or `hwdef-bl.dat` file becomes a valid board target. This is why there are 430 ChibiOS board definitions without a single line of Python per board.

Each board resolves to a subclass of `chibios` (the base ChibiOS board class), which inherits `Board`. The `Board.configure()` method:
- Loads the toolchain (ARM GCC cross-compiler, typically `g++-10.2.1`)
- Runs `cxx_checks.py` to probe for `std::isfinite`, `std::isinf`, `std::isnan` and namespace handling
- Always enables `AP_Scripting` and Lua (unless explicitly disabled)
- Embeds Lua scripts from `ROMFS/scripts/` into a ROMFS C header at build time

### Build Variants and Multi-Vehicle

All vehicles build from the **same source tree** — the differentiation is the top-level wscript target:

| Target | Binary |
|--------|--------|
| `bin/arducopter` | Copter |
| `bin/arduplane` | Plane |
| `bin/ardurover` | Rover |
| `bin/ardusub` | Sub |
| `bin/antennatracker` | Antenna Tracker |
| `bin/AP_Periph` | CAN peripheral nodes |
| `bin/blimp` | Blimp |

There are no separate branches or subdirectories per vehicle at the source level. Vehicle type is selected at link time via the top-level `wscript` for each vehicle directory (e.g., `ArduCopter/wscript`).

### Key Build Options

The `wscript` exposes ~30 `--enable-*`/`--disable-*` flags parsed at configure time. Notable ones:
- `--bootloader` — switches hwdef source to `hwdef-bl.dat` instead of `hwdef.dat`
- `--signed-fw` + `--private-key` — enables firmware signing
- `--default-parameters <file>` — embeds a parameter defaults file into the APJ at build time (patched into firmware via `apj_tool.py:embedded_defaults`)
- `--disable-scripting` / `--enable-scripting` — Lua scripting engine
- `--enable-opendroneid` — OpenDroneID (Remote ID) support
- `--coverage` — gcov coverage instrumentation

### Compiler Checks (`cxx_checks.py`)

WAF probes the cross-compiler for:
- `std::isfinite`, `std::isinf`, `std::isnan` in `<cmath>`
- Whether these functions need `using std::xxx` to avoid GCC namespace strictness errors
- Results are written to `ap_config.h`

Default CXX warning flags are aggressive: `-Wall -Wextra -Werror=format -Werror=shadow -Werror=return-type -Werror=unused-result -Werror=unused-variable -Werror=narrowing -Werror=undef`. ChibiOS builds enforce `-fsingle-precision-constant` to prevent accidental promotion of float literals to double.

---

## 2. Bootloader

### Architecture

Source: `Tools/AP_Bootloader/AP_Bootloader.cpp`

The bootloader is a standalone ChibiOS application built with `./waf configure --board <BOARD> --bootloader && ./waf bootloader`. It uses the board's `hwdef-bl.dat` (not `hwdef.dat`) and targets a 16 KB (F4) or larger allocation at the **start** of flash.

Memory layout (typical STM32H743 example):
- `0x08000000` — Bootloader (16–32 KB depending on MCU)
- `0x08008000` — Application start (offset configured by `APP_START_OFFSET_KB` in hwdef)
- External flash (`0x90000000`) — Optional, for boards with QSPI flash

### Boot Sequence

```
flash_init()
 ├─ Check ECC errors (STM32H7)
 ├─ Check watchdog reset → fast boot
 ├─ Check RTC magic for fast reboot / CAN bootload mode
 ├─ check_good_firmware() → CRC + board_id check
 ├─ try_boot=true → jump_to_app() immediately
 └─ timeout=5000ms (HAL_BOOTLOADER_TIMEOUT default)
     ├─ USB/UART bootloader protocol loop
     └─ After timeout → jump_to_app()
```

For CAN-connected peripherals, there is a separate CAN-DFU path (`can_start()` / `can_update()`) that does not use the USB/UART protocol at all.

A **network bootloader** path exists for Ethernet-capable boards (`AP_BOOTLOADER_NETWORK_ENABLED`), served via a tiny HTTP server with `Tools/AP_Bootloader/Web/index.html` as the UI.

An **SD card flash** path exists (`flash_from_sd()`) for boards that can load firmware from a microSD slot without any host connection.

### Protocol

Source: `Tools/AP_Bootloader/bl_protocol.cpp`

The bootloader implements the **PX4/Pixhawk bootloader protocol v5**, originally from the PX4 project, ported to ChibiOS. This is a custom binary protocol over USB CDC ACM or UART. The sequence is:

```
GET_SYNC  (0x21)  → verify board present
GET_DEVICE (0x22) → get board_id, board_rev, fw_size
CHIP_ERASE (0x23) → erase application flash
loop:
  PROG_MULTI (0x27) → write up to 64 bytes at program address
GET_CRC   (0x29)  → verify CRC over entire flash area
RESET     (0x30)  → finalize, reset, jump to app
```

External flash (QSPI) has its own parallel protocol commands (`PROTO_EXTF_*`, opcodes 0x34–0x37).

The upload tool on the host side is `Tools/scripts/uploader.py` (invoked by `waf upload`). For firmware ID safety, `AP_CheckFirmware` validates board_id against the APJ header before allowing a flash operation.

### Board IDs

Every board has an `APJ_BOARD_ID` (numeric ID) declared in its `hwdef.dat`. The authoritative list is `Tools/AP_Bootloader/board_types.txt`. This ID is embedded in both the bootloader and the APJ firmware package, allowing the bootloader and GCS tools to reject firmware built for a different board.

### Pre-built Bootloaders

ArduPilot ships **pre-compiled bootloader binaries** in `Tools/bootloaders/` — currently 751 files (`.bin` + `.hex` pairs for ~375 boards). This means users never need to build the bootloader themselves for mainstream hardware. The build server regenerates these whenever the bootloader source changes.

---

## 3. Firmware Distribution (APJ Format)

### The APJ Container

Source: `Tools/ardupilotwaf/chibios.py:generate_apj` task

`.apj` is a **JSON file** with the binary payload embedded as zlib-compressed, base64-encoded data. Full field list:

```json
{
  "board_id": 140,
  "magic": "APJFWv1",
  "description": "Firmware for a MatekH743 board",
  "image": "<base64(zlib(binary))>",
  "extf_image": "<base64(zlib(extflash_binary))>",
  "summary": "MatekH743",
  "version": "0.1",
  "image_size": 1234567,
  "extf_image_size": 0,
  "flash_total": 2097152,
  "image_maxsize": 2097152,
  "flash_free": 862345,
  "git_identity": "a1b2c3d4",
  "board_revision": 0,
  "USBID": ["0x3162/0x004B"],
  "manufacturer": "Matek Systems",
  "brand_name": "Matek H743",
  "build_time": 1711234567,
  "signed_firmware": false
}
```

The `apj_tool.py` script can open an APJ, extract the binary, patch in embedded default parameters, and repack. This allows downstream integrators to ship firmware with custom defaults without rebuilding.

### Additional Firmware Formats

| Format | Use |
|--------|-----|
| `.bin` | Raw binary, used with DroneCAN/UAVCAN OTA, direct ST-Link flash |
| `.hex` | Intel HEX, used with DFU tools (`dfu-util`) |
| `.abin` | SkyViper-specific with metadata headers prepended |
| `.elf` | Full ELF with symbols, used with GDB/OpenOCD |

### Firmware Server and Manifest

Distribution is via `firmware.ardupilot.org`. The `Tools/scripts/build_binaries.py` script builds all vehicles for all boards and organizes them into a directory tree:

```
binaries/
  Copter/
    stable/
      MatekH743/
        arducopter.apj
        arducopter.bin
        git-version.txt
    latest/
    beta/
  Plane/
  Rover/
  ...
```

`Tools/scripts/generate_manifest.py` scans this tree and generates a `manifest.json` that describes every available firmware with metadata including:
- `platform` (board name)
- `vehicletype` (Copter, Plane, etc.)
- `frame` (mapped to MAVLink frame type: QUADROTOR, FIXED_WING, etc.)
- `release-type` (OFFICIAL/BETA/DEV)
- `firmware-version` and `git_sha`
- Download URL

GCS tools (Mission Planner, QGroundControl) fetch this manifest to populate their firmware selection UI. Board identification happens via board_id: when a FC is connected over USB, the GCS reads the board_id from the bootloader and filters the manifest to matching boards.

---

## 4. SITL Testing

### sim_vehicle.py

Source: `Tools/autotest/sim_vehicle.py`

This is the entry point for all SITL sessions. It:
1. Builds the SITL binary (or uses a pre-built one)
2. Launches the simulator backend (JSBSim for fixed-wing, built-in physics for copter/rover)
3. Launches the SITL binary with `--model <frame>`, `--speedup <n>`, and physics UDP ports
4. Launches MAVProxy as the GCS bridge
5. Optionally launches FlightGear for 3D visualization

SITL connects to GCS tools over TCP/UDP:
- `tcp:127.0.0.1:5760` — primary MAVLink connection (GCS)
- `udp:127.0.0.1:5501` — SITL physics (internal)
- MAVProxy proxies these to external GCS tools

### Test Framework Architecture

Source: `Tools/autotest/vehicle_test_suite.py` (base class), vehicle-specific files like `arducopter.py`

The autotest framework is a Python test harness built on `pymavlink`. Each test:
- Connects to SITL via MAVLink
- Sends commands, waits for responses with timeout
- Raises `NotAchievedException`, `AutoTestTimeoutException`, etc. on failure

The `arducopter.py` file alone is **16,496 lines** and contains the full test suite for ArduCopter. It is split into test groups: `tests1a`, `tests1b`, `tests1c`, `tests1d`, `tests1e`, `tests2a`, `tests2b` — each containing dozens of individual tests.

### Test Coverage (ArduCopter examples)

The test suite covers essentially every major flight mode and subsystem:

- **Flight modes**: STABILIZE, ALT_HOLD, LOITER, AUTO, GUIDED, CIRCLE, DRIFT, POSHOLD, BRAKE, RTL, SMART_RTL, LAND, AUTOTUNE, ACRO, THROW, FLIP
- **Failsafes**: RC failsafe, battery failsafe, GPS failsafe, geofence, terrain failsafe
- **Navigation**: waypoint following, spline waypoints, terrain following, rally points
- **Sensors**: GPS, rangefinder, optical flow, beacon positioning, external odometry
- **Advanced**: EKF3 lane switching, FFT notch filters, dynamic notch, gyro FFT harmonic
- **Scripting**: Lua script loading and execution tests
- **CAN**: DroneCAN GPS mission, log download over CAN
- **OpenDroneID / UTM**: Remote ID broadcast validation
- **Logging**: DataFlash erase, log replay

Planes, Rovers, Subs, and HeliCopters each have similarly comprehensive suites.

### Test Speedup

SITL runs at **100x speedup** by default (`default_speedup = 100`), compressing hours of flight into seconds of real time. Long endurance tests that would take 30 minutes real-time run in 18 seconds.

---

## 5. CI/CD Pipeline

Source: `.github/workflows/`

### Workflow Files

| File | Purpose |
|------|---------|
| `test_sitl_copter.yml` | SITL build + autotest for ArduCopter |
| `test_sitl_plane.yml` | Same for Plane |
| `test_sitl_rover.yml` | Same for Rover |
| `test_sitl_sub.yml` | Same for Sub |
| `test_sitl_blimp.yml` | Same for Blimp |
| `test_sitl_periph.yml` | Same for AP_Periph |
| `test_sitl_tracker.yml` | Same for AntennaTracker |
| `test_chibios.yml` | ChibiOS hardware builds |
| `test_unit_tests.yml` | Unit tests + SITL build check |
| `test_scripting.yml` | Lua scripting tests |
| `test_coverage.yml` | Code coverage measurement |
| `test_replay.yml` | Log replay validation |
| `test_dds.yml` | DDS/ROS2 integration |
| `test_size.yml` | Firmware size regression guard |
| `test_scripts.yml` | Python script linting |
| `pre-commit.yml` | Pre-commit hook checks |
| `cygwin_build.yml` | Windows (Cygwin) build |
| `esp32_build.yml` | ESP32 target build |
| `macos_build.yml` | macOS build |

### SITL Copter CI Pipeline (representative)

**Step 1 — Build** (parallel: GCC + Clang):
```
./waf configure --board sitl --debug
./waf build --target bin/arducopter
```
Uses `ardupilot/ardupilot-dev-base:v0.1.3` Docker container. ccache is restored from GitHub Actions cache.

**Step 2 — Autotest** (7 parallel jobs):
`sitltest-copter-tests1a`, `1b`, `1c`, `1d`, `1e`, `2a`, `2b`

Each job runs on Ubuntu 22.04 with `--privileged` (needed for ptrace/SysV shared memory). A job failure does not cancel sibling jobs (`fail-fast: false`).

### ChibiOS Hardware Build Matrix

The `test_chibios.yml` builds the following targets in CI on every PR:

| Config | Description |
|--------|-------------|
| `stm32f7` | STM32F7 family firmware |
| `stm32h7` | STM32H7 family firmware |
| `fmuv2-plane` | FMUv2/Pixhawk1 Plane |
| `periph-build` | CAN peripheral nodes |
| `iofirmware` | IO co-processor firmware |
| `CubeOrange-bootloader` | CubeOrange bootloader |
| `fmuv3-bootloader` | Pixhawk 3 bootloader |
| `revo-bootloader` | OpenPilot Revolution bootloader |
| `stm32h7-debug` | H7 debug build |
| `MatekF405-Wing` | Popular budget FC |
| `CubeOrange-ODID` | OpenDroneID build |
| `signing` | Firmware signing verification |
| `configure-all` | Validates all 430 boards configure without error |
| `build-options-defaults-test` | Tests all `--enable-*` combos |

### Unit Tests

`test_unit_tests.yml` runs three parallel configs × two toolchains (GCC + Clang):

1. `unit-tests` — C++ unit tests under `tests/` via gtest
2. `examples` — Builds all example programs
3. `sitl` — SITL target builds cleanly

Python unittests also run for `extract_param_defaults` and `annotate_params`.

### CI Runtime Estimate

Based on the structure: each vehicle's SITL CI across 7 parallel test jobs on a fresh container takes roughly **20–40 minutes**. The full matrix (all vehicles + ChibiOS builds) would take ~60–90 minutes wall-clock with GitHub Actions' parallelism.

---

## 6. Lua Applets

Source: `libraries/AP_Scripting/applets/`

ArduPilot ships **~52 Lua applets** (counted as `.lua` files in the applets directory). These can be embedded in firmware at build time via `--embed-<name>` flags, or loaded at runtime from SD card/ROMFS.

### Complete Applet List

| Applet | Description |
|--------|-------------|
| `Aerobatics/` | Aerobatic flight mode scripting (subdirectory) |
| `BattEstimate.lua` | Battery capacity estimator using discharge curves |
| `BatteryTag.lua` | Battery tagging/tracking by serial number |
| `CAN_playback.lua` | Replay logged CAN frames for testing |
| `Gimbal_Camera_Mode.lua` | Automatic gimbal mode based on camera state |
| `Heli_IM_COL_Tune.lua` | Heli internal model collective tuning |
| `Heli_idle_control.lua` | Helicopter idle throttle control |
| `Hexsoon LEDs.lua` | LED control for Hexsoon-branded FC |
| `MissionSelector.lua` | Select missions from SD card via RC switch |
| `ONVIF_Camera_Control.lua` | ONVIF IP camera control (PTZ, focus) |
| `Param_Controller.lua` | Dynamic parameter control via flight modes |
| `QuadPlane_Low_Alt_FW_mode_prevention.lua` | Block fixed-wing mode below safe altitude on VTOL |
| `RockBlock-9704.lua` | RockBlock 9704 satellite modem driver |
| `RockBlock.lua` | RockBlock satellite modem (original) |
| `Script_Controller.lua` | Multi-script management framework |
| `SmartAudio.lua` | VTX SmartAudio power control |
| `UniversalAutoLand.lua` | Custom autoland logic for any vehicle type |
| `VTOL-quicktune.lua` | One-click VTOL parameter autotuning |
| `advance-wp.lua` | Advance to next waypoint via RC |
| `ahrs-set-origin.lua` | Set EKF origin from external source |
| `ahrs-source-extnav-optflow.lua` | Switch AHRS source between ExternalNav and OptFlow |
| `arming-checks.lua` | Custom pre-arm check framework |
| `camera-change-setting.lua` | Adjust camera settings via RC input |
| `copter-deadreckon-home.lua` | GPS-denied dead-reckoning home return |
| `copter-slung-payload.lua` | Slung payload oscillation damping |
| `copter_terrain_brake.lua` | Terrain-aware braking for copter |
| `crsf-calibrate.lua` | CRSF RC calibration helper |
| `follow-target-send.lua` | Broadcast own position for Follow mode from companion |
| `forward_flight_motor_shutdown.lua` | Shut down rear motors in VTOL forward flight |
| `leds_on_a_switch.lua` | Toggle LEDs via RC channel |
| `motor_failure_test.lua` | Simulate motor failure for testing |
| `mount-poi.lua` | Point gimbal at point of interest |
| `net-ntrip.lua` | NTRIP RTK corrections over network |
| `net_webserver.lua` | Embedded web server for status/config |
| `param-lockdown.lua` | Lock parameter changes in flight |
| `pelco_d_antennatracker.lua` | Pelco-D protocol for pan-tilt camera trackers |
| `plane_follow.lua` | Plane follow mode enhancements |
| `plane_package_place.lua` | Delivery drop logic for Plane |
| `plane_precland.lua` | Precision landing for Plane |
| `plane_ship_landing.lua` | Ship/moving platform landing logic |
| `quadplane_terrain_avoid.lua` | Terrain avoidance in QuadPlane VTOL transition |
| `repl.lua` | Interactive Lua REPL over serial |
| `revert_param.lua` | Revert parameters to defaults after timeout |
| `rover-quicktune.lua` | One-click rover tuning |
| `runcam_on_arm.lua` | Start RunCam recording on arm |
| `throttle_kill.lua` | Emergency throttle cut via RC |
| `video-stream-information.lua` | Broadcast video stream URLs via MAVLink |
| `winch-control.lua` | Winch/cable controller |
| `x-quad-cg-allocation.lua` | Dynamic CG-corrected motor allocation for X-quad |
| `WebExamples/` | Web UI example scripts |

### Functionality Applets Provide Beyond Core Firmware

- **Hardware drivers** that don't warrant permanent C++ inclusion (RockBlock modem, Pelco-D pan-tilt heads)
- **Experimental flight logic** (ship landing, dead reckoning, slung payload) that is too niche or not safety-certified for core
- **System integrations** (ONVIF cameras, NTRIP corrections) requiring network access
- **Tuning automation** (VTOL-quicktune, rover-quicktune) — users run these once then remove them
- **Operational procedures** (param-lockdown, revert-param, mission selector) specific to operational contexts

### Meridian's Approach to Lua

Meridian currently skips Lua entirely. The correct long-term approach depends on the use case:
- For **driver scripts and integrations**: Rust plugins (via a stable FFI or trait interface) are safer and faster
- For **field-configurable logic** (the real use case for most applets): a WASM sandbox would provide the same runtime-loadable, sandboxed execution model without a Lua dependency
- The **highest-value applets to replicate in Rust first** are: `VTOL-quicktune`, `copter-deadreckon-home`, `UniversalAutoLand`, and `arming-checks` (custom pre-arm checks)

---

## 7. Parameter Defaults System

### Embedded Defaults

The `--default-parameters <file>` WAF option calls `apj_tool.py:embedded_defaults` to find a magic sentinel region in the firmware binary where parameter defaults are stored, and overwrites it with the provided file content. This happens **after compilation** — the same compiled binary can have different embedded defaults for different frame configurations.

The sentinel region is a fixed-size area (currently 8192 bytes) that the bootloader/firmware checks at startup. If valid parameter data is present, those values override compiled-in defaults. This allows a single firmware binary to support multiple frame variants.

### Frame Parameter Files

Source: `Tools/Frame_params/`

ArduPilot ships **~30 frame-specific parameter sets** for popular vehicles:

- `3DR_Iris+.param`, `Solo-Copter-GreenCube.param` — Legacy 3DR vehicles
- `Holybro-S500.param`, `Holybro-X500v2-bdshot.param`, `Holybro-QAV250.param` — Holybro kits
- `Hexsoon-edu450.param`, `Hexsoon-edu650.param`, `Hexsoon-td860.param` — Hexsoon EDU series
- `WLToys_V383_HeliQuad.param` — Toy-class helicopter conversions
- `EFlight_Convergence.param` — VTOL tilt-rotor
- `Parrot_Bebop.param`, `Parrot_Bebop2.param` — Consumer drone ports
- Various boat, rover, and QuadPlane configs under subdirectories
- `ModalAI/` and `XPlane/` subdirectories for specific platforms

These `.param` files are human-readable text: `PARAM_NAME VALUE` per line. The build system, QGC, and Mission Planner all understand this format.

---

## 8. Documentation System

### Parameter Documentation

Source: `Tools/autotest/param_metadata/param_parse.py`

ArduPilot uses **structured comments in C++ source** to generate parameter documentation. Every `AP_PARAM_*` declaration in library code is accompanied by `// @Param:`, `// @Description:`, `// @Range:`, `// @Units:`, `// @User:` comment annotations. The `param_parse.py` script walks all source files, extracts these annotations, and generates output in multiple formats:
- **HTML** — for the ArduPilot wiki
- **XML** — for Mission Planner parameter loading
- **JSON** — for QGC
- **RST** — for Sphinx documentation
- **Markdown**

### Log Message Documentation

Source: `Tools/autotest/logger_metadata/parse.py`

Log messages are documented via `// @LoggerMessage:`, `// @Description:`, `// @Field:` comments adjacent to `AP_Logger::Write()` calls. The parse script scans source for these annotations and generates HTML/XML/RST/Markdown documentation of every log message.

### Build Documentation Scripts

- `Tools/scripts/build_parameters.sh` — runs `param_parse.py` for all vehicles
- `Tools/scripts/build_log_message_documentation.sh` — runs logger metadata parse
- `Tools/scripts/build_docs.sh` — builds full Sphinx docs
- `Tools/scripts/generate_lua_docs.sh` — generates Lua API documentation

---

## 9. What Meridian Needs

### Build System

**Cargo workspace is sufficient** for Meridian's needs. No WAF equivalent is needed. The key gaps are:

1. **Board-specific feature flags**: Cargo's `features` system handles this cleanly. Each supported board gets a feature flag (e.g., `board-cube-orange`, `board-matek-h743`). Consider a `build.rs` that reads a board HAL config file analogous to `hwdef.dat`.

2. **Embedded default parameters**: Meridian needs a mechanism to embed parameter defaults into the binary at build time. A `build.rs` script can generate a Rust `include_bytes!()` or a `static` array from a `.param` file, placed at a known linker section. This replicates `apj_tool.py` behavior.

3. **ROMFS / embedded files**: Lua applets are embedded via `AP_ROMFS`. Meridian's equivalent (config files, geofence data, default missions) can use `include_bytes!()` in a dedicated crate.

### Firmware Packaging

Meridian should define its own firmware container format. Recommended approach:

```
MFW (Meridian Firmware) = JSON envelope + zlib+base64 binary
{
  "magic": "MFWv1",
  "board_id": <numeric>,
  "board_name": "MatekH743",
  "vehicle": "copter",
  "version": "0.1.0",
  "git_sha": "a1b2c3d",
  "image": "<base64(zlib(binary))>",
  "image_size": <bytes>,
  "flash_total": <bytes>,
  "build_time": <unix_timestamp>
}
```

This is essentially the APJ format with different field names. It keeps compatibility with the same tooling pattern (Python script to pack/unpack) while being Meridian-native.

For **initial deployment**, use `.bin` directly with ST-Link/OpenOCD — this removes the packaging requirement until a GCS ecosystem exists.

### Test Infrastructure

Beyond `cargo test`, Meridian needs:

1. **SITL harness**: A Rust SITL target that links against a 3D physics model. Options:
   - Embed a minimal physics engine (rigid body + rotor model) as a Rust crate
   - Interface to JSBSim via its C++ API or via UDP (ArduPilot's pattern)
   - Use the `gazebo`/`ROS2` ecosystem (overkill for initial testing)

   **Recommended**: Start with a pure-Rust minimal quadrotor physics model for basic SITL. This eliminates the JSBSim dependency and runs at 1000x speedup easily.

2. **MAVLink test harness**: A Rust test client that connects to the SITL over MAVLink, sends commands, and asserts on vehicle state. The `mavlink` crate exists for this. Model after ArduPilot's `vehicle_test_suite.py`.

3. **Hardware-in-the-loop (HIL)**: Not needed initially, but eventually want the ability to run firmware on a real FC with simulated sensors (the `--sim` flag equivalent).

4. **Parameter regression tests**: A test that boots SITL, checks all parameter defaults, and fails if they change unexpectedly.

### Firmware Distribution for 430 Boards

This is the hardest scaling problem. ArduPilot's 430-board support comes from:
1. Per-board `hwdef.dat` files (board HAL configuration)
2. `add_dynamic_boards_chibios()` auto-discovery
3. Pre-built bootloaders checked into the repo
4. `build_binaries.py` running on a dedicated build server

**For Meridian, the realistic path:**
1. **Phase 1 (now)**: Support 1–5 boards explicitly. Ship `.bin` files.
2. **Phase 2 (mid-term)**: Define a Meridian `hwdef.toml` format analogous to ArduPilot's `hwdef.dat`, use a `build.rs`-based auto-discovery system.
3. **Phase 3 (long-term)**: Host a firmware server (could be GitHub Releases + a manifest JSON) and add GCS integration. The manifest format is straightforward JSON — no firmware.ardupilot.org equivalent is needed immediately.

### Bootloader Options

Three choices:

**Option A — Use ArduPilot's bootloader (recommended short-term)**

ArduPilot's bootloader is MIT-licensed (PX4 origin) and already supports 375+ boards with pre-built binaries in `Tools/bootloaders/`. The bootloader protocol is well-documented (bl_protocol.h). Meridian firmware could be flashed by the same `uploader.py` tool. The constraint: the bootloader validates board_id. Meridian must either:
- Match ArduPilot's board_id assignment in `board_types.txt`
- Or build with `--enable-check-firmware=0` equivalent (disable board_id check)

The board_id check can be disabled in the bootloader. This is the fastest path to hardware deployment.

**Option B — STM32 DFU bootloader (simplest)**

STM32's built-in ROM bootloader (at `0x1FF00000` or equivalent) accepts `.hex` via USB DFU using `dfu-util`. No custom bootloader needed. Limitation: requires holding BOOT0 pin to enter DFU mode. No OTA, no CAN update.

**Option C — Write a Meridian bootloader in Rust**

Long-term correct answer for a production autopilot. Implement the same PX4 bootloader protocol for compatibility with existing GCS upload tools. This is a ~2000 line Rust project. Not justified until Meridian has several users on real hardware.

**Recommendation**: Use ArduPilot's bootloader binaries from `Tools/bootloaders/` for Phase 1/2. Write a Meridian-native Rust bootloader as a Phase 3 project once the firmware is stable.

---

## Summary Table: ArduPilot vs. Meridian Parity

| Capability | ArduPilot | Meridian Today | Priority |
|------------|-----------|----------------|----------|
| Build system | WAF + Python | Cargo workspace | Done |
| Board auto-discovery | hwdef.dat scan | Manual features | Medium |
| Firmware packaging | APJ (JSON+zlib) | Raw .bin | Medium |
| Parameter embed at build | apj_tool.py | Not yet | High |
| Bootloader | Custom ChibiOS, 375 boards | Use ArduPilot's | High |
| SITL | JSBSim + Python harness | Not yet | High |
| MAVLink test harness | vehicle_test_suite.py | Not yet | High |
| CI/CD | GitHub Actions 25 workflows | Not yet | Medium |
| Firmware distribution | firmware.ardupilot.org + manifest.json | GitHub Releases | Low |
| GCS integration | Mission Planner + QGC | None | Low |
| Lua scripting | 52+ applets | Skip/WASM later | Low |
| Param documentation | Annotated C++ → XML/HTML | Not yet | Low |
| Log documentation | @LoggerMessage annotations | Not yet | Low |
| Frame parameter sets | ~30 .param files | Not yet | Medium |

---

*Files read for this audit:*
- `D:\projects\ardupilot\wscript`
- `D:\projects\ardupilot\Tools\ardupilotwaf\boards.py`
- `D:\projects\ardupilot\Tools\ardupilotwaf\cxx_checks.py`
- `D:\projects\ardupilot\Tools\ardupilotwaf\chibios.py`
- `D:\projects\ardupilot\Tools\ardupilotwaf\ardupilotwaf.py`
- `D:\projects\ardupilot\Tools\AP_Bootloader\AP_Bootloader.cpp`
- `D:\projects\ardupilot\Tools\AP_Bootloader\bl_protocol.cpp`
- `D:\projects\ardupilot\Tools\AP_Bootloader\bl_protocol.h`
- `D:\projects\ardupilot\Tools\AP_Bootloader\README.md`
- `D:\projects\ardupilot\Tools\AP_Bootloader\Web\index.html`
- `D:\projects\ardupilot\Tools\autotest\sim_vehicle.py`
- `D:\projects\ardupilot\Tools\autotest\arducopter.py`
- `D:\projects\ardupilot\Tools\autotest\vehicle_test_suite.py`
- `D:\projects\ardupilot\Tools\autotest\param_metadata\param_parse.py`
- `D:\projects\ardupilot\Tools\autotest\logger_metadata\parse.py`
- `D:\projects\ardupilot\.github\workflows\test_sitl_copter.yml`
- `D:\projects\ardupilot\.github\workflows\test_chibios.yml`
- `D:\projects\ardupilot\.github\workflows\test_unit_tests.yml`
- `D:\projects\ardupilot\Tools\scripts\generate_manifest.py`
- `D:\projects\ardupilot\Tools\scripts\apj_tool.py`
- `D:\projects\ardupilot\Tools\scripts\build_binaries.py`
