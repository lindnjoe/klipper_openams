# OpenAMS for Klipper (AFC Integration Fork)

Version `0.0.3`

This repository provides the OpenAMS Klipper extras plus an AFC integration layer that maps OpenAMS hardware into Armored Turtle AFC lanes.

## What is in this repo

- Core OpenAMS Klipper modules (`src/*.py`) for:
  - OAMS CAN-bus hardware control
  - FPS pressure sensor handling
  - OAMS manager orchestration (load/unload/retry/runout/clog logic)
  - HDC1080 temperature/humidity support
  - Moonraker status persistence client
- AFC integration modules (`AFC_OpenAMS.py`, `openams_integration.py`) that are copied into AFC extras.
- Configuration templates:
  - `AFC_Oams.cfg` (OAMS, FPS, manager, sensor config)
  - `AFC_AMS_1.cfg` (AFC lane/hub mapping)
- Installer script (`install-openams.sh`) that symlinks OpenAMS extras/scripts into Klipper and appends moonraker/HDC1080 template entries.

---

## Current architecture (v0.0.3)

### OpenAMS modules (installed to `klippy/extras`)

- `oams.py` — OAMS hardware abstraction over CAN bus.
- `fps.py` — filament pressure sensor driver.
- `oams_manager.py` — central control logic for load/unload, retries, runout handling, and detection tunables.
- `hdc1080.py` — HDC1080 environmental sensor driver.
- `openams_moonraker.py` — writes manager state to Moonraker DB.

### AFC integration modules (installed to AFC extras)

- `AFC_OpenAMS.py` — integrates AFC lane workflows with OpenAMS manager and virtual sensors.
- `openams_integration.py` — shared event bus, lane registry, hardware service, and manager facade.

---

## Key implementation features currently present

- Lane-based AFC integration (`[AFC_lane ...]`) tied to OpenAMS unit/spool positions.
- Event-driven lane/sensor state propagation via shared integration helpers.
- Configurable runout reload margin globally (`[oams_manager]`) and per FPS (`[fps ...]`).
- Configurable clog and stuck-spool detection toggles and thresholds in `[oams_manager]`.
- Moonraker DB persistence through `openams_moonraker.py`.
- Optional Kalico mode support in FPS driver (`use_kalico`).

---

## Repository layout

```text
.
├── AFC_AMS_1.cfg
├── AFC_Oams.cfg
├── AFC_OpenAMS.py
├── openams_integration.py
├── install-openams.sh
├── scripts/
│   └── canbus_logger.py
├── src/
│   ├── fps.py
│   ├── hdc1080.py
│   ├── oams.py
│   ├── oams_manager.py
│   └── openams_moonraker.py
└── file_templates/
    ├── HDC1080.cfg
    ├── moonraker_update.txt
    └── openams.service
```

---

## Installation

## 1) Install/update AFC first

This fork assumes Armored Turtle AFC is already installed and working.

## 2) Install OpenAMS extras from this repo

From repo root:

```bash
chmod +x install-openams.sh
./install-openams.sh
```

Useful options:

```bash
./install-openams.sh -k <klipper_path> -s <klipper_service_name> -c <moonraker_config_path>
```

Uninstall:

```bash
./install-openams.sh -u
```

What the installer currently does:

- Verifies Klipper service and folder layout.
- Symlinks `src/*.py` into `~/klipper/klippy/extras/`.
- Symlinks `scripts/*.py` into `~/klipper/scripts/`.
- Appends update manager block from `file_templates/moonraker_update.txt` into `moonraker.conf`.
- Appends HDC1080 registration from `file_templates/HDC1080.cfg` into Klipper `temperature_sensors.cfg`.
- Restarts services as needed.

## 3) Place AFC integration files in AFC extras

Copy/sync these two files into your AFC extras directory:

- `AFC_OpenAMS.py`
- `openams_integration.py`

## 4) Add config sections

- Start from `AFC_Oams.cfg` for OAMS/FPS/manager sections.
- Start from `AFC_AMS_1.cfg` for lane/hub mapping.
- Include/adapt those sections in your printer config based on your hardware UUIDs, PTFE lengths, hubs, and tool mappings.

---

## Configuration highlights

### `[oams_manager]`

Commonly tuned options in this repo template:

- `reload_before_toolhead_distance`
- `clog_sensitivity`
- `enable_clog_detection`
- `enable_stuck_spool_detection`
- `debounce_delay`
- `stuck_spool_*` thresholds
- `clog_pressure_target`
- `post_load_pressure_dwell`
- `load_fps_stuck_threshold`
- `engagement_pressure_threshold`

### `[fps ...]`

- `pin`, `reversed`, `oams`, `extruder`
- Optional per-FPS override: `reload_before_toolhead_distance`
- Optional Kalico compatibility: `use_kalico`

### `[AFC_lane ...]`

Each lane maps one AFC lane to an OpenAMS unit slot (e.g. `unit: AMS_1:1`) and normal AFC lane properties (`hub`, `map`, etc.).

---

## Notes

- `OPENAMS_VERSION` is currently `0.0.3` in integration modules.
- `mainsail.zip` is included in the repo as an optional UI asset.
- This fork is focused on AFC lane integration and does not replace upstream AFC installation steps.

---

## License

MIT (see `LICENSE`).
