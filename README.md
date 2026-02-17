<img width="1024" height="1024" alt="OpenAMS" src="https://github.com/user-attachments/assets/7b515408-d0b3-437f-b0a4-8c7128d2e922" />

# OpenAMS for Klipper

> **Version 0.0.3** — Lane-based AFC integration with event-driven monitoring

A Klipper integration for OpenAMS that enables multi-material printing with automatic filament management, runout detection, and intelligent retry logic.

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Architecture](#architecture)
- [What's New](#whats-new)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Install/Update AFC](#1-installupdate-afc)
    - [Lane Architecture Primer](#lane-architecture-primer)
    - [Install the AFC Add-On](#install-the-afc-add-on)
    - [Stage AFC Configuration Templates](#stage-afc-configuration-templates)
    - [AFC Hardware Configuration Checklist](#afc-hardware-configuration-checklist)
  - [Install OpenAMS](#2-install-openams)
    - [Switching to This Fork](#switching-to-this-fork)
    - [Custom Installation Paths](#custom-installation-paths)
  - [Apply AFC Integration Files](#3-apply-afc-integration-files)
- [Configuration](#configuration)
  - [OpenAMS Manager Settings](#openams-manager-settings)
  - [OAMS Hardware Settings](#oams-hardware-settings)
  - [FPS Configuration](#fps-configuration)
  - [Retry Behavior](#retry-behavior)
  - [Clog Detection Settings](#clog-detection-settings)
  - [Advanced Detection Tunables](#advanced-detection-tunables)
- [Optional Features](#optional-features)
  - [Mainsail AFC Panel](#mainsail-afc-panel)
- [Initial Calibration](#initial-calibration)
- [Infinite Spooling](#infinite-spooling)
- [Troubleshooting](#troubleshooting)
  - [CAN Bus Debugging](#can-bus-debugging)
- [Credits](#credits)

## Overview

OpenAMS provides automated filament handling for Klipper-based 3D printers. This fork integrates tightly with Armored Turtle's AFC (Automatic Filament Changer) add-on using a **lane-based architecture**. The combination delivers end-to-end multi-material automation—from AFC's physical filament routing to OpenAMS' retry logic, runout handling, and print-state awareness.

### Full Integration at a Glance

- **AFC** exposes lanes, runout sensors, and hub LEDs.
- **OpenAMS** maps those AFC lanes into the AMS manager, applies retry and clog detection logic, and keeps Moonraker/Klipper informed of state changes.
- **Optional services** like Spoolman enrich the integration with live spool metadata.

### Example Settings

The snippets below show how the integration pieces fit together. Adjust lane names and MCU UUIDs to match your hardware.

<details>
<summary><strong>OpenAMS Manager</strong></summary>

```ini
[oams_manager]
# Optional: start loading replacement filament early
reload_before_toolhead_distance: 0.0

# Optional: lane-wide clog sensitivity (low/medium/high)
clog_sensitivity: medium

# Optional: enable/disable detection systems
enable_clog_detection: True
enable_stuck_spool_detection: True
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
