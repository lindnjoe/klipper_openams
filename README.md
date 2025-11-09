https://chatgpt.com/backend-api/estuary/content?id=file_00000000136c722f8107cec3fd13d05d&ts=489632&p=fs&cid=1&sig=05940d5fc3f42b78948de0ef2f8f3c640273ea7d0ff55594851047e21e59ea09&v=0

# OpenAMS for Klipper

A Klipper integration for OpenAMS that enables multi-material printing with automatic filament management, runout detection, and intelligent retry logic.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [First Time Installation](#first-time-installation)
  - [Switching to This Fork](#switching-to-this-fork)
  - [Custom Installation Paths](#custom-installation-paths)
- [Configuration](#configuration)
  - [OpenAMS Manager Settings](#openams-manager-settings)
  - [OAMS Hardware Settings](#oams-hardware-settings)
  - [Retry Behavior](#retry-behavior)
  - [Clog Detection Settings](#clog-detection-settings)
- [AFC Integration](#afc-integration)
  - [Installing AFC](#installing-afc)
  - [Configuration Files](#configuration-files)
  - [Smart Temperature Purge Macros](#smart-temperature-purge-macros)
  - [AFC Hardware Configuration](#afc-hardware-configuration)
- [Optional Features](#optional-features)
  - [Spoolman LED Sync](#spoolman-led-sync)
  - [Mainsail AFC Panel](#mainsail-afc-panel)
- [Initial Calibration](#initial-calibration)
- [Infinite Spooling](#infinite-spooling)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)

## Overview

OpenAMS provides automated filament handling for Klipper-based 3D printers. This fork integrates with Armored Turtle's AFC (Automatic Filament Changer) add-on to provide a complete multi-material printing solution.

**Key capabilities:**
- Automatic filament loading and unloading
- Intelligent retry logic for stuck filament detection
- Clog detection with configurable sensitivity
- Runout detection and automatic lane switching
- Spoolman integration for filament tracking
- LED status indicators with optional color sync

## Features

- **Automatic Retry Logic**: Configurable retry attempts for both load and unload operations with exponential backoff
- **Clog Detection**: Three sensitivity levels (low, medium, high) to detect filament clogs during printing
- **Runout Handling**: Automatic filament runout detection with configurable reload distance
- **Infinite Spooling**: Seamless lane switching for continuous printing without manual intervention
- **Temperature Management**: Smart purge temperature calculation for multi-material prints
- **LED Status Indicators**: Visual feedback with optional Spoolman color synchronization
- **HDC1080 Sensor Support**: Temperature and humidity monitoring within the AMS unit

## Prerequisites

Before installing OpenAMS, ensure you have:

1. **Klipper** installed and running
2. **Moonraker** configured
3. **MCU's id's** uuid's or serial id for FPS and AMS board
4. **Optional but recommended:**
   - Spoolman for filament tracking
   - Mainsail or Fluidd web interface

## Installation

### First Time Installation

If this is your first time installing OpenAMS, use the provided installation script:

```bash
cd ~
git clone https://github.com/lindnjoe/klipper_openams.git
cd klipper_openams
./install-openams.sh
```

The installation script will:
1. Link OpenAMS Python modules to Klipper's extras directory
2. Add HDC1080 temperature sensor support
3. Configure Moonraker update manager
4. Restart Klipper services

### Switching to This Fork

If you already have OpenAMS installed and want to switch to this fork:

```bash
cd ~/klipper_openams
git remote add lindnjoe https://github.com/lindnjoe/klipper_openams 2>/dev/null \
  || git remote set-url lindnjoe https://github.com/lindnjoe/klipper_openams
git fetch lindnjoe master
git checkout -B lindnjoe-master lindnjoe/master
./install-openams.sh
```

The `git checkout -B` command creates a local `lindnjoe-master` branch that tracks this repository, allowing easy updates with `git pull`.

### Custom Installation Paths

If your directory structure differs from the standard layout, configure the installation with additional parameters:

```bash
./install-openams.sh [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>]
```

**Parameters:**
- `-k` : Path to Klipper installation (default: `~/klipper`)
- `-s` : Klipper service name (default: `klipper`)
- `-c` : Configuration directory path (default: `~/printer_data/config`)

**Example:**
```bash
./install-openams.sh -k /home/pi/klipper -c /home/pi/printer_data/config
```

### Post-Installation File Copy *Don't do this now, do this after full installation is complete*

***********THE FOLLOWING STEP MUST BE DONE LAST TO UPDATE TO CURRENT WORKING VERSION - WILL REMOVE WHEN UPSTREAM UPDATED********
**Important:** After installation, copy the UPDATED AFC integration modules to your Klipper AFC add-on extras folder:

```bash
cp AFC_OpenAMS.py ~/Klipper-Add-On/extras/
cp openams_integration.py ~/Klipper-Add-On/extras/
```

*Note: This manual copy step will be removed once these modules are merged upstream into the AFC add-on.*

## Configuration

### OpenAMS Manager Settings

The `[oams_manager]` section controls global OpenAMS behavior.:

```ini
[oams_manager]
# Optional: Distance before toolhead to trigger reload during runout (default: 0.0)
# Set this to start loading the next filament before the current one fully runs out
# Useful for reducing print pauses during runout
reload_before_toolhead_distance: 0.0

# Optional: Clog detection sensitivity: low, medium, high (default: medium)
# Controls how aggressive the clog detection system is
clog_sensitivity: medium
```

**Configuration Tips:**
- **reload_before_toolhead_distance**: Set this to a positive value to load replacement spool sooner. Helpful with longer ptfe lengths and faster printing speeds. May require manual tuning to get just right for your printer.
- **clog_sensitivity**: Start with `medium`. Increase to `high` if clogs go undetected. Decrease to `low` if false positives occur.

### OAMS Hardware Settings

For each OAMS unit, configure retry behavior in your OAMS hardware configuration file (typically `AFC_Oams.cfg`):

```ini
[oams oams1]
mcu: oams_mcu1

# Retry Configuration (all optional - defaults shown in comments)
load_retry_max: 3              # Maximum number of load retry attempts (default: 3)
unload_retry_max: 2            # Maximum number of unload retry attempts (default: 2)
retry_backoff_base: 1.0        # Base delay in seconds between retries (default: 1.0)
retry_backoff_max: 5.0         # Maximum delay between retries (default: 5.0)
# auto_unload_on_failed_load: True  # Defaults to True - only set if you need False

# CAN bus UUID Or Serial Id if using USB
canbus_uuid: <your_unique_OAMS_MCU1_UUID>
serial: /dev/serial/by-id/

# Additional settings as needed...
```

**Important Configuration Notes:**

1. **MCU UUIDs**: You must update the `canbus_uuid or serial: /dev/serial/by-id/` values with your actual hardware UUIDs. Find them with:
   ```bash
   ~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
   or
   ls /dev/serial/by-id/*  
   ```

2. **Retry Settings**: The defaults work well for most setups, but you may need to adjust:
   - Increase `load_retry_max` if filament occasionally fails to load on first attempt
   - Increase `retry_backoff_base` if your hardware needs more recovery time
  

### Retry Behavior

The OpenAMS system includes automatic retry logic for both load and unload operations to handle temporary failures gracefully:

**Load Retries:**
- Default: 3 attempts with exponential backoff (1s, 2s, 4s delays)
- Monitors encoder movement during loading
- Automatically unloads and retries if filament gets stuck
- Only pauses the printer if all retry attempts fail

**Unload Retries:**
- Default: 2 attempts with exponential backoff (1s, 2s delays)
- Monitors encoder movement during unloading
- Aborts stuck operations and retries automatically
- Only pauses the printer if all retry attempts fail

**Backoff Calculation:**
```
delay = min(retry_backoff_base * attempt_number, retry_backoff_max)
```

**Example:** With `retry_backoff_base: 1.0` and `retry_backoff_max: 5.0`:
- Retry 1: 1 second delay
- Retry 2: 2 seconds delay
- Retry 3: 3 seconds delay
- Retry 4+: 5 seconds delay (capped at max)

### Clog Detection Settings

The `clog_sensitivity` setting in `[oams_manager]` controls how aggressive the clog detection is:

| Sensitivity | Observation Window | Tolerance | Best For |
|-------------|-------------------|-----------|----------|
| **low** | 48mm extrusion | More tolerant | Printers with flow variations, flexible materials |
| **medium** (default) | 24mm extrusion | Balanced | General use, most materials |
| **high** | 12mm extrusion | More sensitive | Quick clog detection, important prints |

**Tuning Tips:**
- Start with `medium` sensitivity
- If you get false clog detections (pauses when no clog exists), lower to `low`
- If clogs aren't detected quickly enough, increase to `high`
- Consider your material: flexible filaments may need `low` sensitivity

## AFC Integration

This OpenAMS fork is designed to work with Armored Turtle's AFC (Automatic Filament Changer) Klipper add-on.

### Installing AFC

Clone the Armored Turtle Repo

```bash
cd ~
git clone https://github.com/ArmoredTurtle/AFC-Klipper-Add-On.git
```

Install the AFC add-on from the multi_extruder branch:

```bash
cd ~/Klipper-Add-On
./install-afc.sh -b multi_extruder
```

**Important Setup Notes:**

1. **Read the AFC Documentation**: Most of the Armored Turtle setup documentation applies directly to OpenAMS. Review it at: https://www.armoredturtle.xyz/docs/boxturtle/initial_startup/01-overview.html

2. **Box Turtle vs OpenAMS**: Some documentation is specific to Box Turtle hardware and won't apply to OpenAMS. When in doubt:
   - Ask for clarification
   - Leave default settings in place
   - The OpenAMS-specific config files override Box Turtle defaults

3. **Installation Prompts**: The AFC installer doesn't list an OpenAMS-specific unit type. When prompted:
   - Choose the **Box Turtle** option
   - Accept the default settings
   - After installation, **delete** the generated `AFC_Turtle_1.cfg` file:
     ```bash
     rm ~/printer_data/config/AFC/AFC_Turtle_1.cfg
     ```
     You may also need to delete the AFC.var.unit file that is autogenerated on startup as it will have the wrong info and may cause issues with the system prepping on load. The system will regenerate this on next startup and will give an error for it not existing, this is expected.
     ```bash
     rm ~/printer_data/config/AFC/AFC.var.unit
     ```
     

4. **Best Practice**: Install AFC while your OpenAMS unit is **empty** to avoid interruptions during the system file updates.

### Configuration Files

After AFC installation, copy the OpenAMS-specific configuration files to your AFC directory:

```bash
cp AFC_AMS1.cfg ~/printer_data/config/AFC/
cp AFC_Oams.cfg ~/printer_data/config/AFC/
cp AFC_Oams_Macros.cfg ~/printer_data/config/AFC/
```

Replace `~/printer_data` with your actual printer data path if different.

**Configuration File Overview:**

| File | Purpose | Must Edit? |
|------|---------|------------|
| `AFC_AMS1.cfg` | Defines AFC lanes mapped to OpenAMS slots | Yes - configure for your setup |
| `AFC_Oams.cfg` | OpenAMS hardware configuration (MCU, sensors, etc.) | Yes - set CAN UUIDs and calibration values |
| `AFC_Oams_Macros.cfg` | Standard load/unload macros | Yes - customize for your workflow |
| `AFC_Oams_Smart_Purge_Temp_Macros.cfg` | Advanced temperature-aware macros | Optional - see below |

**Editing Configuration Files:**

1. **AFC_Oams.cfg**:
   - Set your CAN bus UUIDs for FPS and OAMS MCU boards
   - Configure `_oams_macro_variables` for your specific printer geometry
   - Adjust retry settings if needed

2. **AFC_AMS1.cfg**:
   - Map lanes to your specific AMS unit slots  -  *preconfigured for T0-T3*
   - Set LED indices if using LED indicators
   - Configure hub settings and bowden lengths  -  *will be done later with auto configuration*

3. **Include in printer.cfg**:
   ```ini
   [include AFC/AFC_AMS1.cfg]
   [include AFC/AFC_Oams.cfg]
   [include AFC/AFC_Oams_Macros.cfg]
   ```

### Smart Temperature Purge Macros

For advanced multi-material printing with intelligent temperature management, use the smart purge macros instead of the standard macros:

```bash
cp AFC_AMS1.cfg ~/printer_data/config/AFC/
cp AFC_Oams.cfg ~/printer_data/config/AFC/
cp AFC_Oams_Smart_Purge_Temp_Macros.cfg ~/printer_data/config/AFC/AFC_Oams_Macros.cfg
```

**How Smart Purge Works:**

The smart purge macros automatically manage temperatures during tool changes:

1. **During Unload** (`SAFE_UNLOAD_FILAMENT`):
   - Heats to the **old lane's configured temperature** (or 240°C default)
   - Uses a safety floor of 210°C minimum
   - Records which lane was unloaded for next tool change

2. **During Load** (`_TX`):
   - Retrieves the previously unloaded lane
   - Calculates the **maximum** of old and new lane temperatures
   - Heats to the higher temperature before loading
   - This ensures high-temp filament (e.g., PETG at 250°C) can be properly purged when switching to lower-temp materials (e.g., PLA at 210°C)

3. **First Load** (after reboot):
   - Falls back to 240°C default if no previous spool loaded
   - Can be customized in the macro

**Configuration:**

Set extruder temperatures for each lane in `AFC_AMS1.cfg`:

```ini
[AFC_lane lane0]
unit: AMS_1:1
extruder_temp: 210    # PLA temperature
# ... other settings ...

[AFC_lane lane1]
unit: AMS_1:2
extruder_temp: 250    # PETG temperature
# ... other settings ...
```

**Important:** Smart purge macros leave the heater at the purge temperature when finished. Your print start G-code or slicer must set the final temperature for the active tool.

### AFC Hardware Configuration

Before first boot with AFC, configure the tool sensor pin in the AFC hardware configuration:

```bash
nano ~/printer_data/config/AFC/AFC_Hardware.cfg
```

Find the `[AFC_extruder extruder]` section and set the `pin_tool_start:` value:

```ini
[AFC_extruder extruder]
# Use AMS_extruder if FPS (Filament Pressure Sensor) handles filament sensing with ramming
pin_tool_start: AMS_extruder

# OR use your toolhead filament sensor pin if not using FPS ramming
# pin_tool_start: ^PC0  # Example - adjust to your wiring
```

**Choosing the correct pin:**
- **Using FPS with ramming**: Set `pin_tool_start: AMS_extruder`
- **Using toolhead sensor**: Set to your actual sensor pin (e.g., `^PC0`, `^PA1`, etc.)

Once all configuration files are in place and edited, reboot the host to ensure AFC services reload:

```bash
sudo reboot
```

**Important:** Only load spools into the AMS **after** the first boot with AFC completes.

## Optional Features

### Spoolman LED Sync

The Spoolman LED sync feature allows the OpenAMS active tool LED to display the actual filament color from Spoolman instead of the default blue color.

**Features:**
- Automatically reads filament color from Spoolman via AFC
- Sets the active tool LED to match the actual filament color
- Falls back to a configurable default color if Spoolman data is not available
- Error states (stuck spool, clog) still override with red flashing
- Non-active lanes continue to use their normal AFC LED colors
- Requires Spoolman integration to be active in AFC

**Setup:**

1. Copy the Spoolman LED sync configuration file:

```bash
cp spoolman_led_sync.cfg ~/printer_data/config/AFC/
```

2. Edit `~/printer_data/config/AFC/spoolman_led_sync.cfg` and enable the feature:

```ini
[spoolman_led_sync]
enable: True
default_color: 0000FF  # Blue (default) - used when no Spoolman data available
```

3. Optionally customize the default LED color for lanes without Spoolman data:

**Common color examples:**
- White: `FFFFFF`
- Red: `FF0000`
- Green: `00FF00`
- Yellow: `FFFF00`
- Purple: `FF00FF`
- Cyan: `00FFFF`
- Orange: `FFA500`

4. Restart Klipper to load the new configuration:

```
FIRMWARE_RESTART
```

To disable the feature later, set `enable: False` in the configuration file and restart Klipper.

**How It Works:**

When a lane is loaded into the toolhead:
1. The module checks if the lane has a spool assigned in Spoolman
2. If a spool is found, it reads the filament color from Spoolman
3. The active tool LED is set to that color instead of the default blue
4. If no Spoolman data exists, it uses the configured `default_color`
5. Other lanes (not loaded) continue using their standard AFC LED colors

### Mainsail AFC Panel

Enable the optional Mainsail AFC panel for easy lane management and status monitoring.

**Installation:**

1. Backup your existing Mainsail installation:

```bash
cd ~/mainsail
tar -czf ~/mainsail-backup-$(date +%Y%m%d).tar.gz .
```

2. Extract the included panel files:

```bash
cd ~/klipper_openams
unzip -o mainsail.zip -d ~/mainsail/
```

3. Clear your browser cache and reload Mainsail

**Features:**
- Visual lane status display
- Quick lane selection
- Runout configuration interface
- Spool management integration

## Initial Calibration

After completing the OpenAMS and AFC installation, calibrate each OpenAMS unit to ensure accurate filament detection and optimal performance.

**Calibration Process:**

1. Ensure your AMS unit has at least one spool loaded

2. Run the calibration command via the Klipper console:
   ```
   AFC_CALIBRATION
   ```

3. When prompted, select your OpenAMS unit from the list

4. The calibration process will automatically:
   - Measure PTFE tube lengths for each lane
   - Calibrate HUB_HES sensor values
   - Store the configuration values in your config file

5. Once calibration completes, restart Klipper to load the new settings:
   ```
   FIRMWARE_RESTART
   ```

**Repeat this process for each OpenAMS unit** in your system. Proper calibration ensures:
- Reliable lane detection
- Accurate filament position tracking
- Prevention of loading errors during multi-material prints

**Troubleshooting Calibration:**
- Ensure filament is loaded in the lane being calibrated
- Verify encoder (hub) is clean and functioning
- Check that tube lengths are within reasonable bounds (50-2000mm typical)

## Infinite Spooling

Infinite spooling allows automatic switching between lanes when a spool runs out, enabling continuous printing without manual intervention.

**Key Features:**
- No longer requires filament groups to be configured in OpenAMS
- OpenAMS manages all runout handling for AMS lanes
- Supports infinite spooling between lanes on the same extruder/FPS
- Falls back to AFC logic for other scenarios
- Can be configured via console or Mainsail panel

**Configuration:**

Assign runout lanes using either method:

**Method 1: Klipper Console**
```
SET_RUNOUT LANE=<lane #> RUNOUT=<lane #>
```

Example: Set lane 1 to use lane 2 as runout backup:
```
SET_RUNOUT LANE=1 RUNOUT=2
```

**Method 2: AFC Panel (Mainsail)**
1. Navigate to the AFC panel
2. Select the primary lane
3. Set the runout lane in the configuration dropdown


**Multi-Lane Chains:**

You can create chains of runouts for extended printing:
```
SET_RUNOUT LANE=1 RUNOUT=2
SET_RUNOUT LANE=2 RUNOUT=3
SET_RUNOUT LANE=3 RUNOUT=4
```

In this example, the printer will automatically switch through lanes 1→2→3→4 as spools run out.

**Material Matching:**

For best results:
- Chain lanes with the same material type and color
- Use Spoolman to track filament properties
- Update spool weights regularly for accurate runout detection

## Troubleshooting

### Stuck Spool Detection

If you experience issues with stuck spool detection during load or unload operations:

**Check retry configuration:**
```ini
[oams oams1]
load_retry_max: 3         # Try 3 times before giving up
unload_retry_max: 2       # Try 2 times before giving up
retry_backoff_base: 1.0   # Wait 1s, 2s, 3s between attempts
```

**Verify retry behavior:**
- Monitor Klipper logs: `tail -f ~/printer_data/logs/klippy.log`
- Look for messages like "letting retry logic handle it" and "retry X/Y"
- Retries should happen automatically before pausing

**Common solutions:**
- **Increase retry counts**: Set `load_retry_max: 5` if filament occasionally needs extra attempts
- **Adjust retry delays**: Set `retry_backoff_base: 2.0` if hardware needs more recovery time
- **Check encoder**: Clean the OAMS encoder wheel and verify it rotates freely
- **Verify filament path**: Ensure PTFE tubes are not kinked or obstructed

### CAN Bus Issues

**Symptoms:**
- OpenAMS units not detected
- Intermittent connection losses
- Calibration failures

**Solutions:**

1. **Verify CAN termination resistors** (120Ω at each end of the bus)
2. **Check CAN bus speed** matches across all devices (typically 500000 or 1000000)
3. **Query CAN devices:**
   ```bash
   ~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
   ```
4. **Check Klipper logs** for CAN timeout errors
5. **Verify wiring:** Ensure CAN_H and CAN_L are not swapped

### Clog Detection False Positives

If the printer pauses due to false clog detection:

1. **Lower sensitivity:**
   ```ini
   [oams_manager]
   clog_sensitivity: low
   ```

2. **Check for actual flow issues:**
   - Partial nozzle clogs
   - Extruder tension too tight/loose
   - Filament diameter variations

3. **Verify encoder function:**
   - Clean encoder wheel
   - Check encoder wiring
 
### LED Issues

**LEDs not changing color:**

1. **Verify LED index configuration** in `AFC_AMS1.cfg`:
   ```ini
   [AFC_lane lane0]
   led_index: AFC_indicator:1
   ```

2. **Check LED strip configuration** 
3. **Test LEDs directly:**
   ```
   SET_LED LED=AFC_indicator INDEX=1 RED=1.0 GREEN=0 BLUE=0
   ```

**Spoolman LED sync not working:**

1. **Verify Spoolman integration is active** in AFC
2. **Check that `enable: True`** in `spoolman_led_sync.cfg`
3. **Ensure lanes have spools assigned** in Spoolman
4. **Check Klipper logs** for spoolman_led_sync errors
5. **Restart Klipper** after configuration changes

### Filament Loading Failures

**Filament won't load to toolhead:**

1. **Check AFC_Hardware.cfg pin configuration:**
   ```ini
   [AFC_extruder extruder]
   pin_tool_start: AMS_extruder  # or your sensor pin
   ```

2. **Verify bowden length calibration:**
   - Run `AFC_CALIBRATION` for each unit
   - Check that measured lengths are reasonable
   - Adjust `afc_bowden_length` in hub configuration

3. **Test individual components:**
   - Manually extrude filament to verify extruder works
   - Check that toolhead sensor triggers correctly
   - Verify PTFE tube path is clear

4. **Increase retry attempts temporarily:**
   ```ini
   [oams oams1]
   load_retry_max: 5
   ```

### Configuration File Errors

**Klipper won't start after configuration changes:**

1. **Check klippy.log** for specific error messages:
   ```bash
   tail -50 ~/printer_data/logs/klippy.log
   ```

2. **Common issues:**
   - Missing or incorrect CAN UUIDs
   - Duplicate section names
   - Invalid pin names
   - Syntax errors in macros

3. **Test configuration syntax:**
   ```bash
   ~/klippy-env/bin/python ~/klipper/klippy/klippy.py ~/printer_data/config/printer.cfg -d ~/printer_data/klipper.dict -l /tmp/test.log
   ```

### Getting Help

If you're still experiencing issues:

1. **Check Klipper logs:** `~/printer_data/logs/klippy.log`
2. **Enable debug logging** for OpenAMS components
3. **Gather information:**
   - Klipper version
   - AFC version
   - OpenAMS fork commit hash: `git -C ~/klipper_openams rev-parse HEAD`
   - Full error messages from logs
   - Configuration files

4. **Ask for help:**
   - GitHub Issues: https://github.com/lindnjoe/klipper_openams/issues


## Credits

This project was created by **knight.rad_iant** and **Armored Turtle Team** on Discord.

Based on the original OpenAMS project with enhancements for AFC integration, retry logic, clog detection, and Spoolman LED synchronization.
