<img width="512" height="512" alt="image" src="https://github.com/user-attachments/assets/32c8de82-3b0e-4ee3-b696-ab354c2b7430" />


# OpenAMS for Klipper

A Klipper integration for OpenAMS that enables multi-material printing with automatic filament management, runout detection, and intelligent retry logic.

## Table of Contents

- [Overview](#overview)
- [What's New](#whats-new)
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
  - [Lane-Based Architecture](#lane-based-architecture)
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

OpenAMS provides automated filament handling for Klipper-based 3D printers. This fork integrates with Armored Turtle's AFC (Automatic Filament Changer) add-on using a **lane-based architecture** for flexible multi-material printing.

**Key capabilities:**
- Lane-based filament management through AFC integration
- Automatic filament loading and unloading with pressure sensing
- Intelligent retry logic for stuck filament detection
- Clog detection with configurable sensitivity
- Event-driven sensor monitoring for optimal performance
- Runout detection and automatic lane switching
- Spoolman integration for filament tracking
- LED status indicators with optional color sync

## What's New

**Recent Major Updates:**

### Lane-Based Architecture (Current Version)
The system has transitioned from filament groups to a lane-based architecture for better AFC integration:

- **AFC Lanes**: Each OpenAMS slot is now configured as an AFC lane with independent settings
- **Improved Macros**: All toolchange macros now use `LANE` parameters (`LANE=lane0`) instead of `GROUP` parameters
- **Event-Driven Sensors**: Sensor monitoring switched from polling to event-based for better performance
- **AFC Runout Integration**: Runout handling now integrates directly with AFC's lane system
- **Flexible Hub Mapping**: Each lane can be mapped to different hubs for complex routing scenarios

**Migration Notes:**
- If upgrading from an older version, your filament group configuration will need to be converted to AFC lanes
- Macro calls have changed from `GROUP=T0` to `LANE=lane0` format
- Runout configuration now uses AFC's `SET_MAP_RUNOUT` command instead of OpenAMS filament groups
- See the [AFC Integration](#afc-integration) section for detailed configuration examples

## Features

- **Lane-Based Architecture**: Integration with AFC lanes for flexible spool configuration and mapping
- **Event-Driven Sensors**: Efficient event-based monitoring instead of constant polling for better performance
- **Automatic Retry Logic**: Configurable retry attempts for both load and unload operations with exponential backoff
- **Clog Detection**: Three sensitivity levels (low, medium, high) to detect filament clogs during printing
- **Runout Handling**: Automatic filament runout detection integrated with AFC lane system
- **Infinite Spooling**: Seamless lane switching for continuous printing using AFC runout configuration
- **Smart Temperature Management**: Optional lane-specific temperature control for multi-material prints (disabled by default)
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

This OpenAMS fork is designed to work with Armored Turtle's AFC (Automatic Filament Changer) Klipper add-on using a **lane-based architecture**.

### Lane-Based Architecture

OpenAMS integrates with AFC through **lanes** instead of the legacy filament group system. Each lane represents:
- One physical spool slot on your OpenAMS unit
- A mapping to a specific hub for filament routing
- Custom load/unload commands specific to that lane
- Optional LED indicators
- Association with a tool number (T0, T1, T2, T3, etc.)

**Benefits of Lane-Based Configuration:**
- More flexible spool-to-tool mappings
- Better integration with AFC's toolchange system
- Easier to configure runout behavior per lane
- Supports multiple hubs and complex routing
- Compatible with Spoolman for per-lane temperature settings

**Example:** A single OpenAMS with 4 slots becomes 4 AFC lanes (lane0, lane1, lane2, lane3), each independently configurable with its own hub, toolchange macros, and runout behavior.

### Installing AFC

Clone the Armored Turtle Repo

```bash
cd ~
git clone https://github.com/ArmoredTurtle/AFC-Klipper-Add-On.git
```

Install the AFC add-on from the multi_extruder branch:

```bash
cd ~/AFC-Klipper-Add-On
./install-afc.sh -b multi_extruder
```

**Important Setup Notes:**

1. **Read the AFC Documentation**: Most of the Armored Turtle setup documentation applies directly to OpenAMS. Review it at: (https://www.armoredturtle.xyz/docs/afc-klipper-add-on/index.html)

2. **Box Turtle vs OpenAMS**: Some documentation is specific to Box Turtle hardware and won't apply to OpenAMS. When in doubt:
   - Ask for clarification
   - Leave default settings in place
   - The OpenAMS-specific config files override Box Turtle defaults

3. **Installation Prompts**: During the AFC installation, you'll be presented with an interactive menu:
   - Press **T** to cycle through installation types until you see **OpenAMS**
   - Select the **OpenAMS** unit type when prompted
   - Enter an AMS name (default: `AMS_1`) or use the default
   - Configure your preferred options (tip forming, cutters, macros, etc.)
   - Complete the installation

<img width="1086" height="803" alt="image" src="https://github.com/user-attachments/assets/7b62feea-d566-4be5-9d44-5b79644fc841" />


4. **Best Practice**: Install AFC while your OpenAMS unit is **empty** to avoid interruptions during the system file updates.

### Configuration Files

After AFC installation, copy the OpenAMS-specific configuration files to your AFC directory:

```bash
cp AFC_Oams.cfg ~/printer_data/config/AFC/
cp AFC_Oams_Smart_Purge_Temp_Macros.cfg ~/printer_data/config/AFC/
```

Replace `~/printer_data` with your actual printer data path if different.

**Configuration File Overview:**

| File | Purpose | Must Edit? |
|------|---------|------------|
| `AFC_AMS1.cfg` | Defines AFC lanes mapped to OpenAMS slots and hubs | Yes - configure lanes and T-number mappings |
| `AFC_Oams.cfg` | OpenAMS hardware configuration (MCU, sensors, FPS) | Yes - set CAN UUIDs and calibration values |
| `AFC_Oams_Macros.cfg` | Basic load/unload macros using lane-based parameters | Optional - included by AFC |
| `AFC_Oams_Smart_Purge_Temp_Macros.cfg` | Enhanced macros with optional smart temperature control (disabled by default) | Optional - alternative to AFC_Oams_Macros.cfg |

**Editing Configuration Files:**

1. **AFC_Oams.cfg**:
   - Set your CAN bus UUIDs for FPS and OAMS MCU boards
   - Configure `_oams_macro_variables` for your specific printer geometry
   - Adjust retry settings and sensor thresholds if needed
   - Configure FPS (Filament Pressure Sensor) pin and settings

2. **AFC_AMS1.cfg** (Lane Configuration):
   - Each lane maps one OpenAMS slot to a tool number (T0, T1, T2, T3)
   - Lanes are preconfigured as `lane0` through `lane3` mapped to T0-T3
   - Each lane specifies its `unit` (e.g., `AMS_1:1` for slot 1), hub, and custom load/unload macros
   - Set LED indices if using LED indicators
   - Hub settings and bowden lengths will be auto-calibrated

   **Example lane configuration:**
   ```ini
   [AFC_lane lane0]
   unit: AMS_1:1          # AMS unit and slot number
   load_to_hub: False     # Filament goes directly to toolhead
   hub: Hub_1             # Associated hub for this lane
   map: T0                # Toolchange macro mapping
   custom_load_cmd: _TX1 LANE=lane0
   custom_unload_cmd: SAFE_UNLOAD_FILAMENT1
   ```

3. **Macro Files** (Choose ONE):
   - **AFC_Oams_Macros.cfg**: Basic macros for load/unload using lane parameters
   - **AFC_Oams_Smart_Purge_Temp_Macros.cfg**: Enhanced macros with optional temperature management

4. **Include in printer.cfg** (or AFC auto-includes them if in AFC folder):
   ```ini
   [include AFC/AFC_Oams.cfg]
   [include AFC/AFC_Oams_Macros.cfg]
   # OR use smart purge macros instead:
   # [include AFC/AFC_Oams_Smart_Purge_Temp_Macros.cfg]
   ```

### Smart Temperature Purge Macros

The macros support both basic and advanced toolchange operations using **lane-based parameters**. All tool macros (T0-T3) now pass lane names (`lane0`, `lane1`, etc.) to the underlying `_TX1` and `_TX` macros.

**Macro Architecture:**
- **T0-T3 macros**: User-facing toolchange commands that call `_TX1 LANE=laneX`
- **_TX1 macro**: Wrapper that passes lane parameters to the generic `_TX` macro
- **_TX macro**: Core toolchange logic supporting temperature management and AFC integration
- **SAFE_UNLOAD_FILAMENT1**: Unload macro that can accept optional LANE parameter

The macros include optional intelligent temperature management for multi-material printing. **By default, smart temperature is disabled**, allowing you full manual control over temperatures.

**Smart Temperature Toggle:**

To enable or disable smart temperature management, edit `AFC_Oams_Smart_Purge_Temp_Macros.cfg`:

```ini
[gcode_macro _oams_smart_temp_settings]
# Enable or disable smart temperature adjustment during load/unload
# When enabled: Uses lane-specific temps and calculates optimal purge temps
# When disabled: Uses current extruder temperature without any changes
#                (allows you to manually control temperature)
# Default: False
variable_enable_smart_temp: False  # Set to True to enable
gcode:
    # This macro just holds variables, no gcode needed
```

Restart Klipper after making changes: `FIRMWARE_RESTART`

**How Smart Purge Works (When Enabled):**

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

Set extruder temperatures for each lane in `AFC_AMS1.cfg` If using Spoolman this will not be needed as it will pull the spool info from there:

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

**Important:**
- Smart temperature is **disabled by default**
- When enabled, smart purge macros leave the heater at the purge temperature when finished. Your print start G-code or slicer must set the final temperature for the active tool

### AFC Hardware Configuration

Before first boot with AFC, configure the tool sensor pin in the AFC hardware configuration if you didn't do this during installation:

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

Infinite spooling allows automatic switching between lanes when a spool runs out, enabling continuous printing without manual intervention. OpenAMS now integrates with AFC's lane-based runout system for seamless operation.

**Key Features:**
- Integrated with AFC lane runout configuration
- OpenAMS detects runout and notifies AFC to load the next lane
- Supports infinite spooling between lanes on the same extruder/FPS
- Automatic lane switching based on AFC runout settings
- Can be configured via AFC console commands or Mainsail AFC panel

**How It Works:**
1. OpenAMS monitors the FPS (Filament Pressure Sensor) and hub sensors
2. When a spool runs empty, OpenAMS detects the runout event
3. OpenAMS notifies AFC about the runout
4. AFC handles the lane switching using its configured runout mappings
5. The new lane is automatically loaded and printing continues

**Configuration:**

Runout lanes are configured through AFC using the `SET_MAP_RUNOUT` command:

**Method 1: Klipper Console (AFC Command)**
```
SET_MAP_RUNOUT MAP=<tool> LANE=<lane_name>
```

Example: Set T0 to use lane1 as runout backup:
```
SET_MAP_RUNOUT MAP=T0 LANE=lane1
```

**Method 2: AFC Panel (Mainsail)**
1. Navigate to the AFC panel in Mainsail
2. Select the tool/lane
3. Configure the runout lane in the AFC interface

**Multi-Lane Chains:**

You can create chains of runouts for extended printing by configuring multiple mappings:
```
SET_MAP_RUNOUT MAP=T0 LANE=lane1
SET_MAP_RUNOUT MAP=T1 LANE=lane2
SET_MAP_RUNOUT MAP=T2 LANE=lane3
```

**Material Matching:**

For best results:
- Assign lanes with the same material type and color to the same tool
- Use Spoolman to track filament properties across lanes
- Update spool weights regularly for accurate runout detection
- Configure appropriate runout chains through AFC

**Note:** This system replaces the previous filament group configuration. All runout handling now goes through AFC's lane system for better integration and flexibility.

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
