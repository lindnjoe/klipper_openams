# Troubleshooting: "Pin 'AMS_extruder' is not a valid pin name" Error

## Problem

You see this error during Klipper startup:
```
msgproto.enumeration_error: Unknown value 'AMS_extruder' in enumeration 'pin'
configparser.Error: Pin 'AMS_extruder' is not a valid pin name on mcu 'mcu'
```

Even though you have correctly configured:
```ini
[AFC_extruder extruder]
pin_tool_start: AMS_extruder
```

## Root Cause

The `AMS_extruder` virtual pin handler is registered by the `AFC_OpenAMS.py` module when it loads the `[AFC_OpenAMS ...]` configuration section. **If Klipper processes your `[AFC_extruder extruder]` section BEFORE loading `[AFC_OpenAMS AMS_1]`, the virtual pin hasn't been registered yet**, causing the error.

## Solution: Fix Configuration Loading Order

### Step 1: Check Your printer.cfg Include Order

Open your `printer.cfg` file and look at the order of includes:

```bash
nano ~/printer_data/config/printer.cfg
```

**CRITICAL:** Ensure AFC_AMS files are included BEFORE AFC_Hardware.cfg:

```ini
# ✅ CORRECT ORDER - AFC_OpenAMS loads first, then AFC_extruder
[include AFC/AFC_AMS_1.cfg]        # Contains [AFC_OpenAMS AMS_1] - MUST BE FIRST
[include AFC/AFC_Oams.cfg]          # OpenAMS hardware config
[include AFC/AFC_Hardware.cfg]      # Contains [AFC_extruder extruder] - MUST BE AFTER
[include AFC/AFC_Oams_Smart_Purge_Temp_Macros.cfg]

# ❌ WRONG ORDER - Will cause the error!
[include AFC/AFC_Hardware.cfg]      # AFC_extruder loads before patch is applied
[include AFC/AFC_AMS_1.cfg]         # AFC_OpenAMS loads too late
```

### Step 2: Alternative - Move AFC_extruder to AFC_AMS_1.cfg

If you can't control the include order (e.g., AFC auto-includes AFC_Hardware.cfg first), move the `[AFC_extruder extruder]` section INTO the AFC_AMS_1.cfg file:

```bash
nano ~/printer_data/config/AFC/AFC_AMS_1.cfg
```

Add this at the TOP of AFC_AMS_1.cfg, AFTER the `[AFC_OpenAMS AMS_1]` section:

```ini
[AFC_OpenAMS AMS_1]
oams: oams1
extruder: extruder

# Add AFC_extruder section here - it will load after AFC_OpenAMS patch is applied
[AFC_extruder extruder]
pin_tool_start: AMS_extruder

# Rest of your lanes...
[AFC_lane lane0]
...
```

Then remove or comment out the `[AFC_extruder extruder]` section from AFC_Hardware.cfg:

```bash
nano ~/printer_data/config/AFC/AFC_Hardware.cfg
```

```ini
# Moved to AFC_AMS_1.cfg to ensure correct loading order
#[AFC_extruder extruder]
#pin_tool_start: AMS_extruder
```

### Step 3: Verify the Fix

1. Save all configuration files
2. Restart Klipper:
   ```bash
   sudo systemctl restart klipper
   ```
3. Check logs for errors:
   ```bash
   tail -f ~/printer_data/logs/klippy.log
   ```

## Why This Happens

Klipper processes configuration sections sequentially as it reads them from include files. The AFC_OpenAMS.py module uses **monkey-patching** to intercept `AMS_extruder` pins during the `[AFC_OpenAMS ...]` load:

1. When `[AFC_OpenAMS AMS_1]` loads → calls `load_config_prefix()` → applies `_patch_extruder_for_virtual_ams()`
2. The patch modifies the `AFCExtruder.__init__()` to intercept `AMS_extruder` and replace it with "buffer"
3. When `[AFC_extruder extruder]` loads later → uses patched code → virtual pin works

If the order is reversed, step 3 happens before step 1, and the patch isn't applied yet.

## Additional Checks

### Verify AFC_OpenAMS Module is Installed

```bash
ls -la ~/klipper/klippy/extras/AFC_OpenAMS.py
```

You should see a symlink to the AFC_OpenAMS.py file. If not, reinstall:

```bash
cd ~/klipper_openams
./install-openams.sh
```

### Check for Multiple AFC_extruder Sections

```bash
grep -r "^\[AFC_extruder" ~/printer_data/config/
```

Make sure you only have ONE `[AFC_extruder extruder]` section and it loads AFTER `[AFC_OpenAMS AMS_1]`.

### Verify Config Syntax

Ensure there are no typos:
- `[AFC_OpenAMS AMS_1]` (not AFC_OpenAms or AFC_OPENAMS)
- `pin_tool_start: AMS_extruder` (not ams_extruder or AMS_Extruder)

## Still Not Working?

If you've verified the loading order and still see the error, check:

1. **AFC branch**: Ensure you installed AFC from the `multi_extruder` branch:
   ```bash
   cd ~/AFC-Klipper-Add-On
   git branch
   ```
   You should see `* multi_extruder`

2. **Klipper restart**: Some changes require a full restart, not just FIRMWARE_RESTART:
   ```bash
   sudo systemctl restart klipper
   ```

3. **Check for conflicting modules**: Some other modules might interfere with AFC_extruder loading. Review your klippy.log for any AFC-related errors.

4. **Enable debug logging**: Add this to your printer.cfg temporarily:
   ```ini
   [respond]
   default_type: echo
   ```

## Summary

**The fix**: Ensure `[AFC_OpenAMS AMS_1]` is loaded BEFORE `[AFC_extruder extruder]` in your configuration files. The easiest way is to include AFC_AMS_1.cfg before AFC_Hardware.cfg in your printer.cfg.
