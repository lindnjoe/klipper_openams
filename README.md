*** Copy the AFC_OpenAMS.py and openams_integration.py that are included to the ~/Klipper-Add-On/extras folder. This won't be necessary once these are pulled in upstream, I will delete this when that is done.***


Install OpenAMS using the provided script:

```bash
cd ~  
git clone https://github.com/lindnjoe/klipper_openams.git  
cd klipper_openams  
./install-openams.sh
```

If your directory structure differs, you can configure the installation script with additional parameters:

```bash
./install-openams.sh [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>]
```

Configuration
OpenAMS Manager Settings
```ini
[oams_manager]
# Optional: Distance before toolhead to trigger reload during runout (default: 0.0)
#reload_before_toolhead_distance: 0.0

# Optional: Clog detection sensitivity: low, medium, high (default: medium)
#clog_sensitivity: medium
```

Example configuration with clog detection enabled:
```ini
[oams_manager]
reload_before_toolhead_distance: 50.0  # Trigger reload 50mm before toolhead
clog_sensitivity: medium                # Balanced clog detection
```

OAMS Hardware Settings
For each OAMS unit, configure the retry behavior in your OAMS hardware configuration file:

```ini
[oams oams_name]
# ... other settings ...

# Optional: Maximum number of load retry attempts (default: 3)
#load_retry_max: 3

# Optional: Maximum number of unload retry attempts (default: 2)
#unload_retry_max: 2

# Optional: Base delay in seconds between retries (default: 1.0)
# Actual delays use exponential backoff: base_delay * attempt_number
#retry_delay_base: 1.0
```

Retry Behavior
The OpenAMS system includes automatic retry logic for both load and unload operations:

**Load Retries:**
- Default: 3 attempts with exponential backoff (1s, 2s, 4s delays)
- Monitors encoder movement during loading
- Aborts stuck operations and retries automatically
- Only pauses the printer if all retry attempts fail

**Unload Retries:**
- Default: 2 attempts with exponential backoff (1s, 2s delays)
- Monitors encoder movement during unloading
- Aborts stuck operations and retries automatically
- Only pauses the printer if all retry attempts fail

Example Configuration:

```ini
[oams oams_1]
# ... connection settings ...
load_retry_max: 3        # Try loading up to 3 times
unload_retry_max: 2      # Try unloading up to 2 times
retry_delay_base: 1.0    # Wait 1s, 2s, 4s between attempts
```

Clog Detection Settings
The `clog_sensitivity` setting in `[oams_manager]` controls how aggressive the clog detection is:

- **low**: Longer observation window (48mm extrusion), more tolerant
- **medium** (default): Balanced settings (24mm extrusion)
- **high**: Shorter observation window (12mm extrusion), more sensitive

Integrating with Armored Turtle's AFC Add-on
https://github.com/ArmoredTurtle/AFC-Klipper-Add-On

To integrate OpenAMS with the Armored Turtle AFC Klipper add-on, install the add-on from the multi_extruder branch using the project's installation script:

```bash
./install-afc.sh -b multi_extruder
```

The installer does not yet list an OpenAMS-specific unit type; when prompted, choose the Box Turtle option and accept the default settings. Once the installation completes, delete the generated `AFC_Turtle_1.cfg` file from the `printer_data/config/AFC/` directory to avoid conflicts with the OpenAMS configuration.

For best results, perform the installation while your OpenAMS unit is empty so that the AFC installer can update its system files without interruption. After installing the add-on, copy the provided `AFC_AMS1.cfg`, `AFC_oamsc.cfg`, and `AFC_oams_macros.cfg` files into the AFC configuration directory so that they can be included by your printer configuration:

```bash
cp AFC_AMS1.cfg <printer_data path>/config/AFC/
cp AFC_oamsc.cfg <printer_data path>/config/AFC/
cp AFC_oams_macros.cfg <printer_data path>/config/AFC/
```

Edit each file to match your specific hardware setup.

If you used the default installation layout, the destination path will be `~/printer_data/config/AFC/`. 

Before rebooting, update the AFC hardware configuration to ensure the tool sensor pin is defined. Edit `printer_data/config/AFC/AFC_Hardware.cfg` and set `pin_tool_start:` within the `[AFC_extruder extruder]` section. Use `pin_tool_start: AMS_extruder` when the filament pressure sensor (FPS) handles filament sensing with ramming; otherwise, set the value to the toolhead filament sensor pin that matches your printer's wiring.

Once all file operations and configuration edits are complete, reboot the host running Klipper to ensure the AFC services reload with the new files. Load spools into the AMS only after that first boot finishes.

Infinite spooling no longer requires filament groups to be configured in OpenAMS. You may still define groups within your OpenAMS configuration if you prefer. OpenAMS now manages all runout handling for AMS lanes, including infinite spooling between lanes on the same extruder/FPS, and falls back to the AFC logic for any remaining scenarios. Assign runout lanes either via the Klipper console command `SET_RUNOUT LANE=<lane #> RUNOUT=<lane #>` or through the AFC panel in Mainsail.

To enable the optional Mainsail AFC panel, extract the included `mainsail.zip` archive into your Mainsail installation directory. Before extracting, back up and remove the existing contents of that directory to ensure the new panel files replace the previous version cleanly.

Initial Calibration
After completing the OpenAMS and AFC installation, calibrate each OpenAMS unit to ensure accurate filament detection and optimal performance. The AFC system provides the calibration tools needed for this process.

1. Run the calibration command via the Klipper console:
   ```
   AFC_CALIBRATION
   ```

2. When prompted, select your OpenAMS unit from the list

3. The calibration process will automatically:
   - Measure PTFE tube lengths for each lane
   - Calibrate HUB_HES sensor values
   - Store the configuration values in your config file

4. Once calibration completes, restart Klipper to load the new settings:
   ```
   FIRMWARE_RESTART
   ```

Repeat this process for each OpenAMS unit in your system. Proper calibration ensures reliable lane detection and prevents loading errors during multi-material prints.

Troubleshooting
Stuck Spool Detection
If you experience issues with stuck spool detection during load or unload operations:

- **Check retry counts**: Verify your `load_retry_max` and `unload_retry_max` settings are appropriate (defaults: 3 and 2)
- **Adjust retry delays**: Increase `retry_delay_base` if your hardware needs more time between attempts
- **Monitor logs**: Look for messages like "letting retry logic handle it" and "retry X/Y" to confirm retries are working
- **Verify encoder**: Ensure your OAMS encoder is clean and functioning properly

The system will automatically retry stuck operations before pausing. Only after all retry attempts are exhausted will the printer pause and require manual intervention.

Credits
This project was made by knight.rad_iant on Discord
