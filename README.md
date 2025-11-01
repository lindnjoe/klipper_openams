# OpenAMS for Klipper  
OpenAMS Klipper Plugin

## Installation

### Automatic Installation

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

### Integrating with Armored Turtle's AFC Add-on

To integrate OpenAMS with the [Armored Turtle AFC Klipper add-on](https://github.com/ArmoredTurtle/AFC-Klipper-Add-On), install the add-on from
the `multi_extruder` branch using the project's installation script:

```bash
./install-afc.sh -b multi_extruder
```

The installer does not yet list an OpenAMS-specific unit type; when prompted,
choose the **Box Turtle** option and accept the default settings. Once the
installation completes, delete the generated `AFC_Turtle_1.cfg` file from the
`printer_data/config/AFC/` directory to avoid conflicts with the OpenAMS
configuration.

For best results, perform the installation while your OpenAMS unit is empty so
that the AFC installer can update its system files without interruption. After
installing the add-on, copy the provided `AFC_AMS1.cfg` file into the Klipper
configuration directory so that it can be included by your printer
configuration:

```bash
cp AFC_AMS1.cfg <printer_data path>/config/AFC/
```

If you used the default installation layout, the destination path will be
`~/printer_data/config/AFC/`. Once all file operations are complete, reboot the
host running Klipper to ensure the AFC services reload with the new files.
Load spools into the AMS only after that first boot finishes.

Infinite spooling no longer requires filament groups to be configured in
OpenAMS. You may still define groups within your OpenAMS configuration if you
prefer. Assign runout lanes either via the Klipper console command
`SET_RUNOUT LANE=<lane #> RUNOUT=<lane #>` or through the AFC panel in
Mainsail.

To enable the optional Mainsail AFC panel, extract the included `mainsail.zip`
archive into your Mainsail installation directory. Before extracting, back up
and remove the existing contents of that directory to ensure the new panel
files replace the previous version cleanly.

## Credits  

This project was made by knight.rad_iant on Discord.

---
