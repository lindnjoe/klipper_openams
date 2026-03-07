# AFC M109 Deadband Override Module
#
# This module provides a toggleable feature to automatically use the deadband
# configured for each AFC extruder when using M109 commands.
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class AFC_M109_Deadband:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # Get configuration options
        self.enabled = config.getint('enabled', 1)  # Default: enabled

        # Store reference to AFC and original function
        self.afc = None
        self.original_afc_m109 = None

        # Register event handler to hook in after AFC loads
        self.printer.register_event_handler("klippy:ready",
                                           self.handle_ready)

        # Register gcode commands for enable/disable
        self.gcode.register_command('AFC_M109_DEADBAND_ENABLE',
                                   self.cmd_ENABLE,
                                   desc=self.cmd_ENABLE_help)
        self.gcode.register_command('AFC_M109_DEADBAND_DISABLE',
                                   self.cmd_DISABLE,
                                   desc=self.cmd_DISABLE_help)
        self.gcode.register_command('AFC_M109_DEADBAND_STATUS',
                                   self.cmd_STATUS,
                                   desc=self.cmd_STATUS_help)

    def handle_ready(self):
        """
        This runs after Klipper is ready. We re-register M109/AFC_M109 with our wrapper.
        """
        # Get reference to AFC object
        try:
            self.afc = self.printer.lookup_object('AFC')
        except:
            raise self.printer.config_error(
                "AFC_M109_deadband requires AFC to be loaded")

        # Save original AFC M109 function
        if hasattr(self.afc, '_cmd_AFC_M109'):
            self.original_afc_m109 = self.afc._cmd_AFC_M109

            # Create wrapper that will be registered as the command handler
            deadband_wrapper = self
            afc_instance = self.afc

            def wrapper(gcmd, wait=True):
                return deadband_wrapper.wrapped_afc_m109(afc_instance, gcmd, wait)

            # Use AFC's rename pattern to replace M109 and AFC_M109 handlers
            # 1. Unregister existing M109 and save old handler
            prev_m109 = self.gcode.register_command('M109', None)
            if prev_m109 is not None:
                # Register old handler with renamed name for fallback
                self.gcode.register_command('_AFC_DEADBAND_RENAMED_M109', prev_m109,
                                          desc="Original M109 handler before deadband wrapper")

            # 2. Unregister existing AFC_M109
            prev_afc_m109 = self.gcode.register_command('AFC_M109', None)
            if prev_afc_m109 is not None:
                # Register old handler with renamed name
                self.gcode.register_command('_AFC_DEADBAND_RENAMED_AFC_M109', prev_afc_m109,
                                          desc="Original AFC_M109 handler before deadband wrapper")

            # 3. Register our wrapper for both commands
            self.gcode.register_command('M109', wrapper,
                                       desc=self.afc._cmd_AFC_M109_help)
            self.gcode.register_command('AFC_M109', wrapper,
                                       desc=self.afc._cmd_AFC_M109_help)

            self.gcode.respond_info("AFC M109 Deadband: Registered command handlers")
        else:
            self.gcode.respond_info(
                "AFC_M109_deadband: Could not find AFC's M109 handler")

    cmd_ENABLE_help = "Enable automatic AFC deadband usage in M109"
    def cmd_ENABLE(self, gcmd):
        self.enabled = 1
        gcmd.respond_info("AFC M109 Deadband Override: ENABLED - "
                         "M109 will use AFC extruder deadband settings")

    cmd_DISABLE_help = "Disable automatic AFC deadband usage in M109"
    def cmd_DISABLE(self, gcmd):
        self.enabled = 0
        gcmd.respond_info("AFC M109 Deadband Override: DISABLED - "
                         "M109 will use default AFC behavior")

    cmd_STATUS_help = "Show status of AFC M109 deadband override"
    def cmd_STATUS(self, gcmd):
        status = "ENABLED" if self.enabled else "DISABLED"
        gcmd.respond_info("AFC M109 Deadband Override: %s" % status)

    def wrapped_afc_m109(self, afc_self, gcmd, wait=True):
        """
        Wrapper around AFC's M109 that automatically adds deadband
        Note: afc_self is the AFC instance, self is the AFC_M109_Deadband instance
        """
        # Get parameters from the command
        toolnum = gcmd.get_int('T', None, minval=0)
        temp = gcmd.get_float('S', 0.0)
        deadband_override = gcmd.get_float('D', None)

        # If override is enabled and no deadband was manually specified
        if self.enabled and deadband_override is None and temp > 0 and wait:
            deadband = self._get_deadband_for_tool(toolnum)
            if deadband is not None:
                # We need to handle the M109 logic ourselves with the deadband
                # This replicates AFC's _cmd_AFC_M109 logic but with our deadband

                # Determine which extruder to use.
                if toolnum is not None:
                    extruder_name = "extruder" if toolnum == 0 else "extruder%d" % toolnum
                    extruder = self.printer.lookup_object(extruder_name, None)
                    if extruder is None:
                        map_str = "T{}".format(toolnum)
                        lane = self.afc.function.get_lane_by_map(map_str)
                        if lane is not None:
                            extruder = lane.extruder_obj
                        if extruder is None:
                            self.afc.logger.error("extruder not configured for T{}".format(toolnum))
                            return
                else:
                    next_lane_name = getattr(afc_self, "next_lane_load", None)
                    next_lane = afc_self.lanes.get(next_lane_name) if next_lane_name else None
                    if next_lane is not None:
                        extruder = next_lane.extruder_obj
                        extruder_name = extruder.get_name()
                    else:
                        toolhead = self.printer.lookup_object('toolhead')
                        extruder = toolhead.get_extruder()
                        extruder_name = extruder.get_name()

                # Get heater and set temperature
                pheaters = self.printer.lookup_object('heaters')
                heater = extruder.get_heater()

                # Log what we're doing
                gcmd.respond_info("M109: Using AFC deadband %.1fC for %s" %
                                (deadband, extruder_name))

                # Set temperature without waiting
                pheaters.set_temperature(heater, temp, False)

                # Now wait with our deadband tolerance
                self.afc._wait_for_temp_within_tolerance(heater, temp, deadband)
                return

        # Call original AFC M109 function
        self.original_afc_m109(gcmd, wait)

    def _get_deadband_for_tool(self, toolnum):
        """
        Get the deadband value from AFC extruder configuration
        """
        # Determine which extruder to look up
        if toolnum is not None:
            extruder_name = "extruder" if toolnum == 0 else "extruder%d" % toolnum
        else:
            # Get current extruder
            toolhead = self.printer.lookup_object('toolhead')
            extruder_name = toolhead.get_extruder().get_name()

        # Look up AFC extruder object
        afc_extruder_name = "AFC_extruder %s" % extruder_name
        try:
            afc_extruder = self.printer.lookup_object(afc_extruder_name)
            # Get deadband attribute
            if hasattr(afc_extruder, 'deadband'):
                return afc_extruder.deadband
        except:
            pass

        return None

def load_config(config):
    return AFC_M109_Deadband(config)