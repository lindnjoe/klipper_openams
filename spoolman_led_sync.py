"""
OpenAMS Spoolman LED Sync Module

Optional module that sets the ACTIVE TOOL LED to match Spoolman filament color
instead of the default blue color. Non-active lanes use their normal AFC colors.

Requires AFC and Spoolman integration to be active.

Configuration:
[spoolman_led_sync]
enable: True  # Set to False to disable
default_color: 0000FF  # Hex color for lanes without Spoolman data (default: blue)
"""

import logging

class SpoolmanLEDSync:
    def __init__(self, printer, config):
        self.printer = printer
        self.name = config.get_name()
        self.logger = logging.getLogger(self.name)

        # Configuration
        self.enabled = config.getboolean('enable', False)
        self.default_color = config.get('default_color', '0000FF')

        if not self.enabled:
            self.logger.info("Spoolman LED sync disabled")
            return

        self.logger.info("Spoolman LED sync enabled")

        # Defer AFC lookup until ready
        self.afc = None

        # Register for ready event to hook after all units are loaded
        printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        """Hook into AFC after all units are loaded (klippy:ready event)"""
        self.logger.info("Spoolman LED sync: klippy:ready event fired")

        try:
            self.afc = self.printer.lookup_object('AFC')
            self.logger.info("Found AFC object")
            self._connect_to_afc()
        except Exception as e:
            self.logger.error("Failed to find AFC: %s", e)
            self.enabled = False

    def _connect_to_afc(self):
        """Hook into AFC's LED system"""
        try:
            if not hasattr(self.afc, 'function'):
                self.logger.error("AFC object has no 'function' attribute")
                self.enabled = False
                return

            self.logger.info("AFC has function attribute, attempting hook...")

            # Register for toolhead activation events
            # AFC calls handle_activate_extruder when lanes change
            self._hook_into_afc()

            self.logger.info("Hook complete, module fully initialized")

        except Exception as e:
            self.logger.exception("Failed to hook into AFC")
            self.enabled = False

    def _hook_into_afc(self):
        """
        Hook into AFC's lane activation system without modifying AFC code.
        We wrap each unit's lane_tool_loaded function to override LED color.
        """
        try:
            # Check if AFC has units
            if not hasattr(self.afc, 'units'):
                self.logger.error("AFC object has no 'units' attribute")
                return

            num_units = len(self.afc.units)
            self.logger.info("AFC has %d units registered", num_units)

            if num_units == 0:
                self.logger.warning("No units registered in AFC yet - cannot hook")
                return

            # Hook into each unit's lane_tool_loaded method
            units_hooked = 0
            for unit_name, unit_obj in self.afc.units.items():
                try:
                    self.logger.info("Attempting to hook unit: %s", unit_name)
                    original_lane_tool_loaded = unit_obj.lane_tool_loaded

                    def make_wrapped_lane_tool_loaded(original_func, unit_name):
                        """Create a wrapper with closure over original function"""
                        def wrapped_lane_tool_loaded(lane):
                            # Check if lane has Spoolman color
                            hex_color = self._get_lane_color(lane)
                            if hex_color and hex_color != self.default_color:
                                # Use Spoolman color instead of default blue
                                led_color_str = self._hex_to_led_string(hex_color)
                                self.logger.info("Setting active tool %s LED to Spoolman color %s",
                                               lane.name, hex_color)
                                self.afc.function.afc_led(led_color_str, lane.led_index)
                            else:
                                # No Spoolman color, use default behavior
                                original_func(lane)
                        return wrapped_lane_tool_loaded

                    unit_obj.lane_tool_loaded = make_wrapped_lane_tool_loaded(original_lane_tool_loaded, unit_name)
                    units_hooked += 1
                    self.logger.info("Successfully hooked lane_tool_loaded for unit: %s", unit_name)
                except Exception as e:
                    self.logger.error("Failed to hook unit %s: %s", unit_name, e)

            self.logger.info("Successfully hooked %d/%d units for LED color override", units_hooked, num_units)

        except Exception as e:
            self.logger.exception("Failed to hook into AFC units: %s", e)

    def _update_lane_leds(self):
        """Update LEDs for all lanes based on Spoolman colors"""
        self.logger.info("_update_lane_leds called (enabled=%s, afc=%s)", self.enabled, self.afc is not None)

        if not self.enabled or not self.afc:
            self.logger.warning("Skipping LED update - not enabled or no AFC")
            return

        try:
            lane_count = len(self.afc.lanes) if hasattr(self.afc, 'lanes') else 0
            self.logger.info("Updating LEDs for %d lanes", lane_count)

            for lane_name, lane in self.afc.lanes.items():
                self._set_lane_led_color(lane)
        except Exception as e:
            self.logger.exception("Error updating lane LEDs")

    def _set_lane_led_color(self, lane):
        """
        Set the active tool's LED to its Spoolman color instead of default blue.
        Only updates the lane that is currently loaded in the toolhead.
        """
        lane_name = getattr(lane, 'name', 'unknown')

        # Skip if lane has no LED
        if not hasattr(lane, 'led_index') or lane.led_index is None:
            self.logger.debug("Lane %s: No LED index, skipping", lane_name)
            return

        # Only set color for loaded lanes
        prep = hasattr(lane, 'prep_state') and lane.prep_state
        load = hasattr(lane, 'load_state') and lane.load_state

        if not (prep and load):
            self.logger.debug("Lane %s: Not loaded (prep=%s, load=%s), skipping", lane_name, prep, load)
            return

        # Check if this is the active toolhead lane
        is_active = (hasattr(lane, 'extruder_obj') and lane.extruder_obj is not None and
                     hasattr(lane.extruder_obj, 'lane_loaded') and
                     lane.extruder_obj.lane_loaded == lane.name)

        self.logger.info("Lane %s: loaded, is_active=%s", lane_name, is_active)

        if is_active:
            # For the active tool, override blue with Spoolman color
            hex_color = self._get_lane_color(lane)
            led_color_str = self._hex_to_led_string(hex_color)

            self.logger.info("Setting active tool %s LED to color %s (string: %s)",
                           lane_name, hex_color, led_color_str)

            try:
                self.afc.function.afc_led(led_color_str, lane.led_index)
                self.logger.info("Successfully set active tool LED for %s to color %s", lane_name, hex_color)
            except Exception as e:
                self.logger.exception("Failed to set LED for %s", lane_name)

    def _get_lane_color(self, lane):
        """
        Get the hex color for a lane from Spoolman data.
        Returns default color if Spoolman data not available.
        """
        # Try to get color from lane (set by Spoolman)
        if hasattr(lane, 'color') and lane.color:
            hex_color = lane.color
            # Remove # prefix if present
            if hex_color.startswith('#'):
                hex_color = hex_color[1:]
            return hex_color

        # Fallback to default blue
        return self.default_color

    def _hex_to_led_string(self, hex_color):
        """
        Convert hex color to AFC LED string format (R,G,B,W as floats 0-1).
        Uses AFC's existing HexToLedString if available, otherwise implements it.
        """
        try:
            # Try to use AFC's built-in converter
            if hasattr(self.afc.function, 'HexToLedString'):
                led_values = self.afc.function.HexToLedString(hex_color)
                # Convert list to comma-separated string
                return ','.join(str(v) for v in led_values)
        except Exception as e:
            self.logger.debug("Couldn't use AFC's HexToLedString, using fallback: %s", e)

        # Fallback implementation
        try:
            # Parse hex: RRGGBB
            if len(hex_color) >= 6:
                r = int(hex_color[0:2], 16) / 255.0
                g = int(hex_color[2:4], 16) / 255.0
                b = int(hex_color[4:6], 16) / 255.0
            else:
                # Invalid hex, use default
                r, g, b = 0.0, 0.0, 1.0  # Blue

            # Add white channel (0 unless pure white)
            w = 1.0 if hex_color.upper() == "FFFFFF" else 0.0

            return f"{r},{g},{b},{w}"
        except Exception as e:
            self.logger.error("Failed to convert hex %s: %s", hex_color, e)
            return "0,0,1,0"  # Blue fallback

def load_config(config):
    return SpoolmanLEDSync(config.get_printer(), config)
