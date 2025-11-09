"""
OpenAMS Spoolman LED Sync Module

Optional module that sets lane LED colors to match Spoolman filament color
or custom configured colors. Supports all LED states including active tool,
ready, not ready, loading, unloading, fault, etc.

Requires AFC and Spoolman integration to be active.

Configuration:
[spoolman_led_sync]
enable: True  # Set to False to disable

# LED Color Settings (all optional, default to AFC's configured colors)
# Format: RRGGBB (6-digit hex without # prefix)
# When Spoolman data is available, filament color overrides these for loaded states

default_color: 0000FF        # Active tool without Spoolman data (default: blue)
ready_color:                 # Lane ready (filament at load sensor) - uses Spoolman color or this
not_ready_color:             # Lane without filament
loading_color:               # Lane loading filament
prep_loaded_color:           # Lane prep sensor loaded
unloading_color:             # Lane unloading filament
fault_color:                 # Lane fault state
tool_loaded_idle_color:      # Tool loaded but idle
"""

import logging

class SpoolmanLEDSync:
    def __init__(self, printer, config):
        self.printer = printer
        self.name = config.get_name()
        self.logger = logging.getLogger(self.name)

        # Configuration
        self.enabled = config.getboolean('enable', False)

        # LED color overrides (None means use AFC's default)
        self.default_color = config.get('default_color', '0000FF')  # Tool loaded fallback
        self.ready_color = config.get('ready_color', None)
        self.not_ready_color = config.get('not_ready_color', None)
        self.loading_color = config.get('loading_color', None)
        self.prep_loaded_color = config.get('prep_loaded_color', None)
        self.unloading_color = config.get('unloading_color', None)
        self.fault_color = config.get('fault_color', None)
        self.tool_loaded_idle_color = config.get('tool_loaded_idle_color', None)

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

            # Hook the main afc_led function to intercept ALL LED changes
            self._hook_afc_led_function()

            # Also hook unit methods for additional context
            self._hook_into_afc()

            self.logger.info("Hook complete, module fully initialized")

        except Exception as e:
            self.logger.exception("Failed to hook into AFC")
            self.enabled = False

    def _hook_afc_led_function(self):
        """Hook the main afc_led function to intercept all LED color changes"""
        try:
            original_afc_led = self.afc.function.afc_led

            def wrapped_afc_led(status, index):
                """Intercept LED changes and apply custom colors if configured"""
                # Try to find which lane this LED belongs to
                lane = self._find_lane_by_led_index(index)

                if lane:
                    # Check if we should override this color
                    new_status = self._get_override_color(status, lane)
                    if new_status is not None:
                        self.logger.debug("Overriding LED color for %s index %s: %s -> %s",
                                        lane.name, index, status, new_status)
                        status = new_status

                # Call original function with potentially modified color
                return original_afc_led(status, index)

            self.afc.function.afc_led = wrapped_afc_led
            self.logger.info("Successfully hooked afc_led function")

        except Exception as e:
            self.logger.exception("Failed to hook afc_led function: %s", e)

    def _find_lane_by_led_index(self, index):
        """Find the lane that owns this LED index"""
        if not hasattr(self.afc, 'lanes'):
            return None

        for lane in self.afc.lanes.values():
            if hasattr(lane, 'led_index') and lane.led_index == index:
                return lane
        return None

    def _get_override_color(self, original_color, lane):
        """
        Determine if we should override the color based on our configuration.
        Returns the override color string, or None if no override should happen.
        """
        # Match the original color to see what state is being set
        # and check if we have a custom color configured for that state

        # Check for Spoolman color for ready/loaded states
        spoolman_color = self._get_lane_color(lane)
        if spoolman_color:
            spoolman_led_str = self._hex_to_led_string(spoolman_color)

            # If setting tool_loaded color and we have Spoolman data, use it
            if (hasattr(lane, 'led_tool_loaded') and original_color == lane.led_tool_loaded):
                return spoolman_led_str

            # If setting ready color and we don't have a custom ready_color configured, use Spoolman
            if (hasattr(lane, 'led_ready') and original_color == lane.led_ready and
                self.ready_color is None):
                return spoolman_led_str

        # Check for custom color overrides
        if self.ready_color and hasattr(lane, 'led_ready') and original_color == lane.led_ready:
            return self._hex_to_led_string(self.ready_color)

        if self.not_ready_color and hasattr(lane, 'led_not_ready') and original_color == lane.led_not_ready:
            return self._hex_to_led_string(self.not_ready_color)

        if self.loading_color and hasattr(lane, 'led_loading') and original_color == lane.led_loading:
            return self._hex_to_led_string(self.loading_color)

        if self.prep_loaded_color and hasattr(lane, 'led_prep_loaded') and original_color == lane.led_prep_loaded:
            return self._hex_to_led_string(self.prep_loaded_color)

        if self.unloading_color and hasattr(lane, 'led_unloading') and original_color == lane.led_unloading:
            return self._hex_to_led_string(self.unloading_color)

        if self.fault_color and hasattr(lane, 'led_fault') and original_color == lane.led_fault:
            return self._hex_to_led_string(self.fault_color)

        if self.tool_loaded_idle_color and hasattr(lane, 'led_tool_loaded_idle') and original_color == lane.led_tool_loaded_idle:
            return self._hex_to_led_string(self.tool_loaded_idle_color)

        # Also check AFC global colors
        if self.not_ready_color and hasattr(self.afc, 'led_not_ready') and original_color == self.afc.led_not_ready:
            return self._hex_to_led_string(self.not_ready_color)

        if self.fault_color and hasattr(self.afc, 'led_fault') and original_color == self.afc.led_fault:
            return self._hex_to_led_string(self.fault_color)

        return None

    def _hook_into_afc(self):
        """
        Hook into AFC's lane LED system without modifying AFC code.
        We wrap multiple unit LED functions to override colors.
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

            # Hook into each unit's LED methods
            units_hooked = 0
            for unit_name, unit_obj in self.afc.units.items():
                try:
                    self.logger.info("Attempting to hook unit: %s", unit_name)

                    # Hook lane_tool_loaded
                    if hasattr(unit_obj, 'lane_tool_loaded'):
                        original_lane_tool_loaded = unit_obj.lane_tool_loaded
                        unit_obj.lane_tool_loaded = self._make_wrapped_lane_tool_loaded(original_lane_tool_loaded)

                    # Hook lane_loaded (ready state)
                    if hasattr(unit_obj, 'lane_loaded'):
                        original_lane_loaded = unit_obj.lane_loaded
                        unit_obj.lane_loaded = self._make_wrapped_lane_loaded(original_lane_loaded)

                    # Hook lane_tool_unloaded (goes to ready state)
                    if hasattr(unit_obj, 'lane_tool_unloaded'):
                        original_lane_tool_unloaded = unit_obj.lane_tool_unloaded
                        unit_obj.lane_tool_unloaded = self._make_wrapped_lane_tool_unloaded(original_lane_tool_unloaded)

                    # Hook lane_unloaded (not ready state)
                    if hasattr(unit_obj, 'lane_unloaded'):
                        original_lane_unloaded = unit_obj.lane_unloaded
                        unit_obj.lane_unloaded = self._make_wrapped_lane_unloaded(original_lane_unloaded)

                    # Hook lane_loading
                    if hasattr(unit_obj, 'lane_loading'):
                        original_lane_loading = unit_obj.lane_loading
                        unit_obj.lane_loading = self._make_wrapped_lane_loading(original_lane_loading)

                    units_hooked += 1
                    self.logger.info("Successfully hooked LED functions for unit: %s", unit_name)
                except Exception as e:
                    self.logger.error("Failed to hook unit %s: %s", unit_name, e)

            self.logger.info("Successfully hooked %d/%d units for LED color override", units_hooked, num_units)

        except Exception as e:
            self.logger.exception("Failed to hook into AFC units: %s", e)

    def _make_wrapped_lane_tool_loaded(self, original_func):
        """Wrapper for lane_tool_loaded - active tool with Spoolman color or default"""
        def wrapped(lane):
            hex_color = self._get_lane_color(lane)
            if hex_color and hex_color != self.default_color:
                # Use Spoolman color
                led_color_str = self._hex_to_led_string(hex_color)
                self.logger.info("Setting active tool %s LED to Spoolman color %s", lane.name, hex_color)
                self.afc.function.afc_led(led_color_str, lane.led_index)
            else:
                # Use configured default or AFC default
                original_func(lane)
        return wrapped

    def _make_wrapped_lane_loaded(self, original_func):
        """Wrapper for lane_loaded - ready state, can use Spoolman color or custom ready color"""
        def wrapped(lane):
            # Try Spoolman color first if configured to use it
            hex_color = self._get_lane_color(lane) if self.ready_color is None else None

            if hex_color and hex_color != self.default_color:
                # Use Spoolman color for ready state
                led_color_str = self._hex_to_led_string(hex_color)
                self.logger.debug("Setting ready lane %s LED to Spoolman color %s", lane.name, hex_color)
                self.afc.function.afc_led(led_color_str, lane.led_index)
            elif self.ready_color:
                # Use custom ready color
                led_color_str = self._hex_to_led_string(self.ready_color)
                self.logger.debug("Setting ready lane %s LED to custom color %s", lane.name, self.ready_color)
                self.afc.function.afc_led(led_color_str, lane.led_index)
            else:
                # Use AFC default
                original_func(lane)
        return wrapped

    def _make_wrapped_lane_tool_unloaded(self, original_func):
        """Wrapper for lane_tool_unloaded - back to ready state"""
        def wrapped(lane):
            if self.ready_color:
                led_color_str = self._hex_to_led_string(self.ready_color)
                self.logger.debug("Setting unloaded tool lane %s LED to ready color %s", lane.name, self.ready_color)
                self.afc.function.afc_led(led_color_str, lane.led_index)
            else:
                original_func(lane)
        return wrapped

    def _make_wrapped_lane_unloaded(self, original_func):
        """Wrapper for lane_unloaded - not ready state"""
        def wrapped(lane):
            if self.not_ready_color:
                led_color_str = self._hex_to_led_string(self.not_ready_color)
                self.logger.debug("Setting unloaded lane %s LED to not ready color %s", lane.name, self.not_ready_color)
                self.afc.function.afc_led(led_color_str, lane.led_index)
            else:
                original_func(lane)
        return wrapped

    def _make_wrapped_lane_loading(self, original_func):
        """Wrapper for lane_loading"""
        def wrapped(lane):
            if self.loading_color:
                led_color_str = self._hex_to_led_string(self.loading_color)
                self.logger.debug("Setting loading lane %s LED to loading color %s", lane.name, self.loading_color)
                self.afc.function.afc_led(led_color_str, lane.led_index)
            else:
                original_func(lane)
        return wrapped

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
    return SpoolmanLEDSync(config.get_printer(), config)
