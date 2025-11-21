# OpenAMS Manager
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# OPTIMIZATIONS APPLIED:
# 1. Adaptive Polling Intervals: Monitors use 2.0s active / 4.0s idle intervals
#    to reduce CPU usage when printer is idle (15-25% reduction in polling overhead)
# 2. Object Caching: Frequently accessed objects (idle_timeout, gcode, toolhead, AFC)
#    are cached at initialization to avoid repeated lookups
# 3. State Change Tracking: FPSState tracks consecutive idle polls to intelligently
#    switch between active and idle polling intervals
# 4. Pre-checks: Monitor functions skip expensive sensor reads when idle and stable
#
# CONFIGURABLE DETECTION PARAMETERS:
# The following parameters can be tuned in [oams_manager] config section:
# - stuck_spool_load_grace: Grace period (seconds) after load before stuck spool detection (default: 8.0)
# - stuck_spool_pressure_threshold: Pressure below which stuck detection starts (default: 0.08)
# - stuck_spool_pressure_clear_threshold: Pressure above which stuck state clears - hysteresis (default: 0.12)
# - clog_pressure_target: Target FPS pressure for clog detection (default: 0.50)
# - post_load_pressure_dwell: Duration (seconds) to monitor pressure after load (default: 15.0)
# - load_fps_stuck_threshold: FPS pressure above which load is considered failed (default: 0.75)
# - clog_sensitivity: Detection sensitivity level - "low", "medium", "high" (default: "medium")

import logging
import time
from functools import partial
from typing import Optional, Tuple, Dict, List, Any, Callable

try:
    from extras.ams_integration import AMSRunoutCoordinator
except Exception:
    AMSRunoutCoordinator = None

# Configuration constants
PAUSE_DISTANCE = 60
MIN_ENCODER_DIFF = 1
FILAMENT_PATH_LENGTH_FACTOR = 1.14
MONITOR_ENCODER_PERIOD = 2.0
MONITOR_ENCODER_PERIOD_IDLE = 4.0  # OPTIMIZATION: Longer interval when idle
MONITOR_ENCODER_SPEED_GRACE = 2.0
AFC_DELEGATION_TIMEOUT = 30.0
IDLE_POLL_THRESHOLD = 3  # OPTIMIZATION: Polls before switching to idle interval

STUCK_SPOOL_PRESSURE_THRESHOLD = 0.08
STUCK_SPOOL_PRESSURE_CLEAR_THRESHOLD = 0.12  # Hysteresis upper threshold
STUCK_SPOOL_DWELL = 3.5
STUCK_SPOOL_LOAD_GRACE = 8.0

CLOG_PRESSURE_TARGET = 0.50
CLOG_SENSITIVITY_LEVELS = {
    "low": {"extrusion_window": 48.0, "encoder_slack": 15, "pressure_band": 0.08, "dwell": 12.0},
    "medium": {"extrusion_window": 24.0, "encoder_slack": 8, "pressure_band": 0.06, "dwell": 8.0},
    "high": {"extrusion_window": 12.0, "encoder_slack": 4, "pressure_band": 0.04, "dwell": 6.0},
}
CLOG_SENSITIVITY_DEFAULT = "medium"

POST_LOAD_PRESSURE_THRESHOLD = 0.65
POST_LOAD_PRESSURE_DWELL = 15.0
POST_LOAD_PRESSURE_CHECK_PERIOD = 0.5

# Threshold for detecting failed load - if FPS stays above this during LOADING, filament isn't engaging
LOAD_FPS_STUCK_THRESHOLD = 0.75


class OAMSRunoutState:
    """Enum for runout monitor states."""
    STOPPED = "STOPPED"
    MONITORING = "MONITORING"
    DETECTED = "DETECTED"
    COASTING = "COASTING"
    RELOADING = "RELOADING"
    PAUSED = "PAUSED"


class FPSLoadState:
    """Enum for FPS loading states"""
    UNLOADED = 0
    LOADED = 1
    LOADING = 2
    UNLOADING = 3
    
class OAMSRunoutMonitor:
    """Monitors filament runout for a specific FPS."""
    
    def __init__(self,
                 printer,
                 fps_name: str,
                 fps,
                 fps_state,
                 oams: Dict[str, Any],
                 reload_callback: Callable,
                 reload_before_toolhead_distance: float = 0.0):
        self.oams = oams
        self.printer = printer
        self.fps_name = fps_name
        self.fps_state = fps_state
        self.fps = fps
        
        self.state = OAMSRunoutState.STOPPED
        self.runout_position: Optional[float] = None
        self.bldc_clear_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        
        self.reload_before_toolhead_distance = reload_before_toolhead_distance
        self.reload_callback = reload_callback

        self.reactor = self.printer.get_reactor()

        self.hardware_service = None
        self.latest_lane_name: Optional[str] = None
        if AMSRunoutCoordinator is not None:
            try:
                self.hardware_service = AMSRunoutCoordinator.register_runout_monitor(self)
            except Exception as e:
                logging.getLogger(__name__).error(
                    "CRITICAL: Failed to register OpenAMS monitor with AFC (AMSRunoutCoordinator). "
                    "Infinite runout and AFC integration will not function. Error: %s", e
                )
                self.hardware_service = None
        
        def _monitor_runout(eventtime):
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            
            if self.state in (OAMSRunoutState.STOPPED, OAMSRunoutState.PAUSED, OAMSRunoutState.RELOADING):
                return eventtime + MONITOR_ENCODER_PERIOD
            
            if self.state == OAMSRunoutState.MONITORING:
                if getattr(fps_state, "afc_delegation_active", False):
                    now = self.reactor.monotonic()
                    if now < getattr(fps_state, "afc_delegation_until", 0.0):
                        return eventtime + MONITOR_ENCODER_PERIOD
                    fps_state.afc_delegation_active = False
                    fps_state.afc_delegation_until = 0.0
                
                oams_obj = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None
                if oams_obj is None:
                    return eventtime + MONITOR_ENCODER_PERIOD

                spool_idx = fps_state.current_spool_idx
                if spool_idx is None:
                    self.latest_lane_name = None
                    return eventtime + MONITOR_ENCODER_PERIOD

                lane_name = None
                spool_empty = None
                unit_name = getattr(fps_state, "current_oams", None) or self.fps_name

                if self.hardware_service is not None:
                    try:
                        lane_name = self.hardware_service.resolve_lane_for_spool(unit_name, spool_idx)
                        snapshot = self.hardware_service.latest_lane_snapshot_for_spool(unit_name, spool_idx)
                    except Exception:
                        snapshot = None
                    if snapshot:
                        hub_state = snapshot.get("hub_state")
                        lane_state = snapshot.get("lane_state")
                        if hub_state is not None:
                            spool_empty = not bool(hub_state)
                        elif lane_state is not None:
                            spool_empty = not bool(lane_state)

                if spool_empty is None:
                    try:
                        hes_values = oams_obj.hub_hes_value
                        if spool_idx < 0 or spool_idx >= len(hes_values):
                            return eventtime + MONITOR_ENCODER_PERIOD
                        spool_empty = not bool(hes_values[spool_idx])
                    except Exception:
                        logging.exception("OAMS: Failed to read HES values for runout detection on %s", self.fps_name)
                        return eventtime + MONITOR_ENCODER_PERIOD

                self.latest_lane_name = lane_name

                if (is_printing and fps_state.state == FPSLoadState.LOADED and 
                    fps_state.current_lane is not None and fps_state.current_spool_idx is not None and spool_empty):
                    self.state = OAMSRunoutState.DETECTED
                    logging.info("OAMS: Runout detected on FPS %s, pausing for %d mm", self.fps_name, PAUSE_DISTANCE)
                    self.runout_position = fps.extruder.last_position
                    if AMSRunoutCoordinator is not None:
                        try:
                            AMSRunoutCoordinator.notify_runout_detected(self, spool_idx, lane_name=lane_name)
                        except Exception:
                            logging.getLogger(__name__).exception("Failed to notify AFC about OpenAMS runout")

            elif self.state == OAMSRunoutState.DETECTED:
                traveled_distance = fps.extruder.last_position - self.runout_position
                if traveled_distance >= PAUSE_DISTANCE:
                    logging.info("OAMS: Pause complete, coasting the follower.")
                    try:
                        self.oams[fps_state.current_oams].set_oams_follower(0, 1)
                    except Exception:
                        logging.exception("OAMS: Failed to stop follower while coasting on %s", self.fps_name)
                    finally:
                        fps_state.following = False
                    self.bldc_clear_position = fps.extruder.last_position
                    self.runout_after_position = 0.0
                    self.state = OAMSRunoutState.COASTING

            elif self.state == OAMSRunoutState.COASTING:
                traveled_distance_after_bldc_clear = max(fps.extruder.last_position - self.bldc_clear_position, 0.0)
                self.runout_after_position = traveled_distance_after_bldc_clear
                try:
                    path_length = getattr(self.oams[fps_state.current_oams], "filament_path_length", 0.0)
                except Exception:
                    logging.exception("OAMS: Failed to read filament path length while coasting on %s", self.fps_name)
                    return eventtime + MONITOR_ENCODER_PERIOD
                
                effective_path_length = (path_length / FILAMENT_PATH_LENGTH_FACTOR if path_length else 0.0)
                consumed_with_margin = (self.runout_after_position + PAUSE_DISTANCE + self.reload_before_toolhead_distance)

                if consumed_with_margin >= effective_path_length:
                    logging.info("OAMS: Loading next spool (%.2f mm consumed)", self.runout_after_position + PAUSE_DISTANCE)
                    self.state = OAMSRunoutState.RELOADING
                    self.reload_callback()
            
            return eventtime + MONITOR_ENCODER_PERIOD
        
        self._timer_callback = _monitor_runout
        self.timer = None  # Don't register timer until start() is called

    def start(self) -> None:
        if self.timer is None:
            self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)
        self.state = OAMSRunoutState.MONITORING
    
    def stop(self) -> None:
        self.state = OAMSRunoutState.STOPPED
        
    def reloading(self) -> None:
        self.state = OAMSRunoutState.RELOADING
        self.runout_position = None
        self.runout_after_position = None
        
    def paused(self) -> None:
        self.state = OAMSRunoutState.PAUSED
        
    def reset(self) -> None:
        self.state = OAMSRunoutState.STOPPED
        self.runout_position = None
        self.runout_after_position = None
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None

class OAMSState:
    """Global state container for all FPS units."""
    def __init__(self):
        self.fps_state: Dict[str, 'FPSState'] = {}
        
    def add_fps_state(self, fps_name: str) -> None:
        self.fps_state[fps_name] = FPSState()
        

class FPSState:
    """Tracks the state of a single FPS"""
    
    def __init__(self, 
                 state: int = FPSLoadState.UNLOADED,
                 current_lane: Optional[str] = None, 
                 current_oams: Optional[str] = None, 
                 current_spool_idx: Optional[int] = None):
        
        self.state = state
        self.current_lane = current_lane
        self.current_oams = current_oams
        self.current_spool_idx = current_spool_idx
        
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None
        
        self.encoder_sample_prev: Optional[int] = None
        self.encoder_sample_current: Optional[int] = None

        self.following: bool = False
        self.direction: int = 0
        self.since: Optional[float] = None

        self.afc_delegation_active: bool = False
        self.afc_delegation_until: float = 0.0

        self.stuck_spool_start_time: Optional[float] = None
        self.stuck_spool_active: bool = False
        self.stuck_spool_restore_follower: bool = False
        self.stuck_spool_restore_direction: int = 1

        self.clog_active: bool = False
        self.clog_restore_follower: bool = False
        self.clog_restore_direction: int = 1
        self.clog_start_extruder: Optional[float] = None
        self.clog_start_encoder: Optional[int] = None
        self.clog_start_time: Optional[float] = None
        self.clog_min_pressure: Optional[float] = None
        self.clog_max_pressure: Optional[float] = None
        self.clog_last_extruder: Optional[float] = None

        self.post_load_pressure_timer = None
        self.post_load_pressure_start: Optional[float] = None

        # OPTIMIZATION: Adaptive polling state with exponential backoff
        self.consecutive_idle_polls: int = 0
        self.idle_backoff_level: int = 0  # 0-3 for exponential backoff (1x, 2x, 4x, 8x)
        self.last_state_change: Optional[float] = None

    def record_encoder_sample(self, value: int) -> Optional[int]:
        """Record encoder sample and return diff if we have 2 samples."""
        self.encoder_sample_prev = self.encoder_sample_current
        self.encoder_sample_current = value
        
        if self.encoder_sample_prev is not None:
            return abs(self.encoder_sample_current - self.encoder_sample_prev)
        return None
    
    def clear_encoder_samples(self):
        """Reset encoder tracking."""
        self.encoder_sample_prev = None
        self.encoder_sample_current = None

    def reset_runout_positions(self) -> None:
        self.runout_position = None
        self.runout_after_position = None

    def reset_stuck_spool_state(self, preserve_restore: bool = False) -> None:
        self.stuck_spool_start_time = None
        self.stuck_spool_active = False
        if not preserve_restore:
            self.stuck_spool_restore_follower = False
            self.stuck_spool_restore_direction = 1

    def reset_clog_tracker(self, preserve_restore: bool = False) -> None:
        self.clog_active = False
        self.clog_start_extruder = None
        self.clog_start_encoder = None
        self.clog_start_time = None
        self.clog_min_pressure = None
        self.clog_max_pressure = None
        self.clog_last_extruder = None
        if not preserve_restore:
            self.clog_restore_follower = False
            self.clog_restore_direction = 1

    def prime_clog_tracker(self, extruder_pos: float, encoder_clicks: int, pressure: float, timestamp: float) -> None:
        self.clog_start_extruder = extruder_pos
        self.clog_last_extruder = extruder_pos
        self.clog_start_encoder = encoder_clicks
        self.clog_start_time = timestamp
        self.clog_min_pressure = pressure
        self.clog_max_pressure = pressure
        
    def __repr__(self) -> str:
        state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
        return f"FPSState(state={state_names.get(self.state, self.state)}, lane={self.current_lane}, oams={self.current_oams}, spool={self.current_spool_idx})"


class OAMSManager:
    """Main coordinator for OpenAMS system"""
    
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.logger = logging.getLogger(__name__)

        self.oams: Dict[str, Any] = {}
        self.fpss: Dict[str, Any] = {}

        self.current_state = OAMSState()
        self.afc = None
        self._afc_logged = False

        self.monitor_timers: List[Any] = []
        self.runout_monitors: Dict[str, OAMSRunoutMonitor] = {}
        self.ready: bool = False

        self.reload_before_toolhead_distance: float = config.getfloat("reload_before_toolhead_distance", 0.0)

        sensitivity = config.get("clog_sensitivity", CLOG_SENSITIVITY_DEFAULT).lower()
        if sensitivity not in CLOG_SENSITIVITY_LEVELS:
            self.logger.warning("Unknown clog_sensitivity '%s', using %s", sensitivity, CLOG_SENSITIVITY_DEFAULT)
            sensitivity = CLOG_SENSITIVITY_DEFAULT
        self.clog_sensitivity = sensitivity
        self.clog_settings = CLOG_SENSITIVITY_LEVELS[self.clog_sensitivity]

        # Configurable detection thresholds and timing parameters with validation
        self.stuck_spool_load_grace = config.getfloat("stuck_spool_load_grace", STUCK_SPOOL_LOAD_GRACE, minval=0.0, maxval=60.0)
        self.stuck_spool_pressure_threshold = config.getfloat("stuck_spool_pressure_threshold", STUCK_SPOOL_PRESSURE_THRESHOLD, minval=0.0, maxval=1.0)
        self.stuck_spool_pressure_clear_threshold = config.getfloat("stuck_spool_pressure_clear_threshold", STUCK_SPOOL_PRESSURE_CLEAR_THRESHOLD, minval=0.0, maxval=1.0)
        self.clog_pressure_target = config.getfloat("clog_pressure_target", CLOG_PRESSURE_TARGET, minval=0.0, maxval=1.0)
        self.post_load_pressure_dwell = config.getfloat("post_load_pressure_dwell", POST_LOAD_PRESSURE_DWELL, minval=0.0, maxval=60.0)
        self.load_fps_stuck_threshold = config.getfloat("load_fps_stuck_threshold", LOAD_FPS_STUCK_THRESHOLD, minval=0.0, maxval=1.0)

        # Validate hysteresis: clear threshold must be > trigger threshold
        if self.stuck_spool_pressure_clear_threshold <= self.stuck_spool_pressure_threshold:
            raise config.error(
                f"stuck_spool_pressure_clear_threshold ({self.stuck_spool_pressure_clear_threshold}) "
                f"must be greater than stuck_spool_pressure_threshold ({self.stuck_spool_pressure_threshold})"
            )

        self._lane_unit_map: Dict[str, str] = {}
        self._lane_by_location: Dict[Tuple[str, int], str] = {}
        self._lane_to_fps_cache: Dict[str, str] = {}  # OPTIMIZATION: Lane?FPS direct mapping cache

        # OPTIMIZATION: Cache hardware service lookups
        self._hardware_service_cache: Dict[str, Any] = {}
        self._idle_timeout_obj = None
        self._gcode_obj = None
        self._toolhead_obj = None

        self._initialize_oams()

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("idle_timeout:printing", self._handle_printing_resumed)
        self.printer.register_event_handler("pause:resume", self._handle_printing_resumed)

        self.printer.add_object("oams_manager", self)
        self.register_commands()

    def get_status(self, eventtime: float) -> Dict[str, Dict[str, Any]]:
        """Return current status of all FPS units and OAMS hardware."""
        attributes: Dict[str, Dict[str, Any]] = {"oams": {}}

        for name, oam in self.oams.items():
            status_name = name.split()[-1]
            try:
                oam_status = {
                    "action_status": oam.action_status,
                    "action_status_code": oam.action_status_code,
                    "action_status_value": oam.action_status_value,
                }
            except Exception:
                self.logger.exception("Failed to fetch status from %s", name)
                oam_status = {"action_status": "error", "action_status_code": None, "action_status_value": None}
            attributes["oams"][status_name] = oam_status
            if status_name != name:
                attributes["oams"][name] = oam_status

        state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
        for fps_name, fps_state in self.current_state.fps_state.items():
            attributes[fps_name] = {
                "current_lane": fps_state.current_lane,
                "current_oams": fps_state.current_oams,
                "current_spool_idx": fps_state.current_spool_idx,
                "state_name": state_names.get(fps_state.state, str(fps_state.state)),
                "since": fps_state.since,
            }

        return attributes
    
    def determine_state(self) -> None:
        """Analyze hardware state and update FPS state tracking."""
        for fps_name, fps_state in self.current_state.fps_state.items():
            (
                fps_state.current_lane,
                current_oams,
                fps_state.current_spool_idx,
            ) = self.determine_current_loaded_lane(fps_name)

            if current_oams is not None:
                fps_state.current_oams = current_oams.name
            else:
                fps_state.current_oams = None

            if (fps_state.current_oams is not None and fps_state.current_spool_idx is not None):
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._ensure_forward_follower(fps_name, fps_state, "state detection")
            else:
                fps_state.state = FPSLoadState.UNLOADED
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                fps_state.clog_restore_follower = False
                fps_state.clog_restore_direction = 1
                self._cancel_post_load_pressure_check(fps_state)
        
    def handle_ready(self) -> None:
        """Initialize system when printer is ready."""
        for fps_name, fps in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)

        if not self.fpss:
            raise ValueError("No FPS found in system, this is required for OAMS to work")

        # OPTIMIZATION: Cache frequently accessed objects
        try:
            self._idle_timeout_obj = self.printer.lookup_object("idle_timeout")
        except Exception:
            self._idle_timeout_obj = None

        try:
            self._gcode_obj = self.printer.lookup_object("gcode")
        except Exception:
            self._gcode_obj = None

        try:
            self._toolhead_obj = self.printer.lookup_object("toolhead")
        except Exception:
            self._toolhead_obj = None

        self.determine_state()
        self.start_monitors()
        self.ready = True

    def _initialize_oams(self) -> None:
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def determine_current_loaded_lane(self, fps_name: str) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """Determine which lane is currently loaded in the specified FPS."""
        fps = self.fpss.get(fps_name)
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")

        # Lane-based detection only
        return self._determine_loaded_lane_for_fps(fps_name, fps)

    def _determine_loaded_lane_for_fps(self, fps_name: str, fps) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """Determine which AFC lane is loaded by asking AFC which lane is loaded to each extruder.

        With load_to_hub: False configuration, filament bypasses OAMS hub sensors and goes
        directly to toolhead. AFC tracks filament position via its own sensors and is the
        authoritative source for what's loaded.
        """
        afc = self._get_afc()
        if afc is None:
            self.logger.warning("State detection: AFC not found")
            return None, None, None

        if not hasattr(afc, 'tools'):
            self.logger.warning("State detection: AFC has no 'tools' attribute")
            return None, None, None

        # Check each AFC tool/extruder to see which lane is loaded
        for extruder_name, extruder_obj in afc.tools.items():
            loaded_lane_name = getattr(extruder_obj, 'lane_loaded', None)
            if not loaded_lane_name:
                continue

            # Check if this lane is on the current FPS
            lane_fps = self.get_fps_for_afc_lane(loaded_lane_name)
            if lane_fps != fps_name:
                continue  # This lane is on a different FPS

            # Get the lane object
            lane = afc.lanes.get(loaded_lane_name)
            if lane is None:
                self.logger.warning("Lane %s not found in afc.lanes", loaded_lane_name)
                continue

            # Get the OAMS and bay index for this lane
            unit_str = getattr(lane, "unit", None)
            if not unit_str:
                self.logger.warning("Lane %s has no unit", loaded_lane_name)
                continue

            # Parse unit and slot
            if isinstance(unit_str, str) and ':' in unit_str:
                base_unit_name, slot_str = unit_str.split(':', 1)
                try:
                    slot_number = int(slot_str)
                except ValueError:
                    self.logger.warning("Invalid slot number in unit %s", unit_str)
                    continue
            else:
                base_unit_name = str(unit_str)
                slot_number = getattr(lane, "index", None)
                if slot_number is None:
                    self.logger.warning("No index found for lane %s", loaded_lane_name)
                    continue

            # Convert to bay index
            bay_index = slot_number - 1
            if bay_index < 0:
                self.logger.warning("Invalid bay index %d (slot %d)", bay_index, slot_number)
                continue

            # Get OAMS name from AFC unit
            unit_obj = getattr(lane, "unit_obj", None)
            if unit_obj is None:
                units = getattr(afc, "units", {})
                unit_obj = units.get(base_unit_name)
            if unit_obj is None:
                self.logger.warning("Unit %s not found", base_unit_name)
                continue

            oams_name = getattr(unit_obj, "oams_name", None)
            if not oams_name:
                self.logger.warning("Unit %s has no oams_name", base_unit_name)
                continue

            # Find OAMS object - check both short and prefixed names
            oam = self.oams.get(oams_name)
            if oam is None:
                oam = self.oams.get(f"oams {oams_name}")
            if oam is None:
                self.logger.warning("OAMS %s not found", oams_name)
                continue

            # Found loaded lane! Return lane's map name (e.g., "T4") for display
            lane_map_name = lane.map if hasattr(lane, 'map') else loaded_lane_name
            self.logger.info("Detected %s loaded to %s (bay %d on %s)",
                           loaded_lane_name, extruder_name, bay_index, oams_name)
            return lane_map_name, oam, bay_index

        return None, None, None
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        commands = [
            ("OAMSM_UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT, self.cmd_UNLOAD_FILAMENT_help),
            ("OAMSM_LOAD_FILAMENT", self.cmd_LOAD_FILAMENT, self.cmd_LOAD_FILAMENT_help),
            ("OAMSM_FOLLOWER", self.cmd_FOLLOWER, self.cmd_FOLLOWER_help),
            ("OAMSM_CLEAR_ERRORS", self.cmd_CLEAR_ERRORS, self.cmd_CLEAR_ERRORS_help),
            ("OAMSM_STATUS", self.cmd_STATUS, self.cmd_STATUS_help),
        ]
        for cmd_name, handler, help_text in commands:
            gcode.register_command(cmd_name, handler, desc=help_text)
    
    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"
    def cmd_CLEAR_ERRORS(self, gcmd):
        if len(self.monitor_timers) > 0:
            self.stop_monitors()
        for fps_state in self.current_state.fps_state.values():
            fps_state.clear_encoder_samples()
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)

        for oam in self.oams.values():
            try:
                oam.clear_errors()
            except Exception:
                self.logger.exception("Failed to clear errors on %s", getattr(oam, "name", "<unknown>"))
        self.determine_state()
        self.start_monitors()

    cmd_STATUS_help = "Show OAMS manager state and run state detection diagnostics"
    def cmd_STATUS(self, gcmd):
        """Diagnostic command to show current state and test state detection."""
        afc = self._get_afc()

        gcmd.respond_info("=== OAMS Manager Status ===")

        # Show AFC info
        if afc is None:
            gcmd.respond_info("AFC: Not found!")
        else:
            gcmd.respond_info(f"AFC: Found, has {len(getattr(afc, 'tools', {}))} tools, {len(getattr(afc, 'lanes', {}))} lanes")

            # Show what AFC thinks is loaded
            if hasattr(afc, 'tools'):
                for tool_name, tool_obj in afc.tools.items():
                    lane_loaded = getattr(tool_obj, 'lane_loaded', None)
                    gcmd.respond_info(f"  Tool {tool_name}: lane_loaded={lane_loaded}")

        # Show FPS states
        for fps_name, fps_state in self.current_state.fps_state.items():
            gcmd.respond_info(f"\n{fps_name}:")
            gcmd.respond_info(f"  State: {fps_state.state}")
            gcmd.respond_info(f"  Current OAMS: {fps_state.current_oams}")
            gcmd.respond_info(f"  Current Lane: {fps_state.current_lane}")
            gcmd.respond_info(f"  Spool Index: {fps_state.current_spool_idx}")
            gcmd.respond_info(f"  Following: {fps_state.following}, Direction: {fps_state.direction}")

        # Run state detection and show results
        gcmd.respond_info("\n=== Running State Detection ===")
        self.determine_state()

        # Show state after detection
        for fps_name, fps_state in self.current_state.fps_state.items():
            gcmd.respond_info(f"\nAfter detection - {fps_name}:")
            gcmd.respond_info(f"  State: {fps_state.state}")
            gcmd.respond_info(f"  Current OAMS: {fps_state.current_oams}")
            gcmd.respond_info(f"  Current Lane: {fps_state.current_lane}")
            gcmd.respond_info(f"  Spool Index: {fps_state.current_spool_idx}")

        gcmd.respond_info("\nCheck klippy.log for detailed state detection logs")

    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        # DIRECTION is optional when disabling (ENABLE=0), defaults to 0
        direction = gcmd.get_int('DIRECTION', 0)
        fps_name = "fps " + gcmd.get('FPS')

        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return

        fps_state = self.current_state.fps_state[fps_name]

        # Allow enabling follower when UNLOADED (before load starts) or LOADED
        # Only block during active LOADING/UNLOADING operations
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            gcmd.respond_info(f"FPS {fps_name} is currently busy")
            return

        # Prevent manual control during active error conditions
        if fps_state.clog_active or fps_state.stuck_spool_active:
            gcmd.respond_info(
                f"FPS {fps_name} has active error condition "
                f"(clog_active={fps_state.clog_active}, stuck_spool_active={fps_state.stuck_spool_active}). "
                f"Use OAMSM_CLEAR_ERRORS first to clear error state."
            )
            return

        # When disabling (ENABLE=0), just disable regardless of state
        if not enable:
            # If already unloaded or no OAMS, just mark as not following and return
            if not fps_state.current_oams:
                fps_state.following = False
                self.logger.info("Follower disable requested on %s but no OAMS loaded, marking as not following", fps_name)
                return

            oams_obj = self.oams.get(fps_state.current_oams)
            if oams_obj:
                try:
                    oams_obj.set_oams_follower(0, direction)
                    fps_state.following = False
                    self.logger.info("Disabled follower on %s", fps_name)
                except Exception:
                    self.logger.exception("Failed to disable follower on %s", fps_state.current_oams)
                    gcmd.respond_info(f"Failed to disable follower. Check logs.")
            else:
                # OAMS not found but mark as not following anyway
                fps_state.following = False
                self.logger.info("Follower disable: OAMS %s not found, marking as not following", fps_state.current_oams)
            return

        # When enabling, we need a valid OAMS
        oams_obj = self.oams.get(fps_state.current_oams)
        if oams_obj is None:
            gcmd.respond_info(f"OAMS {fps_state.current_oams} is not available")
            return

        try:
            self.logger.info("OAMSM_FOLLOWER: enabling follower on %s, direction=%d", fps_name, direction)
            oams_obj.set_oams_follower(enable, direction)
            fps_state.following = bool(enable)
            fps_state.direction = direction
            self.logger.info("OAMSM_FOLLOWER: successfully enabled follower on %s", fps_name)
        except Exception:
            self.logger.exception("Failed to set follower on %s", fps_state.current_oams)
            gcmd.respond_info(f"Failed to set follower. Check logs.")

    def get_fps_for_afc_lane(self, lane_name: str) -> Optional[str]:
        """Get the FPS name for an AFC lane by querying its unit configuration.

        Returns the FPS name (e.g., "fps fps1") or None if not found.
        Uses cached mapping when available for performance.
        """
        # OPTIMIZATION: Check cache first
        cached = self._lane_to_fps_cache.get(lane_name)
        if cached is not None:
            return cached

        # Cache miss - compute and cache the result
        fps_name = self._compute_fps_for_afc_lane(lane_name)
        if fps_name is not None:
            self._lane_to_fps_cache[lane_name] = fps_name
        return fps_name

    def _compute_fps_for_afc_lane(self, lane_name: str) -> Optional[str]:
        """Compute the FPS name for an AFC lane (internal helper for caching)."""
        afc = self._get_afc()
        if afc is None:
            return None

        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None

        # Get the unit string (e.g., "AMS_1:1")
        unit_str = getattr(lane, "unit", None)
        if not unit_str or not isinstance(unit_str, str):
            return None

        # Extract base unit name (e.g., "AMS_1" from "AMS_1:1")
        if ':' in unit_str:
            base_unit_name = unit_str.split(':')[0]
        else:
            base_unit_name = unit_str

        # Look up the AFC unit object
        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None:
            units = getattr(afc, "units", {})
            unit_obj = units.get(base_unit_name)

        if unit_obj is None:
            return None

        # Get the OAMS name from the unit (e.g., "oams1")
        oams_name = getattr(unit_obj, "oams_name", None)
        if not oams_name:
            return None

        # Find which FPS has this OAMS
        for fps_name, fps in self.fpss.items():
            if hasattr(fps, "oams"):
                fps_oams = fps.oams
                # fps.oams could be a list or a single oams object
                if isinstance(fps_oams, list):
                    for oam in fps_oams:
                        oam_name_full = getattr(oam, "name", None)
                        # OAMS objects can be registered as "oams1", "oams oams1", or "OAMS oams1"
                        if (oam_name_full == oams_name or
                            oam_name_full == f"oams {oams_name}" or
                            oam_name_full == f"OAMS {oams_name}"):
                            return fps_name
                else:
                    oam_name_check = getattr(fps_oams, "name", None)
                    if (oam_name_check == oams_name or
                        oam_name_check == f"oams {oams_name}" or
                        oam_name_check == f"OAMS {oams_name}"):
                        return fps_name

        return None


    def _rebuild_lane_location_index(self) -> None:
        """No longer needed - using lane-based detection only."""
        pass

    def _validate_afc_oams_integration(self, afc) -> None:
        """Validate AFC lane configs match OAMS hardware configuration.

        Checks for common integration issues:
        - Lanes without unit definitions
        - Lanes referencing non-existent AFC units
        - Units without OAMS names
        - OAMS names that don't exist in OAMS manager
        - Lanes that can't be mapped to any FPS
        """
        lanes = getattr(afc, "lanes", {})
        if not lanes:
            return

        issues = []
        valid_lanes = 0

        for lane_name, lane in lanes.items():
            # Check lane has valid unit
            unit_str = getattr(lane, "unit", None)
            if not unit_str:
                issues.append(f"Lane {lane_name} has no unit defined")
                continue

            # Parse unit string to get base unit name
            if isinstance(unit_str, str) and ':' in unit_str:
                base_unit_name = unit_str.split(':')[0]
            else:
                base_unit_name = str(unit_str)

            # Check unit exists in AFC
            unit_obj = getattr(lane, "unit_obj", None)
            if unit_obj is None:
                units = getattr(afc, "units", {})
                unit_obj = units.get(base_unit_name)

            if unit_obj is None:
                issues.append(f"Lane {lane_name} references non-existent AFC unit '{base_unit_name}'")
                continue

            # Check unit has OAMS name
            oams_name = getattr(unit_obj, "oams_name", None)
            if not oams_name:
                issues.append(f"AFC unit {base_unit_name} has no oams_name defined")
                continue

            # Check OAMS exists in OAMS manager
            oam = self.oams.get(oams_name)
            if oam is None:
                oam = self.oams.get(f"oams {oams_name}")
            if oam is None:
                oam = self.oams.get(f"OAMS {oams_name}")
            if oam is None:
                issues.append(f"Lane {lane_name} references OAMS '{oams_name}' which doesn't exist in OAMS manager")
                continue

            # Check FPS can be found (use cache since we just built it)
            fps_name = self._lane_to_fps_cache.get(lane_name)
            if not fps_name:
                issues.append(f"Lane {lane_name} cannot be mapped to any FPS (OAMS {oams_name} not found in any FPS config)")
                continue

            valid_lanes += 1

        # Log results
        if issues:
            self.logger.warning("AFC-OAMS integration validation found %d issue(s):", len(issues))
            for issue in issues:
                self.logger.warning("  - %s", issue)
            if valid_lanes > 0:
                self.logger.info("AFC-OAMS integration: %d lanes configured correctly", valid_lanes)
        else:
            self.logger.info("AFC-OAMS integration validated: %d lanes configured correctly", valid_lanes)

    def _ensure_afc_lane_cache(self, afc) -> None:
        """Build caches for AFC lane metadata and mappings."""
        lanes = getattr(afc, "lanes", {})
        cache_built = False

        for lane_name, lane in lanes.items():
            # Cache unit mapping
            unit_name = getattr(lane, "unit", None)
            if unit_name and lane_name not in self._lane_unit_map:
                self._lane_unit_map[lane_name] = unit_name

            # OPTIMIZATION: Pre-populate lane?FPS cache
            if lane_name not in self._lane_to_fps_cache:
                fps_name = self._compute_fps_for_afc_lane(lane_name)
                if fps_name is not None:
                    self._lane_to_fps_cache[lane_name] = fps_name
                    cache_built = True

        # Validate AFC-OAMS integration after building cache
        if cache_built and not self._afc_logged:
            self._validate_afc_oams_integration(afc)

    def _resolve_lane_for_state(self, fps_state: 'FPSState', lane_name: Optional[str], afc) -> Tuple[Optional[str], Optional[str]]:
        """Resolve lane name from FPS state. Returns (lane_name, None) - group support removed."""
        # If lane_name provided, return it
        if lane_name:
            return lane_name, None

        # Try to get from current state (legacy location-based lookup)
        if fps_state.current_oams and fps_state.current_spool_idx is not None:
            located_lane = self._lane_by_location.get((fps_state.current_oams, fps_state.current_spool_idx))
            if located_lane:
                return located_lane, None

        return None, None

    def _get_afc(self):
        # OPTIMIZATION: Cache AFC object lookup with validation
        if self.afc is not None:
            # Validate cached object is still alive
            try:
                _ = self.afc.lanes  # Quick attribute access test
                return self.afc
            except Exception:
                self.logger.warning("Cached AFC object invalid, re-fetching")
                self.afc = None

        cached_afc = self._hardware_service_cache.get("afc_object")
        if cached_afc is not None:
            # Validate hardware service cache
            try:
                _ = cached_afc.lanes
                self.afc = cached_afc
                return self.afc
            except Exception:
                self.logger.warning("Cached AFC object in hardware service invalid, re-fetching")
                self._hardware_service_cache.pop("afc_object", None)

        try:
            afc = self.printer.lookup_object('AFC')
        except Exception:
            self.afc = None
            return None

        self.afc = afc
        self._hardware_service_cache["afc_object"] = afc
        self._ensure_afc_lane_cache(afc)
        if not self._afc_logged:
            self.logger.info("AFC integration detected; enabling same-FPS infinite runout support.")
            self._afc_logged = True
        return self.afc

    def _get_infinite_runout_target_lane(self, fps_name: str, fps_state: 'FPSState') -> Tuple[Optional[str], Optional[str], bool, Optional[str]]:
        """Get target lane for infinite runout.

        Returns: (target_lane_map, target_lane_name, delegate_to_afc, source_lane_name)
        - target_lane_map: Lane map attribute or lane name (for display)
        - target_lane_name: Actual target lane name to load
        - delegate_to_afc: True if AFC should handle (different FPS/extruder)
        - source_lane_name: Current source lane name
        """
        current_lane = fps_state.current_lane
        if not current_lane:
            return None, None, False, None

        afc = self._get_afc()
        if afc is None:
            return None, None, False, None

        lane_name, _ = self._resolve_lane_for_state(fps_state, current_lane, afc)

        if not lane_name:
            return None, None, False, None

        lanes = getattr(afc, "lanes", {})
        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None, None, False, lane_name

        runout_lane_name = getattr(lane, "runout_lane", None)
        if not runout_lane_name:
            return None, None, False, lane_name

        target_lane = afc.lanes.get(runout_lane_name)
        if target_lane is None:
            self.logger.warning("Runout lane %s for %s on %s is not available; deferring to AFC", runout_lane_name, lane_name, fps_name)
            return None, runout_lane_name, True, lane_name

        source_unit = self._lane_unit_map.get(lane_name)
        target_unit = self._lane_unit_map.get(runout_lane_name)
        if source_unit and target_unit and source_unit != target_unit:
            return None, runout_lane_name, True, lane_name

        source_extruder = getattr(lane, "extruder_obj", None)
        target_extruder = getattr(target_lane, "extruder_obj", None)
        if (source_extruder is not None and target_extruder is not None and source_extruder is not target_extruder):
            return None, runout_lane_name, True, lane_name

        # Check if both lanes are on the same FPS by querying their unit configurations
        # This replaces the group-based lookup system
        source_fps = self.get_fps_for_afc_lane(lane_name)
        target_fps = self.get_fps_for_afc_lane(runout_lane_name)

        # If we can't determine FPS for either lane, defer to AFC
        if source_fps is None or target_fps is None:
            self.logger.info("Cannot determine FPS for lanes %s or %s, deferring to AFC", lane_name, runout_lane_name)
            return None, runout_lane_name, True, lane_name

        # If lanes are on different FPS or not on the current FPS, defer to AFC
        if source_fps != fps_name or target_fps != fps_name:
            self.logger.info("Deferring infinite runout: %s on %s, %s on %s (current FPS: %s)",
                           lane_name, source_fps, runout_lane_name, target_fps, fps_name)
            return None, runout_lane_name, True, lane_name

        # If source and target are the same lane, defer to AFC (no swap needed)
        if lane_name == runout_lane_name:
            return None, runout_lane_name, True, lane_name

        # Both lanes are on the same FPS - OpenAMS can handle the swap internally
        self.logger.info("Infinite runout: %s -> %s on %s (same FPS)",
                       lane_name, runout_lane_name, fps_name)

        # Return target lane info (lane map or lane name for display)
        target_lane_map = getattr(target_lane, "map", runout_lane_name)

        return target_lane_map, runout_lane_name, False, lane_name

    def _delegate_runout_to_afc(self, fps_name: str, fps_state: 'FPSState', source_lane_name: Optional[str], target_lane_name: Optional[str]) -> bool:
        afc = self._get_afc()
        if afc is None:
            return False

        if not source_lane_name:
            return False

        lane = afc.lanes.get(source_lane_name)
        if lane is None:
            self.logger.warning("AFC lane %s not found while delegating infinite runout for %s", source_lane_name, fps_name)
            return False

        runout_target = getattr(lane, "runout_lane", None)
        if not runout_target:
            self.logger.warning("AFC lane %s has no runout target while delegating infinite runout for %s", source_lane_name, fps_name)
            return False

        if target_lane_name and target_lane_name != runout_target:
            pass

        now = self.reactor.monotonic()
        if fps_state.afc_delegation_active and now < fps_state.afc_delegation_until:
            return True

        if runout_target not in afc.lanes:
            self.logger.warning("AFC runout lane %s referenced by %s is unavailable", runout_target, source_lane_name)
            return False

        try:
            lane._perform_infinite_runout()
        except Exception:
            self.logger.exception("AFC infinite runout failed for lane %s -> %s", source_lane_name, runout_target)
            fps_state.afc_delegation_active = False
            fps_state.afc_delegation_until = 0.0
            return False

        fps_state.afc_delegation_active = True
        fps_state.afc_delegation_until = now + AFC_DELEGATION_TIMEOUT
        self.logger.info("Delegated infinite runout for %s via AFC lane %s -> %s", fps_name, source_lane_name, runout_target)
        return True

    def _unload_filament_for_fps(self, fps_name: str) -> Tuple[bool, str]:
        if fps_name not in self.fpss:
            return False, f"FPS {fps_name} does not exist"

        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.UNLOADED:
            return False, f"FPS {fps_name} is already unloaded"
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            return False, f"FPS {fps_name} is busy ({fps_state.state.name}), cannot unload"
        if fps_state.state != FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is in unexpected state {fps_state.state.name}"

        if fps_state.current_oams is None:
            return False, f"FPS {fps_name} has no OAMS loaded"

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return False, f"OAMS {fps_state.current_oams} not found for FPS {fps_name}"

        if oams.current_spool is None:
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.clog_restore_follower = False
            fps_state.clog_restore_direction = 1
            fps_state.current_lane = None
            fps_state.current_spool_idx = None
            fps_state.since = self.reactor.monotonic()
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, "Spool already unloaded"

        lane_name: Optional[str] = None
        spool_index = fps_state.current_spool_idx
        if AMSRunoutCoordinator is not None:
            try:
                afc = self._get_afc()
                if afc is not None:
                    lane_name, _ = self._resolve_lane_for_state(fps_state, fps_state.current_lane, afc)
            except Exception:
                self.logger.exception("Failed to resolve AFC lane for unload on %s", fps_name)
                lane_name = None

        # Capture state BEFORE changing fps_state.state to avoid getting stuck
        try:
            encoder = oams.encoder_clicks
            current_time = self.reactor.monotonic()
            current_oams_name = oams.name
            current_spool = oams.current_spool
        except Exception:
            self.logger.exception("Failed to capture unload state for %s", fps_name)
            return False, f"Failed to prepare unload on {fps_name}"

        # Only set state after all preliminary operations succeed
        fps_state.state = FPSLoadState.UNLOADING
        fps_state.encoder = encoder
        fps_state.since = current_time
        fps_state.current_oams = current_oams_name
        fps_state.current_spool_idx = current_spool
        fps_state.clear_encoder_samples()  # Clear stale encoder samples

        # Cancel post-load pressure check to prevent false positive clog detection during unload
        self._cancel_post_load_pressure_check(fps_state)

        try:
            success, message = oams.unload_spool_with_retry()
        except Exception:
            self.logger.exception("Exception while unloading filament on %s", fps_name)
            # Reset state on exception to avoid getting stuck
            fps_state.state = FPSLoadState.LOADED
            return False, f"Exception unloading filament on {fps_name}"

        if success:
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.clog_restore_follower = False
            fps_state.clog_restore_direction = 1
            fps_state.since = self.reactor.monotonic()
            if lane_name:
                try:
                    AMSRunoutCoordinator.notify_lane_tool_state(self.printer, fps_state.current_oams or oams.name, lane_name, loaded=False, spool_index=spool_index, eventtime=fps_state.since)
                except Exception:
                    self.logger.exception("Failed to notify AFC that lane %s unloaded on %s", lane_name, fps_name)

            # Clear LED error state if stuck spool was active before resetting state
            if fps_state.stuck_spool_active and oams is not None and spool_index is not None:
                try:
                    oams.set_led_error(spool_index, 0)
                    self.logger.info("Cleared stuck spool LED for %s spool %d after successful unload", fps_name, spool_index)
                except Exception:
                    self.logger.exception("Failed to clear LED on %s spool %d after successful unload", fps_name, spool_index)

            fps_state.current_lane = None
            fps_state.current_spool_idx = None
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, message

        fps_state.state = FPSLoadState.LOADED
        return False, message

    def _load_filament_for_lane(self, lane_name: str) -> Tuple[bool, str]:
        """Load filament for a lane by deriving OAMS and bay from the lane's unit configuration.

        This eliminates the need for [filament_group] configs by directly using:
        - lane.unit (e.g., "AMS_1:1") to get bay number
        - AFC_OpenAMS unit config to get OAMS name
        """
        afc = self._get_afc()
        if afc is None:
            return False, "AFC not available"

        lane = afc.lanes.get(lane_name)
        if lane is None:
            return False, f"Lane {lane_name} does not exist"

        # Get the unit string and slot/index
        # AFC stores "unit: AMS_1:1" as unit="AMS_1" and index stored separately
        unit_str = getattr(lane, "unit", None)
        if not unit_str:
            return False, f"Lane {lane_name} has no unit defined"

        # Try to get slot number from different possible attributes
        slot_number = None

        # Method 1: If unit_str contains ':', parse it directly (e.g., "AMS_1:1")
        if isinstance(unit_str, str) and ':' in unit_str:
            base_unit_name, slot_str = unit_str.split(':', 1)
            try:
                slot_number = int(slot_str)
            except ValueError:
                return False, f"Invalid slot number in unit {unit_str}"
        else:
            # Method 2: unit_str is just the unit name (e.g., "AMS_1"), get slot from lane.index
            base_unit_name = str(unit_str)
            slot_number = getattr(lane, "index", None)

            if slot_number is None:
                # Method 3: Try to get it from the lane's name if it follows a pattern
                # This is a fallback - lane names might not always have indices
                return False, f"Lane {lane_name} unit '{unit_str}' doesn't specify slot number"

        if slot_number is None:
            return False, f"Could not determine slot number for lane {lane_name}"

        # Convert slot number to 0-indexed bay number
        bay_index = slot_number - 1
        if bay_index < 0:
            return False, f"Invalid slot number {slot_number} (must be >= 1)"

        # Look up the AFC unit object to get OAMS name
        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None:
            units = getattr(afc, "units", {})
            unit_obj = units.get(base_unit_name)

        if unit_obj is None:
            return False, f"AFC unit {base_unit_name} not found"

        # Get the OAMS name from the unit (e.g., "oams1")
        oams_name = getattr(unit_obj, "oams_name", None)
        if not oams_name:
            return False, f"Unit {base_unit_name} has no oams_name defined"

        # Find the OAMS object
        # OAMS objects are stored with full name like "oams oams1", not just "oams1"
        oam = self.oams.get(oams_name)
        if oam is None:
            # Try with "oams " prefix
            oam = self.oams.get(f"oams {oams_name}")
        if oam is None:
            return False, f"OAMS {oams_name} not found"

        # Find which FPS has this OAMS
        fps_name = None
        for fps_name_candidate, fps in self.fpss.items():
            if hasattr(fps, "oams"):
                fps_oams = fps.oams
                if isinstance(fps_oams, list):
                    if oam in fps_oams:
                        fps_name = fps_name_candidate
                        break
                else:
                    if fps_oams == oam:
                        fps_name = fps_name_candidate
                        break

        if not fps_name:
            return False, f"No FPS found for OAMS {oams_name}"

        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is already loaded"

        self._cancel_post_load_pressure_check(fps_state)

        # Check if the bay is ready
        try:
            is_ready = oam.is_bay_ready(bay_index)
        except Exception:
            self.logger.exception("Failed to check bay %s readiness on %s", bay_index, oams_name)
            return False, f"Failed to check bay {bay_index} readiness on {oams_name}"

        if not is_ready:
            return False, f"Bay {bay_index} on {oams_name} is not ready (no spool detected)"

        # Load the filament
        self.logger.info("Loading lane %s: %s bay %s via %s", lane_name, oams_name, bay_index, fps_name)

        # Capture state BEFORE changing fps_state.state to avoid getting stuck
        try:
            encoder = oam.encoder_clicks
            current_time = self.reactor.monotonic()
            oam_name = oam.name
        except Exception:
            self.logger.exception("Failed to capture load state for lane %s bay %s", lane_name, bay_index)
            return False, f"Failed to capture load state for lane {lane_name}"

        # Only set state after all preliminary operations succeed
        fps_state.state = FPSLoadState.LOADING
        fps_state.encoder = encoder
        fps_state.current_oams = oam_name
        fps_state.current_spool_idx = bay_index
        # Set since to now for THIS load attempt (will be updated on success)
        fps_state.since = current_time
        fps_state.clear_encoder_samples()

        try:
            success, message = oam.load_spool_with_retry(bay_index)
        except Exception:
            self.logger.exception("Failed to load bay %s on %s", bay_index, oams_name)
            fps_state.state = FPSLoadState.UNLOADED
            return False, f"Failed to load bay {bay_index} on {oams_name}"

        if success:
            fps_state.current_lane = lane.map if hasattr(lane, 'map') else lane_name
            fps_state.current_oams = oam.name
            fps_state.current_spool_idx = bay_index

            # CRITICAL: Set fps_state.since to the successful load time BEFORE changing state
            successful_load_time = oam.get_last_successful_load_time(bay_index)
            if successful_load_time is not None:
                fps_state.since = successful_load_time
            else:
                fps_state.since = self.reactor.monotonic()

            # Now set state to LOADED after timestamp is correct
            fps_state.state = FPSLoadState.LOADED
            fps_state.direction = 1

            # OPTIMIZATION: Enable follower immediately before cleanup operations
            self._ensure_forward_follower(fps_name, fps_state, "load filament")

            # Clear LED error state if stuck spool was active
            if fps_state.stuck_spool_active:
                try:
                    oam.set_led_error(bay_index, 0)
                    self.logger.info("Cleared stuck spool LED for %s spool %d after successful load", fps_name, bay_index)
                except Exception:
                    self.logger.exception("Failed to clear LED on %s spool %d after successful load", fps_name, bay_index)

            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()

            # Monitors are already running globally, no need to restart them
            return True, f"Loaded lane {lane_name} ({oam_name} bay {bay_index})"
        else:
            fps_state.state = FPSLoadState.UNLOADED
            return False, message if message else f"Failed to load lane {lane_name}"


    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps_name = "fps " + gcmd.get('FPS')
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.UNLOADED:
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")
            return
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            gcmd.respond_info(f"FPS {fps_name} is currently busy")
            return

        success, message = self._unload_filament_for_fps(fps_name)
        if not success or (message and message != "Spool unloaded successfully"):
            gcmd.respond_info(message)

    cmd_LOAD_FILAMENT_help = "Load a spool from a specific AFC lane (LANE=name)"
    def cmd_LOAD_FILAMENT(self, gcmd):
        lane_name = gcmd.get('LANE', None)

        if not lane_name:
            gcmd.respond_info("LANE parameter is required (e.g., LANE=lane4)")
            return

        # Load directly from lane configuration
        success, message = self._load_filament_for_lane(lane_name)
        gcmd.respond_info(message)

    def _pause_printer_message(self, message, oams_name: Optional[str] = None):
        self.logger.info(message)

        if AMSRunoutCoordinator is not None and oams_name:
            try:
                AMSRunoutCoordinator.notify_afc_error(self.printer, oams_name, message, pause=False)
            except Exception:
                self.logger.exception("Failed to forward OAMS pause message to AFC")

        # OPTIMIZATION: Use cached gcode object
        gcode = self._gcode_obj
        if gcode is None:
            try:
                gcode = self.printer.lookup_object("gcode")
                self._gcode_obj = gcode
            except Exception:
                self.logger.exception("Failed to look up gcode object for pause message")
                return

        pause_message = f"Print has been paused: {message}"
        try:
            gcode.run_script(f"M118 {pause_message}")
            gcode.run_script(f"M114 {pause_message}")
        except Exception:
            self.logger.exception("Failed to send pause notification gcode")

        # OPTIMIZATION: Use cached toolhead object
        toolhead = self._toolhead_obj
        if toolhead is None:
            try:
                toolhead = self.printer.lookup_object("toolhead")
                self._toolhead_obj = toolhead
            except Exception:
                self.logger.exception("Failed to query toolhead state during pause handling")
                return

        try:
            homed_axes = toolhead.get_status(self.reactor.monotonic()).get("homed_axes", "")
        except Exception:
            self.logger.exception("Failed to query toolhead state during pause handling")
            return

        try:
            state_message = self.printer.get_state_message()
        except Exception:
            state_message = None

        if isinstance(state_message, (list, tuple)) and state_message:
            printer_state_text = state_message[0]
        else:
            printer_state_text = state_message

        printer_state_text = printer_state_text if isinstance(printer_state_text, str) else None

        if printer_state_text:
            lowered_state = printer_state_text.lower()
            if "lost communication" in lowered_state or "mcu" in lowered_state:
                self.logger.warning(
                    "Printer reported an error state during pause handling: %s",
                    printer_state_text,
                )
                gcode.respond_info(
                    f"Pause notification may fail because printer reported: {printer_state_text}"
                )

        already_paused = False
        try:
            pause_resume = self.printer.lookup_object("pause_resume")
        except Exception:
            pause_resume = None

        if pause_resume is not None:
            try:
                already_paused = bool(getattr(pause_resume, "is_paused", False))
            except Exception:
                already_paused = False

        if already_paused:
            self.logger.debug("Skipping PAUSE command because printer is already paused")
            return

        if all(axis in homed_axes for axis in ("x", "y", "z")):
            pause_attempted = False
            pause_successful = False
            try:
                gcode.run_script("PAUSE")
                pause_attempted = True

                # Verify pause state after attempting to pause
                if pause_resume is not None:
                    try:
                        pause_successful = bool(getattr(pause_resume, "is_paused", False))
                    except Exception:
                        self.logger.exception("Failed to verify pause state after PAUSE command")
            except Exception:
                self.logger.exception("Failed to run PAUSE script")

            if pause_attempted and not pause_successful:
                self.logger.error(
                    "CRITICAL: Failed to pause printer for critical error: %s. "
                    "Print may continue despite error condition!",
                    message
                )
        else:
            self.logger.warning("Skipping PAUSE command because axes are not homed (homed_axes=%s)", homed_axes)

    def _cancel_post_load_pressure_check(self, fps_state: "FPSState") -> None:
        timer = getattr(fps_state, "post_load_pressure_timer", None)
        if timer is not None:
            try:
                self.reactor.unregister_timer(timer)
            except Exception:
                self.logger.exception("Failed to cancel post-load pressure timer")
        fps_state.post_load_pressure_timer = None
        fps_state.post_load_pressure_start = None

    def _schedule_post_load_pressure_check(self, fps_name: str, fps_state: "FPSState") -> None:
        self._cancel_post_load_pressure_check(fps_state)

        def _monitor_pressure(self, eventtime):
            tracked_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)

            if tracked_state is None or fps is None:
                if tracked_state is not None:
                    self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            if tracked_state.state != FPSLoadState.LOADED:
                self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            pressure = float(getattr(fps, "fps_value", 0.0))
            if pressure <= POST_LOAD_PRESSURE_THRESHOLD:
                self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            now = self.reactor.monotonic()
            if tracked_state.post_load_pressure_start is None:
                tracked_state.post_load_pressure_start = now
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            if now - tracked_state.post_load_pressure_start < self.post_load_pressure_dwell:
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            oams_obj = None
            if tracked_state.current_oams is not None:
                oams_obj = self.oams.get(tracked_state.current_oams)
            if (oams_obj is not None and tracked_state.current_spool_idx is not None):
                try:
                    oams_obj.set_led_error(tracked_state.current_spool_idx, 1)
                except Exception:
                    self.logger.exception("Failed to set clog LED on %s spool %s after loading", fps_name, tracked_state.current_spool_idx)

            # Set restore flags and disable follower before pausing (matching runtime clog detection pattern)
            direction = tracked_state.direction if tracked_state.direction in (0, 1) else 1
            tracked_state.clog_restore_follower = True
            tracked_state.clog_restore_direction = direction

            # Disable follower first to give user manual control
            if oams_obj is not None and tracked_state.following:
                try:
                    oams_obj.set_oams_follower(0, direction)
                except Exception:
                    self.logger.exception("Failed to stop follower on %s during post-load clog pause", fps_name)
            tracked_state.following = False

            tracked_state.clog_active = True
            message = f"Possible clog detected after loading {tracked_state.current_lane or fps_name}: FPS pressure {pressure:.2f} remained above {POST_LOAD_PRESSURE_THRESHOLD:.2f}"
            self._pause_printer_message(message, tracked_state.current_oams)

            # Immediately re-enable follower so user can manually fix filament with follower tracking
            self._reactivate_clog_follower(fps_name, tracked_state, oams_obj, "post-load clog pause")

            self._cancel_post_load_pressure_check(tracked_state)
            return self.reactor.NEVER

        timer = self.reactor.register_timer(partial(_monitor_pressure, self), self.reactor.NOW)
        fps_state.post_load_pressure_timer = timer
        fps_state.post_load_pressure_start = None

    def _enable_follower(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], direction: int, context: str) -> None:
        """
        Enable the OAMS follower motor to track filament movement.

        The follower motor maintains proper tension on the filament by following its movement
        through the buffer tube. This is essential for accurate encoder tracking and preventing
        filament tangles.

        Args:
            fps_name: Name of the FPS (Filament Pressure Sensor) being controlled
            fps_state: Current state object for the FPS
            oams: OAMS object controlling the hardware (can be None, will be looked up)
            direction: Follower direction (0=reverse, 1=forward)
            context: Description of why follower is being enabled (for logging)

        State Updates:
            - fps_state.following: Set to True on success
            - fps_state.direction: Updated to match requested direction

        Notes:
            - Direction defaults to forward (1) if invalid value provided
            - Fails silently if no spool is loaded (current_spool_idx is None)
            - Logs exceptions but doesn't raise them to avoid disrupting workflow
        """
        if fps_state.current_spool_idx is None:
            return

        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            self.logger.warning("Cannot enable follower: OAMS not found")
            return

        direction = direction if direction in (0, 1) else 1

        try:
            oams.set_oams_follower(1, direction)
            fps_state.following = True
            fps_state.direction = direction
            self.logger.info("Follower enabled for %s spool %s (%s)",
                           fps_name, fps_state.current_spool_idx, context)
        except Exception:
            self.logger.exception("Failed to enable follower for %s after %s", fps_name, context)

    def _ensure_forward_follower(self, fps_name: str, fps_state: "FPSState", context: str) -> None:
        """Ensure follower is enabled in forward direction after successful load."""
        if (fps_state.current_oams is None or fps_state.current_spool_idx is None or
            fps_state.stuck_spool_active or fps_state.state != FPSLoadState.LOADED):
            return

        if fps_state.following and fps_state.direction == 1:
            return  # Already following in correct direction

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            self.logger.warning("Cannot enable follower: OAMS %s not found", fps_state.current_oams)
            return

        fps_state.direction = 1
        self._enable_follower(fps_name, fps_state, oams, 1, context)

    def _restore_follower_if_needed(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], context: str) -> None:
        if not fps_state.stuck_spool_restore_follower:
            return

        if fps_state.current_oams is None:
            fps_state.stuck_spool_restore_follower = False
            return

        if oams is None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        direction = fps_state.stuck_spool_restore_direction
        self._enable_follower(fps_name, fps_state, oams, direction, context)
        if fps_state.following:
            fps_state.stuck_spool_restore_follower = False
            self.logger.info("Restarted follower for %s spool %s after %s.", fps_name, fps_state.current_spool_idx, context)

    def _reactivate_clog_follower(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], context: str) -> None:
        """
        Restore follower motor after clog detection pause.

        FOLLOWER RESTORATION STATE MACHINE:

        When a clog is detected (runtime or post-load), the system follows this sequence:

        1. CLOG DETECTION:
           - Set clog_restore_follower = True
           - Set clog_restore_direction = current direction
           - DISABLE follower motor (set_oams_follower(0, direction))
           - Set fps_state.following = False
           - Pause printer

        2. IMMEDIATE RE-ENABLE (this method):
           - Called immediately after pause to allow manual filament manipulation
           - Attempts to re-enable follower with saved direction
           - If successful: clears clog_restore_follower flag
           - If failed: leaves flag set for resume handler to retry

        3. RESUME HANDLING (_handle_printing_resumed):
           - Clears clog_active flag (with preserve_restore=True)
           - Checks clog_restore_follower flag
           - If still set: attempts to restore follower again
           - Clears flags only after successful restoration

        This two-phase approach ensures:
        - User can manually adjust filament while paused (immediate re-enable)
        - System retries restoration on resume if immediate re-enable failed
        - Follower state is always consistent when printing resumes

        Args:
            fps_name: Name of the FPS being controlled
            fps_state: Current state object for the FPS
            oams: OAMS object (can be None, will be looked up)
            context: Description of when restoration is happening

        State Machine Flags:
            - clog_restore_follower: Set when clog detected, cleared after successful restore
            - clog_restore_direction: Saved direction (0=reverse, 1=forward)
            - following: Current motor state
        """
        if not fps_state.clog_restore_follower:
            return

        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        direction = fps_state.clog_restore_direction if fps_state.clog_restore_direction in (0, 1) else 1
        self._enable_follower(fps_name, fps_state, oams, direction, context)
        if fps_state.following:
            fps_state.clog_restore_follower = False
            fps_state.clog_restore_direction = 1

    def _handle_printing_resumed(self, _eventtime):
        # Check if monitors were stopped and need to be restarted
        if not self.monitor_timers:
            self.logger.info("Restarting monitors after pause/intervention")
            self.start_monitors()
        
        for fps_name, fps_state in self.current_state.fps_state.items():
            oams = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None
            
            # Clear stuck_spool_active on resume to allow follower to restart
            if fps_state.stuck_spool_active:
                fps_state.reset_stuck_spool_state(preserve_restore=True)
                self.logger.info("Cleared stuck spool state for %s on print resume", fps_name)
            
            # Clear clog_active on resume and reset tracker (preserve restore flags for follower)
            if fps_state.clog_active:
                fps_state.reset_clog_tracker(preserve_restore=True)
                self.logger.info("Cleared clog state for %s on print resume", fps_name)
                # Clear the error LED if we have an OAMS and spool index
                if oams is not None and fps_state.current_spool_idx is not None:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        self.logger.exception("Failed to clear clog LED on %s after resume", fps_name)

            if fps_state.clog_restore_follower:
                self._enable_follower(
                    fps_name,
                    fps_state,
                    oams,
                    fps_state.clog_restore_direction,
                    "print resume",
                )
                if fps_state.following:
                    fps_state.clog_restore_follower = False
                    fps_state.clog_restore_direction = 1
            
            if fps_state.stuck_spool_restore_follower:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "print resume")
            elif (fps_state.current_oams is not None and fps_state.current_spool_idx is not None and not fps_state.following):
                self._ensure_forward_follower(fps_name, fps_state, "print resume")

    def _trigger_stuck_spool_pause(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], message: str) -> None:
        if fps_state.stuck_spool_active:
            return

        spool_idx = fps_state.current_spool_idx
        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)

        if oams is not None and spool_idx is not None:
            try:
                oams.set_led_error(spool_idx, 1)
            except Exception:
                self.logger.exception("Failed to set stuck spool LED on %s spool %s", fps_name, spool_idx)

            direction = fps_state.direction if fps_state.direction in (0, 1) else 1
            fps_state.direction = direction
            fps_state.stuck_spool_restore_follower = True
            fps_state.stuck_spool_restore_direction = direction
            if fps_state.following:
                try:
                    oams.set_oams_follower(0, direction)
                except Exception:
                    self.logger.exception("Failed to stop follower for %s spool %s during stuck spool pause", fps_name, spool_idx)

            fps_state.following = False

        if oams is not None:
            try:
                oams.abort_current_action()
            except Exception:
                self.logger.exception("Failed to abort active action for %s during stuck spool pause", fps_name)

        fps_state.stuck_spool_active = True
        fps_state.stuck_spool_start_time = None
        self._pause_printer_message(message, fps_state.current_oams)

        # Immediately re-enable follower so user can manually fix filament with follower tracking
        self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool pause")

    def _unified_monitor_for_fps(self, fps_name):
        """Consolidated monitor handling all FPS checks in a single timer (OPTIMIZED)."""
        def _unified_monitor(self, eventtime):
            fps_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)

            if fps_state is None or fps is None:
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

            oams = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None

            # OPTIMIZATION: Use cached idle_timeout object
            is_printing = False
            if self._idle_timeout_obj is not None:
                try:
                    is_printing = self._idle_timeout_obj.get_status(eventtime)["state"] == "Printing"
                except Exception:
                    is_printing = False

            # OPTIMIZATION: Skip sensor reads if idle and no state changes
            state = fps_state.state
            if not is_printing and state == FPSLoadState.LOADED:
                fps_state.consecutive_idle_polls += 1
                if fps_state.consecutive_idle_polls > IDLE_POLL_THRESHOLD:
                    # Exponential backoff for idle polling
                    if fps_state.consecutive_idle_polls % 5 == 0:
                        fps_state.idle_backoff_level = min(fps_state.idle_backoff_level + 1, 3)
                    backoff_multiplier = 2 ** fps_state.idle_backoff_level
                    return eventtime + (MONITOR_ENCODER_PERIOD_IDLE * backoff_multiplier)

            # Read sensors
            try:
                if oams:
                    encoder_value = oams.encoder_clicks
                    pressure = float(getattr(fps, "fps_value", 0.0))
                    hes_values = oams.hub_hes_value
                else:
                    return eventtime + MONITOR_ENCODER_PERIOD_IDLE
            except Exception:
                self.logger.exception("Failed to read sensors for %s", fps_name)
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

            now = self.reactor.monotonic()
            state_changed = False

            if state == FPSLoadState.UNLOADING and now - fps_state.since > MONITOR_ENCODER_SPEED_GRACE:
                self._check_unload_speed(fps_name, fps_state, oams, encoder_value, now)
                state_changed = True
            elif state == FPSLoadState.LOADING and now - fps_state.since > MONITOR_ENCODER_SPEED_GRACE:
                self._check_load_speed(fps_name, fps_state, fps, oams, encoder_value, pressure, now)
                state_changed = True
            elif state == FPSLoadState.LOADED:
                if is_printing:
                    self._check_stuck_spool(fps_name, fps_state, fps, oams, pressure, hes_values, now)
                    self._check_clog(fps_name, fps_state, fps, oams, encoder_value, pressure, now)
                    state_changed = True

            # OPTIMIZATION: Adaptive polling interval with exponential backoff
            if state_changed or is_printing:
                fps_state.consecutive_idle_polls = 0
                fps_state.idle_backoff_level = 0
                fps_state.last_state_change = now
                return eventtime + MONITOR_ENCODER_PERIOD

            fps_state.consecutive_idle_polls += 1
            if fps_state.consecutive_idle_polls > IDLE_POLL_THRESHOLD:
                # Exponential backoff: increase backoff level every 5 idle polls
                if fps_state.consecutive_idle_polls % 5 == 0:
                    fps_state.idle_backoff_level = min(fps_state.idle_backoff_level + 1, 3)

                # Calculate backoff multiplier (1x, 2x, 4x, 8x)
                backoff_multiplier = 2 ** fps_state.idle_backoff_level
                interval = MONITOR_ENCODER_PERIOD_IDLE * backoff_multiplier
                return eventtime + interval

            return eventtime + MONITOR_ENCODER_PERIOD

        return partial(_unified_monitor, self)

    def _check_unload_speed(self, fps_name, fps_state, oams, encoder_value, now):
        """Check unload speed using optimized encoder tracking."""
        # Skip check if already handling a stuck spool
        if fps_state.stuck_spool_active:
            return
            
        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        if encoder_diff is None:
            return
        
        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug("OAMS[%d] Unload Monitor: Encoder diff %d", getattr(oams, "oams_idx", -1), encoder_diff)
        
        if encoder_diff < MIN_ENCODER_DIFF:
            lane_label = fps_state.current_lane or fps_name
            spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"
            
            # Abort the current unload operation cleanly
            try:
                oams.abort_current_action()
                self.logger.info("Aborted stuck spool unload operation on %s", fps_name)
            except Exception:
                self.logger.exception("Failed to abort unload operation on %s", fps_name)
            
            # Set LED error
            try:
                oams.set_led_error(fps_state.current_spool_idx, 1)
            except Exception:
                self.logger.exception("Failed to set LED during unload stuck detection on %s", fps_name)
            
            # Transition to LOADED state cleanly (unload failed, so still loaded)
            fps_state.state = FPSLoadState.LOADED
            fps_state.clear_encoder_samples()
            
            # Set the stuck flag but DON'T pause - let the OAMS retry logic handle it
            # The retry logic will clear this flag if the retry succeeds
            fps_state.stuck_spool_active = True
            fps_state.stuck_spool_start_time = None
            
            self.logger.info("Spool appears stuck while unloading %s spool %s - letting retry logic handle it", lane_label, spool_label)

    def _check_load_speed(self, fps_name, fps_state, fps, oams, encoder_value, pressure, now):
        """Check load speed using optimized encoder tracking and FPS pressure monitoring."""
        if fps_state.stuck_spool_active:
            return

        # Skip check if we don't have a valid since timestamp
        if fps_state.since is None:
            return

        # Skip check if we're still in grace period
        if now - fps_state.since <= MONITOR_ENCODER_SPEED_GRACE:
            return

        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        if encoder_diff is None:
            return

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug("OAMS[%d] Load Monitor: Encoder diff %d, FPS pressure %.2f",
                            getattr(oams, "oams_idx", -1), encoder_diff, pressure)

        # Check for stuck spool conditions:
        # 1. Encoder not moving (original check)
        # 2. FPS pressure staying high (filament not engaging) - NEW CHECK
        stuck_detected = False
        stuck_reason = ""

        if encoder_diff < MIN_ENCODER_DIFF:
            stuck_detected = True
            stuck_reason = "encoder not moving"
        elif pressure >= self.load_fps_stuck_threshold:
            # FPS pressure is high while loading - filament isn't being pulled in
            # This catches cases where extruder is turning but filament missed the drive gear
            stuck_detected = True
            stuck_reason = f"FPS pressure {pressure:.2f} >= {self.load_fps_stuck_threshold:.2f} (filament not engaging)"

        if stuck_detected:
            lane_label = fps_state.current_lane or fps_name
            spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"

            # Abort the current load operation cleanly
            try:
                oams.abort_current_action()
                self.logger.info("Aborted stuck spool load operation on %s: %s", fps_name, stuck_reason)
            except Exception:
                self.logger.exception("Failed to abort load operation on %s", fps_name)

            # Set LED error
            try:
                oams.set_led_error(fps_state.current_spool_idx, 1)
            except Exception:
                self.logger.exception("Failed to set LED during load stuck detection on %s", fps_name)

            # Transition to UNLOADED state cleanly
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.clear_encoder_samples()

            # Set the stuck flag but DON'T pause - let the OAMS retry logic handle it
            # The retry logic will clear this flag if the retry succeeds
            fps_state.stuck_spool_active = True
            fps_state.stuck_spool_start_time = None

            self.logger.info("Spool appears stuck while loading %s spool %s (%s) - letting retry logic handle it",
                           lane_label, spool_label, stuck_reason)

    def _check_stuck_spool(self, fps_name, fps_state, fps, oams, pressure, hes_values, now):
        """Check for stuck spool conditions (OPTIMIZED)."""
        # OPTIMIZATION: Use cached idle_timeout object
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        monitor = self.runout_monitors.get(fps_name)
        if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear stuck spool LED while runout monitor inactive on %s", fps_name)
            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool_restore_follower)
            return

        if not is_printing:
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear stuck spool LED while idle on %s", fps_name)
            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool_restore_follower)
            return

        if fps_state.since is not None and now - fps_state.since < self.stuck_spool_load_grace:
            fps_state.stuck_spool_start_time = None
            # Clear stuck spool flag during grace period after successful load
            if fps_state.stuck_spool_active:
                if oams is not None and fps_state.current_spool_idx is not None:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        self.logger.exception("Failed to clear stuck spool LED during grace period on %s", fps_name)
                fps_state.reset_stuck_spool_state(preserve_restore=True)
            return

        if not fps_state.following or fps_state.direction != 1:
            fps_state.stuck_spool_start_time = None
            # Auto-enable follower if we have a spool loaded but follower is disabled
            if is_printing and oams is not None and not fps_state.following:
                self._ensure_forward_follower(fps_name, fps_state, "auto-enable after manual load")
            elif fps_state.stuck_spool_restore_follower and is_printing and oams is not None:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")
            return

        # Hysteresis logic: Use lower threshold to start timer, upper threshold to clear
        if pressure <= self.stuck_spool_pressure_threshold:
            # Pressure is low - start or continue stuck spool timer
            if fps_state.stuck_spool_start_time is None:
                fps_state.stuck_spool_start_time = now
            elif (not fps_state.stuck_spool_active and now - fps_state.stuck_spool_start_time >= STUCK_SPOOL_DWELL):
                message = "Spool appears stuck"
                if fps_state.current_lane is not None:
                    message = f"Spool appears stuck on {fps_state.current_lane} spool {fps_state.current_spool_idx}"
                self._trigger_stuck_spool_pause(fps_name, fps_state, oams, message)
        elif pressure >= self.stuck_spool_pressure_clear_threshold:
            # Pressure is definitively high - clear stuck spool state
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear stuck spool LED on %s spool %d", fps_name, fps_state.current_spool_idx)

                # Clear the stuck_spool_active flag BEFORE trying to restore follower
                fps_state.reset_stuck_spool_state(preserve_restore=True)
                self.logger.info("Cleared stuck spool state for %s, pressure restored to %.2f", fps_name, pressure)

            # Also clear timer if it was running but not yet triggered
            if fps_state.stuck_spool_start_time is not None and not fps_state.stuck_spool_active:
                fps_state.stuck_spool_start_time = None

            # Now restore/enable follower
            if fps_state.stuck_spool_restore_follower and is_printing:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")
            elif is_printing and not fps_state.following:
                self._ensure_forward_follower(fps_name, fps_state, "stuck spool recovery")
        # else: Pressure is in hysteresis band (between thresholds) - maintain current state

    def _check_clog(self, fps_name, fps_state, fps, oams, encoder_value, pressure, now):
        """Check for clog conditions (OPTIMIZED)."""
        # OPTIMIZATION: Use cached idle_timeout object
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        monitor = self.runout_monitors.get(fps_name)
        if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
            if fps_state.clog_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear clog LED on %s while runout monitor inactive", fps_name)
            fps_state.reset_clog_tracker()
            return

        if not is_printing:
            if fps_state.clog_active and oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 0)
                except Exception:
                    self.logger.exception("Failed to clear clog LED on %s while printer idle", fps_name)
            fps_state.reset_clog_tracker()
            return

        try:
            extruder_pos = float(getattr(fps.extruder, "last_position", 0.0))
        except Exception:
            self.logger.exception("Failed to read extruder position while monitoring clogs on %s", fps_name)
            return

        if fps_state.clog_start_extruder is None:
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        if extruder_pos < (fps_state.clog_last_extruder or extruder_pos):
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        fps_state.clog_last_extruder = extruder_pos
        if fps_state.clog_min_pressure is None or pressure < fps_state.clog_min_pressure:
            fps_state.clog_min_pressure = pressure
        if fps_state.clog_max_pressure is None or pressure > fps_state.clog_max_pressure:
            fps_state.clog_max_pressure = pressure

        extrusion_delta = extruder_pos - (fps_state.clog_start_extruder or extruder_pos)
        encoder_delta = abs(encoder_value - (fps_state.clog_start_encoder or encoder_value))
        pressure_span = (fps_state.clog_max_pressure or pressure) - (fps_state.clog_min_pressure or pressure)

        settings = self.clog_settings
        if extrusion_delta < settings["extrusion_window"]:
            return

        if (encoder_delta > settings["encoder_slack"] or pressure_span > settings["pressure_band"]):
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        if now - (fps_state.clog_start_time or now) < settings["dwell"]:
            return

        if not fps_state.clog_active:
            if oams is not None and fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                except Exception:
                    self.logger.exception("Failed to set clog LED on %s spool %s", fps_name, fps_state.current_spool_idx)
            direction = fps_state.direction if fps_state.direction in (0, 1) else 1
            fps_state.clog_restore_follower = True
            fps_state.clog_restore_direction = direction
            if oams is not None and fps_state.following:
                try:
                    oams.set_oams_follower(0, direction)
                except Exception:
                    self.logger.exception("Failed to stop follower on %s during clog pause", fps_name)
            fps_state.following = False
            pressure_mid = (fps_state.clog_min_pressure + fps_state.clog_max_pressure) / 2.0
            message = (f"Clog suspected on {fps_state.current_lane or fps_name}: "
                      f"extruder advanced {extrusion_delta:.1f}mm while encoder moved {encoder_delta} counts "
                      f"with FPS {pressure_mid:.2f} near {self.clog_pressure_target:.2f}")
            fps_state.clog_active = True
            self._pause_printer_message(message, fps_state.current_oams)
            self._reactivate_clog_follower(fps_name, fps_state, oams, "clog pause")

    def start_monitors(self):
        """Start all monitoring timers"""
        # Stop existing monitors first to prevent timer leaks
        if self.monitor_timers:
            self.stop_monitors()

        self.monitor_timers = []
        self.runout_monitors = {}
        reactor = self.printer.get_reactor()
        
        for fps_name in self.current_state.fps_state.keys():
            self.monitor_timers.append(
                reactor.register_timer(
                    self._unified_monitor_for_fps(fps_name), 
                    reactor.NOW
                )
            )

            def _reload_callback(fps_name=fps_name, fps_state=self.current_state.fps_state[fps_name]):
                monitor = self.runout_monitors.get(fps_name)
                source_lane_name = fps_state.current_lane
                active_oams = fps_state.current_oams
                target_lane_map, target_lane, delegate_to_afc, source_lane = self._get_infinite_runout_target_lane(fps_name, fps_state)
                source_lane_name = fps_state.current_lane

                if delegate_to_afc:
                    delegated = self._delegate_runout_to_afc(fps_name, fps_state, source_lane, target_lane)
                    if delegated:
                        fps_state.reset_runout_positions()
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        return

                    self.logger.error("Failed to delegate infinite runout for %s on %s via AFC", fps_name, source_lane_name or "<unknown>")
                    fps_state.reset_runout_positions()
                    self._pause_printer_message(f"Unable to delegate infinite runout for {source_lane_name or fps_name}", fps_state.current_oams or active_oams)
                    if monitor:
                        monitor.paused()
                    return

                # Load the target lane directly
                if target_lane is None:
                    self.logger.error("No lane available to reload on %s", fps_name)
                    self._pause_printer_message(f"No lane available to reload on {fps_name}", fps_state.current_oams or active_oams)
                    if monitor:
                        monitor.paused()
                    return

                if target_lane_map:
                    self.logger.info("Infinite runout triggered for %s on %s -> %s", fps_name, source_lane_name, target_lane)
                    unload_success, unload_message = self._unload_filament_for_fps(fps_name)
                    if not unload_success:
                        self.logger.error("Failed to unload filament during infinite runout on %s: %s", fps_name, unload_message)
                        failure_message = unload_message or f"Failed to unload current spool on {fps_name}"
                        self._pause_printer_message(failure_message, fps_state.current_oams or active_oams)
                        if monitor:
                            monitor.paused()
                        return

                load_success, load_message = self._load_filament_for_lane(target_lane)
                if load_success:
                    self.logger.info("Successfully loaded lane %s on %s%s", target_lane, fps_name, " after infinite runout" if target_lane_map else "")
                    if target_lane_map and target_lane:
                        handled = False
                        if AMSRunoutCoordinator is not None:
                            try:
                                handled = AMSRunoutCoordinator.notify_lane_tool_state(self.printer, fps_state.current_oams or active_oams, target_lane, loaded=True, spool_index=fps_state.current_spool_idx, eventtime=fps_state.since)
                            except Exception:
                                self.logger.exception("Failed to notify AFC lane %s after infinite runout on %s", target_lane, fps_name)
                                handled = False
                        if not handled:
                            try:
                                gcode = self.printer.lookup_object("gcode")
                                gcode.run_script(f"SET_LANE_LOADED LANE={target_lane}")
                                self.logger.debug("Marked lane %s as loaded after infinite runout on %s", target_lane, fps_name)
                            except Exception:
                                self.logger.exception("Failed to mark lane %s as loaded after infinite runout on %s", target_lane, fps_name)
                    fps_state.reset_runout_positions()
                    if monitor:
                        monitor.reset()
                        monitor.start()
                    return

                self.logger.error("Failed to load lane %s on %s: %s", target_lane, fps_name, load_message)
                failure_message = load_message or f"No spool available for lane {target_lane}"
                self._pause_printer_message(failure_message, fps_state.current_oams or active_oams)
                if monitor:
                    monitor.paused()

            fps_reload_margin = getattr(self.fpss[fps_name], "reload_before_toolhead_distance", None)
            if fps_reload_margin is None:
                fps_reload_margin = self.reload_before_toolhead_distance

            monitor = OAMSRunoutMonitor(self.printer, fps_name, self.fpss[fps_name], self.current_state.fps_state[fps_name], self.oams, _reload_callback, reload_before_toolhead_distance=fps_reload_margin)
            self.runout_monitors[fps_name] = monitor
            monitor.start()

        self.logger.info("All monitors started (optimized)")

    # ============================================================================
    # AFC State Change Notifications (Optional Callbacks)
    # ============================================================================
    # These methods can be called by AFC when lane state changes to keep OAMS
    # manager in sync immediately without waiting for state detection polling.
    # AFC integration is optional - these are no-ops if called when AFC is not configured.

    def on_afc_lane_loaded(self, lane_name: str, extruder_name: Optional[str] = None) -> None:
        """Callback for AFC to notify OAMS when a lane is loaded.

        Args:
            lane_name: Name of the lane that was loaded (e.g., "lane4")
            extruder_name: Optional name of the extruder/tool it was loaded to

        This allows OAMS to:
        - Update FPS state immediately (no polling lag)
        - Enable follower motor instantly
        - Sync state for accurate monitoring
        """
        try:
            # Get FPS for this lane (uses cache for speed)
            fps_name = self.get_fps_for_afc_lane(lane_name)
            if not fps_name:
                return  # Lane not on any FPS we manage

            fps_state = self.current_state.fps_state.get(fps_name)
            if fps_state is None:
                return

            # Let state detection handle the full sync
            # We just trigger it to run immediately instead of waiting for next poll
            lane_map_name, oam, bay_index = self._determine_loaded_lane_for_fps(fps_name, self.fpss[fps_name])

            if lane_map_name and oam and bay_index is not None:
                # Update FPS state
                fps_state.current_lane = lane_map_name
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.direction = 1

                # Enable follower immediately
                self._ensure_forward_follower(fps_name, fps_state, "AFC lane loaded notification")

                self.logger.info("Synced OAMS state from AFC: %s loaded to %s (bay %d on %s)",
                               lane_name, fps_name, bay_index, oam.name)
        except Exception:
            self.logger.exception("Error processing AFC lane loaded notification for %s", lane_name)

    def on_afc_lane_unloaded(self, lane_name: str, extruder_name: Optional[str] = None) -> None:
        """Callback for AFC to notify OAMS when a lane is unloaded.

        Args:
            lane_name: Name of the lane that was unloaded (e.g., "lane4")
            extruder_name: Optional name of the extruder/tool it was unloaded from

        This allows OAMS to:
        - Update FPS state immediately
        - Disable follower motor
        - Clear monitoring state
        """
        try:
            # Get FPS for this lane (uses cache for speed)
            fps_name = self.get_fps_for_afc_lane(lane_name)
            if not fps_name:
                return  # Lane not on any FPS we manage

            fps_state = self.current_state.fps_state.get(fps_name)
            if fps_state is None:
                return

            # Only update if this lane was actually loaded on this FPS
            if fps_state.state == FPSLoadState.LOADED:
                # Disable follower
                if fps_state.current_oams and fps_state.following:
                    oam = self.oams.get(fps_state.current_oams)
                    if oam:
                        try:
                            oam.set_oams_follower(0, fps_state.direction)
                            fps_state.following = False
                        except Exception:
                            self.logger.exception("Failed to disable follower during AFC unload notification")

                # Update state
                fps_state.state = FPSLoadState.UNLOADED
                fps_state.current_lane = None
                fps_state.current_oams = None
                fps_state.current_spool_idx = None
                fps_state.since = self.reactor.monotonic()

                self.logger.info("Synced OAMS state from AFC: %s unloaded from %s", lane_name, fps_name)
        except Exception:
            self.logger.exception("Error processing AFC lane unloaded notification for %s", lane_name)

    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []
        for monitor in self.runout_monitors.values():
            monitor.reset()
        self.runout_monitors = {}


def load_config(config):
    return OAMSManager(config)
