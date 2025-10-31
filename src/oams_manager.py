# OpenAMS Manager
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import time
from functools import partial
from collections import deque
from typing import Optional, Tuple, Dict, List, Any, Callable

try:  # pragma: no cover - integration optional during testing
    from extras.ams_integration import AMSRunoutCoordinator
except Exception:  # pragma: no cover - AFC not present
    AMSRunoutCoordinator = None

# Configuration constants
PAUSE_DISTANCE = 60  # mm to pause before coasting follower
ENCODER_SAMPLES = 2  # Number of encoder samples to collect
MIN_ENCODER_DIFF = 1  # Minimum encoder difference to consider movement
FILAMENT_PATH_LENGTH_FACTOR = 1.14  # Factor for calculating filament path traversal
MONITOR_ENCODER_LOADING_SPEED_AFTER = 2.0  # seconds
MONITOR_ENCODER_PERIOD = 2.0  # seconds
MONITOR_ENCODER_UNLOADING_SPEED_AFTER = 2.0  # seconds
AFC_DELEGATION_TIMEOUT = 30.0  # seconds to suppress duplicate AFC runout triggers

STUCK_SPOOL_PRESSURE_THRESHOLD = 0.08  # Pressure indicating the spool is no longer feeding
STUCK_SPOOL_DWELL = 3.5  # Seconds the pressure must remain below the threshold before pausing
STUCK_SPOOL_LOAD_GRACE = 8.0  # Grace period after a swap/load before stuck detection arms


CLOG_PRESSURE_TARGET = 0.50
CLOG_SENSITIVITY_LEVELS = {
    "low": {
        "extrusion_window": 48.0,
        "encoder_slack": 15,
        "pressure_band": 0.08,
        "dwell": 12.0,
    },
    "medium": {
        "extrusion_window": 24.0,
        "encoder_slack": 8,
        "pressure_band": 0.06,
        "dwell": 8.0,
    },
    "high": {
        "extrusion_window": 12.0,
        "encoder_slack": 4,
        "pressure_band": 0.04,
        "dwell": 6.0,
    },
}
CLOG_SENSITIVITY_DEFAULT = "medium"


POST_LOAD_PRESSURE_THRESHOLD = 0.56  # FPS value indicating a possible clog after loading
POST_LOAD_PRESSURE_DWELL = 15.0  # Seconds pressure must remain above the threshold
POST_LOAD_PRESSURE_CHECK_PERIOD = 0.5  # Interval between post-load pressure checks


class OAMSRunoutState:
    """Enum for runout monitor states."""
    STOPPED = "STOPPED"          # Monitor is disabled
    MONITORING = "MONITORING"    # Actively watching for runout
    DETECTED = "DETECTED"        # Runout detected, pausing before coast
    COASTING = "COASTING"        # Follower coasting, preparing next spool
    RELOADING = "RELOADING"      # Loading next spool in sequence
    PAUSED = "PAUSED"           # Monitor paused due to error/manual intervention


class FPSLoadState:
    """Enum for FPS loading states."""
    UNLOADED = "UNLOADED"    # No filament loaded
    LOADED = "LOADED"        # Filament loaded and ready
    LOADING = "LOADING"      # Currently loading filament
    UNLOADING = "UNLOADING"  # Currently unloading filament
    
class OAMSRunoutMonitor:
    """
    Monitors filament runout for a specific FPS and handles automatic reload.
    
    State Management:
    - Tracks runout detection and follower coasting
    - Manages automatic spool switching within filament groups
    - Coordinates with OAMS hardware for filament loading
    """
    
    def __init__(self,
                 printer,
                 fps_name: str,
                 fps,
                 fps_state,
                 oams: Dict[str, Any],
                 reload_callback: Callable,
                 reload_before_toolhead_distance: float = 0.0):
        # Core references
        self.oams = oams
        self.printer = printer
        self.fps_name = fps_name
        self.fps_state = fps_state
        self.fps = fps
        
        # State tracking
        self.state = OAMSRunoutState.STOPPED
        self.runout_position: Optional[float] = None
        self.bldc_clear_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        
        # Configuration
        self.reload_before_toolhead_distance = reload_before_toolhead_distance
        self.reload_callback = reload_callback

        self.reactor = self.printer.get_reactor()

        self.hardware_service = None
        self.latest_lane_name: Optional[str] = None
        if AMSRunoutCoordinator is not None:
            try:
                self.hardware_service = AMSRunoutCoordinator.register_runout_monitor(self)
            except Exception:
                logging.getLogger(__name__).exception(
                    "Failed to register OpenAMS monitor with AMSRunoutCoordinator"
                )
        
        def _monitor_runout(eventtime):
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            
            if self.state == OAMSRunoutState.STOPPED or self.state == OAMSRunoutState.PAUSED or self.state == OAMSRunoutState.RELOADING:
                pass
            elif self.state == OAMSRunoutState.MONITORING:
                #logging.info("OAMS: Monitoring runout, is_printing: %s, fps_state: %s, fps_state.current_group: %s, fps_state.current_spool_idx: %s, oams: %s" % (is_printing, fps_state.state_name, fps_state.current_group, fps_state.current_spool_idx, fps_state.current_oams))
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
                    logging.debug(
                        "OAMS: Skipping runout monitor for %s - no active spool index",
                        self.fps_name,
                    )
                    return eventtime + MONITOR_ENCODER_PERIOD

                lane_name = None
                spool_empty = None
                unit_name = getattr(fps_state, "current_oams", None) or self.fps_name

                if self.hardware_service is not None:
                    try:
                        lane_name = self.hardware_service.resolve_lane_for_spool(
                            unit_name, spool_idx
                        )
                        snapshot = self.hardware_service.latest_lane_snapshot_for_spool(
                            unit_name, spool_idx
                        )
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
                            logging.debug(
                                "OAMS: Skipping runout monitor for %s - spool index %s out of range",
                                self.fps_name,
                                spool_idx,
                            )
                            self.latest_lane_name = lane_name
                            return eventtime + MONITOR_ENCODER_PERIOD
                        spool_empty = not bool(hes_values[spool_idx])
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to read HES values for runout detection on %s",
                            self.fps_name,
                        )
                        self.latest_lane_name = lane_name
                        return eventtime + MONITOR_ENCODER_PERIOD

                self.latest_lane_name = lane_name

                if (
                    is_printing
                    and fps_state.state_name == "LOADED"
                    and fps_state.current_group is not None
                    and fps_state.current_spool_idx is not None
                    and spool_empty
                ):
                    self.state = OAMSRunoutState.DETECTED
                    logging.info(
                        "OAMS: Runout detected on FPS %s, pausing for %d mm before coasting the follower.",
                        self.fps_name,
                        PAUSE_DISTANCE,
                    )
                    self.runout_position = fps.extruder.last_position
                    if AMSRunoutCoordinator is not None:
                        try:
                            AMSRunoutCoordinator.notify_runout_detected(
                                self, spool_idx, lane_name=lane_name
                            )
                        except Exception:
                            logging.getLogger(__name__).exception(
                                "Failed to notify AFC about OpenAMS runout"
                            )

            elif self.state == OAMSRunoutState.DETECTED:
                traveled_distance = fps.extruder.last_position - self.runout_position
                if traveled_distance >= PAUSE_DISTANCE:
                    logging.info("OAMS: Pause complete, coasting the follower.")
                    try:
                        self.oams[fps_state.current_oams].set_oams_follower(0, 1)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to stop follower while coasting on %s",
                            self.fps_name,
                        )
                    finally:
                        # Ensure internal follower state matches the hardware when we
                        # intentionally stop it for coasting. Otherwise subsequent
                        # calls to _ensure_forward_follower() think the follower is
                        # still running and will not re-enable it.
                        fps_state.following = False
                    self.bldc_clear_position = fps.extruder.last_position
                    self.runout_after_position = 0.0
                    self.state = OAMSRunoutState.COASTING

            elif self.state == OAMSRunoutState.COASTING:
                traveled_distance_after_bldc_clear = max(
                    fps.extruder.last_position - self.bldc_clear_position, 0.0
                )
                self.runout_after_position = traveled_distance_after_bldc_clear
                try:
                    path_length = getattr(
                        self.oams[fps_state.current_oams], "filament_path_length", 0.0
                    )
                except Exception:
                    logging.exception(
                        "OAMS: Failed to read filament path length while coasting on %s",
                        self.fps_name,
                    )
                    return eventtime + MONITOR_ENCODER_PERIOD
                effective_path_length = (
                    path_length / FILAMENT_PATH_LENGTH_FACTOR if path_length else 0.0
                )
                consumed_with_margin = (
                    self.runout_after_position
                    + PAUSE_DISTANCE
                    + self.reload_before_toolhead_distance
                )

                if consumed_with_margin >= effective_path_length:
                    logging.info(
                        "OAMS: Loading next spool (%.2f mm consumed + margin %.2f mm >= effective path %.2f mm).",
                        self.runout_after_position + PAUSE_DISTANCE,
                        self.reload_before_toolhead_distance,
                        effective_path_length,
                    )
                    self.state = OAMSRunoutState.RELOADING
                    self.reload_callback()
            else:
                raise ValueError(f"Invalid state: {self.state}")
            return eventtime + MONITOR_ENCODER_PERIOD
        self._timer_callback = _monitor_runout
        self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)

    def start(self) -> None:
        """Start monitoring for filament runout."""
        if self.timer is None:
            self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)
        self.state = OAMSRunoutState.MONITORING
    
    def stop(self) -> None:
        """Stop monitoring for filament runout."""
        self.state = OAMSRunoutState.STOPPED
        
    def reloading(self) -> None:
        """Set state to reloading and reset positions."""
        self.state = OAMSRunoutState.RELOADING
        self.runout_position = None
        self.runout_after_position = None
        
    def paused(self) -> None:
        """Pause the monitor due to error or manual intervention."""
        self.state = OAMSRunoutState.PAUSED
        
    def reset(self) -> None:
        """Reset monitor to stopped state and clean up."""
        self.state = OAMSRunoutState.STOPPED
        self.runout_position = None
        self.runout_after_position = None
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None

class OAMSState:
    """
    Global state container for all FPS units in the system.
    
    Attributes:
    - fps_state: Dictionary mapping FPS names to their FPSState objects
    """
    
    def __init__(self):
        self.fps_state: Dict[str, 'FPSState'] = {}
        
    def add_fps_state(self, fps_name: str) -> None:
        """Add a new FPS state tracker."""
        self.fps_state[fps_name] = FPSState()
        

class FPSState:
    """
    Tracks the state of a single FPS (Filament Pressure Sensor).
    
    Key State Variables:
    - state_name: Current loading state (LOADED, UNLOADED, LOADING, UNLOADING)
    - current_group: Filament group name (e.g., "T0", "T1") if loaded
    - current_oams: Name of the OAMS unit currently loaded
    - current_spool_idx: Index (0-3) of the spool bay currently loaded
    
    Monitoring State:
    - encoder_samples: Recent encoder readings for motion detection
    - following: Whether follower mode is active
    - direction: Follower direction (0=forward, 1=reverse)
    - since: Timestamp when current state began
    """
    
    def __init__(self, 
                 state_name: str = FPSLoadState.UNLOADED, 
                 current_group: Optional[str] = None, 
                 current_oams: Optional[str] = None, 
                 current_spool_idx: Optional[int] = None):
        
        # Primary state tracking
        self.state_name = state_name  # FPSLoadState: LOADED, UNLOADED, LOADING, UNLOADING
        self.current_group = current_group  # Filament group name (T0, T1, etc.)
        self.current_oams = current_oams  # OAMS unit name currently loaded
        self.current_spool_idx = current_spool_idx  # Spool bay index (0-3)
        
        # Runout tracking
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        
        # Timer references (for cleanup)
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None
        
        # Motion monitoring
        self.encoder_samples = deque(maxlen=ENCODER_SAMPLES)  # Recent encoder readings

        # Follower state
        self.following: bool = False  # Whether follower mode is active
        self.direction: int = 0  # Follower direction (0=forward, 1=reverse)
        self.since: Optional[float] = None  # Timestamp when current state began

        # AFC delegation state
        self.afc_delegation_active: bool = False
        self.afc_delegation_until: float = 0.0

        # Stuck spool detection
        self.stuck_spool_start_time: Optional[float] = None
        self.stuck_spool_active: bool = False
        self.stuck_spool_restore_follower: bool = False
        self.stuck_spool_restore_direction: int = 1

        # Clog detection
        self.clog_active: bool = False
        self.clog_start_extruder: Optional[float] = None
        self.clog_start_encoder: Optional[int] = None
        self.clog_start_time: Optional[float] = None
        self.clog_min_pressure: Optional[float] = None
        self.clog_max_pressure: Optional[float] = None
        self.clog_last_extruder: Optional[float] = None

        # Post-load pressure validation
        self.post_load_pressure_timer = None
        self.post_load_pressure_start: Optional[float] = None


    def reset_runout_positions(self) -> None:
        """Clear runout position tracking."""
        self.runout_position = None
        self.runout_after_position = None

    def reset_stuck_spool_state(self, preserve_restore: bool = False) -> None:
        """Clear any latched stuck spool indicators."""
        self.stuck_spool_start_time = None
        self.stuck_spool_active = False
        if not preserve_restore:
            self.stuck_spool_restore_follower = False
            self.stuck_spool_restore_direction = 1

    def reset_clog_tracker(self) -> None:
        """Reset clog detection telemetry so monitoring restarts fresh."""
        self.clog_active = False
        self.clog_start_extruder = None
        self.clog_start_encoder = None
        self.clog_start_time = None
        self.clog_min_pressure = None
        self.clog_max_pressure = None
        self.clog_last_extruder = None

    def prime_clog_tracker(
        self,
        extruder_pos: float,
        encoder_clicks: int,
        pressure: float,
        timestamp: float,
    ) -> None:
        """Prime clog detection with the most recent telemetry sample."""
        self.clog_start_extruder = extruder_pos
        self.clog_last_extruder = extruder_pos
        self.clog_start_encoder = encoder_clicks
        self.clog_start_time = timestamp
        self.clog_min_pressure = pressure
        self.clog_max_pressure = pressure
        
    def __repr__(self) -> str:
        return f"FPSState(state_name={self.state_name}, current_group={self.current_group}, current_oams={self.current_oams}, current_spool_idx={self.current_spool_idx})"

    def __str__(self) -> str:
        return f"State: {self.state_name}, Group: {self.current_group}, OAMS: {self.current_oams}, Spool Index: {self.current_spool_idx}"

class OAMSManager:
    """
    Main coordinator for OpenAMS system with multiple FPS units.
    
    Manages:
    - Multiple FPS (Filament Pressure Sensor) units
    - OAMS (OpenAMS) hardware units  
    - Filament groups (T0, T1, etc.) mapping to FPS units
    - Automatic filament runout detection and switching
    
    Key Attributes:
    - fpss: Dictionary of FPS objects {fps_name: fps_object}
    - oams: Dictionary of OAMS objects {oams_name: oams_object}  
    - filament_groups: Dictionary of filament groups {group_name: group_object}
    - current_state: OAMSState tracking all FPS states
    """
    
    def __init__(self, config):
        # Core configuration and printer interface
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        
        # Hardware object collections
        self.filament_groups: Dict[str, Any] = {}
        self.oams: Dict[str, Any] = {}
        self.fpss: Dict[str, Any] = {}

        # State management
        self.current_state = OAMSState()
        self.current_group: Optional[str] = None
        self.afc = None
        self._afc_logged = False

        # Monitoring and control
        self.monitor_timers: List[Any] = []
        self.runout_monitors: Dict[str, OAMSRunoutMonitor] = {}
        self.ready: bool = False

        # Configuration parameters
        self.reload_before_toolhead_distance: float = config.getfloat(
            "reload_before_toolhead_distance",
            0.0,
        )

        sensitivity = config.get("clog_sensitivity", CLOG_SENSITIVITY_DEFAULT).lower()
        if sensitivity not in CLOG_SENSITIVITY_LEVELS:
            logging.warning(
                "OAMS: Unknown clog_sensitivity '%s', falling back to %s",
                sensitivity,
                CLOG_SENSITIVITY_DEFAULT,
            )
            sensitivity = CLOG_SENSITIVITY_DEFAULT
        self.clog_sensitivity = sensitivity
        self.clog_settings = CLOG_SENSITIVITY_LEVELS[self.clog_sensitivity]

        # Cached mappings
        self.group_to_fps: Dict[str, str] = {}
        self._canonical_lane_by_group: Dict[str, str] = {}
        self._canonical_group_by_lane: Dict[str, str] = {}
        self._lane_unit_map: Dict[str, str] = {}
        self._lane_by_location: Dict[Tuple[str, int], str] = {}

        # Initialize hardware collections
        self._initialize_oams()
        self._initialize_filament_groups()

        # Register with printer and setup event handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler(
            "idle_timeout:printing",
            self._handle_printing_resumed,
        )
        self.printer.register_event_handler(
            "pause:resume",
            self._handle_printing_resumed,
        )

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
                logging.exception(
                    "OAMS: Failed to fetch status from %s", name
                )
                oam_status = {
                    "action_status": "error",
                    "action_status_code": None,
                    "action_status_value": None,
                }
            attributes["oams"][status_name] = oam_status
            if status_name != name:
                attributes["oams"][name] = oam_status

        for fps_name, fps_state in self.current_state.fps_state.items():
            attributes[fps_name] = {
                "current_group": fps_state.current_group,
                "current_oams": fps_state.current_oams,
                "current_spool_idx": fps_state.current_spool_idx,
                "state_name": fps_state.state_name,
                "since": fps_state.since,
            }

        return attributes
    
    def determine_state(self) -> None:
        """
        Analyze hardware state and update FPS state tracking.
        
        For each FPS:
        1. Check which filament group is currently loaded
        2. Identify the active OAMS unit and spool bay
        3. Update state to LOADED if filament is present
        """
        for fps_name, fps_state in self.current_state.fps_state.items():
            (
                fps_state.current_group,
                current_oams,
                fps_state.current_spool_idx,
            ) = self.determine_current_loaded_group(fps_name)

            if current_oams is not None:
                fps_state.current_oams = current_oams.name
            else:
                fps_state.current_oams = None

            if (
                fps_state.current_oams is not None
                and fps_state.current_spool_idx is not None
            ):
                fps_state.state_name = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._ensure_forward_follower(
                    fps_name,
                    fps_state,
                    "state detection",
                )
            else:
                fps_state.state_name = FPSLoadState.UNLOADED
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._cancel_post_load_pressure_check(fps_state)
        
    def handle_ready(self) -> None:
        """
        Initialize system when printer is ready.
        
        1. Discover and register all FPS units
        2. Create state tracking for each FPS
        3. Determine current hardware state
        4. Start monitoring timers
        """
        # Discover all FPS units in the system
        for fps_name, fps in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)
            
        if not self.fpss:
            raise ValueError("No FPS found in system, this is required for OAMS to work")

        self._rebuild_group_fps_index()

        # Initialize system state and start monitoring
        self.determine_state()
        self.start_monitors()
        self.ready = True

    def _initialize_oams(self) -> None:
        """Discover and register all OAMS hardware units."""
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def _initialize_filament_groups(self) -> None:
        """Discover and register all filament group configurations."""
        for name, group in self.printer.lookup_objects(module="filament_group"):
            name = name.split()[-1]  # Extract group name from full object name
            logging.info(f"OAMS: Adding group {name}")
            self.filament_groups[name] = group
    
    def determine_current_loaded_group(self, fps_name: str) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """
        Determine which filament group is currently loaded in the specified FPS.
        
        Args:
            fps_name: Name of the FPS to check
            
        Returns:
            Tuple of (group_name, oams_object, bay_index) or (None, None, None) if unloaded
            
        Process:
        1. Get the FPS object
        2. Check each filament group for loaded bays
        3. Verify the OAMS is connected to this FPS
        4. Return the first match found
        """
        fps = self.fpss.get(fps_name)
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")
            
        # Check each filament group for loaded spools
        for group_name, group in self.filament_groups.items():
            for oam, bay_index in group.bays:
                try:
                    is_loaded = oam.is_bay_loaded(bay_index)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to query bay %s on %s while determining loaded group",
                        bay_index,
                        getattr(oam, "name", "<unknown>"),
                    )
                    continue

                # Check if this bay has filament loaded and the OAMS is connected to this FPS
                if is_loaded and oam in fps.oams:
                    return group_name, oam, bay_index
                    
        return None, None, None
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "OAMSM_UNLOAD_FILAMENT",
            self.cmd_UNLOAD_FILAMENT,
            desc=self.cmd_UNLOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "OAMSM_LOAD_FILAMENT",
            self.cmd_LOAD_FILAMENT,
            desc=self.cmd_LOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "OAMSM_FOLLOWER",
            self.cmd_FOLLOWER,
            desc=self.cmd_FOLLOWER_help,
        )
        
        # gcode.register_command(
        #     "OAMSM_CURRENT_LOADED_GROUP",
        #     self.cmd_CURRENT_LOADED_GROUP,
        #     desc=self.cmd_CURRENT_LOADED_GROUP_help,
        # )
        
        gcode.register_command(
            "OAMSM_CLEAR_ERRORS",
            self.cmd_CLEAR_ERRORS,
            desc=self.cmd_CLEAR_ERRORS_help,
        )
    
    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"
    def cmd_CLEAR_ERRORS(self, gcmd):
        if len(self.monitor_timers) > 0:
            self.stop_monitors()
        for (fps_name, fps_state) in self.current_state.fps_state.items():
            fps_state.encoder_samples.clear()
            fps_state.reset_stuck_spool_state()
            self._cancel_post_load_pressure_check(fps_state)

        for oams_name, oam in self.oams.items():
            try:
                oam.clear_errors()
            except Exception:
                logging.exception(
                    "OAMS: Failed to clear errors on %s", oams_name
                )
        self.determine_state()
        self.start_monitors()

        return
    
    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        if enable is None:
            gcmd.respond_info("Missing ENABLE parameter")
            return
        direction = gcmd.get_int('DIRECTION')
        if direction is None:
            gcmd.respond_info("Missing DIRECTION parameter")
            return
        fps_name = gcmd.get('FPS')
        fps_name = "fps " + fps_name
        if fps_name is None:
            gcmd.respond_info("Missing FPS parameter")
            return
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state_name == "UNLOADED":
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")
            return
        if fps_state.state_name == "LOADING":
            gcmd.respond_info(f"FPS {fps_name} is currently loading a spool")
            return
        if fps_state.state_name == "UNLOADING":
            gcmd.respond_info(f"FPS {fps_name} is currently unloading a spool")
            return
        oams_obj = self.oams.get(fps_state.current_oams)
        if oams_obj is None:
            gcmd.respond_info(
                f"OAMS {fps_state.current_oams} is not available for follower control"
            )
            return

        try:
            oams_obj.set_oams_follower(enable, direction)
            encoder_clicks = oams_obj.encoder_clicks
            current_spool = oams_obj.current_spool
        except Exception:
            logging.exception(
                "OAMS: Failed to set follower %s direction %s on %s",
                enable,
                direction,
                fps_state.current_oams,
            )
            gcmd.respond_info(
                f"Failed to set follower on {fps_state.current_oams}. Check logs for details."
            )
            return

        fps_state.following = bool(enable)
        fps_state.direction = direction
        fps_state.encoder = encoder_clicks
        fps_state.current_spool_idx = current_spool
        return
    

    def _rebuild_group_fps_index(self) -> None:
        """Build a lookup table from filament groups to their owning FPS."""
        mapping: Dict[str, str] = {}
        for group_name, group in self.filament_groups.items():
            for fps_name, fps in self.fpss.items():
                if any(oam in fps.oams for oam in group.oams):
                    mapping[group_name] = fps_name
                    break
        self.group_to_fps = mapping

    def group_fps_name(self, group_name: str) -> Optional[str]:
        if group_name not in self.group_to_fps and self.fpss:
            self._rebuild_group_fps_index()
        return self.group_to_fps.get(group_name)

    def _normalize_group_name(self, group: Optional[str]) -> Optional[str]:
        """Return a trimmed filament group name or None if invalid."""
        if not group or not isinstance(group, str):
            return None
        group = group.strip()
        if not group:
            return None
        if " " in group:
            group = group.split()[-1]
        return group

    def _rebuild_lane_location_index(self) -> None:
        """Map each (OAMS name, bay index) tuple to its canonical AFC lane."""
        mapping: Dict[Tuple[str, int], str] = {}
        for group_name, lane_name in self._canonical_lane_by_group.items():
            group = self.filament_groups.get(group_name)
            if not group:
                continue
            for oam, bay_index in group.bays:
                mapping[(oam.name, bay_index)] = lane_name
        self._lane_by_location = mapping

    def _ensure_afc_lane_cache(self, afc) -> None:
        """Capture the canonical AFC lane mapping when AFC is available."""
        lanes = getattr(afc, "lanes", {})
        updated = False
        for lane_name, lane in lanes.items():
            canonical_group = self._normalize_group_name(getattr(lane, "_map", None))
            if canonical_group is None:
                canonical_group = self._normalize_group_name(getattr(lane, "map", None))
            if canonical_group:
                if lane_name not in self._canonical_group_by_lane:
                    self._canonical_group_by_lane[lane_name] = canonical_group
                    updated = True
                if canonical_group not in self._canonical_lane_by_group:
                    self._canonical_lane_by_group[canonical_group] = lane_name
                    updated = True
            unit_name = getattr(lane, "unit", None)
            if unit_name and lane_name not in self._lane_unit_map:
                self._lane_unit_map[lane_name] = unit_name
        if updated:
            self._rebuild_lane_location_index()

    def _resolve_lane_for_state(
        self,
        fps_state: 'FPSState',
        group_name: Optional[str],
        afc,
    ) -> Tuple[Optional[str], Optional[str]]:
        """Determine the canonical AFC lane and group for the provided FPS state."""

        normalized_group = self._normalize_group_name(group_name)
        lane_name: Optional[str] = None

        # Prefer the physical OAMS location currently tracked by the FPS state.
        if fps_state.current_oams and fps_state.current_spool_idx is not None:
            lane_name = self._lane_by_location.get(
                (fps_state.current_oams, fps_state.current_spool_idx)
            )
            if lane_name:
                lane_group = self._canonical_group_by_lane.get(lane_name)
                if lane_group:
                    normalized_group = lane_group

        # Fall back to the canonical mapping captured from AFC at startup.
        if lane_name is None and normalized_group:
            lane_name = self._canonical_lane_by_group.get(normalized_group)

        lanes = getattr(afc, "lanes", {})

        # As a last resort, inspect the lanes directly using their original map assignments.
        if lane_name is None and normalized_group:
            lane_name = next(
                (
                    name
                    for name, lane in lanes.items()
                    if self._normalize_group_name(getattr(lane, "_map", None))
                    == normalized_group
                ),
                None,
            )

        canonical_group = normalized_group
        if lane_name:
            lane = lanes.get(lane_name)
            if lane is not None:
                canonical_candidate = self._normalize_group_name(
                    getattr(lane, "_map", None)
                )
                if canonical_candidate is None:
                    canonical_candidate = self._normalize_group_name(
                        getattr(lane, "map", None)
                    )

                updated = False
                if canonical_candidate:
                    canonical_group = canonical_candidate
                    if lane_name not in self._canonical_group_by_lane:
                        self._canonical_group_by_lane[lane_name] = canonical_candidate
                        updated = True
                    if canonical_candidate not in self._canonical_lane_by_group:
                        self._canonical_lane_by_group[canonical_candidate] = lane_name
                        updated = True
                unit_name = getattr(lane, "unit", None)
                if unit_name and lane_name not in self._lane_unit_map:
                    self._lane_unit_map[lane_name] = unit_name
                if updated:
                    self._rebuild_lane_location_index()

        return lane_name, canonical_group

    def _get_afc(self):
        """Lazily retrieve the AFC object if it is available."""
        if self.afc is not None:
            return self.afc
        try:
            afc = self.printer.lookup_object('AFC')
        except Exception:
            self.afc = None
            return None
        self.afc = afc
        self._ensure_afc_lane_cache(afc)
        if not self._afc_logged:
            logging.info("OAMS: AFC integration detected; enabling same-FPS infinite runout support.")
            self._afc_logged = True
        return self.afc

    def _get_infinite_runout_target_group(
        self,
        fps_name: str,
        fps_state: 'FPSState',
    ) -> Tuple[Optional[str], Optional[str], bool, Optional[str]]:
        """
        Return the target filament group and lane for infinite runout, if configured.

        The third element of the tuple indicates whether the runout handling should be
        delegated back to AFC (for example when the configured runout lane is not on
        the same FPS and therefore cannot be handled by OAMS directly).
        """
        current_group = fps_state.current_group
        normalized_group = self._normalize_group_name(current_group)
        if normalized_group is None:
            return None, None, False, None

        afc = self._get_afc()
        if afc is None:
            return None, None, False, None

        lane_name, resolved_group = self._resolve_lane_for_state(
            fps_state,
            normalized_group,
            afc,
        )

        if resolved_group and resolved_group != normalized_group:
            normalized_group = resolved_group
            fps_state.current_group = resolved_group

        if not lane_name:
            logging.debug(
                "OAMS: Unable to resolve AFC lane for group %s on %s",
                normalized_group,
                fps_name,
            )
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
            logging.warning(
                "OAMS: Runout lane %s for %s on %s is not available; deferring to AFC",
                runout_lane_name,
                normalized_group,
                fps_name,
            )
            return None, runout_lane_name, True, lane_name

        source_unit = self._lane_unit_map.get(lane_name)
        target_unit = self._lane_unit_map.get(runout_lane_name)
        if source_unit and target_unit and source_unit != target_unit:
            logging.debug(
                "OAMS: Runout lane %s (%s) for %s on %s belongs to different unit %s; deferring to AFC",
                runout_lane_name,
                target_unit,
                normalized_group,
                fps_name,
                source_unit,
            )
            return None, runout_lane_name, True, lane_name

        source_extruder = getattr(lane, "extruder_obj", None)
        target_extruder = getattr(target_lane, "extruder_obj", None)
        if (
            source_extruder is not None
            and target_extruder is not None
            and source_extruder is not target_extruder
        ):
            logging.debug(
                "OAMS: Deferring infinite runout for %s on %s because lane %s (%s) spools to %s (%s)",
                normalized_group,
                fps_name,
                lane_name,
                getattr(source_extruder, "name", "unknown"),
                runout_lane_name,
                getattr(target_extruder, "name", "unknown"),
            )
            return None, runout_lane_name, True, lane_name

        target_group = self._canonical_group_by_lane.get(runout_lane_name)
        if not target_group:
            target_group = self._normalize_group_name(getattr(target_lane, "_map", None))
        if not target_group:
            target_group = self._normalize_group_name(getattr(target_lane, "map", None))

        if not target_group:
            logging.debug(
                "OAMS: Runout lane %s for %s on %s has no canonical group; deferring to AFC",
                runout_lane_name,
                normalized_group,
                fps_name,
            )
            return None, runout_lane_name, True, lane_name

        updated = False
        if runout_lane_name not in self._canonical_group_by_lane:
            self._canonical_group_by_lane[runout_lane_name] = target_group
            updated = True
        if target_group not in self._canonical_lane_by_group:
            self._canonical_lane_by_group[target_group] = runout_lane_name
            updated = True
        if updated:
            self._rebuild_lane_location_index()

        if target_group == normalized_group:
            logging.debug(
                "OAMS: Runout lane %s for %s on %s does not map to a different filament group; deferring to AFC",
                runout_lane_name,
                normalized_group,
                fps_name,
            )
            return None, runout_lane_name, True, lane_name

        if normalized_group not in self.filament_groups:
            logging.debug(
                "OAMS: Source group %s is not managed by OAMS; deferring to AFC",
                normalized_group,
            )
            return None, runout_lane_name, True, lane_name

        if target_group not in self.filament_groups:
            logging.debug(
                "OAMS: Runout mapping %s -> %s is not managed by OAMS; deferring to AFC",
                normalized_group,
                target_group,
            )
            return None, runout_lane_name, True, lane_name

        source_fps = self.group_fps_name(normalized_group)
        target_fps = self.group_fps_name(target_group)
        if source_fps != fps_name or target_fps != fps_name:
            logging.info(
                "OAMS: Deferring infinite runout for %s on %s to AFC lane %s because target group %s loads via %s",
                normalized_group,
                fps_name,
                runout_lane_name,
                target_group,
                target_fps or "unknown FPS",
            )
            return None, runout_lane_name, True, lane_name

        logging.info(
            "OAMS: Infinite runout configured for %s on %s -> %s (lanes %s -> %s)",
            normalized_group,
            fps_name,
            target_group,
            lane_name,
            runout_lane_name,
        )
        return target_group, runout_lane_name, False, lane_name

    def _delegate_runout_to_afc(
        self,
        fps_name: str,
        fps_state: 'FPSState',
        source_lane_name: Optional[str],
        target_lane_name: Optional[str],
    ) -> bool:
        """Ask AFC to perform the infinite runout swap for the provided lane."""

        afc = self._get_afc()
        if afc is None:
            logging.debug(
                "OAMS: Cannot delegate infinite runout for %s; AFC not available",
                fps_name,
            )
            return False

        if not source_lane_name:
            logging.debug(
                "OAMS: Cannot delegate infinite runout for %s; no source lane recorded",
                fps_name,
            )
            return False

        lane = afc.lanes.get(source_lane_name)
        if lane is None:
            logging.warning(
                "OAMS: AFC lane %s not found while delegating infinite runout for %s",
                source_lane_name,
                fps_name,
            )
            return False

        runout_target = getattr(lane, "runout_lane", None)
        if not runout_target:
            logging.warning(
                "OAMS: AFC lane %s has no runout target while delegating infinite runout for %s",
                source_lane_name,
                fps_name,
            )
            return False

        if target_lane_name and target_lane_name != runout_target:
            logging.debug(
                "OAMS: AFC lane %s runout target mismatch (%s != %s) while delegating infinite runout for %s",
                source_lane_name,
                runout_target,
                target_lane_name,
                fps_name,
            )

        now = self.reactor.monotonic()
        if fps_state.afc_delegation_active and now < fps_state.afc_delegation_until:
            logging.debug(
                "OAMS: AFC infinite runout for %s still in progress; skipping duplicate trigger",
                fps_name,
            )
            return True

        if runout_target not in afc.lanes:
            logging.warning(
                "OAMS: AFC runout lane %s referenced by %s is unavailable",
                runout_target,
                source_lane_name,
            )
            return False

        try:
            lane._perform_infinite_runout()
        except Exception:
            logging.exception(
                "OAMS: AFC infinite runout failed for lane %s -> %s",
                source_lane_name,
                runout_target,
            )
            fps_state.afc_delegation_active = False
            fps_state.afc_delegation_until = 0.0
            return False

        fps_state.afc_delegation_active = True
        fps_state.afc_delegation_until = now + AFC_DELEGATION_TIMEOUT
        logging.info(
            "OAMS: Delegated infinite runout for %s via AFC lane %s -> %s",
            fps_name,
            source_lane_name,
            runout_target,
        )
        return True

    def _unload_filament_for_fps(self, fps_name: str) -> Tuple[bool, str]:
        """Unload filament from the specified FPS and update state."""
        if fps_name not in self.fpss:
            return False, f"FPS {fps_name} does not exist"

        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state_name != FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is not currently loaded"

        if fps_state.current_oams is None:
            return False, f"FPS {fps_name} has no OAMS loaded"

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return False, f"OAMS {fps_state.current_oams} not found for FPS {fps_name}"

        if oams.current_spool is None:
            fps_state.state_name = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            fps_state.since = self.reactor.monotonic()
            self.current_group = None
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
                    lane_name, _ = self._resolve_lane_for_state(
                        fps_state,
                        fps_state.current_group,
                        afc,
                    )
            except Exception:
                logging.exception(
                    "OAMS: Failed to resolve AFC lane for unload on %s", fps_name
                )
                lane_name = None

        try:
            fps_state.state_name = FPSLoadState.UNLOADING
            fps_state.encoder = oams.encoder_clicks
            fps_state.since = self.reactor.monotonic()
            fps_state.current_oams = oams.name
            fps_state.current_spool_idx = oams.current_spool
        except Exception:
            logging.exception(
                "OAMS: Failed to capture unload state for %s", fps_name
            )
            return False, f"Failed to prepare unload on {fps_name}"

        try:
            success, message = oams.unload_spool()
        except Exception:
            logging.exception(
                "OAMS: Exception while unloading filament on %s", fps_name
            )
            return False, f"Exception unloading filament on {fps_name}"

        if success:
            fps_state.state_name = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.since = self.reactor.monotonic()
            if lane_name:
                try:
                    AMSRunoutCoordinator.notify_lane_tool_state(
                        self.printer,
                        fps_state.current_oams or oams.name,
                        lane_name,
                        loaded=False,
                        spool_index=spool_index,
                        eventtime=fps_state.since,
                    )
                except Exception:
                    logging.exception(
                        "OAMS: Failed to notify AFC that lane %s unloaded on %s",
                        lane_name,
                        fps_name,
                    )
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            self.current_group = None
            fps_state.reset_stuck_spool_state()

            fps_state.reset_clog_tracker()

            self._cancel_post_load_pressure_check(fps_state)

            return True, message

        fps_state.state_name = FPSLoadState.LOADED
        return False, message

    def _load_filament_for_group(self, group_name: str) -> Tuple[bool, str]:
        """Load filament for the provided filament group."""
        if group_name not in self.filament_groups:
            return False, f"Group {group_name} does not exist"

        fps_name = self.group_fps_name(group_name)
        if fps_name is None:
            return False, f"No FPS associated with group {group_name}"

        fps_state = self.current_state.fps_state[fps_name]

        self._cancel_post_load_pressure_check(fps_state)

        for (oam, bay_index) in self.filament_groups[group_name].bays:
            try:
                is_ready = oam.is_bay_ready(bay_index)
            except Exception:
                logging.exception(
                    "OAMS: Failed to query readiness of bay %s on %s",
                    bay_index,
                    getattr(oam, "name", "<unknown>"),
                )
                continue

            if not is_ready:
                continue

            try:
                fps_state.state_name = FPSLoadState.LOADING
                fps_state.encoder = oam.encoder_clicks
                fps_state.since = self.reactor.monotonic()
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
            except Exception:
                logging.exception(
                    "OAMS: Failed to capture load state for group %s bay %s",
                    group_name,
                    bay_index,
                )
                fps_state.state_name = FPSLoadState.UNLOADED
                fps_state.current_group = None
                fps_state.current_spool_idx = None
                fps_state.current_oams = None
                continue

            try:
                success, message = oam.load_spool(bay_index)
            except Exception:
                logging.exception(
                    "OAMS: Exception while loading group %s bay %s",
                    group_name,
                    bay_index,
                )
                success, message = False, f"Exception loading spool {bay_index} on {group_name}"

            if success:
                fps_state.current_group = group_name
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                fps_state.state_name = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.direction = 1
                self.current_group = group_name
                fps_state.reset_stuck_spool_state()

                fps_state.reset_clog_tracker()

                self._ensure_forward_follower(
                    fps_name,
                    fps_state,
                    "load filament",
                )

                self._schedule_post_load_pressure_check(fps_name, fps_state)

                if AMSRunoutCoordinator is not None:
                    lane_name: Optional[str] = None
                    try:
                        afc = self._get_afc()
                        if afc is not None:
                            lane_name, _ = self._resolve_lane_for_state(
                                fps_state,
                                group_name,
                                afc,
                            )
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to resolve AFC lane for group %s on %s",
                            group_name,
                            fps_name,
                        )
                        lane_name = None

                    if lane_name:
                        try:
                            AMSRunoutCoordinator.notify_lane_tool_state(
                                self.printer,
                                fps_state.current_oams or oam.name,
                                lane_name,
                                loaded=True,
                                spool_index=fps_state.current_spool_idx,
                                eventtime=fps_state.since,
                            )
                        except Exception:
                            logging.exception(
                                "OAMS: Failed to notify AFC that lane %s loaded for %s",
                                lane_name,
                                group_name,
                            )

                return True, message

            fps_state.state_name = FPSLoadState.UNLOADED
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            fps_state.current_oams = None

            fps_state.following = False
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()

            self._cancel_post_load_pressure_check(fps_state)

            return False, message

        return False, f"No spool available for group {group_name}"

    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps_name = gcmd.get('FPS')
        if fps_name is None:
            gcmd.respond_info("Missing FPS parameter")
            return
        fps_name = "fps " + fps_name
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state_name == "UNLOADED":
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")
            return
        if fps_state.state_name == "LOADING":
            gcmd.respond_info(f"FPS {fps_name} is currently loading a spool")
            return
        if fps_state.state_name == "UNLOADING":
            gcmd.respond_info(f"FPS {fps_name} is currently unloading a spool")
            return

        success, message = self._unload_filament_for_fps(fps_name)
        if not success or (message and message != "Spool unloaded successfully"):
            gcmd.respond_info(message)
        return

    cmd_LOAD_FILAMENT_help = "Load a spool from an specific group"
    def cmd_LOAD_FILAMENT(self, gcmd):
        # determine which fps this group is assigned to
        group_name = gcmd.get('GROUP')
        if group_name not in self.filament_groups:
            gcmd.respond_info(f"Group {group_name} does not exist")
            return
        fps_name = self.group_fps_name(group_name)
        fps_state = self.current_state.fps_state[fps_name]
        if self.current_state.fps_state[fps_name].state_name == "LOADED":
            gcmd.respond_info(f"Group {group_name} is already loaded")
            return
        success, message = self._load_filament_for_group(group_name)
        gcmd.respond_info(message)
        return

        
    def _pause_printer_message(self, message, oams_name: Optional[str] = None):
        logging.info(f"OAMS: {message}")

        if AMSRunoutCoordinator is not None and oams_name:
            try:
                AMSRunoutCoordinator.notify_afc_error(
                    self.printer, oams_name, message, pause=False
                )
            except Exception:
                logging.getLogger(__name__).exception(
                    "Failed to forward OAMS pause message to AFC"
                )

        try:
            gcode = self.printer.lookup_object("gcode")
        except Exception:
            logging.exception("OAMS: Failed to look up gcode object for pause message")
            return

        pause_message = f"Print has been paused: {message}"
        try:
            gcode.run_script(f"M118 {pause_message}")
            gcode.run_script(f"M114 {pause_message}")
        except Exception:
            logging.exception("OAMS: Failed to send pause notification gcode")

        try:
            toolhead = self.printer.lookup_object("toolhead")
            homed_axes = toolhead.get_status(self.reactor.monotonic()).get("homed_axes", "")
        except Exception:
            logging.exception("OAMS: Failed to query toolhead state during pause handling")
            return

        if all(axis in homed_axes for axis in ("x", "y", "z")):
            try:
                gcode.run_script("PAUSE")
            except Exception:
                logging.exception("OAMS: Failed to run PAUSE script for clog handling")
        else:
            logging.warning(
                "OAMS: Skipping PAUSE command because axes are not homed (homed_axes=%s)",
                homed_axes,
            )


    def _cancel_post_load_pressure_check(self, fps_state: "FPSState") -> None:
        """Stop any pending post-load pressure validation timer."""
        timer = getattr(fps_state, "post_load_pressure_timer", None)
        if timer is not None:
            try:
                self.reactor.unregister_timer(timer)
            except Exception:
                logging.exception("OAMS: Failed to cancel post-load pressure timer")
        fps_state.post_load_pressure_timer = None
        fps_state.post_load_pressure_start = None

    def _schedule_post_load_pressure_check(
        self,
        fps_name: str,
        fps_state: "FPSState",
    ) -> None:
        """Verify the FPS pressure settles after a successful load."""

        self._cancel_post_load_pressure_check(fps_state)

        def _monitor_pressure(self, eventtime):
            tracked_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)

            if tracked_state is None or fps is None:
                if tracked_state is not None:
                    self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            if tracked_state.state_name != FPSLoadState.LOADED:
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

            if now - tracked_state.post_load_pressure_start < POST_LOAD_PRESSURE_DWELL:
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            oams_obj = None
            if tracked_state.current_oams is not None:
                oams_obj = self.oams.get(tracked_state.current_oams)
            if (
                oams_obj is not None
                and tracked_state.current_spool_idx is not None
            ):
                try:
                    oams_obj.set_led_error(tracked_state.current_spool_idx, 1)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to set clog LED on %s spool %s after loading",
                        fps_name,
                        tracked_state.current_spool_idx,
                    )

            tracked_state.clog_active = True

            message = (
                f"Possible clog detected after loading {tracked_state.current_group or fps_name}: "
                f"FPS pressure {pressure:.2f} remained above {POST_LOAD_PRESSURE_THRESHOLD:.2f}"
            )
            self._pause_printer_message(message, tracked_state.current_oams)
            self._cancel_post_load_pressure_check(tracked_state)
            return self.reactor.NEVER

        timer = self.reactor.register_timer(
            partial(_monitor_pressure, self),
            self.reactor.NOW,
        )
        fps_state.post_load_pressure_timer = timer
        fps_state.post_load_pressure_start = None


    def _enable_follower(
        self,
        fps_name: str,
        fps_state: "FPSState",
        oams: Optional[Any],
        direction: int,
        context: str,
    ) -> None:
        """Ensure the follower is running in the requested direction."""
        if fps_state.current_spool_idx is None:
            return

        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        direction = direction if direction in (0, 1) else 1

        try:
            oams.set_oams_follower(1, direction)
            fps_state.following = True
            fps_state.direction = direction
            logging.debug(
                "OAMS: Enabled follower for %s spool %s after %s.",
                fps_name,
                fps_state.current_spool_idx,
                context,
            )
        except Exception:
            logging.exception(
                "OAMS: Failed to enable follower for %s after %s",
                fps_name,
                context,
            )


    def _ensure_forward_follower(
        self,
        fps_name: str,
        fps_state: "FPSState",
        context: str,
    ) -> None:
        """Start the follower forward whenever a spool is loaded."""
        if (
            fps_state.current_oams is None
            or fps_state.current_spool_idx is None
            or fps_state.stuck_spool_active
            or fps_state.state_name != FPSLoadState.LOADED
        ):
            return

        if fps_state.following and fps_state.direction == 1:
            return

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        fps_state.direction = 1
        self._enable_follower(
            fps_name,
            fps_state,
            oams,
            1,
            context,
        )


    def _restore_follower_if_needed(
        self,
        fps_name: str,
        fps_state: "FPSState",
        oams: Optional[Any],
        context: str,
    ) -> None:
        """Restore the follower if a stuck spool pause disabled it."""
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


        self._enable_follower(
            fps_name,
            fps_state,
            oams,
            direction,
            context,
        )
        if fps_state.following:

            fps_state.stuck_spool_restore_follower = False
            logging.info(
                "OAMS: Restarted follower for %s spool %s after %s.",
                fps_name,
                fps_state.current_spool_idx,
                context,
            )


    def _handle_printing_resumed(self, _eventtime):
        """Re-enable any followers that were paused due to a stuck spool."""
        for fps_name, fps_state in self.current_state.fps_state.items():

            oams = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None
            if fps_state.stuck_spool_restore_follower:
                self._restore_follower_if_needed(
                    fps_name,
                    fps_state,
                    oams,
                    "print resume",
                )
            elif (
                fps_state.current_oams is not None
                and fps_state.current_spool_idx is not None
                and not fps_state.following

            ):
                self._ensure_forward_follower(
                    fps_name,
                    fps_state,
                    "print resume",
                )


    def _trigger_stuck_spool_pause(
        self,
        fps_name: str,
        fps_state: "FPSState",
        oams: Optional[Any],
        message: str,
    ) -> None:
        """Pause the printer and set LED indicators for a stuck spool."""
        if fps_state.stuck_spool_active:
            return

        spool_idx = fps_state.current_spool_idx
        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)

        if oams is not None and spool_idx is not None:
            try:
                oams.set_led_error(spool_idx, 1)
            except Exception:
                logging.exception(
                    "OAMS: Failed to set stuck spool LED on %s spool %s",
                    fps_name,
                    spool_idx,
                )

            direction = fps_state.direction if fps_state.direction in (0, 1) else 1
            fps_state.direction = direction
            fps_state.stuck_spool_restore_follower = True
            fps_state.stuck_spool_restore_direction = direction
            if fps_state.following:

                try:
                    oams.set_oams_follower(0, direction)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to stop follower for %s spool %s during stuck spool pause",
                        fps_name,
                        spool_idx,
                    )

            fps_state.following = False


        if oams is not None:
            try:
                oams.abort_current_action()
            except Exception:
                logging.exception(
                    "OAMS: Failed to abort active action for %s during stuck spool pause",
                    fps_name,
                )


        fps_state.stuck_spool_active = True
        fps_state.stuck_spool_start_time = None

        self._pause_printer_message(message, fps_state.current_oams)

    def _monitor_unload_speed_for_fps(self, fps_name):
        def _monitor_unload_speed(self, eventtime):
            #logging.info("OAMS: Monitoring unloading speed state: %s" % self.current_state.name)
            fps_state = self.current_state.fps_state[fps_name]
            oams = None
            if fps_state.current_oams is not None:
                oams = self.oams.get(fps_state.current_oams)
            if (
                fps_state.state_name == "UNLOADING"
                and self.reactor.monotonic() - fps_state.since
                > MONITOR_ENCODER_UNLOADING_SPEED_AFTER
            ):
                if oams is None:
                    return eventtime + MONITOR_ENCODER_PERIOD
                try:
                    encoder_value = oams.encoder_clicks
                except Exception:
                    logging.exception(
                        "OAMS: Failed to read encoder while monitoring unload on %s",
                        fps_name,
                    )
                    return eventtime + MONITOR_ENCODER_PERIOD

                fps_state.encoder_samples.append(encoder_value)
                if len(fps_state.encoder_samples) < ENCODER_SAMPLES:
                    return eventtime + MONITOR_ENCODER_PERIOD
                encoder_diff = abs(
                    fps_state.encoder_samples[-1]
                    - fps_state.encoder_samples[0]
                )
                logging.info(
                    "OAMS[%d] Unload Monitor: Encoder diff %d",
                    getattr(oams, "oams_idx", -1),
                    encoder_diff,
                )
                if encoder_diff < MIN_ENCODER_DIFF:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 1)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to set unload LED on %s",
                            fps_name,
                        )
                    self._pause_printer_message(
                        "Printer paused because the unloading speed of the moving filament was too low",
                        fps_state.current_oams,
                    )
                    logging.info("after unload speed too low")
                    self.stop_monitors()
                    return self.printer.get_reactor().NEVER
            return eventtime + MONITOR_ENCODER_PERIOD
        return partial(_monitor_unload_speed, self)
    
    def _monitor_load_speed_for_fps(self, fps_name):
        def _monitor_load_speed(self, eventtime):
            #logging.info("OAMS: Monitoring loading speed state: %s" % self.current_state.name)
            fps_state = self.current_state.fps_state[fps_name]
            oams = None
            if fps_state.current_oams is not None:
                oams = self.oams.get(fps_state.current_oams)
            if fps_state.stuck_spool_active:
                return eventtime + MONITOR_ENCODER_PERIOD
            if (
                fps_state.state_name == "LOADING"
                and self.reactor.monotonic() - fps_state.since
                > MONITOR_ENCODER_LOADING_SPEED_AFTER
            ):
                if oams is None:
                    return eventtime + MONITOR_ENCODER_PERIOD
                try:
                    encoder_value = oams.encoder_clicks
                except Exception:
                    logging.exception(
                        "OAMS: Failed to read encoder while monitoring load on %s",
                        fps_name,
                    )
                    return eventtime + MONITOR_ENCODER_PERIOD

                fps_state.encoder_samples.append(encoder_value)
                if len(fps_state.encoder_samples) < ENCODER_SAMPLES:
                    return eventtime + MONITOR_ENCODER_PERIOD
                encoder_diff = abs(fps_state.encoder_samples[-1] - fps_state.encoder_samples[0])
                logging.info(
                    "OAMS[%d] Load Monitor: Encoder diff %d",
                    getattr(oams, "oams_idx", -1),
                    encoder_diff,
                )
                if encoder_diff < MIN_ENCODER_DIFF:
                    group_label = fps_state.current_group or fps_name
                    spool_label = (
                        str(fps_state.current_spool_idx)
                        if fps_state.current_spool_idx is not None
                        else "unknown"
                    )
                    message = (
                        "Spool appears stuck while loading"
                        if fps_state.current_group is None
                        else f"Spool appears stuck while loading {group_label} spool {spool_label}"
                    )
                    self._trigger_stuck_spool_pause(
                        fps_name,
                        fps_state,
                        oams,
                        message,
                    )
                    self.stop_monitors()
                    return self.printer.get_reactor().NEVER
            return eventtime + MONITOR_ENCODER_PERIOD
        return partial(_monitor_load_speed, self)


    def _monitor_stuck_spool_for_fps(self, fps_name):
        def _monitor_stuck_spool(self, eventtime):
            fps_state = self.current_state.fps_state[fps_name]
            fps = self.fpss.get(fps_name)
            if fps is None:
                if fps_state.stuck_spool_active and fps_state.current_oams is not None:
                    oams_obj = self.oams.get(fps_state.current_oams)
                    if (
                        oams_obj is not None
                        and fps_state.current_spool_idx is not None
                    ):
                        try:
                            oams_obj.set_led_error(fps_state.current_spool_idx, 0)
                        except Exception:
                            logging.exception(
                                "OAMS: Failed to clear stuck spool LED on %s while fps missing",
                                fps_name,
                            )
                fps_state.reset_stuck_spool_state()
                return eventtime + MONITOR_ENCODER_PERIOD

            if fps_state.state_name != FPSLoadState.LOADED:
                if (
                    fps_state.stuck_spool_active
                    and fps_state.current_oams is not None
                    and fps_state.current_spool_idx is not None
                ):
                    oams_obj = self.oams.get(fps_state.current_oams)
                    if oams_obj is not None:
                        try:
                            oams_obj.set_led_error(fps_state.current_spool_idx, 0)
                        except Exception:
                            logging.exception(
                                "OAMS: Failed to clear stuck spool LED on %s while not loaded",
                                fps_name,
                            )
                fps_state.reset_stuck_spool_state()
                return eventtime + MONITOR_ENCODER_PERIOD

            oams = None
            if fps_state.current_oams is not None:
                oams = self.oams.get(fps_state.current_oams)
            if oams is None or fps_state.current_spool_idx is None:
                if (
                    fps_state.stuck_spool_active
                    and oams is not None
                    and fps_state.current_spool_idx is not None
                ):
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to clear stuck spool LED on %s without active spool",
                            fps_name,
                        )
                fps_state.reset_stuck_spool_state()
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                idle_timeout = self.printer.lookup_object("idle_timeout")
                is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            except Exception:
                is_printing = False

            monitor = self.runout_monitors.get(fps_name)
            if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
                if fps_state.stuck_spool_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to clear stuck spool LED while runout monitor inactive on %s",
                            fps_name,
                        )

                fps_state.reset_stuck_spool_state(
                    preserve_restore=fps_state.stuck_spool_restore_follower
                )

                return eventtime + MONITOR_ENCODER_PERIOD

            if not is_printing:
                if fps_state.stuck_spool_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to clear stuck spool LED while idle on %s",
                            fps_name,
                        )

                fps_state.reset_stuck_spool_state(
                    preserve_restore=fps_state.stuck_spool_restore_follower
                )

                return eventtime + MONITOR_ENCODER_PERIOD

            if fps_state.state_name != FPSLoadState.LOADED or fps_state.current_spool_idx is None:
                if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to clear stuck spool LED while %s not loaded",
                            fps_name,
                        )

                fps_state.reset_stuck_spool_state(
                    preserve_restore=fps_state.stuck_spool_restore_follower
                )

                return eventtime + MONITOR_ENCODER_PERIOD

            pressure = float(getattr(fps, "fps_value", 0.0))
            now = self.reactor.monotonic()

            if fps_state.since is not None and now - fps_state.since < STUCK_SPOOL_LOAD_GRACE:
                fps_state.stuck_spool_start_time = None
                return eventtime + MONITOR_ENCODER_PERIOD

            if not fps_state.following or fps_state.direction != 1:
                fps_state.stuck_spool_start_time = None

                if (
                    fps_state.stuck_spool_restore_follower
                    and is_printing
                    and oams is not None
                ):
                    self._restore_follower_if_needed(
                        fps_name,
                        fps_state,
                        oams,
                        "stuck spool recovery",
                    )

                return eventtime + MONITOR_ENCODER_PERIOD

            if pressure <= STUCK_SPOOL_PRESSURE_THRESHOLD:
                if fps_state.stuck_spool_start_time is None:
                    fps_state.stuck_spool_start_time = now
                elif (
                    not fps_state.stuck_spool_active
                    and now - fps_state.stuck_spool_start_time >= STUCK_SPOOL_DWELL
                ):
                    message = "Spool appears stuck"
                    if fps_state.current_group is not None:
                        message = (
                            f"Spool appears stuck on {fps_state.current_group} spool {fps_state.current_spool_idx}"
                        )
                    self._trigger_stuck_spool_pause(
                        fps_name,
                        fps_state,
                        oams,
                        message,
                    )
            else:
                if fps_state.stuck_spool_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to clear stuck spool LED on %s spool %d",
                            fps_name,
                            fps_state.current_spool_idx,
                        )

                if fps_state.stuck_spool_restore_follower and is_printing:
                    self._restore_follower_if_needed(
                        fps_name,
                        fps_state,
                        oams,
                        "stuck spool recovery",
                    )

                elif is_printing and not fps_state.following:
                    self._ensure_forward_follower(
                        fps_name,
                        fps_state,

                        "stuck spool recovery",
                    )
                if not fps_state.stuck_spool_restore_follower:
                    fps_state.reset_stuck_spool_state()


            return eventtime + MONITOR_ENCODER_PERIOD

        return partial(_monitor_stuck_spool, self)

    def _monitor_clog_for_fps(self, fps_name):
        def _monitor_clog(self, eventtime):
            fps_state = self.current_state.fps_state[fps_name]
            fps = self.fpss.get(fps_name)

            if fps_state.state_name != FPSLoadState.LOADED:
                if (
                    fps_state.clog_active
                    and fps_state.current_oams is not None
                    and fps_state.current_spool_idx is not None
                ):
                    oams_obj = self.oams.get(fps_state.current_oams)
                    if oams_obj is not None:
                        try:
                            oams_obj.set_led_error(fps_state.current_spool_idx, 0)
                        except Exception:
                            logging.exception(
                                "OAMS: Failed to clear clog LED on %s spool %s while idle",
                                fps_name,
                                fps_state.current_spool_idx,
                            )
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            oams = None
            if fps_state.current_oams is not None:
                oams = self.oams.get(fps_state.current_oams)
            if oams is None or fps_state.current_spool_idx is None or fps is None:
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                hes_values = oams.hub_hes_value
                spool_available = bool(
                    hes_values[fps_state.current_spool_idx]
                )
            except Exception:
                logging.exception(
                    "OAMS: Failed to read HES values while monitoring clogs on %s",
                    fps_name,
                )
                return eventtime + MONITOR_ENCODER_PERIOD

            if spool_available:
                if fps_state.clog_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to clear clog LED on %s spool %s after runout",
                            fps_name,
                            fps_state.current_spool_idx,
                        )
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                idle_timeout = self.printer.lookup_object("idle_timeout")
                is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            except Exception:
                is_printing = False

            monitor = self.runout_monitors.get(fps_name)
            if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
                if fps_state.clog_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to clear clog LED on %s spool %s while runout monitor inactive",
                            fps_name,
                            fps_state.current_spool_idx,
                        )
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            if not is_printing:
                if fps_state.clog_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to clear clog LED on %s spool %s while printer idle",
                            fps_name,
                            fps_state.current_spool_idx,
                        )
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                extruder_pos = float(getattr(fps.extruder, "last_position", 0.0))
            except Exception:
                logging.exception(
                    "OAMS: Failed to read extruder position while monitoring clogs on %s",
                    fps_name,
                )
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                encoder_clicks = int(getattr(oams, "encoder_clicks", 0))
            except Exception:
                logging.exception(
                    "OAMS: Failed to read encoder clicks while monitoring clogs on %s",
                    fps_name,
                )
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                pressure = float(getattr(fps, "fps_value", 0.0))
            except Exception:
                logging.exception(
                    "OAMS: Failed to read FPS pressure while monitoring clogs on %s",
                    fps_name,
                )
                return eventtime + MONITOR_ENCODER_PERIOD
            now = self.reactor.monotonic()

            if fps_state.clog_start_extruder is None:
                fps_state.prime_clog_tracker(extruder_pos, encoder_clicks, pressure, now)
                return eventtime + MONITOR_ENCODER_PERIOD

            if extruder_pos < (fps_state.clog_last_extruder or extruder_pos):
                fps_state.prime_clog_tracker(extruder_pos, encoder_clicks, pressure, now)
                return eventtime + MONITOR_ENCODER_PERIOD

            fps_state.clog_last_extruder = extruder_pos
            if fps_state.clog_min_pressure is None or pressure < fps_state.clog_min_pressure:
                fps_state.clog_min_pressure = pressure
            if fps_state.clog_max_pressure is None or pressure > fps_state.clog_max_pressure:
                fps_state.clog_max_pressure = pressure

            extrusion_delta = extruder_pos - (fps_state.clog_start_extruder or extruder_pos)
            encoder_delta = abs(encoder_clicks - (fps_state.clog_start_encoder or encoder_clicks))
            pressure_span = (fps_state.clog_max_pressure or pressure) - (
                fps_state.clog_min_pressure or pressure
            )

            settings = self.clog_settings
            if extrusion_delta < settings["extrusion_window"]:
                return eventtime + MONITOR_ENCODER_PERIOD

            if (
                encoder_delta > settings["encoder_slack"]
                or pressure_span > settings["pressure_band"]
            ):
                fps_state.prime_clog_tracker(extruder_pos, encoder_clicks, pressure, now)
                return eventtime + MONITOR_ENCODER_PERIOD

            if now - (fps_state.clog_start_time or now) < settings["dwell"]:
                return eventtime + MONITOR_ENCODER_PERIOD

            if not fps_state.clog_active:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to set clog LED on %s spool %s",
                        fps_name,
                        fps_state.current_spool_idx,
                    )
                pressure_mid = (fps_state.clog_min_pressure + fps_state.clog_max_pressure) / 2.0
                message = (
                    f"Clog suspected on {fps_state.current_group or fps_name}: "
                    f"extruder advanced {extrusion_delta:.1f}mm while encoder moved {encoder_delta} counts "
                    f"with FPS {pressure_mid:.2f} near {CLOG_PRESSURE_TARGET:.2f}"
                )
                fps_state.clog_active = True
                self._pause_printer_message(message, fps_state.current_oams)

            return eventtime + MONITOR_ENCODER_PERIOD

        return partial(_monitor_clog, self)



    def start_monitors(self):
        self.monitor_timers = []
        self.runout_monitors = {}
        reactor = self.printer.get_reactor()
        for (fps_name, fps_state) in self.current_state.fps_state.items():
            self.monitor_timers.append(reactor.register_timer(self._monitor_unload_speed_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_load_speed_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_stuck_spool_for_fps(fps_name), reactor.NOW))

            self.monitor_timers.append(reactor.register_timer(self._monitor_clog_for_fps(fps_name), reactor.NOW))


            def _reload_callback(fps_name=fps_name, fps_state=fps_state):
                monitor = self.runout_monitors.get(fps_name)
                source_group = fps_state.current_group
                active_oams = fps_state.current_oams
                target_group, target_lane, delegate_to_afc, source_lane = self._get_infinite_runout_target_group(
                    fps_name,
                    fps_state,
                )
                source_group = fps_state.current_group

                if delegate_to_afc:
                    delegated = self._delegate_runout_to_afc(
                        fps_name,
                        fps_state,
                        source_lane,
                        target_lane,
                    )
                    if delegated:
                        fps_state.reset_runout_positions()
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        return

                    logging.error(
                        "OAMS: Failed to delegate infinite runout for %s on %s via AFC",
                        fps_name,
                        source_group or "<unknown>",
                    )
                    fps_state.reset_runout_positions()
                    self._pause_printer_message(
                        f"Unable to delegate infinite runout for {source_group or fps_name}",
                        fps_state.current_oams or active_oams,
                    )
                    if monitor:
                        monitor.paused()
                    return

                group_to_load = target_group or source_group

                if target_group:
                    logging.info(
                        "OAMS: Infinite runout triggered for %s on %s -> %s",
                        fps_name,
                        source_group,
                        target_group,
                    )
                    unload_success, unload_message = self._unload_filament_for_fps(fps_name)
                    if not unload_success:
                        logging.error(
                            "OAMS: Failed to unload filament during infinite runout on %s: %s",
                            fps_name,
                            unload_message,
                        )
                        failure_message = unload_message or f"Failed to unload current spool on {fps_name}"
                        self._pause_printer_message(
                            failure_message,
                            fps_state.current_oams or active_oams,
                        )
                        if monitor:
                            monitor.paused()
                        return

                if group_to_load is None:
                    logging.error("OAMS: No filament group available to reload on %s", fps_name)
                    self._pause_printer_message(
                        f"No filament group available to reload on {fps_name}",
                        fps_state.current_oams or active_oams,
                    )
                    if monitor:
                        monitor.paused()
                    return

                load_success, load_message = self._load_filament_for_group(group_to_load)
                if load_success:
                    logging.info(
                        "OAMS: Successfully loaded group %s on %s%s",
                        group_to_load,
                        fps_name,
                        " after infinite runout" if target_group else "",
                    )
                    if target_group:
                        if target_lane:
                            handled = False
                            if AMSRunoutCoordinator is not None:
                                try:
                                    handled = AMSRunoutCoordinator.notify_lane_tool_state(
                                        self.printer,
                                        fps_state.current_oams or active_oams,
                                        target_lane,
                                        loaded=True,
                                        spool_index=fps_state.current_spool_idx,
                                        eventtime=fps_state.since,
                                    )
                                except Exception:
                                    logging.exception(
                                        "OAMS: Failed to notify AFC lane %s after infinite runout on %s",
                                        target_lane,
                                        fps_name,
                                    )
                                    handled = False
                            if not handled:
                                try:
                                    gcode = self.printer.lookup_object("gcode")
                                    gcode.run_script(f"SET_LANE_LOADED LANE={target_lane}")
                                    logging.debug(
                                        "OAMS: Marked lane %s as loaded after infinite runout on %s",
                                        target_lane,
                                        fps_name,
                                    )
                                except Exception:
                                    logging.exception(
                                        "OAMS: Failed to mark lane %s as loaded after infinite runout on %s",
                                        target_lane,
                                        fps_name,
                                    )
                        else:
                            logging.warning(
                                "OAMS: No runout lane recorded for %s on %s when marking lane loaded",
                                target_group,
                                fps_name,
                            )
                    fps_state.reset_runout_positions()
                    if monitor:
                        monitor.reset()
                        monitor.start()
                    return

                logging.error(
                    "OAMS: Failed to load group %s on %s: %s",
                    group_to_load,
                    fps_name,
                    load_message,
                )
                failure_message = load_message or f"No spool available for group {group_to_load}"
                self._pause_printer_message(
                    failure_message,
                    fps_state.current_oams or active_oams,
                )
                if monitor:
                    monitor.paused()
                return

            fps_reload_margin = getattr(
                self.fpss[fps_name],
                "reload_before_toolhead_distance",
                None,
            )
            if fps_reload_margin is None:
                fps_reload_margin = self.reload_before_toolhead_distance
            else:
                logging.debug(
                    "OAMS: Using FPS-specific reload margin %.2f mm for %s",
                    fps_reload_margin,
                    fps_name,
                )

            monitor = OAMSRunoutMonitor(
                self.printer,
                fps_name,
                self.fpss[fps_name],
                fps_state,
                self.oams,
                _reload_callback,
                reload_before_toolhead_distance=fps_reload_margin,
            )
            self.runout_monitors[fps_name] = monitor
            monitor.start()

        logging.info("OAMS: All monitors started")

    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []
        for monitor in self.runout_monitors.values():
            monitor.reset()
        self.runout_monitors = {}


def load_config(config):

    return OAMSManager(config)

