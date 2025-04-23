# OpenAMS Manager
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


import logging
import time
from functools import partial
from collections import deque
from typing import Optional, Tuple

PAUSE_DISTANCE = 60
ENCODER_SAMPLES = 2
MIN_ENCODER_DIFF = 1
FILAMENT_PATH_LENGTH_FACTOR = 1.14  # Replace magic number with a named constant
MONITOR_ENCODER_LOADING_SPEED_AFTER = 2.0 # in seconds
MONITOR_ENCODER_UNLOADING_SPEED_AFTER = 2.0 # in seconds

class OAMSState:
    def __init__(self):
        self.fps_state = {}
    def add_fps_state(self, fps_name):
        self.fps_state[fps_name] = FPSState()
        
class FPSState:
    def __init__(self, state_name="UNLOADED", current_group = None, current_oams = None, current_spool_idx=None):
        
        self.state_name = state_name # name of the state, e.g. LOADED, UNLOADED, LOADING, UNLOADING
        self.current_group = current_group # name of the group T0, T1, T2, etc., if any
        self.current_oams = current_oams # name of the OAMS loaded, if any
        self.current_spool_idx = current_spool_idx # index of the spool loaded, if any
        
        self.runout_position = None
        self.runout_after_position = None
        
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None
        
        self.encoder_samples = deque(maxlen=ENCODER_SAMPLES)
        
        self.following = False
        self.direction = 0
        self.since = None
        
    def reset_runout_positions(self):
        self.runout_position = None
        self.runout_after_position = None

    def __repr__(self):
        return f"FPSState(state_name={self.state_name}, current_group={self.current_group}, current_oams={self.current_oams}, current_spool_idx={self.current_spool_idx})"

    def __str__(self):
        return f"State: {self.state_name}, Group: {self.current_group}, OAMS: {self.current_oams}, Spool Index: {self.current_spool_idx}"

class OAMSManager:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.filament_groups = {}
        self.oams = {}
        self._initialize_oams()
        self._initialize_filament_groups()
        self.current_state = OAMSState()
        self.reactor = self.printer.get_reactor()
        
        self.monitor_timers = []
        self.ready = False

        self.fpss = {}
        for (fps_name, fps) in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)
        
        self.reload_before_toolhead_distance = config.getfloat("reload_before_toolhead_distance", 0.0)
    
    def determine_state(self):
        for (fps_name, fps_state) in self.current_state.fps_state.items():
            fps_state.current_group, fps_state.current_oam, fps_state.current_spool_idx = self.determine_current_loaded_group(fps_name)
            if fps_state.current_oam is not None and fps_state.current_spool_idx is not None:
                fps_state.state_name = "LOADED"
                fps_state.since = self.reactor.monotonic()
        
    def handle_ready(self):
        self.determine_state()
        self.start_monitors()
        self.ready = True
    
    def _pause_before_coasting(self, eventtime, fps_name, initial_position, pause_distance):
        extruder = self.fpss[fps_name].extruder
        fps_state = self.current_state.fps_state[fps_name]
        current_position = extruder.last_position
        traveled_distance = current_position - initial_position
        if traveled_distance >= pause_distance:
            logging.info("OAMS: Pause complete, coasting the follower.")
            self.oams[fps_state.current_oams].set_oams_follower(0, 1)
            fps_state.load_next_spool_timer = self._register_load_next_spool_timer(eventtime, fps_name, pause_distance)
            return self.printer.get_reactor().NEVER
        return eventtime + 1.0
    
    def _register_pause_timer(self, eventtime, fps_name, pause_distance):
        logging.info(f"OAMS: Filament runout detected on FPS {fps_name}, pausing for {pause_distance} mm before coasting the follower.")
        fps = self.fpss[fps_name]
        fps_state = self.current_state.fps_state[fps_name]
        extruder = fps.extruder
        initial_position = extruder.last_position
        fps_state.monitor_pause_timer = self.printer.get_reactor().register_timer(
            lambda et: self._pause_before_coasting(et, fps_name, initial_position, pause_distance), eventtime)
    
    def _load_next_spool(self, eventtime,fps_name, pause_distance):
        fps = self.fpss[fps_name]
        extruder = fps.extruder
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.runout_position is None:
            fps_state.runout_position = extruder.last_position
            logging.info(f"OAMS: Runout position set to {fps_state.runout_position}")
        else:
            fps_state.runout_after_position = extruder.last_position - fps_state.runout_position
            logging.info(f"OAMS: Traveled after runout: {fps_state.runout_after_position}")
            if fps_state.runout_after_position + pause_distance + self.reload_before_toolhead_distance > self.oams[fps_state.current_oams].filament_path_length / FILAMENT_PATH_LENGTH_FACTOR:
                logging.info("OAMS: Loading next spool in the filament group.")
                for (oam, bay_index) in self.filament_groups[fps_state.current_group].bays:
                    if oam.is_bay_ready(bay_index):
                        success, message = oam.load_spool(bay_index)
                        if success:
                            logging.info(f"OAMS: Successfully loaded spool in bay {bay_index} of OAM {oam.name}")
                            fps_state.state_name = "LOADED"
                            fps_state.since = self.reactor.monotonic()
                            fps_state.current_spool_idx = bay_index
                            fps_state.current_oams = oam.name
                            fps_state.reset_runout_positions()
                            self._register_monitor_spool_timer()
                            return self.printer.get_reactor().NEVER
                        else:
                            logging.error(f"OAMS: Failed to load spool: {message}")
                            raise Exception(message)
                self._pause_print(fps_name)
        return eventtime + 1.0
    
    def _register_load_next_spool_timer(self, eventtime, fps_name, pause_distance):
        fps_state = self.current_state.fps_state[fps_name]
        logging.info("OAMS: Registering timer to load next spool.")
        fps_state.monitor_load_next_spool_timer = self.printer.get_reactor().register_timer(
            lambda et: self._load_next_spool(et, fps_name, pause_distance), eventtime)
    
    def _pause_print(self, fps_name):
        fps = self.fpss[fps_name]
        fps_state = self.current_state.fps_state[fps_name]
        logging.info("OAMS: No spool available, pausing the print.")
        gcode = self.printer.lookup_object("gcode")
        message = f"Print has been paused due to filament runout on group {fps_state.current_group} on FPS {fps_name}"
        gcode.run_script(f"M118 {message}")
        gcode.run_script(f"M114 {message}")
        gcode.run_script("PAUSE")
        self._register_monitor_spool_timer()
    
    # this needs to be a closure that returns a fucntion with just eventtime
    def _monitor_spool(self, fps_name):
        def _monitor_spool_inner(eventtime):
            fps_state = self.current_state.fps_state[fps_name]
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            
            if is_printing and \
                fps_state.name == "LOADED" and \
                fps_state.current_group is not None and \
                fps_state.current_spool_idx is not None and \
                not bool(self.oams[fps_state.current_oams].hub_hes_value[fps_state.current_spool_idx]):
                    self._register_pause_timer(eventtime, fps_name, PAUSE_DISTANCE)
                    return self.printer.get_reactor().NEVER
        
            return eventtime + 1.0
        return partial(_monitor_spool_inner, self)
    
    def _register_monitor_spool_timer(self, fps_name):
        fps_state = self.current_state.fps_state[fps_name]
        reactor = self.printer.get_reactor()
        fps_state.monitor_spool_timer = reactor.register_timer(self._monitor_spool(fps_name), reactor.NOW)

    def _initialize_oams(self):
        for (name, oam) in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def _initialize_filament_groups(self):
        for (name, group) in self.printer.lookup_objects(module="filament_group"):
            name = name.split()[-1]
            logging.info(f"OAMS: Adding group {name}")
            self.filament_groups[name] = group
    
    def determine_current_loaded_group(self, fps_name: str) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        fps = self.fpss[fps_name]
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")
        for group_name, group in self.filament_groups.items():
            for (oam, bay_index) in group.bays:
                if oam.is_bay_loaded(bay_index) and oam in fps.oams:
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
        for _, oam in self.oams.items():
            oam.clear_errors()
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
        fps_state.current_oams.set_oams_follower(enable, direction)
        fps_state.following = enable
        fps_state.direction = direction
        fps_state.encoder = fps_state.current_oams.encoder_clicks
        fps_state.current_spool_idx = fps_state.current_oams.current_spool
        return
    
    def group_fps_name(self, group_name):
        for c_group_name, c_group in self.filament_groups.items():
            if c_group_name == group_name:
                for fps_name, fps in self.fpss.items():
                        for fps_oam in fps.oams:
                            if fps_oam in c_group.oams:
                                return fps_name
        return None
    
    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps_name = gcmd.get('FPS')
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
        if fps_state.state_name == "LOADED":
            oams = self.oams[fps_state.current_oams]
            if oams is None:
                gcmd.respond_info(f"FPS {fps_name} has no OAMS loaded")
                return
            if oams.current_spool is not None:
                fps_state.state_name = "UNLOADING"
                fps_state.encoder = oams.encoder_clicks
                fps_state.since = self.reactor.monotonic()
                fps_state.current_oams = oams.name
                fps_state.current_spool_idx = oams.current_spool
            
                success, message = oams.unload_spool()
                
                if success:
                    fps_state.state_name = "UNLOADED"
                    fps_state.following = False
                    fps_state.direction = 0
                    fps_state.since = self.reactor.monotonic()
                    
                    fps_state.current_group = None
                    fps_state.current_spool_idx = None
                    return
                else:
                    gcmd.respond_info(message)
                    return
        gcmd.respond_info("No spool is loaded in any of the OAMS")
        self.current_group = None
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
        
        for (oam, bay_index) in self.filament_groups[group_name].bays:
            if oam.is_bay_ready(bay_index):
                fps_state.state_name = "LOADING"
                fps_state.encoder = oam.encoder_clicks
                fps_state.since = self.reactor.monotonic()
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                
                success, message = oam.load_spool(bay_index)
                
                if success:
                    fps_state.current_group = group_name
                    fps_state.current_oams = oam.name
                    fps_state.current_spool_idx = bay_index
                    
                    
                    fps_state.state_name = "LOADED"
                    fps_state.since = self.reactor.monotonic()
                    fps_state.current_oams = oam.name
                    fps_state.current_spool_idx = bay_index
                    fps_state.following = False
                    fps_state.direction = 1
                    
                    gcmd.respond_info(message)
                    self.current_group = group_name
                    return
                else:
                    gcmd.respond_info(message)
                    return
        gcmd.respond_info(f"No spool available for group {group_name}")
        return
        
    def _pause_printer_message(self, message):
        logging.info(f"OAMS: {message}")
        gcode = self.printer.lookup_object("gcode")
        message = f"Print has been paused: {message}"
        gcode.run_script(f"M118 {message}")
        gcode.run_script(f"M114 {message}")
        gcode.run_script("PAUSE")
        
    def _monitor_unload_speed_for_fps(self, fps_name):
        def _monitor_unload_speed(self, eventtime):
            #logging.info("OAMS: Monitoring unloading speed state: %s" % self.current_state.name)
            fps_state = self.current_state.fps_state[fps_name]
            oams = self.oams[fps_state.current_oams]
            if fps_state.state_name == "UNLOADING" and self.reactor.monotonic() - fps_state.since > MONITOR_ENCODER_UNLOADING_SPEED_AFTER:
                fps_state.encoder_samples.append(oams.encoder_clicks)
                if len(fps_state.encoder_samples) < ENCODER_SAMPLES:
                    return eventtime + 1.0
                encoder_diff = abs(fps_state.encoder_samples[-1] - fps_state.encoder_samples[0])
                logging.info("OAMS[%d] Unload Monitor: Encoder diff %d" %(oams.oams_idx, encoder_diff))
                if encoder_diff < MIN_ENCODER_DIFF:              
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                    self._pause_printer_message("Printer paused because the unloading speed of the moving filament was too low")
                    logging.info("after unload speed too low")
                    self.stop_monitors()
                    return self.printer.get_reactor().NEVER
            return eventtime + 1.0
        return partial(_monitor_unload_speed, self)
    
    def _monitor_load_speed_for_fps(self, fps_name):
        def _monitor_load_speed(self, eventtime):
            #logging.info("OAMS: Monitoring loading speed state: %s" % self.current_state.name)
            fps_state = self.current_state.fps_state[fps_name]
            oams = self.oams[fps_state.current_oams]
            if fps_state.state_name == "LOADING" and self.reactor.monotonic() - fps_state.since > MONITOR_ENCODER_LOADING_SPEED_AFTER:
                fps_state.encoder_samples.append(oams.encoder_clicks)
                if len(fps_state.encoder_samples) < ENCODER_SAMPLES:
                    return eventtime + 1.0
                encoder_diff = abs(fps_state.encoder_samples[-1] - fps_state.encoder_samples[0])
                logging.info("OAMS[%d] Load Monitor: Encoder diff %d" % (oams.oams_idx, encoder_diff))
                if encoder_diff < MIN_ENCODER_DIFF:
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                    self._pause_printer_message("Printer paused because the loading speed of the moving filament was too low")
                    self.stop_monitors()
                    return self.printer.get_reactor().NEVER
            return eventtime + 1.0
        return partial(_monitor_load_speed, self)
    
    def start_monitors(self):
        self.monitor_timers = []
        reactor = self.printer.get_reactor()
        for (fps_name, _) in self.current_state.fps_state.items():
            self.monitor_timers.append(reactor.register_timer(self._monitor_unload_speed(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_load_speed_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_spool(fps_name), reactor.NOW))
        logging.info("OAMS: Monitors started")
    
    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []
        for fps_state in self.current_state.fps_state.values():
            fps_state.monitor_spool_timer = None
            fps_state.monitor_pause_timer = None
            fps_state.monitor_load_next_spool_timer = None

def load_config(config):
    return OAMSManager(config)
