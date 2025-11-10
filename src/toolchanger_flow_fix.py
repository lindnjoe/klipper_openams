# Monkey patch to fix flow rate and filament tracking with toolchangers
#
# This module fixes two issues that occur with toolchangers:
# 1. Volumetric flow (mm³/s) showing 0.0 or not tracking correctly
# 2. AFC lane changes causing negative filament tracking
#
# To use, add this to your printer.cfg:
#   [toolchanger_flow_fix]
#
# Copyright (C) 2025  Joe Lindner <lindnjoe@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging

class ToolchangerFlowFix:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = None
        self.printer.register_event_handler("klippy:connect",
                                           self._handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                           self._handle_ready)

    def _handle_connect(self):
        self.gcode = self.printer.lookup_object('gcode')

        # Patch motion_report to track active extruder velocity
        motion_report = self.printer.lookup_object('motion_report', None)
        if motion_report is not None:
            self._patch_motion_report(motion_report)
        else:
            logging.warning("toolchanger_flow_fix: motion_report not found!")

        # Try to patch AFC if it exists
        afc = self.printer.lookup_object('AFC', None)
        if afc is not None:
            self._patch_afc(afc)

        logging.info("toolchanger_flow_fix: Patches applied successfully")

    def _patch_motion_report(self, motion_report):
        """Patch PrinterMotionReport to track active extruder velocity"""
        original_get_status = motion_report.get_status

        def patched_get_status(eventtime):
            # Check timing BEFORE calling original (which updates next_status_time)
            should_update = eventtime >= motion_report.next_status_time
            status = original_get_status(eventtime)

            # Only update extruder velocity when status was actually refreshed
            if should_update and motion_report.dtrapqs:
                toolhead = motion_report.printer.lookup_object('toolhead')
                active_extruder = toolhead.get_extruder()

                if active_extruder is not None:
                    extruder_name = active_extruder.get_name()
                    ehandler = motion_report.dtrapqs.get(extruder_name)

                    if ehandler is not None:
                        mcu = motion_report.printer.lookup_object('mcu')
                        print_time = mcu.estimated_print_time(eventtime)
                        pos, velocity = ehandler.get_trapq_position(print_time)

                        if pos is not None and velocity is not None:
                            status = dict(status)
                            status['live_extruder_velocity'] = velocity
                            motion_report.last_status = status

            return status

        motion_report.get_status = patched_get_status

    def _patch_afc(self, afc):
        """Patch AFC save_pos/restore_pos to skip toolchange moves in filament tracking"""
        original_save_pos = afc.save_pos
        original_restore_pos = afc.restore_pos
        printer = self.printer
        saved_state = [None, None]  # [saved_e_pos, saved_last_epos]
        afc_in_progress = [False]  # Flag to block print_stats updates during AFC

        # Patch print_stats to skip updates during AFC operations
        print_stats = printer.lookup_object('print_stats', None)
        if print_stats:
            original_update = print_stats._update_filament_usage

            def patched_update(eventtime):
                if afc_in_progress[0]:
                    return  # Skip update during AFC operations
                original_update(eventtime)

            print_stats._update_filament_usage = patched_update
            logging.info("Patched print_stats to skip updates during AFC operations")

        def patched_save_pos():
            gcode_move = printer.lookup_object('gcode_move')
            print_stats = printer.lookup_object('print_stats', None)
            reactor = printer.get_reactor()
            eventtime = reactor.monotonic()
            gc_status = gcode_move.get_status(eventtime)

            saved_state[0] = gc_status['position'].e
            if print_stats:
                saved_state[1] = print_stats.last_epos
                logging.info("AFC save_pos: E=%.3f, last_epos=%.3f, filament_used=%.3f"
                           % (saved_state[0], saved_state[1], print_stats.filament_used))
            else:
                logging.info("AFC save_pos: E=%.3f (no print_stats)" % saved_state[0])

            afc_in_progress[0] = True  # Block print_stats updates
            original_save_pos()

        def patched_restore_pos(move_z_first=True):
            try:
                logging.info("AFC restore_pos: CALLED with move_z_first=%s" % move_z_first)
                original_restore_pos(move_z_first)
                logging.info("AFC restore_pos: original_restore_pos completed")
            except Exception as e:
                logging.error("AFC restore_pos: original_restore_pos FAILED: %s" % str(e))
                afc_in_progress[0] = False  # Re-enable on error
                raise

            try:
                print_stats = printer.lookup_object('print_stats', None)
                gcode_move = printer.lookup_object('gcode_move')
                if print_stats and saved_state[0] is not None and saved_state[1] is not None:
                    reactor = printer.get_reactor()
                    eventtime = reactor.monotonic()
                    gc_status = gcode_move.get_status(eventtime)
                    current_e = gc_status['position'].e
                    e_delta = current_e - saved_state[0]

                    # Restore last_epos to skip toolchange moves
                    old_last_epos = print_stats.last_epos
                    print_stats.last_epos = saved_state[1] + e_delta

                    logging.info("AFC restore_pos: E changed %.3f->%.3f (delta=%.3f), "
                               "last_epos %.3f->%.3f, filament_used=%.3f"
                               % (saved_state[0], current_e, e_delta,
                                  old_last_epos, print_stats.last_epos,
                                  print_stats.filament_used))

                    saved_state[0] = None
                    saved_state[1] = None
                else:
                    if saved_state[0] is None:
                        logging.warning("AFC restore_pos called but no saved state!")
                    else:
                        logging.info("AFC restore_pos: no print_stats")

                afc_in_progress[0] = False  # Re-enable print_stats updates
            except Exception as e:
                logging.error("AFC restore_pos: patch logic FAILED: %s" % str(e))
                import traceback
                logging.error(traceback.format_exc())
                afc_in_progress[0] = False  # Re-enable on error

        afc.save_pos = patched_save_pos
        afc.restore_pos = patched_restore_pos

    def _handle_ready(self):
        self.gcode.register_command('FLOW_FIX_STATUS',
                                   self.cmd_FLOW_FIX_STATUS,
                                   desc="Report toolchanger flow fix status")

    cmd_FLOW_FIX_STATUS_help = "Report current flow fix status and diagnostics"
    def cmd_FLOW_FIX_STATUS(self, gcmd):
        gcode_move = self.printer.lookup_object('gcode_move')
        motion_report = self.printer.lookup_object('motion_report', None)
        toolhead = self.printer.lookup_object('toolhead')
        afc = self.printer.lookup_object('AFC', None)
        print_stats = self.printer.lookup_object('print_stats', None)

        # Get current state
        active_extruder = toolhead.get_extruder()
        extruder_name = active_extruder.get_name() if active_extruder else "None"
        extrude_factor = gcode_move.extrude_factor

        msg = "Flow Fix Status:\n"
        msg += "  Active Extruder: %s\n" % extruder_name
        msg += "  Extrude Factor: %.3f (%.1f%%)\n" % (extrude_factor, extrude_factor * 100)

        if motion_report:
            status = motion_report.get_status(self.printer.get_reactor().monotonic())
            msg += "  Live Extruder Velocity: %.3f mm/s\n" % status.get('live_extruder_velocity', 0)
            msg += "  Live XYZ Velocity: %.3f mm/s\n" % status.get('live_velocity', 0)

            # Check if trapq exists for active extruder
            if active_extruder:
                ehandler = motion_report.dtrapqs.get(extruder_name)
                msg += "  Trapq for %s: %s\n" % (extruder_name, "Found" if ehandler else "NOT FOUND")
                msg += "  Available trapqs: %s\n" % ", ".join(motion_report.dtrapqs.keys())

        if print_stats:
            stats = print_stats.get_status(self.printer.get_reactor().monotonic())
            msg += "\nPrint Stats:\n"
            msg += "  State: %s\n" % stats.get('state', 'unknown')
            msg += "  Filament used: %.3f mm\n" % stats.get('filament_used', 0)
            msg += "  Total duration: %.1f s\n" % stats.get('total_duration', 0)
            msg += "  Print duration: %.1f s\n" % stats.get('print_duration', 0)
            msg += "  Last E position: %.3f\n" % print_stats.last_epos

        if afc:
            msg += "\nAFC Info:\n"
            msg += "  Current lane: %s\n" % (afc.current if hasattr(afc, 'current') else "Unknown")
            msg += "  In toolchange: %s\n" % (afc.in_toolchange if hasattr(afc, 'in_toolchange') else "Unknown")
            msg += "  Position saved: %s\n" % (afc.position_saved if hasattr(afc, 'position_saved') else "Unknown")

        gcmd.respond_info(msg)

def load_config(config):
    return ToolchangerFlowFix(config)
