# Filament Group
#
# Copyright (C) 2023-2025 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


import logging

class FilamentGroup:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.group_name = config.get_name().split()[-1]
        self.bays = []
        self.oams = []
        self._initialize_bays(config)

    def _initialize_bays(self, config):
        bays = config.get("group").split(",")
        for bay_assignment in bays:
            oams_name, bay_index = bay_assignment.strip('"').split("-")
            oam = self.printer.lookup_object("oams " + oams_name.strip())
            self.add_bay(oam, int(bay_index))

    def add_bay(self, oam, bay_index):
        bay_id = (oam, bay_index)
        self.bays.append(bay_id)
        if oam not in self.oams:
            self.oams.append(oam)

    def is_any_spool_loaded(self):
        for oam in self.oams:
            if oam.query_spool_status() is not None:
                return True
        return False
    
def load_config_prefix(config):
    return FilamentGroup(config)

def load_config(config):
    return FilamentGroup(config)
