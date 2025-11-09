# HDC1080 Sensor Loader
#
# Simple module to register the HDC1080 temperature sensor without
# modifying Klipper's temperature_sensors.cfg file.
#
# Add this to your printer.cfg to enable HDC1080 sensor support:
# [hdc1080_loader]
#
# Then you can use HDC1080 sensors like:
# [temperature_sensor my_hdc1080]
# sensor_type: HDC1080
# i2c_address: 64
# i2c_bus: i2c0a

import logging

class HDC1080Loader:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.logger = logging.getLogger("hdc1080_loader")

        # Register HDC1080 sensor factory with heaters
        try:
            from . import hdc1080
            pheater = self.printer.lookup_object("heaters")
            pheater.add_sensor_factory("HDC1080", hdc1080.HDC1080)
            self.logger.info("HDC1080 temperature sensor registered successfully")
        except Exception as e:
            self.logger.error("Failed to register HDC1080 sensor: %s", e)
            raise config.error(f"HDC1080 sensor registration failed: {e}")

def load_config(config):
    return HDC1080Loader(config)