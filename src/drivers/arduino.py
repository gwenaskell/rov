from enum import Enum
from src.drivers.mappings import A2C

import digitalio

import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

from adafruit_bus_device.spi_device import SPIDevice


class AnalogInput:
    def __init__(self) -> None:
        self.driver = A2C

        # create the cs (chip select)
        cs = digitalio.DigitalInOut(self.driver.ce.pin.bcm_pin)

        self.device = SPIDevice(self.driver.bus, cs)
