from enum import Enum
from src.drivers.mappings import A2C

import digitalio

import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn


class AnalogChans(Enum):
    VBATT_0 = MCP.P0
    VBATT_1 = MCP.P1
    EXT_PRESSURE = MCP.P2
    CURRENT_MOT = MCP.P3
    SURFACE_UP = MCP.P4
    SURFACE_LOW = MCP.P5
    WATER_AR = MCP.P6
    WATER_AV = MCP.P7


class AnalogInput:
    def __init__(self) -> None:
        self.driver = A2C

        # create the cs (chip select)
        cs = digitalio.DigitalInOut(self.driver.ce.pin.bcm_pin)

        # create the mcp object
        self.mcp = MCP.MCP3008(self.driver.bus, cs)

        self.chans = {ch: AnalogIn(self.mcp, ch.value) for ch in AnalogChans}

    def get_perc_val(self, chan: AnalogChans) -> int:
        return int(self.chans[chan].value*100/65535)
