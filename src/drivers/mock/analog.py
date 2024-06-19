from enum import Enum


class AnalogChans(Enum):
    VBATT_0 = 0
    VBATT_1 = 1
    EXT_PRESSURE = 2
    CURRENT_MOT = 3
    SURFACE_UP = 4
    SURFACE_LOW = 5
    WATER_AR = 6
    WATER_AV = 7


class AnalogReader:
    def __init__(self) -> None:
        pass

    def get_perc_val(self, chan: AnalogChans) -> int:
        return {
            AnalogChans.VBATT_0: 90,
            AnalogChans.VBATT_1: 90,
            AnalogChans.EXT_PRESSURE: 50,
            AnalogChans.CURRENT_MOT: 10,
            AnalogChans.SURFACE_UP: 100,
            AnalogChans.SURFACE_LOW: 100,
            AnalogChans.WATER_AR: 0,
            AnalogChans.WATER_AV: 0,
        }[chan]
