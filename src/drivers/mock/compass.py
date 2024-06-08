from dataclasses import dataclass


@dataclass
class CompassValue:
    bx: float
    by: float
    bz: float


class Compass:

    def __init__(self, **kwargs):
        pass

    def soft_reset(self):
        pass

    def __set_mode(self, mode, odr, sens, osr):
        pass

    def __read_raw_data(self, reg_address):
        return 0

    def get_state(self) -> CompassValue:
        return CompassValue(0, 0, 0)

    def get_bearing(self) -> CompassValue:
        return CompassValue(0, 0, 0)

    def read_temp(self):
        return 0

    def set_declination(self, value):
        pass
