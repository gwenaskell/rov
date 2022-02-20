from dataclasses import dataclass


@dataclass
class WaterValue:
    flood_av: bool
    flood_ar: bool
    immerged: bool


class WaterSensor:
    def __init__(self) -> None:
        pass

    def get_state(self) -> WaterValue:
        return WaterValue(False, False, False)
