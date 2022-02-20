from dataclasses import dataclass


@dataclass
class ThermoValue:
    t_ext: float
    t_esc: float


class Thermometer:
    def __init__(self) -> None:
        pass

    def get_state(self) -> ThermoValue:
        return ThermoValue(0, 0)
