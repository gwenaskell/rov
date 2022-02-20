from dataclasses import dataclass


@dataclass
class PressureValue:
    p_int: float
    p_ext: float


class PressureSensor:
    def __init__(self) -> None:
        pass

    def get_pressure(self) -> PressureValue:
        return PressureValue(0, 0)
