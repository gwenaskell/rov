from dataclasses import dataclass


@dataclass
class MultimeterValue:
    v_batt: float
    v_rasp: float
    i_mot: float


class Multimeter:
    def __init__(self) -> None:
        pass

    def get_values(self) -> MultimeterValue:
        return MultimeterValue(0, 0, 0)
