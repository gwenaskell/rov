from dataclasses import dataclass


@dataclass
class AccelValue:
    ax: float
    ay: float
    az: float
    wx: float
    wy: float
    wz: float


class IMU:
    AccelValue = AccelValue

    def __init__(self) -> None:
        pass

    def get_state(self) -> AccelValue:
        return AccelValue(0, 0, 0, 0, 0, 0)
