from dataclasses import dataclass
from typing import Tuple
from ..drivers.accelero import Accelerometer
from ..drivers.compass import Compass


@dataclass
class RovState:
    # speed
    vx: float
    vy: float
    vz: float
    # acceleration
    ax: float
    ay: float
    az: float
    # angle
    thx: float
    thy: float
    thz: float
    # angular speed
    wx: float
    wy: float
    wz: float


class IMU:
    def __init__(self) -> None:
        self.accelero = Accelerometer()
        self.magnetometer = Compass()

    def get_current_state(self) -> RovState:
        return RovState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
