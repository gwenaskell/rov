from .imu import RovState
from .pilot import Commands


class PID:
    def __init__(self,
                 Kp=1.0,
                 Ki=0.0,
                 Kd=0.0) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def get_correction(self, delta_s: float, state: RovState, inputs) -> Commands:
        return Commands(0, 0, 0, 0, 0, 0)
