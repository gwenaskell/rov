from .imu import RovState
from .classes import Commands


class PID:
    def __init__(self,
                 Kp=1.0,
                 Ki=0.0,
                 Kd=0.0) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.last_tm = 0

    def get_correction(self, state: RovState, commands: Commands) -> Commands:
        return commands
