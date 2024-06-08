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
        
        # TODO
        corrected_commands = Commands(
            fx=commands.fx,
            fz=commands.fz,
            cx=commands.cx,
            cy=commands.cy,
            cz=commands.cz,
            tm_ms=commands.tm_ms
        )
        
        return corrected_commands

# import time
# import math

# def fexp(i):
#     return 1/(1+math.exp(-i))

# def ftan(i):
#     return math.tanh(i) # plus rapide
