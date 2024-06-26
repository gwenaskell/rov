
from typing import Literal
from .plot import plotter


class Thruster:
    def __init__(self, loc: Literal["left", "right", "tail"]) -> None:
        self.armed = False
        self.id = loc
        self.ratio = 0

    def set_pwm(self, ratio: int):
        if not self.armed:
            raise Exception("thruster not armed")
        if self.ratio == ratio:
            return
        self.ratio = ratio
        if self.id == "left":
            plotter.tau_l = ratio
        elif self.id == "right":
            plotter.tau_r = ratio
        else:
            plotter.tau_t = ratio

    def arm_thruster(self):
        print("thruster armed")
        self.armed = True

    def stop(self):
        self.armed = False
