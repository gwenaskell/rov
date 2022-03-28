import os
from typing import Literal, Union
import pigpio
import time

from src.drivers.mappings import ThrusterLeft, ThrusterRight, ThrusterTail


class Thruster:
    def __init__(self, id: Union[Literal["left"], Literal["right"], Literal["tail"]]) -> None:
        self.max_value = 2000
        self.min_value = 700
        self.zero_value = 1000

        if id == "right":
            self.driver = ThrusterRight
        elif id == "left":
            self.driver = ThrusterLeft
        else:
            self.driver = ThrusterTail

        self.armed = False
        self.perc = 0
        self.id = id

    def set_pwm(self, perc: int):
        if not self.armed:
            raise RuntimeError("thruster not armed")
        if self.perc == perc:
            return
        self.ratio = perc
        speed = perc * (self.max_value-self.zero_value) + \
            self.zero_value  # TODO handle negatives
        self.driver.set_pulsewidth(int(speed))

    def arm_thruster(self):
        self.driver.setup()
        self.driver.set_pulsewidth(0)
        time.sleep(1)
        self.driver.set_pulsewidth(self.max_value)
        time.sleep(1)
        self.driver.set_pulsewidth(self.min_value)
        time.sleep(1)
        self.driver.set_pulsewidth(self.zero_value)
        self.armed = True

    def stop(self):
        self.driver.stop()
        self.armed = False
