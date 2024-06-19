from math import pi
from typing import Literal, Union

from src.drivers.mappings import StepperLeft, StepperRight
try:
    import RPi.GPIO as GPIO
except:
    pass

# https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/Documentation/Nema11A4988.md

NB_STEPS = 200

stepper_resolutions = {  # informative
    'Full': (0, 0, 0),
    'Half': (1, 0, 0),
    '1/4': (0, 1, 0),
    '1/8': (1, 1, 0),
    '1/16': (1, 1, 1)
}


class StepperMotor:
    def __init__(self, loc: Union[Literal["left"], Literal["right"]]) -> None:
        self.step_position = 0

        self.nb_steps = NB_STEPS

        self.angle_by_step = 2.0*pi / self.nb_steps

        # one full rotation will take one second.
        self.min_step_time = 1.0/200  # to be evaluated

        self.spin = False  # rotation spin (clockwise / anticlockwise)
        self.last_transition = False

        if loc == "left":
            self.driver = StepperLeft
        else:
            self.driver = StepperRight

    def angle_to_step_index(self, angle: float) -> int:
        return round(angle*self.nb_steps/(2*pi))

    def get_current_angle(self) -> float:
        return self.step_position * (2.0*pi)/self.nb_steps

    def setup(self):
        self.driver.direction.setup()
        self.driver.step.setup()

    def bring_sally_up(self, clockwise: bool):
        if self.last_transition == True:
            raise RuntimeError("stepper state is already UP")

        self.last_transition = True

        if clockwise != self.spin:
            self.spin = clockwise
            self.driver.direction.write(self.spin)

        self.driver.step.write(True)

        if clockwise:
            self.step_position += 1
        else:
            self.step_position -= 1

    def bring_sally_down(self):
        if self.last_transition == False:
            raise RuntimeError("stepper state is already DOWN")

        self.last_transition = False

        self.driver.step.write(False)

    def stop(self):
        self.driver.step.write(False)
        self.driver.direction.write(False)
