
from math import pi
from typing import Literal, Union
from .plot import plotter


class StepperMotor:
    def __init__(self, id: Union[Literal["left"], Literal["right"]]) -> None:
        self.id = id

        self.step_position = 0

        self.nb_steps = 200

        self.angle_by_step = 2.0*pi / self.nb_steps

        # one full rotation will take two seconds.
        self.min_step_time = 2.0/200  # to be evaluated

        self.spin = True
        self.last_transition = False

    def angle_to_step_index(self, angle: float) -> int:
        return round(angle*self.nb_steps/(2*pi))

    def get_current_angle(self) -> float:
        return self.step_position * (2.0*pi)/self.nb_steps

    def bring_sally_up(self, clockwise: bool):
        if self.last_transition == True:
            raise Exception("stepper state is already UP")

        self.last_transition = True

        if clockwise != self.spin:
            self.spin = clockwise

        if clockwise:
            self.step_position += 1
        else:
            self.step_position -= 1

        angle = self.get_current_angle()
        # print(self.id, angle)

        if self.id == "left":
            plotter.phi_l = angle
        else:
            plotter.phi_r = angle

    def bring_sally_down(self):
        if self.last_transition == False:
            raise Exception("stepper state is already DOWN")

        self.last_transition = False

    def stop(self):
        pass


class StepperMotor:
    def __init__(self, id: Union[Literal["left"], Literal["right"]]) -> None:
        self.step_position = 0

        self.nb_steps = 200

        self.angle_by_step = 2.0*pi / self.nb_steps

        # one full rotation will take one second.
        self.min_step_time = 1.0/200  # to be evaluated

        self.spin = False  # rotation spin (clockwise / anticlockwise)
        self.last_transition = False

    def angle_to_step_index(self, angle: float) -> int:
        return round(angle*self.nb_steps/(2*pi))

    def get_current_angle(self) -> float:
        return self.step_position * (2.0*pi)/self.nb_steps

    def setup(self):
        pass

    def bring_sally_up(self, clockwise: bool):
        if self.last_transition == True:
            raise RuntimeError("stepper state is already UP")

        self.last_transition = True

        if clockwise != self.spin:
            self.spin = clockwise

        if clockwise:
            self.step_position += 1
        else:
            self.step_position -= 1

    def bring_sally_down(self):
        if self.last_transition == False:
            raise RuntimeError("stepper state is already DOWN")

        self.last_transition = False

    def stop(self):
        pass
