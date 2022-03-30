from multiprocessing import Manager, Process
import time
from typing import Tuple
import typing
from src.components.classes import Commands, Status, ThrusterState, ThrusterVector

from src.components.tracker import SetpointsTracker

from math import sin, cos, atan, pi


class Pilot:
    """Pilot handles conversion of the input commands expressed in terms of thrust and moment on (x, y, z)
    into instructions for the motors.

    The thrusters are updated progressively in a separate process using the SetpointsTracker, by periodically 
    checking the desired state and incrementally moving the thrusters towards this state.
    """

    def __init__(self) -> None:
        self.tracker = SetpointsTracker()
        self.left_reversed = False
        self.right_reversed = False

        self.nb_stepper_steps = self.tracker.get_nb_steps()

        self.eps = 0.15  # minimum vertical thrust to maintain the ROV underwater
        self.reverse_efficiency = 0.9

        # angle threshold to switch to forward thrust
        self.forward_threshold = 40.0/180*pi + pi/2
        # angle threshold to switch to reverse thrust
        self.reverse_threshold = 50.0/180*pi + pi/2

        self.stopped = True

        self.tracker_process = typing.cast(Process, None)

        self.states_proxy = typing.cast(dict, None)

    def start(self):
        if not self.stopped:
            raise RuntimeError("pilot already running")
        self.stopped = False

        manager = Manager()
        self.states_proxy = manager.dict({
            "status": Status.RUNNING,
            "targets": {
                "tail_thrust": 0,
                "left_state": None,
                "right_state": None,
                "tm": round(time.time()*1000),
            }
        })
        self.apply_setpoints(Commands(0.0, 0.0, 0.0, 0.0, 0.0, 0))

        self.tracker_process = Process(
            target=self.tracker.run, args=(self.states_proxy,))
        self.tracker_process.start()

    def angle_to_step_index(self, angle: float) -> int:
        return round(angle*self.nb_stepper_steps/(2*pi))

    def stop(self):
        if self.stopped:
            raise RuntimeError("pilot already stopped")

        self.stopped = True
        self.states_proxy["status"] = Status.STOPPED
        self.tracker_process.join()

    def stop_engines(self):
        self.states_proxy["status"] = Status.PAUSED

    def apply_setpoints(self, commands: Commands):
        """computes the target thrusters states corresponding to these commands."""
        # TODO: apply proper coefs on cy depending on rov geometry and center of gravity
        # TODO: project self.eps on earth vertical axis

        left_thrust = ThrusterVector(
            commands.fx - commands.cz,
            commands.fz + commands.cx - commands.cy/2)
        right_thrust = ThrusterVector(
            commands.fx + commands.cz,
            commands.fz - commands.cx - commands.cy/2)

        tail_thrust = commands.fz + commands.cy - self.eps

        left_norm = (left_thrust.f_x**2+left_thrust.f_z**2)**0.5

        if left_norm > 1:
            left_thrust.f_x /= left_norm
            left_thrust.f_z /= left_norm
            right_thrust.f_x /= left_norm
            right_thrust.f_z /= left_norm
            tail_thrust /= left_norm

        right_norm = (right_thrust.f_x**2+right_thrust.f_z**2)**0.5
        if right_norm > 1:
            left_thrust.f_x /= right_norm
            left_thrust.f_z /= right_norm
            right_thrust.f_x /= right_norm
            right_thrust.f_z /= right_norm
            tail_thrust /= right_norm

        tail_norm = abs(tail_thrust)
        if tail_norm > 1:
            left_thrust.f_x /= tail_norm
            left_thrust.f_z /= tail_norm
            right_thrust.f_x /= tail_norm
            right_thrust.f_z /= tail_norm
            tail_thrust /= tail_norm

        left_target_state, self.left_reversed = self.compute_desired_state(
            left_thrust, self.left_reversed)
        right_target_state, self.right_reversed = self.compute_desired_state(
            right_thrust, self.right_reversed)

        # print(left_target_state.pos, left_target_state.tau,
        #       right_target_state.pos, right_target_state.tau)

        self.states_proxy["targets"] = {
            "tail_thrust": tail_thrust,
            "left_state": left_target_state,
            "right_state": right_target_state,
            "tm": commands.tm_ms,
        }

    def compute_desired_state(self, vector: ThrusterVector, reverse_spin: bool) -> Tuple[ThrusterState, bool]:
        """mathematical conversion of the thrust vector into angle and thrust"""

        # Get axial commands from c_x
        tau = (self.eps**2 + (1.0-self.eps**2)*vector.f_x**2)**0.5
        if self.eps > 0:
            phi = atan(vector.f_x/self.eps)
        else:
            phi = 0

        # Get full commands from axial commands and c_z
        div = cos(phi) * tau - vector.f_z
        tau = (tau**2 - 2*tau*vector.f_z*cos(phi)+vector.f_z**2)**0.5
        if div != 0:
            if div > 0:  # phi goes to the upper part of the area
                phi = atan(sin(phi) * tau / div)
            else:
                if phi > 0:
                    phi = atan(sin(phi) * tau / div) + pi
                else:
                    phi = atan(sin(phi) * tau / div) - pi
        else:
            if phi > 0:
                phi = pi / 2
            elif phi < 0:
                phi = - pi / 2

        if reverse_spin:
            # should we use forward?
            if abs(phi) < self.forward_threshold:
                reverse_spin = False
                # state is already designed for a forward thrust, no change to make
            else:
                # maintain reversed
                phi, tau = self._reverse(phi, tau)

        else:
            # should we use reverse?
            if abs(phi) > self.reverse_threshold:
                phi, tau = self._reverse(phi, tau)
                reverse_spin = True

                # tau becomes negative

        tau = bound(tau)

        return ThrusterState(tau=tau, pos=self.angle_to_step_index(phi), thrust_coef=1.0), reverse_spin

    def _reverse(self, phi, tau):
        """returns the opposite state of the thruster that give the exact same thrust.

        reverse_efficiency estimates the coef of efficiency of reverse thrust compared to 
        forward thrust. We increase reverse thrust to compensate this lower efficiency.
        """
        if phi > 0:
            phi = phi - pi
        else:
            phi = phi + pi
        # boost, since reverse rotation is less efficient
        tau = -tau/self.reverse_efficiency
        return phi, tau


def bound(x):
    if x > 1:
        return 1.0
    if x < -1:
        return -1.0
    return x
