from multiprocessing import Manager, Process
import time
from typing import Tuple
import typing
from src.components.classes import Commands, NamespaceProxy, Status, ThrusterState, ThrusterVector
from src.components.classes import RovState

from src.components.tracker import SetpointsTracker

from src.components.geometry import get_earth_axis_coordinates

from math import atan2, pi


class Pilot:
    """Pilot handles conversion of the input commands expressed in terms of thrust and moment on (x, y, z)
    into instructions for the motors.

    The thrusters are updated progressively in a separate process using the SetpointsTracker, by periodically 
    checking the desired state and incrementally moving the thrusters towards this state.
    
    The data bridge between the two processes is materialized by the field states_proxy
    """

    def __init__(self) -> None:
        self.tracker = SetpointsTracker()
        self.left_reversed = False
        self.right_reversed = False

        self.nb_stepper_steps = self.tracker.get_nb_steps()

        self.eps = 0.2  # minimum vertical thrust to maintain the ROV underwater
        self.reverse_efficiency = 0.9 # efficiency of propellers when rotated backward

        # angle threshold to switch to forward thrust
        self.forward_threshold = 40.0/180*pi + pi/2
        # angle threshold to switch to reverse thrust (hysteresis)
        self.reverse_threshold = 50.0/180*pi + pi/2

        self.status: Status = Status.STOPPED

        self.tracker_process = typing.cast(Process, None)

        # data channel between this process and the tracker
        
        
        manager = Manager()
        self.states_proxy = typing.cast(NamespaceProxy, manager.dict({
            "status": Status.STOPPED,
            "targets": {
                "tail_thrust": 0,
                "left_state": ThrusterState(pos=0, tau=0),
                "right_state": ThrusterState(pos=0, tau=0),
            }
        }))

    def angle_to_step_index(self, angle: float) -> int:
        return round(angle*self.nb_stepper_steps/(2*pi))

    def _start_tracker_process(self):
        self.tracker_process = Process(
            target=self.tracker.run, args=(self.states_proxy,))
        self.tracker_process.start()

    def switch_engines(self, status: Status):
        if status == self.status:
            print("warning: no change implied by engines status switch")
            return

        must_relaunch = False

        if self.status == Status.STOPPED:
            must_relaunch = True
            if self.tracker_process and self.tracker_process.is_alive():
                self.tracker_process.join()
                print("stopped tracker")

        self.status = status
        self.states_proxy["status"] = status

        if must_relaunch:
            print("launching tracker")
            self._start_tracker_process()

    def stop(self):
        self.status = Status.STOPPED
        self.states_proxy["status"] = Status.STOPPED
        if self.tracker_process and self.tracker_process.is_alive():
            self.tracker_process.join()
            print("pilot: stopped")

    def apply_setpoints(self, commands: Commands, state: RovState, bridle=False):
        """computes the target thrusters states corresponding to these commands.

        If the engines were stopped, they are resumed to running state
        """

        if self.status == Status.STOPPED:
            raise RuntimeError("cannot apply setpoints: thrusters are stopped")
        
        max_thrust = 1
        
        if bridle:
            max_thrust = 0.5
            commands.fx *= 0.5
            commands.fz *= 0.5
            commands.cx *= 0.5
            commands.cy *= 0.5
            commands.cz *= 0.5
            
            # do not bridle self.eps! (except if it makes exceed max_thrust)
        
        if commands.surface:
            left_thrust = ThrusterVector(commands.fx - commands.cz, 0)
            right_thrust = ThrusterVector(commands.fx + commands.cz, 0)
            tail_thrust = 0
        else:
            # zero z axis to erase the epsilon term if we don't want to maintain the rov underwater
            z_axis = get_earth_axis_coordinates(state.q)

            # TODO: apply proper coefs on cy depending on rov geometry and center of gravity
            left_thrust = ThrusterVector(
                commands.fx - commands.cz - self.eps*z_axis.x,
                commands.fz + commands.cx - commands.cy/2 - self.eps*z_axis.z)
            right_thrust = ThrusterVector(
                commands.fx + commands.cz - self.eps*z_axis.x,
                commands.fz - commands.cx - commands.cy/2 - self.eps*z_axis.z)

            # tail motor is oriented towards the surface. A positive thrust pushes the ROV downwards.
            tail_thrust = -(commands.fz + commands.cy - self.eps*z_axis.z)

        left_norm: float = (left_thrust.f_x**2+left_thrust.f_z**2)**0.5

        if left_norm > max_thrust:
            fact = left_norm / max_thrust
            left_thrust.f_x /= fact
            left_thrust.f_z /= fact
            right_thrust.f_x /= fact
            right_thrust.f_z /= fact
            tail_thrust /= fact

        right_norm: float = (right_thrust.f_x**2+right_thrust.f_z**2)**0.5

        if right_norm > max_thrust:
            fact = right_norm / max_thrust
            left_thrust.f_x /= fact
            left_thrust.f_z /= fact
            right_thrust.f_x /= fact
            right_thrust.f_z /= fact
            tail_thrust /= fact

        tail_norm = abs(tail_thrust)

        if tail_norm > max_thrust:
            fact = tail_norm / max_thrust
            left_thrust.f_x /= fact
            left_thrust.f_z /= fact
            right_thrust.f_x /= fact
            right_thrust.f_z /= fact
            tail_thrust /= fact

        left_target_state, self.left_reversed = self.compute_desired_state(
            left_thrust, self.left_reversed, commands.surface)
        right_target_state, self.right_reversed = self.compute_desired_state(
            right_thrust, self.right_reversed, commands.surface)

        self.states_proxy["targets"] = {
            "tail_thrust": tail_thrust,
            "left_state": left_target_state,
            "right_state": right_target_state,
        }

        if self.status != Status.RUNNING:
            self.status = Status.RUNNING
            self.states_proxy["status"] = Status.RUNNING

    def compute_desired_state(self, vector: ThrusterVector, reversed_spin: bool, surface: bool) -> Tuple[ThrusterState, bool]:
        """mathematical conversion of the thrust vector into angle and thrust"""
        
        if surface:
            # we slightly orientate thurst downwards to maintain the ROV at the surface during movement, thus the *1.1
            return ThrusterState(tau=vector.f_x, pos=self.angle_to_step_index(-pi/2*1.1)), vector.f_x < 0
        
        tau = (vector.f_x**2 + vector.f_z**2)**0.5
        
        
        phi = atan2(vector.f_x, -vector.f_z)

        if reversed_spin:
            # should we use forward?
            if abs(phi) < self.forward_threshold:
                reversed_spin = False
                # state is already computed for a forward thrust, no change to make
            else:
                # maintain reversed
                phi, tau = self._reverse(phi, tau)

        else:
            # should we use reverse?
            if abs(phi) > self.reverse_threshold:
                phi, tau = self._reverse(phi, tau)
                reversed_spin = True

                # tau becomes negative

        tau = bound(tau)
        

        return ThrusterState(tau=tau, pos=self.angle_to_step_index(phi), thrust_coef=1.0), reversed_spin

    def _reverse(self, phi, tau):
        """returns the opposite state of the thruster that gives the exact same thrust.

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
