from multiprocessing import Manager, Process
from dataclasses import dataclass
import math
from threading import Thread
import time
from typing import Optional, Tuple
import typing

from .utils.countdown import CountDownLatch
from ..drivers.thruster import Thruster
from ..drivers.stepper import StepperMotor
from math import sin, cos, atan, pi
from src.drivers.mock.plot import plotter


@dataclass
class Commands:
    fx: float  # forward thrust
    fz: float  # vertical thrust
    cx: float  # roll
    cy: float  # pitch
    cz: float  # yaw
    tm: int


@dataclass
class ThrusterVector:  # thrust vector of a front (left/right) thruster
    f_x: float  # thrust on x axis
    f_z: float  # thrust on z axis


@dataclass
class ThrusterState:  # state of a front (left/right) thruster
    tau: float  # engine thrust
    pos: int  # stepper position
    # usually 1 or 0. Used when thrust of all thrusters must be disabled or reduced during a rotation
    thrust_coef: float


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
            "stopped": False,
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
        self.states_proxy["stopped"] = True
        self.tracker_process.join()

    def apply_setpoints(self, commands: Commands):
        """computes the target thrusters states corresponding to these commands."""
        # TODO: apply proper coefs on cy depending on rov geometry and center of gravity
        # TODO: project self.eps on earth vertical axis

        left_thrust = ThrusterVector(
            bound(commands.fx - commands.cz),
            bound(commands.fz - commands.cx + commands.cy/2))
        right_thrust = ThrusterVector(
            bound(commands.fx + commands.cz),
            bound(commands.fz + commands.cx + commands.cy/2))

        left_target_state, self.left_reversed = self.compute_desired_state(
            left_thrust, self.left_reversed)
        right_target_state, self.right_reversed = self.compute_desired_state(
            right_thrust, self.right_reversed)

        # print(left_target_state.pos, left_target_state.tau,
        #       right_target_state.pos, right_target_state.tau)

        self.states_proxy["targets"] = {
            "tail_thrust": bound(commands.fz - commands.cy - self.eps),
            "left_state": left_target_state,
            "right_state": right_target_state,
            "tm": commands.tm,
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


class SetpointsTracker:
    def __init__(self) -> None:
        self.thruster_left = ThrusterController(
            Thruster(id="left"), StepperMotor(id="left"))
        self.thruster_right = ThrusterController(
            Thruster(id="right"), StepperMotor(id="right"))
        self.thruster_tail = Thruster(id="tail")

        self.tail_thrust = 0.0

        self.min_step_time = self.thruster_left.step_time_ms

    def get_nb_steps(self) -> int:
        return self.thruster_left.stepper.nb_steps

    def run(self, target_states: dict):
        """run is started in a subprocess. target_state is a multiprocessing proxy"""
        latch = CountDownLatch(count=3)

        self.thruster_left.launch(self.thruster_right, latch)
        self.thruster_right.launch(self.thruster_left, latch)

        self.thruster_tail.arm_thruster()
        latch.count_down()
        latch.wait()

        plotter.start_display()

        self.tail_thruster_loop(target_states)

        plotter.stop_display()

    def tail_thruster_loop(self, target_states: dict):
        count_to_10 = 0

        target_tail_thrust = 0
        try:
            while True:
                tm = time.time()

                # every 10 iterations, update target state
                count_to_10 += 1
                if count_to_10 == 10:
                    count_to_10 = 0
                    # withdraw new target states from proxy
                    if target_states["stopped"]:
                        print("stopped tracking setpoints")
                        return

                    targets = target_states["targets"]

                    # if self.thruster_left.target_state != targets["left_state"]:
                    #     print("delay: ", round(time.time()*1000)-targets["tm"])

                    target_tail_thrust = targets["tail_thrust"]
                    self.thruster_left.target_state = targets["left_state"]
                    self.thruster_right.target_state = targets["right_state"]
                

                try:
                    if self.tail_thrust != target_tail_thrust:
                        max_delta = max(abs(self.thruster_left.get_delta()), abs(
                            self.thruster_right.get_delta()))
                        if max_delta == 0:
                            self.tail_thrust = target_tail_thrust
                        else:
                            self.tail_thrust = self.tail_thrust + \
                                (target_tail_thrust - self.tail_thrust) * \
                                1/max_delta

                    applied_thrust = self.tail_thrust * self.thruster_left.get_thrust_coef() * \
                        self.thruster_right.get_thrust_coef()

                    self.thruster_tail.set_pwm(round(applied_thrust*100))

                    time.sleep(self.min_step_time+tm - time.time())
                except Exception as e:
                    print(e)
        finally:
            self.thruster_left.stop()
            self.thruster_right.stop()
            self.thruster_tail.stop()
            print("stopped all thrusters")


class ThrusterController:
    def __init__(self, thruster: Thruster, stepper: StepperMotor) -> None:
        self.thruster = thruster
        self.stepper = stepper
        self.state = ThrusterState(0.0, 0, 1.0)
        self.target_state = ThrusterState(0.0, 0, 1.0)

        self.angle_incr = self.stepper.angle_by_step
        self.step_time_ms = self.stepper.min_step_time
        self.stopped = True

        self.loop_thread = typing.cast(Thread, None)

    def update_target(self, new_target: ThrusterState):
        self.target_state = new_target

    def get_thrust_coef(self) -> float:
        return self.state.thrust_coef

    def get_delta(self) -> int:
        return self.target_state.pos - self.state.pos

    def launch(self, other: 'ThrusterController', latch: CountDownLatch):
        if not self.stopped:
            print("controller already running")
            return
        self.stopped = False

        self.loop_thread = Thread(target=self.state_tracking_loop,
                                  args=(other, latch))

        self.loop_thread.start()

    def stop(self):
        self.stopped = True
        if self.loop_thread is not None:
            self.loop_thread.join()
            self.loop_thread = None

    def state_tracking_loop(self, other: 'ThrusterController', latch: CountDownLatch):
        self.thruster.arm_thruster()

        latch.count_down()
        latch.wait()
        """This function ensures a smooth transition from the current thruster state to the target state.

        It assumes thrust transition is negligible compared to stepper rotation time.
        """
        while not self.stopped:
            tm = time.time()

            try:
                speed_coef = 1.0
                spin = 0

                opposite_delta = other.get_delta()
                other_thrust_coef = other.get_thrust_coef()

                delta = self.target_state.pos - self.state.pos

                next_state = self.increment_state(
                    self.state, self.target_state, opposite_delta)

                # reminder: even if state did not change, the state of the other thruster might have

                tau = next_state.tau * next_state.thrust_coef * other_thrust_coef

                # warning: do not assign this tau to next_state

                spin = next_state.pos - self.state.pos  # 1, 0 or -1

                if spin != 0 and abs(opposite_delta) > abs(delta):
                    speed_coef = abs(delta) / abs(opposite_delta)
                    # avoid sleeping too long if we have a very small move to make while the opposite
                    # thruster as a long rotation to perform.
                    #
                    # We will reach the target position sooner but it ensures we loop frequently enough
                    # to catch new updates of target state
                    if speed_coef < 0.1:
                        speed_coef = 0.1

                # if next_state != self.state:
                #     print(next_state)

                self.state = next_state

                # do not use next_state.tau
                self.thruster.set_pwm(round(tau*100))

                if spin != 0:
                    clockwise = spin > 0
                    sleep_time = (self.step_time_ms/2) / speed_coef

                    time.sleep(sleep_time + tm - time.time())
                    self.stepper.bring_sally_up(clockwise)

                    time.sleep(sleep_time)
                    self.stepper.bring_sally_down()

                    continue

            except Exception as e:
                print(e)

            # here the sleep time is arbitrary
            time.sleep(self.step_time_ms)

        self.thruster.stop()
        self.stepper.stop()

    def increment_state(self, current: ThrusterState, target: ThrusterState, opposite_delta: int) -> ThrusterState:
        new = ThrusterState(tau=current.tau, pos=current.pos, thrust_coef=1.0)

        if current.pos == target.pos and current.tau == target.tau:
            return new

        if current.pos != target.pos:
            delta_pos = target.pos - current.pos

            delta_phi = delta_pos*self.angle_incr

            spin = 1 if delta_pos > 0 else -1

            incr_angle = self.angle_incr * spin

            new.pos = current.pos + spin

            if abs(delta_phi) < pi:  # less than half turn
                # same spin (we do not reverse thrust)
                if current.tau * target.tau > 0:
                    # most frequent condition.
                    #
                    # this formula allows a smooth transition from current vector state to next vector state.
                    #
                    # The head of the transition vector (i.e the 'new' state) slides on the line going from
                    # the head of the initial vector to the head of the final vector
                    new.tau = abs(target.tau * current.tau) * sin(delta_phi) / \
                        (current.tau * sin(incr_angle) +
                         target.tau * sin(delta_phi - incr_angle))
                elif cos(delta_phi) > 0:
                    # this is a particular case where we slightly turn the thruster (less than a quarter turn) but
                    # by reversing the thrust in the meantime.
                    # It is also entered to reactivate thrust after it was disabled to perform a half turn.
                    #
                    # We will apply the target thrust immediately (this may not be relevant since in this case the thruster may
                    # be the slowest to transition, to be verified) but we apply a coefficient increasing from 0 as the delta decreases.
                    #
                    # In practice, the next iteration should immediately enter the above condition so this
                    # case is not frequent
                    new.tau = target.tau
                    new.thrust_coef = cos(delta_phi)
                else:
                    # more than a quarter turn to perform while reversing thrust. Turn off thrusters.
                    new.thrust_coef = 0
            else:
                # more than half a turn to perform. We must turn off thrusters.
                new.thrust_coef = 0
        else:
            if opposite_delta == 0:
                new.tau = target.tau  # immediately apply target thrust
            else:
                # progressively update thrust to follow transition of the opposite motor
                opposite_incr_perc = 1 / abs(opposite_delta)
                new.tau = current.tau + \
                    (target.tau - current.tau)*opposite_incr_perc

        return new


def bound(x):
    if x > 1:
        return 1.0
    if x < -1:
        return -1.0
    return x
