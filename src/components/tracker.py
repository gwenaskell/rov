from threading import Thread
import time
import typing

from src.components.classes import NamespaceProxy, Status, TargetsProxy, ThrusterState

from .utils.countdown import CountDownLatch
from .accessors import Thruster
from .accessors import StepperMotor
from math import sin, cos, pi
from src.drivers.mock.plot import plotter


class SetpointsTracker:
    def __init__(self) -> None:
        self.thruster_left = ThrusterController(
            Thruster(id="left"), StepperMotor(id="left"))
        self.thruster_right = ThrusterController(
            Thruster(id="right"), StepperMotor(id="right"))
        self.thruster_tail = Thruster(id="tail")

        self.tail_thrust = 0.0

        self.min_step_time = self.thruster_left.step_time

    def get_nb_steps(self) -> int:
        return self.thruster_left.stepper.nb_steps

    def run(self, states_proxy: NamespaceProxy):
        """run is started in a subprocess"""
        latch = CountDownLatch(count=3)

        self.thruster_left.launch(self.thruster_right, latch)
        self.thruster_right.launch(self.thruster_left, latch)

        self.thruster_tail.arm_thruster()
        latch.count_down()
        latch.wait()

        plotter.start_display()

        self.tail_thruster_loop(states_proxy)

        plotter.stop_display()

    def _pause(self):
        self.thruster_left.pause()
        self.thruster_right.pause()
        self.thruster_tail.set_pwm(0)
        self.tail_thrust = 0

    def tail_thruster_loop(self, states_proxy: NamespaceProxy):
        count_to_10 = 0

        target_tail_thrust = 0

        paused = False

        try:
            while True:
                tm = time.time()

                # every 10 iterations, update target state
                count_to_10 += 1
                if count_to_10 == 10:
                    count_to_10 = 0
                    # withdraw new target states from proxy

                    status: Status = states_proxy["status"]
                    if status == Status.STOPPED:
                        print("stopped tracking setpoints")
                        return
                    if status == Status.PAUSED and not paused:
                        print("pausing thrusters")
                        self._pause()
                        paused = True
                    else:
                        targets: TargetsProxy = states_proxy["targets"]

                        if paused:
                            print("resuming thrusters")
                            paused = False
                            self.thruster_left.resume()
                            self.thruster_right.resume()

                        # if self.thruster_left.target_state != targets["left_state"]:
                        #     print("delay: ", round(time.time()*1000)-targets["tm"])

                        target_tail_thrust = targets["tail_thrust"]
                        self.thruster_left.target_state = targets["left_state"]
                        self.thruster_right.target_state = targets["right_state"]

                if paused:
                    sleep("tail", self.min_step_time)
                    continue

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

                    self.tail_thrust = self.tail_thrust * self.thruster_left.get_thrust_coef() * \
                        self.thruster_right.get_thrust_coef()

                    self.thruster_tail.set_pwm(round(self.tail_thrust*100))

                    sleep("tail", self.min_step_time+tm - time.time())
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
        self.status = Status.STOPPED
        self.state = ThrusterState(0.0, 0)
        self.target_state = ThrusterState(0.0, 0)

        self.angle_incr = self.stepper.angle_by_step
        self.step_time = self.stepper.min_step_time

        self.loop_thread = typing.cast(Thread, None)

    def update_target(self, new_target: ThrusterState):
        self.target_state = new_target

    def get_thrust_coef(self) -> float:
        return self.state.thrust_coef

    def get_delta(self) -> int:
        return self.target_state.pos - self.state.pos

    def launch(self, other: 'ThrusterController', latch: CountDownLatch):
        if self.status != Status.STOPPED:
            print("controller already running")
            return
        self.status = Status.RUNNING

        self.loop_thread = Thread(target=self.state_tracking_loop,
                                  args=(other, latch))

        self.loop_thread.start()

    def stop(self):
        self.status = Status.STOPPED
        if self.loop_thread is not None:
            self.loop_thread.join()
            self.loop_thread = None

    def pause(self):
        if self.status == Status.RUNNING:
            self.status = Status.PAUSED

    def resume(self):
        if self.status == Status.STOPPED:
            raise RuntimeError("cannot resume a stopped thruster")
        self.status = Status.RUNNING

    def state_tracking_loop(self, other: 'ThrusterController', latch: CountDownLatch):
        self.thruster.arm_thruster()

        latch.count_down()
        latch.wait()
        """This function ensures a smooth transition from the current thruster state to the target state.

        It assumes thrust transition is negligible compared to stepper rotation time.
        """
        while self.status != Status.STOPPED:
            try:
                if self.status == Status.PAUSED:
                    if self.state.tau != 0:
                        self.thruster.set_pwm(0)
                        self.state.tau = 0
                    sleep(self.thruster.id, self.step_time)
                    continue

                tm = time.time()

                speed_coef = 1.0
                spin = 0

                opposite_delta = other.get_delta()
                other_thrust_coef = other.get_thrust_coef()

                delta = self.target_state.pos - self.state.pos

                next_state = self.increment_state(
                    self.state, self.target_state, opposite_delta)

                # reminder: even if state did not change, the state of the other thruster might have

                next_state.tau = next_state.tau * other_thrust_coef

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
                self.thruster.set_pwm(round(next_state.tau*100))

                if spin != 0:
                    clockwise = spin > 0
                    sleep_time = (self.step_time/2) / speed_coef

                    sleep_remaining = sleep_time + tm - time.time()
                    offset = 0.0
                    if sleep_remaining < 0:
                        offset = -sleep_remaining
                        sleep_remaining = 0

                    time.sleep(sleep_remaining)
                    self.stepper.bring_sally_up(clockwise)

                    time.sleep(sleep_time-offset)  # offset is <= 0
                    self.stepper.bring_sally_down()

                    continue

            except Exception as e:
                print(e)

            # here the sleep time is arbitrary
            sleep(self.thruster.id, self.step_time)

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

            incr_angle = self.angle_incr * spin  # non zero

            new.pos = current.pos + spin

            if abs(delta_phi) < pi:  # less than half turn
                cos_delta = cos(delta_phi)

                # same propeller spin (we do not reverse thrust) and going from non-zero to non-zero
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
                elif cos_delta > 0:
                    # this is a particular case where we slightly turn the thruster (less than a quarter turn) but
                    # by reversing the thrust in the meantime.
                    # It is also entered to reactivate thrust after it was disabled to perform a half turn.
                    #
                    new.tau = new.tau + (target.tau-new.tau) * cos_delta
                else:
                    # more than a quarter turn to perform while reversing thrust. Turn off thrusters.
                    new.thrust_coef = 0
                    new.tau = 0
            else:
                # more than half a turn to perform. We must turn off thrusters.
                new.thrust_coef = 0
                new.tau = 0
        else:
            if opposite_delta == 0:
                new.tau = target.tau  # immediately apply target thrust
            else:
                # progressively update thrust to follow transition of the opposite motor
                opposite_incr_perc = 1 / abs(opposite_delta)
                new.tau = current.tau + \
                    (target.tau - current.tau)*opposite_incr_perc

        return new


def sleep(id: str, val: float):
    if val < 0:
        print(id, " > negative sleep: ", val)
        return
    time.sleep(val)
