import asyncio
from time import time

from src.api.classes import GamePad
from src.components.safety import Safety
from src.components.sensors import Sensors

from dataclasses import dataclass
from typing import Optional
from .imu import IMU, RovState
from .pid import PID
from .classes import Commands, Feedbacks, Status
from .pilot import Pilot
from asyncio import Queue, Task, create_task, wait_for


@dataclass
class Inputs:
    # speed
    c_vx: float
    c_vz: float
    # angular speed
    c_wx: float
    c_wy: float
    c_wz: float


class Switch:
    def __init__(self, init_val=False) -> None:
        self.bool = init_val
        self.pressed = False

    def update(self, cur_state: bool) -> bool:
        if cur_state:  # button pressed
            if self.pressed:
                # we already know it is pressed
                return self.bool
            self.pressed = True
            self.bool = not self.bool
        elif self.pressed:  # button was released
            self.pressed = False
        return self.bool


class Backend:
    def __init__(self) -> None:
        self.feedbacks: Feedbacks = Feedbacks()
        self.imu = IMU()
        self.pid = PID()
        self.pilot = Pilot()
        self.sensors = Sensors()
        self.exiting = False
        self.run_task: Optional[Task] = None
        self.surface_switch = Switch(True)
        self.bridled_switch = Switch(True)
        self._waiting_fresh_input: bool = False

    async def stop(self):
        self.exiting = True
        if self.run_task:
            await self.run_task
            self.run_task = None
        print("backend stopped")

    def start(self, inputs_queue: Queue):
        if self.run_task:
            raise RuntimeError("backend already running")
        self.exiting = False
        self.run_task = create_task(self._run(inputs_queue))

    def switch_engines(self, status: Status):
        self._waiting_fresh_input = False  # override
        self.pilot.switch_engines(status)

    async def _run(self, inputs_queue: Queue):
        inputs: GamePad = GamePad(connected=False)

        self.feedbacks.measurements = await self.sensors.read_sensors()

        create_task(self.imu.run())
        last_sensors_read = time()
        tm = last_sensors_read
        tick = tm

        while True:
            if self.exiting:
                self.imu.stop()
                self.pilot.stop()
                return

            try:
                sensors_t = None

                tm = time()

                if tm - last_sensors_read > 0.5:  # two times per second
                    sensors_t = create_task(self.sensors.read_sensors())
                    last_sensors_read = tm

                self.feedbacks.tracker_iter_time_ms = self.pilot.get_feedbacks()["tracker_iter_time_ms"]

                self.feedbacks.iter_ms = int(1000 * (tm - tick))

                try:
                    inputs = await wait_for(inputs_queue.get(), timeout=0.2)
                except asyncio.TimeoutError:
                    # keep previous inputs value
                    pass

                tick = time()

                imu_data = self.imu.get_current_state()

                self.feedbacks.euler = imu_data.q.to_euler()

                if sensors_t is not None:
                    self.feedbacks.measurements = await sensors_t

                bridled = self.bridled_switch.update(inputs.buttons.L2)

                self.feedbacks.bridled = bridled or Safety.must_bridle(self.feedbacks.measurements)

                self._iter(inputs, imu_data)

                self.feedbacks.status = self.pilot.status

            except Exception as e:
                print("backend exception:", e)
                await asyncio.sleep(1)

    def _set_waiting(self):
        self.pilot.switch_engines(Status.PAUSED)
        self._waiting_fresh_input = True

    def _iter(self, inputs: GamePad, state: RovState):
        if not self._waiting_fresh_input:
            if self.pilot.status != Status.RUNNING:
                return  # skip update, thrusters are paused or stopped
            if time() * 1000 - inputs.tm_ms > 2000:
                print("too much time elapsed since last input. pausing thrusters")
                self._set_waiting()
                return
        if self._waiting_fresh_input:  # resume
            if time() * 1000 - inputs.tm_ms > 2000:
                return  # continue to wait
            self._waiting_fresh_input = False

        surface_mode = self.surface_switch.update(inputs.buttons.L)

        commands = Commands(
            fx=float(inputs.sticks.leftX) / 100,
            fz=0,
            cx=float(inputs.sticks.rightY) / 100,
            cy=0,
            cz=-float(inputs.sticks.leftY) / 100,
            tm_ms=inputs.tm_ms,
            surface=surface_mode,
        )

        if inputs.buttons.R:
            commands.fz = -float(inputs.sticks.rightX) / 100
        else:
            commands.cy = float(inputs.sticks.rightX) / 100

        commands = self.pid.apply_correction(state, commands)

        # print(commands)

        self.pilot.apply_setpoints(commands, state, bridle=self.feedbacks.bridled)
