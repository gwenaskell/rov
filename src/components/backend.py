import asyncio
import os
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
    def __init__(self, init_val = False) -> None:
        self.bool = init_val
        self.pressed = False
    
    def update(self, cur_state: bool) -> bool:
        if cur_state:  # button pressed
            if self.pressed:
                # we already know it is pressed
                return self.bool
            self.pressed = True
            self.bool = not self.bool
        elif self.pressed: # button was released
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
        self.pilot.init()
        inputs: GamePad = GamePad(connected=False)

        self.feedbacks.measurements = await self.sensors.read_sensors()
        
        create_task(self.imu.run())
        last_sensors_read = time()
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

                try:
                    inputs = await wait_for(inputs_queue.get(), timeout=0.2)
                except asyncio.TimeoutError:
                    # keep previous inputs value
                    pass

                tick = time()

                imu_data = self.imu.get_current_state()

                if sensors_t is not None:
                    self.feedbacks.measurements = await sensors_t

                self.feedbacks.bridled = Safety.must_bridle(
                    self.feedbacks.measurements)

                self._iter(inputs, imu_data)

                self.feedbacks.iter_ms = round(
                    1000*(time()-tick)+self.feedbacks.iter_ms/2)

                self.feedbacks.status = self.pilot.status

            except Exception as e:
                print(e)

    def _set_waiting(self):
        self.pilot.switch_engines(Status.PAUSED)
        self._waiting_fresh_input = True

    def _iter(self, inputs: GamePad, state: RovState):
        if not self._waiting_fresh_input:
            if self.pilot.status != Status.RUNNING:
                return  # skip update, thrusters are paused or stopped
            if time()*1000 - inputs.tm_ms > 2000:
                print(
                    "too much time elapsed since last input. pausing thrusters")
                self._set_waiting()
                return
        if self._waiting_fresh_input:  # resume
            if time()*1000 - inputs.tm_ms > 2000:
                return  # continue to wait
            self._waiting_fresh_input = False

        surface_mode = self.surface_switch.update(inputs.buttons.L)

        commands = Commands(fx=float(inputs.sticks.leftX)/100, fz=0,
                            cx=float(inputs.sticks.rightY)/100, cy=float(inputs.sticks.rightY)/100,
                            cz=-float(inputs.sticks.leftY)/100, tm_ms=inputs.tm_ms, surface=surface_mode)

        if inputs.buttons.R:
            commands.fz = -float(inputs.sticks.rightX)/100

        commands = self.pid.get_correction(state, commands)

        self.pilot.apply_setpoints(commands, state, bridle=self.feedbacks.bridled)
