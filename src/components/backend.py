import asyncio
from optparse import Option
import os

from src.api.classes import GamePad

if not os.getenv("ONBOARD"):
    from ..drivers.mock import mock
    mock()

from calendar import c
from dataclasses import dataclass
from multiprocessing import Process
from multiprocessing.connection import Connection
from tkinter import mainloop
from typing import Optional
from .imu import IMU
from .pid import PID
from .classes import Commands
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


class Backend:
    def __init__(self) -> None:
        self.imu = IMU()
        self.pid = PID()
        self.pilot = Pilot()
        self.exiting = False
        self.run_task: Optional[Task] = None

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

    async def _run(self, inputs_queue: Queue):
        self.pilot.start()
        inputs: Optional[GamePad] = None
        while True:
            try:
                # read imu data while we wait for next iteration
                imu_t = create_task(self.imu.get_current_state())

                try:
                    inputs = await wait_for(inputs_queue.get(), timeout=0.2)
                except asyncio.TimeoutError:
                    pass

                imu_data = await imu_t

                if self.exiting:
                    self.pilot.stop_engines()
                    return

                if not inputs:
                    self.pilot.stop_engines()
                    continue

                # commands = self.pid.get_correction(0, state, controls)

                commands = Commands(fx=float(inputs.sticks.leftX)/100, fz=0,
                                    cx=float(inputs.sticks.rightY)/100, cy=0,
                                    cz=-float(inputs.sticks.leftY)/100, tm_ms=inputs.tm_ms)

                if inputs.buttons.R:
                    commands.fz = -float(inputs.sticks.rightX)/100
                else:
                    commands.cy = float(inputs.sticks.rightX)/100

                self.pilot.apply_setpoints(commands)
            except Exception as e:
                print(e)
