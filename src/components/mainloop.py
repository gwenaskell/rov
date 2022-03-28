import os

from src.api.api import GamePad

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
from .pilot import Commands, Pilot
from asyncio import Queue

@dataclass
class Inputs:
    # speed
    c_vx: float
    c_vz: float
    # angular speed
    c_wx: float
    c_wy: float
    c_wz: float


class MainLoop:
    def __init__(self) -> None:
        self.imu = IMU()
        self.pid = PID()
        self.pilot = Pilot()

    async def connect(self, inputs_queue: Queue):
        self.pilot.start()
        while True:
            try:
                i: Optional[GamePad] = await inputs_queue.get()

                if not i:
                    return

                # state = self.imu.get_current_state()

                # commands = self.pid.get_correction(0, state, controls)

                commands = Commands(fx=float(i.sticks.leftX)/100, fz=0,
                                    cx=float(i.sticks.rightY)/100, cy=0,
                                    cz=-float(i.sticks.leftY)/100, tm=i.tm)
                
                if i.buttons.R:
                    commands.fz = -float(i.sticks.rightX)/100
                else:
                    commands.cy = float(i.sticks.rightX)/100

                self.pilot.apply_setpoints(commands)
            except Exception as e:
                print(e)


async def run(inputs_queue: Queue):
    loop = MainLoop()
    try:
        await loop.connect(inputs_queue)
    except KeyboardInterrupt:
        pass
    print("mainloop stopped")
    print("plot stopped")
