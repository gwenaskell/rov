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

    def connect(self, input_stream: Connection):
        self.pilot.start()
        while True:
            try:
                i: Optional[GamePad] = input_stream.recv()

                if not i:
                    return

                # state = self.imu.get_current_state()

                # commands = self.pid.get_correction(0, state, controls)

                commands = Commands(fx=float(i.sticks.leftX)/100, fz=0,
                                    cx=float(i.sticks.rightY)/100, cy=float(i.sticks.rightX)/100,
                                    cz=float(i.sticks.leftY)/100, tm=i.tm)

                self.pilot.apply_setpoints(commands)
            except Exception as e:
                print(e)


def run(input_stream: Connection):
    loop = MainLoop()
    try:
        loop.connect(input_stream)
    except KeyboardInterrupt:
        pass
    print("mainloop stopped")
    print("plot stopped")
