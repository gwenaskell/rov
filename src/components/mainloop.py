from calendar import c
from dataclasses import dataclass
from multiprocessing.connection import Connection
from tkinter import mainloop
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
        while True:
            i: Inputs = input_stream.recv()

            # state = self.imu.get_current_state()

            # commands = self.pid.get_correction(0, state, controls)

            commands = Commands(fx=i.c_vx, fz=i.c_vz,
                                cx=i.c_wx, cy=i.c_wy, cz=i.c_wz)

            self.pilot.apply_setpoints(commands)


def run(input_stream: Connection):
    loop = MainLoop()
    loop.connect(input_stream)
