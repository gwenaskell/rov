from dataclasses import dataclass
import math
from asyncio import coroutine, sleep
from time import time
import numpy as np
import imufusion
from .accessors import IMU as _Imu_driver, Compass, AccelValue


from .classes import RovState




def rad(deg):
    return deg*math.pi / 180

def deg(rad):
    return rad * 180 / math.pi

def _modulo(deg):
    if deg < -350:
        return 360 - deg
    if deg > 350:
        return deg - 360
    return deg


class IMU:
    def __init__(self, dt = 0.05) -> None:
        self.dt = dt
        self.imu = _Imu_driver()
        self.magnetometer = Compass()

        self.fusion = imufusion.Ahrs()
        
        self.prev_state: RovState = RovState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        
        self.offset = imufusion.Offset(100)

        self.fusion.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                                  0.5,  # gain
                                                  2000,  # gyroscope range
                                                  10,  # acceleration rejection
                                                  10,  # magnetic rejection
                                                  100)  # recovery trigger period
        
        self.prev_time = 0
        
        self.euler = [0, 0, 0]

        self.ang_speed = [0, 0, 0]

        self._stop = False
    
    async def run(self):
        self.prev_time = 0
        while not self._stop:
            self._update()
            await sleep(self.dt)

    def stop(self):
        self._stop = True

    def _update(self):
        cur_tm = time()
        dt = cur_tm - self.prev_time
        self.prev_time = cur_tm
        
        accel = self.imu.get_state()
        compass = self.magnetometer.get_state()
        
        gyro = self.offset.update(np.array([accel.wx, accel.wy, accel.wz]))
        
        self.fusion.update(gyro, np.array([accel.ax, accel.ay, accel.az]), np.array([compass.bx, compass.by, compass.bz]), dt)
        
        quat: imufusion.Quaternion = self.fusion.quaternion
        
        angles: list = quat.to_euler()

        self.ang_speed[0]=_modulo(angles[0] - self.euler[0])/dt
        self.ang_speed[1]=_modulo(angles[1] - self.euler[1])/dt
        self.ang_speed[2]=_modulo(angles[2] - self.euler[2])/dt
        
        self.euler = angles

    def get_current_state(self) -> RovState:
        quat: imufusion.Quaternion = self.fusion.quaternion
        
        angles: list = quat.to_euler()
        
        a: list = self.fusion.linear_acceleration
        
        state = RovState(
            ax=a[0],
            ay=a[1],
            az=a[2],
            thx=self.euler[0],
            thy=self.euler[1],
            thz=self.euler[2],
            # vx=self.prev_state.vx + a[0]*dt,
            # vy=self.prev_state.vy + a[1]*dt,
            # vz=self.prev_state.vz + a[2]*dt,
            wx=self.ang_speed[0],
            wy=self.ang_speed[1],
            wz=self.ang_speed[2],
        )

        self.prev_state = state
        
        return state
        
        # # Étape de prédiction
        # self.ekf.predict(dt, accel)

        # roll = math.atan2(accel.ay, accel.az)
        # pitch = math.atan2(-accel.ax, math.sqrt(accel.ay**2 + accel.az**2))
        # yaw = math.atan2(-compass.by, compass.bx)

        # # Étape de mise à jour
        # return self.ekf.update(accel, roll, pitch, yaw)

