from random import randint
from typing import Tuple, Any
import numpy as np
import math
from .classes import RovState
from .accessors import AccelValue, CompassValue

def rand():
    return (randint(0, 100) - 50)/100

def rand_anomaly():
    if randint(0, 20) > 0:
        return 0
    return rand()*0.3

def noise_acc(val: float) -> float:
    return val #* (1 + rand()*0.2) + rand()*0.1 + rand_anomaly()

def noise_gyr(val: float) -> float:
    return val #* (1+rand()*0.05) + rand()*0.05 + rand_anomaly()

def noise_mag(val: float) -> float:
    return val * (1 + rand()*0.02) + rand()*0.02 + rand_anomaly()

class Simulator:
    def __init__(self):
        
        self._iter_count = 0

        # [x, y, z, vx, vy, vz, ax, ay, az]
        self.xyz = [0.0]*9

        self.wxyz = [0.0]*9

        self.xyz_prev = [0.0]*9

        self.wxyz_prev = [0.0]*9
        
        # target force vector
        self.target = np.array([0.001, 0.0, 0.0])
        
        # current force vector
        self.attrac = np.array([0.001, 0.0, 0.0])
        
        self.gyro_err = [0.0, 0.0, 0.0]

        self.g_vec_0: Any = np.array([0, 0, 9.8])
        self.g_vec = np.array([*self.g_vec_0])

        self.mag_vec_0: Any = np.array([1, 0, 0])
        self.mag_vec = np.array([*self.mag_vec_0])

        # [ax, ay, az]
        self.accel = [*(self.attrac+self.g_vec)]
        self.accel_old = self.accel.copy()

        # [vx, vy, vz,]
        self.gyro = [0.0, 0.0, 0.0]
        self.compass = [*self.mag_vec]

    def rotate_vector_towards(self, v, u, angle = 0.01):
        # Convert degrees to radians
        
        if np.linalg.norm(u) == 0:
            return v
        
        n_v = np.linalg.norm(v)
        
        if n_v == 0:
            v = np.array([1, 0.0, 0.0])
        
        # Normalize the vectors
        v = v / n_v
        u = u / np.linalg.norm(u)
        
        # Compute the cosine of the angle between v and u
        cos_angle = np.dot(v, u)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Clip to handle numerical precision issues
        
        # Compute the actual angle
        cur_angle = math.acos(cos_angle)
        
        # If the angle is less than or equal to one degree, set v to u
        if cur_angle <= angle:
            return u * n_v
        
        # Compute the rotation axis (cross product of v and u)
        k = np.cross(v, u)
        
        # Normalize the rotation axis
        k = k / np.linalg.norm(k)
        
        # Rodrigues' rotation formula components
        k_cross_v = np.cross(k, v)
        k_dot_v = np.dot(k, v)
        
        v_rotated = v * math.cos(angle) + k_cross_v * math.sin(angle) + k * k_dot_v * (1 - math.cos(angle))
        
        return v_rotated * n_v

    def iter(self, dt):
        self._iter_count+=1

        self.xyz_prev = self.xyz.copy()
        self.wxyz_prev = self.wxyz.copy()

        # occasionnally update target
        if self._iter_count % 500 == 0:
            for i in range(3):
                self.target[i] = rand()*2


        # slowly move current force vector towards target
        self.attrac = self.rotate_vector_towards(self.attrac, self.target)

        self.attrac *= min(max(np.linalg.norm(self.target) / np.linalg.norm(self.attrac), 0.99), 1.01)

        # PFD: F/m - (c/m)*v = a

        c_m = 1 # arbitrary
        
        # compute and integrate acceleration
        
        for i in range(3):
            self.xyz[i+6] = self.attrac[i] - c_m*self.xyz[i+3]

            self.xyz[i+3] += self.xyz[i+6]*dt
            self.xyz[i] += self.xyz[i+3]*dt

        # compute and integrate angular acceleration (arbitrarily decide that torque is equivalent to attrac)

        for i in range(3):
            self.wxyz[i+6] = self.attrac[i] - c_m*self.wxyz[i+3]
            
            self.wxyz[i+3] += self.wxyz[i+6]*dt
            self.wxyz[i] += self.wxyz[i+3]*dt
        
        self._iter_measurements()
    
    def get_state(self):
        return RovState(
            vx=self.xyz[3],
            vy=self.xyz[4],
            vz=self.xyz[5],
            ax=self.xyz[6],
            ay=self.xyz[7],
            az=self.xyz[8],
            thx=self.wxyz[0],
            thy=self.wxyz[1],
            thz=self.wxyz[2],
            wx=self.wxyz[3],
            wy=self.wxyz[4],
            wz=self.wxyz[5],
        ), self.mag_vec
    
    def get_rotation_matrix(self):
        # Convert angles from degrees to radians
        roll = self.wxyz[0]
        pitch = self.wxyz[1]
        yaw = self.wxyz[2]
        
        # Rotation matrix around x-axis (roll)
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        # Rotation matrix around y-axis (pitch)
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        # Rotation matrix around z-axis (yaw)
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation matrix
        R = R_z @ R_y @ R_x
        
        return R

    def _iter_measurements(self):
        R = self.get_rotation_matrix()
        
        self.g_vec: Any = np.dot(R, self.g_vec_0)
        
        self.mag_vec: Any = np.dot(R, self.mag_vec_0)

        accel_old = self.accel.copy()
        
        for i in range(3):
            max_accel_speed = 0.001
            
            prev_diff = self.accel[i] - self.accel_old[i]
            
            new_accel_val = self.xyz[i+6] + self.g_vec[i]
            diff = new_accel_val - self.accel[i]
            self.accel[i] += max(min(diff, prev_diff + max_accel_speed), prev_diff-max_accel_speed)

            max_gyro_speed = 0.1

            new_gyro_val = self.wxyz[i+3]
            diff = new_gyro_val - self.gyro[i]
            self.gyro[i] += max(min(diff, max_gyro_speed), -max_gyro_speed)
            # self.gyro_err[i] = max(min(self.gyro_err[i]+self.gyro[i]*0.0005, 0.1), -0.1)

            diff = self.mag_vec[i] - self.compass[i]
            max_compass_speed = 0.0001 + 0.01*diff**2
            self.compass[i] += max(min(diff, max_compass_speed), -max_compass_speed)
        
        self.accel_old = accel_old

    def get_measurements(self) -> Tuple[AccelValue, CompassValue]:
        return (
            AccelValue(
                ax=noise_acc(self.accel[0]),
                ay=noise_acc(self.accel[1]),
                az=noise_acc(self.accel[2]),
                wx=noise_gyr(self.gyro[0] + self.gyro_err[0]),
                wy=noise_gyr(self.gyro[1] + self.gyro_err[1]),
                wz=noise_gyr(self.gyro[2] + self.gyro_err[2]),
            ),
            CompassValue(
                bx=noise_mag(self.compass[0]),
                by=noise_mag(self.compass[1]),
                bz=noise_mag(self.compass[2]),
            ),
        )
