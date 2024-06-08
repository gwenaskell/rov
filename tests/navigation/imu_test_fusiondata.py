from typing import List
from src.components.imu_simulator import Simulator, RovState
from src.components.imu import IMU
from src.components.accessors import AccelValue, CompassValue


import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import math


data = np.genfromtxt("imufusion_sensor_data.csv", delimiter=",", skip_header=1)

states_imu: List[RovState] = []

t = data[:, 0]
aax = data[:, 4]
aay = data[:, 5]
aaz = data[:, 6]
wgx = data[:, 1]
wgy = data[:, 2]
wgz = data[:, 3]
bx = data[:, 6]
by = data[:, 7]
bz = data[:, 8]

simulator = Simulator()

cur_idx = {"val": 0}


def rad(deg):
    return deg*math.pi / 180

def deg(deg):
    return deg * 180 / math.pi

class FakeAccel:
    def get_state(self):
        return AccelValue(
            ax=aax[cur_idx['val']],
            ay=aax[cur_idx['val']],
            az=aax[cur_idx['val']],
            wx=(wgx[cur_idx['val']]),
            wy=(wgx[cur_idx['val']]),
            wz=(wgx[cur_idx['val']]),
        )

class FakeCompass:    
    def get_state(self):
        return CompassValue(
            bx=bx[cur_idx['val']],
            by=by[cur_idx['val']],
            bz=bz[cur_idx['val']],
        )

imu = IMU()
imu.imu = FakeAccel()
imu.magnetometer = FakeCompass()


for i in range(len(t)):
    cur_idx['val'] = i
    state_imu = imu.get_current_state(1/100)
    states_imu.append(state_imu)


# exit(0)

def config_axes(axes, size):
    for i in range(size):
        axes[i].grid()
        axes[i].legend()


fig, axs = plt.subplots(6, 1)
# Plot each signal on a separate subplot
axs[0].plot(t, [s.vx for s in states_imu])
axs[0].set_title('vx')
axs[1].plot(t, [s.vy for s in states_imu])
axs[1].set_title('vy')
axs[2].plot(t, [s.vz for s in states_imu])
axs[2].set_title('vz')
axs[3].plot(t, [s.ax for s in states_imu])
axs[3].plot(t, aax)
axs[3].set_title('ax')
axs[4].plot(t, [s.ay for s in states_imu])
axs[4].plot(t, aay)
axs[4].set_title('ay')
axs[5].plot(t, [s.az for s in states_imu])
axs[5].plot(t, aaz)
axs[5].set_title('az')
config_axes(axs, 6)




figw, axsw = plt.subplots(3, 1)
# Plot each signal on a separate subplot
axsw[0].plot(t, [s.wx for s in states_imu])
axsw[0].plot(t, wgx)
axsw[0].set_title('wx')
axsw[1].plot(t, [s.wy for s in states_imu])
axsw[1].plot(t, wgy)
axsw[1].set_title('wy')
axsw[2].plot(t, [s.wz for s in states_imu])
axsw[2].plot(t, wgz)
axsw[2].set_title('wz')
config_axes(axs, 3)


figb, axsb = plt.subplots(3, 1)
# Plot each signal on a separate subplot
axsb[0].plot(t, bx)
axsb[0].set_title('bx')
axsb[1].plot(t, by)
axsb[1].set_title('by')
axsb[2].plot(t, bz)
axsb[2].set_title('bz')
config_axes(axs, 3)
# Display the plot
# plt.subplots_adjust(bottom=0.1, right=0.8, top=0.9)
# plt.tight_layout()
plt.show()