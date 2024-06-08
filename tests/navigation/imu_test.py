from typing import List
from src.lab.imu_simulator import Simulator
from src.components.imu import IMU, RovState
from src.components.accessors import AccelValue, CompassValue


import matplotlib
import matplotlib.pyplot as plt
import numpy as np

TIMEFRAME = 10
N = 10000
N_SAMPLE = 1000


t = np.linspace(0, TIMEFRAME, N)
t_echant = np.linspace(0, TIMEFRAME, N_SAMPLE)
states: List[RovState] = []
states_imu: List[RovState] = []

aax = []
aay = []
aaz = []
wgx = []
wgy = []
wgz = []
mx = []
my = []
mz = []
bx = []
by = []
bz = []

simulator = Simulator()

class FakeAccel:    
    def get_state(self):
        return AccelValue(
            ax=aax[len(aax) - 1],
            ay=aax[len(aay) - 1],
            az=aax[len(aaz) - 1],
            wx=wgx[len(wgx) - 1],
            wy=wgx[len(wgy) - 1],
            wz=wgx[len(wgz) - 1],
        )

class FakeCompass:    
    def get_state(self):
        return CompassValue(
            bx=bx[len(bx) - 1],
            by=by[len(by) - 1],
            bz=bz[len(bz) - 1],
        )

imu = IMU()
imu.imu = FakeAccel()
imu.magnetometer = FakeCompass()


for i in range(N):
    simulator.iter(TIMEFRAME/N)
    state, mag = simulator.get_state()
    states.append(state)

    mx.append(mag[0])
    my.append(mag[1])
    mz.append(mag[2])
    if i%(N/N_SAMPLE) == 0:
        accel, compass = simulator.get_measurements()
        aax.append(accel.ax)
        aay.append(accel.ay)
        aaz.append(accel.az)
        wgx.append(accel.wx)
        wgy.append(accel.wy)
        wgz.append(accel.wz)
        bx.append(compass.bx)
        by.append(compass.by)
        bz.append(compass.bz)
        state_imu = imu.get_current_state(TIMEFRAME / N_SAMPLE)
        states_imu.append(state_imu)


# exit(0)


def config_axes(axes, size):
    for i in range(size):
        axes[i].grid()
        axes[i].legend()

fig, axs = plt.subplots(6, 1)
# Plot each signal on a separate subplot
axs[0].plot(t, [s.vx for s in states], label = "vx")
axs[0].plot(t_echant, [s.vx for s in states_imu], label = "vx_est")

axs[1].plot(t, [s.vy for s in states], label = "vy")
axs[1].plot(t_echant, [s.vy for s in states_imu], label = "wy_est")

axs[2].plot(t, [s.vz for s in states], label = "vz")
axs[2].plot(t_echant, [s.vz for s in states_imu], label = "vz_est")

axs[3].plot(t, [s.ax for s in states], label = "ax")
axs[3].plot(t_echant, [s.ax for s in states_imu], label = "ax_est")
axs[3].plot(t_echant, aax, label = "ax_acc")

axs[4].plot(t, [s.ay for s in states], label = "ay")
axs[4].plot(t_echant, [s.ay for s in states_imu], label = "ay_est")
axs[4].plot(t_echant, aay, label = "ay_acc")

axs[5].plot(t, [s.az for s in states], label = "az")
axs[5].plot(t_echant, [s.az for s in states_imu], label = "az_est")
axs[5].plot(t_echant, aaz, label = "az_acc")

config_axes(axs, 6)




figw, axsw = plt.subplots(6, 1)
# Plot each signal on a separate subplot
axsw[0].plot(t, [s.thx for s in states], label = "thx")
axsw[0].plot(t_echant, [s.thx for s in states_imu], label = "thx_est")

axsw[1].plot(t, [s.thy for s in states], label = "thy")
axsw[1].plot(t_echant, [s.thy for s in states_imu], label = "thy_est")

axsw[2].plot(t, [s.thz for s in states], label = "thz")
axsw[2].plot(t_echant, [s.thz for s in states_imu], label = "thz_est")

axsw[3].plot(t, [s.wx for s in states], label = "wx")
axsw[3].plot(t_echant, [s.wx for s in states_imu], label = "wx_est")
axsw[3].plot(t_echant, wgx, label = "wx_gyro")

axsw[4].plot(t, [s.wy for s in states], label = "wy")
axsw[4].plot(t_echant, [s.wy for s in states_imu], label = "wy_est")
axsw[4].plot(t_echant, wgy, label = "wy_gyro")

axsw[5].plot(t, [s.wz for s in states], label = "wz")
axsw[5].plot(t_echant, [s.wz for s in states_imu], label = "wz_est")
axsw[5].plot(t_echant, wgz, label = "wz_gyro")

config_axes(axsw, 6)


# figb, axsb = plt.subplots(3, 1)
# Plot each signal on a separate subplot
# axsb[0].plot(t, mx)
# axsb[0].plot(t_echant, bx)
# axsb[0].set_title('bx')
# axsb[1].plot(t, my)
# axsb[1].plot(t_echant, by)
# axsb[1].set_title('by')
# axsb[2].plot(t, mz)
# axsb[2].plot(t_echant, bz)
# axsb[2].set_title('bz')
# config_axes(axs, 3)
# Display the plot
# plt.subplots_adjust(bottom=0.1, right=0.8, top=0.9)
# plt.tight_layout()
plt.show()