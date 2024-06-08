import imufusion
import matplotlib.pyplot as pyplot
import numpy
import sys

# Import sensor data
data = numpy.genfromtxt("imufusion_sensor_data.csv", delimiter=",", skip_header=1)

sample_rate = 100  # 100 Hz

timestamp = data[:, 0]
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]
magnetometer = data[:, 7:10]

# Instantiate algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                   0.5,  # gain
                                   2000,  # gyroscope range
                                   10,  # acceleration rejection
                                   10,  # magnetic rejection
                                   5 * sample_rate)  # recovery trigger period = 5 seconds

# Process sensor data
delta_time = numpy.diff(timestamp, prepend=timestamp[0])

euler = numpy.empty((len(timestamp), 3))
accel = numpy.empty((len(timestamp), 3))

internal_states = numpy.empty((len(timestamp), 6))
flags = numpy.empty((len(timestamp), 4))

for index in range(len(timestamp)):
    gyroscope[index] = offset.update(gyroscope[index])

    ahrs.update(gyroscope[index], accelerometer[index], magnetometer[index], delta_time[index])

    euler[index] = ahrs.quaternion.to_euler()
    
    accel[index] = ahrs.linear_acceleration

    ahrs_internal_states = ahrs.internal_states
    internal_states[index] = numpy.array([ahrs_internal_states.acceleration_error,
                                          ahrs_internal_states.accelerometer_ignored,
                                          ahrs_internal_states.acceleration_recovery_trigger,
                                          ahrs_internal_states.magnetic_error,
                                          ahrs_internal_states.magnetometer_ignored,
                                          ahrs_internal_states.magnetic_recovery_trigger])

    ahrs_flags = ahrs.flags
    flags[index] = numpy.array([ahrs_flags.initialising,
                                ahrs_flags.angular_rate_recovery,
                                ahrs_flags.acceleration_recovery,
                                ahrs_flags.magnetic_recovery])

 
def plot_bool(axis, x, y, label):
    axis.plot(x, y, "tab:cyan", label=label)
    pyplot.sca(axis)
    pyplot.yticks([0, 1], ["False", "True"])
    axis.grid()
    axis.legend()

def disp(x):
    print(x)
    return x

def ddt(angle):
    mod = lambda x: x if 200 > x > - 200 else (x - 360 if x > 0 else 360 + x)
    return numpy.array([0, *[mod(angle[i] - angle[i-1])/(timestamp[i]-timestamp[i-1]) for i in range(1, len(timestamp))]])

# Plot Euler angles
figure, axes = pyplot.subplots(nrows=3, sharex=True)#, gridspec_kw={"height_ratios": [6, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1]})

axes[0].plot(timestamp, euler[:, 0], "tab:red", label="Roll")
axes[0].plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
axes[0].plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
axes[0].set_ylabel("Degrees")
axes[0].grid()
axes[0].legend()

axes[1].plot(timestamp, ddt(euler[:, 0]), "tab:red", label="wx")
axes[1].plot(timestamp, ddt(euler[:, 1]), "tab:green", label="wy")
axes[1].plot(timestamp, ddt(euler[:, 2]), "tab:blue", label="wz")
axes[1].set_ylabel("Degrees/s")
axes[1].grid()
axes[1].legend()

axes[2].plot(timestamp, gyroscope[:, 0], "tab:red", label="gx")
axes[2].plot(timestamp, gyroscope[:, 1], "tab:green", label="gy")
axes[2].plot(timestamp, gyroscope[:, 2], "tab:blue", label="gz")
axes[2].set_ylabel("Degrees/s")
axes[2].grid()
axes[2].legend()

figure, axes = pyplot.subplots(nrows=2, sharex=True)#, gridspec_kw={"height_ratios": [6, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1]})

axes[0].plot(timestamp, accelerometer[:, 0], "tab:red", label="ax_obs")
axes[0].plot(timestamp, accelerometer[:, 1], "tab:green", label="ay_obs")
axes[0].plot(timestamp, accelerometer[:, 2], "tab:blue", label="az_obs")
axes[0].set_ylabel("g")
axes[0].grid()
axes[0].legend()

axes[1].plot(timestamp, accel[:, 0], "tab:red", label="ax")
axes[1].plot(timestamp, accel[:, 1], "tab:green", label="ay")
axes[1].plot(timestamp, accel[:, 2], "tab:blue", label="az")
axes[1].set_ylabel("g")
axes[1].grid()
axes[1].legend()

# Plot Euler angles
figure, axes = pyplot.subplots(nrows=10, sharex=True, gridspec_kw={"height_ratios": [1, 1, 2, 1, 1, 1, 2, 1, 1, 1]})

# Plot initialising flag
plot_bool(axes[1], timestamp, flags[:, 0], "Initialising")

# Plot angular rate recovery flag
plot_bool(axes[2], timestamp, flags[:, 1], "Angular rate recovery")

# Plot acceleration rejection internal states and flag
axes[3].plot(timestamp, internal_states[:, 0], "tab:olive", label="Acceleration error")
axes[3].set_ylabel("Degrees")
axes[3].grid()
axes[3].legend()

plot_bool(axes[4], timestamp, internal_states[:, 1], "Accelerometer ignored")

axes[5].plot(timestamp, internal_states[:, 2], "tab:orange", label="Acceleration recovery trigger")
axes[5].grid()
axes[5].legend()

plot_bool(axes[6], timestamp, flags[:, 2], "Acceleration recovery")

# Plot magnetic rejection internal states and flag
axes[7].plot(timestamp, internal_states[:, 3], "tab:olive", label="Magnetic error")
axes[7].set_ylabel("Degrees")
axes[7].grid()
axes[7].legend()

plot_bool(axes[8], timestamp, internal_states[:, 4], "Magnetometer ignored")

axes[9].plot(timestamp, internal_states[:, 5], "tab:orange", label="Magnetic recovery trigger")
axes[9].grid()
axes[9].legend()

plot_bool(axes[0], timestamp, flags[:, 3], "Magnetic recovery")

pyplot.show(block="no_block" not in sys.argv)  # don't block when script run by CI
