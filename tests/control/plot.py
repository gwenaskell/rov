from threading import Thread
from math import cos, sin, pi
import time
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Arrow
from commands import ControlCommand

print("matplotlib.__version__   = ", matplotlib.__version__)
print("matplotlib.get_backend() = ", matplotlib.get_backend())


# radar green, solid grid lines
plt.rc('grid', color='#316931', linewidth=1, linestyle='-')
plt.rc('xtick', labelsize=15)
plt.rc('ytick', labelsize=15)

# make a square figure
fig = plt.figure(figsize=(10, 7))
left_ax = fig.add_axes([0.05, 0.3, 0.4, 0.6], polar=True)
right_ax = fig.add_axes([0.55, 0.3, 0.4, 0.6], polar=True)
tail_ax = fig.add_axes([0.48, 0.3, 0.04, 0.5], xlim=(-0.1, 0.1), ylim=(-1, 1))

r = [0, 1]
y = [0, 3.14159]
left_ax.plot(y, r, color='#000000', lw=1)
left_ax.set_rmax(1.0)
right_ax.plot(y, r, color='#000000', lw=1)
right_ax.set_rmax(1.0)
plt.grid(True)


def make_arrow(axis, tau, phi, real=False):
    (facecolor, edgecolor, alpha) = ('blue', 'blue',
                                     0.3) if real else ('green', 'black', 0.6)
    if tau >= 0:
        return axis.add_patch(Arrow(phi-0.5*3.14, 0.001, 0, tau, alpha=alpha, width=0.2,
                                    edgecolor=edgecolor, facecolor=facecolor, lw=3, zorder=5))
    else:
        return axis.add_patch(Arrow(phi-0.5*3.14, abs(tau), 0, 0.001 + tau, alpha=alpha, width=0.4,
                                    edgecolor=edgecolor, facecolor=facecolor, lw=3, zorder=5))


def make_tail_arrow(tau, real=False):
    (facecolor, edgecolor, alpha) = ('blue', 'blue',
                                     0.3) if real else ('green', 'black', 0.6)
    return tail_ax.add_patch(Arrow(0, 0, 0.0, tau, alpha=alpha, width=0.1,
                                   edgecolor=edgecolor, facecolor=facecolor, lw=3, zorder=5))


class Controller:
    def __init__(self):
        self.servo_speed = pi  # rad by seconds
        self.step_time = 0.05
        self.c_x = 0
        self.c_delta = 0
        self.c_z = 0
        self.c_theta = 0
        self.command = ControlCommand()

        self.left_vect = make_arrow(
            left_ax, self.command.tau_l, self.command.phi_l)
        self.right_vect = make_arrow(
            right_ax, self.command.tau_r, self.command.phi_r)
        self.tail_vect = make_tail_arrow(0.2)

        self.phi_l_re, self.tau_l_re = self.command.phi_l, self.command.tau_l
        self.phi_r_re, self.tau_r_re = self.command.phi_r, self.command.tau_r

        self.left_vect_re = make_arrow(
            left_ax, self.tau_l_re, self.phi_l_re, True)
        self.right_vect_re = make_arrow(
            right_ax, self.tau_r_re, self.phi_r_re, True)
        fig.canvas.draw_idle()

        self.stop = False
        self.thread = Thread(target=self.move)

        self.thread.start()

    def update(self):
        self.command.update(self.c_x, self.c_delta, self.c_z, self.c_theta)
        self.left_vect.remove()
        self.right_vect.remove()
        self.left_vect = make_arrow(
            left_ax, self.command.tau_l, self.command.phi_l, True)
        self.right_vect = make_arrow(
            right_ax, self.command.tau_r, self.command.phi_r, True)
        fig.canvas.draw_idle()

    def update_real_values(self, phi, tau, phi_re, tau_re, speed_factor):
        chgd = False
        opposite_thrust_fact = 1.0
        if phi_re != phi or tau_re != tau:
            chgd = True
            if phi_re != phi:
                gap = phi - phi_re

                incr = min(self.servo_speed*self.step_time, abs(gap)
                           ) * (1 if gap > 0 else -1) * speed_factor
                phi_re = phi_re + incr

                if abs(gap) < pi:
                    if tau_re * tau > 0:
                        tau_re = abs(tau * tau_re) * sin(gap) / \
                            (tau_re * sin(incr) + tau * sin(gap - incr))
                    elif cos(gap) > 0:
                        tau_re = tau * cos(gap)
                        opposite_thrust_fact = cos(gap)
                    else:
                        tau_re = 0
                        opposite_thrust_fact = 0
                else:
                    tau_re = 0
                    opposite_thrust_fact = 0
            else:
                tau_re = tau
        return phi_re, tau_re, chgd, opposite_thrust_fact

    def move(self):
        while not self.stop:
            gap_l, gap_r = abs(
                self.command.phi_l-self.phi_l_re), abs(self.command.phi_r-self.phi_r_re)
            speed_fact_l, speed_fact_r = 1.0, 1.0
            if gap_l > 0 and gap_r > 0:
                speed_fact_l, speed_fact_r = min(
                    gap_l/gap_r, 1.0), min(gap_r/gap_l, 1.0)

            self.phi_l_re, self.tau_l_re, chgd_l, pow_fact_r = self.update_real_values(
                self.command.phi_l, self.command.tau_l, self.phi_l_re, self.tau_l_re, speed_fact_l)

            self.phi_r_re, self.tau_r_re, chgd_r, pow_fact_l = self.update_real_values(
                self.command.phi_r, self.command.tau_r, self.phi_r_re, self.tau_r_re, speed_fact_r)

            self.tau_l_re *= pow_fact_l
            self.tau_r_re *= pow_fact_r

            if chgd_l:
                self.left_vect_re.remove()
                self.left_vect_re = make_arrow(
                    left_ax, self.tau_l_re, self.phi_l_re)

            if chgd_r:
                self.right_vect_re.remove()
                self.right_vect_re = make_arrow(
                    right_ax, self.tau_r_re, self.phi_r_re)

            fig.canvas.draw_idle()
            time.sleep(self.step_time)

    def updatec_x(self, val):
        self.c_x = val
        self.update()

    def updatec_delta(self, val):
        self.c_delta = val
        self.update()

    def updatec_z(self, val):
        self.c_z = val
        self.update()

    def updatec_theta(self, val):
        self.c_theta = val
        self.update()


ctrl = Controller()

ax_c_x = plt.axes([0.3, 0.2, 0.35, 0.03])
slid_c_x = Slider(ax_c_x, 'c_x', -1, 1, valinit=0, valstep=0.04)
slid_c_x.on_changed(ctrl.updatec_x)

ax_c_delta = plt.axes([0.3, 0.15, 0.35, 0.03])
slid_c_delta = Slider(ax_c_delta, 'c_delta', -1, 1, valinit=0, valstep=0.04)
slid_c_delta.on_changed(ctrl.updatec_delta)

ax_c_z = plt.axes([0.3, 0.1, 0.35, 0.03])
slid_c_z = Slider(ax_c_z, 'c_z', -1, 1, valinit=0, valstep=0.04)
slid_c_z.on_changed(ctrl.updatec_z)

ax_c_theta = plt.axes([0.3, 0.05, 0.35, 0.03])
slid_c_theta = Slider(ax_c_theta, 'c_theta', -1, 1, valinit=0, valstep=0.04)
slid_c_theta.on_changed(ctrl.updatec_theta)

# fig.savefig("arrow_in_polar_plot_at_zero_degree.png")
try:
    plt.show()
except KeyboardInterrupt:
    ctrl.stop = True
    ctrl.thread.join(timeout=1)
    raise
