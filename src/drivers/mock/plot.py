from multiprocessing import Queue
import multiprocessing
from matplotlib.patches import Arrow
import matplotlib.pyplot as plt
from re import T
from threading import Thread
from math import pi
import time
from typing import Any
import matplotlib

# https://stackoverflow.com/questions/36181316/python-matplotlib-plotting-in-another-process


class Plotter:
    def __init__(self):
        self.left_ax: Any = None
        self.right_ax: Any = None
        self.tail_ax: Any = None
        self.fig: Any = None

        self.tau_l, self.phi_l, self.tau_r, self.phi_r, self.tau_t = 0.0, 0.0, 0.0, 0.0, 0.0

        self.stop = False

    def inputs_loop(self, p: Queue):
        try:
            count = 0
            phi_l = self.phi_l
            tm = time.time()
            while not self.stop:
                time.sleep(0.1)
                
                ok = False
                if self.phi_l != phi_l:
                    tm = round(time.time()*1000)
                    ok = True
                p.put([self.tau_l, self.phi_l, self.tau_r, self.phi_r, self.tau_t, count])
                if ok:
                    print(self.phi_l-phi_l, round(time.time()*1000)-tm)
                    phi_l = self.phi_l
                count += 1
        finally:
            p.put("stop")

    def start_display(self):
        print("starting display")
        """start_display must be calls from the same process which runs the thrusters and steppers"""
        queue = Queue(maxsize=1)
        p = multiprocessing.Process(target=self.plot, args=(queue,))
        p.start()
        Thread(target=self.inputs_loop, args=(queue,)).start()

    def stop_display(self):
        """stop_display must be run from the same process which called start_display"""
        self.stop = True

    def plot(self, p: Queue):
        # plt.ion()  # interactive mode
        # radar green, solid grid lines
        plt.rc('grid', color='#316931', linewidth=1, linestyle='-')
        plt.rc('xtick', labelsize=15)
        plt.rc('ytick', labelsize=15)

        # make a square figure
        self.fig = plt.figure(figsize=(20, 12))
        self.left_ax = self.fig.add_axes([0.05, 0.3, 0.4, 0.6], polar=True)
        self.right_ax = self.fig.add_axes([0.55, 0.3, 0.4, 0.6], polar=True)
        self.tail_ax = self.fig.add_axes(
            [0.48, 0.3, 0.04, 0.5], xlim=(-0.1, 0.1), ylim=(-100, 100))

        r = [0, 100]
        y = [0, 3.14159]
        self.left_ax.plot(y, r, color='#000000', lw=1)
        self.left_ax.set_rmax(100)
        self.right_ax.plot(y, r, color='#000000', lw=1)
        self.right_ax.set_rmax(100)
        plt.grid(True)

        self.left_vect = self.make_arrow(self.left_ax, 0.0, 0.0)
        self.right_vect = self.make_arrow(self.right_ax, 0.0, 0.0)
        self.tail_vect = self.make_tail_arrow(0)

        # self.left_vect_re = make_arrow(left_ax, 0, 0)
        # self.right_vect_re = make_arrow(left_ax, 0, 0)
        self.fig.canvas.draw()

        self.loop(p)

    def loop(self, p: Queue):
        plt.show(block=False)
        # plt.draw()
        ok = True
        while True:
            try:
                data = p.get()
                if data == "stop":
                    break
                tau_l, phi_l, tau_r, phi_r, tau_t, count = data[0], data[1], data[2], data[3], data[4], data[5]

                
                if ok and tau_l == 100:
                    ok = False
                    # print("max")
                else:
                    ok = True

                self.left_vect.remove()
                self.right_vect.remove()
                self.tail_vect.remove()

                self.left_vect = self.make_arrow(
                    self.left_ax, tau_l, phi_l, False)
                self.right_vect = self.make_arrow(
                    self.right_ax, tau_r, phi_r, False)
                self.tail_vect = self.make_tail_arrow(tau_t)

                # self.left_ax.relim()
                # self.right_ax.relim()
                # self.tail_ax.relim()

                # self.left_ax.autoscale_view(True, True, True)
                # self.right_ax.autoscale_view(True, True, True)
                # self.tail_ax.autoscale_view(True, True, True)
                self.fig.canvas.draw()
            except:
                break

        print("closing plot")
        plt.close()

    def make_arrow(self, axis, tau, phi, real=False):
        (facecolor, edgecolor, alpha) = ('blue', 'blue',
                                         0.3) if real else ('green', 'black', 0.6)
        if tau >= 0:
            return axis.add_patch(Arrow(phi-0.5*3.14, 0.001, 0, tau, alpha=alpha, width=0.2,
                                        edgecolor=edgecolor, facecolor=facecolor, lw=3, zorder=5))
        else:
            return axis.add_patch(Arrow(phi-0.5*3.14, abs(tau), 0, 0.001 + tau, alpha=alpha, width=0.4,
                                        edgecolor=edgecolor, facecolor=facecolor, lw=3, zorder=5))

    def make_tail_arrow(self, tau, real=False):
        (facecolor, edgecolor, alpha) = ('blue', 'blue',
                                         0.3) if real else ('green', 'black', 0.6)
        return self.tail_ax.add_patch(Arrow(0, 0, 0.0, tau, alpha=alpha, width=0.1,
                                            edgecolor=edgecolor, facecolor=facecolor, lw=3, zorder=5))


plotter = Plotter()


# fig.savefig("arrow_in_polar_plot_at_zero_degree.png")
