from math import sin, cos, atan, acos, pi

from src.components.pilot import Pilot
from src.components.classes import Commands, Status, ThrusterState, RovState, Quaternion

def bound(x):
    if x > 1:
        return 1.0
    if x < -1:
        return -1.0
    return x


class ControlCommand:
    def __init__(self):
        self.eps = 0.15
        self.forward = True
        self.reverse_efficiency = 0.8
        self.forward_threshold = 40/180*pi + pi/2
        self.reverse_threshold = 50/180*pi + pi/2
        
        self.tau_l, self.phi_l, self.tau_r, self.phi_r, self.tau_tail = 0, 0, 0, 0, 0
        
        self.pilot = Pilot()
        self.pilot.status = Status.RUNNING
        self.pilot.states_proxy = {
            "status": Status.STOPPED,
            "targets": {
                "tail_thrust": 0,
                "left_state": None,
                "right_state": None,
            }
        }

        self.update(0, 0, 0, 0, 0, Quaternion())
    
    def _steps_to_angle(self, steps):
        return 2*pi*steps / self.pilot.nb_stepper_steps

    def update(self, f_x, f_z, c_x, c_y, c_z, q):
        self.pilot.apply_setpoints(
            Commands(fx=f_x, fz=f_z, cx=c_x, cy=c_y, cz=c_z, tm_ms=0, surface=False),
            state=RovState(ax=0, ay=0, az=0, q=q, wx=0, wy=0, wz=0),
        )

        ls: ThrusterState = self.pilot.states_proxy['targets']['left_state']
        lr: ThrusterState = self.pilot.states_proxy['targets']['right_state']
        self.tau_tail: ThrusterState = self.pilot.states_proxy['targets']['tail_thrust']
        
        self.tau_l, self.phi_l = ls.tau, self._steps_to_angle(ls.pos)
        self.tau_r, self.phi_r = lr.tau, self._steps_to_angle(lr.pos)
        
        # self.tau_l, self.phi_l, self.tau_r, self.phi_r = self._get_commands(f_x, c_x, c_y, c_z)

    def _get_commands(self, f_x, c_x, c_y, c_z):
        f_x_l = bound(f_x - c_x)
        f_x_r = bound(f_x + c_x)

        c_y_l = bound(c_y - c_z)
        c_y_r = bound(c_y + c_z)

        tau_l, phi_l = self._get_symmetric_commands(f_x_l, c_y_l)
        tau_r, phi_r = self._get_symmetric_commands(f_x_r, c_y_r)

        return tau_l, phi_l, tau_r, phi_r
    
    def get_revert(self, phi, tau):
        if phi > 0:
            phi = phi - pi
        else:
            phi = phi + pi
        tau = -tau/self.reverse_efficiency # boost, since reverse rotation is less efficient
        return phi, tau

    def _get_symmetric_commands(self, f_x, c_y):
        c_y = c_y*abs(c_y)/2

        # Get axial commands from f_x
        tau = (self.eps**2 + (1.0-self.eps**2)*f_x**2)**0.5
        if self.eps > 0:
            phi = atan(f_x/self.eps)
        else:
            phi = 0

        # Get full commands from axial commands and c_y
        div = cos(phi) * tau - c_y
        tau = (tau**2 - 2*tau*c_y*cos(phi)+c_y**2)**0.5
        if div != 0:
            if div > 0: # phi goes to the upper part of the area
                phi = atan(sin(phi) * tau / div)
            else:
                if phi > 0:
                    phi = atan(sin(phi) * tau / div) + pi
                else:
                    phi = atan(sin(phi) * tau / div) - pi
        else:
            if phi > 0:
                phi = pi / 2
            elif phi < 0:
                phi = - pi /2


        if self.forward:
            # should we use reverse?
            if abs(phi) > self.reverse_threshold:
                phi, tau = self.get_revert(phi, tau)
                self.forward = False

                # tau becomes negative

        else:
            # should we use forward?
            if abs(phi) < self.forward_threshold:
                self.forward = True
            else:
                phi, tau = self.get_revert(phi, tau)
        
        tau = bound(tau)

        return tau, phi