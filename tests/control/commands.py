from math import sin, cos, atan, acos, pi

from src.components.pilot import Pilot
from src.components.classes import Commands, NamespaceProxy, Status, TargetsProxy, ThrusterState, ThrusterVector, RovState, Quaternion

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
        self.tau_l, self.phi_l, self.tau_r, self.phi_r = self._get_commands(0, 0, 0, 0)
        
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
    
    def _steps_to_angle(self, steps):
        return 2*pi*steps / self.pilot.nb_stepper_steps

    def update(self, c_x, c_delta, c_z, c_theta):
        self.pilot.apply_setpoints(
            Commands(fx=c_x, fz=0, cx=c_delta, cz=c_z, cy=c_theta, tm_ms=0),
            state=RovState(ax=0, ay=0, az=0, q=Quaternion(), wx=0, wy=0, wz=0),
        )
        
        ls: ThrusterState = self.pilot.states_proxy['targets']['left_state']
        lr: ThrusterState = self.pilot.states_proxy['targets']['right_state']
        
        self.tau_l, self.phi_l = ls.tau, self._steps_to_angle(ls.pos)
        self.tau_r, self.phi_r = lr.tau, self._steps_to_angle(lr.pos)
        
        # self.tau_l, self.phi_l, self.tau_r, self.phi_r = self._get_commands(c_x, c_delta, c_z, c_theta)

    def _get_commands(self, c_x, c_delta, c_z, c_theta):
        c_x_l = bound(c_x - c_delta)
        c_x_r = bound(c_x + c_delta)

        c_z_l = bound(c_z - c_theta)
        c_z_r = bound(c_z + c_theta)

        tau_l, phi_l = self._get_symmetric_commands(c_x_l, c_z_l)
        tau_r, phi_r = self._get_symmetric_commands(c_x_r, c_z_r)

        return tau_l, phi_l, tau_r, phi_r
    
    def get_revert(self, phi, tau):
        if phi > 0:
            phi = phi - pi
        else:
            phi = phi + pi
        tau = -tau/self.reverse_efficiency # boost, since reverse rotation is less efficient
        return phi, tau

    def _get_symmetric_commands(self, c_x, c_z):
        c_z = c_z*abs(c_z)/2

        # Get axial commands from c_x
        tau = (self.eps**2 + (1.0-self.eps**2)*c_x**2)**0.5
        if self.eps > 0:
            phi = atan(c_x/self.eps)
        else:
            phi = 0

        # Get full commands from axial commands and c_z
        div = cos(phi) * tau - c_z
        tau = (tau**2 - 2*tau*c_z*cos(phi)+c_z**2)**0.5
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