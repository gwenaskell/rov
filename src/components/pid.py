from .imu import RovState
from .classes import Commands, Quaternion, Vec
import numpy as np

# TODO apply more realistic coefficients
MAX_ROT_X = 3 # rad/s
MAX_ROT_Y = 2
MAX_ROT_Z = 1

def bound(val, lim=1):
    if val >= lim:
        return 1.0*lim
    mlim = -1*lim
    if val <= mlim:
        return mlim
    return val

class PID:
    def __init__(self,
                 Kp=1.0,
                 Ki=0.1,
                 Kd=1.0, integral_fade_rate=0.1, max_integral_value=10) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.last_tm = 0
        
        self.last_err = Vec()
        
        # integral of the error in the referential 
        self.err_integ = Vec()
        
        self.last_q = Quaternion()
        
        # unit: s-1. Controls the speed at which the the integral fades over time when the error is zero.
        # a value of 0 means no fade.
        # a value of 1 means a decrease of 1/2 to 1/e=1/2.718 over one second. (faster when dt is smaller. 0.39 for dt = 0.1)
        self.integral_fade_rate = integral_fade_rate
        
        assert max_integral_value >= 1
        self.max_integral_value = max_integral_value

    @staticmethod
    def _compute_w(q1: Quaternion, q2: Quaternion, dt) -> Vec:
        """compute angular speed from the the evolution of a quaternion during dt"""
        # https://mariogc.com/post/angular-velocity-quaternions/
        return (2 / dt) * Vec(
            q1.w*q2.x - q1.x*q2.w - q1.y*q2.z + q1.z*q2.y,
            q1.w*q2.y + q1.x*q2.z - q1.y*q2.w - q1.z*q2.x,
            q1.w*q2.z - q1.x*q2.y + q1.y*q2.x - q1.z*q2.w)

    @staticmethod
    def _command_to_target_speed(cx, cy, cz) -> Vec:
        return Vec(
            MAX_ROT_X*cx,
            MAX_ROT_Y*cy,
            MAX_ROT_Z*cz,
        )

    def apply_correction(self, state: RovState, commands: Commands):
        """apply commands coefficients on cx, cy, cz based on the gap between current angular position and euler commands"""
        
        dt = self.last_tm - state.time
        
        self.last_tm = state.time
        
        w_re = self._compute_w(self.last_q, state.q, dt)
        
        self.last_q = state.q
        
        w_target = self._command_to_target_speed(commands.cx, commands.cy, commands.cz)
        
        err = w_target - w_re
        
        err_var = (err - self.last_err)/dt
        
        self.last_err = err

        # # for the integral to actually make sense, we must compute it in a fixed referential.
        # self.err_integ_ref = self.err_integ_ref + state.q.rotate(err)*dt
        
        # # convert it back to our referential
        # err_integ = state.q.convert_coordinates(self.err_integ_ref)
        
        for i in range(3):
            self.err_integ[i] = bound(self.err_integ[i]/(1 + self.integral_fade_rate*dt) + err[i]*dt, self.max_integral_value)

        
        # TODO check signs
        return Commands(
            fx=commands.fx,
            fz=commands.fz,
            cx=bound(self.Kp*err.x + self.Kd * err_var.x + self.Ki * self.err_integ.x),
            cy=bound(self.Kp*err.y + self.Kd * err_var.y + self.Ki * self.err_integ.y),
            cz=bound(self.Kp*err.z + self.Kd * err_var.z + self.Ki * self.err_integ.z),
            tm_ms=commands.tm_ms
        )

# import time
# import math

# def fexp(i):
#     return 1/(1+math.exp(-i))

# def ftan(i):
#     return math.tanh(i) # plus rapide
