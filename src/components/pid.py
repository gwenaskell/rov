from .imu import RovState
from .classes import Commands, Quaternion, Vec
from .settings import PIDSettings


def bound_round(val, lim: float = 1.0):
    if val >= lim:
        return 1.0 * lim
    mlim = -1 * lim
    if val <= mlim:
        return mlim
    return round(val * 100) / 100.0


class PID:
    def __init__(self) -> None:
        self.last_tm = 0

        self.last_err = Vec()

        # integral of the error in the referential
        self.err_integ = Vec()

        self.last_q = Quaternion()

    @property
    def params(self) -> PIDSettings:
        return PIDSettings.get()

    @staticmethod
    def _compute_w(q1: Quaternion, q2: Quaternion, dt) -> Vec:
        """compute angular speed from the the evolution of a quaternion during dt"""
        # https://mariogc.com/post/angular-velocity-quaternions/
        return (2 / dt) * Vec(
            q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y,
            q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x,
            q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w,
        )

    def _command_to_target_speed(self, cx, cy, cz) -> Vec:
        p = self.params
        return Vec(
            p.max_rot_x * cx,
            p.max_rot_y * cy,
            p.max_rot_z * cz,
        )

    def apply_correction(self, state: RovState, commands: Commands):
        """apply commands coefficients on cx, cy, cz based on the gap between current angular position and euler commands"""

        dt = state.time - self.last_tm

        self.last_tm = state.time

        w_re = self._compute_w(self.last_q, state.q, dt)

        self.last_q = state.q

        w_target = self._command_to_target_speed(commands.cx, commands.cy, commands.cz)

        err = w_target - w_re

        err_var = (err - self.last_err) / dt

        self.last_err = err

        # # for the integral to actually make sense, we must compute it in a fixed referential.
        # self.err_integ_ref = self.err_integ_ref + state.q.rotate(err)*dt

        # # convert it back to our referential
        # err_integ = state.q.convert_coordinates(self.err_integ_ref)

        for i in range(3):
            self.err_integ[i] = bound_round(
                self.err_integ[i] / (1 + self.params.integral_fade_rate * dt) + err[i] * dt,
                self.params.max_integral_value,
            )

        if not self.params.on:
            return commands

        Kp, Ki, Kd = self.params.Kp, self.params.Ki, self.params.Kd
        # TODO check signs
        return Commands(
            fx=commands.fx,
            fz=commands.fz,
            cx=bound_round(Kp * err.x + Kd * err_var.x + Ki * self.err_integ.x),
            cy=bound_round(Kp * err.y + Kd * err_var.y + Ki * self.err_integ.y),
            cz=bound_round(Kp * err.z + Kd * err_var.z + Ki * self.err_integ.z),
            tm_ms=commands.tm_ms,
            surface=commands.surface,
        )


# import time
# import math

# def fexp(i):
#     return 1/(1+math.exp(-i))

# def ftan(i):
#     return math.tanh(i) # plus rapide
