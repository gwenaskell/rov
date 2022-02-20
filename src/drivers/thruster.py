
import pigpio  # importing GPIO library
import os  # importing os library so as to communicate with the system
import time  # importing time library to make Rpi wait because its too impatient


# TODO  sudo pigpio should be executed before importing it

ESC = 4  # Connect the ESC in this GPIO pin


class Thruster:
    def __init__(self) -> None:
        self.max_value = 2000.0
        self.min_value = 700.0
        self.zero_value = 1000.0
        self.pi = pigpio.pi()
        self.armed = False
        self.ratio = 0

    def set_pwm(self, ratio: float):
        if not self.armed:
            raise Exception("thruster not armed")
        if self.ratio == ratio:
            return
        self.ratio = ratio
        speed = ratio * (self.max_value-self.zero_value) + \
            self.zero_value  # TODO handle negatives
        self.pi.set_servo_pulsewidth(ESC, speed)

    def arm_thruster(self):
        self.pi.set_servo_pulsewidth(ESC, 0)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(ESC, self.max_value)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(ESC, self.min_value)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(ESC, self.zero_value)
        self.armed = True

    # This will stop every action your Pi is performing for ESC ofcourse.
    def stop(self):
        self.pi.set_servo_pulsewidth(ESC, 0)
        self.pi.stop()
