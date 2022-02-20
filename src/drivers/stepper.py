from math import pi
import RPi.GPIO as GPIO
import time

# https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/Documentation/Nema11A4988.md


class StepperMotor:
    def __init__(self, resolution: str = "Full") -> None:
        self.direction_pin = 20
        self.step_pin = 21
        self.mode_pins = (14, 15, 18)

        self.step_position = 0

        self.resolution = resolution

        self.resolutions = {'Full': 1.0,
                            'Half': 1.0/2,
                            '1/4': 1.0/4,
                            '1/8': 1.0/8,
                            '1/16': 1.0/16
                            }

        self.resolutions_gpio = {'Full': (0, 0, 0),
                                 'Half': (1, 0, 0),
                                 '1/4': (0, 1, 0),
                                 '1/8': (1, 1, 0),
                                 '1/16': (1, 1, 1)
                                 }

        self.nb_steps = int(200/self.resolutions[resolution])

        self.angle_by_step = 2.0*pi / self.nb_steps

        # one full rotation will take one second.
        self.min_step_time = 1.0/200 * \
            self.resolutions[resolution]  # to be evaluated

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.mode_pins, GPIO.OUT)
        GPIO.output(self.mode_pins, self.resolutions_gpio[resolution])
        self.spin = True
        GPIO.output(self.direction_pin, self.spin)
        self.last_transition = False

    def angle_to_step_index(self, angle: float) -> int:
        return round(angle*self.nb_steps/(2*pi))

    def get_current_angle(self) -> float:
        return self.step_position * (2.0*pi)/self.nb_steps

    def bring_sally_up(self, clockwise: bool):
        if self.last_transition == True:
            raise Exception("stepper state is already UP")

        self.last_transition = True

        if clockwise != self.step_pin:
            self.spin = clockwise
            GPIO.output(self.direction_pin, self.spin)

        GPIO.output(self.step_pin, True)

        if clockwise:
            self.step_position += 1
        else:
            self.step_position -= 1

    def bring_sally_down(self):
        if self.last_transition == False:
            raise Exception("stepper state is already DOWN")

        self.last_transition = False

        GPIO.output(self.step_pin, False)

    def stop(self):
        GPIO.output(self.step_pin, False)
        GPIO.output(self.direction_pin, False)
        for pin in self.mode_pins:
            GPIO.output(pin, False)


class A4988Nema(object):
    """ Class to control a Nema bi-polar stepper motor with a A4988 also tested with DRV8825"""

    def __init__(self, direction_pin, step_pin, mode_pins, motor_type="A4988"):
        """ class init method 3 inputs
        (1) direction type=int , help=GPIO pin connected to DIR pin of IC
        (2) step_pin type=int , help=GPIO pin connected to STEP of IC
        (3) mode_pins type=tuple of 3 ints, help=GPIO pins connected to
        Microstep Resolution pins MS1-MS3 of IC, can be set to (-1,-1,-1) to turn off
        GPIO resolution.
        (4) motor_type type=string, help=Type of motor two options: A4988 or DRV8825
        """
        self.motor_type = motor_type
        self.direction_pin = direction_pin
        self.step_pin = step_pin

        if mode_pins[0] != -1:
            self.mode_pins = mode_pins
        else:
            self.mode_pins = False

        self.stop_motor = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def motor_stop(self):
        """ Stop the motor """
        self.stop_motor = True

    def resolution_set(self, steptype):
        """ method to calculate step resolution
        based on motor type and steptype"""
        if self.motor_type == "A4988":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (1, 1, 1)}
        elif self.motor_type == "DRV8825":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (0, 0, 1),
                          '1/32': (1, 0, 1)}
        elif self.motor_type == "LV8729":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (0, 0, 1),
                          '1/32': (1, 0, 1),
                          '1/64': (0, 1, 1),
                          '1/128': (1, 1, 1)}
        else:
            print("Error invalid motor_type: {}".format(self.motor_type))
            quit()

        # error check stepmode
        if steptype in resolution:
            pass
        else:
            print("Error invalid steptype: {}".format(steptype))
            quit()

        if self.mode_pins != False:
            GPIO.output(self.mode_pins, resolution[steptype])

    def motor_go(self, clockwise=False, steptype="Full",
                 steps=200, stepdelay=.005, verbose=False, initdelay=.05):
        """ motor_go,  moves stepper motor based on 6 inputs
         (1) clockwise, type=bool default=False
         help="Turn stepper counterclockwise"
         (2) steptype, type=string , default=Full help= type of drive to
         step motor 5 options
            (Full, Half, 1/4, 1/8, 1/16) 1/32 for DRV8825 only
         (3) steps, type=int, default=200, help=Number of steps sequence's
         to execute. Default is one revolution , 200 in Full mode.
         (4) stepdelay, type=float, default=0.05, help=Time to wait
         (in seconds) between steps.
         (5) verbose, type=bool  type=bool default=False
         help="Write pin actions",
         (6) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.
        """
        self.stop_motor = False
        # setup GPIO
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.output(self.direction_pin, clockwise)
        if self.mode_pins != False:
            GPIO.setup(self.mode_pins, GPIO.OUT)

        try:
            # dict resolution
            self.resolution_set(steptype)
            time.sleep(initdelay)

            for i in range(steps):
                if self.stop_motor:
                    raise StopMotorInterrupt
                else:
                    GPIO.output(self.step_pin, True)
                    time.sleep(stepdelay)
                    GPIO.output(self.step_pin, False)
                    time.sleep(stepdelay)
                    if verbose:
                        print("Steps count {}".format(
                            i+1), end="\r", flush=True)

        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib:")
        except StopMotorInterrupt:
            print("Stop Motor Interrupt : RpiMotorLib: ")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")
        else:
            # print report status
            if verbose:
                print("\nRpiMotorLib, Motor Run finished, Details:.\n")
                print("Motor type = {}".format(self.motor_type))
                print("Clockwise = {}".format(clockwise))
                print("Step Type = {}".format(steptype))
                print("Number of steps = {}".format(steps))
                print("Step Delay = {}".format(stepdelay))
                print("Intial delay = {}".format(initdelay))
                print("Size of turn in degrees = {}"
                      .format(degree_calc(steps, steptype)))
        finally:
            # cleanup
            GPIO.output(self.step_pin, False)
            GPIO.output(self.direction_pin, False)
            if self.mode_pins != False:
                for pin in self.mode_pins:
                    GPIO.output(pin, False)
