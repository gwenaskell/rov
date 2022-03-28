from .stepper import StepperMotor
from .thruster import Thruster
from .accelero import Accelerometer
from .compass import Compass
from .plot import plotter
from ...drivers import thruster, stepper, accelero, compass


def mock():
    thruster.Thruster = Thruster
    stepper.StepperMotor = StepperMotor
    accelero.Accelerometer = Accelerometer
    compass.Compass = Compass
    print("mocked ROV drivers")
