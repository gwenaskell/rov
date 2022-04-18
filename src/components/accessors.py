import os

if os.getenv("ONBOARD"):
    from ..drivers.accelero import IMU
    from ..drivers.compass import Compass
    from ..drivers.thruster import Thruster
    from ..drivers.stepper import StepperMotor
    from ..drivers.analog import AnalogChans, AnalogReader
    from ..drivers.arduino import Arduino
else:
    from ..drivers.mock.accelero import IMU
    from ..drivers.mock.compass import Compass
    from ..drivers.mock.stepper import StepperMotor
    from ..drivers.mock.thruster import Thruster
