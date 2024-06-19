import os

plotter = None

if os.getenv("ONBOARD"):
    from ..drivers.accelero import IMU, AccelValue
    from ..drivers.compass import Compass, CompassValue
    from ..drivers.thruster import Thruster
    from ..drivers.stepper import StepperMotor
    from ..drivers.analog import AnalogChans, AnalogReader
    from ..drivers.arduino import Arduino
else:
    from ..drivers.mock.accelero import IMU, AccelValue
    from ..drivers.mock.compass import Compass, CompassValue
    from ..drivers.mock.thruster import Thruster
    from ..drivers.mock.stepper import StepperMotor
    from ..drivers.mock.analog import AnalogChans, AnalogReader
    from ..drivers.mock.arduino import Arduino
    from ..drivers.mock.plot import plotter


class Display():
    def __init__(self):
        pass

    def __enter__(self):
        if plotter is not None:
            plotter.start_display()
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        if plotter is not None:
            print("stopping display")
            plotter.stop_display()
            print("stopped display")
