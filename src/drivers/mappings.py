
from typing import Tuple, Union
from .gpio import *

ThrusterLeft = PWMDriver()


ThrusterRight = PWMDriver()


ThrusterTail = PWMDriver()


class _Stepper:
    def __init__(self) -> None:
        self.direction: Final[PinOutput] = PinOutput()
        self.step: Final[PinOutput] = PinOutput()


StepperLeft = _Stepper()

StepperRight = _Stepper()


SwitchLeft = PinOutput()

SwitchRight = PinOutput()

SwitchTail = PinOutput()

Lights = PWMDriver()


Imu = I2CDriver(0x68)

Compass = I2CDriver(0x0)

PressureIn = I2CDriver(0x0)


GPS = UARTDriver()


A2C = SPIDriver()

Arduino = SPIDriver()


_SDA_ = (Imu.sda, Compass.sda, PressureIn.sda)
_SCL_ = (Imu.scl, Compass.scl, PressureIn.scl)

_Unused_ = None

MAPPINGS: MappingProxyType[int, Union[None, PinDriver, Tuple[PinDriver, ...]]] = MappingProxyType({
    1:  None,                      2: None,
    3:  _SDA_,                     4: None,
    5:  _SCL_,                     6: None,
    7:  Lights,                    8: GPS.tx,
    9:  None,                     10: GPS.rx,
    11: ThrusterLeft,             12: _Unused_,
    13: ThrusterRight,            14: None,
    15: ThrusterTail,             16: _Unused_,
    17: None,                     18: _Unused_,
    19: (A2C.mosi, Arduino.mosi), 20: None,
    21: (A2C.miso, Arduino.miso), 22: _Unused_,
    23: (A2C.sck, Arduino.sck),   24: A2C.ce,
    25: None,                     26: Arduino.ce,
    27: None,                     28: None,
    29: StepperLeft.direction,    30: None,
    31: StepperLeft.step,         32: _Unused_,
    33: StepperRight.direction,   34: None,
    35: StepperRight.step,        36: SwitchLeft,
    37: _Unused_,                 38: SwitchRight,
    39: None,                     40: SwitchTail,
})


def allocate():
    for i in MAPPINGS:
        drivers = MAPPINGS[i]
        if drivers is None:
            continue
        if isinstance(drivers, PinDriver):
            drivers = (drivers,)
        for driv in drivers:
            if driv.pin is not undefined_pin:
                raise RuntimeError("pin driver is connected twice: at %s and %s" % (
                    driv.pin.physical_pin, i))
            driv.pin = PINS[i]
            if driv.pin.typ != PinType.GPIO:
                raise RuntimeError(
                    "pin driver has been allocated on a non-gpio pin at position %s" % i)

            if driv._bound_bcm is not None:
                if driv.pin.bcm_role is None:
                    raise RuntimeError(
                        "pin driver at pos %s is bound to a role but corresponding pin has no role" % i)
                if driv.pin.bcm_role is not driv._bound_bcm:
                    raise RuntimeError(
                        "bcm role mismatch between pin and driver at position %s" % i)


allocate()
