from enum import Enum
from optparse import Option
from types import MappingProxyType
from typing import Final, List, Optional, cast
import adafruit_blinka.board.raspberrypi.raspi_40pin as bcm
from adafruit_blinka.microcontroller.bcm283x.pin import Pin as BcmPin
import RPi.GPIO
import busio
import board
from pulseio import PWMOut

# Docs:
#
# https://docs.circuitpython.org/projects/blinka/en/latest/index.html
# https://github.com/adafruit/Adafruit_CircuitPython_MPU6050


class PinType(Enum):
    GPIO = 0
    GND = 1
    POW = 2


class Pin:
    def __init__(self, pin: int, bcm_pin: BcmPin, typ: PinType = PinType.GPIO, bcm_role: Optional[BcmPin] = None) -> None:
        self.typ = typ
        # bcm pin number
        self.id: int = cast(int, self.bcm_pin.id)
        # physical pin number
        self.physical_pin = pin
        # bcm pin object
        self.bcm_pin = bcm_pin
        # bcm pin object with specific role
        self.bcm_role = bcm_role


_no_gpio = BcmPin(-1)

Ground: Final[Pin] = Pin(0, _no_gpio, PinType.GND)
Power3v: Final[Pin] = Pin(0, _no_gpio, PinType.POW)
Power5v: Final[Pin] = Pin(0, _no_gpio, PinType.POW)


def GPIO(bcm_number: BcmPin):
    return Pin(0, bcm_number)


def SPI(bcm_number: BcmPin, role: BcmPin):
    return Pin(0, bcm_number, bcm_role=role)


def EEPROM(bcm_number: BcmPin):
    return Pin(0, bcm_number)


def I2C(bcm_number: BcmPin, role: BcmPin):
    return Pin(0, bcm_number, bcm_role=role)


def UART(bcm_number: BcmPin, role: BcmPin):
    return Pin(0, bcm_number, bcm_role=role)


PINS: MappingProxyType[int, Pin] = MappingProxyType({
    1:  Power3v,                 2: Power5v,
    3:  I2C(bcm.D2, bcm.SDA),    4: Power5v,
    5:  I2C(bcm.D3, bcm.SCL),    6: Ground,
    7:  GPIO(bcm.D4),            8: UART(bcm.D14, bcm.TX),
    9:  Ground,                 10: UART(bcm.D15, bcm.RX),
    11: GPIO(bcm.D17),          12: GPIO(bcm.D18),
    13: GPIO(bcm.D27),          14: Ground,
    15: GPIO(bcm.D22),          16: GPIO(bcm.D23),
    17: Power3v,                18: GPIO(bcm.D24),
    19: SPI(bcm.D10, bcm.MOSI), 20: Ground,
    21: SPI(bcm.D9, bcm.MISO),  22: GPIO(bcm.D25),
    23: SPI(bcm.D11, bcm.SCK),  24: SPI(bcm.D8, bcm.CE0),
    25: Ground,                 26: SPI(bcm.D7, bcm.CE1),
    27: EEPROM(bcm.D0),         28: EEPROM(bcm.D1),  # do not connect
    29: GPIO(bcm.D5),           30: Ground,
    31: GPIO(bcm.D6),           32: GPIO(bcm.D12),
    33: GPIO(bcm.D13),          34: Ground,
    35: GPIO(bcm.D19),          36: GPIO(bcm.D16),
    37: GPIO(bcm.D26),          38: GPIO(bcm.D20),
    39: Ground,                 40: GPIO(bcm.D21),
})


_nb_pins = 40

_nb_gpio = 28


def validate():
    array: List[Optional[Pin]] = [None]*_nb_pins
    gpios: List[Optional[Pin]] = [None]*_nb_gpio
    for i in PINS:
        if i < 1 or i > _nb_pins:
            raise RuntimeError("illegal pin number:", i)
        if array[i-1] is not None:
            raise RuntimeError("pin number attributed twice:", i)
        pin = PINS[i]
        if pin.typ == "gpio":
            pin.physical_pin = i

            assert isinstance(pin.bcm_pin.id, int)

            if gpios[pin.bcm_pin.id] is not None:
                raise RuntimeError(
                    "bcm number attributed twice:", pin.bcm_pin.id)
            gpios[pin.bcm_pin.id] = pin
        elif not (pin is Ground or pin is Power3v or pin is Power5v):
            raise RuntimeError("unexpected pin value at", i, ":", pin)

        array[i-1] = pin
    for i in range(_nb_pins):
        if array[i] is None:
            raise RuntimeError("undeclared pin: ", i+1)
    for i in range(_nb_gpio):
        if gpios[i] is None:
            raise RuntimeError("undeclared gpio: ", i)


validate()


class PinMode(Enum):
    IN = 0
    OUT = 1


undefined_pin = Pin(0, _no_gpio)


class PinDriver:
    def __init__(self, bound: Optional[BcmPin] = None) -> None:
        self.pin: Pin = undefined_pin  # waiting for allocation
        self._bound_bcm = bound
        self.ready = False

    def _setup(self, mode):
        if self.pin is undefined_pin:
            raise RuntimeError("pin driver not allocated")
        RPi.GPIO.setup(self.pin.id, mode)
        self.ready = True

    def assert_ready(self):
        if not self.ready:
            raise RuntimeError("pin driver not set up")


class PinInput(PinDriver):
    def setup(self):
        self._setup(RPi.GPIO.IN)

    def read(self) -> bool:
        self.assert_ready()

        return RPi.GPIO.input(self.pin.id)


class PinOutput(PinDriver):
    def setup(self):
        self._setup(RPi.GPIO.OUT)

    def write(self, state: bool):
        self.assert_ready()

        RPi.GPIO.output(self.pin.id, state)


class PWMDriver(PinDriver):
    def __init__(self, bound: Optional[BcmPin] = None) -> None:
        super().__init__(bound)

        self.pwm: PWMOut

    def setup(self):
        if self.pin is undefined_pin:
            raise RuntimeError("pin driver not allocated")
        self.pwm = PWMOut(self.pin.id)
        self.ready = True

    def stop(self):
        self.set_pulsewidth(0)
        self.pwm.deinit()

    def set_pulsewidth(self, val: int):
        self.pwm.frequency = val


class I2CDriver:
    addresses: List[int] = []

    def __init__(self, address: int) -> None:
        if address in I2CDriver.addresses:
            raise RuntimeError("error: I2C address %s used twice" % address)
        I2CDriver.addresses.append(address)

        self.address = address

        self.sda: Final[PinDriver] = PinDriver(bound=bcm.SDA)
        self.scl: Final[PinDriver] = PinDriver(bound=bcm.SCL)

        self.bus = busio.I2C(scl=bcm.SCL, sda=bcm.SDA)


class SPIDriver:
    _used_selectors = []

    def __init__(self) -> None:
        self.mosi: Final[PinDriver] = PinDriver(bound=bcm.MOSI)
        self.miso: Final[PinDriver] = PinDriver(bound=bcm.MISO)
        self.sck: Final[PinDriver] = PinDriver(bound=bcm.SCK)

        self.ce: Final[PinDriver] = PinDriver()

        self.bus = busio.SPI(clock=bcm.SCK, MISO=bcm.MISO, MOSI=bcm.MOSI)


class UARTDriver:
    def __init__(self) -> None:
        self.tx: Final[PinDriver] = PinDriver(bound=bcm.TX)
        self.rx: Final[PinDriver] = PinDriver(bound=bcm.RX)
