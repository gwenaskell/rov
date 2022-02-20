from src.components.pilot import Commands
from src.drivers.multimeter import Multimeter
from src.drivers.pressure import PressureSensor
from src.drivers.thermo import Thermometer
from src.drivers.water import WaterSensor


class Safety:
    def __init__(self) -> None:
        self.multimeter = Multimeter()
        self.pressure = PressureSensor()
        self.thermometer = Thermometer()
        self.water_sensor = WaterSensor()

    def rectify(self, commands: Commands) -> Commands:
        return commands
