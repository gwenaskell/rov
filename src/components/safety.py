from src.components.pilot import Commands
from src.drivers.pressure import PressureSensor
from src.drivers.thermo import Thermometer


class Safety:
    def __init__(self) -> None:
        self.pressure = PressureSensor()
        self.thermometer = Thermometer()

    def rectify(self, commands: Commands) -> Commands:
        return commands
