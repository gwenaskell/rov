from dataclasses import dataclass

from src.drivers.mappings import PressureIn


class PressureSensor:
    def __init__(self) -> None:
        self.driver = PressureIn

    def get_pressure(self) -> int:
        return 0
