from src.drivers.mappings import Switch1


class Switch():
    def __init__(self) -> None:
        self.on = False

        self.driver = Switch1

    def set(self, on: bool):
        self.driver.write(on)
