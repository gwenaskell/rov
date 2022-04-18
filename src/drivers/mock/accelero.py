from ..accelero import AccelValue


class IMU:
    def __init__(self) -> None:
        pass

    def get_state(self) -> AccelValue:
        return AccelValue(0, 0, 0, 0, 0, 0)
