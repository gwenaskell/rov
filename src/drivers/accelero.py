from dataclasses import dataclass

from src.drivers.mappings import Imu


@dataclass
class AccelValue:
    ax: float
    ay: float
    az: float
    wx: float
    wy: float
    wz: float


class IMU:
    def __init__(self) -> None:
        from mpu6050 import mpu6050
        self.driver = Imu
        self.sensor = mpu6050(self.driver.address)

    def get_state(self) -> AccelValue:
        accel = self.sensor.get_accel_data()

        assert accel is not None

        gyro = self.sensor.get_gyro_data()
        print(gyro['x'])
        print(gyro['y'])
        print(gyro['z'])

        return AccelValue(accel['x'], accel['y'], accel['z'], gyro['x'], gyro['y'], gyro['z'])
