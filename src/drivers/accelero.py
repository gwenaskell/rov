from dataclasses import dataclass
import adafruit_mpu6050

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
        self.driver = Imu
        self.sensor = adafruit_mpu6050.MPU6050(
            self.driver.bus, address=self.driver.address)

    def get_state(self) -> AccelValue:
        accel = self.sensor.acceleration

        gyro = self.sensor.gyro

        return AccelValue(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2])
