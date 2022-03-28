try:
    import RPi.GPIO as GPIO
except:
    pass


class Switch():
    def __init__(self) -> None:
        self.on = False
        self.pin = 0  # TODO

        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.out(self.pin, False)

    def set(self, on: bool):
        GPIO.out(self.pin, on)
        self.on = on
