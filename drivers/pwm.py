import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

# Setup GPIO Pins
GPIO.setup(12, GPIO.OUT)
GPIO.setup(32, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
GPIO.setup(35, GPIO.OUT)

# Set PWM instance and their frequency
pwm12 = GPIO.PWM(12, 0.5)
pwm32 = GPIO.PWM(32, 0.75)
pwm33 = GPIO.PWM(33, 0.66)
pwm35 = GPIO.PWM(35, 0.87)

# Start PWM with 50% Duty Cycle
pwm12.start(50)
pwm32.start(50)
pwm33.start(50)
pwm35.start(50)

raw_input('Press return to stop:')	#Wait

# Stops the PWM
pwm12.stop()
pwm32.stop()
pwm33.stop()
pwm35.stop()

# Cleans the GPIO
GPIO.cleanup()