from evdev import InputDevice, categorize, ecodes

# creates object 'gamepad' to store the data
# you can call it whatever you like
gamepad = InputDevice('/dev/input/event22')

# prints out device info at start
print(gamepad)

# evdev takes care of polling the controller in a loop
for event in gamepad.read_loop():
    print(event)
    # print(categorize(event))
