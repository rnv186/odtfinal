from machine import Pin, PWM
import neopixel
import time

# --- SETUP ---

# Button
button = Pin(12, Pin.IN, Pin.PULL_UP)

# NeoPixel
np_pin = Pin(5)
n_pixels = 16
np = neopixel.NeoPixel(np_pin, n_pixels)

# Servos
servo1 = PWM(Pin(15), freq=50)
servo2 = PWM(Pin(4), freq=50)

# IR Sensor
ir_sensor = Pin(14, Pin.IN)

# --- FUNCTIONS ---

def move_servo(servo, angle):
    duty = int((angle / 180) * 102) + 26
    servo.duty(duty)
    time.sleep(0.5)

def loading_animation():
    for _ in range(3):
        for i in range(n_pixels):
            np[i] = (0, 255, 0)
            np.write()
            time.sleep(0.03)
            np[i] = (0, 0, 0)
            np.write()

def reset_neopixel():
    for i in range(n_pixels):
        np[i] = (0, 0, 0)
    np.write()

# --- MAIN PROGRAM LOOP ---

while True:
    # Set starting positions
    move_servo(servo1, 0)
    move_servo(servo2, 90)   # <--- start servo2 at 90 degrees!
    reset_neopixel()

    print("Waiting for button press...")
    while button.value() == 1:
        time.sleep(0.01)

    print("Button pressed!")
    loading_animation()

    print("Moving Servo 1...")
    move_servo(servo1, 90)  # Servo 1 moves 90 degrees

    print("Waiting for IR sensor trigger...")
    while ir_sensor.value() == 1:
        time.sleep(0.01)

    print("IR triggered!")

    print("Moving Servo 2 back to 0...")
    move_servo(servo2, 0)  # Servo 2 comes back to 0

    print("Cycle complete! Resetting in 3 seconds...")
    time.sleep(3)
