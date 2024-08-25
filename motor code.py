import RPi.GPIO as GPIO
import keyboard
from time import sleep

# Pins for Motor Driver Inputs
Motor1A = 16
Motor1B = 20
Motor2A = 21
Motor2B = 26  # Assuming these are the GPIO pins for the second motor
#Motor2B = 17
#Motor2E = 25
 

 
def setup():
    GPIO.setmode(GPIO.BCM)             # GPIO Numbering
    GPIO.setup(Motor1A, GPIO.OUT)      # All pins as Outputs
    GPIO.setup(Motor1B, GPIO.OUT)
    #GPIO.setup(Motor1E, GPIO.OUT)
    GPIO.setup(Motor2A, GPIO.OUT)
    GPIO.setup(Motor2B, GPIO.OUT)
    #GPIO.setup(Motor2E, GPIO.OUT)
    global motor1_pwm
    #motor1_pwm = GPIO.PWM(Motor1A, 100)  # PWM frequency = 100 Hz
    #motor1_pwm.start(0)  # Start PWM with 0% duty cycle

def move_forward(speed):
   # motor1_pwm.ChangeDutyCycle(speed)
    print('hello')
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    #GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    #GPIO.output(Motor2E, GPIO.HIGH)(
    sleep(1)

def move_backward():
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    #GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    #GPIO.output(Motor2E, GPIO.HIGH)
    sleep(1)

def move_left():
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    #GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    #GPIO.output(Motor2E, GPIO.HIGH)
    sleep(1)

def move_right():
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    #GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    #GPIO.output(Motor2E, GPIO.HIGH)
    sleep(1)

def stop():
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor2A, GPIO.LOW)

if __name__ == '__main__':
    setup()
    try:
        while True:
            #move_forward(10)
            direction = input("Enter direction (w: forward, s: backward, any other key to stop): ")
            if direction == 'w':
                print('W')
                move_forward(10)
            elif direction == 's':
                print('S')
                move_backward()
            elif direction == 'a':
                print('A')
                move_left()
            elif direction == 'd':
                print('D')
                move_right()
            else:
                stop()
    except KeyboardInterrupt:
        stop()