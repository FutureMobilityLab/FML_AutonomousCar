# #!/usr/bin/python3

# This Script is an ssh method of controlling the Traxxas Car, using ssh keyboard.
# Kept in repository for use when not controlling car via ROS node, and for low level testing.

from sshkeyboard import listen_keyboard
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

boundedSignal = lambda x, l, u: l if x < l else u if x > u else x

class keys:
    def __init__(self):
        self.SteerCMD = 0
        self.ThrottlePercent = 0.0
        self.ThrottleCMD = 4915

def SteeringCommand(cmd):
        i2c = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        pca = PCA9685(i2c)
        pca.frequency = 50
        TraxxasServo = servo.Servo(pca.channels[0])
        steeringClipped = boundedSignal(cmd,-0.65,0.65)
        TraxxasServo.angle = (steeringClipped * 180 / 3.14159265) + 90
        pca.deinit()

def ThrottleCommand(ThrottleCMD):
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    ThrottleCMDClipped = boundedSignal(ThrottleCMD,0.05*65535,0.1*66535)
    pca.channels[1].duty_cycle = int(ThrottleCMDClipped)
    pca.deinit()  

def ThrottleOutput(ThrottlePercent):
    OutputPID = 4915 + (ThrottlePercent)*(6554-4915)
    TraxxasI2C.ThrottleCMD = OutputPID
    ThrottleCommand(TraxxasI2C.ThrottleCMD)


TraxxasI2C = keys()
def press(key):
    if key == "w":
        TraxxasI2C.ThrottlePercent += 0.01
    if key == "s":
        TraxxasI2C.ThrottlePercent -= 0.01
    if key == "a":
        TraxxasI2C.SteerCMD += 0.05
    if key == "d":
        TraxxasI2C.SteerCMD -= 0.05
    if key == "x":
        TraxxasI2C.ThrottlePercent = 0
    if key == "c":
        TraxxasI2C.SteerCMD = 0  

    TraxxasI2C.SteerCMD = boundedSignal(TraxxasI2C.SteerCMD,-.65,.65)
    TraxxasI2C.ThrottlePercent = boundedSignal(TraxxasI2C.ThrottlePercent,0,1)
    SteeringCommand(TraxxasI2C.SteerCMD)
    ThrottleOutput(TraxxasI2C.ThrottlePercent)
    print("Steering Command:"+str(TraxxasI2C.SteerCMD))
    print("Throttle Command:"+str(TraxxasI2C.ThrottleCMD))
    

listen_keyboard(on_press=press,delay_second_char = 0.00, delay_other_chars=0.0)
