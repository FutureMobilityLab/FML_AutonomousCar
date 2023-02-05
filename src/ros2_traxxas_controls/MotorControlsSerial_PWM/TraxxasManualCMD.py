#!/usr/bin/python3

from board import SCL, SDA
import busio
# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import keyboard
import time
import os

boundedSignal = lambda x, l, u: l if x < l else u if x > u else x

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



def main():
    ThrottleCMD = 0
    SteerCMD = 0
    while True:
        if keyboard.is_pressed('w'):
            ThrottleCMD += 100
        if keyboard.is_pressed('s'):
            ThrottleCMD -= 100
        if keyboard.is_pressed('x'):
            ThrottleCMD = 0
        if keyboard.is_pressed('a'):
            SteerCMD += 0.1
        if keyboard.is_pressed('d'):
            SteerCMD -= 0.1
        if keyboard.is_pressed('c'):
            SteerCMD = 0
        
        SteeringCommand(SteerCMD)
        ThrottleCommand(ThrottleCMD)
        os.system('clear')
        print("Steering Command:"+str(SteerCMD))
        print("Throttle Command:"+str(ThrottleCMD))
        time.sleep(.1)


if __name__ == "__main__":
    main()