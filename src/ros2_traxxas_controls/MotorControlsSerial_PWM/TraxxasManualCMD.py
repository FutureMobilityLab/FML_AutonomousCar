# #!/usr/bin/python3

# from board import SCL, SDA
# import busio
# # Import the PCA9685 module. Available in the bundle and here:
# #   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
# from adafruit_motor import servo
# from adafruit_pca9685 import PCA9685
# import sshkeyboard
# import time
# import os

# boundedSignal = lambda x, l, u: l if x < l else u if x > u else x

# class ManualSteering:
#     def __init__(self):
#         self.ThrottleCMD = 0
#         self.SteerCMD = 0

#     def onPress(key):
#         if key == "w":
#             ThrottleCMD += 50
#         if key == "s":
#             ThrottleCMD -= 50
#         if key == "a":
#             SteerCMD += 0.05
#         if key == "d":
#             ThrottleCMD -= 0.05
#         if key == "x":
#             ThrottleCMD = 0
#         if key == "c":
#             ThrottleCMD = 0   

# def SteeringCommand(cmd):
#         i2c = busio.I2C(SCL, SDA)
#         # Create a simple PCA9685 class instance.
#         pca = PCA9685(i2c)
#         pca.frequency = 50
#         TraxxasServo = servo.Servo(pca.channels[0])
#         steeringClipped = boundedSignal(cmd,-0.65,0.65)
#         TraxxasServo.angle = (steeringClipped * 180 / 3.14159265) + 90
#         pca.deinit()

# def ThrottleCommand(ThrottleCMD):
#     i2c = busio.I2C(SCL, SDA)
#     pca = PCA9685(i2c)
#     pca.frequency = 50
#     ThrottleCMDClipped = boundedSignal(ThrottleCMD,0.05*65535,0.1*66535)
#     pca.channels[1].duty_cycle = int(ThrottleCMDClipped)
#     pca.deinit()  

# def main():
#     Traxxas = ManualSteering()
#     while True:
#         sshkeyboard.listen_keyboard(on_press=Traxxas.onPress)
#         SteeringCommand(Traxxas.SteerCMD)
#         ThrottleCommand(Traxxas.ThrottleCMD)
#         # os.system('clear')
#         print("Steering Command:"+str(Traxxas.SteerCMD))
#         print("Throttle Command:"+str(Traxxas.ThrottleCMD))
#         time.sleep(0.05)
        
# if __name__ == "__main__":
#     main()

import time
from sshkeyboard import listen_keyboard
from board import SCL, SDA
import busio
# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import os

boundedSignal = lambda x, l, u: l if x < l else u if x > u else x

class keys:
    def __init__(self):
        self.SteerCMD = 0
        self.ThrottleCMD = 4500

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


keyboardlog = keys()
def press(key):
    if key == "w":
        keyboardlog.ThrottleCMD += 50
    if key == "s":
        keyboardlog.ThrottleCMD -= 50
    if key == "a":
        keyboardlog.SteerCMD += 0.05
    if key == "d":
        keyboardlog.SteerCMD -= 0.05
    if key == "x":
        keyboardlog.ThrottleCMD = 4500
    if key == "c":
        keyboardlog.SteerCMD = 0  

    SteeringCommand(keyboardlog.SteerCMD)
    ThrottleCommand(keyboardlog.ThrottleCMD)
    print("Steering Command:"+str(keyboardlog.SteerCMD))
    print("Throttle Command:"+str(keyboardlog.ThrottleCMD))
    time.sleep(0.05)

listen_keyboard(on_press=press,delay_second_char = 0.00, delay_other_chars=0.0)