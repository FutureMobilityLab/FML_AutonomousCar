from board import SCL, SDA
import busio
# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

boundedSteering = lambda x, l, u: l if x < l else u if x > u else x

def SteeringCommand(cmd):
    i2c = busio.I2C(SCL, SDA)
    # Create a simple PCA9685 class instance.
    pca = PCA9685(i2c)
    pca.frequency = 50
    TraxxasServo = servo.Servo(pca.channels[0])
    steeringClipped = boundedSteering(cmd,-0.65,0.65)
    TraxxasServo.angle = (steeringClipped * 180 / 3.14159265) + 90
    pca.deinit()
