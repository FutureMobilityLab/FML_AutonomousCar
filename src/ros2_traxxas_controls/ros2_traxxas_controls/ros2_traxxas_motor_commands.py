import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from board import SCL, SDA
import busio
# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

boundedSignal = lambda x, l, u: l if x < l else u if x > u else x

class MotorCommands(Node):

    def __init__(self):
        super().__init__('motor_commands')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription #prevents unused variable warning

    def SteeringCommand(SteeringCMD):
        i2c = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        pca = PCA9685(i2c)
        pca.frequency = 50
        TraxxasServo = servo.Servo(pca.channels[0])
        steeringClipped = boundedSignal(SteeringCMD,-0.65,0.65)
        TraxxasServo.angle = (steeringClipped * 180 / 3.14159265) + 90
        pca.deinit()

    def ThrottleCommand(ThrottleCMD):
        i2c = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c)
        pca.frequency = 50
        ThrottleCMDClipped = boundedSignal(ThrottleCMD,0,0.5*66535)
        pca.channels[1].duty_cycle = ThrottleCMDClipped
        pca.deinit()
    

def main(args=Node):
    rclpy.init(args=args)

    motor_commands = MotorCommands()

    rclpy.spin(motor_commands)

    motor_commands.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()