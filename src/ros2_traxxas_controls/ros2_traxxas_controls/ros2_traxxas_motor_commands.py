import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
boundedSignal = lambda x, l, u: l if x < l else u if x > u else x

class MotorCommands(Node):

    def __init__(self):
        super().__init__('TraxxasSteering')
        self.subscription = self.create_subscription(AckermannDriveStamped,'cmd_ackermann',self.ackermann_callback,10)
        self.subscription = self.create_subscription(Odometry,'odometry/filtered',self.odom_callback,10)
        self.subscription  # prevent unused variable warning
        self.Kp = 1
        self.Ki = 1
        self.errorIntegrated = 0
        self.timeLastLooped = 3

    
    def LoopPID(AckermannCMD):
        error = AckermannCMD.drive.speed - MotorCommands.v
        ThrottleRegisterVal = MotorCommands.Kp * error + MotorCommands.Ki * MotorCommands.errorIntegrated
        MotorCommands.errorIntegrated = MotorCommands.errorIntegrated + error * .1
        return ThrottleRegisterVal

    def odom_callback(self,msg):
       self.v = msg.twist.twist.linear.x

    def ackermann_callback(self,ackermann_cmd):
        i2c = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        pca = PCA9685(i2c)
        pca.frequency = 50
        TraxxasServo = servo.Servo(pca.channels[0])
        steeringClipped = boundedSignal(ackermann_cmd.drive.steering_angle,-0.65,0.65)
        TraxxasServo.angle = (steeringClipped * 180.0 / 3.14159265) + 90.0
        # pca.deinit()

        # i2c = busio.I2C(SCL, SDA)
        # pca = PCA9685(i2c)
        # pca.frequency = 50
        ThrottleCMD = self.LoopPID(ackermann_cmd.drive.speed)
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