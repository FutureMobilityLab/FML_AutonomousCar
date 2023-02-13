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
        self.Kp = 100
        self.Ki = 10
        self.errorIntegrated = 0
        tempTimeStamp = self.get_clock().now()
        tempTimemsg = tempTimeStamp.to_msg()
        self.timeLastLooped = tempTimemsg.sec + tempTimemsg.nanosec * 10**-9

    def getTimeDiff(self,timestamp):
        timeNow = timestamp.sec + timestamp.nsec * 10**-9
        timediff = timeNow - self.timeLastLooped
        return (timediff)
    
    def LoopPID(self,AckermannCMD):
        error = AckermannCMD.drive.speed - self.v
        integratorTimeStep = self.getTimeDiff(self,AckermannCMD.header.stamp)
        ThrottleDesired = self.Kp * error + self.Ki * self.errorIntegrated * integratorTimeStep
        self.errorIntegrated = self.errorIntegrated + error * integratorTimeStep
        ThrottleRegisterVal = 4915 + 16.38 * ThrottleDesired ##converts to register value ()
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
        ThrottleCMD = self.LoopPID(self,ackermann_cmd.drive.speed)
        ThrottleCMDClipped = int(boundedSignal(ThrottleCMD,0,0.1*66535))
        pca.channels[1].duty_cycle = ThrottleCMDClipped
        pca.deinit()
        print('ThrottleCMD:',ThrottleCMD,'SteerCMD:',steeringClipped)



def main(args=None):
    rclpy.init(args=args)

    motor_commands = MotorCommands()

    rclpy.spin(motor_commands)

    motor_commands.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()