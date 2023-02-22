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
        super().__init__("motor_driver")
        self.command_subscription = self.create_subscription(AckermannDriveStamped,'cmd_ackermann',self.ackermann_callback,10)
        self.speed_subscription = self.create_subscription(Odometry,'odometry/filtered',self.odom_callback,10)
        self.command_subscription  # prevents unused variable warning
        self.speed_subscription
        self.declare_parameter("kp", 5)
        self.declare_parameter("ki",  10)
        self.declare_parameter("throttle_register_idle", 4915)
        self.declare_parameter("throttle_register_full", 6335)
        self.declare_parameter("throttle_register_revr", 3276)
        self.declare_parameter("max_steer_angle", 0.65)
        self.Kp = self.get_parameter("kp").value
        self.Ki = self.get_parameter("ki").value
        self.throttle_idle = self.get_parameter("throttle_register_idle").value
        self.throttle_full = self.get_parameter("throttle_register_full").value
        self.throttle_revr = self.get_parameter("throttle_register_revr").value
        self.max_steer_angle = self.get_parameter("max_steer_angle").value
        self.throttle_pcnt_increment = (self.throttle_full-self.throttle_idle)/100
        self.errorIntegrated = 0
        tempTimeStamp = self.get_clock().now()
        tempTimemsg = tempTimeStamp.to_msg()
        self.timeLastLooped = tempTimemsg.sec + tempTimemsg.nanosec * 10**-9
        self.v = 0
        self.get_logger().info(F"""Motor PI Control Gains:
        Kp: {self.Kp}
        Ki: {self.Ki}""")

    def getTimeDiff(self,timestamp):
        timeNow = timestamp.sec + timestamp.nanosec * 10**-9
        timediff = timeNow - self.timeLastLooped
        self.timeLastLooped = timeNow
        return (timediff)
    
    def LoopPID(self,AckermannCMD):
        error = AckermannCMD.drive.speed - self.v
        integratorTimeStep = self.getTimeDiff(AckermannCMD.header.stamp)
        ThrottleDesired = self.Kp * error + self.Ki * self.errorIntegrated
        ThrottleRegisterVal = self.throttle_idle + self.throttle_pcnt_increment * ThrottleDesired ##converts to register value ()
        if AckermannCMD.drive.speed * self.v < 0.0: # If direction commanded is reverse of speed
            self.errorIntegrated = 0
            self.get_logger().info(f"""***REVERSE COMMAND - RESETTING INTEGRAL***""")
        if ThrottleRegisterVal < self.throttle_full and ThrottleRegisterVal > self.throttle_revr:
            self.errorIntegrated = self.errorIntegrated + error * integratorTimeStep
        return ThrottleRegisterVal

    def odom_callback(self,msg):
       self.v = msg.twist.twist.linear.x

    def ackermann_callback(self,ackermann_cmd):
        i2c = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c)
        pca.frequency = 50
        TraxxasServo = servo.Servo(pca.channels[0])
        steeringClipped = boundedSignal(ackermann_cmd.drive.steering_angle,-self.max_steer_angle,self.max_steer_angle)
        TraxxasServo.angle = (steeringClipped * 180.0 / 3.14159265) + 90.0
        pca.deinit()

        i2c = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c)
        pca.frequency = 50
        ThrottleCMD = self.LoopPID(ackermann_cmd)
        ThrottleCMDClipped = int(boundedSignal(ThrottleCMD,self.throttle_revr,self.throttle_full))
        pca.channels[1].duty_cycle = ThrottleCMDClipped
        pca.deinit()
        self.get_logger().info(f"""Throttle Command: {ThrottleCMDClipped}   Steering Command: {TraxxasServo.angle}""")



def main(args=None):
    rclpy.init(args=args)

    motor_commands = MotorCommands()

    rclpy.spin(motor_commands)

    motor_commands.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()