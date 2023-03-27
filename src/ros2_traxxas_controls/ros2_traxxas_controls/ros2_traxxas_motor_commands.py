import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
        self.accel_subscription = self.create_subscription(Imu,'imu',self.accel_callback,10)
        self.command_subscription  # prevents unused variable warning
        self.speed_subscription
        # Sets Default Parameters
        self.declare_parameter("kp", 15.0)
        self.declare_parameter("ki",  5.0)
        self.declare_parameter("throttle_register_idle", 4915)
        self.declare_parameter("throttle_register_full", 6335)
        self.declare_parameter("throttle_register_revr", 3276)
        self.declare_parameter("max_steer_angle", 0.65)
        self.declare_parameter("max_accel", 1.0)
        # Overrrides Parameters if Config File is Passed
        self.Kp = self.get_parameter("kp").value
        self.Ki = self.get_parameter("ki").value
        self.throttle_idle = self.get_parameter("throttle_register_idle").value
        self.throttle_full = self.get_parameter("throttle_register_full").value
        self.throttle_revr = self.get_parameter("throttle_register_revr").value
        self.max_steer_angle = self.get_parameter("max_steer_angle").value
        self.max_accel = self.get_parameter("max_accel").value
        # Unchanging Parameters
        self.throttle_pcnt_increment = (self.throttle_full-self.throttle_idle)/100
        self.errorIntegrated = 0
        tempTimeStamp = self.get_clock().now()
        tempTimemsg = tempTimeStamp.to_msg()
        self.timeoutCount = 0
        self.timeLastLooped = tempTimemsg.sec + tempTimemsg.nanosec * 10**-9
        self.a = 0
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
        if integratorTimeStep > 0.5:
            self.errorIntegrated = 0.0
            self.get_logger().info(f"""***INTEGRATOR TIMEOUT - RESETTING INTEGRAL***""")
        if ThrottleDesired > 75:
            self.timeoutCount += 1
            if self.timeoutCount > 20:
                ThrottleRegisterVal = self.throttle_idle
                self.get_logger().info(f"""***MAX THROTTLE TIMEOUT - SETTING TO IDLE AND QUITTING***""")
                raise SystemExit
        else:
            self.timeoutCount = 0
        if ThrottleRegisterVal < self.throttle_full and ThrottleRegisterVal > self.throttle_revr:
            if abs(self.a) < self.max_accel:
                self.errorIntegrated = self.errorIntegrated + error * integratorTimeStep
            else:
                self.errorIntegrated = self.errorIntegrated
                # self.get_logger().info(f"""***EXCESSIVE ACCELERATION - HOLDING INTEGRAL TERM***""")
        return ThrottleRegisterVal

    def odom_callback(self,msg):
        self.v = msg.twist.twist.linear.x

    def accel_callback(self,msg):
        self.a = msg.linear_acceleration.x

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
        # self.get_logger().info(f"""Throttle Command: {ThrottleCMDClipped}   Steering Command: {TraxxasServo.angle}""")

def main(args=None):
    rclpy.init(args=args)

    motor_commands = MotorCommands()

    try:
        rclpy.spin(motor_commands)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except ExternalShutdownException:
        pass
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    TraxxasServo = servo.Servo(pca.channels[0])
    TraxxasServo.angle = 90
    pca.deinit

    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    pca.channels[1].duty_cycle = motor_commands.throttle_idle
    pca.deinit()
    rclpy.logging.get_logger("EXITCMD").info(f"""FINAL THROTTLE: {motor_commands.throttle_idle}   FINAL STEER: {TraxxasServo.angle}""")
    rclpy.logging.get_logger("Quitting Motor Controller Node").info("Done")
    motor_commands.destroy_node()
    rclpy.try_shutdown()

    

if __name__ == '__main__':
    main()
