import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node, QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import numpy as np


class MotorCommands(Node):
    def __init__(self):
        super().__init__("motor_driver")
        FMLCarQoS = QoSProfile(history=1, depth=1, reliability=2, durability=2)

        self.command_subscription = self.create_subscription(
            AckermannDriveStamped, "cmd_ackermann", self.ackermann_callback, FMLCarQoS
        )
        self.speed_subscription = self.create_subscription(
            Odometry, "rear_wss", self.rear_wss_callback, FMLCarQoS
        )
        self.accel_subscription = self.create_subscription(
            Imu, "imu/data", self.accel_callback, FMLCarQoS
        )
        self.accel_x = 0
        self.command_subscription  # prevents unused variable warning
        self.speed_subscription

        # Sets Default Parameters
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("ki", 20.0)
        self.declare_parameter("kt", 2.0)
        self.declare_parameter("throttle_register_idle", 4915)
        self.declare_parameter("throttle_register_full", 6335)
        self.declare_parameter("throttle_register_revr", 3276)
        self.declare_parameter("max_steer_angle", 0.65)
        self.declare_parameter("max_accel", 5.0)
        self.declare_parameter("crash_accel", 10.0)
        # Overrrides Parameters if Config File is Passed
        self.Kp = self.get_parameter("kp").value
        self.Ki = self.get_parameter("ki").value
        self.Kt = self.get_parameter("kt").value
        self.throttle_idle = self.get_parameter("throttle_register_idle").value
        self.throttle_full = self.get_parameter("throttle_register_full").value
        self.throttle_revr = self.get_parameter("throttle_register_revr").value
        self.max_steer_angle = self.get_parameter("max_steer_angle").value
        self.max_accel = self.get_parameter("max_accel").value
        self.crash_accel = self.get_parameter("crash_accel").value
        # Unchanging Parameters
        self.throttle_pcnt_increment = (self.throttle_full - self.throttle_idle) / 100
        self.errorIntegrated = 0
        tempTimeStamp = self.get_clock().now()
        tempTimemsg = tempTimeStamp.to_msg()
        self.timeoutCount = 0
        self.timeLastLooped = tempTimemsg.sec + tempTimemsg.nanosec * 10**-9
        self.v = 0
        self.debugBool = False
        self.get_logger().info(
            f"""Motor PI Control Gains:
        Kp: {self.Kp}
        Ki: {self.Ki}
        Kt: {self.Kt}"""
        )

        # Setup I2C bus interface and PCA9685 class instance.
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.TraxxasServo = servo.Servo(self.pca.channels[0])
        self.throttleChannel = self.pca.channels[1]

    def getTimeDiff(self, timestamp):
        timeNow = timestamp.sec + timestamp.nanosec * 10**-9
        timediff = timeNow - self.timeLastLooped
        self.timeLastLooped = timeNow
        return timediff

    def getThrottleCmd(self, AckermannCMD: AckermannDriveStamped):
        error = AckermannCMD.drive.speed - self.v
        steerangle = AckermannCMD.drive.steering_angle
        integratorTimeStep = self.getTimeDiff(AckermannCMD.header.stamp)
        if integratorTimeStep > 0.5:
            self.errorIntegrated = 0.0
            self.get_logger().info("***INTEGRATOR TIMEOUT - RESETTING INTEGRAL***")
        else:
            self.errorIntegrated = self.errorIntegrated + error * integratorTimeStep
        ThrottleDesired = (
            self.Kp * error + self.Ki * self.errorIntegrated + self.Kt * abs(steerangle)
        )
        if self.debugBool:
            self.get_logger().info(
                f"Measured Velocity: {self.v}\tError: {error}\tThrottle Out:"
                f" {ThrottleDesired}"
            )

        ThrottleRegisterVal = (
            self.throttle_idle + self.throttle_pcnt_increment * ThrottleDesired
        )  # converts to register value ()

        # if ThrottleDesired > 20:
        #     ThrottleRegisterVal = self.throttle_idle
        #     self.get_logger().info(
        #         "***MAX THROTTLE - SETTING TO IDLE AND QUITTING***"
        #     )
        #     raise SystemExit

        if abs(self.accel_x) > self.max_accel and np.sign(self.accel_x) == np.sign(self.v):
            self.errorIntegrated = self.errorIntegrated
            self.get_logger().info("***EXCESSIVE ACCELERATION - HOLDING INTEGRAL***")

        if abs(self.accel_x) > self.crash_accel:
            ThrottleRegisterVal = self.throttle_idle
            self.get_logger().info(
                f"*** GOT {self.accel_x} M/S^2 - ASSUMED COLLISION - SETTING IDLE AND QUITTING"
            )
            raise SystemExit

        # if ThrottleDesired > 5 and self.v < 0.02:
        #     self.timeoutCount += 1
        #     if self.timeoutCount > 20:
        #         ThrottleRegisterVal = self.throttle_idle
        #         self.get_logger().info(
        #             "*** THROTTLE ACTIVE BUT NO MOTION - ASSUMED COLLISION - "
        #             "SETTING IDLE AND QUITTING"
        #         )
        #         raise SystemExit

        return ThrottleRegisterVal

    def sendServoCmd(self) -> None:
        """Send servo angle to Traxxas servo."""
        steeringClipped = np.clip(
            self.ackermann_cmd.drive.steering_angle,
            -self.max_steer_angle,
            self.max_steer_angle,
        )
        self.TraxxasServo.angle = (steeringClipped * 180.0 / 3.14159265) + 90.0
        # self.pca.deinit()

    def sendThrottleCmd(self) -> None:
        """Send throttle command to Traxxas speed controller over i2c."""
        ThrottleCMD = self.getThrottleCmd(self.ackermann_cmd)
        ThrottleCMDClipped = np.clip(ThrottleCMD, self.throttle_revr, self.throttle_full)
        self.throttleChannel.duty_cycle = int(ThrottleCMDClipped)
        # self.pca.deinit()

    def rear_wss_callback(self, msg: Odometry) -> None:
        self.v = msg.twist.twist.linear.x
        odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"Odom delay: {now_time - odom_time} s.")

    def accel_callback(self, msg: Imu) -> None:
        self.accel_x = msg.linear_acceleration.x

    def ackermann_callback(self, msg: AckermannDriveStamped) -> None:
        """When a new command is received send a steer and throttle command."""
        self.ackermann_cmd = msg
        cmd_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"Command delay: {now_time - cmd_time} s.")
        self.sendServoCmd()
        servo_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"Servo delay:{servo_time - now_time}s")
        self.sendThrottleCmd()
        throttle_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"Throttle delay:{throttle_time - servo_time}s")


def main(args=None):
    rclpy.init(args=args)

    motor_commands = MotorCommands()

    try:
        rclpy.spin(motor_commands)
    except KeyboardInterrupt:
        pass
    # SystemExit is used to indicate a system failure in getThrottleCmd().
    except SystemExit:
        pass
    except ExternalShutdownException:
        pass

    # Before shutting down, send zero steer command and idle throttle command.
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
    rclpy.logging.get_logger("EXITCMD").info(
        f"""FINAL THROTTLE: {motor_commands.throttle_idle}   FINAL STEER: {TraxxasServo.angle}"""
    )
    rclpy.logging.get_logger("Quitting Motor Controller Node").info("Done")
    motor_commands.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
