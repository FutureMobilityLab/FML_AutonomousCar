import atexit
import sys
from abc import ABC, abstractmethod
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class Teleop(Node, ABC):
    def __init__(self):
        atexit.register(self._emergency_stop)
        Node.__init__(self, "traxxas_teleop")

        self.declare_parameter("linear_max", 0.1)
        self.declare_parameter("angular_max", 1.0)
        self.declare_parameter("publish_rate", 10.0)
        self.LINEAR_MAX = self.get_parameter("linear_max").value

        self.ANGULAR_MAX = self.get_parameter("angular_max").value

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "cmd_ackermann", qos_profile_system_default
        )
        self._make_twist = self._make_twist_stamped
        rate = 1 / self.get_parameter("publish_rate").value
        self.create_timer(rate, self._publish)
        self.linear = 0.0
        self.angular = 0.0

    @abstractmethod
    def on_press(self, *args):
        pass

    def write_twist(self, linear=None, angular=None):
        if linear is not None:
            if abs(linear) <= self.LINEAR_MAX:
                self.linear = linear
            else:
                self.get_logger().error(
                    f"Trying to set a linear speed {linear} outside of allowed range of [{-self.LINEAR_MAX}, {self.LINEAR_MAX}]"
                )
        if angular is not None:
            if abs(angular) <= self.ANGULAR_MAX:
                self.angular = angular
            else:
                self.get_logger().error(
                    f"Trying to set a angular speed {angular} outside of allowed range of [{-self.ANGULAR_MAX}, {self.ANGULAR_MAX}]"
                )
        self._update_screen()

    def _make_twist_unstamped(self, linear, angular):
        ackermann_msg = AckermannDrive()
        ackermann_msg.speed = linear
        ackermann_msg.steering_angle = angular
        return ackermann_msg

    def _make_twist_stamped(self, linear, angular):
        ackermann_msg_stamped = AckermannDriveStamped()
        ackermann_msg_stamped.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg_stamped.drive.speed = linear
        ackermann_msg_stamped.drive.steering_angle = angular
        return ackermann_msg_stamped

    def _publish(self):
        twist = self._make_twist(self.linear, self.angular)
        self.publisher_.publish(twist)

    def _update_screen(self):
        sys.stdout.write(f"Linear: {self.linear:.2f}, Angular: {self.angular:.2f}\r")

    def _emergency_stop(self):
        self.publisher_.publish(self._make_twist(0.0, 0.0))
