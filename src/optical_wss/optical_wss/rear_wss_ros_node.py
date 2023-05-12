#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import TwistWithCovarianceStamped

PI = 3.14159265359


class RearWss(Node):
    """Ros2 node for rear optical wheel speed sensors. This connects directly
    to an Arduino Uno over a UART protocol. The arduino sends strings that
    we have to parse and interpret. It only sends right and left wheel speeds
    and the update rate is inconsistent.
    """

    def __init__(self):
        super().__init__("rear_wss")
        # Setup serial connection with 9600 baud rate and a timeout of 1 sec.
        self.serial_connection = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        # Reset the input buffer to clear incomplete data.
        self.serial_connection.reset_input_buffer()
        self.msg_buffer = ""

        # Sensor parameters.
        self.wheel_radius = 0.057  # [m]
        self.track = 0.28  # [m]
        self.num_slots = 72
        # Store previous values between publications. This is like a zero-order
        # hold. The arduino is not guaranteed to publish both sensor readings
        # everytime.
        self.rl_wss = 0  # [m/s]
        self.rr_wss = 0  # [m/s]

        self.twist_publisher = self.create_publisher(
            TwistWithCovarianceStamped, "rear_wss", 10
        )

        # Wait until we receive from the arduino that the connection is complete.
        msg = ""
        self.get_logger().info("Waiting for Arduino-Rpi connection to complete.")
        while msg != "Setup Complete":
            if self.serial_connection.in_waiting > 0:
                msg = self.serial_connection.readline().decode("utf-8").rstrip()
        self.get_logger().info("Arduino-Rpi connection complete.")

        # Begin sensor loop.
        self.time_of_last_msg = self.get_clock().now().nanoseconds * 1e-9
        self.sample_rate = 0.05
        self.wss_timer = self.create_timer(self.sample_rate, self.get_wss)

    def get_wss(self):
        new_msg = self.serial_connection.read_all().decode("utf-8")
        # It is possible that the arduino does not send a message. So we handle
        # this with a timeout.
        time_now = self.get_clock().now().nanoseconds * 1e-9
        # The first and last messages may be cutoff, but the middle messages
        # will not be.
        if len(new_msg) < 15:
            if time_now - self.time_of_last_msg > 1.0:
                self.get_logger().error(f"time:{time_now - self.time_of_last_msg}")
                raise IOError
        else:
            self.time_of_last_msg = time_now
        self.msg_buffer += new_msg

        # Keep only the last 50 chars because they are newest.
        if len(self.msg_buffer) > 50:
            self.msg_buffer = self.msg_buffer[len(self.msg_buffer) - 50 :]
        msgs = self.msg_buffer.split("\n")

        # It is assumed the last messages are the most recent.
        for msg in reversed(msgs):
            # The msgs are not guaranteed to have both RR and RL. Some of the
            # messages are cutoff. So we filter those out.
            if "RR" not in msg and "RL" not in msg:
                continue
            lines = msg.rstrip().split(",")
            # A full message will have 4 lines.
            if len(lines) < 4:
                continue
            # A full message will not have any empty lines.
            if not all([len(line) > 0 for line in lines]):
                continue
            self.rl_wss = float(lines[1]) * self.wheel_radius
            self.rr_wss = float(lines[3]) * self.wheel_radius
            break

        # It is not guaranteed that msgs will contain messages. Publish the
        # velocity anyway.
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.twist.linear.x = (self.rr_wss + self.rl_wss) / 2
        # Set linear x velocity covariance by assuming variance is the WSS
        # resolution.
        wss_velocity_resolution = (
            1 / self.sample_rate * PI / self.num_slots * self.wheel_radius
        )
        twist.twist.covariance[0] = wss_velocity_resolution**2
        twist.twist.twist.angular.z = (self.rr_wss - self.rl_wss) / self.track
        # Set yaw rate covariance by assuming variance is WSS yaw rate resolution.
        # TODO: This assumes no correlation between yaw rate and velocity which is
        # very wrong. But I do not know how to compute this.
        wss_yaw_rate_resolution = wss_velocity_resolution / self.track
        twist.twist.covariance[35] = wss_yaw_rate_resolution**2
        self.twist_publisher.publish(twist)
        self.serial_connection.flush()


def main(args=None):
    rclpy.init(args=args)

    rear_wss = RearWss()

    rclpy.spin(rear_wss)

    # Destroy the node explicitly
    rear_wss.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
