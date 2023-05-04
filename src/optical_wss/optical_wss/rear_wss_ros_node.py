#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time

from geometry_msgs.msg import TwistStamped


class RearWss(Node):
    '''Ros2 node for rear optical wheel speed sensors. This connects directly 
    to an Arduino Uno over a UART protocol. The arduino sends strings that 
    we have to parse and interpret. It only sends right and left wheel speeds
    and the update rate is inconsistent.
    '''

    def __init__(self):
        super().__init__('rear_wss')
        # Setup serial connection with 9600 baud rate and a timeout of 1 sec.
        self.serial_connection = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        # Reset the input buffer to clear incomplete data.
        self.serial_connection.reset_input_buffer()
        time.sleep(0.5)  # sleep for half second for buffer to clear.

        self.wheel_radius = 0.057  # [m]
        self.track = 0.28  # [m]

        self.twist_publisher = self.create_publisher(
            TwistStamped, 'rear_wss', 10)

        # Wait until we receive from the arduino that the connection is complete.
        msg = ''
        self.get_logger().info('Waiting for Arduino-Rpi connection to complete.')
        while msg != 'Setup Complete':
            if self.serial_connection.in_waiting > 0:
                msg = self.serial_connection.readline().decode('utf-8').rstrip()
        self.get_logger().info('Arduino-Rpi connection complete.')

        # Begin sensor loop.
        self.wss_timer = self.create_timer(0.01, self.get_wss)

    def get_wss(self):
        lines = self.serial_connection.readlines(10)
        self.get_logger().info(f'Read from serial connection: {len(lines)}')
        parsed_lines = [l.decode('utf-8').rstrip().split(',') for l in lines]
        self.get_logger().info(f'stripped lines: {len(lines)}')
        has_rl = False
        has_rr = False
        for line in parsed_lines.reverse():
            if line[0] == 'RL' and not has_rl:
                rl_wss = float(line[1])*self.wheel_radius
                self.get_logger().info(f'RL: {rl_wss}')
                has_rl = True
            if line[0] == 'RR' and not has_rr:
                rr_wss = float(line[1])*self.wheel_radius
                self.get_logger().info(f'RR: {rl_wss}')
                has_rr = True
            if has_rr and has_rl:
                break
        self.get_logger().info(f'Exited loop. Creating twist message.')
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = (rr_wss + rl_wss) / 2
        twist.twist.angular.z = (rr_wss - rl_wss) / self.track
        self.twist_publisher.publish(twist)
        self.get_logger().info(f'Finished publishing, clearing connection.')
        self.serial_connection.flush()


def main(args=None):
    rclpy.init(args=args)

    rear_wss = RearWss()

    rclpy.spin(rear_wss)

    # Destroy the node explicitly
    rear_wss.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
