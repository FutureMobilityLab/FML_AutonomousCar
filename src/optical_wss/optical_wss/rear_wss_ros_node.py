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
        time.sleep(2)  # wait for connection to be established.
        # Reset the input buffer to clear incomplete data.
        self.serial_connection.reset_input_buffer()
        time.sleep(0.5)  # sleep for half second for buffer to clear.
        self.msg_buffer = ''

        self.wheel_radius = 0.057  # [m]
        self.track = 0.28  # [m]
        # Store previous values between publications. This is like a zero-order
        # hold. The arduino is not guaranteed to publish both sensor readings
        # everytime.
        self.rl_wss = 0  # [m/s]
        self.rr_wss = 0  # [m/s]

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
        self.time_of_last_msg = self.get_clock().now().nanoseconds * 1e-9
        self.wss_timer = self.create_timer(0.02, self.get_wss)

    def get_wss(self):
        self.get_logger().info(
            f'InWaiting:{self.serial_connection.in_waiting}')
        # Read all messages, they are separated by new line character.
        self.msg_buffer += self.serial_connection.read_all().decode('utf-8')
        if len(self.msg_buffer) > 50:
            self.msg_buffer = self.msg_buffer[len(self.msg_buffer) - 50:]
        msgs = self.msg_buffer.split('\n')
        self.get_logger().info(
            f'Read:{msgs}, InWaiting:{self.serial_connection.in_waiting}')

        # It is possible that the arduino does not send a message. So we handle
        # this with a timeout.
        time_now = self.get_clock().now().nanoseconds * 1e-9
        # The first and last messages may be cutoff, but the middle messages
        # will not be.
        if len(msgs) < 3:
            self.get_logger().info(f'time:{time_now}')
            if time_now - self.time_of_last_msg > 1.:
                self.get_logger().info(
                    f'time:{time_now - self.time_of_last_msg}')
                raise IOError
        else:
            self.time_of_last_msg = time_now

        # It is assumed the last messages are the most recent.
        for msg in reversed(msgs):
            # The msgs are not guaranteed to have both RR and RL. Some of the
            # messages are cutoff. So we filter those out.
            if "RR" not in msg and "RL" not in msg:
                continue
            lines = msg.rstrip().split(',')
            # If we have a full message, the first line should be RL.
            for line in lines:
                if line[0] == 'RL':
                    self.rl_wss = float(line[1])*self.wheel_radius
                    self.rr_wss = float(line[3])*self.wheel_radius
                    self.get_logger().info(f'RL: {self.rl_wss}')
                    self.get_logger().info(f'RR: {self.rr_wss}')
                else:
                    break
        self.get_logger().info(f'Exited loop. Creating twist message.')

        # It is not guaranteed that msgs will contain messages. Publish the
        # velocity anyway.
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = (self.rr_wss + self.rl_wss) / 2
        twist.twist.angular.z = (self.rr_wss - self.rl_wss) / self.track
        self.twist_publisher.publish(twist)
        self.get_logger().info(f'Finished publishing, clearing connection.')
        self.serial_connection.flush()
        self.get_logger().info(
            f'Finished flush: InWaiting:{self.serial_connection.in_waiting}')


def main(args=None):
    rclpy.init(args=args)

    rear_wss = RearWss()

    rclpy.spin(rear_wss)

    # Destroy the node explicitly
    rear_wss.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
