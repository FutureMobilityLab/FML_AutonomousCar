import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class HeartbeatPublisher(Node):

    def __init__(self):
        super().__init__('heartbeat_publisher')
        self.publisher_ = self.create_publisher(String, 'heartbeat', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hearbeat Count: %d' % self.i
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    heartbeat_publisher = HeartbeatPublisher()

    rclpy.spin(heartbeat_publisher)

    heartbeat_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()