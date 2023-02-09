import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class TopicSubscriberCSV(Node):

    def __init__(self):
        super().__init__('csv_parser')
        self.subscription = self.create_subscription(Imu,'imu',self.imu_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription  # prevent unused variable warning

    def imu_callback(self,msg):
       self.get_logger().info('I heard: "%f"' % msg.data)

    def odom_callback(self,msg):
       self.get_logger().info('I heard: "%f"' % msg.data)





def main(args=None):
    rclpy.init(args=args)

    topic_subscriber_csv = TopicSubscriberCSV()

    rclpy.spin(topic_subscriber_csv)

    # Destroy the node explicitly
    topic_subscriber_csv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()