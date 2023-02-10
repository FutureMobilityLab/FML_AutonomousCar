import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped

class TopicSubscriberCSV(Node):

    def __init__(self):
        super().__init__('csv_parser')
        self.subscription = self.create_subscription(Imu,'imu',self.imu_callback,10)
        self.subscription = self.create_subscription(AccelWithCovarianceStamped,'accel/filtered',self.filtered_accel_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription = self.create_subscription(Odometry,'odometry/filtered',self.filtered_odom_callback,10)
        self.subscription  # prevent unused variable warning

    def imu_callback(self,msg):
       x = msg.linear_acceleration.x
       y = msg.linear_acceleration.y
       z = msg.linear_acceleration.z
    #    print('Messages',x,y,z)

    def odom_callback(self,msg):
       v = msg.twist.twist.linear.x
    #    print('Velocity',v)

    def filtered_accel_callback(self,msg):
       x_filt = msg.accel.accel.linear.x
       y_filt = msg.accel.accel.linear.y
       z_filt = msg.accel.accel.linear.z
    #    print('Filtered Messages',x_filt,y_filt,z_filt)

    def filtered_odom_callback(self,msg):
       v_filt = msg.twist.twist.linear.x
    #    print('Filtered Velocity',v_filt)




def main(args=None):
    rclpy.init(args=args)

    topic_subscriber_csv = TopicSubscriberCSV()

    rclpy.spin(topic_subscriber_csv)

    # Destroy the node explicitly
    topic_subscriber_csv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()