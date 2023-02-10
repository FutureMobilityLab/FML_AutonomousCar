import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
import csv

## FORMAT IS AS FOLLOWS:
#  TopicID, ROSTime (sec), ROSTime (nsec), x, y, z, x_filt, y_filt, z_filt, v, v_filt

class TopicSubscriberCSV(Node):

    def __init__(self):
        super().__init__('csv_parser')
        HeaderNames = ['ROSTOPIC','TIME_SEC','TIME_NSEC','X','Y','Z','X_FILT','Y_FILT','Z_FILT','V','V_FILT']
        with open('ROS_Bag_Parsed.csv', 'w', newline='') as file:
         writer = csv.DictWriter(file, fieldnames = HeaderNames)
         writer.writeheader()
         file.close()

        self.subscription = self.create_subscription(Imu,'imu',self.imu_callback,10)
        self.subscription = self.create_subscription(AccelWithCovarianceStamped,'accel/filtered',self.filtered_accel_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription = self.create_subscription(Odometry,'odometry/filtered',self.filtered_odom_callback,10)
        self.subscription  # prevent unused variable warning

    def send_to_file(self,list):
      with open('ROS_Bag_Parsed.csv','a') as writefile:
         writer = csv.writer(writefile)
         writer.writerow(list)
         writefile.close
      
    def imu_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       x = msg.linear_acceleration.x
       y = msg.linear_acceleration.y
       z = msg.linear_acceleration.z
       csv_msg = ["imu",time_sec,time_nanosec,x,y,z, None, None, None, None, None]
       self.send_to_file(csv_msg)

    def odom_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       v = msg.twist.twist.linear.x
       csv_msg = ["odom",time_sec,time_nanosec, None, None, None, None, None, None,v, None]
       self.send_to_file(csv_msg)

    def filtered_accel_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       x_filt = msg.accel.accel.linear.x
       y_filt = msg.accel.accel.linear.y
       z_filt = msg.accel.accel.linear.z
       csv_msg = ["imu_filtered",time_sec,time_nanosec, None, None, None,x_filt,y_filt,z_filt, None, None]
       self.send_to_file(csv_msg)

    def filtered_odom_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       v_filt = msg.twist.twist.linear.x
       csv_msg = ["imu",time_sec,time_nanosec, None, None, None, None, None, None, None,v_filt]
       self.send_to_file(csv_msg)
       
    
def main(args=None):
    rclpy.init(args=args)

    topic_subscriber_csv = TopicSubscriberCSV()

    rclpy.spin(topic_subscriber_csv)

    # Destroy the node explicitly
    topic_subscriber_csv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()