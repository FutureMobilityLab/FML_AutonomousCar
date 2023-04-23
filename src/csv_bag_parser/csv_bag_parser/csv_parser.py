import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped,PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
import csv

## FORMAT IS AS FOLLOWS:
#  TopicID, ROSTime (sec), ROSTime (nsec), x, y, z, x_filt, y_filt, z_filt, v, v_filt

class TopicSubscriberCSV(Node):

    def __init__(self):
        super().__init__('csv_parser')
        HeaderNames = ['ROSTOPIC','TIME_SEC','TIME_NSEC','X','Y','PSI','X_FILT','Y_FILT','Z_FILT','V','V_FILT','SPEED_CMD','STEER_CMD']
        with open('ROS_Bag_Parsed.csv', 'w', newline='') as file:
         writer = csv.DictWriter(file, fieldnames = HeaderNames)
         writer.writeheader()
         file.close()

        self.imu_subscription = self.create_subscription(Imu,'imu',self.imu_callback,10)
        self.imu_filt_subscription = self.create_subscription(AccelWithCovarianceStamped,'accel/filtered',self.filtered_accel_callback,10)
        self.odom_subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.odom_filtsubscription = self.create_subscription(Odometry,'odometry/filtered',self.filtered_odom_callback,10)
        self.amcl_subscription = self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.amcl_callback,10)
        self.cmd_subscription = self.create_subscription(AckermannDriveStamped,'cmd_ackermann',self.cmd_callback,10)

      #   self.subscription  # prevent unused variable warning

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
       csv_msg = ["imu",time_sec,time_nanosec,x,y,z, None, None, None, None, None, None, None]
       self.send_to_file(csv_msg)

    def odom_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       v = msg.twist.twist.linear.x
       csv_msg = ["odom",time_sec,time_nanosec, None, None, None, None, None, None,v, None, None, None]
       self.send_to_file(csv_msg)

    def filtered_accel_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       x_filt = msg.accel.accel.linear.x
       y_filt = msg.accel.accel.linear.y
       z_filt = msg.accel.accel.linear.z
       csv_msg = ["imu_filtered",time_sec,time_nanosec, None, None, None,x_filt,y_filt,z_filt, None, None, None, None]
       self.send_to_file(csv_msg)

    def filtered_odom_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       v_filt = msg.twist.twist.linear.x
       csv_msg = ["odom_filtered",time_sec,time_nanosec, None, None, None, None, None, None, None,v_filt, None, None]
       self.send_to_file(csv_msg)

    def amcl_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       pose_x = msg.pose.pose.position.x
       pose_y = msg.pose.pose.position.y
       orientations = msg.pose.pose.orientation
       quaternion_pose = [orientations.x,orientations.y,orientations.z,orientations.w]
       [_,_,pose_psi] = euler_from_quaternion(quaternion_pose)
       csv_msg = ["amcl_pose",time_sec,time_nanosec, pose_x, pose_y, pose_psi, None, None, None, None, None, None, None]
       self.send_to_file(csv_msg)

    def cmd_callback(self,msg):
       time_sec = msg.header.stamp.sec
       time_nanosec = msg.header.stamp.nanosec
       cmd_speed = msg.drive.speed
       cmd_steer = msg.drive.steering_angle
       csv_msg = ["cmd_ackermann",time_sec,time_nanosec, None, None, None, None, None, None, None, None, cmd_speed, cmd_steer]
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