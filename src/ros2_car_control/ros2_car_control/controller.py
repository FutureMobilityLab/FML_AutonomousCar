import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import json
from ros2_car_control.stanleyController import StanleyController
from ros2_car_control.mpcController import MPCController
from ros2_car_control.youlaController import YoulaController
import numpy as np 

class Controller(Node):
    def __init__(self):
        super().__init__("car_controller")
        self.localization_subscriber = self.create_subscription(Odometry,'odometry/filtered',self.odometry_callback,10)
        self.heartbeat_subscriber = self.create_subscription(String,'heartbeat',self.heartbeat_callback,1)
        self.pose_subscriber = self.create_subscription(Odometry,'map/odometry/filtered',self.pose_callback,10)
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped,'cmd_ackermann',10)
        waypointsdir = open('src/ros2_car_control/config/waypoints.json') # TODO: PUT THAT PATH/NAME HERE
        waypointsfile = json.load(waypointsdir)
        # TODO: CREATE STANDALONE ADJUSTMENT
        untranslated_waypoints = waypointsfile['smoothed_wpts']
        waypoints_x = [x[0] for x in untranslated_waypoints]
        waypoints_y = [y[1] for y in untranslated_waypoints]
        self.waypoints = [[waypoints_x[i]*0.05-5.93,waypoints_y[i]*0.05-12.8] for i in range(len(waypoints_x))]
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.controller)
        self.control_method = "stanley"
        self.speed_setpoint = 1.0
        self.v = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_max = 2.0
        self.heartbeat_timeout = 0.5 #s
        self.heartbeat_alarm = 0

        match self.control_method:
            case "stanley":
                    self.controller_function = StanleyController()
            case "youla":
                    self.controller_function = YoulaController()
            case "mpc":
                    self.controller_function = MPCController()


    def odometry_callback(self,msg):
        self.v = msg.twist.twist.linear.x

    def pose_callback(self,msg):
        orientations = msg.pose.pose.orientation
        quaternion_pose = [orientations.x,orientations.y,orientations.z,orientations.w]
        [_,_,self.yaw] = euler_from_quaternion(quaternion_pose)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def heartbeat_callback(self,msg):
        time_now = self.get_clock().now()
        if time_now - self.heartbeat_last_time > self.heartbeat_timeout:
            self.heartbeat_alarm = 1
        self.heartbeat_last_time = time_now


    def controller(self):
        if self.v > self.v_max or self.heartbeat_alarm == 1:  #if odom unsafe, comms lost, etc:
            self.cmd_steer = 0.0
            self.cmd_speed = 0.0
        # TODO: ADD PATH CONSTRAINTS TO KILL MOTOR IF OUT OF RANGE
        else:
            (self.cmd_steer,self.cmd_speed) = self.controller_function.get_commands(self.waypoints,self.x,self.y,self.yaw,self.v)
            
        
        ackermann_command = AckermannDriveStamped()
        ackermann_command.header.stamp = self.get_clock().now().to_msg()
        ackermann_command.drive.speed = self.cmd_speed
        ackermann_command.drive.steering_angle = self.cmd_steer
        self.ackermann_publisher.publish(ackermann_command)

def main(args=None):
    rclpy.init(args=args)

    car_controller = Controller()

    rclpy.spin(car_controller)

    # Destroy the node explicitly
    car_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()