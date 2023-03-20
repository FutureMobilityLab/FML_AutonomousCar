import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import json, os
from ros2_car_control.stanleyController import StanleyController
from ros2_car_control.mpcController import MPCController
from ros2_car_control.youlaController import YoulaController
import numpy as np


class Controller(Node):
    def __init__(self):
        super().__init__("car_controller")
        self.odometry_subscriber = self.create_subscription(Odometry,'odometry/filtered',self.odometry_callback,10)
        self.heartbeat_subscriber = self.create_subscription(String,'heartbeat',self.heartbeat_callback,1)
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.pose_callback,10)
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped,'cmd_ackermann',10)
        timer_period = 0.1
        self.waypoints = waypoints()
        self.timer = self.create_timer(timer_period, self.controller)
        self.declare_parameter("control_method","stanley")
        self.declare_parameter("speed_setpoint",1.0)
        self.declare_parameter("v_max",2.0)
        self.control_method = self.get_parameter("control_method").value
        self.speed_setpoint = self.get_parameter("speed_setpoint").value
        self.v_max = self.get_parameter("v_max").value
        self.v = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.declare_parameter("heartbeat_timeout",0.5)
        self.heartbeat_timeout = self.get_parameter("heartbeat_timeout").value
        self.heartbeat_alarm = 0
        self.heartbeat_last_time = self.get_clock().now().nanoseconds
        # print(self.control_method)
        match self.control_method:
            case "stanley":
                    self.controller_function = StanleyController(self.waypoints)
            case "youla":
                    self.controller_function = YoulaController(self.waypoints)
            case "mpc":
                    self.controller_function = MPCController(self.waypoints)


    def odometry_callback(self,msg):
        self.v = msg.twist.twist.linear.x

    def pose_callback(self,msg):
        orientations = msg.pose.pose.orientation
        quaternion_pose = [orientations.x,orientations.y,orientations.z,orientations.w]
        [_,_,self.yaw] = euler_from_quaternion(quaternion_pose)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def heartbeat_callback(self,msg):
        time_now = self.get_clock().now().nanoseconds
        if time_now * 10**-9 - self.heartbeat_last_time * 10**-9 > self.heartbeat_timeout:
            self.heartbeat_alarm = 1
        self.heartbeat_last_time = time_now


    def controller(self):
        if self.v > self.v_max or self.heartbeat_alarm == 1 or self.x == 0.0:  #if odom unsafe, comms lost, etc:
            self.cmd_steer = 0.0
            self.cmd_speed = 0.0
        # TODO: ADD PATH CONSTRAINTS TO KILL MOTOR IF OUT OF RANGE
        else:
            (self.cmd_steer,self.cmd_speed) = self.controller_function.get_commands(self.x,self.y,self.yaw,self.v)
            
        
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

class waypoints():
     def __init__(self):
        waypointsdir = '/home/george/FML_AutonomousCar/src/ros2_car_control/config/waypoints.json'
        with open(waypointsdir,"r") as read_file:
            waypointsfile = json.load(read_file)
            untranslated_waypoints = waypointsfile['smoothed_wpts']
            self.x = np.array([x[1] for x in untranslated_waypoints]) #switch back later
            self.y = np.array([y[0] for y in untranslated_waypoints])
        
            self.x = self.x*0.05 - 0.466    #commentif not using conversions from starter map
            self.y = -self.y*0.05 +2.1

        x_diffs = np.diff(self.x)
        y_diffs = np.diff(self.y)
        self.psi = np.arctan2(y_diffs,x_diffs)
        self.psi = np.append(self.psi, self.psi[-1])

        print(self.psi)


if __name__ == '__main__':
    main()
