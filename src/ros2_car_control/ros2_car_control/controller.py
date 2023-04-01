import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from ros2_car_control.stanleyController import StanleyController
from ros2_car_control.mpcController import MPCController
from ros2_car_control.youlaController import YoulaController
from ros2_car_control.purePursuitController import PurePursuitController
from ros2_car_control.fetchWaypoints import waypoints
from visualization_msgs.msg import Marker
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__("car_controller")
        self.odometry_subscriber = self.create_subscription(Odometry,'odometry/filtered',self.odometry_callback,10)
        self.heartbeat_subscriber = self.create_subscription(String,'heartbeat',self.heartbeat_callback,1)
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.pose_callback,10)
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped,'cmd_ackermann',10)
        self.point_ref_publisher = self.create_publisher(Marker,'ref_point',10)
        self.waypoints = waypoints()
        self.cmd_timer = self.create_timer(0.05, self.controller)
        self.marker_timer = self.create_timer(1.0,self.ref_point)
        self.declare_parameter("control_method","youla")
        self.control_method = self.get_parameter("control_method").value
        self.declare_parameter("v_max",2.0)
        self.v_max = self.get_parameter("v_max").value
        self.declare_parameter("heartbeat_timeout",0.5)
        self.heartbeat_timeout = self.get_parameter("heartbeat_timeout").value
        self.heartbeat_alarm = 0
        self.run_flag = 0           # Set to 1 when home testing
        self.heartbeat_last_time = self.get_clock().now().nanoseconds
        self.final_thresh = 0.5

        self.v = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.reference_point_x = 0.0
        self.reference_point_y = 0.0
        self.point_out = Marker()

        self.declare_parameter("max_steer",0.65)
        self.declare_parameter("speed_setpoint",1.0)
        control_params = {
            "max_steer"     : self.get_parameter("max_steer").value,
            "speed_setpoint": self.get_parameter("speed_setpoint").value,
        }
        match self.control_method:
            case "pp":
                    self.declare_parameter("pp_lookahead",0.5)
                    self.declare_parameter("wheelbase",.404)
                    pp_params = {
                         "L"         : self.get_parameter("wheelbase").value,
                         "lookahead" : self.get_parameter("pp_lookahead").value,
                    }
                    control_params.update(pp_params)
                    self.controller_function = PurePursuitController(self.waypoints,control_params)
            case "stanley":
                    self.declare_parameter("stanley_k",0.65)
                    self.declare_parameter("stanley_k_soft",1.0)
                    self.declare_parameter("wheelbase",.404)
                    stanley_params = {
                         "k"      : self.get_parameter("stanley_k").value,
                         "k_soft" : self.get_parameter("stanley_k_soft").value,
                         "L"      : self.get_parameter("wheelbase").value,
                    }
                    control_params.update(stanley_params)
                    self.controller_function = StanleyController(self.waypoints,control_params)
            case "youla":
                    self.declare_parameter("Youla_GcX",4)
                    self.declare_parameter("Youla_GcA",[1.8934, -1.0907, 0.4767, -0.2260,
                                                        1.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.5, 0.0, 0.0,
                                                        0.0, 0.0, 0.5, 0.0])
                    self.declare_parameter("Youla_GcB",[8.0, 0.0, 0.0, 0.0])
                    self.declare_parameter("Youla_GcC",[0.5445, -1.2909, 2.0142, -1.0329])
                    self.declare_parameter("Youla_GcD",[0.0])
                    youla_params = {
                        "n_GcX":self.get_parameter("Youla_GcX").value,
                        "GcA":  self.get_parameter("Youla_GcA").value,
                        "GcB":  self.get_parameter("Youla_GcB").value,
                        "GcC":  self.get_parameter("Youla_GcC").value,
                        "GcD":  self.get_parameter("Youla_GcD").value,
                    }
                    control_params.update(youla_params)
                    self.controller_function = YoulaController(self.waypoints,control_params)
            case "mpc":
                    self.declare_parameter("mpc_tf",1.0)
                    self.declare_parameter("mpc_N",20)
                    self.declare_parameter("mpc_nx",7)
                    self.declare_parameter("mpc_nu",1)                    
                    mpc_params = {
                        "tf": self.get_parameter("mpc_tf").value,
                        "N" : self.get_parameter("mpc_N").value,
                        "nx": self.get_parameter("mpc_nx").value,
                        "nu": self.get_parameter("mpc_nu").value,
                    }
                    control_params.update(mpc_params)
                    self.controller_function = MPCController(self.waypoints,control_params)

        self.get_logger().info(f"INITIALIZING CONTROLLER - TYPE : {self.control_method}")
        for key in control_params:
            self.get_logger().info(f"{key}: {control_params[key]}")



    def odometry_callback(self,msg):
        self.v = msg.twist.twist.linear.x

    def pose_callback(self,msg):
        orientations = msg.pose.pose.orientation
        quaternion_pose = [orientations.x,orientations.y,orientations.z,orientations.w]
        [_,_,self.yaw] = euler_from_quaternion(quaternion_pose)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.run_flag == 0:
            self.run_flag = 1 # Allow run after telemetry first captured
            self.get_logger().info(f'TELEMETRY ONLINE -- PERMITTING CONTROL START')

    def heartbeat_callback(self,msg):
        self.heartbeat_last_time = self.get_clock().now().nanoseconds

    def ref_point(self):
        self.point_out.header.frame_id = "map"
        self.point_out.type = Marker.SPHERE
        self.point_out.action = Marker.ADD
        self.point_out.ns = "reference_point"
        self.point_out.id = 1
        self.point_out.pose.position.x = self.reference_point_x
        self.point_out.pose.position.y = self.reference_point_y
        self.point_out.scale.x = 0.1
        self.point_out.scale.y = 0.1
        self.point_out.scale.z = 0.1
        self.point_out.pose.orientation.w = 1.0
        self.point_out.frame_locked = False
        self.point_out.color.a = 1.0
        self.point_out.color.r = 1.0
        self.point_out.lifetime = Duration(seconds=1).to_msg()

        self.point_ref_publisher.publish(self.point_out)


    def controller(self):
        if self.get_clock().now().nanoseconds * 10**-9 - self.heartbeat_last_time * 10**-9 > self.heartbeat_timeout:
            self.heartbeat_alarm = 1
            self.get_logger().info(f'HEARTBEAT ALARM ACTIVE')

        last_waypoint_dist = np.linalg.norm([self.waypoints.x[-1]-self.x,self.waypoints.y[-1]-self.y])
        

        if self.v > self.v_max or self.heartbeat_alarm == 1 or self.run_flag == 0 or last_waypoint_dist < self.final_thresh:
            # If odometry unsafe, heartbeat fails, run flag disabled, or close enough to end of line
            self.cmd_steer = 0.0
            self.cmd_speed = 0.0
        else:
            (self.cmd_steer,self.cmd_speed,self.reference_point_x,self.reference_point_y) = self.controller_function.get_commands(self.x,self.y,self.yaw,self.v)

        self.get_logger().info(f'Steer Command: {self.cmd_steer}')
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
