import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node, QoSProfile
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from ros2_car_control.stanleyController import StanleyController
from ros2_car_control.mpcController import MPCController
from ros2_car_control.ltiController import LTIController
from ros2_car_control.openLoopController import OpenLoopChirp
from ros2_car_control.purePursuitController import PurePursuitController
from ros2_car_control.pvYoula import PVYoulaController
from ros2_car_control.fetchWaypoints import waypoints
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
import numpy as np
import sys


class Controller(Node):
    def __init__(self):
        super().__init__("car_controller")
        FMLCarQoS = QoSProfile(history=1, depth=1, reliability=2, durability=2)
        ctrl_sample_time = 0.05

        # Create transform listener to get current pose in map frame for position
        # feedback.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_frame = ""
        self.map_frame = "map"

        # Subscribe to heartbeat for guaranteeing healthy communication with laptop.
        # self.heartbeat_subscriber = self.create_subscription(
        #     String, "heartbeat", self.heartbeat_callback, FMLCarQoS
        # )

        # Subscribe to odometry for current velocity.
        self.v = 0.0
        self.odometry_subscriber = self.create_subscription(
            Odometry, "odometry/filtered", self.odometry_callback, FMLCarQoS
        )

        # Publish control commands.
        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, "cmd_ackermann", FMLCarQoS
        )

        # Publish the waypoint used as reference for control.
        self.point_ref_publisher = self.create_publisher(Marker, "ref_point", FMLCarQoS)

        # Publish the current pose used as feedback in control (to simplify data
        # analysis).
        self.pose_publisher = self.create_publisher(
            PoseStamped, "current_pose", FMLCarQoS
        )

        # Define generic node parameters.
        self.declare_parameter("control_method", "stanley")
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("heartbeat_timeout", 1.0)
        self.declare_parameter("max_steer", 0.65)
        self.declare_parameter("speed_setpoint", 1.0)
        self.declare_parameter("max_accel", 1.0)

        # Get generic parameter values.
        self.control_method = self.get_parameter("control_method").value
        self.max_speed = self.get_parameter("max_speed").value
        self.heartbeat_timeout = self.get_parameter("heartbeat_timeout").value
        control_params = {
            "max_steer": self.get_parameter("max_steer").value,
            "speed_setpoint": self.get_parameter("speed_setpoint").value,
            "max_accel": self.get_parameter("max_accel").value,
        }

        # Define safety-check flags.
        self.heartbeat_alarm = 0
        self.run_flag = 0  # Set to 1 when home testing
        self.heartbeat_last_time = self.get_clock().now().nanoseconds
        # self.final_thresh = 0.5

        # Get waypoints from json, the specific controller is responsible for finding
        # and updating the reference points x and y.
        self.waypoints = waypoints()
        self.reference_point_x = 0.0
        self.reference_point_y = 0.0
        self.reference_point_psi = 0.0

        # Define and get controller-specific parameters.
        match self.control_method:
            case "pp":
                self.declare_parameter("pp_lookahead", 0.5)
                self.declare_parameter("wheelbase", 0.404)
                pp_params = {
                    "L": self.get_parameter("wheelbase").value,
                    "lookahead": self.get_parameter("pp_lookahead").value,
                }
                control_params.update(pp_params)
                self.controller_function = PurePursuitController(
                    self.waypoints, control_params
                )
            case "stanley":
                self.declare_parameter("stanley_k", 0.65)
                self.declare_parameter("stanley_k_soft", 1.0)
                self.declare_parameter("wheelbase", 0.404)
                stanley_params = {
                    "k": self.get_parameter("stanley_k").value,
                    "k_soft": self.get_parameter("stanley_k_soft").value,
                    "L": self.get_parameter("wheelbase").value,
                }
                control_params.update(stanley_params)
                self.controller_function = StanleyController(
                    self.waypoints, control_params
                )
            case "youla":
                self.declare_parameter("Youla_GcX", 1)
                self.declare_parameter("Youla_GcA", [0.0])
                self.declare_parameter("Youla_GcB", [0.0])
                self.declare_parameter("Youla_GcC", [0.0])
                self.declare_parameter("Youla_GcD", [0.0])
                self.declare_parameter("Youla_lookahead", 0.0)
                youla_params = {
                    "n_GcX": self.get_parameter("Youla_GcX").value,
                    "GcA": self.get_parameter("Youla_GcA").value,
                    "GcB": self.get_parameter("Youla_GcB").value,
                    "GcC": self.get_parameter("Youla_GcC").value,
                    "GcD": self.get_parameter("Youla_GcD").value,
                    "lookahead": self.get_parameter("Youla_lookahead").value,
                }
                control_params.update(youla_params)
                self.controller_function = LTIController(self.waypoints, control_params)
            case "hinf":
                self.declare_parameter("hinf_n_GcX", 1)
                self.declare_parameter("hinf_GcA", [0.0])
                self.declare_parameter("hinf_GcB", [0.0])
                self.declare_parameter("hinf_GcC", [0.0])
                self.declare_parameter("hinf_GcD", [0.0])
                self.declare_parameter("hinf_lookahead", 0.0)
                hinf_params = {
                    "n_GcX": self.get_parameter("hinf_n_GcX").value,
                    "GcA": self.get_parameter("hinf_GcA").value,
                    "GcB": self.get_parameter("hinf_GcB").value,
                    "GcC": self.get_parameter("hinf_GcC").value,
                    "GcD": self.get_parameter("hinf_GcD").value,
                    "lookahead": self.get_parameter("hinf_lookahead").value,
                }
                control_params.update(hinf_params)
                self.controller_function = LTIController(self.waypoints, control_params)
            case "mpc":
                self.declare_parameter("mpc_tf", 1.0)
                self.declare_parameter("mpc_N", 20)
                self.declare_parameter("mpc_nx", 7)
                self.declare_parameter("mpc_nu", 1)
                mpc_params = {
                    "tf": self.get_parameter("mpc_tf").value,
                    "N": self.get_parameter("mpc_N").value,
                    "nx": self.get_parameter("mpc_nx").value,
                    "nu": self.get_parameter("mpc_nu").value,
                }
                control_params.update(mpc_params)
                self.controller_function = MPCController(self.waypoints, control_params)
            case "open_chirp":
                self.declare_parameter("open_chirp_start_frequency_hz", 0.0)
                self.declare_parameter("open_chirp_end_frequency_hz", 4.0)
                self.declare_parameter("open_chirp_start_amplitude_rad", 0.5)
                self.declare_parameter("open_chirp_end_amplitude_rad", 0.25)
                self.declare_parameter("open_chirp_duration_s", 4.0)
                self.declare_parameter("open_chirp_settling_time_s", 1.0)
                open_chirp_params = {
                    "start_frequency_hz": self.get_parameter(
                        "open_chirp_start_frequency_hz"
                    ).value,
                    "end_frequency_hz": self.get_parameter(
                        "open_chirp_end_frequency_hz"
                    ).value,
                    "start_amplitude_rad": self.get_parameter(
                        "open_chirp_start_amplitude_rad"
                    ).value,
                    "end_amplitude_rad": self.get_parameter(
                        "open_chirp_end_amplitude_rad"
                    ).value,
                    "duration_s": self.get_parameter("open_chirp_duration_s").value,
                    "settling_time_s": self.get_parameter(
                        "open_chirp_settling_time_s"
                    ).value,
                    "controller_clock": self.get_clock(),
                }
                control_params.update(open_chirp_params)
                self.controller_function = OpenLoopChirp(control_params)
            case "PV_Youla":
                self.declare_parameter("PV_Youla_n_GcX", 5.0)
                self.declare_parameter("PV_Youla_lookahead_gain", 0.8)
                pv_youla_params = {
                    "n_GcX": self.get_parameter("PV_Youla_n_GcX").value,
                    "ctrl_sample_time_s": ctrl_sample_time,
                    "lookahead_gain": self.get_parameter(
                        "PV_Youla_lookahead_gain"
                    ).value,
                }
                control_params.update(pv_youla_params)
                self.controller_function = PVYoulaController(
                    self.waypoints, control_params
                )

        # Log controller configuration.
        self.get_logger().info(
            f"INITIALIZING CONTROLLER - TYPE : {self.control_method}"
        )
        for key in control_params:
            self.get_logger().info(f"\t{key}: {control_params[key]}")

        # Start timers.
        self.cmd_timer = self.create_timer(ctrl_sample_time, self.controller)

    # def heartbeat_callback(self, msg):
    #     # When a heartbeat is received, update the time it was received.
    #     self.heartbeat_last_time = self.get_clock().now().nanoseconds

    def odometry_callback(self, msg):
        self.v = msg.twist.twist.linear.x
        # Allow controller to run after first odometry captured.
        if self.run_flag == 0:
            self.run_flag = 1

    def publish_ref_point_marker(self):
        """Publish an RVIZ marker to display the reference point being used."""
        ref_point_marker = Marker()
        ref_point_marker.header.frame_id = "map"
        ref_point_marker.type = Marker.SPHERE
        ref_point_marker.action = Marker.ADD
        ref_point_marker.ns = "reference_point"
        ref_point_marker.id = 1
        ref_point_marker.pose.position.x = self.reference_point_x
        ref_point_marker.pose.position.y = self.reference_point_y
        (qx, qy, qz, qw) = quaternion_from_euler(0.0, 0.0, self.reference_point_psi)
        ref_point_marker.pose.orientation.x = qx
        ref_point_marker.pose.orientation.y = qy
        ref_point_marker.pose.orientation.z = qz
        ref_point_marker.pose.orientation.w = qw
        ref_point_marker.scale.x = 0.1
        ref_point_marker.scale.y = 0.1
        ref_point_marker.scale.z = 0.1
        ref_point_marker.frame_locked = False
        ref_point_marker.color.a = 1.0
        ref_point_marker.color.r = 1.0
        ref_point_marker.lifetime = Duration(seconds=1).to_msg()

        self.point_ref_publisher.publish(ref_point_marker)

    def publish_current_pose(self, pose, time_stamp):
        """Publish the vehicle's pose."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = time_stamp
        pose_stamped.pose = pose
        self.pose_publisher.publish(pose_stamped)

    def controller(self):
        """Publish current_pose, ref_point marker, and control command."""

        # Get current pose from transform.
        from_frame = "base_link"
        to_frame = "map"
        try:
            t = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame} to {from_frame}: {ex}"
            )
            return
        current_pose_time_stamp = t.header.stamp
        quaternion_pose = [
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ]
        [_, _, yaw] = euler_from_quaternion(quaternion_pose)
        current_pose = Pose()
        current_pose.position.x = t.transform.translation.x
        current_pose.position.y = t.transform.translation.y
        current_pose.position.z = t.transform.translation.z
        current_pose.orientation.x = t.transform.rotation.x
        current_pose.orientation.y = t.transform.rotation.y
        current_pose.orientation.z = t.transform.rotation.z
        current_pose.orientation.w = t.transform.rotation.w
        self.publish_current_pose(current_pose, current_pose_time_stamp)

        # Publish reference point marker.
        self.publish_ref_point_marker()

        # Compute control commands.
        (
            self.cmd_steer,
            self.cmd_speed,
            self.reference_point_x,
            self.reference_point_y,
            self.reference_point_psi,
        ) = self.controller_function.get_commands(
            current_pose.position.x, current_pose.position.y, yaw, self.v
        )
        # Clip steer angle to max steer.
        max_steer = self.get_parameter("max_steer").value
        self.cmd_steer = np.clip(self.cmd_steer, -max_steer, max_steer)
        if abs(self.cmd_steer) >= max_steer:
            self.get_logger().warning(
                f"Steering saturated. Requested < |{max_steer}|, but got {self.cmd_steer}"
            )

        # # Raise heartbeat_alarm if heartbeat hasn't been received.
        # if (
        #     self.get_clock().now().nanoseconds * 10**-9
        #     - self.heartbeat_last_time * 10**-9
        #     > self.heartbeat_timeout
        # ):
        #     self.heartbeat_alarm = 1
        #     self.get_logger().info("HEARTBEAT ALARM ACTIVE")
        #     self.cmd_steer = 0.0
        #     self.cmd_speed = 0.0

        # last_waypoint_dist = np.linalg.norm(
        #     [self.waypoints.x[-1] - self.x, self.waypoints.y[-1] - self.y]
        # )

        # Handle each error separately for clear info message.
        if self.v > self.max_speed:
            self.cmd_steer = 0.0
            self.cmd_speed = 0.0
            self.get_logger().info("Speed unsafe")
        if self.run_flag == 0:
            self.cmd_steer = 0.0
            self.cmd_speed = 0.0
            self.get_logger().info("Run flag disabled")
        # if last_waypoint_dist < self.final_thresh:
        #     self.cmd_steer = 0.0
        #     self.cmd_speed = 0.0
        #     self.get_logger().info("Close to end of line.")

        # Publish controller command.
        ackermann_command = AckermannDriveStamped()
        ackermann_command.header.stamp = self.get_clock().now().to_msg()
        ackermann_command.drive.speed = self.cmd_speed
        ackermann_command.drive.steering_angle = self.cmd_steer
        self.ackermann_publisher.publish(ackermann_command)


def main(args=None):
    rclpy.init(args=args)

    car_controller = Controller()

    try:
        rclpy.spin(car_controller)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        car_controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
