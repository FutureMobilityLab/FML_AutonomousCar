import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="car_slam").find(
        "car_slam"
    )
    wss_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="optical_wss"
    ).find("optical_wss")
    default_model_path = os.path.join(pkg_share, "src/description/car_description.urdf")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    rplidar_node = launch_ros.actions.Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[
            {
                "frame_id": "laser_frame",
                "angle_compensate": True,
                "scan_mode": "Standard",
                "serial_baudrate": 115200,
            }
        ],
    )
    mpu6050driver_node = launch_ros.actions.Node(
        package="mpu6050driver",
        executable="mpu6050driver",
        name="mpu6050driver_node",
        output="screen",
        emulate_tty=True,
    )
    bno055driver_node = launch_ros.actions.Node(
        package="bno055_sensor",
        executable="bno055_sensor_node",
        name="bno055_sensor_node",
        output="screen",
        emulate_tty=True,
    )
    as5600driver_node = launch_ros.actions.Node(
        package="as5600driver",
        executable="as5600driver",
        name="as5600driver_node",
        output="screen",
        emulate_tty=True,
    )
    optical_wss_node = launch_ros.actions.Node(
        package="optical_wss",
        executable="optical_wss",
        name="optical_wss",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    robot_localization_map_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_map_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/map_ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("/odometry/filtered", "/map/odometry/filtered"),
            ("/accel/filtered", "/map/accel/filtered"),
        ],
    )
    nav2_map_server_node = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": os.path.join(pkg_share, "config/lab_map.yaml")}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    nav2_amcl_node = launch_ros.actions.Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[os.path.join(pkg_share, "config/amcl.yaml")],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],  # Added in reference to Construct project, unsure if relative namespaces are causing issue
    )
    nav2_lifecycle_manager = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )
    rviz_ref_point_recast_node = launch_ros.actions.Node(
        package="topic_tools",
        executable="relay",
        name="relay",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"input_topic": "/ref_point"},
            {"output_topic": "/ref_point_rviz"},
        ],
    )
    rviz_waypoints_recast_node = launch_ros.actions.Node(
        package="topic_tools",
        executable="relay",
        name="relay",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"input_topic": "/waypoints"},
            {"output_topic": "/waypoints_rviz"},
        ],
    )
    rviz_pose_hist_recast_node = launch_ros.actions.Node(
        package="topic_tools",
        executable="relay",
        name="relay",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"input_topic": "/pose_hist"},
            {"output_topic": "/pose_hist_rviz"},
        ],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="False",
                description="Flag to enable use_sim_time",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            # mpu6050driver_node,
            bno055driver_node,
            as5600driver_node,
            optical_wss_node,
            rplidar_node,
            robot_localization_node,
            nav2_map_server_node,
            nav2_amcl_node,
            nav2_lifecycle_manager,
            rviz_ref_point_recast_node,
            rviz_waypoints_recast_node,
            rviz_pose_hist_recast_node,
        ]
    )
