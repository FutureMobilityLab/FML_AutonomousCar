from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('as5600driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'as5600.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    as5600driver_node = Node(
        package='as5600driver',
        executable='as5600driver',
        name='as5600driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file]
    )

    ld.add_action(params_declare)
    ld.add_action(as5600driver_node)
    return ld
