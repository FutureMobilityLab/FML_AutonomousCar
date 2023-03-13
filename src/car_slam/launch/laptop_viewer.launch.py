import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_slam').find('car_slam')
    # default_model_path = os.path.join(pkg_share, 'src/description/car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/laptop_viewer.rviz')
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    heartbeat_node = launch_ros.actions.Node(
        package='ros2_car_control',
        executable='heartbeat',
        name='heartbeat',
        output='screen',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        rviz_node,
        heartbeat_node,
    ])