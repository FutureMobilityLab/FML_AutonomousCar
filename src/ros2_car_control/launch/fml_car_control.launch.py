import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    ctrl_pkg_share = launch_ros.substitutions.FindPackageShare(package='ros2_car_control').find('ros2_car_control')
    trxs_pkg_share = launch_ros.substitutions.FindPackageShare(package='ros2_traxxas_controls').find('ros2_traxxas_controls')
    print(trxs_pkg_share)
    traxxas_driver_node = launch_ros.actions.Node(
        package='ros2_traxxas_controls',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[os.path.join(trxs_pkg_share, 'config/motor_driver.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    car_controller_node = launch_ros.actions.Node(
        package='ros2_car_control',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[os.path.join(ctrl_pkg_share, 'config/motor_driver.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    waypoints_pub_node = launch_ros.actions.Node(
        package='ros2_car_control',
        executable='waypoints',
        name='waypoints',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        traxxas_driver_node,
        car_controller_node,
        waypoints_pub_node
    ])