import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_slam').find('car_slam')
    default_model_path = os.path.join(pkg_share, 'src/description/car_description.urdf')
    # default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )
    rplidar_node = launch_ros.actions.Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'serial_baudrate': 115200
        }]
    )
    mpu6050driver_node = launch_ros.actions.Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050driver_node',
        output="screen",
        emulate_tty=True,
    )
    as5600driver_node = launch_ros.actions.Node(
        package='as5600driver',
        executable='as5600driver',
        name='as5600driver_node',
        output="screen",
        emulate_tty=True,
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    slam_toolbox_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/slam.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    traxxas_driver_node = launch_ros.actions.Node(
        package='ros2_traxxas_controls',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/motor_driver.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),
        # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                      description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        mpu6050driver_node,
        as5600driver_node,
        rplidar_node,
        robot_localization_node,
        # rviz_node,
        slam_toolbox_node,
        traxxas_driver_node
    ])