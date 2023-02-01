
# AS5600 Driver for I2C
This package converts raw angle data from an AS5600 sensor to an odom message for use with Nav2 in ROS. This functionality assumes the use of the FML_AutonomousCar, with requisite gear ratio, sensor mounting, and tire installation.

## Dependencies
-  libi2c-dev

Build the package in your workspace:

    colcon build --packages-select as5600driver

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 launch as5600driver as5600driver_launch.py

