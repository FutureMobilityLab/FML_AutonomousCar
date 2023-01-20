# FML_AutonomousCar
Code and other documentation regarding the Future Mobility Lab scale test vehicle used for SLAM, motion planning and controls research
Note on building with colcon on an RPI:  
  colcon build --symlink-install --executor sequential --event-handlers console_direct+  
Reduce workload to processor prevents crashing

Nav2 Installation Guide:  
https://navigation.ros.org/getting_started/index.html#installation  

RPLIDAR SDK:  
https://github.com/babakhani/rplidar_ros2  

MPU6050 Plugin:
https://github.com/hiwad-aziz/ros2_mpu6050_driver  

Note: Check USB/I2C authority first, update if write access not permitted:  
ls -l /dev |grep ttyUSB  
sudo chmod 666 /dev/ttyUSB0  
ls -l /dev |grep ttyUSB  
sudo chmod 666 /dev/i2c-1
