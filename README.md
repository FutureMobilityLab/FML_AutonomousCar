# FML_AutonomousCar
Code and other documentation regarding the Future Mobility Lab scale test vehicle used for SLAM, motion planning and controls research
Note on building with colcon on an RPI:  
  colcon build --symlink-install --executor sequential --event-handlers console_direct+  
Reduce workload to processor prevents crashing

https://github.com/babakhani/rplidar_ros2 - Source to RPLIDAR
Note: Check USB authority first, update if write access not permitted:
ls -l /dev |grep ttyUSB
sudo chmod 666 /dev/ttyUSB0
