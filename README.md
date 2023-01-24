# FML_AutonomousCar
Code and other documentation regarding the Future Mobility Lab scale test vehicle used for SLAM, motion planning and controls research. This repo assumes installation of ROS 2 Humble Hawksbill on an Ubuntu 22.04 machine. Tested on a Raspberry Pi 4b with 8 GB RAM.

ROS Humble Hawksbill Installation Guide:  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html  

## Dependency References (Installed during setup)

Nav2 Installation Guide:  
https://navigation.ros.org/getting_started/index.html#installation  

MPU6050 Plugin:
https://github.com/hiwad-aziz/ros2_mpu6050_driver  

## Installation:

1. Clone this repository to a local machine
2. Run dependencies.sh
```
. dependencies.sh
```
3. Build the workspace
```
  colcon build --symlink-install --executor sequential --event-handlers console_direct+  
```
Note on building with colcon on an RPI - Additional build parameters reduce workload to processor,  which prevents crashing when working with larger packages


Note: Check USB/I2C authority first:
```
ls -l /dev |grep ttyUSB && ls -l /dev |grep i2c-1
```
Update if write access not permitted:
```
sudo chmod 666 /dev/ttyUSB0  
sudo chmod 666 /dev/i2c-1
```
