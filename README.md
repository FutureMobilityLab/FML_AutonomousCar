# **FML_AutonomousCar**
Code and documentation for the Future Mobility Lab scale test vehicle used for SLAM, motion planning and controls research. This repo assumes installation of ROS 2 Humble Hawksbill on an Ubuntu 22.04 machine. Tested on a Raspberry Pi 4b with 8 GB RAM.

ROS Humble Hawksbill Installation Guide:  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html  

## Dependency References (Installed during setup)

Nav2 Installation Guide:  
https://navigation.ros.org/getting_started/index.html#installation  

MPU6050 Plugin:  
https://github.com/hiwad-aziz/ros2_mpu6050_driver  

## Installation (Coming Soon: Replacement with Debian Package):

1. Clone this repository to a local machine
2. Run dependencies.sh
```
cd FML_AutonomousCar && . dependencies.sh
```
3. Build the workspace
```
colcon build --symlink-install
```
Note on building with colcon on an RPI:  
If building fails due to overload of compute unit, it is possible to use 
```
--executor sequential --event-handlers console_direct+ 
```
arguments to reduce workload to the cpu and improve stability.

## Post Install Preparation
Check USB/I2C authority, and update write access if not permitted (generally it will not be):
```
ls -l /dev |grep ttyUSB && ls -l /dev |grep i2c-1
```
```
sudo chmod 666 /dev/ttyUSB0  
sudo chmod 666 /dev/i2c-1
```

Prior to running, it is necessary to set all rmw implementations to fast rtps:
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
In our experience, these settings will need to be reset each time the RPI is restarted. This can be streamlined by adding the following alias to the end of .bashrc
```
alias fml="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && source ./install/setup.bash && sudo chmod 777 /dev/ttyUSB0 && sudo chmod 777 /dev/i2c-1"
```
which can then simply be run in the root folder with:
```
fml
```

# **Running the Car**

## Format
The FML Autonomous Car operates on a principle of distributed computing, and as such has functionalities split across the two required Raspberry Pi units. Therefore it is necessary to configure both SBC's with the instructions above or otherwise generate two copies of the same configured drive. When running, the Raspberry Pi's and remote laptop each have a unique purpose. The laptop sends a remote heartbeat signal, that will direct the car to kill all motor commands on loss of connection. One Raspberry Pi handles all localization and pose estimation, while the other determines control outputs and commands the motors. For ease of understanding, one unit can be considered the 'Here I Am' module and the other as the 'Here I Go' module.

## Robot Configuration
```
ros2 launch car_slam fml_car_display.launch.py
```
Use this launch file to test the configuration of the car, and verify that the sensors, robot model, and general structure of the car are correct before moving onto the next step. This launch file opens Rviz2 and provides a quick format to test ros2 topics and validate links and transforms.

## Mapping (Coming Soon: Rework for SLAM Toolbox)
```
ros2 launch car_slam fml_car_mapping.launch.py
```
Use this launch file to perform a mapping session. This will use slam-toolbox to generate a map of the region to be used for testing. To control the robot, use the Ubuntu Hotspot and connect the controller module to this network. Then run the following:
```
ros2 run ros2_traxxas_controls keyboard_teleop_hold
```
This will enable the remote laptop to command the motors using WASD or arrow keys. Once a mapping session is complete, run the Nav2 map saver:
```
ros2 run nav2_map_server map_saver_cli
```
This will save the map .pgm and .yaml files to the current directory.

## Testing
Copy the .pgm and .yaml map files into the car_slam config folder. [TODO: Explain Naming Process and config for loading maps]. Then, run:
```
ros2 launch car_slam fml_car_localization.launch.py
```
On the localization module. To run the test, run the controller launch file on the control module:
```
ros2 launch ros2_car_control fml_car_control.launch.py
```
