#!/bin/bash
## cd to the directory containing this file, then run:

## chmod u+x dependencies.sh 

## to make this file executable
## Prior to Running, ensure that ROS Humble Hawksbill is installed
##
##  RUN COMMAND:	 sudo ./dependencies.sh


#Prosima FastRTPS Protocal - Minimizes Latency between ethernet connected ports
REQUIRED_PKG="ros-humble-rmw-fastrtps-cpp"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

#RPLIDAR ROS2 SDK Implementaion - Publishes /scan topic
#cd src
#if ! git clone https://github.com/babakhani/rplidar_ros2
#then
#  echo "RPLidar Dependencies Failed - Check if Already Installed in /src"
#else
#  echo "RPLidar Dependencies Successful"
#fi

#sudo chmod 777 -R rplidar_ros2




