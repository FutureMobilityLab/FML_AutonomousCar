#!/bin/bash
## NOTE: May not need to change permissions, abstracted board unlocking to an instruction set to improve system alteration visibility
## cd to the directory containing this file, then run:

## chmod u+x dependencies.sh 

## to make this file executable
## Prior to Running, ensure that ROS Humble Hawksbill is installed [ros-humble-desktop-full and ros-dev-tools]
##
##  RUN COMMAND:	sudo ./dependencies.sh


#Prosima FastRTPS Protocal - Minimizes Latency between ethernet connected ports
REQUIRED_PKG="ros-humble-rmw-fastrtps-cpp"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

#IMU ROS2 SDK Implementaion - Publishes imu topic
cd src
if ! git clone https://github.com/babakhani/rplidar_ros2
then
  echo "IMU Dependencies Failed - Check if Already Installed in /src"
else
  echo "IMU Dependencies Successful"
fi

#sudo chmod 777 -R rplidar_ros2

REQUIRED_PKG="ros-humble-xacro"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

REQUIRED_PKG="ros-humble-joint-state-publisher-gui"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

REQUIRED_PKG="ros-humble-joint-state-publisher-gui"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

REQUIRED_PKG="ros-humble-navigation2"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

#sudo chmod 777 -R rplidar_ros2

REQUIRED_PKG="python-smbus"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

REQUIRED_PKG="i2c-tools"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

pip3 install lgpio adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit

