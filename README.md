# **FML_AutonomousCar**
Code and documentation for the Future Mobility Lab scale test vehicle used for SLAM, motion planning and controls research. This repo assumes installation of ROS 2 Humble Hawksbill on an Ubuntu 22.04 machine. Tested on a Raspberry Pi 4b with 8 GB RAM.

# Installation:
This repo requires the installation of [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) on [Ubuntu 22.04 LTS Server or Ubuntu 22.04 LTS Desktop](https://releases.ubuntu.com/jammy/) running on a Raspberry Pi 4b with 8 GB RAM.

```
sudo apt install git -y && git clone git@github.com:FutureMobilityLab/FML_AutonomousCar.git
cd ./FML_AutonomousCar && git submodule update --init --recursive
```

NOTE: After cloning the repo, we need to initialize all of the [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) by using the `git submodule` command to pull in all of the external repositories that this project depends on being installed before building our ROS2 environment.

## Install Casadi & Acados

Casadi and Acados is only required to run the Model Predictive Controller (MPC). You can skip this part if you do not plan on running the MPC.

### Acados Dependencies Installation
For Raspberry Pi, use the arm64 installation commands.
```
#########################
#   Arm64 Architecture  #
#########################
sudo apt install ./packages/releases/arm64/fml-acados-deps_1.0-1_arm64.deb -y

#########################
#   Amd64 Architecture  #
#########################
sudo apt install ./packages/releases/amd64/fml-acados-deps_1.0-1_amd64.deb -y
```

NOTE: To remove the dependencies off the system run ... 
```
sudo apt purge fml-acados-deps
```

### Install Casadi

ARM64 INSTALLATION

```
cd ..  # step back one level above FML_AutonomousCar
git clone https://github.com/casadi/casadi.git casadi
cd casadi
git checkout 3.5.5
mkdir build
cd build
cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON ..
make
sudo make install
```

AMD64 INSTALLATION
```
pip install casadi

```

### Install Acados
```
cd ../..
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4
```

Find Absolute Path to ACADOS Root folder, replace following lines <acados_root> (ie. /home/boba/acados)

```
pip install -e <acados_root>/interfaces/acados_template --no-deps

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
export ACADOS_SOURCE_DIR="<acados_root>"
```

## Install ROS2 Humble Hawksbill 

### Setup UTF8 & Ros2 Repo Sources

```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS2 on RaspberryPi 
```
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Install ROS2 on Desktop
```
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Install ROS2 Dependencies 

The Debian package that installs all of the external dependencies this repo uses is CPU architecture specific. Most desktop/laptops with Intel or AMD CPUs are `amd64` while raspberry pis (ie. Raspberry Pi 4 Model B) are `arm64/aarch64`. NOTE: (Apple Silicon is also `arm64`)

To determine which architecture your system has run the following command ... 

```
dpkg --print-architecture 
```

Then install the correct package as shown below...

```
#########################
#   Arm64 Architecture  #
#########################
sudo apt install ./packages/releases/arm64/fml-autonomous-veh-deps_1.0-1_arm64.deb -y

#########################
#   Amd64 Architecture  #
#########################
sudo apt install ./packages/releases/amd64/fml-autonomous-veh-deps_1.0-1_amd64.deb -y
```

NOTE: To remove the dependencies off the system run ... 
```
sudo apt purge fml-autonomous-veh-deps
```

## Setup Raspberry Pi 4 to use BNO055 Sensor
If you are using the BNO055 sensor there is a known problem with Raspberry Pi. Raspberry Pi I2C hardware does not fully support clock stretching. One ["fix"](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching) is to lower the baud rate of the I2C pins, but this often results in large spikes in the data and limits the update rate of reading the sensor. A better solution is to use [GPIO overlays](https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes/software-i2c) to convert some of Raspberry Pi's GPIO pins into I2C pins. The instructions in these links require you to modify `/boot/config.txt`, however if you are using Ubuntu 22 you will not have this file and instead you should modify `/boot/firmware/config.txt`. You can add the following lines:

```
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=17,i2c_gpio_scl=27,i2c_gpio_delay_us=2
```
The `i2c_gpio_sda` and `i2c_gpio_scl` do not need to be these specific pins. Use whichever are available on your Raspberry Pi 4. You also may not need the `i2c_gpio_delay_us` argument. After making these changes reboot your device. You can verify that the BNO055 is detected and which address it is using by calling `i2cdetect -y 3`.

> A note on BNO055 breakout boards: If you are using ADAFruit's breakout board please refer to their documentation to make sure the BNO055 is configured to use i2c. If you are using the WCMCU-055 breakout board (it's purple and has VCC, GND, ATX, LRX, I2C, INT, RES, and BOOT pinouts) you will need to bridge the S0 and S1 pins to negative. This will configure the device to use I2C. You will also need to use 3.3v to power the board, not the 5v power.

Once you have setup the Raspberry Pi to connect with the BNO055 over I2C, please follow the install instructions in the README for the ros2_bno055_sensor submodule. Mainly you will have to (1) clone the submodules if you have not already done so, and (2) apply a software patch. After this you are ready to build.

## Build ROS2 Workspace 

**IMPORTANT!!!: Before you continue, reboot the system and change into the FML repo directory and then run the following command.**

```
colcon build
```

## Post Install Preparation
This project uses the I2C protocol to communicate with most of its sensors and actuators. Raspberry Pi 4 Model B has dedicated I2C pins (GPIO 2 & 3, a.k.a. pins 3 & 5).
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

## Networking two Raspberry Pis over ethernet
With two Raspberry Pis you will want all ROS communication to go through an ethernet connection rather over wifi. For easy workflows, it would also be nice to have both Raspberry Pis connected to the internet. This can be achieved by having both connected over ethernet and to the wifi. This will allow a laptop to ssh into either computer. To set this up, there are several resources available on the internet.

You will need to consider one of the Raspberry Pis as the network sharer. It doesn't matter which, but we prefer to use the Pi that will run localization rather than the one that runs control.

### Setup the localization (Here I Am) Raspberry Pi network
First you will setup a new connection (you can change the con-name if you wish) that enables internet sharing and sets a static IP address: 
```
nmcli connection add type ethernet ifname eth0 ipv4.method shared con-name local
```
The shared IPv4 method does multiple things:

- enables IP forwarding for the interface;
- adds firewall rules and enables masquerading;
- starts dnsmasq as a DHCP and DNS server.

You can check the status of the network devices:
```
nmcli d
```

10.42.0.1/24  is the default IP address set by NetworkManager. You may need to change this if it conflicts with an existing IP address:
```
nmcli connection modify local ipv4.addresses 192.168.42.1/24
```

You can verify the IP address for `eth0` by running `ifconfig`.

Remember to activate again the connection profile after any change to apply the new values:
```
nmcli connection up local

ip -o addr show enp1s0
```

To verify the connected devices you can run `nmcli d` or to verify the connections, run `ncmli c`.

### Setup the control (Here I Go) Raspberry Pi network

run `sudo nano /etc/netplan/50-cloud-init.yaml` to open and edit the netplan yaml configuration file. You may have a different name in `/etc/netplan/` or multiple, but try to find the one with wifi and ethernet. 

Inside this file, modify it so you have: 
```                                        
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    version: 2
    ethernets:
        renderer: NetworkManager
        eth0:
           dhcp4: no
           addresses:
               - 10.42.0.2/24
           routes:
               - to: default
                 via: 10.42.0.1
           nameservers:
                 addresses: [10.42.0.1]
    wifis:
        renderer: NetworkManager
        wlan0:
            access-points:
                 "WifiName":
                    password: "wifi_password"
            dhcp4: true
            optional: true
```
Note: yaml uses strict indentations, so take care to keep everything indented the same amount.

After saving, run `sudo apply netplan`.

Verify that the ethernet device is running with `nmcli d`. If you do not see the ethernet device as green (connected) you will need to turn on that connection. Do this with `nmcli c up <name-of-ethernet-connection`. For understanding the usage of `nmcli` run `nmcli --help`, `nmcli c --help`, `nmcli d --help`.

You should now be able to ping each Pi from the other Pi over ethernet. Use the IP addresses for each. You can run `ifconfig` to find those IP addresses.

## Robot Configuration
```
ros2 launch car_slam laptop_viewer.launch.py
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
ros2 launch car_slam localization.launch.py
```
On the localization module. To run the test, run the controller launch file on the control module:
```
ros2 launch ros2_car_control fml_car_control.launch.py
```



## References 
Nav2 Installation Guide:  
https://navigation.ros.org/getting_started/index.html#installation  

MPU6050 Plugin:  
https://github.com/hiwad-aziz/ros2_mpu6050_driver  

ROS2 Humble Hawksbill:  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html  

=======
# **Useful Aliases**:

Some Useful Aliases can be added to streamline workflow on the car

Connect to the RPI (assumes names of rpi-hia and rpi-hig):
```
alias rpi-{hia or hig}="ssh {USERNAME}@{COMPUTER}-rpi4.local"
```
At the FML_AutonomousCar Root Directory, setup the car parameters for running. This can also be added directly to ./bashrc but is kept in an alias to minimally alter the system when not running the car.
```
alias fml="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && source ./install/setup.bash && sudo chmod 777 /dev/ttyUSB0 && sudo chmod 777 /dev/i2c-1"
```
