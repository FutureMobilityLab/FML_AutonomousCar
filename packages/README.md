# Packages 

The Future Mobility Lab, autonomous vehicle project requires external code in order to function. The easiest way to install these dependencies is via Debian packages which are typically installed via the `sudo apt install ...` command. However doing this manually is laborious and error prone. In order to simplify the users experience, the cleanest way of automatically installing or removing all of these dependencies, is to create our own Debian package which can be installed or removed by a single command and will neatly install or remove all of the other requirements this project needs.   


## Current Packages 

1. **fml-acados-deps**: This package will pull in all of the dependencies for the Acados framework for realtime MPC. 
2. **fml-autonomous-veh-deps**: This package will pull in all of the dependencies and make all of the system configuration changes required to run the required ROS2 packages.


## **TLDR**: 
To install this package run ... 

```
sudo apt install ./packages/releases/fml-ros2-autonomous-vehicle-deps_1.0-1_amd64.deb
```


To remove this package run ... 

```
sudo apt remove --purge fml-ros2-autonomous-vehicle-deps
```


## Debian Packages
Debian packages the defacto standard for any Debian base Linux operating system including Ubtunu, Debian, Kali Linux, Linux Mint ...etc. These packages make it deploying and maintaining code more manageable at scale. For more details than you've ever wished to know about Debian packages, what they are and how to build or use them checkout [Intro to Debian Packaging](https://wiki.debian.org/Packaging/Intro).


### Building Debian Packages 
In order to build this Debian package you must first install the following requirements. 

```
sudo apt install build-essential binutils lintian debhelper dh-make devscripts
```


```
dpkg-buildpackage -b -us -uc
```


### Installing Debian Packages 
Note that since our package is a meta package, it will be installing other packages which must be grabbed from the internet. Since all of the source code for our package 

**DO NOT** use `dpkg` to install this package (e.g ~~`sudo dpkg -i <your-package-name>.deb`~~)! The `dpkg` command is a minimalist package installer and **WILL NOT** pull external dependencies that we require from the internet. Instead, use a more full featured package manager like `apt`. This package manager will see that our package depends on other external packages and will fetch them from the internet, during the installation process.

```
sudo apt install <your-package-name>.deb
```


### Removing Debian Packages
Removing our packge from your computer will not only uninstall our package but will also uninstall all of the dependencies which our package pulled in externally, during the installation process. Passing the `purge` command will remove the our packages configuration files from the machine as well.  

```
sudo apt remove --purge <your-packge-name>
```