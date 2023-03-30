######################
#	Make Targets
######################
# Makefiles are very useful at providing a clean way of implementing functionality on Linux systems. 
# Typically, they are used for compiling small C/C++ projects into binary executables; however, they 
# can also be used to run other commands, such as installing packages like this one. 
#
# This Make file has 3 "make targets" (install, build, & purge), by running the command 
# "make <target>" [e.g. make install] this cmd will install the dependency packages in this repo onto 
# whatever computer the command was run on.
# 
# Reference: https://opensource.com/article/18/8/what-how-makefile

install: 
	echo "Installing acados and fml system dependencies"
	if ! dpkg -l | grep fml-acados-deps -c >>/dev/null; then sudo apt-get install fml-acados-deps*; fi
	if ! dpkg -l | grep fml-autonomous-veh-deps -c >>/dev/null; then sudo apt-get install fml-autonomous-veh-deps*; fi

build:
	echo "Building acados and fml system dependency Debian packages"
	(cd fml-autonomous-veh-deps; dpkg-buildpackage -uc -us -b)
	(cd fml-acados-deps; dpkg-buildpackage -uc -us -b)
	(cd fml-acados-deps; make clean)
	(cd fml-autonomous-veh-deps; make clean)
	(rm -f *.buildinfo *.changes)

purge:
	echo "Purging acados and fml Debian packages"
	if dpkg -l | grep fml-acados-deps -c >>/dev/null; then sudo apt-get purge fml-acados-deps* -y; fi
	if dpkg -l | grep fml-ros2-autonomous-vehicle-deps -c >>/dev/null; then sudo apt-get install fml-autonomous-veh-deps* -y; fi