##############################
# 		  TO DO LIST:
##############################
# 0) Make Sure git submodules are updated and pointing to the correct branches
# 1) Install FML Ros2 Dependencies
# 2) Install Acados Dependencies 
# 3) Build Casadi from source (arm64 bullshit)
# 4) Build Acados (requires the casadi build)
# 5) Pip install acados_template and set ENV vars.
# 6) Add target for removing all dependencies 
# 7) Add target for removing Acados dependencies
# 8) Add target for removing FML ROS2 dependencies
##############################	


all:
	echo "Make all"

build:
	echo "Make build"
	(make build_deps)

install: 
	echo "Make Install"

clean: 
	echo "Make clean"


#############################################
#		Dependency Targets
#############################################

#     Build Targets 

build_ros2_deps:
	(cd packages/fml-ros2-autonomous-vehicle-deps; dpkg-buildpackage -uc -us -b)

build_acados_deps:
	(cd packages/fml-acados-deps; dpkg-buildpackage -uc -us -b)

build_deps:
	(make build_acados_deps)
	(make build_ros2_deps)
	(cd packages; make clean)



#     Clean Targets 
clean_ros2_deps:
	(cd packages/fml-ros2-autonomous-vehicle-deps; make clean)

clean_acados_deps:
	(cd packages/fml-acados-deps; make clean)
	(make clean_aca)

#     Installation Targets 
# install_ros2_deps:
#     [ -z `dpkg -l | grep fml-ros2-autonomous-vehicle-deps` ] && sudo apt install ./packages/fml-ros2-autonomous-vehicle-deps*.deb

# remove_ros2_deps:
# install_acados_deps:
# 	[ -z `dpkg -l | grep fml-acados-deps` ] && sudo apt install ./packages/fml-acados-deps*.deb
# remove_acados_deps:


##############################################
#		Submodule Targets
#############################################


build_casadi:

build_acados:

install_acados:




	(cd casadi; mkdir -p build; cd build; cmake -DWITH_PYTHON=ON .. )





