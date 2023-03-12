install: 
	echo "Running Make install"

	if ! dpkg -l | grep fml-acados-deps -c >>/dev/null; then sudo apt-get install fml-acados-deps*; fi
	if ! dpkg -l | grep fml-ros2-autonomous-vehicle-deps -c >>/dev/null; then sudo apt-get install fml-ros2-autonomous-vehicle-deps*; fi

purge:
	echo "purge packages"

build:
	(cd casadi; mkdir -p build; cd build; cmake -DWITH_PYTHON=ON .. )

clean: 
	(cd fml-acados-deps; make clean)
	(cd fml-ros2-autonomous-vehicle-deps; make clean)
	rm -f *.buildinfo *.changes
