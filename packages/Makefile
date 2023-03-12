install: 
	echo "Running Make install"

build:
	(cd casadi; mkdir -p build; cd build; cmake -DWITH_PYTHON=ON .. )

clean: 

	(cd fml-acados-deps; make clean)
	rm -f *.buildinfo
	rm -f *.changes
