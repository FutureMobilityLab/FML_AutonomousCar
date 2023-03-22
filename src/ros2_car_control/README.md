# **ROS2 Car Control**

TODO: Outline of Controllers

## **Pure Pursuit**

Pure pursuit is perhaps the most simple geometric steering controller, which searches the path for a point at a set lookahead distance, and uses this reference as the desired steering angle.

## **Stanley**

This controller uses the Stanley Control Law presented for the 2005 DARPA Grand Challenge<sup>1</sup>. In which current yaw, velocity, and cross-track error are provided as inputs to the controller. This yields the following control equation.

TODO: Insert equation

## **Youla**

Currently, the discrete state-space controller matrices used in this implementation require the use of MATLAB. These files will be provided in the scripts folder of this repository. Ideally, in the future these programs will be converted for use with either Octave or Python and remove the MATLAB dependency.

TODO: Octave Testing and Integration

## **MPC**

The MPC controller used here utilizes Acados C code generation, and as such requires installation of Acados and Casadi. Code in this repository can be directly used without changing, but there is a separate process for generation as Casadi is not targeted for ARM architecture at the time of writing (Q2 2023).

### **Code Generation**

On an x86 computer with Acados and Casadi properly installed, code can be generated. In the root directory of FML_AutonomousCar, run:

```
python3 /src/ros2_car_control/MPC/DBM_Generator.py
```
A new folder **c_generated_code** should be created. This can be copied over to the pi using scp, git, or any other means.

### **Deployment to the Rasbperry Pi**

When deploying recently generated C code, which has not been rebuilt for the ARM processor on the RPI, it is necessary to regenerate the shared library.

From the FML_AutonomousCar root directory:

```
cd c_generated_code
rm -rf build
mkdir -p build
cd build
cmake .. -D BUILD_ACADOS_OCP_SOLVER_LIB = ON
make
mv *.so ../
```


### **Citations**

\[1\] - http://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf