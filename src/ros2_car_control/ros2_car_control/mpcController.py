import ctypes
mpc_lib = ctypes.CDLL("/home/george/FML_AutonomousCar/acados/examples/acados_python/race_cars/c_generated_code/libacados_ocp_solver_Spatialbicycle_model.so")

class MPCController():
    def get_commands(controller):
        steering_angle = 3.0
        return steering_angle