import time, os
import numpy as np
from acados_generator import *
from getPath import getPath
import matplotlib.pyplot as plt

"""
Example of the frc_racecars in simulation without obstacle avoidance:
This example is for the optimal racing of the frc race cars. The model is a simple bicycle model and the lateral acceleration is constraint in order to validate the model assumptions.
The simulation starts at s=-2m until one round is completed(s=8.71m). The beginning is cut in the final plots to simulate a 'warm start'. 
"""
[xref, yref, psiref] = getPath("waypoints.json")

Tf = 1  # prediction horizon
N = 20  # number of discretization steps
T = 10.00  # maximum simulation time[s]
sref_N = 3  # reference for final reference progress

# load model
# _, _, acados_solver = acados_settings(Tf, N, track)  #constraint,model, acados_solver
acados_solver = AcadosOcpSolver(None,generate=False,build=True,json_file="acados_ocp.json")
# acados_solver = AcadosOcpSolver.build('/home/george/FML_AutonomousCar/acados/examples/acados_python/race_cars/c_generated_code')
# dimensions
nx = 5 # model.x.size()[0]
nu = 1 # model.u.size()[0]
# print("size nx, nu: {} | {}".format(nx,nu))
ny = nx + nu
Nsim = int(T * N / Tf)

# initialize data structs
simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))
s0 = -2 #model.x0[0]
tcomp_sum = 0
tcomp_max = 0

# simulate
for i in range(Nsim):
    # update reference
    sref = s0 + sref_N
    for j in range(N):
        yref = np.array([s0 + (sref - s0) * j / N, 0, 0, 0, 0, 0, 0, 0])
        acados_solver.set(j, "yref", yref)
    yref_N = np.array([sref, 0, 0, 0, 0, 0])
    acados_solver.set(N, "yref", yref_N)

    # solve ocp
    t = time.time()

    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))

    elapsed = time.time() - t
    # manage timings
    tcomp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed

    # get solution
    x0 = acados_solver.get(0, "x")
    u0 = acados_solver.get(0, "u")
    for j in range(nx):
        simX[i, j] = x0[j]
    for j in range(nu):
        simU[i, j] = u0[j]

    # update initial condition
    x0 = acados_solver.get(1, "x")
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)
    s0 = x0[0]

    #check if done and terminate TODO

# # Print some stats
print("Average computation time: {}".format(tcomp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))
print("Average speed:{}m/s".format(np.average(simX[:, 3])))
print("Lap time: {}s".format(Tf * Nsim / N))
# avoid plotting when running on Travis
# if os.environ.get("ACADOS_ON_CI") is None:
#     plt.show()
