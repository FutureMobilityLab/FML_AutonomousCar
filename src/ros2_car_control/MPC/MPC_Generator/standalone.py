import time, os
import numpy as np
from DBM_generator import *
from getPath import getPath
import matplotlib.pyplot as plt
'''
STATES: X, Y, x, y, y_dot, psi, psi_dot
INPUTS: delta
'''


[xrefs, yrefs, psirefs] = getPath("waypoints.json")
xrefs = np.array(xrefs*0.05)
yrefs = np.array(yrefs*0.05)
psirefs = np.array(psirefs)
print('''
STARTING POSITIONS:
{0:.3f}  {1:.3f}
'''.format(xrefs[0],yrefs[0],psirefs[0]))
print(len(xrefs),len(yrefs))

Tf = 1  # prediction horizon
N = 20  # number of discretization steps
T = 15.00  # maximum simulation time[s]

# load model
acados_solver = AcadosOcpSolver(None,generate=False,build=True,json_file="acados_ocp.json")

# Dimensions
nx = 7 # model.x.size()[0]
nu = 1 # model.u.size()[0]
# print("size nx, nu: {} | {}".format(nx,nu))
ny = nx + nu
Nsim = int(T * N / Tf)

# initialize data structs
simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))
tcomp_sum = 0
tcomp_max = 0
x0 = np.array([xrefs[0],yrefs[0],0,0,0,psirefs[0],0])
acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)
# simulate
normArray= []
for i in range(Nsim):
    # update reference
    normArray = (xrefs - x0[0])**2 + (yrefs - x0[1])**2
    start_ref = np.argmin(normArray)
    for j in range(N):
        yref = np.array([xrefs[start_ref+j], yrefs[start_ref+j], 0, 0, 0, psirefs[start_ref+j], 0, 0])
        acados_solver.set(j, "yref", yref)
    yref_N = np.array([xrefs[start_ref+N], yrefs[start_ref+N], 0, 0, 0, psirefs[start_ref+N], 0])
    acados_solver.set(N, "yref", yref_N)

    # solve ocp
    t = time.time()

    status = acados_solver.solve()
    # if status != 0:
    #     print("acados returned status {} in closed loop iteration {}.".format(status, i))

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
    normArray = []  #Reset Norm Array

    # if x0[0] == xrefs[-1]:
    #     N0 = np.where(np.diff(np.sign(simX[:, 0])))[0][0]
    #     Nsim = i - N0  # correct to final number of simulation steps for plotting
    #     simX = simX[N0:i, :]
    #     simU = simU[N0:i, :]
    #     break

# Stats Printout
print("Average computation time: {}".format(tcomp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))
print("Lap time: {}s".format(Tf * Nsim / N))

#Graphing
plt.figure()
plt.plot(xrefs,yrefs,'--',color='k')
psiref_x = np.cos(simX[:,5])
psiref_y = np.sin(simX[:,5])
plt.plot(simX[:,0],simX[:,1])
plt.quiver(simX[:,0],simX[:,1],psiref_x,psiref_y)
plt.show()
print(simX)