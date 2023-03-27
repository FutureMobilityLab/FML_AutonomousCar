import time, os
import numpy as np
from DBM_generator import *
from getPath import getPath
import matplotlib.pyplot as plt
'''
STATES: X, Y, x, y, y_dot, psi, psi_dot
INPUTS: delta
'''

Tf = 1  # prediction horizon
N = 20  # number of discretization steps
v = 1.0 # Velocity Setpoint

waypointsdir = os.path.join(os.path.dirname( __file__ ), '..', 'config','waypoints.json')
[xrefs, yrefs, psirefs] = getPath(waypointsdir)
xrefs = np.array(xrefs*0.05)
yrefs = np.array(yrefs*0.05)
psirefs = np.array(psirefs)

# Get Total Path Distances
sumDist = 0
srefs = np.array([])
for i in range(len(xrefs)):
    srefs = np.append(srefs,sumDist)
    try:
        sumDist = sumDist + np.sqrt((xrefs[i+1] - xrefs[i])**2 + (yrefs[i+1] - yrefs[i])**2)
    except IndexError:
        pass

print('''
STARTING POSITIONS:
{0:.3f}  {1:.3f}
'''.format(xrefs[0],yrefs[0],psirefs[0]))

# Load Model
acados_solver = AcadosOcpSolver(None,generate=False,build=False,json_file="acados_ocp.json")

# Dimensions
nx = 7 # model.x.size()[0]
nu = 1 # model.u.size()[0]
# print("size nx, nu: {} | {}".format(nx,nu))
ny = nx + nu

Nsim= int(srefs[-1] * N / (v * Tf))
ds = v*Tf/N

# initialize data structs
simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))
tcomp_sum = 0
tcomp_max = 0
x0 = np.array([xrefs[0]-0.5,yrefs[0],0,0,0,psirefs[0]+0.2,0])
acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)
# simulate
for i in range(Nsim):
    # update reference
    normArray = np.sqrt((xrefs - x0[0])**2 + (yrefs - x0[1])**2)
    start_ref = np.argmin(normArray)
    current_sref = srefs[start_ref]
    for j in range(N):
        try:
            relative_s_dist = current_sref - srefs 
            refs0_idx = len(relative_s_dist[relative_s_dist > 0])
            sratio = ((current_sref - srefs[refs0_idx])/(srefs[refs0_idx+1]-srefs[refs0_idx]))
            xrefs_interpolated = sratio*(xrefs[refs0_idx+1]-xrefs[refs0_idx])+xrefs[refs0_idx]
            yrefs_interpolated = sratio*(yrefs[refs0_idx+1]-yrefs[refs0_idx])+yrefs[refs0_idx]
            psirefs_interpolated = psirefs[refs0_idx]
            yref = np.array([xrefs_interpolated, yrefs_interpolated, 0, 0, 0, psirefs_interpolated, 0, 0])
        except IndexError:
            yref = np.array([xrefs[-1], yrefs[-1], 0, 0, 0, psirefs[-1], 0, 0])

        acados_solver.set(j, "yref", yref)
        current_sref += ds
    try:        
        yref_N = yref[:-1]
    except IndexError:
        yref_N = np.array([xrefs[-1], yrefs[-1], 0, 0, 0, psirefs[-1], 0])
    acados_solver.set(N, "yref", yref_N)
    # solve ocp
    t = time.time()

    # status = acados_solver.solve()
    acados_solver.solve()
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
    print(float(u0))
    for j in range(nx):
        simX[i, j] = x0[j]
    for j in range(nu):
        simU[i, j] = u0[j]

    # update initial condition
    x0 = acados_solver.get(1, "x")
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)
    normArray = []  #Reset Norm Array

    if x0[0] == xrefs[-1]:
        N0 = np.where(np.diff(np.sign(simX[:, 0])))[0][0]
        Nsim = i - N0  # correct to final number of simulation steps for plotting
        simX = simX[N0:i, :]
        simU = simU[N0:i, :]
        break

# Stats Printout
print("Average computation time: {}".format(tcomp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))
print("Lap time: {}s".format(Tf * Nsim / N))

#Graphing
plt.figure(1)
plt.plot(xrefs,yrefs,'--',color='k')
psiref_x = np.cos(simX[:,5])
psiref_y = np.sin(simX[:,5])
plt.plot(simX[:,0],simX[:,1])
plt.quiver(simX[:,0],simX[:,1],psiref_x,psiref_y)


plt.figure(2)
plt.plot(np.linspace(0,Tf*Nsim/N,Nsim),simU[:,0])
plt.show()