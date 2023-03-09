'''
Model References:
Kinematic and dynamic vehicle models for autonomous driving control design
https://www.researchgate.net/publication/308851864_Kinematic_and_dynamic_vehicle_models_for_autonomous_driving_control_design

Predictive Active Steering Control for Autonomous Vehicle Systems
https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4162483
'''

from casadi import *

def bicycle_model():
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Dynamic_Bicycle"
    ## Race car parameters
    m = 5.568 # mass
    Cf = 3.675 # Cornering Stiffness of Front Tires
    Cr = 3.661 # Cornering Stiffness of Rear Tires
    lf = 0.205 # Distance from CG to Front Axle
    lr = 0.199 # Distance from CG to Rear Axle
    Iz = 0.167 # Rotational Inertia
    vx_setpoint = 1.0 # Tuned Controller Speed

    ## CasADi Model
    # set up states & controls
    X = MX.sym("X")
    Y = MX.sym("Y")
    x = MX.sym("x")
    y = MX.sym("y")
    psi = MX.sym("psi")

    X_dot = MX.sym("X_dot")
    Y_dot = MX.sym("Y_dot")
    x_dot = MX.sym("x_dot")
    y_dot = MX.sym("y_dot")
    psi_dot = MX.sym("psi_dot")

    # x_d_dot = MX.sym("x_d_dot")       # Zero, assumed constant forward velocity
    y_d_dot = MX.sym("y_d_dot")
    psi_d_dot = MX.sym("psi_d_dot")

    # states = vertcat(X, Y, x, y, x_dot, y_dot, psi, psi_dot)      # Full Dynamics
    states = vertcat(X, Y, x, y, y_dot, psi, psi_dot)        # Reduced with Constant Velocity

    # controls
    delta = MX.sym("delta")
    u = vertcat(delta)

    # xdot
    # states_dot = vertcat(X_dot, Y_dot, x_dot, y_dot, x_d_dot, y_d_dot, psi_dot, psi_d_dot)
    states_dot = vertcat(X_dot, Y_dot, x_dot, y_dot, y_d_dot, psi_dot, psi_d_dot)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    # REF: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4162483
    # F_xf = 0                                        # Assume Const Vel
    # F_xr = 0                                        # Assume Const Vel
    # F_yf = - Cf * (delta - arctan(y_dot/x_dot))     # Only Term which is non zero force to tire
    # F_yr = 0                                        # Assume no rear steering

    # x_dot * cos(psi) - y_dot * sin(psi),            # X_dot
    # x_dot * sin(psi) + y_dot * cos(psi),            # Y_dot
    # 1.0,                                            # x_dot
    # y_dot,                                          # y_dot
    # -x_dot * psi_dot + 2 * F_yf/m + 2 * F_yr/m,     # y_d_dot
    # psi_dot,                                        # psi_dot
    # 2 * lf * F_yf / Iz - 2 * lr * F_yr / Iz,        # psi_d_dot

    F_yf = Cf * (delta - arctan(y_dot/vx_setpoint))
    # Explicit Dynamics [x_dot = f(x,u)]
    f_expl = vertcat(
        vx_setpoint * cos(psi) - y_dot * sin(psi),      # X_dot
        vx_setpoint * sin(psi) + y_dot * cos(psi),      # Y_dot
        vx_setpoint,                                    # x_dot
        y_dot,                                          # y_dot
        -vx_setpoint * psi_dot + 2 * F_yf/m,            # y_d_dot
        psi_dot,                                        # psi_dot
        2 * lf * F_yf / Iz,                             # psi_d_dot
    )

    # state bounds
    model.delta_min = -0.65  # minimum steering angle [rad]
    model.delta_max = 0.65  # maximum steering angle [rad]

    # Define initial conditions
    model.x0 = np.array([0, 0, 0, 0, 0, 0, 0])

    # define constraints struct
    constraint.expr = vertcat(delta)

    # Define model struct
    params = types.SimpleNamespace()
    params.m  = m
    params.Cf = Cf
    params.Cr = Cr
    params.lr = lr
    params.lf = lf
    params.Iz = Iz
    params.x_dot = x_dot
    model.f_impl_expr = states_dot - f_expl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = states_dot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params
    return model, constraint



if __name__ == "__main__":
    bicycle_model()