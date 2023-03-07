'''
Model Reference:
Kinematic and dynamic vehicle models for autonomous driving control design
https://www.researchgate.net/publication/308851864_Kinematic_and_dynamic_vehicle_models_for_autonomous_driving_control_design
'''

from casadi import *

def bicycle_model():
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Dynamic_Bicycle"
    ## Race car parameters
    m = 5.568 # mass
    Cf = 7.35 # Cornering Stiffness of Front Tires
    Cr = 7.35 # Cornering Stiffness of Rear Tires
    lf = .18 # Distance from CG to Front Axle
    lr = .2 # Distance from CG to Rear Axle
    Iz = .2444 # Rotational Inertia

    ## CasADi Model
    # set up states & controls
    x = MX.sym("x_dot")
    a_x = MX.sym("a_x")
    y = MX.sym("y_dot")
    psi = MX.sym("psi_d_dot")
    X = MX.sym("X_dot")
    Y = MX.sym("Y_dot")
    states = vertcat(x, y, psi_dot, X, Y)

    # controls
    delta = MX.sym("delta")
    u = vertcat(delta)

    # xdot
    x_dot = MX.sym("x_dot")
    y_dot = MX.sym("y_dot")
    psi_dot = MX.sym("psi_d_dot")
    X_dot = MX.sym("X_dot")
    Y_dot = MX.sym("Y_dot")
    states_dot = vertcat(x_dot, y_dot, psi_d_dot, X_dot, Y_dot)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    alpha_f = delta - arctan(y_dot/abs(x_dot))
    alpha_r = -arctan(y_dot/abs(x_dot))
    F_cf = -Cf*alpha_f
    F_cr = -Cr*alpha_r
    x_d_dot = psi_dot * y_dot + a_x
    y_d_dot = -psi_dot * x_dot + 2/m * (F_cf*cos(delta)+F_cr)
    psi_d_dot = 2/Iz * (lf*F_cf - lr*F_cr)
    X_dot = x_dot * cos(psi) - y_dot * sin(psi)
    Y_dot = x_dot * sin(psi) + y_dot * cos(psi)


    f_expl = vertcat(
        x_d_dot,
        y_d_dot,
        psi_d_dot,
        X_dot,
        Y_dot,
    )

    # state bounds
    model.delta_min = -0.40  # minimum steering angle [rad]
    model.delta_max = 0.40  # maximum steering angle [rad]

    # input bounds
    model.ddelta_min = -2.0  # minimum change rate of stering angle [rad/s]
    model.ddelta_max = 2.0  # maximum change rate of steering angle [rad/s]

    # nonlinear constraint
    constraint.alat_min = -2  # maximum lateral force [m/s^2]
    constraint.alat_max = 2  # maximum lateral force [m/s^1]

    # Define initial conditions
    model.x0 = np.array([0, 0, 0, 0, 0])

    # define constraints struct
    constraint.expr = delta

    # Define model struct
    params = types.SimpleNamespace()
    params.Cf = Cf
    params.Cr = Cr
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
