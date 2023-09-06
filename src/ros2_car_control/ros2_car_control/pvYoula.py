"""Python implementation of parameter-varying Youla controller as a Class intended to be
used by ROS2 Node Controller in controller.py."""

from ros2_car_control.utilities import (
    get_closest_waypoint,
    get_lateral_errors,
    get_accel_dist,
    get_speed_cmd,
)

import numpy as np
from typing import Dict, Tuple


class PVYoulaController:
    """A frequency-based parameter-varying controller designed using an error state
    space model.

    The model is as follows:
        - dx = A*x + B*u
        - y = C*x + D*u
    where:
        - x is the state vector: [crosstrack error (m), d(x1)/dt,
            yaw error (rad),d(x3)/dt]
        - u is the input vector: [steering angle (rad)]
        The A,B,C,D matrices are derived in Rajamani's Vehicle Dynamics and
        Control book in Chapters 2 and 3.

    The transfer function generated for this controller is implemented using a state
    space realization. The state space matrices are therefore parameter-varying.
    """

    def __init__(self, waypoints: np.ndarray, ctrl_params: Dict[str, float]):
        n_GcX = int(ctrl_params.get("n_GcX"))  # Number of controller states
        self.waypoints = waypoints
        self.velocity_setpoint = ctrl_params.get("speed_setpoint")
        self.max_steer = ctrl_params.get("max_steer")
        self.max_accel = ctrl_params.get("max_accel")
        self.d_accel = get_accel_dist(self.velocity_setpoint, self.max_accel)
        self.d_decel = self.waypoints.d[-1] - self.d_accel
        self.Gc_states = np.zeros((n_GcX, 1))
        self.ctrl_sample_time = ctrl_params["ctrl_sample_time_s"]
        self.lookahead_gain = ctrl_params["lookahead_gain"]

        self.vehicle_params = {
            "mass_kg": 5.568,
            "front_to_cg_m": 0.205,
            "wheelbase_m": 0.404,
            "yaw_inertia_kg/m^2": 0.133,
            "front_corner_stiff_N/rad": 95.5942,
            "rear_corner_stiff_N/rad": 99.112,
            "omega_n1": 1.4,
            "zeta_1": 0.99,
            "omega_n2": 19,
            "zeta_2": 0.5,
            "tau_3": 1 / 0.87,
        }

    def compute_A_matrix(self, long_vel: float, lookahead_dist: float) -> np.ndarray:
        """Compute the state space A matrix from the given parameters."""
        m = self.vehicle_params["mass_kg"]
        C_alpha_f = self.vehicle_params["front_corner_stiff_N/rad"]
        C_alpha_r = self.vehicle_params["rear_corner_stiff_N/rad"]
        l_f = self.vehicle_params["front_to_cg_m"]
        l_r = self.vehicle_params["wheelbase_m"] - l_f
        omega_n1 = self.vehicle_params["omega_n1"]
        zeta_1 = self.vehicle_params["zeta_1"]
        omega_n2 = self.vehicle_params["omega_n2"]
        zeta_2 = self.vehicle_params["zeta_2"]
        tau_3 = self.vehicle_params["tau_3"]
        I_z = self.vehicle_params["yaw_inertia_kg/m^2"]
        d_s = lookahead_dist
        V_x = long_vel
        return np.array(
            [
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    (
                        C_alpha_r
                        * (l_f + l_r)
                        * (
                            omega_n1**2
                            + omega_n2**2
                            + omega_n1 * omega_n2 * zeta_1 * zeta_2 * 4.0
                            + omega_n1 * omega_n2**2 * tau_3 * zeta_1 * 2.0
                            + omega_n1**2 * omega_n2 * tau_3 * zeta_2 * 2.0
                        )
                        * -2.0
                    )
                    / (tau_3 * (I_z + d_s * l_f * m)),
                ],
                [
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    (
                        1.0
                        / V_x**2
                        * (
                            C_alpha_f
                            * C_alpha_r
                            * V_x**2
                            * (l_f + l_r)
                            * (
                                omega_n1 * zeta_1 * 2.0
                                + omega_n2 * zeta_2 * 2.0
                                + omega_n1**2 * tau_3
                                + omega_n2**2 * tau_3
                                + omega_n1 * omega_n2 * tau_3 * zeta_1 * zeta_2 * 4.0
                            )
                            * 1.3152e4
                            + C_alpha_f
                            * C_alpha_r
                            * V_x
                            * (d_s + l_r)
                            * (l_f + l_r)
                            * (
                                omega_n1**2
                                + omega_n2**2
                                + omega_n1 * omega_n2 * zeta_1 * zeta_2 * 4.0
                                + omega_n1 * omega_n2**2 * tau_3 * zeta_1 * 2.0
                                + omega_n1**2 * omega_n2 * tau_3 * zeta_2 * 2.0
                            )
                            * 1.3152e4
                        )
                        * (-1.520681265206813e-4)
                    )
                    / (C_alpha_f * tau_3 * (I_z + d_s * l_f * m)),
                ],
                [
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    (
                        1.0
                        / V_x**2
                        * (
                            C_alpha_f
                            * V_x**2
                            * (I_z + d_s * l_f * m)
                            * (
                                omega_n1**2
                                + omega_n2**2
                                + omega_n1 * omega_n2 * zeta_1 * zeta_2 * 4.0
                                + omega_n1 * omega_n2**2 * tau_3 * zeta_1 * 2.0
                                + omega_n1**2 * omega_n2 * tau_3 * zeta_2 * 2.0
                            )
                            * 6.576e3
                            + C_alpha_f
                            * C_alpha_r
                            * V_x**2
                            * (l_f + l_r)
                            * (
                                omega_n1 * tau_3 * zeta_1 * 2.0
                                + omega_n2 * tau_3 * zeta_2 * 2.0
                                + 1.0
                            )
                            * 1.3152e4
                            + C_alpha_f
                            * C_alpha_r
                            * V_x
                            * (d_s + l_r)
                            * (l_f + l_r)
                            * (
                                omega_n1 * zeta_1 * 2.0
                                + omega_n2 * zeta_2 * 2.0
                                + omega_n1**2 * tau_3
                                + omega_n2**2 * tau_3
                                + omega_n1 * omega_n2 * tau_3 * zeta_1 * zeta_2 * 4.0
                            )
                            * 1.3152e4
                        )
                        * (-1.520681265206813e-4)
                    )
                    / (C_alpha_f * tau_3 * (I_z + d_s * l_f * m)),
                ],
                [
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    (
                        1.0
                        / V_x**2
                        * (
                            C_alpha_f
                            * V_x**2
                            * (I_z + d_s * l_f * m)
                            * (
                                omega_n1 * zeta_1 * 2.0
                                + omega_n2 * zeta_2 * 2.0
                                + omega_n1**2 * tau_3
                                + omega_n2**2 * tau_3
                                + omega_n1 * omega_n2 * tau_3 * zeta_1 * zeta_2 * 4.0
                            )
                            * 6.576e3
                            + C_alpha_f
                            * C_alpha_r
                            * V_x**2
                            * tau_3
                            * (l_f + l_r)
                            * 1.3152e4
                            + C_alpha_f
                            * C_alpha_r
                            * V_x
                            * (d_s + l_r)
                            * (l_f + l_r)
                            * (
                                omega_n1 * tau_3 * zeta_1 * 2.0
                                + omega_n2 * tau_3 * zeta_2 * 2.0
                                + 1.0
                            )
                            * 1.3152e4
                        )
                        * (-1.520681265206813e-4)
                    )
                    / (C_alpha_f * tau_3 * (I_z + d_s * l_f * m)),
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    (
                        (
                            tau_3
                            * (
                                C_alpha_f * C_alpha_r * V_x * l_r**2 * 2.0
                                + C_alpha_f * C_alpha_r * V_x * d_s * l_f * 2.0
                                + C_alpha_f * C_alpha_r * V_x * d_s * l_r * 2.0
                                + C_alpha_f * C_alpha_r * V_x * l_f * l_r * 2.0
                            )
                            * 6.576e3
                            + (
                                C_alpha_f * I_z * V_x**2
                                + C_alpha_f * V_x**2 * d_s * l_f * m
                            )
                            * (
                                omega_n1 * tau_3 * zeta_1 * 2.0
                                + omega_n2 * tau_3 * zeta_2 * 2.0
                                + 1.0
                            )
                            * 6.576e3
                        )
                        * (-1.520681265206813e-4)
                    )
                    / (
                        tau_3
                        * (
                            C_alpha_f * I_z * V_x**2
                            + C_alpha_f * V_x**2 * d_s * l_f * m
                        )
                    ),
                ],
            ]
        )

    def compute_C_matrix(self, long_vel: float, lookahead_dist: float) -> np.ndarray:
        m = self.vehicle_params["mass_kg"]
        C_alpha_f = self.vehicle_params["front_corner_stiff_N/rad"]
        C_alpha_r = self.vehicle_params["rear_corner_stiff_N/rad"]
        l_f = self.vehicle_params["front_to_cg_m"]
        l_r = self.vehicle_params["wheelbase_m"] - l_f
        omega_n1 = self.vehicle_params["omega_n1"]
        zeta_1 = self.vehicle_params["zeta_1"]
        omega_n2 = self.vehicle_params["omega_n2"]
        zeta_2 = self.vehicle_params["zeta_2"]
        tau_3 = self.vehicle_params["tau_3"]
        I_z = self.vehicle_params["yaw_inertia_kg/m^2"]
        d_s = lookahead_dist
        V_x = long_vel
        return np.array(
            [
                [
                    (
                        1.0
                        / V_x**2
                        * omega_n1**2
                        * omega_n2**2
                        * (
                            C_alpha_f * C_alpha_r * l_f**2 * 4.0
                            + C_alpha_f * C_alpha_r * l_r**2 * 4.0
                            - C_alpha_f * V_x**2 * l_f * m * 2.0
                            + C_alpha_r * V_x**2 * l_r * m * 2.0
                            + C_alpha_f * C_alpha_r * l_f * l_r * 8.0
                        )
                        * (6.25e2 / 8.22e2)
                    )
                    / (C_alpha_f * tau_3 * (I_z + d_s * l_f * m)),
                    (
                        1.0
                        / V_x**2
                        * (
                            omega_n1
                            * omega_n2
                            * (
                                omega_n1 * omega_n2 * 7.3e1
                                + omega_n1 * zeta_2 * 1.0e4
                                + omega_n2 * zeta_1 * 1.0e4
                                + omega_n1 * omega_n2 * tau_3 * 5.0e3
                            )
                            * (
                                C_alpha_f * C_alpha_r * l_f**2 * 4.0
                                + C_alpha_f * C_alpha_r * l_r**2 * 4.0
                                - C_alpha_f * V_x**2 * l_f * m * 2.0
                                + C_alpha_r * V_x**2 * l_r * m * 2.0
                                + C_alpha_f * C_alpha_r * l_f * l_r * 8.0
                            )
                            + V_x
                            * omega_n1**2
                            * omega_n2**2
                            * (
                                C_alpha_f * I_z
                                + C_alpha_r * I_z
                                + C_alpha_f * l_f**2 * m
                                + C_alpha_r * l_r**2 * m
                            )
                            * 1.0e4
                        )
                    )
                    / (C_alpha_f * tau_3 * (I_z + d_s * l_f * m) * 6.576e3),
                    (
                        1.0
                        / V_x**2
                        * (
                            omega_n1
                            * omega_n2
                            * (
                                omega_n1 * zeta_2 * 2.0
                                + omega_n2 * zeta_1 * 2.0
                                + omega_n1 * omega_n2 * tau_3
                            )
                            * (
                                C_alpha_f * C_alpha_r * l_f**2 * 4.0
                                + C_alpha_f * C_alpha_r * l_r**2 * 4.0
                                - C_alpha_f * V_x**2 * l_f * m * 2.0
                                + C_alpha_r * V_x**2 * l_r * m * 2.0
                                + C_alpha_f * C_alpha_r * l_f * l_r * 8.0
                            )
                            * 7.3e1
                            + V_x
                            * omega_n1
                            * omega_n2
                            * (
                                omega_n1 * omega_n2 * 7.3e1
                                + omega_n1 * zeta_2 * 1.0e4
                                + omega_n2 * zeta_1 * 1.0e4
                                + omega_n1 * omega_n2 * tau_3 * 5.0e3
                            )
                            * (
                                C_alpha_f * I_z
                                + C_alpha_r * I_z
                                + C_alpha_f * l_f**2 * m
                                + C_alpha_r * l_r**2 * m
                            )
                            * 2.0
                            + I_z * V_x**2 * m * omega_n1**2 * omega_n2**2 * 5.0e3
                        )
                    )
                    / (C_alpha_f * tau_3 * (I_z + d_s * l_f * m) * 6.576e3),
                    (
                        1.0
                        / V_x**2
                        * (
                            V_x
                            * omega_n1
                            * omega_n2
                            * (
                                omega_n1 * zeta_2 * 2.0
                                + omega_n2 * zeta_1 * 2.0
                                + omega_n1 * omega_n2 * tau_3
                            )
                            * (
                                C_alpha_f * I_z
                                + C_alpha_r * I_z
                                + C_alpha_f * l_f**2 * m
                                + C_alpha_r * l_r**2 * m
                            )
                            * 1.46e2
                            + I_z
                            * V_x**2
                            * m
                            * omega_n1
                            * omega_n2
                            * (
                                omega_n1 * omega_n2 * 7.3e1
                                + omega_n1 * zeta_2 * 1.0e4
                                + omega_n2 * zeta_1 * 1.0e4
                                + omega_n1 * omega_n2 * tau_3 * 5.0e3
                            )
                        )
                    )
                    / (C_alpha_f * tau_3 * (I_z + d_s * l_f * m) * 6.576e3),
                    (
                        I_z
                        * m
                        * omega_n1
                        * omega_n2
                        * (
                            omega_n1 * zeta_2 * 2.0
                            + omega_n2 * zeta_1 * 2.0
                            + omega_n1 * omega_n2 * tau_3
                        )
                        * 1.110097323600973e-2
                    )
                    / (C_alpha_f * tau_3 * (I_z + d_s * l_f * m)),
                ]
            ]
        )

    def get_commands(
        self, x: float, y: float, yaw: float, v: float
    ) -> Tuple[float, float, float, float, float]:
        v = np.clip(v, 0.1, 5)
        lookahead_dist = self.lookahead_gain * v
        lookahead_point = np.array(
            [[x + lookahead_dist * np.cos(yaw), y + lookahead_dist * np.sin(yaw), yaw]]
        )
        waypoints = np.hstack(
            (
                self.waypoints.x[np.newaxis].T,
                self.waypoints.y[np.newaxis].T,
                self.waypoints.psi[np.newaxis].T,
            )
        )
        closest_waypoint, closest_i = get_closest_waypoint(lookahead_point, waypoints)

        lateral_error, _ = get_lateral_errors(lookahead_point, closest_waypoint)
        lateral_error = np.array([[lateral_error]])

        A = self.compute_A_matrix(v, lookahead_dist)
        B = np.array([[0], [0], [0], [0], [1]])
        C = self.compute_C_matrix(v, lookahead_dist)
        D = np.array([[0]])

        # Solve ODE using Heun's Method (explicit Trapezoidal rule).
        dx = A @ self.Gc_states + B @ lateral_error
        next_state_est = self.Gc_states + self.ctrl_sample_time * dx
        next_dx_est = A @ next_state_est + B @ lateral_error
        new_Gc_states = self.Gc_states + self.ctrl_sample_time / 2 * (dx + next_dx_est)
        steering_angle = float(C @ new_Gc_states + D @ lateral_error)

        if (
            abs(steering_angle) < self.max_steer
        ):  # Attempting to prevent wind-up which occurs immediately with controller
            self.Gc_states = new_Gc_states

        # Ramp up or down velocity based on distance traveled.
        d = self.waypoints.d[closest_i]
        speed_cmd = get_speed_cmd(
            d, self.d_accel, self.d_decel, self.max_accel, self.velocity_setpoint
        )

        return (
            steering_angle,
            speed_cmd,
            closest_waypoint[0, 0],
            closest_waypoint[0, 1],
            closest_waypoint[0, 2],
        )
