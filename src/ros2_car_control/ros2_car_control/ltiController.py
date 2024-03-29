"""Python implementation of a Linear Time Invariant lateral controller as a Class. 
Intended to be used by ROS2 Node Controller in controller.py."""

from ros2_car_control.utilities import (
    get_closest_waypoint,
    get_lateral_errors,
    get_accel_dist,
    get_speed_cmd,
)

import numpy as np
from typing import Dict, Tuple


class LTIController:
    """A frequency-based lateral controller designed using an error state space model.

    The model is as follows:
        - dx = A*x + B*u
        - y = C*x + D*u
    where:
        - x is the state vector: [crosstrack error (m), d(x1)/dt,
            yaw error (rad),d(x3)/dt]
        - u is the input vector: [steering angle (rad)]
        The A,B,C,D matrices are derived in Rajamani's Vehicle Dynamics and
        Control book in Chapters 2 and 3.

    The transfer function of the state space model is created by setting the
    output state matrix, C, to [1, 0, lookahead, 0]. The controller is assumed
    to be designed in the continuous-time domain and converted to a discrete
    state space equation.
    """

    def __init__(self, waypoints: np.ndarray, ctrl_params: Dict[str, float]):
        n_GcX = ctrl_params.get("n_GcX")  # Number of controller states
        self.waypoints = waypoints
        self.lookahead = ctrl_params.get("lookahead")
        self.velocity_setpoint = ctrl_params.get("speed_setpoint")
        self.max_steer = ctrl_params.get("max_steer")
        self.max_accel = ctrl_params.get("max_accel")
        self.d_accel = get_accel_dist(self.velocity_setpoint, self.max_accel)
        self.d_decel = self.waypoints.d[-1] - self.d_accel

        self.Gc_states = np.zeros((n_GcX, 1))
        self.GcA = np.array(ctrl_params.get("GcA")).reshape(n_GcX, n_GcX)
        self.GcB = np.array(ctrl_params.get("GcB")).reshape(n_GcX, 1)
        self.GcC = np.array(ctrl_params.get("GcC")).reshape(1, n_GcX)
        self.GcD = np.array(ctrl_params.get("GcD")).reshape(1, 1)

    def get_commands(
        self, x: float, y: float, yaw: float, v: float
    ) -> Tuple[float, float, float, float, float]:
        lookahead_point = np.array(
            [[x + self.lookahead * np.cos(yaw), y + self.lookahead * np.sin(yaw), yaw]]
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
        new_Gc_states = self.GcA @ self.Gc_states + self.GcB @ lateral_error
        steering_angle = float(self.GcC @ self.Gc_states + self.GcD @ lateral_error)

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
