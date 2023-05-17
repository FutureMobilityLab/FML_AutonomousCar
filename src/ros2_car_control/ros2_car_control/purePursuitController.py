"""Python implementation of Pure Pursuit lateral controller as a Class. Intended to be
used by ROS2 Node Controller in controller.py."""

from ros2_car_control.closestPoint import get_closest_waypoint, get_lateral_errors
import numpy as np
from typing import Dict, Tuple


class PurePursuitController:
    """Pure Pursuit is a geometrical controller that is tasked with finding the
    curvature of a path required to bring a robot off of a trajectory back onto the
    trajectory. It is a nonlinear controller that is surprisingly robust against model
    uncertainty and even implementation errors. For more information refer to Carnegie
    Mellon University's Technical Report 92-01: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf.

    The controller has one tuning parameter: lookahead distance [m]

    The implementation requires one model parameter: wheel base [m]
    """

    def __init__(self, waypoints: np.ndarray, ctrl_params: Dict[str, float]):
        self.L = ctrl_params.get("L")  # wheelbase
        self.max_steer = ctrl_params.get("max_steer")  # max_steer
        self.velocity_setpoint = ctrl_params.get("speed_setpoint")
        self.lookahead_dist = ctrl_params.get("lookahead")

        self.prev_steering_angle = 0.0
        self.debug_bool = False

        self.waypoints = waypoints

    def get_commands(
        self, x: float, y: float, yaw: float, v: float
    ) -> Tuple[float, float, float, float]:
        # Get path point closest to the vehicle.
        front_axle = np.array(
            [[x + self.L / 2.0 * np.cos(yaw), y + self.L / 2.0 * np.sin(yaw), yaw]]
        )
        waypoints = np.hstack(
            (
                self.waypoints.x[np.newaxis].T,
                self.waypoints.y[np.newaxis].T,
                self.waypoints.psi[np.newaxis].T,
            )
        )
        dist = np.linalg.norm(front_axle[0, 0:2] - waypoints[:, 0:2], axis=0)
        closest_i = np.argmin(dist)

        # Get goal point, the closest point a lookahead distance after
        for i in range(closest_i, len(self.waypoints.x)):
            if dist[i] >= self.lookahead_dist:
                lookahead_vec_y = self.waypoints.y[i] - front_axle[0, 1]
                lookahead_vec_x = self.waypoints.x[i] - front_axle[0, 0]
                # TODO: Find out where this expression comes from. The CMU algorithm
                # calls for a transformation to local coordinates, computation of
                # curvature, and then conversion from curvature to steer angle.
                steering_angle = np.arctan2(lookahead_vec_y, lookahead_vec_x) - yaw
                point_ref_index = i
                break

        speed_cmd = self.velocity_setpoint

        return (
            steering_angle,
            speed_cmd,
            self.waypoints.x[point_ref_index],
            self.waypoints.y[point_ref_index],
        )
