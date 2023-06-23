"""Python implementation of Stanley lateral controller as a Class. Intended to be used
by ROS2 Node Controller in controller.py."""

from ros2_car_control.closestPoint import get_closest_waypoint, get_lateral_errors

from typing import Dict, Tuple
import numpy as np


class StanleyController:
    """Stanley controller is a nonlinear controller developed at Stanford
    University for the DARPA Grand Challenge. It makes use of a Kinematic
    bicycle car model, and is proven stable for a given nonlinear
    trajectory. See Autonomous Automobile Trajectory Tracking for Off-Road
    Driving: Controller Design, Experimental Validation and Racing by
    Hoffman, Tomlin, Montemerlo, and Thrun.
    https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf

    There are two tuning parameters:
        control_gain:                (float) time constant [1/s]
        softening_gain:              (float) softening gain [m/s]
    but the softening gain is typically held constant at 1.

    There is one model parameter:
        wheelbase:                   (float) vehicle wheelbase [m]
    """

    def __init__(self, waypoints: np.ndarray, ctrl_params: Dict[str, float]):
        self.k = ctrl_params.get("k")  # control_gain
        self.k_soft = ctrl_params.get("k_soft")  # softening_gain
        self.L = ctrl_params.get("L")  # wheelbase
        self.velocity_setpoint = ctrl_params.get("speed_setpoint")

        self.prev_steering_angle = 0.0
        self.debug_bool = False

        self.waypoints = waypoints

    def get_commands(
        self, x: float, y: float, yaw: float, v: float
    ) -> Tuple[float, float, float, float]:
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
        closest_waypoint, closest_i = get_closest_waypoint(front_axle, waypoints)

        crosstrack_error, yaw_term = get_lateral_errors(front_axle, closest_waypoint)

        if self.debug_bool:
            print(f"Pose: {front_axle},\tNearest Point: {closest_waypoint}")
            print(f"Crosstrack Error:{crosstrack_error}")

        # Stanley Control law.
        tangent_term = np.arctan2((self.k * crosstrack_error), (v + self.k_soft))
        steering_angle = yaw_term + tangent_term

        self.prev_steering_angle = steering_angle

        speed_cmd = self.velocity_setpoint

        if self.debug_bool:
            print("Psi: " + str(yaw_term) + "\tTangent Term: " + str(tangent_term))
            print("Stanley Output: " + str(steering_angle))

        return (
            steering_angle,
            speed_cmd,
            closest_waypoint[0, 0],
            closest_waypoint[0, 1],
        )
