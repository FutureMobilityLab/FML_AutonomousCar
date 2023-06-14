import numpy as np
from typing import Tuple


def get_closest_waypoint(
    pose: np.ndarray, waypoints: np.ndarray
) -> Tuple[np.ndarray, int]:
    """Returns the waypoint that is closest to the position.

    Args:
        pose: global X, Y, position [m] and yaw [rad] of the vehicle.
        waypoints: global X, Y positions [m], and yaw angles [rad].

    Returns:
        The waypoint (X,Y,yaw) that is closest the position and its index in waypoints.
    """

    dist = np.linalg.norm(pose[0, 0:2] - waypoints[:, 0:2], axis=1)
    closest_i = np.argmin(dist)
    return waypoints[closest_i][np.newaxis], closest_i


def get_lateral_errors(
    pose: np.ndarray, reference_pose: np.ndarray
) -> Tuple[float, float]:
    """Compute the crosstrack error and the yaw error.

    Args:
        pose: global X, Y, position [m] and yaw [rad] of the vehicle.
        reference_pose: desired global X, Y, position [m] and yaw [rad].

    Returns:
        The component of the distance between positions that is orthogonal to
        the reference yaw angle and the difference between the yaw and
        reference yaw angle.
    """

    ref_to_axle = pose[0, 0:2] - reference_pose[0, 0:2]
    crosstrack_vector = np.array(
        [np.sin(reference_pose[0, 2]), -np.cos(reference_pose[0, 2])]
    )
    crosstrack_err = ref_to_axle.dot(crosstrack_vector)
    yaw_err = pose[0, 2] - reference_pose[0, 2]
    return crosstrack_err, -yaw_err
