import json
import numpy as np
import yaml
from PIL import Image
import os


class waypoints:
    def __init__(self):
        username = os.getlogin()
        waypointsdir = str(
            "/home/"
            + username
            + "/FML_AutonomousCar/src/ros2_car_control/config/waypoints.json"
        )
        mapdir = str(
            "/home/" + username + "/FML_AutonomousCar/src/car_slam/config/lab_map.pgm"
        )
        mapcfgdir = str(
            "/home/" + username + "/FML_AutonomousCar/src/car_slam/config/lab_map.yaml"
        )
        # Get Image Properties
        graph_height = Image.open(mapdir).size[1]
        # print(graph_height)

        with open(mapcfgdir, "r") as read_file:
            yaml_params = yaml.load(read_file, Loader=yaml.SafeLoader)
            resolution = yaml_params["resolution"]
            origin = yaml_params["origin"]
            # print(resolution)
            # print(origin)

        with open(waypointsdir, "r") as read_file:
            # Get untranslated waypoints.
            waypointsfile = json.load(read_file)
            untranslated_waypoints = waypointsfile['smoothed_wpts']
            points = np.array([point[0:2] for point in untranslated_waypoints])

            # Translate waypoints.
            points[:,0] = points[:,0] - origin[0] + 1.3
            points[:,1] = -points[:,1] + origin[1] - 1.4 + 2*(graph_height*resolution)

            # Compute distance attribute.
            self.x = []
            self.y = []
            self.d = [0]
            for i in range(1, len(points)):
                self.x.append(points[i,0])
                self.y.append(points[i,1])
                distance = np.linalg.norm(points[i] - points[i-1])
                self.d.append(self.d[i - 1] + distance)

        x_diffs = np.diff(self.x)
        y_diffs = np.diff(self.y)
        self.psi = np.arctan2(y_diffs, x_diffs)
        self.psi = np.append(self.psi, self.psi[-1])

        # Convert to arrays.
        self.x = np.array(self.x)
        self.y = np.array(self.y)
        self.psi = np.array(self.psi)
        self.d = np.array(self.d)


if __name__ == "__main__":
    waypoints()
