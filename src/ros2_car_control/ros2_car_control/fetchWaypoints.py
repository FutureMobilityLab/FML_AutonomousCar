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
            waypointsfile = json.load(read_file)
            untranslated_waypoints = waypointsfile["smoothed_wpts"]
            translate_x = lambda x: x - origin[0] + 1.3
            translate_y = (
                lambda y: -y + origin[1] - 1.4 + 2 * (graph_height * resolution)
            )
            self.x = [translate_x(untranslated_waypoints[0][0])]
            self.y = [translate_y(untranslated_waypoints[0][1])]
            self.d = [0]
            for i in range(1, len(untranslated_waypoints)):
                prev_point = np.array(
                    translate_x(untranslated_waypoints[i - 1][0]),
                    translate_y(untranslated_waypoints[i - 1][1]),
                )
                point = np.array(
                    translate_x(untranslated_waypoints[i][0]),
                    translate_y(untranslated_waypoints[i][1]),
                )
                self.x.append(point[0])
                self.y.append(point[1])
                distance = np.linalg.norm(point - prev_point)
                self.d.append(self.d[i - 1] + distance)

            self.x = (
                self.x - origin[0] + 1.3
            )  # comment if not using conversions from starter map
            self.y = -self.y + origin[1] - 1.4 + 2 * (graph_height * resolution)
            # print(self.y)

        x_diffs = np.diff(self.x)
        y_diffs = np.diff(self.y)
        self.psi = np.arctan2(y_diffs, x_diffs)
        self.psi = np.append(self.psi, self.psi[-1])
        # print(f"First point: [{self.x[1]},{self.y[1]}]")
        # print(f"Last point: [{self.x[-1]},{self.y[-1]}]")


if __name__ == "__main__":
    waypoints()
