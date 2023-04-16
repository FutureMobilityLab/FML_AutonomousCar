import json
import numpy as np
import yaml
from PIL import Image

class waypoints():
     def __init__(self):
        waypointsdir = '/home/george/FML_AutonomousCar/src/ros2_car_control/config/waypoints2.json'
        mapdir = '/home/george/FML_AutonomousCar/src/car_slam/config/lab_map.pgm'
        mapcfgdir = '/home/george/FML_AutonomousCar/src/car_slam/config/lab_map.yaml'
        #Get Image Properties
        graph_height = Image.open(mapdir).size[1]
        print(graph_height)

        with open(mapcfgdir,"r") as read_file:
            yaml_params = yaml.load(read_file, Loader=yaml.SafeLoader)
            resolution = yaml_params["resolution"]
            origin = yaml_params["origin"]
            print(resolution)
            print(origin)

        with open(waypointsdir,"r") as read_file:
            waypointsfile = json.load(read_file)
            untranslated_waypoints = waypointsfile['smoothed_wpts']
            self.x = np.array([x[1] for x in untranslated_waypoints]) #switch back later
            self.y = np.array([y[0] for y in untranslated_waypoints])
        
            self.x = self.x#*resolution + origin[0]   #comment if not using conversions from starter map
            self.y = -self.y#*resolution + (graph_height*resolution+origin[1])
            print(self.y)

        x_diffs = np.diff(self.x)
        y_diffs = np.diff(self.y)
        self.psi = np.arctan2(y_diffs,x_diffs)
        self.psi = np.append(self.psi, self.psi[-1])
        print(f"First point: [{self.x[1]},{self.y[1]}]")
        print(f"Last point: [{self.x[-1]},{self.y[-1]}]")

if __name__ == '__main__':
    waypoints()