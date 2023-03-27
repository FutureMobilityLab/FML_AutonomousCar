import json
import numpy as np

class waypoints():
     def __init__(self):
        waypointsdir = '/home/george/FML_AutonomousCar/src/ros2_car_control/config/waypoints.json'
        with open(waypointsdir,"r") as read_file:
            waypointsfile = json.load(read_file)
            untranslated_waypoints = waypointsfile['smoothed_wpts']
            self.x = np.array([x[1] for x in untranslated_waypoints]) #switch back later
            self.y = np.array([y[0] for y in untranslated_waypoints])
        
            self.x = self.x*0.05 - 0.466    #comment if not using conversions from starter map
            self.y = -self.y*0.05 +2.1

        x_diffs = np.diff(self.x)
        y_diffs = np.diff(self.y)
        self.psi = np.arctan2(y_diffs,x_diffs)
        self.psi = np.append(self.psi, self.psi[-1])
        print(f"First point: [{self.x[1]},{self.y[1]}]")
        print(f"Last point: [{self.x[-1]},{self.y[-1]}]")
