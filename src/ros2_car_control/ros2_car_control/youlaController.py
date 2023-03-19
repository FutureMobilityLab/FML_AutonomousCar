import numpy as np
import yaml, os

class YoulaController():
    def __init__(self,waypoints):
        n_gcx = 7 #Number of controller states
        self.Gc_states = np.zeros((1,n_gcx))
        config_path = os.path.join(__file__,'../config/car_control.yaml')
        stream =  open(config_path,'r')
        config_file = yaml.load(stream)
        self.GcA = np.matrix([config_file.get('Youla_GcA')])
        self.GcB = np.matrix([config_file.get('Youla_GcB')])
        self.GcC = np.matrix([config_file.get('Youla_GcC')])
        self.GcD = np.matrix([config_file.get('Youla_GcD')])
        self.waypoints = waypoints
        print(self.GcA)

    def get_commands(self,pose_x,pose_y,pose_psi,v):
        distance_to_waypoint = []

        for i in range(len(self.waypoints.x)):
            distance_to_waypoint.append((pose_x - self.waypoints.x[i])**2 + (pose_y - self.waypoints.y[i])**2)
            nearest_waypoint_index = distance_to_waypoint.index(min(distance_to_waypoint))

        lateral_error_size = np.sqrt(distance_to_waypoint[nearest_waypoint_index])
        psi_vector = [np.cos(pose_psi),np.sin(pose_psi)]
        error_vector = [self.waypoints.x[nearest_waypoint_index] - pose_x, self.waypoints.y[nearest_waypoint_index]-pose_y]
        lateral_error_sign = np.sign(np.cross(psi_vector,error_vector)[2])

        lateral_error = lateral_error_size  * lateral_error_sign

        new_Gc_states = self.GcA * self.Gc_states + self.GcB * lateral_error
        steering_angle = self.GcC * self.Gc_states + self.GcD * lateral_error

        self.Gc_states = new_Gc_states
        return steering_angle