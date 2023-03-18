import numpy as np
import yaml

class YoulaController():
    def __init__(self,waypoints):
        n_gcx = 7 #Number of controller states
        self.Gc_states = np.zeros((1,n_gcx))
        stream =  open('car_control.yaml','r')
        config_file = yaml.load(stream)
        self.GcA = np.matrix([config_file.get('GcA')])
        self.GcB = np.matrix([config_file.get('GcB')])
        self.GcC = np.matrix([config_file.get('GcC')])
        self.GcD = np.matrix([config_file.get('GcD')])
        self.waypoints = waypoints

    def get_commands(self,pose_x,pose_y,pose_psi,v):
        # Not sure if ROS 2 fixes this problem, but you might have to store the
        # waypoints locally so that the callback doesn't overwrite them.
        # waypoints = self.waypoints

        distance_to_waypoint = []
        # waypoints = []

        for i in range(len(self.waypoints.x)):
            distance_to_waypoint.append((pose_x - self.waypoints.x[i])**2 + (pose_y - self.waypoints.y[i])**2)
            # waypoints.append([self.waypoints.x[i],self.waypoints.y[i],self.waypoints.psi[i])
            nearest_waypoint_index = distance_to_waypoint.index(min(distance_to_waypoint))
        # nearest_waypoint_index = np.argmin(distance_to_waypoint)
        # ref_x, ref_y, ref_yaw = waypoints[nearest_waypoint_index]
        
        # lookahead = 0
        # front_axle_x = pose_x + lookahead * np.cos(pose_psi)
        # front_axle_y = pose_y + lookahead * np.sin(pose_psi)

        # ref_to_axle = np.array([front_axle_x - ref_x, front_axle_y - ref_y])
        # crosstrack_vector = np.array([np.sin(ref_yaw), -np.cos(ref_yaw)])
        # lateral_error = ref_to_axle.dot(crosstrack_vector)

        # This lateral error calculation might not work out if there is a
        # lookahead distance. Consider the example where there is zero
        # crosstrack error with a lookahead distance of 1 m. I think this calc
        # will produce a lateral error of 1 or -1 m. The above calculation is
        # what I used for the DOT project. Maybe it'll work.
        lateral_error_size = np.sqrt(distance_to_waypoint[nearest_waypoint_index])
        psi_vector = [np.cos(pose_psi),np.sin(pose_psi)]
        error_vector = [self.waypoints.x[nearest_waypoint_index] - pose_x, self.waypoints.y[nearest_waypoint_index]-pose_y]
        lateral_error_sign = np.sign(np.cross(psi_vector,error_vector)[2])

        lateral_error = lateral_error_size  * lateral_error_sign

        # new_Gc_states = self.GcA.dot(self.Gc_states) \
        #    + self.GcB.dot(lateral_error)
        # steering_angle = self.GcC.dot(self.Gc_states) \
        #    + self.GcD.dot(lateral_error)

        # Not sure if you'll have a dimension issue here. If you do try the
        # code snippet above. Alternatively you replace "*" with "@" in 
        # python 3.
        new_Gc_states = self.GcA * self.Gc_states + self.GcB * lateral_error
        steering_angle = self.GcC * self.Gc_states + self.GcD * lateral_error

        self.Gc_states = new_Gc_states
        return steering_angle