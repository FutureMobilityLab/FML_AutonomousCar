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
        self.v = config_file.get("speed_setpoint") # Velocity Setpoint
        print(self.GcA)

    def get_commands(self,pose_x,pose_y,pose_psi,v):
        # Not sure if ROS 2 fixes this problem, but you might have to store the
        # waypoints locally so that the callback doesn't overwrite them.
        # waypoints = self.waypoints

        distance_to_waypoint = []
        # waypoints = []

        # There are two ways we can implement the output feedback here: 
        #   1. we find the reference waypoint closest to some point ahead of
        #   the vehicle (lookahead) and compute crosstrack error.
        #   2. we find the reference waypoint closest to the vehicle c.g. and
        #   compute crosstrack error and yaw error. Then the total error is
        #   crosstrack_error + lookahead*yaw_error.

        # lookahead = 0
        # front_axle_x = pose_x + lookahead * np.cos(pose_psi)
        # front_axle_y = pose_y + lookahead * np.sin(pose_psi)

        for i in range(len(self.waypoints.x)):
            # Uncomment below 3 lines for option 1.
            # distance_to_waypoint.append(
            #     (front_axle_x - self.waypoints.x[i])**2 \
            #         + (front_axle_y - self.waypoints.y[i])**2)

            distance_to_waypoint.append((pose_x - self.waypoints.x[i])**2 + (pose_y - self.waypoints.y[i])**2)
            # waypoints.append([self.waypoints.x[i],self.waypoints.y[i],self.waypoints.psi[i])
            nearest_waypoint_index = distance_to_waypoint.index(min(distance_to_waypoint))
        # nearest_waypoint_index = np.argmin(distance_to_waypoint)
        # ref_x, ref_y, ref_psi = waypoints[nearest_waypoint_index]

        # ref_to_axle = np.array([front_axle_x - ref_x, front_axle_y - ref_y])
        # crosstrack_vector = np.array([np.sin(ref_psi), -np.cos(ref_psi)])
        # lateral_error = -ref_to_axle.dot(crosstrack_vector)

        # Uncomment below for option 2.
        # lateral_error += pose_psi - ref_psi

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
        speed_cmd = self.v
        return steering_angle, speed_cmd, self.waypoints.x[nearest_waypoint_index], self.waypoints.y[nearest_waypoint_index]