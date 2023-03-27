import numpy as np

class YoulaController():
    def __init__(self,waypoints,ctrl_params):
        n_GcX = ctrl_params.get("n_GcX") #Number of controller states
        self.waypoints = waypoints

        self.Gc_states = np.zeros((n_GcX,1))
        self.GcA = np.matrix(np.array(ctrl_params.get("GcA")).reshape(n_GcX,n_GcX))
        self.GcB = np.matrix(np.array(ctrl_params.get("GcB")).reshape(n_GcX,1))
        self.GcC = np.matrix(np.array(ctrl_params.get("GcC")).reshape(1,n_GcX))
        self.GcD = np.matrix(np.array(ctrl_params.get("GcD")).reshape(1,1))

        self.v = ctrl_params.get("speed_setpoint") # Velocity Setpoint

    def get_commands(self,pose_x,pose_y,pose_psi,v):

        distance_to_waypoint = []

        lookahead = 0.404
        front_axle_x = pose_x + lookahead * np.cos(pose_psi)
        front_axle_y = pose_y + lookahead * np.sin(pose_psi)

        for i in range(len(self.waypoints.x)):
            distance_to_waypoint.append((front_axle_x - self.waypoints.x[i])**2 + (front_axle_y - self.waypoints.y[i])**2)

            nearest_waypoint_index = distance_to_waypoint.index(min(distance_to_waypoint))

        # nearest_waypoint_index = np.argmin(distance_to_waypoint)
        ref_x = self.waypoints.x[nearest_waypoint_index]
        ref_y = self.waypoints.y[nearest_waypoint_index]
        ref_psi = self.waypoints.psi[nearest_waypoint_index]
        

        ref_to_axle = np.array([front_axle_x - ref_x, front_axle_y - ref_y])
        crosstrack_vector = np.array([np.sin(ref_psi), -np.cos(ref_psi)])
        lateral_error = -ref_to_axle.dot(crosstrack_vector)

        # Uncomment below for option 2.
        # lateral_error += pose_psi - ref_psi

        # This lateral error calculation might not work out if there is a
        # lookahead distance. Consider the example where there is zero
        # crosstrack error with a lookahead distance of 1 m. I think this calc
        # will produce a lateral error of 1 or -1 m. The above calculation is
        # what I used for the DOT project. Maybe it'll work.
        lateral_error_size = np.sqrt(distance_to_waypoint[nearest_waypoint_index])
        psi_vector = [np.cos(pose_psi),np.sin(pose_psi),0]
        error_vector = [self.waypoints.x[nearest_waypoint_index] - pose_x, self.waypoints.y[nearest_waypoint_index]-pose_y,0]
        lateral_error_sign = np.sign(np.cross(psi_vector,error_vector)[2])

        lateral_error = np.matrix([lateral_error_size  * lateral_error_sign])

        new_Gc_states = self.GcA @ self.Gc_states + self.GcB @ lateral_error
        steering_angle = float(self.GcC @ self.Gc_states + self.GcD @ lateral_error)

        self.Gc_states = new_Gc_states
        speed_cmd = self.v
        return steering_angle, speed_cmd, self.waypoints.x[nearest_waypoint_index], self.waypoints.y[nearest_waypoint_index]