# from controller import Controller
import numpy as np

class PurePursuitController():
    def __init__(self,waypoints,ctrl_params):
        self.L = ctrl_params.get("L") #wheelbase
        self.max_steer = ctrl_params.get("max_steer") #max_steer
        self.velocity_setpoint = ctrl_params.get("speed_setpoint")
        self.lookahead_dist = ctrl_params.get("pp_lookahead")

        self.prev_steering_angle = 0.0
        self.debug_bool = False

        self.waypoints = waypoints

    def get_commands(self,x,y,yaw,v):
        distance_to_waypoint = []

        front_axle_x = x + self.L/2.0 * np.cos(yaw)
        front_axle_y = y + self.L/2.0 * np.sin(yaw)
        for i in range(len(self.waypoints.x)):
            distance_to_waypoint.append(np.sqrt((front_axle_x - self.waypoints.x[i])**2 + (front_axle_y - self.waypoints.y[i])**2))
            nearest_waypoint_index = distance_to_waypoint.index(min(distance_to_waypoint))

        for i in range(nearest_waypoint_index,len(self.waypoints.x)):
            if distance_to_waypoint[i] >= self.lookahead_dist:
                lookahead_vec_y = front_axle_y-self.waypoints.y[nearest_waypoint_index]
                lookahead_vec_x = front_axle_x-self.waypoints.x[nearest_waypoint_index]
                steer_angle = np.arctan2(lookahead_vec_y,lookahead_vec_x)


        # Constrains steering angle to the vehicle limits.
        steering_angle = np.clip(steer_angle, -self.max_steer, self.max_steer)

        speed_cmd = self.velocity_setpoint

        return steering_angle, speed_cmd, self.waypoints.x[nearest_waypoint_index], self.waypoints.y[nearest_waypoint_index]
