# from controller import Controller
import numpy as np
import time

class StanleyController():
    def __init__(self,waypoints,ctrl_params):
        self.k = ctrl_params.get("k") #control_gain
        self.k_soft = ctrl_params.get("k_soft") #softening_gain
        self.L = ctrl_params.get("L") #wheelbase
        self.max_steer = ctrl_params.get("max_steer") #max_steer
        self.velocity_setpoint = ctrl_params.get("speed_setpoint")

        self.prev_steering_angle = 0.0
        self.debug_bool = False

        self.waypoints = waypoints

    def get_commands(self,x,y,yaw,v):
        distance_to_waypoint = []

        front_axle = np.array([[x + self.L/2.0 * np.cos(yaw), y + self.L/2.0 * np.sin(yaw)]])
        now = time.time()
        print(f"Getting closest waypoint:{now}")
        waypoints = np.array([[self.waypoints.x, self.waypoints.y]])
        dist = np.norm(front_axle - waypoints,axis=0)
        nearest_waypoint_index = np.argmin(dist)
        print(f"Done getting closest waypoint, took: {time.time() - now}s")
        now = time.time()

        yaw_ref = self.waypoints.psi[nearest_waypoint_index]
        body_to_ref = np.array([front_axle_x - self.waypoints.x[nearest_waypoint_index], front_axle_y - self.waypoints.y[nearest_waypoint_index],0.0])
        path_vector = np.array([np.cos(yaw_ref), np.sin(yaw_ref),0.0])
        crosstrack_error = distance_to_waypoint[nearest_waypoint_index]
        error_sign = np.sign(np.cross(body_to_ref,path_vector)[2])
        
        if self.debug_bool == True:
            print('Pose: '+str(front_axle_x)+' '+str(front_axle_y)+'\tNearest Points: '+str(self.waypoints.x[nearest_waypoint_index])+' '+str(self.waypoints.y[nearest_waypoint_index]))
            print('Crosstrack Error:'+str(crosstrack_error)+'\tCrosstrack Vector:'+str(path_vector))
        # Stanley Control law.
        yaw_term = yaw_ref - yaw
        tangent_term = np.arctan2((self.k*crosstrack_error*error_sign),(v + self.k_soft))
        steer_angle = yaw_term + tangent_term


        # Constrains steering angle to the vehicle limits.
        steering_angle = np.clip(steer_angle, -self.max_steer, self.max_steer)

        self.prev_steering_angle = steering_angle

        speed_cmd = self.velocity_setpoint

        if self.debug_bool == True:
            print('Psi: '+str(yaw_term)+'\tTangent Term: '+str(tangent_term))
            print('Stanley Output: '+str(steering_angle))
        print(f'Done computing stanley output, took:{time.time() - now}s')

        return steering_angle, speed_cmd, self.waypoints.x[nearest_waypoint_index], self.waypoints.y[nearest_waypoint_index]
