# from controller import Controller
import numpy as np

class StanleyController():
    def __init__(self,waypoints):
        self.k = .5 #control_gain
        self.k_soft = 1 #softening_gain
        self.L = 0.404 #wheelbase
        self.max_steer = 0.65 #max_steer
        self.prev_steering_angle = 0
        self.debug_bool = True
        self.velocity_setpoint = 1.0
        self.waypoints = waypoints

    def get_commands(self,x,y,yaw,v):
        distance_to_waypoint = []

        front_axle_x = x + self.L * np.cos(yaw)
        front_axle_y = y + self.L * np.sin(yaw)
        for i in range(len(self.waypoints.x)):
            distance_to_waypoint.append(np.sqrt((front_axle_x - self.waypoints.x[i])**2 + (front_axle_y - self.waypoints.y[i])**2))
            nearest_waypoint_index = distance_to_waypoint.index(min(distance_to_waypoint))

        yaw_ref = self.waypoints.psi[nearest_waypoint_index]
        ref_to_axle = np.array([front_axle_x - self.waypoints.x[nearest_waypoint_index], front_axle_y - self.waypoints.y[nearest_waypoint_index]])
        crosstrack_vector = np.array([np.cos(yaw_ref), np.sin(yaw_ref)])
        crosstrack_error = -(ref_to_axle[0]*crosstrack_vector[1] - \
                           ref_to_axle[1]*crosstrack_vector[0])
        
        if self.debug_bool == True:
            print('Pose: '+str(front_axle_x)+' '+str(front_axle_y)+'\tNearest Points: '+str(self.waypoints.x[nearest_waypoint_index])+' '+str(self.waypoints.y[nearest_waypoint_index]))
            print('Crosstrack Error:'+str(crosstrack_error)+'\tCrosstrack Vector:'+str(crosstrack_vector))
        # Stanley Control law.
        yaw_term = yaw_ref - yaw
        tangent_term = np.arctan((self.k*crosstrack_error /
                                  (v + self.k_soft)))
        steer_angle = yaw_term + tangent_term


        # Constrains steering angle to the vehicle limits.
        steering_angle = np.clip(steer_angle, -self.max_steer, self.max_steer)

        self.prev_steering_angle = steering_angle

        if nearest_waypoint_index == len(self.waypoints.x):
            speed_cmd = 0
        else:
            speed_cmd = self.velocity_setpoint

        if self.debug_bool == True:
            print('Psi: '+str(yaw_term)+'\tTangent Term: '+str(tangent_term))
            print('Stanley Output: '+str(steering_angle))

        return steering_angle, speed_cmd