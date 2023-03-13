# from controller import Controller
import numpy as np

class StanleyController():
    def __init__(self):
        self.k = 1 #control_gain
        self.k_soft = 1 #softening_gain
        self.L = 0.404 #wheelbase
        self.max_steer = 0.65 #max_steer
        self.prev_steering_angle = 0
        self.debug_bool = True
        self.velocity_setpoint = 1.0
        
    def getYawRef(self,waypoints,index):
        try:
            rise = waypoints[index+1][1]-waypoints[index][1]
            run = waypoints[index+1][0]-waypoints[index][0]
        except IndexError:
            rise = waypoints[index][1]-waypoints[index-1][1]
            run = waypoints[index][0]-waypoints[index-1][0]
        yawref = np.arctan2(rise,run)
        return yawref

    def get_commands(self,waypoints,x,y,yaw,v):
        distance_to_waypoint = []

        front_axle_x = x + self.L * np.cos(yaw)
        front_axle_y = y + self.L * np.sin(yaw)
        for i in range(len(waypoints)):
            distance_to_waypoint.append((front_axle_x - waypoints[i][0])**2 + (front_axle_y - waypoints[i][1])**2)
            nearest_waypoint_index = distance_to_waypoint.index(min(distance_to_waypoint))

        yaw_ref = self.getYawRef(waypoints,nearest_waypoint_index)
        ref_to_axle = np.array([front_axle_x - waypoints[nearest_waypoint_index][0], front_axle_y - waypoints[nearest_waypoint_index][1]])
        crosstrack_vector = np.array([np.cos(yaw_ref), np.sin(yaw_ref)])
        crosstrack_error = -(ref_to_axle[0]*crosstrack_vector[1] - \
                           ref_to_axle[1]*crosstrack_vector[0])
        
        if self.debug_bool == True:
            print('Pose: '+str(front_axle_x)+' '+str(front_axle_y)+'\tNearest Points: '+str(waypoints[nearest_waypoint_index][0])+' '+str(waypoints[nearest_waypoint_index][1]))
            print('Crosstrack Error:'+str(crosstrack_error)+'\tCrosstrack Vector:'+str(crosstrack_vector))
        # Stanley Control law.
        yaw_term = yaw_ref - yaw
        tangent_term = np.arctan((self.k*crosstrack_error /
                                  (v + self.k_soft)))
        steer_angle = yaw_term + tangent_term


        # Constrains steering angle to the vehicle limits.
        steering_angle = np.clip(steer_angle, -self.max_steer, self.max_steer)

        self.prev_steering_angle = steering_angle

        if waypoints[nearest_waypoint_index] == waypoints[-1]:
            speed_cmd = 0
        else:
            speed_cmd = self.velocity_setpoint

        if self.debug_bool == True:
            print('Psi: '+str(yaw_term)+'\tTangent Term: '+str(tangent_term))
            print('Stanley Output: '+str(steering_angle))

        return steering_angle, speed_cmd