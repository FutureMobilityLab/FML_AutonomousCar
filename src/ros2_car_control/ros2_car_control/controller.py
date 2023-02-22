import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import json

from ros2_car_control.ros2_car_control import stanleyController, mpcController, youlaController

class Controller(Node):
    def __init__(self):
        super().__init__("car_controller")
        self.localization_subscriber = self.create_subscription(Odometry,'odometry/filtered',self.odometry_callback,10)
        # TODO: ADD HEARTBEAT SUBSCRIBER
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped,'cmd_ackermann',10)
        # TODO: ADD ACTION CLIENT TO GENERATE WAYPOINTS, RETURN FILEPATH OF JSON
        # self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.controller)
        # TODO: FETCH WAYPOINTS FROM ACTION CLIENT FILEPATH
        self.control_method = "stanley"
        self.speed_setpoint = 1

    def odometry_callback(self,msg):
        self.v = msg.twist.twist.linear.x
        # TODO: EULER FROM QUATERNION TO GET X,Y, YAW

    def controller(self):
        if self.v > 2:  #if odom unsafe, comms lost, etc:
            self.cmd_steer = 0
            self.cmd_speed = 0
        # TODO: ADD PATH CONSTRAINTS TO KILL MOTOR IF OUT OF RANGE
        else:
            match self.control_method:
                case "stanley":
                    self.cmd_steer = stanleyController()
                    self.cmd_speed = self.speed_setpoint
                case "mpc":
                    self.cmd_steer = mpcController()
                    self.cmd_speed = self.speed_setpoint
                case "youla":
                    self.cmd_steer = youlaController()
                    self.cmd_speed = self.speed_setpoint
        
        ackermann_command = AckermannDriveStamped()
        ackermann_command.header.stamp = self.get_clock().now().to_msg()
        ackermann_command.drive.speed = self.cmd_speed
        ackermann_command.drive.steering_angle = self.cmd_steer
        self.ackermann_publisher.publish(ackermann_command)

def main(args=None):
    rclpy.init(args=args)

    car_controller = Controller()

    rclpy.spin(car_controller)

    # Destroy the node explicitly
    car_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()