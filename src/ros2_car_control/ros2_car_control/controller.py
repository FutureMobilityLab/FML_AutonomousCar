import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

class Controller(Node):
    pass

def main(args=None):
    rclpy.init(args=args)

    car_controller = Controller()

    rclpy.spin(car_controller)

    # Destroy the node explicitly
    car_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()