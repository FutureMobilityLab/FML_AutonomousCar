import rclpy
from rclpy.node import Node
from fetchWaypoints import waypoints
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler


class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'waypoints', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.message_out = MarkerArray()
        self.waypoints_to_publish = waypoints()

    def timer_callback(self):  
        self.message_out.markers = []
        for i in range(len(self.waypoints_to_publish.x)):
            marker_out = Marker()
            marker_out.header.frame_id = "map"
            marker_out.header.stamp = self.get_clock().now().to_msg()
            marker_out.type = Marker.ARROW
            marker_out.action = Marker.ADD
            marker_out.ns = "wpts_"+str(i)
            marker_out.id = i
            marker_out.pose.position.x = self.waypoints_to_publish.x[i]
            marker_out.pose.position.y = self.waypoints_to_publish.y[i]
            marker_out.scale.x = 0.1
            marker_out.scale.y = 0.01
            marker_out.scale.z = 0.01
            [ori_x,ori_y,ori_z,ori_w] = quaternion_from_euler(0.0,0.0,self.waypoints_to_publish.psi[i])
            marker_out.pose.orientation.x = ori_x
            marker_out.pose.orientation.y = ori_y
            marker_out.pose.orientation.z = ori_z
            marker_out.pose.orientation.w = ori_w
            marker_out.frame_locked = False
            marker_out.color.a = 1.0
            marker_out.color.b = 1.0
            marker_out.lifetime = Duration(seconds=1).to_msg()
            self.message_out.markers.append(marker_out)   
        self.publisher_.publish(self.message_out)


def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()