#!/usr/bin/env python3
from planner import PathPlanner

import sys
import rclpy
from rclpy.node import Node


class AsyncPathPlannerClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        self.cli = self.create_client(PathPlanner, 'path_planner_service')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.req = PathPlanner.Request()

    def send_request(self, my_path="www.google.com"):
        self.req.map_path = my_path
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = AsyncPathPlannerClient()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(f"Result {response.path_json_path}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()