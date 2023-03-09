#!/usr/bin/env python3
from ros2_path_planning_interfaces.srv import PathPlanner
from planner import PathPlanner

import rclpy
from rclpy.node import Node

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planning_service')
        self.srv = self.create_service(PathPlanner, 'path_planner_service', self.compute_path)
        self._logger = self.get_logger()
        self._logger.info(f"Starting path planner service.")
    
    def compute_path(self, request, response):
        # Run Path Planning Algorithm
        self._logger.info("Instantiating Path Planner")
        planner = PathPlanner(request.conf_path, False)
        self._logger.info("Computing Path... ")
        planner.run()
        self._logger.info("Path Planning Complete!")

        # Return Path to Output Data
        response.path_json_path = planner._planner_output_path
        return response
    
def main():
    rclpy.init()
    path_serv = PathPlannerService()
    rclpy.spin(path_serv)
    rclpy.shutdown()

if __name__ == "__main__":
    main()