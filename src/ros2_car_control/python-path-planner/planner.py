#!/usr/bin/env python3

"""File: planner.py

Description: This file implements the project wide planner class whose responsibility it is to ingest a planner config,
and output the corresponding waypoints to a JSON file. This PathPlanner class is the main engine driving the operation
of our ROS2 path planning service. When the ROS2 path planning client send a request to the server, the server will invoke
this class while passing it the path to the path planner configuration file. 

NOTE: that this file also doubles as a planner utility with a 
command line interface (CLI), which the user can invoke directly from the console.
"""
# From Python Standard Library
from argparse import ArgumentParser
from datetime import datetime
import json
from pathlib import Path
import yaml

from algorithms import AStar, RRT, RRTStar, PRM, DijkstaShortestPath
from post_processing.path_smoothing import BezierPathSmoothing


class PathPlanner:
    DEF_CONFIG_PATH = "configs/connected_8.yaml"
    planner_mapping = {"Astar": AStar, "RRT": RRT, "RRTstar": RRTStar, "PRM": PRM, "Dijkstra": DijkstaShortestPath}
    
    def __init__(self, conf_path=None, preview_path=False):
        self._config_path = conf_path if conf_path else Path(self.DEF_CONFIG_PATH).absolute()
        self._preview = preview_path
        self._config = self._load_config()

        self._algo = None

    #######################
    #   Private Methods
    #######################
 
    def _load_config(self):
        """Load Path Planner Config from path."""
        with open(self._config_path , 'r') as file:
            full_config = yaml.safe_load(file)     
        config = full_config["FML"]
        return config

    def _run_planner(self):
        """Method runs the selected planner algorithm and returns the outputs."""
        planner = self._config["planner"]["algorithm"]
        self._algo = self.planner_mapping[planner](self._config)
        output, output_idx = self._algo.run()
        return output, output_idx

    def _apply_path_smoothing(self, raw_data_coords, raw_data_idx):
        """Method  applies path smoothing the output of the planning algorithm and returns waypoints,
        along the smoothed trajectory.
        """
        bezier = BezierPathSmoothing(num_waypoints=self._config["smoothing"]["num_wpts"])

        x_smooth_idx, y_smooth_idx = bezier.compute_smooth_path(raw_data_idx)
        x_smooth_coords, y_smooth_coords = bezier.compute_smooth_path(raw_data_coords)  

        output_smooth_idx = [(x_smooth_idx[i], y_smooth_idx[i]) for i in range(len(x_smooth_idx))]
        output_smooth_coords = [(x_smooth_coords[i], y_smooth_coords[i]) for i in range(len(x_smooth_coords))]

        return output_smooth_coords, output_smooth_idx

    def _export_path_data(self, raw_data, smooth_data):
        """Method exports the path planning data into a JSON file, along with meta data."""

        meta_data = {"map": self._config["map"], "planner": self._config["planner"]}
        json_dict = {"meta": meta_data,"raw_wpts": raw_data, "smoothed_wpts": smooth_data}

        dt = datetime.now().strftime("%Y%m%dT%H%M%S")
        path_to_json = f"{self._config['output']['path']}/{dt}_path_planner.json" 
        
        with open(path_to_json, "w") as outfile:
            json.dump(json_dict, outfile)

        return path_to_json

    #######################
    #   Public Methods
    ####################### 

    def run(self):
        """Methods runs the entire plath planning sequence."""
        output_path, output_indices = self._run_planner()
        smooth_path, smooth_path_idx = self._apply_path_smoothing(output_path, output_indices)
        path_to_json = self._export_path_data(output_path, smooth_path)

        if self._preview:
            self._algo.plot_path(smooth_path, smooth_path_idx)

        return path_to_json

def cli_options():
    parser = ArgumentParser()
    parser.add_argument(
        "--config",
        default=None,
        help="Path to a Path Planner yaml configuration file."
    )

    parser.add_argument(
        "--preview-path",
        default=False,
        action="store_true",
        help="Option to preview the solved path if it exists"
    )

    return parser.parse_args()

def main():
    args = cli_options()
    conf_path = args.config
    preview_path = args.preview_path

    planner = PathPlanner(conf_path, preview_path)
    planner.run()


if __name__ == "__main__":
    main()   