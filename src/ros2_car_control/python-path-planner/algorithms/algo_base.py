"""File: algo_base.py

Description: This file provides the base functionality inherited by ALL algorithms classes. 
This includes the GrimapMixin for parsing the grid maps configuration file, and an abstract
'run()' method which the inheriting algorithm must implement.
"""

from abc import ABC, abstractmethod
from maps.pgm_mixin import GridmapMixin


class AlgorithmBase(ABC, GridmapMixin):

    def __init__(self, config):

        self._config = config
        self._start_point = tuple(config["planner"]["origin"])
        self._goal_point =tuple(config["planner"]["goal_point"])
        self._x0_offset = config["planner"]["x0_offset"] 
        self._y0_offset = config["planner"]["y0_offset"] 

        _, map_conf_path = self._extract_map_path(config["map"]["path"])
        map_conf = self._parse_config(map_conf_path)
        self._resolution = map_conf["resolution"]

        if config["planner"]["initialize_from_pixel_indices"]:
            self._start_idx = tuple(config["planner"]["start_ind"])
            self._goal_idx =tuple(config["planner"]["goal_ind"])
        else:
            self._start_idx = self.coords2px(self._start_point)
            self._goal_idx = self.coords2px(self._goal_point)

    #######################
    #   Public Methods
    #######################

    def coords2px(self, coords):
        """Converts the (x, y) coordinates of the Map (in meters) to Pixel indices (i,j)."""

        x = self._x0_offset + coords[0]
        y = self._y0_offset + coords[1]

        index_i = int(x/self._resolution)
        index_j = int(y/self._resolution)

        px_coords = (index_i, index_j)

        return px_coords

    def px2coord(self, indices):
        """Converts the Pixel indices (i,j) to (x, y) coordinates of the Map (in meters)."""

        x_coord = indices[1]*self._resolution + self._x0_offset
        y_coord = indices[0]*self._resolution + self._y0_offset
        coord = (x_coord, y_coord)
        return coord

    def convert_output_to_coords(self, output_indices):
        output_coords = [self.px2coord(ind) for ind in output_indices] 
        return output_coords

    #######################
    #   Abstract Methods
    #######################

    @abstractmethod
    def run(self):
        """Abstract method MUST be implement in class which inherits AlgorithmBase."""
        pass

    @abstractmethod
    def plot_path(self, smooth_path, smooth_path_idx):
        pass