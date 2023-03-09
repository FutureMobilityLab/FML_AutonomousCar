"""File pgm_mixin.py

Description: This file implements a Mixin class. That is a class which is intended to be inherited,
by other classes, but is not a base class. It merely integrates standalone features defined in the Mixin
class to the class which is inheriting it. 

In this case, this mixin class contains several helper methods for parsing and accessing the grid map, and it 
associate yaml config file. 
"""

import yaml
import cv2
import numpy as np
from copy import deepcopy
import math

class GridmapMixin:

    def _extract_map_path(self, config_path):
        """Method extracts and returns the paths of the grid map and its associated yaml config,
        assuming they exist in the same directory and are named the same with the exception of the
        file extension.
        """
        root_path = config_path.split(".")[0]
        map_path = f"{root_path}.pgm"
        conf_path = f"{root_path}.yaml"
        return map_path, conf_path

    def _parse_map(self, file_name):
        """Method accepts the path to the grid map and reads the map into a numpy array."""
        image = cv2.imread(file_name,-1)
        return image 

    def _parse_config(self, conf_path):
        """Method parses the grid map yaml config file into a python dictionary."""
        with open(conf_path, 'r') as file:
            conf = yaml.safe_load(file)
        return conf

    def erode_img(self, img, kernel_shape=(3,3), num_iter=4):
        """Method applies CV erosion, to the desired grid map."""
        kernel = np.ones(kernel_shape, np.uint8)
        img_erosion = cv2.erode(img, kernel, iterations=num_iter)
        return img_erosion

    def dilate_img(self, img, kernel_shape=(3,3), num_iter=5):
        """Method applies CV dilation, to the desired grid map."""
        kernel = np.ones(kernel_shape, np.uint8)
        img_dilation = cv2.dilate(img, kernel, iterations=num_iter)
        return img_dilation
    
    def _circle_check(self, center, point, radius):
        dist = math.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2)
        return True if dist < radius else False

    def _inflation_region(self, ind, radius, grid_size):
        buffer = 0

        grid_x, grid_y = grid_size[0]-1, grid_size[1]-1

        x, y = ind[0], ind[1]

        min_x_offset = int(x - radius - buffer) 
        min_y_offset = int(y - radius - buffer )
        max_x_offset = int(x + radius + buffer )
        max_y_offset = int(y + radius + buffer )

        min_x_ind = min_x_offset if min_x_offset > 0 else 0
        min_y_ind = min_y_offset if min_y_offset > 0 else 0
        max_x_ind = max_x_offset if max_x_offset < grid_x else grid_x
        max_y_ind = max_y_offset if max_y_offset < grid_y else grid_y

        inf_x_range = range(min_x_ind, max_x_ind)
        inf_y_range = range(min_y_ind, max_y_ind)

        return inf_x_range, inf_y_range 

    def _inflate_map_obstacle(self, obs_radius, input_map):
        inf_map = deepcopy(input_map)

        for center, val in np.ndenumerate(input_map): 

            if val == 0: 
                x_range, y_range = self._inflation_region(center, obs_radius, inf_map.shape)
                for x_ind in x_range:
                    for y_ind in y_range:
                        check = self._circle_check(center, (x_ind, y_ind), obs_radius)

                        if check:
                            inf_map[(x_ind, y_ind)] = 0
        return inf_map
