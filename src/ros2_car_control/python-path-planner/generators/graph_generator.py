"""File: graph_generator.py

Description: Most of the path planning algorithms that we implement in this project are based on a 'graph search'. This file 
implements the mechanics of generating the mathematical graph which our planner algorithms need. Specifically, this file implements
the GraphGenerator class which does most of the heavy lifting in generating the graph from a probablistic
occupancy grid map, which gets passed in via a config file. Additionally, it applies erosion, dilation, & inflations operations, to 
in attempt to prevent ill-posed paths. These include, mapping noise, wall hugging and more...

NOTE: As an abstract base class, classes which inherit this base functionality must implement the methods 
reconstuct_and_plot_grid_map and  build_graph.
"""
from copy import deepcopy
import numpy as np 
from maps.pgm_mixin import GridmapMixin
from datastructures.graph import Graph
from datastructures.node import Node


class GraphGenerator(GridmapMixin):
    default_config = "configs/default_config.yaml"

    def __init__(self, config=None, graph=Graph, node=Node):
        self._graph = graph()
        self._node = node
        self._config = config if config else self.default_config

        # Extract Grid Map and Config Paths 
        self._map_path, self._map_conf_path = self._extract_map_path(config["map"]["path"])
        self._map_conf = self._parse_config(self._map_conf_path)
        self._pgm_map = self._parse_map(self._map_path)
        self._original_map = deepcopy(self._pgm_map)

        # General Configuration
        self._map_high = config["map"]["threshold"]
        self._connectivity = True if config["planner"]["Astar"]["connectivity"] == "connected-8" else False
        self._major_diam_in = config["vehicle"]["major_diameter"]
        self._minor_diam_in = config["vehicle"]["minor_diameter"]
        self._inflation_radius = self._config["map"]["inflation_radius"] # in Pixels

        # Map Configuration 
        self._resolution = self._map_conf["resolution"] # meters/pixel
        self._mode = self._map_conf["mode"]
        self._origin = self._map_conf["origin"]
        self._occupied_thresh = self._map_conf["occupied_thresh"]
        self._free_thresh = self._map_conf["free_thresh"]
        self._pgm_name = self._map_conf["image"]

        # Constants 
        self._inch_2_meters = 0.0254
        self._meters_2_inch = 39.3701
        self._minor_robot_diam = self._minor_diam_in * self._inch_2_meters        # Meters
        self._major_robot_diam =  self._major_diam_in * self._inch_2_meters       # Meters  

        input_map = self._original_map
        input_map = self._dilate_and_erode_map(input_map)

        # Apply Obstacle Inflation
        if self._config["map"]["inflate"]:            
            input_map = self._inflate_map_obstacle(self._inflation_radius, input_map)

        self._pgm_map = input_map 
        
        
    #############################
    #       Private Methods 
    #############################

    def _dilate_and_erode_map(self, input_map):

        if self._config["map"]["cv"]["apply"]:
        
            # Apply Erosion
            shape = self._config["map"]["cv"]["kernel"]
            iter = self._config["map"]["cv"]["erosion"]["num_iter"]
            eroded_map = self.erode_img(input_map, shape,iter)

            # Apply Dialation 
            shape = self._config["map"]["cv"]["kernel"]
            iter = self._config["map"]["cv"]["dilation"]["num_iter"]
            dilated_map = self.dilate_img(input_map, shape,iter)

            composite_img = dilated_map

            for index , val in np.ndenumerate(eroded_map):

                if val ==0:
                    composite_img[index] = val

            return composite_img
        return input_map

    def build_graph(self, prune=True):

        for idx, value in np.ndenumerate(self._pgm_map):
            self._graph.add_node(self._node(index=(idx[0],idx[1])))

        # Create all Edges in the Graph (Connected-4)
        for idx, value in np.ndenumerate(self._pgm_map):

            if value < self._map_high:
                continue

            i, j = idx[0], idx[1]

            n_offset = i-1 if i-1 >= 0 else 0
            s_offset = i+1 if i+1 <self._pgm_map.shape[0] else self._pgm_map.shape[0] -1
            e_offset = j-1 if j-1 >= 0 else 0
            w_offset = j+1 if j+1 <self._pgm_map.shape[1] else self._pgm_map.shape[1] -1


            ne_idx = (n_offset, e_offset) 
            nw_idx = (n_offset, w_offset) 

            se_idx = (s_offset, e_offset) 
            sw_idx = (s_offset, w_offset)


            north_idx = (n_offset, j)
            south_idx = (s_offset, j)
            east_idx = (i, e_offset)
            west_idx = (i, w_offset)

            cur_node = self._graph.get_node_from_index(idx)

            # North Node 
            if self._pgm_map[north_idx] == self._map_high:
                north_node = self._graph.get_node_from_index(north_idx)
                self._graph.add_edge(cur_node, north_node, 1)
            # South Node
            if self._pgm_map[south_idx] == self._map_high:
                south_node = self._graph.get_node_from_index(south_idx)
                self._graph.add_edge(cur_node, south_node, 1)
            # West Node
            if self._pgm_map[west_idx] == self._map_high:
                west_node = self._graph.get_node_from_index(west_idx)
                self._graph.add_edge(cur_node, west_node, 1)
            # East Node
            if self._pgm_map[east_idx] == self._map_high:
                east_node = self._graph.get_node_from_index(east_idx)
                self._graph.add_edge(cur_node, east_node, 1)

            if self._connectivity:
                # North West Node 
                if self._pgm_map[nw_idx] == self._map_high:
                    nw_node = self._graph.get_node_from_index(nw_idx)
                    self._graph.add_edge(cur_node, nw_node, 1.414)
                # North East Node 
                if self._pgm_map[ne_idx] == self._map_high:
                    nw_node = self._graph.get_node_from_index(ne_idx)
                    self._graph.add_edge(cur_node, nw_node, 1.414)
                # South West Node 
                if self._pgm_map[sw_idx] == self._map_high:
                    sw_node = self._graph.get_node_from_index(sw_idx)
                    self._graph.add_edge(cur_node, sw_node, 1.414)
                # South East Node 
                if self._pgm_map[se_idx] == self._map_high:
                    se_node = self._graph.get_node_from_index(se_idx)
                    self._graph.add_edge(cur_node, se_node, 1.414)

        if prune:
            self._graph.prune_graph()

        return self._graph
