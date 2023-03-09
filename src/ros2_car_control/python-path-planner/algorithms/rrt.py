"""File: rrt.py

Description: This file implement the Rapidly Exploring Random Tree (RRT) algorithm. This algorithm 
will leverage the simplicity of using 'tree' data structures over the instantiation of a graph. Also \
note that unlike other graph search algorithms (A*, Dijkstra) ...etc we cannot build a graph before
applying the algorithm, since RRT using runtime random sampling to populate its tree datastructure. 
"""

import random
import math
from copy import deepcopy
from math import sqrt

import matplotlib.pyplot as plt

from datastructures.tree import Tree, TreeNode
from algorithms.algo_base import AlgorithmBase
from post_processing.bresenham import bresenham


class RRT(AlgorithmBase):

    def __init__(self, config):
        super().__init__(config)    

        self._start_point = tuple(self._config["planner"]["start_ind"])
        self._goal_point =tuple(self._config["planner"]["goal_ind"])
        self._goal_node = None 

        self._tree = Tree(root_pos=self._start_point)
        
        self._map_path, self._map_conf_path = self._extract_map_path(self._config["map"]["path"])
        self._map_conf = self._parse_config(self._map_conf_path)
        self._map = self._parse_map(self._map_path) 
        self._map_size = self._map.shape

        self._num_nodes = self._config["planner"]["RRT"]["num_samples"]
        self._step_size = self._config["planner"]["RRT"]["step_size"]

        self._inflation_radius = self._config["map"]["inflation_radius"] # in Pixels
        self._original_map = deepcopy(self._map)

        # Apply Obstacle Inflation
        if self._config["map"]["inflate"]:            
            self._map = self._inflate_map_obstacle(self._inflation_radius, self._map)

    #######################
    #   Private Methods
    #######################

    def _is_obstacle(self, pos):
        x_pos , y_pos = int(pos[0]), int(pos[1])

        if self._map[(x_pos, y_pos)] !=254:
            return True
        else:
            return False

    def _plot_tree(self):

        fig = plt.figure()
        ax1 = fig.add_subplot(1,2,1)
        ax1.imshow(self._map, cmap="gray")
        ax1.set_title("RRT Search")
        ax1.scatter([self._start_idx[1]],[self._start_idx[0]])
        ax1.scatter([self._goal_idx[1]],[self._goal_idx[0]])

        def _recurse_tree(node):
            node_pos = node.get_position()
            x_pos, y_pos = node_pos[1], node_pos[0]

            children = node.get_children()

            for child in children:
                child_pos = child.get_position()
                x_child, y_child = child_pos[1], child_pos[0]
                ax1.plot([x_pos,x_child],[y_pos,y_child],'k-')
                _recurse_tree(child)

        _recurse_tree(self._tree.get_root_node())

        plt.show()

    def _compute_shortest_path(self):
        def _recurse_tree(node):
            parent = node.get_parent()
            if parent:
                total_path.append(parent)
                _recurse_tree(parent)

        total_path = [] 

        if self._goal_node:
            total_path.append(self._goal_node)
            _recurse_tree(self._goal_node)

        total_path_pos = [node.get_position() for node in total_path]
        return total_path, total_path_pos

    #######################
    #   Public Methods
    #######################

    def plot_path(self, smooth_path=[], smooth_path_idx=[]):

        fig = plt.figure()
        ax1 = fig.add_subplot(1,3,1)
        ax1.imshow(self._map, cmap="gray")
        ax1.set_title("RRT Search")
        ax1.scatter([self._start_idx[1]],[self._start_idx[0]])
        ax1.scatter([self._goal_idx[1]],[self._goal_idx[0]])

        def _recurse_tree(node):
            node_pos = node.get_position()
            x_pos, y_pos = node_pos[1], node_pos[0]

            children = node.get_children()

            for child in children:
                child_pos = child.get_position()
                x_child, y_child = child_pos[1], child_pos[0]
                ax1.plot([x_pos,x_child],[y_pos,y_child],'g-')
                _recurse_tree(child)

        _recurse_tree(self._tree.get_root_node())

        _, total_path_pos = self._compute_shortest_path()
        
        x_smooth = [data[0] for data in total_path_pos] 
        y_smooth = [data[1] for data in total_path_pos] 

        # fig = plt.figure()
        ax2 = fig.add_subplot(1,3,2)
        ax2.plot(y_smooth, x_smooth)
        ax2.set_title("Grid Map")
        ax2.imshow(self._original_map, cmap='gray')

        x_smooth = [data[0] for data in smooth_path_idx] 
        y_smooth = [data[1] for data in smooth_path_idx] 
        
        ax3 = fig.add_subplot(1,3,3)
        ax3.set_title("Inflated Grid Map")
        ax3.plot(y_smooth, x_smooth)
        ax3.imshow(self._map, cmap='gray')
        plt.show()

    def run(self):
        self._is_goal = False 
        ctr = 0
        while not self._is_goal:

            if ctr > self._num_nodes:
                break
            ctr += 1

            x_rand = random.uniform(0, self._map_size[1])
            y_rand = random.uniform(0, self._map_size[0])

            min_node, min_val = self._tree.get_nearest_node((x_rand, y_rand))

            min_x, min_y = min_node.get_position()

            delta_x = x_rand - min_x
            delta_y = y_rand - min_y
            dist = sqrt((delta_x) **2 + (delta_y)**2)

            x_new = delta_x*(self._step_size/dist)
            y_new = delta_y*(self._step_size/dist)
            new_pos = (x_new + min_x, y_new + min_y)

            if self._is_obstacle(new_pos):
                continue 
            else: 
                candidate_node = TreeNode(min_node, position=new_pos)

                bres_points = bresenham(min_node.get_position(), candidate_node.get_position())
                self.connect_through_obstacle = False
                for point in bres_points:

                    if self._is_obstacle(point):
                        self.connect_through_obstacle = True
                        break
                
                if not self.connect_through_obstacle:
                    self._tree.add_node(candidate_node, min_node)

            if self._circle_check(self._goal_idx, new_pos, self._config["planner"]["RRT"]["goal_tolerance"]):
                self._goal_node = TreeNode(min_node, position=self._goal_point)
                self._tree.add_node(self._goal_node, min_node)
                break

        _, total_path_pos = self._compute_shortest_path()
        return total_path_pos, total_path_pos

if __name__ == "__main__":
    pass