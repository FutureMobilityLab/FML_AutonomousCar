"""File: prm.py

Description: This file implement the Probablistic Road Map (PRM) algorithm. This algorithm 
is similar to RRT's in so much as it is based on random sampling of the configuration space,
which is used to generate a graph of the free space. Once this graph is constructed Dijkstra's
algorithm is used to determine the shortest path from starting to goal positions.  

NOTE: Similar to the RRTs we cannot prebuild the graph for this algorithm as the construction of
the graph occurs during the runtime of the algorithm.
"""

import numpy as np 
from copy import deepcopy
import math


import matplotlib.pyplot as plt

from algorithms.algo_base import AlgorithmBase
from algorithms.dijkstra import DijkstaShortestPath
from datastructures.graph import Graph
from datastructures.node import Node
from post_processing.bresenham import bresenham

from scipy.spatial import KDTree

class PRM(AlgorithmBase):      
        
    def __init__(self, config):  
        super().__init__(config)

        self._graph = Graph()
    
        self._configuration_samples = None
        self._node_indices = [] 

        self._dijkstra_solver = None
        self._kdtree = None
        
        self._map_path, self._map_conf_path = self._extract_map_path(self._config["map"]["path"])
        self._map_conf = self._parse_config(self._map_conf_path)
        self._map = self._parse_map(self._map_path) 
        self._map_size = self._map.shape

        self._inflation_radius = self._config["map"]["inflation_radius"] # in Pixels
        self._original_map = deepcopy(self._map)

        # Apply Obstacle Inflation
        if self._config["map"]["inflate"]:            
            self._map = self._inflate_map_obstacle(self._inflation_radius, self._map)

        self._num_nodes = self._config["planner"]["PRM"]["num_samples"]

    #######################
    #   Private Methods
    #######################

    def _heuristic(self, curr_node, goal_node, discount=.95):
        """Method computes the heuristic that A* will use to attempt to improve 
        convergence to the desired solution. Typically either Euclidian or Manhattan distances,
        between the current and the goal nodes.
        """
        x_start = curr_node._index[1]
        x_goal = goal_node._index[1]
        y_start = curr_node._index[0]
        y_goal = goal_node._index[0]
        heur_dist = math.sqrt((x_goal - x_start)**2 + (y_goal-y_start)**2)
        return heur_dist * discount

    def _sample_configuration_space(self):
        j_rand = np.random.uniform(0, self._map_size[1], size=(1, self._num_nodes))[0]
        i_rand = np.random.uniform(0, self._map_size[0], size=(1, self._num_nodes))[0]
        self._configuration_samples = [(i_rand[k], j_rand[k]) for k in range(self._num_nodes)] 

    def _build_graph(self):
        self._start_node = Node(index=self._start_idx)
        self._goal_node = Node(index=self._goal_idx)

        self._graph.add_node(self._start_node)
        self._graph.add_node(self._goal_node)

        # Node Creation 
        for rand_configuration in self._configuration_samples:

            # Check if configuration is an obstacle
            if self._is_obstacle(rand_configuration):
                continue
            else:
                # Create a new Node at free space random configuration
                new_node = Node(index=rand_configuration)
                self._graph.add_node(new_node)
                self._node_indices.append(rand_configuration)

        # Create KDTree for Nearest Neighbor Search
        self._kdtree = KDTree(data=np.asarray(self._node_indices))

        # Create Edges Between Neighboring Nodes
        for node in self._graph.graph_dict().keys():
            index = np.asarray(node.get_index())

            # Query Nearest Points to Indices
            _, nearest_pt_indices = self._kdtree.query(
                index , k=self._config["planner"]["PRM"]["num_neighbors"])
            
            nearest_node_list = [self._graph.get_node_from_index(self._node_indices[index]) for index in nearest_pt_indices]

             # For each nearest point, check if the current node is already connected on the graph else connect it
            for nearest_node in nearest_node_list:

                if nearest_node in self._graph.get_neighbors(node):
                    continue

                else:

                    bres_points = bresenham(node.get_index(), nearest_node.get_index())
                    self.connect_through_obstacle = False
                    for point in bres_points:

                        if self._is_obstacle(point):
                            self.connect_through_obstacle = True
                            break

                    if not self.connect_through_obstacle:

                        self._graph.add_edge(node, nearest_node, weight=self._heuristic(node, nearest_node))

    def _compute_shortest_path(self):
        self._dijkstra_solver = DijkstaShortestPath(self._config, self._graph)
        total_path, total_path_idx = self._dijkstra_solver.run()
        return total_path, total_path_idx

    def _is_obstacle(self, pos):
        x_pos , y_pos = int(pos[0]), int(pos[1])

        if self._map[(x_pos, y_pos)] !=254:
            return True
        else:
            return False

    #######################
    #   Public Methods
    #######################

    def plot_roadmap(self):

        fig = plt.figure()
        ax1 = fig.add_subplot(1,2,1)
        ax1.imshow(self._map)

        for node in self._graph.graph_dict().keys():
            node_idx = node.get_index()

            neighbors = self._graph.get_neighbors(node)
            neighbor_idx = [node.get_index() for node in neighbors]


            for n_index in neighbor_idx:
                ax1.plot([node_idx[1], n_index[1]],[node_idx[0] ,n_index[0]],'k-')

        ax1.scatter([self._start_idx[1]],[self._start_idx[0]])
        ax1.scatter([self._goal_idx[1]],[self._goal_idx[0]])

        current = self._goal_idx

        for node_idx in self._output:
            ax1.plot([current[1], node_idx[1]],[current[0] ,node_idx[0]],'r-')
            current = node_idx

        plt.show()

    def plot_path(self, smooth_path=[], smooth_path_idx=[]):

        fig = plt.figure()
        fig.suptitle("Probablistic Road Maps")
        ax1 = fig.add_subplot(1,3,1)
        ax1.imshow(self._map, cmap="gray")
        ax1.set_title("PRM Search")

        for node in self._graph.graph_dict().keys():
            node_idx = node.get_index()

            neighbors = self._graph.get_neighbors(node)
            neighbor_idx = [node.get_index() for node in neighbors]


            for n_index in neighbor_idx:
                ax1.plot([node_idx[1], n_index[1]],[node_idx[0] ,n_index[0]],'g-')

        ax2 = fig.add_subplot(1,3,2)
        ax2.set_title("Grid Map")
        ax2.imshow(self._original_map, cmap='gray')

        ax3 = fig.add_subplot(1,3,3)

        current = self._start_idx

        for node_idx in self._output:

            ax2.plot([current[1], node_idx[1]],[current[0] ,node_idx[0]],'r-')
            ax3.plot([current[1], node_idx[1]],[current[0] ,node_idx[0]],'r-')
            current = node_idx
            if current == self._start_node:
                break

        x_smooth = [data[0] for data in smooth_path_idx] 
        y_smooth = [data[1] for data in smooth_path_idx] 

        ax2.plot(y_smooth, x_smooth)
        
        
        ax3.set_title("Inflated Grid Map")
        ax3.plot(y_smooth, x_smooth)
        ax3.imshow(self._map, cmap='gray')
        plt.show()

    def run(self):
        self._sample_configuration_space()
        self._build_graph()
        total_path, total_path_idx = self._compute_shortest_path()
        self._output = total_path_idx
        return total_path, total_path_idx

if __name__ == "__main__":
    pass