"""File: dijkstra.py

Description: This file implement Dijkstra's Shortest Path algorithm. Since this algorithm uses the exact 
same graph generate to solve the A* algorthm, this class will instantiate the same graph generator object
as the A* path planner algorithm.
"""

from algorithms.algo_base import AlgorithmBase
from generators.graph_generator import GraphGenerator
import math
import matplotlib.pyplot as plt


class DijkstaShortestPath(AlgorithmBase):

    def __init__(self, config, graph=None):
        super().__init__(config)

        if graph is None: 
            self._graph_generator = GraphGenerator(config)
            self._graph = self._graph_generator.build_graph()
        else:
            self._graph = graph
        
        self._open_set = set()
        self._dist = dict()
        self._prev = dict() 

    #######################
    #   Private Methods
    #######################

    def _initialize_open_set(self):
        """Initialize the dist and prev dicts & add every node to the open set."""

        for node in self._graph.graph_dict().keys():
            self._dist[node] = math.inf
            self._prev[node] = None
            self._open_set.add(node)

    def _get_minimum_dist(self):
        """Parses ever node in the open set to determines the node with the minimum distance."""

        min_val = math.inf
        min_node = None
         
        for node in self._open_set:

            dist = self._dist[node]

            if dist < min_val:
                min_val = dist
                min_node = node

        return min_val, min_node

    def _construct_shortest_path(self, goal_node, init_node):
        """Provided with goal and start nodes, this method constructs the shortest path, by walking
        the _prev dict backwards from the goal node to the start. NOTE: This method only applies 
        at the termination of the algorithm.
        """
        total_path = [goal_node]
        total_path_idx = [goal_node.get_index()] 

        current = goal_node

        while current != init_node:

            prev = self._prev[current]

            total_path.append(prev)
            total_path_idx.append(prev._index)

            current= prev

        # Reverse Order: start node -> goal node
        total_path.reverse()
        total_path_idx.reverse()
        total_path_coords = self.convert_output_to_coords(total_path_idx)

        return total_path_coords, total_path_idx

    #######################
    #   Public Methods
    #######################

    def plot_path(self, smooth_path, smooth_path_idx):

        x_smooth = [data[0] for data in smooth_path_idx] 
        y_smooth = [data[1] for data in smooth_path_idx] 

        fig = plt.figure()
        ax1 = fig.add_subplot(1,2,1)
        ax1.plot(y_smooth, x_smooth)
        ax1.set_title("Inflated Grid Map")
        ax1.imshow(self._graph_generator._pgm_map, cmap='gray')
        
        ax2 = fig.add_subplot(1,2,2)
        ax2.set_title("Grid Map")
        ax2.plot(y_smooth, x_smooth)
        ax2.imshow(self._graph_generator._original_map, cmap='gray')
        plt.show()
    
    def run(self):
        """Apply Dijkstra's Shortest path Algorithm."""
        
        # Initialize Start and Goal Nodes from Map pixel indices
        init_node = self._graph.get_node_from_index(self._start_idx)
        goal_node = self._graph.get_node_from_index(self._goal_idx)

        # Populate Open Set 
        self._initialize_open_set()
        
        # Distance to start node is 0 be definition
        self._dist[init_node] = 0

        while self._open_set:
            min_val, min_node = self._get_minimum_dist()
            self._open_set.remove(min_node)

            if min_node == goal_node:
                return self._construct_shortest_path(goal_node, init_node)

            for neighbor in self._graph.get_neighbors(min_node):

                if neighbor in self._open_set:

                    alt = self._dist[min_node] + self._graph.get_edge(min_node, neighbor) 

                    if alt < self._dist[neighbor]:
                        self._dist[neighbor] = alt
                        self._prev[neighbor] = min_node
                else:
                    continue

if __name__ == "__main__":
    pass