"""File: a_star.py

Description: This file implement the A* path planning algorithm. Note that this class does not create the 
graph that A* will search, but instead instantiates a GraphGenerator object which ingests the probability 
grid map, applies an inflation operation, and generates the graph data structure which our A* algorithm can 
traverse. For details on graph generation please see 'graph_generator_base.py'.
"""

from algorithms.algo_base import AlgorithmBase
from generators.graph_generator import GraphGenerator
import math
import matplotlib.pyplot as plt


class AStar(AlgorithmBase):

    def __init__(self, config):
        super().__init__(config)

        self._graph_generator = GraphGenerator(config)
        self._graph = self._graph_generator.build_graph()
        
        self._open_set = set()
        self._closed_set = set()
        self._prev_node = dict()

        self._f_score = {node: math.inf for node in self._graph.graph_dict().keys()}
        self._g_score = {node: math.inf for node in self._graph.graph_dict().keys()}

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

        if self._config["planner"]["Astar"]["heuristic"] == "manhattan":
            heur_dist = abs(x_goal - x_start) + abs(y_goal-y_start)
        else:
            heur_dist = math.sqrt((x_goal - x_start)**2 + (y_goal-y_start)**2)
        return heur_dist * discount

    def _construct_shortest_path(self, current):
        """Provided with goal and start nodes, this method constructs the shortest path, by walking
        the _prev dict backwards from the goal node to the start. NOTE: This method only applies 
        at the termination of the algorithm.
        """
        total_path = [current]
        total_path_idx = [current.get_index()] 

        while current in self._prev_node.keys():
                prev = self._prev_node[current]
                total_path.append(prev)
                total_path_idx.append(prev._index)
                current = prev

        # Reverse Order: start node -> goal node
        total_path.reverse()
        total_path_idx.reverse()
        total_path_coords = self.convert_output_to_coords(total_path_idx)

        return total_path_coords, total_path_idx

    def _get_lowest_f_score_in_open_set(self):
        """Since we had trouble getting A* via Priority Queue to work properly, this method 
        manually computes the node with the lowest 'f_score' in the open set.
        """
        min_val = math.inf
        min_node = None
         
        for node in self._open_set:

            f_score = self._f_score[node]

            if f_score < min_val:
                min_val = f_score
                min_node = node

        return min_val, min_node

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
        """Apply A* Heuristic Search Algorithm."""

        # Initialize Start and Goal Nodes
        start_node = self._graph.get_node_from_index(self._start_idx)
        end_node = self._graph.get_node_from_index(self._goal_idx)

        # Initialize Node & Heuristic Costs
        self._f_score[start_node] = 0
        self._g_score[start_node] = 0

        # Initialize Open Set
        self._open_set.add(start_node)

        while self._open_set:
            f_score, current = self._get_lowest_f_score_in_open_set()

            if current == end_node:
                return self._construct_shortest_path(end_node)

            self._open_set.remove(current)
            self._closed_set.add(current)

            for neighbor in self._graph.get_neighbors(current):
                
                if neighbor in self._closed_set:
                    continue

                tentative_g_score = self._g_score[current] + self._graph.get_edge(current, neighbor) 

                if neighbor not in self._open_set:
                    self._open_set.add(neighbor)

                elif tentative_g_score >= self._g_score[neighbor]:
                    continue

                self._g_score[neighbor] = tentative_g_score
                self._f_score[neighbor] = self._g_score[neighbor] + self._heuristic(neighbor, end_node)
                self._prev_node[neighbor] = current                                                               

        return False

if __name__ == "__main__":
    pass
