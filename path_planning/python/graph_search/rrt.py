import numpy as np 
import math
import matplotlib.pyplot as plt
import random

from occupancy_grid_graph import NodeBase


class Node(NodeBase):
    def __init__(self, value=None, name=None, coords=None) -> None:
        super().__init__(value, name)

        self._coords = coords
    #     self._children = [] 

    # def add_child(self, node):
    #     self._children.append(node)

    # def remove_child(self, node):
    #     self._children.remove(node)

    @property
    def coords(self):
        return self._coords
        

class Tree:
    def __init__(self, root_coords, occupancy_grid) -> None:
        self._root_coords = root_coords
        self._root = Node(coords=self._root_coords)
        self._grid = occupancy_grid
        self._adjacency_dict = dict()

        self._adjacency_dict[self._root] = set()

    def get_parent(self, node):
        for n, child_set in self._adjacency_dict.items():
            for child in child_set:
                if child == node:
                    return n

    def get_children(self, node):
        return self._adjacency_dict[node]

    def add_node(self, parent, node):
        child_set = self._adjacency_dict[parent]
        child_set.add(node)

    def remove_node(self, node):
        pass

    def find_closest(self, coords):
        def _find_closest(node, target_coords):
            node_coords = node._coords 
            
        _find_closest(self._root, coords)
    
class RRT:
    def __init__(self, occupancy_grid, start_coords, goal_coords, max_iter=2000, step=.5, tol=.1) -> None:

        self._occupancy_grid = occupancy_grid
        self._start_coords = start_coords
        self._goal_coords = goal_coords
        self._rows = self._occupancy_grid.shape[0]
        self._cols = self._occupancy_grid.shape[1]
        self._max_iter = max_iter
        self._step_size = step
        self._tolerance = tol
        self._tree = Tree(self._start_coords, self._occupancy_grid)

    def _sample(self):
        coords = (random.uniform(0, self._rows), random.uniform(0, self._cols))
        return coords

    def _find_closest_node(self, coords):
        return self._tree.find_closest(coords)

    def _append_new_node(self, parent, coords):

        x_samp, y_samp = coords
        x_par, y_par = parent.coords

        h = math.sqrt((x_samp - x_par)**2 + (y_samp - y_par)**2)

        t = self._step_size / h

        x_node = (1 - t)*x_par + t*x_samp
        y_node = (1 - t)*y_par + t*y_samp

        node_coords = (x_node, y_node)

        if not self._is_occupied(node_coords):
            node = Node(coords=node_coords)
            self._tree.add_node(parent, node)

    def _is_occupied(self, coords):
        row, col = coords
        return True if self._occupancy_grid[row][col] == 1 else False

    def run(self):
        for iter in range(self._max_iter):
            sampled_coords = self._sample()
            nearest_node = self._find_closest_node(sampled_coords)
            self._append_new_node(self._start_coords, sampled_coords)
        return self._tree

    def plot(self):
        tree = self.run()
        pass


if __name__ == "__main__":
    
    size = (10, 10)
    start_coord = (0,0)
    goal_coord = (9,9)

    grid = np.zeros(shape=size)

    rrt = RRT(grid, start_coord, goal_coord)

    rrt.run()


