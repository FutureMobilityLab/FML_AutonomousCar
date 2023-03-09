"""File: tree.py

Description: Implements both Tree and TreeNode classes, which implement a tree datastructure and the nodes
which are used to build a tree respectively. 
"""

from datastructures.node import Node
from math import sqrt, inf
import math
from scipy.spatial import KDTree
import numpy as np


class TreeNode(Node):

    def __init__(self, parent, index=(None, None), position=(None, None), cost=0):
        super().__init__(index, position)

        self._children = [] 
        self._parent = parent
        self._cost = cost

    def get_cost(self):
        return self._cost
    
    def set_cost(self, cost):
        self._cost = cost

    def add_child(self, node):
        self._children.append(node)

    def remove_child(self, node):
        self._children.remove(node)
        
    def get_children(self):
        return self._children

    def get_parent(self):
        return self._parent


class Tree:

    def __init__(self, root_pos=(0, 0)):
        self._root_node = TreeNode(None, position=root_pos)


        self._node_ind_list = [self._root_node.get_position()] 


        






    def _heuristic(self, curr_node, goal_node):
        x_start = curr_node._position[1]
        x_goal = goal_node._position[1]
        y_start = curr_node._position[0]
        y_goal = goal_node._position[0]

        heur_dist = math.sqrt((x_goal - x_start)**2 + (y_goal-y_start)**2)
        return heur_dist

    def add_node(self, new_node, parent):

        def _recurse_tree(node):
            for child in node.get_children():

                if child == parent:
                    incr_cost  = self._heuristic(parent, new_node)

                    total_cost = incr_cost + parent.get_cost()
                    new_node.set_cost(total_cost)
                    parent.add_child(new_node)


                    break
                else:
                    _recurse_tree(child)

        if parent == self._root_node:
            incr_cost  = self._heuristic(parent, new_node)
            total_cost = incr_cost + parent.get_cost()
            new_node.set_cost(total_cost)
            self._root_node.add_child(new_node)
        else:
            _recurse_tree(self._root_node)

    def remove_node(self, node):
        def _recurse_tree(node):
            for child in node.get_children():

                if child == node:
                    parent = node.get_parent()
                    parent.remove_child(node)
                else:
                    _recurse_tree(child)

        if node in self._root_node.get_children():
            self._root_node.remove_child(node)
        else:
            _recurse_tree(self._root_node)
    
    def get_nearest_node(self, pos):


        # Create KDTree for Nearest Neighbor Search
        kdtree = KDTree(data=np.asarray(self._node_ind_list))


        x_pos, y_pos = pos[0], pos[1]





        # Query Nearest Points to Indices
        _, nearest_pt_indices = kdtree.query(pos)


        near_neighbor_mapping = dict()

        

        def _find_near_neighbor(node):
            node_pos = node.get_position()
            x_node, y_node = node_pos[0], node_pos[1]
            dist = sqrt((x_node-x_pos)**2 + (y_node-y_pos)**2)
            near_neighbor_mapping[dist] = node

            for child in node.get_children():
                _find_near_neighbor(child)

        # Find the mapping between the nearest nodes and given position
        _find_near_neighbor(self._root_node)

        min_val = min(near_neighbor_mapping.keys())
        min_node = near_neighbor_mapping[min_val]

        return min_node, min_val

    def print_tree(self):

        def _print_tree(node):

            children = node.get_children()
            print(f"Node: {node}: Pos = {node.get_position()} : Childen = {children}")

            for child in children:
                _print_tree(child)

        _print_tree(self._root_node)

    def get_root_node(self):
        return self._root_node


if __name__ == "__main__":
    tree = Tree()

    root = tree.get_root_node()

    a = TreeNode(tree._root_node, position=(1,1))
    b = TreeNode(tree._root_node, position=(2,2))
    c = TreeNode(a, position=(3,3))
    d = TreeNode(c, position=(4,4))
    e = TreeNode(c, position=(5,5))
    f = TreeNode(d, position=(6,6))
    g = TreeNode(a, position=(7,7))
    h = TreeNode(e, position=(8,8))
    i = TreeNode(b, position=(9,9))
    j = TreeNode(d, position=(10,10))
    k = TreeNode(tree._root_node, position=(11,11))


    tree.add_node(k, root)
    tree.add_node(a, root)
    tree.add_node(b, root)

    tree.add_node(c, a)
    tree.add_node(d, c)
    tree.add_node(e, c)

    tree.add_node(f, d)
    tree.add_node(g, a)
    tree.add_node(h, e)
    tree.add_node(i, b)
    tree.add_node(j, d)


    # tree.remove_node(k)
    # # tree.remove_node(a)
    # tree.remove_node(b)




    tree.print_tree()
    print("####################################")
    print(tree.get_nearest_node((4.45, 4.45)))