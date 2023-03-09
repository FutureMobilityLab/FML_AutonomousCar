"""File: graph.py

Description: Graph class represents a mathematical graph with nodes and edges, and include all of the requisite, 
methods to add nodes, or query the graph about node neighbors...etc. 
"""

from datastructures.edge import WeightedEdge as Edge

class Graph:

    def __init__(self):
        self._graph_dict = dict() 
        self._idx_2_node = dict() 

    def add_node(self, node):

        if not self.contains(node):
            self._graph_dict[node] = dict() 

            self._idx_2_node[node._index] = node 

    def add_edge(self, node1, node2, weight):

        if node1 in self._graph_dict.keys() and node2 in self._graph_dict.keys():
            self._graph_dict[node1][node2] = Edge(weight)
            self._graph_dict[node2][node1] = Edge(weight)

        else:
            raise KeyError("Cannot Add an Edge between nodes that do not exist")

    def get_neighbors(self, node):
        node_ls = [] 

        neighbor_dict = self._graph_dict[node]

        for node, edge in neighbor_dict.items():

            node_ls.append(node)

        return node_ls


    def get_edge(self, node1, node2):
        for node , edge in self._graph_dict[node1].items():

            if node == node2:
                return edge.get_weight()

    def graph_dict(self):
        return self._graph_dict


    def get_node_from_index(self, idx):
        return self._idx_2_node[idx]

    def prune_graph(self):

        del_list = [] 

        for node, neighbors in self._graph_dict.items():

            if len(neighbors.keys()) == 0:
                del_list.append(node)

        for node in del_list:
            del self._graph_dict[node]


    def contains(self, node):
        return True if node in self._graph_dict.keys() else False
    

    def print_graph(self):
        for node, adj_dict in self._graph_dict.items():

            name_ls = [node._index for node, edge in adj_dict.items()]
            print(f"Node = {node._index}, Adjency List = {name_ls}")


if __name__ == "__main__":

    g = Graph()
    print("hi")