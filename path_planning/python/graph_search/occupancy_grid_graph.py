import numpy as np 
import math
from queue import PriorityQueue


class NodeBase:
    def __init__(self, value=None, name=None) -> None:
        self._value = value
        self._name = name

    def get_name(self):
        return self._name

    def get_value(self):
        return self._value
    
    def set_value(self, value):
        self._value = value

class Node(NodeBase):
    def __init__(self, value=None, name=None, grid_pos=None) -> None:
        super().__init__(value, name)

        self._f = None 
        self._g = math.inf 
        self._h = math.inf 
        self._grid_coords = grid_pos

    def __repr__(self) -> str:
        
        return f"Node {self._grid_coords}, Value = {self._value}"

    @property
    def f(self):
        return self._f
    
    @property
    def g(self):
        return self._g

    @property
    def h(self):
        return self._h

    @property
    def coords(self):
        return self._grid_coords

class Graph:
    def __init__(self) -> None:
        self._graph = dict()

    def addNode(self, node: Node):
        if not self._graph.get(node, None):
            self._graph[node] = set()
        else:
            # Node Already Exists
            pass

    def removeNode(self, node: Node):
        # 1. Get the Current neighbors of the node
        # 2. Remove the Node from the Graph Dict
        # 3. Remove every refernce of the node in the neighbors lists
        neighbors = self._graph[node] # Tuple (Node, Edge)
        if neighbors:
            for n_node, edge in neighbors:
                self._graph[n_node].discard((node, edge))
                # self.removeEdge(node, n_node)
            del self._graph[node]

    def addEdge(self, weight, node_x: Node, node_y: Node):
        # 1. Check Nodes exists 
        # 2. Create Edge Object with correct edge weight
        # 3. Add Nodes as neighbors

        if node_x in self._graph.keys() and node_y in self._graph.keys():
            x_neighbors = self._graph[node_x]
            y_neighbors = self._graph[node_y]

            x_neighbors.add((node_y, weight))
            y_neighbors.add((node_x, weight))
        else:
            raise ValueError("Either Node X or Node Y has not been inserted into the Graph")

    def removeEdge(self, node_x: Node, node_y: Node):
        # 1. Check Nodes exists 
        # 2. Create Edge Object with correct edge weight
        # 3. Remove Nodes as neighbors
        if node_x in self._graph.keys() and node_y in self._graph.keys():

            x_neighbors = self._graph[node_x]
            y_neighbors = self._graph[node_y]

            for neighbors in [x_neighbors, y_neighbors]:
                for node, edge in neighbors:

                    if node == node_y or node == node_x:
                        neighbors.discard((node, edge))
                        break
        else:
            raise ValueError("Either Node X or Node Y has not been inserted into the Graph") 

    def getEdgeValue(self, node_x: Node, node_y: Node):
        neighbors = self._graph[node_x]
        for node, weight in neighbors:
            if node == node_y:
                return weight

    def getNodeValue(self, node: Node):
        return node.get_value()

    def setNodeValue(self, node: Node, val):
        node.set_value(val)

    def adjacent(self, node_x: Node, node_y: Node) -> bool:
        x_neighbors = self._graph[node_x]
        y_neighbors = self._graph[node_y]
        
        for neighbors in [x_neighbors, y_neighbors]:
            for node, edge in neighbors:
                if node == node_y or node == node_x:
                    return True
        return False

    def neighbors(self, node: Node):
        return self._graph[node]

class PlannerGraph(Graph):
    """Adjacency List Implementation of a Graph generated from a uniform occupency grid."""
    def __init__(self, grid, start_coords, goal_coords) -> None:
        super().__init__()

        self._grid = grid
        self._start_coords = start_coords
        self._goal_coords = goal_coords
        self._start_node = None 
        self._goal_node = None
        self._max_rows = grid.shape[0]
        self._max_cols = grid.shape[1]

        self._build_graph()

    def _get_node_by_coords(self, node_coords):
        for node, _ in self._graph.items():
            if node._grid_coords == node_coords:
                return node

    def _build_graph(self, def_node_val=math.inf, def_edge_val=1):
        """
        Grids are indexed as standard Matrices
        """
        def _add_nodes():
            for idx in np.ndenumerate(self._grid):
                (row, col), value = idx

                if not value:
                    node = Node(value=def_node_val, grid_pos=(row, col))                            # Create the Node 
                    self.addNode(node)                                                          # Add the Node to the Graph 

                    if (row, col) == self._start_coords:
                        self._start_node = node
                    elif (row, col) == self._goal_coords:
                        self._goal_node = node

        def _add_edges():

            def _get_connected_four_coords(coords):
                row, col = coords
                m_row = p_row = m_col = p_col = None

                if not value:

                    m_row = row
                    p_row = row
                    m_col = col
                    p_col = col

                    m_row -=1 
                    p_row +=1 
                    m_col -=1 
                    p_col +=1

                    if m_row < 0:
                        m_row += 1
                    if p_row >= self._max_rows-1:
                        p_row -= 1 
                    if m_col < 0:
                        m_col += 1
                    if p_col > self._max_cols-1:
                        p_col -= 1

                    connected_coords = [(m_row, col), (p_row, col), (row, m_col), (row, p_col)]

                    coords_4 = []
                    for pos in connected_coords:

                        if pos != coords:
                            coords_4.append(pos)
                return coords_4

            def _get_connected_four_nodes(coords):
                con_4_coord_list = _get_connected_four_coords(coords)
                node_list = [self._get_node_by_coords(cords) for cords in con_4_coord_list ]
                return node_list
    

            for idx in np.ndenumerate(self._grid):
                (row, col), value = idx

                if not value:
                    node = self._get_node_by_coords((row, col))
                    connected_nodes = _get_connected_four_nodes((row, col))

                    for c_node in connected_nodes:
                        try: 
                            self.addEdge(def_edge_val, node, c_node)
                        except:
                            pass

        _add_nodes()
        _add_edges()
