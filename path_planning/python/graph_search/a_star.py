import numpy as np 
import math 
from occupancy_grid_graph import PlannerGraph
from queue import PriorityQueue

class AStar:
    def __init__(self, grid, start_coords, goal_coords) -> None:
        
        self._max_rows = grid.shape[0]
        self._max_cols = grid.shape[1]
        self._start_coords = start_coords
        self._goal_coords = goal_coords
        self._open_set = PriorityQueue()
        self._close_set = set()
        self._path = [] 
        self._came_from = dict()
        self._graph = PlannerGraph(grid, start_coords=start_coords, goal_coords=goal_coords)

        self._start = self._graph._start_node
        self._goal = self._graph._goal_node

        self._is_valid()

    def reconstruct_path(self, current=None):

        path = []
        if current is None: 
            current = self._start
            path.append(current)

        for current in self._came_from.keys():
            current = self._came_from[current]
            path.append(current)

        return path

        

    def _is_valid(self):
        start_row = self._start_coords[0]
        start_col = self._start_coords[1]

        goal_row = self._goal_coords[0]
        goal_col = self._goal_coords[1]

        if start_row > self._max_rows or goal_row > self._max_rows:
            raise ValueError("Number of Rows Violate Grid Size")

        if start_col > self._max_cols or goal_col > self._max_cols:
            raise ValueError("Number of Columns Violate Grid Size")

    def _compute_heuristic(self, cur_pos, goal_pos):
        cur_x, cur_y = cur_pos
        goal_x, goal_y = goal_pos
        return math.sqrt((goal_x - cur_x)**2 + (goal_y - cur_y)**2)*.9

    def run(self):

        self._open_set.put((0, id(self._start), self._start))
        self._start._f = 0
        self._start._g = 0

        while not self._open_set.empty():
            _, _, current = self._open_set.get()
            if current == self._goal:
                return 
            for neighbor, _ in self._graph.neighbors(current):
                d = self._graph.getEdgeValue(current, neighbor)
                tent_g = current._g + d

                if tent_g < neighbor._g:
                    self._came_from[neighbor] = current
                    print(f"Current {current} -> neightbor {neighbor}")
                    neighbor._g = tent_g
                    neighbor._f = tent_g + self._compute_heuristic(current._grid_coords, self._goal_coords)                       
                    if neighbor not in self._open_set.queue:
                        self._open_set.put((neighbor._f, id(neighbor), neighbor))


if __name__ == "__main__":

    size = (10,10)
    grid = np.zeros(shape=size)
    grid[0][2] = 1
    grid[1][2] = 1
    grid[2][2] = 1
    grid[3][2] = 1
    grid[4][2] = 1
    grid[5][2] = 1
    grid[6][2] = 1



    grid[3][6] = 1
    grid[4][6] = 1
    grid[5][6] = 1
    grid[6][6] = 1
    grid[7][6] = 1
    grid[8][6] = 1
    grid[9][6] = 1


    print(grid)
    print()
    a_s = AStar(grid, start_coords=(0,0), goal_coords=(9,9))
    
    a_s.run()
    path = a_s.reconstruct_path()
    path = [node._grid_coords for node in path]
    print(path)


    viz_grid = np.zeros(shape=size)

    shortest_path = []

    for coords in path:

        ctr = 0

        for c in path:

            if c == coords:
                ctr += 1

        if ctr >1:
            shortest_path.append(coords)




    for idx in path:
        row, col = idx

        viz_grid[row][col] = 5

    print(viz_grid)

    print()

    print(set(shortest_path))
