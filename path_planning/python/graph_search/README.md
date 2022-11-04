# Graph Search Path Planners
The premise of graph search path planner is to cast the path planning problem into a graph data structure traversal problem. This has the advantage of being a very well studied field of computer science with a variety of well known algorithms to choose from 


## Path Planners
In the context of this project, we will assume that planners are useful in static environments that are already mapped, inside of which our vehicle can be localized. 

### Dijkstra Shortest Path
This seminal algorithm was the first developed by Edgar Dijkstra to determine the shortest distance between two nodes on a graph, via a dynamic programming approach. However, as a uniformly weighted search this algorithm will expand its search to all nodes of a graph equally. This means that this algorithm will expend substatial compute on traversing nodes that are not likely to aid in reaching the goal node. 

### A*
A* is a heuristic search algorithm which weights its search by a heuristic measure. Usually, this heuristic is Euclidean or Metropolitan measures. The advantage this graph has over Dijkstra's is that it will preferentially explore nodes that minimize the distance to the goal node.

### Probablistic Roadmap (PRM)


# Monte Carlo / Sampling Path Planners 

### RRT

### RRT* 

### Informed RRT*