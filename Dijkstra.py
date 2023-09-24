import math
import numpy as np

graph = {

    # 'a': Vertice(),
    'b': {'c': 1, 'f': 5},
    'c': {'f': 6, 'd': 2},
    'd': {'e': 3, 'g': 6},
    'e': {'g': 3, 'h': 4},
    'f': {'e': 1, 'h': 8},
    'g': {'h': 2},
    'h': {}
}


# neighborid = graph['a'].out[0].to
# graph[neighborid].out

"""
data: Read comment in ALTalgorithm.py for ALT().
start and goal: node id in form of integers for array element access.
"""
def dijkstra(graph, start, goal, lat=[], lon=[], data={}):
    shortest_distance = {}  # shortest distance so far
    track_predecessor = {}  # route to the current node so far
    unseenNodes = list(range(0, len(graph)))  # Unseen nodes so far
    infinity = math.inf  # largest number.
    track_path = []  # the tracked path back to start
    iterations = 0

    for node in unseenNodes:
        shortest_distance[node] = infinity

    shortest_distance[start] = 0

    while unseenNodes:
        min_distance_node = None
        iterations += 1
        # Find the closest unseen node relative to the starting point, named min_distance_node
        for node in unseenNodes:
            if min_distance_node is None:
                min_distance_node = node
            elif shortest_distance[node] < shortest_distance[min_distance_node]:
                min_distance_node = node
        # Check if we have reached the goal
        if min_distance_node is goal:
            break
        path_options = []
        # Look at all the possible next steps going from min_distance_node and add them to path_options
        neighbors = graph[min_distance_node]
        for i in range(0, len(neighbors), 2):
            path_options.append((neighbors[i], neighbors[i+1]))
        # Compute sum of dist from start to min_distance_node and dist from min_distance_node to neighbor of next step
        # Compare it to the current shortest distance of neighbor and update accordingly
        for neighbor_node, weight in path_options:
            distance = weight + shortest_distance[min_distance_node]
            if distance < shortest_distance[neighbor_node]:
                shortest_distance[neighbor_node] = distance
                # Keep track of the shortest steps so far
                track_predecessor[neighbor_node] = min_distance_node
        # Remove min_distance_node from unseenNodes, because it has now been seen
        unseenNodes.remove(min_distance_node)

    currentNode = goal
    # Build up the optimal path as a list
    while currentNode != start:

        try:
            track_path.insert(0, currentNode)
            currentNode = track_predecessor[currentNode]
        except KeyError:
            print('Path not reachable')
            break
    track_path.insert(0, start)

    goal_ = shortest_distance[goal]
    print('Number of iterations: ', iterations)
    print('Total nodes scanned by Dijkstra: ', len(graph) - len(unseenNodes))
    if goal_ != infinity:
        print('Shortest distance is ' + str(shortest_distance[goal]))
        print('And the path is ' + str(track_path))
    return track_path


if __name__ == "__main__":
    dijkstra(graph, 'a', 'g')
