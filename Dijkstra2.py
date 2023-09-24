import heapq
import math


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

"""
data: Read comment in ALTalgorithm.py for ALT().
"""
def dijkstra2(graph, start, goal, lat=[], lon=[], data={}):
    shortest_distance = {node: math.inf for node in range(len(graph))}
    shortest_distance[start] = 0
    track_predecessor = {}
    track_path = []
    visited = set()
    heap = [(0., start)]  # a heap of (distance, node) pairs

    iterations = 0
    while heap:
        (distance, current_node) = heapq.heappop(heap)
        iterations += 1
        if current_node in visited:
            continue

        visited.add(current_node)

        # Ignore nodes that have already been visited
        if distance > shortest_distance[current_node]:
            continue
        # Check if the current_node is the goal
        if current_node == goal:
            break
        neighbors = graph[current_node]
        for i in range(0, len(neighbors), 2):
            distance_to_neighbor = distance + neighbors[i+1] # Total dist from start to neighbor
            neighbor = neighbors[i]
            if distance_to_neighbor < shortest_distance[neighbor]: # If a better path is found
                shortest_distance[neighbor] = distance_to_neighbor # Update the distance
                track_predecessor[neighbor] = current_node # Have neighbor remember the previous step
                heapq.heappush(heap, (distance_to_neighbor, neighbor))

    current_node = goal

    while current_node != start:
        try:
            track_path.insert(0, current_node)
            current_node = track_predecessor[current_node]
        except KeyError:
            raise ValueError('Path not reachable')

    track_path.insert(0, start)
    print('Number of iterations: ', iterations)
    print('Total nodes scanned by Dijkstra2: ', len(visited))
    print('Shortest distance is ' + str(shortest_distance[goal]))
    print('And the path is ' + str(track_path))
    return track_path


if __name__ == "__main__":
    dijkstra2(graph, 'a', 'g')
