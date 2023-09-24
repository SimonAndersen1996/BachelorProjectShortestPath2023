import heapq
import math
from Python.ALTalgorithm import invert_graph

graph = [[3, 4, 1, 3], [2, 3], [], [4, 0], [2, 1]]
def bidirectional_dijkstra(graph, start, goal):
    # Initialize the search frontiers from both ends
    shortest_distance = {node: math.inf for node in range(len(graph))}
    shortest_distance[start] = 0
    forward_heap = [(0, start)]  # a heap of (distance, node) pairs for forward search
    forward_pred = {}  # predecessor nodes for forward search
    backward_distance = {node: math.inf for node in range(len(graph))}
    backward_distance[goal] = 0
    backward_heap = [(0, goal)]  # a heap of (distance, node) pairs for backward search
    backward_pred = {}  # predecessor nodes for backward search
    meeting_node = None
    inverted = invert_graph(graph)
    visited = set()

    # Initialize the minimum distance to reach the goal
    min_distance = math.inf
    iterations = 0
    # Search until the two frontiers meet in the middle
    while forward_heap and backward_heap:
        # Perform one step of the forward search
        forward_distance, forward_node = heapq.heappop(forward_heap)
        iterations += 1
        visited.add(forward_node)
        # If we reach a node that has already been explored in the backward search,
        # we have found the shortest path
        if forward_node in backward_pred:
            meeting_node = forward_node
            distance = forward_distance + backward_distance[forward_node]
            if distance < min_distance:
                min_distance = distance
            break

        neighbors = graph[forward_node]
        for i in range(0, len(neighbors), 2):
            neighbor_node = neighbors[i]
            weight = neighbors[i+1]
            distance_to_neighbor = forward_distance + weight
            if distance_to_neighbor < shortest_distance[neighbor_node]:
                shortest_distance[neighbor_node] = distance_to_neighbor
                forward_pred[neighbor_node] = forward_node
                heapq.heappush(forward_heap, (distance_to_neighbor, neighbor_node))

        # Perform one step of the backward search
        back_dist, backward_node = heapq.heappop(backward_heap)
        visited.add(backward_node)

        # If we reach a node that has already been explored in the forward search,
        # we have found the shortest path
        if backward_node in forward_pred:
            meeting_node = backward_node
            distance = back_dist + shortest_distance[backward_node]
            if distance < min_distance:
                min_distance = distance
            break

        neighbors = inverted[backward_node]
        for i in range(0, len(neighbors), 2):
            neighbor_node = neighbors[i]
            weight = neighbors[i+1]
            distance_to_neighbor = back_dist + weight
            if distance_to_neighbor < backward_distance[neighbor_node]:
                backward_distance[neighbor_node] = distance_to_neighbor
                backward_pred[neighbor_node] = backward_node
                heapq.heappush(backward_heap, (distance_to_neighbor, neighbor_node))
    # Construct the shortest path
    forward_path = []
    current_node = forward_pred[meeting_node]
    while current_node != start:
        forward_path.insert(0, current_node)
        current_node = forward_pred[current_node]
    forward_path.insert(0, start)

    backward_path = []
    current_node = meeting_node
    while current_node != goal:
        backward_path.append(current_node)
        current_node = backward_pred[current_node]
    backward_path.append(goal)
    shortest_distance[goal] = shortest_distance[meeting_node] + backward_distance[meeting_node]
    shortest_path = forward_path
    shortest_path.extend(backward_path)
    print('Number of iterations: ', iterations)
    print('Total nodes scanned by Bidirectional Dijkstra: ', len(visited))
    print('Shortest distance is ' + str(shortest_distance[goal]))
    print('And the path is ' + str(shortest_path))
    return min_distance, shortest_path

def new_bidirectional_dijkstra(graph, start, goal):
    # Initialize the search frontiers from both ends
    shortest_distance = {node: math.inf for node in range(len(graph))}
    shortest_distance[start] = 0
    forward_heap = [(0, start)]  # a heap of (distance, node) pairs for forward search
    forward_pred = {}  # predecessor nodes for forward search
    backward_distance = {node: math.inf for node in range(len(graph))}
    backward_distance[goal] = 0
    backward_heap = [(0, goal)]  # a heap of (distance, node) pairs for backward search
    backward_pred = {}  # predecessor nodes for backward search
    meeting_node = None
    inverted = invert_graph(graph)
    visited = set()

    # Initialize the minimum distance to reach the goal
    min_distance = math.inf
    iterations = 0
    # Search until all nodes have been explored or the minimum distance has been updated
    while forward_heap and backward_heap:
        # Perform one step of the forward search
        forward_distance, forward_node = heapq.heappop(forward_heap)
        iterations += 1
        visited.add(forward_node)

        # If we reach a node that has already been explored in the backward search,
        # we have found a potentially shorter path
        if forward_node in backward_pred:
            meeting_node = forward_node
            distance = forward_distance + backward_distance[forward_node]
            if distance < min_distance:
                min_distance = distance
                shortest_distance[goal] = distance
                shortest_path = []
                current_node = forward_pred[meeting_node]
                while current_node != start:
                    shortest_path.insert(0, current_node)
                    current_node = forward_pred[current_node]
                shortest_path.insert(0, start)
                current_node = meeting_node
                while current_node != goal:
                    shortest_path.append(current_node)
                    current_node = backward_pred[current_node]
                shortest_path.append(goal)

        neighbors = graph[forward_node]
        for i in range(0, len(neighbors), 2):
            neighbor_node = neighbors[i]
            weight = neighbors[i + 1]
            distance_to_neighbor = forward_distance + weight
            if distance_to_neighbor < shortest_distance[neighbor_node]:
                shortest_distance[neighbor_node] = distance_to_neighbor
                forward_pred[neighbor_node] = forward_node
                heapq.heappush(forward_heap, (distance_to_neighbor, neighbor_node))

        # Perform one step of the backward search
        back_dist, backward_node = heapq.heappop(backward_heap)
        visited.add(backward_node)

        # If we reach a node that has already been explored in the forward search,
        # we have found a potentially shorter path
        if backward_node in forward_pred:
            meeting_node = backward_node
            distance = back_dist + shortest_distance[backward_node]
            if distance < min_distance:
                min_distance = distance
                shortest_distance[goal] = distance
                shortest_path = []
                current_node = forward_pred[meeting_node]
                while current_node != start:
                    shortest_path.insert(0, current_node)
                    current_node = forward_pred[current_node]
                shortest_path.insert(0, start)
                current_node = meeting_node
                while current_node != goal:
                    shortest_path.append(current_node)
                    current_node = backward_pred[current_node]
                shortest_path.append(goal)

        neighbors = inverted[backward_node]
        for i in range(0, len(neighbors), 2):
            neighbor_node = neighbors[i]
            weight = neighbors[i + 1]
            distance_to_neighbor = back_dist + weight
            if distance_to_neighbor < backward_distance[neighbor_node]:
                backward_distance[neighbor_node] = distance_to_neighbor
                backward_pred[neighbor_node] = backward_node
                heapq.heappush(backward_heap, (distance_to_neighbor, neighbor_node))

    # Output the result
    if shortest_path is not None:
        print('Number of iterations:', iterations)
        print('Total nodes scanned by Bidirectional Dijkstra:', len(visited))
        print('Shortest distance is', min_distance)
        print('And the path is', shortest_path)
    else:
        print('No path found.')
    return shortest_path

