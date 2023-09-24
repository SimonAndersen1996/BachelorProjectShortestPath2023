import heapq
import math
import time

import numpy as np
import random
import gc


"""
Returns a list of distances from landmark l. Entry i is the distance from landmark l to node i.
"""
def from_landmark_Dijkstra(l, graph):
    shortest_distance = [float('inf') for _ in range(len(graph))]
    shortest_distance[l] = 0
    heap = [(0, l)]

    while heap:
        dist, current_node = heapq.heappop(heap)

        neighbors = graph[current_node]
        for i in range(0, len(neighbors), 2):
            new_dist = shortest_distance[current_node] + neighbors[i+1]
            if new_dist < shortest_distance[neighbors[i]]:
                shortest_distance[neighbors[i]] = new_dist
                heapq.heappush(heap, (new_dist, neighbors[i]))
    return shortest_distance

"""
Produces an inverted version the given graph in such a way that all the edges of the original point in the opposite
 direction.
"""
def invert_graph(graph):
    inverted = [[] for _ in range(len(graph))]

    for u in range(len(graph)):
        neighbors = graph[u]
        for v in range(0, len(neighbors), 2):
            vertex = neighbors[v]
            weight = neighbors[v+1]
            inverted[vertex].append(u)
            inverted[vertex].append(weight)
    return inverted


"""
:param: graph: Array of neighbor array. i'th entry is a list of neighbors to vertex i. The j'th entry, which is an even
    number, is the index id goal of the neighbor to vertex i and the j+1'th entry, which is an odd number, is the weight of
    the edge from i to goal.
:param: L0: The first landmark, Typically the most central point
:return:
landmarks - a list of node id index to identify which node is a landmark
landmark_distances - a dictionary where node id index 'n' is the key and the value is an array 'arr' is a list of
                     distances. Each entry 'i' in the array 'arr' corresponds to the distance from the landmark 'n'
                     to node 'i'.
"""

def farthest_landmark_selection_2(L0, graph, k=4):
    def from_landmark_distances(L0, graph):
        landmarks = list([L0])
        from_landmark = dict()
        from_landmark[L0] = from_landmark_Dijkstra(L0, graph)

        while len(landmarks) < k:
            # Organize the distances of landmarks as a matrix
            A = np.array(list(from_landmark.values()))
            _, col = np.shape(A)

            candidates = np.zeros(col)

            for i in range(col):
                candidates[i] = np.max(A[:, i])

            candidates = np.where(np.isinf(candidates), -np.inf, candidates)

            while True:
                l = np.argmax(candidates)
                if l in landmarks:
                    candidates[l] = -np.inf
                else:
                    landmarks.append(l)
                    from_landmark[l] = from_landmark_Dijkstra(l, graph)
                    break

        return landmarks, from_landmark

    landmarks, from_landmark = from_landmark_distances(L0, graph)

    # Compute the distances from each vertex to landmarks
    inverted_graph = invert_graph(graph)
    to_landmark = dict()

    for l in landmarks:
        to_landmark[l] = from_landmark_Dijkstra(l, inverted_graph)

    return landmarks, from_landmark, to_landmark

def farthest_landmark_selection_1(L0, graph, k=4):

    """
    Responsible for selecting the landmarks, based on the farthest landmark approach. Also to generate a dictionary
    of distances from the landmark to points.
    :param L0: The first landmark. Typically, the most central point.
    :param graph: The graph to run Dijkstra's algorithm on.
    :param k: The number of landmarks to be selected.
    :return: (landmarks, from_landmark) a tuple where landmarks is a list of point identifiers that are chosen as
        landmarks. from_landmark is a dictionary where key is the landmark and value is a list where the i'th entry
        is the distance from the landmark key to point i.
    """
    def from_landmark_distances(L0, graph, k=k):
        landmarks = list([L0])
        from_landmark = dict()
        from_landmark[L0] = from_landmark_Dijkstra(L0, graph)

        while len(landmarks) < k:
            # Organize the distances of landmarks as a matrix
            A = np.array(list(from_landmark.values()))

            # Sum the columns of A, such that we get an array where entry i corresponds to the sum of distances between
            # node i and all existing landmarks.
            dist_sum = np.sum(A, axis=0)
            masked_dist_sum = np.where(np.isinf(dist_sum), -np.inf, dist_sum)
            while True:
                l = np.argmax(masked_dist_sum)
                if l in landmarks: # If l is already in landmarks, disqualify it and find the next farthest
                    masked_dist_sum[l] = -np.inf
                else:
                    landmarks.append(l)
                    from_landmark[l] = from_landmark_Dijkstra(l, graph)
                    break

        return landmarks, from_landmark

    landmarks, from_landmark = from_landmark_distances(L0, graph)

    # Compute the distances from each vertex to landmarks
    inverted_graph = invert_graph(graph)
    to_landmark = dict()

    for l in landmarks:
        to_landmark[l] = from_landmark_Dijkstra(l, inverted_graph)

    return landmarks, from_landmark, to_landmark



"""
data: dictionary that maps name of preprocessing metadata to data. For instance, our preprocessing may involve 
    computation of landmarks. Then there will be a key-value pair in data dictionary with key 'landmarks' and value
    that is an array of node id's that indicate which node is a landmark. There would also be a key-value pair with
    key 'landmark_distances' and value is an array where each entry i is an array for landmark i. The entry j of 
    landmark i is the distance between landmark i and node j.
    data dictionary may include other kinds of data related to preprocessing.
"""
def ALT(graph, start, goal, lat=[], lon=[], data={}):
    # landmarks is a list of node id index to identify which node is a landmark
    # from_landmark is a dictionary where node id index 'n' is the key and the value is an array 'arr' is a list of
    # distances. Each entry 'i' in the array 'arr' corresponds to the distance from the landmark 'n' to node 'i'.
    # to_landmark is symmetric to from_landmark. It is just the distance from vertex to landmark.
    landmarks, from_landmark, to_landmark = set(data['landmarks']), data['from_landmark'], data['to_landmark']

    def h_L(landmark, u):
        d_l_t = from_landmark[landmark][goal]
        d_l_u = from_landmark[landmark][u]
        d_t_l = to_landmark[landmark][goal]
        d_u_l = to_landmark[landmark][u]
        return max(d_u_l - d_t_l, d_l_t - d_l_u)


    optimized_landmarks = set()
    value = float('-inf')
    landmark_1 = None
    for l in landmarks:
        new_val = abs(h_L(l, start) - h_L(l, goal))
        if new_val > value:
            landmark_1 = l
            value = new_val
    optimized_landmarks.add(landmark_1)

    dist = float('inf')
    landmark_2 = None
    for l in landmarks.difference(optimized_landmarks):
        h_val = h_L(l, start)
        if dist > h_val:
            landmark_2 = l
            dist = h_val
    optimized_landmarks.add(landmark_2)

    dist = float('inf')
    landmark_3 = None
    for l in landmarks.difference(optimized_landmarks):
        h_val = h_L(l, goal)
        if dist > h_val:
            landmark_3 = l
            dist = h_val
    optimized_landmarks.add(landmark_3)

    dist = float('inf')
    landmark_4 = None
    for l in landmarks.difference(optimized_landmarks):
        new_distance = h_L(l, start) + h_L(l, goal)
        if dist > new_distance:
            landmark_4 = l
            dist = new_distance
    optimized_landmarks.add(landmark_4)

    """
    Finds the maximum lower bound for the distance between node u and node goal using the triangle inequality.
    """
    def ALT_heuristic(u):
        h = float('-inf')
        for landmark in optimized_landmarks:
            h = max(h, h_L(landmark, u))
        return h

    gc.collect()

    shortest_distance = {node: math.inf for node in range(len(graph))}
    shortest_distance[start] = 0
    track_predecessor = {}
    track_path = []
    seenNodes = set()
    heap = []
    heapq.heappush(heap, (0, start))
    iterations = 0
    h_start = ALT_heuristic(start)
    while heap:
        _, current_node = heapq.heappop(heap)
        iterations += 1

        if current_node in seenNodes: # We have already visited this node
            continue
        seenNodes.add(current_node)

        if current_node == goal: # We have found the goal
            break

        # Find the maximum lower bound from current_node to goal
        # h_curr_node = ALT_heuristic(current_node, goal, landmarks, from_landmark, to_landmark)
        #print('Approx distance to goal: ', h_curr_node)

        neighbors = graph[current_node]
        for i in range(0, len(neighbors), 2):
            neighbor_node = neighbors[i]

            if neighbor_node in seenNodes: # We have already visited this neighbor
                continue

            weight = neighbors[i + 1]
            dist_to_neighbor = shortest_distance[current_node] + weight  # Distance from start to neighbor

            if dist_to_neighbor < shortest_distance[neighbor_node]: # We have found a shorter distance
                # Find the maximum lower bound from neighbor to goal
                h_neighbor = ALT_heuristic(neighbor_node)
                shortest_distance[neighbor_node] = dist_to_neighbor
                track_predecessor[neighbor_node] = current_node
                priority = shortest_distance[neighbor_node] + h_neighbor - h_start
                # Heuristic score of the neighbor
                heapq.heappush(heap, (priority, neighbor_node))

    current_node = goal
    while current_node != start:
        try:
            track_path.insert(0, current_node)
            current_node = track_predecessor[current_node]
        except KeyError:
            print('Path not reachable')
            break
    track_path.insert(0, start)
    print('Number of iterations: ', iterations)
    print('Total nodes scanned by ALT: ', len(seenNodes))
    print('Shortest distance is ' + str(shortest_distance[goal]))
    print('And the path is ' + str(track_path))
    return track_path



