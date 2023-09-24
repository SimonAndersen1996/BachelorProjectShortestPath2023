import math
from heapq import heappush, heappop

graph = {

    'a': {'b': 1, 'c': 2},
    'b': {'c': 1, 'f': 5},
    'c': {'f': 6, 'd': 2},
    'd': {'e': 3, 'g': 6},
    'e': {'g': 3, 'h': 4},
    'f': {'e': 1, 'h': 8},
    'g': {'h': 2},
    'h': {}
}

def haversine(point1, point2):
    R = 6371  # Radius of the earth in km
    lat1, lon1 = point1
    lat2, lon2 = point2
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(lat1)) \
        * math.cos(math.radians(lat2)) * math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = R * c  # Distance in km
    return d

def astar(graph, start, goal, lat=[], lon=[], data={}):
    shortest_distance = {}
    track_predecessor = {}
    heap = []
    infinity = math.inf
    track_path = []
    visited = set()

    for node in range(len(graph)):
        shortest_distance[node] = infinity

    shortest_distance[start] = 0

    # Add the start node to the heap
    heappush(heap, (0, start))
    iterations = 0
    while heap:
        # Pop the node with the smallest f(x)
        current_distance, current_node = heappop(heap)
        iterations += 1
        if current_node in visited:
            continue

        visited.add(current_node)

        # Check if the current_node is the goal
        if current_node == goal:
            break

        # Scan the neighbors of current_node
        neighbors = graph[current_node]
        for i in range(0, len(neighbors), 2):
            neighbor_node, weight = neighbors[i], neighbors[i+1]
            # Calculate the g(x) and h(x) values
            g = weight + shortest_distance[current_node]
            # Heuristic function (There are different heuristic functions for A*. We use haversine to the goal for now)
            h = haversine((lat[neighbor_node], lon[neighbor_node]), (lat[goal], lon[goal]))

            # Calculate the f(x) value
            f = g + h

            # Update the shortest distance and track predecessor if a better path is found
            if shortest_distance[neighbor_node] > g:
                shortest_distance[neighbor_node] = g
                track_predecessor[neighbor_node] = current_node

                # Add the neighbor node to the heap with the new f(x) value
                heappush(heap, (f, neighbor_node))  

    currentNode = goal

    while currentNode != start:
        try:
            track_path.insert(0, currentNode)
            currentNode = track_predecessor[currentNode]
        except KeyError:
            print('Path not reachable')
            break
    track_path.insert(0, start)

    if shortest_distance[goal] != infinity:
        print('Number of iterations: ', iterations)
        print('Total nodes scanned by A*: ', len(visited))
        print('Shortest distance is ' + str(shortest_distance[goal]))
        print('And the path is ' + str(track_path))
        # Color the visited nodes in the track path

        return track_path, visited

if __name__ == "__main__":
    astar(graph, 'a', 'g') 

"""
graph = {
    'a': {'b': 1, 'c': 2},
    'b': {'c': 1, 'f': 5},
    'c': {'f': 6, 'd': 2},
    'd': {'e': 3, 'g': 6},
    'e': {'g': 3, 'h': 4},
    'f': {'e': 1, 'h': 8},
    'g': {'h': 2},
    'h': {}
}

def manhattan_distance(point1, point2, scale_factor):
    lat1, lon1 = point1
    lat2, lon2 = point2
    # Convert latitude and longitude to grid-based coordinates
    grid_lat1 = lat1 * scale_factor  # Convert latitude to grid-based coordinate
    grid_lon1 = lon1 * scale_factor  # Convert longitude to grid-based coordinate
    grid_lat2 = lat2 * scale_factor
    grid_lon2 = lon2 * scale_factor
    # Calculate Manhattan distance
    distance = abs(grid_lat2 - grid_lat1) + abs(grid_lon2 - grid_lon1)
    return distance

def astar(graph, start, goal, lat=[], lon=[], scale_factor=55, data={}):
    shortest_distance = {}
    track_predecessor = {}
    heap = []
    infinity = math.inf
    track_path = []
    visited = set()

    for node in range(len(graph)):
        shortest_distance[node] = infinity

    shortest_distance[start] = 0

    # Add the start node to the heap
    heappush(heap, (0, start))
    iterations = 0

    while heap:
        # Pop the node with the smallest f(x)
        current_distance, current_node = heappop(heap)
        iterations += 1

        if current_node in visited:
            continue

        visited.add(current_node)

        # Check if the current_node is the goal
        if current_node == goal:
            break

        # Scan the neighbors of current_node
        neighbors = graph[current_node]
        for i in range(0, len(neighbors), 2):
            neighbor_node, weight = neighbors[i], neighbors[i + 1]
            # Calculate the g(x) and h(x) values
            g = weight + shortest_distance[current_node]
            # Heuristic function (Manhattan distance to the goal)
            h = manhattan_distance((lat[neighbor_node], lon[neighbor_node]), (lat[goal], lon[goal]), scale_factor)

            # Calculate the f(x) value
            f = g + h

            # Update the shortest distance and track predecessor if a better path is found
            if shortest_distance[neighbor_node] > g:
                shortest_distance[neighbor_node] = g
                track_predecessor[neighbor_node] = current_node

                # Add the neighbor node to the heap with the new f(x) value
                heappush(heap, (f, neighbor_node))

    currentNode = goal

    while currentNode != start:
        try:
            track_path.insert(0, currentNode)
            currentNode = track_predecessor[currentNode]
        except KeyError:
            print('Path not reachable')
            break
    track_path.insert(0, start)

    if shortest_distance[goal] != infinity:
        print('Number of iterations: ', iterations)
        print('Total nodes scanned by A*: ', len(visited))
        print('Shortest distance is ' + str(shortest_distance[goal]))
        print('And the path is ' + str(track_path))
        # Color the visited nodes in the track path

        return track_path, visited


def euclidean(point1, point2):
    lat1, lon1 = point1
    lat2, lon2 = point2
    dx = lat2 - lat1
    dy = lon2 - lon1
    return math.sqrt(dx**2 + dy**2)

def astar(graph, start, goal, lat=[], lon=[], data={}):
    shortest_distance = {}
    track_predecessor = {}
    heap = []
    infinity = math.inf
    track_path = []
    visited = set()

    for node in range(len(graph)):
        shortest_distance[node] = infinity

    shortest_distance[start] = 0

    # Add the start node to the heap
    heappush(heap, (0, start))
    iterations = 0
    while heap:
        # Pop the node with the smallest f(x)
        current_distance, current_node = heappop(heap)
        iterations += 1
        if current_node in visited:
            continue

        visited.add(current_node)

        # Check if the current_node is the goal
        if current_node == goal:
            break

        # Scan the neighbors of current_node
        neighbors = graph[current_node]
        for i in range(0, len(neighbors), 2):
            neighbor_node, weight = neighbors[i], neighbors[i+1]
            # Calculate the g(x) and h(x) values
            g = weight + shortest_distance[current_node]
            # Heuristic function (using Euclidean distance instead of Haversine)
            h = euclidean((lat[neighbor_node], lon[neighbor_node]), (lat[goal], lon[goal]))

            # Calculate the f(x) value
            f = g + h

            # Update the shortest distance and track predecessor if a better path is found
            if shortest_distance[neighbor_node] > g:
                shortest_distance[neighbor_node] = g
                track_predecessor[neighbor_node] = current_node

                # Add the neighbor node to the heap with the new f(x) value
                heappush(heap, (f, neighbor_node))

    currentNode = goal

    while currentNode != start:
        try:
            track_path.insert(0, currentNode)
            currentNode = track_predecessor[currentNode]
        except KeyError:
            print('Path not reachable')
            break
    track_path.insert(0, start)

    if shortest_distance[goal] != infinity:
        print('Number of iterations: ', iterations)
        print('Total nodes scanned by A*: ', len(visited))
        print('Shortest distance is ' + str(shortest_distance[goal]))
        print('And the path is ' + str(track_path))
        # Color the visited nodes in the track path

        return track_path, visited

"""




