from Python.Dijkstra2 import dijkstra2
from Python.WriteToFile import readFrom
from Python.QuadTree import build_quad_tree
from Python.EdgeCompression import compress
import numpy as np

class ShortestPath:

    def __init__(self):
        self.algorithm = dijkstra2

    """
    
    """
    def compute_shortest_path(self, lat_arr, lon_arr, pickled_file, s_lat, s_lon, t_lat, t_lon, data={}):
        graph = readFrom(pickled_file)
        #graph = compress([i for i in range(len(graph))], graph)
        # TODO: Use the QuadTree to search the start and end nodes, given the coordinates.
        QT = data.get('QuadTree')
        if QT is None:
            QT = build_quad_tree(lat_arr, lon_arr, m=2)
        start = QT.root_search_node(s_lat, s_lon, lat_arr, lon_arr)
        end = QT.root_search_node(t_lat, t_lon, lat_arr, lon_arr)
        print('Start node: ', start)
        print('End node: ', end)
        return self.algorithm(graph, start, end, lat=lat_arr, lon=lon_arr, data=data)

    """
    coord is a tuple of float
    This is a naive implementation that assumes that the coordinates are exactly on an existing point.
    """
    def find_node(self, graph, coord):
        min = np.infty
        result = None
        for id, node in graph.items():
            dist = np.sqrt((coord[0] - node.lat)**2 + (coord[1] - node.lon)**2)
            if dist < min:
                min = dist
                result = id
        return result
