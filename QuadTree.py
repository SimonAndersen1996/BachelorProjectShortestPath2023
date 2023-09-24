import numpy as np
import math
from functools import reduce
from sys import getsizeof

class QuadTree():

    def __init__(self, lon1, lat1, lon2, lat2):
        self.lon1 = lon1
        self.lat1 = lat1
        self.lon2 = lon2
        self.lat2 = lat2
        self.node_indices = [] # Array of node index identifiers
        self.children = [None, None, None, None]

    """
    Searches the next child overlapping the lat and lon in the tree, returning the index number of child.
    """
    def _get_quad(self, lat, lon):
        lon_mid = (self.lon1 + self.lon2) / 2
        lat_mid = (self.lat1 + self.lat2) / 2
        if lon <= lon_mid:
            if lat <= lat_mid:
                return 2
            else:
                return 0
        else:
            if lat <= lat_mid:
                return 3
            else:
                return 1
    """
    The QuadTree searches for a node within itself that is closest to the given lat and lon coordinates. 
    """
    def search_node(self, lat, lon, lat_arr, lon_arr):
        quad = self._get_quad(lat, lon)
        quad_tree = self.children[quad]
        if quad_tree is None:
            node = _nearest_node(lat, lon, lat_arr, lon_arr, self.node_indices)
            # Should look through other children if node is None
            if node is None:
                other_children = [0,1,2,3]
                other_children.remove(quad)
                candidates = [] # list of possible points as the closest to the click
                for child in other_children:
                    if self.children[child] is None:
                        continue
                    candidates.append(self.children[child].search_node(lat, lon, lat_arr, lon_arr))
                return _nearest_node(lat, lon, lat_arr, lon_arr, candidates)
            else:
                return node
        else:
            return quad_tree.search_node(lat, lon, lat_arr, lon_arr)

    def root_search_node(self, lat, lon, lat_arr, lon_arr):
        #print("Root search node invoked")
        node = self.search_node(lat, lon, lat_arr, lon_arr)
        #print("Node in simple search: ", node)
        nlat, nlon = lat_arr[node], lon_arr[node]
        # Compute the distance between the query coordinate and the "closest" node found
        dist = haversine(lat, lon, nlat, nlon)
        candidates = self.query_range(lon-dist, lat-dist, lon+dist, lat+dist, lat_arr, lon_arr)
        return _nearest_node(lat, lon, lat_arr, lon_arr, candidates)

    def center_point(self, latitudes, longitudes):
        lon_offset = (self.lon2 - self.lon1) / 4
        lat_offset = (self.lat2 - self.lat1) / 4
        mid_lon = (self.lon1 + self.lon2) / 2
        mid_lat = (self.lat1 + self.lat2) / 2
        canditates = self.query_range(mid_lon - lon_offset, mid_lat - lat_offset,
                                      mid_lon + lon_offset, mid_lat + lat_offset, latitudes, longitudes)
        return _nearest_node(mid_lat, mid_lon, latitudes, longitudes, canditates)

    """
    Gathers a list of identifiers of nodes that intersects with the range defined by x1, y1, x2 and y2
    """
    def query_range(self, x1, y1, x2, y2, lat_arr, lon_arr):
        result = []
        if self._intersects(x1, y1, x2, y2):
            for node in self.node_indices:
                if x1 <= lon_arr[node] <= x2 and y1 <= lat_arr[node] <= y2:
                    result.append(node)
            for child in self.children:
                if child is not None:
                    result.extend(child.query_range(x1, y1, x2, y2, lat_arr, lon_arr))
        return result
    def _intersects(self, x1, y1, x2, y2):
        return not (x1 > self.lon2 or x2 < self.lon1 or y1 > self.lat2 or y2 < self.lat1)

"""
Takes arrays of latitudes and longtitudes.
m is the minimum number of points a square at leaf level must contain
"""
def build_quad_tree(latitude, longtitude, m=100):

    """
    point_indices is an array of point id that belong to the square.
    min_points is the minimum number of points a square must contain.
    """
    def build_tree_node(point_indices, min_points):
        if len(point_indices) < min_points:
            return None

        # Get the coordinates of points within the current square
        new_lon = [] #reduce((lambda list, i: np.append(list, longtitude[i])), point_indices, np.array([]))
        new_lat = [] #reduce((lambda list, i: np.append(list, latitude[i])), point_indices, np.array([]))
        for i in range(len(point_indices)):
            new_lon.append(longtitude[point_indices[i]])
        for i in range(len(point_indices)):
            new_lat.append(latitude[point_indices[i]])

        # Compute the middle point for the square
        xmin = np.min(new_lon)
        xmax = np.max(new_lon)
        xmean = (xmin + xmax) / 2
        west, east = _sort_indices(point_indices, longtitude, xmean)

        ymin = np.min(new_lat)
        ymax = np.max(new_lat)
        ymean = (ymin + ymax) / 2
        south, north = _sort_indices(point_indices, latitude, ymean)

        # Organize the points into their respective four regions of the current square
        """ Set solution for computing ordinal directions """
        northwest = list(set(north).intersection(set(west)))
        northeast = list(set(north).intersection(set(east)))
        southwest = list(set(south).intersection(set(west)))
        southeast = list(set(south).intersection(set(east)))

        # TODO: As it is now, there will be duplicates of a point id being added in the same region.
        #  We must ensure that an id is not added if it already exists. For now, we use set datastructure as
        #  a solution
        """ List solution for computing ordinal directions
        northwest = []
        northeast = []
        southwest = []
        southeast = []
        for i in range(len(west)):
            if latitude[west[i]] >= ymean:
                northwest.append(west[i])
            else:
                southwest.append(west[i])
        for i in range(len(east)):
            if latitude[east[i]] >= ymean:
                northeast.append(east[i])
            else:
                southeast.append(east[i])
        for i in range(len(north)):
            if longtitude[north[i]] >= xmean:
                northeast.append(north[i])
            else:
                northwest.append(north[i])
        for i in range(len(south)):
            if longtitude[south[i]] >= xmean:
                southeast.append(south[i])
            else:
                southwest.append(south[i])
        """

        squares = [northwest, northeast, southwest, southeast]

        # Instantiate corresponding tree node for the current square
        QT = QuadTree(xmin, ymin, xmax, ymax)

        # Build the children for the current tree node
        for i in range(len(squares)):
            child = build_tree_node(squares[i], m)
            if child == None:
                QT.node_indices.extend(squares[i])
            else:
                QT.children[i] = child
        return QT

    return build_tree_node(np.arange(len(longtitude)), 0)






"""
A sorting algorithm that takes an array and returns the indices of the sorted array in two parts, one with values below
split and one with values above split.
"""
def _sort_indices(points, arr, split):
    lower, upper = [], []
    for i in range(len(points)):
        i_ = points[i]
        if arr[i_] <= split:
            lower.append(points[i])
        else:
            upper.append(points[i])
    return lower, upper

def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the earth in km
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(lat1)) \
        * math.cos(math.radians(lat2)) * math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = R * c  # Distance in km
    return d

""" Given a list of possible nodes, find the one that is closest to the coordinate (lat, lon)"""
def _nearest_node(lat, lon, lat_arr, lon_arr, nodes):
    best_node = None
    best_dist = np.infty
    for i in nodes:
        dist = haversine(lat, lon, lat_arr[i], lon_arr[i])
        if dist < best_dist:
            best_dist = dist
            best_node = i
    return best_node