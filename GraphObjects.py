class Vertice():

    def __init__(self, id):
        self.id = id
        self.lon = -1
        self.lat = -1
        # out is a list of outgoing edges of this node
        self.outgoing = []
        # ingoing is a list of ingoing edges of this node
        self.ingoing = []
        # Item in neighbors dictionary is: id:distance
        # id: identifier of the neighbor node
        # distance: the distance from this Vertice to the neighbor
        self.neighbors = {}

    def get_neighbor_id(self):
        res = []
        for edge in self.outgoing:
            res.append(edge.to_node)
        return res

class Edge():
    """
    u and v are nodes/vertices at the ends of the arc/edge.
    They come as unique id's
    """
    def __init__(self, u, v):
        self.length = -1
        self.from_node = u
        self.to_node = v
