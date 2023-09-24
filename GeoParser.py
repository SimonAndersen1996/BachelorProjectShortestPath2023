from Python import GraphObjects
# TODO: Test this file with implementation of Dijkstra's
"""
Expects to receive a GeoDataFrame of edges as input.
Will output a set of Vertices that know their neighbors and their distance
"""
def parseGeoDataFrame(gdf):
    vertices = {} # A dictionary of Vertices
    for index, row in gdf.iterrows():
        x1, y1, x2, y2 = getEdgeCoordinates(row["geometry"])
        try:
            u = vertices[row["u"]]
        except:
            u = GraphObjects.Vertice(row["u"])
        u.lon = x1
        u.lat = y1
        try:
            v = vertices[row["v"]]
        except:
            v = GraphObjects.Vertice(row["v"])
        v.lon = x2
        v.lat = y2
        u.neighbors[v.id] = row["length"]
        v.neighbors[u.id] = row["length"]
        vertices[row["u"]] = u
        vertices[row["v"]] = v
    return vertices

"""
Given the edge, return the coordinates of its endpoint.
Type of 'edge' is LineString from the GeoDataFrame.
"""
def getEdgeCoordinates(edge):
    u, v = edge.coords  # Extract the pairs of coordinates
    x1, y1 = u  # Extract the coordinates of one of the points, the first one
    x2, y2 = v
    return x1, y1, x2, y2

