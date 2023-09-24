import GeoParser
import GraphObjects
from pyrosm import OSM
from pyrosm import get_data
from pyrosm import OSM, get_data
import osmnx as ox
from GraphPlotter import plotGraphGDF

def main():
    # Pyrosm comes with a couple of test datasets
    # that can be used straight away without
    # downloading anything
    fp = "test.pbf"

    # Initialize the OSM parser object
    osm = OSM(fp)

    # Read all drivable roads
    # =======================
    drive_net = osm.get_network(network_type="driving")
    #drive_net.plot()
    drive_net.head(2)


    # Initialize the reader
    osm = OSM("../Ressources/test.pbf")

    # Get all walkable roads and the nodes
    nodes, edges = osm.get_network(nodes=True)

    # Check first rows in the edge
    edges.head()
    G = osm.to_graph(nodes, edges, graph_type="networkx")
    G
    ox.plot_graph(G)

    #print(edges.dtypes)
    #print(edges["geometry"])
    """
    
    for index, row in edges.iterrows():
        u, v = row["geometry"].coords # Extract the pairs of coordinates
        x, y = u # Extract the coordinates of one of the points, the first one
        print(x) # Print the first coordinate of the first point. It is a float
    """
    """
    for index, row in nodes.iterrows():
    print(row)
    """
    data = GeoParser.parseGeoDataFrame(edges)
    plotGraphGDF(data)


if __name__ == "__main__":
    main()

