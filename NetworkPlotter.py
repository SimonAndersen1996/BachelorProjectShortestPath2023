import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from Python.ShortestPath import ShortestPath

class NetworkPlotterClass():

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect("button_press_event", self.onclick)
        self.G = nx.Graph()
        self.click = 0
        self.SP = ShortestPath()
        self.graph = None
        self.lat = []
        self.lon = []
        self.s_lat = 0.
        self.s_lon = 0.
        self.t_lat = 0.
        self.t_lon = 0.
        # Various preprocessing data in the form of a dictionary
        self.data = {}
        # The pickle_file is solely used to invoke the ShortestPath object
        self.pickle_file = ''

    """
    graph: the array of arrays of neighbors and weights
    lat_arr: i'th entry is the latitude of node i
    lon_arr: i'th entry is the longitude of node i
    contracted_indices: list of nodes that have been contracted and should not be plotted
    """
    def plotgraph(self, graph, lat_arr, lon_arr):
        self.lat = lat_arr
        self.lon = lon_arr
        all_nodes = [i for i in range(len(graph))]
        pos = dict()
        for u in all_nodes:
            neigbors = graph[u]
            lon = lon_arr[u]
            lat = lat_arr[u]
            pos[u] = (lon, lat)
            for v in range(0, len(neigbors), 2):
                self.G.add_edge(u, neigbors[v])

        nx.draw(self.G, pos=pos, ax=self.ax, node_size=0)
        plt.show()

    def onclick(self, event):
        self.click += 1
        ix, iy = event.xdata, event.ydata
        if self.click == 1:
            print("START COORDS: ", ix, iy)
            self.s_lat, self.s_lon = iy, ix
            self.ax.plot(ix, iy, 'yo')
            plt.draw()
            plt.show()
        elif self.click == 2:
            print("TARGET COORDS: ", ix, iy)
            self.t_lat, self.t_lon = iy, ix
            self.ax.plot(ix, iy, 'go')
            plt.draw()
            plt.show()
            print("Computing the shortest path")
            shortest_path = self.SP.compute_shortest_path(self.lat, self.lon, self.pickle_file, self.s_lat, self.s_lon, self.t_lat, self.t_lon, self.data)

            # TODO: Plot the shortest path
            self._plot_path(shortest_path)
            plt.draw()
            plt.show()
        else:
            pass
    def _plot_path(self, path):
        lats = []
        lons = []
        for i in range(len(path)):
            lats.append(self.lat[path[i]])
            lons.append(self.lon[path[i]])
        plt.plot(lons, lats, 'r-')

if __name__ == "__main__":
    G = nx.Graph()
    G.add_nodes_from([i for i in range(3)])
    #G.add_edges_from([(1,2), (2,3), (3,1)])

    pos = {0:(0,0), 1:(1,1), 2:(-1,1)}
    fig, ax = plt.subplots()

    nx.draw(G, pos=pos, ax=ax)
    plt.show()
