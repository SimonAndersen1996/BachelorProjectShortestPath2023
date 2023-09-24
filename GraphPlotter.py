import matplotlib.pyplot as plt
from matplotlib.artist import Artist
import numpy as np
from Python import WriteToFile
import copy

from Python.ShortestPath import ShortestPath


class GraphPlotterClass:
    """
    graph_name: the file name of the pickled graph, which is an array of neighbor arrays
    """
    def __init__(self, graph_name):
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect("button_press_event", self.onclick)
        self.click = 0
        self.SP = ShortestPath()
        self.graph = None
        self.lat = []
        self.lon = []
        self.nodes = {}
        self.s_lat = 0.
        self.s_lon = 0.
        self.t_lat = 0.
        self.t_lon = 0.
        # Various preprocessing data in the form of a dictionary
        self.data = dict()
        # The pickle_file is solely used to invoke the ShortestPath object
        self.pickle_file = graph_name


    def onclick(self, event):
        self.click += 1
        ix, iy = event.xdata, event.ydata
        if self.click == 1:
            print("START COORDS: ", ix, iy)
            self.s_lat, self.s_lon = iy, ix
        elif self.click == 2:
            print("TARGET COORDS: ", ix, iy)
            self.t_lat, self.t_lon = iy, ix
            print("Computing the shortest path")
            shortest_path = self.SP.compute_shortest_path(self.lat, self.lon, self.pickle_file, self.s_lat, self.s_lon, self.t_lat, self.t_lon, self.data)
            ways = [[]]
            for entry in shortest_path:
                key = list(self.nodes.keys())[list(self.nodes.values()).index(entry)]
                ways[0].append(str(key))

            # self.ax.cla()
            self.plotedges(self.lat, self.lon, ways, self.nodes, color='-b')
            plt.draw()
        else:
            pass



    """
    latidude & longitude: arrays of coordinates
    ways: array of ways. Each element is also an array of the original id of the osm-file
    nodes: dictionary where original id's are mapped to corresponding index id
    index: list of index id that should be excluded from plotting
    """
    def plotgraph(self, latitude, longitude, ways, nodes, index):
        self.lat = latitude
        self.lon = longitude
        self.nodes = nodes
        latcopy, loncopy = copy.copy(latitude), copy.copy(longitude)
        count = 0
        for ele in index:
            #print(ele)
            del latcopy[ele-count]
            del loncopy[ele-count]
            count += 1
        self.plotpoints(latcopy, loncopy)
        self.plotedges(latitude, longitude, ways, nodes, color='-r')
        self.fig.canvas.mpl_connect('button_press_event', self.onclick)

    def plotedges(self, latitude, longitude, ways, nodes, color):
        for way in ways:
            #print("Now plotting way: ", way)
            x, y = [], []
            for i in range(len(way)):
                value = nodes[way[i]]
                currentlat, currentlon = latitude[value], longitude[value]
                x.append(currentlon), y.append(currentlat)
            self.ax.plot(x, y, color)
            x.clear(), y.clear()
        plt.show()

    def plotpoints(self, latitude, longitude):
        self.ax.plot(longitude, latitude, 'o')



if __name__ == "__main__":
    # lat, lon = [10.3, 10.5, 10.7, 10.75, 10.6, 10.2, 10.9], [56.7, 56.8, 56.9, 56.32, 56.43, 56.64, 56.35]
    # ways = [['1', '2', '3', '4'], ['5', '6', '7']]
    # index = [1, 2, 4]
    # nodes, lat, lon, graph, ways = MyOSMParser.parseOSMFile('../Ressources/viborgvej.osm')
    # pickle = WriteToFile.writeTo('../Ressources/pickledViborgvej.txt', graph)
    # plotter.plotgraph(lat, lon, ways, nodes, [])

    """
    Our graph is a single jagged line with nodes that are named in the following order: [3, 2, 0, 4, 1]
    """
    nodes = {'1111': 0, '2222': 1, '3333': 2, '4444': 3, '5555': 4}
    graph = [[2, 1, 4, 1], [4, 1], [0, 1, 3, 1], [2, 1], [0, 1, 1, 1]]
    lat, lon = [1., 2., 3., 4., 5.], [3., 5., 2., 1., 4.]
    ways = [['4444', '3333', '1111', '5555', '2222']]
    pickle = WriteToFile.writeTo('../Ressources/pickleTest1.txt', graph)
    plotter = GraphPlotterClass('../Ressources/pickleTest1.txt')
    plotter.plotgraph(lat, lon, ways, nodes, [])
