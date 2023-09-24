import unittest, time

from Python import MyOSMParser
from Python.NetworkPlotter import NetworkPlotterClass
from Python.WriteToFile import readFrom, writeTo
from Python.QuadTree import build_quad_tree
from Python.Astar import astar
from Python.EdgeCompression import compress
from memory_profiler import profile

class MyTestPlot(unittest.TestCase):

    def setUp(self):
        self.nodes1, self.lat1, self.lon1, self.graph1, self.ways1 = MyOSMParser.parseOSMFile(
            '../Ressources/skjoldhøj.osm')
        self.nodes2, self.lat2, self.lon2, self.graph2, self.ways2 = MyOSMParser.parseOSMFile(
            '../Ressources/test.osm')
        writeTo('../Ressources/pickledSkoedstrup.txt', self.graph2)

    @profile()
    def test_networkx_skoedstrup(self):
        networkplt = NetworkPlotterClass()
        networkplt.pickle_file = '../Ressources/pickledSkoedstrup.txt'
        networkplt.plotgraph(self.graph2, self.lat2, self.lon2)

    def test_networkx_skjoldhoj1(self):
        networkplt = NetworkPlotterClass()
        networkplt.pickle_file = '../Ressources/pickledSkjoldhoj.txt'
        networkplt.SP.algorithm = astar
        networkplt.plotgraph(self.graph1, self.lat1, self.lon1)

    def test_networkx_skjoldhoj2(self):
        _, _, index_to_remove, new_graph = compress(self.nodes1, self.graph1)
        writeTo('picklefiles/SkjoldhojGraphContracted.txt', new_graph)
        networkplt = NetworkPlotterClass()
        networkplt.pickle_file = 'picklefiles/SkjoldhojGraphContracted.txt'
        networkplt.pickle_file = '../Ressources/pickledSkjoldhoj.txt'
        networkplt.SP.algorithm = astar
        networkplt.plotgraph(self.graph1, self.lat1, self.lon1)

    def test_networkx_DK(self):
        graph = readFrom('../Ressources/pickledDenmarkGraph.txt')
        lat = readFrom('../Ressources/pickledDenmarkLat.txt')
        lon = readFrom('../Ressources/pickledDenmarkLon.txt')
        nodes = readFrom('../Ressources/pickledDenmark.txt')
        QT = build_quad_tree(lat, lon, m=200)
        #_, _, index_to_remove, new_graph = compress(nodes, graph)
        networkplt = NetworkPlotterClass()
        networkplt.data['QuadTree'] = QT
        networkplt.SP.algorithm = astar
        networkplt.pickle_file = '../Ressources/pickledDenmarkGraph.txt'
        start = time.time()
        networkplt.plotgraph(graph, lat, lon)
        end = time.time()
        print("Time for plotting Denmark: ", end - start)

    def test_coompress(self):
        start = time.time()
        nodes = readFrom('../Ressources/pickledDenmark.txt')
        graph = readFrom('../Ressources/pickledDenmarkGraph.txt')
        _, _, index_to_remove, new_graph = compress(nodes, graph)
        end = time.time()
        print("Time for coompres: ", end-start)


if __name__ == '__main__':
    unittest.main()
