import copy
import time
import unittest
from Python import MyOSMParser, Dijkstra, Dijkstra2, Astar, GraphPlotter
from Python.EdgeCompression import compress
from Python.NetworkPlotter import NetworkPlotterClass
from Python.WriteToFile import writeTo, readFrom
from Python.ALTalgorithm import ALT, farthest_landmark_selection_1
from Python.QuadTree import build_quad_tree, _nearest_node


class MyAlgorithm(unittest.TestCase):

    def setUp(self):
        self.nodes, self.lat, self.lon, self.graph, self.ways = MyOSMParser.parseOSMFile('../Ressources/test.osm')
        self.pickle_graph = 'picklefiles/pickled_graph.txt'

        self.nodes1, self.lat1, self.lon1, self.graph1, self.ways1 = MyOSMParser.parseOSMFile('../Ressources/skjoldhøj.osm')

    def test_dijkstra(self):
        ids = ['5438483250', '365391638', '355160356', '355160359', '355160364', '355160367', '355160372', '355160375', '355160380', '371215991', '4684142794', '355160393', '355160396']
        expected = list(map(self.look, ids))
        start = self.look('5438483250')
        end = self.look('355160396')
        result = Dijkstra.dijkstra(self.graph, start, end)
        print("Expected: ", expected)
        print("Result: ", result)
        assert result == expected

    """
    A version Dijkstra that uses heaps 
    """
    def test_dijkstra2(self):
        ids = ['5438483250', '365391638', '355160356', '355160359', '355160364', '355160367', '355160372', '355160375',
               '355160380', '371215991', '4684142794', '355160393', '355160396']
        expected = list(map(self.look, ids))
        start = self.look('5438483250')
        end = self.look('355160396')
        result = Dijkstra2.dijkstra2(self.graph, start, end)
        print("Expected: ", expected)
        print("Result: ", result)
        assert result == expected

    def test_A_star(self):
        ids = ['5438483250', '365391638', '355160356', '355160359', '355160364', '355160367', '355160372', '355160375',
               '355160380', '371215991', '4684142794', '355160393', '355160396']
        expected = list(map(self.look, ids))
        start = self.look('5438483250')
        end = self.look('355160396')
        result = Astar.astar(self.graph, start, end, lat=self.lat, lon=self.lon)
        print("Expected: ", expected)
        print("Result: ", result)
        assert result == expected

    def test_ALT_algorithm(self):
        ids = ['5438483250', '365391638', '355160356', '355160359', '355160364', '355160367', '355160372', '355160375',
               '355160380', '371215991', '4684142794', '355160393', '355160396']
        expected = list(map(self.look, ids))
        start = self.look('5438483250')
        end = self.look('355160396')
        QT = build_quad_tree(self.lat, self.lon, m=4)
        center_point = QT.center_point(self.lat, self.lon)
        print("'bout to create landmarks")
        landmarks, from_landmark, to_landmark = farthest_landmark_selection_1(center_point, self.graph)
        print("Created landmarks")
        data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark}
        result = ALT(self.graph, start, end, lat=self.lat, lon=self.lon, data=data)
        print("Expected: ", expected)
        print("Result: ", result)
        assert result == expected

    def test_FUCK_ALT(self):
        lat = readFrom('../Ressources/Denmark/DK_lat.txt')
        lon = readFrom('../Ressources/Denmark/DK_lon.txt')
        QT = readFrom('../Ressources/Denmark/DK_QT.txt')
        graph = readFrom('../Ressources/Denmark/DK_graph.txt')
        landmarks = readFrom('../Ressources/Denmark/DK_landmarks_4_1.txt')
        from_landmark = readFrom('../Ressources/Denmark/DK_from_L_4_1.txt')
        to_landmark = readFrom('../Ressources/Denmark/DK_to_L_4_1.txt')

        data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark, 'QuadTree': QT}
        start = QT.root_search_node(55.8, 9.0, lat, lon)
        end = QT.root_search_node(60.0, 10.0, lat, lon)
        print('Start: ', start)
        print('End: ', end)

        ALT(graph, start, end, lat=lat, lon=lon, data=data)

    def test_ALT_plot(self):
        print("Starting test")
        file = '../Ressources/pickledSkjoldhoj.txt'
        writeTo('../Ressources/pickledSkjoldhoj.txt', self.graph1)
        old_nodes = copy.copy(self.nodes1)
        QT = build_quad_tree(self.lat1, self.lon1, m=4)
        center_point = QT.center_point(self.lat1, self.lon1)
        landmarks, from_landmark, to_landmark = farthest_landmark_selection_1(center_point, self.graph1, k=4)
        data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark}
        _, _, remove_index, _ = compress(self.nodes1, self.graph1)
        plotter = GraphPlotter.GraphPlotterClass(file)
        plotter.data = data
        plotter.SP.algorithm = ALT
        plotter.plotgraph(self.lat1, self.lon1, self.ways1, old_nodes, remove_index)

    def test_ALT_DK(self):
        data = readFrom('../Ressources/Denmark/DK_data.txt')
        graph = readFrom('../Ressources/Denmark/DK_graph.txt')
        lat = readFrom('../Ressources/Denmark/DK_lat.txt')
        lon = readFrom('../Ressources/Denmark/DK_lon.txt')
        networkplt = NetworkPlotterClass()
        networkplt.data = data
        networkplt.SP.algorithm = ALT
        networkplt.pickle_file = '../Ressources/Denmark/DK_graph.txt'
        start = time.time()

        networkplt.plotgraph(graph, lat, lon)
        end = time.time()
        print("Time for plotting Denmark: ", end - start)


    # Auxiliary functions
    def look(self, x):
        return self.nodes[x]

if __name__ == '__main__':
    unittest.main()
