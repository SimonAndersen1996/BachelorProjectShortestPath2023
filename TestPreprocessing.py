import unittest
from Python import MyOSMParser, WriteToFile, Dijkstra, Dijkstra2, WhiteList
from Python.QuadTree import QuadTree, build_quad_tree
from Python.EdgeCompression import compress
from Python.ALTalgorithm import farthest_landmark_selection_1
from random import randrange
import numpy as np

class MyPreprocessing(unittest.TestCase):

    def setUp(self):
        self.nodes1 = {'1': 0, '2': 1, '3': 2, '4': 3, '5': 4, '6': 5}
        self.graph1 = [[1, 1], [0, 1, 2, 1], [1, 1, 3, 1], [2, 1, 4, 1], [3, 1, 5, 1], [4, 1]]
        self.nodes2 = {'1': 0, '2': 1, '3': 2, '4': 3, '5': 4, '6': 5, '7': 6}
        self.graph2 = [[1, 1], [0, 1, 2, 1], [1, 1, 3, 1], [2, 1, 4, 1, 6, 2], [3, 1, 5, 1], [4, 1], [3, 2]]
        self.lat1 = [0, 100, 1, 1]
        self.lon1 = [0, 100, 1, 4]
        self.lat2 = [0, 100]
        self.lat2.extend([i for i in range(10)])
        self.lon2 = [0, 100]
        self.lon2.extend([i for i in range(10)])



    def test1(self):
        _, _, _, graph = compress(self.nodes1, self.graph1)
        expected1, expected2 = [5, 5], [0, 5]
        assert expected1 == graph[0]
        assert expected2 == graph[5]

    def testCompressAndDijkstra(self):
        _, _, _, graph = compress(self.nodes1, self.graph1)
        expected = [0, 5]
        result = Dijkstra.dijkstra(graph, 0, 5)
        assert expected == result


    def testAdvancedCompressAndDijkstra(self):
        _, _, _, graph = compress(self.nodes2, self.graph2)
        print("Graph compressed: ", graph)
        expected = [0, 3, 5]
        result = Dijkstra2.dijkstra2(graph, 0, 5)
        assert expected == result

    def testQuadTree1(self):
        QT = build_quad_tree(self.lat1, self.lon1)
        expected = 2
        result = QT.search_node(1, 1, self.lat1, self.lon1)
        assert result == expected
        expected = 1
        result = QT.search_node(95, 95, self.lat1, self.lon1)
        assert result == expected

    # Should find a closest point despite clicking on a square with no points within, by find a point in another square
    def testQuadTree2(self):
        QT = build_quad_tree(self.lat2, self.lon2, m=5)
        result = QT.search_node(55, 5, self.lat2, self.lon2)
        assert result != None

    def test_QT_katrinebjerg(self):
        _, lat, lon, _, _ = MyOSMParser.parseOSMFile('../Ressources/katrinebjerg.osm')
        QT = build_quad_tree(lat, lon, m=50)
        print("QuadTree of katrinebjerg built")
        node = randrange(0, len(lat))
        print('(', lat[node], ', ', lon[node], ')')
        result = QT.search_node(lat[node], lon[node], lat, lon)
        print('(', lat[result], ', ', lon[result], ')')
        print("Expected: ", node)
        print("Result: ", result)
        assert result == node

    def test_farthest_landmark_selection(self):
        landmarks, landmark_distances = farthest_landmark_selection_1(0, self.graph1, k=2)
        print(landmarks)
        print(landmark_distances)
