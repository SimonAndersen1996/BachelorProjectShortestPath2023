import unittest
from Python import MyOSMParser, WriteToFile, Dijkstra, Dijkstra2, WhiteList
from Python.Astar import astar
from Python.EdgeCompression import compress
from Python.GraphPlotter import GraphPlotterClass
from Python.ShortestPath import ShortestPath
from memory_profiler import profile

class MyParser(unittest.TestCase):


    def setUp(self):
        self.nodes, self.lat, self.lon, self.graph, self.ways = MyOSMParser.parseOSMFile('../Ressources/test.osm')
        self.pickle_graph = 'picklefiles/pickled_graph.txt'
        self.shortest_path = ShortestPath()


    def test_dijkstra(self):
        look = lambda x : self.nodes[x]
        ids = ['5438483250', '365391638', '355160356', '355160359', '355160364', '355160367', '355160372', '355160375', '355160380', '371215991', '4684142794', '355160393', '355160396']
        expected = list(map(look, ids))
        start = look('5438483250')
        end = look('355160396')
        result = Dijkstra.dijkstra(self.graph, start, end)
        print("Expected: ", expected)
        print("Result: ", result)
        assert result == expected

    def test_dijkstra2(self):
        look = lambda x: self.nodes[x]
        ids = ['5438483250', '365391638', '355160356', '355160359', '355160364', '355160367', '355160372', '355160375',
               '355160380', '371215991', '4684142794', '355160393', '355160396']
        expected = list(map(look, ids))
        start = look('5438483250')
        end = look('355160396')
        result = Dijkstra2.dijkstra2(self.graph, start, end)
        print("Expected: ", expected)
        print("Result: ", result)
        assert result == expected

    def test_Astar(self):
        look = lambda x: self.nodes[x]
        ids = ['5438483250', '365391638', '355160356', '355160359', '355160364', '355160367', '355160372', '355160375',
               '355160380', '371215991', '4684142794', '355160393', '355160396']
        expected = list(map(look, ids))
        start = look('5438483250')
        end = look('355160396')
        result = astar(self.graph, start, end, lat=self.lat, lon=self.lon)
        print("Expected: ", expected)
        print("Result: ", result)
        assert result == expected

    @profile()
    def test_plot(self):
        compressed_nodes, _, deleted_nodes, graph = compress(self.nodes, self.graph)
        plotter = GraphPlotterClass(graph)
        plotter.plotgraph(self.lat, self.lon, self.ways, compressed_nodes, deleted_nodes)

    def test_pickle_unpickle(self):
        WriteToFile.writeTo(self.pickle_graph, self.graph)
        unpickle = WriteToFile.readFrom(self.pickle_graph)
        start = self.nodes['5438483250']
        end = self.nodes['365391638']
        Dijkstra.dijkstra(unpickle, start, end)

    def test_pickle_plot(self):
        lat = "picklefiles/testlat.txt"
        lon = "picklefiles/testlon.txt"
        ways = "picklefiles/testways.txt"
        nodes = "picklefiles/testnodes.txt"

        WriteToFile.writeTo(lat, self.lat)
        WriteToFile.writeTo(lon, self.lon)
        WriteToFile.writeTo(ways, self.ways)
        WriteToFile.writeTo(nodes, self.nodes)

        unpickle_lat = WriteToFile.readFrom(lat)
        unpickle_lon = WriteToFile.readFrom(lon)
        unpickle_ways = WriteToFile.readFrom(ways)
        unpickle_nodes = WriteToFile.readFrom(nodes)
        plotter = GraphPlotterClass(self.pickle_graph)
        plotter.plotgraph(unpickle_lat, unpickle_lon, unpickle_ways, unpickle_nodes, [])
        plotter = GraphPlotterClass(self.pickle_graph)
        plotter.plotgraph(self.lat, self.lon, self.ways, self.nodes, [])

    def test_shortest_path(self):
        expected = list(map(self.look, ['5438483250', '365391638', '355160356', '355160359', '355160364', '355160367',
                                        '355160372', '355160375', '355160380', '371215991', '4684142794', '355160393',
                                        '355160396']))
        WriteToFile.writeTo(self.pickle_graph, self.graph)
        start = self.look('5438483250')
        end = self.look('355160396')
        result = self.shortest_path.compute_shortest_path(self.lat, self.lon, self.pickle_graph,
                                                          self.lat[start], self.lon[start],
                                                          self.lat[end], self.lon[end])
        assert expected == result

    def test_plot_of_shortest_path(self):
        start = self.look('5438483250')
        end = self.look('355160396')
        shortest_path = self.shortest_path.compute_shortest_path(self.lat, self.lon, self.pickle_graph,
                                                                 self.lat[start], self.lon[start],
                                                                 self.lat[end], self.lon[end])
        #gplotter = GraphPlotterClass()
        # TODO: Plot the optimal path.


    def test_plotter_with_shortest_path(self):
        #katrinebjerg = '../Ressources/katrinebjerg.txt'
        #katrine_graph, katrine_edges = MyOSMParser.parseOSMFile('../Ressources/katrinebjerg.osm')
        #WriteToFile.writeTo(katrinebjerg, katrine_graph)
        #gplotter = GraphPlotter()
        #gplotter.pickle_file = katrinebjerg
        #gplotter.plotGraph(katrine_graph, katrine_edges)
        pass


    # Auxiliary functions
    def look(self, x):
        return self.nodes[x]

if __name__ == '__main__':
    unittest.main()
