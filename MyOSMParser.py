from Python import GraphObjects
from xml.etree.ElementTree import iterparse
import numpy as np
from Python import WhiteList
from Python.GraphPlotter import GraphPlotterClass
from Python.WriteToFile import writeTo, readFrom
from Python.EdgeCompression import compress
from Python.Dijkstra import dijkstra
import copy
import bz2
import time
import math

def parseOSMFile(file_name, bz=False):
    start = time.time()
    # Lists to keep track of the information on nodes / edges


    # First run through the ways
    graph, latitude, longitude, nodes, ways = parse_way_elements(bz, file_name) # Here we use the helper function parse_way_element
    loop1 = time.time()
    print("Done with first run through file, running through way elements")
    print("Time so far: ", loop1 - start)

    # Second run through the nodes
    latitude, longitude = parse_node_elements(bz, file_name, latitude, longitude, nodes) # Here we find lat and lon

    loop2 = time.time()
    print("Finished with second run through file, looking for nodes")
    print("Time so far: ", loop2 - start)

    # Computing weights with haversine
    print("Computing weight for edges ...")
    for i in range(len(graph)):
        neighbors = graph[i]
        print(f'Weight {i}')
        for j in range(0, len(neighbors), 2):
            dist = archaversine(i, neighbors[j], latitude, longitude)
            graph[i][j+1] = dist

    return nodes, latitude, longitude, graph, ways


def parse_node_elements(bz, file_name, latitude, longitude, nodes):
    with bz2.open(file_name, 'rb') if bz else open(file_name, 'rb') as file:
        it2 = iterparse(file, events=('start', 'end'))
        _, root2 = next(it2)
        for event, e in it2:
            if event == 'end' and e.tag == 'node' and e.attrib['id'] in nodes:
                latitude[nodes[e.attrib['id']]] = float(e.attrib['lat'])
                longitude[nodes[e.attrib['id']]] = float(e.attrib['lon'])
            root2.clear()  # allow garbage collection of parsed elements
    return latitude, longitude


def parse_way_elements(bz, file_name):
    longitude = []
    latitude = []
    nodes = {}
    graph = []
    ways = []

    with bz2.open(file_name, 'rb') if bz else open(file_name, 'rb') as file:
        # Counter for lists
        count = 0
        it1 = iterparse(file, events=('start', 'end'))
        _, root1 = next(it1)
        for event, e in it1:
            if event == 'end':
                if e.tag == 'way':  # Check if the element is a way
                    # print('Found a way element', e)
                    # Check if the type of way element is valid
                    if any(c.tag == 'tag' and c.attrib['k'] == 'highway' and c.attrib['v'] in WhiteList.whitelist for c
                           in e):
                        # print('Way element is valid')

                        # Collect the references in way element
                        refs = [c.attrib['ref'] for c in e if c.tag == 'nd']
                        ways.append(refs)
                        for ref in refs:
                            if nodes.get(ref) is None:
                                nodes[ref] = count
                                graph.append([])
                                latitude.append(0)
                                longitude.append(0)
                                count += 1

                        # The following stub assumes that the way element is one-way
                        if any(c.tag == 'tag' and c.attrib['k'] == 'oneway' and c.attrib['v'] == 'yes' for c in e):
                            prev_ref = refs[0]
                            for ref in refs[1:]:
                                # Start comment here
                                weight = 0
                                graph[nodes[prev_ref]].extend([nodes[ref], weight])
                                prev_ref = ref
                        # The following stub assumes that the way element is two-way

                        else:
                            prev_ref = refs[0]
                            for ref in refs[1:]:
                                weight = 0
                                graph[nodes[prev_ref]].extend([nodes[ref], weight])
                                graph[nodes[ref]].extend([nodes[prev_ref], weight])
                                prev_ref = ref
            root1.clear()
        file.close()
    return graph, latitude, longitude, nodes, ways


"""
    point1 and point2 are the two points on the map we wish to compute the distance of
    lat and lon are arrays of floats. Their indices correspond to the points on the map
"""
def archaversine(point1, point2, lat_arr, lon_arr):
    lat1, lon1 = lat_arr[point1], lon_arr[point1]
    lat2, lon2 = lat_arr[point2], lon_arr[point2]

    # distance between latitudes
    # and longitudes
    dLat = (lat2 - lat1) * math.pi / 180.0
    dLon = (lon2 - lon1) * math.pi / 180.0

    # convert to radians
    lat1 = (lat1) * math.pi / 180.0
    lat2 = (lat2) * math.pi / 180.0

    # apply formulae
    a = (pow(math.sin(dLat / 2), 2) +
         pow(math.sin(dLon / 2), 2) *
         math.cos(lat1) * math.cos(lat2))
    rad = 6371 # Radius of the Earth in km
    c = 2 * math.asin(math.sqrt(a))
    return rad * c

def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the earth in km
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(lat1)) \
        * math.cos(math.radians(lat2)) * math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = R * c  # Distance in km
    return d

def oldParseOSMFile(file_name, bz=False):
    start = time.time()
    graph_nodes = {}
    twoways = []
    # First run through the ways
    with bz2.open(file_name, 'rb') if bz else open(file_name, 'rb') as file:
        it1 = iterparse(file, events=('start', 'end'))
        _, root1 = next(it1)
        for event, e in it1:
            if event == 'end':
                if e.tag == 'way': # Check if the element is a way
                    whitelisted = any(c.tag == 'tag' and c.attrib['v'] in WhiteList.whitelist for c in e)
                    if any(c.tag == 'tag' and c.attrib['k'] == 'highway' and whitelisted for c in e):
                        # Collect the references in way element
                        refs = [c.attrib['ref'] for c in e if c.tag == 'nd']
                        # The following stub assumes that the way element is one-way
                        if any(c.tag == 'tag' and c.attrib['k'] == 'oneway' and c.attrib['v'] == 'yes' for c in e):
                            prev_ref = refs[0]
                            # A guard to avoid overwriting
                            isInGraph = prev_ref in graph_nodes
                            if not isInGraph:
                                graph_nodes[prev_ref] = GraphObjects.Vertice(prev_ref)
                            for ref in refs[1:]:
                                isInGraph = ref in graph_nodes
                                if not isInGraph:
                                    graph_nodes[ref] = GraphObjects.Vertice(ref)
                                edge = GraphObjects.Edge(prev_ref, ref)
                                graph_nodes[prev_ref].outgoing.append(edge)
                                graph_nodes[ref].ingoing.append(edge)
                                prev_ref = ref
                        # The following stub assumes that the way element is two-way
                        else:
                            # A guard to avoid overwriting
                            prev_ref = refs[0]
                            isInGraph = prev_ref in graph_nodes
                            if not isInGraph:
                                graph_nodes[prev_ref] = GraphObjects.Vertice(prev_ref)
                            for ref in refs[1:]:
                                isInGraph = ref in graph_nodes
                                if not isInGraph:
                                    graph_nodes[ref] = GraphObjects.Vertice(ref)
                                edge = GraphObjects.Edge(prev_ref, ref)
                                graph_nodes[prev_ref].outgoing.append(edge)
                                graph_nodes[ref].ingoing.append(edge)
                                prev_ref = ref
                                twoways.append(edge)
            root1.clear()
        file.close()
    loop1 = time.time()
    print("Done with first run through file, running through way elements")
    print("Time so far: ", loop1 - start)

    # Add the other direction for all twoway edges
    for edge in twoways:
        new_edge = GraphObjects.Edge(edge.to_node, edge.from_node)
        graph_nodes[edge.to_node].outgoing.append(new_edge)
        graph_nodes[edge.from_node].ingoing.append(new_edge)

    loop2 = time.time()
    print("Done making bidirectional edges")
    print("Time so far: ", loop2 - start)

    # Second run through the nodes
    with bz2.open(file_name, 'rb') if bz else open(file_name, 'rb') as file:
        it2 = iterparse(file, events=('start', 'end'))
        _, root2 = next(it2)
        for event, e in it2:
            if event == 'end' and e.tag == 'node' and e.attrib['id'] in graph_nodes:
                vertice = graph_nodes[e.attrib['id']]
                vertice.lat = float(e.attrib['lat'])
                vertice.lon = float(e.attrib['lon'])
            root2.clear()  # allow garbage collection of parsed elements
    # Compute the euclidean distance between nodes and their neighbors

    loop3 = time.time()
    print("Finished with second run through file, looking for nodes")
    print("Time so far: ", loop3 - start)

    # Run through all nodes in graph to compute length of edges
    for node in graph_nodes.values():
        for edge in node.outgoing:
            neighbor = graph_nodes[edge.to_node]
            edge.length = haversine(node.lat, node.lon, neighbor.lat, neighbor.lon)

    loop4 = time.time()
    print("Done with the final loop")
    print("Total time: ", loop4 - start)

    return graph_nodes


def parseDenmark():
    print('Parsing Denmark')
    start_parse = time.time()
    nodes, lat, lon, out, ways = parseOSMFile('../Ressources/denmark.osm.bz2', bz=True)
    end_parse = time.time()
    print("Running time of parse: ", end_parse - start_parse)
    print('Finished parsing\nSaving Denmark')
    writeTo('../Ressources/pickledDenmarkGraph.txt', out)
    writeTo('../Ressources/pickledDenmark.txt', nodes)
    writeTo('../Ressources/pickledDenmarkLat.txt', lat)
    writeTo('../Ressources/pickledDenmarkLon.txt', lon)
    writeTo('../Ressources/pickledDenmarkWays.txt', ways)
    # read_graph = readFrom('../Ressources/pickledDenmarkGraph.txt')
    _, _, to_remove, graph = compress(nodes, out)
    print('Plotting Denmark')
    plotter = GraphPlotterClass('../Ressources/pickledDenmarkGraph.txt')
    start = time.time()
    plotter.plotgraph(lat, lon, graph, ways, to_remove)
    end = time.time()
    print("Plotting time: ", end - start)

def parseViborgvej():
    print('Parsing Viborgvej')
    nodes, lat, lon, out, ways = parseOSMFile('../Ressources/viborgvej.osm')
    print('Finished parsing\nSaving Viborgvej')
    # print('Compressing edges ...')
    # newnodes, compressed_edges = compressor.compress(nodes, out)
    # print('Finished compressing edges')
    writeTo('../Ressources/pickledViborgvejEdges.txt', out)
    writeTo('../Ressources/pickledViborgvej.txt', nodes)
    read_graph = readFrom('../Ressources/pickledViborgvej.txt')
    read_edges = readFrom('../Ressources/pickledViborgvejEdges.txt')
    print('Plotting Viborgvej')
    start = time.time()
    end = time.time()
    print("Plotting time: ", end - start)
    print("Number of nodes: ", len(nodes))
    print("Nodes: ", nodes)
    print("length of lon: ", len(lon))
    print("Longitude list: ", lon)
    print("length of lat: ", len(lat))
    print("Latitude list: ", lat)
    print("length of out: ", len(out))
    print("Outgoing_edges list: ", out)

def parseSkjoldhoj():
    print('Parsing Skjoldhøj')
    nodes, lat, lon, graph, ways = parseOSMFile('../Ressources/skjoldhøj.osm')
    writeTo('../Ressources/pickledSkjoldhoj.txt', graph)
    old_nodes = copy.copy(nodes)
    print('Finished parsing\nSaving Skjoldhøj')
    newnodes, removed_id, removed_index, out1 = compress(nodes, graph)
    plotter = GraphPlotterClass('../Ressources/pickledSkjoldhoj.txt')
    print('List of removable stuff: ', removed_index)
    print('Plotting Skjoldhoj')
    start = time.time()
    plotter.plotgraph(lat, lon, ways, old_nodes, removed_index)
    end = time.time()


if __name__ == "__main__":
    parseDenmark()

