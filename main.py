import random
import time
import networkx as nx
import matplotlib.pyplot as plt
from Python import MyOSMParser
from Python.Dijkstra import dijkstra
from Python.Dijkstra2 import dijkstra2
from Python.Astar import astar
from Python.ALTalgorithm import farthest_landmark_selection_1, ALT, farthest_landmark_selection_2
from Python.EdgeCompression import compress
from Python.MyOSMParser import archaversine
from Python.NetworkPlotter import NetworkPlotterClass
from Python.QuadTree import build_quad_tree
from Python.WriteToFile import writeTo, readFrom
from sys import getsizeof
import cProfile

def plot_visited(graph, visited, lat_arr, lon_arr, G, ax):
    all_nodes = [i for i in range(len(graph))]
    pos = dict()
    for u in all_nodes:
        neigbors = graph[u]
        lon = lon_arr[u]
        lat = lat_arr[u]
        pos[u] = (lon, lat)
        for v in range(0, len(neigbors), 2):
            G.add_edge(u, neigbors[v])

    nx.draw(G, pos=pos, ax=ax, node_size=0)
    nodes_to_color = list(visited)
    node_colors = ['green' if node in nodes_to_color else 'none' for node in G.nodes()]
    node_sizes = [1 if node in nodes_to_color else 0 for node in G.nodes()]
    nx.draw_networkx_nodes(G, pos=pos, node_color=node_colors, node_size=node_sizes)
    # plt.show()

def plot_graph(graph, lat_arr, lon_arr, G, ax):
    all_nodes = [i for i in range(len(graph))]
    pos = dict()
    for u in all_nodes:
        neigbors = graph[u]
        lon = lon_arr[u]
        lat = lat_arr[u]
        pos[u] = (lon, lat)
        for v in range(0, len(neigbors), 2):
            G.add_edge(u, neigbors[v])

    nx.draw(G, pos=pos, ax=ax, node_size=0)
    # plt.show(block=False)

def _plot_path(path, lat_arr, lon_arr, color):
    lats = []
    lons = []
    for i in range(len(path)):
        lats.append(lat_arr[path[i]])
        lons.append(lon_arr[path[i]])
    plt.plot(lons, lats, f'{color}-')

def run_and_plot(graph, path, visited, lat_arr, lon_arr):
    fig, ax = plt.subplots()
    G = nx.Graph()
    plot_graph(graph, lat_arr, lon_arr, G, ax)
    plt.show(block=False)
    plot_visited(graph, visited, lat_arr, lon_arr, G, ax)
    plt.draw()
    _plot_path(path, lat_arr, lon_arr)
    plt.draw()
    plt.pause(0.001)
    plt.show()

def sanity_check(k=4, interpretation=1):
    lat = readFrom('../Ressources/Denmark/DK_lat.txt')
    lon = readFrom('../Ressources/Denmark/DK_lon.txt')
    QT = readFrom('../Ressources/Denmark/DK_QT.txt')
    graph = readFrom('../Ressources/Denmark/DK_graph.txt')
    landmarks = readFrom(f'../Ressources/Denmark/DK_landmarks_{k}_{interpretation}.txt')
    from_landmark = readFrom(f'../Ressources/Denmark/DK_from_L_{k}_{interpretation}.txt')
    to_landmark = readFrom(f'../Ressources/Denmark/DK_to_L_{k}_{interpretation}.txt')

    print('No. of landmarks', len(landmarks))

    data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark, 'QuadTree': QT}
    i = 0
    while(True):
        start = random.randrange(len(graph))
        end = random.randrange(len(graph))
        print('Start node: ', start)
        print('End node: ', end)

        t0 = time.time()
        dijkstra_res = dijkstra2(graph, start, end)
        t1 = time.time()
        print("Dijkstra run time: ", t1 - t0)
        print()

        # _, bi_res = bidirectional_dijkstra(graph, start, end)
        # bi_res = new_bidirectional_dijkstra(graph, start, end)
        t2 = time.time()
        # print("Bidirectional Dijkstra run time: ", t2 - t1)
        # print()

        astar_res, _ = astar(graph, start, end, lat=lat, lon=lon)
        t3 = time.time()
        print("A* run time: ", t3 - t2)
        print()

        alt_res = ALT(graph, start, end, data=data)
        t4 = time.time()
        print("ALT run time: ", t4 - t3)
        print()

        # isSame1 = compare_paths(dijkstra_res, bi_res, 'Dijkstra vs Bidirectinal Dijkstra')
        isSame2 = compare_paths(dijkstra_res, astar_res, 'Dijkstra vs A*')
        isSame3 = compare_paths(dijkstra_res, alt_res, 'Dijkstra vs ALT')
        print_path_comparisons([isSame2, isSame3])

        print('Done with iteration ', i)
        print()
        i += 1

"""
:param k: The number of landmarks to preprocess for ALT.
"""
def preprocess_DK(k):
    t0 = time.time()
    nodes, lat, lon, graph, ways = MyOSMParser.parseOSMFile('../Ressources/denmark.osm.bz2', bz=True)
    t1 = time.time()
    print('Parsing time: ', t1-t0)
    writeTo('../Ressources/Denmark/DK_nodes.txt', nodes)
    writeTo('../Ressources/Denmark/DK_lat.txt', lat)
    writeTo('../Ressources/Denmark/DK_lon.txt', lon)
    writeTo('../Ressources/Denmark/DK_graph.txt', graph)
    writeTo('../Ressources/Denmark/DK_ways.txt', ways)
    t2 = time.time()
    print('Pickling time: ', t2-t1)
    QT = build_quad_tree(lat, lon, m=200)
    t3 = time.time()
    print('QuadTree build time: ', t3-t2)
    writeTo('../Ressources/Denmark/DK_QT.txt', QT)
    center_point = QT.center_point(lat, lon)
    landmarks, from_landmark, to_landmark = farthest_landmark_selection_2(center_point, graph, k=k)
    t4 = time.time()
    print('Landmark preprocessing time: ', t4-t3)
    writeTo(f'../Ressources/Denmark/DK_landmarks_{k}.txt', landmarks)
    writeTo(f'../Ressources/Denmark/DK_from_L_{k}.txt', from_landmark)
    writeTo(f'../Ressources/Denmark/DK_to_L_{k}.txt', to_landmark)
    data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark, 'QuadTree': QT}
    writeTo('../Ressources/Denmark/DK_data.txt', data)
    t5 = time.time()
    print('Total preprocessing time: ', t5-t0)

def make_landmarks(k):
    graph = readFrom('../Ressources/Denmark/DK_graph.txt')
    lat = readFrom('../Ressources/Denmark/DK_lat.txt')
    lon = readFrom('../Ressources/Denmark/DK_lon.txt')
    QT = readFrom('../Ressources/Denmark/DK_QT.txt')
    center_point = QT.center_point(lat, lon)
    landmarks, from_landmark, to_landmark = farthest_landmark_selection_1(center_point, graph, k=k)
    # The last digit of the landmark picklefiles denotes if we use the first interpretation of being the farthest
    # landmark or the other interpretation.
    # 1: Farthest means to have the largest sum of distances to the current set of landmarks
    # 2: Farthest means to have the farthest distance to the closest landmark
    writeTo(f'../Ressources/Denmark/DK_landmarks_{k}_1.txt', landmarks)
    writeTo(f'../Ressources/Denmark/DK_from_L_{k}_1.txt', from_landmark)
    writeTo(f'../Ressources/Denmark/DK_to_L_{k}_1.txt', to_landmark)

def compare_all_algorithms():
    lat = readFrom('../Ressources/Denmark/DK_lat.txt')
    lon = readFrom('../Ressources/Denmark/DK_lon.txt')
    QT = readFrom('../Ressources/Denmark/DK_QT.txt')
    graph = readFrom('../Ressources/Denmark/DK_graph.txt')
    start = QT.root_search_node(57.04, 9.92, lat, lon)
    end = QT.root_search_node(55.49, 9.47, lat, lon)
    print('Start node: ', start)
    print('End node: ', end)
    print('Total number of nodes in graph: ', len(graph))
    # profiler = cProfile.Profile()

    t0 = time.time()
    dijkstra2(graph, start, end)
    t1 = time.time()
    print("Dijkstra run time: ", t1 - t0)
    print()

    # bi_res = new_bidirectional_dijkstra(graph, start, end)
    t2 = time.time()
    # print("Bidirectional Dijkstra run time: ", t2 - t1)
    # print()

    astar(graph, start, end, lat=lat, lon=lon)
    t3 = time.time()
    print("A* run time: ", t3 - t2)
    print()

    # landmarks = readFrom('../Ressources/Denmark/DK_landmarks_8_1.txt')
    # from_landmark = readFrom('../Ressources/Denmark/DK_from_L_8_1.txt')
    # to_landmark = readFrom('../Ressources/Denmark/DK_to_L_8_1.txt')
    # data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark}

    # alt2_res = ALT(graph, start, end, data=data)
    # t4 = time.time()
    # print(f"ALT {len(landmarks)} run time: ", t4 - t3)
    # print()

    landmarks = readFrom('../Ressources/Denmark/DK_landmarks_4_1.txt')
    from_landmark = readFrom('../Ressources/Denmark/DK_from_L_4_1.txt')
    to_landmark = readFrom('../Ressources/Denmark/DK_to_L_4_1.txt')
    data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark}

    t5 = time.time()
    ALT(graph, start, end, data=data)
    t6 = time.time()
    print(f"ALT {len(landmarks)} run time: ", t6 - t5)
    print()

    landmarks = readFrom('../Ressources/Denmark/DK_landmarks_8_1.txt')
    from_landmark = readFrom('../Ressources/Denmark/DK_from_L_8_1.txt')
    to_landmark = readFrom('../Ressources/Denmark/DK_to_L_8_1.txt')
    data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark}

    t7 = time.time()
    # profiler.enable()
    ALT(graph, start, end, data=data)
    # profiler.disable()
    # profiler.print_stats()
    t8 = time.time()
    print(f"ALT {len(landmarks)} run time: ", t8 - t7)
    print()

    landmarks = readFrom('../Ressources/Denmark/DK_landmarks_16_1.txt')
    from_landmark = readFrom('../Ressources/Denmark/DK_from_L_16_1.txt')
    to_landmark = readFrom('../Ressources/Denmark/DK_to_L_16_1.txt')
    data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark}

    t9 = time.time()
    ALT(graph, start, end, data=data)
    t10 = time.time()
    print(f"ALT {len(landmarks)} run time: ", t10 - t9)
    print()

    # isSame1 = compare_paths(dijkstra_res, bi_res, 'Dijkstra vs Bidirectinal Dijkstra')
    # isSame2 = compare_paths(dijkstra_res, astar_res, 'Dijkstra vs A*')
    # isSame3 = compare_paths(dijkstra_res, alt2_res, 'Dijkstra vs ALT 2')
    # isSame4 = compare_paths(dijkstra_res, alt4_res, 'Dijkstra vs ALT 4')
    # isSame5 = compare_paths(dijkstra_res, alt8_res, 'Dijkstra vs ALT 8')
    # isSame6 = compare_paths(dijkstra_res, alt16_res, 'Dijkstra vs ALT 16')
    # print_path_comparisons([isSame2, isSame4, isSame5, isSame6])

    # G = nx.Graph()
    # _, ax = plt.subplots()
    #
    # plot_graph(graph, lat, lon, G, ax)
    # _plot_path(alt_res, lat, lon)
    # plt.draw()
    # plt.pause(0.001)
    # plt.show()


def compare_paths(p1, p2, msg):
    check = p1 == p2
    print(msg + ': ', check)
    return check

def print_path_comparisons(booleans):
    if not all(booleans):
        raise ValueError('Some of the algorithms return different paths')
    print("All three give the same shortest path: ", all(booleans))

def dijkstra_vs_heap():
    lat = readFrom('../Ressources/Denmark/DK_lat.txt')
    lon = readFrom('../Ressources/Denmark/DK_lon.txt')
    graph = readFrom('../Ressources/Denmark/DK_graph.txt')
    QT = readFrom('../Ressources/Denmark/DK_QT.txt')
    start = QT.root_search_node(55.04, 9.42, lat, lon)
    end = QT.root_search_node(55.25, 9.49, lat, lon)
    t0 = time.time()
    res1 = dijkstra(graph, start, end)
    t1 = time.time()
    print('Dijkstra runtime: ', t1 - t0)
    res2 = dijkstra2(graph, start, end)
    t2 = time.time()
    print('Dijkstra2 runtime: ', t2 - t1)

    compare_paths(res1, res2, 'Dijkstra vs Dijkstra2')


def run_and_compare(alg1, alg1_name, alg2, alg2_name, s_lat, s_lon, t_lat, t_lon):
    lat = readFrom('../Ressources/Denmark/DK_lat.txt')
    lon = readFrom('../Ressources/Denmark/DK_lon.txt')
    graph = readFrom('../Ressources/Denmark/DK_graph.txt')
    QT = readFrom('../Ressources/Denmark/DK_QT.txt')
    landmarks = readFrom('../Ressources/Denmark/DK_landmarks_4_1.txt')
    from_landmark = readFrom('../Ressources/Denmark/DK_from_L_4_1.txt')
    to_landmark = readFrom('../Ressources/Denmark/DK_to_L_4_1.txt')
    start = QT.root_search_node(s_lat, s_lon, lat, lon)
    end = QT.root_search_node(t_lat, t_lon, lat, lon)
    data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark}


    t0 = time.time()
    res1 = alg1(graph, start, end, lat=lat, lon=lon, data=data)
    t1 = time.time()
    print(f'{alg1_name} time: {t1-t0}')
    res2 = alg2(graph, start, end, lat=lat, lon=lon, data=data)
    t2 = time.time()
    print(f'{alg2_name} time: {t2-t1}')

    compare_paths(res1, res2, f'{alg1_name} vs {alg2_name}')

def landmark1_vs_landmark2(k):
    lat = readFrom('../Ressources/Denmark/DK_lat.txt')
    lon = readFrom('../Ressources/Denmark/DK_lon.txt')
    graph = readFrom('../Ressources/Denmark/DK_graph.txt')
    QT = readFrom('../Ressources/Denmark/DK_QT.txt')
    start = QT.root_search_node(55.04, 9.42, lat, lon)
    end = QT.root_search_node(55.25, 9.49, lat, lon)

    landmarks1 = readFrom(f'../Ressources/Denmark/DK_landmarks_{k}_1.txt')
    from_landmark1 = readFrom(f'../Ressources/Denmark/DK_from_L_{k}_1.txt')
    to_landmark1 = readFrom(f'../Ressources/Denmark/DK_to_L_{k}_1.txt')
    print('No of landmarks1: ', len(landmarks1))

    t0 = time.time()
    res1 = ALT(graph, start, end, lat=lat, lon=lon, data={'landmarks': landmarks1, 'from_landmark': from_landmark1,
                                                          'to_landmark': to_landmark1})
    t1 = time.time()
    print('Time of ALT with interpretation 1: ', t1 - t0)
    print()

    landmarks2 = readFrom(f'../Ressources/Denmark/DK_landmarks_{k}_2.txt')
    from_landmark2 = readFrom(f'../Ressources/Denmark/DK_from_L_{k}_2.txt')
    to_landmark2 = readFrom(f'../Ressources/Denmark/DK_to_L_{k}_2.txt')
    print('No of landmarks2: ', len(landmarks2))

    t2 = time.time()
    res2 = ALT(graph, start, end, lat=lat, lon=lon, data={'landmarks': landmarks2, 'from_landmark': from_landmark2,
                                                          'to_landmark': to_landmark2})
    t3 = time.time()
    print('Time of ALT with interpretation 2: ', t3 - t2)
    print()

    compare_paths(res1, res2, 'Two ALT with different way of computing landmarks give the same path')



if __name__ == '__main__':
     # make_landmarks(8)
     # make_landmarks(16)
    # preprocess_DK(4)
     # lat = readFrom('../Ressources/Denmark/DK_lat.txt')
     # lon = readFrom('../Ressources/Denmark/DK_lon.txt')
     # graph = readFrom('../Ressources/Denmark/DK_graph.txt')
     # QT = readFrom('../Ressources/Denmark/DK_QT.txt')
     # start = QT.root_search_node(57.72, 10.58, lat, lon)
     # end = QT.root_search_node(55.67, 12.57, lat, lon)
     compare_all_algorithms()
    # landmarks = readFrom('../Ressources/Denmark/DK_landmarks_8_1.txt')
    # from_landmark = readFrom('../Ressources/Denmark/DK_from_L_8_1.txt')
    # to_landmark = readFrom('../Ressources/Denmark/DK_to_L_8_1.txt')
    # data = {'landmarks': landmarks, 'from_landmark': from_landmark, 'to_landmark': to_landmark}
    # t0 = time.time()
    # ALT(graph, start, end, lat=lat, lon=lon, data=data)
    # t1 = time.time()
    # print('Time of algorithm: ', t1 - t0)

