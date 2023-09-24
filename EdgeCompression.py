"""
nodes: dictionary that maps osm identifiers to index identifiers for our graph representation
outgoing: our graph, represented as an array of arrays
"""
import copy


def compress(nodes, outgoing):
    compressed_nodes = []
    list_of_nodes_to_removed = []
    outgoing = copy.deepcopy(outgoing)
    #print("BEFORE COMPRESSION:")
    #print("Nodes: ", nodes)

    #print()
    #print("AFTER COMPRESSION")

    for i in range(len(outgoing)):
        print(i)
        if len(outgoing[i]) == 4:
            list_of_nodes_to_removed.append(i)
            u = outgoing[i][0]
            v = outgoing[i][2]
            sum = outgoing[i][1] + outgoing[i][3]
            outu = outgoing[u]
            outv = outgoing[v]
            for j in range(0, len(outu), 2):
                if outu[j] == i:
                    outu[j] = v
                    outu[j+1] = sum
            for j in range(0, len(outv), 2):
                if outv[j] == i:
                    outv[j] = u
                    outv[j + 1] = sum

    #print("These nodes can be compressed:", list_of_nodes_to_removed)
    for key, value in nodes.items():
        print(value)
        if value in list_of_nodes_to_removed:
            compressed_nodes.append(key)

    for item in compressed_nodes:
        print('Third loop')
        del nodes[item]

    print("New nodes: ", nodes)

    """
    nodes: dictionary mapping original id to index id updated with fewer nodes. Compressed nodes have been removed.
    compressed_nodes: list of original id belonging to nodes that are compressed, i.e. removed
    list_of_nodes_to_removed: list of index id belonging to nodes that are compressed, i.e. removed
    outgoing: a modified graph. Suppose we have a path that goes u -> v -> w and we wish to contract v. To do so
        we delete v as a neighbor for u and w becomes the new neighbor for u, with a weight that is the sum of the 
        weights of (u,v) and (v,w). v is still able to go to w, but if we come via u, we will never visit v.
    """
    return nodes, compressed_nodes, list_of_nodes_to_removed, outgoing

if __name__ == "__main__":
    nodes = {'253267351': 0, '474397330': 1, '293403040': 2, '1018753950': 3, '1018753539': 4, '253267043': 5,
             '2416546': 6}
    out = [[1, 0, 5, 0, 6, 0], [0, 0, 2, 0], [1, 0, 3, 0], [2, 0, 4, 0], [0, 0, 3, 0], [0, 0, 6, 0], [5, 0]]
    _, comp, id, _ = compress(nodes, out)
    print(comp)
    print(id)
