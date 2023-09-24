import GraphObjects

def saveGraph(nodes, name):
    f = open(name + ".txt", "w")
    for node in nodes:
        # TODO: What do we do at each iteration? Which format?
        f.write(node)
    pass

# TODO: How do we read from the file?
"""
Returns the array of nodes as Vertice objects.
"""
def loadGraph(name):
    pass