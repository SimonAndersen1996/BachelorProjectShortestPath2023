import _pickle as pickle
from Python.Dijkstra import dijkstra

#example graph
exDict = {
    'a': {'b': 3, 'c': 4, 'd': 7},
    'b': {'c': 1, 'f': 5},
    'c': {'f': 6, 'd': 2},
    'd': {'e': 3, 'g': 6},
    'e': {'g': 3, 'h': 4},
    'f': {'e': 1, 'h': 8},
    'g': {'h': 2},
    'h': {}
}

def writeTo(filename, graph):
    with open(filename, 'wb') as file:
        pickleRick = pickle.dumps(graph)
        file.write(pickleRick)

def readFrom(filename):
    with open(filename, 'rb') as file:
        unpickle = pickle.load(file)
        return unpickle

def main():
    filename = 'pickledGraph.txt'
    print("exDict before turning into a pickle: ", exDict)
    print('commence pickling...')
    writeTo(filename, exDict)
    print('pickling complete... Commence unpickling')
    unpickle = readFrom(filename)
    print("exDict after turning into a pickle: ", unpickle)
    print()
    print('Is before and after the same? ', exDict==unpickle)


if __name__ == '__main__':
    main()