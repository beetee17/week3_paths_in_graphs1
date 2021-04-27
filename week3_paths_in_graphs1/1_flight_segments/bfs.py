#Uses python3

import sys
from collections import defaultdict
from queue import Queue

class Vertex():
    def __init__(self, index):
        self.index = index

        self.visited = 0

        self.pre = None
        self.post = None

        # for computing of shortest path tree
        self.dist = -1

        # use boolean to keep track of if vertex has been discovered instead of setting to infinity
        self.dist_defined = False

        # for computing layer of vertex in tree and retracing shortest path  
        self.prev = None


class Graph():
    def __init__(self, edges, vertices):

        # create a dict with vertex as key and list of its neighbours as values
        self.adj = defaultdict(list)

        self.edges = edges

        # for an udirected graph, b is adjacent to a and vice versa
        for (a, b) in edges:
            self.adj[vertices[a-1]].append(vertices[b-1])
            self.adj[vertices[b-1]].append(vertices[a-1])
                    
        self.vertices = vertices
   
        self.acyclic = True

        self.clock = 1
    
    def previst(self, v):
    
        v.pre = self.clock
        self.clock += 1
        
    def postvist(self, v):

        v.post = self.clock
        self.clock += 1

        v.visited = 1

    def explore(self, v):

        v.visited = -1
        # -1 indicates that the neighbours of this vertex is currently being explored in the recursion sub routines
        # pre-vist block
        self.previst(v)

        # explore each neighbour of the vertex 
        for neighbour in self.adj[v]:

            # if the neighbours point back to the original vertex, we have found a cycle (there is a series of edges which start from v and end at v)
            if neighbour.visited == -1:
                self.acyclic = False

            if neighbour.visited == 0:
                self.explore(neighbour)

        # post-visit block
        self.postvist(v)

    def make_shortest_path_tree(self, s):
        """compute the shortest path tree given the starting vertex's index"""

        s = vertices[s]

        # init starting vertex distance to 0
        s.dist = 0
        s.dist_defined = True

        # create queue for processing of neighbours
        Q = Queue(maxsize=2*len(self.edges) + 1)

        # init the first layer of the tree
        for neighbour in self.adj[s]:
            neighbour.prev = s
            Q.put_nowait(neighbour)

        # iterate until all reachable vertices have been discovered
        while not Q.empty():

            # dequeue
            v = Q.get_nowait()

            if not v.dist_defined:

                # set distance if it has not been previously discovered
                v.dist = v.prev.dist + 1
                v.dist_defined = True

                # enqueue all its undiscovred neighbours
                for neighbour in self.adj[v]:

                    # do not allow overwriting of prev
                    if not neighbour.prev:
                        neighbour.prev = v

                    Q.put_nowait(neighbour)


def distance(graph, s, t):
    """Given an undirected graph with 𝑛 vertices and 𝑚 edges and two vertices 𝑢 and 𝑣, compute the length of a shortest path between 𝑢 and 𝑣 (that is, the minimum number of edges in a path from 𝑢 to 𝑣)
    
    Output the minimum number of edges in a path from 𝑢 to 𝑣, or −1 if there is no path"""

    graph.make_shortest_path_tree(s)
    return graph.vertices[t].dist

if __name__ == '__main__':
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n, m = data[0:2]

    vertices = [Vertex(i) for i in range(n)]

    data = data[2:]
    edges = list(zip(data[0:(2 * m):2], data[1:(2 * m):2]))

    s, t = data[2 * m] - 1, data[2 * m + 1] - 1

    print(distance(Graph(edges, vertices), s, t))
