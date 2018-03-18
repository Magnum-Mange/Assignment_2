import helpers

class Node:
    """
        A node contains a list of pointers to its neighbors, its degree, a unique index and the coordinates of its point.
    """

    def __init__(self,type="NULL",position_x=0,position_y=0,index=-1):
        self.neighbors = []
        self.degree = 0
        self.type = type
        self.position = [position_x,position_y]
        self.index = index

    def __eq__(self, other):
        """
            Defines equality between nodes as equality of indices.
        """
        if (isinstance(other,self.__class__)):
            return self.index == other.index
        return False



class Graph(object):
    """Class describing a graph as a collection of nodes"""
    def __init__(self, number_of_agents=1):
        self.nodes = {}
        self.corners = []
        self.size = 0
        self.poi = []
        self.starts = [0]*number_of_agents
        self.goals = [0]*number_of_agents


    def add_node(self,n):
        """
            Adds the node n to the graph, increasing the node count by one
        """
        self.nodes[n.index] = n
        self.size += 1
        if (n.type == "POI"):
            self.poi.append(n.index)
        elif(n.type == "CORNER"):
            self.corners.append(n.index)

    def get_node(self,ind):
        return self.nodes[ind]

    def update_visibility(self,m):
        """
            Method creating neighbor lists based on the map m.
        """
        for n in self.nodes.values():
            for n2 in self.nodes.values():
                if n2 != n:
                    if (m.is_visible(n,n2)):
                        n.neighbors.append(n2.index)

    def shortest_path(self,n1):
        """
            Uses Djikstra to compute the shortest path from n1 to n2.
            n1 and n2 are indices.
            returns indices
        """
        explored = {n1:0} # all explored nodes so far, with the cost as values
        paths = {n1:[n1]} # associates nodes to the path to get there
        from_explored = {n1:0} # nodes to explore from
        #while (not n2 in explored.keys()):
        while (len(from_explored.keys()) > 0):
            new_explored = {}
            for n in from_explored.keys():
                for next in self.get_node(n).neighbors:
                    cost = helpers.euclid_distance(self.get_node(n).position,self.get_node(next).position) + explored[n]
                    if next in explored.keys():
                        if cost < explored[next]: # if we've already seen the node, but with higher cost
                            paths[next] = paths[n] + [next]
                            explored[next] = cost
                            if (self.get_node(next).type == 'CORNER'):
                                new_explored[next] = cost
                    else: # if the node has not been seen before
                        explored[next] = cost
                        paths[next] = paths[n] + [next]
                        if (self.get_node(next).type == 'CORNER'):
                                new_explored[next] = cost
            from_explored = new_explored
        return paths

    def compute_shorts(self):
        ret = {}
        for n in self.nodes.keys():
            print(n)
            ret[n] = self.shortest_path(n)
        self.shorts = ret

    def quick_path(self,n1,n2):
        return self.shorts[n1][n2]



