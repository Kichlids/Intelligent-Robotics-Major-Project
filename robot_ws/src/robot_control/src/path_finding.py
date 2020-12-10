# Reference: https://www.annytab.com/a-star-search-algorithm-in-python/

# This class represent a graph
class Graph:
    # Initialize the class
    def __init__(self, graph_dict=None, directed=True):
        self.graph_dict = graph_dict or {}
        self.directed = directed
        if not directed:
            self.make_undirected()
    
    # Create an undirected graph by adding symmetric edges
    def make_undirected(self):
        for a in list(self.graph_dict.keys()):
            for (b, dist) in self.graph_dict[a].items():
                self.graph_dict.setdefault(b, {})[a] = dist
                
    # Add a link from A and B of given distance, and also add the inverse link if the graph is undirected
    def connect(self, A, B, distance=1):
        self.graph_dict.setdefault(A, {})[B] = distance
        if not self.directed:
            self.graph_dict.setdefault(B, {})[A] = distance
            
    # Get neighbors or a neighbor
    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)
            
    # Return a list of nodes in the graph
    def nodes(self):
        s1 = set([k for k in self.graph_dict.keys()])
        s2 = set([k2 for v in self.graph_dict.values() for k2, v2 in v.items()])
        nodes = s1.union(s2)
        return list(nodes)
        
# This class represent a node
class Node:
    # Initialize the class
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.g = 0 # Distance to start node
        self.h = 0 # Distance to goal node
        self.f = 0 # Total cost
    # Compare nodes
    def __eq__(self, other):
        return self.name == other.name
    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.name, self.f))


class Astar:

    def __init__(self, nodes, graph):

        self.nodes = nodes
        self.graph = graph

        # Create heuristics (straight-line distance, air-travel distance)
        self.heuristics = {}

        #self.compute_heuristics('Node6')

        '''
        for key in self.heuristics:
            print(key + ': ' + str(self.heuristics.get(key)))
        '''
        
        # Run the search algorithm
        # path = self.astar_search(self.graph, self.heuristics, 'Node2', 'Node6')
        # print(path)
        # print()

    def compute_heuristics(self, end_node_name):

        x0 = self.nodes.get(end_node_name)[0]
        y0 = self.nodes.get(end_node_name)[1]
        
        # Construct heristics with air travel distance
        for node_name in self.nodes:
            x1 = self.nodes.get(node_name)[0]
            y1 = self.nodes.get(node_name)[1]

            dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5

            self.heuristics[node_name] = dist     

    # A* search
    def astar_search(self, start, end):

        self.compute_heuristics(end)
        
        # Create lists for open nodes and closed nodes
        open = []
        closed = []
        # Create a start node and an goal node
        start_node = Node(start, None)
        goal_node = Node(end, None)
        # Add the start node
        open.append(start_node)
        
        # Loop until the open list is empty
        while len(open) > 0:
            # Sort the open list to get the node with the lowest cost first
            open.sort()
            # Get the node with the lowest cost
            current_node = open.pop(0)
            # Add the current node to the closed list
            closed.append(current_node)
            
            # Check if we have reached the goal, return the path
            if current_node == goal_node:
                path = []
                while current_node != start_node:
                    path.append([current_node.name, current_node.g])
                    current_node = current_node.parent
                path.append([current_node.name, current_node.g])
                # Return reversed path
                return path[::-1], path[0][1]
            # Get neighbours
            neighbors = self.graph.get(current_node.name)
            # Loop neighbors
            for key, value in neighbors.items():
                # Create a neighbor node
                neighbor = Node(key, current_node)
                # Check if the neighbor is in the closed list
                if(neighbor in closed):
                    continue
                # Calculate full path cost
                neighbor.g = current_node.g + self.graph.get(current_node.name, neighbor.name)
                neighbor.h = self.heuristics.get(neighbor.name)
                neighbor.f = neighbor.g + neighbor.h
                # Check if neighbor is in open list and if it has a lower f value
                if(self.add_to_open(open, neighbor) == True):
                    # Everything is green, add neighbor to open list
                    open.append(neighbor)
        # Return None, no path is found
        return None
        

    # Check if a neighbor should be added to open list
    def add_to_open(self, open, neighbor):
        for node in open:
            if (neighbor == node and neighbor.f > node.f):
                return False
        return True
    
    # A* search but returns path and distance
    def astar(self, start, end):
        path, dist = self.astar_search(start, end)
        pure_path = []
        for i in range(len(path)):
            pure_path.append(path[i][0])
        
        return pure_path, dist

    # return tour path
    def find_tour_path(self, start, important_nodes):
        list_impt_nodes = important_nodes
        tour_path = []
        temp_path = []
        target_node = ''
        current_node = start
        total_dist = 0
        
        # keeps looping until all the important nodes have been removed from the list
        while list_impt_nodes:
            min_cost = 100
            # find the important node with the least distance/cost from the current node
            for node in list_impt_nodes:
                path, dist = self.astar(current_node, node)
                if min_cost > dist:
                    min_cost = dist
                    temp_path = path
                    target_node = node
                    #print(temp_path)
                    #print(min_cost)
            # add the path to important node with the least distance 
            tour_path = tour_path + temp_path
            tour_path.pop()
            current_node = target_node
            # remove it from the list
            list_impt_nodes.remove(target_node)
            total_dist = total_dist + min_cost
            # print(current_node)
            # print(tour_path)
            # print(min_cost)
            
        # find path from last important node to the start location to complete tour
        path, dist = self.astar(current_node, start)
        tour_path = tour_path + path
        total_dist = total_dist + dist
        
        return tour_path, total_dist

