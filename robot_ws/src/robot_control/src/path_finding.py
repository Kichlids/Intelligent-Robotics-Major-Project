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

    def __init__(self):
        # Construct nodes
        self.nodes = {}
        self.nodes['Node1'] = [0, 0]
        self.nodes['Node2'] = [0, 10]
        self.nodes['Node3'] = [20, 10]
        self.nodes['Node4'] = [5, 0]
        self.nodes['Node5'] = [5, 5]
        self.nodes['Node6'] = [20, 5]

        # Create a graph and conections between nodes using actual distance
        self.graph = Graph()
        self.graph.connect('Node1', 'Node2', 10)
        self.graph.connect('Node1', 'Node4', 5)
        self.graph.connect('Node2', 'Node3', 20)
        self.graph.connect('Node4', 'Node5', 5)
        self.graph.connect('Node5', 'Node6', 15)
        self.graph.connect('Node3', 'Node6', 5)

        # Make graph undirected, create symmetric connections (A->B == B->A)
        self.graph.make_undirected()
        # Create heuristics (straight-line distance, air-travel distance)
        self.heuristics = {}

        self.compute_heuristics('Node6')

        '''
        for key in self.heuristics:
            print(key + ': ' + str(self.heuristics.get(key)))
        '''
        
        # Run the search algorithm
        path = self.astar_search(self.graph, self.heuristics, 'Node2', 'Node6')
        print(path)
        print()

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
    def astar_search(self, graph, heuristics, start, end):
        
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
                    path.append(current_node.name + ': ' + str(current_node.g))
                    current_node = current_node.parent
                path.append(start_node.name + ': ' + str(start_node.g))
                # Return reversed path
                return path[::-1]
            # Get neighbours
            neighbors = graph.get(current_node.name)
            # Loop neighbors
            for key, value in neighbors.items():
                # Create a neighbor node
                neighbor = Node(key, current_node)
                # Check if the neighbor is in the closed list
                if(neighbor in closed):
                    continue
                # Calculate full path cost
                neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
                neighbor.h = heuristics.get(neighbor.name)
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

astar = Astar()
