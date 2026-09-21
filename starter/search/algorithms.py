import heapq

class Dijkstra:
    def __init__(self, gridded_map):
        self.gridded_map = gridded_map  # a Map instance
        self.frontier: list[State] = []
        self.closed_list: dict[int, State] = {}

    def search(self, start, goal):
        self.frontier = []
        self.closed_list = {}
        current: State = start
        self.closed_list[current.state_hash()] = current

        while current != goal:
            # get children and heappush qualified to frontier (set cost and parent)
            children = self.gridded_map.successors(current)
            for child in children:
                if child.state_hash() not in self.closed_list:
                    child.set_cost(child.get_g())
                    child.set_parent(current)
                    heapq.heappush(self.frontier, child)

            # if frontier is empty, the problem has not solution
            if not self.frontier:
                return None, -1, len(self.closed_list)

            # pop child from frontier and check if it is already in the closed list; if yes, pop again
            current = heapq.heappop(self.frontier)
            while current.state_hash() in self.closed_list:
                if not self.frontier:
                    return None, -1, len(self.closed_list)
                current = heapq.heappop(self.frontier)

            # add current to closed list
            self.closed_list[current.state_hash()] = current

        # current == goal now
        path = []
        cost = current.get_cost()
        expanded = len(self.closed_list)
        while not(current.get_parent() is None):
            path.append(current)
            current = current.get_parent()

        # return path, cost, expanded
        return path.reverse(), cost, expanded

    def get_closed_data(self):
        return self.closed_list

class AStar:
    def __init__(self, gridded_map):
        self.gridded_map = gridded_map
        self.frontier: list[State] = []
        self.closed_list: dict[int, State] = {}

    def search(self, start, goal):
        g_x = goal.get_x()
        g_y = goal.get_y()

        self.frontier = []
        self.closed_list = {}
        current: State = start
        self.closed_list[current.state_hash()] = current

        while current != goal:
            # get children and heappush qualified to frontier (set cost and parent)
            children = self.gridded_map.successors(current)
            for child in children:
                if child.state_hash() not in self.closed_list:
                    d_x = abs(child.get_x() - g_x)
                    d_y = abs(child.get_y() - g_y)
                    h = 1.5 * min(d_x, d_y) + abs(d_x - d_y)
                    child.set_cost(child.get_g() + h)
                    child.set_parent(current)
                    heapq.heappush(self.frontier, child)

            # if frontier is empty, the problem has not solution
            if not self.frontier:
                return None, -1, len(self.closed_list)

            # pop child from frontier and check if it is already in the closed list; if yes, pop again
            current = heapq.heappop(self.frontier)
            while current.state_hash() in self.closed_list:
                if not self.frontier:
                    return None, -1, len(self.closed_list)
                current = heapq.heappop(self.frontier)

            # add current to closed list
            self.closed_list[current.state_hash()] = current

        # current == goal now
        path = []
        cost = current.get_cost()
        expanded = len(self.closed_list)
        while not(current.get_parent() is None):
            path.append(current)
            current = current.get_parent()

        # return path, cost, expanded
        return path.reverse(), cost, expanded

    def get_closed_data(self):
        return self.closed_list
    

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def set_parent(self, parent):
        """
        Sets the parent of a node in the search tree
        """
        self._parent = parent

    def get_parent(self):
        """
        Returns the parent of a node in the search tree
        """
        return self._parent
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost
    