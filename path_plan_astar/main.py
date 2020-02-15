import numpy as np

def find_path(the_map):
    """
    Finds a path from start and end position on the map
    """
    # If map is absent
    if the_map.size == 0: return None
    # Create PathPlan object for path finding
    p = PathPlan(the_map)
    # Get path from start to end
    path = p.find_path()
    return path

class Node:
    def __init__(self, coords):
        """
        Object to store parent and distance information for path calculation
        """
        self.coords = coords
        self.root = None
        self.h = 0
        self.g = 0
        self.f = self.h + self.g

class PathPlan:
    def __init__(self, the_map):
        """
        Object that finds path from start to end
        """
        self.the_map = the_map
        self.r, self.c = np.shape(the_map)
        self.start, self.end = self._find_start_and_end()       
        self.direction = None
        # representaions of obstacle and drivable space on the map
        self.obstacle = 'X'

    def _find_start_and_end(self):
        # Gets the co-ordinates for start and end position from map
        start = None
        end = None
        for i in range(self.r):
            for j in range(self.c):
                if self.the_map[i,j] == 'S':
                    # Since X and Y co-ordinate are indices of column and row respectively
                    start = (i,j)
                if self.the_map[i,j] == 'E':
                    # Since X and Y co-ordinate are indices of column and row respectively
                    end = (i,j)
                if start and end:
                    # start and end position found
                    break
        return start, end

    def find_path(self):
        # Check the direction of start to the end
        self.direction = self._get_direction()
        # Check edge cases first
        edge_case_found = self._check_edge_cases()
        if edge_case_found:
            return None
        # Find path from start to end
        path = self._search_path()
        return path

    def _check_edge_cases(self):
        # start and end are separate
        if self.direction == -1:
            # end is to the left of start
            # Swap the start end pairs  
            # but dont forget to swap the answers at the end
            self.start, self.end = self.end, self.start
        if not self._is_reachable(self.start[0], self.start[1]): 
            # start is blocked by obstacles on all four sides
            return True
        if not self._is_reachable(self.end[0], self.end[1]): 
            # end is blocked by obstacles on all four sides
            return True
        return False

    def _get_direction(self):
        # end is to the left of start return -1 else 1
        return -1 if self.start[1] > self.end[1] else 1

    def _is_reachable(self, i, j):
        check = self._is_position_valid(i+1, j) + self._is_position_valid(i-1, j) + self._is_position_valid(i, j+1) + self._is_position_valid(i, j-1)
        # The position is reachable if any of the four side around it is open
        return True if check>0 else False

    def _is_position_valid(self, i, j):
        if i < 0 or i > self.r-1 or j < 0 or j > self.c-1 or self.the_map[i,j] == self.obstacle:
            # Either position is outside the map or the position contains an obstacle
            return 0
        return 1

    def _convert_to_node(self, coords):
        # converts co-ordinates to Node object
        return Node(coords)

    def _get_children(self, root):
        # Get children from four directions
        child_up, child_down, child_right, child_left = (root[0]-1, root[1]), (root[0]+1, root[1]), (root[0], root[1]+1), (root[0], root[1]-1)
        return [child_up, child_down, child_right, child_left]

    def _euclidean_distance_squared(self, child, goal):
        # Calculates Euclidean squared distance
        dist = ((goal[0] - child[0])**2)+((goal[1] - child[1])**2)
        return dist

    def _search_path(self):
        # Searches for path from start to end
        openset = set()
        closedset = set()
        openlist = []
        # Get start point and convert to node
        current = self._convert_to_node(self.start)
        # get end point and convert to node
        end_node = self._convert_to_node(self.end)
        # Add current to openset
        openset.add(current.coords)
        openlist.append(current)
        # iterate through the the_map
        while openset:
            # Find the item in openset with minimum f
            current = min(openlist, key=lambda x: x.g + x.h)
            # check if the current is the end
            if current.coords == end_node.coords:
                path = []
                while current.root:
                    # Co-ordinates should be flipped as indices of 
                    # column in 'x' coordinate and indices of row
                    # is 'y' coordinate
                    path.append((current.coords[1], current.coords[0]))
                    current = current.root
                path.append((current.coords[1], current.coords[0]))
                if self.direction == 1:
                    # Backtracking causes the path to be reversed
                    # Hence reverse the list to get the correct path
                    path = path[::-1]
                # if direction == -1 the path is right as we have exchanged
                # start and end
                return path
            # remove the item from openset
            openset.remove(current.coords)
            openlist.pop(openlist.index(current))
            # add current to closed set
            closedset.add(current.coords)
            # get children
            children = self._get_children(current.coords)
            for child in children:
                check = self._is_position_valid(child[0], child[1])
                # check if child is valid
                if check == 1 and child not in closedset:
                    # convert child to node
                    child_node = self._convert_to_node(child)
                    if child_node.coords in openset:
                        # If child is already in openset check if it beat the g score
                        gnew = current.g + self._euclidean_distance_squared(child_node.coords, current.coords)
                        if child_node.g > gnew:
                            # update node to have new root if current g is better than previous
                            child_node.g = gnew
                            child_node.f = child_node.g + child_node.h
                            child_node.root = current
                            openlist.pop(openlist.index(child_node))
                            openlist.append(child_node)
                    else:
                        # If not in openset
                        child_node.g = current.g + self._euclidean_distance_squared(child_node.coords, current.coords)
                        child_node.h = self._euclidean_distance_squared(child_node.coords, end_node.coords)
                        child_node.f = child_node.g + child_node.h
                        # set root to current item
                        child_node.root = current
                        # add it to openset
                        openset.add(child_node.coords)
                        openlist.append(child_node)
        return None

if __name__ == "__main__":
    # Example to understand the usage of the function
    the_map = np.array([['S', 'X', '', '', '', 'X', 'X', 'E', '', 'X'],
                         ['', 'X', 'X', '', '', 'X', 'X', '', 'X', 'X'],
                         ['', 'X', 'X', '', '', '', 'X', '', '', 'X'],
                         ['', '', 'X', '', '', 'X', 'X', '', 'X', 'X'],
                         ['', '', '', '', 'X', 'X', '', '', '', ''],
                         ['', '', '', '', 'X', 'X', '', '', '', ''],
                         ['X', '', '', '', '', '', '', '', '', ''],
                         ['X', 'X', 'X', '', '', '', '', '', 'X', 'X']])
    print(find_path(the_map))