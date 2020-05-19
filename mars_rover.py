import queue
import operator

"""Consist of x and y co-ordinates of a particular location"""


class Node:

    # constructor
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return str(self.x) + "," + str(self.y)

    def __eq__(self, other):
        if other is not None:
            return str(self.x) == str(other.x) and str(self.y) == str(other.y)
        return True

    def __ne__(self, other):
        if other is not None:
            return str(self.x) != str(other.x) or str(self.y) != str(other.y)
        return True

    def __cmp__(self, other):
        return str(self.x) == str(other.x) and str(self.y) == str(other.y)

    def __hash__(self):
        return (hash(self.x)) + (2 * hash(self.y))

    # is_valid_location checks if a particular location is within the search space
    def is_valid_location(self, i, j, col, row):
        if 0 <= i < int(col) and 0 <= j < int(row):
            return True
        return False

    # returns the cell to the north of the current cell
    def get_north(self, col, row):
        if self.is_valid_location(self.x, self.y - 1, col, row):
            return Node(self.x, self.y - 1)
        return None

    # returns the cell to the north east of the current cell
    def get_north_east(self, col, row):
        if self.is_valid_location(self.x + 1, self.y - 1, col, row):
            return Node(self.x + 1, self.y - 1)
        return None

    # returns the cell to the north west of the current cell
    def get_north_west(self, col, row):
        if self.is_valid_location(self.x - 1, self.y - 1, col, row):
            return Node(self.x - 1, self.y - 1)
        return None

    # returns the cell to the west of the current cell
    def get_west(self, col, row):
        if self.is_valid_location(self.x - 1, self.y, col, row):
            return Node(self.x - 1, self.y)
        return None

    # returns the cell to the south west of the current cell
    def get_south_west(self, col, row):
        if self.is_valid_location(self.x - 1, self.y + 1, col, row):
            return Node(self.x - 1, self.y + 1)
        return None

    # returns the cell to the south of the current cell
    def get_south(self, col, row):
        if self.is_valid_location(self.x + 1, self.y, col, row):
            return Node(self.x + 1, self.y)
        return None

    # returns the cell to the south east of the current cell
    def get_south_east(self, col, row):
        if self.is_valid_location(self.x + 1, self.y + 1, col, row):
            return Node(self.x + 1, self.y + 1)
        return None

    # returns the cell to the east of the current cell
    def get_east(self, col, row):
        if self.is_valid_location(self.x, self.y + 1, col, row):
            return Node(self.x, self.y + 1)
        return None


"""Implements the required algorithm and finds the best path from the landing site to the target sites"""


class Search:
    actions = {"north": "get_north", "east": "get_east", "south": "get_south", "west": "get_west",
               "north_east": "get_north_east", "north_west": "get_north_west", "south_east": "get_south_east",
               "south_west": "get_south_west"}

    # constructor
    def __init__(self, type_of_search, landing_site, target_sites, state, max_z_elevation, col, row):
        self.type_of_search = type_of_search
        self.landing_site = landing_site
        self.target_sites = target_sites
        self.state = state
        self.max_z_elevation = max_z_elevation
        self.col = col
        self.row = row

    def __repr__(self):
        return "Type of search: " + str(self.type_of_search) + " Landing Site: " + str(
            self.landing_site) + " Target sites: " + \
               str(self.target_sites) + " Max Z elevation:" + str(self.max_z_elevation)

    # checks if the given site os equal to the target site
    def goal_test(self, curr_site, target_site):
        return curr_site == target_site

    # checks if the action is diagonal movement from one cell to the other
    def is_move_diagonal(self, action):
        return action == "north_east" or action == "north_west" or action == "south_east" or action == "south_west"

    def find_route(self, target_site):
        path = []
        if self.type_of_search == "BFS":
            path = self.bfs(target_site)
        if self.type_of_search == "UCS":
            path = self.ucs(target_site)
        if self.type_of_search == "A*":
            path = self.a_star(target_site)
        return path

    # forms the route from landing sie to target site
    def form_route(self, child_parent_dict, target_site):
        # list to store the entire path
        route = []
        present_site = target_site
        # append the site locations to the list as long as the site whose parent is landing site is found
        while present_site != self.landing_site:
            route.append(present_site)
            present_site = child_parent_dict[present_site]
        route.append(self.landing_site)
        # reverse the list
        route = route[::-1]
        return route

    # BFS Algorithm
    def bfs(self, target_site):
        # FIFO queue to push the locations of visited cells and pop the one by one
        frontier = queue.deque()

        # list to keep a track of all the locations visited
        explored = []

        # dictionary to store the parent cell location for each cell location
        child_parent_dict = {}

        # check if landing and target site is same
        if self.goal_test(self.landing_site, target_site):
            final_path = self.form_route(None, self.landing_site)
            return final_path

        frontier.append(self.landing_site)

        while True:
            if not frontier:
                return "FAIL"
            current_site = frontier.popleft()
            explored.append(current_site)
            for action in self.actions:
                site = eval(
                    "current_site" + "." + self.actions.get(action) + "(" + self.col + ", " + self.row + ")")

                # check if the rover is allowed to move from the current cell to the child cell
                if site is not None and abs(
                        int(self.state[site.y][site.x]) - int(self.state[current_site.y][current_site.x])) <= int(
                    self.max_z_elevation):
                    if site not in explored and site not in frontier:
                        if self.goal_test(site, target_site):
                            child_parent_dict[site] = current_site
                            final_path = self.form_route(child_parent_dict, site)
                            return final_path
                        frontier.append(site)
                        child_parent_dict[site] = current_site

    # UCS Algorithm
    def ucs(self, target_site):
        # dictionary to store the cell location and path cost from landing site
        priority_frontier = {self.landing_site: 0}

        # list to keep a track of all the visited cells
        explored = []

        # dictionary to store parent location for each cell
        child_parent_dict = {}

        # dictionary to maintain all the cell locations with path cost from landing site
        site_path_cost = {self.landing_site: 0}

        while True:
            if not priority_frontier:
                return "FAIL"

            # sort the priority queue by path cost
            sites_sorted_by_path_cost = sorted(priority_frontier.items(), key=operator.itemgetter(1))

            # pop the cell location with lowest path cost
            current_site = sites_sorted_by_path_cost[0][0]
            del priority_frontier[current_site]

            if self.goal_test(current_site, target_site):
                final_path = self.form_route(child_parent_dict, current_site)
                return final_path

            explored.append(current_site)

            for action in self.actions:
                site = eval("current_site" + "." + self.actions.get(action) + "(" + self.col + ", " + self.row + ")")
                if site is not None:
                    if site not in explored and site not in priority_frontier:
                        if abs(int(self.state[site.y][site.x]) - int(self.state[current_site.y][current_site.x])) <= \
                                int(self.max_z_elevation):
                            child_parent_dict[site] = current_site
                            if self.is_move_diagonal(action):
                                site_path_cost[site] = site_path_cost.get(current_site) + 14
                                priority_frontier[site] = site_path_cost.get(site)
                            else:
                                site_path_cost[site] = site_path_cost.get(current_site) + 10
                                priority_frontier[site] = site_path_cost.get(site)

                    # If cell is visited check if current path cost for that cell is less than path cost associated
                    # with it previously If yes, associate this new value to the cell. Change its parent to the
                    # current site
                    else:
                        if self.is_move_diagonal(action):
                            if priority_frontier.get(site) > site_path_cost.get(current_site) + 14:
                                child_parent_dict[site] = current_site
                                priority_frontier[site] = site_path_cost.get(current_site) + 14
                                site_path_cost[site] = site_path_cost.get(current_site) + 14
                        else:
                            if priority_frontier.get(site) > site_path_cost.get(current_site) + 10:
                                child_parent_dict[site] = current_site
                                priority_frontier[site] = site_path_cost.get(current_site) + 10
                                site_path_cost[site] = site_path_cost.get(current_site) + 10

    # A* Search Algorithm
    def a_star(self, target_site):
        # Heuristic used: Diagonal Distance

        diagonal_distance = max(
            (abs(int(self.landing_site.x) - int(target_site.x))), (abs(int(self.landing_site.y) - int(target_site.y))),
            abs(int(self.state[self.landing_site.y][self.landing_site.x]) - int(
                self.state[int(target_site.y)][int(target_site.x)])))

        # dictionary to store sites with their f(n) = g(n) + h(n) values
        priority_frontier = {self.landing_site: 0 + diagonal_distance}

        # list to keep track of visited sites
        explored = []

        # dictionary to store parent location of each cell
        child_parent_dict = {}

        # dictionary to maintain all the cell locations with f(n)
        site_path_cost = {self.landing_site: 0 + diagonal_distance}

        while True:
            if not priority_frontier:
                return "FAIL"

            # sort the priority queue by path cost
            sites_sorted_by_path_cost = sorted(priority_frontier.items(), key=operator.itemgetter(1))

            # pop the cell location with lowest path cost
            current_site = sites_sorted_by_path_cost[0][0]
            del priority_frontier[current_site]

            if self.goal_test(current_site, target_site):
                final_path = self.form_route(child_parent_dict, current_site)
                return final_path

            explored.append(current_site)

            for action in self.actions:
                site = eval("current_site" + "." + self.actions.get(action) + "(" + self.col + ", " + self.row + ")")
                if site is not None:
                    if site not in explored and site not in priority_frontier:
                        if abs(int(self.state[site.y][site.x]) - int(self.state[current_site.y][current_site.x])) <= \
                                int(self.max_z_elevation):
                            # calculate diagonal distance between site and target site
                            heuristic = max(
                                (abs(int(site.x) - int(target_site.x))), (abs(int(site.y) - int(target_site.y))),
                                abs(int(self.state[site.y][site.x]) - int(
                                    self.state[int(target_site.y)][int(target_site.x)])))
                            z_elevation_diff = abs(int(self.state[site.y][site.x]) - int(
                                self.state[int(target_site.y)][int(target_site.x)]))
                            child_parent_dict[site] = current_site
                            if self.is_move_diagonal(action):
                                site_path_cost[site] = (site_path_cost.get(
                                    current_site) + 14 + z_elevation_diff) + heuristic
                                priority_frontier[site] = site_path_cost.get(site)
                            else:
                                site_path_cost[site] = (site_path_cost.get(
                                    current_site) + 10 + z_elevation_diff) + heuristic
                                priority_frontier[site] = site_path_cost.get(site)

                    # If cell is visited check if current f(n) for that cell is less than f(n) associated with it
                    # previously If yes, associate this new value to the cell. Change its parent to the current site
                    else:
                        # calculate diagonal distance between site and target site
                        heuristic = max(
                            (abs(int(site.x) - int(target_site.x))), (abs(int(site.y) - int(target_site.y))),
                            abs(int(self.state[site.y][site.x]) - int(
                                self.state[int(target_site.y)][int(target_site.x)])))

                        z_elevation_diff = abs(
                            int(self.state[site.y][site.x]) - int(self.state[int(target_site.y)][int(target_site.x)]))

                        if self.is_move_diagonal(action):
                            if priority_frontier.get(site) > (
                                    site_path_cost.get(current_site) + 14 + z_elevation_diff) + heuristic:
                                child_parent_dict[site] = current_site
                                site_path_cost[site] = (site_path_cost.get(
                                    current_site) + 14 + z_elevation_diff) + heuristic
                                priority_frontier[site] = site_path_cost.get(site)
                        else:
                            if priority_frontier.get(site) > (
                                    site_path_cost.get(current_site) + 10 + z_elevation_diff) + heuristic:
                                child_parent_dict[site] = current_site
                                site_path_cost[site] = (site_path_cost.get(
                                    current_site) + 10 + z_elevation_diff) + heuristic
                                priority_frontier[site] = site_path_cost.get(site)


# open the input file to read the input
input_file = open("input.txt")

# search_type would store the type of algorithm to be implemented
search_type = input_file.readline().strip()

# width and height would store the size of the search space
width, height = input_file.readline().split()

# store the x and y location of the landing site
x_loc_of_landing_site, y_loc_of_landing_site = input_file.readline().split()

# start_site of type Node would store the location of landing site
start_site = Node(int(x_loc_of_landing_site), int(y_loc_of_landing_site))

# store the maximum allowable z elevation differences to be able to travel from one site to another
z_elevation = input_file.readline().strip()

# store the number of sites to find path to
no_of_target_sites = input_file.readline().strip()

# all_target_sites is a list to store the locations of all the target sites
all_target_sites = []

# add each location to all_target_sites
for i in range(int(no_of_target_sites)):
    x, y = input_file.readline().split()
    n = Node(x, y)
    all_target_sites.append(n)

# search_space stores the elevation values for each site
search_space = [line.split() for line in input_file.readlines()]

# create an object of type Search
s = Search(search_type, start_site, all_target_sites, search_space, z_elevation, width, height)

# final_route is a list that stores the paths for all the target sites
final_route = []

# create an output file to write the output
output_file = open("output.txt", 'w')

# store the path for each target site in final_route and write it to the output file
for i in range(int(no_of_target_sites)):
    final_route = s.find_route(all_target_sites[i])
    if final_route is not "FAIL":
        for j in range(len(final_route)):
            output_file.write(str(final_route[j]) + " ")
        output_file.write("\n")
    else:
        output_file.write(str(final_route) + "\n")

# close the input and output files
output_file.close()
input_file.close()