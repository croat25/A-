
""" ---------------------------------------------------------
Author: Shawn Cramp
ID: 111007290
Author: Edward Huang
ID: 100949380
Author: Bruno Salapic
ID: 100574460
Author: Konrad Bania
ID: 110447960

Description: CP468 Final Assignment
Date: November, 13th, 2015
-------------------------------------------------------------
Assignment Task:

The purpose of this CP468 term project is to design and implement an
A*-based algorithm to solve a path planning problem. You can use
 the programming language of your choice

Consider a Museum room that is patrolled by N robots at night.
At a pre-determined time, the robots are supposed to rendezvous
at a given point R in the room. The robots move inside the room,
and the room contains obstacles, such as chairs and benches for
the visitors, paintings, sculptures etc. The robots are supposed
to know the locations of the obstacles in the room. Implement an
A*-based algorithm to compute the path of each robot, from its
initial position to the given rendezvous point R.
-------------------------------------------------------------
Import Declarations ------------------------------------- """
import heapq
import math
import copy
import os
import time
import sys


""" ---------------------------------------------------------
Global Declarations ------------------------------------- """
TRAVEL_COST = 1  # Cost to travel from one node to the next


""" ---------------------------------------------------------
Class Declarations -------------------------------------- """


class PriorityQueue:
    """
    Priority Queue

    Used to store nodes.
    """
    def __init__(self):
        self._values = []

    def empty(self):
        return len(self._values) == 0

    def length(self):
        return len(self._values)

    def put(self, item, priority):
        heapq.heappush(self._values, (priority, item))

    def get(self):
        return heapq.heappop(self._values)[1]


class Map:
    """
    Map object for holding all nodes in the map, as well as
    children to each node
    """
    def __init__(self, width, height, rendezvous, robots, layout, node_dict):
        self.width = width
        self.height = height
        self.rendezvous = rendezvous
        self.robots = robots
        self.layout = layout
        self.nodes = node_dict

    def children(self, node):
        return self.nodes[node]


class Robot:
    """
    Robot object for holding robot paths and locations
    """
    def __init__(self, start, finish, came_from, cost_so_far, path_cost):
        """
        Temp Comment
        """
        self.start = start
        self.finish = finish
        self.path_cost = path_cost
        self.came_from = came_from
        self.cost_so_far = cost_so_far

    def dijkstra(self, node_map):
        """
        Temp Comment
        """
        dijkstra_stuff = 0
        sys.stdout.write('\r {}%'.format(dijkstra_stuff))
        
        distance = copy.deepcopy(self.path_cost)
        
        distance_calc = copy.deepcopy(self.path_cost)
        
        node = copy.deepcopy(self.finish)
        pathing = [node]
        while distance != -1:
            
            sys.stdout.write('\r {}% Completed...'.format((dijkstra_stuff*1.0 / distance_calc*1.0) * 100))
            dijkstra_stuff += 1
            
            node = pathing[-1]
            # print('Checking Children of Node: {}'.format(node))
            # print('Distance: {}'.format(distance))
            children = node_map.children(node)
            # print('Node Children: {}'.format(children))
            for child in children:
                
                if child in self.cost_so_far.keys():
                    # print('Child Cost: {}: {}'.format(child, self.cost_so_far[child]))
                    if self.cost_so_far[child] == distance:
                        # print('If Check: Passed')
                        pathing.append(child)
                        break
                    else:
                        pass
                        # print('If Check: Failed')
            distance -= 1
            # print('-'*18)

        return list(reversed(pathing))

    def pprint(self, layout, width, height):
        """

        """
        print('Node Costs')
        for key, val in self.cost_so_far.iteritems():
            # print('{}: {}'.format(key, val))
            layout[key] = val

        array = []
        for i in range(0, height):
            temp = []
            for j in range(0, width):
                temp.append(9)
            array.append(temp)

        for key, val in layout.iteritems():
            # print('{}: {}'.format(key, val))
            array[key[0]][key[1]] = val

        print('Layout Map by Cost')
        print('#: Wall     . : UnExplored Floor Space\n')
        for i in array:
            for j in i:
                if j == '#':
                    sys.stdout.write('|')
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('{} '.format(j))
            sys.stdout.write('\n')


""" ---------------------------------------------------------
Function Declarations ----------------------------------- """


def valid_coordinate(node_set, x, y, shift):
    """
    Check if node at node_set[x][y] is a valid node for robots
    to travel.

    :param node_set:
    :param x:
    :param y:
    :return:
    """
    valid = False
    new_x = x + shift[0]
    new_y = y + shift[1]

    if (new_x >= 0 and new_x < len(node_set)) and (new_y >= 0 and new_y < len(node_set[0])):
        if node_set[new_x][new_y] == 0:
            valid = True
        else:
            pass
    else:
        pass

    return valid


def get_children(node_set, x, y):
    """
    Using 2D list of all nodes in the map and an x and y coordinate
    in the map.  Get the child nodes attached to the x and y
    coordinate parameter.

    :param node_set:
        2D array of all nodes in the map
    :param x:
        x coordinate of current node
    :param y:
        y coordinate of current node
    :return children:
        dictionary of key node and children
    """
    temp = []
    neighbours = ((1, 0), (0, 1), (-1, 0), (0, -1))

    for shift in neighbours:
        #  check = node_set[x + value[0]][y + value[1]]
        if valid_coordinate(node_set, x, y, shift):
            temp.append((x + shift[0], y + shift[1]))
            #  print(temp)

    children = {(x, y): temp}
    return children


def build_dictionary(node_set):
    """
    Build the dictionary of valid nodes and their explorable children.

    :param node_set:
    :return:
    """
    nodes = {}
    layout = {}

    for x, x_line in enumerate(node_set):
        for y, y_line in enumerate(x_line):
            if node_set[x][y] == 0:
                layout[(x, y)] = '.'  # for drawing
                children = get_children(node_set, x, y)
                nodes.update(children)
            elif node_set[x][y] == 1:
                layout[(x, y)] = '#'  # for drawing

    return nodes, layout


def map_coordinates(map_handle):
    """
    Create Map object with all coordinates and neighbours

    :param map_handle:
        File handle for Map
    :return:
        Coordinates List
    """

    ''' List of Robots '''
    robots = []

    ''' Initial Declarations.  These will be overwritten '''
    width = 0
    height = 0
    rendezvous = (0, 0)

    ''' 2D Array to initialize nodes '''
    temp = []

    ''' Build Map Object '''
    for i, line in enumerate(open(map_handle, 'r')):

        if i == 0:  # when i is 0
            line = line.strip().split(" ")
            width = line[1]
            height = line[0]

        elif i == 1:  # when i is 1
            line = line.strip().split(" ")
            robot_count = int(line[0])

        elif i - 2 < robot_count:
            robots.append(tuple(map(int, line.strip().split(" "))))

        elif i == 2 + robot_count:
            rendezvous = tuple(map(int, line.strip().split(" ")))

        else:
            temp.append(map(int, line.strip()))

    node_dict, layout = build_dictionary(temp)

    return Map(width, height, rendezvous, robots, layout, node_dict)


def heuristic(g, h, distance):
    """
    A* Heuristic
    f(n) = g(n) + h(n)

    :param g:
        Cost to reach node from start
        Eg. (2, 1)
    :param h:
        Estimated cost to get from n to goal
        Eg. (4, 7)
    :return f(n):
        Estimated total cost of cheapest
        solution through n
    """
    return math.sqrt((abs(h[0] - g[0]) ** 2) + (abs(h[1] - g[1]) ** 2)) + distance


def a_star(coordinates, robot, rendezvous):
    """
    Complete A* Search Algorithm.

    :param coordinates:
        Map Coordinates Object
    :param rendezvous:
        Where the Robots meet.  (Ending Node Node)
        Eg. (5, 7)
    :param robot:
        Location of Robot.
        Eg. (2, 1)
    :return found:
        True if Robot is found, else False.
    :return cost:
        Cost of Path to Robot
    """

    ''' Cost of Initial Node '''
    priority = 0

    ''' Priority Queue for evaluating locations '''
    frontier = PriorityQueue()

    ''' Put Root node into Queue '''
    frontier.put(robot, priority)

    ''' Initialize '''
    came_from = {}
    cost_so_far = {}
    came_from[robot] = None
    cost_so_far[robot] = 0

    ''' Loop through Queue '''
    while not frontier.empty():
        current_node = frontier.get()

        ''' If current node is robot node,
        break and return true -------- '''
        if current_node == rendezvous:
            break

        ''' Evaluate neighbouring nodes to current node '''
        for node in coordinates.children(current_node):
            # print('Current: {}'.format(current_node))
            # print('Currently Evaling Child: {}'.format(node))
            new_cost = cost_so_far[current_node] + TRAVEL_COST
            # print('New Cost for Node: {}'.format(new_cost))
            if node not in cost_so_far or new_cost < cost_so_far[node]:
                # print('If Check: Passed')

                cost_so_far[node] = new_cost
                priority = new_cost + heuristic(node, rendezvous, new_cost)

                frontier.put(node, priority)

                # print('Node Priority: {}'.format(priority))
                # print('Frontier Size: {}'.format(frontier.length()))

                came_from[node] = current_node
                # print('-'*9)
            else:
                pass
                # print('If Check: Failed')
                # print('Frontier Size: {}'.format(frontier.length()))
                # print('-'*9)

    return came_from, cost_so_far


def init_robots(node_map):
    """
<<<<<<< HEAD
    
=======

>>>>>>> 4ea83ae2feac247d47021c295b6599daf8dacc5d
    """
    robots = []
    for robot_start in node_map.robots:
        if robot_start in node_map.nodes:
            print('{} initialized'.format(robot_start))
            came_from, cost_so_far = a_star(node_map, robot_start, node_map.rendezvous)
            robots.append(Robot(robot_start, node_map.rendezvous, came_from, cost_so_far, cost_so_far[node_map.rendezvous]))
        else:
            print('Robot Location Invalid')

    return robots


""" ---------------------------------------------------------
Console Execution Functions ---------------------------------

All functions below are used for execution of the program and
are no involved in the logical process.

The Main function will be called upon code execution and the
user will be presented with a list of options on how to
proceed and view the code output he/she would like to view.

-------------------------------------------------------------
--------------------------------------------------------- """


def map_details(node_map):
    """

    :param node_map:
    :return:
    """
    print('Mapping Data: \n')
    print('Width: {:>10}'.format(node_map.width))
    print('Height: {:>10}'.format(node_map.height))
    print('Rendezvous: {:>10}'.format(node_map.rendezvous))
    print('Robots: {:>10}'.format(node_map.robots))


def node_dictionary(dictionary, indent=0):
    """
    Print Dictionary to console in a readable fashion

    :param dictionary:
        Dictionary to print
    :param indent:
    :return:
    """
    for key, value in dictionary.iteritems():
        print('\t' * indent + str(key))
        if isinstance(value, dict):
            node_dictionary(value, indent+1)
        else:
            print('\t' * (indent+1) + str(value))


def optimal_print(layout, path, width, height, start, finish):
        output_file = open('output/output{}.txt'.format(start), 'w')
    
        array = []
        for i in range(0, height):
            temp = []
            for j in range(0, width):
                temp.append(9)
            array.append(temp)
        
        for key, val in layout.iteritems():
            array[key[0]][key[1]] = val
        
        for pos, val in enumerate(path): 
            if (pos + 1) < len(path):
                next = path[pos + 1]
                
            #  print('a: {}    b: {}'.format(val, next))
            array[val[0]][val[1]] = direction(val, next, start, finish)
            
        print('Layout Map by Cost')
        print('#: Wall     . : UnExplored Floor Space\n')
        for i in array:  
            for j in i:
                if j == '#':
                    #sys.stdout.write(u'\u25A0')
                    #sys.stdout.write(' ')
                    output_file.write('|')
                    output_file.write(' ')
                else:
                    #sys.stdout.write(j + ' ')
                    output_file.write(j + ' ')
            #sys.stdout.write('\n')
            output_file.write('\n')


def print_paths(node_map, robots):
    """

    :param node_map:
    :param robots:
    :return:
    """
    for guy in robots:
        print('Evaluating Robot: {} -> {}'.format(guy.start, guy.finish))
        layout = copy.deepcopy(node_map.layout)
        width = int(copy.deepcopy(node_map.width))
        height = int(copy.deepcopy(node_map.height))
        # guy.pprint(layout, width, height)

        layout = copy.deepcopy(node_map.layout)
        mapping = copy.deepcopy(node_map)
        print('One Optimal Path:')
        print('Path Cost: {}'.format(guy.path_cost))
        optimal_path = guy.dijkstra(mapping)
        print(optimal_path)
        print('Optimal Path Only:')
        print(guy.start)
        print(guy.finish)
        optimal_print(layout, optimal_path, width, height, guy.start, guy.finish)
        # print(guy.cost_so_far.keys())
        print('-'*18)


def direction(a, b, start, finish):
    if (a[0] < b[0]):  # Down
        direction = 'V'
    elif (a[0] > b[0]):  # Up
        direction = '^'
    elif (a[1] > b[1]):  # Left
        direction = '<'
    elif (a[1] < b[1]):  # Right
        direction = '>'
    
    if a == start:
        direction = 'S'
    elif a == finish:
        direction = 'F'

    return direction


def main():
    """
    Main Loop of Program

    :return:
    """

    print('-----------------------------------------\n'
          'Executing CP468 Final Project\n'
          'Created By:\n'
          '-----------------------------------------\n'
          'Shawn Cramp\n'
          'Edward Huang\n'
          'Bruno Salapic\n'
          'Konrad Bania\n'
          '-----------------------------------------\n'
          'Final Project Option 3\n'
          '-----------------------------------------\n'
          'Map Options:\n')

    ''' Create File list '''
    base_dir = os.path.abspath(os.path.dirname(__file__))
    map_files = os.listdir(base_dir + '\maps')

    for i, map_file in enumerate(map_files):
        print('{}: {}'.format(i+1, map_file))

    map_selection = int(input('\nEnter desired map number: ')) - 1

    print('-'*41)
    print('Performing A* Search with map {}'.format(map_files[map_selection]))
    print('-'*41)

    ''' Create Map Object '''
    map_handle = 'maps/' + map_files[map_selection]

    ''' 2D Array of nodes in Map '''
    node_map = map_coordinates(map_handle)

    ''' Display Robots Created '''
    robots = init_robots(node_map)
    print('A* Completed on {} robot(s):'.format(len(node_map.robots)))
    for i in robots:
        print('Robot: {} -> {}'.format(i.start, i.finish))

    print('-'*40)

    ''' Program Options Loop '''
    exit_program = False
    while not exit_program:

        print('\nSelect next option...')
        print('1: Puzzle Details')
        print('2: Map Nodes and Children')
        print('3: A* Search Results')

        option = int(input('\nSelect an option: '))

        if option == 1:
            map_details(node_map)
        elif option == 2:
            print('Dictionary:')
            node_dictionary(node_map.nodes)
        elif option == 3:
            print_paths(node_map, robots)
        else:
            break

    print('-'*50)


""" Launch Main Program """
main()
