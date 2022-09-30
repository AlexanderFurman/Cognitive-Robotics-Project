from math import sqrt
from tkinter.tix import TEXT
import os
import sys

path = '/'.join(__file__.split('/')[:-2])
print(path)
print("current paths = ", sys.path)
if path not in sys.path:
    sys.path.append('/'.join(__file__.split('/')[:-2]))
print("current paths = ", sys.path)

class PDDL_ProblemGenerator:
    def __init__(self, domain_name, width, traversal_weight, num_rovers, num_ppl, pos_cave, pos_base, max_mission_duration):
        # self.width = width
        #generates 2d array, where each node in the graph contains its relative pos.
        self.num_rovers = num_rovers
        self.num_people = num_ppl
        
        self.width = width
        self.num_waypoints = width**2
        # self.graph = [[i, j] for i in range(width) for j in range(width)]
        self.graph = [[(self.width*i + j + 1) for j in range(width)] for i in range(width)]
        self.perp_traversal_weight = traversal_weight
        self.diag_traversal_weight = traversal_weight*sqrt(2)
        self.domain_name = domain_name
        self.max_mission_duration = max_mission_duration
        self.problem_name = ''

        self.pos_cave = pos_cave
        self.pos_base = pos_base

        self.people = []
        self.rovers = []
        self.waypoints = []

        self.specify_people()
        self.specify_rovers()
        self.specify_waypoints()

    def specify_waypoints(self):
        for i in range(self.num_waypoints):
            self.waypoints.append('waypoint' + str(i+1))

    def specify_people(self):
        for i in range(self.num_people):
            self.people.append('person' + str(i+1))

    def specify_rovers(self):
        for i in range(self.num_rovers):
            self.rovers.append('rover' + str(i+1))

    # Function to check whether position is valid or not
    def isValidPos(self, i, j):
        if (i < 0 or j < 0 or i > self.width - 1 or j > self.width - 1):
            return 0
        return 1

    # Function that returns all adjacent elements
    def get_adjacent(self, arr, i, j):
    
        # Initialising a vector array
        # where adjacent element will be stored
        v = []
    
        # Checking for all the possible adjacent positions
        if (self.isValidPos(i - 1, j - 1)):
            v.append(arr[i - 1][j - 1])
        if (self.isValidPos(i - 1, j)):
            v.append(arr[i - 1][j])
        if (self.isValidPos(i - 1, j + 1)):
            v.append(arr[i - 1][j + 1])
        if (self.isValidPos(i, j - 1)):
            v.append(arr[i][j - 1])
        if (self.isValidPos(i, j + 1)):
            v.append(arr[i][j + 1])
        if (self.isValidPos(i + 1, j - 1)):
            v.append(arr[i + 1][j - 1])
        if (self.isValidPos(i + 1, j)):
            v.append(arr[i + 1][j])
        if (self.isValidPos(i + 1, j + 1)):
            v.append(arr[i + 1][j + 1])
    
        # Returning the vector
        return v

    # Function to check if two
    # integers are on the same
    # diagonal of the matrix
    def checkSameDiag(self, x, y):
        
        # Storing indexes of x in I, J
        # Storing Indexes of y in P, Q
        for i in range(self.width):
            for j in range(self.width):
                if self.graph[i][j] == x:
                    I, J = i, j
                if self.graph[i][j] == y:
                    P, Q = i, j
                    
        # Condition to check if the
        # both the elements are in
        # same diagonal of a matrix
        if P-Q == I-J or P + Q == I + J:
            return 1
        return 0


    def add_problem(self):
        text = ''
        i = 0
        while os.path.exists("pddl/mars-problem-%i.pddl" % i):
            i += 1
        self.problem_name = 'mars-problem-' + str(i)
        text += '(problem ' + self.problem_name + ')\n'
        return text

    def add_domain(self):
        text = '\n(:domain ' + self.domain_name + ')'
        return text
    
    def add_objects(self):
        text = '\n(:objects '

        for waypoint in self.waypoints:
            text += waypoint + ' '

        for rover in self.rovers:
            text += rover + ' '

        for person in self.people:
            text += person + ' '
        
        text += ')\n'

        return text

    def add_init(self):
        text = '\n(:init \n'

        text += '(= (mission-duration) 0)'
        text += '\n\n'

        for rover in self.rovers:
            text += '(= (battery-level ' + rover + ') 100) '
        text += '\n\n'

        for rover in self.rovers:
            text += '(rover ' + rover + ') '
        text += '\n\n'

        for person in self.people:
            text += '(cargo ' + person + ') '
        text += '\n\n'

        for waypoint in self.waypoints:
            text += '(location ' + waypoint + ') '
        text += '\n\n'

        for i in range(len(self.graph)):
            for j in range(len(self.graph)):
                adj_waypoints = self.get_adjacent(self.graph, i, j)
                for adj_waypoint in adj_waypoints:
                    current_wpt = self.waypoints[self.width*i + j]
                    text += '(path ' + current_wpt + ' waypoint' + str(adj_waypoint) + ') '
                    diag_query = self.checkSameDiag(self.width*i+j+1,adj_waypoint)
                    if diag_query:
                        text += '(= (time-to-traverse ' + current_wpt + ' waypoint' + str(adj_waypoint) + ') ' + str(self.diag_traversal_weight) + ')'
                    else:
                        text += '(= (time-to-traverse ' + current_wpt + ' waypoint' + str(adj_waypoint) + ') ' + str(self.perp_traversal_weight) + ')'
                    text += '\n'
        text += '\n'

        cave_loc = self.waypoints[self.width*self.pos_cave[0] + self.pos_cave[1]]
        base_loc = self.waypoints[self.width*self.pos_base[0] + self.pos_base[1]]
        for rover in self.rovers:
            text += '(at ' + rover + ' ' + base_loc + ') '
        for person in self.people:
            text += '(at ' + person + ' ' + cave_loc + ') '
        text += '\n\n'
        
        for rover in self.rovers:
            text += '(empty ' + rover + ') '
        text += '\n\n'

        for person in self.people:
            text += '(outside-rover ' + person + ') '
        text += '\n\n'

        text += ')\n'
        return text
    
    def add_goal(self):
        text = '\n(:goal \n'
        base_loc = self.waypoints[self.width*self.pos_base[0] + self.pos_base[1]]

        for person in self.people:
            text += '(at ' + person + ' ' + base_loc + ') '
        text += '\n\n'

        for rover in self.rovers:
            text += '(> (battery-level ' + rover + ') 0) '
        text += '\n\n'

        text += '(< (mission-duration) ' + str(self.max_mission_duration) + ')'

        text += '\n)'
        return text

    def create_pddl_file(self):
        text = '(define '
        text += self.add_problem()
        text += self.add_domain()
        text += self.add_objects()
        text += self.add_init()
        text += self.add_goal()
        text += '\n)'

        print(text)

        with open('pddl/' + self.problem_name + '.pddl', 'w+') as f:
            f.write(text)
            f.close()

        
        
 

        


generator = PDDL_ProblemGenerator('rover-environment', 3, 10, 1, 1, [0,0], [1,1], 240)
print(generator.graph)
generator.create_pddl_file()

