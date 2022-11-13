import math, random
import matplotlib.pyplot as plt
import numpy as np

class Rectangle:
    def __init__(self, origin, width, height, color='grey'):
        self.width = width
        self.height = height
        self.color = color
        self.origin = origin

    def plot_rectangle(self, ax):
        rectangle = plt.Rectangle(xy=self.origin, width=self.width, height=self.height, color=self.color)
        ax.add_patch(rectangle)
        return

class Map(Rectangle):
    def __init__(self, origin=np.array([0, 0]), width=100, height=100, color='#e77d11'):
        super().__init__(origin=origin, width=width, height=height, color=color)

class Edge:
    def __init__(self, node1, node2, cost=None, color='black'):
        self.nodes = [node1,node2]
        self.xy1 = node1.center
        self.xy2 = node2.center
        self.color = color
        self.plotted = False
        if cost == None:
            self.cost = Euclidean_Distance(self.xy1,self.xy2)
        else:
            self.cost = cost

    def __eq__(self, other):
        count = 0
        for i in range(2):
            if (self.xy1[i] == other.xy1[i] and self.xy2[i] == other.xy2[i]) or (self.xy1[i] == other.xy2[i] and self.xy2[i] == other.xy1[i]):
                count += 1
        if count == 2:
            return True
        return False

    def Highlight_Nodes(self, ax):
        for n in self.nodes:
            if not isinstance(n,Start_Node) and not isinstance(n,Goal_Node):
                n.plot_node(ax, color='yellow')
            elif isinstance(n,Start_Node):
                n.plot_node(ax, color='#81d4fa')
            elif isinstance(n,Goal_Node):
                n.plot_node(ax, color='#5ced73')
        return

    def Check_Edge_Obstacle_Collision(self, obstacle):
        '''
        Assuming point robot.
        Complexity: O(?)
        '''
        p1 = self.xy1; p2 = self.xy2; o = obstacle.center
        o_p1 = self.Vector_Diff(p1, o)
        o_p2 = self.Vector_Diff(p2, o)
        p2_p1 = self.Vector_Diff(p2, p1)
        p1_p2 = self.Vector_Diff(p1, p2)

        min_dist = np.Inf
        max_dist = max(Euclidean_Distance(o,p1), Euclidean_Distance(o,p2))
        if np.dot(o_p1, p1_p2) > 0 and np.dot(o_p2,p2_p1) > 0:
            min_dist = 2*self.Triangle_Area(o, p1, p2)/Euclidean_Distance(p1, p2)
        else:
            min_dist = min(Euclidean_Distance(o, p1), Euclidean_Distance(o,p2))

        if min_dist <= obstacle.radius and max_dist >= obstacle.radius:
            return True # collision
        else:
            return False # no collision
    
    def Vector_Diff(self, p2, p1):
        return [p2[i]-p1[i] for i in range(len(p1))]

    def Triangle_Area(self, a, b, c):
        ab = np.array([b[0]-a[0], b[1]-a[1]])
        ac = np.array([c[0]-a[0], c[1]-a[1]])
        return abs(np.cross(ab, ac))/2
    
    def plot_edge(self, ax, color=None):
        if color==None:
            color = self.color
        xdata = [self.xy1[0], self.xy2[0]]; ydata = [self.xy1[1], self.xy2[1]]
        edge = plt.Line2D(xdata, ydata, color=color)
        ax.add_line(edge)
        return
    
class Node:
    def __init__(self, center, name=None, cost=0.0, parent_index=-1, color='black'):
        self.center = center
        self.cost = cost
        self.parent_index = parent_index
        self.color = color
        self.name = name
        self.plotted = False

    def __str__(self):
        return str(self.center[0]) + "," + str(self.center[1]) + "," + str(self.cost) + "," + str(self.parent_index)

    def plot_node(self, ax, color=None, s=25, zorder=10):
        if color == None:
            c = self.color
        else:
            c = color
        ax.scatter(self.center[0], self.center[1], c=c, s=s, zorder=zorder)
        return

class Start_Node(Node):
    def __init__(self, center):
        super().__init__(center, name='S', color='blue')
    
    def plot_node(self, ax, color='blue'):
        super().plot_node(ax, color=color, s=50, zorder=15)
        return

class Goal_Node(Node):
    def __init__(self, center, name):
        super().__init__(center, name=name, color='#154f30')

    def plot_node(self, ax, color='#154f30'):
        super().plot_node(ax, color=color, s=50, zorder=15)
        return

class Graph:
    def __init__(self, nodes):
        self.nodes = nodes
        self.graph = {}
    
    def Get_Outgoing_Edges(self, node):
        "Returns the neighbors of a node."
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections
    
    def Get_Edge_Value(self, node1, node2):
        "Returns the value of an edge between two nodes."
        return self.graph[node1][node2]

class Circle:
    def __init__(self, radius, center, color):
        self.radius = radius
        self.center = center
        self.color = color

    def plot_circle(self, ax):
        circle = plt.Circle((self.center[0], self.center[1]), self.radius, color=self.color)
        ax.add_patch(circle)
        return

class Obstacle(Circle):
    def __init__(self, radius, center):
        super().__init__(radius, center, color='#c1440e')

class Goal(Circle):
    def __init__(self, radius, center):
        super().__init__(radius, center, color='green')

    def Create_Goal_Node(self, name):
        min_x = self.center[0] - self.radius; max_x = self.center[0] + self.radius
        min_y = self.center[1] - self.radius; max_y = self.center[1] + self.radius
        no_col = True
        while no_col:
            x = random.random() * (max_x - min_x) + min_x
            y = random.random() * (max_y - min_y) + min_y
            g = Goal_Node([x,y], name=name)
            if not Check_Point_Not_In_Circle(g,self):
                no_col = False
        return g

def Euclidean_Distance(xy1,xy2):
    '''
    Complexity: O(1)
    '''
    return math.dist(xy1, xy2)

def Check_Circles_No_Collision(circle1,circle2):
    '''
    Complexity: O(1)
    '''    
    if Euclidean_Distance(circle1.center,circle2.center) < circle1.radius + circle2.radius:
        return False # collision
    return True # no collision

def Check_Point_Not_In_Circle(point,circle):
    '''
    Complexity: O(1)
    '''    
    if Euclidean_Distance(point.center,circle.center) < circle.radius:
        return False # collision
    return True # no collision

def Get_Edge(node1, node2, edges):
    for e in edges:
        if Edge(node1,node2) == e:
            return e
    return -1