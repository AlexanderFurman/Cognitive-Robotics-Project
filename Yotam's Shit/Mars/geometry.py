import math, random
import matplotlib.pyplot as plt

class Map:
    def __init__(self, width, height, color='#e77d11'):
        self.width = width
        self.height = height
        self.color = color

    def plot_map(self, ax):
        rectangle = plt.Rectangle([0, 0], self.width, self.height, color=self.color)
        ax.add_patch(rectangle)
        return

class Edge:
    def __init__(self, node1, node2, cost=None, color='black'):
        self.nodes = [node1,node2]
        self.xy1 = node1.center
        self.xy2 = node2.center
        self.color = color
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
        Complexity: O(1)
        '''  
        [x0, y0] = obstacle.center; [x1, y1] = self.xy1; [x2, y2] = self.xy2
        a = y1 - y2; b = x2 - x1; c = (-b*y1) + (-a*x1)
        dist = abs(a*x0 + b*y0 + c) / math.sqrt(a**2 + b**2)
        if dist <= obstacle.radius:
            return True # collision
        return False # no collision
    
    def plot_edge(self, ax):
        xdata = [self.xy1[0], self.xy2[0]]; ydata = [self.xy1[1], self.xy2[1]]
        edge = plt.Line2D(xdata, ydata, color=self.color)
        ax.add_line(edge)
        return
    
class Solution_Edge:
    def __init__(self, node1, node2):
        super().__init__(node1, node2, color='yellow')

class Node:
    def __init__(self, center, cost=0.0, parent_index=-1, color='black'):
        self.center = center
        self.cost = cost
        self.parent_index = parent_index
        self.color = color

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
        super().__init__(center, color='blue')
    
    def plot_node(self, ax, color='blue'):
        super().plot_node(ax, color=color, s=50, zorder=15)
        return

class Goal_Node(Node):
    def __init__(self, center):
        super().__init__(center, color='#154f30')

    def plot_node(self, ax, color='#154f30'):
        super().plot_node(ax, color=color, s=50, zorder=15)
        return

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

    def Create_Goal_Node(self):
        min_x = self.center[0] - self.radius; max_x = self.center[0] + self.radius
        min_y = self.center[1] - self.radius; max_y = self.center[1] + self.radius
        no_col = True
        while no_col:
            x = random.random() * (max_x - min_x) + min_x
            y = random.random() * (max_y - min_y) + min_y
            g = Goal_Node([x,y])
            if not Check_Point_Not_In_Circle(g,self):
                no_col = False
        return g

def Euclidean_Distance(xy1,xy2):
    '''
    Complexity: O(1)
    '''    
    return math.sqrt((xy2[0]-xy1[0])**2 + (xy2[1]-xy1[1])**2)

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