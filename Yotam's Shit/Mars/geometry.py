import math
import matplotlib.pyplot as plt

class Map:
    def __init__(self, width, height, color='#e77d11'):
        self.width = width
        self.height = height
        self.color = color

    def plot_map(self, ax):
        rectangle = plt.Rectangle([0, 0], self.width, self.height, color=self.color)
        ax.add_patch(rectangle)

class Circle:
    def __init__(self, radius, centre, color):
        self.radius = radius
        self.centre = centre
        self.color = color

    def plot_circle(self, ax):
        circle = plt.Circle((self.centre[0], self.centre[1]), self.radius, color=self.color)
        ax.add_patch(circle)

class Obstacle(Circle):
    def __init__(self, radius, centre):
        super().__init__(radius, centre, color='#c1440e')

class Goal(Circle):
    def __init__(self, radius, centre):
        super().__init__(radius, centre, color='green')

def Euclidean_Distance(xy1,xy2):
    return math.sqrt((xy2[0]-xy1[0])**2 + (xy2[1]-xy1[1])**2)

def Check_Circles_No_Collision(circle1,circle2):
    if Euclidean_Distance(circle1.centre,circle2.centre) < circle1.radius + circle2.radius:
        return False # if there is a collision
    else:
        return True # no collision

def Check_Point_Not_In_Circle(point,circle):
    if Euclidean_Distance(point,circle.centre) < circle.radius:
        return False # if there is a collision
    else:
        return True # no collision