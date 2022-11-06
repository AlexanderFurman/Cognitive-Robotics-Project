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
    
class Plotter:
    def __init__(self, map, obstacles, goals):
        self.fig, self.ax = plt.subplots()
        self.map = map
        self.obstacles = obstacles
        self.goals = goals

        self.ax.set_ylim(0,map.height)
        self.ax.set_xlim(0,map.width)
        self.ax.set_title('Mars Workspace')

        self.map.plot_map(self.ax)
        for o in self.obstacles:
            o.plot_circle(self.ax)
        for g in self.goals:
            g.plot_circle(self.ax)

        plt.show()

def Euclidean_Distance(xy1,xy2):
    return math.sqrt((xy2[0]-xy1[0])**2 + (xy2[1]-xy1[1])**2)

def Check_Circles_Collision(circle1,circle2):
    if Euclidean_Distance(circle1.centre,circle2.centre) < circle1.radius + circle2.radius:
        return False # if there is a collision
    else:
        return True # no collision

m = Map(100,100)
N_obs = random.randint(5, 20)
obs = []
for i in range(N_obs):
    r = random.randint(2, 10)
    x = random.randint(0+r, 100-r)
    y = random.randint(0+r, 100-r)
    obs.append(Obstacle(r,[x,y]))

N_goals = random.randint(3, 6)
goals = []
for i in range(N_goals):
    r = random.randint(4, 8)
    x = random.randint(0+r, 100-r)
    y = random.randint(0+r, 100-r)
    
    g = Goal(r,[x,y])
    no_col = True
    for o in obs:
        no_col = no_col and Check_Circles_Collision(g,o)
    for p in goals:
        no_col = no_col and Check_Circles_Collision(g,p)
    if no_col:
        goals.append(g)

Plotter(m,obs,goals)