import matplotlib.pyplot as plt

class Map:
    def __init__(self, width, radius, dist):
        self.width = width
        self.dist = dist
        self.n_waypoints = width**2
        self.radius = radius
        self.waypoints = []
        self.edges = []

    def generate_waypoints(self):
        for i in range(self.width):
            for j in range(self.width):
                self.waypoints.append(Waypoint(self.radius, (i*self.dist, j*self.dist)))
            
    def generate_edges(self):
        for i in range(self.width - 1):
            for j in range(self.width -1):
                edge_x = [i*self.dist, (i+1)*self.dist]
                edge_y = [j*self.dist, (j+1)*self.dist]
                self.edges.append(edge_x)
                self.edges.append(edge_y)

        for i in range(1,self.width):
            for j in range(self.width-1):
                edge_x = [i*self.dist, (i-1)*self.dist]
                edge_y = [j*self.dist, (j+1)*self.dist]
                self.edges.append(edge_x)
                self.edges.append(edge_y)
        
        for i in range(self.width):
            for j in range(self.width):
                if j < self.width - 1:
                    edge_x = [i*self.dist, i*self.dist]
                    edge_y = [j*self.dist, (j+1)*self.dist]
                    self.edges.append(edge_x)
                    self.edges.append(edge_y)
                if i < self.width - 1:
                    edge_x = [i*self.dist, (i+1)*self.dist]
                    edge_y = [j*self.dist, j*self.dist]
                    self.edges.append(edge_x)
                    self.edges.append(edge_y)

    def plot(self, ax):
        # draw the picture representation of the map
        ax.plot(*self.edges, color = 'g', linewidth = 3, zorder=1)

        for waypoint in self.waypoints:
            waypoint.plot(ax)

    


class Waypoint:
    # location should be given as a tuple (x,y)
    def __init__(self, radius, location):
        self.radius = radius
        self.location = location
        self.colour = [142/255, 92/255, 92/255, 1]
    
    def assign_special(self, type):
        if type == 'cave':
            pass

    def highlight(self):
        self.colour[3] = 0.5
    
    def unhighlight(self):
        self.colour[3] = 1
        

    def plot(self, ax):
        circle = plt.Circle(self.location, self.radius, ec='k', fc = (*self.colour,), zorder = 2)
        ax.add_patch(circle)


class Rover:
    def __init__(self, length, location):
        self.length = length
        self.location = location
        self.plot_location = (self.location[0] - self.length/2, self.location[1] - self.length/2)
        self.colour = [0/255, 230/255, 255/255, 1]

    def update_location(self, location):
        self.location = location
        self.plot_location = (self.location[0] - self.length/2, self.location[1] - self.length/2)

    def assign_loaded(self):
        self.colour = [34/255, 139/255, 151/255, 1]

    def assign_unloaded(self):
        self.colour = [0/255, 230/255, 255/255, 1]

    def plot(self, ax):
        rectangle = plt.Rectangle(self.plot_location, self.length, self.length, ec='k', fc=self.colour, zorder=3)
        ax.add_patch(rectangle)


class Person:
    def __init__(self, size, location):
        self.size = size
        self.location = location #array [x,y]
        self.colour = [150/255, 56/255, 233/255, 1]

    def update_location(self, location):
        self.location = location

    def plot(self, ax):
        ax.plot(*self.location, markerfacecolor = self.colour, markeredgecolor = 'k', markersize=self.size, marker="^", zorder=4)

    







if __name__ == "__main__":
    fig, ax = plt.subplots()
    person = Person(20, [0,0])
    rover = Rover(0.2, [0,0])
    map = Map(3, 0.2, 2)
    map.generate_waypoints()
    map.generate_edges()
    map.plot(ax)
    person.plot(ax)
    rover.plot(ax)
    ax.autoscale() 
    ax.set_aspect('equal', 'box')
    plt.show()