import random
import matplotlib.pyplot as plt
from geometry import *

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

def Create_Obstacles(N_obs):
    obs = []
    for i in range(N_obs):
        r = random.randint(2, 10)
        x = random.randint(0+r, 100-r)
        y = random.randint(0+r, 100-r)
        obs.append(Obstacle(r,[x,y]))
    return obs

def Create_Goals(N_goals, obs):
    goals = []
    while len(goals) < N_goals:
        r = random.randint(4, 8)
        x = random.randint(0+r, 100-r)
        y = random.randint(0+r, 100-r)
        g = Goal(r,[x,y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g,o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g,p)
        if no_col:
            goals.append(g)
    return goals

def main():
    m = Map(100,100)
    N_obs = random.randint(5, 20)
    obs = Create_Obstacles(N_obs)
    N_goals = random.randint(3, 6)
    goals = Create_Goals(N_goals, obs)
    Plotter(m,obs,goals)
    return

if __name__ == '__main__':
    main()