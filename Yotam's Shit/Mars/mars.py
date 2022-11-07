import random
from Geometry import *
from PRM import *
from Plotter import Plotter

def Create_Obstacles(N_obs):
    obs = []
    for _ in range(N_obs):
        r = random.randint(2, 10)
        x = random.randint(0+r, 100-r)
        y = random.randint(0+r, 100-r)
        obs.append(Obstacle(r,[x,y]))
    return obs

def Create_Goal_Zones(N_goals, obs):
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

def Create_Start_Node(obs, goals):
    start_not_found = True
    while start_not_found:
        start_not_found = False
        x = random.randint(0, 100)
        y = random.randint(0, 100)
        start = Start_Node([x,y])
        for o in obs:
            if not Check_Point_Not_In_Circle(start,o):
                start_not_found = True
                continue
        for g in goals:
            if not Check_Point_Not_In_Circle(start,g):
                start_not_found = True
                continue
    return start

def main():
    N_obs = random.randint(5, 20)
    N_goals = random.randint(3, 6)

    map = Map(100,100)
    obs = Create_Obstacles(N_obs)
    goals = Create_Goal_Zones(N_goals, obs)
    start = Create_Start_Node(obs, goals)
    
    samples = Create_Samples(map, start, obs, goals, N_samples=100)
    roadmap = Create_Roadmap(samples, obs)
    Plotter(map, obs, goals, start, samples, roadmap).plot()
    #Plotter(map, obs, goals, start, samples, roadmap).Create_GIF()

    ##TODO: Here we replot the C-space with only the relevant nodes and edges from the PRM solution
    #solution_nodes, solution_edges = PRM_Solve(map, start, goals, obs)
    #Plotter(map, obs, goals, start, solution_nodes, solution_edges).replot()
    return

if __name__ == '__main__':
    main()