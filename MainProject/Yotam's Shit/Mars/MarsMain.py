import random
from MarsGeometry import *
from MotionPlanner import *
from MarsPlotter import *

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
    N_goals = 3

    map = Map(100,100)
    obs = Create_Obstacles(N_obs)
    goals = Create_Goal_Zones(N_goals, obs)
    start = Create_Start_Node(obs, goals)
    
    samples, goal_nodes = Create_Samples(map, start, obs, goals, N_samples=200)
    roadmap, PRM_graph = Create_Roadmap(samples, obs, N_knn=5)
    #Plotter(map, obs, goals, start, samples=samples, roadmap=roadmap, PRM_graph=PRM_graph, save_gif=False).plot()

    new_graph, trajectories = PRM_Solve(start, goals, samples, goal_nodes, roadmap, PRM_graph)
    
    #Plotter(map, obs, goals, start, samples=samples, roadmap=roadmap, trajectories=trajectories, save_gif=True).Create_GIF(replot=True)
    Plotter(map, obs, goals, start, trajectories=trajectories).replot(clean=True)
    Visualize_Final_Graph(new_graph)
    return

if __name__ == '__main__':
    main()