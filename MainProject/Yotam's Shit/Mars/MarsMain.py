import random, argparse
from MarsGeometry import *
from MarsMotionPlanner import *
from MarsPlotter import *
from MarsTaskPlanner import *

def Create_Obstacles(N_obs):
    """
    Creates the obstacles for our problem
    :param N_obs: the number of obstacles to instantiate
    :returns: list of the obstacle instances
    """
    obs = []
    for _ in range(N_obs):
        r = random.randint(2, 10)
        x = random.randint(0+r, 100-r)
        y = random.randint(0+r, 100-r)
        obs.append(Obstacle(r,[x,y]))
    return obs

def Create_Goal_Zones(N_goals, obs):
    """
    Creates the goal zones for our problem
    :param N_goals: the number of goal zones to instantiate
    :param obs: list of the obstacle instances in our problem
    :returns: list of the goal zone instances
    """
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
    """
    Creates the start node for our problem
    :param obs: list of the obstacle instances in our problem
    :param goals: list of the goal zone instances
    :returns: the start node instance
    """
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
    argus = options()
    N_goals = argus.g
    show_animation = argus.a
    
    N_obs = random.randint(5, 20)

    map = Map(100,100)
    obs = Create_Obstacles(N_obs)
    goals = Create_Goal_Zones(N_goals, obs)
    start = Create_Start_Node(obs, goals)
    
    samples, goal_nodes = Create_Samples(map, start, obs, goals, N_samples=200)
    roadmap, PRM_graph = Create_Roadmap(samples, obs, N_knn=5)
    new_graph, trajectories = PRM_Solve(start, goals, samples, goal_nodes, roadmap, PRM_graph)
    final_trajectory = TaskPlanner(new_graph, save_pddl=True).GetFinalTrajectory()
    if final_trajectory == None:
        return
        
    plotter = Plotter(map, obs, goals, start, samples=samples, roadmap=roadmap, PRM_graph=PRM_graph, new_graph=new_graph, trajectories=trajectories, final_traj=final_trajectory)
    show_animation = False
    if show_animation:
        plotter.plot_init()
        plotter.plot_PRM()
        plotter.plot_Dijkstra()
        plotter.Visualize_Final_Graph()
    else:
        plotter.show_anim = False
        plotter.plot4()
    plotter.plot_final()
    return

def options():
    parser = argparse.ArgumentParser(description="Online decoder for packets extracted with netio_cat or ", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    #parser.add_argument("-t", dest='t', default='MON', choices=['L1A','MON'], help="Channel type")
    parser.add_argument("-g", dest='g', required=False, default=3, help="Number of goals to create")
    parser.add_argument("-a", dest='a', default=False, type=str, help="Input patterns file")
    parser.add_argument("-chan", dest='chan', default='Input/BandStripCountsCombined.csv', type=str, help="Input strip channels file")
    parser.add_argument("-b", dest='b', default='Input/BandTds.csv', type=str, help="Input band tds file")
    parser.add_argument("-ab", dest='ab', default='Input/sTGC_AB_Mapping_WithPadTDSChannels.csv', type=str, help="Input ab mapping file")
    parser.add_argument("-strips", dest='strips', default='Input/Strips_Names.csv', type=str, help="Input strips names file")
    parser.add_argument('-o', metavar='create_output', required=False, default=False, help='Declare whether or not you want the script to produce an excel file with the relevant data')
    return parser.parse_args()

if __name__ == '__main__':
    main()