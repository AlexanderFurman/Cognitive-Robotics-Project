import random, argparse
from Nav_Modules.Nav_Geometry import *
from Nav_Modules.Nav_MotionPlanner import *
from Nav_Modules.Nav_Plotter import *
from Nav_Modules.Nav_TaskPlanner import *
from Arm_Run import Arm_Run

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

def CreateOutputFolder(save_imgs,save_gifs,save_pddl):
    if not os.path.exists("Output"):
        os.mkdir("Output")

    temp_folder = "Output/Temp_Images"
    if not os.path.exists(temp_folder):
        os.mkdir(temp_folder)
    elif os.path.exists(temp_folder):
        for filename in os.listdir(temp_folder):
            file_path = os.path.join(temp_folder, filename)
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
        
    path = None
    if save_imgs or save_gifs or save_pddl:
        i = 0
        while os.path.exists("Output/%i/" % i):
            i += 1
        path = "Output/%i/" % i
        os.mkdir(path)
    return path

def main():
    argus = options()
    N_goals = argus.n; N_samples = argus.s; show_knn = eval(argus.k); show_animation = eval(argus.a)
    save_pddl = eval(argus.p); save_gifs = eval(argus.g); save_imgs = eval(argus.i); show_arm = eval(argus.arm); show_nav = eval(argus.nav)
    if argus.o is not None:
        N_obs = argus.o
    else:
        N_obs = random.randint(5, 20)
    path = CreateOutputFolder(save_imgs,save_gifs,save_pddl)

    print("** Starting the Ignorance simulation **\n")
    if show_nav:
        map = Map()
        obs = Create_Obstacles(N_obs)
        goals = Create_Goal_Zones(N_goals, obs)
        start = Create_Start_Node(obs, goals)
        
        samples, goal_nodes = Create_Samples(map, start, obs, goals, N_samples=N_samples)
        roadmap, PRM_graph = Create_Roadmap(samples, obs, N_knn=5)
        new_graph, trajectories = PRM_Solve(start, goals, samples, goal_nodes, roadmap, PRM_graph)
        final_trajectory = TaskPlanner(new_graph, save_pddl=save_pddl, output_path=path).GetFinalTrajectory()
        if final_trajectory == None:
            return
            
        plotter = Plotter(map, obs, goals, start, samples, roadmap, PRM_graph, new_graph, trajectories, final_trajectory, output_path=path, save_img=save_imgs, save_gif=save_gifs, show_anim=show_animation)
        if show_animation or show_knn or save_gifs: 
            plotter.plot_init()
            plotter.plot_PRM()
            if show_knn:
                plotter.plot_knn()
            plotter.plot_Dijkstra()
            plotter.Visualize_Final_Graph()
        else:
            plotter.plot4()
        plotter.plot_final()

    if show_arm:
        Arm_Run(output_path=path, save_img=save_imgs, save_gif=save_gifs, show_anim=show_animation)
    print("** Ignorance simulation complete! **")
    return

def options():
    parser = argparse.ArgumentParser(description="Integrated Task & Motion Planning for a Simplified Mars Rover Exploration Problem", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-nav", dest='nav', required=False, default="True", help="Run the navigation simulation")
    parser.add_argument("-arm", dest='arm', required=False, default="True", help="Run the arm simulation")
    parser.add_argument("-n", dest='n', required=False, default=3, help="Number of goals to create")
    parser.add_argument("-o", dest='o', required=False, default=None, help="Number of obstacles to create (default will randomize the number of obstacles)")
    parser.add_argument("-s", dest='s', required=False, default=200, help="Number of samples for PRM")
    parser.add_argument("-a", dest='a', required=False, default="False", help="Show animations")
    parser.add_argument("-k", dest='k', required=False, default="False", help="Show the k-nearest neighbors animation")
    parser.add_argument("-p", dest='p', required=False, default="False", help="Save .pddl files")
    parser.add_argument("-i", dest='i', required=False, default="False", help="Save .png image files")
    parser.add_argument("-g", dest='g', required=False, default="False", help="Save .gif animation files")
    return parser.parse_args()

if __name__ == '__main__':
    main()