import numpy as np
from Arm_Modules.Arm_Robot import JointState, Robot
from Arm_Modules.Arm_Plotter import Plotter
from Arm_Modules.Arm_Environment import Environment, distance
from Arm_Modules.Arm_RRT_Planner import RRTPlanner
from Nav_Modules.Nav_Geometry import Obstacle

def Arm_Run(output_path, save_img=False, save_gif=False, show_anim=False):
    robot = Robot(2)
    obstacle1 = Obstacle(10, np.array([13,23]))
    obstacle2 = Obstacle(10, np.array([20, 20]))
    obstacle3 = Obstacle(10, np.array([25,0]))
    obstacle4 = Obstacle(3, np.array([10,12]))
    obstacle5 = Obstacle(3, np.array([13,13]))
    obstacle6 = Obstacle(3, np.array([16,4]))
    target1 = np.array([0,11])
    target2 = np.array([16,8])
    target3 = np.array([-1, 7])
    environment = Environment(robot, [obstacle1, obstacle2, obstacle3, obstacle5, obstacle6], [target2], epsilon = 1, is_wall = False, is_floor = True)
    plotter = Plotter(environment, output_path=output_path, save_img=save_img, show_anim=show_anim, save_gif=save_gif)

    np.random.seed(0)  
    rrt_planner = RRTPlanner(environment, 0.05, 20000)
    tree = rrt_planner.rrt([JointState(np.pi/2), JointState(np.pi/2)])
    paths = rrt_planner.generate_paths()

    if not show_anim:
        plotter.show_rrt(tree.nodes, paths)
    plotter.rrt_plot(paths[0], tree.nodes, rrt_planner.random_samples, rrt_planner.nearest_nodes)
    
    min_length = 100000000
    current_pos = rrt_planner.gripper_values[0]
    for pos in rrt_planner.gripper_values:
        if distance(pos, target3) < min_length:
            min_length = distance(pos, target3)
            current_pos = pos

    print("Gripper position found which is closest to goal: ", current_pos)
    print(f"Min. distance to target: {min_length:0.5f}\n")
    nodes = paths[0]
    states = [nodes[i].joint_values for i in range(len(nodes))]

    if save_gif or show_anim:
        plotter.generate_trajectory(states, output_path, n_frames = 3)
    return

if __name__ == '__main__':
    Arm_Run()