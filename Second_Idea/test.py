from pydoc import pathdirs
import sys

from Robot.joint_state import JointState
import os
print("current working dir: ", os.getcwd())
# path = '/'.join(__file__.split('/')[:-2])
# if path not in sys.path:
#     sys.path.append('/'.join(__file__.split('/')[:-2]))
# print("current working dir: ", os.getcwd())
print("paths = ", sys.path)
from Plotter.plotter import Plotter
from matplotlib import pyplot as plt
from Robot.robot import Robot
from Environment.obstacle import Obstacle
from Environment.environment import Environment
from Planner.rrt_planner import RRTPlanner
import numpy as np
from Planner.data_structures import TreeNode

def distance(p1, p2):
        return np.linalg.norm(p2-p1)

if __name__ == '__main__':
    robot = Robot(2)
    # obstacle = Obstacle(3, np.array([15,15]))
    obstacle1 = Obstacle(1, np.array([13,5]))
    obstacle2 = Obstacle(1, np.array([10,5]))
    obstacle3 = Obstacle(1, np.array([20, 20]))
    target1 = np.array([0,11])
    target2 = np.array([12,7])
    target3 = np.array([-1, 7])
    environment = Environment(robot, [obstacle1, obstacle2], [target2], epsilon = 1, is_wall = False, is_floor = False)
    # robot.forward_kinematics()
    plotter = Plotter(environment)
    # plotter.interactive_plot()
    # joint_states_i, joint_states_f = [JointState() for _ in range(4)], [JointState(3.14/4), JointState(3.14/4), JointState(-3.14/2), JointState()]
    # plotter.generate_trajectory(joint_states_i, joint_states_f, 5, 30)

    # plotter.interactive_plot()
    # np.random.seed(4)
    np.random.seed(5)
    
    rrt_planner = RRTPlanner(environment, 0.05, 3000)
    tree = rrt_planner.rrt([JointState(0) for _ in range(2)])
    paths = rrt_planner.generate_paths()
    plotter.show_rrt(tree.nodes, paths)
    min_length = 100000000
    current_pos = rrt_planner.gripper_values[0]
    for pos in rrt_planner.gripper_values:
        if distance(pos, target3) < min_length:
            min_length = distance(pos, target3)
            current_pos = pos

    print("gripper pos found which is closest to goal: ", current_pos)
    print("min dist to target:", min_length)
    # print(paths[0][-1].vectorized_values())
    print(paths[0][i].vectorized_values() for i in range(len(paths[0])))
    nodes = paths[0]
    states = [nodes[i].joint_values for i in range(len(nodes))]
    plotter.generate_trajectory(states, n_frames = 3)

    # fig,ax = plt.subplots()
    # plt.plot()

