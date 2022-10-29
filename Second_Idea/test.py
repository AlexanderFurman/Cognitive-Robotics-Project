import sys

from Robot.joint_state import JointState
path = '/'.join(__file__.split('/')[:-2])
if path not in sys.path:
    sys.path.append('/'.join(__file__.split('/')[:-2]))
    
from Plotter.plotter import Plotter
from matplotlib import pyplot as plt
from Robot.robot import Robot
from Environment.obstacle import Obstacle
from Environment.environment import Environment
import numpy as np

if __name__ == '__main__':
    robot = Robot(4)
    # obstacle = Obstacle(3, np.array([15,15]))
    obstacle1 = Obstacle(1, np.array([5,5]))
    obstacle2 = Obstacle(1, np.array([10,5]))
    obstacle3 = Obstacle(1, np.array([20, 25]))
    target1 = np.array([8,3])
    target2 = np.array([5,8])
    target3 = np.array([20,25])
    environment = Environment(robot, [obstacle1, obstacle2, obstacle3], [target1, target2, target3])
    # robot.forward_kinematics()
    plotter = Plotter(environment)

    # joint_states_i, joint_states_f = [JointState() for _ in range(4)], [JointState(3.14/4), JointState(3.14/4), JointState(-3.14/2), JointState()]
    # plotter.generate_trajectory(joint_states_i, joint_states_f, 5, 30)

    plotter.interactive_plot()

