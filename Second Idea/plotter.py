import matplotlib.pyplot as plt
import numpy as np
from robot import Robot

class Plotter:
    def __init__(self, robot):
        self.robot = robot
        self.fig = plt.figure()

    def plot_robot(self):
        xs = [joint_pos[0] for joint_pos in self.robot.joint_positions]
        ys = [joint_pos[1] for joint_pos in self.robot.joint_positions]
        xs = np.array_split(xs, self.robot.n_dof/2)
        ys = np.array_split(ys, self.robot.n_dof/2)
        points = []
        for i in range(len(xs)):
            points.append(xs)
            points.append(ys)
        print(points)
        plt.plot(*points, marker = 'o')
        plt.show()
        


if __name__ == '__main__':
    robot = Robot()
    plotter = Plotter(robot)

    plotter.plot_robot()


