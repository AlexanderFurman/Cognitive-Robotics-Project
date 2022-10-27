import matplotlib.pyplot as plt
import numpy as np
from robot import Robot
from joint_state import JointState
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
import os
from math import pi

class Plotter:
    def __init__(self, robot):
        self.robot = robot
        self.fig, self.ax = plt.subplots()
        self.frames= []
        self.ln, = self.ax.plot([],[],'ro')


    def connect_points(self, array, point1, point2):
        xs = [point1[0], point2[0]]
        ys = [point1[1], point2[1]]
        array.append(xs)
        array.append(ys)
        return


    def generate_cartesian_points(self):
        xs = [joint_pos[0] for joint_pos in self.robot.joint_positions]
        ys = [joint_pos[1] for joint_pos in self.robot.joint_positions]
        points = []
        for i in range(len(xs)-1):
            self.connect_points(points,[xs[i],ys[i]], [xs[i+1],ys[i+1]])
        # print(points)
        # plt.plot(*points, marker = 'o')
        # plt.show()
        return points

    def generate_config_waypoints(self,initial_joint_states, final_joint_states, n_frames, traj_type):
        waypoints = np.array(initial_joint_states)
        a_i = np.array([])
        a_f = np.array([])
        for joint_state_i, joint_state_f in zip(initial_joint_states, final_joint_states):
            a_i = np.append(a_i, joint_state_i.value)
            a_f = np.append(a_f, joint_state_f.value)

        if traj_type == 'linear':
            for i in range(1,n_frames): 
                joint_states = np.array([])

                a_t = a_i + ((a_f-a_i)/(n_frames-1))*i

                for j in range(self.robot.n_dof):
                    joint_states = np.append(joint_states, JointState(a_t[j]))
                print("waypoints = ", waypoints)
                print("joint_states = ", joint_states)
                waypoints = np.vstack([waypoints, joint_states])

        return waypoints
        
    def generate_trajectory(self, initial_joint_states, final_joint_states, framerate = 5, n_frames = 25, traj_type = 'linear'):
        waypoints = self.generate_config_waypoints(initial_joint_states, final_joint_states, n_frames, traj_type)
        for i in range(len(waypoints)):
            vals = []
            for joint_state in waypoints[i]:
                vals.append(joint_state.value)
            self.robot.set_joint_values(vals)
            self.robot.forward_kinematics()
        self.frames.append(self.generate_cartesian_points())

        ani = FuncAnimation(self.fig, self.animate, frames=n_frames, interval=1000/framerate, repeat=False)
        i = 0
        while os.path.exists("plots/lin_animation%i.gif" % i):
            i += 1

        ani.save("plots/lin_animation%i.gif" %i, dpi=300, writer=PillowWriter(fps=framerate))

    def animate(self):
        for frame in self.frames:
            


if __name__ == '__main__':
    robot = Robot(3)
    plotter = Plotter(robot)

    joint_state_i, joint_state_f = [JointState() for _ in range(3)],[JointState(pi) for _ in range(3)]

    plotter.generate_trajectory(joint_state_i, joint_state_f)


