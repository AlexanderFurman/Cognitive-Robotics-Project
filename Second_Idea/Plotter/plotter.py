# import sys
# path = '/'.join(__file__.split('/')[:-2])
# if path not in sys.path:
#     sys.path.append('/'.join(__file__.split('/')[:-2]))
    
import matplotlib.pyplot as plt
import numpy as np
from Robot.joint_state import JointState
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
import os
from matplotlib.widgets import Slider, Button

class Plotter:
    def __init__(self, environment):
        self.environment = environment
        self.robot = environment.robot
        self.obstacles = environment.obstacles
        self.targets = environment.targets
        self.gripper_positions = []
        self.fig, self.ax = plt.subplots()
        self.frames= []
        # self.ln, = self.ax.plot([],[],'ro')
        self.collision_in_frame = []

    def plot_obstacles(self):
        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle.centre[0], obstacle.centre[1]), obstacle.radius, color='g')
            self.ax.add_patch(circle)

    def connect_points(self, array, point1, point2):
        xs = [point1[0], point2[0]]
        ys = [point1[1], point2[1]]
        array.append(xs)
        array.append(ys)
        return


    def generate_cartesian_points(self):
        xs = [joint_pos[0] for joint_pos in self.robot.joint_positions]
        ys = [joint_pos[1] for joint_pos in self.robot.joint_positions]
        xs.append(self.robot.gripper_position[0])
        ys.append(self.robot.gripper_position[1])
        points = []
        for i in range(len(xs)-1):
            self.connect_points(points,[xs[i],ys[i]], [xs[i+1],ys[i+1]])
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
                waypoints = np.vstack([waypoints, joint_states])

        #TODO: add piecewise and polynomial trajectories

        return waypoints
        
    def generate_trajectory(self, initial_joint_states, final_joint_states, framerate = 15, n_frames = 75, traj_type = 'linear'):
        waypoints = self.generate_config_waypoints(initial_joint_states, final_joint_states, n_frames, traj_type)
        for i in range(len(waypoints)):
            vals = []
            for joint_state in waypoints[i]:
                vals.append(joint_state.value)
            self.robot.set_joint_values(vals)
            self.robot.forward_kinematics()
            self.gripper_positions.append(self.robot.gripper_position)
            self.frames.append(self.generate_cartesian_points())
            self.collision_in_frame.append(self.environment.query_robot_collision())
        ani = FuncAnimation(self.fig, self.animate, frames=n_frames, interval=1000/framerate, repeat=False)
        i = 0
        while os.path.exists("Plots/lin_animation%i.gif" % i):
            i += 1

        ani.save("Plots/lin_animation%i.gif" %i, dpi=300, writer=PillowWriter(fps=framerate))

    def animate(self, i):
        self.ax.clear()
        self.ax.set_ylim(0, 22)
        self.ax.set_xlim(0,32)
        self.plot_obstacles()
        if self.collision_in_frame[i] is True:
            plt.plot(*self.frames[i], marker = 'o', color = 'r', lw=4)
        else:
            plt.plot(*self.frames[i], marker = 'o', color = 'b', lw=4)

        plt.plot(self.gripper_positions[i][0], self.gripper_positions[i][1], marker='o', color='purple')

    
    def interactive_plot(self):
        self.fig.subplots_adjust(left=0, right=1-self.robot.n_dof*.05, bottom=0.25)
        self.plot_obstacles()

        self.robot.forward_kinematics()
        self.ax.plot(*self.generate_cartesian_points(), marker = 'o')
        self.ax.set_xbound(-self.robot.n_dof*self.robot.link_length,self.robot.n_dof*self.robot.link_length)
        self.ax.set_ybound(0,self.robot.n_dof*self.robot.link_length)
        sliders = [None for _ in range(self.robot.n_dof)]
        for i in range(self.robot.n_dof):
                axis = self.fig.add_axes([1+(-self.robot.n_dof + i)*.05, 0.25, 0.05, 0.63])
                sliders[i] = Slider(
                    ax=axis,
                    label="Joint " + str(i),
                    valmin=-np.pi,
                    valmax=np.pi,
                    valinit=self.robot.joint_values[i].value,
                    orientation="vertical"
                )
        
        def update(val):
            self.robot.set_joint_values(sliders[i].val for i in range(len(sliders)))
            self.robot.forward_kinematics()
            # line.set_data(*self.generate_cartesian_points())
            self.ax.clear()
            self.plot_obstacles()
            if self.environment.query_robot_collision() is True:
                self.ax.plot(*self.generate_cartesian_points(), marker='o', color = 'r', lw=4)
            else:
                self.ax.plot(*self.generate_cartesian_points(), marker='o', color = 'b', lw=4)
            self.ax.set_xbound(-self.robot.n_dof*self.robot.link_length,self.robot.n_dof*self.robot.link_length)
            self.ax.set_ybound(0,self.robot.n_dof*self.robot.link_length)
            self.fig.canvas.draw_idle()

        # register the update function with each slider
        for slider in sliders:
            slider.on_changed(update)

        plt.show()
        
                


# if __name__ == '__main__':
#     robot = Robot(3)
#     plotter = Plotter(robot)

#     joint_state_i, joint_state_f = [JointState() for _ in range(3)],[JointState(pi) for _ in range(3)]

#     plotter.generate_trajectory(joint_state_i, joint_state_f)


