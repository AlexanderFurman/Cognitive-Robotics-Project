import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
from matplotlib.widgets import Slider
from Arm_Modules.Arm_Robot import JointState
from Nav_Modules.Nav_Geometry import Rectangle, Circle

class Plotter:
    def __init__(self, environment, output_path=None, save_img=False, save_gif=False, show_anim=False ):
        self.environment = environment
        self.output_path = output_path
        self.save_img = save_img
        self.save_gif = save_gif
        self.show_anim=show_anim
        self.robot = environment.robot
        self.obstacles = environment.obstacles
        self.targets = environment.targets
        self.fig, self.ax = plt.subplots()
        self.gripper_positions = []
        self.frames= []
        self.collision_in_frame = []
        self.goal_in_frame = []

    def plot_obstacles(self):
        for obs in self.obstacles:
            obs.plot_circle(self.ax)
        return

    def plot_background(self):
        floor = Rectangle(np.array([-20,-20]), 40, 17.5, color='#e77d11')
        floor.plot_rectangle(self.ax)
        sky = Rectangle(np.array([-20,-2.5]), 40, 22.5, color='#5adeff')
        sky.plot_rectangle(self.ax)
        return
    
    def plot_boundaries(self):
        if self.environment.is_floor:
            xy = np.array([-0.5,0])*self.robot.n_dof*self.robot.link_length + np.array([0,-5])
            rect = Rectangle(xy, self.robot.n_dof*self.robot.link_length, 5)
            rect.plot_rectangle(self.ax)
        if self.environment.is_wall:
            xy = np.array([-5,0])
            rect = Rectangle(xy, 5, self.robot.n_dof*self.robot.link_length)
            rect.plot_rectangle(self.ax)
        return

    def plot_rover(self):
        wheel_r = 2.5; wheel_color = 'black'
        Circle(wheel_r, np.array([-5,-5]), wheel_color).plot_circle(self.ax)
        Circle(wheel_r, np.array([5,-5]), wheel_color).plot_circle(self.ax)
        wheel_r = 2.2; wheel_color = 'grey'
        Circle(wheel_r, np.array([-5,-5]), wheel_color).plot_circle(self.ax)
        Circle(wheel_r, np.array([5,-5]), wheel_color).plot_circle(self.ax)

        xy = np.array([-0.5,0])*self.robot.n_dof*self.robot.link_length + np.array([0,-5])
        Rectangle(xy, self.robot.n_dof*self.robot.link_length, 5).plot_rectangle(self.ax)

        wheel_r = 2.5; wheel_color = 'black'
        Circle(wheel_r, np.array([-10,-5]), wheel_color).plot_circle(self.ax)
        Circle(wheel_r, np.array([10,-5]), wheel_color).plot_circle(self.ax)
        Circle(wheel_r, np.array([0,-5]), wheel_color).plot_circle(self.ax)
        wheel_r = 2.2; wheel_color = 'grey'
        Circle(wheel_r, np.array([-10,-5]), wheel_color).plot_circle(self.ax)
        Circle(wheel_r, np.array([10,-5]), wheel_color).plot_circle(self.ax)
        Circle(wheel_r, np.array([0,-5]), wheel_color).plot_circle(self.ax)
        return

    def plot_targets(self):
        for target in self.targets:
            circle = plt.Circle((target[0], target[1]), self.environment.epsilon, color='g')
            self.ax.add_patch(circle)
        return

    def connect_points(self, array, point1, point2):
        xs = [point1[0], point2[0]]
        ys = [point1[1], point2[1]]
        array.append(xs)
        array.append(ys)
        return

    def show_rrt(self, nodes, paths):
        self.ax.clear()
        plt.grid()
        self.ax.set_ylim(self.robot.min_joint_limit,self.robot.max_joint_limit)
        self.ax.set_xlim(self.robot.min_joint_limit,self.robot.max_joint_limit)

        root_node = nodes.pop(0)
        root_vals = root_node.vectorized_values()
        plt.plot(root_vals[0], root_vals[1])
        for node in nodes:
            self.connect_parent_and_child(node)
        if len(paths) !=0:
            path = paths[0]
            root_node = path.pop(0)
            plt.plot(root_vals[0], root_vals[1], color='g', markersize = 5)
            for node in reversed(path):
                self.connect_parent_and_child(node, 'g', 5)
        plt.title("RRT C-Space")
        plt.xlabel(f"{len(nodes)} Nodes Generated Until Solution")
        if self.save_img:
            name = self.output_path + "RRT_C-Space.png"
            plt.savefig(name)
        plt.show()
        return  

    def rrt_plot(self, path, nodes, sample_nodes, nearest_nodes):
        _, axis = plt.subplots()
        self.ax = axis
        plt.grid()
        self.ax.set_ylim(-np.pi,np.pi)
        self.ax.set_xlim(-np.pi,np.pi)

        root_node = nodes.pop(0)
        root_vals = root_node.vectorized_values()
        plt.plot(root_vals[0], root_vals[1], 0.05, color='grey')
        for (node, sample_node, nearest_node) in zip(nodes, sample_nodes, nearest_nodes):
            xlabel_string = 'Nodes: ' + str(nodes.index(node)+1) + ' / ' + str(len(nodes))
            self.ax.set_xlabel(xlabel_string)
            
            sample_vec = sample_node.vectorized_values()
            node_vec = node.vectorized_values()
            near_vec = nearest_node.vectorized_values()

            circle1 = plt.Circle((sample_vec[0], sample_vec[1]), 0.05, color='blue')
            circle2 = plt.Circle((near_vec[0], near_vec[1]), 0.05, color='orange')
            circle3 = plt.Circle((node_vec[0], node_vec[1]), 0.05, color='grey')
            self.ax.add_patch(circle1)
            plt.pause(0.001)
            self.ax.add_patch(circle2)
            plt.pause(0.001)
            self.ax.add_patch(circle3)
            plt.pause(0.001)
            self.connect_parent_and_child(node)
            plt.pause(0.001)
            circle1.remove()
            circle2.remove()
            circle3.remove()
        start_vec = path[0].vectorized_values()
        goal_vec = path[-1].vectorized_values()
        circle1 = plt.Circle((start_vec[0], start_vec[1]), 0.1, color='blue')
        circle2 = plt.Circle((goal_vec[0],goal_vec[1]), 0.1, color='green')
        self.ax.add_patch(circle1)
        self.ax.add_patch(circle2)
        for node in path:
            self.connect_parent_and_child(node, color='lime')
            # xlabel_string = 'Nodes: ' + str(len(paths)) + ' / ' + str(len(path) + ', ')
            #self.ax.set_xlabel(xlabel_string)
            plt.pause(0.001)

        plt.show()
        return
        
    def connect_parent_and_child(self, child_node, color='grey', markersize = 2):
        joint_values_parent = child_node.predecessor.vectorized_values()
        joint_values_child= child_node.vectorized_values()
        xs, ys = [joint_values_parent[0], joint_values_child[0]], [joint_values_parent[1], joint_values_child[1]]
        plt.plot(xs, ys, marker = 'o', color=color, markersize=markersize, linewidth=markersize/2)
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
        return waypoints
        
    def generate_linear_trajectory(self, initial_joint_states, final_joint_states, framerate = 15, n_frames = 75, traj_type = 'linear', output_name=None):
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
        while os.path.exists(output_name+"Arm_Animation%i.gif" % i):
            i += 1
        ani.save(output_name+"Arm_Animation%i.gif" %i, dpi=300, writer=PillowWriter(fps=framerate))
        return

    def generate_trajectory(self, path, framerate = 15, n_frames = 75, traj_type = 'linear', output_name=None):
        #plt.cla()
        for i in range(len(path)-1):
            waypoints = self.generate_config_waypoints(path[i], path[i+1], n_frames, traj_type)
            for i in range(len(waypoints)):
                vals = []
                for joint_state in waypoints[i]:
                    vals.append(joint_state.value)
                self.robot.set_joint_values(vals)
                self.robot.forward_kinematics()
                self.gripper_positions.append(self.robot.gripper_position)
                self.frames.append(self.generate_cartesian_points())
                self.collision_in_frame.append(self.environment.query_robot_collision())
                self.goal_in_frame.append(self.environment.query_robot_at_goal())
        
        if self.show_anim:
            print("Animating...")
            _, axis = plt.subplots()
            self.ax = axis
            for i in range(len(self.frames)):
                self.animate(i)
                plt.pause(0.01)
            plt.show()

        if self.save_gif:
            print("in save gif")
            ani = FuncAnimation(self.fig, self.animate, frames=len(self.frames), interval=1000/framerate, repeat=False)
            i = 0
            while os.path.exists(output_name+"Arm_Animation%i.gif" % i):
                i += 1
            ani.save(output_name+"Arm_Animation%i.gif" %i, dpi=300, writer=PillowWriter(fps=framerate))
            print("Complete!")
        return

    def animate(self, i):
        self.ax.clear()
        self.ax.set_xlim(-self.robot.n_dof*self.robot.link_length,self.robot.n_dof*self.robot.link_length)
        self.ax.set_ylim(-self.robot.n_dof*self.robot.link_length,self.robot.n_dof*self.robot.link_length)
        self.plot_background()
        self.plot_obstacles()
        self.plot_targets()
        self.plot_rover()
        if self.collision_in_frame[i] is True:
            self.ax.plot(*self.frames[i], marker = 'o', color = 'r', lw=4)
        elif self.goal_in_frame[i] is True:
            self.ax.plot(*self.frames[i], marker = 'o', color = 'lime', lw=4)
        else:
            self.ax.plot(*self.frames[i], marker = 'o', color = '#cccccc', lw=4)

        self.ax.set_title("Final Rover Arm Motion (W-Space)")
        self.ax.set_xlabel("x [dm]")
        self.ax.set_ylabel("y [dm]")
        self.ax.plot(self.gripper_positions[i][0], self.gripper_positions[i][1], marker='o', color='purple', markersize = 10)
        return
       
    def interactive_plot(self):
        self.fig.subplots_adjust(left=0, right=1-self.robot.n_dof*.05, bottom=0.25)
        self.plot_obstacles()
        self.plot_boundaries()
        self.plot_targets()

        self.robot.forward_kinematics()
        self.ax.plot(*self.generate_cartesian_points(), marker = 'o')
        self.ax.plot(self.robot.gripper_position[0], self.robot.gripper_position[1], marker='o', color='purple', markersize=10)
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
            self.ax.clear()
            self.plot_obstacles()
            self.plot_boundaries()
            self.plot_targets()
            if self.environment.query_robot_collision() is True:
                self.ax.plot(*self.generate_cartesian_points(), marker='o', color = 'r', lw=4)
            else:
                self.ax.plot(*self.generate_cartesian_points(), marker='o', color = 'b', lw=4)

            self.ax.plot(self.robot.gripper_position[0], self.robot.gripper_position[1], marker='o', color='purple', markersize=10)
            self.ax.set_xbound(-self.robot.n_dof*self.robot.link_length,self.robot.n_dof*self.robot.link_length)
            self.ax.set_ybound(0,self.robot.n_dof*self.robot.link_length)
            self.fig.canvas.draw_idle()

        # register the update function with each slider
        for slider in sliders:
            slider.on_changed(update)

        plt.show()
        return