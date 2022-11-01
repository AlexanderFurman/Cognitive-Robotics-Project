from random import random
from Planner.data_structures import *
from Robot.joint_state import JointState
import numpy as np
import matplotlib.pyplot as plt

class RRTPlanner:
    def __init__(self, environment, step, max_iterations = 10000):
        self.environment = environment
        self.step = step
        self.max_iterations = max_iterations
        self.tree = Tree()
        self.goal_nodes = []
        self.gripper_values = []
        self.random_samples = []
        self.nearest_nodes = []

        # self.tree.add_node(TreeNode(environment.robot.joint_values)) #root node given jointstates which match robots current jointstates

    def sample_random_state(self):
        # np.random.seed(1)
        node_in_tree = True
        while node_in_tree:
            rand_angles = np.random.uniform(self.environment.robot.min_joint_limit, self.environment.robot.max_joint_limit, self.environment.robot.n_dof) # check if uniform works, try also np.random.rand
            print("rand angles = ", rand_angles)
            # rand_angles = np.random.rand(2) * (self.environment.robot.max_joint_limit-self.environment.robot.min_joint_limit) + self.environment.robot.min_joint_limit
            joint_values_node = TreeNode([JointState(rand_angles[i]) for i in range(len(rand_angles))])
            node_in_tree = self.tree.query_node_in_graph(joint_values_node)
        return joint_values_node

    def query_state_cause_collision(self, angles):
        self.environment.robot.set_joint_values(angles)
        self.environment.robot.forward_kinematics()
        return self.environment.query_robot_collision()

    def query_state_reach_target(self, angles):
        self.environment.robot.set_joint_values(angles)
        self.environment.robot.forward_kinematics()
        # for target in self.environment.targets:
        return self.environment.query_robot_at_goal()
        # return False

    def extend(self, rand_node, near_node):
        rand_node_vector = rand_node.vectorized_values()
        near_node_vector = near_node.vectorized_values()
        new_node_vector = self.step*(rand_node_vector - near_node_vector)/np.linalg.norm(rand_node_vector - near_node_vector) + near_node_vector
        new_node = TreeNode([JointState(new_node_vector[i]) for i in range(len(new_node_vector))])
        if not self.tree.query_node_in_graph(new_node):
            return new_node
        else:
            print("ERROR: node already exists within tree")
            return

    def rrt(self, initial_joint_values):
        self.tree.add_node(TreeNode(initial_joint_values))
        flag = False
        for i in range(self.max_iterations):
            if len(self.goal_nodes) > 0:
                return self.tree
            print(f"iteration number: {i}")
            random_node = self.sample_random_state()
            self.random_samples.append(random_node)
            nearest_node = self.tree.nearest_neighbour(random_node)
            self.nearest_nodes.append(nearest_node)
            new_node = self.extend(random_node, nearest_node)


            if not self.query_state_cause_collision(new_node.vectorized_values()) and not self.tree.query_node_in_graph(new_node):
                self.tree.add_node(new_node)
                self.gripper_values.append(self.environment.robot.gripper_position)
                if self.query_state_reach_target(new_node.vectorized_values()):
                    new_node.set_goal_true()
                    self.goal_nodes.append(new_node)
                # self.tree.add_edge(nearest_node, new_node) # check that predecessor, successor working -- update: id doesnt
                new_node.set_predecessor(nearest_node)
                nearest_node.add_successor(new_node)
        return self.tree

    def generate_paths(self):
        #TODO generate 'shortest' path between set of nodes
        paths = []
        for goal_node in self.goal_nodes:
            path = []
            current_node = goal_node
            while current_node.predecessor is not None:
                path.append(current_node)
                current_node = current_node.predecessor
            path.append(current_node) # adds initial state
            path.reverse()
            paths.append(path)
        return paths

    

        

    