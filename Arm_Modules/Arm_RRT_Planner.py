import numpy as np
from Arm_Modules.Arm_DataStructures import *
from Arm_Modules.Arm_Robot import JointState

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

    def sample_random_state(self):
        node_in_tree = True
        while node_in_tree:
            rand_angles = np.random.uniform(self.environment.robot.min_joint_limit, self.environment.robot.max_joint_limit, self.environment.robot.n_dof) # check if uniform works, try also np.random.rand
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
        return self.environment.query_robot_at_goal()

    def extend(self, rand_node, near_node):
        rand_node_vector = rand_node.vectorized_values()
        near_node_vector = near_node.vectorized_values()
        new_node_vector = self.step*(rand_node_vector - near_node_vector)/np.linalg.norm(rand_node_vector - near_node_vector) + near_node_vector
        new_node = TreeNode([JointState(new_node_vector[i]) for i in range(len(new_node_vector))])
        if not self.tree.query_node_in_graph(new_node):
            return new_node
        else:
            print("ERROR: Node already exists within tree...")
            return

    def rrt(self, initial_joint_values):
        self.tree.add_node(TreeNode(initial_joint_values))
        print("Running the RRT Algorithm...")
        for i in range(self.max_iterations):
            if len(self.goal_nodes) > 0:
                print(f"Finished running the RRT Algorithm, after {i} iterations.\n")
                return self.tree
            random_node = self.sample_random_state()
            nearest_node = self.tree.nearest_neighbour(random_node)
            new_node = self.extend(random_node, nearest_node)

            if not self.query_state_cause_collision(new_node.vectorized_values()) and not self.tree.query_node_in_graph(new_node):
                self.nearest_nodes.append(nearest_node)
                self.random_samples.append(random_node)
                self.tree.add_node(new_node)
                self.gripper_values.append(self.environment.robot.gripper_position)
                if self.query_state_reach_target(new_node.vectorized_values()):
                    new_node.set_goal_true()
                    self.goal_nodes.append(new_node)
                new_node.set_predecessor(nearest_node)
                nearest_node.add_successor(new_node)
        print(f"Finished running the RRT Algorithm, after {i} iterations.\n")
        return self.tree

    def rrt_star(self, initial_joint_values):
        self.tree.add_node(TreeNode(initial_joint_values))
        for i in range(self.max_iterations):
            if len(self.goal_nodes) > 0:
                return self.tree
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
                new_node.set_predecessor(nearest_node)
                new_node.update_cost(self.step)
                nearest_node.add_successor(new_node)

                #The following lines add the rewiring capabilities of rrt*:
                nearest_neighbours= self.tree.neareast_neighbours(new_node, i)
                for node_i in nearest_neighbours:
                    self.rewire_rrt_star(node_i, new_node)
                for node_i in nearest_neighbours:
                    self.rewire_rrt_star(new_node, node_i)
        return self.tree

    def rewire_rrt_star(self, node_i, node_j):
        if node_j.predecessor is None:
            return
        else:
            if node_i.cost < node_j.predecessor.cost:
                print("Rewiring...")
                if self.passed_collision_test:
                    node_j.predecessor.remove_successor(node_j)
                    node_j.set_predecessor(node_i)
                    node_j.update_cost(self.step)
                    node_i.add_successor(node_j)
        return

    def passed_collision_test(self, node_i, node_j, steps =5):
        vectors = np.linspace(node_i.vectorized_values(), node_j.vectorized_values, steps)
        for vector in vectors:
            if self.query_state_cause_collision(vector):
                return False
        return True
        
    def generate_paths(self):
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