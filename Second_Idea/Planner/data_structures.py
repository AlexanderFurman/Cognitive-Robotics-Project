from Robot.joint_state import JointState
import numpy as np

class Node:
    def __init__(self, joint_values):
        self.joint_values = joint_values #list of JointStates --> for robot with 3dof: need [JointState(angle1), JointState(angle2), JointState(angle3)]
        self.is_goal = False
    
    def set_goal_true(self):
        self.goal = True

    def vectorized_values(self):
        return np.array([self.joint_values[i].value for i in range(len(self.joint_values))])

    def equal_to(self, node):
        return np.array_equal(node.vectorized_values, self.vectorized_values)

class Edge:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
        self.weight = np.linalg.norm(node1.vectorized_values() - node2.vectorized_values())

    def vectorized_values(self):
        return np.array([self.node1.vectorized_values, self.node2.vectorized_values])

    def equal_to(self, edge):
        return np.array_equal(edge.vectorized_values, self.vectorized_values)

class Graph:
    def __init__(self, bidirected = True):
        self.nodes = []
        self.edges = []
        self.bidirected = bidirected

    def add_node(self, node):
        self.nodes.append(node)

    def add_edge(self, node1, node2):
        self.edges.append(node1, node2)

    def nearest_neighbour(self, node):
        distances = [np.linalg.norm(node.vectorized_values() - node_i.vectorized_values()) for node_i in self.nodes]
        min_idx = np.argmin(distances)
        return self.nodes[min_idx]

    def query_node_in_graph(self, node):
        for node_i in self.nodes:
            if node_i.is_equal(node):
                return True
        return False

    def query_edge_in_graph(self, edge):
        for edge_i in self.edges:
            if self.bidirected:
                if edge_i.is_equal(edge) or edge_i.is_equal(np.flip(edge)):
                    return True
            else:
                if edge_i.is_equal(edge):
                    return True
        return False



class TreeNode(Node):
    def __init__(self, joint_values, predecessor = None):
        super().__init__(joint_values)
        self.predecessor = predecessor
        self.successors = []
    
    def add_successor(self, successor):
        self.successors.append(successor)

    def set_predecessor(self, predecessor):
        self.predecessor = predecessor

    def query_root(self):
        if self.predecessor is None:
            return True
        return False

    def query_succesors(self):
        if len(self.successors) > 0:
            return True
        return False

class Tree(Graph):
    def __init__(self):
        super().__init__()

    def add_edge(self, node1, node2):
        if super().query_node_in_graph(node1) and super().query_node_in_graph(node2):
            print("ERROR: cannot add edge between nodes who both have parents")
        super().add_edge(node1, node2)
        if super().query_node_in_graph(node1):
            node1.add_successor(node2)
            node2.set_predecessor(node1)
        else:
            node2.add_successor(node1)
            node1.set_predecessor(node2)


        


    