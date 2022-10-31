import numpy as np
from math import sin, cos
from Robot.joint_state import JointState
import copy

class Robot:
    def __init__(self, n_dof=5, link_length = 10, base = [0,0], min_joint_limit = -np.pi/2, max_joint_limit = np.pi/2):
        self.n_dof = n_dof
        self.link_length = link_length
        self.base = base
        self.joint_values = [JointState() for _ in range(n_dof)]
        self.min_joint_limit = min_joint_limit
        self.max_joint_limit = max_joint_limit
        self.joint_positions = [np.array([i*self.link_length, 0]) for i in range(self.n_dof)]
        self.gripper_position = np.array([(self.n_dof)*self.link_length, 0])

    def rotation_mat_2D(self, theta):
        mat = np.array([[cos(theta), -sin(theta)],
                       [sin(theta), cos(theta)]])
        return mat

    def set_joint_values(self, angles):
        for i,angle in enumerate(angles):
            self.joint_values[i].set_state(angle)

    def forward_kinematics(self):
        current_gripper_pos = copy.deepcopy(self.gripper_position)
        self.joint_positions = [np.array([i*self.link_length, 0]) for i in range(self.n_dof)]
        self.gripper_position = np.array([(self.n_dof)*self.link_length, 0])
        self.joint_positions[0] = np.array([0,0])
        theta_current = self.joint_values[0].value
        
        for i in range(1, self.n_dof):
            self.joint_positions[i] = np.add(self.joint_positions[i-1], np.array([cos(theta_current), sin(theta_current)]).dot(self.link_length))
            theta_current += self.joint_values[i].value
        self.gripper_position =  np.add(self.joint_positions[-1], np.array([cos(theta_current), sin(theta_current)]).dot(self.link_length))
        # if np.array_equal(self.gripper_position,current_gripper_pos):
        #     print("no change")
        # else:
        #     print("gripper_pos = ", self.gripper_position)