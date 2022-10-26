import numpy as np
from math import sin, cos
from joint_state import JointState

class Robot:
    def __init__(self, n_dof=6, link_length = 10, base = [0,0]):
        self.n_dof = n_dof
        self.link_length = link_length
        self.base = base
        self.joint_values = [JointState() for _ in range(n_dof)]
        self.joint_positions = [np.array([i*self.link_length, 0]) for i in range(self.n_dof)]

    def rotation_mat_2D(self, theta):
        mat = np.array[[cos(theta), -sin(theta)],
                       [sin(theta), cos(theta)]]
        return mat

    def forward_kinematics(self, x, y):
        R = self.mat(self.joint_values[0].value)
        for i in range(1,self.n_dof):
            self.joint_positions[i] = R.dot(self.joint_positions[i])
            R = np.dot(R,self.rotation_mat_2D(self.joint_values[i].value))



    