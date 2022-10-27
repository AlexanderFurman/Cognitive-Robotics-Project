import numpy as np

class Environment:
    def __init__(self, robot, obstacles, targets):
        self.robot = robot
        self.obstacles = obstacles
        self.targets = targets

    def distance(self, p1, p2):
        return np.linalg.norm(p2-p1)

    def triangle_area(self, a, b, c):
        ab = np.array([b[0]-a[0], b[1]-a[1]])
        ac = np.array([c[0]-a[0], c[1]-a[1]])
        
        return np.cross(ab, ac)/2

    def query_link_collision(self, radius, o, p1, p2):
        #this code which has been edited out checks between line and circle,
        #whereas the other code is between line-SEGMENT and circle
        # min_dist = 2*self.triangle_area(o, p1, p2)/self.distance(p1, p2)
        # if min_dist <= radius

        o_p1 = p1-o
        o_p2 = p2-o
        p1_p2 = p2-p1

        min_dist = np.Inf
        max_dist = max(self.distance(o,p1), self.distance(o,p2))

        if o_p1.dot(p1_p2) > 0 and o_p2.dot(p1_p2) > 0:
            min_dist = 2*self.triangle_area(o, p1, p2)/self.distance(p1, p2)
        else:
            min_dist = min(self.distance(o, p1), self.distance(o,p2))

        if min_dist <= radius and max_dist >= radius:
            return True
        else:
            return False

    def query_robot_collision(self):
        for i in range(len(self.robot.joint_positions)-1):
            for j in range(len(self.obstacles)):
                if self.query_link_collision(self.obstacles[j].radius, self.obstacles[j].centre, self.robot.joint_positions[i], self.robot.joint_positions[i+1]):
                    return True
                
        for i in range(len(self.obstacles)):
            for j in range(len(self.robot.joint_positions)-1):
                if self.query_link_collision(self.obstacles[i].radius, self.obstacles[i].centre, self.robot.joint_positions[j], self.robot.joint_positions[j+1]):
                    return True
            if self.query_link_collision(self.obstacles[i].radius, self.obstacles[i].centre, self.robot.joint_positions[-1], self.robot.gripper_position):
                return True
        return False


    