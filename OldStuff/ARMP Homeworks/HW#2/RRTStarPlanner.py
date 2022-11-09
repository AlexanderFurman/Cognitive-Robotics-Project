import numpy
from RRTTree import RRTTree

class RRTStarPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        

    def Plan(self, start_config, goal_config, step_size = 0.001):
        
        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)

        # TODO (student): Implement your planner here.
        plan.append(start_config)
        plan.append(goal_config)
        return numpy.array(plan)

    def extend(self, x_rand, x_near):
        # TODO (student): Implement an extend logic.
        pass

