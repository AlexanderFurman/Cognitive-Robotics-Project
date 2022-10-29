from Planner.data_structures import *

class RRTPlanner:
    def __init__(self, environment):
        self.environment = environment
        self.tree = Tree()
