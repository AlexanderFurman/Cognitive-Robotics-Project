import operator
import numpy

class RRTTree(object):
    
    def __init__(self, planning_env, task="mp"):
        
        self.planning_env = planning_env
        self.task = task
        self.vertices = {}
        self.edges = {}

        # inspecion planning properties
        if self.task == "ip":
            self.max_coverage = 0
            self.max_coverage_id = 0

    def add_vertex(self, config, inspected_points=None):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices[vid] = RRTVertex(config=config, inspected_points=inspected_points)

        # check if vertex has the highest coverage so far, and replace if so
        if self.task == "ip":
            v_coverage = self.planning_env.compute_coverage(inspected_points=inspected_points)
            if v_coverage > self.max_coverage:
                self.max_coverage = v_coverage
                self.max_coverage_id = vid

        return vid

    def add_edge(self, sid, eid, edge_cost=0):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges[eid] = sid
        self.vertices[eid].set_cost(cost=self.vertices[sid].cost + edge_cost)

    def is_goal_exists(self, config):
        '''
        Check if goal exists.
        @param config Configuration to check if exists.
        '''
        goal_idx = self.get_idx_for_config(config=config)
        if goal_idx is not None:
            return True
        return False

    def get_vertex_for_config(self, config):
        '''
        Search for the vertex with the given config and return it if exists
        @param config Configuration to check if exists.
        '''
        v_idx = self.get_idx_for_config(config=config)
        if v_idx is not None:
            return self.vertices[v_idx]
        return None

    def get_idx_for_config(self, config):
        '''
        Search for the vertex with the given config and return the index if exists
        @param config Configuration to check if exists.
        '''
        valid_idxs = [v_idx for v_idx, v in self.vertices.items() if (v.config == config).all()]
        if len(valid_idxs) > 0:
            return valid_idxs[0]
        return None

    def get_nearest_config(self, config):
        '''
        Find the nearest vertex for the given config and returns its state index and configuration
        @param config Sampled configuration.
        '''
        # compute distances from all vertices
        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.planning_env.robot.compute_distance(config, vertex.config))

        # retrieve the id of the nearest vertex
        vid, _ = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid].config

class RRTVertex(object):

    def __init__(self, config, cost=0, inspected_points=None):

        self.config = config
        self.cost = cost
        self.inspected_points = inspected_points

    def set_cost(self, cost):
        '''
        Set the cost of the vertex.
        '''
        self.cost = cost