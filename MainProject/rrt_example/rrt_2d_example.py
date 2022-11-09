from abstract_rrt import AbstractRRT
import numpy as np
import matplotlib.pyplot as plt

class RRT_2D(AbstractRRT):
    def __init__(self, start_state, target_state, step, x_boundaries, y_boundaries, target_sampling_probability=0.05, isRRT_star=False, max_num_iteration=10000, sample_close_to_path=False, rrt_radius_factor=1., target_threshold=0):
        super().__init__(start_state, target_state, step, target_sampling_probability, isRRT_star, max_num_iteration, sample_close_to_path, rrt_radius_factor, target_threshold)
        self._x_boundaries = x_boundaries
        self._y_boundaries = y_boundaries
        
    
    def _sample_state(self):
        # prefer points that are closer to the reference line
        from scipy import stats
        x = np.array([k/100. for k in range(100)]) * self._x_boundaries[1]
        ref_line = np.array([x, x]).T
        max_dist = self._x_boundaries[1] * np.sqrt(2)
        while True:
            sample_point = np.array([self._x_boundaries[0], self._y_boundaries[0]]) + np.array([np.diff(self._x_boundaries)[0], np.diff(self._y_boundaries)[0]]) * np.random.rand(2)
            sample_value = np.random.rand(1)[0] 
            dist = np.min(np.linalg.norm(sample_point - ref_line, axis=(1)))
            if dist == 0:
                return sample_point
            prob = (1. - dist/max_dist)**5
            if prob > sample_value:
                return sample_point


    def _states_dist(self, states1, states2):
        '''
        Must support vectorized calculations
        '''
        return np.linalg.norm(states1 - states2, axis=-1)
    

    # def _lookahead_edge_is_collision(self, state1, state2):
    #     raise NotImplementedError


    def _edge_is_collision(self, state1, state2):
        return False


    # def _lookahead_state_is_collision(self, state):
    #     raise NotImplementedError


    def _state_is_collision(self, state):
        return False

    
    
    def plot(self):
        plt.cla()
        plt.scatter(self._states[:,0], self._states[:,1], c='k', s=1)
        plt.scatter([self.target_node().state()[0]], [self.target_node().state()[1]], c='r')
        plt.scatter([self.start_node().state()[0]], [self.start_node().state()[1]], c='g')
        self._plot_path(self.target_node_id())
        plt.axis('equal')
                


    def _plot_edges_recursive(self, node_id, color='k'):
        for successor_id in self._nodes[node_id].successors():
            # print(node_id, self._nodes[node_id].successors())
            plt.plot([self._nodes[node_id].state()[0], self._nodes[successor_id].state()[0]], [self._nodes[node_id].state()[1], self._nodes[successor_id].state()[1]], color)
            # try:
            #     plt.plot([self._nodes[node_id].state()[0], self._nodes[successor_id].state()[0]], [self._nodes[node_id].state()[1], self._nodes[successor_id].state()[1]], color)
            # except:
            #     import pdb; pdb.set_trace()
            self._plot_edges_recursive(successor_id)


    def _plot_path(self, node_id, color='b'):
        predecessor_id = self._nodes[node_id].predecessor()
        if predecessor_id is None:
            return
        curr_id = node_id
        while curr_id != self.start_node_id():
            predecessor_id = self._nodes[curr_id].predecessor()
            plt.plot([self._nodes[curr_id].state()[0], self._nodes[predecessor_id].state()[0]], [self._nodes[curr_id].state()[1], self._nodes[predecessor_id].state()[1]], color)
            curr_id = predecessor_id

# np.random.seed(12344)
rrt = RRT_2D(np.array([0.,0.]), np.array([10.,10.]), .5, np.array([0.,10.]), np.array([0.,10.]), target_sampling_probability=0.01, isRRT_star=True, max_num_iteration=5000, sample_close_to_path=False, rrt_radius_factor=5., target_threshold=0)
if rrt.run_RRT():
    rrt.plot()
    plt.show()
