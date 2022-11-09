import pdb
import numpy as np
from pyrsistent import s
import tqdm
import matplotlib.pyplot as plt
import copy

class Node:
    def __init__(self, id, state, g_value, successors_ids=[], predecessor_id=None):
        '''
        _state is an n-degree np.array (in an nDOF system)
        '''
        self._state = state
        self._g_value = g_value
        self._id = id
        self._successors_ids = copy.copy(successors_ids)
        self._predecessor_id = predecessor_id
    
    def add_successor(self, successor_id):
        if successor_id not in self._successors_ids:
            self._successors_ids.append(successor_id)

    def remove_successor(self, successor_id):
        if successor_id not in self._successors_ids:
            self._successors_ids.remove(successor_id)

    def set_predecessor(self, predecessor_id):
        self._predecessor_id = predecessor_id

    def set_g_value(self, g_value):
        self._g_value = g_value
        
    def successors(self):
        return self._successors_ids

    def predecessor(self):
        return self._predecessor_id
    
    def id(self):
        return self._id
    
    def state(self):
        return self._state

    def g_value(self):
        return self._g_value


class AbstractRRT:
    def __init__(self, 
                 start_state, 
                 target_state,
                 step,
                 target_sampling_probability=0.05, 
                 isRRT_star=False, 
                 max_num_iteration=10000, 
                 sample_close_to_path=False,
                 rrt_radius_factor=1.,
                 target_threshold=0.):
        '''
        A state is an n-degree np.array (in an nDOF system)
        start_state - start configuration
        target_state - target configuration
        step - step of the RRT
        _nodes and _states have corelated keys and indices respectively
        '''
        self._step = step
        self._nodes = {0: Node(0, start_state, 0)}
        self._states = np.array([start_state])
        self._max_node_id = 0
        self._target_state = target_state
        self._target_sampling_probability = target_sampling_probability
        self._isRRT_star = isRRT_star
        self._max_num_iteration = max_num_iteration
        self._sample_close_to_path = sample_close_to_path
        self._target_threshold = target_threshold
        self._rrt_radius_factor = float(rrt_radius_factor)
        self._target_node_id = None
        self._path_valid = False
        self.min_dist_to_target = 99999

    def _sample_state(self):
        raise NotImplementedError
    
    
    def _states_dist(self, states1, states2):
        '''
        Must support vectorized calculations
        '''
        raise NotImplementedError
    

    # def _lookahead_edge_is_collision(self, state1, state2):
    #     raise NotImplementedError


    def _edge_is_collision(self, state1, state2):
        raise NotImplementedError


    # def _lookahead_state_is_collision(self, state):
    #     raise NotImplementedError


    def _state_is_collision(self, state):
        raise NotImplementedError
    

    def plot(self):
        raise NotImplementedError


    def start_node(self):
        return self._nodes[0]


    def start_node_id(self):
        return 0

    
    def is_valid(self):
        return self._path_valid


    def target_state(self):
        return self._target_state


    def target_node_id(self):
        return self._target_node_id
    

    def target_node(self):
        if self._target_node_id is not None:
            return self._nodes[self._target_node_id]
        return None


    def _add_node(self, state, g_value=np.inf, successors_ids=[], predecessor_id=None):
        self.min_dist_to_target = np.minimum(self.min_dist_to_target, np.linalg.norm(np.array(state)-self._target_state))
        # print(self.min_dist_to_target)
        new_id = self._max_node_id + 1
        self._max_node_id = new_id
        self._nodes[new_id] = Node(new_id, state, g_value=g_value, successors_ids=successors_ids, predecessor_id=predecessor_id)
        self._states = np.concatenate([self._states, [state]], axis=0)
        if predecessor_id is not None:
            self._nodes[predecessor_id].add_successor(new_id)
        return new_id


    def _step_state(self, nearest_state, new_state):
        norm = np.linalg.norm(new_state - nearest_state)
        if norm > self._step:
            vec = (new_state - nearest_state)
            return nearest_state + vec / norm * self._step
        else:
            return new_state


    def _nearest_node_id(self, state, return_dist=False):
        dists = self._states_dist(state, self._states)
        if return_dist:
            min_index = np.argmin(dists)
            return min_index, dists[min_index]
        return np.argmin(dists)


    def _nearest_nodes_ids(self, node_id, radius, return_dists=False):
        dists = self._states_dist(self._states[node_id], self._states)
        nodes_ids = np.where(dists < radius)[0]
        if node_id in nodes_ids:
            nodes_ids = np.array([k for k in nodes_ids if k != node_id])
        if return_dists:
            if len(nodes_ids) == 0:
                return nodes_ids, np.array([])
            else:
                return nodes_ids, dists[nodes_ids]
        return nodes_ids
        

    def run_RRT(self):
        '''
        TODO: Add lookahead
              Add RRT*
              use _sample_close_to_path in RRT*
        '''
        self._path_valid = False
        for i in tqdm.tqdm(range(self._max_num_iteration)):
            # Sample point
            is_target = False
            if self._target_node_id is None and np.random.rand(1)[0] <= self._target_sampling_probability:
                new_state = np.copy(self._target_state)
                is_target = True
            else:
                new_state = self._sample_state()
                if self._state_is_collision(new_state):
                    continue
            
            # NN
            nearest_node_id, dist = self._nearest_node_id(new_state, return_dist=True)
            
            # Creating new state considering the step
            new_state = self._step_state(self._states[nearest_node_id], new_state)

            # Check for edge collision
            if self._edge_is_collision(self._states[nearest_node_id], new_state):
                continue
            
            # don't add new node if it's target node
            if (is_target or self._states_dist(new_state, self._target_state) == 0) and self._target_node_id is not None:
                self._nodes[self._target_node_id].set_predecessor(nearest_node_id)
                self._nodes[self._target_node_id].set_g_value(self._nodes[nearest_node_id].g_value() + dist)
                self._nodes[nearest_node_id].add_successor(self._target_node_id)
                new_node_id = self._target_node_id
            else:
                new_node_id = self._add_node(new_state, 
                                   predecessor_id=nearest_node_id, 
                                   g_value=self._nodes[nearest_node_id].g_value() + dist)
                if self._states_dist(new_state, self._target_state) == 0:
                    self._target_node_id = new_node_id
                    self._path_valid = True
                
            # stop if close enough to target and not in RRT*
            if not self._isRRT_star and self._states_dist(new_state, self._target_state) <= self._target_threshold:
                break
            elif self._isRRT_star:
                radius = self._rrt_radius_factor * np.power(np.log2(self._max_node_id + 1) / (self._max_node_id + 1), 1/len(self._target_state))
                nearest_nodes_ids, nearest_nodes_dists = self._nearest_nodes_ids(new_node_id, radius=radius, return_dists=True)
                for nn_id, nn_dist in zip(nearest_nodes_ids, nearest_nodes_dists):
                    self._rewire(nn_id, new_node_id, nn_dist)
                for nn_id, nn_dist in zip(nearest_nodes_ids, nearest_nodes_dists):
                    self._rewire(new_node_id, nn_id, nn_dist)
        return self._path_valid


    def _rewire(self, child_id, potential_parent_id, dist):
        if not self._edge_is_collision(child_id, potential_parent_id):
            if dist + self._nodes[potential_parent_id].g_value() < self._nodes[child_id].g_value():
                self._nodes[self._nodes[child_id].predecessor()].remove_successor(child_id)
                self._nodes[child_id].set_predecessor(potential_parent_id)
                self._nodes[potential_parent_id].add_successor(child_id)
                self._nodes[child_id].set_g_value(dist + self._nodes[potential_parent_id].g_value())

    
    def get_path(self, final_node_id):
        if final_node_id not in self._nodes.keys():
            return []
        states = [self._nodes[final_node_id].state()]
        predecessor_id = self._nodes[final_node_id].predecessor()
        if predecessor_id is None:
            return
        curr_id = final_node_id
        while curr_id != self.start_node_id():
            predecessor_id = self._nodes[curr_id].predecessor()
            curr_id = predecessor_id
            states.append(self._nodes[curr_id].state())
        return states[::-1]