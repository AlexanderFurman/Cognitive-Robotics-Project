import sys
path = '/'.join(__file__.split('/')[:-2])
print("current paths = ", sys.path)
if path not in sys.path:
    sys.path.append('/'.join(__file__.split('/')[:-2]))

from abstract_rrt import AbstractRRT
import numpy as np
import matplotlib.pyplot as plt
import json

from ctr_2tubes.CTR_2tube import CTR_2tube
from ctr_2tubes.Tube import Tube
from ctr_2tubes.TubeState import TubeState
from ctr_2tubes.CTRState import CTRState
from ctr_2tubes.PlotterNew import PlotterNew

class CTR_RRT(AbstractRRT):
    def __init__(self, step, params_json_path, target_sampling_probability=0.05, isRRT_star=False, max_num_iteration=10000, sample_close_to_path=False, rrt_radius_factor=1., target_threshold=0):
        #loading tube information from json file
        with open(params_json_path, "r") as f:
            params = json.loads(f.read())

        #loading tubes and tube states:
        self.params_tubes = params.get('tube')
        self.params_initial_states = params.get('tube state initial')
        self.params_final_states = params.get('tube state final')
        tubes = []
        start_state = []
        target_state = []
        for a in range (0,2):
            curr_params = []
            for key in self.params_tubes[a]:
                curr_params.append(self.params_tubes[a].get(key))
            curr_params.pop(0) #gets rid of ID
            tubes.append(Tube(*curr_params))

            curr_params = []
            for key in self.params_initial_states[a]:
                curr_params.append(self.params_initial_states[a].get(key))
            curr_params.pop(0) #gets rid of ID
            curr_params[0]=np.radians(curr_params[0])
            start_state.append(TubeState(*curr_params))

            curr_params = []
            for key in self.params_final_states[a]:
                curr_params.append(self.params_final_states[a].get(key))
            curr_params.pop(0) #gets rid of ID
            curr_params[0]=np.radians(curr_params[0])
            target_state.append(TubeState(*curr_params))

        start_state = CTRState(start_state)
        self.tolerance = params.get('tolerance') #tolerance allowed when solving ode
        self.ctr = CTR_2tube(tubes[0:2], start_state)
        
        super().__init__(self._flatten_ctr_state(start_state), self._flatten_ctr_state(target_state), step, target_sampling_probability, isRRT_star, max_num_iteration, sample_close_to_path, rrt_radius_factor, target_threshold)

        # sampling params
        self.max_translation = 0.15
        self.min_translation = -0.1
        self.translation_span = self.max_translation - self.min_translation

        self.max_angle = np.pi
        self.min_angle = -np.pi
        self.angle_span = self.max_angle - self.min_angle


    def vec_to_state(self, vec):
        tubes = []
        length_start_index = int(len(vec) / 2)
        for i in range(int(len(vec) / 2)):
            tubes.append(TubeState(vec[length_start_index + i], vec[i]))
        return CTRState(tubes)


    def _flatten_ctr_state(self, ctr_state):
        states = ctr_state
        if type(ctr_state) == CTRState:
            states = [state for state in ctr_state.state_arr]
        output = []
        for state in states:
            output.append(state.length)
        for state in states:
            output.append(state.angle)
        return output

    
    def _sample_state(self):
        sample_point = np.random.rand(4)
        scaling_vec = np.array([self.translation_span, self.translation_span, self.angle_span, self.angle_span])
        base_vec = np.array([self.min_translation, self.min_translation, self.min_angle, self.min_angle])
        return base_vec + sample_point * scaling_vec


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

    
    def plot_planner(self, ax=None):
        if ax is None:
            plt.figure()
            ax = plt.axes(projection='3d')
        plt.grid()
        states = self.get_path(self.target_node_id())
        points = np.array([self.state_to_vec(point) for point in states])
        ax.scatter(points[:,0], points[:,1], points[:,2], c=(points[:,3] - np.min(points[:,3]))/(np.max(points[:,3]) - np.min(points[:,3])))
        ax.plot(points[:,0], points[:,1], points[:,2], 'k')
        ax.set_xlabel('length_0')
        ax.set_ylabel('length_1')
        ax.set_zlabel('angle_0')
        

    def state_to_vec(self, state):
        vec = []
        for tube_State in state.state_arr:
            vec.append(tube_State.length)
        for tube_State in state.state_arr:
            vec.append(tube_State.angle)
        return np.array(vec)


    def plot(self):
        plotter = PlotterNew(self.ctr, 5, shadow=True)
        path = self.get_path(rrt.target_node_id())
        # plotter.trajectory_plot(self.ctr, path)
        plotter.interactive_plot(self.ctr, path, rrt)
        # plotter = PlotterNew(self.ctr, 5, shadow=True)
        # plotter.trajectory_plot(ctr, CTRState(tube_state_i[0:2]),CTRState(tube_state_f[0:2]))


    def get_path(self, final_node_id):
        path = super().get_path(final_node_id)
        ctr_states = []
        for state in path:
            ctr_states.append(self.vec_to_state(state))
        return ctr_states

np.random.seed(12344)
rrt = CTR_RRT(.01, 'motion_planner/tube_params.json', target_sampling_probability=0.01, isRRT_star=False, max_num_iteration=100000, sample_close_to_path=False, rrt_radius_factor=0.1, target_threshold=0)
if rrt.run_RRT():
    # rrt.plot_planner()
    # plt.pause(1)
    rrt.plot()
    # rrt.plot()
    # plt.show()
