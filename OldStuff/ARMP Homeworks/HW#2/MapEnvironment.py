import numpy
from IPython import embed
from matplotlib import pyplot as plt

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        self.map = numpy.loadtxt(mapfile)
        self.xlimit = [0, self.map.shape[1]]
        self.ylimit = [0, self.map.shape[0]]

        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

        # Display the map
        plt.imshow(self.map, interpolation='nearest')

    def compute_distance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations.
        #
        pass


    def state_validity_checker(self, config):

        #
        # TODO: Implement a state validity checker
        # Return true if valid.
        #
        return True

    def edge_validity_checker(self, config1, config2):

        #
        # TODO: Implement an edge validity checker
        #
        #
        pass

    def compute_heuristic(self, config):
        
        #
        # TODO: Implement a function to compute heuristic.
        #
        pass

    def visualize_plan(self, plan):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        input plan should be in [x, y] convention.
        '''
        plt.imshow(self.map, interpolation='nearest')
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i,0], plan[i+1, 0]]
            y = [plan[i,1], plan[i+1, 1]]
            plt.plot(x, y, 'k')
        plt.show()