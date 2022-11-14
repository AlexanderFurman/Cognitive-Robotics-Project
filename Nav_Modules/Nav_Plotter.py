import os, imageio
import matplotlib.pyplot as plt
import networkx as nx
from Nav_Modules.Nav_Geometry import *
from matplotlib.axes._axes import _log as matplotlib_axes_logger
matplotlib_axes_logger.setLevel('ERROR')

class Plotter:
    def __init__(self, map, obstacles, goals, start, samples, roadmap, PRM_graph, new_graph, trajectories, final_traj, output_path=None, axis=None, save_img=False, save_gif=False, show_anim=True):
        self.map = map
        self.obstacles = obstacles
        self.goals = goals
        self.start = start
        self.samples = samples
        self.roadmap = roadmap
        self.PRM_graph = PRM_graph
        self.new_graph =  new_graph
        self.trajectories = trajectories
        self.final_traj = final_traj
        self.ax = axis
        self.save_img = save_img
        self.save_gif = save_gif
        self.show_anim = show_anim
        self.output_path = output_path
    
    def plot_init(self, axis=None):
        """
        Plots the initial image (.png) of the map, including obstacles, goal zones, and the starting point
        :param axis: axis of the subplot to add the plot to
        """
        if axis is None:
            _, axis = plt.subplots()
        axis.set_ylim(0,self.map.height)
        axis.set_xlim(0,self.map.width)
        axis.set_title('Initial Map (C-Space)')

        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)
        self.start.plot_node(axis)

        if self.show_anim:
            plt.show()
        if self.save_img:
            plt.savefig(self.output_path+'Initial_Map.png')
        return

    def plot_knn(self):
        """
        Plots the map (.png or .gif) and shows the k-Nearest Neighbors of each node after the initial roadmap sampling (by default, this function is not used)
        """
        self.fig, self.ax = plt.subplots()
        self.ax.set_ylim(0,self.map.height)
        self.ax.set_xlim(0,self.map.width)
        self.ax.set_title('k-Nearest Neighbors Display')
        
        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(self.ax)
        for o in self.obstacles:
            o.plot_circle(self.ax)
        for g in self.goals:
            g.plot_circle(self.ax)

        # plotting all the nodes (including the start and goal nodes)
        for s in self.samples:
            s.plot_node(self.ax)

        # plotting neighbors
        i = 0
        for key in self.PRM_graph.graph:
            key.plot_node(self.ax, color='yellow')
            xlabel_string = 'kNN Check: ' + str(i+1) + ' / ' + str(len(self.PRM_graph.graph))
            self.ax.set_xlabel(xlabel_string)
            i += 1
            plt.pause(0.001)
            for n in self.PRM_graph.graph[key]:
                n.plot_node(self.ax, color='orange')
            plt.pause(0.001)

        plt.show()
        if self.save_img:
            plt.savefig(self.output_path+'kNN.png')
        return

    def plot_PRM(self, axis=None):
        """
        Plots the map (.png or .gif) after the PRM is generated, including the initial map contents as well as the sampled points and the edges of the roadmap
        :param axis: axis of the subplot to add the plot to
        """
        if axis is None:
            _, axis = plt.subplots()
        axis.set_ylim(0,self.map.height)
        axis.set_xlim(0,self.map.width)
        axis.set_title('Probabilistic Roadmap')

        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)

        if self.save_gif:
            img_list = ['Output/Temp_Images/1.png']
            plt.savefig('Output/Temp_Images/1.png')

        # plotting all the nodes (including the start and goal nodes)
        for idx_s, s in enumerate(self.samples):
            s.plot_node(axis)
            xlabel_string = 'Nodes: ' + str(idx_s+1) + ' / ' + str(len(self.samples))
            axis.set_xlabel(xlabel_string)
            if len(self.samples) < 100:
                if self.save_gif:
                    img_list.append('Output/Temp_Images/' + str(1+idx_s) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(1+idx_s) + '.png')
                if self.show_anim:
                    plt.pause(0.001)
            elif len(self.samples) >= 100:
                if idx_s % 10 == 0 or idx_s == len(self.samples) - 1:
                    if self.save_gif:
                        img_list.append('Output/Temp_Images/' + str(1+idx_s) + '.png')
                        plt.savefig('Output/Temp_Images/' + str(1+idx_s) + '.png')
                    if self.show_anim:
                        plt.pause(0.001)

        if self.save_gif:
            for i in range(1,10):
                img_list.append('Output/Temp_Images/' + str(1+idx_s+i) + '.png')
                plt.savefig('Output/Temp_Images/' + str(1+idx_s+i) + '.png')

        # plotting all the edges
        for idx_e, e in enumerate(self.roadmap):
            e.plot_edge(axis)
            e.Highlight_Nodes(axis)
            xlabel_string_new = xlabel_string + ', Edges: ' + str(idx_e+1) + ' / ' + str(len(self.roadmap))
            axis.set_xlabel(xlabel_string_new)
            if len(self.roadmap) < 200:
                if self.save_gif:
                    img_list.append('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                if self.show_anim:
                    plt.pause(0.001)
            elif len(self.roadmap) >= 200 and len(self.roadmap) < 1000:
                if idx_e == len(self.roadmap) - 1:
                    axis.set_xlabel(xlabel_string_new + ', Plotting Complete!')   
                if idx_e % 20 == 0 or idx_e == len(self.roadmap) - 1:
                    if self.save_gif:
                        img_list.append('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                        plt.savefig('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                    if self.show_anim:
                        plt.pause(0.001)
            elif len(self.roadmap) >= 1000:
                if idx_e == len(self.roadmap) - 1:
                    axis.set_xlabel(xlabel_string_new + ', Plotting Complete!')   
                if idx_e % 200 == 0 or idx_e == len(self.roadmap) - 1:
                    if self.save_gif:
                        img_list.append('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                        plt.savefig('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                    if self.show_anim:
                        plt.pause(0.001)
        if self.show_anim:
            plt.show()
        if self.save_img:
            plt.savefig(self.output_path+'/PRM.png')
        if self.save_gif:
            for j in range(1,15):
                img_list.append('Output/Temp_Images/' + str(10+idx_s+idx_e+j) + '.png')
                plt.savefig('Output/Temp_Images/' + str(10+idx_s+idx_e+j) + '.png')
            return img_list
        return

    def plot_Dijkstra(self, axis=None, clean=False):
        """
        Plots the map (.png or .gif) showing the paths obtained by the Dijstrka algorithm, including those paths, the PRM (optinal), and the initial map contents
        :param axis: axis of the subplot to add the plot to
        :param clean: if True, then only the Dijkstra paths are shown, otherwise the whole PRM is shown as well
        """
        if axis is None:
            _, axis = plt.subplots()
        axis.set_ylim(0,self.map.height)
        axis.set_xlim(0,self.map.width)
        if clean:
            axis.set_title('Dijkstra Paths between Critical Nodes')
        else:
            axis.set_title('PRM with Dijkstra Paths')

        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)
        if not clean: # show the original PRM, else show only the shortest paths
            for s in self.samples:
                s.plot_node(axis)
            for e in self.roadmap:
                e.plot_edge(axis)

        if self.save_gif:
            img_list = ['Output/Temp_Images/1.png']
            plt.savefig('Output/Temp_Images/1.png')

        count = 0
        # plotting the solution trajectories (including the start and goal nodes)
        for idx_t, traj in enumerate(self.trajectories):
            xlabel_string = 'Trajectories: ' + str(idx_t+1) + ' / ' + str(len(self.trajectories))
            if idx_t == len(self.trajectories) - 1:
                axis.set_xlabel(xlabel_string + ', Plotting Complete!') 
            else:
                axis.set_xlabel(xlabel_string)
            for o in traj:
                if isinstance(o,Node):
                    if not isinstance(o,Start_Node) and not isinstance(o,Goal_Node):
                        o.plot_node(axis, color='yellow')
                    else:
                        o.plot_node(axis)
                elif isinstance(o,Edge):
                    o.plot_edge(axis, color='yellow')
                if self.save_gif:
                    count += 1
                    img_list.append('Output/Temp_Images/' + str(1+count) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(1+count) + '.png')
                if self.show_anim:
                    plt.pause(0.001)
        
        if self.save_gif:
            for j in range(1,15):
                img_list.append('Output/Temp_Images/' + str(1+count+j) + '.png')
                plt.savefig('Output/Temp_Images/' + str(1+count+j) + '.png')

        if self.save_img:
            plt.savefig(self.output_path+'Dijkstra.png')
        if self.show_anim:
            plt.show()
        if self.save_gif:
            return img_list
        return

    def Visualize_Final_Graph(self, axis=None):
        """
        Plots a simplified graph (.png) showing the connections between the critical nodes as well as their weights
        :param new_graph: dictionary which contains the distances between pairs of critical nodes as well as the best path on the PRM that connects them
        """
        new_graph = self.new_graph
        if axis is None:
            _, axis = plt.subplots()
    
        G = nx.Graph()
        for n in new_graph:
            G.add_node(n.name, pos=(n.center[0],n.center[1]), color=n.color)
            for n2 in new_graph[n]:
                cost = new_graph[n][n2]['Value']
                cost = f"{cost:.3f}"
                G.add_edge(n.name, n2.name, weight=cost, color='yellow')
        pos = nx.get_node_attributes(G,'pos')
        nx.draw(G,pos,ax=axis,with_labels=True)
        labels = nx.get_edge_attributes(G,'weight')
        nx.draw_networkx_edge_labels(G,pos,ax=axis,edge_labels=labels)

        axis.set_title('Simplified Graph Representation')
        if self.show_anim:
            plt.show()
        if self.save_img:
            plt.savefig(self.output_path+'Simplified_Graph.png')
        return

    def plot4(self):
        _, axis = plt.subplots(2, 2)
        # Initial Map - [0,0]
        self.plot_init(axis=axis[0,0])
        # PRM - [0,1]
        self.plot_PRM(axis=axis[0,1])
        # Dijkstra Paths - [1,0]
        self.plot_Dijkstra(axis=axis[1,0], clean=True)
        # Final Trajectory - [1,1]
        self.Visualize_Final_Graph(axis=axis[1,1])

        plt.show()
        return

    def plot_final(self):
        """
        Plots the map (.png or .gif) showing the paths obtained by the Dijstrka algorithm, including those paths, the PRM (optinal), and the initial map contents
        :param axis: axis of the subplot to add the plot to
        """
        fig, axis = plt.subplots()
        axis.set_ylim(0,self.map.height)
        axis.set_xlim(0,self.map.width)
        axis.set_title('Final C-Space Trajectory for the Mars Rover')

        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)

        if self.save_gif:
            img_list = ['Output/Temp_Images/1.png']
            plt.savefig('Output/Temp_Images/1.png')

        count = 0
        # plotting the solution trajectories (including the start and goal nodes)
        for idx_t, traj in enumerate(self.final_traj):
            xlabel_string = 'Trajectories: ' + str(idx_t+1) + ' / ' + str(len(self.final_traj))
            if idx_t == len(self.final_traj) - 1:
                axis.set_xlabel(xlabel_string + ', Plotting Complete!') 
            else:
                axis.set_xlabel(xlabel_string)
            count += 1
            #any_plotted = False
            for o in traj:
                if isinstance(o,Node):
                    if not isinstance(o,Start_Node) and not isinstance(o,Goal_Node):
                        o.plot_node(axis, color=((255-(20*count))/255, (255-(20*count))/255, 0))
                        #if not o.plotted and not any_plotted:
                        #    o.plot_node(self.ax, color='yellow')
                        #    any_plotted = True
                        #else:
                        #    o.plot_node(self.ax, color=((170-(20*count))/255, (255-(20*count))/255, 0))
                        #    any_plotted = True
                    else:
                        o.plot_node(axis)
                elif isinstance(o,Edge):
                    o.plot_edge(axis, color=((255-(30*count))/255, (255-(30*count))/255, 0))
                    #if not o.plotted and not any_plotted:
                    #    o.plot_edge(self.ax, color='yellow')
                    #    any_plotted = True
                    #else:
                    #    #o.plot_edge(self.ax, color=(255/255, (234-(10*count))/255, 0))
                    #    o.plot_edge(self.ax, color=((170-(20*count))/255, (255-(20*count))/255, 0))
                    #    any_plotted = True
                #o.plotted = True
                if self.save_gif:
                    #count += 1
                    img_list.append('Output/Temp_Images/' + str(1+count) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(1+count) + '.png')
                if self.show_anim:
                    plt.pause(0.01)

        plt.show()
        if self.save_img:
            plt.savefig(self.output_path+'Final_Map.png')
        if self.save_gif:
            for j in range(1,15):
                img_list.append('Output/Temp_Images/' + str(1+count+j) + '.png')
                plt.savefig('Output/Temp_Images/' + str(1+count+j) + '.png')
        if self.save_gif:
            return img_list
        return

    def Create_GIF(self, replot=False, clean=False):
        if not replot:
            img_list = self.plot()
            name = self.output_path+"PRM_Animation"
        else:
            img_list = self.replot(clean)
            name = self.output_path+"Dijkstra_Animation"

        i = 0
        while os.path.exists(name + "%i.gif" % i):
            i += 1
        with imageio.get_writer(name + "%i.gif" % i, mode='I') as writer:
            for filename in img_list:
                image = imageio.imread(filename)
                writer.append_data(image)

        # Remove the temporary image files
        for filename in set(img_list):
            os.remove(filename)
        return