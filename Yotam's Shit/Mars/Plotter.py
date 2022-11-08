import os, imageio
import matplotlib.pyplot as plt
import networkx as nx
from Geometry import *

class Plotter:
    def __init__(self, map, obstacles, goals, start, samples=None, roadmap=None, PRM_graph=None, trajectories=None, save_gif=False):
        self.map = map
        self.obstacles = obstacles
        self.goals = goals
        self.start = start
        self.samples = samples
        self.roadmap = roadmap
        self.PRM_graph = PRM_graph
        self.trajectories = trajectories
        self.save_gif = save_gif
        
    def plot(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_ylim(0,self.map.height)
        self.ax.set_xlim(0,self.map.width)
        self.ax.set_title('Mars C-Space')

        # plotting the map, obstacles, and goal zones
        self.map.plot_map(self.ax)
        for o in self.obstacles:
            o.plot_circle(self.ax)
        for g in self.goals:
            g.plot_circle(self.ax)

        if self.save_gif:
            img_list = ['GIFs/Temp_Images/1.png']
            plt.savefig('GIFs/Temp_Images/1.png')

        # plotting all the nodes (including the start and goal nodes)
        for idx_s, s in enumerate(self.samples):
            s.plot_node(self.ax)
            xlabel_string = 'Nodes: ' + str(idx_s+1) + ' / ' + str(len(self.samples))
            self.ax.set_xlabel(xlabel_string)
            if len(self.samples) < 100:
                if self.save_gif:
                    img_list.append('GIFs/Temp_Images/' + str(1+idx_s) + '.png')
                    plt.savefig('GIFs/Temp_Images/' + str(1+idx_s) + '.png')
                plt.pause(0.001)
            elif len(self.samples) >= 100:
                if idx_s % 10 == 0 or idx_s == len(self.samples) - 1:
                    if self.save_gif:
                        img_list.append('GIFs/Temp_Images/' + str(1+idx_s) + '.png')
                        plt.savefig('GIFs/Temp_Images/' + str(1+idx_s) + '.png')
                    plt.pause(0.001)

        if self.save_gif:
            for i in range(1,10):
                img_list.append('GIFs/Temp_Images/' + str(1+idx_s+i) + '.png')
                plt.savefig('GIFs/Temp_Images/' + str(1+idx_s+i) + '.png')

        # plotting all the edges
        for idx_e, e in enumerate(self.roadmap):
            e.plot_edge(self.ax)
            e.Highlight_Nodes(self.ax)
            xlabel_string_new = xlabel_string + ', Edges: ' + str(idx_e+1) + ' / ' + str(len(self.roadmap))
            self.ax.set_xlabel(xlabel_string_new)
            if len(self.roadmap) < 200:
                if self.save_gif:
                    img_list.append('GIFs/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                    plt.savefig('GIFs/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                plt.pause(0.001)
            elif len(self.roadmap) >= 200:
                if idx_e == len(self.roadmap) - 1:
                    self.ax.set_xlabel(xlabel_string_new + ', Plotting Complete!')   
                if idx_e % 20 == 0 or idx_e == len(self.roadmap) - 1:
                    if self.save_gif:
                        img_list.append('GIFs/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                        plt.savefig('GIFs/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                    plt.pause(0.001)
        
        if self.save_gif:
            for j in range(1,15):
                img_list.append('GIFs/Temp_Images/' + str(10+idx_s+idx_e+j) + '.png')
                plt.savefig('GIFs/Temp_Images/' + str(10+idx_s+idx_e+j) + '.png')

        plt.show()
        if self.save_gif:
            return img_list
        return

    def replot(self, clean=False):
        self.fig, self.ax = plt.subplots()
        self.ax.set_ylim(0,self.map.height)
        self.ax.set_xlim(0,self.map.width)
        if clean:
            self.ax.set_title('Solved Mars C-Space')
        else:
            self.ax.set_title('Mars PRM with Dijkstra Paths')

        # plotting the map, obstacles, and goal zones
        self.map.plot_map(self.ax)
        for o in self.obstacles:
            o.plot_circle(self.ax)
        for g in self.goals:
            g.plot_circle(self.ax)
        if not clean: # show the original PRM, else show only the shortest paths
            for s in self.samples:
                s.plot_node(self.ax)
            for e in self.roadmap:
                e.plot_edge(self.ax)

        if self.save_gif:
            img_list = ['GIFs/Temp_Images/1.png']
            plt.savefig('GIFs/Temp_Images/1.png')

        count = 0
        # plotting the solution trajectories (including the start and goal nodes)
        for idx_t, traj in enumerate(self.trajectories):
            xlabel_string = 'Trajectories: ' + str(idx_t+1) + ' / ' + str(len(self.trajectories))
            if idx_t == len(self.trajectories) - 1:
                self.ax.set_xlabel(xlabel_string + ', Plotting Complete!') 
            else:
                self.ax.set_xlabel(xlabel_string)
            for o in traj:
                if isinstance(o,Node):
                    if not isinstance(o,Start_Node) and not isinstance(o,Goal_Node):
                        o.plot_node(self.ax, color='yellow')
                    else:
                        o.plot_node(self.ax)
                elif isinstance(o,Edge):
                    o.plot_edge(self.ax, color='yellow')
                if self.save_gif:
                    count += 1
                    img_list.append('GIFs/Temp_Images/' + str(1+count) + '.png')
                    plt.savefig('GIFs/Temp_Images/' + str(1+count) + '.png')
                if not clean:
                    plt.pause(0.001)
        
        if self.save_gif:
            for j in range(1,15):
                img_list.append('GIFs/Temp_Images/' + str(1+count+j) + '.png')
                plt.savefig('GIFs/Temp_Images/' + str(1+count+j) + '.png')

        plt.show()

        if self.save_gif:
            return img_list
        return

    def plot_knn(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_ylim(0,self.map.height)
        self.ax.set_xlim(0,self.map.width)
        self.ax.set_title('Mars C-Space')
        
        # plotting the map, obstacles, and goal zones
        self.map.plot_map(self.ax)
        for o in self.obstacles:
            o.plot_circle(self.ax)
        for g in self.goals:
            g.plot_circle(self.ax)

        # plotting all the nodes (including the start and goal nodes)
        for idx_s, s in enumerate(self.samples):
            s.plot_node(self.ax)
            xlabel_string = 'Nodes: ' + str(idx_s+1) + ' / ' + str(len(self.samples))
            self.ax.set_xlabel(xlabel_string)

        # plotting neighbors
        i = 0
        for key in self.PRM_graph:
            key.plot_node(self.ax, color='yellow')
            xlabel_string_new = xlabel_string + ', kNN Check: ' + str(i+1) + ' / ' + str(len(self.PRM_graph))
            self.ax.set_xlabel(xlabel_string_new)
            i += 1
            plt.pause(0.001)
            for n in self.PRM_graph[key]:
                n.plot_node(self.ax, color='orange')
            plt.pause(0.1)

        plt.show()
        return

    def Create_GIF(self, replot=False, clean=False):
        if not replot:
            img_list = self.plot()
            name = "GIFs/PRM_Animation"
        else:
            img_list = self.replot(clean)
            name = "GIFs/Dijkstra_Animation"

        i = 0
        while os.path.exists(name + "%i.gif" % i):
            i += 1
        with imageio.get_writer(name + "%i.gif" % i, mode='I') as writer:
            for filename in img_list:
                image = imageio.imread(filename)
                writer.append_data(image)

        # Remove the image files
        for filename in set(img_list):
            os.remove(filename)
        return

def Visualize_Final_Graph(new_graph):
    G = nx.Graph()
    for n in new_graph:
        G.add_node(n.name, pos=(n.center[0],n.center[1]), color=n.color)
        for n2 in new_graph[n]:
            cost = new_graph[n][n2]['Value']
            cost = f"{cost:.3f}"
            G.add_edge(n.name, n2.name, weight=cost, color='yellow')

    pos = nx.get_node_attributes(G,'pos')
    nx.draw(G,pos,with_labels=True)
    labels = nx.get_edge_attributes(G,'weight')
    nx.draw_networkx_edge_labels(G,pos,edge_labels=labels)

    plt.title('Simplified Graph Representation')
    plt.show()
    return