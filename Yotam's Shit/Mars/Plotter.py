import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from Geometry import *

class Plotter:
    def __init__(self, map, obstacles, goals, start, samples=None, roadmap=None, solution_nodes=None, solution_edges=None):
        self.map = map
        self.obstacles = obstacles
        self.goals = goals
        self.start = start
        self.samples = samples
        self.roadmap = roadmap
        self.solution_nodes = solution_nodes
        self.solution_edges = solution_edges

    def plot(self, fig=None, ax=None):
        if fig == None or ax == None:
            fig, ax = plt.subplots()
        self.fig = fig; self.ax = ax
        #self.fig, self.ax = plt.subplots()
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
            if len(self.samples) < 100:
                plt.pause(0.001)
            elif len(self.samples) >= 100:
                if idx_s % 10 == 0 or idx_s == len(self.samples) - 1:
                    plt.pause(0.001)

        # plotting all the edges
        for idx_e, e in enumerate(self.roadmap):
            e.plot_edge(self.ax)
            e.Highlight_Nodes(self.ax)
            xlabel_string_new = xlabel_string + ', Edges: ' + str(idx_e+1) + ' / ' + str(len(self.roadmap))
            self.ax.set_xlabel(xlabel_string_new)
            if len(self.roadmap) < 200:
                plt.pause(0.001)
            elif len(self.roadmap) >= 200:
                if idx_e == len(self.roadmap) - 1:
                    self.ax.set_xlabel(xlabel_string_new + ', Plotting Complete!')   
                if idx_e % 20 == 0 or idx_e == len(self.roadmap) - 1:
                    plt.pause(0.001)

        plt.show()
        return

    def replot(self, fig=None, ax=None):
        if fig == None or ax == None:
            fig, ax = plt.subplots()
        self.fig = fig; self.ax = ax
        self.ax.set_ylim(0,self.map.height)
        self.ax.set_xlim(0,self.map.width)
        self.ax.set_title('Solved Mars C-Space')

        # plotting the map, obstacles, and goal zones
        self.map.plot_map(self.ax)
        for o in self.obstacles:
            o.plot_circle(self.ax)
        for g in self.goals:
            g.plot_circle(self.ax)

        # plotting the solution nodes (including the start and goal nodes)
        for idx_n, n in enumerate(self.solution_nodes):
            n.plot_node(self.ax)
            xlabel_string = 'Nodes: ' + str(idx_n+1) + ' / ' + str(len(self.solution_nodes))
            self.ax.set_xlabel(xlabel_string)
            plt.pause(0.001)

        # plotting the solution edges
        for idx_e, e in enumerate(self.solution_edges):
            e.plot_edge(self.ax)
            xlabel_string_new = xlabel_string + ', Edges: ' + str(idx_e+1) + ' / ' + str(len(self.solution_edges))
            if idx_e == len(self.roadmap) - 1:
                self.ax.set_xlabel(xlabel_string_new + ', Plotting Complete!') 
            else:
                self.ax.set_xlabel(xlabel_string_new)
            plt.pause(0.001)

        plt.show()
        return

    def Create_GIF(self, framerate = 15, n_frames = 75):
        #TODO: Make it work
        fig, _ = plt.subplots()
        ani = FuncAnimation(fig, self.plot(), frames=n_frames, interval=1000/framerate, repeat=False)
        i = 0
        while os.path.exists("GIFs/PRM_Animation%i.gif" % i):
            i += 1
        ani.save("GIFs/PRM_Animation%i.gif" % i, dpi=300, writer=PillowWriter(fps=framerate))
        return