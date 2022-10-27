from time import time
import matplotlib.pyplot as plt

class Plotter:
    def __init__(self, map, framerate = 5):
        self.fig = plt.figure()
        self.framerate = framerate
        self.map = map

    def plot(self):
        

