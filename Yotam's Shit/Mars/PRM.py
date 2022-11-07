import random
from scipy.spatial import KDTree
from Geometry import *

def Create_Goal_Nodes(goals):
    return [g.Create_Goal_Node() for g in goals]

def Create_Samples(map, start_node, obstacles, goals, N_samples=100, N_knn=3):
    samples = []
    while len(samples) <= N_samples:
        tx = random.random() * map.width
        ty = random.random() * map.height

        obs_and_goals = obstacles + goals
        obs_and_goals_kd_tree = KDTree([o.center for o in obs_and_goals])

        dist, idx = obs_and_goals_kd_tree.query([tx, ty], k=N_knn)        
        no_col = True
        for i in range(N_knn):
            if dist[i] <= obs_and_goals[idx[i]].radius:
                no_col = False
                break
        if no_col:
            samples.append(Node([tx,ty]))

    samples.append(start_node)
    goal_nodes = Create_Goal_Nodes(goals)
    return samples + goal_nodes

def Create_Roadmap(samples, obstacles, N_knn=3):
    """
    Creates the roadmap
    :param samples: list of XY positions of sampled points [m]
    :param obstacles: list of obstacles in the C-Space
    :param obstacle_kd_tree: KDTree object of obstacles
    :returns: the roadmap
    """
    roadmap = []
    samples_kd_tree = KDTree([s.center for s in samples])
    #obs_kd_tree = KDTree([o.center for o in obstacles])
    N_sample = len(samples)
    for s in samples:
        edges = []
        dists, idx = samples_kd_tree.query(s.center, k=N_sample)
        for i in range(1, N_sample):
            neighbor = samples[idx[i]]
            #if dists[i] > max_edge_length:
            #    continue

            #_, obs_idx1 = obs_kd_tree.query(s.center, k=5)
            #_, obs_idx2 = obs_kd_tree.query(neighbor.center, k=5)
            #obs_idx = list(set([*obs_idx1, *obs_idx2])) # concatenates the two index lists and removes duplicate elements
                
            no_col = True
            edge = Edge(s, neighbor, cost=dists[i])
            for e in roadmap:
                if edge == e:
                    continue
            for o in obstacles:
                if edge.Check_Edge_Obstacle_Collision(o):
                    no_col = False
            if no_col:
                roadmap.append(edge)
                edges.append(edge)
            if len(edges) >= N_knn:
                break
        #roadmap.append(edges)
    # plot_road_map(road_map, sample_x, sample_y)
    return roadmap

def Dijkstra_Planner(start_node, samples, roadmap):
    """
    Runs Dijkstra's algorithm on the roadmap
    """
    #TODO: should return a list of edges that connect every goal node to the other goal nodes and to the start node, and a list of the relevant nodes 
    solution_nodes, solution_edges = None, None
    return solution_nodes, solution_edges

def PRM_Solve(map, start_node, goals, obstacles):
    """
    Runs PRM planning algorithm
    """
    samples = Create_Samples(map, start_node, obstacles, goals)
    roadmap = Create_Roadmap(samples, obstacles)
    solution_nodes, solution_edges = Dijkstra_Planner(start_node, samples, roadmap) 
    return solution_nodes, solution_edges