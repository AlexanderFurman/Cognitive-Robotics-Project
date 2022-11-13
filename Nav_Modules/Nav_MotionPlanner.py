import random, sys
from scipy.spatial import KDTree
from Nav_Modules.Nav_Geometry import *

def Create_Goal_Nodes(goals):
    return [g.Create_Goal_Node(name='G'+str(idx+1)) for idx, g in enumerate(goals)]

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
    return samples + goal_nodes, goal_nodes

def Create_Roadmap(samples, obstacles, N_knn=3):
    """
    Creates the roadmap
    :param samples: list of XY positions of sampled points [m]
    :param obstacles: list of obstacles in the C-Space
    :param obstacle_kd_tree: KDTree object of obstacles
    :returns: the roadmap
    """
    samples_kd_tree = KDTree([s.center for s in samples])
    N_sample = len(samples)
    roadmap = []
    #graph = {}
    PRM_graph = Graph(samples)
    for s in samples:
        edges = []; PRM_graph.graph[s] = {}
        dists, idx = samples_kd_tree.query(s.center, k=N_sample)       
        for i in range(1,N_sample):
            neighbor = samples[idx[i]]
            #if dists[i] > max_edge_length:
            #    continue
            no_col = True
            edge = Edge(s, neighbor, cost=dists[i])
            for e in roadmap:
                if edge == e:
                    continue
            for o in obstacles:
                if edge.Check_Edge_Obstacle_Collision(o):
                    no_col = False
            if no_col:
                PRM_graph.graph[s][neighbor] = edge.cost
                roadmap.append(edge)
                edges.append(edge)
            if len(edges) >= N_knn:
                break
        if len(PRM_graph.graph[s]) < 1:
            print('Oopsy! Not all goal/start nodes could be connected to the roadmap.')
    return roadmap, PRM_graph

def Dijkstra_Algorithm(PRM_graph, start_node, goal_node, edges):
    """
    Runs Dijkstra's algorithm on the roadmap
    """
    unvisited_nodes = list(PRM_graph.nodes)
    shortest_path = {}
    previous_nodes = {}
    # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
    # However, we initialize the starting node's value with 0   
    shortest_path[start_node] = 0
    while unvisited_nodes:
        current_min_node = None
        for node in unvisited_nodes: # Iterate over the nodes
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node

        # The code block below retrieves the current node's neighbors and updates their distances
            neighbors = PRM_graph.Get_Outgoing_Edges(current_min_node)
            for neighbor in neighbors:
                tentative_value = shortest_path[current_min_node] + PRM_graph.Get_Edge_Value(current_min_node, neighbor)
                if tentative_value < shortest_path[neighbor]:
                    shortest_path[neighbor] = tentative_value
                    # We also update the best path to the current node
                    previous_nodes[neighbor] = current_min_node
        unvisited_nodes.remove(current_min_node)

    path = []
    node = goal_node

    if node not in previous_nodes:
        print("\tPath could not be obtained...")
        return None, None, None

    while node != start_node:
        path.append(node)
        node = previous_nodes[node]
 
    # Add the start node manually
    path.append(start_node)
    
    path_value = shortest_path[goal_node]
    best_path = list(reversed(path))

    best_edges = []
    for i in range(len(path)-1):
        current_edge = Get_Edge(best_path[i], best_path[i+1], edges)
        best_edges.append(current_edge)

    print(f"\tDijkstra search completed, shortest path has value {path_value:.3f}")
    return path_value, best_path, best_edges

def PRM_Solve(start_node, goals, samples, goal_nodes, roadmap, PRM_graph):
    """
    Runs PRM planning algorithm
    """
    N_goals = len(goals)
    N_traj_expected = int(N_goals*(N_goals+1)/2)
    print("Running Dijkstra's Algorithm on the Probabilistic Roadmap, with " + str(len(samples)) + " nodes left to explore and " + str(N_traj_expected) + " trajectories expected...")

    new_graph = {}
    new_graph[start_node] = {}
    for g in goal_nodes:
        new_graph[g] = {}
    trajectories = []
    for g in goal_nodes:
        path_value, best_path, best_edges = Dijkstra_Algorithm(PRM_graph, start_node, g, roadmap)
        if path_value==None or best_path==None or best_edges==None:
            pass
        else:
            new_graph[start_node][g] = {'Value' : path_value, 'Trajectory' : []}
            for i in range(len(best_edges)):
                new_graph[start_node][g]['Trajectory'].append(best_path[i])
                new_graph[start_node][g]['Trajectory'].append(best_edges[i])
            new_graph[start_node][g]['Trajectory'].append(best_path[-1])
            trajectories.append(new_graph[start_node][g]['Trajectory'])
        for g2 in goal_nodes:
            if not g == g2 and not g in new_graph[g2]:
                path_value, best_path, best_edges = Dijkstra_Algorithm(PRM_graph, g, g2, roadmap)
                if path_value==None or best_path==None or best_edges==None:
                    pass
                else:
                    new_graph[g][g2] = {'Value' : path_value, 'Trajectory' : []}
                    #new_graph[g2][g] = None
                    for j in range(len(best_edges)):
                        new_graph[g][g2]['Trajectory'].append(best_path[j])
                        new_graph[g][g2]['Trajectory'].append(best_edges[j])
                    new_graph[g][g2]['Trajectory'].append(best_path[-1])
                    trajectories.append(new_graph[g][g2]['Trajectory'])
    print("Finished running Dijkstra's Algorithm on the PRM, with " + str(len(trajectories)) + " trajectories found.\n")
    return new_graph, trajectories