# Integrated Task & Motion Planning for a Simplified Mars Rover Exploration Problem
## Cognitive Robotics Spring 2022 Final Project - Alex Furman & Yotam Granov

<p align="center">
  <img src=https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Rover.jpg>
</p>

## (0) Introduction
### Abstract
In this work, we implement task and motion planning methods in order to solve a simplified exploration problem for a hypothetical Mars rover called *Ignorance*. The rover is tasked with autonomously navigating across the surface of Mars to multiple goal zones without hitting any obstacles, and we must ensure that it reaches all goal zones assigned to it. At each goal zone, the rover will extract a geological sample from a cave at that location (which it must first map out), and once it has retrieved samples from every goal zone it must return to its base with the samples.

To solve this problem, we sampled a large number of points in the rover's configuration space (C-space) and implemented the Probabilistic Roadmap (PRM) algorithm in order to build a graph connecting the sampled points to their k-nearest neighbors. A single point is sampled from within each goal zone, which we'll call a goal node, and every goal node will be connected to the PRM graph (along with the location of the base, which we treat as the start node). We then apply Dijkstra's shortest path algorithm between the start node and every goal node, as well as between every pair of goal nodes, in order to construct a simple weighted graph that represents the total distance between each crucial node (the start and goal nodes). We treat this graph as a weighted Travelling Salesman Problem (without the limitation that each node can only be visited once), and solve it using the Optimal Fast Downward classical planner in order to obtain the final path for *Ignorance*. In order to effectively integrate the task and motion planning here, we utilize the Unified Planning Framework library for Python, which allows us to easily encode classical planning problems into PDDL with geometric constraints obtained from the motion planner (which is implemented completely in Python).

Finally, at each goal point we show the motion planning procedure for how the rover's arm will retrieve the geological sample without colliding with any of the cave's walls, and we do so using the Rapidly Exploring Random Trees (RRT) algorithm. We also implement the optimized RRT algorithm (known as RRT*) for this same purpose, and compare the efficiency of the two algorithms when applied to our problem.

### Running the Code
TO DO

## (1) Rover Navigation
In the first part of the problem, we want to plan the path of the rover between its starting point and a variety of goal zones (by default there are 3 such zones), all of which are randomly generated at the start of a run. There are a variety of obstacles (represented as circles of varying sizes) which are also randomly generated across the map - the number of obstacles can either be stochastic (between 5 and 20) or determined by the user. The rover, which is treated as a point for the sake of simplicity, must find a path that begins at the start node (represented as a single point), passes through each of the goal zones (represented as circles of random radii) at least once, and terminates to the start node - all without colliding with any obstacles.

### Initial Map
First, the initial map of the environment for the navigation problem is produced. It is shown in the following figure:
<p align="center">
  <img src=https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Initial_Map.png>
</p>

The rover's start point is represented by the blue point, the goal zones are the green circles, and the obstacles are the dark orange circles. The free space of the rover is colored light orange.

### Probabilistic Roadmap (PRM)
Next, we sample a certain number of points (the default is 200) from the rover's workspace, including one point per goal zone (this becomes the goal node that will be added to the roadmap, and they are represented in dark green). We use a k-Nearest Neighbors (kNN) algorithm (the default k is 5) to connect each sample to its nearest neighbors (i.e. create an edge between them), and then we run a collision detection algorithm to check whether any of the produced edges collide with any of the obstacles. In the following animation, we first see the sampling process, where the black points represent the sample nodes that were produced (using uniform random sampling). The second part of the animation shows the production of valid edges using the kNN and collision detection algorithms:
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/PRM_Animation.gif)

The yellow points represent nodes which have been connected to their neighbors, and the black lines between the yellow points represent the appropriate edges. The start and goal nodes are highlighted once they are connected to the roadmpa. Once the edge production process is completed, we are left with a graph that should, in most cases (this is a probabilistic process after all), connect the start node to all of the goal nodes and most of the goal nodes to each other, and thus we obtain our Probabilistic Roadmap (PRM).

### Dijkstra's Algorithm
Once we have our PRM, we would like to find the shortest paths connecting each goal node to the start node, as well as those connecting the goal nodes to each other (when possible). Our PRM is a weighted undirected graph, where the weights are determined by the Euclidean distance between each pair of nodes (i.e. the length of the edge connecting them), and so we can implement Dijkstra's shortest path algorithm to get these shortest paths. The following animation shows the paths obtained by Dijkstra's algorithm, highlighted in yellow:
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Dijkstra_Animation.gif)

### Travelling Salesman Problem
<p align="center">
  <img src=https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Simplified_Graph.png>
</p>

### Final Trajectory
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Final_Trajectory_Animation.gif)

## (2) Arm Motion Planning
Once we reach a location of interest, our mars rover neeeds to retrieve the geological sample, without colliding with the obstacles
around it. Thus it was decided to use RRT to map out a trajectory for the rover's robot arm.

### RRT Algorithm
The RRT algorithm works by interacting with 2 spaces: the Configuration Space (C-Space), and the Workspace (W-Space). Given a robot arm with n joints, the C-Space is a set of dimension $R^n$, which contains all possible configuration of the robot's joints. For example, given a simple R-R robot arm whose motors can each make one full revolution, the dimension of the C-Space is $R^2$, with the bounds $([0, 2\pi),[0, 2\pi))$. 
The W-Space contains the physical representation of the robot, the goal, and all obstacles. The dimension of the W-Space in our case is simplified to $R^2$. (Note that the RRT algorithm still works for higher dimensions of C-Space, and W-Sapce - however runtimes will inevitably increase)
The RRT algorithm works by expanding a tree of configuration nodes randomly over the configuration space (C-Space). Our tree starts with a single root configuration in the C-Space (This represents the initial joint values of the robot). From there, a random configuration is sampled, and the tree locates the node in the tree which is nearest to the random configuration. From there a new node is created in the direction of the random node, a step-size away from the nearest node. The forward kinematics of the robot arm is run with this new configuration, and the robot queries whether it has collided with its environment. If it has, the node is discarded. If it has not colllided, the node is added to the tree, with the nearest node as its parent. We repeat this process until we find a configuration which causes the robot arm to reach the goal. The trajectory to the goal can now be extracted by following the branch containing the goal-node

### Rapidly Exploring Random Trees (RRT)
<p align="center">
  <img src=https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/RRT_C-Space.png>
</p>

### Final Trajectory
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Arm_Animation.gif)

### References

[1] Wenjun Cheng & Yuhui Gao. ["Using PDDL to Solve Vehicle Routing Problems"](https://hal.inria.fr/hal-01383334). *8th International
Conference on Intelligent Information Processing (IIP)*, 2014.

[2] Tim Chinenov. ["Robotic Path Planning: RRT and RRT*"](https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378). 2019. 

[3] Alexey Klochay. ["Implementing Dijkstra’s Algorithm in Python"](https://www.udacity.com/blog/2021/10/implementing-dijkstras-algorithm-in-python.html). *Udacity*, 2021. 

[4] Atsushi Sakai, Daniel Ingram, Joseph Dinius, Karan Chawla, Antonin Raffin & Alexis Paques. ["PythonRobotics: A Python Code Collection of Robotics Algorithms"](https://arxiv.org/abs/1808.10703). 2018. GitHub: https://github.com/AtsushiSakai/PythonRobotics
