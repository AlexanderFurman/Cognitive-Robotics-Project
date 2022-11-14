# Integrated Task & Motion Planning for a Simplified Mars Rover Exploration Problem
## Cognitive Robotics Spring 2022 Final Project - Alex Furman & Yotam Granov

![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Rover.jpg)

## (0) Introduction
In this work, we implement task and motion planning methods in order to solve a simplified exploration problem for a hypothetical Mars rover called *Ignorance*. The rover is tasked with autonomously navigating across the surface of Mars to multiple goal zones without hitting any obstacles, and we must ensure that it reaches all goal zones assigned to it. At each goal zone, the rover will extract a geological sample from a cave at that location (which it must first map out), and once it has retrieved samples from every goal zone it must return to its base with the samples.

To solve this problem, we sampled a large number of points in the rover's configuration space (C-space) and implemented the Probabilistic Roadmap (PRM) algorithm in order to build a graph connecting the sampled points to their k-nearest neighbors. A single point is sampled from within each goal zone, which we'll call a goal node, and every goal node will be connected to the PRM graph (along with the location of the base, which we treat as the start node). We then apply Dijkstra's shortest path algorithm between the start node and every goal node, as well as between every pair of goal nodes, in order to construct a simple weighted graph that represents the total distance between each crucial node (the start and goal nodes). We treat this graph as a weighted Travelling Salesman Problem (without the limitation that each node can only be visited once), and solve it using the Optimal Fast Downward classical planner in order to obtain the final path for *Ignorance*. In order to effectively integrate the task and motion planning here, we utilize the Unified Planning Framework library for Python, which allows us to easily encode classical planning problems into PDDL with geometric constraints obtained from the motion planner (which is implemented completely in Python).

Finally, at each goal point we show the motion planning procedure for how the rover's arm will retrieve the geological sample without colliding with any of the cave's walls, and we do so using the Rapidly Exploring Random Trees (RRT) algorithm. We also implement the optimized RRT algorithm (known as RRT*) for this same purpose, and compare the efficiency of the two algorithms when applied to our problem.

## (1) Rover Navigation

### Initial Map
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Initial_Map.png)

### Probabilstic Roadmap (PRM)
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/PRM.png)

### Dijkstra's Algorithm
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Dijkstra.png)

### Travelling Salesman Problem
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Simplified_Graph.png)

### Final Trajectory
TO DO

## (2) Arm Motion Planning

### Rapidly Exploring Random Trees (RRT)
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/RRT_C-Space.png)

### Final Trajectory
![](https://github.com/AlexanderFurman/Cognitive-Robotics-Project/blob/main/Graphics/Arm_Animation.gif)

### References

[1] Wenjun Cheng & Yuhui Gao. ["Using PDDL to Solve Vehicle Routing Problems"](https://hal.inria.fr/hal-01383334). *8th International
Conference on Intelligent Information Processing (IIP)*, 2014.

[2] Tim Chinenov. ["Robotic Path Planning: RRT and RRT*"](https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378). 2019. 

[3] Alexey Klochay. ["Implementing Dijkstra’s Algorithm in Python"](https://www.udacity.com/blog/2021/10/implementing-dijkstras-algorithm-in-python.html). *Udacity*, 2021. 

[4] Atsushi Sakai, Daniel Ingram, Joseph Dinius, Karan Chawla, Antonin Raffin & Alexis Paques. ["PythonRobotics: A Python Code Collection of Robotics Algorithms"](https://arxiv.org/abs/1808.10703). 2018. GitHub: https://github.com/AtsushiSakai/PythonRobotics
