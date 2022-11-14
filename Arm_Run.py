import numpy as np
from Arm_Modules.Arm_Robot import JointState, Robot
from Arm_Modules.Arm_Plotter import Plotter
from Arm_Modules.Arm_Environment import Environment
from Arm_Modules.Arm_RRT_Planner import RRTPlanner
from Nav_Modules.Nav_Geometry import Obstacle

def distance(p1, p2):
    return np.linalg.norm(p2-p1)

def run():
    robot = Robot(2)#, link_length=5)
    # obstacle = Obstacle(3, np.array([15,15]))
    obstacle1 = Obstacle(1, np.array([13,5]))
    obstacle2 = Obstacle(1, np.array([3, 14]))
    obstacle3 = Obstacle(1, np.array([10,10]))
    target1 = np.array([0,11])
    target2 = np.array([12,7])
    target3 = np.array([-1, 7])
    environment = Environment(robot, [obstacle1, obstacle2, obstacle3], [target2], epsilon = 1, is_wall = False, is_floor = True)
    # robot.forward_kinematics()
    plotter = Plotter(environment)
    ######plotter.interactive_plot()

    # joint_states_i, joint_states_f = [JointState() for _ in range(4)], [JointState(3.14/4), JointState(3.14/4), JointState(-3.14/2), JointState()]
    # plotter.generate_trajectory(joint_states_i, joint_states_f, 5, 30)

    # plotter.interactive_plot()
    # np.random.seed(4)
    np.random.seed(6)
    
    rrt_planner = RRTPlanner(environment, 0.05)
    tree = rrt_planner.rrt([JointState(np.pi/2), JointState(np.pi)])
    #tree = rrt_planner.rrt([JointState(0), JointState(np.pi), JointState(0), JointState(np.pi)])
    paths = rrt_planner.generate_paths()
    plotter.show_rrt(tree.nodes, paths)
    min_length = 100000000
    current_pos = rrt_planner.gripper_values[0]
    for pos in rrt_planner.gripper_values:
        if distance(pos, target3) < min_length:
            min_length = distance(pos, target3)
            current_pos = pos

    print("Gripper position found which is closest to goal: ", current_pos)
    print(f"Min. distance to target: {min_length:0.3f}\n")
    # print(paths[0][-1].vectorized_values())
    #print(paths[0][i].vectorized_values() for i in range(len(paths[0])))
    nodes = paths[0]
    states = [nodes[i].joint_values for i in range(len(nodes))]
    plotter.generate_trajectory(states, n_frames = 3)

    # fig,ax = plt.subplots()
    # plt.plot()
    return

if __name__ == '__main__':
    run()