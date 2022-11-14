import warnings
warnings.simplefilter("ignore", UserWarning) # For some reason we get a warning that Fast Downward can't solve this type of problem (it can) - we'll ignore the warning
from unified_planning.shortcuts import *
from unified_planning.model.metrics import *
from unified_planning.io import PDDLWriter
from Nav_Modules.Nav_Geometry import *

class TaskPlanner:
    def __init__(self, new_graph, save_pddl=False, output_path=None):
        self.new_graph = new_graph
        self.save_pddl = save_pddl
        self.output_path = output_path
        self.goals = []
        for node in new_graph:
            if isinstance(node, Start_Node):
                self.start = node # there's only one start node
            elif isinstance(node, Goal_Node):
                self.goals.append(node)
        
    def BuildProblem(self):
        ## Declaring types
        Rover = UserType("Rover")
        Location = UserType("Location")
        Start = UserType("Start",father=Location)
        ### Unfortunately, we need to create a separate type for each of the goal nodes
        ### (this is part of the workaround for using the Fast Downward Planner without numeric fluents)
        Goals = []; N_goals = len(self.new_graph)
        for i in range(N_goals):
            Goals.append(UserType(f'Goal{i+1}',father=Location))

        ## Creating predicates (fluents)
        Rover_At = Fluent("Rover_At", BoolType(), rover=Rover, location=Location)
        Visited = Fluent("Visited", BoolType(), rover=Rover, location=Location)
        
        ## Declaring objects
        rover = Object("rover", Rover)
        start = Object(self.start.name, Start)
        goals = [Object(f'G{i}', Goals[i-1]) for i in range(1,4)]

        ## Creating action costs
        costs = {}
        for n1 in self.new_graph:
            if not (n1.name in costs):
                costs[n1.name] = {}
            for n2 in self.new_graph:
                if not (n2.name in costs):
                    costs[n2.name] = {}
                if n2 in self.new_graph[n1]:
                    costs[n1.name][n2.name] = Int(int(self.new_graph[n1][n2]['Value'])) ## We must convert the distances to integers, in order for the planner to work
                    costs[n2.name][n1.name] = Int(int(self.new_graph[n1][n2]['Value'])) # We will assume that the plans obtained will remain optimal regardless (due to the scale of our workspace)
        if self.start.name not in costs:
            print("Start node is not connected to the other nodes, please try again...")
            return None
        
        ## Creating actions
        locations = [g for g in goals]
        locations.append(start)
        moves = []; mac = {} #mac keeps track of the action costs
        for i in locations:
            for j in locations:
                if i!=j:
                    if j.name not in costs[i.name]:
                        continue
                    move = InstantaneousAction(f"move_{i.name}_{j.name}", r=Rover, l_from=i.type, l_to=j.type)
                    r = move.parameter("r")
                    l_from = move.parameter("l_from")
                    l_to = move.parameter("l_to")
                    move.add_precondition(Rover_At(r,l_from))
                    move.add_effect(Rover_At(r,l_from), False)
                    move.add_effect(Rover_At(r,l_to), True)
                    move.add_effect(Visited(r,l_to), True)
                    moves.append(move)
                    mac[move] = costs[i.name][j.name]
                    
        ## Populating the problem with fluents and actions
        problem = Problem("Mars")
        problem.add_fluent(Rover_At, default_initial_value=False)
        problem.add_fluent(Visited, default_initial_value=False)
        problem.add_actions(moves)

        ## Adding objects
        problem.add_object(rover)
        problem.add_object(start)
        problem.add_objects(goals)

        ## Setting the initial state
        problem.set_initial_value(Rover_At(rover,start), True)
        problem.set_initial_value(Visited(rover,start), True)
        
        ## Setting the goal state and metric
        for g in goals:
            problem.add_goal(Visited(rover,g))
        problem.add_goal(Rover_At(rover,start))
        problem.add_quality_metric(MinimizeActionCosts(mac))

        ## Save the .pddl files if you want
        if self.save_pddl:
            w = PDDLWriter(problem)
            w.write_domain(self.output_path+'domain_Mars.pddl')
            w.write_problem(self.output_path+'problem_Mars.pddl')
        return problem

    def SolveProblem(self):
        problem = self.BuildProblem()
        if problem == None:
            return None
        up.shortcuts.get_env().credits_stream = None # this just hides the credits of the planner
        with OneshotPlanner(name='fast-downward-opt') as planner:
            result = planner.solve(problem)
            plan = result.plan
            if plan is not None:
                print("Optimal Fast Downward Planner returned the following plan:")
                print('\t'+str(plan))
            else:
                print("Whoopsie, no plan could be found...")
        return plan

    def GetFinalTrajectory(self):
        plan = self.SolveProblem()
        if plan == None:
            return None
        new_graph = self.new_graph
        
        ## Getting the actions from the plan
        actions = []
        for i in range(len(plan.actions)):
            l1 = plan.actions[i].actual_parameters[1]
            l2 = plan.actions[i].actual_parameters[2]
            actions.append([l1, l2])

        ## Turning the list of actions into a trajectory
        final_trajectory = []; total_distance = 0
        for j in range(len(actions)):
            start_node = str(actions[j][0])
            end_node = str(actions[j][1])
            for n1 in new_graph:
                for n2 in new_graph:
                    if n2 in new_graph[n1]:
                        if n1.name == start_node and n2.name == end_node:
                            final_trajectory.append(new_graph[n1][n2]['Trajectory'])
                            total_distance += new_graph[n1][n2]['Value']         
                        elif n2.name == start_node and n1.name == end_node:  
                            final_trajectory.append(new_graph[n1][n2]['Trajectory'][::-1])
                            total_distance += new_graph[n1][n2]['Value'] 
        print(f"Total cost of the proposed path: {total_distance:0.3f}\n")
        return final_trajectory