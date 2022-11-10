from unified_planning.shortcuts import *
#import unified_planning as up
from unified_planning.model.metrics import *
from unified_planning.io import PDDLWriter

# Declaring types
Rover = UserType("Rover")
Location = UserType("Location")
Start = UserType("Start",father=Location)
Goal = UserType("Goal",father=Location)
Goals = []
for i in range(1,4):
    Goals.append(UserType(f'Goal{i}',father=Goal))

# Creating problem ‘variables’
Rover_At = Fluent("Rover_At", BoolType(), rover=Rover, location=Location)
Visited = Fluent("Visited", BoolType(), rover=Rover, location=Location)
#Adjacent = Fluent("Adjacent", BoolType(), l1=Location, l2=Location)
#Distance = Fluent("Distance", RealType(), l1=Location, l2=Location)
#Distance_Travelled = Fluent("Distance_Travelled", IntType())

# Declaring objects
rover = Object("rover", Rover)
start = Object("S", Start)
goals = [Object(f'G{i}', Goals[i-1]) for i in range(1,4)]
#goals = [Object(f'G{i}', Goal) for i in range(1,4)]

mac = {}
# Creating actions
costs = {'S':{'S':Int(0),'G1':Int(1),'G2':Int(2),'G3':Int(4)}, 'G1':{'S':Int(1),'G1':Int(0),'G2':Int(5),'G3':Int(2)}, 
            'G2':{'S':Int(2),'G1':Int(5),'G2':Int(0),'G3':Int(4)}, 'G3':{'S':Int(4),'G1':Int(2),'G2':Int(4),'G3':Int(0)}}
moves = []
locations = [g for g in goals]
locations.append(start)
for i in locations:
    for j in locations:
        if i!=j:
            move = InstantaneousAction(f"move_{i.name}_{j.name}", r=Rover, l_from=i.type, l_to=j.type)
            r = move.parameter("r")
            l_from = move.parameter("l_from")
            l_to = move.parameter("l_to")
            move.add_precondition(Rover_At(r,l_from))
            #move.add_precondition(Adjacent(l_from,l_to))
            move.add_effect(Rover_At(r,l_from), False)
            move.add_effect(Rover_At(r,l_to), True)
            move.add_effect(Visited(r,l_to), True)
            mac[move] = costs[i.name][j.name]
            moves.append(move)

#print(moves)
#print(mac)
        #problem.set_initial_value(Distance(i,j), costs[i.name][j.name])
#
#for n1 in self.new_graph:
#            for n2 in self.new_graph:
#                if n2 in self.new_graph[n1]:
#                    for l in locations:
#                        if n1.name == l.name:
#                            l1 = l
#                        if n2.name == l.name:
#                            l2 = l
#                    problem.set_initial_value(Adjacent(l1,l2), True)
#                    problem.set_initial_value(Adjacent(l2,l1), True)


#move = InstantaneousAction("move", r=Rover, l_from=Location, l_to=Location)
## parameters
#r = move.parameter("r")
##_from = move.parameter("l_from")
#l_to = move.parameter("l_to")
## preconditions
#move.add_precondition(Rover_At(r,l_from))
#move.add_precondition(Adjacent(l_from,l_to))
## effects
#move.add_effect(Rover_At(r,l_from), False)
#move.add_effect(Rover_At(r,l_to), True)
#move.add_effect(Visited(r,l_to), True)
#move.add_effect(Distance_Travelled, Plus(Distance_Travelled, Distance(l_from,l_to)))
#move.add_increase_effect(Distance_Travelled, Distance(l_from,l_to))

#mac[move] = Distance(l_from, l_to)

#print(move.parameters)
#costs = {'S':{'S':0,'G1':1,'G2':2,'G3':4}, 'G1':{'S':1,'G1':0,'G2':5,'G3':2}, 'G2':{'S':2,'G1':5,'G2':0,'G3':4}, 'G3':{'S':4,'G1':2,'G2':4,'G3':0}}
#mac[move] = costs[move.parameters[1]][move.parameters[2]]



# Populating the problem with initial state and goals
problem = Problem("Mars")
problem.add_fluent(Rover_At, default_initial_value=False)
problem.add_fluent(Visited, default_initial_value=False)
#problem.add_fluent(Adjacent, default_initial_value=True)
#problem.add_fluent(Distance, default_initial_value=0)
#problem.add_fluent(Distance_Travelled)#, default_initial_value=0.0)
problem.add_actions(moves)
## add objects
problem.add_object(rover)
problem.add_object(start)
problem.add_objects(goals)
## initial state
problem.set_initial_value(Rover_At(rover,start), True)
problem.set_initial_value(Visited(rover,start), True)

#locations = [g for g in goals]
#locations.append(start)
#new_costs = {}
#costs = {'S':{'S':0,'G1':1,'G2':2,'G3':4}, 'G1':{'S':1,'G1':0,'G2':5,'G3':2}, 'G2':{'S':2,'G1':5,'G2':0,'G3':4}, 'G3':{'S':4,'G1':2,'G2':4,'G3':0}}
#for i in locations:
#    for j in locations:
        #pass
        #problem.set_initial_value(Distance(i,j), costs[i.name][j.name])
#problem.set_initial_value(Distance_Travelled, 0)

for g in goals:
    problem.add_goal(Visited(rover,g))
problem.add_goal(Rover_At(rover,start))


problem.add_quality_metric(MinimizeActionCosts(mac))
#problem.add_quality_metric(MinimizeExpressionOnFinalState(Distance_Travelled()))
print(problem.kind)

w = PDDLWriter(problem)
w.write_domain('tmp/domain_Mars.pddl')
w.write_problem('tmp/problem_Mars.pddl')

#with OneshotPlanner(problem_kind=problem.kind) as planner:
#with OneshotPlanner(name='enhsp') as planner:
with OneshotPlanner(name='fast-downward') as planner:
    result = planner.solve(problem)
    plan = result.plan
    if plan is not None:
        print("%s returned:" % planner.name)
        print(result.plan)
        #for start, action, duration in plan.timed_actions:
        #    print("%s: %s [%s]" % (float(start), action, float(duration)))
    else:
        print("No plan found.")

#with OneshotPlanner(name='pyperplan') as planner:
#    result = planner.solve(problem)
#    if result.status == up.solvers.PlanGenerationResultStatus.SOLVED_SATISFICING:
#        print("Pyperplan returned: %s" % result.plan)
#    else:
#        print("No plan found.")