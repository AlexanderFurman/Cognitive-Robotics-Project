from pddl.PDDL_problem_generator import PDDL_ProblemGenerator

class Interface:
    def __init__(self, domain_name, width, traversal_weight, num_rovers, num_ppl, pos_cave, pos_base, max_mission_duration):
        self.pddl_generator = PDDL_ProblemGenerator(domain_name, width, traversal_weight, num_rovers, num_ppl, pos_cave, pos_base, max_mission_duration)
        
        # self.map = Map(width, perp_line_length)