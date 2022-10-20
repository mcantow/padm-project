# pip install padm-project-2022f/pddl-parser
from pddl_parser.PDDL import PDDL_Parser

parser = PDDL_Parser()
parser.parse_domain('domain.pddl')
parser.parse_problem('problem.pddl')
print(parser.problem_name)
print(parser.objects)
print(parser.state)
print(parser.positive_goals)
print(parser.negative_goals)
