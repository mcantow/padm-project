# pip install padm-project-2022f/pddl-parser
from pddl_parser.PDDL import PDDL_Parser

class ActivityPlanner:
    def mySolve(self, parser):
        state = parser.state
        plan = []
        queue = [(state, plan)]
        #accumulate list of all possible actions using groundify method
        all_actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                all_actions.append(act)

        while len(queue) > 0:
            state, plan = queue.pop(0)
            available_actions = self.get_available_actions(state, all_actions)
            for action in available_actions:
                new_state = self.update_state(state, action)
                if self.is_goal(new_state, parser):
                    full_plan = plan + [action]
                    return full_plan
                new_plan = plan + [action]
                queue.append((new_state, new_plan))
        return None
    
    
    def is_goal(self, state, parser):
        goal_positive_conditions = parser.positive_goals
        goal_negative_conditions = parser.negative_goals
        for condition in goal_positive_conditions:
            if condition not in state:
                return False
        for condition in goal_negative_conditions:
            if condition in state:
                return False
        return True

    def get_available_actions(self, state, all_actions):
        available_actions = []
        for action in all_actions:
            if self.is_valid_action(state, action):
                available_actions.append(action)
        return available_actions
        


    def is_valid_action(self, state, act):
        positive_preconditions = act.positive_preconditions 
        negative_preconditions = act.negative_preconditions
        for condition in positive_preconditions:
            if condition not in state:
                return False
        for condition in negative_preconditions:
            if condition in state:
                return False
        return True


    def update_state(self, state, action):
        updated_state = []
        for condition in  state:
            if condition not in action.del_effects:
                updated_state.append(condition)
        for condition in action.add_effects:
            if condition not in state:
                updated_state.append(condition)
        return updated_state
        
if __name__ == "__main__":
    parser = PDDL_Parser()
    parser.parse_domain('domain.pddl')
    parser.parse_problem('problem.pddl')
    planner = ActivityPlanner()
    plan = planner.mySolve(parser)
    if plan is None:
        print("NO PLAN FOUND")
    else:
        print("PLAN FOUND:")
        for action in plan:
            print(action.name)
        
"""
frozenset({('ontable', 'potted_meat_can1'), ('holding', 'roboarm', 'sugar_box0'), ('open', 'indigo_drawer_top')})













"""
